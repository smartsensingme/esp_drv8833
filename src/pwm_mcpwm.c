/**
 * @file pwm_mcpwm.c
 * @brief MCPWM implementation of the private PWM backend contract.
 *
 * Only one backend is compiled. The command layer validates duty and owns the
 * task mutex; application code must never call these functions directly.
 * Channel indexes map to physical IN1..IN4, independently of wheel identity.
 */
#include "driver/mcpwm_prelude.h"
#include "esp_rom_sys.h"
#include "pwm_backend.h"
#include "soc/soc_caps.h"
#include <math.h>
#include <stdlib.h>

/** Requested MCPWM timer tick rate, Hz; used to derive integer periods. */
#define RESOLUTION_HZ 80000000U
/**
 * Backend-owned runtime resources, allocated by drv8833_pwm_create().
 * Caller serialization is mandatory; partial initialization is tracked so
 * rollback releases only the resources successfully acquired.
 */
struct drv8833_pwm {
  /** Owned shared carrier timebase for both motors. */
  mcpwm_timer_handle_t timer;

  /** Owned operators for physical bridges A and B. */
  mcpwm_oper_handle_t operators[2];

  /** Owned compare thresholds, one per physical input IN1..IN4. */
  mcpwm_cmpr_handle_t comparators[4];

  /** Owned routed output generators, one per physical input. */
  mcpwm_gen_handle_t generators[4];

  /** Timer successfully enabled; controls rollback disable. */
  bool enabled;

  /** Timer start succeeded; cleanup requests stop at a period boundary. */
  bool started;

  /** Carrier period in RESOLUTION_HZ ticks, constrained to 2..65535. */
  uint32_t period;
};

/**
 * Internal task constructor called by esp_drv8833_create(), not applications.
 * @param c Generic-validated config with positive pwm_freq_hz; IDF validates
 * group.
 * @param out Non-NULL destination, NULL on error, owned backend on success.
 * @return ESP_OK, ESP_ERR_INVALID_ARG for noninteger/out-of-range period or
 * group, ESP_ERR_NO_MEM, or an IDF MCPWM resource/configuration error. Calls
 * backend delete on partial failure. Allocates one timer/two operators/four
 * output pairs; forces all generators LOW before enabling the carrier. No ULT
 * brake is wired.
 */
esp_err_t drv8833_pwm_create(const esp_drv8833_config_t *c,
                             drv8833_pwm_t **out) {
  *out = NULL;

  /* Period validation block: generic validation has already rejected zero
   * frequency. */
  if (c->mcpwm_group < 0 || RESOLUTION_HZ % c->pwm_freq_hz != 0 ||
      RESOLUTION_HZ / c->pwm_freq_hz < 2 ||
      RESOLUTION_HZ / c->pwm_freq_hz > 65535)
    return ESP_ERR_INVALID_ARG;

  /* Allocation block: zero handles/flags make partial initialization cleanable.
   */
  drv8833_pwm_t *p = calloc(1, sizeof(*p));
  if (!p)
    return ESP_ERR_NO_MEM;

  p->period = RESOLUTION_HZ / c->pwm_freq_hz;

  /* Timebase block: share one up-counting carrier across independent motor
   * duties. */
  mcpwm_timer_config_t timer = {
      .group_id = c->mcpwm_group,
      .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
      .resolution_hz = RESOLUTION_HZ,
      .period_ticks = p->period,
      .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
  };
  esp_err_t err = mcpwm_new_timer(&timer, &p->timer);
  if (err != ESP_OK)
    goto fail;

  /* Operator/routing block: two generators and comparators per physical bridge.
   */
  int pins[] = {c->in1_gpio, c->in2_gpio, c->in3_gpio, c->in4_gpio};
  for (unsigned bridge = 0; bridge < 2; bridge++) {
    mcpwm_operator_config_t op = {.group_id = c->mcpwm_group};
    err = mcpwm_new_operator(&op, &p->operators[bridge]);
    if (err != ESP_OK)
      goto fail;
    err = mcpwm_operator_connect_timer(p->operators[bridge], p->timer);
    if (err != ESP_OK)
      goto fail;
    for (unsigned side = 0; side < 2; side++) {
      unsigned i = 2 * bridge + side;
      mcpwm_comparator_config_t cmp = {.flags.update_cmp_on_tez = true};
      err =
          mcpwm_new_comparator(p->operators[bridge], &cmp, &p->comparators[i]);
      if (err != ESP_OK)
        goto fail;
      mcpwm_generator_config_t gen = {.gen_gpio_num = pins[i]};
      err = mcpwm_new_generator(p->operators[bridge], &gen, &p->generators[i]);
      if (err != ESP_OK)
        goto fail;

      /* Initialization block: force LOW before installing normal pulse actions.
       */
      err = mcpwm_generator_set_force_level(p->generators[i], 0, true);
      if (err != ESP_OK)
        goto fail;
      err = mcpwm_comparator_set_compare_value(p->comparators[i], 1);
      if (err != ESP_OK)
        goto fail;

      /* Waveform block: HIGH at timer zero, LOW at comparison; compare updates
       * at zero. */
      err = mcpwm_generator_set_action_on_timer_event(
          p->generators[i],
          MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                       MCPWM_TIMER_EVENT_EMPTY,
                                       MCPWM_GEN_ACTION_HIGH));
      if (err != ESP_OK)
        goto fail;
      err = mcpwm_generator_set_action_on_compare_event(
          p->generators[i], MCPWM_GEN_COMPARE_EVENT_ACTION(
                                MCPWM_TIMER_DIRECTION_UP, p->comparators[i],
                                MCPWM_GEN_ACTION_LOW));
      if (err != ESP_OK)
        goto fail;
    }
  }

  /* Start block: all outputs remain forced LOW while the timebase begins. */
  err = mcpwm_timer_enable(p->timer);
  if (err != ESP_OK)
    goto fail;
  p->enabled = true;
  err = mcpwm_timer_start_stop(p->timer, MCPWM_TIMER_START_NO_STOP);
  if (err != ESP_OK)
    goto fail;
  p->started = true;
  *out = p;
  return ESP_OK;

  /* Rollback block: preserve the original setup error after best-effort
   * teardown. */

fail:
  drv8833_pwm_delete(p);
  return err;
}

/**
 * Internal task output write called by stop_locked() and
 * esp_drv8833_set_speed().
 * @param p Initialized backend, exclusively owned while the component mutex is
 * held.
 * @param i Physical input index 0..3; trusted internal caller precondition.
 * @param duty Finite magnitude in 0..100 percent, validated by command layer.
 * @return ESP_OK or first IDF comparator/generator error. No normal-path
 * allocation. Zero/full-period counts use persistent LOW/HIGH force.
 * Intermediate values update a compare threshold for the next timer-zero event
 * and release force; the first pulse after force release can be partial, not a
 * synchronized pair.
 */
esp_err_t drv8833_pwm_set(drv8833_pwm_t *p, unsigned i, float duty) {
  /* Duty block: saturate endpoint behavior through force instead of
   * compare-edge ambiguity. */
  uint32_t ticks = (uint32_t)lroundf(duty * (float)p->period / 100.0f);
  if (ticks == 0)
    return mcpwm_generator_set_force_level(p->generators[i], 0, true);

  if (ticks >= p->period)
    return mcpwm_generator_set_force_level(p->generators[i], 1, true);

  /* Intermediate-duty block: stage the threshold, then resume timer-driven
   * actions. */
  esp_err_t err = mcpwm_comparator_set_compare_value(p->comparators[i], ticks);
  if (err != ESP_OK)
    return err;

  return mcpwm_generator_set_force_level(p->generators[i], -1, true);
}

/**
 * Internal task cleanup used by backend-create rollback, component-create
 * rollback and esp_drv8833_delete(); no external application caller.
 * @param p Owned partial/complete backend, or NULL for a harmless no-op.
 * @return First MCPWM cleanup error after attempting every acquired resource.
 * Forces outputs LOW, stops the carrier, waits roughly one period plus margin,
 * then disables/deletes resources in dependency order. Frees runtime storage
 * even on error; failed IDF cleanup may leave a peripheral resource unreleased.
 */
esp_err_t drv8833_pwm_delete(drv8833_pwm_t *p) {
  if (!p)
    return ESP_OK;

  /* Cleanup block: preserve the first error but attempt later resource
   * releases. */
  esp_err_t first = ESP_OK;

/** Evaluate a teardown call and preserve the first error without early return.
 */
#define CLEANUP(call)                                                          \
  do {                                                                         \
    esp_err_t e = (call);                                                      \
    if (first == ESP_OK)                                                       \
      first = e;                                                               \
  } while (0)

  /* Output shutdown block: force every allocated generator LOW before stopping
   * timer. */
  for (unsigned i = 0; i < 4; i++) {
    if (p->generators[i])
      CLEANUP(mcpwm_generator_set_force_level(p->generators[i], 0, true));
  }

  /* Timer shutdown block: STOP_EMPTY completes at the next carrier boundary. */
  if (p->started) {
    CLEANUP(mcpwm_timer_start_stop(p->timer, MCPWM_TIMER_STOP_EMPTY));

    /* Requested busy wait covers an integer-truncated period plus 2 us. */
    esp_rom_delay_us(1000000U / (RESOLUTION_HZ / p->period) + 2);
  }

  /* Dependency teardown block: disable the timebase, remove child objects, then
   * timer. */
  if (p->enabled)
    CLEANUP(mcpwm_timer_disable(p->timer));
  for (unsigned i = 0; i < 4; i++) {
    if (p->generators[i])
      CLEANUP(mcpwm_del_generator(p->generators[i]));
    if (p->comparators[i])
      CLEANUP(mcpwm_del_comparator(p->comparators[i]));
  }
  for (unsigned i = 0; i < 2; i++) {
    if (p->operators[i])
      CLEANUP(mcpwm_del_operator(p->operators[i]));
  }
  if (p->timer)
    CLEANUP(mcpwm_del_timer(p->timer));
#undef CLEANUP
  free(p);
  return first;
}

/* Explicit static HIGH for active braking; command layer owns serialization. */
esp_err_t drv8833_pwm_force_high(drv8833_pwm_t *p, unsigned index) {
  return mcpwm_generator_set_force_level(p->generators[index], 1, true);
}
