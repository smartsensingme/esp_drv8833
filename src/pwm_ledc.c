/**
 * @file pwm_ledc.c
 * @brief LEDC implementation of the private PWM backend contract.
 *
 * Only one backend is compiled. The command layer validates duty and owns the
 * task mutex; application code must never call these functions directly.
 * Channel indexes map to physical IN1..IN4, independently of wheel identity.
 */
#include "driver/ledc.h"
#include "pwm_backend.h"
#include "soc/soc_caps.h"
#include <math.h>
#include <stdlib.h>

/**
 * Backend-owned runtime resources, allocated by drv8833_pwm_create().
 * Caller serialization is mandatory; partial initialization is tracked so
 * rollback releases only the resources successfully acquired.
 */
struct drv8833_pwm {
  /** Exclusively reserved low-speed timer selected by the application. */
  ledc_timer_t timer;

  /** First of four exclusive consecutive channel indexes. */
  int first;

  /** Number of channels successfully configured, 0..4, for rollback. */
  unsigned configured;

  /** Whether timer setup succeeded and must be deconfigured. */
  bool timer_ready;

  /** 2^resolution, used as the 100-percent endpoint in hardware duty units. */
  uint32_t full_scale;
};

/**
 * Internal task constructor called by esp_drv8833_create(), not applications.
 * @param c Validated generic config; LEDC timer/channels must be exclusive.
 * @param out Non-NULL destination, NULL on failure, owned runtime on success.
 * @return ESP_OK, ESP_ERR_INVALID_ARG for resource/frequency resolution errors,
 * ESP_ERR_NO_MEM, or an IDF LEDC error. Calls backend delete on partial
 * failure. Allocates only during initialization; XTAL is 40 MHz on the
 * supported targets.
 */
esp_err_t drv8833_pwm_create(const esp_drv8833_config_t *c,
                             drv8833_pwm_t **out) {
  *out = NULL;

  /* Validation block: timer and all four consecutive channels must fit the
   * target. */
  if (c->ledc_timer < 0 || c->ledc_timer >= LEDC_TIMER_MAX ||
      c->ledc_first_channel < 0 ||
      c->ledc_first_channel > SOC_LEDC_CHANNEL_NUM - 4) {
    return ESP_ERR_INVALID_ARG;
  }

  /* Allocation block: preserve partial-setup progress for failure cleanup. */
  drv8833_pwm_t *p = calloc(1, sizeof(*p));
  if (!p)
    return ESP_ERR_NO_MEM;

  p->timer = c->ledc_timer;
  p->first = c->ledc_first_channel;

  /* Resolution block: use a stable XTAL source and compute one cached scale. */
  unsigned bits = ledc_find_suitable_duty_resolution(40000000, c->pwm_freq_hz);
  if (bits == 0) {
    free(p);
    return ESP_ERR_INVALID_ARG;
  }
  p->full_scale = 1U << bits;

  /* Timer block: configure only the reserved timer and check integer readback.
   */
  ledc_timer_config_t timer = {
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .timer_num = p->timer,
      .duty_resolution = bits,
      .freq_hz = c->pwm_freq_hz,
      .clk_cfg = LEDC_USE_XTAL_CLK,
  };
  esp_err_t err = ledc_timer_config(&timer);
  if (err != ESP_OK) {
    free(p);
    return err;
  }
  p->timer_ready = true;
  if (ledc_get_freq(LEDC_LOW_SPEED_MODE, p->timer) != c->pwm_freq_hz) {
    err = ESP_ERR_INVALID_ARG;
    goto fail;
  }

  /* Routing block: initialize every physical channel with zero duty and LOW
   * idle. */
  int pins[] = {c->in1_gpio, c->in2_gpio, c->in3_gpio, c->in4_gpio};
  for (unsigned i = 0; i < 4; i++) {
    ledc_channel_config_t channel = {
        .gpio_num = pins[i],
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = p->first + i,
        .timer_sel = p->timer,
        .duty = 0,
    };
    err = ledc_channel_config(&channel);
    if (err != ESP_OK)
      goto fail;
    p->configured++;
    err = ledc_stop(LEDC_LOW_SPEED_MODE, p->first + i, 0);
    if (err != ESP_OK)
      goto fail;
  }
  *out = p;
  return ESP_OK;

  /* Rollback block: release what succeeded while returning the original setup
   * error. */

fail:
  drv8833_pwm_delete(p);
  return err;
}

/**
 * Internal task output write called by stop_locked() and
 * esp_drv8833_set_speed().
 * @param p Initialized backend, with caller holding the component mutex.
 * @param index Physical input index 0..3; already validated by internal
 * callers.
 * @param duty Finite unsigned duty 0..100 percent, validated by the command
 * layer.
 * @return IDF LEDC stop/set/update error, or ESP_OK. No allocation or logging
 * on successful control. Zero uses idle LOW; positive values round to the
 * nearest full-scale tick. Two public IDF writes are serialized by caller
 * ownership.
 */
esp_err_t drv8833_pwm_set(drv8833_pwm_t *p, unsigned index, float duty) {
  ledc_channel_t ch = p->first + index;
  if (duty == 0.0f)
    return ledc_stop(LEDC_LOW_SPEED_MODE, ch, 0);

  /* Quantization block: sub-tick commands may resolve to zero; handle that as
   * idle. */
  uint32_t ticks = (uint32_t)lroundf(duty * (float)p->full_scale / 100.0f);
  if (ticks == 0)
    return ledc_stop(LEDC_LOW_SPEED_MODE, ch, 0);

  /* Update block: avoid fade service, its lazy allocations and extra waits.
   * Exclusive channel ownership plus the component mutex serialize the pair. */
  esp_err_t err = ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, ticks);
  if (err != ESP_OK)
    return err;

  return ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
}

/**
 * Internal task cleanup called by backend-create rollback, component-create
 * rollback and esp_drv8833_delete(). Not called externally by applications.
 * @param p Owned complete/partial backend, or NULL for a harmless no-op.
 * @return First IDF cleanup error after attempting all configured resources.
 * Runtime storage is freed even on error; caller must never retry with p.
 * Stops channels before unbinding, then pauses and deconfigures its timer.
 */
esp_err_t drv8833_pwm_delete(drv8833_pwm_t *p) {
  if (!p)
    return ESP_OK;

  /* Cleanup block: continue teardown after errors, retaining the first result.
   */
  esp_err_t first = ESP_OK;
  for (unsigned i = 0; i < p->configured; i++) {
    esp_err_t e = ledc_stop(LEDC_LOW_SPEED_MODE, p->first + i, 0);
    if (first == ESP_OK)
      first = e;
    ledc_channel_config_t ch = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = p->first + i,
        .deconfigure = true,
    };
    e = ledc_channel_config(&ch);
    if (first == ESP_OK)
      first = e;
  }

  /* Timer teardown block: IDF requires pause before timer deconfiguration. */
  if (p->timer_ready) {
    esp_err_t e = ledc_timer_pause(LEDC_LOW_SPEED_MODE, p->timer);
    if (first == ESP_OK)
      first = e;
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = p->timer,
        .deconfigure = true,
    };
    e = ledc_timer_config(&timer);
    if (first == ESP_OK)
      first = e;
  }
  free(p);
  return first;
}
