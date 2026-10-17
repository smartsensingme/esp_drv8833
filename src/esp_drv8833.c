/**
 * @file esp_drv8833.c
 * @brief Backend-independent command ownership and notification-only ULT path.
 *
 * Task commands own a mutex; the short spinlock protects fields shared with the
 * integer-only ISR. Fault callbacks run in a worker after releasing both locks.
 * This file never implements speed regulation or infers an electrical cause
 * from the temporal context attached to a ULT capture.
 */
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "pwm_backend.h"
#include <math.h>
#include <stdlib.h>

/** Worker notification bit for one accepted, unacknowledged ULT signal. */
#define FAULT_EVENT (1U << 0)
/** Worker notification bit requesting shutdown; takes priority over dispatch.
 */
#define EXIT_EVENT (1U << 1)
/** Requested nSLEEP startup busy wait, in microseconds; not a task WCET bound.
 */
#define WAKE_US 1000U

/**
 * Private instance allocated internally by esp_drv8833_create() and freed by
 * esp_drv8833_delete(). Configuration is immutable after publication;
 * pending-event/context fields use spin, task-side command state uses mutex.
 * Never exposed to the application.
 */
struct esp_drv8833 {
  /** Copied immutable configuration; callback_context remains
   * application-owned. */
  esp_drv8833_config_t config;

  /** Owned selected-backend instance, NULL until backend creation succeeds. */
  drv8833_pwm_t *pwm;

  /** Task command/state mutex; callbacks execute after releasing it. */
  SemaphoreHandle_t mutex;

  /** Owned binary semaphore acknowledging the worker has stopped using this
   * object. */
  SemaphoreHandle_t worker_done;

  /** Owned fault task handle, initialized before the GPIO ISR is enabled. */
  TaskHandle_t worker;

  /** Cross-core/ISR synchronization; never nest blocking work inside this lock.
   */
  portMUX_TYPE spin;

  /** Spin-protected signal acknowledgement latch; unrelated to motor enable. */
  bool latched;

  /** Spin-protected occupied capture slot, cleared only when the worker copies
   * it. */
  bool event_pending;

  /** Spin-protected accepted snapshot retained after dispatch/acknowledgement.
   */
  esp_drv8833_fault_event_t pending_event;

  /** Spin-protected most recent transition type, initially NONE from calloc. */
  esp_drv8833_fault_context_t transition_context;

  /** Spin-protected start time on the ESP timer microsecond timebase. */
  int64_t transition_begin_us;

  /** Spin-protected completion time used for the inclusive trailing window. */
  int64_t transition_end_us;

  /** Spin-protected phase flag; task preemption can lengthen the active phase.
   */
  bool transition_active;

  /** Spin-protected count of serviced falling edges, wrapping modulo 2^32. */
  uint32_t edges;

  /** Handler registered successfully; deletion must remove it before freeing
   * state. */
  bool isr_added;

  /** EEP output initialized; cleanup may safely attempt to hold it low. */
  bool sleep_ready;

  /** ULT GPIO initialized; cleanup may reset this exclusive input. */
  bool fault_ready;

  /** Task-mutex-protected shutdown latch; commands/acknowledgement reject it.
   */
  bool closing;

  /** Publication gate: failure cleanup must never dispatch a user callback. */
  bool ready;

  /** Shutdown retry guard: never notify a task that may already have exited. */
  bool worker_stop_sent;

  /** Creation core ID; ULT deletion must use this same pinned service core. */
  int lifecycle_core;

  /** Mutex-protected cached task state; live ISR fields are overlaid by
   * esp_drv8833_get_state(). */
  esp_drv8833_state_t state;

  /** Cached physical signs for bridges A/B: -1, 0, +1 after mapping/inversion.
   */
  int direction[2];
};

/**
 * Internal duration conversion used by take() and esp_drv8833_delete().
 * @param ms Positive wait request in milliseconds.
 * @return Tick count rounded up; quantization does not bound scheduler latency.
 */
static TickType_t wait_ticks(unsigned ms) {
  return (ms + portTICK_PERIOD_MS - 1) / portTICK_PERIOD_MS;
}

/**
 * Internal task-only mutex acquisition used by fault_worker(),
 * esp_drv8833_set_speed(), esp_drv8833_clear_fault(), esp_drv8833_get_state()
 * and esp_drv8833_delete().
 * @param d Initialized instance with an owned mutex.
 * @return True after acquiring ownership, false after the rounded 20-ms
 * timeout.
 */
static bool take(esp_drv8833_handle_t d) {
  return xSemaphoreTake(d->mutex, wait_ticks(20)) == pdTRUE;
}

/**
 * Internal level read called by esp_drv8833_create() and
 * esp_drv8833_get_state(), not from an ISR.
 * @param d Instance whose immutable GPIO configuration is valid.
 * @return Current ULT-low indication, or false when the optional pin is absent.
 * This sample is not latched history and may change immediately after the read.
 */
static bool fault_low(esp_drv8833_handle_t d) {
  return d->config.fault_gpio >= 0 && gpio_get_level(d->config.fault_gpio) == 0;
}

/**
 * Internal metadata publication used by esp_drv8833_create(), stop_locked() and
 * esp_drv8833_set_speed().
 * @param d Instance owned by the calling task (mutex held, or unpublished
 * create).
 * @param context Transition being attempted, not a guarantee of hardware
 * success. Records the timer origin while spin is held so the ISR sees a
 * coherent phase.
 */
static void transition_begin(esp_drv8833_handle_t d,
                             esp_drv8833_fault_context_t context) {
  portENTER_CRITICAL(&d->spin);
  d->transition_context = context;
  d->transition_begin_us = esp_timer_get_time();
  d->transition_active = true;
  portEXIT_CRITICAL(&d->spin);
}

/**
 * Internal phase completion used by esp_drv8833_create(), stop_locked() and
 * esp_drv8833_set_speed().
 * @param d Task-owned instance as required by transition_begin().
 * Starts the configured post-operation association window without changing ULT.
 */
static void transition_end(esp_drv8833_handle_t d) {
  portENTER_CRITICAL(&d->spin);
  d->transition_end_us = esp_timer_get_time();
  d->transition_active = false;
  portEXIT_CRITICAL(&d->spin);
}

/**
 * Internal integer-only snapshot constructor called by signal_fault() and
 * fault_isr(); caller already holds spin with the appropriate task/ISR API.
 * @param d Internally allocated instance with initialized timer metadata.
 * Captures service time, not the physical edge time. Elapsed duration saturates
 * at UINT32_MAX; callback delays and future transitions cannot relabel this
 * copy.
 */
static void IRAM_ATTR capture_event_locked(esp_drv8833_handle_t d) {
  int64_t now = esp_timer_get_time();

  /* Association block: zero window disables only the trailing phase; active
   * work still carries context. The boundary is inclusive at microsecond
   * resolution. */
  bool associated =
      d->transition_context != ESP_DRV8833_FAULT_CONTEXT_NONE &&
      (d->transition_active ||
       (d->config.fault_context_window_us > 0 &&
        now - d->transition_end_us <= d->config.fault_context_window_us));

  /* Value capture block: preserve acceptance metadata independently of
   * delivery. */
  d->pending_event.falling_edges = d->edges;
  d->pending_event.timestamp_us = now;
  d->pending_event.context =
      associated ? d->transition_context : ESP_DRV8833_FAULT_CONTEXT_NONE;
  d->pending_event.transition_active = associated && d->transition_active;

  /* Saturation block: keep the public 32-bit duration valid after a long
   * preemption. */
  int64_t elapsed = associated ? now - d->transition_begin_us : 0;
  d->pending_event.transition_elapsed_us =
      elapsed > UINT32_MAX ? UINT32_MAX : (uint32_t)elapsed;
}

/**
 * Internal task-only initial-level capture called only by esp_drv8833_create().
 * @param d Instance with worker initialized and ULT already observed low.
 * Sets one pending event under spin and notifies outside the critical section.
 * A pre-existing low does not increment edges; already-latched captures
 * coalesce.
 */
static void signal_fault(esp_drv8833_handle_t d) {
  portENTER_CRITICAL(&d->spin);
  bool notify = !d->latched;
  if (notify) {
    d->latched = true;
    d->event_pending = true;
    capture_event_locked(d);
  }
  portEXIT_CRITICAL(&d->spin);
  if (notify)
    xTaskNotify(d->worker, FAULT_EVENT, eSetBits);
}

/**
 * Internal task stop used by esp_drv8833_set_speed() and esp_drv8833_delete();
 * caller holds mutex.
 * @param d Live instance with PWM resources initialized.
 * @return First PWM/EEP error, after attempting all four low outputs and EEP.
 * Only a completely successful stop zeros cached commands/directions; EEP cache
 * updates independently on a successful pin write. Repeated cached stop does
 * not renew COAST association. No ULT latch is set by a peripheral error.
 */
static esp_err_t stop_locked(esp_drv8833_handle_t d) {
  /* Transition block: do not extend the window for an already-stopped cache. */
  bool changed = d->direction[0] != 0 || d->direction[1] != 0 ||
                 (d->sleep_ready && !d->state.sleeping);
  if (changed)
    transition_begin(d, ESP_DRV8833_FAULT_CONTEXT_COAST);

  /* Output block: continue best-effort writes after an individual channel
   * error. */
  esp_err_t first = ESP_OK;
  for (unsigned i = 0; i < 4; i++) {
    esp_err_t e = drv8833_pwm_set(d->pwm, i, 0.0f);
    if (first == ESP_OK)
      first = e;
  }
  if (d->sleep_ready) {
    esp_err_t e = gpio_set_level(d->config.sleep_gpio, 0);
    if (e == ESP_OK)
      d->state.sleeping = true;
    if (first == ESP_OK)
      first = e;
  }

  /* Cache block: partial hardware success must not claim a completed pair stop.
   */
  d->state.last_stop_error = first;
  if (first == ESP_OK) {
    d->state.left_percent = 0.0f;
    d->state.right_percent = 0.0f;
    d->direction[0] = d->direction[1] = 0;
  }
  if (changed)
    transition_end(d);
  return first;
}

/**
 * Internal task wake called only by esp_drv8833_set_speed() while holding
 * mutex.
 * @param d Instance with inputs previously held low in sleep.
 * @return ESP_OK if already awake or after the requested 1000-us startup wait;
 * otherwise the GPIO error without applying PWM. No task delay or allocation.
 */
static esp_err_t wake_locked(esp_drv8833_handle_t d) {
  if (!d->state.sleeping)
    return ESP_OK;

  esp_err_t e = gpio_set_level(d->config.sleep_gpio, 1);
  if (e != ESP_OK)
    return e;

  d->state.sleeping = false;

  /* Startup block: keep low inputs throughout the IC wake interval. */
  esp_rom_delay_us(WAKE_US);
  return ESP_OK;
}

/**
 * Internal per-GPIO callback registered by esp_drv8833_create(), invoked by the
 * IDF GPIO ISR service. No external application calls this entry.
 * @param arg Borrowed internal instance, valid until handler removal.
 * Counts every serviced falling edge, captures only the first unacknowledged
 * signal, then notifies/yields using ISR-safe RTOS calls. Never writes PWM/EEP,
 * uses floating point, formats text, allocates or invokes application
 * callbacks.
 */
static void IRAM_ATTR fault_isr(void *arg) {
  /* Borrow block: constructor initialized this internal-RAM context before
   * enabling the GPIO source. No allocation occurs in this ISR. */
  esp_drv8833_handle_t d = arg;
  portENTER_CRITICAL_ISR(&d->spin);

  /* Capture block: servicing repeats still updates evidence, without callback
   * storms. */
  d->edges++;
  bool notify = !d->latched;
  if (notify) {
    d->latched = true;
    d->event_pending = true;
    capture_event_locked(d);
  }
  portEXIT_CRITICAL_ISR(&d->spin);

  /* Handoff block: no RTOS notification or yield while holding the instance
   * spinlock. */
  if (notify) {
    BaseType_t higher = pdFALSE;
    xTaskNotifyFromISR(d->worker, FAULT_EVENT, eSetBits, &higher);
    if (higher)
      portYIELD_FROM_ISR();
  }
}

/**
 * Internal task entry created by esp_drv8833_create(); invoked only by
 * FreeRTOS.
 * @param arg Borrowed instance kept alive until worker_done acknowledgement.
 * Waits on notification bits, retries bounded mutex acquisition while
 * preserving exit requests, copies the occupied event slot, then calls on_fault
 * outside locks. An absent callback still consumes the slot for polling
 * clients. Shutdown discards pending work and signals completion before
 * self-deletion.
 */
static void fault_worker(void *arg) {
  esp_drv8833_handle_t d = arg;
  for (;;) {
    /* Release block: notifications are bits, not a lossless queue of every
     * edge. */
    uint32_t events = 0;
    xTaskNotifyWait(0, UINT32_MAX, &events, portMAX_DELAY);
    if (events & EXIT_EVENT)
      break;
    if (!(events & FAULT_EVENT))
      continue;

    /* Contention block: preserve EXIT during retries and yield between failed
     * attempts. */
    while (!take(d)) {
      uint32_t extra = 0;
      xTaskNotifyWait(0, UINT32_MAX, &extra, 0);
      if (extra & EXIT_EVENT)
        goto done;
      vTaskDelay(1);
    }

    /* Dispatch block: copy the accepted value and free the slot before
     * application code. */
    esp_drv8833_fault_event_t event = {0};
    portENTER_CRITICAL(&d->spin);
    bool notify = d->ready && !d->closing && d->event_pending;
    if (notify) {
      event = d->pending_event;
      d->event_pending = false;
    }
    portEXIT_CRITICAL(&d->spin);
    xSemaphoreGive(d->mutex);

    /* Callback block: application code may use public APIs because neither lock
     * is held. */
    if (notify && d->config.on_fault) {
      d->config.on_fault(d, &event, d->config.callback_context);
    }
  }

done:
  xSemaphoreGive(d->worker_done);

  /* worker_done permits the caller to free d; never dereference it again. */
  vTaskDelete(NULL);
}

/**
 * Internal pure validation called only by esp_drv8833_create().
 * @param c Possibly NULL configuration to inspect without allocating resources.
 * @return True for generic ranges/mapping/distinct valid GPIOs, false
 * otherwise. Backend resource ranges/frequency resolution and board wiring are
 * checked elsewhere; this function cannot establish pin availability on a
 * board.
 */
static bool valid_config(const esp_drv8833_config_t *c) {
  if (!c || !c->pwm_freq_hz || c->reversal_coast_us > 1000 ||
      c->fault_context_window_us > 1000000 ||
      (c->left_bridge != ESP_DRV8833_BRIDGE_A &&
       c->left_bridge != ESP_DRV8833_BRIDGE_B) ||
      c->sleep_gpio < -1 || c->fault_gpio < -1 ||
      (c->on_fault && c->fault_gpio < 0))
    return false;

  /* GPIO validation block: optional -1 is accepted only for EEP/ULT; no
   * aliasing. */
  int pins[] = {c->in1_gpio, c->in2_gpio,   c->in3_gpio,
                c->in4_gpio, c->sleep_gpio, c->fault_gpio};
  for (unsigned i = 0; i < 6; i++) {
    if (i >= 4 && pins[i] == -1)
      continue;
    if (i == 5 ? !GPIO_IS_VALID_GPIO(pins[i])
               : !GPIO_IS_VALID_OUTPUT_GPIO(pins[i]))
      return false;
    for (unsigned j = 0; j < i; j++)
      if (pins[j] == pins[i])
        return false;
  }
  return true;
}

/**
 * Public application-only constructor; full parameter/lifetime contract is in
 * esp_drv8833.h. Calls valid_config(), transition helpers, backend create,
 * fault_low() and signal_fault(); starts fault_worker() and registers
 * fault_isr(). All callback-visible state is initialized before enabling the
 * source. On failure, release acquired resources while preserving the original
 * error.
 */
esp_err_t esp_drv8833_create(const esp_drv8833_config_t *c,
                             esp_drv8833_handle_t *out) {
  if (!out)
    return ESP_ERR_INVALID_ARG;
  *out = NULL;
  if (!valid_config(c))
    return ESP_ERR_INVALID_ARG;

  /* Allocation block: keep ISR-visible state in internal byte-addressable RAM.
   */
  esp_drv8833_handle_t d =
      heap_caps_calloc(1, sizeof(*d), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!d)
    return ESP_ERR_NO_MEM;

  d->config = *c;
  d->lifecycle_core = xPortGetCoreID();
  d->spin = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;
  transition_begin(d, ESP_DRV8833_FAULT_CONTEXT_INITIALIZATION);
  d->mutex = xSemaphoreCreateMutex();
  d->worker_done = xSemaphoreCreateBinary();
  esp_err_t err = ESP_ERR_NO_MEM;
  if (!d->mutex || !d->worker_done)
    goto fail;
  xSemaphoreTake(d->mutex, portMAX_DELAY);

  /* EEP setup block: preload LOW before enabling output drive. */
  if (c->sleep_gpio >= 0) {
    err = gpio_set_level(c->sleep_gpio, 0);
    if (err != ESP_OK)
      goto fail;
    gpio_config_t pin = {
        .pin_bit_mask = 1ULL << c->sleep_gpio,
        .mode = GPIO_MODE_OUTPUT,
    };
    err = gpio_config(&pin);
    if (err != ESP_OK)
      goto fail;
    d->sleep_ready = true;
    d->state.sleeping = true;
  }

  /* Coast setup block: establish all four LOW levels before PWM owns the pins.
   */
  int inputs[] = {c->in1_gpio, c->in2_gpio, c->in3_gpio, c->in4_gpio};
  uint64_t mask = 0;
  for (unsigned i = 0; i < 4; i++) {
    err = gpio_set_level(inputs[i], 0);
    if (err != ESP_OK)
      goto fail;
    mask |= 1ULL << inputs[i];
  }
  gpio_config_t input_outputs = {.pin_bit_mask = mask,
                                 .mode = GPIO_MODE_OUTPUT};
  err = gpio_config(&input_outputs);
  if (err != ESP_OK)
    goto fail;

  /* Backend block: exactly one compiled implementation owns the PWM resources.
   */
  err = drv8833_pwm_create(c, &d->pwm);
  if (err != ESP_OK)
    goto fail;

  /* ULT setup block: configure input first, then prepare worker, handler and
   * trigger. */
  if (c->fault_gpio >= 0) {
    gpio_config_t pin = {
        .pin_bit_mask = 1ULL << c->fault_gpio,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    err = gpio_config(&pin);
    if (err != ESP_OK)
      goto fail;
    d->fault_ready = true;
    err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
      goto fail;
    if (xTaskCreatePinnedToCore(fault_worker, "drv8833_fault", 3072, d, 5,
                                &d->worker, 0) != pdPASS) {
      err = ESP_ERR_NO_MEM;
      goto fail;
    }
    err = gpio_isr_handler_add(c->fault_gpio, fault_isr, d);
    if (err != ESP_OK)
      goto fail;
    d->isr_added = true;
    err = gpio_set_intr_type(c->fault_gpio, GPIO_INTR_NEGEDGE);
    if (err != ESP_OK)
      goto fail;
    err = gpio_intr_enable(c->fault_gpio);
    if (err != ESP_OK)
      goto fail;

    /* Initial-level block: recognize low without requiring a new falling edge.
     */
    if (fault_low(d)) {
      signal_fault(d);
    }
  }

  /* Publication block: callback delivery can begin after releasing this mutex.
   */
  transition_end(d);
  d->ready = true;
  *out = d;
  xSemaphoreGive(d->mutex);
  return ESP_OK;

  /* Rollback block: ready remains false here, so no user callback is
   * dispatched. */

fail:
  if (d->mutex && d->worker_done)
    xSemaphoreGive(d->mutex);
  if (d->isr_added)
    gpio_isr_handler_remove(c->fault_gpio);
  if (d->worker) {
    xTaskNotify(d->worker, EXIT_EVENT, eSetBits);
    xSemaphoreTake(d->worker_done, portMAX_DELAY);
  }
  drv8833_pwm_delete(d->pwm);
  if (d->sleep_ready)
    gpio_set_level(c->sleep_gpio, 0);
  if (d->fault_ready)
    gpio_reset_pin(c->fault_gpio);
  if (d->mutex)
    vSemaphoreDelete(d->mutex);
  if (d->worker_done)
    vSemaphoreDelete(d->worker_done);
  free(d);
  return err;
}

/**
 * Public command called by the application and internally by
 * esp_drv8833_coast(). See esp_drv8833.h for units, errors and task-context
 * restrictions. Calls transition helpers, wake_locked(), backend writes and
 * stop_locked() on failure. ULT state is intentionally absent from the
 * motor-command acceptance policy.
 */
esp_err_t esp_drv8833_set_speed(esp_drv8833_handle_t d, float right,
                                float left) {
  if (!d || !isfinite(right) || !isfinite(left) || fabsf(right) > 100.0f ||
      fabsf(left) > 100.0f)
    return ESP_ERR_INVALID_ARG;

  if (!take(d))
    return ESP_ERR_TIMEOUT;

  esp_err_t err = ESP_OK;
  bool transition = false;
  if (d->closing) {
    err = ESP_ERR_INVALID_STATE;
    goto done;
  }

  /* Zero-command block: common stop/sleep semantics also cover the
   * esp_drv8833_coast() wrapper. */
  if (right == 0.0f && left == 0.0f) {
    err = stop_locked(d);
    goto done;
  }

  /* Mapping block: right/left identity is independent of physical wiring
   * polarity. */
  float command[2];
  command[d->config.left_bridge] = d->config.invert_left ? -left : left;
  command[1 - d->config.left_bridge] = d->config.invert_right ? -right : right;

  /* Classification block: prioritize wake, reversal, startup, then partial
   * stop. */
  bool reversing = false;
  for (unsigned b = 0; b < 2; b++) {
    int direction = (command[b] > 0.0f) - (command[b] < 0.0f);
    reversing |= d->direction[b] && direction && d->direction[b] != direction;
  }
  bool starting = (right != 0.0f && d->state.right_percent == 0.0f) ||
                  (left != 0.0f && d->state.left_percent == 0.0f);
  bool stopping = (right == 0.0f && d->state.right_percent != 0.0f) ||
                  (left == 0.0f && d->state.left_percent != 0.0f);
  esp_drv8833_fault_context_t context =
      d->state.sleeping ? ESP_DRV8833_FAULT_CONTEXT_WAKE
      : reversing       ? ESP_DRV8833_FAULT_CONTEXT_REVERSAL
      : starting        ? ESP_DRV8833_FAULT_CONTEXT_MOTOR_START
      : stopping        ? ESP_DRV8833_FAULT_CONTEXT_COAST
                        : ESP_DRV8833_FAULT_CONTEXT_NONE;
  transition = context != ESP_DRV8833_FAULT_CONTEXT_NONE;
  if (transition)
    transition_begin(d, context);

  /* Wake block: complete the requested startup interval before applying drive.
   */
  err = wake_locked(d);
  if (err != ESP_OK)
    goto fail;
  for (unsigned b = 0; b < 2; b++) {
    int direction = (command[b] > 0.0f) - (command[b] < 0.0f);
    if (d->direction[b] && direction && d->direction[b] != direction) {
      for (unsigned side = 0; side < 2; side++) {
        err = drv8833_pwm_set(d->pwm, 2 * b + side, 0.0f);
        if (err != ESP_OK)
          goto fail;
      }
    }
  }
  if (reversing)
    esp_rom_delay_us(d->config.reversal_coast_us);

  /* Actuation block: force the inactive input low before changing the active
   * PWM. */
  for (unsigned b = 0; b < 2; b++) {
    unsigned active = 2 * b + (command[b] < 0.0f);
    unsigned inactive = 2 * b + (command[b] >= 0.0f);
    err = drv8833_pwm_set(d->pwm, inactive, 0.0f);
    if (err != ESP_OK)
      goto fail;
    err = drv8833_pwm_set(d->pwm, active, fabsf(command[b]));
    if (err != ESP_OK)
      goto fail;
    d->direction[b] = (command[b] > 0.0f) - (command[b] < 0.0f);
  }

  /* Commit block: publish logical commands only after both bridge updates
   * succeed. */
  d->state.right_percent = right;
  d->state.left_percent = left;
  goto done;

  /* Failure block: return the original write error; expose the stop attempt
   * separately. */

fail:
  stop_locked(d);

done:
  if (transition)
    transition_end(d);
  xSemaphoreGive(d->mutex);
  return err;
}

/**
 * Public application-only wrapper; delegates to esp_drv8833_set_speed(d,0,0).
 * No component function invokes this wrapper. See the header for task/error
 * contract; both motors freewheel and optional EEP goes low, without braking.
 */
esp_err_t esp_drv8833_coast(esp_drv8833_handle_t d) {
  return esp_drv8833_set_speed(d, 0.0f, 0.0f);
}

/**
 * Public application/callback-only acknowledgement; no internal callers.
 * See the header for pending/shutdown/timeout returns. The pending slot must be
 * copied before clearing the latch; this never changes PWM, EEP, counts,
 * context or the physical ULT input, even if the input remains low.
 */
esp_err_t esp_drv8833_clear_fault(esp_drv8833_handle_t d) {
  if (!d)
    return ESP_ERR_INVALID_ARG;

  if (!take(d))
    return ESP_ERR_TIMEOUT;

  esp_err_t err = ESP_ERR_INVALID_STATE;
  if (d->closing)
    goto done;

  /* Acknowledgement block: refusing a still-occupied slot prevents lost
   * accepted events. */
  portENTER_CRITICAL(&d->spin);
  if (d->event_pending) {
    err = ESP_ERR_INVALID_STATE;
  } else {
    d->latched = false;
    err = ESP_OK;
  }
  portEXIT_CRITICAL(&d->spin);

done:
  xSemaphoreGive(d->mutex);
  return err;
}

/**
 * Public application/callback-only readback; no internal callers.
 * See the header for destination ownership and timeout returns. Task fields use
 * mutex, ISR fields use spin, and physical ULT is sampled separately. No output
 * write or allocation occurs; esp_drv8833_get_state() remains usable for a
 * closing handle.
 */
esp_err_t esp_drv8833_get_state(esp_drv8833_handle_t d,
                                esp_drv8833_state_t *state) {
  if (!d || !state)
    return ESP_ERR_INVALID_ARG;

  if (!take(d))
    return ESP_ERR_TIMEOUT;

  /* Snapshot block: software state is coherent; GPIO level is a separate live
   * sample. */
  *state = d->state;
  state->fault_pin_low = fault_low(d);
  portENTER_CRITICAL(&d->spin);
  state->fault_latched = d->latched;
  state->fault_event_pending = d->event_pending;
  state->fault_falling_edges = d->edges;
  state->last_fault_event = d->pending_event;
  portEXIT_CRITICAL(&d->spin);
  xSemaphoreGive(d->mutex);
  return ESP_OK;
}

/**
 * Public application-only destructor; no internal callers. Calls stop_locked()
 * and backend delete after synchronizing ISR/worker lifetime. See the header
 * for the distinction between retryable early returns and final cleanup errors
 * that still invalidate the handle. Never remove a shared global GPIO service.
 */
esp_err_t esp_drv8833_delete(esp_drv8833_handle_t d) {
  if (!d)
    return ESP_ERR_INVALID_ARG;

  /* Context block: handler removal must occur on the pinned GPIO-service core.
   */
  if (d->config.fault_gpio >= 0 && xPortGetCoreID() != d->lifecycle_core)
    return ESP_ERR_INVALID_STATE;

  if (d->worker && xTaskGetCurrentTaskHandle() == d->worker)
    return ESP_ERR_INVALID_STATE;

  if (!take(d))
    return ESP_ERR_TIMEOUT;

  /* Shutdown block: disallow new commands before releasing the task mutex. */
  d->closing = true;
  esp_err_t first = stop_locked(d);
  xSemaphoreGive(d->mutex);

  /* ISR teardown block: keep state allocated if handler removal cannot be
   * confirmed. */
  if (d->isr_added) {
    esp_err_t e = gpio_isr_handler_remove(d->config.fault_gpio);
    if (e != ESP_OK)
      return e;
    d->isr_added = false;
  }

  /* Worker teardown block: send EXIT only once, including timeout retry paths.
   */
  if (d->worker) {
    if (!d->worker_stop_sent) {
      xTaskNotify(d->worker, EXIT_EVENT, eSetBits);
      d->worker_stop_sent = true;
    }
    if (xSemaphoreTake(d->worker_done, wait_ticks(1000)) != pdTRUE)
      return ESP_ERR_TIMEOUT;
    d->worker = NULL;
  }

  /* Final cleanup block: handle lifetime ends here even when peripheral cleanup
   * fails. */
  esp_err_t e = drv8833_pwm_delete(d->pwm);
  if (first == ESP_OK)
    first = e;
  if (d->fault_ready)
    gpio_reset_pin(d->config.fault_gpio);

  /* Leave initialized EEP as an output LOW instead of allowing it to float. */

  vSemaphoreDelete(d->mutex);
  vSemaphoreDelete(d->worker_done);
  free(d);
  return first;
}
