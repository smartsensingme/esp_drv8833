/**
 * @file esp_drv8833.h
 * @brief Public task-context API for one dual-motor DRV8833 instance.
 *
 * Signed commands are logical right/left duty percentages, not measured speed.
 * ULT is notification-only. GPIO/PWM resources belong exclusively to the
 * instance; the application owns motor regulation and the fault-response
 * policy.
 */
#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Opaque instance allocated by esp_drv8833_create(), borrowed by
 * commands/callbacks and invalidated when destruction reaches final cleanup,
 * including cleanup errors. Never copy/dereference its contents. A null handle
 * is invalid for every public operation except initialization of the caller's
 * output variable before esp_drv8833_create(). */
typedef struct esp_drv8833 *esp_drv8833_handle_t;

/** Physical output bridge assigned to the logical left wheel. */
typedef enum {
  /** IN1/IN2 drive OUT1/OUT2; the right wheel uses bridge B. */
  ESP_DRV8833_BRIDGE_A = 0,

  /** IN3/IN4 drive OUT3/OUT4; the right wheel uses bridge A. */
  ESP_DRV8833_BRIDGE_B = 1,
} esp_drv8833_bridge_t;

/** Temporal association captured at ISR service/initial GPIO read time.
 * Never proves that the transition caused ULT, or that an event is harmless.
 * Simultaneous command changes prioritize WAKE, REVERSAL, MOTOR_START, COAST.
 */
typedef enum {
  /** No active/recent transition at capture; this does not diagnose the cause.
   */
  ESP_DRV8833_FAULT_CONTEXT_NONE = 0,

  /** Instance initialization, including a pre-existing low ULT input. */
  ESP_DRV8833_FAULT_CONTEXT_INITIALIZATION,

  /** EEP low-to-high wake operation, before PWM drive is applied. */
  ESP_DRV8833_FAULT_CONTEXT_WAKE,

  /** Zero-to-nonzero wheel command while awake, or without an EEP GPIO. */
  ESP_DRV8833_FAULT_CONTEXT_MOTOR_START,

  /** One/both wheels stop, including sleep entry or a stop attempt on error. */
  ESP_DRV8833_FAULT_CONTEXT_COAST,

  /** Nonzero command changes sign, including the intermediate coast interval.
   */
  ESP_DRV8833_FAULT_CONTEXT_REVERSAL,
} esp_drv8833_fault_context_t;

/** Immutable value snapshot of one accepted ULT signal.
 * The first capture is retained until dispatch; later edges only advance the
 * live counter. Copy this value to retain it after callback return. No wire
 * format, packing, serialization or stable binary ABI is defined for this type.
 */
typedef struct {
  /** Serviced falling-edge count at acceptance, modulo 2^32. May be zero for
   * ULT already low at esp_drv8833_create(). Not the number of failed motors.
   */
  uint32_t falling_edges;

  /** Most recent active/recent transition, or NONE outside the configured
   * window. */
  esp_drv8833_fault_context_t context;

  /** esp_timer time in microseconds at capture, not exact physical edge time
   * and not callback delivery time. Counter origin is the ESP timer timebase.
   */
  int64_t timestamp_us;

  /** Microseconds since associated transition began; saturated at UINT32_MAX.
   * Includes active duration plus trailing delay; zero when context is NONE. */
  uint32_t transition_elapsed_us;

  /** True during the transition operation; false in its trailing window or
   * with context NONE. Task preemption can extend the active interval. */
  bool transition_active;
} esp_drv8833_fault_event_t;

/**
 * @brief Application callback invoked internally only by fault_worker().
 *
 * Runs on the driver's Core-0 priority-5 task, after releasing its mutex.
 * May call command/state/clear APIs, but never delete this driver. Keep work
 * bounded: slow callbacks delay future delivery and may time out destruction.
 * Pending events may be dropped during shutdown. No automatic stop is applied.
 * The callback can run as soon as esp_drv8833_create() publishes the instance,
 * so prepare the context before esp_drv8833_create(), rather than initializing
 * it after return.
 *
 * @param driver Borrowed originating instance; the application retains
 * ownership.
 * @param event Read-only worker-stack snapshot, valid only during this call.
 * @param context Borrowed config.callback_context, possibly NULL; never freed.
 */
typedef void (*esp_drv8833_fault_callback_t)(
    esp_drv8833_handle_t driver, const esp_drv8833_fault_event_t *event,
    void *context);

/** Configuration copied by esp_drv8833_create(); edits after creation have no
 * effect. Initialize with ESP_DRV8833_CONFIG_DEFAULT(), then assign four
 * mandatory distinct output GPIOs. Backend-unused fields are ignored. Optional
 * context is shallow-copied and must outlive the instance. Recreate to
 * reconfigure.
 */
typedef struct {
  /** Mandatory output GPIO for physical bridge A input 1; default -1 is
   * invalid. */
  int in1_gpio;

  /** Mandatory output GPIO for physical bridge A input 2; default -1 is
   * invalid. */
  int in2_gpio;

  /** Mandatory output GPIO for physical bridge B input 1; default -1 is
   * invalid. */
  int in3_gpio;

  /** Mandatory output GPIO for physical bridge B input 2; default -1 is
   * invalid. */
  int in4_gpio;

  /** EEP/nSLEEP output, or -1 when absent (default). Both commands zero drive
   * EEP low; a nonzero command wakes the bridge with a 1000-us busy wait. */
  int sleep_gpio;

  /** ULT/nFAULT input with internal pull-up, or -1 when absent (default).
   * Active low; enables notification-only ISR/worker, never motor inhibition.
   */
  int fault_gpio;

  /** Bridge assigned to the left wheel, A by default; right uses the other.
   * Accepts only ESP_DRV8833_BRIDGE_A or ESP_DRV8833_BRIDGE_B. */
  esp_drv8833_bridge_t left_bridge;

  /** False by default; true reverses left-wheel logical polarity, not
   * magnitude. */
  bool invert_left;

  /** False by default; true reverses right-wheel logical polarity
   * independently. */
  bool invert_right;

  /** Common positive carrier frequency in Hz, default 20000. Individual duties
   * are independent. LEDC must resolve the requested frequency; MCPWM requires
   * 80000000/frequency to be an integer period in 2..65535 ticks. */
  uint32_t pwm_freq_hz;

  /** Additional software coast delay on driven reversal, 0..1000 us,
   * default 50. Runs in the calling task; not the IC internal dead time or a
   * speed ramp. */
  uint32_t reversal_coast_us;

  /** Post-transition association window, 0..1000000 us, default 2000.
   * Zero tags only captures during the transition; never filters an event. */
  uint32_t fault_context_window_us;

  /** LEDC-only exclusive timer index 0..3, default 0; ignored by MCPWM. */
  int ledc_timer;

  /** LEDC-only first of four exclusive consecutive channels, default 0.
   * Range 0..2 on C3, 0..4 on S3; ignored by MCPWM. */
  int ledc_first_channel;

  /** MCPWM-only group index, 0 or 1 on S3, default 0. Requires resources for
   * one timer/two operators/four comparator-generator pairs; ignored by LEDC.
   */
  int mcpwm_group;

  /** Optional callback, NULL by default. Requires fault_gpio >= 0 when set.
   * Without it, ULT state can still be polled and explicitly acknowledged. */
  esp_drv8833_fault_callback_t on_fault;

  /** Borrowed callback argument, NULL by default. Driver does not copy/free
   * the pointed-to object; it must remain valid until destruction finishes. */
  void *callback_context;
} esp_drv8833_config_t;

/** Aggregate initializer with board-neutral GPIOs (-1), bridge A as left,
 * inversion false, 20-kHz PWM, 50-us reversal coast, 2000-us context window,
 * backend indexes zero and callback/context NULL. The omitted aggregate fields
 * are initialized to zero. Does not allocate resources or create an instance.
 */
#define ESP_DRV8833_CONFIG_DEFAULT()                                           \
  {.in1_gpio = -1,                                                             \
   .in2_gpio = -1,                                                             \
   .in3_gpio = -1,                                                             \
   .in4_gpio = -1,                                                             \
   .sleep_gpio = -1,                                                           \
   .fault_gpio = -1,                                                           \
   .left_bridge = ESP_DRV8833_BRIDGE_A,                                        \
   .pwm_freq_hz = 20000,                                                       \
   .reversal_coast_us = 50,                                                    \
   .fault_context_window_us = 2000,                                            \
   .ledc_timer = 0,                                                            \
   .ledc_first_channel = 0,                                                    \
   .mcpwm_group = 0}

/** Value-owned readback copied by esp_drv8833_get_state(). No allocation or
 * ownership transfer. Software fields are synchronized; physical ULT is sampled
 * separately and can change before/after the snapshot. Not measured RPM or a
 * guarantee of physical output state after a peripheral error.
 */
typedef struct {
  /** Last completely applied logical right command in -100..100 percent,
   * before polarity inversion. Zero initially/after a successful stop. */
  float right_percent;

  /** Last completely applied logical left command, with the same units/range.
   */
  float left_percent;

  /** True only if EEP is present and the last successful EEP write was low.
   * This is cached GPIO command state, not a measurement of the IC. */
  bool sleeping;

  /** An accepted ULT signal awaits application acknowledgement; never blocks
   * drive. */
  bool fault_latched;

  /** True until the worker copies the accepted signal;
   * esp_drv8833_clear_fault() refuses acknowledgement during this stage to
   * avoid losing the pending event. */
  bool fault_event_pending;

  /** Instantaneous ULT-low sample, false when ULT GPIO is absent.
   * Independent of the retained signal latch and capture snapshot. */
  bool fault_pin_low;

  /** Current serviced ULT falling-edge count, modulo 2^32, including suppressed
   * repeat edges. Initial-low capture does not increment this counter. */
  uint32_t fault_falling_edges;

  /** Most recently accepted signal, retained after acknowledgement and
   * zero-initialized before any signal. Independent of callback delivery. */
  esp_drv8833_fault_event_t last_fault_event;

  /** Result of the latest stop attempt; ESP_OK initially. On error, cached
   * command values may describe an older successful state; inspect API errors.
   */
  esp_err_t last_stop_error;
} esp_drv8833_state_t;

/**
 * @brief Allocate an instance and initialize outputs in coast/optional sleep.
 *
 * External application entry only; no component function calls it. Internally
 * validates config, initializes GPIO/PWM, creates synchronization objects and
 * optionally starts fault_worker() before enabling fault_isr(). Captures ULT
 * already low once. The config is copied; its callback context remains
 * borrowed.
 *
 * @param config Non-NULL input, with exclusive valid pins/PWM resources.
 * @param out Non-NULL destination; set to NULL on failure, owned handle on
 * success.
 * @return ESP_OK, ESP_ERR_INVALID_ARG for config/output errors, ESP_ERR_NO_MEM
 * for allocation/task failure, or a GPIO/PWM resource/initialization error.
 *
 * Task-context lifecycle operation with allocations, not a periodic-loop API.
 * With ULT, caller must be pinned to the GPIO ISR-service core. Sharing an
 * existing service requires the application to ensure that core matches. A
 * newly installed GPIO service is non-IRAM and remains installed on deletion.
 * Handler/capture code uses IRAM and internal instance memory, but task-side
 * operation during flash/cache suspension has no deadline guarantee. Failure
 * attempts cleanup and retains EEP low if already initialized; no handle is
 * returned. Input pins may remain configured low after a failed creation.
 */
esp_err_t esp_drv8833_create(const esp_drv8833_config_t *config,
                             esp_drv8833_handle_t *out);

/**
 * @brief Apply independent signed PWM commands, right first and left second.
 *
 * Called externally by the application and internally by esp_drv8833_coast().
 * Maps/inverts logical commands, classifies a transition, calls wake_locked()
 * and drv8833_pwm_set(), and attempts stop_locked() on a peripheral write
 * error. Zero selects coast for a wheel; both zero also select sleep if EEP
 * exists. Direct nonzero reversals force the affected inputs low before a
 * configured busy wait. ULT signal/latch never inhibits this operation. Pair
 * writes are sequential, not simultaneous hardware transactions.
 *
 * @param driver Valid live instance; lifecycle must not run concurrently.
 * @param right_percent Finite logical right duty in [-100,100], percent.
 * @param left_percent Finite logical left duty in [-100,100], percent.
 * @return ESP_OK; ESP_ERR_INVALID_ARG for null/nonfinite/out-of-range input
 * without changing outputs; ESP_ERR_TIMEOUT on mutex contention;
 * ESP_ERR_INVALID_STATE during shutdown; or first peripheral error, even if
 * the subsequent best-effort stop succeeds. No persistent ULT inhibit is set.
 *
 * Task-only, no allocation/logging. Mutex wait requests 20 ms rounded up to
 * RTOS ticks; scheduler latency is additional. Wake requests 1000 us busy wait;
 * reversal requests 0..1000 us. These delays are not end-to-end WCET bounds.
 * State commands update only after the full pair succeeds or a stop succeeds.
 */
esp_err_t esp_drv8833_set_speed(esp_drv8833_handle_t driver,
                                float right_percent, float left_percent);

/**
 * @brief Request freewheel on both motors and optional EEP sleep.
 *
 * External application entry only. Internally calls
 * esp_drv8833_set_speed(driver,0,0), which calls stop_locked(); no component
 * function invokes this wrapper. Not active braking and not an automatic ULT
 * response. Repeated zero does not renew the COAST context window when cached
 * state is already stopped.
 *
 * @param driver Valid live instance, task context, serialized by the mutex.
 * @return Same argument, timeout, shutdown or peripheral errors as
 * esp_drv8833_set_speed().
 */
esp_err_t esp_drv8833_coast(esp_drv8833_handle_t driver);

/**
 * @brief Acknowledge only the ULT latch and rearm the next notification.
 *
 * External application/callback entry only; no component function calls it.
 * Uses the instance mutex and spinlock, without GPIO, PWM or EEP writes.
 * Does not reset counters, last_fault_event, context windows or motor commands.
 * May clear while ULT is low; another signal then needs a new falling edge.
 * With ULT absent it is a harmless no-op on a live instance.
 *
 * @param driver Valid instance; may be used from on_fault() after dispatch.
 * @return ESP_OK, ESP_ERR_INVALID_ARG for NULL, ESP_ERR_TIMEOUT on mutex
 * contention, or ESP_ERR_INVALID_STATE while an event awaits worker copying
 * or shutdown has begun. Query fault_event_pending or acknowledge in callback.
 * Task-only. Does not acknowledge/repair any electrical condition in the IC.
 */
esp_err_t esp_drv8833_clear_fault(esp_drv8833_handle_t driver);

/**
 * @brief Copy synchronized software state and a separately sampled ULT level.
 *
 * External application/callback entry only; no component function calls it.
 * Copies cached commands under mutex, samples fault_low(), and copies latch,
 * counter and event under spinlock. No PWM write, allocation, log or ownership
 * transfer occurs. A snapshot is not an atomic physical measurement of ULT
 * and all outputs. Query remains available while destruction awaits retry.
 *
 * @param driver Valid allocated instance; must not race its destruction.
 * @param state Non-NULL caller-owned destination, unchanged on entry failure.
 * @return ESP_OK, ESP_ERR_INVALID_ARG for NULL input/output, or
 * ESP_ERR_TIMEOUT on the rounded-up 20-ms mutex wait.
 */
esp_err_t esp_drv8833_get_state(esp_drv8833_handle_t driver,
                                esp_drv8833_state_t *state);

/**
 * @brief Stop and destroy the instance, or retain it on retryable shutdown
 * errors.
 *
 * External application entry only; no component function calls it. Internally
 * calls stop_locked(), removes the GPIO handler, requests worker exit, waits
 * for worker_done and calls drv8833_pwm_delete(). Never uninstalls the global
 * GPIO ISR service; initialized EEP remains an output low after normal cleanup.
 *
 * @param driver Live owned handle. Caller must exclude all concurrent API
 * calls. With ULT, use a pinned task on the creation/GPIO-service core.
 * @return ESP_ERR_INVALID_ARG for NULL; ESP_ERR_INVALID_STATE from callback
 * or a different lifecycle core; ESP_ERR_TIMEOUT on mutex/worker wait;
 * GPIO-handler removal error; or the first stop/backend-cleanup error.
 *
 * Worker wait requests 1000 ms rounded up to RTOS ticks. Timeout on worker or
 * failure removing its ISR retains a closing instance for state query/retry;
 * commands and clear are rejected then. An initial mutex timeout or failed
 * context check leaves the instance untouched. Once backend cleanup is reached,
 * the handle is freed even if cleanup returns an error; never retry that freed
 * handle. Pending notifications can be dropped. This task-only operation can
 * allocate/free through IDF internals and is not for a periodic control loop.
 */
esp_err_t esp_drv8833_delete(esp_drv8833_handle_t driver);

#ifdef __cplusplus
}
#endif
