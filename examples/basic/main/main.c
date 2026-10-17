/**
 * @file main.c
 * @brief Open-loop bench sequence; pin configuration comes from menuconfig.
 *
 * The peripheral maintains PWM after app_main returns. ULT callbacks report
 * temporal context and explicitly rearm the latch without stopping the motors.
 * This is not a velocity regulator or a periodic acquisition/control loop.
 */
#include "esp_drv8833.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

/**
 * Internal presentation helper called only by on_fault().
 * @param context Captured enum value; unknown values use the fallback name.
 * @return Borrowed static string, never freed or modified. Task-only logging
 * use; no allocation and no driver state change.
 */
static const char *fault_context_name(esp_drv8833_fault_context_t context) {
  switch (context) {
  case ESP_DRV8833_FAULT_CONTEXT_INITIALIZATION:
    return "initialization";

  case ESP_DRV8833_FAULT_CONTEXT_WAKE:
    return "wake";

  case ESP_DRV8833_FAULT_CONTEXT_MOTOR_START:
    return "motor_start";

  case ESP_DRV8833_FAULT_CONTEXT_COAST:
    return "coast";

  case ESP_DRV8833_FAULT_CONTEXT_REVERSAL:
    return "reversal";

  default:
    return "unassociated";
  }
}

/**
 * Application callback registered by app_main(), invoked externally by the
 * component fault_worker() on Core 0 outside driver locks.
 * @param driver Borrowed instance used for explicit acknowledgement.
 * @param event Read-only captured value, valid only for this invocation.
 * @param context Unused borrowed application argument (NULL in this example).
 * Calls fault_context_name(), logging and esp_drv8833_clear_fault(). No stop is
 * applied and no event is discarded based on context. Reports acknowledgement
 * errors instead of assuming every clear succeeded; formatted logs belong
 * outside the ISR.
 */
static void on_fault(esp_drv8833_handle_t driver,
                     const esp_drv8833_fault_event_t *event, void *context) {
  (void)context;

  /* Presentation block: log service-time association, not a proven electrical
   * cause. */
  ESP_LOGW("example", "ULT: context=%s, phase=%s, elapsed=%lu us, edges=%lu",
           fault_context_name(event->context),
           event->transition_active ? "during" : "after/outside",
           (unsigned long)event->transition_elapsed_us,
           (unsigned long)event->falling_edges);

  /* Policy block: the application chooses to acknowledge every delivered
   * signal. */
  esp_err_t err = esp_drv8833_clear_fault(driver);
  if (err != ESP_OK)
    ESP_LOGE("example", "ULT acknowledgement: %s", esp_err_to_name(err));
}

/**
 * External ESP-IDF application entry, called by the main task; no internal
 * callers. Configures the component using menuconfig GPIOs and fixed C wheel
 * mapping, then executes the user bench sequence: 3-s wait, 70% drive, 9-s
 * wait, coast, 2-s wait, 75% drive. Creation aborts on error; this minimal
 * example checks only the final drive result, not every intermediate command
 * return. The instance is intentionally kept alive after return so PWM and
 * optional fault worker continue; use explicit coast/delete in an application
 * shutdown.
 */
void app_main(void) {
  /* Configuration block: only GPIOs come from Kconfig; mapping/polarity stay in
   * C. */
  esp_drv8833_config_t config = ESP_DRV8833_CONFIG_DEFAULT();
  config.in1_gpio = CONFIG_EXAMPLE_DRV8833_IN1;
  config.in2_gpio = CONFIG_EXAMPLE_DRV8833_IN2;
  config.in3_gpio = CONFIG_EXAMPLE_DRV8833_IN3;
  config.in4_gpio = CONFIG_EXAMPLE_DRV8833_IN4;
  config.sleep_gpio = CONFIG_EXAMPLE_DRV8833_SLEEP;
  config.fault_gpio = CONFIG_EXAMPLE_DRV8833_FAULT;
  config.left_bridge = ESP_DRV8833_BRIDGE_A;
  config.invert_left = false;
  config.invert_right = false;
  if (config.fault_gpio >= 0)
    config.on_fault = on_fault;

  /* Lifecycle block: initialize before starting motion; ULT can be signalled
   * here. */
  esp_drv8833_handle_t driver = NULL;
  ESP_ERROR_CHECK(esp_drv8833_create(&config, &driver));

  /* First drive block: hardware PWM continues throughout task delays. */
  ESP_LOGI("example", "Starting both motors in 3 seconds");
  vTaskDelay(pdMS_TO_TICKS(3000));
  esp_err_t err = esp_drv8833_set_speed(driver, 70.0f, 70.0f);
  ESP_LOGI("example", "Starting both motors in 9 seconds");
  vTaskDelay(pdMS_TO_TICKS(9000));

  /* Stop/restart block: both zero also lowers EEP when the optional pin exists.
   */
  esp_drv8833_coast(driver);
  vTaskDelay(pdMS_TO_TICKS(2000));
  err = esp_drv8833_set_speed(driver, 75.0f, 75.0f);
  ESP_LOGI("example", "ALL DONE!");

  /* Result block: report final command failure; the component attempts
   * best-effort stop. */
  if (err != ESP_OK) {
    ESP_LOGE("example", "Command rejected: %s; motors remain stopped",
             esp_err_to_name(err));
  }
}
