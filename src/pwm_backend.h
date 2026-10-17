/**
 * @file pwm_backend.h
 * @brief Private interface implemented by exactly one compile-selected backend.
 *
 * Application code includes esp_drv8833.h instead. Generic configuration and
 * finite-duty validation are provided by the command layer. No backend offers
 * its own task synchronization, ISR API, fault policy or wheel mapping.
 */
#pragma once
#include "esp_drv8833.h"

/**
 * Opaque backend-owned allocation. Create returns ownership to the command
 * layer; delete consumes it, including when final cleanup reports an error.
 */
typedef struct drv8833_pwm drv8833_pwm_t;

/**
 * Internal-only constructor called by esp_drv8833_create().
 * @param config Non-NULL generic-validated configuration; frequency is
 * positive.
 * @param out Non-NULL output, set to NULL on failure.
 * @return ESP_OK, invalid configuration/resolution, no-memory/resource errors,
 * or an IDF backend error. Task context; allocation and rollback allowed.
 */
esp_err_t drv8833_pwm_create(const esp_drv8833_config_t *config,
                             drv8833_pwm_t **out);

/**
 * Internal-only write called by esp_drv8833_set_speed() and stop_locked().
 * @param pwm Valid backend, under the command-layer mutex.
 * @param index Physical IN1..IN4 index, 0..3.
 * @param duty Finite magnitude, 0..100 percent; nearest hardware tick is
 * applied.
 * @return ESP_OK or first IDF error. Zero forces a LOW idle/force path rather
 * than waiting for a zero comparator pulse. No allocation in normal commands.
 * Caller must enforce all ranges, inactive-input ordering and exclusive
 * ownership.
 */
esp_err_t drv8833_pwm_set(drv8833_pwm_t *pwm, unsigned index, float duty);

/**
 * Internal-only destructor called by esp_drv8833_delete(), constructor rollback
 * and the selected backend constructor on its own partial failure.
 * @param pwm Complete/partial owned backend, or NULL (no-op).
 * @return First teardown error after best-effort remaining cleanup; storage is
 * freed on every non-NULL call. Never retry a consumed backend pointer.
 * Task-only.
 */
esp_err_t drv8833_pwm_delete(drv8833_pwm_t *pwm);
