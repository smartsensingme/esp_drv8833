#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_INVALID_ARG 1
#define ESP_ERR_INVALID_STATE 2
#define ESP_ERR_NO_MEM 3
#define ESP_ERR_TIMEOUT 4
#define IRAM_ATTR
#define MALLOC_CAP_INTERNAL 1
#define MALLOC_CAP_8BIT 2
static inline void *heap_caps_calloc(size_t n, size_t size, unsigned caps) {
  (void)caps;
  return calloc(n, size);
}

typedef unsigned TickType_t;
typedef int BaseType_t;
typedef void *SemaphoreHandle_t;
typedef void *TaskHandle_t;
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(p) ((void)(p))
#define portEXIT_CRITICAL(p) ((void)(p))
#define portENTER_CRITICAL_ISR(p) ((void)(p))
#define portEXIT_CRITICAL_ISR(p) ((void)(p))
#define portYIELD_FROM_ISR() ((void)0)
#define portMAX_DELAY UINT32_MAX
#define portTICK_PERIOD_MS 1
#define pdTRUE 1
#define pdFALSE 0
#define pdPASS 1
#define eSetBits 0
static inline int xPortGetCoreID(void) { return 0; }
static inline SemaphoreHandle_t xSemaphoreCreateMutex(void) {
  return malloc(1);
}
static inline SemaphoreHandle_t xSemaphoreCreateBinary(void) {
  return malloc(1);
}
static inline BaseType_t xSemaphoreTake(SemaphoreHandle_t s, TickType_t ticks) {
  (void)s;
  (void)ticks;
  return pdTRUE;
}
static inline void xSemaphoreGive(SemaphoreHandle_t s) { (void)s; }
static inline void vSemaphoreDelete(SemaphoreHandle_t s) { free(s); }
static inline void vTaskDelay(TickType_t ticks) { (void)ticks; }
static inline void vTaskDelete(TaskHandle_t t) { (void)t; }
static inline TaskHandle_t xTaskGetCurrentTaskHandle(void) { return NULL; }
extern void (*mock_worker)(void *);
extern void *mock_worker_arg;
extern uint32_t mock_notifications;
extern unsigned mock_fault_notifications;
static inline int xTaskCreatePinnedToCore(void (*fn)(void *), const char *name,
                                          unsigned stack, void *arg,
                                          unsigned priority, TaskHandle_t *out,
                                          int core) {
  mock_worker = fn;
  mock_worker_arg = arg;
  mock_notifications = 0;
  (void)name;
  (void)stack;
  (void)arg;
  (void)priority;
  (void)core;
  *out = (void *)(uintptr_t)1;
  return pdPASS;
}
static inline void xTaskNotify(TaskHandle_t t, uint32_t bits, int mode) {
  (void)t;
  mock_notifications |= bits;
  if (bits & 1)
    mock_fault_notifications++;
  (void)mode;
}
static inline void xTaskNotifyFromISR(TaskHandle_t t, uint32_t bits, int mode,
                                      BaseType_t *higher) {
  (void)t;
  mock_notifications |= bits;
  if (bits & 1)
    mock_fault_notifications++;
  (void)mode;
  *higher = pdFALSE;
}
static inline void xTaskNotifyWait(uint32_t entry, uint32_t exit,
                                   uint32_t *bits, TickType_t wait) {
  (void)entry;
  (void)exit;
  (void)wait;
  *bits = mock_notifications ? mock_notifications : 2;
  mock_notifications = 0;
}

extern int mock_levels[64];
extern void (*mock_isr)(void *);
extern void *mock_isr_arg;
extern void (*mock_on_delay)(void);
extern unsigned mock_delay_us;
extern int64_t mock_time_us;
static inline int64_t esp_timer_get_time(void) { return mock_time_us; }
typedef struct {
  uint64_t pin_bit_mask;
  int mode;
  int pull_up_en;
  int intr_type;
} gpio_config_t;
#define GPIO_MODE_OUTPUT 1
#define GPIO_MODE_INPUT 2
#define GPIO_PULLUP_ENABLE 1
#define GPIO_INTR_DISABLE 0
#define GPIO_INTR_NEGEDGE 1
#define GPIO_IS_VALID_GPIO(pin) ((pin) >= 0 && (pin) < 22)
#define GPIO_IS_VALID_OUTPUT_GPIO(pin) GPIO_IS_VALID_GPIO(pin)
static inline esp_err_t gpio_config(const gpio_config_t *c) {
  (void)c;
  return ESP_OK;
}
static inline esp_err_t gpio_set_level(int pin, int level) {
  mock_levels[pin] = level;
  return ESP_OK;
}
static inline int gpio_get_level(int pin) { return mock_levels[pin]; }
static inline esp_err_t gpio_reset_pin(int pin) {
  mock_levels[pin] = 0;
  return ESP_OK;
}
static inline esp_err_t gpio_install_isr_service(int flags) {
  (void)flags;
  return ESP_OK;
}
static inline esp_err_t gpio_isr_handler_add(int pin, void (*fn)(void *),
                                             void *arg) {
  (void)pin;
  mock_isr = fn;
  mock_isr_arg = arg;
  return ESP_OK;
}
static inline esp_err_t gpio_isr_handler_remove(int pin) {
  (void)pin;
  mock_isr = NULL;
  return ESP_OK;
}
static inline esp_err_t gpio_set_intr_type(int pin, int type) {
  (void)pin;
  (void)type;
  return ESP_OK;
}
static inline esp_err_t gpio_intr_enable(int pin) {
  (void)pin;
  return ESP_OK;
}
static inline void esp_rom_delay_us(unsigned us) {
  mock_delay_us += us;
  mock_time_us += us;
  if (mock_on_delay)
    mock_on_delay();
}
