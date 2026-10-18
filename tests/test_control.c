#include "mock_idf.h"
#include "pwm_backend.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

int mock_levels[64];
void (*mock_isr)(void *);
void *mock_isr_arg;
void (*mock_on_delay)(void);
unsigned mock_delay_us;
int64_t mock_time_us;
void (*mock_worker)(void *);
void *mock_worker_arg;
uint32_t mock_notifications;
unsigned mock_fault_notifications;
static unsigned callback_count;
static esp_drv8833_fault_event_t last_event;
static bool acknowledge_in_callback;

static void on_fault(esp_drv8833_handle_t driver,
                     const esp_drv8833_fault_event_t *event, void *context) {
  assert(context == &callback_count);
  callback_count++;
  last_event = *event;
  esp_drv8833_state_t state;
  assert(esp_drv8833_get_state(driver, &state) == ESP_OK);
  assert(state.fault_latched && !state.fault_event_pending);
  if (acknowledge_in_callback)
    assert(esp_drv8833_clear_fault(driver) == ESP_OK);
}

/* Deterministically execute queued worker dispatches; no real RTOS scheduler.
 */
static void dispatch(void) { mock_worker(mock_worker_arg); }

struct drv8833_pwm {
  float duty[4];
};
static drv8833_pwm_t *active_pwm;
static bool fail_once;
static int fail_high_index = -1;
static unsigned writes;
static void (*on_pwm_write)(void);

esp_err_t drv8833_pwm_create(const esp_drv8833_config_t *config,
                             drv8833_pwm_t **out) {
  (void)config;
  *out = active_pwm = calloc(1, sizeof(**out));
  return ESP_OK;
}
esp_err_t drv8833_pwm_set(drv8833_pwm_t *p, unsigned i, float duty) {
  writes++;
  if (fail_once) {
    fail_once = false;
    return ESP_FAIL;
  }
  p->duty[i] = duty;
  /* No bridge should ever have both inputs active, including reversal. */
  assert(p->duty[0] == 0.0f || p->duty[1] == 0.0f ||
         (p->duty[0] == 100.0f && p->duty[1] == 100.0f));
  assert(p->duty[2] == 0.0f || p->duty[3] == 0.0f ||
         (p->duty[2] == 100.0f && p->duty[3] == 100.0f));
  if (on_pwm_write)
    on_pwm_write();
  return ESP_OK;
}
esp_err_t drv8833_pwm_force_high(drv8833_pwm_t *p, unsigned i) {
  if (fail_once || fail_high_index == (int)i) {
    fail_once = false;
    fail_high_index = -1;
    return ESP_FAIL;
  }
  p->duty[i] = 100.0f;
  return ESP_OK;
}
esp_err_t drv8833_pwm_delete(drv8833_pwm_t *p) {
  free(p);
  active_pwm = NULL;
  return ESP_OK;
}

static esp_drv8833_config_t board(void) {
  esp_drv8833_config_t c = ESP_DRV8833_CONFIG_DEFAULT();
  c.in1_gpio = 0;
  c.in2_gpio = 1;
  c.in3_gpio = 2;
  c.in4_gpio = 3;
  c.sleep_gpio = 10;
  c.fault_gpio = 7;
  mock_levels[7] = 1;
  c.on_fault = on_fault;
  c.callback_context = &callback_count;
  return c;
}
static void stopped(void) {
  for (unsigned i = 0; i < 4; i++)
    assert(active_pwm->duty[i] == 0.0f);
  assert(mock_levels[10] == 0);
}
static void edge(void) {
  assert(mock_isr);
  mock_isr(mock_isr_arg);
}
static void edge_during_wake(void) {
  mock_on_delay = NULL;
  edge(); /* Short pulse has returned high by the task's level read. */
}

static void edge_during_pwm(void) {
  on_pwm_write = NULL;
  edge();
}

static void expect_event(esp_drv8833_handle_t d,
                         esp_drv8833_fault_context_t context, bool active) {
  dispatch();
  assert(last_event.context == context);
  assert(last_event.transition_active == active);
  esp_drv8833_state_t s;
  assert(esp_drv8833_get_state(d, &s) == ESP_OK);
  assert(s.last_fault_event.timestamp_us == last_event.timestamp_us);
  assert(s.last_fault_event.context == context);
  assert(esp_drv8833_clear_fault(d) == ESP_OK);
}

static void test_contexts(void) {
  esp_drv8833_config_t c = board();
  esp_drv8833_handle_t d;
  mock_time_us = 100000;
  assert(esp_drv8833_create(&c, &d) == ESP_OK);
  mock_time_us += 2001;
  edge();
  expect_event(d, ESP_DRV8833_FAULT_CONTEXT_NONE, false);
  assert(last_event.transition_elapsed_us == 0);

  mock_on_delay = edge_during_wake;
  assert(esp_drv8833_set_speed(d, 70, 70) == ESP_OK);
  expect_event(d, ESP_DRV8833_FAULT_CONTEXT_WAKE, true);
  assert(last_event.transition_elapsed_us == 1000);
  mock_time_us += 2000; // Inclusive trailing-window boundary.
  edge();
  int64_t captured_time = mock_time_us;
  assert(esp_drv8833_coast(d) == ESP_OK); // New context before old dispatch.
  mock_time_us += 9000; // Callback delay must not relabel the captured event.
  expect_event(d, ESP_DRV8833_FAULT_CONTEXT_WAKE, false);
  assert(last_event.timestamp_us == captured_time);
  assert(last_event.transition_elapsed_us == 3000);

  assert(esp_drv8833_coast(d) ==
         ESP_OK); // Repeated zero must not renew context.
  edge();
  expect_event(d, ESP_DRV8833_FAULT_CONTEXT_NONE, false);
  assert(esp_drv8833_set_speed(d, 70, 70) == ESP_OK);
  mock_time_us += 2001;
  assert(esp_drv8833_set_speed(d, 75, 75) == ESP_OK); // Ordinary duty change.
  edge();
  expect_event(d, ESP_DRV8833_FAULT_CONTEXT_NONE, false);

  on_pwm_write = edge_during_pwm;
  assert(esp_drv8833_set_speed(d, -75, -75) == ESP_OK);
  expect_event(d, ESP_DRV8833_FAULT_CONTEXT_REVERSAL, true);
  on_pwm_write = edge_during_pwm;
  assert(esp_drv8833_set_speed(d, 0, -75) == ESP_OK);
  expect_event(d, ESP_DRV8833_FAULT_CONTEXT_COAST, true);
  assert(mock_levels[10] == 1);
  on_pwm_write = edge_during_pwm;
  assert(esp_drv8833_set_speed(d, -75, -75) == ESP_OK);
  expect_event(d, ESP_DRV8833_FAULT_CONTEXT_MOTOR_START, true);
  on_pwm_write = edge_during_pwm;
  assert(esp_drv8833_coast(d) == ESP_OK);
  expect_event(d, ESP_DRV8833_FAULT_CONTEXT_COAST, true);
  assert(esp_drv8833_delete(d) == ESP_OK);

  c = board();
  c.sleep_gpio = -1;
  c.fault_context_window_us = 0;
  assert(esp_drv8833_create(&c, &d) == ESP_OK);
  on_pwm_write = edge_during_pwm;
  assert(esp_drv8833_set_speed(d, 70, 70) == ESP_OK);
  expect_event(d, ESP_DRV8833_FAULT_CONTEXT_MOTOR_START, true);
  edge(); // Zero trailing window: even same timestamp after API is outside.
  expect_event(d, ESP_DRV8833_FAULT_CONTEXT_NONE, false);
  assert(esp_drv8833_delete(d) == ESP_OK);
  c.fault_context_window_us = 1000001;
  assert(esp_drv8833_create(&c, &d) == ESP_ERR_INVALID_ARG);
}

int main(void) {
  esp_drv8833_config_t c = board();
  esp_drv8833_handle_t d = NULL;
  esp_drv8833_state_t s;
  assert(esp_drv8833_create(&c, &d) == ESP_OK);
  stopped();
  assert(esp_drv8833_set_speed(d, 30, 60) == ESP_OK);
  assert(mock_delay_us == 1000);
  assert(active_pwm->duty[0] == 60 && active_pwm->duty[2] == 30);
  unsigned before = writes;
  assert(esp_drv8833_set_speed(d, NAN, 30) == ESP_ERR_INVALID_ARG);
  assert(esp_drv8833_set_speed(d, 30, INFINITY) == ESP_ERR_INVALID_ARG);
  assert(esp_drv8833_set_speed(d, 101, 30) == ESP_ERR_INVALID_ARG);
  assert(writes == before);
  assert(esp_drv8833_set_speed(d, -100, 60) == ESP_OK);
  assert(mock_delay_us == 1050);
  assert(active_pwm->duty[2] == 0 && active_pwm->duty[3] == 100);
  assert(esp_drv8833_set_speed(d, 0, 60) == ESP_OK);
  assert(mock_levels[10] ==
         1); /* A stopped wheel must not sleep its sibling. */

  before = writes;
  edge();
  edge();
  edge();
  assert(mock_fault_notifications == 1); // One notification while latched.
  assert(writes == before && mock_levels[10] == 1);
  assert(esp_drv8833_get_state(d, &s) == ESP_OK && s.fault_latched);
  assert(s.fault_event_pending && s.fault_falling_edges == 3);
  assert(esp_drv8833_clear_fault(d) == ESP_ERR_INVALID_STATE); // No lost event.
  assert(esp_drv8833_set_speed(d, 30, 30) == ESP_OK); // Latch is informational.
  before = writes;
  dispatch();
  assert(callback_count == 1 && last_event.falling_edges == 1);
  assert(writes == before); // Worker never writes PWM/EEP.
  assert(active_pwm->duty[0] == 30 && active_pwm->duty[2] == 30);
  edge();
  dispatch();
  assert(callback_count == 1 && mock_fault_notifications == 1);
  mock_levels[7] = 0;
  assert(esp_drv8833_set_speed(d, 40, 50) ==
         ESP_OK); // Even persistent ULT low.
  before = writes;
  unsigned delay_before = mock_delay_us;
  assert(esp_drv8833_clear_fault(d) == ESP_OK); // Acknowledge while ULT is low.
  assert(writes == before && mock_delay_us == delay_before);
  assert(esp_drv8833_get_state(d, &s) == ESP_OK && !s.fault_latched);
  assert(s.fault_pin_low && s.right_percent == 40 && s.left_percent == 50);
  dispatch();
  assert(callback_count == 1); // No immediate notification for existing low.
  mock_levels[7] = 1;
  edge();
  dispatch();
  assert(callback_count == 2 && last_event.falling_edges == 5);
  assert(esp_drv8833_clear_fault(d) == ESP_OK);
  acknowledge_in_callback = true;
  edge();
  dispatch();
  assert(callback_count == 3);
  assert(esp_drv8833_get_state(d, &s) == ESP_OK && !s.fault_latched);
  acknowledge_in_callback = false;

  assert(esp_drv8833_coast(d) == ESP_OK); // App retains stop authority.
  stopped();
  mock_on_delay = edge_during_wake;
  assert(esp_drv8833_set_speed(d, 30, 30) == ESP_OK);
  dispatch();
  assert(callback_count ==
         4); // Wake pulse signals but does not reject command.
  assert(last_event.context == ESP_DRV8833_FAULT_CONTEXT_WAKE);
  assert(last_event.transition_active);
  assert(active_pwm->duty[0] == 30 && active_pwm->duty[2] == 30);
  assert(esp_drv8833_clear_fault(d) == ESP_OK);
  fail_once = true;
  assert(esp_drv8833_set_speed(d, 40, 40) == ESP_FAIL);
  stopped();
  assert(esp_drv8833_get_state(d, &s) == ESP_OK && !s.fault_latched);
  assert(esp_drv8833_set_speed(d, 30, 30) ==
         ESP_OK); // Peripheral retry allowed.
  assert(esp_drv8833_delete(d) == ESP_OK);

  c = board();
  mock_levels[7] =
      0; // Existing low at create generates one signal, zero edges.
  assert(esp_drv8833_create(&c, &d) == ESP_OK);
  assert(esp_drv8833_set_speed(d, 20, 20) == ESP_OK);
  dispatch();
  assert(callback_count == 5 && last_event.falling_edges == 0);
  assert(last_event.context == ESP_DRV8833_FAULT_CONTEXT_INITIALIZATION);
  assert(last_event.transition_active);
  assert(esp_drv8833_clear_fault(d) == ESP_OK);
  assert(esp_drv8833_delete(d) == ESP_OK);

  c = board();
  c.on_fault = NULL; // Polling without callback has the same latch contract.
  assert(esp_drv8833_create(&c, &d) == ESP_OK);
  edge();
  dispatch();
  assert(callback_count == 5);
  assert(esp_drv8833_get_state(d, &s) == ESP_OK && s.fault_latched);
  assert(!s.fault_event_pending);
  assert(esp_drv8833_clear_fault(d) == ESP_OK);
  assert(esp_drv8833_delete(d) == ESP_OK);

  c = board();
  c.left_bridge = ESP_DRV8833_BRIDGE_B;
  c.invert_left = true;
  c.fault_gpio = -1;
  c.on_fault = NULL;
  c.sleep_gpio = -1;
  assert(esp_drv8833_create(&c, &d) == ESP_OK);
  assert(esp_drv8833_set_speed(d, 20, 40) == ESP_OK);
  assert(active_pwm->duty[0] == 20 && active_pwm->duty[3] == 40);
  assert(esp_drv8833_get_state(d, &s) == ESP_OK && !s.sleeping);
  assert(esp_drv8833_delete(d) == ESP_OK);
  c.in2_gpio = c.in1_gpio;
  assert(esp_drv8833_create(&c, &d) == ESP_ERR_INVALID_ARG && d == NULL);
  c = board();
  c.on_fault = NULL;
  c.fault_gpio = -1;
  assert(esp_drv8833_create(&c, &d) == ESP_OK);
  assert(esp_drv8833_brake(NULL) == ESP_ERR_INVALID_ARG);
  assert(esp_drv8833_coast_awake(NULL) == ESP_ERR_INVALID_ARG);
  assert(esp_drv8833_brake(d) == ESP_OK);
  for (unsigned i = 0; i < 4; i++)
    assert(active_pwm->duty[i] == 100.0f);
  assert(esp_drv8833_get_state(d, &s) == ESP_OK && s.braking && !s.sleeping);
  assert(s.left_percent == 0.0f && s.right_percent == 0.0f);
  assert(esp_drv8833_set_speed(d, -20, 30) == ESP_OK);
  assert(esp_drv8833_get_state(d, &s) == ESP_OK && !s.braking);
  assert(esp_drv8833_brake(d) == ESP_OK);
  assert(esp_drv8833_coast_awake(d) == ESP_OK);
  assert(esp_drv8833_get_state(d, &s) == ESP_OK && !s.braking && !s.sleeping);
  for (unsigned i = 0; i < 4; i++)
    assert(active_pwm->duty[i] == 0.0f);
  assert(mock_levels[c.sleep_gpio] == 1);
  assert(esp_drv8833_coast(d) == ESP_OK);
  assert(esp_drv8833_get_state(d, &s) == ESP_OK && s.sleeping);
  for (int i = 0; i < 4; i++) {
    fail_high_index = i;
    assert(esp_drv8833_brake(d) == ESP_FAIL);
    stopped();
    assert(esp_drv8833_get_state(d, &s) == ESP_OK && !s.braking);
  }
  fail_once = true;
  assert(esp_drv8833_brake(d) == ESP_FAIL);
  stopped();
  assert(esp_drv8833_brake(d) == ESP_OK);
  assert(esp_drv8833_delete(d) == ESP_OK);
  c.sleep_gpio = -1;
  assert(esp_drv8833_create(&c, &d) == ESP_OK);
  assert(esp_drv8833_brake(d) == ESP_OK);
  assert(esp_drv8833_coast_awake(d) == ESP_OK);
  assert(esp_drv8833_delete(d) == ESP_OK);
  test_contexts();
  puts("control policy and ULT context tests passed");
}
