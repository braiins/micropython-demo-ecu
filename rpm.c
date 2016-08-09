/**
 * @file rpm.c
 * @brief rpm handler implementation
 *
 */

#include <rpm.h>

/* driver framework includes */
#include <freertos-drivers/gpio/gpio_port.h>

/* lib-rtos includes */
#include <lib-rtos/time.h>
#include <lib-rtos/assert.h>

#include "config.app.h"

/** Platform specific timer configuration */
extern const struct platform_timer_config rpm_input_capture_timer_cfg;
/** Platform specific input capture configuration */
extern const struct platform_input_capture_config rpm_input_capture_cfg;

void rpm__init(struct rpm *self)
{
  int result;
  /** Underlying hardware timer required by rpm_ic */
  struct timer *hw_timer;
  /** Input capture channel needed for RPM measurement */
  struct input_capture *rpm_ic;

  hw_timer = timer__new(CONFIG_APP_RPM_TIMER,
                        CONFIG_APP_RPM_TIMER_PERIOD,
                        &rpm_input_capture_timer_cfg);
  assert_ptr(hw_timer);

  /* Input capture will be able to measure signals with periods up to
   * CONFIG_APP_RPM_MAX_PERIOD_LIMIT. While averaging, the timeout is not
   * relevant as the input capture channel is being only sampled and
   * never waited on. */
  rpm_ic =
    input_capture__new(CONFIG_APP_RPM_INPUT_CAPTURE,
                       INPUT_CAPTURE__LAST_EVENT_SAMPLE_MODE,
                       hw_timer, &rpm_input_capture_cfg,
                       TIME__OS_TICK_INFINITE_TIMEOUT);
  assert_ptr(rpm_ic);

  self->rpm_gauge = dst_gauge__new(rpm_ic);
  assert_ptr(self->rpm_gauge);

  result = task__init(&self->task, (task__entry_method_t) rpm__task,
                      self, "rpm",
                      CONFIG_APP_RPM_TASK_STACK_SIZE,
                      CONFIG_APP_RPM_TASK_PRIORITY);
  assert(result == E_OK);

  /* Initialize the LED pin RPM indicator */
  gpio_port__enable(CONFIG_APP_RPM_LED_PORT);
  gpio_port__set_pin_output(CONFIG_APP_RPM_LED_PORT,
                            CONFIG_APP_RPM_LED_PIN);

}


void rpm__task(struct rpm *self)
{
  time__os_tick_t now;

  now = time__get_os_tick_count();
  while (1) {
    dst_gauge__sample_signal_period(self->rpm_gauge,
                                    TIME__MS_TO_TICKS
                                    (CONFIG_APP_RPM_MAX_PERIOD_LIMIT_MS,
                                     CONFIG_APP_TIMER_BASE_FREQ));
    now =
      task__sleep_until(now,
                        TIME__MS_TO_OS_TICKS(
                          CONFIG_APP_RPM_SAMPLING_PERIOD_MS));

    /* Indicate measurement completion by toggling the LED */
    gpio_port__wr_pin(CONFIG_APP_RPM_LED_PORT, CONFIG_APP_RPM_LED_PIN,
                      !gpio_port__rd_pin(CONFIG_APP_RPM_LED_PORT,
                                         CONFIG_APP_RPM_LED_PIN));
  }
}
