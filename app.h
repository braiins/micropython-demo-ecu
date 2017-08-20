/**
 * @file app.h
 * @author Copyright (c) 2012 Braiins Systems s.r.o. All rights reserved.
 * @brief Receiver telemetry/extension application
 */
#ifndef _APP_H_
#define _APP_H_

/* driver framework includes */
#include <freertos-drivers/timer/timer.h>
#include <freertos-drivers/input-capture/input_capture.h>

/* lib-rtos includes */
#include <lib-rtos/task.h>

/* lib-daq includes */
#include <lib-daq/dst-gauge/dst_gauge.h>
#include <lib-daq/tm-logger/tm_logger.h>

#include "rpm.h"

/* lib-ic includes */
#include <lib-ic/gps/ic_gtpa_010.h>
#include <lib-ic/thermocouple/ic_max31855.h>

/* Micropython HAL includes */
#include "micropython-freertos-hal/gpio/upy_gpio_pin.h"

struct pipe_control {
  unsigned int rpm;
  int srv_position;
};

/**
 * Application gathers all telemetry sensors.
 */
struct app {
  /** GPS module instance */
  struct ic_gtpa_010 *gps;

  /** JETI receiver UDI data source */
  //struct jeti_udi *rc_receiver;
  struct jeti_exbus *rc_receiver;

  /** Task for logging the measured results to the console */
  struct task logger_task;

  /** Task for sampling individual sensors */
  struct task sensors_task;

  /** Task for reading RC receiver channel data */
  struct task rc_receiver_task;

  /** Task for engine management and control */
  struct task engine_control_task;

  /** Task that performs the actual RPM measurement */
  struct rpm rpm;

  /** Task that performs the logging of all telemetry data */
  struct tm_logger logger;

  /** Thermocouple temperature measurement */
  struct ic_max31855 *tc[CONFIG_APP_THERMOCOUPLE_SENSOR_COUNT];

  /** Servo instance for controlling the needle valve */
  struct servo *mixture_srv;

  int full_throttle_threshold;

  /** Pipe control channel number */
  unsigned int pipe_ctl_channel_num;

  struct pipe_control pipe_table[CONFIG_APP_TUNED_PIPE_TABLE_SIZE];
  int pipe_length_idx;

  unsigned int deciding_rpm, last_pipe_rpm;
  /** Temperature values measured from all thermocouples */
  volatile struct {
    struct {
      int32_t temp_fixed;
      int32_t junction_temp_fixed;
      float temp_float;
      float junction_temp_float;
      int temp_status;
    } tc[CONFIG_APP_THERMOCOUPLE_SENSOR_COUNT];

    /** Temperature from the NTC 10k thermistor */
    int32_t ntc_thermistor_temp;
    int ntc_thermistor_status;
  } telemetry_data;

};


/**
 * Grouping for each thermocouple converter
 * @memberof app
 */
struct app__platform_settings_tc_pins {
  /** Chip Select */
  const upy_gpio_pin__obj_t *cs;
  /** Clock */
  const upy_gpio_pin__obj_t *clk;
  /** Data in pin configuration (RX) */
  const upy_gpio_pin__obj_t *data_in;
};


/**
 * Initializes the application
 *
 * @memberof app
 * @param *self - this application instance
 *
 * @return E_OK if initialization has suceeded
 */
int app__init(struct app *self);


/**
 * Creates an input capture channel used for RPM measurement
 *
 * @memberof app
 * @private
 * @param *self - this application instance
 * @param *hw_timer - pointer to the hardware timer that will be used
 * by the input capture channel as its time base
 *
 * @return new input capture channel instance
 */
struct input_capture* app__new_input_capture(struct app *self,
                                             struct timer *hw_timer);


/**
 * Main task responsible for logging the measurement results.
 *
 * The reporting period is configurable via
 * CONFIG_APP_LOGGER_TASK_PERIOD_MS.
 *
 * @memberof app
 * @param *self - this application instance
 */
void app__logger_task(struct app *self);


/**
 * Main task responsible for sensors sampling.
 *
 *
 * @memberof app
 * @param *self - this application instance
 */

void app__sensors_task(struct app *self);


/**
 * RC receiver task responsible for reading RC channel data.
 *
 * @memberof app
 * @param *self - this application instance
 */
void app__rc_receiver_task(struct app *self);

/**
 * RC receiver task responsible for reading RC channel data.
 *
 * @memberof app
 * @param *self - this application instance
 */
void app__rc_receiver_task_exbus(struct app *self);



/**
 * Engine control task
 *
 * @memberof app
 * @param *self - this application instance
 */
void app__engine_control_task(struct app *self);


/** Platform specific timer configuration */
extern const struct platform_timer_config input_capture_timer_cfg;
/** Platform specific input capture configuration */
extern const struct platform_input_capture_config rpm_input_capture_cfg;
/** Platform specific servo pwm configuration */
extern const struct platform_pwm_config srv_pwm_cfg;


#endif /* _APP_H_ */
