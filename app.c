/**
 * @file app.c
 * @brief Demo ECU application
 *
 *
 *
 */
/* C-library includes */
#include <stdio.h>

#include <app.h>

/* lib-rtos includes */
#include <lib-rtos/time.h>
#include <lib-rtos/assert.h>
#include <lib-rtos/util.h>



/* Driver includes */
#include <freertos-drivers/gpio/gpio_pin.h>
#include <freertos-drivers/uart/uart.h>
//#include <freertos-drivers/adc/adc.h>

#include "micropython-freertos-hal/gpio/upy_gpio_pin.h"

/* lib-ic includes */
#include <lib-ic/thermocouple/ic_max_tcdc_backend--gpio.h>

/* lib-rc includes */
#include <lib-rc/jeti/jeti_udi.h>
#include <lib-rc/jeti/jeti_exbus.h>
#include <lib-rc/jeti/jeti_ex.h>
#include <lib-rc/servo/servo.h>

/* Micropython includes */
#include "py/runtime.h"

/* lib-analog-sensors includes */
//#include <lib-analog-sensors/ntc-thermistor/ntc_thermistor.h>




int app__init(struct app *self)
{
  int result;
  int i;

  rpm__init(&self->rpm);

  result = task__init(&self->sensors_task,
                      (task__entry_method_t)app__sensors_task,
                      self, "sensors",
                      CONFIG_APP_SENSORS_TASK_STACK_SIZE,
                      CONFIG_APP_SENSORS_TASK_PRIORITY);
  if (result != E_OK) {
    goto task_creation_failed;
  }

  result = task__init(&self->rc_receiver_task,
                      (task__entry_method_t)app__rc_receiver_task_exbus,
                      self, "receiver RC",
                      CONFIG_APP_RC_RECEIVER_TASK_STACK_SIZE,
                      CONFIG_APP_RC_RECEIVER_TASK_PRIORITY);
#if 0
  result = task__init(&self->rc_receiver_task,
                      (task__entry_method_t)app__rc_receiver_task,
                      self, "receiver RC",
                      CONFIG_APP_RC_RECEIVER_TASK_STACK_SIZE,
                      CONFIG_APP_RC_RECEIVER_TASK_PRIORITY);
#endif
  result = task__init(&self->engine_control_task,
                      (task__entry_method_t)app__engine_control_task,
                      self, "engine ctl",
                      CONFIG_APP_RC_RECEIVER_TASK_STACK_SIZE,
                      CONFIG_APP_RC_RECEIVER_TASK_PRIORITY);
  /* Pipe control channel needs to be setup explicitely */
  self->pipe_ctl_channel_num = -1;
  for (i = 0; i < CONFIG_APP_TUNED_PIPE_TABLE_SIZE; i++) {
    self->pipe_table[i].rpm = 0;
    self->pipe_table[i].srv_position = 0;
  }

  task_creation_failed:
  return result;
}

extern const struct platform_gpio_config gps_enable_pin_cfg;
extern const upy_gpio_pin__obj_t upy_gpio_pin__tc_clk;
extern const upy_gpio_pin__obj_t upy_gpio_pin__tc_cs1;
extern const upy_gpio_pin__obj_t upy_gpio_pin__tc_cs2;
extern const upy_gpio_pin__obj_t upy_gpio_pin__tc_data_in;

/*
extern const struct platform_adc_config temp1_sensor_adc_cfg;
extern const struct ntc_thermistor__coeff ntc_thermistor_10k_coeff;
extern const struct ntc_thermistor__config temp1_sensor_config;
*/
static const struct jeti_ex_sensor__descr jeti_ex_descriptors[] = {
  JETI_EX_SENSOR__DESCR_INIT("ECU test", "", 0, 0),
  JETI_EX_SENSOR__DESCR_INIT("RPM", "\xb0\x43", JETI_EX__TYPE_22b, 0),
  JETI_EX_SENSOR__DESCR_INIT("Speed", "km/h", JETI_EX__TYPE_14b, 0),
  JETI_EX_SENSOR__DESCR_INIT("ExTemp.", "\xb0\x43", JETI_EX__TYPE_14b, 1),
  JETI_EX_SENSOR__DESCR_INIT("Pipe", "%", JETI_EX__TYPE_14b, 0),
  JETI_EX_SENSOR__DESCR_INIT("Mixture", "%", JETI_EX__TYPE_14b, 0),
};
enum {
  RPM = 1,
  SPEED,
  EX_TEMP,
  PIPE,
  //MIXTURE,
  APP__JETI_EX_SENSOR_COUNT
};
static struct jeti_ex_sensor jeti_ex_sensors[] = {
  {
    .value = 0,
    .descr = &jeti_ex_descriptors[0],
  },
  {
    .value = 0,
    .descr = &jeti_ex_descriptors[1],
  },
  {
    .value = 0,
    .descr = &jeti_ex_descriptors[2],
  },
  {
    .value = 0,
    .descr = &jeti_ex_descriptors[3],
  },
  {
    .value = 0,
    .descr = &jeti_ex_descriptors[4],
  },
  {
    .value = 0,
    .descr = &jeti_ex_descriptors[5],
  }
};
static uint8_t ex_msg[JETI_EX__MAX_MESSAGE_SIZE];


void app__sensors_task(struct app *self)
{
  const struct gpio_pin *gps_enable_pin;
  /* Backend GPIO driver for MAXIM thermocouple-digital-converters */
  struct ic_max_tcdc_backend *tc_backend;
  struct jeti_ex *jeti_ex;
  time__os_tick_t now, last_slow_sensors_sample_time, last_telem_labels_tx_time;
  /* NTC connected ADC */
#if 0
  struct adc *temp1_sensor_adc;
  struct ntc_thermistor *temp1_sensor;
#endif
  gps_enable_pin = gpio_pin__new(&gps_enable_pin_cfg);
  assert_ptr(gps_enable_pin);

  self->gps = ic_gtpa_010__new(CONFIG_APP_GPS_UART_PORT,
                               CONFIG_APP_GPS_UART_BAUD,
                               gps_enable_pin);
  assert_ptr(self->gps);

  /* Initialize the GPIO backend for thermocouple converter */
  tc_backend = ic_max_tcdc_backend__new(&upy_gpio_pin__tc_cs1.pin->config,
                                        &upy_gpio_pin__tc_clk.pin->config,
                                        &upy_gpio_pin__tc_data_in.pin->config);
  assert_ptr(tc_backend);
  self->tc = ic_max31855__new(tc_backend);
  assert_ptr(self->tc);

  jeti_ex = jeti_ex__new(jeti_ex_sensors, APP__JETI_EX_SENSOR_COUNT, 0xa400,
                         0x0001);
  assert_ptr(jeti_ex);
#if 0
  temp1_sensor_adc =
    adc__new(CONFIG_APP_TEMPERATURE_ADC, TIME__OS_TICK_INFINITE_TIMEOUT,
             &temp1_sensor_adc_cfg);
  assert_ptr(temp1_sensor_adc);
  temp1_sensor = ntc_thermistor__new(temp1_sensor_adc,
  				     &ntc_thermistor_10k_coeff,
				     &temp1_sensor_config);
#endif

  ic_gtpa_010__disable(self->gps);
  ic_gtpa_010__enable(self->gps);

  now = time__get_os_tick_count();
  last_slow_sensors_sample_time =
    now - TIME__MS_TO_OS_TICKS(CONFIG_APP_SENSORS_SLOW_SAMPLE_PERIOD_MS);
  last_telem_labels_tx_time =
    now - TIME__MS_TO_OS_TICKS(500);

  while (1) {
    int retval;
    size_t ex_msg_len;
    ic_gtpa_010__read(self->gps);
    now = time__get_os_tick_count();
    /*ic_max31855__read_temp_float(self->tc, &self->tc_temp_float);*/
    if (time__is_after_eq(now, last_slow_sensors_sample_time +
			  TIME__MS_TO_OS_TICKS(CONFIG_APP_SENSORS_SLOW_SAMPLE_PERIOD_MS))) {
      last_slow_sensors_sample_time = now;
#if 0
      self->telemetry_data.tc_temp_status =
        ic_max31855__read_temp_fixed(self->tc,
                                     (int32_t*)&self->telemetry_data
                                       .tc_temp_fixed,
                                     (int32_t*)&self->telemetry_data
                                       .tc_junction_temp_fixed);
#endif
      self->telemetry_data.tc_temp_status =
        ic_max31855__read_temp_float(self->tc,
                                     (float*)&self->telemetry_data
                                       .tc_temp_float,
                                     (float*)&self->telemetry_data
                                       .tc_junction_temp_float);
#if 0
      self->telemetry_data.ntc_thermistor_status =
        ntc_thermistor__read(temp1_sensor, &temp);
      self->telemetry_data.ntc_thermistor_temp = (int32_t)(temp * 100);
#endif
      /* Send labels every 500 ms */
#if 1
      if (time__is_after_eq(now, last_telem_labels_tx_time +
			  TIME__MS_TO_OS_TICKS(500))) {
        last_telem_labels_tx_time = now;
        retval = jeti_ex__create_text_message(jeti_ex, ex_msg,
                                              JETI_EX__MAX_MESSAGE_SIZE,
                                              &ex_msg_len);
      } else {
#endif
        retval = jeti_ex__create_data_message(jeti_ex, ex_msg,
                                              JETI_EX__MAX_MESSAGE_SIZE,
                                              &ex_msg_len);
      }
      assert(ex_msg_len <= JETI_EX__MAX_MESSAGE_SIZE);
      if (retval == E_OK) {
        jeti_exbus__submit_ex_msg(self->rc_receiver, ex_msg);
      }
    }
  }
}

#if 0
/**
 * This task takes care of reading data from the receiver.
 *
 * @param *self - this application
 */
void app__rc_receiver_task(struct app *self)
{
  struct jeti_udi *receiver;

  receiver = jeti_udi__new(CONFIG_APP_RC_RECEIVER_UART_PORT, JETI_UDI__12);

  assert_ptr(receiver);

  self->rc_receiver = receiver;

  while (1) {
    jeti_udi__rx_task(receiver);
  }
}
#endif

/**
 * This task takes care of reading data from the receiver.
 *
 * @param *self - this application
 */
void app__rc_receiver_task_exbus(struct app *self)
{
  struct jeti_exbus *exbus;

  exbus = jeti_exbus__new(CONFIG_APP_RC_RECEIVER_UART_PORT);

  assert_ptr(exbus);

  self->rc_receiver = exbus;

  while (1) {
    jeti_exbus__rx_task(exbus);
  }
}


void app__calculate_shorter_pipe(struct app *self, unsigned int rpm)
{
  int i;
  // Iterate thru first N-1 elements, take the last by default.
  for (i = 0; i < (CONFIG_APP_TUNED_PIPE_TABLE_SIZE - 1); i++) {
    if (rpm < self->pipe_table[i].rpm) {
      break;
    }
  }
  self->pipe_length_idx = i;
}


void app__calculate_longer_pipe(struct app *self, unsigned int rpm)
{
  int i;
  // Iterate from second element since we take (i-1)-th element.
  for (i = 1; i < CONFIG_APP_TUNED_PIPE_TABLE_SIZE; i++) {
    if (rpm <= self->pipe_table[i].rpm) {
      break;
    }
  }
  // Take the previous element, we want to go down here.
  self->pipe_length_idx = i - 1;
}


/**
 * This task takes care of engine management.
 *
 * @param *args - no argument is passed to the task
 */

void app__engine_control_task(struct app *self)
{
  struct servo *pipe_srv;
  /* allows to waking up the task at the correct frequency. */
  time__os_tick_t now;
  unsigned int last_pipe_rpm = 0;
  int last_pipe_length_idx = 0;

  self->pipe_length_idx = 0;

  pipe_srv = servo__new(3, &srv_pwm_cfg, CONFIG_APP_SRV_PWM_CLOCK,
			-1600, 1600);

  assert_ptr(pipe_srv);

  now = time__get_os_tick_count();
  while (1) {
    now = task__sleep_until(now,
			    TIME__MS_TO_OS_TICKS(10));
    unsigned int rpm = rpm__get(&self->rpm);

    if (self->pipe_ctl_channel_num != -1) {
      bool auto_control = false;
      int auto_control_value = rc_channels__get(&self->rc_receiver->last_servo_positions, 6);
      int throttle_value = rc_channels__get(&self->rc_receiver->last_servo_positions, 2);
      if ((auto_control_value > 500) && (auto_control_value < 2048)) {
        auto_control = true;
      } else {
        servo__set_position(pipe_srv,
                            rc_channels__get(&self->rc_receiver->last_servo_positions,
                                             self->pipe_ctl_channel_num));
      }

      last_pipe_length_idx = self->pipe_length_idx;

      /* throttling up - hacked handling of threshold for negative throttle setups.. */
      if (((self->full_throttle_threshold < 0) &&
           (throttle_value < self->full_throttle_threshold)) ||
          ((self->full_throttle_threshold > 0) &&
            (throttle_value > self->full_throttle_threshold))) {
        app__calculate_shorter_pipe(self, rpm);
      } else {
        app__calculate_longer_pipe(self, rpm);
      }
      if (last_pipe_length_idx != self->pipe_length_idx) {
        self->last_pipe_rpm = last_pipe_rpm;
        self->deciding_rpm = rpm;
        last_pipe_rpm = rpm;
        if (auto_control) {
          servo__set_position(pipe_srv,
                              self->pipe_table[self->pipe_length_idx].srv_position);
        }
      }

    }
  }
}


/**
 * Application python object wrapper
 */
typedef struct {
  mp_obj_base_t base;
  /** Indication that the application has already been initialized */
  bool initialized;

  struct app app;
} app__obj_t;


/**
 * \method rpm()
 *
 * Return value: Current RPM value
 */
STATIC mp_obj_t app__get_rpm(mp_obj_t self_in)
{
  app__obj_t *self = MP_OBJ_TO_PTR(self_in);

  return mp_obj_new_int(rpm__get(&self->app.rpm));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(app__get_rpm_obj, app__get_rpm);


/**
 * \method tc_temp1()
 *
 * Return value: Current RPM value
 */
STATIC mp_obj_t app__get_tc_temp(mp_obj_t self_in)
{
  app__obj_t *self = MP_OBJ_TO_PTR(self_in);
  mp_obj_t tuple[] = {
    mp_obj_new_int(self->app.telemetry_data.tc_temp_status),
    mp_obj_new_float(self->app.telemetry_data.tc_temp_float),
    mp_obj_new_float(self->app.telemetry_data.tc_junction_temp_float),
    mp_obj_new_int(self->app.telemetry_data.tc_temp_fixed),
    mp_obj_new_int(self->app.telemetry_data.tc_junction_temp_fixed),
  };

  return mp_obj_new_tuple(sizeof(tuple)/sizeof(mp_obj_t), tuple);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(app__get_tc_temp_obj, app__get_tc_temp);


/**
 * \method gps()
 *
 * Return value: GPS telemetry data
 */
STATIC mp_obj_t app__get_gps(mp_obj_t self_in)
{
  app__obj_t *self = MP_OBJ_TO_PTR(self_in);
  mp_obj_t tuple[] = {
    mp_obj_new_int(self->app.gps->info.fix),
    mp_obj_new_int(self->app.gps->info.sig),
    mp_obj_new_float(self->app.gps->info.speed),
    mp_obj_new_float(self->app.gps->info.lat),
    mp_obj_new_float(self->app.gps->info.lon),
    mp_obj_new_float(self->app.gps->info.elv),
    mp_obj_new_float(self->app.gps->info.direction),
    mp_obj_new_float(self->app.gps->info.PDOP),
    mp_obj_new_float(self->app.gps->info.HDOP),
    mp_obj_new_float(self->app.gps->info.VDOP),
  };

  return mp_obj_new_tuple(sizeof(tuple)/sizeof(mp_obj_t), tuple);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(app__get_gps_obj, app__get_gps);


/**
 * \method gps_datetime()
 *
 * Return value: GPS telemetry data
 */
STATIC mp_obj_t app__get_gps_datetime(mp_obj_t self_in)
{
  app__obj_t *self = MP_OBJ_TO_PTR(self_in);
  mp_obj_t tuple[] = {
    mp_obj_new_int(self->app.gps->info.utc.year),
    mp_obj_new_int(self->app.gps->info.utc.mon),
    mp_obj_new_int(self->app.gps->info.utc.day),
    mp_obj_new_int(self->app.gps->info.utc.hour),
    mp_obj_new_int(self->app.gps->info.utc.min),
    mp_obj_new_int(self->app.gps->info.utc.sec),
    /* Convert milliseconds to microseconds */
    mp_obj_new_int(self->app.gps->info.utc.hsec * 1000),
  };

  return mp_obj_new_tuple(sizeof(tuple)/sizeof(mp_obj_t), tuple);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(app__get_gps_datetime_obj, app__get_gps_datetime);


/**
 * \method set_full_throttle_threshold()
 *
 * Return value: Current RPM value
 */
STATIC mp_obj_t app__set_full_throttle_threshold(mp_obj_t self_in, mp_obj_t full_throttle_threshold)
{
  app__obj_t *self = MP_OBJ_TO_PTR(self_in);

  self->app.full_throttle_threshold = mp_obj_get_int(full_throttle_threshold);

  return MP_OBJ_NEW_SMALL_INT(self->app.full_throttle_threshold);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(app__set_full_throttle_threshold_obj, app__set_full_throttle_threshold);


/**
 * \method set_pipe_ctl_channel()
 *
 * Return value: Current RPM value
 */
STATIC mp_obj_t app__set_pipe_ctl_channel(mp_obj_t self_in, mp_obj_t pipe_channel)
{
  app__obj_t *self = MP_OBJ_TO_PTR(self_in);

  self->app.pipe_ctl_channel_num = mp_obj_get_int(pipe_channel);

  return MP_OBJ_NEW_SMALL_INT(self->app.pipe_ctl_channel_num);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(app__set_pipe_ctl_channel_obj, app__set_pipe_ctl_channel);


/**
 * \method set_pipe_table_entry()
 *
 * Return value: Current RPM value
 */
STATIC mp_obj_t app__set_pipe_table_entry(mp_obj_t self_in, mp_obj_t tuple)
{
  mp_uint_t len;
  mp_obj_t *elem;
  app__obj_t *self = MP_OBJ_TO_PTR(self_in);
  int idx;

  mp_obj_get_array(tuple, &len, &elem);

  if (len != 3) {
    nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_TypeError, "exactly  3 elements for each pipe table entry expected (%d given)", len));
  }
  idx = mp_obj_get_int(elem[0]);
  if ((idx < 0) || (idx >= CONFIG_APP_TUNED_PIPE_TABLE_SIZE)) {
      nlr_raise(
		mp_obj_new_exception_msg_varg(&mp_type_ValueError,
					      "Index %d for pipe table out of range, allowed 0-%d",
					      idx,
					      CONFIG_APP_TUNED_PIPE_TABLE_SIZE));
  }
  self->app.pipe_table[idx].rpm = mp_obj_get_int(elem[1]);
  self->app.pipe_table[idx].srv_position = mp_obj_get_int(elem[2]);

  return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(app__set_pipe_table_entry_obj, app__set_pipe_table_entry);


STATIC mp_obj_t app__set_jeti_ex_sensors(mp_obj_t self_in, mp_obj_t tuple)
{
  mp_uint_t i, len;
  mp_obj_t *elem;

  mp_obj_get_array(tuple, &len, &elem);

  if (len != (APP__JETI_EX_SENSOR_COUNT - 1)) {
    nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_TypeError, "exactly  %d "
      "sensor values expected (%d given)",
                                            APP__JETI_EX_SENSOR_COUNT - 1, len));
  }
  for(i = 0; i < len; i++) {
    jeti_ex_sensors[i + 1].value = mp_obj_get_int(elem[i]);
  }
#if 0
  jeti_ex_sensors[RPM] = rpm__get(&self->rpm);
  jeti_ex_sensors[SPEED] = self->gps->info.speed;
  jeti_ex_sensors[EX_TEMP] = (int32_t)self->telemetry_data.tc_temp_float);
  jeti_ex_sensors[PIPE] = self->pipe_length_idx;
  jeti_ex_sensors[MIXTURE] = self->;
#endif

  return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(app__set_jeti_ex_sensors_obj, app__set_jeti_ex_sensors);

/**
 * \method pipe_length_idx()
 *
 * Return value: Current RPM value
 */
STATIC mp_obj_t app__get_pipe_length_telem_data(mp_obj_t self_in)
{
  app__obj_t *self = MP_OBJ_TO_PTR(self_in);

  mp_obj_t tuple[3] = {
    mp_obj_new_int(self->app.pipe_length_idx),
    mp_obj_new_int(self->app.last_pipe_rpm),
    mp_obj_new_int(self->app.deciding_rpm),
  };

  return mp_obj_new_tuple(sizeof(tuple)/sizeof(mp_obj_t), tuple);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(app__get_pipe_length_telem_data_obj, app__get_pipe_length_telem_data);


/**
 * \method rc_channels()
 *
 * Return value: Current RC channels
 */
STATIC mp_obj_t app__get_rc_channels(mp_obj_t self_in)
{
  struct rc_channels channels;
  int i;

  app__obj_t *self = MP_OBJ_TO_PTR(self_in);
  mp_obj_t tuple[12];

  //jeti_udi__read(self->app.rc_receiver, &channels);
  jeti_exbus__read(self->app.rc_receiver, &channels);
  for (i = 0; i < 12; i++) {
    tuple[i] = mp_obj_new_int(channels.channels[i]);
  };

  return mp_obj_new_tuple(sizeof(tuple)/sizeof(mp_obj_t), tuple);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(app__get_rc_channels_obj,
                                 app__get_rc_channels);

STATIC const mp_rom_map_elem_t app__locals_dict_table[] = {
  { MP_ROM_QSTR(MP_QSTR_rpm), MP_ROM_PTR(&app__get_rpm_obj) },
  { MP_ROM_QSTR(MP_QSTR_full_throttle_threshold), MP_ROM_PTR(&app__set_full_throttle_threshold_obj) },
  { MP_ROM_QSTR(MP_QSTR_pipe_ctl_channel), MP_ROM_PTR(&app__set_pipe_ctl_channel_obj) },
  { MP_ROM_QSTR(MP_QSTR_pipe_table_entry), MP_ROM_PTR(&app__set_pipe_table_entry_obj) },
  { MP_ROM_QSTR(MP_QSTR_jeti_ex_sensors), MP_ROM_PTR(&app__set_jeti_ex_sensors_obj) },
  { MP_ROM_QSTR(MP_QSTR_pipe_length_telem_data), MP_ROM_PTR(&app__get_pipe_length_telem_data_obj) },
  { MP_ROM_QSTR(MP_QSTR_tc_temp), MP_ROM_PTR(&app__get_tc_temp_obj) },
  { MP_ROM_QSTR(MP_QSTR_gps), MP_ROM_PTR(&app__get_gps_obj) },
  { MP_ROM_QSTR(MP_QSTR_gps_datetime), MP_ROM_PTR(&app__get_gps_datetime_obj) },
  { MP_ROM_QSTR(MP_QSTR_rc_channels), MP_ROM_PTR(&app__get_rc_channels_obj) },
};

STATIC MP_DEFINE_CONST_DICT(app__locals_dict, app__locals_dict_table);

STATIC mp_obj_t app__make_new(const mp_obj_type_t *type, size_t n_args,
                              size_t n_kw, const mp_obj_t *args);

STATIC const mp_obj_type_t app__type = {
  { &mp_type_type },
  .name = MP_QSTR_app,
  /*.print = app__print,*/
  .make_new = app__make_new,
  .locals_dict = (mp_obj_t)&app__locals_dict,
};

/* TODO: replace singleton instance with a pointer and detect initialization
 * by non-NULL value etc. However, consider what will the garbage collector do
 * to the application if reference to it is lost! */
STATIC app__obj_t app__instance = {
  .base = {
    .type = &app__type,
  },
  .initialized = false,
};


/**
 * @memberof app
 * @private
 */
STATIC mp_obj_t app__make_new(const mp_obj_type_t *type, size_t n_args,
                              size_t n_kw, const mp_obj_t *args)
{
  int status;
  mp_arg_check_num(n_args, n_kw, 0, 0, false);

  /* Attempt to initialize the application only once */
  if (!app__instance.initialized) {
    status = app__init(&app__instance.app);
    if (status != E_OK) {
      nlr_raise(
		mp_obj_new_exception_msg_varg(&mp_type_ValueError,
					      "Application initialization failed (%d)",
					      status));
    }
    app__instance.initialized = true;
  }

  return MP_OBJ_FROM_PTR(&app__instance);
}


STATIC const mp_rom_map_elem_t app_module_globals_table[] = {
  { MP_ROM_QSTR(MP_QSTR_get), MP_ROM_PTR(&app__type) },
};


STATIC MP_DEFINE_CONST_DICT(app_module_globals, app_module_globals_table);

const mp_obj_module_t app_module = {
    .base = { &mp_type_module },
    .name = MP_QSTR_app,
    .globals = (mp_obj_dict_t*)&app_module_globals,
};
