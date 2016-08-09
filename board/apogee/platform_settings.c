/**
 * @file platform_settings.c
 * @author Copyright (c) 2016 Braiins Systems s.r.o. All rights reserved.
 * @brief Platform specific settings - GPIO configuration, etc.
 */
#include <freertos-drivers/uart/uart.h>
#include <freertos-drivers/gpio/gpio_pin.h>
#include <freertos-drivers/sd/sd.h>
#include <freertos-drivers/input-capture/platform_input_capture.h>
#include <freertos-drivers/timer/platform_timer_config.h>
#include <freertos-drivers/pwm/platform_pwm.h>
#include <freertos-drivers/pwm/platform_pwm_config.h>

#include "py/obj.h"

#include "micropython-freertos-hal/gpio/upy_gpio_pin.h"
#include "micropython-freertos-hal/sd/upy_sd.h"

#include "config.app.h"


/** REPL */
/* REPL console, USART1 console, RX: port A, pin 10; TX: port B, pin 6 */
PLATFORM_UART_CONFIG__INIT_GPIO_CFG(1, A, 10, 7, B, 6, 7);

/* Command stream from RC receiver
 * USART2, RX: port A, pin 3; TX: port A, pin 2 */
PLATFORM_UART_CONFIG__INIT_GPIO_CFG(2, A, 3, 7, A, 2, 7);
/* RX2 polarity PB13 */
UPY_GPIO_PIN__PLATFORM_ROM_DEFINE(rx_polarity, B, 13, OUTPUT_PP, NOPULL, FAST);

/* Alternative full duplex connection to the RC receiver,
 * USART6, RX: port C, pin 7; TX: port B, pin 6 */
PLATFORM_UART_CONFIG__INIT_HALF_DUPLEX_GPIO_CFG(6, C, 7, C, 6, 8);


/** SD slot */
/* SDIO subsystem configuration for 4-bit bus width (SD card standard) */
UPY_SD_CONFIG__PLATFORM_ROM_DEFINE(sd_config, 4);

/** GPS */
/* GPS on UART4, RX: port A, pin 1; TX: port A, pin 0 */
PLATFORM_UART_CONFIG__INIT_GPIO_CFG(4, A, 1, 8, A, 0, 8);
/* GPS enable pin */
const struct platform_gpio_config gps_enable_pin_cfg =
  PLATFORM_GPIO_CONFIG__INIT(A, GPIO_PIN_0, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,
                             GPIO_SPEED_FREQ_HIGH, 0);

/** Thermocouple/temperature */
#if 0
/* MAX6675 SCK pin */
const struct platform_gpio_config tc_clock_pin_cfg =
  PLATFORM_GPIO_CONFIG__INIT_GPIO(F, 2, _8MA, STD_WPU, OUT);
/* MAX6675 CS pin */
const struct platform_gpio_config tc_cs_pin_cfg =
  PLATFORM_GPIO_CONFIG__INIT_GPIO(A, 0, GPIO_PIN_0, GPIO_MODE_OUTPUT_PP,
                                  GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, 0);
/* MAX6675 SO pin */
const struct platform_gpio_config tc_data_pin_cfg =
  PLATFORM_GPIO_CONFIG__INIT_GPIO(A, 0, GPIO_PIN_0, GPIO_MODE_INPUT,
                                  GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, 0);
#endif


/* Configuration for PWM channels for servos */
const struct platform_pwm_config srv_pwm_cfg = {
  .base_init = {
    .Prescaler = CONFIG_APP_SRV_PWM_PRESCALER - 1,
    .CounterMode = TIM_COUNTERMODE_UP,
    .Period = 0,
    .ClockDivision = TIM_CLOCKDIVISION_DIV1,
    .RepetitionCounter = 0,
  },
  .oc_init = {
    .Pulse = 0,
    .OCMode = TIM_OCMODE_PWM1,
    .OCPolarity = TIM_OCPOLARITY_HIGH,
    .OCNPolarity = TIM_OCNPOLARITY_HIGH,
    .OCIdleState = TIM_OCIDLESTATE_SET,
    .OCNIdleState= TIM_OCNIDLESTATE_RESET,
    .OCFastMode = TIM_OCFAST_DISABLE,
  }
};


/* SRV3 output index 3, TIM3, port B, pin 4, alternate function 2 */
PLATFORM_PWM_CONFIG__INIT_GPIO_CFG(3, 3, B, 4, 2);
/* TIM3, channel 1 */
PLATFORM_PWM__INIT(3, 3, 1);


/** LED's */
/* LED 2 (orange) - PC13, LED3 (green) - PC1, LED 4 (yellow) - PC3, LED1 (red) PDC0 */
/* Signalling LED used in the application */
UPY_GPIO_PIN__PLATFORM_ROM_DEFINE(led1, C, 0, OUTPUT_PP, NOPULL, FAST);
UPY_GPIO_PIN__PLATFORM_ROM_DEFINE(led2, C, 13, OUTPUT_PP, NOPULL, FAST);
UPY_GPIO_PIN__PLATFORM_ROM_DEFINE(led3, C, 1, OUTPUT_PP, NOPULL, FAST);

/** SPI1 connector - Thermocouple(s) */
/* CS1 PB9 */
UPY_GPIO_PIN__PLATFORM_ROM_DEFINE(tc_cs1, B, 9, OUTPUT_PP, NOPULL, FAST);
/* MOSI1 PA7 - used as CS2 */
UPY_GPIO_PIN__PLATFORM_ROM_DEFINE(tc_cs2, A, 7, OUTPUT_PP, NOPULL, FAST);
/* SCK1 PB9 */
UPY_GPIO_PIN__PLATFORM_ROM_DEFINE(tc_clk, A, 5, OUTPUT_OD, PULLUP, FAST);
/* MISO1 PA6*/
UPY_GPIO_PIN__PLATFORM_ROM_DEFINE(tc_data_in, A, 6, INPUT, NOPULL, FAST);


STATIC const mp_rom_map_elem_t bsp_module_globals_table[] = {
{ MP_ROM_QSTR(MP_QSTR_led1), UPY_GPIO_PIN__ROM_PTR(led1) },
{ MP_ROM_QSTR(MP_QSTR_led2), UPY_GPIO_PIN__ROM_PTR(led2) },
{ MP_ROM_QSTR(MP_QSTR_led3), UPY_GPIO_PIN__ROM_PTR(led3) },
{ MP_ROM_QSTR(MP_QSTR_sd_config), UPY_SD_CONFIG__ROM_PTR(sd_config) },
{ MP_ROM_QSTR(MP_QSTR_tc_cs1), UPY_GPIO_PIN__ROM_PTR(tc_cs1) },
{ MP_ROM_QSTR(MP_QSTR_tc_cs2), UPY_GPIO_PIN__ROM_PTR(tc_cs2) },
{ MP_ROM_QSTR(MP_QSTR_tc_clk), UPY_GPIO_PIN__ROM_PTR(tc_clk) },
{ MP_ROM_QSTR(MP_QSTR_tc_data_in), UPY_GPIO_PIN__ROM_PTR(tc_data_in) },
{ MP_ROM_QSTR(MP_QSTR_rx_polarity), UPY_GPIO_PIN__ROM_PTR(rx_polarity) },
};
STATIC MP_DEFINE_CONST_DICT(bsp_module_globals, bsp_module_globals_table);


/** BSP module  */
const mp_obj_module_t bsp_module = {
    .base = { &mp_type_module },
    .name = MP_QSTR_bsp,
    .globals = (mp_obj_dict_t*)&bsp_module_globals,
};


/** RPM measurement on SRV5 connector */
/* RPM input capture uses Timer 2, channel 1 */
PLATFORM_INPUT_CAPTURE__INIT(0, 2, 1);
/* Port A, pin 15, alternate function 1 */
PLATFORM_INPUT_CAPTURE_CONFIG__INIT_GPIO_CFG(0, 2, A, 15, 1);

/* Configuration for input capture channel */
const struct platform_timer_config rpm_input_capture_timer_cfg = {
  .base_init = {
    .Prescaler = CONFIG_APP_TIMER_PRESCALER - 1,
    .CounterMode = TIM_COUNTERMODE_UP,
    .Period = 0,
    .ClockDivision = TIM_CLOCKDIVISION_DIV1,
    .RepetitionCounter = 0,
  }
};

const struct platform_input_capture_config rpm_input_capture_cfg = {
  .ic_init = {
    .ICFilter = 0,
    .ICPolarity = TIM_ICPOLARITY_RISING,
    .ICPrescaler = TIM_ICPSC_DIV1,
    .ICSelection = TIM_ICSELECTION_DIRECTTI,
  },
  /* configuration for the slave mode controller the channel 1 filtered input
   * (TI1FP1) is used for resetting the counter upon every rising edge. This
   * results in having the signal period stored in CCR register. */
  .slave_config = {
    .InputTrigger = TIM_TS_TI1FP1,
    .SlaveMode = TIM_SLAVEMODE_RESET,
    .TriggerPolarity = TIM_ICPOLARITY_RISING,
    .TriggerFilter = 0,
    .TriggerPrescaler = TIM_TRIGGERPRESCALER_DIV1,
  },
};

