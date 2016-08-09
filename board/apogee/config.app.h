#ifndef FREERTOS_INTEGRATION_TEST_CONFIG_APP_H
#define FREERTOS_INTEGRATION_TEST_CONFIG_APP_H

/* allows measuring up to period of ~51s (0.02 Hz) signal with PCLK1 @ 84
 * MHz */
#define CONFIG_APP_TIMER_PRESCALER 1

/* Timer 2 is on APB1 bus. Its clock speed is PCLK1 from HAL, however,
 * since the divisor for APB is > 1, we have to multiply the frequency,
 * since the clock signal for timers is being multiplied by 2. See 'Clock tree'
 * figure in DM00031020 datasheet */
#define CONFIG_APP_TIMER_BASE_FREQ ((HAL_RCC_GetPCLK1Freq() * 2) / \
CONFIG_APP_TIMER_PRESCALER)

/* Actual period */
#define CONFIG_APP_RPM_TIMER_PERIOD  (1ULL << 32)

/* LED 4 (yellow) for RPM measurement signalling */
#define CONFIG_APP_RPM_LED_PORT C
#define CONFIG_APP_RPM_LED_PIN 3

#define CONFIG_APP_SRV_PWM_PRESCALER 32

/* PWM channels use timers from APB1 bus. Its clock speed is PCLK1 from HAL, however, since the divisor for APB is
 * >= 2, we have to multiply the frequency, since the clock signal for timers is being multiplied by 2. See 'Clock tree'
 * figure in DM00031020 datasheet */
#define CONFIG_APP_SRV_PWM_CLOCK ((HAL_RCC_GetPCLK1Freq() * 2) / CONFIG_APP_SRV_PWM_PRESCALER)

#define CONFIG_APP_TUNED_PIPE_TABLE_SIZE 3
#endif
