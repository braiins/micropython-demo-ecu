/**
 * @file main.c
 *
 * @brief UART driver demonstration - a simple UPCASE echo service
 *
 * This demo contains 1 task that listens for incoming characters and
 * echoes all characters back in upper case. Each processed character
 * is also indicated by toggling the LED.
 */

/* Standard includes. */
#include <stdio.h>
#include <stdarg.h>
#include <ctype.h>

/* FreeRTOS includes */
#include <FreeRTOS.h>
#include <task.h>

/* FreeRTOS drivers */
#include <freertos-drivers/uart/uart.h>
#include <freertos-drivers/gpio/gpio_port.h>

/* lib-rtos includes */
#include <lib-rtos/time.h>
#include <lib-rtos/assert.h>
#include <lib-rtos/task.h>

/* board support package */
#include <bsp.h>

struct task micropython_task;
extern void micropython_task_method(void *args);


/**
 * Hardware initialization
 */
static inline void hw_init()
{
  bsp_init();
}

/*
 * Create the demo tasks then start the scheduler.
 */
int main(void)
{
  int result;

  hw_init();

  result = task__init(&micropython_task,
		      (task__entry_method_t)micropython_task_method,
		      NULL, "uPy-Task",
		      CONFIG_APP_MICROPYTHON_TASK_STACK_SIZE,
		      CONFIG_APP_MICROPYTHON_TASK_PRIORITY);

  assert(result == E_OK);

  /* Finally start the scheduler. */
  vTaskStartScheduler();

  /* Will only reach here if there is insufficient heap available to start
   * the scheduler. */
  return 0;
}

void vApplicationIdleHook(void)
{
}
