/**
 * @file rpm.h
 * @author Copyright (c) 2012 Braiins Systems s.r.o. All rights reserved.
 * @brief RPM measuring handler class see details below
 */
#ifndef _RPM_H_
#define _RPM_H_

/* driver framework includes */
#include <freertos-drivers/timer/timer.h>
#include <freertos-drivers/input-capture/input_capture.h>

/* lib-rtos includes */
#include <lib-rtos/task.h>

/* lib-daq includes */
#include <lib-daq/dst-gauge/dst_gauge.h>

#include "config.app.h"

/**
 * Represents the complete rpm measurement handler. The handler
 * consists of a dst_gauge that uses input capture channel
 * running. The maximum period of the RPM signal is givin by
 * CONFIG_APP_RPM_MAX_PERIOD_LIMIT.
 *
 * Each RPM sample is indicated by toggling the associated LED
 * (configured via CONFIG_APP_RPM_LED_{PORT,PIN})
 */
struct rpm {
  /** Performs the actual RPM measurement */
  struct dst_gauge *rpm_gauge;

  /** Task that performs the actual RPM measurement */
  struct task task;
};


/**
 * Initializes the rpm handler and its task
 *
 * @memberof rpm
 * @param *self - this rpm task instance
 */
void rpm__init(struct rpm *self);


/**
 * Current RPM value accessor
 *
 * @memberof rpm
 * @param *self - this rpm instance
 *
 * @return current RPM value or 0 when no signal is present
 */
static inline unsigned int rpm__get(struct rpm *self)
{
  timer__duration_t rpm_ticks;

  rpm_ticks = dst_gauge__get_ticks(self->rpm_gauge);

  return TIME__TICKS_TO_RPM(rpm_ticks, CONFIG_APP_TIMER_BASE_FREQ);
}


/**
 * Main task responsible for rpm sampling
 *
 * @memberof rpm
 * @param *self - this rpm instance
 */
void rpm__task(struct rpm *self);


#endif /* _RPM_H_ */
