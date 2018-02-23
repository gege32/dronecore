/*
 * flightcontroller.h
 *
 *  Created on: Feb 15, 2018
 *      Author: gege3
 */

#ifndef TASKS_FLIGHTCONTROLLER_H_
#define TASKS_FLIGHTCONTROLLER_H_

#include "arm_math.h"
#include "cmsis_os.h"
#include "semihosting/Trace.h"
#include "datamodels.h"

extern QueueHandle_t sensorDataQueue;

void FlightControllerTask(void* const arguments);

#endif /* TASKS_FLIGHTCONTROLLER_H_ */
