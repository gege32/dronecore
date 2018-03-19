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

extern QueueHandle_t communicationToFlightControllerDataQueue;

arm_pid_instance_q31 * rear_left_motor_pid;
arm_pid_instance_q31 * rear_right_motor_pid;
arm_pid_instance_q31 * front_left_motor_pid;
arm_pid_instance_q31 * front_right_motor_pid;

void FlightControllerTask(void* const arguments);

#endif /* TASKS_FLIGHTCONTROLLER_H_ */
