/*
 * flightcontroller.h
 *
 *  Created on: Feb 15, 2018
 *      Author: gege3
 */

#ifndef TASKS_FLIGHTCONTROLLER_H_
#define TASKS_FLIGHTCONTROLLER_H_

#include "stm32f1xx_hal.h"
#include "arm_math.h"
#include "cmsis_os.h"
#include "semihosting/Trace.h"
#include "datamodels.h"

#define MAX_MOTOR_THROTTLE 1700
#define MAX_LIMIT_MOTOR_THROTTLE 1500
#define IDLE_MOTOR_THROTTLE 1200
#define MIN_MOTOR_THROTTLE 1000

#define FRONT_LEFT_MOTOR_TIMER TIM_CHANNEL_1 //PA15
#define FRONT_RIGHT_MOTOR_TIMER TIM_CHANNEL_2 //PB3
#define REAR_RIGHT_MOTOR_TIMER TIM_CHANNEL_3 //PB10
#define REAR_LEFT_MOTOR_TIMER TIM_CHANNEL_4 //PB11

extern QueueHandle_t sensorDataQueue;

extern QueueHandle_t communicationToFlightControllerDataQueue;

extern QueueHandle_t PIDtuningDataQueue;

extern UART_HandleTypeDef huart1;

extern TIM_HandleTypeDef htim2;

arm_pid_instance_f32 * roll_pid_instance;
arm_pid_instance_f32 * pitch_pid_instance;
arm_pid_instance_f32 * yaw_pid_instance;
arm_pid_instance_f32 * throttle_pid_instance;

uint32_t front_right_throttle;
uint32_t front_left_throttle;
uint32_t rear_right_throttle;
uint32_t rear_left_throttle;

void FlightControllerTask(void* const arguments);
void CalibrateESC();

#endif /* TASKS_FLIGHTCONTROLLER_H_ */
