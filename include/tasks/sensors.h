/*
 * sensors.h
 *
 *  Created on: Feb 12, 2018
 *      Author: Horvath_Gergo
 */

#ifndef TASKS_SENSORS_H_
#define TASKS_SENSORS_H_

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "semihosting/Trace.h"
#include "sensordriver/mpu6050.h"
#include "sensordriver/bmp180.h"
#include "datamodels.h"
#include "arm_math.h"

SemaphoreHandle_t dataReady;

extern UART_HandleTypeDef huart1;

extern TIM_HandleTypeDef htim3;

extern QueueHandle_t sensorDataQueue;

CalibrationData * bmp180_calibration_data;

void SensorMeasurementTask(void const* argument);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

#endif TASKS_SENSORS_H_
