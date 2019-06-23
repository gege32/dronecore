/*
 * communication.h
 *
 *  Created on: Feb 15, 2018
 *      Author: gege3
 */

#ifndef TASKS_COMMUNICATION_H_
#define TASKS_COMMUNICATION_H_

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "semihosting/Trace.h"

#include "commdriver/fsia6b.h"

#include "datamodels.h"

extern UART_HandleTypeDef huart1;

extern QueueHandle_t PIDtuningDataQueue;

void CommunicationTask(void const* argument);

#endif /* TASKS_COMMUNICATION_H_ */
