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

#include "commdriver/nrf24.h"

#include "datamodels.h"

extern QueueHandle_t communicationToFlightControllerDataQueue;

void CommunicationTask(void const* argument);

#endif /* TASKS_COMMUNICATION_H_ */
