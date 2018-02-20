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
#include "commdriver/Wifi.h"

void CommunicationTask(void const* argument);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);



#endif /* TASKS_COMMUNICATION_H_ */
