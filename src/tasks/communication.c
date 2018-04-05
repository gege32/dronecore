/*
 * communication.c
 *
 *  Created on: Feb 15, 2018
 *      Author: gege3
 */

#include "tasks/communication.h"

void CommunicationTask(void const* argument){

	trace_puts("commStart");
    ESP8266_initialize((UART_HandleTypeDef*)argument);

    for(;;){

    }

}
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
//
//    if(huart == uart_wifi){
//
//    }
//}
