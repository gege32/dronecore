/*
 * communication.c
 *
 *  Created on: Feb 15, 2018
 *      Author: gege3
 */

#include "tasks/communication.h"

void CommunicationTask(void const* argument){

    trace_puts("comm task start");

    Wifi_Init(osPriorityNormal, (UART_HandleTypeDef*)argument);

    Wifi_Station_ConnectToAp("Fostartaly", "q1e3tw2r4", "00:11:22:33:44:55");
    trace_puts("conn on wifi");

    for(;;){
        trace_puts("megbassza");
    }

}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

    if(huart == uart_handle){
        Wifi_RxCallBack();
    }
}
