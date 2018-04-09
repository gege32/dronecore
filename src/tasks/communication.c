/*
 * communication.c
 *
 *  Created on: Feb 15, 2018
 *      Author: gege3
 */

#include "tasks/communication.h"

void CommunicationTask(void const* argument){

    uint8_t* databuffer = pvPortMalloc(sizeof(uint8_t) * 64);

	trace_puts("commStart");
    ESP8266_initialize((UART_HandleTypeDef*)argument);
    ESP8266_checksocket();

    for(;;){

        osDelay(500);
        ESP8266_readdata(databuffer, 10);

    }

}
