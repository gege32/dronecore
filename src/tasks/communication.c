/*
 * communication.c
 *
 *  Created on: Feb 15, 2018
 *      Author: gege3
 */

#include "tasks/communication.h"

void CommunicationTask(void const* argument){

    trace_puts("sensor task started");

    nRF24_HAL_Init((SPI_HandleTypeDef*)argument);

    uint32_t calc = 0;

    for(;;){
        calc++;

    }

}
