/*
 * flightcontroller.c
 *
 *  Created on: Feb 15, 2018
 *      Author: gege3
 */

#include "tasks/flightcontroller.h"

void FlightControllerTask(void* const arguments){

	trace_puts("initFC");
	SensorData buffer;
	BaseType_t newMessage;

	for(;;){


		newMessage = xQueueReceive(sensorDataQueue, &buffer, 50);
		if(newMessage == pdTRUE){
			trace_puts("newMSG");
		}



	}
}
