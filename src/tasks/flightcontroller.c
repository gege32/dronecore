/*
 * flightcontroller.c
 *
 *  Created on: Feb 15, 2018
 *      Author: gege3
 */

#include "tasks/flightcontroller.h"

void FlightControllerTask(void* const arguments){

	trace_puts("initFC");
	SensorData_TypeDef buffer;
	BaseType_t newMessage;



	arm_pid_init_q31(rear_left_motor_pid, 1);
	arm_pid_init_q31(rear_right_motor_pid, 1);
	arm_pid_init_q31(front_left_motor_pid, 1);
	arm_pid_init_q31(front_right_motor_pid, 1);

	for(;;){


		newMessage = xQueueReceive(sensorDataQueue, &buffer, 50);
		if(newMessage == pdTRUE){
			trace_puts("newMSG");
		}





	}
}
