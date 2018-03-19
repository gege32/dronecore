/*
 * flightcontroller.c
 *
 *  Created on: Feb 15, 2018
 *      Author: gege3
 */

#include "tasks/flightcontroller.h"

void FlightControllerTask(void* const arguments){

	trace_puts("initFC");
	SensorData_TypeDef* buffer = pvPortMalloc(sizeof(SensorData_TypeDef*));
	BaseType_t newMessage;

	rear_left_motor_pid = pvPortMalloc(sizeof(arm_pid_instance_q31*));
	rear_right_motor_pid = pvPortMalloc(sizeof(arm_pid_instance_q31*));
	front_left_motor_pid = pvPortMalloc(sizeof(arm_pid_instance_q31*));
	front_right_motor_pid = pvPortMalloc(sizeof(arm_pid_instance_q31*));

	float32_t pidgain_f [] = {3.0f, 0.001f, 0.1f };


	rear_left_motor_pid->Kp = rear_right_motor_pid->Kp = front_left_motor_pid->Kp = front_right_motor_pid->Kp = 5;
	rear_left_motor_pid->Ki = rear_right_motor_pid->Ki = front_left_motor_pid->Ki = front_right_motor_pid->Ki = 5;
	rear_left_motor_pid->Kd = rear_right_motor_pid->Kd = front_left_motor_pid->Kd = front_right_motor_pid->Kd = 5;

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
