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

	roll_pid_instance = pvPortMalloc(sizeof(arm_pid_instance_q31));
	pitch_pid_instance = pvPortMalloc(sizeof(arm_pid_instance_q31));
	yaw_pid_instance = pvPortMalloc(sizeof(arm_pid_instance_q31));
	height_pid_instance = pvPortMalloc(sizeof(arm_pid_instance_q31));

	float32_t pidgain_f [] = {3.0f, 0.001f, 0.1f };
	q31_t pidgain_q [3];

	arm_float_to_q31(pidgain_f, pidgain_q, 3);

	roll_pid_instance->Kp = pitch_pid_instance->Kp = yaw_pid_instance->Kp = height_pid_instance->Kp = pidgain_q[0];
	roll_pid_instance->Ki = pitch_pid_instance->Ki = yaw_pid_instance->Ki = height_pid_instance->Ki = pidgain_q[1];
	roll_pid_instance->Kd = pitch_pid_instance->Kd = yaw_pid_instance->Kd = height_pid_instance->Kd = pidgain_q[2];

	arm_pid_init_q31(roll_pid_instance, 1);
	arm_pid_init_q31(pitch_pid_instance, 1);
	arm_pid_init_q31(yaw_pid_instance, 1);
	arm_pid_init_q31(height_pid_instance, 1);

	for(;;){


		newMessage = xQueueReceive(sensorDataQueue, &buffer, 50);
		if(newMessage == pdTRUE){

		}





	}
}
