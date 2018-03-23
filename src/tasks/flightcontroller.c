/*
 * flightcontroller.c
 *
 *  Created on: Feb 15, 2018
 *      Author: gege3
 */

#include "tasks/flightcontroller.h"

void FlightControllerTask(void* const arguments){

	trace_puts("initFC");
	char szoveg [40];
	SensorData_TypeDef* buffer = pvPortMalloc(sizeof(SensorData_TypeDef));
	BaseType_t newMessage;

	roll_pid_instance = pvPortMalloc(sizeof(arm_pid_instance_q31));
	pitch_pid_instance = pvPortMalloc(sizeof(arm_pid_instance_q31));
	yaw_pid_instance = pvPortMalloc(sizeof(arm_pid_instance_q31));
	height_pid_instance = pvPortMalloc(sizeof(arm_pid_instance_q31));

	float32_t pidgain_f [] = {0.9f, 0.0001f, 0.01f};
	q31_t pidgain_q [3];

	q31_t controlled_q [3];
	float32_t data_f[3];

	uint32_t front_left_correction = 0;
	uint32_t front_right_correction = 0;
	uint32_t rear_left_correction = 0;
	uint32_t rear_right_correction = 0;

	uint32_t roll_correction = 0;
	uint32_t pitch_correction = 0;
	uint32_t yaw_correction = 0;

	front_right_throttle = MIN_MOTOR_THROTTLE;
	front_left_throttle = MIN_MOTOR_THROTTLE;
	rear_right_throttle = MIN_MOTOR_THROTTLE;
	rear_left_throttle = MIN_MOTOR_THROTTLE;

	arm_float_to_q31(pidgain_f, pidgain_q, 3);

	roll_pid_instance->Kp = pitch_pid_instance->Kp = yaw_pid_instance->Kp = height_pid_instance->Kp = pidgain_q[0];
	roll_pid_instance->Ki = pitch_pid_instance->Ki = yaw_pid_instance->Ki = height_pid_instance->Ki = pidgain_q[1];
	roll_pid_instance->Kd = pitch_pid_instance->Kd = yaw_pid_instance->Kd = height_pid_instance->Kd = pidgain_q[2];

	arm_pid_init_q31(roll_pid_instance, 1);
	arm_pid_init_q31(pitch_pid_instance, 1);
	arm_pid_init_q31(yaw_pid_instance, 1);
	arm_pid_init_q31(height_pid_instance, 1);

	HAL_TIM_Base_Start(&htim2);

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

//    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1000);
//    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1000);
//    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 2000);
//    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 2000);

    front_left_throttle = 1500;
    front_right_throttle = 1500;
    rear_left_throttle = 1500;
    rear_right_throttle = 1500;

	for(;;){


		newMessage = xQueueReceive(sensorDataQueue, buffer, 50);
		if(newMessage == pdTRUE){
		    controlled_q[0] = arm_pid_q31(roll_pid_instance, buffer->roll);
		    controlled_q[1] = arm_pid_q31(pitch_pid_instance, buffer->pitch);
		    controlled_q[2] = arm_pid_q31(yaw_pid_instance, buffer->yaw);

		    arm_q31_to_float(controlled_q, data_f, 3);

		    roll_correction = data_f[0] * 1000;
		    pitch_correction = data_f[1] * 1000;
		    yaw_correction = data_f[2] * 1000;

		    if(roll_correction > 0){
		    	front_left_correction += roll_correction;
		    	front_right_correction -= roll_correction;
		    	rear_left_correction += roll_correction;
		    	rear_right_correction -= roll_correction;
		    }else{
		    	front_left_correction -= roll_correction;
		    	front_right_correction += roll_correction;
		    	rear_left_correction -= roll_correction;
		    	rear_right_correction += roll_correction;
		    }
		    if(pitch_correction > 0){
		    	front_left_correction += pitch_correction;
		    	front_right_correction += pitch_correction;
		    	rear_left_correction -= pitch_correction;
		    	rear_right_correction -= pitch_correction;
		    }else{
		    	front_left_correction -= pitch_correction;
		    	front_right_correction -= pitch_correction;
		    	rear_left_correction += pitch_correction;
		    	rear_right_correction += pitch_correction;
		    }

		    front_left_throttle += front_left_correction;
		    front_right_throttle += front_right_correction;
		    rear_left_throttle += rear_left_correction;
		    rear_right_throttle += rear_right_correction;

		    //checking not to run out of throttle bounds
		    if(front_left_throttle < MIN_MOTOR_THROTTLE){
		    	front_left_throttle = MIN_MOTOR_THROTTLE;
		    }else if(front_left_throttle > MAX_MOTOR_THROTTLE){
		    	front_left_throttle = MAX_MOTOR_THROTTLE;
		    }
		    if(front_right_throttle < MIN_MOTOR_THROTTLE){
		    	front_right_throttle = MIN_MOTOR_THROTTLE;
		    }else if(front_right_throttle > MAX_MOTOR_THROTTLE){
		    	front_right_throttle = MAX_MOTOR_THROTTLE;
		    }
		    if(rear_left_throttle < MIN_MOTOR_THROTTLE){
		    	rear_left_throttle = MIN_MOTOR_THROTTLE;
		    }else if(rear_left_throttle > MAX_MOTOR_THROTTLE){
		    	rear_left_throttle = MAX_MOTOR_THROTTLE;
		    }
		    if(rear_right_throttle < MIN_MOTOR_THROTTLE){
		    	rear_right_throttle = MIN_MOTOR_THROTTLE;
		    }else if(rear_right_throttle > MAX_MOTOR_THROTTLE){
		    	rear_right_throttle = MAX_MOTOR_THROTTLE;
		    }

			__HAL_TIM_SET_COMPARE(&htim2, FRONT_LEFT_MOTOR_TIMER, front_left_throttle);
			__HAL_TIM_SET_COMPARE(&htim2, FRONT_RIGHT_MOTOR_TIMER, front_right_throttle);
			__HAL_TIM_SET_COMPARE(&htim2, REAR_LEFT_MOTOR_TIMER, rear_left_throttle);
			__HAL_TIM_SET_COMPARE(&htim2, REAR_RIGHT_MOTOR_TIMER, rear_right_throttle);

			front_left_correction = 0;
			front_right_correction = 0;
			rear_left_correction = 0;
			rear_right_correction = 0;

		    snprintf(szoveg, 40, "%if,%i,%i,%i\r", front_left_throttle, front_right_throttle, rear_left_throttle, rear_right_throttle);
		    HAL_UART_Transmit(&huart1, szoveg, 40, 20);
		}
	}
}

void CalibrateESC(){

	//sending max
	__HAL_TIM_SET_COMPARE(&htim2, FRONT_LEFT_MOTOR_TIMER, MAX_MOTOR_THROTTLE);
	__HAL_TIM_SET_COMPARE(&htim2, FRONT_RIGHT_MOTOR_TIMER, MAX_MOTOR_THROTTLE);
	__HAL_TIM_SET_COMPARE(&htim2, REAR_LEFT_MOTOR_TIMER, MAX_MOTOR_THROTTLE);
	__HAL_TIM_SET_COMPARE(&htim2, REAR_RIGHT_MOTOR_TIMER, MAX_MOTOR_THROTTLE);

	osDelay(5000);

	//back to zero throttle

	__HAL_TIM_SET_COMPARE(&htim2, FRONT_LEFT_MOTOR_TIMER, MIN_MOTOR_THROTTLE);
	__HAL_TIM_SET_COMPARE(&htim2, FRONT_RIGHT_MOTOR_TIMER, MIN_MOTOR_THROTTLE);
	__HAL_TIM_SET_COMPARE(&htim2, REAR_LEFT_MOTOR_TIMER, MIN_MOTOR_THROTTLE);
	__HAL_TIM_SET_COMPARE(&htim2, REAR_RIGHT_MOTOR_TIMER, MIN_MOTOR_THROTTLE);
}
