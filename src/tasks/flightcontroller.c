/*
 * flightcontroller.c
 *
 *  Created on: Feb 15, 2018
 *      Author: gege3
 */

#include "tasks/flightcontroller.h"

void FlightControllerTask(void* const arguments) {

    trace_puts("initFC");
    char szoveg[42];
    SensorData_TypeDef* buffer = pvPortMalloc(sizeof(SensorData_TypeDef));
    ControllerInput_TypeDef* comm_buffer = pvPortMalloc(sizeof(ControllerInput_TypeDef));
    BaseType_t newMessage;

    comm_buffer->throttle = 0;
    comm_buffer->delta_roll = 0;
    comm_buffer->delta_pitch = 0;
    comm_buffer->delta_yaw = 0;

    roll_pid_instance = pvPortMalloc(sizeof(arm_pid_instance_q31));
    pitch_pid_instance = pvPortMalloc(sizeof(arm_pid_instance_q31));
    yaw_pid_instance = pvPortMalloc(sizeof(arm_pid_instance_q31));
    throttle_pid_instance = pvPortMalloc(sizeof(arm_pid_instance_q31));

    float32_t pidgain_f[] = { 0.6f, 0.02f, 0.4f };
    q31_t pidgain_q[3];

    q31_t controlled_q[4];
    q31_t diff_q[4];
    float32_t data_f[4];

    float32_t front_right_throttle_f = 0;
    float32_t front_left_throttle_f = 0;
    float32_t rear_right_throttle_f = 0;
    float32_t rear_left_throttle_f = 0;

    q31_t throttle = 0;
    float32_t throttle_avg = 0.0;

    front_right_throttle = MIN_MOTOR_THROTTLE;
    front_left_throttle = MIN_MOTOR_THROTTLE;
    rear_right_throttle = MIN_MOTOR_THROTTLE;
    rear_left_throttle = MIN_MOTOR_THROTTLE;

    arm_float_to_q31(pidgain_f, pidgain_q, 3);

    roll_pid_instance->Kp = pitch_pid_instance->Kp = yaw_pid_instance->Kp = throttle_pid_instance->Kp = pidgain_q[0];
    roll_pid_instance->Ki = pitch_pid_instance->Ki = yaw_pid_instance->Ki = throttle_pid_instance->Ki = pidgain_q[1];
    roll_pid_instance->Kd = pitch_pid_instance->Kd = yaw_pid_instance->Kd = throttle_pid_instance->Kd = pidgain_q[2];

    arm_pid_init_q31(roll_pid_instance, 1);
    arm_pid_init_q31(pitch_pid_instance, 1);
    arm_pid_init_q31(yaw_pid_instance, 1);
    arm_pid_init_q31(throttle_pid_instance, 1);

    HAL_TIM_Base_Start(&htim2);

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

    CalibrateESC();

    for (;;) {

        xQueueReceive(sensorDataQueue, buffer, portMAX_DELAY);
        xQueueReceive(communicationToFlightControllerDataQueue, comm_buffer, 0);
//        comm_buffer->throttle = 0x40000000; //for testing purposes

        if (comm_buffer->throttle == 0) {
            __HAL_TIM_SET_COMPARE(&htim2, FRONT_LEFT_MOTOR_TIMER, MIN_MOTOR_THROTTLE);
            __HAL_TIM_SET_COMPARE(&htim2, FRONT_RIGHT_MOTOR_TIMER, MIN_MOTOR_THROTTLE);
            __HAL_TIM_SET_COMPARE(&htim2, REAR_LEFT_MOTOR_TIMER, MIN_MOTOR_THROTTLE);
            __HAL_TIM_SET_COMPARE(&htim2, REAR_RIGHT_MOTOR_TIMER, MIN_MOTOR_THROTTLE);
        } else {

            arm_sub_q31(&(comm_buffer->delta_roll), &(buffer->roll), diff_q, 1);
            arm_sub_q31(&(comm_buffer->delta_pitch), &(buffer->pitch), diff_q + 1, 1);
//        arm_sub_q31(&(comm_buffer->delta_yaw), &(buffer->yaw), diff_q + 2, 1); // for yaw the rotation speed is what need to be taken in cosideration
            arm_sub_q31(&(comm_buffer->throttle), &throttle, diff_q + 3, 1);

            controlled_q[0] = arm_pid_q31(roll_pid_instance, diff_q[0] >> 2);
            controlled_q[1] = arm_pid_q31(pitch_pid_instance, diff_q[1] >> 2);
            controlled_q[2] = arm_pid_q31(yaw_pid_instance, comm_buffer->delta_yaw >> 2);
            controlled_q[3] = arm_pid_q31(throttle_pid_instance, diff_q[3] >> 2);

            arm_q31_to_float(controlled_q, data_f, 4);

            //0: roll
            //1: pitch
            //2: yaw
            //3: throttle

            front_left_throttle_f = (data_f[3] - data_f[0] + data_f[1] + data_f[2]);
            front_right_throttle_f = (data_f[3] + data_f[0] + data_f[1] - data_f[2]);
            rear_left_throttle_f = (data_f[3] - data_f[0] - data_f[1] - data_f[2]);
            rear_right_throttle_f = (data_f[3] + data_f[0] - data_f[1] + data_f[2]);

            throttle_avg = (front_left_throttle_f + front_right_throttle_f + rear_right_throttle_f + rear_left_throttle_f) / (float32_t) 4.0;
            arm_float_to_q31(&throttle_avg, &throttle, 1);

            front_left_throttle = ((front_left_throttle_f + (float32_t) 20.0) / (float32_t) 16.0) * (float32_t) 1000.0;
            front_right_throttle = ((front_right_throttle_f + (float32_t) 20.0) / (float32_t) 16.0) * (float32_t) 1000.0;
            rear_left_throttle = ((rear_left_throttle_f + (float32_t) 20.0) / (float32_t) 16.0) * (float32_t) 1000.0;
            rear_right_throttle = ((rear_right_throttle_f + (float32_t) 20.0) / (float32_t) 16.0) * (float32_t) 1000.0;

            if(front_left_throttle > MAX_LIMIT_MOTOR_THROTTLE){
                front_left_throttle = MAX_LIMIT_MOTOR_THROTTLE;
            }
            if(front_right_throttle > MAX_LIMIT_MOTOR_THROTTLE){
                front_right_throttle = MAX_LIMIT_MOTOR_THROTTLE;
            }
            if(rear_left_throttle > MAX_LIMIT_MOTOR_THROTTLE){
                rear_left_throttle = MAX_LIMIT_MOTOR_THROTTLE;
            }
            if(rear_right_throttle > MAX_LIMIT_MOTOR_THROTTLE){
                rear_right_throttle = MAX_LIMIT_MOTOR_THROTTLE;
            }

            __HAL_TIM_SET_COMPARE(&htim2, FRONT_LEFT_MOTOR_TIMER, front_left_throttle);
            __HAL_TIM_SET_COMPARE(&htim2, FRONT_RIGHT_MOTOR_TIMER, front_right_throttle);
            __HAL_TIM_SET_COMPARE(&htim2, REAR_LEFT_MOTOR_TIMER, rear_left_throttle);
            __HAL_TIM_SET_COMPARE(&htim2, REAR_RIGHT_MOTOR_TIMER, rear_right_throttle);
        }

//        snprintf(szoveg, 42, "%+.6f,%+.6f,%+.6f,%+.6f\r\n", data_f[0], data_f[1], data_f[2], data_f[3]);
//
//        HAL_UART_Transmit(&huart1, szoveg, 42, 20);

//        snprintf(szoveg, 22, "%i,%i,%i,%i\r\n", front_left_throttle, front_right_throttle, rear_left_throttle, rear_right_throttle);
//        HAL_UART_Transmit(&huart1, szoveg, 22, 20);

    }
}

void CalibrateESC() {

    //sending max
    __HAL_TIM_SET_COMPARE(&htim2, FRONT_LEFT_MOTOR_TIMER, MAX_MOTOR_THROTTLE);
    __HAL_TIM_SET_COMPARE(&htim2, FRONT_RIGHT_MOTOR_TIMER, MAX_MOTOR_THROTTLE);
    __HAL_TIM_SET_COMPARE(&htim2, REAR_LEFT_MOTOR_TIMER, MAX_MOTOR_THROTTLE);
    __HAL_TIM_SET_COMPARE(&htim2, REAR_RIGHT_MOTOR_TIMER, MAX_MOTOR_THROTTLE);

    osDelay(7000);

    //back to zero throttle

    __HAL_TIM_SET_COMPARE(&htim2, FRONT_LEFT_MOTOR_TIMER, MIN_MOTOR_THROTTLE);
    __HAL_TIM_SET_COMPARE(&htim2, FRONT_RIGHT_MOTOR_TIMER, MIN_MOTOR_THROTTLE);
    __HAL_TIM_SET_COMPARE(&htim2, REAR_LEFT_MOTOR_TIMER, MIN_MOTOR_THROTTLE);
    __HAL_TIM_SET_COMPARE(&htim2, REAR_RIGHT_MOTOR_TIMER, MIN_MOTOR_THROTTLE);

    osDelay(4000);

}
