/*
 * flightcontroller.c
 *
 *  Created on: Feb 15, 2018
 *      Author: gege3
 */

#include "tasks/flightcontroller.h"

void FlightControllerTask(void* const arguments) {

    trace_puts("initFC");
    char szoveg[80];
    SensorData_TypeDef* buffer = pvPortMalloc(sizeof(SensorData_TypeDef));
    ControllerInput_TypeDef* comm_buffer = pvPortMalloc(sizeof(ControllerInput_TypeDef));

    comm_buffer->throttle = 0;
    comm_buffer->delta_roll = 0;
    comm_buffer->delta_pitch = 0;
    comm_buffer->delta_yaw = 0;

    roll_pid_instance = pvPortMalloc(sizeof(arm_pid_instance_f32));
    pitch_pid_instance = pvPortMalloc(sizeof(arm_pid_instance_f32));
    yaw_pid_instance = pvPortMalloc(sizeof(arm_pid_instance_f32));
    throttle_pid_instance = pvPortMalloc(sizeof(arm_pid_instance_f32));

    PIDtuning_TypeDef* pid_gains = pvPortMalloc(sizeof(PIDtuning_TypeDef));
    pid_gains->p = 50.0f;
    pid_gains->i = 0.0f;
    pid_gains->d = 0.0f;

    float32_t controlled_values[] = { 0.0f, 0.0f, 0.0f, 0.0f };

    float32_t front_right_throttle_f = 0;
    float32_t front_left_throttle_f = 0;
    float32_t rear_right_throttle_f = 0;
    float32_t rear_left_throttle_f = 0;

    float32_t throttle = 0.0f;

    float32_t throttle_avg = 0.0f;

    front_right_throttle = MIN_MOTOR_THROTTLE;
    front_left_throttle = MIN_MOTOR_THROTTLE;
    rear_right_throttle = MIN_MOTOR_THROTTLE;
    rear_left_throttle = MIN_MOTOR_THROTTLE;

    roll_pid_instance->Kp = pitch_pid_instance->Kp = yaw_pid_instance->Kp = pid_gains->p;
    roll_pid_instance->Ki = pitch_pid_instance->Ki = yaw_pid_instance->Ki = pid_gains->i;
    roll_pid_instance->Kd = pitch_pid_instance->Kd = yaw_pid_instance->Kd = pid_gains->d;

    throttle_pid_instance->Kp = 1.0f;
    throttle_pid_instance->Ki = 0.0f;
    throttle_pid_instance->Kd = 0.0f;

    arm_pid_init_f32(roll_pid_instance, 1);
    arm_pid_init_f32(pitch_pid_instance, 1);
    arm_pid_init_f32(yaw_pid_instance, 1);
    arm_pid_init_f32(throttle_pid_instance, 1);

    HAL_TIM_Base_Start(&htim2);

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

    BaseType_t new_message;

//    CalibrateESC();

    for (;;) {

        xQueueReceive(sensorDataQueue, buffer, portMAX_DELAY);
        xQueueReceive(communicationToFlightControllerDataQueue, comm_buffer, 0);

        new_message = xQueueReceive(PIDtuningDataQueue, pid_gains, 0);
        if (new_message == pdTRUE) {
            roll_pid_instance->Kp = pitch_pid_instance->Kp = yaw_pid_instance->Kp = pid_gains->p;
            roll_pid_instance->Ki = pitch_pid_instance->Ki = yaw_pid_instance->Ki = pid_gains->i;
            roll_pid_instance->Kd = pitch_pid_instance->Kd = yaw_pid_instance->Kd = pid_gains->d;
            arm_pid_init_f32(roll_pid_instance, 1);
            arm_pid_init_f32(pitch_pid_instance, 1);
            arm_pid_init_f32(yaw_pid_instance, 1);
            throttle = 0.0f;
        }

        xQueueReceive(communicationToFlightControllerDataQueue, comm_buffer, 0);
        new_message = xQueueReceive(sensorDataQueue, buffer, portMAX_DELAY);

//        comm_buffer->throttle = 0.5f;
        if (new_message) {

            if (comm_buffer->throttle == 0.0) {
                __HAL_TIM_SET_COMPARE(&htim2, FRONT_LEFT_MOTOR_TIMER, MIN_MOTOR_THROTTLE);
                __HAL_TIM_SET_COMPARE(&htim2, FRONT_RIGHT_MOTOR_TIMER, MIN_MOTOR_THROTTLE);
                __HAL_TIM_SET_COMPARE(&htim2, REAR_LEFT_MOTOR_TIMER, MIN_MOTOR_THROTTLE);
                __HAL_TIM_SET_COMPARE(&htim2, REAR_RIGHT_MOTOR_TIMER, MIN_MOTOR_THROTTLE);
                arm_pid_reset_f32(roll_pid_instance);
                arm_pid_reset_f32(pitch_pid_instance);
                arm_pid_reset_f32(yaw_pid_instance);
                arm_pid_reset_f32(throttle_pid_instance);
                throttle = 0.0f;
            } else {

                controlled_values[0] = arm_pid_f32(roll_pid_instance, comm_buffer->delta_roll - buffer->roll);
                controlled_values[1] = arm_pid_f32(pitch_pid_instance, comm_buffer->delta_pitch - buffer->pitch);
                controlled_values[2] = arm_pid_f32(yaw_pid_instance, comm_buffer->delta_yaw);
                controlled_values[3] = arm_pid_f32(throttle_pid_instance, ((comm_buffer->throttle + 1) * 1000) - throttle_avg);

                throttle += controlled_values[3];
//            throttle = (comm_buffer->throttle + 1) * 1000;
                //0: roll
                //1: pitch
                //2: yaw
                //3: throttle

                front_left_throttle_f = (throttle + controlled_values[0] - controlled_values[1] + controlled_values[2]);
                front_right_throttle_f = (throttle - controlled_values[0] - controlled_values[1] - controlled_values[2]);
                rear_left_throttle_f = (throttle + controlled_values[0] + controlled_values[1] - controlled_values[2]);
                rear_right_throttle_f = (throttle - controlled_values[0] + controlled_values[1] + controlled_values[2]);

                throttle_avg = (front_left_throttle_f + front_right_throttle_f + rear_right_throttle_f + rear_left_throttle_f) / (float32_t) 4.0;

                front_left_throttle = (uint32_t) front_left_throttle_f;
                front_right_throttle = (uint32_t) front_right_throttle_f;
                rear_left_throttle = (uint32_t) rear_left_throttle_f;
                rear_right_throttle = (uint32_t) rear_right_throttle_f;

                if (front_left_throttle > MAX_LIMIT_MOTOR_THROTTLE) {
                    front_left_throttle = MAX_LIMIT_MOTOR_THROTTLE;
                }
                if (front_right_throttle > MAX_LIMIT_MOTOR_THROTTLE) {
                    front_right_throttle = MAX_LIMIT_MOTOR_THROTTLE;
                }
                if (rear_left_throttle > MAX_LIMIT_MOTOR_THROTTLE) {
                    rear_left_throttle = MAX_LIMIT_MOTOR_THROTTLE;
                }
                if (rear_right_throttle > MAX_LIMIT_MOTOR_THROTTLE) {
                    rear_right_throttle = MAX_LIMIT_MOTOR_THROTTLE;
                }

                __HAL_TIM_SET_COMPARE(&htim2, FRONT_LEFT_MOTOR_TIMER, front_left_throttle);
                __HAL_TIM_SET_COMPARE(&htim2, FRONT_RIGHT_MOTOR_TIMER, front_right_throttle);
                __HAL_TIM_SET_COMPARE(&htim2, REAR_LEFT_MOTOR_TIMER, rear_left_throttle);
                __HAL_TIM_SET_COMPARE(&htim2, REAR_RIGHT_MOTOR_TIMER, rear_right_throttle);

//            snprintf(szoveg, 22, "%i,%i,%i,%i\r\n", front_left_throttle, front_right_throttle, rear_left_throttle, rear_right_throttle);
//            HAL_UART_Transmit(&huart1, szoveg, 22, 20);
            }

        }

        snprintf(szoveg, 72, "%+.6f,%+.6f,%+.6f,%+.6f,%+.6f,%+.6f,%+.6f\r\n", buffer->roll, buffer->pitch, buffer->yaw, controlled_values[0], controlled_values[1], controlled_values[2], controlled_values[3]);
        HAL_UART_Transmit(&huart1, szoveg, 72, 20);

        //        snprintf(szoveg, 32, "%+.6f,%+.6f,%+.6f\r\n", data_f[0], data_f[1], data_f[2]);
        //        HAL_UART_Transmit(&huart1, szoveg, 32, 20);

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
