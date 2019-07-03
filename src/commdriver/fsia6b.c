/*
 * fsia6b.c
 *
 *  Created on: Jun 3, 2019
 *      Author: gege3
 */

#include <arm_math.h>
#include <commdriver/fsia6b.h>
#include <portable.h>
#include <queue.h>
#include <stdint-gcc.h>
#include <stm32f1xx_hal_def.h>
#include <stm32f1xx_hal_uart.h>

void fsia6b_getRemoteControlData() {
    //since its possible to start the read while the receiver is already sending, I read two packages to make sure to read 1 full package
    HAL_UART_Receive_DMA(&huart2, buffer, PACKAGE_LENGHT * 2);
    xSemaphoreTake(controllerDataReady, portMAX_DELAY);

    int index = 0;
    for (; (buffer[index] != 0x20 && buffer[index + 1] != 0x40) && index < PACKAGE_LENGHT + 1; index++);
    if (buffer[index + 0] == 0x20 && buffer[index + 1] == 0x40) {
        controllerInput->throttle = buffer[index + 7] << 8 | buffer[index + 6];
        controllerInput->delta_roll = (float32_t) ((buffer[index + 3] << 8 | buffer[index + 2]) - 1500) / 500.0;
        controllerInput->delta_pitch = (float32_t) ((buffer[index + 5] << 8 | buffer[index + 4]) - 1500) / 500.0;
        controllerInput->delta_yaw = (float32_t) ((buffer[index + 9] << 8 | buffer[index + 8]) - 1500) / 500.0;
        controllerInput->vra = buffer[index + 11] << 8 | buffer[index + 10];
        controllerInput->vrb = buffer[index + 13] << 8 | buffer[index + 12];

        controllerInput->swa = buffer[index + 15] << 8 | buffer[index + 14];
        controllerInput->swb = buffer[index + 17] << 8 | buffer[index + 16];
        controllerInput->swc = buffer[index + 19] << 8 | buffer[index + 18];
        controllerInput->armMotors = (buffer[index + 21] << 8 | buffer[index + 20]);
        if (controllerInput->armMotors == 2000) {
            controllerInput->armMotors = 1;
        } else {
            controllerInput->armMotors = 0;
        }
        xQueueOverwrite(communicationToFlightControllerDataQueue, controllerInput);
    }

}

void fsia6b_init() {
//    buffer = pvPortMalloc(sizeof(uint8_t) * PACKAGE_LENGHT * 2);
    controllerInput = pvPortMalloc(sizeof(ControllerInput_TypeDef));
    controllerDataReady = xSemaphoreCreateBinary();

    //waiting for the receiver to start transmitting
    while (HAL_UART_Receive(&huart2, buffer, 1, 100) == HAL_TIMEOUT);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    if((huart->Instance == USART2)){
        __HAL_UART_FLUSH_DRREGISTER(huart);
        xSemaphoreGiveFromISR(controllerDataReady, pdFALSE);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
    if((huart->Instance == USART2)){
        Error_Handler();
    }
}
