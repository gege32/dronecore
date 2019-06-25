/*
 * fsia6b.c
 *
 *  Created on: Jun 3, 2019
 *      Author: gege3
 */

#include "commdriver/fsia6b.h"

void fsia6b_getRemoteControlData(){
    //since its possible to start the read while the receiver is already sending, I read two packages to make sure to read 1 full package
    HAL_StatusTypeDef status = HAL_UART_Receive_DMA(&huart2, buffer, PACKAGE_LENGHT * 2);
}

void fsia6b_init(){
    buffer = pvPortMalloc(sizeof(uint8_t) * PACKAGE_LENGHT * 2);
    controllerInput = pvPortMalloc(sizeof(ControllerInput_TypeDef));
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    int index = 0;
    for(;(buffer[index] != 0x20 && buffer[index+1] != 0x40) && index < PACKAGE_LENGHT + 1; index++);
    if(buffer[index+0] == 0x20 && buffer[index+1] == 0x40){
        controllerInput->throttle = buffer[index+7] << 8 | buffer[index+6];
        controllerInput->delta_roll = (float32_t)((buffer[index+3] << 8 | buffer[index+2]) - 1500) / 500.0;
        controllerInput->delta_pitch = (float32_t)((buffer[index+5] << 8 | buffer[index+4]) - 1500) / 500.0;
        controllerInput->delta_yaw = (float32_t)((buffer[index+9] << 8 | buffer[index+8]) - 1500 ) / 500.0;
        controllerInput->vra = buffer[index+11] << 8 | buffer[index+10];
        controllerInput->vrb = buffer[index+13] << 8 | buffer[index+12];

        controllerInput->swa = buffer[index+15] << 8 | buffer[index+14];
        controllerInput->swb = buffer[index+17] << 8 | buffer[index+16];
        controllerInput->swc = buffer[index+19] << 8 | buffer[index+18];
        controllerInput->armMotors = (buffer[index+21] << 8 | buffer[index+20]) - 1000;
        xQueueOverwriteFromISR(communicationToFlightControllerDataQueue, controllerInput, pdFALSE);
    }
}
