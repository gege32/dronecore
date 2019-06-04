/*
 * fsia6b.c
 *
 *  Created on: Jun 3, 2019
 *      Author: gege3
 */

#include "commdriver/fsia6b.h"

void fsia6b_getRemoteControlData(ControllerInput_TypeDef controllerInput){
    HAL_StatusTypeDef status = HAL_UART_Receive_DMA(&huart2, buffer, PACKAGE_LENGHT);
}

void fsia6b_init(QueueHandle_t *queueHandle){
    commQueue = queueHandle;
    buffer = pvPortMalloc(sizeof(uint8_t) * PACKAGE_LENGHT);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    ControllerInput_TypeDef controllerinput = pvPortMalloc(sizeof(ControllerInput_TypeDef));
    if(buffer[0] == 0x20 && buffer[1] == 0x40){
        controllerinput->throttle = buffer[7] << 8 | buffer[6];
        controllerinput->delta_roll = buffer[3] << 8 | buffer[2];
        controllerinput->delta_pitch = buffer[5] << 8 | buffer[4];
        controllerinput->delta_yaw = buffer[9] << 8 | buffer[8];
        controllerinput->vra = buffer[11] << 8 | buffer[10];
        controllerinput->vrb = buffer[13] << 8 | buffer[12];
        xQueueOverwrite(commQueue, controllerinput);
    }
}
