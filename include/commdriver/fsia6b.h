#ifndef __FSIA6B_H
#define __FSIA6B_H

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "semihosting/Trace.h"
#include "datamodels.h"

extern UART_HandleTypeDef huart2;

#define PACKAGE_LENGHT 32

uint8_t buffer[64];

extern QueueHandle_t communicationToFlightControllerDataQueue;

ControllerInput_TypeDef* controllerInput;

SemaphoreHandle_t controllerDataReady;

void fsia6b_init();

void fsia6b_getRemoteControlData();

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#endif __FSIA6B_H
