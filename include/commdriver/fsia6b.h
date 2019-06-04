#ifndef __FSIA6B_H
#define __FSIA6B_H

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "semihosting/Trace.h"
#include "datamodels.h"

extern UART_HandleTypeDef huart2;

#define PACKAGE_LENGHT 32

uint8_t *buffer;

QueueHandle_t *commQueue;

void fsia6b_init(QueueHandle_t *queueHandle);

void fsia6b_getRemoteControlData(ControllerInput_TypeDef controllerInput);

#endif __FSIA6B_H
