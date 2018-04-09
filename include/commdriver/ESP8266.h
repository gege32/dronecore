/*
 * ESP8266.h
 *
 *  Created on: Apr 5, 2018
 *      Author: Horvath_Gergo
 */

#ifndef COMMDRIVER_ESP8266_H_
#define COMMDRIVER_ESP8266_H_

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include <stdbool.h>

#define ESP8266_RX_BUFFER_SIZE 64

UART_HandleTypeDef* uart_wifi;

char* esp8266_rx_buffer;

void ESP8266_initialize(UART_HandleTypeDef* huart_handle);

bool ESP8266_checksocket();

uint32_t ESP8266_readdata(uint8_t* buffer, uint32_t count);

#endif /* COMMDRIVER_ESP8266_H_ */
