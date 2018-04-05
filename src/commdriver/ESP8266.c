/*
 * ESP8266.c
 *
 *  Created on: Apr 5, 2018
 *      Author: Horvath_Gergo
 */

#include <commdriver/ESP8266.h>

void ESP8266_initialize(UART_HandleTypeDef* huart_handle){

	esp8266_rx_buffer = pvPortMalloc(sizeof(char) * ESP8266_RX_BUFFER_SIZE);

	//enable ap mode
	HAL_UART_Transmit(uart_wifi,"AT+CWMODE=1\r\n", 13, 10);

	//enable rf power
	HAL_UART_Transmit(uart_wifi,"AT+RFPOWER=82\r\n", 15, 10);

	//enable network
	HAL_UART_Transmit(uart_wifi,"AT+CWSAP=\"TEST\",\"test1234\",5,4,1,0\r\n", 36, 10);

	//enable single connection
	HAL_UART_Transmit(uart_wifi,"AT+CIPMUX=1\r\n", 14, 10);

	//enable tcp server
	HAL_UART_Transmit(uart_wifi,"AT+CIPSERVER=1,22000\r\n", 22, 10);

	//get status
	HAL_UART_Transmit(uart_wifi,"AT+CIPSTATUS\r\n", 14, 10);


	HAL_UART_Receive(uart_wifi, esp8266_rx_buffer, 32, 10);

	trace_puts(esp8266_rx_buffer);

}
