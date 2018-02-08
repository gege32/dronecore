/*
 * ESP01_driver.h
 *
 *  Created on: Feb 8, 2018
 *      Author: gege3
 */

#ifndef PERYPHDRIVER_ESP01_DRIVER_H_
#define PERYPHDRIVER_ESP01_DRIVER_H_

#include <stm32f10x_usart.h>

#define ESP01_AT_RESTART "AT+RST"
#define ESP01_AT_MODE_STA "AT+CWMODE=1"
#define ESP01_AT_MODE_AP "AT+CWMODE=2"
#define ESP01_AT_MODE_BOTH "AT+CWMODE=3"
#define ESP01_AT_JOIN_AP "AT+CWJAP="


typedef struct{

}ESP01Connection_InitTpeDef;

bool ESP01_initialize();
bool ESP01_connect(ESP01Connection_InitTpeDef* ESP01_Connection);
void ESP01_send_command(USART_TypeDef* USARTx, uint9_t, uint8_t lenght);


#endif /* PERYPHDRIVER_ESP01_DRIVER_H_ */
