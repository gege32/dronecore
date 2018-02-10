/*
 * ESP01_driver.c
 *
 *  Created on: Feb 8, 2018
 *      Author: gege3
 */

#include <peryphdriver/ESP01_driver.h>


void ESP01_initialize(){

//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
//    GPIO_InitTypeDef initA;
//    initA.GPIO_Mode = GPIO_Mode_AF_OD;
//    initA.GPIO_Speed = GPIO_Speed_2MHz;
//    initA.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
//    GPIO_Init(GPIOA, &initA);
//
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
//
//    USART_InitTypeDef* USART_InitStruct;
//    USART_StructInit(USART_InitStruct);
//    USART_Init(USART1, USART_InitStruct);

//    ESP01_send_command(USART1, ESP01_AT_RESTART, sizeof(ESP01_AT_RESTART));

}

void ESP01_connect(ESP01Connection_InitTpeDef* ESP01_Connection){

}

void ESP01_send_command(USART_TypeDef* USARTx, const uint8_t* data, uint8_t lenght){

//    while(lenght){
//
//        USART_GetITStatus(USARTx, USART_FLAG_TC);
//        USART_SendData(USARTx, data++);
//
//        lenght--;
//    }

}

void ESP01_read_data(USART_TypeDef* USARTx, uint8_t* data){

    USART_ReceiveData(USARTx);

}
