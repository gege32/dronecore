#ifndef __NRF24_HAL_H
#define __NRF24_HAL_H


// Hardware abstraction layer for NRF24L01+ transceiver (hardware depended functions)
// GPIO pins definition
// GPIO pins initialization and control functions
// SPI transmit functions


#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "semihosting/Trace.h"

SPI_HandleTypeDef* spi_handle;


// SPI port peripheral
#define nRF24_SPI_PORT             SPI2

// nRF24 GPIO peripherals
#define nRF24_GPIO_PERIPHERALS     (RCC_APB2ENR_IOPBEN)

// CE (chip enable) pin (PB11)
#define nRF24_CE_PORT              NRF_CE_PIN_GPIO_Port
#define nRF24_CE_PIN               NRF_CE_PIN_Pin
#define nRF24_CE_L()               GPIO_ResetBits(nRF24_CE_PORT, nRF24_CE_PIN)
#define nRF24_CE_H()               GPIO_SetBits(nRF24_CE_PORT, nRF24_CE_PIN)

// CSN (chip select negative) pin (PB12)
#define nRF24_CSN_PORT             nRF_CSN_PIN_GPIO_Port
#define nRF24_CSN_PIN              NRF_CE_PIN_Pin
#define nRF24_CSN_L()              GPIO_ResetBits(nRF24_CSN_PORT, nRF24_CSN_PIN)
#define nRF24_CSN_H()              GPIO_SetBits(nRF24_CSN_PORT, nRF24_CSN_PIN)

// IRQ pin (PB10)
#define nRF24_IRQ_PORT             GPIOB
#define nRF24_IRQ_PIN              GPIO_Pin_10


// Function prototypes
void nRF24_HAL_Init(SPI_HandleTypeDef* hspi2);
uint8_t nRF24_LL_RW(uint8_t data);

#endif // __NRF24_HAL_H
