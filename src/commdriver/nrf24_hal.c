/*
 * nrf24_hal.c
 *
 *  Created on: Oct 20, 2018
 *      Author: gege3
 */


#include "commdriver/nrf24_hal.h"

// Low level SPI transmit/receive function (hardware depended)
// input:
//   data - value to transmit via SPI
// return: value received from SPI
uint8_t nRF24_LL_RW(uint8_t data) {

    HAL_SPI_TransmitReceive(spi_handle, &data, &spi_input_buffer, 1, 10);

    return spi_input_buffer;

}

void nRF24_HAL_Init(SPI_HandleTypeDef* hspi2){
    spi_handle = hspi2;
}
