/*
 * communication.c
 *
 *  Created on: Feb 15, 2018
 *      Author: gege3
 */

#include "tasks/communication.h"

void CommunicationTask(void const* argument){

    trace_puts("sensor task started");

    nRF24_HAL_Init((SPI_HandleTypeDef*)argument);

    osDelay(100);
    // Configure RX PIPE#1
    static const uint8_t nRF24_ADDR[] = { '1','E','D','O','N' };
    nRF24_SetRFChannel(90); // set RF channel to 2490MHz
    nRF24_SetDataRate(nRF24_DR_2Mbps); // 2Mbit/s data rate
    nRF24_SetCRCScheme(nRF24_CRC_1byte); // 1-byte CRC scheme
    nRF24_SetAddrWidth(5); // address width is 5 bytes
    nRF24_SetAddr(nRF24_PIPE1, nRF24_ADDR); // program pipe address
    nRF24_SetRXPipe(nRF24_PIPE1, nRF24_AA_ON, 32); // enable RX pipe#1 with Auto-ACK: enabled, payload length: 10 bytes
    nRF24_SetTXPower(nRF24_TXPWR_0dBm); // configure TX power for Auto-ACK, good choice - same power level as on transmitter
    nRF24_SetOperationalMode(nRF24_MODE_RX); // switch transceiver to the RX mode
    nRF24_SetPowerMode(nRF24_PWR_UP); // wake-up transceiver (in case if it sleeping)
    // Put the transceiver to the RX mode

    osDelay(100);
    nRF24_CE_H();

    uint8_t status;
    uint8_t nRF24_payload[32];
    uint8_t payload_length = 32;
    nRF24_RXResult pipe;

    uint8_t test = nRF24_Check();
    if(test == 0){
        trace_puts("nrf0");
    }else{
        trace_puts("nrf1");
    }

    for(;;){
        if (nRF24_GetStatus_RXFIFO() != nRF24_STATUS_RXFIFO_EMPTY) {
            // Get a payload from the transceiver
            pipe = nRF24_ReadPayload(nRF24_payload, &payload_length);

            // Clear all pending IRQ flags
            nRF24_ClearIRQFlags();

            // Print a payload contents to trace
            trace_puts(nRF24_payload);
        }
        osDelay(500);
    }

}
