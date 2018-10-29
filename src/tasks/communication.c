/*
 * communication.c
 *
 *  Created on: Feb 15, 2018
 *      Author: gege3
 */

#include "tasks/communication.h"

void CommunicationTask(void const* argument) {

    trace_puts("sensor task started");

    nRF24_HAL_Init((SPI_HandleTypeDef*) argument);

    ControllerInput_TypeDef* incomming = pvPortMalloc(sizeof(ControllerInput_TypeDef));

    osDelay(100);
    // Configure RX PIPE#1
    static const uint8_t nRF24_ADDR[] = { '1', 'E', 'D', 'O', 'N' };
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
    if (test == 0) {
        trace_puts("nrf0");
    } else {
        trace_puts("nrf1");
    }

    for (;;) {
        if (nRF24_GetStatus_RXFIFO() != nRF24_STATUS_RXFIFO_EMPTY) {
            // Get a payload from the transceiver
            pipe = nRF24_ReadPayload(nRF24_payload, &payload_length);

            // Clear all pending IRQ flags
            nRF24_ClearIRQFlags();

            if (nRF24_payload[0] == '<') {
                if (nRF24_payload[1] == 0x01) {

                    incomming->throttle = (((uint32_t) nRF24_payload[2] << 24) | ((uint32_t) nRF24_payload[3] << 16) | ((uint32_t) nRF24_payload[4] << 8) | nRF24_payload[5]);
//                  incomming->delta_roll = (((uint32_t)nRF24_payload[4] << 24) | ((uint32_t)nRF24_payload[5] << 16) | ((uint32_t)nRF24_payload[6] << 8) | nRF24_payload[7]);
//                  incomming->delta_pitch = (((uint32_t)nRF24_payload[8] << 24) | ((uint32_t)nRF24_payload[9] << 16) | ((uint32_t)nRF24_payload[10] << 8) | nRF24_payload[11]);
//                  incomming->delta_yaw = (((uint32_t)nRF24_payload[12] << 24) | ((uint32_t)nRF24_payload[13] << 16) | ((uint32_t)nRF24_payload[14] << 8) | nRF24_payload[15]);
                    incomming->delta_roll = 0;
                    incomming->delta_pitch = 0;
                    incomming->delta_yaw = 0;

                    incomming->throttle += 1000;

                    xQueueSend(communicationToFlightControllerDataQueue, incomming, 1);
                }

            }
        }
        osDelay(100);
    }

}

