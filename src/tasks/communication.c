/*
 * communication.c
 *
 *  Created on: Feb 15, 2018
 *      Author: gege3
 */

#include "tasks/communication.h"

void CommunicationTask(void const* argument) {

    trace_puts("comm task started");

    uint8_t status;
    uint8_t nRF24_payload[32];
    uint8_t payload_length = 32;
    nRF24_RXResult pipe;

    char szoveg[40];

    for (;;) {
        if (nRF24_GetStatus_RXFIFO() != nRF24_STATUS_RXFIFO_EMPTY) {
            // Get a payload from the transceiver
            pipe = nRF24_ReadPayload(nRF24_payload, &payload_length);

            // Clear all pending IRQ flags
            nRF24_ClearIRQFlags();

            if (nRF24_payload[0] == '<' && nRF24_payload[18] == '>') {
                if (nRF24_payload[1] == 0x01) {

                    incomming->throttle = (float32_t) (((uint32_t) nRF24_payload[2] << 8) | ((uint32_t) nRF24_payload[3])) +  ((((uint32_t) nRF24_payload[4] << 8) | (uint32_t)nRF24_payload[5]) / 1000.0f);
                    incomming->delta_roll = (float32_t) (((uint32_t) nRF24_payload[6] << 8) | ((uint32_t) nRF24_payload[7])) +  ((((uint32_t) nRF24_payload[8] << 8) | (uint32_t)nRF24_payload[9]) / 1000.0f);
                    incomming->delta_pitch = (float32_t) (((uint32_t) nRF24_payload[10] << 8) | ((uint32_t) nRF24_payload[11])) +  ((((uint32_t) nRF24_payload[12] << 8) | (uint32_t)nRF24_payload[13]) / 1000.0f);
//                  incomming->delta_yaw = (((uint32_t)nRF24_payload[12] << 24) | ((uint32_t)nRF24_payload[13] << 16) | ((uint32_t)nRF24_payload[14] << 8) | nRF24_payload[15]);
                    xQueueOverwrite(communicationToFlightControllerDataQueue, incomming);
                } else if(nRF24_payload[1] == 0x02){
                    incpid->p = (float32_t) (((uint32_t) nRF24_payload[2] << 8) | ((uint32_t) nRF24_payload[3])) +  ((((uint32_t) nRF24_payload[4] << 8) | (uint32_t)nRF24_payload[5]) / 1000.0f);
                    incpid->i = (float32_t) (((uint32_t) nRF24_payload[6] << 8) | ((uint32_t) nRF24_payload[7])) +  ((((uint32_t) nRF24_payload[8] << 8) | (uint32_t)nRF24_payload[9]) / 1000.0f);
                    incpid->d = (float32_t) (((uint32_t) nRF24_payload[10] << 8) | ((uint32_t) nRF24_payload[11])) +  ((((uint32_t) nRF24_payload[12] << 8) | (uint32_t)nRF24_payload[13]) / 1000.0f);
                    xQueueOverwrite(PIDtuningDataQueue, incpid);
                } else{
                    incomming->throttle = 0.0f;
                    incomming->delta_roll = 0.0f;
                    incomming->delta_pitch = 0.0f;
                    incomming->delta_yaw = 0.0f;
                    xQueueOverwrite(communicationToFlightControllerDataQueue, incomming);
                }

            } else {
                incomming->throttle = 0.0f;
                incomming->delta_roll = 0.0f;
                incomming->delta_pitch = 0.0f;
                incomming->delta_yaw = 0.0f;
                xQueueOverwrite(communicationToFlightControllerDataQueue, incomming);
            }
        }
        osDelay(150);
    }

}

