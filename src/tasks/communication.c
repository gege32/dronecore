/*
 * communication.c
 *
 *  Created on: Feb 15, 2018
 *      Author: gege3
 */

#include "tasks/communication.h"

void CommunicationTask(void const* argument) {
    osDelay(1000);
    trace_puts("comm task started");
    fsia6b_init();
    for (;;) {
        osDelay(150);
        fsia6b_getRemoteControlData();
    }

}

