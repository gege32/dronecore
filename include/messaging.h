/*
 * messaging.h
 *
 *  Created on: Feb 1, 2018
 *      Author: Horvath_Gergo
 */

#ifndef MESSAGING_H_
#define MESSAGING_H_

#include "stm32f1xx.h"

typedef enum{
	MSG_Initial_Handshake = 0x01,
	MSG_Init_Devices = 0x02,
	MSG_Start_Idle = 0x03,
	MSG_Do_Roll_X = 0x10,
	MSG_Do_Pitch_Y = 0x20,
	MSG_Do_Yaw_Z = 0x30,
	MSG_Do_Raise = 0x40,
	MSG_Do_Lower = 0x50,
	MSG_Halt = 0x90
}MSGType_TypeDef;

/*
 * Messages are fixed length, so they can be cast to a message without parsing. All messages can have max 1 byte of information.
 */
typedef struct{
	MSGType_TypeDef MSGType;

	uint8_t MSGData;

}MSG_TypeDef;

#endif /* MESSAGING_H_ */
