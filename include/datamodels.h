/*
 * datamodels.h
 *
 *  Created on: Feb 23, 2018
 *      Author: gehorvath
 */

#ifndef DATAMODELS_H_
#define DATAMODELS_H_

#include "arm_math.h"

typedef struct{
	q31_t roll;
	q31_t pitch;
	q31_t yaw;
	q31_t height;
}SensorData_TypeDef;


#define FC_SET_KP 0X01;
#define FC_SET_KI 0x02;
#define FC_SET_KD 0x04;
#define FC_CALIBRATE_ESC 0x10;
#define FC_START_MOTOR 0x20;
#define FC_STOP_MOTOR 0x40;

typedef struct{
	uint8_t messageType;
	q31_t data;
}FlightControllerInit_TypeDef;


#endif /* DATAMODELS_H_ */
