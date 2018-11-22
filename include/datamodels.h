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
	float32_t roll;
	float32_t pitch;
	float32_t yaw;
	float32_t height;
}SensorData_TypeDef;

typedef struct{
    float32_t w;
    float32_t x;
    float32_t y;
    float32_t z;
}Quternion_TypeDef;

typedef struct{
    float32_t throttle;
    float32_t delta_roll;
    float32_t delta_pitch;
    float32_t delta_yaw;
}ControllerInput_TypeDef;

#define FC_SET_KP 0X01;
#define FC_SET_KI 0x02;
#define FC_SET_KD 0x04;
#define FC_CALIBRATE_ESC 0x10;
#define FC_START_MOTOR 0x20;
#define FC_STOP_MOTOR 0x40;

typedef struct{
	uint8_t messageType;
	q31_t data;
}FlightControllerMessage_TypeDef;


#endif /* DATAMODELS_H_ */
