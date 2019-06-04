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
    uint32_t throttle;
    uint32_t delta_roll;
    uint32_t delta_pitch;
    uint32_t delta_yaw;
    uint32_t vra;
    uint32_t vrb;
}ControllerInput_TypeDef;

typedef struct{
    float32_t p;
    float32_t i;
    float32_t d;
}PIDtuning_TypeDef;

typedef struct{
	uint8_t messageType;
	q31_t data;
}FlightControllerMessage_TypeDef;


#endif /* DATAMODELS_H_ */
