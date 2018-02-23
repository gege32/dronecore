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
	q31_t speed;
}SensorData;



#endif /* DATAMODELS_H_ */
