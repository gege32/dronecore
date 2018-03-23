/*
 * bmp180.h
 *
 *  Created on: Mar 23, 2018
 *      Author: gehorvath
 */

#ifndef SENSORDRIVER_BMP180_H_
#define SENSORDRIVER_BMP180_H_

#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <cmsis_os.h>

#define BMP180_ADDRESS 0xEF
#define BMP180_CHIP_ID 0x55

I2C_HandleTypeDef * hi2c;

typedef struct {
	short AC1;
	short AC2;
	short AC3;
	unsigned short AC4;
	unsigned short AC5;
	unsigned short AC6;
	short B1;
	short B2;
	short MB;
	short MC;
	short MD;

	long UT;
	short oss;
	long UP;

	long X1;
	long X2;
	long B5;
	long T;

	long B6;
	long X3;
	long B3;
	unsigned long B4;
	unsigned long B7;
	long p;

} CalibrationData;

typedef enum {
	BMP180_MODE_ULTRA_LOW_POWER = 0,
	BMP180_MODE_STANDARD = 1,
	BMP180_MODE_HIGH_RESOLUTION = 2,
	BMP180_MODE_ULTRA_HIGH_RESOLUTION = 3,
} BMP180_Mode;

bool bmp180_check_presence();
void bmp180_get_calibration_data(CalibrationData* data);
void bmp180_get_uncompensated_temperature(CalibrationData* data);
void bmp180_get_uncompensated_pressure(CalibrationData* data);
void bmp180_calculate_true_temperature(CalibrationData* data);
void bmp180_calculate_true_pressure(CalibrationData* data);
float bmp180_get_absolute_altitude(CalibrationData* data);
void bmp180_initialize(I2C_HandleTypeDef * hi2c_def);

#endif /* SENSORDRIVER_BMP180_H_ */
