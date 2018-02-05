/*
 * mpu6050_i2c_driver.h
 *
 *  Created on: Feb 4, 2018
 *      Author: gege3
 */

#ifndef MPU6050_I2C_DRIVER_H_
#define MPU6050_I2C_DRIVER_H_

#include "stm32f10x.h"

typedef struct{

}MPU6050Init_TypeDef;

//Address
#define ADDRESS 0xd0;

//Config bitmaps

//Gyroscope selftest
#define XG_ST ((uint8_t)0x80)
#define YG_ST ((uint8_t)0x40)
#define ZG_ST ((uint8_t)0x20)
//Gyro range
#define FS_SEL_250 ((uint8_t)0x00)
#define FS_SEL_500 ((uint8_t)0x08)
#define FS_SEL_1000 ((uint8_t)0x10)
#define FS_SEL_2000 ((uint8_t)0x18)

//Accel selftest
#define XA_ST ((uint8_t)0x80)
#define YA_ST ((uint8_t)0x40)
#define ZA_ST ((uint8_t)0x20)
//Accel range
#define FS_SEL_2G ((uint8_t)0x00)
#define FS_SEL_4G ((uint8_t)0x08)
#define FS_SEL_8G ((uint8_t)0x10)
#define FS_SEL_16G ((uint8_t)0x18)

//PWR_MGMT_1
#define INTERNAL_CLOCK_8MHz ((uint8_t)0x00)
//... TODO: other clock sources
#define TEMP_ENABLE ((uint8_t)0x08)

#define CYCLE ((uint8_t)0x20)
#define SLEEP ((uint8_t)0x40)
#define RESET ((uint8_t)0x80)


//Memory definition

#define GYRO_CONFIG ((uint8_t)0x1B)
#define ACCEL_CONFIG ((uint8_t)0x1C)

#define ACCEL_XOUT_H ((uint8_t)0x3B)
#define ACCEL_XOUT_L ((uint8_t)0x3C)

#define ACCEL_YOUT_H ((uint8_t)0x3D)
#define ACCEL_YOUT_L ((uint8_t)0x3E)

#define ACCEL_ZOUT_H ((uint8_t)0x3F)
#define ACCEL_ZOUT_L ((uint8_t)0x40)

#define TEMP_OUT_H ((uint8_t)0x41)
#define TEMP_OUT_L ((uint8_t)0x42)

#define GYRO_XOUT_H ((uint8_t)0x43)
#define GYRO_XOUT_L ((uint8_t)0x44)

#define GYRO_YOUT_H ((uint8_t)0x45)
#define GYRO_YOUT_L ((uint8_t)0x46)

#define GYRO_ZOUT_H ((uint8_t)0x47)
#define GYRO_ZOUT_L ((uint8_t)0x48)

#define PWR_MGMT_1 ((uint8_t)0x6B)


void init(MPU6050Init_TypeDef* MPU6050InitStruct);

uint16_t read_accel_X();

uint16_t read_accel_Y();

uint16_t read_accel_Z();

uint16_t read_gyro_X();

uint16_t read_gyro_Y();

uint16_t read_gyro_Z();

uint16_t read_temp();

#endif /* MPU6050_I2C_DRIVER_H_ */
