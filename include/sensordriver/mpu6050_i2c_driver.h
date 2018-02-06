/*
 * mpu6050_i2c_driver.h
 *
 *  Created on: Feb 4, 2018
 *      Author: gege3
 */

#ifndef MPU6050_I2C_DRIVER_H_
#define MPU6050_I2C_DRIVER_H_

#include <peryphdriver/i2c_driver.h>

typedef struct{
	uint8_t MPU6050_ClockSource;
	uint8_t MPU6050_GyroscopeRange;
	uint8_t MPU6050_AccelerometerRange;
	I2CDevice_TypeDef I2C_Device;
}MPU6050Init_TypeDef;

//Address
#define MPU6050_DEFAULT_ADDRESS 0xd0;

//Config bitmaps

//Ext sync+DLPF config
#define MPU6050_DLPF_CFG_0 ((uint8_t)0x00)
#define MPU6050_DLPF_CFG_1 ((uint8_t)0x01)
#define MPU6050_DLPF_CFG_2 ((uint8_t)0x02)
#define MPU6050_DLPF_CFG_3 ((uint8_t)0x03)
#define MPU6050_DLPF_CFG_4 ((uint8_t)0x04)
#define MPU6050_DLPF_CFG_5 ((uint8_t)0x05)
#define MPU6050_DLPF_CFG_6 ((uint8_t)0x06)
#define MPU6050_DLPF_CFG_7 ((uint8_t)0x07)


//Gyroscope selftest
#define MPU6050_XG_ST ((uint8_t)0x80)
#define MPU6050_YG_ST ((uint8_t)0x40)
#define MPU6050_ZG_ST ((uint8_t)0x20)
//Gyro range
#define MPU6050_FS_SEL_250 ((uint8_t)0x00)
#define MPU6050_FS_SEL_500 ((uint8_t)0x08)
#define MPU6050_FS_SEL_1000 ((uint8_t)0x10)
#define MPU6050_FS_SEL_2000 ((uint8_t)0x18)

//Accel selftest
#define MPU6050_XA_ST ((uint8_t)0x80)
#define MPU6050_YA_ST ((uint8_t)0x40)
#define MPU6050_ZA_ST ((uint8_t)0x20)
//Accel range
#define MPU6050_FS_SEL_2G ((uint8_t)0x00)
#define MPU6050_FS_SEL_4G ((uint8_t)0x08)
#define MPU6050_FS_SEL_8G ((uint8_t)0x10)
#define MPU6050_FS_SEL_16G ((uint8_t)0x18)

//PWR_MGMT_1
#define MPU6050_INTERNAL_CLOCK_8MHz ((uint8_t)0x00)
#define MPU6050_PLL_GYRO_CLOCK_X ((uint8_t)0x01)
#define MPU6050_PLL_GYRO_CLOCK_Y ((uint8_t)0x02)
#define MPU6050_PLL_GYRO_CLOCK_Z ((uint8_t)0x03)
#define MPU6050_PLL_EXTERNAL_32768KHZ_CLOCK ((uint8_t)0x04)
#define MPU6050_PLL_EXTERNAL_192KHZ_CLOCK ((uint8_t)0x05)
#define MPU6050_PLL_STOP_CLOCK ((uint8_t)0x07)

#define MPU6050_TEMP_ENABLE ((uint8_t)0x08)

#define MPU6050_CYCLE ((uint8_t)0x20)
#define MPU6050_SLEEP ((uint8_t)0x40)
#define MPU6050_RESET ((uint8_t)0x80)


//Memory definition

#define MPU6050_CONFIG ((uint8_t)0x1B)

#define MPU6050_GYRO_CONFIG ((uint8_t)0x1B)
#define MPU6050_ACCEL_CONFIG ((uint8_t)0x1C)

#define MPU6050_ACCEL_XOUT_H ((uint8_t)0x3B)
#define MPU6050_ACCEL_XOUT_L ((uint8_t)0x3C)

#define MPU6050_ACCEL_YOUT_H ((uint8_t)0x3D)
#define MPU6050_ACCEL_YOUT_L ((uint8_t)0x3E)

#define MPU6050_ACCEL_ZOUT_H ((uint8_t)0x3F)
#define MPU6050_ACCEL_ZOUT_L ((uint8_t)0x40)

#define MPU6050_TEMP_OUT_H ((uint8_t)0x41)
#define MPU6050_TEMP_OUT_L ((uint8_t)0x42)

#define MPU6050_GYRO_XOUT_H ((uint8_t)0x43)
#define MPU6050_GYRO_XOUT_L ((uint8_t)0x44)

#define MPU6050_GYRO_YOUT_H ((uint8_t)0x45)
#define MPU6050_GYRO_YOUT_L ((uint8_t)0x46)

#define MPU6050_GYRO_ZOUT_H ((uint8_t)0x47)
#define MPU6050_GYRO_ZOUT_L ((uint8_t)0x48)

#define MPU6050_PWR_MGMT_1 ((uint8_t)0x6B)
#define MPU6050_PWR_MGMT_2 ((uint8_t)0x6C)

#define MPU6050_WHO_AM_I ((uint8_t)0x75)

MPU6050Init_TypeDef* MPU6050_Settings;

void MPU6050_init(MPU6050Init_TypeDef* MPU6050InitStruct);

uint16_t MPU6050_read_accel_X();

uint16_t MPU6050_read_accel_Y();

uint16_t MPU6050_read_accel_Z();

uint16_t MPU6050_read_gyro_X();

uint16_t MPU6050_read_gyro_Y();

uint16_t MPU6050_read_gyro_Z();

uint8_t MPU6050_check_connection();

uint16_t MPU6050_read_temp();

#endif /* MPU6050_I2C_DRIVER_H_ */
