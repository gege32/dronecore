/*
 * mpu6050_i2c_driver.c
 *
 *  Created on: Feb 4, 2018
 *      Author: gege3
 */
#include <sensordriver/mpu6050_i2c_driver.h>

void MPU6050_init(MPU6050Init_TypeDef* MPU6050_InitStruct) {
	MPU6050_Settings = MPU6050_InitStruct;

	//set clock source
	I2C_SetBits(&MPU6050_Settings->I2C_Device, MPU6050_PLL_GYRO_CLOCK_X | MPU6050_CYCLE, MPU6050_PWR_MGMT_1);

	I2C_ResetBits(&MPU6050_Settings->I2C_Device, MPU6050_SLEEP, MPU6050_PWR_MGMT_1);

	I2C_SetBits(&MPU6050_Settings->I2C_Device, MPU6050_DLPF_CFG_3, MPU6050_CONFIG);

	//set gyro resolution
	I2C_SetBits(&MPU6050_Settings->I2C_Device, MPU6050_FS_SEL_250, MPU6050_GYRO_CONFIG);

	//set accel resolution
	I2C_SetBits(&MPU6050_Settings->I2C_Device, MPU6050_FS_SEL_2G, MPU6050_ACCEL_CONFIG);

	I2C_SetBits(&MPU6050_Settings->I2C_Device, 0x00, MPU6050_PWR_MGMT_2);
}

uint16_t MPU6050_read_accel_X(){

	uint8_t highbyte = 0x00;
	I2C_BufferRead(&MPU6050_Settings->I2C_Device, &highbyte, MPU6050_ACCEL_XOUT_H);

	uint8_t lowbyte = 0x00;
	I2C_BufferRead(&MPU6050_Settings->I2C_Device, &lowbyte, MPU6050_ACCEL_XOUT_L);

	uint16_t ret = ((uint16_t)highbyte << 8) | lowbyte;

	return ret;

}

uint16_t MPU6050_read_accel_Y(){

	uint8_t highbyte = 0x00;
	I2C_BufferRead(&MPU6050_Settings->I2C_Device, &highbyte, MPU6050_ACCEL_YOUT_H);

	uint8_t lowbyte = 0x00;
	I2C_BufferRead(&MPU6050_Settings->I2C_Device, &lowbyte, MPU6050_ACCEL_YOUT_L);

	uint16_t ret = ((uint16_t)highbyte << 8) | lowbyte;

	return ret;

}

uint16_t MPU6050_read_accel_Z(){

	uint8_t highbyte = 0x00;
	I2C_BufferRead(&MPU6050_Settings->I2C_Device, &highbyte, MPU6050_ACCEL_ZOUT_H);

	uint8_t lowbyte = 0x00;
	I2C_BufferRead(&MPU6050_Settings->I2C_Device, &lowbyte, MPU6050_ACCEL_ZOUT_L);

	uint16_t ret = ((uint16_t)highbyte << 8) | lowbyte;

	return ret;

}

uint16_t MPU6050_read_gyro_X(){

	uint8_t highbyte = 0x00;
	I2C_BufferRead(&MPU6050_Settings->I2C_Device, &highbyte, MPU6050_GYRO_XOUT_H);

	uint8_t lowbyte = 0x00;
	I2C_BufferRead(&MPU6050_Settings->I2C_Device, &lowbyte, MPU6050_GYRO_XOUT_L);

	uint16_t ret = ((uint16_t)highbyte << 8) | lowbyte;

	return ret;

}

uint16_t MPU6050_read_gyro_Y(){

	uint8_t highbyte = 0x00;
	I2C_BufferRead(&MPU6050_Settings->I2C_Device, &highbyte, MPU6050_GYRO_YOUT_H);

	uint8_t lowbyte = 0x00;
	I2C_BufferRead(&MPU6050_Settings->I2C_Device, &lowbyte, MPU6050_GYRO_YOUT_L);

	uint16_t ret = ((uint16_t)highbyte << 8) | lowbyte;

	return ret;
}

uint16_t MPU6050_read_gyro_Z(){

	uint8_t highbyte = 0x00;
	I2C_BufferRead(&MPU6050_Settings->I2C_Device, &highbyte, MPU6050_GYRO_ZOUT_H);

	uint8_t lowbyte = 0x00;
	I2C_BufferRead(&MPU6050_Settings->I2C_Device, &lowbyte, MPU6050_GYRO_ZOUT_L);

	uint16_t ret = ((uint16_t)highbyte << 8) | lowbyte;

	return ret;
}

uint16_t MPU6050_read_temp(){

	uint8_t highbyte = 0x00;
	I2C_BufferRead(&MPU6050_Settings->I2C_Device, &highbyte, MPU6050_TEMP_OUT_H);

	uint8_t lowbyte = 0x00;
	I2C_BufferRead(&MPU6050_Settings->I2C_Device, &lowbyte, MPU6050_TEMP_OUT_L);

	uint16_t ret = ((uint16_t)highbyte << 8) | lowbyte;

	return ret;

}

uint8_t MPU6050_check_connection(){
	uint8_t buf = 0x00;
	I2C_BufferRead(&MPU6050_Settings->I2C_Device, &buf, MPU6050_WHO_AM_I);
	if(MPU6050_Settings->I2C_Device.address == (buf << 1)){
		return 1;
	}
	return 0;
}
