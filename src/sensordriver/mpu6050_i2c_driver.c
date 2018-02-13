/*
 * mpu6050_i2c_driver.c
 *
 *  Created on: Feb 4, 2018
 *      Author: gege3
 */
#include <sensordriver/mpu6050_i2c_driver.h>

void MPU6050_SetBits(I2C_HandleTypeDef* hi2c1, uint8_t* registerAddress, uint8_t* data);
void MPU6050_ResetBits(I2C_HandleTypeDef* hi2c1, uint8_t* registerAddress, uint8_t* data);

void MPU6050_init(MPU6050Init_TypeDef* MPU6050_InitStruct, I2C_HandleTypeDef* hi2c1) {
	MPU6050_Settings = MPU6050_InitStruct;
	MPU6050_Settings->hi2c1 = hi2c1;

	HAL_I2C_Init(hi2c1);

	//set clock source
	MPU6050_SetBits(hi2c1, MPU6050_PLL_GYRO_CLOCK_X | MPU6050_CYCLE, MPU6050_PWR_MGMT_1);

	MPU6050_ResetBits(hi2c1, MPU6050_SLEEP, MPU6050_PWR_MGMT_1);

	MPU6050_SetBits(hi2c1, MPU6050_DLPF_CFG_3, MPU6050_CONFIG);

	//set gyro resolution
	MPU6050_SetBits(hi2c1, MPU6050_FS_SEL_250, MPU6050_GYRO_CONFIG);

	//set accel resolution
	MPU6050_SetBits(hi2c1, MPU6050_FS_SEL_2G, MPU6050_ACCEL_CONFIG);

	MPU6050_SetBits(hi2c1, 0x00, MPU6050_PWR_MGMT_2);
}

uint16_t MPU6050_read_accel_X(){

	uint8_t highbyte = 0x00;
	HAL_I2C_Master_Transmit(&MPU6050_Settings->hi2c1, MPU6050_Settings->MPU6050_Adress, MPU6050_ACCEL_XOUT_H, 1, 10);
	HAL_I2C_Master_Receive(&MPU6050_Settings->hi2c1, MPU6050_Settings->MPU6050_Adress, &highbyte, 1, 10);

	uint8_t lowbyte = 0x00;
	HAL_I2C_Master_Transmit(&MPU6050_Settings->hi2c1, MPU6050_Settings->MPU6050_Adress, MPU6050_ACCEL_XOUT_L, 1, 10);
	HAL_I2C_Master_Receive(&MPU6050_Settings->hi2c1, MPU6050_Settings->MPU6050_Adress, &highbyte, 1, 10);

	uint16_t ret = ((uint16_t)highbyte << 8) | lowbyte;

	return ret;

}

uint16_t MPU6050_read_accel_Y(){

//	uint8_t highbyte = 0x00;
//	I2C_BufferRead(&MPU6050_Settings->I2C_Device, &highbyte, MPU6050_ACCEL_YOUT_H);
//
//	uint8_t lowbyte = 0x00;
//	I2C_BufferRead(&MPU6050_Settings->I2C_Device, &lowbyte, MPU6050_ACCEL_YOUT_L);
//
//	uint16_t ret = ((uint16_t)highbyte << 8) | lowbyte;
//
//	return ret;

}

uint16_t MPU6050_read_accel_Z(){

//	uint8_t highbyte = 0x00;
//	I2C_BufferRead(&MPU6050_Settings->I2C_Device, &highbyte, MPU6050_ACCEL_ZOUT_H);
//
//	uint8_t lowbyte = 0x00;
//	I2C_BufferRead(&MPU6050_Settings->I2C_Device, &lowbyte, MPU6050_ACCEL_ZOUT_L);
//
//	uint16_t ret = ((uint16_t)highbyte << 8) | lowbyte;
//
//	return ret;

}

uint16_t MPU6050_read_gyro_X(){

//	uint8_t highbyte = 0x00;
//	I2C_BufferRead(&MPU6050_Settings->I2C_Device, &highbyte, MPU6050_GYRO_XOUT_H);
//
//	uint8_t lowbyte = 0x00;
//	I2C_BufferRead(&MPU6050_Settings->I2C_Device, &lowbyte, MPU6050_GYRO_XOUT_L);
//
//	uint16_t ret = ((uint16_t)highbyte << 8) | lowbyte;
//
//	return ret;

}

uint16_t MPU6050_read_gyro_Y(){

//	uint8_t highbyte = 0x00;
//	I2C_BufferRead(&MPU6050_Settings->I2C_Device, &highbyte, MPU6050_GYRO_YOUT_H);
//
//	uint8_t lowbyte = 0x00;
//	I2C_BufferRead(&MPU6050_Settings->I2C_Device, &lowbyte, MPU6050_GYRO_YOUT_L);
//
//	uint16_t ret = ((uint16_t)highbyte << 8) | lowbyte;
//
//	return ret;
}

uint16_t MPU6050_read_gyro_Z(){

//	uint8_t highbyte = 0x00;
//	I2C_BufferRead(&MPU6050_Settings->I2C_Device, &highbyte, MPU6050_GYRO_ZOUT_H);
//
//	uint8_t lowbyte = 0x00;
//	I2C_BufferRead(&MPU6050_Settings->I2C_Device, &lowbyte, MPU6050_GYRO_ZOUT_L);
//
//	uint16_t ret = ((uint16_t)highbyte << 8) | lowbyte;
//
//	return ret;
}

uint16_t MPU6050_read_temp(){

//	uint8_t highbyte = 0x00;
//	I2C_BufferRead(&MPU6050_Settings->I2C_Device, &highbyte, MPU6050_TEMP_OUT_H);
//
//	uint8_t lowbyte = 0x00;
//	I2C_BufferRead(&MPU6050_Settings->I2C_Device, &lowbyte, MPU6050_TEMP_OUT_L);
//
//	uint16_t ret = ((uint16_t)highbyte << 8) | lowbyte;
//
//	return ret;

}

uint8_t MPU6050_check_connection(){
//	uint8_t buf = 0x00;
//	I2C_BufferRead(&MPU6050_Settings->I2C_Device, &buf, MPU6050_WHO_AM_I);
//	if(MPU6050_Settings->I2C_Device.address == (buf << 1)){
//		return 1;
//	}
//	return 0;
}

void MPU6050_SetBits(I2C_HandleTypeDef* hi2c1, uint8_t* registerAddress, uint8_t* data){
	uint8_t buf;

	HAL_I2C_Master_Transmit(hi2c1, MPU6050_Settings->MPU6050_Adress, registerAddress, 1, 10);
	HAL_I2C_Master_Receive(hi2c1, MPU6050_Settings->MPU6050_Adress, &buf, 1, 10);

	buf |= *data;

	HAL_I2C_Master_Transmit(hi2c1, MPU6050_Settings->MPU6050_Adress, &buf, 1, 10);

}
void MPU6050_ResetBits(I2C_HandleTypeDef* hi2c1, uint8_t* registerAddress, uint8_t* data){
	uint8_t buf;

	HAL_I2C_Master_Transmit(hi2c1, MPU6050_Settings->MPU6050_Adress, registerAddress, 1, 10);
	HAL_I2C_Master_Receive(hi2c1, MPU6050_Settings->MPU6050_Adress, &buf, 1, 10);

	buf &= ~*data;

	HAL_I2C_Master_Transmit(hi2c1, MPU6050_Settings->MPU6050_Adress, &buf, 1, 10);

}
