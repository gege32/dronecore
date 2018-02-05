/*
 * i2c_driver.h
 *
 *  Created on: Feb 5, 2018
 *      Author: Horvath_Gergo
 */

#ifndef I2C_DRIVER_H_
#define I2C_DRIVER_H_

#include "stm32f10x.h"
#include "stm32f10x_i2c.h"

typedef struct{

}I2CInit_TypeDef;

typedef struct{
	I2CInit_TypeDef I2CSettings;
	uint8_t address;
	I2C_TypeDef* I2Cx;
}I2CDevice_TypeDef;

void I2C_Initialize(I2CInit_TypeDef* I2CSettings);
void I2C_ByteWrite(I2CDevice_TypeDef* device, u8* pBuffer, u8 writeAddr);
void I2C_BufferRead(I2CDevice_TypeDef* device, u8* pBuffer, u8 readAddr);
void I2C_SetBits(I2CDevice_TypeDef* device, u8 data, u8 writeAddr);
void I2C_ResetBits(I2CDevice_TypeDef* device, u8 data, u8 writeAddr);

#endif /* I2C_DRIVER_H_ */
