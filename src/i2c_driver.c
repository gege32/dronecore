/*
 * i2c_driver.c
 *
 *  Created on: Feb 5, 2018
 *      Author: Horvath_Gergo
 */
#include "i2c_driver.h"

/**
 * @brief  Initializes the I2C peripheral used to drive the MPU6050
 * @param  None
 * @return None
 */
void device.GPIOx_Init(I2CInit_TypeDef I2CSettings)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitTypeDef initB;
	initB.GPIO_Mode = GPIO_Mode_AF_OD;
	initB.GPIO_Speed = GPIO_Speed_2MHz;
	initB.GPIO_Pin = GPIO_Pin_All;
	GPIO_Init(GPIOB, &initB);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_device.GPIOx, ENABLE);
	I2C_InitTypeDef i2cdef;
	I2C_StructInit(&i2cdef);
	i2cdef.I2C_ClockSpeed = 100000;

	I2C_Init(device.GPIOx, &i2cdef);
	I2C_Cmd(device.GPIOx, ENABLE);

}

/**
 * @brief  Writes one byte to the  MPU6050.
 * @param  slaveAddr : slave address MPU6050_DEFAULT_ADDRESS
 * @param  pBuffer : pointer to the buffer  containing the data to be written to the MPU6050.
 * @param  writeAddr : address of the register in which the data will be written
 * @return None
 */
void I2C_ByteWrite(I2CDevice_TypeDef device, u8* pBuffer, u8 writeAddr)
{
    // ENTR_CRT_SECTION();

    /* Send START condition */
    I2C_GenerateSTART(device.GPIOx, ENABLE);

    /* Test on EV5 and clear it */
    while (!I2C_CheckEvent(device.GPIOx, I2C_EVENT_MASTER_MODE_SELECT));

    /* Send MPU6050 address for write */
    I2C_Send7bitAddress(device.GPIOx, device.address, I2C_Direction_Transmitter);

    /* Test on EV6 and clear it */
    while (!I2C_CheckEvent(device.GPIOx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    /* Send the MPU6050's internal address to write to */
    I2C_SendData(device.GPIOx, writeAddr);

    /* Test on EV8 and clear it */
    while (!I2C_CheckEvent(device.GPIOx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    /* Send the byte to be written */
    I2C_SendData(device.GPIOx, *pBuffer);

    /* Test on EV8 and clear it */
    while (!I2C_CheckEvent(device.GPIOx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    /* Send STOP condition */
    I2C_GenerateSTOP(device.GPIOx, ENABLE);

    // EXT_CRT_SECTION();
}

/**
 * @brief  Reads a block of data from the MPU6050.
 * @param  slaveAddr  : slave address MPU6050_DEFAULT_ADDRESS
 * @param  pBuffer : pointer to the buffer that receives the data read from the MPU6050.
 * @param  readAddr : MPU6050's internal address to read from.
 * @param  NumByteToRead : number of bytes to read.
 * @return None
 */
void I2C_BufferRead(I2CDevice_TypeDef device, u8* pBuffer, u8 readAddr, u16 NumByteToRead)
{
    // ENTR_CRT_SECTION();

    /* While the bus is busy */
    while (I2C_GetFlagStatus(device.GPIOx, I2C_FLAG_BUSY));

    /* Send START condition */
    I2C_GenerateSTART(device.GPIOx, ENABLE);

    /* Test on EV5 and clear it */
    while (!I2C_CheckEvent(device.GPIOx, I2C_EVENT_MASTER_MODE_SELECT));

    /* Send MPU6050 address for write */
    I2C_Send7bitAddress(device.GPIOx, device.address, I2C_Direction_Transmitter);

    /* Test on EV6 and clear it */
    while (!I2C_CheckEvent(device.GPIOx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    /* Clear EV6 by setting again the PE bit */
    I2C_Cmd(device.GPIOx, ENABLE);

    /* Send the MPU6050's internal address to write to */
    I2C_SendData(device.GPIOx, readAddr);

    /* Test on EV8 and clear it */
    while (!I2C_CheckEvent(device.GPIOx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    /* Send STRAT condition a second time */
    I2C_GenerateSTART(device.GPIOx, ENABLE);

    /* Test on EV5 and clear it */
    while (!I2C_CheckEvent(device.GPIOx, I2C_EVENT_MASTER_MODE_SELECT));

    /* Send MPU6050 address for read */
    I2C_Send7bitAddress(device.GPIOx, device.address, I2C_Direction_Receiver);

    /* Test on EV6 and clear it */
    while (!I2C_CheckEvent(device.GPIOx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

    /* While there is data to be read */
    while (NumByteToRead)
    {
        if (NumByteToRead == 1)
        {
            /* Disable Acknowledgement */
            I2C_AcknowledgeConfig(device.GPIOx, DISABLE);

            /* Send STOP Condition */
            I2C_GenerateSTOP(device.GPIOx, ENABLE);
        }

        /* Test on EV7 and clear it */
        if (I2C_CheckEvent(device.GPIOx, I2C_EVENT_MASTER_BYTE_RECEIVED))
        {
            /* Read a byte from the MPU6050 */
            *pBuffer = I2C_ReceiveData(device.GPIOx);

            /* Point to the next location where the byte read will be saved */
            pBuffer++;

            /* Decrement the read bytes counter */
            NumByteToRead--;
        }
    }

    /* Enable Acknowledgement to be ready for another reception */
    I2C_AcknowledgeConfig(device.GPIOx, ENABLE);
    // EXT_CRT_SECTION();
}

void I2C_WriteBits(I2CDevice_TypeDef device, u8* pBuffer, u8 writeAddr){

}

