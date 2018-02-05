/*
 * i2c_driver->c
 *
 *  Created on: Feb 5, 2018
 *      Author: Horvath_Gergo
 */
#include <peryphdriver/i2c_driver.h>

/**
 * @brief  Initializes the I2C peripheral
 * @param  None
 * @return None
 */
void I2C_Initialize(I2CInit_TypeDef* I2CSettings)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitTypeDef initB;
	initB.GPIO_Mode = GPIO_Mode_AF_OD;
	initB.GPIO_Speed = GPIO_Speed_2MHz;
	initB.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init(GPIOB, &initB);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	I2C_InitTypeDef i2cdef;
	I2C_StructInit(&i2cdef);
	i2cdef.I2C_ClockSpeed = 100000;

	I2C_Init(I2C1, &i2cdef);
	I2C_Cmd(I2C1, ENABLE);

}

/**
 * @brief  Writes one byte to the  I2C bus->
 * @param  slaveAddr : slave address
 * @param  pBuffer : pointer to the buffer containing the data to be written->
 * @param  writeAddr : address of the register in which the data will be written
 * @return None
 */
void I2C_ByteWrite(I2CDevice_TypeDef* device, u8* pBuffer, u8 writeAddr)
{
    /* Send START condition */
    I2C_GenerateSTART(device->I2Cx, ENABLE);

    /* Test on EV5 and clear it */
    while (!I2C_CheckEvent(device->I2Cx, I2C_EVENT_MASTER_MODE_SELECT));

    /* Send MPU6050 address for write */
    I2C_Send7bitAddress(device->I2Cx, device->address, I2C_Direction_Transmitter);

    /* Test on EV6 and clear it */
    while (!I2C_CheckEvent(device->I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    /* Send the MPU6050's internal address to write to */
    I2C_SendData(device->I2Cx, writeAddr);

    /* Test on EV8 and clear it */
    while (!I2C_CheckEvent(device->I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    /* Send the byte to be written */
    I2C_SendData(device->I2Cx, *pBuffer);

    /* Test on EV8 and clear it */
    while (!I2C_CheckEvent(device->I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    /* Send STOP condition */
    I2C_GenerateSTOP(device->I2Cx, ENABLE);

}

/**
 * @brief  Reads a block of data->
 * @param  slaveAddr  : slave address
 * @param  pBuffer : pointer to the buffer that receives the data read->
 * @param  readAddr : Internal address to read from->
 * @param  NumByteToRead : number of bytes to read->
 * @return None
 */
void I2C_BufferRead(I2CDevice_TypeDef* device, u8* pBuffer, u8 readAddr)
{

    /* While the bus is busy */
    while (I2C_GetFlagStatus(device->I2Cx, I2C_FLAG_BUSY));

    /* Send START condition */
    I2C_GenerateSTART(device->I2Cx, ENABLE);

    /* Test on EV5 and clear it */
    while (!I2C_CheckEvent(device->I2Cx, I2C_EVENT_MASTER_MODE_SELECT));

    /* Send MPU6050 address for write */
    I2C_Send7bitAddress(device->I2Cx, device->address, I2C_Direction_Transmitter);

    /* Test on EV6 and clear it */
    while (!I2C_CheckEvent(device->I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    /* Clear EV6 by setting again the PE bit */
    I2C_Cmd(device->I2Cx, ENABLE);

    /* Send the MPU6050's internal address to write to */
    I2C_SendData(device->I2Cx, readAddr);

    /* Test on EV8 and clear it */
    while (!I2C_CheckEvent(device->I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    /* Send STRAT condition a second time */
    I2C_GenerateSTART(device->I2Cx, ENABLE);

    /* Test on EV5 and clear it */
    while (!I2C_CheckEvent(device->I2Cx, I2C_EVENT_MASTER_MODE_SELECT));

    /* Send MPU6050 address for read */
    I2C_Send7bitAddress(device->I2Cx, device->address, I2C_Direction_Receiver);

    /* Test on EV6 and clear it */
    while (!I2C_CheckEvent(device->I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

    I2C_AcknowledgeConfig(device->I2Cx, DISABLE);
    I2C_GenerateSTOP(device->I2Cx, ENABLE);

    while (!I2C_CheckEvent(device->I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED));

    /* Read a byte from the MPU6050 */
    *pBuffer = I2C_ReceiveData(device->I2Cx);

    while (I2C_GetFlagStatus(device->I2Cx, I2C_FLAG_STOPF));

    /* Enable Acknowledgement to be ready for another reception */
    I2C_AcknowledgeConfig(device->I2Cx, ENABLE);

}

void I2C_SetBits(I2CDevice_TypeDef* device, u8 data, u8 writeAddr){

	uint8_t buff = 0x00;
	I2C_BufferRead(device, &buff, writeAddr);

	buff |= data;

	I2C_ByteWrite(device, &buff, writeAddr);


}

void I2C_ResetBits(I2CDevice_TypeDef* device, u8 data, u8 writeAddr){

    uint8_t buff = 0x00;
	I2C_BufferRead(device, &buff, writeAddr);

	buff = buff & ~data;

	I2C_ByteWrite(device, &buff, writeAddr);


}

