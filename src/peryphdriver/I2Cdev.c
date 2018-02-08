// I2Cdev library collection - Main I2C device class header file
// Abstracts bit and byte I2C R/W functions into a convenient class
// 6/9/2012 by Jeff Rowberg <jeff@rowberg.net>
// 03/28/2017 by Kamnev Yuriy <kamnev.u1969@gmail.com>
//
// Changelog:
//     2017-03-28 - ported to STM32 using Keil MDK Pack

/* ============================================
 I2Cdev device library code is placed under the MIT license
 Copyright (c) 2013 Jeff Rowberg
 Copyright (c) 2017 Kamnev Yuriy
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
 ===============================================
 */

#include <peryphdriver/I2Cdev.h>
#include <string.h>

#include <stm32f10x_i2c.h>

#define _i2c_transmit(dev_addr, data, len, pending)	ARM_I2C_MasterTransmit(dev_addr, data, len, pending)

#define _i2c_receive(dev_addr, data, len, pending) ARM_I2C_MasterReceive(dev_addr, data, len, pending)

#define Timed(x) Timeout = 0xFFFF; while (x) { if (Timeout-- == 0) goto errReturn;}

#define i2c_transmit_ack(dev_addr, data, len) 	_i2c_transmit(dev_addr, data, len, TRUE)
#define i2c_transmit_nack(dev_addr, data, len) 	_i2c_transmit(dev_addr, data, len, FALSE)

#define i2c_receive_ack(dev_addr, data, len)	_i2c_receive(dev_addr, data, len, TRUE)
#define i2c_receive_nack(dev_addr, data, len)	_i2c_receive(dev_addr, data, len, FALSE)

/** Read several byte from an 8-bit device register.
 * @param dev_addr 	I2C slave device address
 * @param reg_addr 	Register regAddr to read from
 * @param len 		How many bytes to read
 * @param data 		Buffer to save data into
 * @return Status of read operation (0 = success, <0 = error)
 */
int8_t I2Cdev_readBytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t len,
		uint8_t *data) {
	int8_t err = 0;
	uint8_t reg_data[1] = { reg_addr };

	err = i2c_transmit_ack(dev_addr, reg_data, 1);

	if (err < 0) {
		return err;
	}

	err = i2c_receive_nack(dev_addr, data, len);

	return err;
}

/** Read a single byte from a 8-bit device register.
 * @param dev_addr 	I2C slave device address
 * @param reg_addr 	Register reg_addr to read from
 * @param data 		Buffer to save data into
 * @return Status of read operation (0 = success, <0 = error)
 */
int8_t I2Cdev_readByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data) {
	return I2Cdev_readBytes(dev_addr, reg_addr, 1, data);
}

/** Read a several 16-bit words from a 16-bit device register.
 * @param dev_addr 	I2C slave device address
 * @param reg_addr 	Register reg_addr to read from
 * @param len		Number of words to read
 * @param data 		Buffer to save data into
 * @return Status of read operation (true = success)
 */
int8_t I2Cdev_readWords(uint8_t dev_addr, uint8_t reg_addr, uint8_t len,
		uint16_t *data) {
	int8_t err;
	uint16_t bytes_num = len * 2;

	uint8_t reg_info[1] = { reg_addr };
	err = i2c_transmit_ack(dev_addr, reg_info, 1)
	;

	if (err < 0) {
		return err;
	}

	uint8_t words_in_bytes[bytes_num];
	err = i2c_receive_nack(dev_addr, words_in_bytes, bytes_num)
	;

	if (err < 0) {
		return err;
	}

	uint8_t words_cnt = 0;
	for (uint16_t i = 0; i < bytes_num; i += 2) {
		data[words_cnt++] = (words_in_bytes[i] << 8) | words_in_bytes[i + 1];
	}

	return 0;
}

/** Read a single word from a 16-bit device register.
 * @param dev_addr 	I2C slave device address
 * @param reg_addr 	Register regAddr to read from
 * @param data 		Container for single word
 * @return Status of read operation (0 = success, <0 = error)
 */
int8_t I2Cdev_readWord(uint8_t dev_addr, uint8_t reg_addr, uint16_t *data) {
	return I2Cdev_readWords(dev_addr, reg_addr, 1, data);
}

/** Read a single bit from a 8-bit device register.
 * @param dev_addr 	I2C slave device address
 * @param reg_addr 	Register regAddr to read from
 * @param bitn 		Bit position to read (0-15)
 * @param data 		Container for single bit value
 * @return Status of read operation (0 = success, <0 = error)
 */
int8_t I2Cdev_readBit(uint8_t dev_addr, uint8_t reg_addr, uint8_t bitn,
		uint8_t *data) {
	int8_t err;

	err = I2Cdev_readByte(dev_addr, reg_addr, data);
	*data = (*data >> bitn) & 0x01;

	return err;
}

/** Read several bits from a 8-bit device register.
 * @param dev_addr 	I2C slave device address
 * @param reg_addr 	Register regAddr to read from
 * @param start_bit First bit position to read (0-7)
 * @param len		Number of bits to read (<= 8)
 * @param data 		Container for right-aligned value
 * @return Status of read operation (0 = success, <0 = error)
 */
int8_t I2Cdev_readBits(uint8_t dev_addr, uint8_t reg_addr, uint8_t start_bit,
		uint8_t len, uint8_t *data) {
	int8_t err;

	uint8_t b;
	if ((err = I2Cdev_readByte(dev_addr, reg_addr, &b)) == 0) {
		uint8_t mask = ((1 << len) - 1) << (start_bit - len + 1);
		b &= mask;
		b >>= (start_bit - len + 1);
		*data = b;
	}

	return err;
}

/** Read a single bit from a 16-bit device register.
 * @param dev_addr 	I2C slave device address
 * @param reg_addr 	Register regAddr to read from
 * @param bit_n 	Bit position to read (0-15)
 * @param data 		Container for single bit value
 * @return Status of read operation (true = success)
 */
int8_t I2Cdev_readBitW(uint8_t dev_addr, uint8_t reg_addr, uint8_t bit_n,
		uint16_t *data) {
	int8_t err;

	err = I2Cdev_readWord(dev_addr, reg_addr, data);
	*data = (*data >> bit_n) & 0x01;

	return err;
}

/** Read several bits from a 16-bit device register.
 * @param dev_addr 	I2C slave device address
 * @param reg_addr 	Register regAddr to read from
 * @param start_bit First bit position to read (0-15)
 * @param len		Number of bits to read (<= 16)
 * @param data 		Container for right-aligned value
 * @return Status of read operation (0 = success, <0 = error)
 */
int8_t I2Cdev_readBitsW(uint8_t dev_addr, uint8_t reg_addr, uint8_t start_bit,
		uint8_t len, uint16_t *data) {
	int8_t err;
	uint16_t w;

	if ((err = I2Cdev_readWord(dev_addr, reg_addr, &w)) == 0) {
		uint16_t mask = ((1 << len) - 1) << (start_bit - len + 1);
		w &= mask;
		w >>= (start_bit - len + 1);
		*data = w;
	}

	return err;
}

/** Write multiple bytes to an 8-bit device register.
 * @param dev_addr 	I2C slave device address
 * @param reg_addr 	First register address to write to
 * @param len 		Number of bytes to write
 * @param data 		Buffer to copy new data from
 * @return Status of operation (0 = success, <0 = error)
 */
int8_t I2Cdev_writeBytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t len,
		uint8_t *data) {
	int8_t err;
	uint8_t ts_data[len + 1];

	ts_data[0] = reg_addr;
	memcpy(ts_data + 1, data, len);

	err = i2c_transmit_nack(dev_addr, ts_data, len + 1)
	;
	return err;
}

/** Write single byte to an 8-bit device register.
 * @param dev_addr 	I2C slave device address
 * @param reg_addr 	Register address to write to
 * @param data 		New byte value to write
 * @return Status of operation (0 = success, <0 = error)
 */
int8_t I2Cdev_writeByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data) {
	int8_t err;

	uint8_t ts_data[2] = { reg_addr, data };
	err = i2c_transmit_nack(dev_addr, ts_data, 2);

	return err;
}

/** Write single 16-bit word to an 16-bit device register.
 * @param dev_addr 	I2C slave device address
 * @param reg_addr 	Register address to write to
 * @param data 		New byte value to write
 * @return Status of operation (0 = success, <0 = error)
 */
int8_t I2Cdev_writeWord(uint8_t dev_addr, uint8_t reg_addr, uint16_t data) {
	int8_t err;
	uint8_t ts_data[3] = { reg_addr, (data >> 8) & 0xFF, data & 0xFF };

	err = i2c_transmit_nack(reg_addr, ts_data, 3);

	return err;
}

/** Write multiple words to a 16-bit device register.
 * @param dev_addr 	I2C slave device address
 * @param reg_addr 	First register address to write to
 * @param len 		Number of words to write
 * @param data 		Buffer to copy new data from
 * @return Status of operation (0 = success, <0 = error)
 */
int8_t I2Cdev_writeWords(uint8_t dev_addr, uint8_t reg_addr, uint8_t len,
		uint16_t *data) {
	uint16_t bytes_num = len * 2 + 1;
	uint8_t bytes[bytes_num];

	bytes[0] = reg_addr;

	uint16_t bytes_pos = 1;
	for (uint8_t i = 0; i < len; i++) {
		bytes[bytes_pos] = (data[i] >> 8) & 0xFF;
		bytes[bytes_pos + 1] = data[i] & 0xFF;

		bytes_pos += 2;
	}

	return i2c_transmit_nack(dev_addr, bytes, bytes_num);
}

/** write a single bit in an 8-bit device register.
 * @param dev_addr 	I2C slave device address
 * @param reg_addr 	Register regAddr to write to
 * @param bit_n 	Bit position to write (0-7)
 * @param data 		New bit value to write
 * @return Status of operation (0 = success, <0 = error)
 */
int8_t I2Cdev_writeBit(uint8_t dev_addr, uint8_t reg_addr, uint8_t bit_n,
		uint8_t data) {
	uint8_t b;
	int8_t err;

	err = I2Cdev_readByte(dev_addr, reg_addr, &b);
	if (err < 0) {
		return err;
	}

	b = (data != 0) ? (b | (1 << bit_n)) : (b &= ~(1 << bit_n));

	return I2Cdev_writeByte(dev_addr, reg_addr, b);
}

/** write a single bit in a 16-bit device register.
 * @param dev_addr 	I2C slave device address
 * @param reg_addr 	Register regAddr to write to
 * @param bit_n 	Bit position to write (0-15)
 * @param data 		New bit value to write
 * @return Status of operation (0 = success, <0 = error)
 */
int8_t I2Cdev_writeBitW(uint8_t dev_addr, uint8_t reg_addr, uint8_t bit_n,
		uint16_t data) {
	uint16_t w;
	I2Cdev_readWord(dev_addr, reg_addr, &w);

	w = (data != 0) ? (w | (1 << bit_n)) : (w &= ~(1 << bit_n));

	return I2Cdev_writeWord(dev_addr, reg_addr, w);
}

/** Write multiple bits in an 8-bit device register.
 * @param dev_addr 	I2C slave device address
 * @param reg_addr 	Register regAddr to write to
 * @param start_bit First bit position to write (0-7)
 * @param len 		Number of bits to write (not more than 8)
 * @param data 		Right-aligned value to write
 * @return Status of operation (0 = success, <0 = error)
 */
int8_t I2Cdev_writeBits(uint8_t dev_addr, uint8_t reg_addr, uint8_t start_bit,
		uint8_t len, uint8_t data) {
	uint8_t b;
	int8_t err;

	if ((err = I2Cdev_readByte(dev_addr, reg_addr, &b)) == 0) {
		uint8_t mask = ((1 << len) - 1) << (start_bit - len + 1);
		data <<= (start_bit - len + 1); // shift data into correct position
		data &= mask; // zero all non-important bits in data
		b &= ~(mask); // zero all important bits in existing byte
		b |= data; // combine data with existing byte

		return I2Cdev_writeByte(dev_addr, reg_addr, b);
	} else {
		return err;
	}
}

/** Write multiple bits in a 16-bit device register.
 * @param dev_addr 	I2C slave device address
 * @param reg_addr 	Register regAddr to write to
 * @param start_bit First bit position to write (0-15)
 * @param len 		Number of bits to write (not more than 16)
 * @param data 		Right-aligned value to write
 * @return Status of operation (0 = success, <0 = error)
 */
int8_t I2Cdev_writeBitsW(uint8_t dev_addr, uint8_t reg_addr, uint8_t start_bit,
		uint8_t len, uint16_t data) {
	uint16_t w;
	int8_t err;

	if ((err = I2Cdev_readWord(dev_addr, reg_addr, &w)) != 0) {
		uint16_t mask = ((1 << len) - 1) << (start_bit - len + 1);
		data <<= (start_bit - len + 1); // shift data into correct position
		data &= mask; // zero all non-important bits in data
		w &= ~(mask); // zero all important bits in existing word
		w |= data; // combine data with existing word
		return I2Cdev_writeWord(dev_addr, reg_addr, w);
	} else {
		return err;
	}
}
/*
 \fn          int32_t ARM_I2C_MasterTransmit (uint32_t addr, const uint8_t *data, uint32_t num, bool xfer_pending)
 \brief       Start transmitting data as I2C Master.
 \param[in]   addr          Slave address (7-bit or 10-bit)
 \param[in]   data          Pointer to buffer with data to transmit to I2C Slave
 \param[in]   num           Number of data bytes to transmit
 \param[in]   xfer_pending  Transfer operation is pending - Stop condition will not be generated
 \return      \ref execution_status
 */
int32_t ARM_I2C_MasterTransmit(uint32_t addr, const uint8_t *data, uint32_t num, bool xfer_pending) {

	__IO uint32_t Timeout = 0;

	/* Enable Error IT (used in all modes: DMA, Polling and Interrupts */
	//    I2Cx->CR2 |= I2C_IT_ERR;
	if (num) {
		Timed(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));

		// Intiate Start Sequence

		I2C_GenerateSTART(I2C1, ENABLE);
		Timed(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

		// Send Address  EV5

		I2C_Send7bitAddress(I2C1, addr, I2C_Direction_Transmitter);
		Timed(
				!I2C_CheckEvent(I2C1,
						I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

		// EV6

		// Write first byte EV8_1

		I2C_SendData(I2C1, *data++);

		while (--num) {

			// wait on BTF

			Timed(!I2C_GetFlagStatus(I2C1, I2C_FLAG_BTF));
			I2C_SendData(I2C1, *data++);
		}

		Timed(!I2C_GetFlagStatus(I2C1, I2C_FLAG_BTF));
//		if(!xfer_pending){
			I2C_GenerateSTOP(I2C1, ENABLE);
			Timed(I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF));
//		}
	}
	return SUCCESS;
	  errReturn:
	     return ERROR;

}

/*
 \fn          int32_t ARM_I2C_MasterReceive (uint32_t addr, uint8_t *data, uint32_t num, bool xfer_pending)
 \brief       Start receiving data as I2C Master.
 \param[in]   addr          Slave address (7-bit or 10-bit)
 \param[out]  data          Pointer to buffer for data to receive from I2C Slave
 \param[in]   num           Number of data bytes to receive
 \param[in]   xfer_pending  Transfer operation is pending - Stop condition will not be generated
 \return      \ref execution_status
 */
int32_t ARM_I2C_MasterReceive(uint32_t addr, uint8_t *data, uint32_t num, bool xfer_pending) {

	__IO uint32_t Timeout = 0;

	  //    I2Cx->CR2 |= I2C_IT_ERR;  interrupts for errors

	  if (!num)
	    return SUCCESS;



	  // Wait for idle I2C interface

	  Timed(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));

	  // Enable Acknowledgement, clear POS flag

	  I2C_AcknowledgeConfig(I2C1, ENABLE);
	  I2C_NACKPositionConfig(I2C1, I2C_NACKPosition_Current);

	  // Intiate Start Sequence (wait for EV5

	  I2C_GenerateSTART(I2C1, ENABLE);
	  Timed(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

	  // Send Address

	  I2C_Send7bitAddress(I2C1, addr, I2C_Direction_Receiver);

	  // EV6

	  Timed(!I2C_GetFlagStatus(I2C1, I2C_FLAG_ADDR));

	  if (num == 1)
	    {

	      // Clear Ack bit

	      I2C_AcknowledgeConfig(I2C1, DISABLE);

	      // EV6_1 -- must be atomic -- Clear ADDR, generate STOP

	      __disable_irq();
	      (void) I2C1->SR2;
	      I2C_GenerateSTOP(I2C1,ENABLE);
	      __enable_irq();

	      // Receive data   EV7

	      Timed(!I2C_GetFlagStatus(I2C1, I2C_FLAG_RXNE));
	      *data++ = I2C_ReceiveData(I2C1);

	    }
	  else if (num == 2)
	    {
	      // Set POS flag

	      I2C_NACKPositionConfig(I2C1, I2C_NACKPosition_Next);

	      // EV6_1 -- must be atomic and in this order

	      __disable_irq();
	      (void) I2C1->SR2;                           // Clear ADDR flag
	      I2C_AcknowledgeConfig(I2C1, DISABLE);       // Clear Ack bit
	      __enable_irq();

	      // EV7_3  -- Wait for BTF, program stop, read data twice

	      Timed(!I2C_GetFlagStatus(I2C1, I2C_FLAG_BTF));

	      __disable_irq();
	      I2C_GenerateSTOP(I2C1,ENABLE);
	      *data++ = I2C1->DR;
	      __enable_irq();

	      *data++ = I2C1->DR;

	    }
	  else
	    {
	      (void) I2C1->SR2;                           // Clear ADDR flag
	      while (num-- != 3)
		{
		  // EV7 -- cannot guarantee 1 transfer completion time, wait for BTF
	          //        instead of RXNE

		  Timed(!I2C_GetFlagStatus(I2C1, I2C_FLAG_BTF));
		  *data++ = I2C_ReceiveData(I2C1);
		}

	      Timed(!I2C_GetFlagStatus(I2C1, I2C_FLAG_BTF));

	      // EV7_2 -- Figure 1 has an error, doesn't read N-2 !

	      I2C_AcknowledgeConfig(I2C1, DISABLE);           // clear ack bit

	      __disable_irq();
	      *data++ = I2C_ReceiveData(I2C1);             // receive byte N-2
	      I2C_GenerateSTOP(I2C1,ENABLE);                  // program stop
	      __enable_irq();

	      *data++ = I2C_ReceiveData(I2C1);             // receive byte N-1

	      // wait for byte N

	      Timed(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
	      *data++ = I2C_ReceiveData(I2C1);

	      num = 0;

	    }

	  // Wait for stop

	  Timed(I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF));
	  return SUCCESS;
	  errReturn:
	     return ERROR;


}
