/*
 * mpu6050_i2c_driver.c
 *
 *  Created on: Feb 4, 2018
 *      Author: gege3
 */
#include <sensordriver/mpu6050_i2c_driver.h>

void MPU6050_SetBits(I2C_HandleTypeDef* hi2c1, uint8_t data, uint8_t registerAddress);
void MPU6050_ResetBits(I2C_HandleTypeDef* hi2c1, uint8_t data, uint8_t registerAddress);

void MPU6050_init(MPU6050Init_TypeDef* MPU6050_InitStruct, I2C_HandleTypeDef* hi2c1) {
	MPU6050_Settings = MPU6050_InitStruct;
	MPU6050_Settings->hi2c1 = hi2c1;

	HAL_I2C_Init(hi2c1);

	MPU6050_SetBits(hi2c1, MPU6050_RESET, MPU6050_PWR_MGMT_1);

	//set clock source
	MPU6050_SetBits(hi2c1, MPU6050_PLL_GYRO_CLOCK_X | MPU6050_CYCLE, MPU6050_PWR_MGMT_1);
	MPU6050_ResetBits(hi2c1, MPU6050_SLEEP, MPU6050_PWR_MGMT_1);
	MPU6050_ResetBits(hi2c1, 0xff, MPU6050_PWR_MGMT_2);


	MPU6050_SetBits(hi2c1, 0x04, MPU6050_SMPLRT_DIV);

	MPU6050_SetBits(hi2c1, MPU6050_DLPF_CFG_3, MPU6050_CONFIG);

	//set gyro resolution
	MPU6050_SetBits(hi2c1, MPU6050_FS_SEL_250, MPU6050_GYRO_CONFIG);

	//set accel resolution
	MPU6050_SetBits(hi2c1, MPU6050_FS_SEL_2G, MPU6050_ACCEL_CONFIG);

//	MPU6050_WriteByte(hi2c1, 0x40, MPU6050_USER_CTRL);
//	MPU6050_WriteByte(hi2c1, 0x78, MPU6050_FIFO_EN);

//	//selftest gyro
//	MPU6050_ResetBits(hi2c1, MPU6050_XG_ST | MPU6050_YG_ST | MPU6050_ZG_ST, MPU6050_GYRO_CONFIG);
//
//    //selftest accel
//    MPU6050_ResetBits(hi2c1, MPU6050_XA_ST | MPU6050_YA_ST | MPU6050_ZA_ST, MPU6050_ACCEL_CONFIG);

}

uint16_t MPU6050_read_register_dword(uint8_t regAddress){

    uint8_t buf [2];
	HAL_I2C_Master_Transmit(MPU6050_Settings->hi2c1, MPU6050_Settings->Adress, &regAddress, 1, 10);
	HAL_I2C_Master_Receive(MPU6050_Settings->hi2c1, MPU6050_Settings->Adress, &buf[0], 1, 10);

	regAddress++;
    HAL_I2C_Master_Transmit(MPU6050_Settings->hi2c1, MPU6050_Settings->Adress, &regAddress, 1, 10);
    HAL_I2C_Master_Receive(MPU6050_Settings->hi2c1, MPU6050_Settings->Adress, &buf[1], 1, 10);

	int16_t ret = ((uint16_t)buf[0] << 8) | buf[1];
	return ret;
}

uint8_t MPU6050_read_register_word(uint8_t regAddress){

    uint8_t buf [1];
    buf[0] = regAddress;
    HAL_I2C_Master_Transmit(MPU6050_Settings->hi2c1, MPU6050_Settings->Adress, buf, 1, 10);
    HAL_I2C_Master_Receive(MPU6050_Settings->hi2c1, MPU6050_Settings->Adress, buf, 1, 10);

    return buf[0];
}

void MPU6050_read_sensor_data(MPU6050SensorData_TypeDef* MPU6050SensorData){
    MPU6050SensorData->AccelX = MPU6050_read_register_dword(MPU6050_ACCEL_XOUT);
    MPU6050SensorData->AccelY = MPU6050_read_register_dword(MPU6050_ACCEL_YOUT);
    MPU6050SensorData->AccelZ = MPU6050_read_register_dword(MPU6050_ACCEL_ZOUT);

    MPU6050SensorData->GyroX = MPU6050_read_register_dword(MPU6050_GYRO_XOUT);
    MPU6050SensorData->GyroY = MPU6050_read_register_dword(MPU6050_GYRO_YOUT);
    MPU6050SensorData->GyroZ = MPU6050_read_register_dword(MPU6050_GYRO_ZOUT);
}

void MPU6050_read_fifo(MPU6050SensorData_TypeDef* MPU6050SensorData){

    uint16_t fifo_count = MPU6050_read_register_dword(MPU6050_FIFO_COUNT);

    if(fifo_count >= 12){
        uint8_t data [12];
        uint8_t address = MPU6050_FIFO_R_W;
        HAL_I2C_Master_Transmit(MPU6050_Settings->hi2c1, MPU6050_Settings->Adress, &address, 1, 10);
        HAL_I2C_Master_Receive(MPU6050_Settings->hi2c1, MPU6050_Settings->Adress, data, 12, 1);

        MPU6050SensorData->AccelX = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
        MPU6050SensorData->AccelY= (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
        MPU6050SensorData->AccelZ = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
        MPU6050SensorData->GyroX  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
        MPU6050SensorData->GyroY = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
        MPU6050SensorData->GyroZ = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;
    }

}

uint8_t MPU6050_check_connection(){
//	uint8_t buf = 0x00;
//	I2C_BufferRead(&MPU6050_Settings->I2C_Device, &buf, MPU6050_WHO_AM_I);
//	if(MPU6050_Settings->I2C_Device.address == (buf << 1)){
//		return 1;
//	}
//	return 0;
}

// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU6050SelfTest(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
   uint8_t rawData[4];
   uint8_t selfTest[6];
   float factoryTrim[6];

   rawData[0] = MPU6050_read_register_word(MPU6050_SELF_TEST_X); // X-axis self-test results
   rawData[1] = MPU6050_read_register_word(MPU6050_SELF_TEST_Y); // Y-axis self-test results
   rawData[2] = MPU6050_read_register_word(MPU6050_SELF_TEST_Z); // Z-axis self-test results
   rawData[3] = MPU6050_read_register_word(MPU6050_SELF_TEST_A); // Mixed-axis self-test results

   // Extract the acceleration test results first
   selfTest[0] = (rawData[0] >> 3) | (rawData[3] & 0x30) >> 4 ; // XA_TEST result is a five-bit unsigned integer
   selfTest[1] = (rawData[1] >> 3) | (rawData[3] & 0x0C) >> 4 ; // YA_TEST result is a five-bit unsigned integer
   selfTest[2] = (rawData[2] >> 3) | (rawData[3] & 0x03) >> 4 ; // ZA_TEST result is a five-bit unsigned integer
   // Extract the gyration test results first
   selfTest[3] = rawData[0]  & 0x1F ; // XG_TEST result is a five-bit unsigned integer
   selfTest[4] = rawData[1]  & 0x1F ; // YG_TEST result is a five-bit unsigned integer
   selfTest[5] = rawData[2]  & 0x1F ; // ZG_TEST result is a five-bit unsigned integer
   // Process results to allow final comparison with factory set values
   factoryTrim[0] = (4096.0*0.34)*(pow( (0.92/0.34) , (((float)selfTest[0] - 1.0)/30.0))); // FT[Xa] factory trim calculation
   factoryTrim[1] = (4096.0*0.34)*(pow( (0.92/0.34) , (((float)selfTest[1] - 1.0)/30.0))); // FT[Ya] factory trim calculation
   factoryTrim[2] = (4096.0*0.34)*(pow( (0.92/0.34) , (((float)selfTest[2] - 1.0)/30.0))); // FT[Za] factory trim calculation
   factoryTrim[3] =  ( 25.0*131.0)*(pow( 1.046 , ((float)selfTest[3] - 1.0) ));             // FT[Xg] factory trim calculation
   factoryTrim[4] =  (-25.0*131.0)*(pow( 1.046 , ((float)selfTest[4] - 1.0) ));             // FT[Yg] factory trim calculation
   factoryTrim[5] =  ( 25.0*131.0)*(pow( 1.046 , ((float)selfTest[5] - 1.0) ));             // FT[Zg] factory trim calculation

 //  Output self-test results and factory trim calculation if desired
 //  Serial.println(selfTest[0]); Serial.println(selfTest[1]); Serial.println(selfTest[2]);
 //  Serial.println(selfTest[3]); Serial.println(selfTest[4]); Serial.println(selfTest[5]);
 //  Serial.println(factoryTrim[0]); Serial.println(factoryTrim[1]); Serial.println(factoryTrim[2]);
 //  Serial.println(factoryTrim[3]); Serial.println(factoryTrim[4]); Serial.println(factoryTrim[5]);

 // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
 // To get to percent, must multiply by 100 and subtract result from 100
   for (int i = 0; i < 6; i++) {
     destination[i] = 100.0 + 100.0*((float)selfTest[i] - factoryTrim[i])/factoryTrim[i]; // Report percent differences
   }

}

void MPU6050_SetBits(I2C_HandleTypeDef* hi2c1, uint8_t data, uint8_t registerAddress){
	uint8_t buf [2];

	HAL_I2C_Master_Transmit(hi2c1, MPU6050_Settings->Adress, &registerAddress, 1, 10);
	HAL_I2C_Master_Receive(hi2c1, MPU6050_Settings->Adress, buf, 1, 10);

	buf[1] = buf[0] | data;
	buf[0] = registerAddress;

	HAL_I2C_Master_Transmit(hi2c1, MPU6050_Settings->Adress, buf, 2, 10);

}
void MPU6050_ResetBits(I2C_HandleTypeDef* hi2c1, uint8_t data, uint8_t registerAddress){
	uint8_t buf [2];

	HAL_I2C_Master_Transmit(hi2c1, MPU6050_Settings->Adress, &registerAddress, 1, 10);
	HAL_I2C_Master_Receive(hi2c1, MPU6050_Settings->Adress, buf, 1, 10);

    buf[1] = buf[0] & ~data;
    buf[0] = registerAddress;

	HAL_I2C_Master_Transmit(hi2c1, MPU6050_Settings->Adress, buf, 2, 10);

}

void MPU6050_WriteByte(I2C_HandleTypeDef* hi2c1, uint8_t data, uint8_t registerAddress){
    uint8_t buf [2];
    buf[0] = registerAddress;
    buf[1] = data;
    HAL_I2C_Master_Transmit(hi2c1, MPU6050_Settings->Adress, buf, 2, 10);

}
