/*
 * sensors.c
 *
 *  Created on: Feb 12, 2018
 *      Author: Horvath_Gergo
 */

#include <tasks/sensors.h>


void SensorMeasurementTask(void const* argument){

	dataReady = xSemaphoreCreateBinary();

	//initialize sensors
	trace_puts("sensor task started");
	mpu6050_init((I2C_HandleTypeDef*)argument);
	osDelay(250);

	char szoveg [70];

    GPIO_InitTypeDef gpio;
    gpio.Pin = GPIO_PIN_13;
    gpio.Pull= GPIO_PULLUP | GPIO_PULLDOWN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &gpio);

    q31_t data [4] = {0};
    q31_t rpy [3] = {0};

    mpu6050_dmpInitialize();
    mpu6050_dmpEnable();

    /* Configure PA5 pin as input floating */
    GPIO_InitTypeDef   GPIO_InitStructure;
    GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Pin = GPIO_PIN_5;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

    //Configure EXTI5-9 interrupt port
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 8, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

    uint8_t i = 0;
    float32_t data_f [4];

	for(;;){

//		xSemaphoreTake(dataReady, portMAX_DELAY);

		  mpu6050_getQuaternionWait(data);
//	      mpu6050_getRollPitchYaw(qw, qx, qy, qz, &roll, &pitch, &yaw);

	    	  arm_q31_to_float(data, data_f, 4);
	    	  snprintf(szoveg, 70, "qw:%.7f,qx:%.7f,qy:%.7f,qz:%.7f\r\n", data_f[0], data_f[1], data_f[2], data_f[3]);
	    	  HAL_UART_Transmit(&huart1, data, 70, 20);
	}

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_5){
//		xSemaphoreGiveFromISR(dataReady, pdFALSE);
	}
}
