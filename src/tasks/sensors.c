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

    double qw, qx, qy, qz, roll, pitch, yaw;

    mpu6050_dmpInitialize();
    mpu6050_dmpEnable();

    /* Configure PA5 pin as input floating */
    GPIO_InitTypeDef   GPIO_InitStructure;
    GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Pin = GPIO_PIN_5;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

    //Configure EXTI5-9 interrupt port
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);


	for(;;){

		xSemaphoreTake(dataReady, portMAX_DELAY);

		  mpu6050_getQuaternionWait(&qw, &qx, &qy, &qz);
	      mpu6050_getRollPitchYaw(qw, qx, qy, qz, &roll, &pitch, &yaw);

	      snprintf(szoveg, 70, "qw:%f,qx:%f,qy:%f,qz:%f,roll:%fpitch:%f,yaw:%f", qw, qx, qy, qz, roll, pitch, yaw);
		  trace_puts(szoveg);
	}

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_5){
		xSemaphoreGiveFromISR(dataReady, pdFALSE);
	}
}
