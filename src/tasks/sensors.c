/*
 * sensors.c
 *
 *  Created on: Feb 12, 2018
 *      Author: Horvath_Gergo
 */

#include <tasks/sensors.h>

void SensorMeasurementTask(void const* argument) {

    dataReady = xSemaphoreCreateBinary();

    bmp180_calibration_data = pvPortMalloc(sizeof(CalibrationData));
    bmp180_calibration_data->oss = 3;

    char szoveg[40];

    //initialize sensors
    trace_puts("sensor task started");
    mpu6050_init((I2C_HandleTypeDef*) argument);
    bmp180_initialize((I2C_HandleTypeDef*) argument);
    bool test = bmp180_check_presence();
    if (test == TRUE) {
        trace_puts("sOK");
        bmp180_get_calibration_data(bmp180_calibration_data);
        bmp180_get_uncompensated_pressure(bmp180_calibration_data);
        bmp180_get_uncompensated_temperature(bmp180_calibration_data);
        bmp180_calculate_true_pressure(bmp180_calibration_data);
        bmp180_calculate_true_temperature(bmp180_calibration_data);
        float f = bmp180_get_absolute_altitude(bmp180_calibration_data);
        snprintf(szoveg, 40, "altitude= %+.6f", f);
        trace_puts(szoveg);
    }
    osDelay(250);

    GPIO_InitTypeDef gpio;
    gpio.Pin = GPIO_PIN_13;
    gpio.Pull = GPIO_PULLUP | GPIO_PULLDOWN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &gpio);

    q31_t data[4] = { 0 };
    q31_t rpy[3] = { 0 };

    mpu6050_dmpInitialize();
    mpu6050_dmpEnable();

    /* Configure PA5 pin as input floating */
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Pin = GPIO_PIN_5;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

    //Configure EXTI5-9 interrupt port
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 8, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

    q31_t data_q[3];
    int8_t data_int[4];
    float32_t data_f[3];

    SensorData_TypeDef* sensor_data = pvPortMalloc(sizeof(SensorData_TypeDef));

    for (;;) {

        xSemaphoreTake(dataReady, portMAX_DELAY);

        mpu6050_getQuaternionWait(data);
        //RPY is in q31, and is scaled down to +-1 range. (except for yaw, since it should be 360deg.
        mpu6050_getRollPitchYaw(data, data_q);
        sensor_data->roll = data_q[0];
        sensor_data->pitch = data_q[1];
        sensor_data->yaw = data_q[2];
        sensor_data->height = 0;

        xQueueSend(sensorDataQueue, sensor_data, 1);

//        arm_q31_to_float(data_q, data_f, 3);
//        snprintf(szoveg, 32, "%+.6f,%+.6f,%+.6f\r\n", data_f[0], data_f[1], data_f[2]);
//
//        HAL_UART_Transmit(&huart1, szoveg, 32, 20);
    }

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_5) {
        xSemaphoreGiveFromISR(dataReady, pdFALSE);
    }
}
