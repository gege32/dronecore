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

    mpu6050_dmpInitialize();
    mpu6050_dmpEnable();

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI1_IRQn, 7, 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);

    float32_t data_f[3];

    SensorData_TypeDef* sensor_data = pvPortMalloc(sizeof(SensorData_TypeDef));
    Quternion_TypeDef* quaternion = pvPortMalloc(sizeof(Quternion_TypeDef));

    for (;;) {

        xSemaphoreTake(dataReady, portMAX_DELAY);

        mpu6050_getQuaternionWait(quaternion);
        //RPY is in float32
        mpu6050_getRollPitchYaw(quaternion, data_f);
        sensor_data->roll = data_f[0];
        //TODO!!! WHYYYYYYY
        sensor_data->pitch = data_f[2];
        sensor_data->yaw = data_f[1];
        sensor_data->height = 0;

        xQueueOverwrite(sensorDataQueue, sensor_data);

        snprintf(szoveg, 32, "%+.6f,%+.6f,%+.6f\r\n", data_f[0], data_f[1], data_f[2]);
        HAL_UART_Transmit(&huart1, szoveg, 32, 20);
    }

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_1) {
        xSemaphoreGiveFromISR(dataReady, pdFALSE);
    }
}
