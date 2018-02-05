/*
 * This file is part of the µOS++ distribution.
 *   (https://github.com/micro-os-plus)
 * Copyright (c) 2014 Liviu Ionescu.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom
 * the Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

// ----------------------------------------------------------------------------
#include <sensordriver/mpu6050_i2c_driver.h>
#include <peryphdriver/i2c_driver.h>
#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"
#include "stm32f10x.h"

// ----------------------------------------------------------------------------
//
// Standalone STM32F1 empty sample (trace via DEBUG).
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the DEBUG output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

int main(int argc, char* argv[]) {
	// At this stage the system clock should have already been configured
	// at high speed.

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	trace_puts("standard output");
	GPIO_InitTypeDef initA;
	initA.GPIO_Mode = GPIO_Mode_Out_PP;
	initA.GPIO_Speed = GPIO_Speed_50MHz;
	initA.GPIO_Pin = GPIO_Pin_All;
	trace_puts("struct ready");

	GPIO_Init(GPIOA, &initA);
	trace_puts("init ready");
	GPIO_ResetBits(GPIOA, GPIO_Pin_All);
	trace_puts("reset to 0");

	//testing i2c


	I2CInit_TypeDef i2cinit;
	I2C_Initialize(&i2cinit);

	I2CDevice_TypeDef deviceSettings;
	deviceSettings.I2Cx = I2C1;
	deviceSettings.address = 0xd0;
	deviceSettings.I2CSettings = i2cinit;

	MPU6050Init_TypeDef init;
	init.I2C_Device = deviceSettings;
	init.MPU6050_AccelerometerRange = MPU6050_FS_SEL_16G;
	init.MPU6050_ClockSource = MPU6050_PLL_GYRO_CLOCK_X;
	init.MPU6050_GyroscopeRange = MPU6050_FS_SEL_2000;

	MPU6050_init(&init);

	trace_puts("MPU6050 initialized");

	MPU6050_check_connection();

	char* temp [10];
	uint16_t tempnum;

	// Infinite loop
	while (1) {

		tempnum = MPU6050_read_accel_X();
		snprintf(&temp, 10, "%i accX", tempnum);
		trace_puts(temp);

		tempnum = MPU6050_read_accel_Y();
		sprintf(&temp, "%i accY", tempnum);
		trace_puts(temp);

		tempnum = MPU6050_read_accel_Z();
		sprintf(&temp, "%i accZ", tempnum);
		trace_puts(temp);

		tempnum = MPU6050_read_gyro_X();
		sprintf(&temp, "%i gyroX", tempnum);
		trace_puts(temp);

		tempnum = MPU6050_read_gyro_Y();
		sprintf(&temp, "%i gyroY", tempnum);
		trace_puts(temp);

		tempnum = MPU6050_read_gyro_Z();
		sprintf(&temp, "%i gyroZ", tempnum);
		trace_puts(temp);

	}
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
