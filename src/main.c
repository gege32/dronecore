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
#include <sensordriver/MPU6050.h>
#include <peryphdriver/I2Cdev.h>
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


	MPU6050(0xd0);
	MPU6050_initialize();

	MPU6050_testConnection();

	trace_puts("MPU6050 initialized");

	MPUdmpInitialize();

	char* temp [20];
	int16_t s_temp;

    Quaternion q;
    MPUdmpGetQuaternion(&q, 0);
    snprintf(&temp, 20, "Q: %i, %i, %i, %i", q.x, q.y, q.z, q.w);
    trace_puts(temp);

	int16_t x;
	int16_t y;
	int16_t z;

	// Infinite loop
	while (1) {

		s_temp = MPU6050_getTemperature();
		s_temp = (s_temp / 340) + 36.53;
		snprintf(&temp, 10, "%i temp", s_temp);
		trace_puts(temp);

		MPU6050_getAcceleration(&x, &y, &z);
		snprintf(&temp, 15, "%i, %i, %i", x, y, z);
		trace_puts(temp);

		MPU6050_getRotation(&x, &y, &z);
		snprintf(&temp, 15, "%i, %i, %i", x, y, z);
		trace_puts(temp);

	}
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
