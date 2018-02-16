################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/sensordriver/mpu6050.c \
../src/sensordriver/mpu6050_i2c_driver.c \
../src/sensordriver/mpu6050dmp6.c 

OBJS += \
./src/sensordriver/mpu6050.o \
./src/sensordriver/mpu6050_i2c_driver.o \
./src/sensordriver/mpu6050dmp6.o 

C_DEPS += \
./src/sensordriver/mpu6050.d \
./src/sensordriver/mpu6050_i2c_driver.d \
./src/sensordriver/mpu6050dmp6.d 


# Each subdirectory must supply rules for building sources it contributes
src/sensordriver/%.o: ../src/sensordriver/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DUSE_FULL_ASSERT -DTRACE -DOS_USE_TRACE_SEMIHOSTING_DEBUG -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=72000000 -I"../include" -I"E:\!!devtools\szakdolgozat\DroneCore\system\CMSIS\Include" -I"E:\!!devtools\szakdolgozat\DroneCore\system\CMSIS\Device\ST\STM32F1xx\Include" -I"E:\!!devtools\szakdolgozat\DroneCore\system\STM32F1xx_HAL_Driver\Inc" -I"E:\!!devtools\szakdolgozat\DroneCore\Middlewares\Third_Party\FreeRTOS\Source\include" -I"E:\!!devtools\szakdolgozat\DroneCore\Middlewares\Third_Party\FreeRTOS\Source\portable\GCC\ARM_CM3" -I"E:\!!devtools\szakdolgozat\DroneCore\Middlewares\Third_Party\FreeRTOS\Source\CMSIS_RTOS" -std=gnu11 -specs=nano.specs -specs=nosys.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


