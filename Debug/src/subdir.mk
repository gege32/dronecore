################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/_write.c \
../src/freertos.c \
../src/main.c \
../src/stm32f1xx_hal_msp.c \
../src/stm32f1xx_hal_timebase_TIM.c \
../src/stm32f1xx_it.c \
../src/system_stm32f1xx.c 

OBJS += \
./src/_write.o \
./src/freertos.o \
./src/main.o \
./src/stm32f1xx_hal_msp.o \
./src/stm32f1xx_hal_timebase_TIM.o \
./src/stm32f1xx_it.o \
./src/system_stm32f1xx.o 

C_DEPS += \
./src/_write.d \
./src/freertos.d \
./src/main.d \
./src/stm32f1xx_hal_msp.d \
./src/stm32f1xx_hal_timebase_TIM.d \
./src/stm32f1xx_it.d \
./src/system_stm32f1xx.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DUSE_FULL_ASSERT -DTRACE -DOS_USE_TRACE_SEMIHOSTING_DEBUG -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=72000000 -I"../include" -I"E:\!!devtools\szakdolgozat\DroneCore\system\CMSIS\Include" -I"E:\!!devtools\szakdolgozat\DroneCore\system\CMSIS\Device\ST\STM32F1xx\Include" -I"E:\!!devtools\szakdolgozat\DroneCore\system\STM32F1xx_HAL_Driver\Inc" -I"E:\!!devtools\szakdolgozat\DroneCore\Middlewares\Third_Party\FreeRTOS\Source\include" -I"E:\!!devtools\szakdolgozat\DroneCore\Middlewares\Third_Party\FreeRTOS\Source\portable\GCC\ARM_CM3" -I"E:\!!devtools\szakdolgozat\DroneCore\Middlewares\Third_Party\FreeRTOS\Source\CMSIS_RTOS" -std=gnu11 -specs=nano.specs -specs=nosys.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


