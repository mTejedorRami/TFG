################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Application/User/adc.c \
../Application/User/dma.c \
../Application/User/fsm.c \
E:/TFG/SW/SW/Src/gpio.c \
E:/TFG/SW/SW/Src/i2c.c \
E:/TFG/SW/SW/Src/main.c \
../Application/User/ppg.c \
../Application/User/stm32l4xx_hal_msp.c \
../Application/User/stm32l4xx_it.c 

OBJS += \
./Application/User/adc.o \
./Application/User/dma.o \
./Application/User/fsm.o \
./Application/User/gpio.o \
./Application/User/i2c.o \
./Application/User/main.o \
./Application/User/ppg.o \
./Application/User/stm32l4xx_hal_msp.o \
./Application/User/stm32l4xx_it.o 

C_DEPS += \
./Application/User/adc.d \
./Application/User/dma.d \
./Application/User/fsm.d \
./Application/User/gpio.d \
./Application/User/i2c.d \
./Application/User/main.d \
./Application/User/ppg.d \
./Application/User/stm32l4xx_hal_msp.d \
./Application/User/stm32l4xx_it.d 


# Each subdirectory must supply rules for building sources it contributes
Application/User/%.o: ../Application/User/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DUSE_HAL_DRIVER -DSTM32L412xx -I"E:/TFG/SW/SW/Inc" -I"E:/TFG/SW/SW/Drivers/STM32L4xx_HAL_Driver/Inc" -I"E:/TFG/SW/SW/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy" -I"E:/TFG/SW/SW/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"E:/TFG/SW/SW/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/User/gpio.o: E:/TFG/SW/SW/Src/gpio.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DUSE_HAL_DRIVER -DSTM32L412xx -I"E:/TFG/SW/SW/Inc" -I"E:/TFG/SW/SW/Drivers/STM32L4xx_HAL_Driver/Inc" -I"E:/TFG/SW/SW/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy" -I"E:/TFG/SW/SW/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"E:/TFG/SW/SW/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/User/i2c.o: E:/TFG/SW/SW/Src/i2c.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DUSE_HAL_DRIVER -DSTM32L412xx -I"E:/TFG/SW/SW/Inc" -I"E:/TFG/SW/SW/Drivers/STM32L4xx_HAL_Driver/Inc" -I"E:/TFG/SW/SW/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy" -I"E:/TFG/SW/SW/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"E:/TFG/SW/SW/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/User/main.o: E:/TFG/SW/SW/Src/main.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DUSE_HAL_DRIVER -DSTM32L412xx -I"E:/TFG/SW/SW/Inc" -I"E:/TFG/SW/SW/Drivers/STM32L4xx_HAL_Driver/Inc" -I"E:/TFG/SW/SW/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy" -I"E:/TFG/SW/SW/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"E:/TFG/SW/SW/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


