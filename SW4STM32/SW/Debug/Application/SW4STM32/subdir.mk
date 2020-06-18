################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
E:/TFG/SW/SW/SW4STM32/startup_stm32l412xx.s 

C_SRCS += \
E:/TFG/SW/SW/SW4STM32/syscalls.c 

OBJS += \
./Application/SW4STM32/startup_stm32l412xx.o \
./Application/SW4STM32/syscalls.o 

C_DEPS += \
./Application/SW4STM32/syscalls.d 


# Each subdirectory must supply rules for building sources it contributes
Application/SW4STM32/startup_stm32l412xx.o: E:/TFG/SW/SW/SW4STM32/startup_stm32l412xx.s
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Assembler'
	@echo $(PWD)
	arm-none-eabi-as -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/SW4STM32/syscalls.o: E:/TFG/SW/SW/SW4STM32/syscalls.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DUSE_HAL_DRIVER -DSTM32L412xx -I"E:/TFG/SW/SW/Inc" -I"E:/TFG/SW/SW/Drivers/STM32L4xx_HAL_Driver/Inc" -I"E:/TFG/SW/SW/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy" -I"E:/TFG/SW/SW/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"E:/TFG/SW/SW/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


