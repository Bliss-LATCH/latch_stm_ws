################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../CAN/Src/can_device.c 

OBJS += \
./CAN/Src/can_device.o 

C_DEPS += \
./CAN/Src/can_device.d 


# Each subdirectory must supply rules for building sources it contributes
CAN/Src/%.o CAN/Src/%.su CAN/Src/%.cyclo: ../CAN/Src/%.c CAN/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../CAN/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-CAN-2f-Src

clean-CAN-2f-Src:
	-$(RM) ./CAN/Src/can_device.cyclo ./CAN/Src/can_device.d ./CAN/Src/can_device.o ./CAN/Src/can_device.su

.PHONY: clean-CAN-2f-Src

