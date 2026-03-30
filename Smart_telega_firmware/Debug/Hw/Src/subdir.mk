################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Hw/Src/hw_gpio_if.c \
../Hw/Src/hw_uart_if.c 

OBJS += \
./Hw/Src/hw_gpio_if.o \
./Hw/Src/hw_uart_if.o 

C_DEPS += \
./Hw/Src/hw_gpio_if.d \
./Hw/Src/hw_uart_if.d 


# Each subdirectory must supply rules for building sources it contributes
Hw/Src/%.o Hw/Src/%.su Hw/Src/%.cyclo: ../Hw/Src/%.c Hw/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Hw/Inc -I../App/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Hw-2f-Src

clean-Hw-2f-Src:
	-$(RM) ./Hw/Src/hw_gpio_if.cyclo ./Hw/Src/hw_gpio_if.d ./Hw/Src/hw_gpio_if.o ./Hw/Src/hw_gpio_if.su ./Hw/Src/hw_uart_if.cyclo ./Hw/Src/hw_uart_if.d ./Hw/Src/hw_uart_if.o ./Hw/Src/hw_uart_if.su

.PHONY: clean-Hw-2f-Src

