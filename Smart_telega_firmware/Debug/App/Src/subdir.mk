################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../App/Src/app_mainloop.c \
../App/Src/app_timebase.c \
../App/Src/cmd_dispatcher.c \
../App/Src/cmd_handlers.c \
../App/Src/cmd_runtime.c \
../App/Src/debug_hooks.c \
../App/Src/diag.c \
../App/Src/led_mapper.c \
../App/Src/led_model.c \
../App/Src/protocol_crc16.c \
../App/Src/protocol_frame.c \
../App/Src/switch_debounce.c \
../App/Src/switch_scan.c \
../App/Src/switch_snapshot.c \
../App/Src/ws2812_if.c \
../App/Src/ws2812_stub_backend.c 

OBJS += \
./App/Src/app_mainloop.o \
./App/Src/app_timebase.o \
./App/Src/cmd_dispatcher.o \
./App/Src/cmd_handlers.o \
./App/Src/cmd_runtime.o \
./App/Src/debug_hooks.o \
./App/Src/diag.o \
./App/Src/led_mapper.o \
./App/Src/led_model.o \
./App/Src/protocol_crc16.o \
./App/Src/protocol_frame.o \
./App/Src/switch_debounce.o \
./App/Src/switch_scan.o \
./App/Src/switch_snapshot.o \
./App/Src/ws2812_if.o \
./App/Src/ws2812_stub_backend.o 

C_DEPS += \
./App/Src/app_mainloop.d \
./App/Src/app_timebase.d \
./App/Src/cmd_dispatcher.d \
./App/Src/cmd_handlers.d \
./App/Src/cmd_runtime.d \
./App/Src/debug_hooks.d \
./App/Src/diag.d \
./App/Src/led_mapper.d \
./App/Src/led_model.d \
./App/Src/protocol_crc16.d \
./App/Src/protocol_frame.d \
./App/Src/switch_debounce.d \
./App/Src/switch_scan.d \
./App/Src/switch_snapshot.d \
./App/Src/ws2812_if.d \
./App/Src/ws2812_stub_backend.d 


# Each subdirectory must supply rules for building sources it contributes
App/Src/%.o App/Src/%.su App/Src/%.cyclo: ../App/Src/%.c App/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Hw/Inc -I../App/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-App-2f-Src

clean-App-2f-Src:
	-$(RM) ./App/Src/app_mainloop.cyclo ./App/Src/app_mainloop.d ./App/Src/app_mainloop.o ./App/Src/app_mainloop.su ./App/Src/app_timebase.cyclo ./App/Src/app_timebase.d ./App/Src/app_timebase.o ./App/Src/app_timebase.su ./App/Src/cmd_dispatcher.cyclo ./App/Src/cmd_dispatcher.d ./App/Src/cmd_dispatcher.o ./App/Src/cmd_dispatcher.su ./App/Src/cmd_handlers.cyclo ./App/Src/cmd_handlers.d ./App/Src/cmd_handlers.o ./App/Src/cmd_handlers.su ./App/Src/cmd_runtime.cyclo ./App/Src/cmd_runtime.d ./App/Src/cmd_runtime.o ./App/Src/cmd_runtime.su ./App/Src/debug_hooks.cyclo ./App/Src/debug_hooks.d ./App/Src/debug_hooks.o ./App/Src/debug_hooks.su ./App/Src/diag.cyclo ./App/Src/diag.d ./App/Src/diag.o ./App/Src/diag.su ./App/Src/led_mapper.cyclo ./App/Src/led_mapper.d ./App/Src/led_mapper.o ./App/Src/led_mapper.su ./App/Src/led_model.cyclo ./App/Src/led_model.d ./App/Src/led_model.o ./App/Src/led_model.su ./App/Src/protocol_crc16.cyclo ./App/Src/protocol_crc16.d ./App/Src/protocol_crc16.o ./App/Src/protocol_crc16.su ./App/Src/protocol_frame.cyclo ./App/Src/protocol_frame.d ./App/Src/protocol_frame.o ./App/Src/protocol_frame.su ./App/Src/switch_debounce.cyclo ./App/Src/switch_debounce.d ./App/Src/switch_debounce.o ./App/Src/switch_debounce.su ./App/Src/switch_scan.cyclo ./App/Src/switch_scan.d ./App/Src/switch_scan.o ./App/Src/switch_scan.su ./App/Src/switch_snapshot.cyclo ./App/Src/switch_snapshot.d ./App/Src/switch_snapshot.o ./App/Src/switch_snapshot.su ./App/Src/ws2812_if.cyclo ./App/Src/ws2812_if.d ./App/Src/ws2812_if.o ./App/Src/ws2812_if.su ./App/Src/ws2812_stub_backend.cyclo ./App/Src/ws2812_stub_backend.d ./App/Src/ws2812_stub_backend.o ./App/Src/ws2812_stub_backend.su

.PHONY: clean-App-2f-Src

