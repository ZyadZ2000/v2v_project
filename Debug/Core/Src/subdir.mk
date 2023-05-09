################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/CAR_CTRL_program.c \
../Core/Src/CLCD_program.c \
../Core/Src/NMEA.c \
../Core/Src/application_tasks.c \
../Core/Src/freertos.c \
../Core/Src/gps.c \
../Core/Src/helper_functions.c \
../Core/Src/interrupts_callback.c \
../Core/Src/main.c \
../Core/Src/rsa_algorithm.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/uartRingBuffer.c \
../Core/Src/ultrasonic.c 

OBJS += \
./Core/Src/CAR_CTRL_program.o \
./Core/Src/CLCD_program.o \
./Core/Src/NMEA.o \
./Core/Src/application_tasks.o \
./Core/Src/freertos.o \
./Core/Src/gps.o \
./Core/Src/helper_functions.o \
./Core/Src/interrupts_callback.o \
./Core/Src/main.o \
./Core/Src/rsa_algorithm.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/uartRingBuffer.o \
./Core/Src/ultrasonic.o 

C_DEPS += \
./Core/Src/CAR_CTRL_program.d \
./Core/Src/CLCD_program.d \
./Core/Src/NMEA.d \
./Core/Src/application_tasks.d \
./Core/Src/freertos.d \
./Core/Src/gps.d \
./Core/Src/helper_functions.d \
./Core/Src/interrupts_callback.d \
./Core/Src/main.d \
./Core/Src/rsa_algorithm.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/uartRingBuffer.d \
./Core/Src/ultrasonic.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/CAR_CTRL_program.cyclo ./Core/Src/CAR_CTRL_program.d ./Core/Src/CAR_CTRL_program.o ./Core/Src/CAR_CTRL_program.su ./Core/Src/CLCD_program.cyclo ./Core/Src/CLCD_program.d ./Core/Src/CLCD_program.o ./Core/Src/CLCD_program.su ./Core/Src/NMEA.cyclo ./Core/Src/NMEA.d ./Core/Src/NMEA.o ./Core/Src/NMEA.su ./Core/Src/application_tasks.cyclo ./Core/Src/application_tasks.d ./Core/Src/application_tasks.o ./Core/Src/application_tasks.su ./Core/Src/freertos.cyclo ./Core/Src/freertos.d ./Core/Src/freertos.o ./Core/Src/freertos.su ./Core/Src/gps.cyclo ./Core/Src/gps.d ./Core/Src/gps.o ./Core/Src/gps.su ./Core/Src/helper_functions.cyclo ./Core/Src/helper_functions.d ./Core/Src/helper_functions.o ./Core/Src/helper_functions.su ./Core/Src/interrupts_callback.cyclo ./Core/Src/interrupts_callback.d ./Core/Src/interrupts_callback.o ./Core/Src/interrupts_callback.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/rsa_algorithm.cyclo ./Core/Src/rsa_algorithm.d ./Core/Src/rsa_algorithm.o ./Core/Src/rsa_algorithm.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/uartRingBuffer.cyclo ./Core/Src/uartRingBuffer.d ./Core/Src/uartRingBuffer.o ./Core/Src/uartRingBuffer.su ./Core/Src/ultrasonic.cyclo ./Core/Src/ultrasonic.d ./Core/Src/ultrasonic.o ./Core/Src/ultrasonic.su

.PHONY: clean-Core-2f-Src

