################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../crc.c \
../led.c \
../main.c \
../mousesensor.c \
../movement.c \
../printf.c \
../printf_gdb.c \
../protocol.c \
../servo.c \
../spi_1.c \
../usart.c 

OBJS += \
./crc.o \
./led.o \
./main.o \
./mousesensor.o \
./movement.o \
./printf.o \
./printf_gdb.o \
./protocol.o \
./servo.o \
./spi_1.o \
./usart.o 

C_DEPS += \
./crc.d \
./led.d \
./main.d \
./mousesensor.d \
./movement.d \
./printf.d \
./printf_gdb.d \
./protocol.d \
./servo.d \
./spi_1.d \
./usart.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Sourcery Linux GCC C Compiler'
	arm-none-eabi-gcc -DUSE_STM32_DISCOVERY -DUSE_STDPERIPH_DRIVER -DSTM32F10X_MD_VL -I"/home/simon/Workspace/stm32vl/stm32_lib/inc" -I"/home/simon/Workspace/stm32vl/libfixmath" -O0 -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m3 -mthumb -g -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


