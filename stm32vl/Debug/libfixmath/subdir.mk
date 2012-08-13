################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../libfixmath/fix16.c \
../libfixmath/fix16_exp.c \
../libfixmath/fix16_sqrt.c \
../libfixmath/fix16_trig.c \
../libfixmath/fract32.c \
../libfixmath/uint32.c 

OBJS += \
./libfixmath/fix16.o \
./libfixmath/fix16_exp.o \
./libfixmath/fix16_sqrt.o \
./libfixmath/fix16_trig.o \
./libfixmath/fract32.o \
./libfixmath/uint32.o 

C_DEPS += \
./libfixmath/fix16.d \
./libfixmath/fix16_exp.d \
./libfixmath/fix16_sqrt.d \
./libfixmath/fix16_trig.d \
./libfixmath/fract32.d \
./libfixmath/uint32.d 


# Each subdirectory must supply rules for building sources it contributes
libfixmath/%.o: ../libfixmath/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Sourcery Linux GCC C Compiler'
	arm-none-eabi-gcc -DUSE_STM32_DISCOVERY -DUSE_STDPERIPH_DRIVER -DSTM32F10X_MD_VL -I"/home/simon/Workspace/stm32vl/stm32_lib/inc" -I"/home/simon/Workspace/stm32vl/libfixmath" -O0 -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m3 -mthumb -g -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


