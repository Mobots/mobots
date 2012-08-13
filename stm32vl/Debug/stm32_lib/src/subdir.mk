################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../stm32_lib/src/core_cm3.c \
../stm32_lib/src/misc.c \
../stm32_lib/src/stm32f10x_adc.c \
../stm32_lib/src/stm32f10x_bkp.c \
../stm32_lib/src/stm32f10x_can.c \
../stm32_lib/src/stm32f10x_cec.c \
../stm32_lib/src/stm32f10x_crc.c \
../stm32_lib/src/stm32f10x_dac.c \
../stm32_lib/src/stm32f10x_dbgmcu.c \
../stm32_lib/src/stm32f10x_dma.c \
../stm32_lib/src/stm32f10x_exti.c \
../stm32_lib/src/stm32f10x_flash.c \
../stm32_lib/src/stm32f10x_fsmc.c \
../stm32_lib/src/stm32f10x_gpio.c \
../stm32_lib/src/stm32f10x_i2c.c \
../stm32_lib/src/stm32f10x_it.c \
../stm32_lib/src/stm32f10x_iwdg.c \
../stm32_lib/src/stm32f10x_pwr.c \
../stm32_lib/src/stm32f10x_rcc.c \
../stm32_lib/src/stm32f10x_rtc.c \
../stm32_lib/src/stm32f10x_sdio.c \
../stm32_lib/src/stm32f10x_spi.c \
../stm32_lib/src/stm32f10x_tim.c \
../stm32_lib/src/stm32f10x_usart.c \
../stm32_lib/src/stm32f10x_wwdg.c \
../stm32_lib/src/system_stm32f10x.c 

OBJS += \
./stm32_lib/src/core_cm3.o \
./stm32_lib/src/misc.o \
./stm32_lib/src/stm32f10x_adc.o \
./stm32_lib/src/stm32f10x_bkp.o \
./stm32_lib/src/stm32f10x_can.o \
./stm32_lib/src/stm32f10x_cec.o \
./stm32_lib/src/stm32f10x_crc.o \
./stm32_lib/src/stm32f10x_dac.o \
./stm32_lib/src/stm32f10x_dbgmcu.o \
./stm32_lib/src/stm32f10x_dma.o \
./stm32_lib/src/stm32f10x_exti.o \
./stm32_lib/src/stm32f10x_flash.o \
./stm32_lib/src/stm32f10x_fsmc.o \
./stm32_lib/src/stm32f10x_gpio.o \
./stm32_lib/src/stm32f10x_i2c.o \
./stm32_lib/src/stm32f10x_it.o \
./stm32_lib/src/stm32f10x_iwdg.o \
./stm32_lib/src/stm32f10x_pwr.o \
./stm32_lib/src/stm32f10x_rcc.o \
./stm32_lib/src/stm32f10x_rtc.o \
./stm32_lib/src/stm32f10x_sdio.o \
./stm32_lib/src/stm32f10x_spi.o \
./stm32_lib/src/stm32f10x_tim.o \
./stm32_lib/src/stm32f10x_usart.o \
./stm32_lib/src/stm32f10x_wwdg.o \
./stm32_lib/src/system_stm32f10x.o 

C_DEPS += \
./stm32_lib/src/core_cm3.d \
./stm32_lib/src/misc.d \
./stm32_lib/src/stm32f10x_adc.d \
./stm32_lib/src/stm32f10x_bkp.d \
./stm32_lib/src/stm32f10x_can.d \
./stm32_lib/src/stm32f10x_cec.d \
./stm32_lib/src/stm32f10x_crc.d \
./stm32_lib/src/stm32f10x_dac.d \
./stm32_lib/src/stm32f10x_dbgmcu.d \
./stm32_lib/src/stm32f10x_dma.d \
./stm32_lib/src/stm32f10x_exti.d \
./stm32_lib/src/stm32f10x_flash.d \
./stm32_lib/src/stm32f10x_fsmc.d \
./stm32_lib/src/stm32f10x_gpio.d \
./stm32_lib/src/stm32f10x_i2c.d \
./stm32_lib/src/stm32f10x_it.d \
./stm32_lib/src/stm32f10x_iwdg.d \
./stm32_lib/src/stm32f10x_pwr.d \
./stm32_lib/src/stm32f10x_rcc.d \
./stm32_lib/src/stm32f10x_rtc.d \
./stm32_lib/src/stm32f10x_sdio.d \
./stm32_lib/src/stm32f10x_spi.d \
./stm32_lib/src/stm32f10x_tim.d \
./stm32_lib/src/stm32f10x_usart.d \
./stm32_lib/src/stm32f10x_wwdg.d \
./stm32_lib/src/system_stm32f10x.d 


# Each subdirectory must supply rules for building sources it contributes
stm32_lib/src/%.o: ../stm32_lib/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Sourcery Linux GCC C Compiler'
	arm-none-eabi-gcc -DUSE_STM32_DISCOVERY -DUSE_STDPERIPH_DRIVER -DSTM32F10X_MD_VL -I"/home/simon/Workspace/stm32vl/stm32_lib/inc" -I"/home/simon/Workspace/stm32vl/libfixmath" -O0 -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m4 -mthumb -g3 -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


