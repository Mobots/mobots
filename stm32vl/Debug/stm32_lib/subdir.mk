################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_UPPER_SRCS += \
../stm32_lib/startup_stm32f10x_md_vl.S 

OBJS += \
./stm32_lib/startup_stm32f10x_md_vl.o 

S_UPPER_DEPS += \
./stm32_lib/startup_stm32f10x_md_vl.d 


# Each subdirectory must supply rules for building sources it contributes
stm32_lib/%.o: ../stm32_lib/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Sourcery Linux GCC Assembler'
	arm-none-eabi-gcc -x assembler-with-cpp -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m4 -mthumb -g3 -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


