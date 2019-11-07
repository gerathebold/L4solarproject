################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c 

OBJS += \
./Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.o 

C_DEPS += \
./Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/%.o: ../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DUSE_FULL_LL_DRIVER '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32L476xx -I"C:/Users/Gerardo/workbench-workspace/L4FreeRTOS/Inc" -I"C:/Users/Gerardo/workbench-workspace/L4FreeRTOS/Drivers/STM32L4xx_HAL_Driver/Inc" -I"C:/Users/Gerardo/workbench-workspace/L4FreeRTOS/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Gerardo/workbench-workspace/L4FreeRTOS/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"C:/Users/Gerardo/workbench-workspace/L4FreeRTOS/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"C:/Users/Gerardo/workbench-workspace/L4FreeRTOS/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Users/Gerardo/workbench-workspace/L4FreeRTOS/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:/Users/Gerardo/workbench-workspace/L4FreeRTOS/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


