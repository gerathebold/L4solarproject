################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/ADCDriver.c \
../Src/BuckServiceLayer.c \
../Src/GPIODriver.c \
../Src/HMIServiceLayer.c \
../Src/I2CDriver.c \
../Src/RTCDriver.c \
../Src/RTCServiceLayer.c \
../Src/SPIDriver.c \
../Src/SwitchCircuitServiceLayer.c \
../Src/TimerDriver.c \
../Src/freertos.c \
../Src/main.c \
../Src/stm32l4xx_hal_msp.c \
../Src/stm32l4xx_hal_timebase_TIM.c \
../Src/stm32l4xx_it.c \
../Src/system_stm32l4xx.c \
../Src/taskBMScommunication.c \
../Src/taskBattConfig.c \
../Src/taskBuckControl.c \
../Src/taskEEPROMManage.c \
../Src/taskHMI.c \
../Src/taskMainStateMachine.c \
../Src/taskSleepManage.c 

OBJS += \
./Src/ADCDriver.o \
./Src/BuckServiceLayer.o \
./Src/GPIODriver.o \
./Src/HMIServiceLayer.o \
./Src/I2CDriver.o \
./Src/RTCDriver.o \
./Src/RTCServiceLayer.o \
./Src/SPIDriver.o \
./Src/SwitchCircuitServiceLayer.o \
./Src/TimerDriver.o \
./Src/freertos.o \
./Src/main.o \
./Src/stm32l4xx_hal_msp.o \
./Src/stm32l4xx_hal_timebase_TIM.o \
./Src/stm32l4xx_it.o \
./Src/system_stm32l4xx.o \
./Src/taskBMScommunication.o \
./Src/taskBattConfig.o \
./Src/taskBuckControl.o \
./Src/taskEEPROMManage.o \
./Src/taskHMI.o \
./Src/taskMainStateMachine.o \
./Src/taskSleepManage.o 

C_DEPS += \
./Src/ADCDriver.d \
./Src/BuckServiceLayer.d \
./Src/GPIODriver.d \
./Src/HMIServiceLayer.d \
./Src/I2CDriver.d \
./Src/RTCDriver.d \
./Src/RTCServiceLayer.d \
./Src/SPIDriver.d \
./Src/SwitchCircuitServiceLayer.d \
./Src/TimerDriver.d \
./Src/freertos.d \
./Src/main.d \
./Src/stm32l4xx_hal_msp.d \
./Src/stm32l4xx_hal_timebase_TIM.d \
./Src/stm32l4xx_it.d \
./Src/system_stm32l4xx.d \
./Src/taskBMScommunication.d \
./Src/taskBattConfig.d \
./Src/taskBuckControl.d \
./Src/taskEEPROMManage.d \
./Src/taskHMI.d \
./Src/taskMainStateMachine.d \
./Src/taskSleepManage.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DUSE_FULL_LL_DRIVER '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32L476xx -I"C:/Users/Gerardo/workbench-workspace/L4FreeRTOS/Inc" -I"C:/Users/Gerardo/workbench-workspace/L4FreeRTOS/Drivers/STM32L4xx_HAL_Driver/Inc" -I"C:/Users/Gerardo/workbench-workspace/L4FreeRTOS/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Gerardo/workbench-workspace/L4FreeRTOS/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"C:/Users/Gerardo/workbench-workspace/L4FreeRTOS/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"C:/Users/Gerardo/workbench-workspace/L4FreeRTOS/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Users/Gerardo/workbench-workspace/L4FreeRTOS/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:/Users/Gerardo/workbench-workspace/L4FreeRTOS/Drivers/CMSIS/Include" -I"C:/Users/Gerardo/workbench-workspace/L4FreeRTOS/Inc"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


