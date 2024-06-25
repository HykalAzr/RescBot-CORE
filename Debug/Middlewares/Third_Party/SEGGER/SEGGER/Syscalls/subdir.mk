################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_GCC.c 

OBJS += \
./Middlewares/Third_Party/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_GCC.o 

C_DEPS += \
./Middlewares/Third_Party/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_GCC.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/SEGGER/SEGGER/Syscalls/%.o Middlewares/Third_Party/SEGGER/SEGGER/Syscalls/%.su Middlewares/Third_Party/SEGGER/SEGGER/Syscalls/%.cyclo: ../Middlewares/Third_Party/SEGGER/SEGGER/Syscalls/%.c Middlewares/Third_Party/SEGGER/SEGGER/Syscalls/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I"C:/Users/HAIKAL/OneDrive/Documents/Education/Udemy/Mastering RTOS/Workspace/RTOS_workspace/FYP_STM32F103RB/Middlewares/Third_Party/SEGGER/Config" -I"C:/Users/HAIKAL/OneDrive/Documents/Education/Udemy/Mastering RTOS/Workspace/RTOS_workspace/FYP_STM32F103RB/Middlewares/Third_Party/SEGGER/OS" -I"C:/Users/HAIKAL/OneDrive/Documents/Education/Udemy/Mastering RTOS/Workspace/RTOS_workspace/FYP_STM32F103RB/Middlewares/Third_Party/SEGGER/SEGGER" -I"C:/Users/HAIKAL/OneDrive/Documents/Education/Udemy/Mastering RTOS/Workspace/RTOS_workspace/FYP_STM32F103RB/Drivers/LoRa" -I"C:/Users/HAIKAL/OneDrive/Documents/Education/Udemy/Mastering RTOS/Workspace/RTOS_workspace/FYP_STM32F103RB/Drivers/LCD" -I"C:/Users/HAIKAL/OneDrive/Documents/Education/Udemy/Mastering RTOS/Workspace/RTOS_workspace/FYP_STM32F103RB/Drivers/GPS" -I"C:/Users/HAIKAL/OneDrive/Documents/Education/Udemy/Mastering RTOS/Workspace/RTOS_workspace/FYP_STM32F103RB/Drivers/IMU" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-SEGGER-2f-SEGGER-2f-Syscalls

clean-Middlewares-2f-Third_Party-2f-SEGGER-2f-SEGGER-2f-Syscalls:
	-$(RM) ./Middlewares/Third_Party/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_GCC.cyclo ./Middlewares/Third_Party/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_GCC.d ./Middlewares/Third_Party/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_GCC.o ./Middlewares/Third_Party/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_GCC.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-SEGGER-2f-SEGGER-2f-Syscalls

