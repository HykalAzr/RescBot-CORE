################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/LoRa/LoRa.c 

OBJS += \
./Drivers/LoRa/LoRa.o 

C_DEPS += \
./Drivers/LoRa/LoRa.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/LoRa/%.o Drivers/LoRa/%.su Drivers/LoRa/%.cyclo: ../Drivers/LoRa/%.c Drivers/LoRa/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I"C:/Users/HAIKAL/OneDrive/Documents/Education/Udemy/Mastering RTOS/Workspace/RTOS_workspace/FYP_STM32F103RB/Middlewares/Third_Party/SEGGER/Config" -I"C:/Users/HAIKAL/OneDrive/Documents/Education/Udemy/Mastering RTOS/Workspace/RTOS_workspace/FYP_STM32F103RB/Middlewares/Third_Party/SEGGER/OS" -I"C:/Users/HAIKAL/OneDrive/Documents/Education/Udemy/Mastering RTOS/Workspace/RTOS_workspace/FYP_STM32F103RB/Middlewares/Third_Party/SEGGER/SEGGER" -I"C:/Users/HAIKAL/OneDrive/Documents/Education/Udemy/Mastering RTOS/Workspace/RTOS_workspace/FYP_STM32F103RB/Drivers/LoRa" -I"C:/Users/HAIKAL/OneDrive/Documents/Education/Udemy/Mastering RTOS/Workspace/RTOS_workspace/FYP_STM32F103RB/Drivers/LCD" -I"C:/Users/HAIKAL/OneDrive/Documents/Education/Udemy/Mastering RTOS/Workspace/RTOS_workspace/FYP_STM32F103RB/Drivers/GPS" -I"C:/Users/HAIKAL/OneDrive/Documents/Education/Udemy/Mastering RTOS/Workspace/RTOS_workspace/FYP_STM32F103RB/Drivers/IMU" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-LoRa

clean-Drivers-2f-LoRa:
	-$(RM) ./Drivers/LoRa/LoRa.cyclo ./Drivers/LoRa/LoRa.d ./Drivers/LoRa/LoRa.o ./Drivers/LoRa/LoRa.su

.PHONY: clean-Drivers-2f-LoRa

