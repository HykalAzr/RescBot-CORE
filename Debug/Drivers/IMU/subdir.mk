################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/IMU/mpu6050.c 

OBJS += \
./Drivers/IMU/mpu6050.o 

C_DEPS += \
./Drivers/IMU/mpu6050.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/IMU/%.o Drivers/IMU/%.su Drivers/IMU/%.cyclo: ../Drivers/IMU/%.c Drivers/IMU/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I"C:/Users/HAIKAL/OneDrive/Documents/Education/Udemy/Mastering RTOS/Workspace/RTOS_workspace/FYP_STM32F103RB/Middlewares/Third_Party/SEGGER/Config" -I"C:/Users/HAIKAL/OneDrive/Documents/Education/Udemy/Mastering RTOS/Workspace/RTOS_workspace/FYP_STM32F103RB/Middlewares/Third_Party/SEGGER/OS" -I"C:/Users/HAIKAL/OneDrive/Documents/Education/Udemy/Mastering RTOS/Workspace/RTOS_workspace/FYP_STM32F103RB/Middlewares/Third_Party/SEGGER/SEGGER" -I"C:/Users/HAIKAL/OneDrive/Documents/Education/Udemy/Mastering RTOS/Workspace/RTOS_workspace/FYP_STM32F103RB/Drivers/LoRa" -I"C:/Users/HAIKAL/OneDrive/Documents/Education/Udemy/Mastering RTOS/Workspace/RTOS_workspace/FYP_STM32F103RB/Drivers/LCD" -I"C:/Users/HAIKAL/OneDrive/Documents/Education/Udemy/Mastering RTOS/Workspace/RTOS_workspace/FYP_STM32F103RB/Drivers/GPS" -I"C:/Users/HAIKAL/OneDrive/Documents/Education/Udemy/Mastering RTOS/Workspace/RTOS_workspace/FYP_STM32F103RB/Drivers/IMU" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-IMU

clean-Drivers-2f-IMU:
	-$(RM) ./Drivers/IMU/mpu6050.cyclo ./Drivers/IMU/mpu6050.d ./Drivers/IMU/mpu6050.o ./Drivers/IMU/mpu6050.su

.PHONY: clean-Drivers-2f-IMU

