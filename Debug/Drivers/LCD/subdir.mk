################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/LCD/liquidcrystal_i2c.c 

OBJS += \
./Drivers/LCD/liquidcrystal_i2c.o 

C_DEPS += \
./Drivers/LCD/liquidcrystal_i2c.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/LCD/%.o Drivers/LCD/%.su Drivers/LCD/%.cyclo: ../Drivers/LCD/%.c Drivers/LCD/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I"C:/Users/HAIKAL/OneDrive/Documents/Education/Udemy/Mastering RTOS/Workspace/RTOS_workspace/FYP_STM32F103RB/Middlewares/Third_Party/SEGGER/Config" -I"C:/Users/HAIKAL/OneDrive/Documents/Education/Udemy/Mastering RTOS/Workspace/RTOS_workspace/FYP_STM32F103RB/Middlewares/Third_Party/SEGGER/OS" -I"C:/Users/HAIKAL/OneDrive/Documents/Education/Udemy/Mastering RTOS/Workspace/RTOS_workspace/FYP_STM32F103RB/Middlewares/Third_Party/SEGGER/SEGGER" -I"C:/Users/HAIKAL/OneDrive/Documents/Education/Udemy/Mastering RTOS/Workspace/RTOS_workspace/FYP_STM32F103RB/Drivers/LoRa" -I"C:/Users/HAIKAL/OneDrive/Documents/Education/Udemy/Mastering RTOS/Workspace/RTOS_workspace/FYP_STM32F103RB/Drivers/LCD" -I"C:/Users/HAIKAL/OneDrive/Documents/Education/Udemy/Mastering RTOS/Workspace/RTOS_workspace/FYP_STM32F103RB/Drivers/GPS" -I"C:/Users/HAIKAL/OneDrive/Documents/Education/Udemy/Mastering RTOS/Workspace/RTOS_workspace/FYP_STM32F103RB/Drivers/IMU" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-LCD

clean-Drivers-2f-LCD:
	-$(RM) ./Drivers/LCD/liquidcrystal_i2c.cyclo ./Drivers/LCD/liquidcrystal_i2c.d ./Drivers/LCD/liquidcrystal_i2c.o ./Drivers/LCD/liquidcrystal_i2c.su

.PHONY: clean-Drivers-2f-LCD

