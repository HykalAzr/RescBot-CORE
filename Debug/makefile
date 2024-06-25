################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Middlewares/Third_Party/SEGGER/SEGGER/Syscalls/subdir.mk
-include Middlewares/Third_Party/SEGGER/SEGGER/subdir.mk
-include Middlewares/Third_Party/SEGGER/Rec/subdir.mk
-include Middlewares/Third_Party/SEGGER/OS/subdir.mk
-include Middlewares/Third_Party/SEGGER/Config/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/subdir.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include Drivers/LoRa/subdir.mk
-include Drivers/LCD/subdir.mk
-include Drivers/IMU/subdir.mk
-include Drivers/GPS/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := FYP_STM32F103RB
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
FYP_STM32F103RB.elf \

MAP_FILES += \
FYP_STM32F103RB.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
FYP_STM32F103RB.list \


# All Target
all: main-build

# Main-build Target
main-build: FYP_STM32F103RB.elf secondary-outputs

# Tool invocations
FYP_STM32F103RB.elf FYP_STM32F103RB.map: $(OBJS) $(USER_OBJS) C:\Users\HAIKAL\OneDrive\Documents\Education\Udemy\Mastering\ RTOS\Workspace\RTOS_workspace\FYP_STM32F103RB\STM32F103RBTX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "FYP_STM32F103RB.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\HAIKAL\OneDrive\Documents\Education\Udemy\Mastering RTOS\Workspace\RTOS_workspace\FYP_STM32F103RB\STM32F103RBTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="FYP_STM32F103RB.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -u _printf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

FYP_STM32F103RB.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "FYP_STM32F103RB.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) FYP_STM32F103RB.elf FYP_STM32F103RB.list FYP_STM32F103RB.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
