################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32L4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
LED_PWM_ADC.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
LED_PWM_ADC.list \

OBJCOPY_BIN += \
LED_PWM_ADC.bin \


# All Target
all: LED_PWM_ADC.elf secondary-outputs

# Tool invocations
LED_PWM_ADC.elf: $(OBJS) $(USER_OBJS) /home/jfbg-linux/STM32CubeIDE/workspace_1.3.0/LED_PWM_ADC/STM32L476RGTX_FLASH.ld
	arm-none-eabi-gcc -o "LED_PWM_ADC.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"/home/jfbg-linux/STM32CubeIDE/workspace_1.3.0/LED_PWM_ADC/STM32L476RGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="LED_PWM_ADC.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

LED_PWM_ADC.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "LED_PWM_ADC.list"
	@echo 'Finished building: $@'
	@echo ' '

LED_PWM_ADC.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "LED_PWM_ADC.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
