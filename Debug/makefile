################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F7xx_HAL_Driver/Src/subdir.mk
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
Cube_QSPI_SDRAM_LTDC.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Cube_QSPI_SDRAM_LTDC.list \

OBJCOPY_BIN += \
Cube_QSPI_SDRAM_LTDC.bin \


# All Target
all: Cube_QSPI_SDRAM_LTDC.elf secondary-outputs

# Tool invocations
Cube_QSPI_SDRAM_LTDC.elf: $(OBJS) $(USER_OBJS) C:\Users\Michael\STM32CubeIDE\Examples_Workspace\Cube_QSPI_SDRAM_LTDC\STM32F750N8HX_FLASH.ld
	arm-none-eabi-gcc -o "Cube_QSPI_SDRAM_LTDC.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m7 -T"C:\Users\Michael\STM32CubeIDE\Examples_Workspace\Cube_QSPI_SDRAM_LTDC\STM32F750N8HX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Cube_QSPI_SDRAM_LTDC.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Cube_QSPI_SDRAM_LTDC.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Cube_QSPI_SDRAM_LTDC.list"
	@echo 'Finished building: $@'
	@echo ' '

Cube_QSPI_SDRAM_LTDC.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "Cube_QSPI_SDRAM_LTDC.bin"
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
