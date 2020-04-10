################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include startup/subdir.mk
-include src/subdir.mk
-include drivers/src/subdir.mk
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
FirstDriver-1.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
FirstDriver-1.list \

OBJCOPY_BIN += \
FirstDriver-1.bin \


# All Target
all: FirstDriver-1.elf secondary-outputs

# Tool invocations
<<<<<<< HEAD
FirstDriver-1.elf: $(OBJS) $(USER_OBJS) /home/fernandoks/Desktop/CubeWorkbench/FirstDriver-1/LinkerScript.ld
	arm-none-eabi-gcc -o "FirstDriver-1.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"/home/fernandoks/Desktop/CubeWorkbench/FirstDriver-1/LinkerScript.ld" --specs=nosys.specs -Wl,-Map="FirstDriver-1.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
=======
<<<<<<< HEAD
FirstDriver.elf: $(OBJS) $(USER_OBJS) /home/quentin/STM32CubeIDE/dev/FirstDriver/LinkerScript.ld
	arm-none-eabi-gcc -o "FirstDriver.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"/home/quentin/STM32CubeIDE/dev/FirstDriver/LinkerScript.ld" --specs=nosys.specs -Wl,-Map="FirstDriver.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
=======
<<<<<<< HEAD
FirstDriver-1.elf: $(OBJS) $(USER_OBJS) /home/fernandoks/Desktop/CubeWorkbench/FirstDriver-1/LinkerScript.ld
	arm-none-eabi-gcc -o "FirstDriver-1.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"/home/fernandoks/Desktop/CubeWorkbench/FirstDriver-1/LinkerScript.ld" --specs=nosys.specs -Wl,-Map="FirstDriver-1.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
=======
FirstDriver.elf: $(OBJS) $(USER_OBJS) /home/fernandoks/Desktop/CubeWorkbench/FirstDriver/LinkerScript.ld
	arm-none-eabi-gcc -o "FirstDriver.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"/home/fernandoks/Desktop/CubeWorkbench/FirstDriver/LinkerScript.ld" --specs=nosys.specs -Wl,-Map="FirstDriver.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
>>>>>>> 7bfe1162eb0f6d92f05b504ea2a5711bb406f19e
>>>>>>> master
>>>>>>> master
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

FirstDriver-1.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "FirstDriver-1.list"
	@echo 'Finished building: $@'
	@echo ' '

FirstDriver-1.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "FirstDriver-1.bin"
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
