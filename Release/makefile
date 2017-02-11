################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
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

# Add inputs and outputs from these tool invocations to the build variables 
LSS += \
RCCar.lss \

FLASH_IMAGE += \
RCCar.hex \

EEPROM_IMAGE += \
RCCar.eep \

SIZEDUMMY += \
sizedummy \


# All Target
all: RCCar.elf secondary-outputs

# Tool invocations
RCCar.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C Linker'
	avr-gcc -Wl,-Map,RCCar.map -mmcu=atmega8 -o "RCCar.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

RCCar.lss: RCCar.elf
	@echo 'Invoking: AVR Create Extended Listing'
	-avr-objdump -h -S RCCar.elf  >"RCCar.lss"
	@echo 'Finished building: $@'
	@echo ' '

RCCar.hex: RCCar.elf
	@echo 'Create Flash image (ihex format)'
	-avr-objcopy -R .eeprom -O ihex RCCar.elf  "RCCar.hex"
	@echo 'Finished building: $@'
	@echo ' '

RCCar.eep: RCCar.elf
	@echo 'Create eeprom image (ihex format)'
	-avr-objcopy -j .eeprom --no-change-warnings --change-section-lma .eeprom=0 -O ihex RCCar.elf  "RCCar.eep"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: RCCar.elf
	@echo 'Invoking: Print Size'
	-avr-size --format=avr --mcu=atmega8 RCCar.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(FLASH_IMAGE)$(ELFS)$(OBJS)$(ASM_DEPS)$(EEPROM_IMAGE)$(S_DEPS)$(SIZEDUMMY)$(S_UPPER_DEPS)$(LSS)$(C_DEPS) RCCar.elf
	-@echo ' '

secondary-outputs: $(LSS) $(FLASH_IMAGE) $(EEPROM_IMAGE) $(SIZEDUMMY)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
