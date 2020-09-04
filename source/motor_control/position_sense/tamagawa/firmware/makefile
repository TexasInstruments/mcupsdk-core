################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

MCU_PLUS_SDK_PATH?=$(abspath ../../../../..)
CCS_PATH?=C:/ti/ccs1011/ccs/ccs_base/pru/include
CG_TOOL_ROOT := C:/ti/ti-cgt-pru_2.3.3

GEN_OPTS__FLAG := 
GEN_CMDS__FLAG := 

ORDERED_OBJS += \
"tamagawa_receiver.obj" \
"tamagawa_receiver.cmd" \
$(GEN_CMDS__FLAG) \
-llibc.a \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# Add inputs and outputs from these tool invocations to the build variables 
CMD_SRCS += \
tamagawa_receiver.cmd 

C_SRCS += \
tamagawa_receiver.c 

C_DEPS += \
tamagawa_receiver.d 

OBJS += \
tamagawa_receiver.obj 

OBJS__QUOTED += \
"tamagawa_receiver.obj" 

C_DEPS__QUOTED += \
"tamagawa_receiver.d" 

C_SRCS__QUOTED += \
"tamagawa_receiver.c" 

EXE_OUTPUTS += \
tamagawa_receiver.out \

EXE_OUTPUTS__QUOTED += \
"tamagawa_receiver.out" \

BIN_OUTPUTS += \
tamagawa_receiver.hex \

BIN_OUTPUTS__QUOTED += \
"tamagawa_receiver.hex" \

# Each subdirectory must supply rules for building sources it contributes
tamagawa_receiver.obj: $(MCU_PLUS_SDK_PATH)/source/motor_control/position_sense/tamagawa/firmware/tamagawa_receiver.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: PRU Compiler'
	"$(CG_TOOL_ROOT)/bin/clpru" -v3 -O0 --include_path="$(CCS_PATH)" --include_path="$(MCU_PLUS_SDK_PATH)/source/motor_control/position_sense/tamagawa/firmware" --include_path="$(CG_TOOL_ROOT)/include" --define=icss1 --define=pru0 --diag_warning=225 --diag_wrap=off --display_error_number --endian=little --hardware_mac=on --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '
	
# All Target
all: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "tamagawa_receiver.out"

# Tool invocations
tamagawa_receiver.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@echo 'Building target: "$@"'
	@echo 'Invoking: PRU Linker'
	"$(CG_TOOL_ROOT)/bin/clpru" -v3 -O0 --diag_warning=225 --diag_wrap=off --display_error_number --endian=little --hardware_mac=on -z -m"tamagawa_receiver.map" --heap_size=0 --stack_size=0 -i"$(CG_TOOL_ROOT)/lib" -i"$(CG_TOOL_ROOT)/include" --reread_libs --define=ICSS1_PRU_CORE=1 --diag_wrap=off --display_error_number --warn_sections --xml_link_info="tamagawa_receiver_linkInfo.xml" -o "tamagawa_receiver.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '
	@$(MAKE) --no-print-directory post-build

tamagawa_receiver.hex: $(EXE_OUTPUTS)
	@echo 'Building secondary target: "$@"'
	@echo 'Invoking: PRU Hex Utility'
	"$(CG_TOOL_ROOT)/bin/hexpru" --diag_wrap=off -o "tamagawa_receiver.hex" $(EXE_OUTPUTS__QUOTED)
	@echo 'Finished building secondary target: "$@"'
	@echo ' '
	@$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(BIN_OUTPUTS__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) "tamagawa_receiver.obj" 
	-$(RM) "tamagawa_receiver.d" 
	-@echo 'Finished clean'
	-@echo ' '

post-build:
	$(CG_TOOL_ROOT)\bin\hexpru.exe tamagawa_receiver_hexpru.cmd tamagawa_receiver.out && $(MCU_PLUS_SDK_PATH)\tools\bin2header\bin2header.exe text.bin tamagawa_receiver_bin.h TamagawaFirmware 4
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

