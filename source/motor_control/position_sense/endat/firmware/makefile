################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe
MCU_PLUS_SDK_PATH?=$(abspath ../../../../..)
CCS_PATH?=C:/ti/ccs1200/ccs/ccs_base/pru/include
CG_TOOL_ROOT := C:/ti/ti-cgt-pru_2.3.3

#Set the default mode
CHANNELS ?= SINGLE

ifeq ($(CHANNELS),MULTI)
MULTI_CHANNEL=ENABLE_MULTI_CHANNEL
endif

ifeq ($(CHANNELS),SINGLE)
MULTI_CHANNEL=0
endif

GEN_OPTS__FLAG :=
GEN_CMDS__FLAG :=

ORDERED_OBJS += \
"endat_main.obj" \
"endat_diagnostic.cmd" \
$(GEN_CMDS__FLAG) \
-llibc.a \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

SUBDIRS := \
. \

# Add inputs and outputs from these tool invocations to the build variables
CMD_SRCS += \
endat_diagnostic.cmd

ASM_SRCS += \
$(MCU_PLUS_SDK_PATH)/source/motor_control/position_sense/endat/firmware/endat_main.asm

OBJS += \
./endat_main.obj

ASM_DEPS += \
./endat_main.d

OBJS__QUOTED += \
"endat_main.obj"

ASM_DEPS__QUOTED += \
"endat_main.d"

ASM_SRCS__QUOTED += \
"$(MCU_PLUS_SDK_PATH)/source/motor_control/position_sense/endat/firmware/endat_main.asm"

# Each subdirectory must supply rules for building sources it contributes
endat_main.obj: $(MCU_PLUS_SDK_PATH)/source/motor_control/position_sense/endat/firmware/endat_main.asm $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: PRU Compiler'
	"$(CG_TOOL_ROOT)/bin/clpru" --include_path="$(CCS_PATH)" --include_path="$(MCU_PLUS_SDK_PATH)/source/motor_control/position_sense/endat/firmware" --include_path="$(CG_TOOL_ROOT)/include" --define=PRU1 --define=$(MULTI_CHANNEL) --define=icssg1 -g --diag_warning=225 --diag_wrap=off --display_error_number --endian=little --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo '

USER_OBJS :=

LIBS := -llibc.a

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables
EXE_OUTPUTS += \
endat_master.out \

EXE_OUTPUTS__QUOTED += \
"endat_master.out" \

BIN_OUTPUTS += \
endat_master.hex \

BIN_OUTPUTS__QUOTED += \
"endat_master.hex" \


# All Target
all: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "endat_master.out"

# Tool invocations
endat_master.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@echo 'Building target: "$@"'
	@echo 'Invoking: PRU Linker'
	"$(CG_TOOL_ROOT)/bin/clpru" --define=PRU1 --define=ENABLE_MULTI_CHANNEL --define=icssg1 -g --diag_warning=225 --diag_wrap=off --display_error_number --endian=little -z -m"endat_master.map" --heap_size=0 --stack_size=0 -i"$(CG_TOOL_ROOT)/lib" -i"$(CG_TOOL_ROOT)/include" --reread_libs --diag_wrap=off --display_error_number --warn_sections --xml_link_info="endat_master_linkInfo.xml" -o "endat_master.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '
	@$(MAKE) --no-print-directory post-build

endat_master.hex: $(EXE_OUTPUTS)
	@echo 'Building files: $(strip $(EXE_OUTPUTS__QUOTED))'
	@echo 'Invoking: PRU Hex Utility'
	"$(CG_TOOL_ROOT)/bin/hexpru"  -o "endat_master.hex" $(EXE_OUTPUTS__QUOTED)
	@echo 'Finished building: $(strip $(EXE_OUTPUTS__QUOTED))'
	@echo ' '
	@$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(BIN_OUTPUTS__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) "endat_main.obj"
	-$(RM) "endat_main.d"
	-@echo 'Finished clean'
	-@echo ' '

post-build:
ifeq ($(CHANNELS),SINGLE)
	$(CG_TOOL_ROOT)\bin\hexpru.exe endat_master_hexpru.cmd endat_master.out && $(MCU_PLUS_SDK_PATH)\tools\bin2header\bin2header.exe endat_master.b00 endat_master_bin.h EnDatFirmware 4
endif
ifeq ($(CHANNELS),MULTI)
	$(CG_TOOL_ROOT)\bin\hexpru.exe endat_master_hexpru.cmd endat_master.out && $(MCU_PLUS_SDK_PATH)\tools\bin2header\bin2header.exe endat_master.b00 endat_master_multi_bin.h EnDatFirmwareMulti 4
endif
	-
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

