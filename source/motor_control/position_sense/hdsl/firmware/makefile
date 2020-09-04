################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe
MCU_PLUS_SDK_PATH?=$(abspath ../../../../..)
CCS_PATH?=C:/ti/ccs1200/ccs/ccs_base/pru/include
CG_TOOL_ROOT := C:/ti/ti-cgt-pru_2.3.3

#Set the default mode
HDSL_MODE ?= FREE_RUN_MODE

ifeq ($(HDSL_MODE),SYNC_MODE)
SYNC_ENABLE=EXT_SYNC_ENABLE
endif

ifeq ($(HDSL_MODE),FREE_RUN_MODE)
SYNC_ENABLE=0
endif



GEN_OPTS__FLAG :=
GEN_CMDS__FLAG :=

ORDERED_OBJS += \
"datalink.obj" \
"datalink_init.obj" \
"main.obj" \
"transport.obj" \
"utils.obj" \
"hdsl_master_icssg.cmd" \
$(GEN_CMDS__FLAG) \
-llibc.a \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here

CMD_SRCS += \
hdsl_master_icssg.cmd

ASM_SRCS += \
datalink.asm \
datalink_init.asm \
main.asm \
transport.asm \
utils.asm

OBJS += \
./datalink.obj \
./datalink_init.obj \
./main.obj \
./transport.obj \
./utils.obj

ASM_DEPS += \
./datalink.d \
./datalink_init.d \
./main.d \
./transport.d \
./utils.d

OBJS__QUOTED += \
"datalink.obj" \
"datalink_init.obj" \
"main.obj" \
"transport.obj" \
"utils.obj"

ASM_DEPS__QUOTED += \
"datalink.d" \
"datalink_init.d" \
"main.d" \
"transport.d" \
"utils.d"

ASM_SRCS__QUOTED += \
"datalink.asm" \
"datalink_init.asm" \
"main.asm" \
"transport.asm" \
"utils.asm"

SUBDIRS := \
. \

USER_OBJS :=

LIBS := -llibc.a

# Each subdirectory must supply rules for building sources it contributes
datalink.obj:  $(MCU_PLUS_SDK_PATH)/source/motor_control/position_sense/hdsl/firmware/datalink.asm $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: PRU Compiler'
	"$(CG_TOOL_ROOT)/bin/clpru" -v3 --include_path="$(CCS_PATH)" --include_path="$(MCU_PLUS_SDK_PATH)/source/motor_control/position_sense/hdsl/firmware" --include_path="$(CG_TOOL_ROOT)/include" --define=am4379 --define=icss1 --define=PRU1 --define=ICSS_G_V_1_0 --define=ENDAT_CHANNEL_0 --define=$(SYNC_ENABLE) --diag_warning=225 --diag_wrap=off --display_error_number --endian=little --hardware_mac=on --asm_listing --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

datalink_init.obj: $(MCU_PLUS_SDK_PATH)/source/motor_control/position_sense/hdsl/firmware/datalink_init.asm $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: PRU Compiler'
	"$(CG_TOOL_ROOT)/bin/clpru" -v3 --include_path="$(CCS_PATH)" --include_path="$(MCU_PLUS_SDK_PATH)/source/motor_control/position_sense/hdsl/firmware" --include_path="$(CG_TOOL_ROOT)/include" --define=am4379 --define=icss1 --define=PRU1 --define=ICSS_G_V_1_0 --define=ENDAT_CHANNEL_0 --define=$(SYNC_ENABLE) --diag_warning=225 --diag_wrap=off --display_error_number --endian=little --hardware_mac=on --asm_listing --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

main.obj: $(MCU_PLUS_SDK_PATH)/source/motor_control/position_sense/hdsl/firmware/main.asm $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: PRU Compiler'
	"$(CG_TOOL_ROOT)/bin/clpru" -v3 --include_path="$(CCS_PATH)" --include_path="$(MCU_PLUS_SDK_PATH)/source/motor_control/position_sense/hdsl/firmware" --include_path="$(CG_TOOL_ROOT)/include" --define=am4379 --define=icss1 --define=PRU1 --define=ICSS_G_V_1_0 --define=ENDAT_CHANNEL_0 --define=$(SYNC_ENABLE) --diag_warning=225 --diag_wrap=off --display_error_number --endian=little --hardware_mac=on --asm_listing --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

transport.obj: $(MCU_PLUS_SDK_PATH)/source/motor_control/position_sense/hdsl/firmware/transport.asm $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: PRU Compiler'
	"$(CG_TOOL_ROOT)/bin/clpru" -v3 --include_path="$(CCS_PATH)" --include_path="$(MCU_PLUS_SDK_PATH)/source/motor_control/position_sense/hdsl/firmware" --include_path="$(CG_TOOL_ROOT)/include" --define=am4379 --define=icss1 --define=PRU1 --define=ICSS_G_V_1_0 --define=ENDAT_CHANNEL_0 --define=$(SYNC_ENABLE) --diag_warning=225 --diag_wrap=off --display_error_number --endian=little --hardware_mac=on --asm_listing --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

utils.obj: $(MCU_PLUS_SDK_PATH)/source/motor_control/position_sense/hdsl/firmware/utils.asm $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: PRU Compiler'
	"$(CG_TOOL_ROOT)/bin/clpru" -v3 --include_path="$(CCS_PATH)" --include_path="$(MCU_PLUS_SDK_PATH)/source/motor_control/position_sense/hdsl/firmware" --include_path="$(CG_TOOL_ROOT)/include" --define=am4379 --define=icss1 --define=PRU1 --define=ICSS_G_V_1_0 --define=ENDAT_CHANNEL_0 --define=$(SYNC_ENABLE) --diag_warning=225 --diag_wrap=off --display_error_number --endian=little --hardware_mac=on --asm_listing --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

LIBS := -llibc.a

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables
EXE_OUTPUTS += \
hdsl_master_icssg.out \

EXE_OUTPUTS__QUOTED += \
"hdsl_master_icssg.out" \

BIN_OUTPUTS += \
hdsl_master_icssg.hex \

BIN_OUTPUTS__QUOTED += \
"hdsl_master_icssg.hex" \


# All Target
all: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "hdsl_master_icssg.out"

# Tool invocations
hdsl_master_icssg.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@echo 'Building target: "$@"'
	@echo 'Invoking: PRU Linker'
	"$(CG_TOOL_ROOT)/bin/clpru" -v3 --define=am4379 --define=icss1 --define=PRU1 --define=ICSS_G_V_1_0 --define=ENDAT_CHANNEL_0 --define=EXT_SYNC_ENABLE --diag_warning=225 --diag_wrap=off --display_error_number --endian=little --hardware_mac=on --asm_listing -z -m"hdsl_master_icssg.map" --heap_size=0 --stack_size=0 -i"$(CG_TOOL_ROOT)/lib" -i"$(CG_TOOL_ROOT)/include" --reread_libs --define=ICSS1_PRU_CORE=1 --diag_wrap=off --display_error_number --warn_sections --xml_link_info="hdsl_master_icssg_linkInfo.xml" --entry_point=main -o "hdsl_master_icssg.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '
	@$(MAKE) --no-print-directory post-build

hdsl_master_icssg.hex: $(EXE_OUTPUTS)
	@echo 'Building files: $(strip $(EXE_OUTPUTS__QUOTED))'
	@echo 'Invoking: PRU Hex Utility'
	"$(CG_TOOL_ROOT)/bin/hexpru"  -o "hdsl_master_icssg.hex" $(EXE_OUTPUTS__QUOTED)
	@echo 'Finished building: $(strip $(EXE_OUTPUTS__QUOTED))'
	@echo ' '
	@$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(BIN_OUTPUTS__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) "datalink.lst" "datalink_init.lst" "main.lst" "transport.lst" "utils.lst"
	-$(RM) "datalink.obj" "datalink_init.obj" "main.obj" "transport.obj" "utils.obj"
	-$(RM) "datalink.d" "datalink_init.d" "main.d" "transport.d" "utils.d"
	-@echo 'Finished clean'
	-@echo ' '

post-build:
ifeq ($(HDSL_MODE),SYNC_MODE)
	-$(CG_TOOL_ROOT)\bin\hexpru.exe hdsl_master_icssg_hexpru.cmd hdsl_master_icssg.out && $(MCU_PLUS_SDK_PATH)\tools\bin2header\bin2header.exe hdsl_master_icssg.b00 hdsl_master_icssg_sync_bin.h Hiperface_DSL_SYNC2_0 4
endif
ifeq ($(HDSL_MODE),FREE_RUN_MODE)
	-$(CG_TOOL_ROOT)\bin\hexpru.exe hdsl_master_icssg_hexpru.cmd hdsl_master_icssg.out && $(MCU_PLUS_SDK_PATH)\tools\bin2header\bin2header.exe hdsl_master_icssg.b00 hdsl_master_icssg_bin.h Hiperface_DSL2_0 4
endif
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

