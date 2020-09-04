################################################################################
# Automatically-generated file. Do not edit!
################################################################################

#   Required input arguments:
#   MCU_PLUS_SDK_PATH=${MCU_PLUS_SDK_PATH}      sdk_dir_path
#   CCS_INSTALL_DIR=${CCS_INSTALL_DIR}          ccs_dir_path
#   CCS_PROJECT_DEBUG=${CWD}       				project_debug_dir_path

SHELL = cmd.exe
CCS_PATH?=$(CCS_INSTALL_DIR)/ccs_base/pru/include
CG_TOOL_ROOT := C:/ti/ti-cgt-pru_2.3.3
SYSCFG_DIR := $(CCS_PROJECT_DEBUG)/syscfg

GEN_OPTS__FLAG :=
GEN_CMDS__FLAG :=

ORDERED_OBJS += \
"./main.obj" \
"$(MCU_PLUS_SDK_PATH)/examples/pru_io/empty/firmware/am64x-evm/icssg0-pru0_fw/ti-pru-cgt/linker.cmd" \
$(GEN_CMDS__FLAG) \
-llibc.a \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# Every subdirectory with source files must be described here
SUBDIRS := \
. \

# Add inputs and outputs from these tool invocations to the build variables
CMD_SRCS += \
$(MCU_PLUS_SDK_PATH)/examples/pru_io/empty/firmware/am64x-evm/icssg0-pru0_fw/ti-pru-cgt/linker.cmd \

ASM_SRCS += \
$(MCU_PLUS_SDK_PATH)/examples/pru_io/empty/firmware/main.asm

OBJS += \
./main.obj

ASM_DEPS += \
./main.d

OBJS__QUOTED += \
"main.obj"

ASM_DEPS__QUOTED += \
"main.d"

ASM_SRCS__QUOTED += \
"$(MCU_PLUS_SDK_PATH)/examples/pru_io/empty/firmware/main.asm"

# Each subdirectory must supply rules for building sources it contributes
main.obj: $(MCU_PLUS_SDK_PATH)/examples/pru_io/empty/firmware/main.asm $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: PRU Compiler'
	"$(CG_TOOL_ROOT)/bin/clpru" -DICSSG0 -DPRU0 -DSLICE0 -v4 --define=SOC_AM64X --include_path="$(CCS_PATH)" --include_path="$(SYSCFG_DIR)" --include_path="${CG_TOOL_ROOT}/include" --include_path="${MCU_PLUS_SDK_PATH}/source" --include_path="${MCU_PLUS_SDK_PATH}/source/pru_io/firmware/common"  --define=_DEBUG_=1 -g --diag_warning=225 --diag_wrap=off --display_error_number --endian=little --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

LIBS := -llibc.a

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables
EXE_OUTPUTS += \
empty_am64x-evm_icssg0-pru0_fw_ti-pru-cgt.out \

EXE_OUTPUTS__QUOTED += \
"empty_am64x-evm_icssg0-pru0_fw_ti-pru-cgt.out" \

BIN_OUTPUTS += \
empty_am64x-evm_icssg0-pru0_fw_ti-pru-cgt.hex \

BIN_OUTPUTS__QUOTED += \
"empty_am64x-evm_icssg0-pru0_fw_ti-pru-cgt.hex" \


# All Target
all: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "empty_am64x-evm_icssg0-pru0_fw_ti-pru-cgt.out"

# Tool invocations
empty_am64x-evm_icssg0-pru0_fw_ti-pru-cgt.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@echo 'Building target: "$@"'
	@echo 'Invoking: PRU Linker'
	"$(CG_TOOL_ROOT)/bin/clpru" -DICSSG0 -DPRU0 -DSLICE0 -v4 --define=SOC_AM64X  --define=_DEBUG_=1 -g --diag_warning=225 --diag_wrap=off --display_error_number --endian=little -z -m"empty_am64x-evm_icssg0-pru0_fw_ti-pru-cgt.map" -i"$(CG_TOOL_ROOT)/lib" --diag_wrap=off --display_error_number --warn_sections --xml_link_info="empty_am64x-evm_icssg0-pru0_fw_ti-pru-cgt_linkInfo.xml" --rom_model -o "empty_am64x-evm_icssg0-pru0_fw_ti-pru-cgt.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '
	@$(MAKE) --no-print-directory post-build

empty_am64x-evm_icssg0-pru0_fw_ti-pru-cgt.hex: $(EXE_OUTPUTS)
	@echo 'Building secondary target: "$@"'
	@echo 'Invoking: PRU Hex Utility'
	"$(CG_TOOL_ROOT)/bin/hexpru" -o "empty_am64x-evm_icssg0-pru0_fw_ti-pru-cgt.hex" $(EXE_OUTPUTS__QUOTED)
	@echo 'Finished building secondary target: "$@"'
	@echo ' '
	@$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(BIN_OUTPUTS__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) "main.obj"
	-$(RM) "main.d"
	-@echo 'Finished clean'
	-@echo ' '

post-build:
	-$(CG_TOOL_ROOT)/bin/hexpru.exe ${MCU_PLUS_SDK_PATH}/examples/pru_io/empty/firmware/am64x-evm/icssg0-pru0_fw/ti-pru-cgt/hexpru.cmd empty_am64x-evm_icssg0-pru0_fw_ti-pru-cgt.out
	-${MCU_PLUS_SDK_PATH}/tools/bin2header/bin2header.exe empty_am64x-evm_icssg0-pru0_fw_ti-pru-cgt.b00 pru_load_bin.h PRUFirmware 4
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
