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
"./micro_scheduler.obj" \
"$(MCU_PLUS_SDK_PATH)/test/networking/icss_emac_loopback/firmware/mii/am263x-cc/icssm-pru1_fw/ti-pru-cgt/pru.cmd" \
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
$(MCU_PLUS_SDK_PATH)/test/networking/icss_emac_loopback/firmware/mii/am263x-cc/icssm-pru1_fw/ti-pru-cgt/pru.cmd \

ASM_SRCS += \
$(MCU_PLUS_SDK_PATH)/test/networking/icss_emac_loopback/firmware/mii/micro_scheduler.asm

OBJS += \
./micro_scheduler.obj

ASM_DEPS += \
./micro_scheduler.d

OBJS__QUOTED += \
"micro_scheduler.obj"

ASM_DEPS__QUOTED += \
"micro_scheduler.d"

ASM_SRCS__QUOTED += \
"$(MCU_PLUS_SDK_PATH)/test/networking/icss_emac_loopback/firmware/mii/micro_scheduler.asm"

# Each subdirectory must supply rules for building sources it contributes
micro_scheduler.obj: $(MCU_PLUS_SDK_PATH)/test/networking/icss_emac_loopback/firmware/mii/micro_scheduler.asm $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: PRU Compiler'
	"$(CG_TOOL_ROOT)/bin/clpru" -DPRU1 -DSLICE1 -al -ax --silicon_version=4 --diag_suppress=10063 --display_error_number --code_address_listing_unit=word -DTX_L2_ENABLED -DMII_TX_PIN_SWAP -v3 -g --diag_wrap=off --diag_warning=225 --hardware_mac=on --preproc_with_compile -DICSS_REV2 -DPRU -DTTS -DHALF_DUPLEX_ENABLED -DICSS_DUAL_EMAC_BUILD -DPTP --define=SOC_AM263X --include_path="$(CCS_PATH)" --include_path="$(SYSCFG_DIR)" --include_path="${MCU_PLUS_SDK_PATH}/source/pru_io/firmware/common" --include_path="${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-Kernel/include" --include_path="${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CR5F" --include_path="${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/am263x/r5f" --include_path="${MCU_PLUS_SDK_PATH}/test/networking/icss_emac_loopback" --include_path="${MCU_PLUS_SDK_PATH}/test/networking/icss_emac_loopback/mii/am263x-cc" --include_path="${MCU_PLUS_SDK_PATH}/test/unity/"  --define=_DEBUG_=1 -g --diag_warning=225 --diag_wrap=off --display_error_number --endian=little --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

LIBS := -llibc.a

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables
EXE_OUTPUTS += \
dual_emac_am263x-cc_icssm-pru1_fw_ti-pru-cgt.out \

EXE_OUTPUTS__QUOTED += \
"dual_emac_am263x-cc_icssm-pru1_fw_ti-pru-cgt.out" \

BIN_OUTPUTS += \
dual_emac_am263x-cc_icssm-pru1_fw_ti-pru-cgt.hex \

BIN_OUTPUTS__QUOTED += \
"dual_emac_am263x-cc_icssm-pru1_fw_ti-pru-cgt.hex" \


# All Target
all: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "dual_emac_am263x-cc_icssm-pru1_fw_ti-pru-cgt.out"

# Tool invocations
dual_emac_am263x-cc_icssm-pru1_fw_ti-pru-cgt.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@echo 'Building target: "$@"'
	@echo 'Invoking: PRU Linker'
	"$(CG_TOOL_ROOT)/bin/clpru" -DPRU1 -DSLICE1 -al -ax --silicon_version=4 --diag_suppress=10063 --display_error_number --code_address_listing_unit=word -DTX_L2_ENABLED -DMII_TX_PIN_SWAP -v3 -g --diag_wrap=off --diag_warning=225 --hardware_mac=on --preproc_with_compile -DICSS_REV2 -DPRU -DTTS -DHALF_DUPLEX_ENABLED -DICSS_DUAL_EMAC_BUILD -DPTP --define=SOC_AM263X  --define=_DEBUG_=1 -g --diag_warning=225 --diag_wrap=off --display_error_number --endian=little -z -m"dual_emac_am263x-cc_icssm-pru1_fw_ti-pru-cgt.map" -i"$(CG_TOOL_ROOT)/lib" --diag_wrap=off --display_error_number --warn_sections --xml_link_info="dual_emac_am263x-cc_icssm-pru1_fw_ti-pru-cgt_linkInfo.xml" --rom_model -o "dual_emac_am263x-cc_icssm-pru1_fw_ti-pru-cgt.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '
	@$(MAKE) --no-print-directory post-build

dual_emac_am263x-cc_icssm-pru1_fw_ti-pru-cgt.hex: $(EXE_OUTPUTS)
	@echo 'Building secondary target: "$@"'
	@echo 'Invoking: PRU Hex Utility'
	"$(CG_TOOL_ROOT)/bin/hexpru" -o "dual_emac_am263x-cc_icssm-pru1_fw_ti-pru-cgt.hex" $(EXE_OUTPUTS__QUOTED)
	@echo 'Finished building secondary target: "$@"'
	@echo ' '
	@$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(BIN_OUTPUTS__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) "micro_scheduler.obj"
	-$(RM) "micro_scheduler.d"
	-@echo 'Finished clean'
	-@echo ' '

post-build:
	-$(CG_TOOL_ROOT)/bin/hexpru.exe ${MCU_PLUS_SDK_PATH}/test/networking/icss_emac_loopback/firmware/mii/icss_emac_hexpru.cmd dual_emac_am263x-cc_icssm-pru1_fw_ti-pru-cgt.out
	-${MCU_PLUS_SDK_PATH}/tools/bin2header/bin2header.exe dual_emac_am263x-cc_icssm-pru1_fw_ti-pru-cgt.b00 PRU1_bin.h PRU1_b00 4
	-cp PRU1_bin.h ${MCU_PLUS_SDK_PATH}/test/networking/icss_emac_loopback/firmware/mii
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
