################################################################################
# Automatically-generated file. Do not edit!
################################################################################
#   Required input arguments :
#   CCS_PROJECT_DEBUG=${CWD}   Use this input argument to include files from external paths

export MCU_PLUS_SDK_PATH?=$(abspath ../../../../../../..)
include $(MCU_PLUS_SDK_PATH)/imports.mak
SYSCFG_DIR := $(CCS_PROJECT_DEBUG)/syscfg
CG_TOOL_ROOT := $(CGT_TI_PRU_PATH)

# Add inputs and outputs from these tool invocations to the build variables
CMD_SRCS += \
	linker.cmd \

ASM_SRCS += \
	main.asm \

OBJS += \
	main.obj \

ASM_DEPS += \
	main.d \

OBJDIR := .

FILES_PATH_common = \
	.. \
	../../.. \
	. \

FILES_PATH := $(FILES_PATH_common)

vpath %.asm $(FILES_PATH)
vpath %.obj $(OBJDIR)
vpath %.cmd $(FILES_PATH)


# Add inputs and outputs from these tool invocations to the build variables
EXE_OUTPUTS += \
empty_am64x-evm_icss_g0_rtu_pru1_fw_ti-pru-cgt.out \

EXE_OUTPUTS__QUOTED += \
"empty_am64x-evm_icss_g0_rtu_pru1_fw_ti-pru-cgt.out" \

BIN_OUTPUTS += \
empty_am64x-evm_icss_g0_rtu_pru1_fw_ti-pru-cgt.hex \

BIN_OUTPUTS__QUOTED += \
"empty_am64x-evm_icss_g0_rtu_pru1_fw_ti-pru-cgt.hex" \

# All Target
all: $(OBJS) $(CMD_SRCS)
	@$(MAKE) --no-print-directory -Onone "empty_am64x-evm_icss_g0_rtu_pru1_fw_ti-pru-cgt.out"

# Each subdirectory must supply rules for building sources it contributes
$(OBJDIR)/%.obj %.obj: %.asm
	@echo 'Building file: "$^"'
	@echo 'Invoking: PRU Compiler'
	"$(CGT_TI_PRU_PATH)/bin/clpru" -DRTU_PRU1 -DSLICE1 -v4 --define=SOC_AM64X --include_path="${CG_TOOL_ROOT}/include" --include_path="${MCU_PLUS_SDK_PATH}/source" --include_path="${MCU_PLUS_SDK_PATH}/source/pru_io/firmware/common"  --define=_DEBUG_=1 -g --diag_warning=225 --diag_wrap=off --display_error_number --endian=little --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $^
	@echo 'Finished building: "$^"'
	@echo ' '

# Tool invocations
empty_am64x-evm_icss_g0_rtu_pru1_fw_ti-pru-cgt.out: $(OBJS) $(CMD_SRCS)
	@echo 'Building target: "$@"'
	@echo 'Invoking: PRU Linker'
	"$(CGT_TI_PRU_PATH)/bin/clpru" -DRTU_PRU1 -DSLICE1 -v4 --define=SOC_AM64X  --define=_DEBUG_=1 -g --diag_warning=225 --diag_wrap=off --display_error_number --endian=little -z -m"empty_am64x-evm_icss_g0_rtu_pru1_fw_ti-pru-cgt.map" --disable_auto_rts   --diag_wrap=off --display_error_number --warn_sections --xml_link_info="empty_am64x-evm_icss_g0_rtu_pru1_fw_ti-pru-cgt_linkInfo.xml" --disable_auto_rts --entry_point=main --diag_suppress=10063-D  --rom_model -o "empty_am64x-evm_icss_g0_rtu_pru1_fw_ti-pru-cgt.out" $^
	@echo 'Finished building target: "$@"'
	@echo ' '
	@$(MAKE) --no-print-directory post-build

# To clean generated files
clean:
	-$(RM) $(BIN_OUTPUTS__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) $(OBJS)
	-$(RM) $(ASM_DEPS)
	-@echo 'Finished clean'
	-@echo ' '

post-build:
		- $(CG_TOOL_ROOT)/bin/hexpru --diag_wrap=off --array --array:name_prefix=RTUPRU1Firmware  -o rtupru1_load_bin.h empty_am64x-evm_icss_g0_rtu_pru1_fw_ti-pru-cgt.out
		- $(SED) -i '0r ${MCU_PLUS_SDK_PATH}/source/pru_io/firmware/pru_load_bin_copyright.h' rtupru1_load_bin.h 
		- $(MOVE) rtupru1_load_bin.h ${MCU_PLUS_SDK_PATH}/examples/pru_io/empty/firmware/am64x-evm/rtupru1_load_bin.h 
	-@echo ' '

.PHONY: all clean
