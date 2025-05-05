################################################################################
# Automatically-generated file. Do not edit!
################################################################################
#   Required input arguments :
#   CCS_PROJECT_DEBUG=${CWD}   Use this input argument to include files from external paths

export MCU_PLUS_SDK_PATH?=$(abspath ../../../../../../../../..)
include $(MCU_PLUS_SDK_PATH)/imports.mak
SYSCFG_DIR := $(CCS_PROJECT_DEBUG)/syscfg
CG_TOOL_ROOT := $(CGT_TI_PRU_PATH)

# Add inputs and outputs from these tool invocations to the build variables
CMD_SRCS += \
	pru.cmd \

ASM_SRCS += \
	emac_MII_Rcv.asm \
	emac_MII_Xmt.asm \
	emac_ptp.asm \
	emac_statistics.asm \
	emac_tts.asm \
	micro_scheduler.asm \

OBJS += \
	emac_MII_Rcv.obj \
	emac_MII_Xmt.obj \
	emac_ptp.obj \
	emac_statistics.obj \
	emac_tts.obj \
	micro_scheduler.obj \

ASM_DEPS += \
	emac_MII_Rcv.d \
	emac_MII_Xmt.d \
	emac_ptp.d \
	emac_statistics.d \
	emac_tts.d \
	micro_scheduler.d \

OBJDIR := .

FILES_PATH_common = \
	../.. \
	../../.. \
	../../../../../source \

FILES_PATH := $(FILES_PATH_common)

vpath %.asm $(FILES_PATH)
vpath %.obj $(OBJDIR)
vpath %.cmd $(FILES_PATH)


# Add inputs and outputs from these tool invocations to the build variables
EXE_OUTPUTS += \
dual_emac_am263px-lp_icss_m0_pru1_fw_ti-pru-cgt.out \

EXE_OUTPUTS__QUOTED += \
"dual_emac_am263px-lp_icss_m0_pru1_fw_ti-pru-cgt.out" \

BIN_OUTPUTS += \
dual_emac_am263px-lp_icss_m0_pru1_fw_ti-pru-cgt.hex \

BIN_OUTPUTS__QUOTED += \
"dual_emac_am263px-lp_icss_m0_pru1_fw_ti-pru-cgt.hex" \

# All Target
all: $(OBJS) $(CMD_SRCS)
	@$(MAKE) --no-print-directory -Onone "dual_emac_am263px-lp_icss_m0_pru1_fw_ti-pru-cgt.out"

# Each subdirectory must supply rules for building sources it contributes
$(OBJDIR)/%.obj %.obj: %.asm
	@echo 'Building file: "$^"'
	@echo 'Invoking: PRU Compiler'
	"$(CGT_TI_PRU_PATH)/bin/clpru" -DPRU1 -DSLICE1 -v4 -al -ax --silicon_version=4 --diag_suppress=10063 --display_error_number --code_address_listing_unit=word -DTX_L2_ENABLED -DMII_TX_PIN_SWAP -v3 -g --diag_wrap=off --diag_warning=225 --hardware_mac=on --preproc_with_compile -DICSS_REV2 -DPRU -DTTS -DHALF_DUPLEX_ENABLED -DICSS_DUAL_EMAC_BUILD -DPTP --define=SOC_AM263PX --define=OS_FW --include_path="$(SYSCFG_DIR)" --include_path="${CG_TOOL_ROOT}/include" --include_path="${MCU_PLUS_SDK_PATH}/source" --include_path="${MCU_PLUS_SDK_PATH}/source/pru_io/firmware/common" --include_path="${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-Kernel/include" --include_path="${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CR5F" --include_path="${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/am263px/r5f" --include_path="${MCU_PLUS_SDK_PATH}/source/networking/icss_emac/firmware/icss_dual_emac" --include_path="${MCU_PLUS_SDK_PATH}/source/networking/icss_emac/firmware/icss_switch" --include_path="${MCU_PLUS_SDK_PATH}/source/networking/icss_emac/firmware/source" --include_path="${MCU_PLUS_SDK_PATH}/test/unity/"  --define=_DEBUG_=1 -g --diag_warning=225 --diag_wrap=off --display_error_number --endian=little --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $^
	@echo 'Finished building: "$^"'
	@echo ' '

# Tool invocations
dual_emac_am263px-lp_icss_m0_pru1_fw_ti-pru-cgt.out: $(OBJS) $(CMD_SRCS)
	@echo 'Building target: "$@"'
	@echo 'Invoking: PRU Linker'
	"$(CGT_TI_PRU_PATH)/bin/clpru" -DPRU1 -DSLICE1 -v4 -al -ax --silicon_version=4 --diag_suppress=10063 --display_error_number --code_address_listing_unit=word -DTX_L2_ENABLED -DMII_TX_PIN_SWAP -v3 -g --diag_wrap=off --diag_warning=225 --hardware_mac=on --preproc_with_compile -DICSS_REV2 -DPRU -DTTS -DHALF_DUPLEX_ENABLED -DICSS_DUAL_EMAC_BUILD -DPTP --define=SOC_AM263PX --define=OS_FW  --define=_DEBUG_=1 -g --diag_warning=225 --diag_wrap=off --display_error_number --endian=little -z -m"dual_emac_am263px-lp_icss_m0_pru1_fw_ti-pru-cgt.map" --disable_auto_rts   --diag_wrap=off --display_error_number --warn_sections --xml_link_info="dual_emac_am263px-lp_icss_m0_pru1_fw_ti-pru-cgt_linkInfo.xml" --disable_auto_rts -i${CG_TOOL_ROOT}/lib -i${CG_TOOL_ROOT}/include --stack_size=0 --heap_size=0 --disable_auto_rts -g --diag_wrap=off --diag_warning=225 --reread_libs --warn_sections --entry_point=micro_scheduler  --rom_model -o "dual_emac_am263px-lp_icss_m0_pru1_fw_ti-pru-cgt.out" $^
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
		-$(CG_TOOL_ROOT)/bin/hexpru ${MCU_PLUS_SDK_PATH}/source/networking/icss_emac/firmware/icss_dual_emac/mii/icss_emac_hexpru.cmd dual_emac_am263px-lp_icss_m0_pru1_fw_ti-pru-cgt.out
		-${MCU_PLUS_SDK_PATH}/tools/bin2header/bin2header.exe dual_emac_am263px-lp_icss_m0_pru1_fw_ti-pru-cgt.b00 PRU1_bin.h PRU1_b00 4
		-cp PRU1_bin.h ${MCU_PLUS_SDK_PATH}/source/networking/icss_emac/firmware/icss_dual_emac/mii
	-@echo ' '

.PHONY: all clean
