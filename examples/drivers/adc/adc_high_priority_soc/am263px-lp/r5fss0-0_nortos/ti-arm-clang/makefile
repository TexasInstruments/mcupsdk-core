
export MCU_PLUS_SDK_PATH?=$(abspath ../../../../../../..)
include $(MCU_PLUS_SDK_PATH)/imports.mak
include $(MCU_PLUS_SDK_PATH)/devconfig/devconfig.mak

CG_TOOL_ROOT=$(CGT_TI_ARM_CLANG_PATH)

CC=$(CG_TOOL_ROOT)/bin/tiarmclang
LNK=$(CG_TOOL_ROOT)/bin/tiarmclang
STRIP=$(CG_TOOL_ROOT)/bin/tiarmstrip
OBJCOPY=$(CG_TOOL_ROOT)/bin/tiarmobjcopy
COV=$(CG_TOOL_ROOT)/bin/tiarmcov
PROFDATA=$(CG_TOOL_ROOT)/bin/tiarmprofdata
COVERAGE_PATH=$(abspath .)
ifeq ($(OS), Windows_NT)
	PYTHON=python
else
	PYTHON=python3
endif

PROFILE?=release
ConfigName:=$(PROFILE)

OUTNAME:=adc_high_priority_soc.$(PROFILE).out
COREOUTNAME:=adc_high_priority_soc.$(PROFILE).optishare.out

BOOTIMAGE_PATH=$(abspath .)
oeconfig?=None

BOOTIMAGE_NAME_MCELF:=adc_high_priority_soc.$(PROFILE).mcelf
BOOTIMAGE_NAME_MCELF_HS:=adc_high_priority_soc.$(PROFILE).mcelf.hs
TARGETS := $(BOOTIMAGE_NAME_MCELF)

ifeq ($(DEVICE_TYPE), HS)
   TARGETS += $(BOOTIMAGE_NAME_MCELF_HS)
endif

FILES_common := \
	adc_high_priority_soc.c \
	main.c \
	ti_drivers_config.c \
	ti_drivers_open_close.c \
	ti_board_config.c \
	ti_board_open_close.c \
	ti_dpl_config.c \
	ti_pinmux_config.c \
	ti_power_clock_config.c \

FILES_PATH_common = \
	.. \
	../../.. \
	generated \

INCLUDES_common := \
	-I${CG_TOOL_ROOT}/include/c \
	-I${MCU_PLUS_SDK_PATH}/source \
	-Igenerated \

DEFINES_common := \
	-DSOC_AM263PX \
	-DOS_NORTOS \

CFLAGS_common := \
	-mcpu=cortex-r5 \
	-mfloat-abi=hard \
	-mfpu=vfpv3-d16 \
	-mthumb \
	-Wall \
	-Werror \
	-g \
	-Wno-gnu-variable-sized-type-not-at-end \
	-Wno-unused-function \

CFLAGS_cpp_common := \
	-Wno-c99-designator \
	-Wno-extern-c-compat \
	-Wno-c++11-narrowing \
	-Wno-reorder-init-list \
	-Wno-register \
	-Wno-writable-strings \
	-Wno-enum-compare \
	-Wno-reserved-user-defined-literal \
	-Wno-unused-const-variable \
	-Wno-vla-cxx-extension \
	-x c++ \

CFLAGS_debug := \
	-D_DEBUG_=1 \

CFLAGS_release := \
	-Os \

LNK_FILES_common = \
	generated/linker.cmd \

LIBS_PATH_common = \
	-Wl,-i${MCU_PLUS_SDK_PATH}/source/kernel/nortos/lib \
	-Wl,-i${MCU_PLUS_SDK_PATH}/source/drivers/lib \
	-Wl,-i${MCU_PLUS_SDK_PATH}/source/board/lib \
	-Wl,-i${MCU_PLUS_SDK_PATH}/source/board/pmic/lib \
	-Wl,-i${CG_TOOL_ROOT}/lib \

LIBS_common = \
	-lnortos.am263px.r5f.ti-arm-clang.${ConfigName}.lib \
	-ldrivers.am263px.r5f.ti-arm-clang.nortos.${ConfigName}.lib \
	-lboard.am263px.r5f.ti-arm-clang.nortos.${ConfigName}.lib \
	-lpmic_blackbird.am263px.r5f.ti-arm-clang.nortos.${ConfigName}.lib \
	-llibc.a \
	-llibsysbm.a \

LFLAGS_common = \
	-Wl,--diag_suppress=10063 \
	-Wl,--ram_model \
	-Wl,--reread_libs \
	-Wl,--gen_xml_func_hash \


LIBS_NAME = \
	nortos.am263px.r5f.ti-arm-clang.${ConfigName}.lib \
	drivers.am263px.r5f.ti-arm-clang.nortos.${ConfigName}.lib \
	board.am263px.r5f.ti-arm-clang.nortos.${ConfigName}.lib \
	pmic_blackbird.am263px.r5f.ti-arm-clang.nortos.${ConfigName}.lib \
	libc.a \
	libsysbm.a \

LIBS_PATH_NAME = \
	${MCU_PLUS_SDK_PATH}/source/kernel/nortos/lib \
	${MCU_PLUS_SDK_PATH}/source/drivers/lib \
	${MCU_PLUS_SDK_PATH}/source/board/lib \
	${MCU_PLUS_SDK_PATH}/source/board/pmic/lib \
	${CG_TOOL_ROOT}/lib \

FILES := $(FILES_common) $(FILES_$(PROFILE))
ASMFILES := $(ASMFILES_common) $(ASMFILES_$(PROFILE))
FILES_PATH := $(FILES_PATH_common) $(FILES_PATH_$(PROFILE))
CFLAGS := $(CFLAGS_common) $(CFLAGS_$(PROFILE))
ifeq ($(INSTRUMENTATION_MODE), yes)
CFLAGS += -fprofile-instr-generate -fcoverage-mapping
endif
DEFINES := $(DEFINES_common) $(DEFINES_$(PROFILE))
INCLUDES := $(INCLUDES_common) $(INCLUDE_$(PROFILE))
LIBS := $(LIBS_common) $(LIBS_$(PROFILE))
LIBS_PATH := $(LIBS_PATH_common) $(LIBS_PATH_$(PROFILE))
LFLAGS := $(LFLAGS_common) $(LFLAGS_$(PROFILE))
LNKOPTFLAGS := $(LNKOPTFLAGS_common) $(LNKOPTFLAGS_$(PROFILE))
LNK_FILES := $(LNK_FILES_common) $(LNK_FILES_$(PROFILE))

OBJDIR := obj/$(PROFILE)/
OBJS := $(FILES:%.c=%.obj)
OBJS += $(ASMFILES:%.S=%.obj)
DEPS := $(FILES:%.c=%.d)

vpath %.obj $(OBJDIR)
vpath %.c $(FILES_PATH)
vpath %.S $(FILES_PATH)
vpath %.lib $(LIBS_PATH_NAME)
vpath %.a $(LIBS_PATH_NAME)

$(OBJDIR)/%.obj %.obj: %.c
	@echo  Compiling: am263px:r5fss0-0:nortos:ti-arm-clang $(OUTNAME): $<
	$(CC) -c $(CFLAGS) $(INCLUDES) $(DEFINES) -MMD -o $(OBJDIR)/$@ $<

$(OBJDIR)/%.obj %.obj: %.S
	@echo  Compiling: am263px:r5fss0-0:nortos:ti-arm-clang $(LIBNAME): $<
	$(CC) -c $(CFLAGS) $(INCLUDES) $(DEFINES) -o $(OBJDIR)/$@ $<

all: $(TARGETS)

SYSCFG_GEN_FILES=generated/ti_drivers_config.c generated/ti_drivers_config.h
SYSCFG_GEN_FILES+=generated/ti_drivers_open_close.c generated/ti_drivers_open_close.h
SYSCFG_GEN_FILES+=generated/ti_dpl_config.c generated/ti_dpl_config.h
SYSCFG_GEN_FILES+=generated/ti_pinmux_config.c generated/ti_power_clock_config.c
SYSCFG_GEN_FILES+=generated/ti_board_config.c generated/ti_board_config.h
SYSCFG_GEN_FILES+=generated/ti_board_open_close.c generated/ti_board_open_close.h

SYSTEM_FLAG ?= false

ifeq ($(SYSTEM_FLAG), false)
	SYSTEM_COMMAND := syscfg $(SYSCFG_GEN_FILES) $(OBJS) $(LNK_FILES) $(LIBS_NAME)
else
	SYSTEM_COMMAND := $(OBJS) $(LNK_FILES) $(LIBS_NAME)
endif

$(OUTNAME):  $(SYSTEM_COMMAND)
	@echo  .
	@echo  Linking: am263px:r5fss0-0:nortos:ti-arm-clang $@ ...
	$(LNK) $(LNKOPTFLAGS) $(LFLAGS) $(LIBS_PATH) -Wl,-m=$(basename $@).map -o $@ $(addprefix $(OBJDIR), $(OBJS)) $(LIBS) $(LNK_FILES) -Wl,--xml_link_info=$(basename $@).lnkxml
	@echo  Linking: am263px:r5fss0-0:nortos:ti-arm-clang $@ Done !!!
	@echo  .

coreout: $(COREOUTNAME)
$(COREOUTNAME):  $(SYSTEM_COMMAND)
	@echo  .
	@echo  Relinking Linking: am263px:r5fss0-0:nortos:ti-arm-clang $@ with OptiShare SSO..
	$(LNK) $(LNKOPTFLAGS) $(LFLAGS) $(LIBS_PATH) -Wl,-m=$(basename $@).map -o $@ $(addprefix $(OBJDIR), $(OBJS)) $(LIBS) $(LNK_FILES) -Wl,--xml_link_info=$(basename $@).lnkxml -Wl,--import_sso=$(SSO_PATH)
	@echo  Relinking with optishare: am263px:r5fss0-0:freertos:ti-arm-clang $@ Done !!!
	@echo  .

clean:
	@echo  Cleaning: am263px:r5fss0-0:nortos:ti-arm-clang $(OUTNAME) ...
	$(RMDIR) $(OBJDIR)
	$(RM) $(OUTNAME)
	$(RM) $(BOOTIMAGE_NAME)
	$(RM) $(BOOTIMAGE_NAME_XIP)
	$(RM) $(BOOTIMAGE_NAME_SIGNED)
	$(RM) $(BOOTIMAGE_RPRC_NAME)
	$(RM) $(BOOTIMAGE_RPRC_NAME_XIP)
	$(RM) $(BOOTIMAGE_NAME_MCELF)
	$(RMDIR) generated/

scrub:
	@echo  Scrubing: am263px:r5fss0-0:nortos:ti-arm-clang adc_high_priority_soc ...
	$(RMDIR) obj
ifeq ($(OS),Windows_NT)
	$(RM) \*.out
	$(RM) \*.map
	$(RM) \*.appimage*
	$(RM) \*.rprc*
	$(RM) \*.tiimage*
	$(RM) \*.bin
	$(RM) \*.lnkxml
	$(RM) \*.ossr
	$(RM) \*.mcelf
	$(RM) \*.mcelf_xip
	$(RM) \*.mcelf-enc
	$(RM) \*.mcelf.hs
else
	$(RM) *.out
	$(RM) *.map
	$(RM) *.appimage*
	$(RM) *.rprc*
	$(RM) *.tiimage*
	$(RM) *.bin
	$(RM) *.lnkxml
	$(RM) *.ossr
	$(RM) *.mcelf
	$(RM) *.mcelf_xip
	$(RM) *.mcelf-enc
	$(RM) *.mcelf.hs
endif
	$(RMDIR) generated

$(OBJS): | $(OBJDIR)

$(OBJDIR):
	$(MKDIR) $@


.NOTPARALLEL:

.INTERMEDIATE: syscfg
$(SYSCFG_GEN_FILES): syscfg

ifeq ($(SYSTEM_FLAG), false)
syscfg: ../example.syscfg
	@echo Generating SysConfig files ...
	$(SYSCFG_NODE) $(SYSCFG_CLI_PATH)/dist/cli.js --product $(SYSCFG_SDKPRODUCT) --context r5fss0-0 --part AM263P4 --package ZCZ_C --output generated/ ../example.syscfg
endif

syscfg-gui:
	$(SYSCFG_GUI_SCRIPT) --product $(SYSCFG_SDKPRODUCT) --device AM263Px --context r5fss0-0 --part AM263P4 --package ZCZ_C --output generated/  ../example.syscfg

#
# Generation of boot image which can be loaded by Secondary Boot Loader (SBL)
#
ifeq ($(OS),Windows_NT)
EXE_EXT=.exe
endif
ifeq ($(OS),Windows_NT)
  BOOTIMAGE_CERT_GEN_CMD=powershell -executionpolicy unrestricted -command $(MCU_PLUS_SDK_PATH)/source/security/security_common/tools/boot/signing/x509CertificateGen.ps1
else
  BOOTIMAGE_CERT_GEN_CMD=$(MCU_PLUS_SDK_PATH)/source/security/security_common/tools/boot/signing/x509CertificateGen.sh
endif
BOOTIMAGE_TEMP_OUT_FILE=temp_stdout_$(PROFILE).txt


BOOTIMAGE_CORE_ID_r5fss0-0 = 0
BOOTIMAGE_CORE_ID_r5fss0-1 = 1
BOOTIMAGE_CORE_ID_r5fss1-0 = 2
BOOTIMAGE_CORE_ID_r5fss1-1 = 3
SBL_RUN_ADDRESS=0x70002000
SBL_DEV_ID=55

APP_IMAGE_SIGN_CMD = $(MCU_PLUS_SDK_PATH)/source/security/security_common/tools/boot/signing/mcu_appimage_x509_cert_gen.py
MCELF_IMAGE_GEN = $(MCU_PLUS_SDK_PATH)/tools/boot/multicore-elf/genimage.py

ifeq ($(OS),Windows_NT)
  XIPGEN_CMD=$(MCU_PLUS_SDK_PATH)/tools/boot/xipGen/xipGen.exe
else
  UNAME_S = $(shell uname -s)
  ifeq ($(UNAME_S), Darwin)
    XIPGEN_CMD=$(MCU_PLUS_SDK_PATH)/tools/boot/xipGen/xipGen.out.mac
  else
    XIPGEN_CMD=$(MCU_PLUS_SDK_PATH)/tools/boot/xipGen/xipGen.out
endif
endif

MULTI_CORE_IMAGE_PARAMS = \
	$(BOOTIMAGE_RPRC_NAME)@$(BOOTIMAGE_CORE_ID_r5fss0-0) \

MULTI_CORE_IMAGE_PARAMS_XIP = \
	$(BOOTIMAGE_RPRC_NAME_XIP)@$(BOOTIMAGE_CORE_ID_r5fss0-0) \

$(BOOTIMAGE_NAME_MCELF): $(OUTNAME)
	@echo  Boot MulticoreELF image: $(BOOTIMAGE_PATH)/$(BOOTIMAGE_NAME_MCELF) ...
	$(PYTHON) $(MCELF_IMAGE_GEN) --core-img=$(BOOTIMAGE_CORE_ID_r5fss0-0):$(OUTNAME) --output=$(BOOTIMAGE_NAME_MCELF) --merge-segments=$(MCELF_MERGE_SEGMENTS_FLAG) --tolerance-limit=$(MCELF_MERGE_SEGMENTS_TOLERANCE_LIMIT) --ignore-context=$(MCELF_IGNORE_CONTEXT_FLAG) --xip=$(MCELF_XIP_RANGE) --xlat=$(MCELF_ADDR_TRANSLATION_PATH) --max-segment-size=$(MCELF_MAX_SEGMENT_SIZE) --otfaConfigFile=$(oeconfig)
	@echo  Boot MulticoreELF ELF image: $(BOOTIMAGE_PATH)/$(BOOTIMAGE_NAME_MCELF) Done !!!
	@echo  .

$(BOOTIMAGE_NAME_MCELF_HS): $(BOOTIMAGE_NAME_MCELF)
ifeq ($(DEVICE_TYPE), HS)
# Sign the appimage using appimage signing script
ifeq ($(ENC_ENABLED),no)
ifeq ($(RSASSAPSS_ENABLED),no)
	@echo Boot image signing: Encryption is disabled.
	$(PYTHON) $(APP_IMAGE_SIGN_CMD) --bin $(BOOTIMAGE_NAME_MCELF) --key $(APP_SIGNING_KEY) --sign_key_id $(APP_SIGNING_KEY_KEYRING_ID) --hash_algo $(APP_SIGNING_HASH_ALGO) --output $(BOOTIMAGE_NAME_MCELF_HS)
else
	@echo Boot image signing: Encryption is disabled. RSASSAPSS is enabled.
	$(PYTHON) $(APP_IMAGE_SIGN_CMD) --bin $(BOOTIMAGE_NAME_MCELF) --key $(APP_SIGNING_KEY) --sign_key_id $(APP_SIGNING_KEY_KEYRING_ID) --hash_algo $(APP_SIGNING_HASH_ALGO) --pss_saltlen $(APP_SIGNING_SALT_LENGTH) --output $(BOOTIMAGE_NAME_MCELF_HS) --rsassa_pss
endif
else
ifeq ($(RSASSAPSS_ENABLED),no)
	@echo Boot image signing: Encryption is enabled.
	$(PYTHON) $(APP_IMAGE_SIGN_CMD) --bin $(BOOTIMAGE_NAME_MCELF) --key $(APP_SIGNING_KEY) --enc y --enckey $(APP_ENCRYPTION_KEY) --kd-salt $(KD_SALT) --sign_key_id $(APP_SIGNING_KEY_KEYRING_ID) --enc_key_id $(APP_ENCRYPTION_KEY_KEYRING_ID) --hash_algo $(APP_SIGNING_HASH_ALGO) --output $(BOOTIMAGE_NAME_MCELF_HS)
	$(RM) $(BOOTIMAGE_NAME_MCELF)-enc
else
	@echo Boot image signing: Encryption is enabled. RSASSAPSS is enabled.
	$(PYTHON) $(APP_IMAGE_SIGN_CMD) --bin $(BOOTIMAGE_NAME_MCELF) --key $(APP_SIGNING_KEY) --enc y --enckey $(APP_ENCRYPTION_KEY) --kd-salt $(KD_SALT) --sign_key_id $(APP_SIGNING_KEY_KEYRING_ID) --enc_key_id $(APP_ENCRYPTION_KEY_KEYRING_ID) --hash_algo $(APP_SIGNING_HASH_ALGO) --pss_saltlen $(APP_SIGNING_SALT_LENGTH) --output $(BOOTIMAGE_NAME_MCELF_HS) --rsassa_pss
	$(RM) $(BOOTIMAGE_NAME_MCELF)-enc
endif
endif
	@echo  Boot image: am263px:r5fss0-0:nortos:ti-arm-clang $(BOOTIMAGE_PATH)/$(BOOTIMAGE_NAME_MCELF_HS) Done !!!
	@echo  .
endif



.PHONY: coverage
coverage:
	@echo Creating Coverage Report for adc_high_priority_soc.$(PROFILE) ...
	$(MKDIR) coverage
	$(PROFDATA) merge -sparse -obj-file=$(OUTNAME) $(OUTNAME).cnt -o adc_high_priority_soc.$(PROFILE).profdata
	$(COV) show --format=html --show-expansions --show-instantiations --show-branches=count --object=$(OUTNAME) -instr-profile=adc_high_priority_soc.$(PROFILE).profdata --output-dir=$(COVERAGE_PATH)/coverage --ignore-filename-regex=build_jenkins
	$(COV) export --format=text --object=$(OUTNAME) --instr-profile=adc_high_priority_soc.$(PROFILE).profdata > coverage/adc_high_priority_soc.$(PROFILE).profdata.json
	node $(MCU_PLUS_SDK_PATH)/tools/smart_placement/clang_coverage_analyse.js --input=coverage/adc_high_priority_soc.$(PROFILE).profdata.json --output-json=coverage/adc_high_priority_soc.$(PROFILE).analysis.json --output=../adc_high_priority_soc.annotations.$(PROFILE).S --top-function-count=500
	@echo Coverage Report Generated at $(COVERAGE_PATH)/coverage folder !!!

-include $(addprefix $(OBJDIR)/, $(DEPS))
