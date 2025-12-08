
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

OUTNAME:=sbl_null.$(PROFILE).out
COREOUTNAME:=sbl_null.$(PROFILE).optishare.out

BOOTIMAGE_PATH=$(abspath .)
BOOTIMAGE_NAME_GP:=sbl_null.$(PROFILE).tiimage
BOOTIMAGE_NAME_HS:=sbl_null.$(PROFILE).hs.tiimage
BOOTIMAGE_NAME_HS_FS:=sbl_null.$(PROFILE).hs_fs.tiimage
BOOTIMAGE_BIN_NAME:=sbl_null.$(PROFILE).bin

ifeq ($(DEVICE_TYPE),HS)
	BOOTIMAGE_NAME=$(BOOTIMAGE_NAME_HS)
else
	BOOTIMAGE_NAME=$(BOOTIMAGE_NAME_GP)
endif

FILES_common := \
	main.c \
	ti_drivers_config.c \
	ti_drivers_open_close.c \
	ti_board_config.c \
	ti_board_open_close.c \
	ti_dpl_config.c \
	ti_pinmux_config.c \
	ti_power_clock_config.c \
	ti_clocktree_pll_config.c \

FILES_PATH_common = \
	.. \
	../../.. \
	generated \

INCLUDES_common := \
	-I${CG_TOOL_ROOT}/include/c \
	-I${MCU_PLUS_SDK_PATH}/source \
	-I${MCU_PLUS_SDK_PATH}/source/security \
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
	-Wl,-i${MCU_PLUS_SDK_PATH}/source/sdl/lib \
	-Wl,-i${MCU_PLUS_SDK_PATH}/source/security/lib \
	-Wl,-i${CG_TOOL_ROOT}/lib \

LIBS_common = \
	-lnortos.am263px.r5f.ti-arm-clang.${ConfigName}.lib \
	-ldrivers.am263px.r5f.ti-arm-clang.nortos.${ConfigName}.lib \
	-lboard.am263px.r5f.ti-arm-clang.nortos.${ConfigName}.lib \
	-lsdl.am263px.r5f.ti-arm-clang.nortos.${ConfigName}.lib \
	-lsecurity.am263px.r5f.ti-arm-clang.nortos.${ConfigName}.lib \
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
	sdl.am263px.r5f.ti-arm-clang.nortos.${ConfigName}.lib \
	security.am263px.r5f.ti-arm-clang.nortos.${ConfigName}.lib \
	libc.a \
	libsysbm.a \

LIBS_PATH_NAME = \
	${MCU_PLUS_SDK_PATH}/source/kernel/nortos/lib \
	${MCU_PLUS_SDK_PATH}/source/drivers/lib \
	${MCU_PLUS_SDK_PATH}/source/board/lib \
	${MCU_PLUS_SDK_PATH}/source/sdl/lib \
	${MCU_PLUS_SDK_PATH}/source/security/lib \
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

all: $(BOOTIMAGE_NAME)

SYSCFG_GEN_FILES=generated/ti_drivers_config.c generated/ti_drivers_config.h
SYSCFG_GEN_FILES+=generated/ti_drivers_open_close.c generated/ti_drivers_open_close.h
SYSCFG_GEN_FILES+=generated/ti_dpl_config.c generated/ti_dpl_config.h
SYSCFG_GEN_FILES+=generated/ti_pinmux_config.c generated/ti_power_clock_config.c
SYSCFG_GEN_FILES+=generated/ti_board_config.c generated/ti_board_config.h
SYSCFG_GEN_FILES+=generated/ti_board_open_close.c generated/ti_board_open_close.h
SYSCFG_GEN_FILES+=generated/ti_clocktree_pll_config.c generated/ti_clocktree_pll_config.h

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
	$(RM) $(BOOTIMAGE_BIN_NAME)
	$(RMDIR) generated/

scrub:
	@echo  Scrubing: am263px:r5fss0-0:nortos:ti-arm-clang sbl_null ...
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
else
	$(RM) *.out
	$(RM) *.map
	$(RM) *.appimage*
	$(RM) *.rprc*
	$(RM) *.tiimage*
	$(RM) *.bin
	$(RM) *.lnkxml
	$(RM) *.ossr
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
# Generation of boot image which can be loaded by ROM Boot Loader (RBL)
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



ifeq ($(DEVICE_TYPE),HS)
BOOTIMAGE_CERT_KEY=$(APP_SIGNING_KEY)
else
BOOTIMAGE_CERT_KEY=$(MCU_PLUS_SDK_PATH)/source/security/security_common/tools/boot/signing/mcu_gpkey.pem
endif
BOOTIMAGE_CERT_GEN_CMD=$(PYTHON) $(MCU_PLUS_SDK_PATH)/source/security/security_common/tools/boot/signing/mcu_rom_image_gen.py

SBL_RUN_ADDRESS=0x70002000

SBL_PREBUILT_PATH=$(MCU_PLUS_SDK_PATH)/tools/boot/sbl_prebuilt/am263px-lp

$(BOOTIMAGE_BIN_NAME): $(OUTNAME)
	$(OBJCOPY) --strip-all -O binary $(OUTNAME) $(BOOTIMAGE_BIN_NAME)

$(BOOTIMAGE_NAME): $(BOOTIMAGE_BIN_NAME)
	@echo  Boot image: am263px:r5fss0-0:nortos:ti-arm-clang $(BOOTIMAGE_PATH)/$@ ...
ifeq ($(DEVICE_TYPE),HS)
ifeq ($(DEBUG_TIFS), yes)
ifeq ($(ENC_SBL_ENABLED),yes)
	$(BOOTIMAGE_CERT_GEN_CMD) --sbl-enc --enc-key $(APP_ENCRYPTION_KEY) --image-bin $(BOOTIMAGE_PATH)/$(BOOTIMAGE_BIN_NAME) --core R5 --swrv 1 --loadaddr $(SBL_RUN_ADDRESS) --sign-key $(BOOTIMAGE_CERT_KEY) --kd-salt $(KD_SALT) --out-image $(BOOTIMAGE_PATH)/$(BOOTIMAGE_NAME) --device am263px
else
	$(BOOTIMAGE_CERT_GEN_CMD) --image-bin $(BOOTIMAGE_PATH)/$(BOOTIMAGE_BIN_NAME) --core R5 --swrv 1 --loadaddr $(SBL_RUN_ADDRESS) --sign-key $(BOOTIMAGE_CERT_KEY) --out-image $(BOOTIMAGE_PATH)/$(BOOTIMAGE_NAME) --device am263px
endif
else
ifeq ($(ENC_SBL_ENABLED),yes)
	$(BOOTIMAGE_CERT_GEN_CMD) --sbl-enc --enc-key $(APP_ENCRYPTION_KEY) --image-bin $(BOOTIMAGE_PATH)/$(BOOTIMAGE_BIN_NAME) --core R5 --swrv 1 --loadaddr $(SBL_RUN_ADDRESS) --sign-key $(BOOTIMAGE_CERT_KEY) --kd-salt $(KD_SALT) --out-image $(BOOTIMAGE_PATH)/$(BOOTIMAGE_NAME) --debug $(DEBUG_OPTION) --device am263px
else
	$(BOOTIMAGE_CERT_GEN_CMD) --image-bin $(BOOTIMAGE_PATH)/$(BOOTIMAGE_BIN_NAME) --core R5 --swrv 1 --loadaddr $(SBL_RUN_ADDRESS) --sign-key $(BOOTIMAGE_CERT_KEY) --out-image $(BOOTIMAGE_PATH)/$(BOOTIMAGE_NAME) --debug $(DEBUG_OPTION) --device am263px
endif
endif
else
	$(BOOTIMAGE_CERT_GEN_CMD) --image-bin $(BOOTIMAGE_PATH)/$(BOOTIMAGE_BIN_NAME) --core R5 --swrv 1 --loadaddr $(SBL_RUN_ADDRESS) --sign-key $(BOOTIMAGE_CERT_KEY) --out-image $(BOOTIMAGE_PATH)/$(BOOTIMAGE_NAME) --debug DBG_SOC_DEFAULT --device am263px
endif
	$(COPY) $(BOOTIMAGE_PATH)/$(BOOTIMAGE_NAME) $(SBL_PREBUILT_PATH)/

	$(RM) $(BOOTIMAGE_TEMP_OUT_FILE)
	@echo  Boot image: am263px:r5fss0-0:nortos:ti-arm-clang $(BOOTIMAGE_PATH)/$@ Done !!!

	@echo  .




.PHONY: coverage
coverage:
	@echo Creating Coverage Report for sbl_null.$(PROFILE) ...
	$(MKDIR) coverage
	$(PROFDATA) merge -sparse -obj-file=$(OUTNAME) $(OUTNAME).cnt -o sbl_null.$(PROFILE).profdata
	$(COV) show --format=html --show-expansions --show-instantiations --show-branches=count --object=$(OUTNAME) -instr-profile=sbl_null.$(PROFILE).profdata --output-dir=$(COVERAGE_PATH)/coverage --ignore-filename-regex=build_jenkins
	$(COV) export --format=text --object=$(OUTNAME) --instr-profile=sbl_null.$(PROFILE).profdata > coverage/sbl_null.$(PROFILE).profdata.json
	node $(MCU_PLUS_SDK_PATH)/tools/smart_placement/clang_coverage_analyse.js --input=coverage/sbl_null.$(PROFILE).profdata.json --output-json=coverage/sbl_null.$(PROFILE).analysis.json --output=../sbl_null.annotations.$(PROFILE).S --top-function-count=500
	@echo Coverage Report Generated at $(COVERAGE_PATH)/coverage folder !!!

-include $(addprefix $(OBJDIR)/, $(DEPS))
