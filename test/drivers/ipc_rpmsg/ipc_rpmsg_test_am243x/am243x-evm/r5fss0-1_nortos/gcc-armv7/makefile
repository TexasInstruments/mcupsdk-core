
export MCU_PLUS_SDK_PATH?=$(abspath ../../../../../../..)
include $(MCU_PLUS_SDK_PATH)/imports.mak
include $(MCU_PLUS_SDK_PATH)/devconfig/devconfig.mak

CG_TOOL_ROOT=$(CGT_GCC_ARMV7_PATH)

CC=$(CG_TOOL_ROOT)/bin/arm-none-eabi-gcc
LNK=$(CG_TOOL_ROOT)/bin/arm-none-eabi-gcc
STRIP=$(CG_TOOL_ROOT)/bin/arm-none-eabi-strip
OBJCOPY=$(CG_TOOL_ROOT)/bin/arm-none-eabi-objcopy
ifeq ($(OS), Windows_NT)
	PYTHON=python
else
	PYTHON=python3
endif

PROFILE?=release
ConfigName:=$(PROFILE)

OUTNAME:=ipc_rpmsg_ut.$(PROFILE).out
COREOUTNAME:=ipc_rpmsg_ut.$(PROFILE).optishare.out

BOOTIMAGE_PATH=$(abspath .)
oeconfig?=None

BOOTIMAGE_NAME_MCELF:=ipc_rpmsg_ut.$(PROFILE).mcelf
BOOTIMAGE_NAME_MCELF_HS:=ipc_rpmsg_ut.$(PROFILE).mcelf.hs
BOOTIMAGE_NAME_HS_FS:=ipc_rpmsg_ut.$(PROFILE).mcelf.hs_fs
TARGETS := $(BOOTIMAGE_NAME_MCELF)
ifeq ($(DEVICE_TYPE), HS)
   TARGETS += $(BOOTIMAGE_NAME_MCELF_HS)
else
   TARGETS += $(BOOTIMAGE_NAME_HS_FS)
endif

FILES_common := \
	test_ipc_rpmsg_am243.c \
	ipc_rpmsg_pos.c \
	ipc_rpmsg_neg.c \
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
	-I${CG_TOOL_ROOT}/arm-none-eabi/include \
	-I${MCU_PLUS_SDK_PATH}/source \
	-I${MCU_PLUS_SDK_PATH}/test/unity/ \
	-Igenerated \

DEFINES_common := \
	-DSOC_AM243X \
	-DOS_NORTOS \

CFLAGS_common := \
	-mcpu=cortex-r5 \
	-mfloat-abi=hard \
	-mfpu=vfpv3-d16 \
	-Wall \
	-Werror \
	-mthumb \
	-g \
	-Wno-unused-function \
	-Wno-enum-compare \
	-Wno-uninitialized \
	-Wno-address-of-packed-member \
	-Wno-int-to-pointer-cast \
	-Wno-stringop-truncation \
	-fgnu89-inline \
	-Wno-pointer-to-int-cast \
	-Wno-unused-variable \
	-Wno-unused-but-set-variable \
	-fdata-sections \
	-ffunction-sections \

CFLAGS_debug := \
	-D_DEBUG_=1 \

CFLAGS_release := \
	-O2 \

LNK_FILES_common = \
	linker.cmd \

LIBS_PATH_common = \
	-L${MCU_PLUS_SDK_PATH}/source/kernel/nortos/lib \
	-L${MCU_PLUS_SDK_PATH}/source/drivers/lib \
	-L${MCU_PLUS_SDK_PATH}/test/unity/lib \
	-L${CGT_GCC_ARMV7_PATH}/lib \

LIBS_common = \
	-l:nortos.am243x.r5f.gcc-armv7.${ConfigName}.lib \
	-l:drivers.am243x.r5f.gcc-armv7.nortos.${ConfigName}.lib \
	-l:unity.am243x.r5f.gcc-armv7.nortos.${ConfigName}.lib \

LFLAGS_common = \
	-mcpu=cortex-r5 \
	-mfloat-abi=hard \
	-mfpu=vfpv3-d16 \
	-Wl,-static \
	-Wl,--gc-sections \
	-Wl,--build-id=none \
	-lstdc++ \
	-lgcc \
	-lm \
	-lc \
	-lnosys \


LIBS_NAME = \
	nortos.am243x.r5f.gcc-armv7.${ConfigName}.lib \
	drivers.am243x.r5f.gcc-armv7.nortos.${ConfigName}.lib \
	unity.am243x.r5f.gcc-armv7.nortos.${ConfigName}.lib \

LIBS_PATH_NAME = \
	${MCU_PLUS_SDK_PATH}/source/kernel/nortos/lib \
	${MCU_PLUS_SDK_PATH}/source/drivers/lib \
	${MCU_PLUS_SDK_PATH}/test/unity/lib \
	${CGT_GCC_ARMV7_PATH}/lib \

FILES := $(FILES_common) $(FILES_$(PROFILE))
ASMFILES := $(ASMFILES_common) $(ASMFILES_$(PROFILE))
FILES_PATH := $(FILES_PATH_common) $(FILES_PATH_$(PROFILE))
CFLAGS := $(CFLAGS_common) $(CFLAGS_$(PROFILE))
DEFINES := $(DEFINES_common) $(DEFINES_$(PROFILE))
INCLUDES := $(INCLUDES_common) $(INCLUDE_$(PROFILE))
LIBS := $(LIBS_common) $(LIBS_$(PROFILE))
LIBS_PATH := $(LIBS_PATH_common) $(LIBS_PATH_$(PROFILE))
LFLAGS := $(LFLAGS_common) $(LFLAGS_$(PROFILE))
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
	@echo  Compiling: am243x:r5fss0-1:nortos:gcc-armv7 $(OUTNAME): $<
	$(CC) -c $(CFLAGS) $(INCLUDES) $(DEFINES) -MMD -MT $@ -o $(OBJDIR)/$@ $<

$(OBJDIR)/%.obj %.obj: %.S
	@echo  Compiling: am243x:r5fss0-1:nortos:gcc-armv7 $(LIBNAME): $<
	$(CC) -c -x assembler-with-cpp $(CFLAGS) $(INCLUDES) $(DEFINES) -o $(OBJDIR)$@ $<

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
	@echo  Linking: am243x:r5fss0-1:nortos:gcc-armv7 $@ ...
	$(LNK) $(LIBS_PATH) -Wl,-Map=$(OUTNAME:%.out=%.map) -o $@ -Wl,--start-group $(LFLAGS) $(addprefix $(OBJDIR), $(OBJS)) $(LIBS) -Wl,--end-group -Wl,-T,$(LNK_FILES)
	@echo  Linking: am243x:r5fss0-1:nortos:gcc-armv7 $@ Done !!!
	@echo  .


clean:
	@echo  Cleaning: am243x:r5fss0-1:nortos:gcc-armv7 $(OUTNAME) ...
	$(RMDIR) $(OBJDIR)
	$(RM) $(OUTNAME)
	$(RM) $(BOOTIMAGE_NAME)
	$(RM) $(BOOTIMAGE_NAME_XIP)
	$(RM) $(BOOTIMAGE_NAME_SIGNED)
	$(RM) $(BOOTIMAGE_NAME_HS)
	$(RM) $(BOOTIMAGE_NAME_HS_FS)
	$(RM) $(BOOTIMAGE_RPRC_NAME)
	$(RM) $(BOOTIMAGE_RPRC_NAME_XIP)
	$(RM) $(BOOTIMAGE_NAME_MCELF)
	$(RMDIR) generated/

scrub:
	@echo  Scrubing: am243x:r5fss0-1:nortos:gcc-armv7 ipc_rpmsg_ut ...
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
	$(RM) \*.mcelf.hs_fs
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
	$(RM) *.mcelf.hs_fs
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
	$(SYSCFG_NODE) $(SYSCFG_CLI_PATH)/dist/cli.js --product $(SYSCFG_SDKPRODUCT) --context r5fss0-1 --part ALV --package ALV --output generated/ --compiler gcc ../example.syscfg
endif

syscfg-gui:
	$(SYSCFG_NWJS) $(SYSCFG_PATH) --product $(SYSCFG_SDKPRODUCT) --device AM243x_ALV_beta --context r5fss0-1 --part ALV --package ALV --output generated/ --compiler gcc ../example.syscfg

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

BOOTIMAGE_CERT_KEY=$(APP_SIGNING_KEY)

BOOTIMAGE_CORE_ID_r5fss0-0 = 4
BOOTIMAGE_CORE_ID_r5fss0-1 = 5
BOOTIMAGE_CORE_ID_r5fss1-0 = 6
BOOTIMAGE_CORE_ID_r5fss1-1 = 7
BOOTIMAGE_CORE_ID_m4fss0-0 = 14
SBL_RUN_ADDRESS=0x70000000
SBL_DEV_ID=55

APP_IMAGE_SIGN_CMD = $(MCU_PLUS_SDK_PATH)/source/security/security_common/tools/boot/signing/appimage_x509_cert_gen.py
MCELF_IMAGE_GEN = $(MCU_PLUS_SDK_PATH)/tools/boot/multicore-elf/genimage_am64x.py

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
	$(BOOTIMAGE_RPRC_NAME)@$(BOOTIMAGE_CORE_ID_r5fss0-1) \

MULTI_CORE_IMAGE_PARAMS_XIP = \
	$(BOOTIMAGE_RPRC_NAME_XIP)@$(BOOTIMAGE_CORE_ID_r5fss0-1) \

$(BOOTIMAGE_NAME_MCELF): $(OUTNAME)
	@echo  Boot MulticoreELF image: $(BOOTIMAGE_PATH)/$(BOOTIMAGE_NAME_MCELF) ...
	$(PYTHON) $(MCELF_IMAGE_GEN) --core-img=$(BOOTIMAGE_CORE_ID_r5fss0-1):$(OUTNAME) --output=$(BOOTIMAGE_NAME_MCELF) --xip=$(MCELF_XIP_RANGE)
	@echo  Boot MulticoreELF ELF image: $(BOOTIMAGE_PATH)/$(BOOTIMAGE_NAME_MCELF) Done !!!
	@echo  .

ifeq ($(DEVICE_TYPE), HS)
# Sign the appimage using appimage signing script
ifeq ($(ENC_ENABLED),no)
	@echo Boot image signing: Encryption is disabled.
	$(PYTHON) $(APP_IMAGE_SIGN_CMD) --bin $(BOOTIMAGE_NAME_MCELF) --authtype 1 --key $(APP_SIGNING_KEY) --output $(BOOTIMAGE_NAME_MCELF_HS)
else
	@echo Boot image signing: Encryption is enabled.
	$(PYTHON) $(APP_IMAGE_SIGN_CMD) --bin $(BOOTIMAGE_NAME_MCELF) --authtype 1 --key $(APP_SIGNING_KEY) --enc y --enckey $(APP_ENCRYPTION_KEY) --output $(BOOTIMAGE_NAME_MCELF_HS)
	$(RM) $(BOOTIMAGE_NAME_MCELF)-enc
endif
	$(RM) $(BOOTIMAGE_NAME_MCELF)
	@echo  Boot image: am243x:r5fss0-1:nortos:gcc-armv7 $(BOOTIMAGE_PATH)/$(BOOTIMAGE_NAME_MCELF_HS) Done !!!
	@echo  .
else
# Sign the appimage for HS-FS using appimage signing script
	$(PYTHON) $(APP_IMAGE_SIGN_CMD) --bin $(BOOTIMAGE_NAME_MCELF) --authtype 1 --key $(APP_SIGNING_KEY) --output $(BOOTIMAGE_NAME_HS_FS)
	$(RM) $(BOOTIMAGE_NAME_MCELF)
	@echo  Boot image: am243x:r5fss0-1:nortos:gcc-armv7 $(BOOTIMAGE_PATH)/$(BOOTIMAGE_NAME_HS_FS) Done !!!
	@echo  .
endif



-include $(addprefix $(OBJDIR)/, $(DEPS))
