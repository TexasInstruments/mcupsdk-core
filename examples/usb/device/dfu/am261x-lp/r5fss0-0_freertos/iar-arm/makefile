
export MCU_PLUS_SDK_PATH?=$(abspath ../../../../../../..)
include $(MCU_PLUS_SDK_PATH)/imports.mak
include $(MCU_PLUS_SDK_PATH)/devconfig/devconfig.mak

CG_TOOL_ROOT=$(CGT_IAR_ARM_PATH)

CC=$(CG_TOOL_ROOT)/bin/iccarm
LNK=$(CG_TOOL_ROOT)/bin/ilinkarm
AS=$(CGT_IAR_ARM_PATH)/bin/iasmarm
STRIP=$(CG_TOOL_ROOT)/bin/
OBJCOPY=$(CG_TOOL_ROOT)/bin/
ifeq ($(OS), Windows_NT)
	PYTHON=python
else
	PYTHON=python3
endif

PROFILE?=release
ConfigName:=$(PROFILE)

OUTNAME:=usb_dfu.$(PROFILE).out
COREOUTNAME:=usb_dfu.$(PROFILE).optishare.out

BOOTIMAGE_PATH=$(abspath .)
oeconfig?=None

BOOTIMAGE_NAME_MCELF:=usb_dfu.$(PROFILE).mcelf
BOOTIMAGE_NAME_MCELF_HS:=usb_dfu.$(PROFILE).mcelf.hs
TARGETS := $(BOOTIMAGE_NAME_MCELF)
ifeq ($(DEVICE_TYPE), HS)
   TARGETS += $(BOOTIMAGE_NAME_MCELF_HS)
else
endif

FILES_common := \
	dfu_nortos.c \
	main.c \
	ti_drivers_config.c \
	ti_drivers_open_close.c \
	ti_board_config.c \
	ti_board_open_close.c \
	ti_dpl_config.c \
	ti_pinmux_config.c \
	ti_power_clock_config.c \
	ti_usb_descriptor.c \

FILES_PATH_common = \
	.. \
	../../.. \
	generated \

INCLUDES_common := \
	-I${MCU_PLUS_SDK_PATH}/source \
	-I${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-Kernel/include \
	-I${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/am261x/r5f \
	-I${MCU_PLUS_SDK_PATH}/source/usb/tinyusb/config/nortos/am261x/dfu_config \
	-I${MCU_PLUS_SDK_PATH}/source/usb/tinyusb/tinyusb-stack/src \
	-I${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/IAR/ARM_CR5F \
	-I${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/IAR_ARM/ARM_CR5F \
	-Igenerated \

DEFINES_common := \
	-DSOC_AM261X \
	-DTINYUSB_INTEGRATION \
	-DOS_FREERTOS \

CFLAGS_common := \
	--cpu=Cortex-R5 \
	--fpu=VFPv3_D16 \
	--cpu_mode thumb \
	--debug \
	--diag_suppress=Pa039 \
	--diag_suppress=Pe177 \
	-e \
	--endian=little \
	--silent \

CFLAGS_cpp_common := \

CFLAGS_debug := \
	-D_DEBUG_=1 \

CFLAGS_release := \
	-Oh \

ASMFLAGS_common := \
    --cpu=Cortex-R5 \
    --fpu=VFPv3_D16 \

LNK_FILES_common = \
	generated/linker.icf \

LIBS_PATH_common = \
	-L${MCU_PLUS_SDK_PATH}/source/kernel/freertos/lib \
	-L${MCU_PLUS_SDK_PATH}/source/drivers/lib \
	-L${MCU_PLUS_SDK_PATH}/source/board/lib \
	-L${MCU_PLUS_SDK_PATH}/source/usb/synp/lib \
	-L${MCU_PLUS_SDK_PATH}/source/usb/tinyusb/lib \

LIBS_common = \
	freertos.am261x.r5f.iar-arm.${ConfigName}.lib \
	drivers.am261x.r5f.iar-arm.freertos.${ConfigName}.lib \
	board.am261x.r5f.iar-arm.freertos.${ConfigName}.lib \
	usbd_synp_freertos.am261x.r5f.iar-arm.${ConfigName}.lib \
	usbd_tusb_dfu_nortos.am261x.r5f.iar-arm.${ConfigName}.lib \

LFLAGS_common = \
	--silent \
	--entry __iar_program_start \
	--semihosting=iar_breakpoint \
	--redirect __write=__write_buffered \


LIBS_NAME = \
	freertos.am261x.r5f.iar-arm.${ConfigName}.lib \
	drivers.am261x.r5f.iar-arm.freertos.${ConfigName}.lib \
	board.am261x.r5f.iar-arm.freertos.${ConfigName}.lib \
	usbd_synp_freertos.am261x.r5f.iar-arm.${ConfigName}.lib \
	usbd_tusb_dfu_nortos.am261x.r5f.iar-arm.${ConfigName}.lib \

LIBS_PATH_NAME = \
	${MCU_PLUS_SDK_PATH}/source/kernel/freertos/lib \
	${MCU_PLUS_SDK_PATH}/source/drivers/lib \
	${MCU_PLUS_SDK_PATH}/source/board/lib \
	${MCU_PLUS_SDK_PATH}/source/usb/synp/lib \
	${MCU_PLUS_SDK_PATH}/source/usb/tinyusb/lib \

FILES := $(FILES_common) $(FILES_$(PROFILE))
ASMFILES := $(ASMFILES_common) $(ASMFILES_$(PROFILE))
FILES_PATH := $(FILES_PATH_common) $(FILES_PATH_$(PROFILE))
CFLAGS := $(CFLAGS_common) $(CFLAGS_$(PROFILE))
ASMFLAGS := $(ASMFLAGS_common)
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
	@echo  Compiling: am261x:r5fss0-0:freertos:iar-arm $(OUTNAME): $<
	$(CC) $(CFLAGS) $(INCLUDES) $(DEFINES) -o $(OBJDIR)/$@ $<

$(OBJDIR)/%.obj %.obj: %.S
	@echo  Compiling: am261x:r5fss0-0:freertos:iar-arm $(LIBNAME): $<
	$(AS) $(ASMFLAGS) $(INCLUDES) $(DEFINES) -o $(OBJDIR)/$@ $<

all: $(TARGETS)

SYSCFG_GEN_FILES=generated/ti_drivers_config.c generated/ti_drivers_config.h
SYSCFG_GEN_FILES+=generated/ti_drivers_open_close.c generated/ti_drivers_open_close.h
SYSCFG_GEN_FILES+=generated/ti_dpl_config.c generated/ti_dpl_config.h
SYSCFG_GEN_FILES+=generated/ti_pinmux_config.c generated/ti_power_clock_config.c
SYSCFG_GEN_FILES+=generated/ti_board_config.c generated/ti_board_config.h
SYSCFG_GEN_FILES+=generated/ti_board_open_close.c generated/ti_board_open_close.h
SYSCFG_GEN_FILES+=generated/ti_usb_descriptor.c generated/ti_usb_config.h

SYSTEM_FLAG ?= false

ifeq ($(SYSTEM_FLAG), false)
	SYSTEM_COMMAND := syscfg $(SYSCFG_GEN_FILES) $(OBJS) $(LNK_FILES) $(LIBS_NAME)
else
	SYSTEM_COMMAND := $(OBJS) $(LNK_FILES) $(LIBS_NAME)
endif

$(OUTNAME):  $(SYSTEM_COMMAND)
	@echo  .
	@echo  Linking: am261x:r5fss0-0:freertos:iar-arm $@ ...
	$(LNK) $(LFLAGS) $(addprefix $(OBJDIR), $(OBJS)) -o $@ $(LIBS) $(LIBS_PATH) --config $(LNK_FILES) --map $(basename $@).map $(ASMFLAGS_common)
	@echo  Linking: am261x:r5fss0-0:freertos:iar-arm $@ Done !!!
	@echo  .


clean:
	@echo  Cleaning: am261x:r5fss0-0:freertos:iar-arm $(OUTNAME) ...
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
	@echo  Scrubing: am261x:r5fss0-0:freertos:iar-arm usb_dfu ...
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
syscfg: example.syscfg
	@echo Generating SysConfig files ...
	$(SYSCFG_NODE) $(SYSCFG_CLI_PATH)/dist/cli.js --product $(SYSCFG_SDKPRODUCT) --context r5fss0-0 --part AM2612 --package ZFG --output generated/ example.syscfg
endif

syscfg-gui:
	$(SYSCFG_GUI_SCRIPT) --product $(SYSCFG_SDKPRODUCT) --device AM261x_ZFG --context r5fss0-0 --part AM2612 --package ZFG --output generated/  example.syscfg

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
	@echo  Boot image: am261x:r5fss0-0:freertos:iar-arm $(BOOTIMAGE_PATH)/$(BOOTIMAGE_NAME_MCELF_HS) Done !!!
	@echo  .
endif



-include $(addprefix $(OBJDIR)/, $(DEPS))
