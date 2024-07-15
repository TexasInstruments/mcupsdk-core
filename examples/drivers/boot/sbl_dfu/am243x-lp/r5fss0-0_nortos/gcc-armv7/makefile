#
# Auto generated makefile
#

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

OUTNAME:=sbl_dfu.$(PROFILE).out

BOOTIMAGE_PATH=$(abspath .)
BOOTIMAGE_NAME_GP:=sbl_dfu.$(PROFILE).tiimage
BOOTIMAGE_NAME_HS:=sbl_dfu.$(PROFILE).hs.tiimage
BOOTIMAGE_NAME_HS_FS:=sbl_dfu.$(PROFILE).hs_fs.tiimage
BOOTIMAGE_BIN_NAME:=sbl_dfu.$(PROFILE).bin

ifeq ($(DEVICE_TYPE),HS)
	BOOTIMAGE_NAME=$(BOOTIMAGE_NAME_HS)
else
	BOOTIMAGE_NAME=$(BOOTIMAGE_NAME_HS_FS)
endif

FILES_common := \
	dfu_descriptors.c \
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
	-I${MCU_PLUS_SDK_PATH}/source/usb/tinyusb/config/nortos/am64x_am243x/dfu_config \
	-I${MCU_PLUS_SDK_PATH}/source/usb/tinyusb/tinyusb-stack/src \
	-I${MCU_PLUS_SDK_PATH}/source/usb/tinyusb/tinyusb-stack/src/device \
	-Igenerated \

DEFINES_common := \
	-DSOC_AM243X \
	-DTINYUSB_INTEGRATION \

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
	-L${MCU_PLUS_SDK_PATH}/source/board/lib \
	-L${MCU_PLUS_SDK_PATH}/source/usb/cdn/lib \
	-L${MCU_PLUS_SDK_PATH}/source/usb/tinyusb/lib \
	-L${CGT_GCC_ARMV7_PATH}/lib \

LIBS_common = \
	-l:nortos.am243x.r5f.gcc-armv7.${ConfigName}.lib \
	-l:drivers.am243x.r5f.gcc-armv7.${ConfigName}.lib \
	-l:board.am243x.r5f.gcc-armv7.${ConfigName}.lib \
	-l:usbd_cdn_nortos.am243x.r5f.gcc-armv7.${ConfigName}.lib \
	-l:usbd_tusb_dfu_nortos.am243x.r5f.gcc-armv7.${ConfigName}.lib \

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
	drivers.am243x.r5f.gcc-armv7.${ConfigName}.lib \
	board.am243x.r5f.gcc-armv7.${ConfigName}.lib \
	usbd_cdn_nortos.am243x.r5f.gcc-armv7.${ConfigName}.lib \
	usbd_tusb_dfu_nortos.am243x.r5f.gcc-armv7.${ConfigName}.lib \

LIBS_PATH_NAME = \
	${MCU_PLUS_SDK_PATH}/source/kernel/nortos/lib \
	${MCU_PLUS_SDK_PATH}/source/drivers/lib \
	${MCU_PLUS_SDK_PATH}/source/board/lib \
	${MCU_PLUS_SDK_PATH}/source/usb/cdn/lib \
	${MCU_PLUS_SDK_PATH}/source/usb/tinyusb/lib \
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
	@echo  Compiling: am243x:r5fss0-0:nortos:gcc-armv7 $(OUTNAME): $<
	$(CC) -c $(CFLAGS) $(INCLUDES) $(DEFINES) -MMD -MT $@ -o $(OBJDIR)/$@ $<

$(OBJDIR)/%.obj %.obj: %.S
	@echo  Compiling: am243x:r5fss0-0:nortos:gcc-armv7 $(LIBNAME): $<
	$(CC) -c -x assembler-with-cpp $(CFLAGS) $(INCLUDES) $(DEFINES) -o $(OBJDIR)$@ $<

all: $(BOOTIMAGE_NAME)

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
	@echo  Linking: am243x:r5fss0-0:nortos:gcc-armv7 $@ ...
	$(LNK) $(LIBS_PATH) -Wl,-Map=$(OUTNAME:%.out=%.map) -o $@ -Wl,--start-group $(LFLAGS) $(addprefix $(OBJDIR), $(OBJS)) $(LIBS) -Wl,--end-group -Wl,-T,$(LNK_FILES)
	@echo  Linking: am243x:r5fss0-0:nortos:gcc-armv7 $@ Done !!!
	@echo  .

clean:
	@echo  Cleaning: am243x:r5fss0-0:nortos:gcc-armv7 $(OUTNAME) ...
	$(RMDIR) $(OBJDIR)
	$(RM) $(OUTNAME)
	$(RM) $(BOOTIMAGE_NAME)
	$(RM) $(BOOTIMAGE_BIN_NAME)
	$(RMDIR) generated/

scrub:
	@echo  Scrubing: am243x:r5fss0-0:nortos:gcc-armv7 sbl_dfu ...
	$(RMDIR) obj
ifeq ($(OS),Windows_NT)
	$(RM) \*.out
	$(RM) \*.map
	$(RM) \*.appimage*
	$(RM) \*.rprc*
	$(RM) \*.tiimage*
	$(RM) \*.bin
else
	$(RM) *.out
	$(RM) *.map
	$(RM) *.appimage*
	$(RM) *.rprc*
	$(RM) *.tiimage*
	$(RM) *.bin
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
	$(SYSCFG_NODE) $(SYSCFG_CLI_PATH)/dist/cli.js --product $(SYSCFG_SDKPRODUCT) --context r5fss0-0 --part ALX --package ALX --output generated/ --compiler gcc ../example.syscfg
endif

syscfg-gui:
	$(SYSCFG_NWJS) $(SYSCFG_PATH) --product $(SYSCFG_SDKPRODUCT) --device AM243x_ALX_beta --context r5fss0-0 --part ALX --package ALX --output generated/ --compiler gcc ../example.syscfg

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

BOOTIMAGE_CERT_KEY=$(APP_SIGNING_KEY)

ifeq ($(DEVICE_TYPE),HS)
BOOTIMAGE_CERT_KEY=$(APP_SIGNING_KEY)
else
BOOTIMAGE_CERT_KEY=$(ROM_DEGENERATE_KEY)
endif
BOOTIMAGE_CERT_GEN_CMD=$(PYTHON) $(MCU_PLUS_SDK_PATH)/source/security/security_common/tools/boot/signing/rom_image_gen.py
SYSFW_PATH=$(MCU_PLUS_SDK_PATH)/source/drivers/sciclient/soc/am64x_am243x
SYSFW_LOAD_ADDR=0x44000
BOARDCFG_LOAD_ADDR=0x7B000
BOARDCFG_BLOB=$(MCU_PLUS_SDK_PATH)/source/drivers/sciclient/sciclient_default_boardcfg/am243x/boardcfg_blob.bin


SBL_RUN_ADDRESS=0x70000000

SBL_PREBUILT_PATH=$(MCU_PLUS_SDK_PATH)/tools/boot/sbl_prebuilt/am243x-lp

$(BOOTIMAGE_BIN_NAME): $(OUTNAME)
	$(OBJCOPY) --strip-all -O binary $(OUTNAME) $(BOOTIMAGE_BIN_NAME)

$(BOOTIMAGE_NAME): $(BOOTIMAGE_BIN_NAME)
	@echo  Boot image: am243x:r5fss0-0:nortos:gcc-armv7 $(BOOTIMAGE_PATH)/$@ ...
ifeq ($(DEVICE_TYPE),HS)
ifeq ($(ENC_SBL_ENABLED),yes)
	$(BOOTIMAGE_CERT_GEN_CMD) --swrv 1 --sbl-enc --enc-key $(APP_ENCRYPTION_KEY) --sbl-bin $(BOOTIMAGE_PATH)/$(BOOTIMAGE_BIN_NAME) --sysfw-bin $(SYSFW_PATH)/sysfw-hs-enc.bin --sysfw-inner-cert $(SYSFW_PATH)/sysfw-hs-enc-cert.bin --boardcfg-blob $(BOARDCFG_BLOB) --sbl-loadaddr $(SBL_RUN_ADDRESS) --sysfw-loadaddr $(SYSFW_LOAD_ADDR) --bcfg-loadaddr $(BOARDCFG_LOAD_ADDR) --key $(BOOTIMAGE_CERT_KEY) --debug DBG_FULL_ENABLE --rom-image $(BOOTIMAGE_PATH)/$(BOOTIMAGE_NAME)
else
	$(BOOTIMAGE_CERT_GEN_CMD) --swrv 1 --sbl-bin $(BOOTIMAGE_PATH)/$(BOOTIMAGE_BIN_NAME) --sysfw-bin $(SYSFW_PATH)/sysfw-hs-enc.bin --sysfw-inner-cert $(SYSFW_PATH)/sysfw-hs-enc-cert.bin --boardcfg-blob $(BOARDCFG_BLOB) --sbl-loadaddr $(SBL_RUN_ADDRESS) --sysfw-loadaddr $(SYSFW_LOAD_ADDR) --bcfg-loadaddr $(BOARDCFG_LOAD_ADDR) --key $(BOOTIMAGE_CERT_KEY) --debug DBG_FULL_ENABLE --rom-image $(BOOTIMAGE_PATH)/$(BOOTIMAGE_NAME)
endif
else
	$(BOOTIMAGE_CERT_GEN_CMD) --swrv 1 --sbl-bin $(BOOTIMAGE_PATH)/$(BOOTIMAGE_BIN_NAME) --sysfw-bin $(SYSFW_PATH)/sysfw-hs-fs-enc.bin --sysfw-inner-cert $(SYSFW_PATH)/sysfw-hs-fs-enc-cert.bin --boardcfg-blob $(BOARDCFG_BLOB) --sbl-loadaddr $(SBL_RUN_ADDRESS) --sysfw-loadaddr $(SYSFW_LOAD_ADDR) --bcfg-loadaddr $(BOARDCFG_LOAD_ADDR) --key $(BOOTIMAGE_CERT_KEY) --rom-image $(BOOTIMAGE_PATH)/$(BOOTIMAGE_NAME)
endif
	$(COPY) $(BOOTIMAGE_PATH)/$(BOOTIMAGE_NAME) tiboot3.bin
	$(COPY) $(BOOTIMAGE_PATH)/$(BOOTIMAGE_NAME) $(SBL_PREBUILT_PATH)/

	$(RM) $(BOOTIMAGE_TEMP_OUT_FILE)
	@echo  Boot image: am243x:r5fss0-0:nortos:gcc-armv7 $(BOOTIMAGE_PATH)/$@ Done !!!

	@echo  .




-include $(addprefix $(OBJDIR)/, $(DEPS))
