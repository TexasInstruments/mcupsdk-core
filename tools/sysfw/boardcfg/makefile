#
# This file is the makefile for generating boardcfg hex array.
#
MCU_PLUS_SDK_PATH=$(abspath ../../../)
include $(MCU_PLUS_SDK_PATH)/imports.mak
include $(MCU_PLUS_SDK_PATH)/devconfig/devconfig.mak
THIS_PATH=$(abspath .)

SOC?=am64x
SOC_RES=
SYSFW_BIN_FOLDER?=am64x_am243x
ifeq ($(SOC),$(filter $(SOC), am64x am243x))
  SOC_FOLDER=$(SOC)
  SOC_RES=$(SOC)
  SOC_PY=am64x
endif

ifeq ($(SOC),$(filter $(SOC), am65x_sr2))
  SOC_FOLDER=am65x
  SOC_RES=am65x
  SOC_PY=am65x_sr2
endif

K3_RESPART_REPO=$(THIS_PATH)/respart/$(SOC)/k3-respart-tool-0.5
K3_RESPART_PRODUCT=$(K3_RESPART_REPO)/.metadata/product.json

# Certificate Generation script and other commands
ifeq ($(OS),Windows_NT)
  EXE_EXT=.exe
  SCICLIENT_CERT_GEN=powershell -executionpolicy unrestricted -command $(MCU_PLUS_SDK_PATH)/source/security/security_common/tools/boot/signing/x509CertificateGen.ps1
  MV=$(CYGWIN_PATH)/mv
else
  SCICLIENT_CERT_GEN=$(MCU_PLUS_SDK_PATH)/source/security/security_common/tools/boot/signing/x509CertificateGen.sh
  MV=mv
endif

ifeq ($(OS),Windows_NT)
PYTHON=python
else
PYTHON=python3
endif
CHMOD=chmod

OBJCOPY=$(CCS_PATH)/utils/tiobj2bin/tiobj2bin

# CC
BOARDCFG_GEN_CC?=$(CGT_TI_ARM_CLANG_PATH)/bin/tiarmclang$(EXE_EXT)

# For converting sysfw binary to a .h file containing the hex array
SYSFW_PATH=$(MCU_PLUS_SDK_PATH)/source/drivers/sciclient/soc/$(SYSFW_BIN_FOLDER)
SYSFW_BIN_NAME=sysfw_hs_fs_signed.bin
SYSFW_HEADER_NAME=$(subst .bin,.h,$(SYSFW_BIN_NAME))
SYSFW_ARRAY_NAME=sysfw

OBJDIR=$(MCU_PLUS_SDK_PATH)/tools/sysfw/boardcfg/obj
OBJEXT=o

# Bin to C header file tools
BIN2C_GEN=$(MCU_PLUS_SDK_PATH)/tools/bin2c/bin2c.py

# Boarcfg blob creator for combined boot
BLOB_GEN=$(MCU_PLUS_SDK_PATH)/tools/sysfw/boardcfg/boardcfg_blob_creator.py

# Certificate Keys
SCICLIENT_CERT_KEY=$(APP_SIGNING_KEY)

# For matching cfg files (purposely truncated to match main cfg file)
BRD_CFG := sciclient_defaultBoard
CFG_DIR := $(MCU_PLUS_SDK_PATH)/source/drivers/sciclient/sciclient_default_boardcfg/$(SOC_FOLDER)

# These will be used to distinguish between an HS configuration and
# .. a GP configuration in the rules below.
ifeq ($(DEVICE_TYPE),HS)
UNSIGN_EXT := hs.bin.unsigned
SGN_EXT    := .hs.signed
HEX_EXT    := _hexhs.h
else
UNSIGN_EXT := .bin.unsigned
SGN_EXT    := .signed
HEX_EXT    := _hex.h
endif

VALIDATOR_JSON := $(MCU_PLUS_SDK_PATH)/tools/sysfw/boardcfg/sysfw_boardcfg_rules.json
VALIDATOR_SCRIPT := $(MCU_PLUS_SDK_PATH)/tools/sysfw/boardcfg/sysfw_boardcfg_validator.py

APP_NAME=sciclient_boardcfg

INCLUDES=-I$(CFG_DIR)
INCLUDES+=-I$(MCU_PLUS_SDK_PATH)/source

ifeq ($(SOC),$(filter $(SOC), am64x))
  CFLAGS_BOARDCFG=-DSOC_AM64X
endif
ifeq ($(SOC),$(filter $(SOC), am243x))
  CFLAGS_BOARDCFG=-DSOC_AM243X
endif
ifeq ($(SOC),$(filter $(SOC), am65x_sr2))
  CFLAGS_BOARDCFG=-DSOC_AM65X
endif
CFLAGS_BOARDCFG+= \
    -Wno-gnu-variable-sized-type-not-at-end \
    -Werror \
    -g \
    -mcpu=cortex-r5 \
    -mfloat-abi=hard \
    -mfpu=vfpv3-d16 \
    -Wno-error=ti-macros

# We want to create an include-able header file with a hex representation of
# .. each board configuration (signed according to SoC). Because each Board
# .. Configuration will require one of these header files, we can simply parse
# .. through the appropriate SOC directory and generate one of these hex header
# .. files for each Board Configuration file.
HEXFILES := $(foreach file, \
		$(wildcard $(CFG_DIR)/$(BRD_CFG)*.c), \
		$(subst .c,$(HEX_EXT),$(file)))

# Append the proper hex-file targets to "all" to guarantee they are
# .. generated if missing/a board configuration file has been changed
.PHONY: sciclient_boardcfg $(HEXFILES)

BCFG=$(OBJDIR)/sciclient_defaultBoardcfg_hex$(UNSIGN_EXT)
BCFG_RM=$(OBJDIR)/sciclient_defaultBoardcfg_rm_hex$(UNSIGN_EXT)
BCFG_PM=$(OBJDIR)/sciclient_defaultBoardcfg_pm_hex$(UNSIGN_EXT)
BCFG_SEC=$(OBJDIR)/sciclient_defaultBoardcfg_security_hex$(UNSIGN_EXT)

BCFG_LINUX=$(OBJDIR)/sciclient_defaultBoardcfg_hex$(UNSIGN_EXT)
BCFG_RM_LINUX=$(OBJDIR)/sciclient_defaultBoardcfg_rm_linux_hex$(UNSIGN_EXT)
BCFG_PM_LINUX=$(OBJDIR)/sciclient_defaultBoardcfg_pm_linux_hex$(UNSIGN_EXT)
BCFG_SEC_LINUX=$(OBJDIR)/sciclient_defaultBoardcfg_security_linux_hex$(UNSIGN_EXT)

BCFG_BLOB_FILE=$(CFG_DIR)/boardcfg_blob.bin
BCFG_BLOB_LINUX_FILE=$(CFG_DIR)/boardcfg_blob_linux.bin

DEVGRP_ALL=0
DEVGRP_MAIN=1
DEVGRP_MCU=2

$(APP_NAME): $(HEXFILES)
	$(PYTHON) $(BLOB_GEN) --sw-rev 1 --devgrp $(DEVGRP_MAIN) --bcfg $(BCFG) --bcfg-rm $(BCFG_RM) --bcfg-pm $(BCFG_PM) --bcfg-sec $(BCFG_SEC) --output-file $(BCFG_BLOB_FILE)
ifeq ($(SOC),$(filter $(SOC), am64x_am243x))
	$(PYTHON) $(BLOB_GEN) --sw-rev 1 --devgrp $(DEVGRP_MAIN) --bcfg $(BCFG_LINUX) --bcfg-rm $(BCFG_RM_LINUX) --bcfg-pm $(BCFG_PM_LINUX) --bcfg-sec $(BCFG_SEC_LINUX) --output-file $(BCFG_BLOB_LINUX_FILE)
endif

sysfw_hex_header:
	cat $(SYSFW_PATH)/sysfw-hs-fs-enc-cert.bin $(SYSFW_PATH)/sysfw-hs-fs-enc.bin > $(SYSFW_PATH)/$(SYSFW_BIN_NAME)
	$(PYTHON) $(BIN2C_GEN) $(SYSFW_PATH)/$(SYSFW_BIN_NAME) $(SYSFW_PATH)/$(SYSFW_HEADER_NAME) $(SYSFW_ARRAY_NAME)

configure:
	$(SYSCFG_NWJS) $(SYSCFG_PATH) --product $(K3_RESPART_PRODUCT) --output generated/ $(K3_RESPART_REPO)/out/$(SOC_RES)-evm.syscfg

configure-gen:
	$(SYSCFG_NODE) $(SYSCFG_CLI_PATH)/dist/cli.js --product $(K3_RESPART_PRODUCT) --output generated/ $(K3_RESPART_REPO)/out/$(SOC_RES)-evm.syscfg
	$(COPY) generated/sciclient_defaultBoardcfg_rm_mcusdk.c $(CFG_DIR)/sciclient_defaultBoardcfg_rm.c
	$(RMDIR) generated/

define CREATE_HEX_GEN

$(2).c:

# Build the Board Configuration object files. A modified Configuration file
# .. will trigger this chain for that particular configuration
$(OBJDIR)/$(1).$(OBJEXT) : $(2).c
	@echo "Build Board Configuration Obj Files"
	$(MKDIR) $(OBJDIR)
	$(BOARDCFG_GEN_CC) $(CFLAGS_BOARDCFG) $(INCLUDES) -c -o $(OBJDIR)/$(notdir $(2)).$(OBJEXT) $(2).c

# Copy the raw binary into an intermediate file (will get deleted by make)
$(OBJDIR)/$(1).bin.unsigned : SHELL:=/bin/bash   # HERE: this is setting the shell for this step only
$(OBJDIR)/$(1).bin.unsigned : $(OBJDIR)/$(1).$(OBJEXT)
	@echo "Create unsigned Board Configuration binary"
	$(OBJCOPY) $(OBJDIR)/$(notdir $(2)).$(OBJEXT) $(OBJDIR)/$(1).bin.unsigned $(CGT_TI_ARM_CLANG_PATH)/bin/tiarmofd $(CGT_TI_ARM_CLANG_PATH)/bin/tiarmhex $(CCS_PATH)/utils/tiobj2bin/mkhex4bin
	$(PYTHON) $(VALIDATOR_SCRIPT) -b $(OBJDIR)/$(1).bin.unsigned -s $(SOC_PY) -l $(OBJDIR)/output_$(BRD_CFG).log -i -o $(OBJDIR)/$(1)_temp.bin.unsigned -r $(VALIDATOR_JSON)
	$(MV) $(OBJDIR)/$(1)_temp.bin.unsigned $(OBJDIR)/$(1).bin.unsigned

# If an HS signature is required, sign it with the HS key (will get deleted by make)
$(OBJDIR)/$(1).bin.hs.signed : $(OBJDIR)/$(1).bin.unsigned
	@echo "Sign Board Configuration with HS key"
ifneq ($(OS),Windows_NT)
	$(CHMOD) a+x $(SCICLIENT_CERT_GEN)
endif
	$(SCICLIENT_CERT_GEN) -b $(OBJDIR)/$(1).bin.unsigned -o $(OBJDIR)/$(1).bin.hs.signed -k $(SCICLIENT_CERT_KEY) -f 1

# If a GP signature is required, sign it with the GP key (will get deleted by make)
$(OBJDIR)/$(1).bin.signed : $(OBJDIR)/$(1).bin.unsigned
	@echo "Sign Board Configuration with GP key"
	@echo "  (note: GP board configuration not ACTUALLY signed with anything)"
	$(COPY) $(OBJDIR)/$(1).bin.unsigned $(OBJDIR)/$(1).bin.signed

# Generate the actual header file from the signed Board Configuration binary.
# This rule determines the type of signature required in the previous rules.
$(CFG_DIR)/$(1).h : $(OBJDIR)/$(1).bin$(SGN_EXT)
	@echo "Generate include-able header file from signed Board Configuration"
	$(PYTHON) $(BIN2C_GEN) $(OBJDIR)/$(1).bin$(SGN_EXT) $(CFG_DIR)/$(1).h $(subst default,,$(notdir $(2)))
#	$(RMDIR) $(OBJDIR)/

endef

$(foreach HEXFILE,$(HEXFILES),$(eval $(call CREATE_HEX_GEN,$(notdir $(basename $(HEXFILE))),$(subst $(HEX_EXT),,$(HEXFILE)))))

# Nothing beyond this point
