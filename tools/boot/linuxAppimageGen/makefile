
export MCU_PLUS_SDK_PATH?=$(abspath ../../..)
include $(MCU_PLUS_SDK_PATH)/imports.mak
include config.mak

LINUX_BOOTIMAGE_NAME_HS=$(LINUX_BOOTIMAGE_NAME).hs
LINUX_BOOTIMAGE_NAME_HS_FS=$(LINUX_BOOTIMAGE_NAME).hs_fs

LD=$(CGT_GCC_AARCH64_PATH)/bin/aarch64-none-elf-ld

LINUX_IMAGE_LIST = \
	$(ATF_BIN_NAME) \
	$(OPTEE_BIN_NAME) \
	$(SPL_BIN_NAME)

ATF_RPRC = $(addsuffix .rprc, $(basename $(ATF_BIN_NAME)))
OPTEE_RPRC := $(addsuffix .rprc, $(basename $(OPTEE_BIN_NAME)))
SPL_RPRC := $(addsuffix .rprc, $(basename $(SPL_BIN_NAME)))

ATF_ELF = $(addsuffix .elf, $(basename $(ATF_BIN_NAME)))
OPTEE_ELF := $(addsuffix .elf, $(basename $(OPTEE_BIN_NAME)))
SPL_ELF := $(addsuffix .elf, $(basename $(SPL_BIN_NAME)))

RPRC_LIST = \
	$(ATF_RPRC) \
	$(OPTEE_RPRC) \
	$(SPL_RPRC)

ELF_FILE_LIST = $(RPRC_LIST:.rprc=.elf)

COMBINED_APPIMAGE_PARAMS = \
	$(BOOTIMAGE_CORE_ID_a53ss0-0) $(ATF_RPRC) \
	$(BOOTIMAGE_CORE_ID_load_only) $(OPTEE_RPRC) \
	$(BOOTIMAGE_CORE_ID_load_only) $(SPL_RPRC)

BOOTIMAGE_TEMP_OUT_FILE=temp_stdout.txt

BOOTIMAGE_CORE_ID_a53ss0-0 = 0
BOOTIMAGE_CORE_ID_r5fss0-0 = 4
BOOTIMAGE_CORE_ID_r5fss0-1 = 5
BOOTIMAGE_CORE_ID_r5fss1-0 = 6
BOOTIMAGE_CORE_ID_r5fss1-1 = 7
BOOTIMAGE_CORE_ID_m4fss0-0 = 14
BOOTIMAGE_CORE_ID_load_only = 21
SBL_DEV_ID=55

ifeq ($(OS),Windows_NT)
	EXE_EXT=.exe
endif

MULTI_CORE_IMAGE_GEN=$(MCU_PLUS_SDK_PATH)/tools/boot/multicoreImageGen/MulticoreImageGen$(EXE_EXT)
OUTRPRC_CMD=$(MCU_PLUS_SDK_PATH)/tools/boot/out2rprc/out2rprc.exe
APP_IMAGE_SIGN_CMD = $(MCU_PLUS_SDK_PATH)/source/security/security_common/tools/boot/signing/appimage_x509_cert_gen.py

ifeq ($(OS), Windows_NT)
	PYTHON=python
else
	PYTHON=python3
endif

#################################################################################
# Targets
#################################################################################
ifeq ($(DEVICE_TYPE),HS)
all: $(LINUX_BOOTIMAGE_NAME_HS)
else
all: $(LINUX_BOOTIMAGE_NAME) $(LINUX_BOOTIMAGE_NAME_HS_FS)
endif

.INTERMEDIATE: $(RPRC_LIST) $(ELF_FILE_LIST)
ifeq ($(DEVICE_TYPE),HS)
$(LINUX_BOOTIMAGE_NAME).hs: $(RPRC_LIST)
else
$(LINUX_BOOTIMAGE_NAME): $(RPRC_LIST)
$(LINUX_BOOTIMAGE_NAME_HS_FS): $(RPRC_LIST)
endif
	@echo .
	@echo Creating
	$(MULTI_CORE_IMAGE_GEN) LE $(SBL_DEV_ID) $(LINUX_BOOTIMAGE_NAME) $(COMBINED_APPIMAGE_PARAMS) >> $(BOOTIMAGE_TEMP_OUT_FILE)
	$(PYTHON) $(APP_IMAGE_SIGN_CMD) --bin $(LINUX_BOOTIMAGE_NAME) --authtype 1 --key $(APP_SIGNING_KEY) --output $(LINUX_BOOTIMAGE_NAME_HS_FS)
ifeq ($(DEVICE_TYPE),HS)
# Sign the appimage using appimage signing script
ifeq ($(ENC_ENABLED),no)
	@echo Boot image signing: Encryption is disabled.
	$(PYTHON) $(APP_IMAGE_SIGN_CMD) --bin $(LINUX_BOOTIMAGE_NAME) --authtype 1 --key $(APP_SIGNING_KEY) --output $(LINUX_BOOTIMAGE_NAME).hs
else
	@echo Boot image signing: Encryption is enabled.
	$(PYTHON) $(APP_IMAGE_SIGN_CMD) --bin $(LINUX_BOOTIMAGE_NAME) --authtype 1 --key $(APP_SIGNING_KEY) --enc y --enckey $(APP_ENCRYPTION_KEY) --output $(LINUX_BOOTIMAGE_NAME).hs
	$(RM) $(LINUX_BOOTIMAGE_NAME)-enc
endif
	$(RM) $(LINUX_BOOTIMAGE_NAME)
	$(RM) $(LINUX_BOOTIMAGE_NAME_HS_FS)
endif
	$(RM) $(BOOTIMAGE_TEMP_OUT_FILE)
	@echo Combined boot image: $@ Done !!!

$(ATF_RPRC): $(ATF_ELF)
	@echo Creating ATF RPRC image
	$(OUTRPRC_CMD) $(ATF_ELF) $(ATF_RPRC)

$(OPTEE_RPRC): $(OPTEE_ELF)
	@echo Creating OPTEE RPRC image
	$(OUTRPRC_CMD) $(OPTEE_ELF) $(OPTEE_RPRC)

$(SPL_RPRC): $(SPL_ELF)
	@echo Creating SPL RPRC image
	$(OUTRPRC_CMD) $(SPL_ELF) $(SPL_RPRC)

$(ATF_ELF):
	$(LD) -b binary -A aarch64 --oformat elf64-littleaarch64 --section-start=.data=$(ATF_LOAD_ADDR) -e $(ATF_LOAD_ADDR) $(PSDK_LINUX_PREBUILT_IMAGES)/$(ATF_BIN_NAME) -o $(ATF_ELF)

$(OPTEE_ELF):
	$(LD) -b binary -A aarch64 --oformat elf64-littleaarch64 --section-start=.data=$(OPTEE_LOAD_ADDR) -e $(OPTEE_LOAD_ADDR) $(PSDK_LINUX_PREBUILT_IMAGES)/$(OPTEE_BIN_NAME) -o $(OPTEE_ELF)

$(SPL_ELF):
	$(LD) -b binary -A aarch64 --oformat elf64-littleaarch64 --section-start=.data=$(SPL_LOAD_ADDR) -e $(SPL_LOAD_ADDR) $(PSDK_LINUX_PREBUILT_IMAGES)/$(SPL_BIN_NAME) -o $(SPL_ELF)

clean:
	@echo Cleaning: $(LINUX_BOOTIMAGE_NAME)
	$(RM) $(LINUX_BOOTIMAGE_NAME)*
	$(RM) *.elf
	$(RM) *.rprc
