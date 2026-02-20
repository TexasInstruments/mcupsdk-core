
export MCU_PLUS_SDK_PATH?=$(abspath ../../../../../..)
include $(MCU_PLUS_SDK_PATH)/imports.mak
include $(MCU_PLUS_SDK_PATH)/devconfig/devconfig.mak

ifeq ($(OS), Windows_NT)
	PYTHON=python
else
	PYTHON=python3
endif

PROFILE?=release

SYSTEM_PROJECT_NAME:=cdc_echo_freertos_system

MULTI_CORE_BOOTIMAGE_PATH=$(abspath .)
MULTI_CORE_BOOTIMAGE_NAME_MCELF:=$(SYSTEM_PROJECT_NAME).$(PROFILE).mcelf
MULTI_CORE_BOOTIMAGE_NAME_MCELF_HS:=$(SYSTEM_PROJECT_NAME).$(PROFILE).mcelf.hs
MULTI_CORE_BOOTIMAGE_NAME_HS_FS:=$(SYSTEM_PROJECT_NAME).$(PROFILE).mcelf.hs_fs

CORE_0=--script ../r5fss0-0_freertos/example.syscfg --context r5fss0-0 --output ../r5fss0-0_freertos/ti-arm-clang/generated
CORE_1=--script ../r5fss0-1_freertos/example.syscfg --context r5fss0-1 --output ../r5fss0-1_freertos/ti-arm-clang/generated

CORES = \
    $(CORE_1) \
    $(CORE_0) \

CORE_IMG_ARGS :=

CORE_4_IMG=../r5fss0-0_freertos/ti-arm-clang/cdc_echo_freertos.$(PROFILE).out
CORE_IMG_ARGS +=--core-img=4:$(CORE_4_IMG)

CORE_5_IMG=../r5fss0-1_freertos/ti-arm-clang/cdc_echo_freertos.$(PROFILE).out
CORE_IMG_ARGS +=--core-img=5:$(CORE_5_IMG)

COREXMLS = \
	../r5fss0-0_freertos/ti-arm-clang/cdc_echo_freertos.$(PROFILE).lnkxml\
	../r5fss0-1_freertos/ti-arm-clang/cdc_echo_freertos.$(PROFILE).lnkxml\

SYSTEM_FLAG := true

all: postBuild

corebuild: syscfg
	$(MAKE) -C ../r5fss0-0_freertos/ti-arm-clang/ all SYSTEM_FLAG=${SYSTEM_FLAG}
	$(MAKE) -C ../r5fss0-1_freertos/ti-arm-clang/ all SYSTEM_FLAG=${SYSTEM_FLAG}


postBuild: corebuild
	$(MAKE) $(MULTI_CORE_BOOTIMAGE_NAME_MCELF)
ifeq ($(DEVICE_TYPE), HS)
	$(MAKE) $(MULTI_CORE_BOOTIMAGE_NAME_MCELF_HS)
else
	$(MAKE) $(MULTI_CORE_BOOTIMAGE_NAME_HS_FS)
endif


clean:
	$(MAKE) -C ../r5fss0-0_freertos/ti-arm-clang/ clean
	$(MAKE) -C ../r5fss0-1_freertos/ti-arm-clang/ clean
	$(RM) $(MULTI_CORE_BOOTIMAGE_NAME)
	$(RM) $(MULTI_CORE_BOOTIMAGE_NAME_SIGNED)
	$(RM) $(MULTI_CORE_BOOTIMAGE_NAME_XIP)
	$(RM) $(MULTI_CORE_BOOTIMAGE_NAME_MCELF)
	$(RM) $(MULTI_CORE_BOOTIMAGE_NAME_MCELF_HS)
	$(RM) $(MULTI_CORE_BOOTIMAGE_NAME_HS_FS)

scrub:
	$(MAKE) -C ../r5fss0-0_freertos/ti-arm-clang/ scrub
	$(MAKE) -C ../r5fss0-1_freertos/ti-arm-clang/ scrub
ifeq ($(OS),Windows_NT)
	$(RM) \*.appimage
	$(RM) \*.appimage.signed
	$(RM) \*.tmp
	$(RM) \*.out
	$(RM) \*.map
	$(RM) \*.lnkxml
	$(RM) \*.appimage_xip
	$(RM) \*.mcelf
	$(RM) \*.mcelf_xip
	$(RM) \*.mcelf-enc
	$(RM) \*.mcelf.hs
	$(RM) \*.mcelf.hs_fs
else
	$(RM) *.tmp
	$(RM) *.out
	$(RM) *.map
	$(RM) *.lnkxml
	$(RM) *.appimage
	$(RM) *.appimage.signed
	$(RM) *.appimage_xip
	$(RM) *.mcelf
	$(RM) *.mcelf_xip
	$(RM) *.mcelf-enc
	$(RM) *.mcelf.hs
	$(RM) *.mcelf.hs_fs
endif

syscfg:
	@echo Generating SysConfig files ...
	$(SYSCFG_NODE) $(SYSCFG_CLI_PATH)/dist/cli.js --product $(SYSCFG_SDKPRODUCT) $(CORES)

syscfg-gui:
	$(SYSCFG_GUI_SCRIPT) --product $(SYSCFG_SDKPRODUCT) --device AM243x_ALV_beta --part ALV --package ALV $(CORES)

#
# Generation of multi-core boot image which can be loaded by Secondary Boot Loader (SBL)
#
ifeq ($(OS),Windows_NT)
EXE_EXT=.exe
endif
BOOTIMAGE_TEMP_OUT_FILE=temp_stdout_$(PROFILE).txt

BOOTIMAGE_CORE_ID_r5fss0-0 = 4
BOOTIMAGE_CORE_ID_r5fss0-1 = 5
BOOTIMAGE_CORE_ID_r5fss1-0 = 6
BOOTIMAGE_CORE_ID_r5fss1-1 = 7
BOOTIMAGE_CORE_ID_m4fss0-0 = 14
SBL_RUN_ADDRESS=0x70000000
SBL_DEV_ID=55

APP_IMAGE_SIGN_CMD = $(MCU_PLUS_SDK_PATH)/source/security/security_common/tools/boot/signing/appimage_x509_cert_gen.py
MCELF_IMAGE_GEN = $(MCU_PLUS_SDK_PATH)/tools/boot/multicore-elf/genimage_am64x.py

$(MULTI_CORE_BOOTIMAGE_NAME_MCELF):
	@echo  Boot multi-core ELF image: $@ ...

	$(PYTHON) $(MCELF_IMAGE_GEN) $(CORE_IMG_ARGS) --output=$(MULTI_CORE_BOOTIMAGE_NAME_MCELF) --xip=$(MCELF_XIP_RANGE)

	@echo  Boot multi-core ELF image: $(MULTI_CORE_BOOTIMAGE_PATH)/$@ Done !!!
	@echo  .

$(MULTI_CORE_BOOTIMAGE_NAME_MCELF_HS): $(MULTI_CORE_BOOTIMAGE_NAME_MCELF)
ifeq ($(DEVICE_TYPE),HS)
# Sign the appimage using appimage signing script
ifeq ($(ENC_ENABLED),no)
	@echo Boot image signing: Encryption is disabled.
	$(PYTHON) $(APP_IMAGE_SIGN_CMD) --bin $(MULTI_CORE_BOOTIMAGE_NAME_MCELF) --authtype 1 --key $(APP_SIGNING_KEY) --output $(MULTI_CORE_BOOTIMAGE_NAME_MCELF_HS)
else
	@echo Boot image signing: Encryption is enabled.
	$(PYTHON) $(APP_IMAGE_SIGN_CMD) --bin $(MULTI_CORE_BOOTIMAGE_NAME_MCELF) --authtype 1 --key $(APP_SIGNING_KEY) --enc y --enckey $(APP_ENCRYPTION_KEY) --output $(MULTI_CORE_BOOTIMAGE_NAME_MCELF_HS)
	$(RM) $(MULTI_CORE_BOOTIMAGE_NAME_MCELF)-enc
endif
	@echo  Boot multi-core image: $(MULTI_CORE_BOOTIMAGE_PATH)/$(MULTI_CORE_BOOTIMAGE_NAME_MCELF_HS) Done !!!
	@echo  .
endif

$(MULTI_CORE_BOOTIMAGE_NAME_HS_FS): $(MULTI_CORE_BOOTIMAGE_NAME_MCELF)
ifeq ($(DEVICE_TYPE),HS)
else
# Sign the appimage for HS-FS using appimage signing script
	$(PYTHON) $(APP_IMAGE_SIGN_CMD) --bin $(MULTI_CORE_BOOTIMAGE_NAME_MCELF) --authtype 1 --key $(APP_SIGNING_KEY) --output $(MULTI_CORE_BOOTIMAGE_NAME_HS_FS)
	$(RM) $(MULTI_CORE_BOOTIMAGE_NAME_MCELF)
	@echo Boot multi-core image: $(MULTI_CORE_BOOTIMAGE_PATH)/$(MULTI_CORE_BOOTIMAGE_NAME_HS_FS) Done !!!
	@echo  .
endif
