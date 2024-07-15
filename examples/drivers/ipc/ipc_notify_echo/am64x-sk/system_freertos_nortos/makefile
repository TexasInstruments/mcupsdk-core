#
# Auto generated makefile
#

export MCU_PLUS_SDK_PATH?=$(abspath ../../../../../..)
include $(MCU_PLUS_SDK_PATH)/imports.mak
include $(MCU_PLUS_SDK_PATH)/devconfig/devconfig.mak

ifeq ($(OS), Windows_NT)
	PYTHON=python
else
	PYTHON=python3
endif

PROFILE?=release

MULTI_CORE_BOOTIMAGE_PATH=$(abspath .)
MULTI_CORE_BOOTIMAGE_NAME:=ipc_notify_echo_system.$(PROFILE).appimage
MULTI_CORE_BOOTIMAGE_NAME_SIGNED:=$(MULTI_CORE_BOOTIMAGE_NAME).signed
MULTI_CORE_BOOTIMAGE_NAME_XIP:=ipc_notify_echo_system.$(PROFILE).appimage_xip
MULTI_CORE_BOOTIMAGE_NAME_HS:=$(MULTI_CORE_BOOTIMAGE_NAME).hs

CORE_0=--script ../r5fss0-0_freertos/example.syscfg --context r5fss0-0 --output ../r5fss0-0_freertos/ti-arm-clang/generated
CORE_1=--script ../r5fss0-1_nortos/example.syscfg --context r5fss0-1 --output ../r5fss0-1_nortos/ti-arm-clang/generated
CORE_2=--script ../r5fss1-0_nortos/example.syscfg --context r5fss1-0 --output ../r5fss1-0_nortos/ti-arm-clang/generated
CORE_3=--script ../r5fss1-1_nortos/example.syscfg --context r5fss1-1 --output ../r5fss1-1_nortos/ti-arm-clang/generated
CORE_4=--script ../m4fss0-0_nortos/example.syscfg --context m4fss0-0 --output ../m4fss0-0_nortos/ti-arm-clang/generated
CORE_5=--script ../a53ss0-0_nortos/example.syscfg --context a53ss0-0 --output ../a53ss0-0_nortos/gcc-aarch64/generated

CORES = \
    $(CORE_5) \
    $(CORE_4) \
    $(CORE_3) \
    $(CORE_2) \
    $(CORE_1) \
    $(CORE_0) \

SYSTEM_FLAG := true

all: postBuild

corebuild: syscfg
	$(MAKE) -C ../r5fss0-0_freertos/ti-arm-clang/ all SYSTEM_FLAG=${SYSTEM_FLAG}
	$(MAKE) -C ../r5fss0-1_nortos/ti-arm-clang/ all SYSTEM_FLAG=${SYSTEM_FLAG}
	$(MAKE) -C ../r5fss1-0_nortos/ti-arm-clang/ all SYSTEM_FLAG=${SYSTEM_FLAG}
	$(MAKE) -C ../r5fss1-1_nortos/ti-arm-clang/ all SYSTEM_FLAG=${SYSTEM_FLAG}
	$(MAKE) -C ../m4fss0-0_nortos/ti-arm-clang/ all SYSTEM_FLAG=${SYSTEM_FLAG}
	$(MAKE) -C ../a53ss0-0_nortos/gcc-aarch64/ all SYSTEM_FLAG=${SYSTEM_FLAG}


postBuild: corebuild
	$(MAKE) $(MULTI_CORE_BOOTIMAGE_NAME)
ifeq ($(DEVICE_TYPE), HS)
	$(MAKE) $(MULTI_CORE_BOOTIMAGE_NAME_HS)
endif

clean:
	$(MAKE) -C ../r5fss0-0_freertos/ti-arm-clang/ clean
	$(MAKE) -C ../r5fss0-1_nortos/ti-arm-clang/ clean
	$(MAKE) -C ../r5fss1-0_nortos/ti-arm-clang/ clean
	$(MAKE) -C ../r5fss1-1_nortos/ti-arm-clang/ clean
	$(MAKE) -C ../m4fss0-0_nortos/ti-arm-clang/ clean
	$(MAKE) -C ../a53ss0-0_nortos/gcc-aarch64/ clean
	$(RM) $(MULTI_CORE_BOOTIMAGE_NAME)
	$(RM) $(MULTI_CORE_BOOTIMAGE_NAME_SIGNED)
	$(RM) $(MULTI_CORE_BOOTIMAGE_NAME_XIP)

scrub:
	$(MAKE) -C ../r5fss0-0_freertos/ti-arm-clang/ scrub
	$(MAKE) -C ../r5fss0-1_nortos/ti-arm-clang/ scrub
	$(MAKE) -C ../r5fss1-0_nortos/ti-arm-clang/ scrub
	$(MAKE) -C ../r5fss1-1_nortos/ti-arm-clang/ scrub
	$(MAKE) -C ../m4fss0-0_nortos/ti-arm-clang/ scrub
	$(MAKE) -C ../a53ss0-0_nortos/gcc-aarch64/ scrub
ifeq ($(OS),Windows_NT)
	$(RM) \*.appimage
	$(RM) \*.appimage.signed
	$(RM) \*.appimage.hs
	$(RM) \*.appimage_xip
else
	$(RM) *.appimage
	$(RM) *.appimage.signed
	$(RM) \*.appimage.hs
	$(RM) *.appimage_xip
endif

syscfg:
	@echo Generating SysConfig files ...
	$(SYSCFG_NODE) $(SYSCFG_CLI_PATH)/dist/cli.js --product $(SYSCFG_SDKPRODUCT) $(CORES)

syscfg-gui:
	$(SYSCFG_NWJS) $(SYSCFG_PATH) --product $(SYSCFG_SDKPRODUCT) --device AM64x --part Default --package ALV $(CORES)

#
# Generation of multi-core boot image which can be loaded by Secondary Boot Loader (SBL)
#
ifeq ($(OS),Windows_NT)
EXE_EXT=.exe
endif
BOOTIMAGE_TEMP_OUT_FILE=temp_stdout_$(PROFILE).txt

BOOTIMAGE_CORE_ID_a53ss0-0 = 0
BOOTIMAGE_CORE_ID_r5fss0-0 = 4
BOOTIMAGE_CORE_ID_r5fss0-1 = 5
BOOTIMAGE_CORE_ID_r5fss1-0 = 6
BOOTIMAGE_CORE_ID_r5fss1-1 = 7
BOOTIMAGE_CORE_ID_m4fss0-0 = 14
SBL_RUN_ADDRESS=0x70000000
SBL_DEV_ID=55

MULTI_CORE_IMAGE_GEN = $(SYSCFG_NODE) $(MCU_PLUS_SDK_PATH)/tools/boot/multicoreImageGen/multicoreImageGen.js
OUTRPRC_CMD = $(SYSCFG_NODE) $(MCU_PLUS_SDK_PATH)/tools/boot/out2rprc/elf2rprc.js
APP_IMAGE_SIGN_CMD = $(MCU_PLUS_SDK_PATH)/source/security/security_common/tools/boot/signing/appimage_x509_cert_gen.py

MULTI_CORE_APP_PARAMS = \
    ../r5fss0-0_freertos/ti-arm-clang/ipc_notify_echo.$(PROFILE).rprc@$(BOOTIMAGE_CORE_ID_r5fss0-0) \
    ../r5fss0-1_nortos/ti-arm-clang/ipc_notify_echo.$(PROFILE).rprc@$(BOOTIMAGE_CORE_ID_r5fss0-1) \
    ../r5fss1-0_nortos/ti-arm-clang/ipc_notify_echo.$(PROFILE).rprc@$(BOOTIMAGE_CORE_ID_r5fss1-0) \
    ../r5fss1-1_nortos/ti-arm-clang/ipc_notify_echo.$(PROFILE).rprc@$(BOOTIMAGE_CORE_ID_r5fss1-1) \
    ../m4fss0-0_nortos/ti-arm-clang/ipc_notify_echo.$(PROFILE).rprc@$(BOOTIMAGE_CORE_ID_m4fss0-0) \
    ../a53ss0-0_nortos/gcc-aarch64/ipc_notify_echo.$(PROFILE).rprc@$(BOOTIMAGE_CORE_ID_a53ss0-0) \

MULTI_CORE_APP_PARAMS_XIP = \
    ../r5fss0-0_freertos/ti-arm-clang/ipc_notify_echo.$(PROFILE).rprc_xip@$(BOOTIMAGE_CORE_ID_r5fss0-0) \
    ../r5fss0-1_nortos/ti-arm-clang/ipc_notify_echo.$(PROFILE).rprc_xip@$(BOOTIMAGE_CORE_ID_r5fss0-1) \
    ../r5fss1-0_nortos/ti-arm-clang/ipc_notify_echo.$(PROFILE).rprc_xip@$(BOOTIMAGE_CORE_ID_r5fss1-0) \
    ../r5fss1-1_nortos/ti-arm-clang/ipc_notify_echo.$(PROFILE).rprc_xip@$(BOOTIMAGE_CORE_ID_r5fss1-1) \
    ../m4fss0-0_nortos/ti-arm-clang/ipc_notify_echo.$(PROFILE).rprc_xip@$(BOOTIMAGE_CORE_ID_m4fss0-0) \
    ../a53ss0-0_nortos/gcc-aarch64/ipc_notify_echo.$(PROFILE).rprc_xip@$(BOOTIMAGE_CORE_ID_a53ss0-0) \

MULTI_CORE_BOOTIMAGE_DEPENDENCY = \
    ../r5fss0-0_freertos/ti-arm-clang/ipc_notify_echo.$(PROFILE).rprc \
    ../r5fss0-1_nortos/ti-arm-clang/ipc_notify_echo.$(PROFILE).rprc \
    ../r5fss1-0_nortos/ti-arm-clang/ipc_notify_echo.$(PROFILE).rprc \
    ../r5fss1-1_nortos/ti-arm-clang/ipc_notify_echo.$(PROFILE).rprc \
    ../m4fss0-0_nortos/ti-arm-clang/ipc_notify_echo.$(PROFILE).rprc \
    ../a53ss0-0_nortos/gcc-aarch64/ipc_notify_echo.$(PROFILE).rprc \

$(MULTI_CORE_BOOTIMAGE_DEPENDENCY):

$(MULTI_CORE_BOOTIMAGE_NAME): $(MULTI_CORE_BOOTIMAGE_DEPENDENCY)
	@echo  Boot multi-core image: $@ ...
	$(MULTI_CORE_IMAGE_GEN) --devID $(SBL_DEV_ID) --out $(MULTI_CORE_BOOTIMAGE_NAME) $(MULTI_CORE_APP_PARAMS) >> $(BOOTIMAGE_TEMP_OUT_FILE)
	$(MULTI_CORE_IMAGE_GEN) --devID $(SBL_DEV_ID) --out $(MULTI_CORE_BOOTIMAGE_NAME_XIP) $(MULTI_CORE_APP_PARAMS_XIP) >> $(BOOTIMAGE_TEMP_OUT_FILE)
	$(RM) $(BOOTIMAGE_TEMP_OUT_FILE)
ifeq ($(DEVICE_TYPE),HS)
# Sign the appimage using appimage signing script
ifeq ($(ENC_ENABLED),no)
	@echo Boot image signing: Encryption is disabled.
	$(PYTHON) $(APP_IMAGE_SIGN_CMD) --bin $(MULTI_CORE_BOOTIMAGE_NAME) --authtype 1 --key $(APP_SIGNING_KEY) --output $(MULTI_CORE_BOOTIMAGE_NAME_HS)
else
	@echo Boot image signing: Encryption is enabled.
	$(PYTHON) $(APP_IMAGE_SIGN_CMD) --bin $(MULTI_CORE_BOOTIMAGE_NAME) --authtype 1 --key $(APP_SIGNING_KEY) --enc y --enckey $(APP_ENCRYPTION_KEY) --output $(MULTI_CORE_BOOTIMAGE_NAME_HS)
	$(RM) $(MULTI_CORE_BOOTIMAGE_NAME)-enc
endif
	$(RM) $(MULTI_CORE_BOOTIMAGE_NAME)
	@echo  Boot multi-core image: $(MULTI_CORE_BOOTIMAGE_PATH)/$@ Done !!!
	@echo  Boot multi-core image: $(MULTI_CORE_BOOTIMAGE_PATH)/$(MULTI_CORE_BOOTIMAGE_NAME_HS) Done !!!
	@echo  .
else
	$(PYTHON) $(APP_IMAGE_SIGN_CMD) --bin $(MULTI_CORE_BOOTIMAGE_NAME) --authtype 1 --key $(APP_SIGNING_KEY) --output $(MULTI_CORE_BOOTIMAGE_NAME).hs_fs
	$(RM) $(MULTI_CORE_BOOTIMAGE_NAME)
	@echo Boot multi-core image: $(MULTI_CORE_BOOTIMAGE_PATH)/$(MULTI_CORE_BOOTIMAGE_NAME).hs_fs Done !!!
	@echo  .
endif
