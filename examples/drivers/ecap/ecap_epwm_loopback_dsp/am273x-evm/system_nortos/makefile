#
# Auto generated makefile
#

export MCU_PLUS_SDK_PATH?=$(abspath ../../../../../..)
include $(MCU_PLUS_SDK_PATH)/imports.mak


PROFILE?=release

MULTI_CORE_BOOTIMAGE_PATH=$(abspath .)
MULTI_CORE_BOOTIMAGE_NAME:=ecap_epwm_loopback_system.$(PROFILE).appimage
MULTI_CORE_BOOTIMAGE_NAME_SIGNED:=$(MULTI_CORE_BOOTIMAGE_NAME).signed
MULTI_CORE_BOOTIMAGE_NAME_XIP:=ecap_epwm_loopback_system.$(PROFILE).appimage_xip

CORE_0=--script ../r5fss0-0_nortos/example.syscfg --context r5fss0-0 --output ../r5fss0-0_nortos/ti-arm-clang/generated
CORE_1=--script ../c66ss0_nortos/example.syscfg --context c66ss0 --output ../c66ss0_nortos/ti-c6000/generated

CORES = \
    $(CORE_1) \
    $(CORE_0) \

all: syscfg
	$(MAKE) -C ../r5fss0-0_nortos/ti-arm-clang/ all
	$(MAKE) -C ../c66ss0_nortos/ti-c6000/ all
	$(MAKE) $(MULTI_CORE_BOOTIMAGE_NAME)

clean:
	$(MAKE) -C ../r5fss0-0_nortos/ti-arm-clang/ clean
	$(MAKE) -C ../c66ss0_nortos/ti-c6000/ clean
	$(RM) $(MULTI_CORE_BOOTIMAGE_NAME)
	$(RM) $(MULTI_CORE_BOOTIMAGE_NAME_SIGNED)
	$(RM) $(MULTI_CORE_BOOTIMAGE_NAME_XIP)

scrub:
	$(MAKE) -C ../r5fss0-0_nortos/ti-arm-clang/ scrub
	$(MAKE) -C ../c66ss0_nortos/ti-c6000/ scrub
ifeq ($(OS),Windows_NT)
	$(RM) \*.appimage
	$(RM) \*.appimage.signed
	$(RM) \*.appimage_xip
else
	$(RM) *.appimage
	$(RM) *.appimage.signed
	$(RM) *.appimage_xip
endif

syscfg:
	@echo Generating SysConfig files ...
	$(SYSCFG_NODE) $(SYSCFG_CLI_PATH)/dist/cli.js --product $(SYSCFG_SDKPRODUCT) $(CORES)

syscfg-gui:
	$(SYSCFG_NWJS) $(SYSCFG_PATH) --product $(SYSCFG_SDKPRODUCT) --device AM273x --part Default --package ZCE $(CORES)

#
# Generation of multi-core boot image which can be loaded by Secondary Boot Loader (SBL)
#
ifeq ($(OS),Windows_NT)
EXE_EXT=.exe
endif
BOOTIMAGE_TEMP_OUT_FILE=temp_stdout_$(PROFILE).txt

BOOTIMAGE_CORE_ID_r5fss0-0 = 0
BOOTIMAGE_CORE_ID_r5fss0-1 = 1
BOOTIMAGE_CORE_ID_c66ss0   = 2
SBL_RUN_ADDRESS=0x10200000
SBL_DEV_ID=55

MULTI_CORE_IMAGE_GEN = $(SYSCFG_NODE) $(MCU_PLUS_SDK_PATH)/tools/boot/multicoreImageGen/multicoreImageGen.js
OUTRPRC_CMD = $(SYSCFG_NODE) $(MCU_PLUS_SDK_PATH)/tools/boot/out2rprc/elf2rprc.js

MULTI_CORE_APP_PARAMS = \
    ../r5fss0-0_nortos/ti-arm-clang/ecap_epwm_loopback.$(PROFILE).rprc@$(BOOTIMAGE_CORE_ID_r5fss0-0) \
    ../c66ss0_nortos/ti-c6000/ecap_epwm_loopback.$(PROFILE).rprc@$(BOOTIMAGE_CORE_ID_c66ss0) \

MULTI_CORE_APP_PARAMS_XIP = \
    ../r5fss0-0_nortos/ti-arm-clang/ecap_epwm_loopback.$(PROFILE).rprc_xip@$(BOOTIMAGE_CORE_ID_r5fss0-0) \
    ../c66ss0_nortos/ti-c6000/ecap_epwm_loopback.$(PROFILE).rprc_xip@$(BOOTIMAGE_CORE_ID_c66ss0) \

MULTI_CORE_BOOTIMAGE_DEPENDENCY = \
    ../r5fss0-0_nortos/ti-arm-clang/ecap_epwm_loopback.$(PROFILE).rprc \
    ../c66ss0_nortos/ti-c6000/ecap_epwm_loopback.$(PROFILE).rprc \

$(MULTI_CORE_BOOTIMAGE_DEPENDENCY):

$(MULTI_CORE_BOOTIMAGE_NAME): $(MULTI_CORE_BOOTIMAGE_DEPENDENCY)
	@echo  Boot multi-core image: $@ ...
	$(MULTI_CORE_IMAGE_GEN) --devID $(SBL_DEV_ID) --out $(MULTI_CORE_BOOTIMAGE_NAME) $(MULTI_CORE_APP_PARAMS) >> $(BOOTIMAGE_TEMP_OUT_FILE)
	$(MULTI_CORE_IMAGE_GEN) --devID $(SBL_DEV_ID) --out $(MULTI_CORE_BOOTIMAGE_NAME_XIP) $(MULTI_CORE_APP_PARAMS_XIP) >> $(BOOTIMAGE_TEMP_OUT_FILE)
	$(RM) $(BOOTIMAGE_TEMP_OUT_FILE)
	@echo  Boot multi-core image: $(MULTI_CORE_BOOTIMAGE_PATH)/$@ Done !!!
	@echo  .
