
export MCU_PLUS_SDK_PATH?=$(abspath ../../../../../..)
include $(MCU_PLUS_SDK_PATH)/imports.mak
include $(MCU_PLUS_SDK_PATH)/devconfig/devconfig.mak

ifeq ($(OS), Windows_NT)
	PYTHON=python
else
	PYTHON=python3
endif

PROFILE?=release

SYSTEM_PROJECT_NAME:=sdl_ecc_pru_scrubber_system

all: postBuild

corebuild:
	$(MAKE) -s -C ../sdl_ecc_pru_scrubber_r5f0/am263px-cc/r5fss0-0_freertos/ti-arm-clang clean
	$(MAKE) -s -C ../sdl_ecc_pru_scrubber_pru0/am263px-cc/icss_m0_pru0_fw/ti-pru-cgt clean
	$(MAKE) -s -C ../sdl_ecc_pru_scrubber_pru0/am263px-cc/icss_m0_pru0_fw/ti-pru-cgt
	$(MAKE) -s -C ../sdl_ecc_pru_scrubber_r5f0/am263px-cc/r5fss0-0_freertos/ti-arm-clang

postBuild: corebuild
