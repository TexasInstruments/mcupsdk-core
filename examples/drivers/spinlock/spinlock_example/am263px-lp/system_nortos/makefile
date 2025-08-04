
export MCU_PLUS_SDK_PATH?=$(abspath ../../../../../..)
include $(MCU_PLUS_SDK_PATH)/imports.mak
include $(MCU_PLUS_SDK_PATH)/devconfig/devconfig.mak

ifeq ($(OS), Windows_NT)
	PYTHON=python
else
	PYTHON=python3
endif

PROFILE?=release

SYSTEM_PROJECT_NAME:=spinlock_example_system

MULTI_CORE_BOOTIMAGE_PATH=$(abspath .)
MULTI_CORE_BOOTIMAGE_NAME_MCELF:=$(SYSTEM_PROJECT_NAME).$(PROFILE).mcelf
MULTI_CORE_BOOTIMAGE_NAME_MCELF_HS:=$(SYSTEM_PROJECT_NAME).$(PROFILE).mcelf.hs

CORE_0=--script ../r5fss0-0_nortos/example.syscfg --context r5fss0-0 --output ../r5fss0-0_nortos/ti-arm-clang/generated
CORE_1=--script ../r5fss0-1_nortos/example.syscfg --context r5fss0-1 --output ../r5fss0-1_nortos/ti-arm-clang/generated

CORES = \
    $(CORE_1) \
    $(CORE_0) \

CORE_IMG_ARGS :=

CORE_0_IMG=../r5fss0-0_nortos/ti-arm-clang/spinlock_example.$(PROFILE).out
CORE_IMG_ARGS +=--core-img=0:$(CORE_0_IMG)

CORE_1_IMG=../r5fss0-1_nortos/ti-arm-clang/spinlock_example.$(PROFILE).out
CORE_IMG_ARGS +=--core-img=1:$(CORE_1_IMG)

MULTI_CORE_BOOTIMAGE_NAME_MCELF_OPTISHARE:=$(SYSTEM_PROJECT_NAME).$(PROFILE).optishare.mcelf

CORE_IMG_OPTISHARE_ARGS :=

CORE_0_OPTISHARE_IMG=../r5fss0-0_nortos/ti-arm-clang/spinlock_example.$(PROFILE).optishare.out
CORE_IMG_OPTISHARE_ARGS +=--core-img=0:$(CORE_0_OPTISHARE_IMG)

CORE_1_OPTISHARE_IMG=../r5fss0-1_nortos/ti-arm-clang/spinlock_example.$(PROFILE).optishare.out
CORE_IMG_OPTISHARE_ARGS +=--core-img=1:$(CORE_1_OPTISHARE_IMG)

COREXMLS = \
	../r5fss0-0_nortos/ti-arm-clang/spinlock_example.$(PROFILE).lnkxml\
	../r5fss0-1_nortos/ti-arm-clang/spinlock_example.$(PROFILE).lnkxml\

SYSTEM_FLAG := true

all: postBuild

corebuild: syscfg
	$(MAKE) -C ../r5fss0-0_nortos/ti-arm-clang/ all SYSTEM_FLAG=${SYSTEM_FLAG}
	$(MAKE) -C ../r5fss0-1_nortos/ti-arm-clang/ all SYSTEM_FLAG=${SYSTEM_FLAG}


postBuild: corebuild
	$(MAKE) $(MULTI_CORE_BOOTIMAGE_NAME_MCELF)
ifeq ($(DEVICE_TYPE), HS)
	$(MAKE) $(MULTI_CORE_BOOTIMAGE_NAME_MCELF_HS)
endif


CG_TOOL_ROOT=$(CGT_TI_ARM_CLANG_PATH)
CC=$(CG_TOOL_ROOT)/bin/tiarmclang
LNK=$(CG_TOOL_ROOT)/bin/tiarmclang

sso: optisave

coreout: sso.out
	@echo .
	@echo generating files with sso
	$(MAKE) -C ../r5fss0-0_nortos/ti-arm-clang/ coreout SYSTEM_FLAG=${SYSTEM_FLAG} SSO_PATH=../../system_nortos/sso.out
	$(MAKE) -C ../r5fss0-1_nortos/ti-arm-clang/ coreout SYSTEM_FLAG=${SYSTEM_FLAG} SSO_PATH=../../system_nortos/sso.out
	@echo generating files with sso done!
	@echo .

sso.out: sso.cmd
	@echo .
	@echo generating sso.out
	$(CC) -Wl,sso.cmd.tmp -o sso.out -Wl,--xml_link_info=sso.lnkxml -Wl,--gen_xml_func_hash
	@echo done
	@echo .

sso.cmd: all $(COREXMLS)
	@echo .
	@echo generating sso.cmd.tmp
	$(OPTISHARE) -o sso.cmd.tmp $(COREXMLS) -s sso.info.tmp --mem_spec 
	@echo generating sso.cmd done
	@echo .

optisave: buildmcelfsso
	$(OPTISAVE) ../r5fss0-0_nortos/ti-arm-clang/spinlock_example.$(PROFILE).lnkxml ../r5fss0-0_nortos/ti-arm-clang/spinlock_example.$(PROFILE).optishare.lnkxml > ../r5fss0-0_nortos/ti-arm-clang/spinlock_example.$(PROFILE).ossr
	$(OPTISAVE) ../r5fss0-1_nortos/ti-arm-clang/spinlock_example.$(PROFILE).lnkxml ../r5fss0-1_nortos/ti-arm-clang/spinlock_example.$(PROFILE).optishare.lnkxml > ../r5fss0-1_nortos/ti-arm-clang/spinlock_example.$(PROFILE).ossr

buildmcelfsso: coreout
	@echo .
	@echo generating mcelf with sso 
	$(PYTHON) $(MCELF_IMAGE_GEN) $(CORE_IMG_OPTISHARE_ARGS) --output=$(MULTI_CORE_BOOTIMAGE_NAME_MCELF_OPTISHARE) --merge-segments=$(MCELF_MERGE_SEGMENTS_FLAG) --tolerance-limit=$(MCELF_MERGE_SEGMENTS_TOLERANCE_LIMIT) --ignore-context=$(MCELF_IGNORE_CONTEXT_FLAG) --xip=$(MCELF_XIP_RANGE) --xlat=$(MCELF_ADDR_TRANSLATION_PATH) --max-segment-size=$(MCELF_MAX_SEGMENT_SIZE) --sso=sso.out
	@echo  Boot multi-core ELF image: $(MULTI_CORE_BOOTIMAGE_PATH)/$@ Done !!!
	@echo  .


clean:
	$(MAKE) -C ../r5fss0-0_nortos/ti-arm-clang/ clean
	$(MAKE) -C ../r5fss0-1_nortos/ti-arm-clang/ clean
	$(RM) $(MULTI_CORE_BOOTIMAGE_NAME)
	$(RM) $(MULTI_CORE_BOOTIMAGE_NAME_SIGNED)
	$(RM) $(MULTI_CORE_BOOTIMAGE_NAME_XIP)
	$(RM) $(MULTI_CORE_BOOTIMAGE_NAME_MCELF)

scrub:
	$(MAKE) -C ../r5fss0-0_nortos/ti-arm-clang/ scrub
	$(MAKE) -C ../r5fss0-1_nortos/ti-arm-clang/ scrub
ifeq ($(OS),Windows_NT)
	$(RM) \*.appimage
	$(RM) \*.appimage.signed
	$(RM) \*.appimage.hs
	$(RM) \*.tmp
	$(RM) \*.out
	$(RM) \*.map
	$(RM) \*.lnkxml
	$(RM) \*.appimage_xip
	$(RM) \*.mcelf
	$(RM) \*.mcelf_xip
	$(RM) \*.mcelf-enc
	$(RM) \*.mcelf.hs
else
	$(RM) *.tmp
	$(RM) *.out
	$(RM) *.map
	$(RM) *.lnkxml
	$(RM) *.appimage
	$(RM) *.appimage.signed
	$(RM) \*.appimage.hs
	$(RM) *.appimage_xip
	$(RM) *.mcelf
	$(RM) *.mcelf_xip
	$(RM) *.mcelf-enc
	$(RM) *.mcelf.hs
endif

syscfg:
	@echo Generating SysConfig files ...
	$(SYSCFG_NODE) $(SYSCFG_CLI_PATH)/dist/cli.js --product $(SYSCFG_SDKPRODUCT) $(CORES)

syscfg-gui:
	$(SYSCFG_NWJS) $(SYSCFG_PATH) --product $(SYSCFG_SDKPRODUCT) --device AM263Px --part AM263P4 --package ZCZ_C $(CORES)

#
# Generation of multi-core boot image which can be loaded by Secondary Boot Loader (SBL)
#
ifeq ($(OS),Windows_NT)
EXE_EXT=.exe
endif
BOOTIMAGE_TEMP_OUT_FILE=temp_stdout_$(PROFILE).txt

BOOTIMAGE_CORE_ID_r5fss0-0 = 0
BOOTIMAGE_CORE_ID_r5fss0-1 = 1
BOOTIMAGE_CORE_ID_r5fss1-0 = 2
BOOTIMAGE_CORE_ID_r5fss1-1 = 3
SBL_RUN_ADDRESS=0x70000000
SBL_DEV_ID=55

APP_IMAGE_SIGN_CMD = $(MCU_PLUS_SDK_PATH)/source/security/security_common/tools/boot/signing/mcu_appimage_x509_cert_gen.py
MCELF_IMAGE_GEN = $(MCU_PLUS_SDK_PATH)/tools/boot/multicore-elf/genimage.py

oeconfig?=None
$(MULTI_CORE_BOOTIMAGE_NAME_MCELF):
	@echo  Boot multi-core ELF image: $@ ...

	$(PYTHON) $(MCELF_IMAGE_GEN) $(CORE_IMG_ARGS) --output=$(MULTI_CORE_BOOTIMAGE_NAME_MCELF) --merge-segments=$(MCELF_MERGE_SEGMENTS_FLAG) --tolerance-limit=$(MCELF_MERGE_SEGMENTS_TOLERANCE_LIMIT) --ignore-context=$(MCELF_IGNORE_CONTEXT_FLAG) --xip=$(MCELF_XIP_RANGE) --xlat=$(MCELF_ADDR_TRANSLATION_PATH) --max-segment-size=$(MCELF_MAX_SEGMENT_SIZE)  --otfaConfigFile=$(oeconfig)

	@echo  Boot multi-core ELF image: $(MULTI_CORE_BOOTIMAGE_PATH)/$@ Done !!!
	@echo  .

$(MULTI_CORE_BOOTIMAGE_NAME_MCELF_HS): $(MULTI_CORE_BOOTIMAGE_NAME_MCELF)
ifeq ($(DEVICE_TYPE), HS)
# Sign the appimage using appimage signing script
ifeq ($(ENC_ENABLED), no)
ifeq ($(RSASSAPSS_ENABLED), no)
	@echo Boot image signing: Encryption is disabled.
	$(PYTHON) $(APP_IMAGE_SIGN_CMD) --bin $(MULTI_CORE_BOOTIMAGE_NAME_MCELF) --key $(APP_SIGNING_KEY) --sign_key_id $(APP_SIGNING_KEY_KEYRING_ID) --hash_algo $(APP_SIGNING_HASH_ALGO) --output $(MULTI_CORE_BOOTIMAGE_NAME_MCELF_HS)
else
	@echo Boot image signing: Encryption is disabled. RSASSAPSS is enabled.
	$(PYTHON) $(APP_IMAGE_SIGN_CMD) --bin $(MULTI_CORE_BOOTIMAGE_NAME_MCELF) --key $(APP_SIGNING_KEY) --sign_key_id $(APP_SIGNING_KEY_KEYRING_ID) --hash_algo $(APP_SIGNING_HASH_ALGO) --pss_saltlen $(APP_SIGNING_SALT_LENGTH) --output $(MULTI_CORE_BOOTIMAGE_NAME_MCELF_HS) --rsassa_pss
endif
else
ifeq ($(RSASSAPSS_ENABLED), no)
	@echo Boot image signing: Encryption is enabled.
	$(PYTHON) $(APP_IMAGE_SIGN_CMD) --bin $(MULTI_CORE_BOOTIMAGE_NAME_MCELF) --key $(APP_SIGNING_KEY) --enc y --enckey $(APP_ENCRYPTION_KEY) --kd-salt $(KD_SALT) --sign_key_id $(APP_SIGNING_KEY_KEYRING_ID) --enc_key_id $(APP_ENCRYPTION_KEY_KEYRING_ID) --hash_algo $(APP_SIGNING_HASH_ALGO) --output $(MULTI_CORE_BOOTIMAGE_NAME_MCELF_HS)
	$(RM) $(MULTI_CORE_BOOTIMAGE_NAME_MCELF)-enc
else
	@echo Boot image signing: Encryption is enabled. RSASSAPSS is enabled.
	$(PYTHON) $(APP_IMAGE_SIGN_CMD) --bin $(MULTI_CORE_BOOTIMAGE_NAME_MCELF) --key $(APP_SIGNING_KEY) --enc y --enckey $(APP_ENCRYPTION_KEY) --kd-salt $(KD_SALT) --sign_key_id $(APP_SIGNING_KEY_KEYRING_ID) --enc_key_id $(APP_ENCRYPTION_KEY_KEYRING_ID) --hash_algo $(APP_SIGNING_HASH_ALGO) --pss_saltlen $(APP_SIGNING_SALT_LENGTH) --output $(MULTI_CORE_BOOTIMAGE_NAME_MCELF_HS) --rsassa_pss
	$(RM) $(MULTI_CORE_BOOTIMAGE_NAME_MCELF)-enc
endif
endif
	@echo  Boot multi-core image: $(MULTI_CORE_BOOTIMAGE_PATH)/$(MULTI_CORE_BOOTIMAGE_NAME_MCELF_HS) Done !!!
	@echo  .
endif
