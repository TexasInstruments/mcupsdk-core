# Device type (HS/GP)
DEVICE_TYPE?=GP

# Algorithm (RSA/ECDSA)
ALGORITHM?=RSA4k

# Path to the signing tools, keys etc
SIGNING_TOOL_PATH?=$(MCU_PLUS_SDK_PATH)/source/security/security_common/tools/boot/signing

# Path to the salt required for calculation of Derived key using manufacturers encryption key.
KD_SALT=$(SIGNING_TOOL_PATH)/kd_salt.txt

# Path to the keys
ROM_DEGENERATE_KEY:=$(SIGNING_TOOL_PATH)/rom_degenerateKey.pem
APP_DEGENERATE_KEY:=$(SIGNING_TOOL_PATH)/app_degenerateKey.pem
ifeq ($(DEVICE),am263x)
	CUST_MPK=$(SIGNING_TOOL_PATH)/mcu_custMpk.pem
	CUST_MEK=$(SIGNING_TOOL_PATH)/mcu_custMek.key
else ifeq ($(DEVICE),am273x)
	CUST_MPK=$(SIGNING_TOOL_PATH)/mcu_custMpk.pem
	CUST_MEK=$(SIGNING_TOOL_PATH)/mcu_custMek.key
else ifeq ($(DEVICE),am261x)
# CustMPK support for ECDSA is available for AM261x in ROM.
    ifeq ($(ALGORITHM),RSA4k)
		CUST_MPK=$(SIGNING_TOOL_PATH)/mcu_custMpk.pem
    endif
    ifeq ($(ALGORITHM),ECDSA256R1)
        CUST_MPK=$(SIGNING_TOOL_PATH)/mcu_custMpk_secp256r1.pem
    endif
    ifeq ($(ALGORITHM),ECDSA384R1)
        CUST_MPK=$(SIGNING_TOOL_PATH)/mcu_custMpk_secp384r1.pem
    endif
    ifeq ($(ALGORITHM),ECDSA521R1)
        CUST_MPK=$(SIGNING_TOOL_PATH)/mcu_custMpk_secp521r1.pem
    endif
    ifeq ($(ALGORITHM),BRAINPOOL512)
        CUST_MPK=$(SIGNING_TOOL_PATH)/mcu_custMpk_brainpool512.pem
    endif
	CUST_MEK=$(SIGNING_TOOL_PATH)/mcu_custMek.key
else ifeq ($(DEVICE),awr294x)
	CUST_MPK=$(SIGNING_TOOL_PATH)/mcu_custMpk.pem
	CUST_MEK=$(SIGNING_TOOL_PATH)/mcu_custMek.key
else ifeq ($(DEVICE),am263px)
	CUST_MPK=$(SIGNING_TOOL_PATH)/mcu_custMpk.pem
	CUST_MEK=$(SIGNING_TOOL_PATH)/mcu_custMek.key
else ifeq ($(DEVICE),am65x)
	CUST_MPK=$(SIGNING_TOOL_PATH)/k3_dev_mpk.pem
else
	CUST_MPK=$(SIGNING_TOOL_PATH)/custMpk_am64x_am243x.pem
	CUST_MEK=$(SIGNING_TOOL_PATH)/custMek_am64x_am243x.txt
endif

# Encryption option for application (yes/no)
ENC_ENABLED?=no

# Encryption option for SBL (yes/no)
ENC_SBL_ENABLED?=yes

# Debug Enable (yes/no)
DBG_ENABLED?=no

# Debug control with TIFS (yes/no)
DEBUG_TIFS?=yes

# RSASSA-PSS scheme option for application signing (yes/no)
RSASSAPSS_ENABLED?=no

# Debug options for HS (DBG_PERM_DISABLE / DBG_SOC_DEFAULT / DBG_PUBLIC_ENABLE / DBG_FULL_ENABLE)
# This option is valid only if DEBUG_TIFS is false
DEBUG_OPTION?=DBG_SOC_DEFAULT

# Generic macros to be used depending on the device type
APP_SIGNING_KEY=
APP_ENCRYPTION_KEY=
APP_SIGNING_HASH_ALGO=
APP_SIGNING_SALT_LENGTH=

ifeq ($(DEVICE_TYPE),HS)
	APP_SIGNING_KEY=$(CUST_MPK)
	APP_ENCRYPTION_KEY=$(CUST_MEK)
	APP_SIGNING_HASH_ALGO=sha512
	APP_SIGNING_SALT_LENGTH=0
else
	APP_SIGNING_KEY=$(APP_DEGENERATE_KEY)
endif

# Key id in keyring for application authentication and decryption
APP_SIGNING_KEY_KEYRING_ID?=0
APP_ENCRYPTION_KEY_KEYRING_ID?=1

# Macros for multicore-elf genimage.py script
MCELF_MERGE_SEGMENTS_FLAG?=true
MCELF_MERGE_SEGMENTS_TOLERANCE_LIMIT?=0
MCELF_IGNORE_CONTEXT_FLAG?=false
MCELF_XIP_RANGE?=0x60000000:0x68000000
# Default am263x address translation JSON is in tools/boot/multicore-elf/deviceData/AddrTranslate/am263x.json
MCELF_ADDR_TRANSLATION_PATH?=none

#Maximum size of a loadable elf segment. 
#MCELF_MERGE_SEGMENTS_FLAG should be set to false to achieve this effect.
MCELF_MAX_SEGMENT_SIZE?=8192

ifeq ($(DEVICE),am273x)
    ifeq ($(DEVICE_TYPE),GP)
        # Only set KEY_VERSION for HS-FS(GP), not for HSSE
        KEY_VERSION=1.1
    endif
endif