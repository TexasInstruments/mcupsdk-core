# Device type (HS/GP)
DEVICE_TYPE?=GP

# Path to the signing tools, keys etc
SIGNING_TOOL_PATH?=$(MCU_PLUS_SDK_PATH)/tools/boot/signing

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
else ifeq ($(DEVICE),awr294x)
	CUST_MPK=$(SIGNING_TOOL_PATH)/mcu_custMpk.pem
	CUST_MEK=$(SIGNING_TOOL_PATH)/mcu_custMek.key
else ifeq ($(DEVICE),am263px)
	CUST_MPK=$(SIGNING_TOOL_PATH)/mcu_custMpk.pem
	CUST_MEK=$(SIGNING_TOOL_PATH)/mcu_custMek.key
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
APP_ENCRYPTION_KEY_KEYRING_ID?=0
