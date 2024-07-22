# Enabling Secure Boot {#SECURE_BOOT}

[TOC]

## Secure Devices and Secure Boot: An Introduction

To put it simply, **secure boot** refers to booting application images in a secure way. Secure boot is a feature/service available in **secure devices**.
\cond SOC_AM243X || SOC_AM64X
Out of the device types **GP** and **HS**, HS device type can do secure boot. In secure devices there are two subtypes:
\endcond

\cond SOC_AM263X || SOC_AM263PX || SOC_AM273X || SOC_AWR294X || SOC_AM261X
 In secure device types (HS) there are two subtypes:
\endcond

1. Field Securable (FS)
2. Security Enforced (SE)

HS device type have FS and SE subtypes. Out of the two subtypes, FS and SE, this guide talks about secure boot in an **HS-SE** device. If the secure device received is an **HS-FS** subtype, it needs to be converted to an HS-SE variant with the customer keys burnt into the device eFUSEs. This is done using a special piece of software called an OTP keywriter. Keywriter documentation is out of scope for this guide.

## Secure Boot Process

Secure boot process, like the normal boot, consists of two stages - ROM loading and SBL loading. ROM loading is when the boot ROM loads the HSM runtime binary onto the HSM core, and the signed SBL binary into the primary boot core, which in most cases is an ARM Cortex R5F. SBL loading is when the SBL reads the signed application image from a boot media, authenticates it, decrypts it, and boots it. Here we describe how the secure process takes place in an HS-SE device.

\cond SOC_AM263X || SOC_AM263PX || SOC_AM273X || SOC_AWR294X || SOC_AM261X
\imageStyle{boot_flow_am263x.png,width:60%}
\image html boot_flow_am263x.png "HIGH LEVEL BOOTFLOW"
\endcond

\cond SOC_AM243X || SOC_AM64X
\note In AM243x/AM64x devices, the HSM runtime binary mentioned is the System Firmware (SYSFW) and HSM core is the DMSC Cortex M3 core. Hereafter for generality sake we'll use the terms 'HSMRt' and 'HSM core' but understand that for AM64x/AM243x this means the SYSFW and Cortex M3 core.
\endcond

\cond SOC_AM263X || SOC_AM263PX || SOC_AM273X || SOC_AM261X
## Public Keyring Support
The keyring is a set of keys which can be imported by SBL after importing the HSM runtime binary on an **HS-SE** device. For importing keyring TIFS-MCU expects an X.509 certificate signed by customer active root of trust (MPK). For certificate generation script usage, see \ref KEYRING_CERT_GEN_PYTHON_SCRIPT.

TIFS-MCU supports application authentication using the following algorithms with Keyring.

<table>
    <tr>
        <th>Signature Algorithm</th>
        <th>Supported Keys</th>
        <th>Hash Function</th>
    </tr>
    <tr>
        <td>RSA-PKCS#1 v1.5</td>
        <td>RSA2k, RSA3k, RSA4k</td>
        <td>SHA256, SHA384, SHA512</td>
    </tr>
    <tr>
        <td>RSASSA-PSS</td>
        <td>RSA2k, RSA3k, RSA4k</td>
        <td>SHA256, SHA384, SHA512</td>
    </tr>
    <tr>
        <td>ECDSA</td>
        <td>P-256, P-384, P-521, BrainpoolP512t1</td>
        <td>SHA256, SHA384, SHA512</td>
    </tr>
</table>

For more information, please refer TIFS-MCU documentation

\endcond
## Secure Boot Support in SDK

### Device configuration file

To make the secure/non-secure differences seamless for the user, a configuration file is provided at `${SDK_INSTALL_PATH}/devconfig/devconfig.mak`.
In this configuration file, you can set certain options like the device type, keys to be used for signing and encryption etc. By default they will point to the dummy customer MPKs and MEKs but if you're using a production device with your own keys, please change the paths here to point to the right key files. Configuration of this file is currently manual, this will be made configurable by a GUI in an upcoming release.

The devconfig.mak file looks something like this:

\cond SOC_AM243X||SOC_AM64X
\code
# Device type (HS/GP)
DEVICE_TYPE?=GP

# Path to the signing tools, keys etc
SIGNING_TOOL_PATH=$(MCU_PLUS_SDK_PATH)/tools/boot/signing

# Encryption option (yes/no)
ENC_ENABLED?=no

# Generic macros to be used depending on the device type
APP_SIGNING_KEY=
APP_ENCRYPTION_KEY=

ifeq ($(DEVICE_TYPE),HS)
	APP_SIGNING_KEY=$(CUST_MPK)
	APP_ENCRYPTION_KEY=$(CUST_MEK)
else
	APP_SIGNING_KEY=$(ROM_DEGENERATE_KEY)
endif
\endcode
\endcond

\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
\code
# Device type (HS/GP)
DEVICE_TYPE?=GP

# Path to the signing tools, keys etc
SIGNING_TOOL_PATH=$(MCU_PLUS_SDK_PATH)/tools/boot/signing

# Path to the salt required for calculation of Derived key using manufacturers encryption key.
KD_SALT=$(SIGNING_TOOL_PATH)/kd_salt.txt

# Path to the keys
ROM_DEGENERATE_KEY:=$(SIGNING_TOOL_PATH)/rom_degenerateKey.pem
ifeq ($(DEVICE),am263x)
    CUST_MPK=$(SIGNING_TOOL_PATH)/mcu_custMpk.pem
	CUST_MEK=$(SIGNING_TOOL_PATH)/mcu_custMek.key
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

\endcode
\endcond

\cond SOC_AM273X
\code
# Device type (HS/GP)
DEVICE_TYPE?=GP

# Path to the signing tools, keys etc
SIGNING_TOOL_PATH=$(MCU_PLUS_SDK_PATH)/tools/boot/signing

# Path to the salt required for calculation of Derived key using manufacturers encryption key.
KD_SALT=$(SIGNING_TOOL_PATH)/kd_salt.txt

# Path to the keys
ROM_DEGENERATE_KEY:=$(SIGNING_TOOL_PATH)/rom_degenerateKey.pem
ifeq ($(DEVICE),am273x)
    CUST_MPK=$(SIGNING_TOOL_PATH)/mcu_custMpk.pem
	CUST_MEK=$(SIGNING_TOOL_PATH)/mcu_custMek.key
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

\endcode
\endcond

\cond SOC_AWR294X
\code
# Device type (HS/GP)
DEVICE_TYPE?=GP

# Path to the signing tools, keys etc
SIGNING_TOOL_PATH=$(MCU_PLUS_SDK_PATH)/tools/boot/signing

# Path to the salt required for calculation of Derived key using manufacturers encryption key.
KD_SALT=$(SIGNING_TOOL_PATH)/kd_salt.txt

# Path to the keys
ROM_DEGENERATE_KEY:=$(SIGNING_TOOL_PATH)/rom_degenerateKey.pem
ifeq ($(DEVICE),awr294x)
    CUST_MPK=$(SIGNING_TOOL_PATH)/mcu_custMpk.pem
	CUST_MEK=$(SIGNING_TOOL_PATH)/mcu_custMek.key
endif

# Encryption option for application (yes/no)
ENC_ENABLED?=no

# Encryption option for SBL (yes/no)
ENC_SBL_ENABLED?=yes

# Debug Enable (yes/no)
DBG_ENABLED?=no

# Debug control with TIFS (yes/no)
DEBUG_TIFS?=yes

# Debug options for HS (DBG_PERM_DISABLE / DBG_SOC_DEFAULT / DBG_PUBLIC_ENABLE / DBG_FULL_ENABLE)
# This option is valid only if DEBUG_TIFS is false
DEBUG_OPTION?=DBG_SOC_DEFAULT

# Generic macros to be used depending on the device type
APP_SIGNING_KEY=
APP_ENCRYPTION_KEY=

ifeq ($(DEVICE_TYPE),HS)
	APP_SIGNING_KEY=$(CUST_MPK)
	APP_ENCRYPTION_KEY=$(CUST_MEK)
else
	APP_SIGNING_KEY=$(ROM_DEGENERATE_KEY)
endif

\endcode
\endcond

This file will be included in all example makefiles

### Signing tool

For signing the binaries, two different scripts are used:

1. ROM signing script - This is a python script used for signing the SBL. The x509 certificate template used in this script is expected by the ROM.
2. Application signing script - This is a python script used for signing the application image. The x509 certificate template used in this script is expected by the HSMRt.

 For more details on the usage of the script, see \ref TOOLS_BOOT_SIGNING

### Generating Secure Boot Image

#### SBL Image Generation

\cond SOC_AM64X || SOC_AM243X
In AM64x/AM243x devices, a combined boot method is employed, by virtue of which the SBL, SYSFW and the SYSFW-BoardConfig are combined and signed with the same certificate. This is built into the make system of the SBL applications in the SDK - SBL_UART, SBL_OSPI, SBL_SD, SBL_NULL, etc. So whenever an SBL application is built, the loadable `*.tiimage` will be a concatenation of the x509 certificate, SBL binary, SYSFW binary and the boardcfg binary blob. In case of HS devices, the SYSFW inner certificate will also be concatenated.
\endcond

SBL and HSMRt are signed via Root of trust Keys i.e. SMPK (if keyrev = 1) or BMPK (if keyrev = 2). MCU+ SDK is only tested with TI-Dummy Keys. Users are expected to generate their own set of keys and use the same for production. This is supposed to be used only with the devices with the same dummy customer MPK burnt into the eFUSEes. If the SDK is supposed to be used with a production/development device with actual customer MPKs burnt into the device, please replace the file at ${SDK_INSTALL_PATH}/tools/boot/signing/custMpk_${SOC}.pem. This is true for also the encryption key used, which can also be found at ${SDK_INSTALL_PATH}/tools/boot/signing/custMek_${SOC}.txt. Whenever any SBL is built, it will be signed with dummy customer MPK, and the signed image generated will have an extension of `*.hs.tiimage`.

For more information, please refer to the OTP Keywriter Documentation.

\cond SOC_AM263X || SOC_AM263PX||SOC_AM273X||SOC_aWR294X || SOC_AM261X
\note HSMRt is built with TIFS-MCU package which is an add-on package on MCU+ SDK.
\endcond

\cond SOC_AM64X || SOC_AM243X
There is no extra step required other than making sure that the MPK used is indeed the one burnt into the eFUSEs.
\endcond

\cond SOC_AM263X || SOC_AM263PX || SOC_AM273X || SOC_AWR294X || SOC_AM261X
It is recommended to pass a key derivation salt for requesting ROM to not use the
Customer MEK directly but generate a derived key to decrypt the SBL/HSMRt images.
ROM uses HKDF (HMAC based Key Derivation Function) to derive a key from the S(B)MEK.
The Key Derivation Function requires a salt which is a non-secret random value
which it uses in a 2 step HKDF-Extract and HKDF-Expand process.
For more information, please refer to [RFC-5869](https://datatracker.ietf.org/doc/html/rfc5869).
With the signing scripts provided as part of MCU+ SDK, this can be done by
passing argument `kdsalt` to the signing script and has been
integrated as the default option in the makefiles for SBL.

\note To maintain the longevity of Customer MEK, it is recommended to update the salt
everytime there is a firmware upgrade.

The salt is supplied to the makefiles via the file `/source/security/security_common/tools/boot/signing/kd_salt.txt`.

Here is an example certificate for SBL that takes care of the above recommendation:

\code
[ req ]
distinguished_name     = req_distinguished_name
x509_extensions        = v3_ca
prompt                 = no

dirstring_type = nobmp

[ req_distinguished_name ]
C                      = US
ST                     = SC
L                      = New York
O                      = Texas Instruments., Inc.
OU                     = SITARA MCU
CN                     = Albert
emailAddress           = Albert@gt.ti.com

[ v3_ca ]
basicConstraints = CA:true
1.3.6.1.4.1.294.1.1=ASN1:SEQUENCE:boot_seq
1.3.6.1.4.1.294.1.2=ASN1:SEQUENCE:image_integrity
1.3.6.1.4.1.294.1.3=ASN1:SEQUENCE:swrv
1.3.6.1.4.1.294.1.4=ASN1:SEQUENCE:encryption

1.3.6.1.4.1.294.1.5=ASN1:SEQUENCE:key_derivation

[ boot_seq ]
certType     =  INTEGER:1
bootCore     =  INTEGER:16
bootCoreOpts =  INTEGER:0
destAddr     =  FORMAT:HEX,OCT:70002000
imageSize    =  INTEGER:199488

[ image_integrity ]
shaType = OID:2.16.840.1.101.3.4.2.3
shaValue = FORMAT:HEX,OCT:1d1b24e6487709f007d87c8b2b593abf1853a82a99a54650de85f40ddc7b5ae4558a68e49ea3668732ea34ff4bcf76cc73e4e354a3b8128726843c71b05c4168

[ swrv ]
swrv = INTEGER:1

[ encryption ]
Iv =FORMAT:HEX,OCT:a80fd98d0fec9dd2713877fa314474e6
Rstring = FORMAT:HEX,OCT:ec3ceedfd8de93bd429425222df99330e61e872fd8b969ed1f315cde40cbd178
Icount = INTEGER:1
Salt = FORMAT:HEX,OCT:acca65ded29296fea498ab8a9a15aaa27445ab7c75757c99125254619e4a513b

[ key_derivation ]
kd_salt = FORMAT:HEX,OCT:acca65ded29296fea498ab8a9a15aaa27445ab7c75757c99125254619e4a513b
\endcode

Note that the salt in encryption extension and key_derivation extension is same.
This is designed so that the same derived key can be used for encrypting application
images. For more information on the different extensions supported by ROM, please
see \ref TOOLS_BOOT_SIGNING.
\endcond

\cond SOC_AM64X || SOC_AM243X
\note We have enabled full debug while signing the ROM image. This is intentional as this is helpful for debug. Once moved from development to production please remove this option from the makefile. For more details see \ref TOOLS_BOOT_SIGNING
\endcond

\cond SOC_AM263X || SOC_AM263PX || SOC_AM273X || SOC_AWR294X || SOC_AM261X
To build SBL image with certificate for HS-SE devices, one can use the following command:
\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
\code
make -s -C examples/drivers/boot/sbl_uart/am263x-cc/r5fss0-0_nortos/ti-arm-clang all DEVICE=am263x DEVICE_TYPE=HS
\endcode
\endcond
\cond SOC_AM273X
\code
make -s -C examples/drivers/boot/sbl_uart/am273x-evm/r5fss0-0_nortos/ti-arm-clang all DEVICE=am273x DEVICE_TYPE=HS
\endcode
\endcond
\cond SOC_AWR294X
\code
make -s -C examples/drivers/boot/sbl_uart/awr294x-evm/r5fss0-0_nortos/ti-arm-clang all DEVICE=awr294x DEVICE_TYPE=HS
\endcode
\endcond
This will add the appropriate extensions as part of the X509 certificate and will
encrypt the image  with Customer MEK as well.
\note To build the same via CCS GUI, update the device type on devconfig.mak and rebuild the SBL.
\note Encryption of SBL images is enabled by default using `ENC_SBL_ENABLED`
flag in devconfig file (which is set as `yes`).
\endcond

\cond SOC_AM263X || SOC_AM263PX || SOC_AM273X || SOC_AM261X
Keyring can be imported in SBL by using keyring import module in sysconfig.
- Add instance of keyring import module in sysconfig.
\imageStyle{keyring_syscfg_1.png,width:50%}
\image html keyring_syscfg_1.png "Keyring Import Module Instance"


- Load the keyring certificate header file which is generated using \ref KEYRING_CERT_GEN_PYTHON_SCRIPT.
\imageStyle{keyring_syscfg_2.png,width:50%}
\image html keyring_syscfg_2.png ""

The sysconfig module auto-generates APIs for keyring import and builds SBL along with keyring certificate.
For certificate generation script usage, see \ref KEYRING_CERT_GEN_PYTHON_SCRIPT.
\endcond

\cond SOC_AM64X | SOC_AM243X
#### Signing the HSM Runtime binary (SYSFW)
As mentioned above, since we follow a combined boot method, SYSFW and SBL is signed with the same certificate using the same key. In case of a GP device this will be a degenerate key for easy parsing from ROM. In the case of an HS device, SYSFW will be already signed with TI MPK (and encrypted). This is then countersigned again with dummy customer MPK during the combined image generation process.
\endcond

#### Secure application image Generation

Depending on the options given in the device configuration file (`devconfig.mak` mentioned above), appimage is generated for HS devices. If encryption is enabled in the configuration file, the binary will be first encrypted with the key specified and then the certificate will be generated using the customer MPK specified. If the device type is set as HS in the configuration file, nothing extra needs to be done for the appimage generation. The final `*.appimage.hs` file generated would be signed with customer MPK (and encrypted with customer MEK if that option is selected).
\cond SOC_AM64X | SOC_AM243X
To dig into the details of the process, one can refer to https://software-dl.ti.com/tisci/esd/latest/6_topic_user_guides/secure_boot_signing.html

The SBL doesn't have innate abilities to do the image integrity check, or verify the SHA512 of the application image. It relies on SYSFW for this. The image is stored in a readable memory and a pointer to the start of the image is passed to the HSMRt with other details like load address, type of authentication etc.
\endcond

\cond SOC_AM263X || SOC_AM263PX || SOC_AM273X || SOC_AWR294X || SOC_AM261X
The SBL doesn't have innate abilities to do the image integrity check, or verify the hash of the application image. It relies on HSMRt for this. The image is stored in a readable memory and a pointer to the start of the image is passed to the HSMRt with other details like load address, type of authentication etc.
\endcond

\cond SOC_AM64X || SOC_AM243X
For more information regarding the authentication request, please refer to http://downloads.ti.com/tisci/esd/latest/2_tisci_msgs/security/PROC_BOOT.html#proc-boot-authenticate-image-and-configure-processor
\endcond

\cond SOC_AM263X || SOC_AM263PX || SOC_AM273X || SOC_AWR294X || SOC_AM261X
Here is an example of x509 certificate template for application images:
\code
[ req ]
distinguished_name     = req_distinguished_name
x509_extensions        = v3_ca
prompt                 = no

dirstring_type = nobmp

[ req_distinguished_name ]
C                      = US
ST                     = SC
L                      = New York
O                      = Texas Instruments., Inc.
OU                     = DSP
CN                     = Albert
emailAddress           = Albert@gt.ti.com

[ v3_ca ]
basicConstraints = CA:true
1.3.6.1.4.1.294.1.1=ASN1:SEQUENCE:boot_seq
1.3.6.1.4.1.294.1.2=ASN1:SEQUENCE:image_integrity
1.3.6.1.4.1.294.1.3=ASN1:SEQUENCE:swrv

1.3.6.1.4.1.294.1.12=ASN1:SEQUENCE:keyring_index

[ boot_seq ]
certType     =  INTEGER:2779054080
bootCore     =  INTEGER:0
bootCoreOpts =  INTEGER:0
destAddr     =  FORMAT:HEX,OCT:00000000
imageSize    =  INTEGER:30332

[ image_integrity ]
shaType = OID:2.16.840.1.101.3.4.2.3
shaValue = FORMAT:HEX,OCT:fd7e922c982cce91be14d5335c9afe34b171c0686103a0159f2334aa39dcd13f244e9f635216b6178f4f81bb7b67cd7596458f8835322e9e203b1f792ae8e874

[ swrv ]
swrv = INTEGER:1

[ keyring_index ]
sign_key_id = INTEGER:0
enc_key_id  = INTEGER:0
\endcode

The application images can be built with the following flags for appending the
X509 certificate for HS-SE devices:

\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
\code
make -s -C examples/hello_world/am263x-cc/r5fss0-0_nortos/ti-arm-clang all DEVICE=am263x DEVICE_TYPE=HS
\endcode
\endcond
\cond SOC_AM273X
\code
make -s -C examples/hello_world/am273x-evm/r5fss0-0_nortos/ti-arm-clang all DEVICE=am273x DEVICE_TYPE=HS
\endcode
\endcond
\cond SOC_AWR294X
\code
make -s -C examples/hello_world/awr294x-evm/r5fss0-0_nortos/ti-arm-clang all DEVICE=awr294x DEVICE_TYPE=HS
\endcode
\endcond

\cond SOC_AM263X || SOC_AM263PX || SOC_AM273X || SOC_AM261X
##### RSASSA-PSS Support

The RSASSA-PSS algorithm can be used instead of the default RSA PKCS#1v1.5 signature algorithm by specifying RSASSAPSS_ENABLED=yes and APP_SIGNING_SALT_LENGTH. Maximum supported salt length value for RSASSA-PSS is **255**.

See the command below for reference.
\endcond

\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
\code
make -s -C examples/hello_world/am263x-cc/r5fss0-0_nortos/ti-arm-clang all DEVICE=am263x DEVICE_TYPE=HS RSASSAPSS_ENABLED=yes APP_SIGNING_SALT_LENGTH=64
\endcode
\endcond
\cond SOC_AM273X
\code
make -s -C examples/hello_world/am273x-evm/r5fss0-0_nortos/ti-arm-clang all DEVICE=am273x DEVICE_TYPE=HS RSASSAPSS_ENABLED=yes APP_SIGNING_SALT_LENGTH=64
\endcode
\endcond

\cond SOC_AM263X || SOC_AM263PX || SOC_AM273X || SOC_AM261X
##### Auxiliary Keys Support
\note Sample auxiliary keys are present at location `${SDK_INSTALL_PATH}/tools/keyring_cert/aux_keys`.
\note RSASSA-PSS algorithm can be used with RSA auxiliary keys.

The auxiliary keys in the keyring begin at index 32. If the application binary is signed with an auxiliary private key, the corresponding public key hash of which is available in the keyring, the customer must specify the keyring index corresponding to the auxiliary key used for authentication.

If imported public key is available at index 38 in keyring for brainpoolp512r1_private.pem,
the application images can be built with the following flags for appending the
X509 certificate for HS-SE devices:

\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
\code
make -s -C examples/hello_world/am263x-cc/r5fss0-0_nortos/ti-arm-clang all DEVICE=am263x DEVICE_TYPE=HS APP_SIGNING_KEY_KEYRING_ID=38 APP_SIGNING_KEY=/home/user/ti/mcu_plus_sdk/tools/keyring_cert/aux_keys/brainpoolp512r1_private.pem
\endcode
\endcond
\cond SOC_AM273X
\code
make -s -C examples/hello_world/am273x-evm/r5fss0-0_nortos/ti-arm-clang all DEVICE=am273x DEVICE_TYPE=HS APP_SIGNING_KEY_KEYRING_ID=38 APP_SIGNING_KEY=/home/user/ti/mcu_plus_sdk/tools/keyring_cert/aux_keys/brainpoolp512r1_private.pem
\endcode
\endcond
\endcond
\cond SOC_AM263X || SOC_AM263PX || SOC_AM273X || SOC_AM261X
Here is an example of x509 certificate template for application images:
\code

[ req ]
distinguished_name     = req_distinguished_name
x509_extensions        = v3_ca
prompt                 = no

dirstring_type = nobmp

[ req_distinguished_name ]
C                      = US
ST                     = SC
L                      = New York
O                      = Texas Instruments., Inc.
OU                     = DSP
CN                     = Albert
emailAddress           = Albert@gt.ti.com

[ v3_ca ]
basicConstraints = CA:true
1.3.6.1.4.1.294.1.1=ASN1:SEQUENCE:boot_seq
1.3.6.1.4.1.294.1.2=ASN1:SEQUENCE:image_integrity
1.3.6.1.4.1.294.1.3=ASN1:SEQUENCE:swrv

1.3.6.1.4.1.294.1.12=ASN1:SEQUENCE:keyring_index

[ boot_seq ]
certType     =  INTEGER:2779054080
bootCore     =  INTEGER:0
bootCoreOpts =  INTEGER:0
destAddr     =  FORMAT:HEX,OCT:00000000
imageSize    =  INTEGER:30332

[ image_integrity ]
shaType = OID:2.16.840.1.101.3.4.2.3
shaValue = FORMAT:HEX,OCT:fd7e922c982cce91be14d5335c9afe34b171c0686103a0159f2334aa39dcd13f244e9f635216b6178f4f81bb7b67cd7596458f8835322e9e203b1f792ae8e874

[ swrv ]
swrv = INTEGER:1

[ keyring_index ]
sign_key_id = INTEGER:38
enc_key_id  = INTEGER:0
\endcode
\endcond
### Encryption support for application images
\note Support for encryption with auxiliary keys will be added in future releases.

Optionally, one can encrypt the application image to meet security goals.
This can be accomplished with adding one more flag ENC_ENABLED with the make command:
\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
\code
make -s -C examples/hello_world/am263x-cc/r5fss0-0_nortos/ti-arm-clang all DEVICE=am263x DEVICE_TYPE=HS ENC_ENABLED=yes
\endcode
\endcond
\cond SOC_AM273X
\code
make -s -C examples/hello_world/am273x-evm/r5fss0-0_nortos/ti-arm-clang all DEVICE=am273x DEVICE_TYPE=HS ENC_ENABLED=yes
\endcode
\endcond
\cond SOC_AWR294X
\code
make -s -C examples/hello_world/awr294x-evm/r5fss0-0_nortos/ti-arm-clang all DEVICE=awr294x DEVICE_TYPE=HS ENC_ENABLED=yes
\endcode
\endcond

Here is an example x509 configuration template with apllication authentication and encryption support:
\code

[ req ]
distinguished_name     = req_distinguished_name
x509_extensions        = v3_ca
prompt                 = no

dirstring_type = nobmp

[ req_distinguished_name ]
C                      = US
ST                     = SC
L                      = New York
O                      = Texas Instruments., Inc.
OU                     = DSP
CN                     = Albert
emailAddress           = Albert@gt.ti.com

[ v3_ca ]
basicConstraints = CA:true
1.3.6.1.4.1.294.1.1=ASN1:SEQUENCE:boot_seq
1.3.6.1.4.1.294.1.2=ASN1:SEQUENCE:image_integrity
1.3.6.1.4.1.294.1.3=ASN1:SEQUENCE:swrv
1.3.6.1.4.1.294.1.4 = ASN1:SEQUENCE:encryption
1.3.6.1.4.1.294.1.12=ASN1:SEQUENCE:keyring_index

[ boot_seq ]
certType     =  INTEGER:2779054080
bootCore     =  INTEGER:0
bootCoreOpts =  INTEGER:0
destAddr     =  FORMAT:HEX,OCT:00000000
imageSize    =  INTEGER:30368

[ image_integrity ]
shaType = OID:2.16.840.1.101.3.4.2.3
shaValue = FORMAT:HEX,OCT:d65fe2f380a4aa365ec90eb646e68c2f1dc738bfd5ba7ee0385cea9f41d1101fac6141c3981367493e17a883d588b692ded275daacaf33ff70a38a5e04cc12fc

[ swrv ]
swrv = INTEGER:1

[ keyring_index ]
sign_key_id = INTEGER:0
enc_key_id  = INTEGER:0


[ encryption ]
initalVector =  FORMAT:HEX,OCT:c31abc423ccd831385b7fdd649be0ee1
randomString =  FORMAT:HEX,OCT:347c0707c36290697d294770da607d43d02d41242eadcd718bb825589d437f22
iterationCnt =  INTEGER:1
salt         =  FORMAT:HEX,OCT:acca65ded29296fea498ab8a9a15aaa27445ab7c75757c99125254619e4a513b
\endcode
\endcond

## Limitations in Secure Boot

\cond SOC_AM64X || SOC_AM243X
- **XIP boot** : Secure boot is yet to be supported for XIP applications. This is due to the fact that the XIP sections are loaded before the SBL parses the other sections. Secure boot of XIP applications will be made available in an upcoming release.

- **Encryption of application image not possible in SBL OSPI** : In other bootloaders like UART and SD, application image can be encrypted using the `ENC_ENABLED` option in the devconfig.mak. But this is not possible when you load the image using SBL OSPI. This is due to the fact that HSM does an in-place authentication and decryption of the image and we load the image directly from the FLASH memory in case of SBL OSPI. FLASH memory, as you would know is most often not directly writable. Due to this limitation not being taken care in the HSM, we can do decryption of images only in the case where the image resides in a volatile RAM-like memory. That is MSMC or DDR.
\endcond

\cond SOC_AM263X || SOC_AM263PX || SOC_AM273X || SOC_AWR294X || SOC_AM261X
- **Authentication of application image directly from flash in SBL QSPI** : Only authentication of application
image from flash is supported in SBL QSPI. This is susceptible to Man-in-The-Middle (MiTM) attacks if Flash is overwritten post-auth.

- **Encryption of application image not supported in SBL QSPI** : In UART bootloader, application image can be encrypted using the `ENC_ENABLED` option in the devconfig.mak. But this is not possible when you load the image using SBL QSPI. This is due to the fact that HSM does an in-place authentication and decryption of the image and we load the image directly from the FLASH memory in case of SBL QSPI. FLASH memory, as you would know is most often not directly writable. Due to this limitation not being taken care in the HSM, we can do decryption of images only in the case where the image resides in a volatile RAM-like memory like OCRAM.

\cond ~SOC_AWR294X
- **Secure Boot is untested on SBL SD and SBL CAN**
\endcond
\cond SOC_AWR294X
- **Currently Secure Boot is supported on SBL_NULL, SBL_QSPI and SBL_UART only**
\endcond
\endcond
