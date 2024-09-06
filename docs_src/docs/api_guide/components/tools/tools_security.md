# Security Related Tools {#TOOLS_SECURITY}

## Signing Scripts {#TOOLS_BOOT_SIGNING}

- To run these scripts, one needs `openssl` installed as mentioned here, \ref INSTALL_OPENSSL
- Signing scripts are a collection of scripts needed to sign ROM images (image booted by ROM - mostly the SBL) and application images (image booted by the SBL)
\cond SOC_AM263X || SOC_AM263PX||SOC_AM273X||SOC_AWR294X || SOC_AM261X
### Signing SBL
\endcond
- The RBL requires the boot image (mostly SBL), to be signed always, even if we are not using secure boot.
  \cond SOC_AM64X || SOC_AM243X
- We follow a combined boot method for ROM images. Here the ROM Bootloader (RBL) boots the SBL, SYSFW and BOARDCFG together. The boot image would be a binary concatenation of x509 Certificate, SBL, SYSFW, BOARDCFG (and the SYSFW inner certificate in case of HS device) binary blobs. We use a python script to generate this final boot image. This script has a dependency on `openssl` as mentioned before, so make sure you've installed it. To generate a combined boot image, one can do as below:

- For GP devices
  \code
  cd ${SDK_INSTALL_PATH}/source/security/security_common/tools/boot/signing
  ${PYTHON} rom_image_gen.py --swrv 1 --sbl-bin <path-to-sbl-binary> --sysfw-bin <path-to-sysfw-binary> --boardcfg-blob <path-to-boardcfg-binary-blob> --sbl-loadaddr ${SBL_RUN_ADDRESS} --sysfw-loadaddr ${SYSFW_LOAD_ADDR} --bcfg-loadaddr ${BOARDCFG_LOAD_ADDR} --key ${BOOTIMAGE_CERT_KEY} --rom-image <path-to-output-image>
  \endcode

- For HS devices, we have to pass the HS SYSFW binaries and also the SYSFW inner certificate to the signing script.
  \code
  cd ${SDK_INSTALL_PATH}/source/security/security_common/tools/boot/signing
  ${PYTHON} rom_image_gen.py --swrv 1 --sbl-bin <path-to-sbl-binary> --sysfw-bin <path-to-sysfw-binary> --sysfw-inner-cert <path-to-sysfw-inner-cert-binary> --boardcfg-blob <path-to-boardcfg-binary-blob> --sbl-loadaddr ${SBL_RUN_ADDRESS} --sysfw-loadaddr ${SYSFW_LOAD_ADDR} --bcfg-loadaddr ${BOARDCFG_LOAD_ADDR} --key ${BOOTIMAGE_CERT_KEY} --debug DBG_FULL_ENABLE --rom-image <path-to-output-image>
  \endcode

- By default SBLs provided in SDK are signed with full debug enable since this is needed for development. You can see from `--debug` switch used above. Once moved to production please remove this switch from the makefile.

- For SBL images or examples which is loaded by SBL, we use a different signing script. This is solely because of the x509 certificate template differences between ROM and SYSFW. In GP devices appimages are not signed. The signing happens only in HS devices. The script usage is:
  \code
  cd ${SDK_INSTALL_PATH}/source/security/security_common/tools/boot/signing
  $(PYTHON) appimage_x509_cert_gen.py --bin <path-to-the-binary> --authtype 1 --key <signing-key-derived-from-devconfig> --output <output-image-name>
  \endcode

- In the case of encryption, two extra options are also passed to the script like so:
  \code
  cd ${SDK_INSTALL_PATH}/source/security/security_common/tools/boot/signing
  $(PYTHON) appimage_x509_cert_gen.py --bin <path-to-the-binary> --authtype 1 --key <signing-key-derived-from-devconfig> --enc y --enckey <encryption-key-derived-from-devconfig> --output <output-image-name>
  \endcode

- These scripts are invoked in makefiles, and the image generation happens automatically along with the example build. So mostly these scripts need not be manually run.
\endcond

\cond SOC_AM263X || SOC_AM263PX || SOC_AM273X || SOC_AWR294X || SOC_AM261X

For SBL images, the script `/source/security/security_common/tools/boot/signing/mcu_rom_image_gen.py`.

For HS-FS devices:
\code
cd ${SDK_INSTALL_PATH}/source/security/security_common/tools/boot/signing
$(PYTHON) mcu_rom_image_gen.py --image-bin <path-to-the-binary> --core R5 --swrv 1 --loadaddr $(SBL_RUN_ADDRESS) --sign-key <signing-key-configured-in-devconfig> --out-image <output-image-name> --debug $(DEBUG_OPTION)
\endcode

For HS-SE devices:
\code
cd ${SDK_INSTALL_PATH}/source/security/security_common/tools/boot/signing
$(PYTHON) mcu_rom_image_gen.py --sbl-enc --enc-key <encryption-key-configured-in-devconfig> --image-bin <path-to-the-binary> --core R5 --swrv 1 --loadaddr $(SBL_RUN_ADDRESS) --sign-key <signing-key-configured-in-devconfig> --kd-salt <path-to-key-derivation-salt> --out-image <output-image-name> --debug $(DEBUG_OPTION)
\endcode

Here, we deep dive into the extension supported in SBL and HSM Runtime (TIFS-MCU)
certificate by ROM.
To get access to the HSM Runtime code and ability to build it for HS-SE device,
please contact your TI representative to get access to the TIFS-MCU package from
MySecureSW.

#### Boot Information OID (1.3.6.1.4.1.294.1.1)
\code
bootInfo ::=  SEQUENCE {
	cert_type:  INTEGER,		-- identifies the certificate type
	boot_core:	INTEGER,		-- identifies the boot core
	core_opts:	INTEGER,		-- Core Options
	load_addr:	OCTET STRING,	-- Global address image destination
	image_size:	INTEGER,		-- Image size in bytes
}
\endcode
The Boot Information Object identifier provides information about the image
which is being loaded. This information is mandatory and needs to be present
else the image boot will fail.

##### Elements of the extension:
**Certificate Type**: The certificate type defines the type of the image which is
being loaded by the Boot-ROM.

These are the supported values:
- 0x1	R5 SBL Boot Image
- 0x2	HSM Runtime (TIFS-MCU) Image

**Boot core**: The boot core identifies the core on which the image will be executing.

These are the supported values:
- 0x0	HSM Core
- 0x10	R5 Core

**Core Options**: The core options are documented in the table below.

These are the supported values:
- 0x0	    Lock Step Mode
- Non-Zero	Dual Core Mode

\note Currently, configuring this option is not supported with Signing scripts.
This feature will be added in future releases.

Core options are applicable only for the R5 SBL Images.

**Load Address**: The load address will be address in the system where the
 image will be loaded. This is applicable for both SBL and TIFS-MCU (HSMRt)
images.

**Image Size**: This is the size in bytes of the R5 SBL or HSM Runtime Image
to which the certificate has been attached.

#### Image Integrity OID (1.3.6.1.4.1.294.1.2)

\code
imageIntegrity ::= SEQUENCE {
	sha_type:	OID,			-- Identifies the SHA type
	hash:		OCTET STRING	-- The SHA of the boot image
}
\endcode

If the X.509 certificate provides the image integrity boot extension,
the Boot-ROM will perform the SHA-512 on the entire image and will verify
 the computed hash with the hash provided in the boot extension.
 In the case of a mismatch the boot will fail.

##### Elements of the extension:

**SHA Type**: The Boot-ROM only supports SHA-512.

The following values are supported:

2.16.840.1.101.3.4.2.3	SHA-512 Object Identifier

Please refer to the Section 2.4 of the RFC-5754 for the SHA-512 Object Identifier.

**Hash**: This is SHA-512 hash which is calculated over the image (R5 SBL/HSM Runtime)

R5 SBL Image:
-	HS-SE Device: Image Integrity is mandatory
-	HS-FS Device: Image Integrity is optional

HSM Runtime Image:
-	HS-SE Device: Image Integrity is mandatory
-	HS-FS Device: Image Integrity is mandatory

#### Software Revision OID (1.3.6.1.4.1.294.1.3)

\code
softwareRevision ::= SEQUENCE {
   revision:	INTEGER -- Software revision
}
\endcode

The information in the software revision is used to indicate the version of
the image which is being loaded.

##### Elements of the extension:

**revision**:
This is the version number. This will be matched to the EFUSE programmed
version to indicate if the image loading should be done or not. This is
applicable only for SE devices.

|EFUSE Revision|	Certificate Revision|	Description |
|--------------|------------------------|---------------|
|0 |	0 | Ignore the revision checking. Images will *always* be loaded|
|0 | >0 | Device does not mandate revision checking. Images will be loaded|
|>0|	0 |EFUSE Version > Certificate Version. Image will *never* be loaded.|
|>0|	>0 |Image will be loaded only if the Certificate revision >= EFUSE revision|

The following fields in efuse ROM help support the above feature:
- SWRV_SBL- This is used to perform the revision checking while loading the R5 SBL
- SWRV_HSM- This is used to perform the revision checking while loading the HSM Runtime

For more information, about how to fuse in these values, please refer to OTP
Keywriter Documentation available on MySecureSW.

#### Image Encryption OID (1.3.6.1.4.1.294.1.4)

\code
imageEncryption ::= SEQUENCE {
   iv:		OCTET STRING -- The initialization vector
   rs:	      OCTET STRING -- Random string
   iter:		INTEGER      -- Iteration count
   salt:		OCTET STRING -- encryption salt value
}
\endcode

The Boot-ROM only supports AES-CBC mode with 256bit keys. The information in the image encryption object identifier is used to decrypt the image.

##### Elements of the extension:

**IV**:
The initialization vector is used during the AES-CBC decryption procedure. The initialization vector needs to be 16bytes.

**rs**:
This is the random string which is 32bytes long and is appended by the script
at the end of the image. The Boot-ROM will decrypt the image and will perform
 a random string comparison to determine if the decryption was successful.

**iter**:
Iteration Count which is used to determine if the HKDF needs to be performed
and key derivation needs to be done. If the iteration count is 0 then the key
from the e-fuse is used as is for the decryption. If the iteration count is
non-zero then the Boot-ROM will perform the HKDF key derivation using the salt
with exactly one iteration (even if iteration count is > 1).
The derived key is then used for the decryption operation.

**salt**:
The salt is used only if the iteration count is non-zero and key derivation is
being done. The salt is fed to the HKDF module to derive the key. The salt
fields should be 32bytes.

For recommendation on the salt, please refer to \ref SECURE_BOOT.

\note It is recommended to update key derivation salt everytime there is a
firmware upgrade. SBL, HSMRt and application images need to be prepared
again, once the salt is changed.

- HS-SE device: Image Encryption is optional
- HS-FS device: Image Encryption is optional

#### Derivation OID (1.3.6.1.4.1.294.1.5)

\code
derivationKey ::= SEQUENCE {
   salt:      OCTET STRING -- encryption salt value
}
\endcode
The Boot-ROM will leave a derived key in the assets interface for the
TIFS-MCU. The key is derived using HKDF from the parameters specified here.
This is useful for decrypting application images by sending request
to TIFS-MCU.
For recommendation on the salt, please refer to \ref SECURE_BOOT.

##### Elements of the extension:

salt:
The salt is limited to be 32bytes and is used for key derivation

This OID is ignored as part of HSM Runtime certificates. This OID is ignored
as part of SBL certificates if the device type is HS-FS.
Key derivation is always done using the active user symmetric key.

#### Debug OID (1.3.6.1.4.1.294.1.8)

\code
Debug::= SEQUENCE {
      uid          OCTET STRING     -- Device unique ID
      debugType    INTEGER          -- Debug type
      coreDbgEn    INTEGER          -- Enable core debug mask
      secCoreDbgEn INTEGER          -- Enable secure core debug mask
}
\endcode
The debug object identifier if specified allows the debug ports to be enabled
for a specific device.

##### Elements of the extension:

**UID**: This is the unique identifier associated with the device.
Device specific unique identifiers can be retrieved using the SOC_ID Parser.

The UID field of all 0s is considered to be a wildcard. This is what is supported
by default in the script.

**Debug Type**:
This is the privilege level of debug.

The privilege levels supported by ROM are as follows:

Privilege Level   | Value | Description
------------------|-------|--------------------
DBG_PERM_DISABLE  |	0     | Disable debug ports for all cores.
DBG_SOC_DEFAULT   | 1     | Maintain debug ports for all cores to device type defaults.
DBG_PUBLIC_ENABLE | 2     |	Enable debug ports on R5FSS0-0 core.

coreDbgEn and secCoreDbgEn: These fields are not used and will be ignored.

\note The Debug OID is not applicable for HSM Runtime (TIFS-MCU) image.

In the SBL Signing scripts, `DEBUG_OPTION` is used to exercise the debug level
if the SBL certificate has debug extension. The SBL certificate as mentioned
before, is processed by ROM. This is different from the debug certificate
supported by TIFS-MCU.

This feature is leveraged via compile time flags in MCU+ SDK for SBL images.

By default, in SDK, the debug extension is not engaged for SBL images.
This is decided by the flag `DEBUG_TIFS` defined in `devconfig.mak`.
By default, `DEBUG_TIFS` is set to `yes`.This means that the debug extension
will not be added to SBL and if the debug needs to be exercised via the certificate
to TIFS-MCU via the debug authentication service, it can be safely done.
This is done to ensure that the registers are written once only to exercise the
debug during a POR cycle.
If the user wants to open up debug on an HS-SE device, they can set `DEBUG_TIFS=no`.
This is when the debug extension is added to the certificate for SBL images.

\note ROM does not allow FULL_ENABLE privilege level via SBL certificate. To open up
debug on HSM core, please refer to Debug Authentication services provided via
TIFS-MCU. In this case, please use the default value of `DEBUG_TIFS` i.e. `yes`
when compiling SBL examples.

The default debug privilege used in SDK is `DBG_SOC_DEFAULT`.

To enable debug on public cores on HS-SE device, one can use the following command
to compile SBL examples.

\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
\code
make -s -C examples/drivers/boot/sbl_qspi/am263x-cc/r5fss0-0_nortos/ti-arm-clang all DEVICE=am263x DEVICE_TYPE=HS DEBUG_TIFS=no DEBUG_OPTION=DBG_PUBLIC_ENABLE
\endcode
\endcond

\cond SOC_AM273X
\code
make -s -C examples/drivers/boot/sbl_qspi/am273x-cc/r5fss0-0_nortos/ti-arm-clang all DEVICE=am273x DEVICE_TYPE=HS DEBUG_TIFS=no DEBUG_OPTION=DBG_PUBLIC_ENABLE
\endcode
\endcond

\cond SOC_AWR294X
\code
make -s -C examples/drivers/boot/sbl_qspi/awr294x-evm/r5fss0-0_nortos/ti-arm-clang all DEVICE=awr294x DEVICE_TYPE=HS DEBUG_TIFS=no DEBUG_OPTION=DBG_PUBLIC_ENABLE
\endcode
\endcond

\endcond

 - Here,
\cond SOC_AM64X || SOC_AM243X
  - `SBL_RUN_ADDRESS` is `0x70000000`
  - In the case of GP device, `BOOTIMAGE_CERT_KEY` is `rom_degenerateKey.pem`
  - In the case of HS device, `BOOTIMAGE_CERT_KEY` is `custMpk_am64x_am243x.pem`. For more details about this see \ref SECURE_BOOT
\endcond
\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
  - `SBL_RUN_ADDRESS` is `0x70002000`
  - In the case of HS-FS devices, the key value is disregarded. A degenerate RSA public key is used to sign the certificate. The image integrity is checked in ROM, nevertheless.
  - In the case of HS-SE device, more details can be found at \ref SECURE_BOOT
  - Refer to TRM Chapter on Initialization, section 5.6.4.1.1 to get help on RSA key pair generation.
\endcond

\cond SOC_AM65X
  - `SBL_RUN_ADDRESS` is `0x41C00100`
  - In the case of GP device, `BOOTIMAGE_CERT_KEY` is `rom_degenerateKey.pem`.
\endcond

These scripts are invoked in makefiles, and the image generation happens
automatically along with the example build. So mostly these scripts need
not be manually run.
If the user build-system is different from TI's makefile system, it needs to
be ensured that the same is followed as part of the post build steps.
The devconfig has ENC_SBL_ENABLED=yes and that is why for HS-SE devices, the SBL
image is encrypted by default.


The Boot Information Object identifier provides information about the image
which is being loaded.

##### Elements of the extension:
**Certificate Type**: The certificate type defines the type of the image which is
being loaded by the Boot-ROM.

These are the supported values:
- 0xA5A50000	Application Image

**Boot core**: This field is reserved and should be assigned with 0.

**Core Options**: This field is reserved and should be assigned with 0.

**Load Address**: This field is reserved and should be assigned with 0.

**Image Size**: This is the size in bytes of the R5 SBL or HSM Runtime Image
to which the certificate has been attached.

#### Image Integrity OID (1.3.6.1.4.1.294.1.2)

\code
imageIntegrity ::= SEQUENCE {
	sha_type:	OID,			-- Identifies the SHA type
	hash:		OCTET STRING	-- The SHA of the boot image
}
\endcode

If the X.509 certificate provides the image integrity boot extension,
the TIFS-MCU will perform the hash calculation on the entire image and will verify
the computed hash with the hash provided in the boot extension.
In the case of a mismatch the boot will fail.

##### Elements of the extension:

**SHA Type**:

The following values are supported:

- 2.16.840.1.101.3.4.2.1	SHA-256 Object Identifier
- 2.16.840.1.101.3.4.2.2	SHA-384 Object Identifier
- 2.16.840.1.101.3.4.2.3	SHA-512 Object Identifier

Amongst these, SDK has been validated against SHA-512 OID.

**Hash**: This is SHA-512 hash which is calculated over the application image

#### Software Revision OID (1.3.6.1.4.1.294.1.3)

\code
softwareRevision ::= SEQUENCE {
   revision:	INTEGER -- Software revision
}
\endcode

The information in the software revision is used to indicate the version of
the image which is being loaded.

##### Elements of the extension:

**revision**:
This is the version number. This will be matched to the EFUSE programmed
version to indicate if the image loading should be done or not.

The following fields in efuse ROM help support the above feature:
- SWRV_APP- This is used to perform the revision checking while loading the application

For more information, about how to fuse in these values, please refer to OTP
Keywriter Documentation available on MySecureSW.

#### Image Encryption OID (1.3.6.1.4.1.294.1.4)

\code
imageEncryption ::= SEQUENCE {
   iv:		OCTET STRING -- The initialization vector
   rs:	    OCTET STRING -- Random string
   iter:	INTEGER      -- Reserved
   salt:	OCTET STRING -- Reserved
}
\endcode

TIFS-MCU only supports AES-CBC mode with 256bit keys. The information in the image encryption object identifier is used to decrypt the image.

##### Elements of the extension:

**IV**:
The initialization vector is used during the AES-CBC decryption procedure. The initialization vector needs to be 16bytes.

**rs**:
This is the random string which is 32bytes long and is appended by the script
at the end of the image. TIFS-MCU will decrypt the image and will perform
a random string comparison to determine if the decryption was successful.

**iter**:
This field is unused and reserved.

**salt**:
This field is unused and reserved.

#### Keyring Index OID (1.3.6.1.4.1.294.1.12)

\code
keyring_index ::= SEQUENCE {
   sign_key_id:		INTEGER -- index of public key hash in public keyring for authentication
   enc_key_id:	    INTEGER -- index of aes key in keyring for decryption
}
\endcode

TIFS-MCU only supports authentication with keyring. The information in enc_key_id is ignored and decryption is done against root of trust
if Image Encryption OID is available in the certificate.

##### Elements of the extension:

**sign_key_id**:
 This is the index used in public keyring for retrieving the public key hash for authentication.

 **enc_key_id**:
 This is the index used in keyring for retrieving the aes key for decryption of application image.
\endcond

\cond SOC_AM263X || SOC_AM263PX || SOC_AM273X ||SOC_AWR294X || SOC_AM261X
### Application Signing

For Application images, the script `/source/security/security_common/tools/boot/signing/mcu_appimage_x509_cert_gen.py`.

For HS-SE devices:
\code
cd ${SDK_INSTALL_PATH}/source/security/security_common/tools/boot/signing
$(PYTHON) mcu_appimage_x509_cert_gen.py --bin <path-to-the-binary> --key <signing-key-configured-in-devconfig>  --enc y --enckey <encryption-key-configured-in-devconfig>  --kd-salt  <path-to-key-derivation-salt> --output <output-image-name>
\endcode

Signing of application images is not required for HS-FS device types.

Here, we deep dive into the extension supported with certificate for Application images
as processed by TIFS-MCU.
To get access to the HSM Runtime code and ability to build it for HS-SE device,
please contact your TI representative to get access to the TIFS-MCU package from
MySecureSW.

#### Boot Information OID (1.3.6.1.4.1.294.1.1)
\code
bootInfo ::=  SEQUENCE {
	cert_type:  INTEGER,		-- identifies the certificate type
	boot_core:	INTEGER,		-- Reserved
	core_opts:	INTEGER,		-- Reserved
	load_addr:	OCTET STRING,	-- Reserved
	image_size:	INTEGER,		-- Image size in bytes
}
\endcode
\endcond

\cond SOC_AM263X || SOC_AM263PX || SOC_AM273X || SOC_AM261X
## Keyring Cert Generation Python Script {#KEYRING_CERT_GEN_PYTHON_SCRIPT}

\note Support for encryption with symmetric auxiliary keys will be added in future releases.

\imageStyle{keyring_tooling_view.png,width:50%}
\image html keyring_tooling_view.png "Keyring certificate generation and import"

\code
$python3 keyring_cert_gen.py --root_key ../../source/security/security_common/tools/boot/signing/mcu_custMpk.pem --keys_info keys.json
\endcode

- This script is used to generate X.509 certificate for keyring.
- Make sure that python3 and its dependent modules are installed in the host machine as mentioned in \ref INSTALL_PYTHON3
- keyring_cert_gen.py parses json object containing the following meta-data for keyring cert generation:
    - keyring_sw_rev: This integer value is used for anti-rollback check against the keyring software revision available in device efuses.
    - keyring_ver: Keyring version is of type integer and is the version which is supported against the TIFS-MCU version being used in HSM.
    - num_of_asymm_keys: Number of Asymmetric keys being imported.
    - num_of_symm_keys: Number of Symmetric keys being imported.
    - keyring_asymm: Array of asymmetric keys with respective key rights and hash algorithm to compute and store the hash of the public key.
\code
{
    "keyring_sw_rev" : 1,
    "keyring_ver" : 1,
    "num_of_asymm_keys" : 7,
    "num_of_symm_keys" : 0,
    "keyring_asymm" :  [
                {
                    "key_rights": "000000AA",
                    "pub_key": "aux_keys/public2k.pem",
                    "hash_algo": "SHA256"
                },
                {
                    "key_rights": "000000AA",
                    "pub_key": "aux_keys/public3k.pem",
                    "hash_algo" : "SHA512"
                },
                {
                    "key_rights": "000000AA",
                    "pub_key": "aux_keys/public4k.pem",
                    "hash_algo" : "SHA512"
                },
                {
                    "key_rights": "000000AA",
                    "pub_key": "aux_keys/prime265v1_public.pem",
                    "hash_algo" : "SHA512"
                },
                {
                    "key_rights": "000000AA",
                    "pub_key": "aux_keys/secp384r1_public.pem",
                    "hash_algo" : "SHA512"
                },
                {
                    "key_rights": "000000AA",
                    "pub_key": "aux_keys/secp521r1_public.pem",
                    "hash_algo" : "SHA384"
                },
                {
                    "key_rights": "000000AA",
                    "pub_key": "aux_keys/brainpoolp512r1_public.pem",
                    "hash_algo" : "SHA384"
                }
      ],
    "keyring_symm" :  []
}

\endcode
- Each asymmetric key json object contain 3 fields:
    - key_rights: Integer value in HEX format which signifies the rights associated with the key.
        - debugAuth (0-3b) - flag to indicate key right for debug certificate authentication
        - imageAuth (4-7b) - flag to indicate key right for application image authentication
        - For example: 0x000000AA represents the public can be used for both imageAuth and debugAuth
    - pub_key: Location of the public key
    - hash_algo: Hash algorithm used for hashing public key.
        - SHA256, SHA384 and SHA512 are valid hash algorithms.
- keyring_cert_gen.py expects 2 mandatory arguments:
    - root_key: certificate is signed using customer MPK and expects path to customer active ROT.
    - keys_info: json file with keyring meta data.
- If RSASSA-PSS algorithm for authentication of keyring certificate is required, provide the --rsassa_pss flag and --pss_saltlen **value**. Maximum supported salt length value for RSASSA-PSS is **255**. Please refer to the command below
\code
$python3 keyring_cert_gen.py --root_key ../../source/security/security_common/tools/boot/signing/mcu_custMpk.pem --keys_info keys.json --rsassa_pss --pss_saltlen 64
\endcode
- A dummy json and keyring certificate header file is also availble in ${SDK_INSTALL_PATH}/tools/keyring_cert for reference.
- Below are the logs after execution of script
    \code
keyring version = 1
keyring software revision = 1
Number of asymmetric keys = 7
Number of symmetric keys = 0

[ keyring_asymm ]
asymm_key0=SEQUENCE:comp0
asymm_key1=SEQUENCE:comp1
asymm_key2=SEQUENCE:comp2
asymm_key3=SEQUENCE:comp3
asymm_key4=SEQUENCE:comp4
asymm_key5=SEQUENCE:comp5
asymm_key6=SEQUENCE:comp6

[ comp0 ]
keyId=INTEGER:32
key_rights=FORMAT:HEX,OCT:000000AA
hash_algo=INTEGER:6
public_key=FORMAT:HEX,OCT:582fc2a6cc997d2df6c0299aeef0a7b606d3fcc2261dfade8c2684c6663ab50b

[ comp1 ]
keyId=INTEGER:33
key_rights=FORMAT:HEX,OCT:000000AA
hash_algo=INTEGER:4
public_key=FORMAT:HEX,OCT:6e430c4a09572240f9acd4b4777b2d662fe28c73ef475280d8a4e7c766d13680b14549a84e6e6c7b69a0ba33840dc9f51f110afb156251a6139c26693e6d4ddf

[ comp2 ]
keyId=INTEGER:34
key_rights=FORMAT:HEX,OCT:000000AA
hash_algo=INTEGER:4
public_key=FORMAT:HEX,OCT:67cd4b91e7966af635a53642102abb2c8fdab1a47277c88eb4ec3e1a6ce5155a2325f66b3d392f3f5091e526057cd5e6b6d5085ae46f169c61de77c7dffbb238

[ comp3 ]
keyId=INTEGER:35
key_rights=FORMAT:HEX,OCT:000000AA
hash_algo=INTEGER:4
public_key=FORMAT:HEX,OCT:79bd183d942cbf1542a2bad2a469242392c515c1546fc8a43e86a024097ba4bb4db65ccade7fb6b252deccd255491151df927f998d468153072cf3e4949d6002

[ comp4 ]
keyId=INTEGER:36
key_rights=FORMAT:HEX,OCT:000000AA
hash_algo=INTEGER:4
public_key=FORMAT:HEX,OCT:4a8959978336886acbe3cf294b1b8e1aa68f1cb836d78aa11332a57cb44c435c82fc672901a2a242e5169543502106ac0cda1b04a9769723154d640e832298ba

[ comp5 ]
keyId=INTEGER:37
key_rights=FORMAT:HEX,OCT:000000AA
hash_algo=INTEGER:2
public_key=FORMAT:HEX,OCT:1b759f0c0b04796cc599033bd58e33730ce6b2380f7442fe030beffa3cc844ce0d196f48e5ebeb6d88eea0f7ddc3beb7

[ comp6 ]
keyId=INTEGER:38
key_rights=FORMAT:HEX,OCT:000000AA
hash_algo=INTEGER:2
public_key=FORMAT:HEX,OCT:b502e951a5f5ed4bc99191511b530597d3a2d356d0f83887a54253a7ec46fb2c081c7d39391f846932357a57133f8c11
    \endcode

\endcond

##  Boot time calculator tool

Checkout the secure boot time calculator tool <a href="../boottime_calculator/index.html">here</a>.

