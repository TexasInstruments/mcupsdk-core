# Security X509 Certificate Documentation {#SECURITY_X509_CERTIFICATE_DOCUMENTATION}

[TOC]

## Introduction
This document describes the X509 extensions supported by the X509 parser in TIFS/hsmRT.

\note This document is only applicable to HS devices. TIFS/hsmRT does not include a X509 parser on GP devices.
## References
- 1. ISO 8824-1 | ITU-T X.680 (08/2015): Information technology - Abstract Syntax Notation One (ASN.1): Specification of basic notation, http://handle.itu.int/11.1002/1000/12479
- 2. ISO 8825-1 | ITU-T X.690 (09/2015): Information technology - ASN.1 encoding rules: Specification of Basic Encoding Rules (BER), Canonical Encoding Rules (CER) and Distinguished Encoding Rules (DER) http://handle.itu.int/11.1002/1000/12483
- 3. ISO/IEC 9594-8 | ITU-T X.509 (10/2016): Information technology - Open Systems Interconnection - The Directory: Public-key and attribute certificate frameworks http://handle.itu.int/11.1002/1000/13031
- 4. Processor Boot Management TISCI Description
## Extensions
The following X509 extensions are supported by TIFS/hsmRT.

Extension Name                                     |Purpose                               |OID
---------------------------------------------------|--------------------------------------|---------------------
TIFS/hsmRT Boot Extension                          |Provide boot information              |1.3.6.1.4.1.294.1.33
TIFS/hsmRT Image Integrity Extension               |Image hash and length                 |1.3.6.1.4.1.294.1.34
TIFS/hsmRT Software Revision Extension             |revision of binary for anti-rollback  |1.3.6.1.4.1.294.1.3
TIFS/hsmRT Load Extension                          |Provide load information              |1.3.6.1.4.1.294.1.35
TIFS/hsmRT Debug Extension                         |To unlock debug port                  |1.3.6.1.4.1.294.1.8
TIFS/hsmRT Encryption Extension                    |Encryption extension                  |1.3.6.1.4.1.294.1.4
TIFS/hsmRT HS Board Configuration Extension        |HS Boardcfg extension                 |1.3.6.1.4.1.294.1.36

### TIFS/hsmRT Boot Extension
This extension adds support for booting various cores on a K3 SOC. It is identified by the OID 1.3.6.1.4.1.294.1.33. The structure of the field is shown below in ASN.1 notation.
\code
(TIFS/HSMRT)-Boot := SEQUENCE
{
    bootCore INTEGER, -- indicates the core in the device that needs to be booted
                      -- with the image accompanying this certificate.
    configFlags_set INTEGER, -- Configuration options for the core being booted.
                             -- flags to set
    configFlags_clr INTEGER, -- Configuration options for the core being booted
                             -- flags to clear
    resetVec OCTET STRING, -- Location of reset vector for the core.

    fieldValid INTEGER -- indicates which of the reserved fields in the extension are
                       -- valid
    rsvd1     INTEGER, -- reserved field for future use
    rsvd2     INTEGER, -- reserved field for future use
    rsvd3     INTEGER, -- reserved field for future use
}
\endcode

The boot extension is decoded into the below structure.
\code
struct sec_boot_ctrl {
    u32 bootCore;
    u32 configFlags_set;
    u32 configFlags_clr;
    u64 resetVec;
}
\endcode
The reserved fields are for future use and are not decoded currently.

Populating the certificate fields
For an example, please refer to the section [ (TIFS/hsmRT)_boot_seq ] in Sample x509 template.

- bootCore should be initialized to the Core ID of the targeted core. Core ID should be obtained from the SOC data. e.g. For booting a core with ID 0x20, the relevant line in the X509 template is

[ boot_seq ]
bootCore = INTEGER:0x20

- configFlags_set

This is a 32 bit field that indicates core specific flags that need to be set before booting the core. The representation is big endian.

- configFlags_clr
    - This is a 32 bit field that indicates core specific flags that need to be cleared before booting core. The representation is big endian.
    - Please refer to the SOC and core specific documentation on the supported flags. The flags are passed to set_processor_config API as is. Please ensure that the flags set or cleared are valid according to the API.
- resetVec
    - This is a 64 bit field indicating the address of the reset vector that needs to be programmed. The representation is big endian.


### TIFS/hsmRT Image Integrity Extension
This extension adds support for specifying a hash and size of the accompanying payload. It is identified by the OID 1.3.6.1.4.1.294.1.34. The structure of the field is shown below in ASN.1 notation.
\code
(TIFS/HSMRT)-INTEGRITY := SEQUENCE
{
   shaType OID, -- indicates OID of the hash used. Must always be set to SHA2-512
                -- OID:2.16.840.1.101.3.4.2.3
   shaValue OCTET STRING, -- SHA2-512 value of the payload
   imageSize INTEGER, -- Size of the image in bytes. This will be amount of data
                      -- used when checking the image integrity, copying the image
                      -- to its destination or when decrypting the image.
}
\endcode

### TIFS/hsmRT Software Revision Extension
TIFS/hsmRT reuses the Software Revision extension/OID as defined by ROM as no additional information is required. This is identified by OID 1.3.6.1.4.1.294.1.3.
\code
(TIFS/HSMRT)-SWREV:= SEQUENCE
{
   swrev INTEGER -- 32 bit value indicating the revision of the binary
}
\endcode
### TIFS/hsmRT Load Extension
TIFS/hsmRT uses the Load extension to determine where the image is loaded as part of authentication. The load extension is identified by OID 1.3.6.1.4.1.294.1.35. This is a new extension defined by TIFS/hsmRT.
\code
(TIFS/HSMRT)-LOAD := SEQUENCE
{
     destAddr OCTET STRING, -- Address to which the image accompanying this certificate
                            -- needs to be copied.
     auth_in_place INTEGER -- Controls if/how the authenticated binary is copied to a different
                           -- location. See below for more information.
}
The load extension is decoded into the below structure.
\endcode
\code
struct ti_load_info {
 u64 destAddr;
 u8  auth_in_place;
};
\endcode
Populating the certificate fields
-   destAddr

    - This is a 64 bit field indicating the address to which the data needs to be copied. The representation is big endian.
- auth_in_place

    - This is an integer field used to indicate whether the binary should be copied to the specified load address during authentication. The valid values for this field and their interpretation are described below.

Value    |Action
---------|------
0        |Normal operation. Binary is copied to load address.
1        |In place operation. Binary is not moved.
2        |In place operation variant. Binary is moved to the beginning of the buffer i.e. binary now starts at the location where the certificate started
Any other value	|invalid operation

### TIFS/hsmRT Debug Extension
\note This extension is not yet finalized.

The custom extension field used for debug control is identified by the OID 1.3.6.1.4.1.294.1.8. The structure of the field is shown below in ASN.1 notation.
\code
UID-Debug ::= SEQUENCE
{
    uid OCTET STRING, -- unique ID of the device for which this certificate applies
    debugCtrl INTEGER, -- debug control information
    coreDbgEn INTEGER, -- Core IDs for which debug must be enabled
    coreDbgSecEn INTEGER, -- Core IDs for which secure debug must be enabled
}
\endcode
The debug control data is decoded as a structure below:
\code
struct sdbg_debug_ctrl {
    u16 debug_priv_level;
    u16 reserved;
    u8 debug_core_sel[MAX_CPU_CORES];
    u8 sec_debug_core_sel[MAX_CPU_CORES];
}
\endcode
The table below shows the way UID-Debug fields are decoded into struct sdbg_debug_ctrl.

sdbg_debug_ctrl member  |X.509 certificate debug extension field
------------------------|---------------------------------------
debug_priv_level        |debugCtrl field is decoded as a u32 value and the lower 16 bit value is picked up
reserved                |debugCtrl field is decoded as a u32 value and the upper 16 bit value is picked up
debug_core_sel          |coreDbgEn field is decoded as an array of u8 values of processor IDs for which non-secure debug should be enabled
sec_debug_core_sel      |coreDbgSecEn field is decoded as an array of u8 values of processor IDs for which secure debug should be enabled
The following table shows the enumeration of values for debug_priv_level:

Enumeration name     |Value   |Meaning
---------------------|--------|-------
DEBUG_DISABLE        |0       |Disable debug
DEBUG_PRESERVE       |1       |Preserve current setting by locking registers
DEBUG_PUBLIC         |2       |Enable debug at public (non-secure) user and privileged level
DEBUG_PUBLIC_USER    |3       |Enable debug at public (non-secure) user level only
DEBUG_FULL           |4       |Enable full debug (both secure and non-secure privileged and user levels)
DEBUG_SECURE_USER    |5       |Enable debug for both secure and non-secure at user level only
For an example, please refer to the section [ debug ] in Sample x509 template.


### TIFS/hsmRT Encryption Extension
TIFS/hsmRT reuses the Encryption extension/OID as defined by ROM as no additional information is required. This is identified by OID 1.3.6.1.4.1.294.1.4.
\code
(TIFS/HSMRT)-ENCRYPT := SEQUENCE
{
   initalVector OCTET STRING,
   randomString OCTET STRING,
   iterationCnt INTEGER,
   salt OCTET STRING
}
\endcode
The encryption extension is decoded into the following data structure.
\code
struct ti_enc_info {
 u8 initialVector[16];
 u8 randomString[32];
 u8 iterationCnt;
 u8 salt[32];
};
\endcode
Populating the certificate fields
- initialVector
    - This is the 16 byte initialization vector to be used in AES-CBC decryption.
- randomString
    - This field indicates the random 32 byte string that was appended to the binary before encrypting the combined binary. TIFS/hsmRT will compare the last 32 bytes of the decryption output against the randomString field from the X509 certificate to verify the success of decryption operation.
- iterationCnt
    - This field is reserved and must be initialized to zero.
- salt
    - This field is 32 bytes long. It is reserved and must be initialized to zero.
\note TIFS/hsmRT always loads the binary to the location specified by TIFS/hsmRT Load Extension before performing in-place decryption at the loaded location.

### TIFS/hsmRT HS Board Configuration Extension

TIFS/hsmRT extends the Encryption extension/OID as defined by ROM for the purpose of authenticating the board configurations on HS devices. The hashes of the 4 board configuration blobs

- Core board configuration
- PM board configurations
- RM board configuration
- Encrypted Security board configuration
and the encryption parameters of the security board configuration are encoded in the board configuration extension of the TIFS/hsmRT outer certificate . This extension is identified by OID 1.3.6.1.4.1.294.1.36.
\code
(TIFS/HSMRT)-HS-BCFG:= SEQUENCE
{
   initalVector OCTET STRING,
   randomString OCTET STRING,
   iterationCnt INTEGER,
   salt OCTET STRING,
   secBoardCfgHash OCTET STRING,
   secBoardCfgVer INTEGER,
   pmBoardCfgHash OCTET STRING,
   rmBoardCfgHash OCTET STRING
   boardCfgHash OCTET STRING
}
\endcode
The extension is decoded into the following data structure.
\code
struct ti_bcfg_info {
 u8 initialVector[16];
 u8 randomString[32];
 u8 iterationCnt;
 u8 salt[32];
 u8 secBoardCfgHash[64];
 u8 secBoardCfgVer;
 u8 pmBoardCfgHash[64];
 u8 rmBoardCfgHash[64];
 u8 boardCfgHash[64];
};
\endcode
Populating the certificate fields
initialVector
This is the 16 byte initialization vector to be used in AES-CBC decryption.
randomString
This field indicates the random 32 byte string that was appended to the board configuration binary before encrypting the combined binary. TIFS/hsmRT will compare the last 32 bytes of the decryption output against the randomString field from the X509 certificate to verify the success of decryption operation.
- iterationCnt
    - This field is reserved and must be initialized to zero.
- salt
    - This field is 32 bytes long. It is reserved and must be initialized to zero.
- secBoardCfgHash
    - This is a 64 byte field containing the SHA2-512 hash of the encrypted security board configuration.
- secBoardCfgVer
    - This field indicates the version of security board configuration and must be initialized to zero.
- pmBoardCfgHash
    - This is a 64 byte field containing the SHA2-512 hash of the PM board configuration.
- rmBoardCfgHash
    - This is a 64 byte field containing the SHA2-512 hash of the RM board configuration.
- boardCfgHash
    - This is a 64 byte field containing the SHA2-512 hash of the main board configuration structure.


## Key writer extensions
The following X509 extensions are supported by TIFS/hsmRT.
Extension Name                                  |Purpose                                                 |OID
------------------------------------------------|--------------------------------------------------------|--------------------
Keywriter: Encrypted AES extension              |Keywriter Encrypted AES extension                       |1.3.6.1.4.1.294.1.64
Keywriter: Encrypted SMPK Signed AES extension  |Keywriter Encrypted SMPK Signed AES extension           |1.3.6.1.4.1.294.1.65
Keywriter: Encrypted BMPK Signed AES extension  |Keywriter Encrypted BMPK Signed AES extension           |1.3.6.1.4.1.294.1.66
Keywriter: AES Encrypted SMPKH                  |Keywriter AES Encrypted SMPKH                           |1.3.6.1.4.1.294.1.67
Keywriter: AES Encrypted SMEK                   |Keywriter AES Encrypted SMEK                            |1.3.6.1.4.1.294.1.68
Keywriter: MPK Options                          |Keywriter MPK Options                                   |1.3.6.1.4.1.294.1.69
Keywriter: AES Encrypted BMPKH                  |Keywriter AES Encrypted BMPKH                           |1.3.6.1.4.1.294.1.70
Keywriter: AES Encrypted BMEK                   |Keywriter AES Encrypted BMEK                            |1.3.6.1.4.1.294.1.71
Keywriter: MEK Options                          |Keywriter MEK Options                                   |1.3.6.1.4.1.294.1.72
Keywriter: AES Encrypted extended OTP           |Keywriter AES Encrypted extended OTP                    |1.3.6.1.4.1.294.1.73
Keywriter: key revision                         |Keywriter key revision                                  |1.3.6.1.4.1.294.1.74
Keywriter: MSV                                  |Keywriter MSV                                           |1.3.6.1.4.1.294.1.76
Keywriter: key count                            |Keywriter key count                                     |1.3.6.1.4.1.294.1.77
Keywriter: software revision (TIFS/HSMRT)       |Keywriter software revision (TIFS/HSMRT)                |1.3.6.1.4.1.294.1.78
Keywriter: software revision SBL                |Keywriter software revision SBL                         |1.3.6.1.4.1.294.1.79
Keywriter: software revision sec boardconfig    |Keywriter software revision SEC BCFG                    |1.3.6.1.4.1.294.1.80
Keywriter: version                              |Keywriter version                                       |1.3.6.1.4.1.294.1.81

### Encrypted AES extension
This extension contains information about the TIFEK(public) encrypted AES-256 key (random key, chosen by customer for keywriter). It is identified by the OID 1.3.6.1.4.1.294.1.64. The structure of the fields is shown below in ASN.1 notation.
\code
KEYWR-ENC-AES := SEQUENCE
{
   val OCTET STRING,     -- TIFEK(pub) encrypted AES-256 key chosen by the user.
   size INTEGER          -- size
}
\endcode
\code
struct keywr_enc_aes {
   u8 val[512];
   u32 size;
};
\endcode
### Encrypted SMPK Signed AES extension
This extension contains information about the TIFEK(public) encrypted, SMPK(priv) signed AES-256 key (random key, chosen by customer for keywriter). It is identified by the OID 1.3.6.1.4.1.294.1.65. The structure of the fields is shown below in ASN.1 notation.
\code
KEYWR-ENC-SMPK-SIGN-AES := SEQUENCE
{
   val OCTET STRING,     -- TIFEK(pub) encrypted, SMPK(priv) signed AES-256 key chosen by the user.
   size INTEGER          -- size
}
\endcode
\code
struct keywr_enc_smpk_sign_aes {
   u8 val[512];
   u32 size;
};
\endcode
### Encrypted BMPK Signed AES extension
This extension contains information about the TIFEK(public) encrypted, BMPK(priv) signed AES-256 key (random key, chosen by customer for keywriter). It is identified by the OID 1.3.6.1.4.1.294.1.66. The structure of the fields is shown below in ASN.1 notation.
\code
KEYWR-ENC-BMPK-SIGN-AES := SEQUENCE
{
   val OCTET STRING,     -- TIFEK(pub) encrypted, BMPK(priv) signed AES-256 key chosen by the user.
   size INTEGER          -- size
}
\endcode
\code
struct keywr_enc_bmpk_sign_aes {
   u8 val[512];
   u32 size;
};
\endcode
### AES Encrypted SMPKH
This extension contains information about the AES-256 key encrypted SMPKH (SHA-512 hashed SMPK Public key). It is identified by the OID 1.3.6.1.4.1.294.1.67 The structure of the fields is shown below in ASN.1 notation.
\code
KEYWR-AES-ENC-SMPKH := SEQUENCE
{
   val OCTET STRING,     -- AES-256 key encrypted SMPKH (SHA-512 used for hashing)
   iv  OCTET STRING,     -- Intitial Value used in AES-256-CBC encryption, 128 bits
   rs  OCTET STRING,     -- Random String used in AES-256-CBC encryption, 256 bits
   size INTEGER,         -- size
   action_flags INTEGER  -- WP | RP | OVRD | ACTIVE
}
\endcode
\code
struct keywr_aes_enc_smpkh {
   u8 val[64];
   u8 iv[16];
   u8 rs[32];
   u32 size;
   u32 action_flags;
};
\endcode
### AES Encrypted SMEK
This extension contains information about the AES-256 key encrypted SMEK. It is identified by the OID 1.3.6.1.4.1.294.1.68 The structure of the fields is shown below in ASN.1 notation.
\code
KEYWR-AES-ENC-SMEK := SEQUENCE
{
   val OCTET STRING,     -- AES-256 key encrypted SMEK
   iv  OCTET STRING,     -- Intitial Value used in AES-256-CBC encryption, 128 bits
   rs  OCTET STRING,     -- Random String used in AES-256-CBC encryption, 256 bits
   size INTEGER,         -- size
   action_flags INTEGER  -- WP | RP | OVRD | ACTIVE
}
\endcode
\code
struct keywr_aes_enc_smek {
   u8 val[64];
   u8 iv[16];
   u8 rs[32];
   u32 size;
   u32 action_flags;
};
\endcode
### MPK Options
\note This extension is currently not supported, and reserved for future.

This extension contains information about the MPK Options It is identified by the OID 1.3.6.1.4.1.294.1.69 The structure of the fields is shown below in ASN.1 notation.
\code
KEYWR-MPK-OPT := SEQUENCE
{
   val OCTET STRING,     -- MPK Options
   action_flags INTEGER  -- WP | RP | OVRD | ACTIVE
}
\endcode
\code
struct keywr_mpk_opt {
   u8 val[64];
   u32 action_flags;
};
\endcode
### AES Encrypted BMPKH
This extension contains information about AES-256 key encrypted BMPKH (SHA-512 hashed BMPK Public key). It is identified by the OID 1.3.6.1.4.1.294.1.70 The structure of the fields is shown below in ASN.1 notation.
\code
KEYWR-AES-ENC-BMPKH := SEQUENCE
{
   val OCTET STRING,     -- AES-256 key encrypted BMPKH (SHA-512 used for hashing)
   iv  OCTET STRING,     -- Intitial Value used in AES-256-CBC encryption, 128 bits
   rs  OCTET STRING,     -- Random String used in AES-256-CBC encryption, 256 bits
   size INTEGER,         -- size
   action_flags INTEGER  -- WP | RP | OVRD | ACTIVE
}
\endcode
\code
struct keywr_aes_enc_bmpkh {
   u8 val[64];
   u8 iv[16];
   u8 rs[32];
   u32 size;
   u32 action_flags;
};
\endcode
### AES Encrypted BMEK
This extension contains information about the AES-256 key encrypted BMEK. It is identified by the OID 1.3.6.1.4.1.294.1.71 The structure of the fields is shown below in ASN.1 notation.
\code
KEYWR-AES-ENC-BMEK := SEQUENCE
{
   val OCTET STRING,     -- AES-256 key encrypted BMEK
   iv  OCTET STRING,     -- Intitial Value used in AES-256-CBC encryption, 128 bits
   rs  OCTET STRING,     -- Random String used in AES-256-CBC encryption, 256 bits
   size INTEGER,         -- size
   action_flags INTEGER  -- WP | RP | OVRD | ACTIVE
}
\endcode
\code
struct keywr_aes_enc_bmek {
   u8 val[64];
   u8 iv[16];
   u8 rs[32];
   u32 size;
   u32 action_flags;
};
\endcode
### MEK Options
\note This extension is currently not supported, and reserved for future.

This extension contains information about the MEK Options, it is identified by the OID 1.3.6.1.4.1.294.1.72 The structure of the fields is shown below in ASN.1 notation.
\code
KEYWR-MEK-OPT := SEQUENCE
{
   val OCTET STRING,     -- MEK Options
   action_flags INTEGER  -- WP | RP | OVRD | ACTIVE
}
\endcode
\code
struct keywr_mek_opt {
   u8 val[64];
   u32 action_flags;
};
\endcode
### AES Encrypted extended OTP
\note This extension is currently not supported, and reserved for future.

This extension contains information about the AES-256 key encrypted extended OTP values. It is identified by the OID 1.3.6.1.4.1.294.1.73 The structure of the fields is shown below in ASN.1 notation.
\code
KEYWR-AES-ENC-EXT-OTP := SEQUENCE
{
   val OCTET STRING,     -- Extended OTP in octet string format
   iv  OCTET STRING,     -- Intitial Value used in AES-256-CBC encryption, 128 bits
   rs  OCTET STRING,     -- Random String used in AES-256-CBC encryption, 256 bits
   wprp OCTET STRING,    -- Write Protect(64 bits) | Read Protect(64 bits) each bit
                            represents each efuse row.
   index INTEGER,        -- Starting index to write ext otp value
   size INTEGER,         -- size of ext_otp value
   action_flags INTEGER  -- ~ | ~ | ~ | ACTIVE
}
\endcode
\code
struct keywr_aes_enc_ext_otp {
   u8 val[64];
   u8 iv[16];
   u8 rs[32];
   u8 wprp[16];
   u32 index;
   u32 size;
   u32 action_flags;
};
\endcode
### key revision
This extension contains information about the Keyrevision field. By default, keyrevision should be set to 1. If it is set to 2, BMPK and BMEK will be used instead of SMPK and SMEK. It is identified by the OID 1.3.6.1.4.1.294.1.74. The structure of the fields is shown below in ASN.1 notation.
\code
KEYWR-KEYREV := SEQUENCE
{
   val OCTET STRING,     -- Keyrev
   action_flags INTEGER  -- WP | RP | OVRD | ACTIVE
}
\endcode
\code
struct keywr_keyrev {
   u8 val[64];
   u32 action_flags;
};
\endcode
### MSV
This extension contains information about the MSV. It is identified by the OID 1.3.6.1.4.1.294.1.76 The structure of the fields is shown below in ASN.1 notation.
\code
KEYWR-MSV := SEQUENCE
{
   val OCTET STRING,     -- MSV
   action_flags INTEGER  -- WP | RP | OVRD | ACTIVE
}
\endcode
\code
struct keywr_msv {
   u8 val[64];
   u32 action_flags;
};
\endcode
### key count
This extension contains information about the Key count field. It is identified by the OID 1.3.6.1.4.1.294.1.77. The structure of the fields is shown below in ASN.1 notation.
\code
KEYWR-KEYCNT := SEQUENCE
{
   val OCTET STRING,     -- Key count
   action_flags INTEGER  -- WP | RP | OVRD | ACTIVE
}
\endcode
\code
struct keywr_keycnt {
   u8 val[64];
   u32 action_flags;
};
\endcode
### software revision (TIFS/HSMRT)
This extension contains information about the Software revision value for (TIFS/HSMRT). It is identified by the OID 1.3.6.1.4.1.294.1.78 The structure of the fields is shown below in ASN.1 notation.
\code
KEYWR-SWREV-(TIFS/HSMRT) := SEQUENCE
{
   val OCTET STRING,     -- SWREV (TIFS/HSMRT)
   action_flags INTEGER  -- WP | RP | OVRD | ACTIVE
}
\endcode
\code
struct keywr_swrev_(TIFS/hsmRT) {
   u8 val[64];
   u32 action_flags;
};
\endcode
### software revision SBL
This extension contains information about the Software revision value for SBL. It is identified by the OID 1.3.6.1.4.1.294.1.79 The structure of the fields is shown below in ASN.1 notation.
\code
KEYWR-SWREV-SBL := SEQUENCE
{
   val OCTET STRING,     -- SWREV SBL
   action_flags INTEGER  -- WP | RP | OVRD | ACTIVE
}
\endcode
\code
struct keywr_swrev_sbl {
   u8 val[64];
   u32 action_flags;
};
\endcode
### software revision sec boardconfig
This extension contains information about the Software revision value for the Secure Boardconfig. It is identified by the OID 1.3.6.1.4.1.294.1.80 The structure of the fields is shown below in ASN.1 notation.
\code
KEYWR-SWREV-SEC-BCFG := SEQUENCE
{
   val OCTET STRING,     -- SWREV SEC BCFG
   action_flags INTEGER  -- WP | RP | OVRD | ACTIVE
}
\endcode
\code
struct keywr_swrev_sec_bcfg {
   u8 val[64];
   u32 action_flags;
};
\endcode
### version
This extension contains information about the Keywriter version. It is identified by the OID 1.3.6.1.4.1.294.1.81 The structure of the fields is shown below in ASN.1 notation.
\code
KEYWR-VERSION := SEQUENCE
{
   val OCTET STRING,     -- SWREV SEC BCFG
}
\endcode
\code
struct keywr_version {
   u8 val[64];
};
\endcode