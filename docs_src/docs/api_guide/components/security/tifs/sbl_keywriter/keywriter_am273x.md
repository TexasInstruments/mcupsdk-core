# AM273X Keywriter {#AM273x_KEY_WRITER}

[TOC]

## Introduction
\note This document is applicable to High Secure (HS) device variants.

### High Secure (HS) Device variants
- <b>HS-FS (Field Securable)</b>: This is the state in which device leaves TI factory where customer keys are not programmed. In this state, device protects the ROM code, TI keys and certain security peripherals. HS-FS devices do not enforce secure boot process.
    - M4 JTAG port is closed, R5 JTAG port is open
    - Security Subsystem Firewalls are closed, SOC Firewalls are open    -
    - ROM Boot expects a TI signed binary (encryption is optional)
    - TIFS/hsmRT binary is signed by the TI Private key (TI MPK). (Refer Signing an unencrypted binary for secure boot for more details)

- <b>HS-SE (Security Enforced)</b>: This is the state of the device after customer keys are programmed. HS-SE devices enforces secure boot.
    - M4, R5 JTAG ports are both closed
    - Security Subsystem, SOC Firewalls are both closed
    - TIFS/hsmRT and SBL needs to be signed with active customer private key (SMPK/BMPK)

### OTP Keywriter : Converts HS-FS ->  HS-SE

OTP (One Time Programmable) keywriter is the software tool to provision customer keys into the device eFuses to enforce secure boot and establish root of trust. Secure boot requires an image to be encrypted (optional) and signed using customer keys, which will be verified by the SoC using the active MEK (for decryption) and MPK Hash (to verify the signature). In case the customer’s MPK and MEK are compromised, there is a second set of backup MPK and MEK fields that can be used, after applying a key revision update.

In order to convert a HS-FS device to HS-SE device, one has to program the customer root key set (optionally backup key set) on the target device using OTP Keywriter.
\imageStyle{hsfs_to_hsse_conversion_am273x.png,width:100%}
\image html hsfs_to_hsse_conversion_am273x.png "hsfs to hsse conversion"

\attention <b>This action is irreversible, so there is no going back once the keys are burned into SoC eFuses</b>.
### Supported OTP keys
Key                             |	Description                                 |	Notes
--------------------------------|-----------------------------------------------|------------------------------------------------------
BMEK	                        |Backup Manufacturer Encryption Key	            |256-bit Customer encryption key for encrypted boot
BMPKH	                        |Backup Manufacturer Public Key Hash	        |BMPK is 4096-bit customer RSA signing key
EXTENDED OTP	                |Extended OTP array	                            |1664 bit extended otp array
KEYCNT	                        |Key count	                                    |2 if BMPK, SMPK are used, 1 if SMPK is used, 0 if none
KEYREV	                        |Key revision	                                |Can have a maximum value = key count
MEK Options	                    |SMEK/BMEK options	                            |5 bit value
MPK Options	                    |SMPK/BMPK options	                            |10 bit value (split into 2 parts)
MSV	                            |Model specific value	                        |24 bit value with 8 bit BCH code
SMEK	                        |Secondary Manufacturer Encryption Key          |256-bit Customer encryption key for encrypted boot
SMPKH	                        |Secondary Manufacturer Public Key Hash         |SMPK is 4096-bit customer RSA signing key
SWREV-APP	                    |Application software revision                  |192 bit value (96 bit with redundancy)
SWREV-SBL	                    |SBL software revision                          |64  bit value (32 bit with redundancy)
SWREV-HSM	                    |Firmware software revision                     |64  bit value (32 bit with redundancy)

\note <b>The current version of OTP Key writer only supports 4096 bit RSA format for SMPK and BMPK. Support for ECDSA and different key lengths will be available in subsequent versions</b>.
## MPK/MEK Options
These are currently not supported, and reserved for future use.

MPK (SMPKH/BMPKH) options are of 10-bit value, split into two 5-bit values, to be programmed contiguous to MPK part 1 and part 2. MEK options are of 5-bit value, programmed contiguous to MEK.

Secure ROM uses MPK options bit index 0 for RMA process. On a HS-SE device, if this bit is set, tiMpk will be used as root of trust instead of SMPKH/BMPKH.

### MSV
Model Specific Value is a 24 bit value.

### BCH code
BCH code is used for error correction.

BCH algorithm (288, 261, 7) is used for SMPKH(BMPKH), SMEK(BMEK).

Out of 261 bits, 256 correspond to SMPKH(BMPKH) part1/2 or SMEK(BMEK) and the remaining 5 bits are the MPK/MEK options. If no options are specified, they are set to 0.

BCH algorithm (32, 20, 5) is used for MSV (24 bit value, and 8 bit BCH code).

BCH codes are computed at time of fuse programming by OTP Keywriter firmware based on plaintext key values.

### SWREV
eFuse values of SWREV-SBL, SWREV-HSM, SWREV-APP, use redundancy to reduce the chance of data corruption.

### Extended OTP
Total 1664 bits are supported i.e 52 words

Extended otp data is written in big-endian format

No of rows to be programmed will be based on indx and size.

For example indx-16 size-32 will lead to two rows

If they were to be write protected, we have to write protect both row 0 and row 1.

Please note that this would cause the remaining bits of row 0 and row 1 also to be write protected.

## Components
OTP Keywriter contents include:

- <b>SDK keywriter app source code</b>

    This is the public keywriter application. It boots directly on the primary boot core of the device i.e R5. It is responsible for sending the OTP configuration to the secure keywriter component running on the security subsystem to blow the keys into the device eFuses.

    - x509 certificate configuration template, and generation script is added as part of source code
    \note This is a reference script. It can either be used as-is with the customer’s keys or integrated within the customer’s Security Subsystem environment.

- <b>Encrypted TIFS/hsmRT keywriter binary</b>

    This is the secure OTP keywriter firmware, and runs on the secure subsystem of the device . It is responsible for verifying the OTP configuration from x509 certificate and blowing the data into the device efuses.
    \note This binary differs from the standard TIFS/hsmRT binary used in production.

- <b>TI FEK (public) key - TI Field Encryption Key</b>

    This is needed by the certificate generation script, to ensure confidentiality of the customer’s key material. TI FEK Public key will be used to encrypt content (refer \ref subpage1), and TI FEK private key (not shared, but accessible by encrypted TIFS/hsmRT keywriter binary) will be used to decrypt it (inside the secure subsystem).
## Important Features & Details
- VPP must be powered (1.7v) when the efuses are programmed. Refer to device data sheet for details on VPP requirements.
\note <b>EVM Setting : Connect jumper on the P1 connector</b>

- OTP keywriter firmware supports a maximum certificate length of 8192 bytes.
\note This length is for individual certificates, and not the x509cert/final_certificate.bin. Check size of primary_cert.bin and seconday_cert.bin (optional depending on BMPK). If certificate length exceeds the supported limit, the keys can be programmed using incremental approach.

- Incremental programming - Until the KEYREV value is set to either 1 or 2, the device is considered as HS-FS and key values can continue to be programmed incrementally. This allows key programming to be done in multiple passes. Refer this for detailed examples.
\note Different offsets of extended OTP array could be programmed, followed by other keys such as MSV, SWREV, MPK/MEK Options, etc.

\attention Once the KEYREV is set to 1 or 2 and SWREV_SBL to respective value, the device becomes HS-SE, and OTP keywriter application will no longer boot, as the user root key takes over as root of trust. <b>programming KEYREV and SWREV_SBL should be done together at the end</b>.
## Build Procedure for keywriter
    - Setup build environment
    - Generate x509 certificate from customer Security Subsystem
    - Build SDK Keywriter app
### Setup build environment
Follow the below steps to make environment ready for keywriter building
\note In case of windows environment, use linux bash(like git bash) terminal for below procedure

- Setup mcu_plus_sdk environment(If setup is not available)
- $cd mcu_plus_sdk/source/security
- $mkdir tifs
- $cd tifs
- $mkdir sbl_keywriter
- $cd sbl_keywriter
- copy <b>am273x-evm.zip</b> package recieved from mysecure
- unzip am273x-evm.zip
- $cd am273x-evm
- $gmake env.sh (Note: In case of linux 'make')
- $source env.sh
\code
Directory structure:
mcu_plus_sdk/source/security/tifs/sbl_keywriter/am273x-evm/mcuplus_boot_test/ti/keywriter/
├── scripts
|      └── gen_custCert_Header
|                 ├── scripts
|                 |      ├── gen_keywr_cert.sh            /* x509 certificate generation script */
|                 |      ├── TPR12_HSM_RSA_FEK_pub.pem
|                 |      ├── construct_ext_otp_data.sh    /* extended otp binary generation script */
|                 |      └── keys
|                 |            ├── aes256.pem
|                 |            ├── smpk.pem
|                 |            ├── smek.key
|                 |            ├── bmpk.pem
|                 |            └── bmek.key
|                 └── x509cert
|                         ├── bin2c.exe
|                         ├── bin2c.out
|                         └── genKW_cust_cert.sh
└── tpr12
       sbl
        ├── include
        |      ├── KW_Cust_Certificate.h
        |      ├── KW_(TIFS/HsmRt)_Image.h
        |      └── ipc_export.h
        ├── src
        |    ├── main.c
        |    ├── sbl_interrupts.c
        |    ├── sbl_interrupt_vector.sr5
        |    └── sbl_startup.sr5
        ├── keywriter.sh
        ├── linker
        ├── makefile
        ├── sbl_kw.mak
        ├── sbl_kw_hex.cmd
        └── sign_sbl_kw.sh
\endcode
### Generate x509 certificate from customer Security Subsystem
The Customer keys are supposed to be private, and not to be distributed out in open. For the scope of this document, Customer Security Subsystem would mean The secure server/ system where the customer intends to generate the x509 certificate. Once the certificate is generated, since the customer key information is encrypted, it is safe to share the certificate in open without revealing the actual keys.

The customer provided key material consists of both Public Key hashes (xMPKH) and secret/secure symmetric keys (xMEK). In order to maintain security, the symmetric keys are encrypted on the secure server (referred to here as Customer Security Subsystem), so that the resulting X509 certificate can be exported and embedded in the Keywriter binary without exposing the secret keys.

\note OpenSSL (1.1.1 11 Sep 2018) is required for building the OTP Keywriter. Check if OpenSSL is installed by typing “openssl version” at the comamnd prompt. If OpenSSL is not installed, download and install OpenSSL for your OS.

    For Windows : The easiest way is to download and install Strawberry Perl The Strawberry Perl installer automatically installs and sets up OpenSSL. Alternately, users can also use any of these Third Party OpenSSL Distributions for Windows. Refer individual links for instructions on how to setup OpensSSL using a particular distribution.
    For Linux : Execute the command sudo apt-get install openssl at the linux command prompt.

\section subpage1 Certificate generation procedure

\imageStyle{certificate_generation_procedure.png,width:100%}
\image html certificate_generation_procedure.png "Certificate generation procedure"

<b>Details of the Customer certificate  generation process</b>

<b>step 1</b> OEM generates a random 256-bit number to be used as an AES encryption key for protecting the OTP data.

<b>step 2</b> The AES-256 key from step 1 is used to encrypt all X509 extension fields, which require encryption protection.

<b>step 3</b> Following X509 extensions are created, using TI FEK (public):
            - Encrypting the AES-256 key with TI FEK
            - Signing the AES-256 key with the SMPK, and encrypting that with the TI FEK
            - (optionally, refer step 6) signing the AES-256 key with the BMPK, and encrypting that with the TI FEK

<b>step 4</b> All of the extensions from steps 1-3 are combined into a X.509 configuration which is used to generate and sign a certificate with the SMPK.

<b>step 5</b> This X509 config is sigend using SMPK (priv).

<b>step 6</b> (Optional) If the OEM chooses to write BMPK/BMEK fields, X509 config from step 5 needs to be signed using BMPK (priv).

The scripts folder has the necessary tools for generating x509 certificate for eFuse programming. It can be copied to a secure server (customer Security Subsystem) where the customer keys are stored, and used to generate the x509 certificate.

Run the gen_keywr_cert.sh shell script to create the certificate. Modify the path arguments to specific keys as necessary. Use -h option for help.
\code
./gen_keywr_cert.sh -h
\endcode
\note ./gen_keywr_cert.sh -g will generate dummy keys for testing in keys/ folder (TI dummy keys are provided default)
      gen_keywr_cert.sh will also generate SHA-512 hash of SMPK, SMEK, BMPK, and BMEK and store them in verify_hash.csv. UART logs will also print out SHA-512 hash of the keys.

- <b>How to generate Customer certificate</b>

  - <b>STEP 1</b>: To generate ext_otp_data.bin
    \code
    ./construct_ext_otp_data.sh -extotp 0x12345678 -indx 0 -size 32
    \endcode

    STEP1  is applicable only if user needs to program ext_otp

    Options

        -extotp   - Value to be written (writes input data in big-endian)

        -indx     - start bit position (should be multiple of 8-bits, Byte oriented)

        -size     - no of bits to be programmed (should be multiple of 8-bits)


  - <b>STEP 2</b>: To generate final_certificate.bin
    \code
    ./gen_keywr_cert.sh -s keys/smpk.pem --smek keys/smek.key -b keys/bmpk.pem --bmek keys/bmek.key -t TPR12_HSM_RSA_FEK_pub.pem -a keys/aes256.key --msv 0xF1E22D --ext-otp ext_otp_data.bin --ext-otp-indx 0 --ext-otp-size 32 --sr-sbl 1 --sr-hsmRT 1 --sr-app 1 --keycnt 2 --keyrev 1
    \endcode
    Above command is just a reference , Customer can generate certificate with the necessary key options before programming KEYREV.

    Certificate will be available in x509cert/final_certificate.bin in customer Security Subsystem .

  - <b>STEP 3</b>: To generate KW_Cust_Certificate.h
    \code
    ./genKW_cust_cert.sh
    \endcode
### Build SDK Keywriter app

\note In case of windows environment, use any linux bash terminal(like git bash) for below procedure

The SDK Keywriter app will load OTP Keywriter binary, and call OTP keywriter API using IPC message service
The response from TIFS/hsmRT gives information about success/failure in customer key programming in SoC eFuses.

- To build sbl keywriter app

<b>$cd `<MCU_PLUS_SDK_PATH>`/source/security/tifs/sbl_keywriter/am273x-evm/mcuplus_boot_test/ti/keywriter/tpr12/sbl</b>

- For Keywriter

    - <b>$source keywriter.sh sblKW</b>

    - Generated binary <b>sbl_kw_tpr12_cert.bin</b> is available at the below path

        <b>$cd mcu_plus_sdk/source/security/tifs/sbl_keywriter/am273x-evm/mcuplus_boot_test/ti/keywriter/tpr12/sbl/binaries</b>

### Running on SoC, using a boot mode of choice

\note For below procedure use bash terminal or cmd

Otp keywriter tool is supported with the following boot options

#### UART BOOT (SOP5)

- EVM settings : \ref BOOTMODE_UART
- Open appropriate COM Port
- Set baud rate : 115200bps
- Select Sbl key writer bin file and send via tera term (XMODEM Protocal)

#### QSPI BOOT (SOP4)

<b>Involves following sequence</b>

- Flashing Keywriter app on Sflash using uniflash utlity
- Keywriter in QSPI Boot mode

- <b>Flashing Keywriter app on Sflash using uniflash utlity</b>

    - <b>STEP 1</b>
        - EVM Setup: \ref BOOTMODE_UART

        - cd mcu_plus_sdk/tools/boot/sbl_prebuilt/am273x-evm

        - cp mcu_plus_sdk/source/security/tifs/sbl_keywriter/am273x-evm/mcuplus_boot_test/ti/keywriter/tpr12/sbl/binaries/sbl_kw_tpr12_cert.bin .

    - <b>STEP 2</b>
        - Open default_sbl_null.cfg
        - Replace <b>sbl_null.release.tiimage</b> file with <b>sbl_kw_tpr12_cert.bin</b>

    - <b>STEP 3</b>
        - Run the following command by updatating the appropriate com port

        - cd ${SDK_INSTALL_PATH}/tools/boot

        - python uart_uniflash.py -p COM<x> --cfg=sbl_prebuilt/am273x-evm/default_sbl_null.cfg

        - <b> Above command will perform uniflash loading followed by keywriter loading on sflash, refer below image for logs while performing step 3</b>

\imageStyle{keywriter_flash_cmd_logs_am273x.png,width:50%}
\image html keywriter_flash_cmd_logs_am273x.png "Logs while Uniflash loading followed by Keywriter loading on sflash"

- <b>Keywriter in QSPI Boot mode</b>

    - EVM Setup \ref BOOTMODE_QSPI
    - reset the device using sw1
    - key programming status information will be displayed on the UART console

#### JTAG mode (TBD)

## Build Functional App
This is to validate the device after OTP writer programming is done i.e functional boot validation on HSSE device

### SBL
- <b>$cd MCU_PLUS_SDK_PATH/source/security/tifs/sbl_keywriter/am273x-evm/mcuplus_boot_test/ti/keywriter/tpr12/sbl</b>
- $source keywriter.sh sbl smpk if KEY_REV = 1
- $source keywriter.sh sbl bmpk if KEY_REV = 2

### HSMRT
- <b>$cd  MCU_PLUS_SDK_PATH/source/security/tifs/sbl_keywriter/am273x-evm/mcuplus_boot_test/ti/keywriter/scripts/gen_custkeyWriter_Header</b>
- $./gen_kw_hsmRt_header.sh FB_test smpk if KEY_REV = 1
- $./gen_kw_hsmRt_header.sh FB_test bmpk if KEY_REV = 2

## Debugging
Following  error codes are updated to UART console as part debug information to the user.
OTP Keywriter error code bit indices
Enum	                        |Value	    |Description
--------------------------------|-----------|------------
KEYWR_ERR_DECRYPT_AES256_KEY	|0	        |Error in Decrypting AES256 key randomly generated by customer
KEYWR_ERR_DECRYPT_BMEK	        |1	        |Error in Decrypting BMEK extension field
KEYWR_ERR_DECRYPT_BMPKH	        |2	        |Error in Decrypting BMPKH extension field
KEYWR_ERR_DECRYPT_SMEK	        |3	        |Error in Decrypting SMEK extension field
KEYWR_ERR_DECRYPT_SMPKH	        |4	        |Error in Decrypting SMPKH extension field
KEYWR_ERR_INTERAL_OP	        |5	        |Internal Operation Error
KEYWR_ERR_INVALID_EXT_COUNT	    |6	        |Invalid extension count in x509 certificate. Either SMPKH, SMEK pair or BMPKH, BMEK, SMPKH, SMEK should be used. Any other combination will trigger error
KEYWR_ERR_PARSE_CERT	        |7	        |Error in parsing certificate
KEYWR_ERR_PARSE_FEK	            |8	        |Error in parsing TI FEK (appended to TIFS binary, before encryption)
KEYWR_ERR_PARSE_SMPK_CERT	    |9	        |Error in parsing SMPK signed certificate (certificate that contains customer key data)
KEYWR_ERR_PROGR_BMEK	        |10	        |Error in programming BMEK into SoC eFuses
KEYWR_ERR_PROGR_BMPKH_PART_1	|11	        |Error in programming BMPKH part 1 into SoC eFuses
KEYWR_ERR_PROGR_BMPKH_PART_2	|12	        |Error in programming BMPKH part 2 into SoC eFuses
KEYWR_ERR_PROGR_KEYCOUNT	    |13	        |Error in programming KEY COUNT into SoC eFuses
KEYWR_ERR_PROGR_KEYREV	        |14	        |Error in programming KEY REV into SoC eFuses
KEYWR_ERR_PROGR_SMEK	        |15	        |Error in programming SMEK into SoC eFuses
KEYWR_ERR_PROGR_SMPKH_PART_1	|16	        |Error in programming SMPKH part 1 into SoC eFuses
KEYWR_ERR_PROGR_SMPKH_PART_2	|17	        |Error in programming SMPKH part 2 into SoC eFuses
KEYWR_ERR_VALIDATION_CERT	    |18	        |Error validating certificate
KEYWR_ERR_VALIDATION_SMPK_CERT	|19	        |Error validating SMPK signed certificate
KEYWR_ERR_VALIDATION_BMPK_KEY	|20	        |Error validating BMPK key
KEYWR_ERR_VALIDATION_SMPK_KEY	|21	        |Error validating SMPK key
KEYWR_ERR_WRITE_PROT_KEYCOUNT	|22	        |Error write protecting key count row
KEYWR_ERR_WRITE_PROT_KEYREV	    |23	        |Error write protecting key revision row
KEYWR_ERR_IMG_INTEG_SMPK_CERT	|24	        |SMPK signed certificate image integrity failed
KEYWR_ERR_PROGR_MSV	            |25	        |Error in programming MSV into SoC eFuses
KEYWR_ERR_PROGR_SWREV	        |26	        |Error in programming SWREV into SoC eFuses
KEYWR_ERR_PROGR_FW_CFG_REV	    |27	        |Error in programming FW CFG REV into SoC eFuses
KEYWR_ERR_DECRYPT_EXT_OTP	    |28	        |Error in Decrypting EXT OTP extension field
KEYWR_ERR_PROGR_EXT_OTP	        |29	        |Error in programming EXT OTP extension field

## Appendix
### Using TI dummy keys from SDK code base
\code
cd <MCU_PLUS_SDK_PATH>

# Copy the customer dummy private key (SMPK private key, PEM format)
cp cust_dev_mpk.pem /source/security/tifs/sbl_keywriter/am273x-evm/mcuplus_boot_test/ti/keywriter/gen_custCert_Header/scripts/keys/smpk.pem

# Copy the customer dummy encryption key (SMEK, converted to binary file)
xxd -p -r cust_dev_mek.txt > /source/security/tifs/sbl_keywriter/am273x-evm/mcuplus_boot_test/ti/keywriter/gen_custCert_Header/scripts/keys/smek.key
\endcode
### Creating x509 certificates for Incremental Programming
#### Pass 1
\code
OTP keywriter can program extended otp array, when supplied with an index, and size of bits to program (both index and size have to be multiples of 8).

Constructing extended otp array

# Step 1: Generate 1664 bit array ext_otp_data.bin
./construct_ext_otp_data.sh -extotp 0xFF -indx 0 -size 8

# Step 2: Edit ext_otp_data.txt for any more changes
vim ext_otp_data.txt

# Step 3: Convert txt to bin file
xxd -r -p ext_otp_data.txt > ext_otp_data.bin

# Setp 4: Generate x509 certificate
./gen_keywr_cert.sh -t ti_fek_public.pem --ext-otp ext_otp_data.bin --ext-otp-indx 0 --ext-otp-size 8
\endcode
\note Steps 2, 3 can be skipped if no changes are needed in extended OTP array

After creating this certificate, building the example keywriter app, and programming ext_otp[0:7], we could also program another offset of ext_otp

#### Pass 2
\code
./construct_ext_otp_data.sh -extotp 0x1234 -indx 8 -size 16
./gen_keywr_cert.sh -t ti_fek_public.pem --ext-otp ext_otp_data.bin --ext-otp-indx 8 --ext-otp-size 16 --ext-otp-wprp 00000000000000010000000000000000
\endcode
\note
The index and size arguments should be same in both the commands

This will create another certificate. Keywriter example app needs to be built after every change in certificate. This could be reapeated many times, but without any overlap in the ext_otp index, size parameters.

#### Pass 3
Since the KEYREV value is not modified until now, we can still use OTP keywriter to program other keys.

Following code snippet would set MSV, SWREV-SBL, SWREV-(TIFS/HSMRT), BMPKH, SMPKH, KEYCNT and KEYREV.

\note It is recommended to write protect SMPKH, SMEK, BMPKH, BMEK efuse rows by providing the specific arguments to gen_keywr_cert.sh script.

\code
./gen_keywr_cert.sh -s keys/smpk.pem -s-wp --mpk-opt 0x21
--smek keys/smek.key --mek-opt 0x1 --smek-wp -t ti_fek_public.pem
-a keys/aes256.key --msv 0xC0FFE --msv-wp --keycnt 1 --keyrev 1
--sr-sbl 3 --sr-(tifs/hsmRT) 4
\endcode
\note
    - KEYCNT should match the number of keys programmed into efuses.
    - KEYREV should always be <= KEYCNT
    - It is recommended to write protect KEYCNT
    - KEYREV could be left without write protection, if a run time KEYREV update is needed.


### X509 Configuration Template
The following x509 configuration template is used, by gen_keywr_cert.sh file, to create the x509 certificate. Each key has an extension field (OID 1.3.6.1.4.1.294.1.64 - 1.3.6.1.4.1.294.1.81), mentioning information such as SHA-512 value, size, IV, RS used in AES encryption.

Details about individual extensions can be obtained form Security x509 Certificate Documentation
\code
[ req ]
distinguished_name = req_distinguished_name
x509_extensions = v3_ca
prompt = no
dirstring_type = nobmp

# This information will be filled by the end user.
# The current data is only a place holder.
# System firmware does not make decisions based
# on the contents of this distinguished name block.
[ req_distinguished_name ]
C = oR
ST = rx
L = gQE843yQV0sag
O = dqhGYAQ2Y4gFfCq0t1yABCYxex9eAxt71f
OU = a87RB35W
CN = x0FSqGTPWbGpuiV
emailAddress = kFp5uGcgWXxcfxi@vsHs9C9qQWGrBs.com

[ v3_ca ]
basicConstraints = CA:true
1.3.6.1.4.1.294.1.64 = ASN1:SEQUENCE:enc_aes_key
1.3.6.1.4.1.294.1.65 = ASN1:SEQUENCE:enc_smpk_signed_aes_key
1.3.6.1.4.1.294.1.66 = ASN1:SEQUENCE:enc_bmpk_signed_aes_key
1.3.6.1.4.1.294.1.67 = ASN1:SEQUENCE:aesenc_smpkh
1.3.6.1.4.1.294.1.68 = ASN1:SEQUENCE:aesenc_smek
1.3.6.1.4.1.294.1.69 = ASN1:SEQUENCE:plain_mpk_options
1.3.6.1.4.1.294.1.70 = ASN1:SEQUENCE:aesenc_bmpkh
1.3.6.1.4.1.294.1.71 = ASN1:SEQUENCE:aesenc_bmek
1.3.6.1.4.1.294.1.72 = ASN1:SEQUENCE:plain_mek_options
1.3.6.1.4.1.294.1.73 = ASN1:SEQUENCE:aesenc_user_otp
1.3.6.1.4.1.294.1.74 = ASN1:SEQUENCE:plain_key_rev
1.3.6.1.4.1.294.1.76 = ASN1:SEQUENCE:plain_msv
1.3.6.1.4.1.294.1.77 = ASN1:SEQUENCE:plain_key_cnt
1.3.6.1.4.1.294.1.78 = ASN1:SEQUENCE:plain_swrev_(tifs/hsmRT)
1.3.6.1.4.1.294.1.79 = ASN1:SEQUENCE:plain_swrev_sbl
1.3.6.1.4.1.294.1.80 = ASN1:SEQUENCE:plain_swrev_sec_brdcfg
1.3.6.1.4.1.294.1.81 = ASN1:SEQUENCE:plain_keywr_min_version

[ enc_aes_key ]
# Replace PUT-ENC-AES-KEY with acutal Encrypted AES Key
val = FORMAT:HEX,OCT:PUT_ENC_AES_KEY
size = INTEGER:PUT_SIZE_ENC_AES

[ enc_bmpk_signed_aes_key ]
# Replace PUT-ENC-BMPK-SIGNED-AES-KEY with acutal Encrypted BMPK signed AES Key
val = FORMAT:HEX,OCT:PUT_ENC_BMPK_SIGNED_AES_KEY
size = INTEGER:PUT_SIZE_ENC_BMPK_SIGNED_AES

[ enc_smpk_signed_aes_key ]
# Replace PUT-ENC-SMPK-SIGNED-AES-KEY with acutal Encrypted SMPK signed AES Key
val = FORMAT:HEX,OCT:PUT_ENC_SMPK_SIGNED_AES_KEY
size = INTEGER:PUT_SIZE_ENC_SMPK_SIGNED_AES

[ aesenc_smpkh ]
# Replace PUT-AESENC-SPMKH with acutal Encrypted AES Key
val = FORMAT:HEX,OCT:PUT_AESENC_SMPKH
iv = FORMAT:HEX,OCT:PUT_IV_AESENC_SMPKH
rs = FORMAT:HEX,OCT:PUT_RS_AESENC_SMPKH
size = INTEGER:PUT_SIZE_AESENC_SMPKH
action_flags = INTEGER:PUT_ACTFLAG_AESENC_SMPKH

[ aesenc_smek ]
# Replace PUT-AESENC-SMEK with acutal Encrypted AES Key
val = FORMAT:HEX,OCT:PUT_AESENC_SMEK
iv = FORMAT:HEX,OCT:PUT_IV_AESENC_SMEK
rs = FORMAT:HEX,OCT:PUT_RS_AESENC_SMEK
size = INTEGER:PUT_SIZE_AESENC_SMEK
action_flags = INTEGER:PUT_ACTFLAG_AESENC_SMEK

[ aesenc_bmpkh ]
# Replace PUT-AESENC-BMPKH with acutal Encrypted BMPKH
val = FORMAT:HEX,OCT:PUT_AESENC_BMPKH
iv = FORMAT:HEX,OCT:PUT_IV_AESENC_BMPKH
rs = FORMAT:HEX,OCT:PUT_RS_AESENC_BMPKH
size = INTEGER:PUT_SIZE_AESENC_BMPKH
action_flags = INTEGER:PUT_ACTFLAG_AESENC_BMPKH

[ aesenc_bmek ]
# Replace PUT-AESENC-BMEK with acutal Encrypted BMEK
val = FORMAT:HEX,OCT:PUT_AESENC_BMEK
iv = FORMAT:HEX,OCT:PUT_IV_AESENC_BMEK
rs = FORMAT:HEX,OCT:PUT_RS_AESENC_BMEK
size = INTEGER:PUT_SIZE_AESENC_BMEK
action_flags = INTEGER:PUT_ACTFLAG_AESENC_BMEK

[ plain_msv ]
# Replace PUT-PLAIN-MSV with actual MSV value
val = FORMAT:HEX,OCT:PUT_PLAIN_MSV
action_flags = INTEGER:PUT_ACTFLAG_PLAIN_MSV

[ plain_mpk_options ]
# Replace PUT-PLAIN-MPK-OPT with actual MPK OPT value
val = FORMAT:HEX,OCT:PUT_PLAIN_MPK_OPT
action_flags = INTEGER:PUT_ACTFLAG_PLAIN_MPK_OPT

[ plain_mek_options ]
# Replace PUT-PLAIN-MEK-OPT with actual MEK OPT value
val = FORMAT:HEX,OCT:PUT_PLAIN_MEK_OPT
action_flags = INTEGER:PUT_ACTFLAG_PLAIN_MEK_OPT

[ plain_key_rev ]
# Replace PUT-PLAIN-KEY-REV with actual Key Rev value
val = FORMAT:HEX,OCT:PUT_PLAIN_KEY_REV
action_flags = INTEGER:PUT_ACTFLAG_PLAIN_KEY_REV

[ plain_key_cnt ]
# Replace PUT-PLAIN-KEY-CNT with actual Key Count value
val = FORMAT:HEX,OCT:PUT_PLAIN_KEY_CNT
action_flags = INTEGER:PUT_ACTFLAG_PLAIN_KEY_CNT

[ plain_swrev_(tifs/hsmRT) ]
# Replace PUT-PLAIN-SWREV-(TIFS/HSMRT) with actual SWREV (TIFS/HSMRT) value
val = FORMAT:HEX,OCT:PUT_PLAIN_SWREV_(TIFS/HSMRT)
action_flags = INTEGER:PUT_ACTFLAG_PLAIN_SWREV_(TIFS/HSMRT)

[ plain_swrev_sbl ]
# Replace PUT-PLAIN-SWREV-SBL with actual SWREV SBL value
val = FORMAT:HEX,OCT:PUT_PLAIN_SWREV_SBL
action_flags = INTEGER:PUT_ACTFLAG_PLAIN_SWREV_SBL

[ plain_swrev_sec_brdcfg ]
# Replace PUT-PLAIN-SWREV-SEC-BRDCFG with actual SWREV SEC BRDCFG value
val = FORMAT:HEX,OCT:PUT_PLAIN_SWREV_SEC_BRDCFG
action_flags = INTEGER:PUT_ACTFLAG_PLAIN_SWREV_SEC_BRDCFG

[plain_keywr_min_version ]
# Replace PUT-PLAIN-KEYWR-MIN-VER with actual KEYWR-MIN-VER value
val = FORMAT:HEX,OCT:PUT_PLAIN_KEYWR_MIN_VER

[ aesenc_user_otp ]
# Replace PUT-AESENC-USER-OTP with actual Encrypted OTP
val = FORMAT:HEX,OCT:PUT_AESENC_USER_OTP
iv = FORMAT:HEX,OCT:PUT_IV_AESENC_USER_OTP
rs = FORMAT:HEX,OCT:PUT_RS_AESENC_USER_OTP
wprp = FORMAT:HEX,OCT:PUT_WPRP_AESENC_USER_OTP
index = INTEGER:PUT_INDX_AESENC_USER_OTP
size = INTEGER:PUT_SIZE_AESENC_USER_OTP
action_flags = INTEGER:PUT_ACTFLAG_AESENC_USER_OTP
\endcode
### Otp Key writer Logs
UART Log
\code
#
# Decrypting extensions..
#
MPK Options:  0x21
MEK Options:  0x1
MPK Opt P1:  0x1
MPK Opt P2:  0x1
MEK Opt   :  0x1
SMPKH extension disabled
SMEK extension disabled
* BMPKH Part 1 BCH code: 8180dd66

* BMPKH Part 2 BCH code: 81873fe5

* BMPK Hash (part-1,2):

384be278a7a50eb25afdffac2e8bd306f82a3b51a770f8056c9ddeb9f31b0d3d01

3ea0063e6de3127a47c8a1443fc7e10dadffb51601aeaeb499d607e02874cd8001

* BMEK BCH code: c1bbe2be

* BMEK Hash: 9352f2c8069698ad4a1e6dfb381723ba4a15948a5e00c5ac004f574f194efe66bc701d378b01ec0bf1a36bef6e7a931d466dbdb38bd2be0aad8afc756aa5ea7a

EXT OTP extension disabled
MSV extension disabled

KEY CNT extension disabled

KEY REV extension disabled

SWREV extension disabled

FW CFG REV extension disabled

* KEYWR VERSION:  0x20000

#
# Programming Keys..
#

* MSV:
[u32] bch + msv:  0x8BAC0FFE
MSV extension disabled
[u32] bch + msv:  0x8BAC0FFE

* SWREV:
[u32] SWREV-(TIFS/HSMRT):  0x3
[u32] SWREV-SBL  :  0x4
SWREV extension disabled
[u32] SWREV-(TIFS/HSMRT):  0x3
[u32] SWREV-SBL  :  0x4

* FW CFG REV:
[u32] SWREV-FW-CFG-REV:  0x5
SWREV SEC BCFG extension disabled
[u32] SWREV-FW-CFG-REV:  0x5

* EXT OTP:
EXT OTP extension disabled

* BMPKH, BMEK:
Programmed 11/11 rows successfully
Programmed 2/2 rows successfully
Programmed 11/11 rows successfully
Programmed 2/2 rows successfully
Programmed 11/11 rows successfully
Programmed 2/2 rows successfully

* SMPKH, SMEK:
SMPKH extension disabled
SMEK extension disabled

* KEYCNT:
[u32] keycnt:  0x0
KEY CNT extension disabled
[u32] keycnt:  0x0

* KEYREV:
[u32] keyrev:  0x0
KEY REV extension disabled
[u32] keyrev:  0x0
\endcode
### Otp Key writer example app memory layout
\imageStyle{memory_layout_am273x.png,width:50%}
\image html memory_layout_am273x.png "Memory Layout"
### Otp Key writer validation
Following is the validation procedure used on OTP keywriter, these steps can be replicated by using the script keywriter_dir/scripts/generate_test_binaries.sh

#### Test binaries
Pass  |Details	                |Value	                                            |Purpose
------|-------------------------|---------------------------------------------------|-------------------------------------------------------------------------------
1	  |Program MSV	            |0xF1E22D	                                        |Demonstrates that MSV can be programmed successfully
2	  |SWREV (SBL+HSM)	        |sbl: 3, TIFS/hsmRT: 4	                            | -
3	  |SWREV (SEC APP)	        |sec-app: 5	                                        | -
4	  |EXT OTP (31:0)           |bits (32 bits)	0x80000001	                        |Ensures that you can program ext_otp area, of size more than 25 bits correctly
5	  |EXT OTP (39:32)          |bits (8 bits)	0x81	                            |Ensures that incremental programming is possible for ext otp
6	  |(WP) Program BMPKH, BMEK (set as dummy key bmpk val)| -                      |Dummy keys are programmed
7	  |(WP) Program SMPKH, SMEK (set as dummy key smpk val)| -	                    |Dummy keys are programmed
8	  |(WP) Program KEYCNT 2	|2 (0x0303)	                                        |Key count progammed
9	  |(~WP) Program KEYREV 1	|1 (0x0101)	                                        |After reboot, device will be HS-SE, use smpk for signing, If Keyrev 2 is programed use BMPK for signing

## OTHER DOCS
- \subpage SECURITY_X509_CERTIFICATE_DOCUMENTATION