# Runtime Key Revision Update Example {#EXAMPLES_RUNTIME_KEYREV}

[TOC]

# Introduction

This example demonstrates the runtime Key Revision update to switch the Root of Trust to the programmed Backup Keys (BMPK/BMEK). The example reads the programmed Key Count and Key Revision and perform checks on the read values to validate if the Key Revision can be updated. If validated, the example enables the VPP and performs the Key Revision update procedure using the dual signed certificate. If the procedure is successful, it means the Root of Trust has been switched to the Backup Keys.

This example is supported only in HS-SE devices. This is a special example, and is booted by ROM. Because of this it is to be treated like a bootloader application. It makes use of Sciclient API calls to do this, there are wrapper functions provided in the example for this.

\note Set this macro **UPDATE_KEYREV** in the `{SDK_PATH}/examples/otp/runtime_keyrev/@VAR_BOARD_NAME_LOWER/r5fss0-0_nortos/main.c` to update the KEY Revision.

# Supported Combinations {#EXAMPLES_RUNTIME_KEYREV_COMBOS}

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/otp/runtime_keyrev/

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/otp/runtime_keyrev/

\endcond

# Steps to Run the Example

## Generate the Dual Signed Certificate

This examples requires a dual signed certificate to securely update the KEYREV. This dual signed certificate is a concatenation of BMPK signed certificate and SMPK signed certificate. The dual signed certificate can be generated with the following steps:

- Change the working directory to `{SDK_PATH}/source/security/security_common/tools/boot/signing`.
- Run the following command to generate the `dual_cert_keyrev.bin` in the same directory. Replace the `{SMPK_PATH}` and `{BMPK_PATH}` with the actual paths to the SMPK and BMPK key respectively.

      python3 dualCertGen.py -s {SMPK_PATH} -b {BMPK_PATH}

- Convert the `dual_cert_keyrev.bin` into a header file `dual_cert_keyrev.h` using the following command.

      cd `{SDK_PATH}/tools/bin2c`
      python3 bin2c.py dual_cert_keyrev.bin dual_cert_keyrev.h KEYREV_CHANGE_CERT

- Replace the `dual_cert_keyrev.h` file in the `{SDK_PATH}/examples/otp/runtime_keyrev` folder with the file generated from 
  dual signed certificate binary( file generated from above step).For example,

      cp dual_cert_keyrev.h {SDK_PATH}/examples/otp/runtime_keyrev/

## Build the example

\note This example runs on HS-SE devices so the `DEVICE_TYPE` in `{SDK_PATH}/devconfig/devconfig.mak` must be set to HS.

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)

## Run the example

This example is a bootloader application and so can be booted over any ROM supported boot media. For typical use cases, following are the steps to flash and boot the example from OSPI.

- Set the bootmode switches to UART bootmode.
- Flash the example using the following command run from `{SDK_PATH}/tools/boot`

      python uart_uniflash.py -p {COM_PORT} --cfg=../../examples/otp/runtime_keyrev/@VAR_BOARD_NAME_LOWER/r5fss0-0_nortos/default_runtime_keyrev_hs.cfg

- After successful flashing, power off the board and set the bootmode to OSPI.
- Power on the board. The example should boot with the logs appearing on MAIN_UART0.

# Sample Output

Shown below is a sample output when the application is run:

\code
Starting Runtime KEYREV writer

DMSC Firmware Version 10.0.8--v10.00.08 (Fiery Fox)
DMSC Firmware revision 0xa
DMSC ABI revision 4.0

Value of KEYREV: 0x1 and KEY Count: 0x2
Dual Certificate found: 0x7000bd80
Updating KEYREV value to 2 ...
Keyrev Writer Response : 0x2
Updating KEYREV successfull
All tests have passed!!
\endcode

# Post Updating KEYREVISION steps to be followed

If this example runs successfully, it means the KEYREV has been updated to `2` switching the Root of Trust to the backup keys (BMPK/BMEK). So, any existing images signed with SMPK and optionally encrypted with SMEK will fail to boot. The images needs to signed with the BMPK and optionally encrypted with the BMEK.

Backup Keys are named `bmpk.pem` and `bmek.key`, following are the steps to configure the signing of the images with Backup Keys:

- Convert the encryption key `bmek.key` to a hex string and save as `bmek.txt`.
```bash
$ cd $path of the folder containing Backup Keys
$ xxd -p -c 10000 bmek.key | tr -d $'\n' | tee bmek.txt
```
- Update the `CUST_MPK` and `CUST_MEK` variables according to the device in `{SDK_PATH}/devconfig/devconfig.mak` to refer to the Backup Keys.
```
CUST_MPK=$(SIGNING_TOOL_PATH)/bmpk.pem
CUST_MEK=$(SIGNING_TOOL_PATH)/bmek.txt
```

After the above changes, rebuild the libs and examples being used.
