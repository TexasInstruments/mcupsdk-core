# Bootflow Migration Guide{#BOOTFLOW_MIGRATION_GUIDE}

Some bootflow changes have been introduced in MCU+SDK, this document aims to explain
the changes and is majorly meant for users of older SDK wanting to upgrade to the
new flow.

## Change Summary

Normal Bootflow(OLD)                                     |Combined Bootflow(NEW)
---------------------------------------------------------|-----------------------------------------------------------------------------------
1. Power On Reset                                        | 1. Power On Reset
2. ROM Bootloader (RBL) loads Secondary Bootloader (SBL) | 2. ROM Bootloader (RBL) loads SBL, SYSFW, and boardcfg together as one binary blob
3. SBL loads SYSFW, and sets boardcfg                    | 3. SBL waits for SYSFW to boot up
4. SBL loads application image(s)                        | 4. SBL loads application image(s)

## Change Details

- To understand what changed in the new combined boot flow, let's revisit the old (legacy) bootflow once:

\imageStyle{normalflow.png, width:30%}
\image html normalflow.png "Normal (Legacy) Boot Flow"

- As it is shown in the above image, in the normal flow the RBL loads the SBL which
then loads the SYSFW and sets boardcfg. To do this, the SBL needs to have the SYSFW
and boardcfg data as hex arrays included in the SBL application source. The actual
loading to the Cortex M3 core would be done by the DMSC ROM running in M3, and the
SBL will just pass a pointer to the SYSFW array. There is an Sciclient API,
`Sciclient_loadFirmware` used for this purpose. We created a wrapper API which calls
this API, `Bootloader_socLoadSysfw`. It can be seen that this API is pretty much the
first one to be invoked in the `main()` of the SBL. This API will also set the boardcfg,
by calling the appropriate Sciclient APIs.

- While this approach is simplistic, it comes with it's challenges. First of all,
size of SYSFW is about 220 KB. This has to be placed in `.rodata` section when
building the SBL application. This is wasted memory and will add to the size of
SBL final image.

- In addition to this, in supporting secure devices, SYSFW would be different.
So for booting in secure devices the SBLs and bootloader library will have to be
re-built to include the new C array with different firmware.

- The solution to these issues is combined boot flow, in which ROM will load the
SYSFW and boardcfg data instead of SBL. In this flow, we don't need to include the
SYSFW or boardcfg in the SBL application, hence saving almost 220 KB of memory which

can be used by applications. Also since the SYSFW and boardcfg data is not part of
the build flow, the same libraries and SBLs can be used for secure/non-secure devices.
Given below is a similar flow diagram for combined boot flow.

\imageStyle{combinedflow.png, width:30%}
\image html combinedflow.png "Combined Boot Flow"

## Compatibility Breaks To Look-out For

- For someone using the SBLs provided in the SDK as is, this update shouldn't change
anything. But for ones who have written their own SBLs following the flow which
we have provided, there might be some compatibility breaks

1. `Bootloader_socLoadSysFw` __API missing__ : Since SBL no longer needs to load
SYSFW, the `Bootloader_socLoadSysFw()` was removed to decrease the library size.
2. __Signing Process__ : Old (Legacy) Boot Flow and New (Combined) Boot Flow is
recognized by ROM by virtue of the format of the x509 Certificate appended to the SBL.
In old (legacy) boot flow, the SBL image to be booted by ROM was created using
`x509Certificate.sh/x509Certificate.ps1` scripts under `tools/boot/signing` in
the SDK. This script generates an x509 certificate for the SBL binary and append
it to the SBL binary to create the final `*.tiimage` file. This signing and
certificate appendment is required for ROM boot.

In combined boot flow, we use a new script, `rom_image_gen.py` to generate the
final `*.tiimage` which will be booted by the ROM.

If you are someone who doesn't use the SDK build system, you can take a look at the
makefiles of any SBL examples to understand the usage of `rom_image_gen.py` script
and take this into your build system as well.

Usage print of `rom_image_gen.py` is as follows:

\code
usage: rom_image_gen.py [-h] [--swrv SWRV] --sbl-bin SBL_BIN [--sbl-enc] [--enc-key ENC_KEY] --sysfw-bin SYSFW_BIN [--sysfw-inner-cert SYSFW_INNER_CERT] --boardcfg-blob BOARDCFG_BLOB --sbl-loadaddr SBL_LOADADDR --sysfw-loadaddr SYSFW_LOADADDR --bcfg-loadaddr BCFG_LOADADDR --key KEY --rom-image ROM_IMAGE [--debug DEBUG]
\endcode

As you can see, the SBL binary and SYSFW binary among other things are __combined__ together to form the final image as opposed to `x509Certificate.sh/ps1` script generating x509 certificate for SBL alone.
