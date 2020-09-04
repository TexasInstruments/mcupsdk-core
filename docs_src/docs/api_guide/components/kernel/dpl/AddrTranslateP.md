# Region based Address Translate {#KERNEL_DPL_ADDR_TRANSLATE_PAGE}

[TOC]

## Features Supported

- APIs to setup region based address translation using RAT HW on supported SOCs
- APIs to translate 48b SOC view system addr to 32b local CPU view address.
- When no address mapping is specified, no translation is done.

## Features NOT Supported

NA

## Important Usage Guidelines

- The translation API \ref AddrTranslateP_getLocalAddr is meant to be used to translate SOC specified peripheral MMR base
  addresses to local CPU accesible addresses, within device drivers. The API internally searches through a small array to find the address translations, hence to be efficient, this API should typically be called once during driver init to find the local address that CPU should use.
\cond SOC_AM64X || SOC_AM243X
- This module is not normally required on R5F CPUs since all peripherals are typically mapped within the 32b address
  space of R5F.
\endcond
\cond SOC_AM64X || SOC_AM243X || SOC_AM62X
- This module is needed for M4F to access peripherals on the MainSS side of the SOC.
\endcond

## Example Usage

Include the below file to access the APIs,
\snippet AddrTranslateP_sample.c include

Example to translate a system address to local CPU address,
\snippet AddrTranslateP_sample.c addr_translate

## API

\ref KERNEL_DPL_ADDR_TRANSLATE
