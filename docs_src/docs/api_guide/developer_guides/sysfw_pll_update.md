# Getting SDK 10.00 PLL Updates on older SDKs {#SYSFW_PLL_UPDATE_GUIDE}

[TOC]

In 10.00 SDK release, PLL programing sequence has been updated to follow the correct programing sequence to avoid PLL instability.
The PLL programing sequence has been updated in following components of the SDK.

- DMSC Firmware

SDK 10.0 has all the updated components by default.

Below section provides information on how to pick up the updates on older SDKs.

## Components to update

- Update DMSC firmware to SDK 10.0 version
   - https://github.com/TexasInstruments/mcupsdk-core/tree/REL.MCUSDK.10.00.00.20/source/drivers/sciclient/soc/am64x_am243x

- Update ABI version for SDK 10.0 (ABI changed in this DMSC version, not normal for all versions)
   - https://github.com/TexasInstruments/mcupsdk-core/commit/b58015393ed67de881fc53e6701a185ca237c319
