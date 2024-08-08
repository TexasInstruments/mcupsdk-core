To add DP83TC812 PHY, please follow the below steps:


# Changes in enet-lld:
1. Copy the phy driver to <mcu_plus_sdk>/source/networking/enet/core/src/phy/ (src) and  <mcu_plus_sdk>/source/networking/enet/core/include/phy/     (api header file)
2. Add .c file for makefile (<mcu_plus_sdk>/source/networking/enet/core/src/phy/makefile) . This adds the source file to enet-lld lib.

Make sure gEnetPhyDrvdp83tc812 is defined in the phy driver source file.

3. Compile enet-lld libs :
`$ cd <mcu_plus_sdk>/source/networking/enet`
`$ make -f makefile.cpsw.am263x.r5f.ti-arm-clang PROFILE=debug clean`
`$ make -f makefile.cpsw.am263x.r5f.ti-arm-clang PROFILE=release clean`
`$ make -f makefile.cpsw.am263x.r5f.ti-arm-clang PROFILE=debug all`
`$ make -f makefile.cpsw.am263x.r5f.ti-arm-clang PROFILE=release all`

4. In application board.c (<mcu_plus_sdk>/examples/drivers/boot/sbl_qspi_enet/am263x-cc/r5fss0-0_nortos/board.c) file
do the following changes:

5. Replace `#include <networking/enet/core/include/phy/dp83867.h>` with `#include <networking/enet/core/include/phy/dp83tc812.h>`
6. Replace `gEnetPhyDrvDp83869` with `gEnetPhyDrvdp83tc812` in the whole file
7. Define PHY specific extended configuration in board.c file

Eg:
`static const Dp83869_Cfg gEnetCpbBoard_dp83869PhyCfg =`

THis has to be replaced with `static const Dp83tc812_Cfg gEnetCpbBoard_dp83tc812PhyCfg
 and fill eht values as the PHY settings.

8. Change the initialition of gEnetCpbBoard_am263xEthPort to below:

static const EnetBoard_PortCfg gEnetCpbBoard_am263xEthPort[] =
{
    {    /* "CPSW3G" */
        .enetType = ENET_CPSW_3G,
        .instId   = 0U,
        .macPort  = ENET_MAC_PORT_1,
        .mii      = { ENET_MAC_LAYER_MII, ENET_MAC_SUBLAYER_REDUCED },
        .phyCfg   =
        {
            .phyAddr         = < phy address>,
            .isStrapped      = true,
            .skipExtendedCfg = false,
            .extendedCfg     = &gEnetCpbBoard_dp83tc812PhyCfg,
            .extendedCfgSize = sizeof(gEnetCpbBoard_dp83tc812PhyCfg),
        },
        .flags    = 0U,
    },
    {    /* "CPSW3G" */
        .enetType = ENET_CPSW_3G,
        .instId   = 0U,
        .macPort  = ENET_MAC_PORT_2,
        .mii      = { ENET_MAC_LAYER_MII, ENET_MAC_SUBLAYER_REDUCED },
        .phyCfg   =
        {
            .phyAddr         = < phy address>,
            .isStrapped      = true,
            .skipExtendedCfg = false,
            .extendedCfg     = &gEnetCpbBoard_dp83tc812PhyCfg,
            .extendedCfgSize = sizeof(gEnetCpbBoard_dp83tc812PhyCfg),
        },
        .flags    = 0U,
    },
};


9. Thats all the changes. You may compile and run the application.
