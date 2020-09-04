#include <stdio.h>
//! [include]
#include <drivers/pcie.h>
//! [include]

#define CONFIG_PCIE0        (0U)
#define BUF_SIZE            (60U)

Pcie_Handle gPcieHandle;
uint32_t dst_buf[BUF_SIZE];

void open(void)
{
//! [open]
    gPcieHandle = Pcie_open(CONFIG_PCIE0);
    DebugP_assert(gPcieHandle != NULL);
//! [open]
}

void close(void)
{
//! [close]
    Pcie_close(gPcieHandle);
//! [close]
}

void obAtuCfg(void)
{
//! [obatu]
    int32_t status;
    Pcie_AtuRegionParams regionParams;
    uint32_t regionIndex = 1;

    regionParams.regionDir = PCIE_ATU_REGION_DIR_OUTBOUND;
    regionParams.tlpType   = PCIE_TLP_TYPE_MEM;

    regionParams.lowerBaseAddr    = 0x68000000UL + 0x01000000U;
    regionParams.upperBaseAddr    = 0x0;

    regionParams.regionWindowSize = 0xFFFU;

    regionParams.lowerTargetAddr    = 0x70000000U;
    regionParams.upperTargetAddr    = 0x0U;

    status = Pcie_atuRegionConfig (gPcieHandle, PCIE_LOCATION_LOCAL, regionIndex, &regionParams);

    DebugP_assert(SystemP_SUCCESS == status);

//! [obatu]
}

void ibAtuCfg(void)
{
//! [ibatu]
    int32_t status;
    Pcie_AtuRegionParams regionParams;
    uint32_t regionIndex = 1;

    regionParams.regionDir = PCIE_ATU_REGION_DIR_INBOUND;
    regionParams.tlpType   = PCIE_TLP_TYPE_MEM;

    regionParams.lowerBaseAddr    = 0x90000000U;
    regionParams.upperBaseAddr    = 0x0;

    regionParams.regionWindowSize = 0xFFFU;

    regionParams.lowerTargetAddr    = (uint32_t)dst_buf;
    regionParams.upperTargetAddr    = 0x0U;

    status = Pcie_atuRegionConfig (gPcieHandle, PCIE_LOCATION_LOCAL, regionIndex, &regionParams);

    DebugP_assert(SystemP_SUCCESS == status);
//! [ibatu]
}

void barCfgEP(void)
{
//! [barCfgEP]
    Pcie_BarCfg barCfg;
    int32_t status;
    uint32_t regionIndex = 0;

    barCfg.location = PCIE_LOCATION_LOCAL;
    barCfg.mode     = PCIE_EP_MODE;
    barCfg.barxc    = PCIE_BARC_32B_MEM_BAR_NON_PREFETCH;
    barCfg.barxa    = PCIE_RCBARA_4K;
    barCfg.idx      = regionIndex;

    status = Pcie_cfgBar (object->handle, &barCfg);

    DebugP_assert(SystemP_SUCCESS == status);

//! [barCfgEP]
}
