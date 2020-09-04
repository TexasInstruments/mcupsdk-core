#include <stdio.h>
//! [include]
#include <drivers/firewall.h>
//! [include]

#define CONFIG_FIREWALL0     (0U)

Firewall_Handle gFirewallHandle;

void open(void)
{
//! [open]
    Firewall_open(CONFIG_FIREWALL0);
    DebugP_assert(gFirewallHandle != NULL);
//! [open]
}

void close(void)
{
//! [close]
    Firewall_close(gFirewallHandle);
//! [close]
}

void cfgfwlReg(void)
{
//! [cfgfwlReg]
    int32_t status;
    Firewall_RegionCfg regionParams;
    Firewall_Attrs gFirewallAttrs;
    uint32_t regionIndex = 1;

    regionParams.regionIndex    = 0;
    regionParams.control        = FWL_CONTROL_ENABLE;
    regionParams.permissions[0] = (uint32_t) PRIVID_EVERYONE << 16 | \
                                             FWL_PERM_SEC_RW;
    regionParams.permissions[1] = 0x0;
    regionParams.permissions[2] = 0x0U;
    regionParams.startAddr      = 0x70000000U;
    regionParams.endAddr        = 0x7003FFFFU;

    gFirewallAttrs.firewallId = 14,
    gFirewallAttrs.totalRegions = 4,
    gFirewallAttrs.regionInfo = regionParams,
    gFirewallAttrs.initRegions = 1,

    status = Firewall_configureRegion(gFirewallHandle, gFirewallAttrs);

    DebugP_assert(SystemP_SUCCESS == status);

//! [cfgfwlReg]
}
