
#include <stdio.h>
//! [include]
#include <drivers/mdio.h>
//! [include]

uint32_t mdioBaseAddress;
uint32_t phyAddress;
int32_t status;

void mdio_init_clock(void)
{
//! [mdio_init_clock]
    status = MDIO_initClock(mdioBaseAddress);

    DebugP_assert(status == SystemP_SUCCESS);
//! [mdio_init_clock]
}

void mdio_link_status(void)
{
//! [mdio_link_status]
    status = MDIO_phyLinkStatus(mdioBaseAddress, phyAddress);
//! [mdio_link_status]
}

void mdio_phy_register_access(void)
{
//! [mdio_phy_register_access]
    uint16_t phyRegVal;

    /* Read a PHY register */
    status = MDIO_phyRegRead(mdioBaseAddress, NULL, phyAddress, 0, &phyRegVal);

    /* Write a PHY register */
    status = MDIO_phyRegWrite(mdioBaseAddress, NULL, phyAddress, 0, phyRegVal);
//! [mdio_phy_register_access]
}
