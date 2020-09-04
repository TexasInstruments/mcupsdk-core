MDIO Manual Mode Enablement {#enetmdio_manualmode}
============================

[TOC]

\note If your silicon is affected with errata <a href="https://www.ti.com/lit/er/sprz457e/sprz457e.pdf" target="_blank">i2329— MDIO interface corruption</a>, then TI suggests to use MDIO Manual mode as software workaround mentioned below


# Workaround for details errata i2329-MDIO interface corruption and its impact:
- more details about errata here <a href="https://www.ti.com/lit/er/sprz457e/sprz457e.pdf" target="_blank">i2329— MDIO interface corruption</a>.
- Workaround enforces to configure MDIO in 'MDIO_MODE_MANUAL' \ref Mdio_Cfg::mode during Mdio_open. MDIO statemachine should not be enabled when using this workaround.
- MDIO_CLK and MDIO_IO pins are toggled using MANUAL_IF_REG registers of MDIO, using software bit-banging logic. The delays inside bit-banging logic is tuned to limit the MDIO_CLK frequency to 500 KHz.
- MDIO link change status callback (API) for application is disabled.
- Polling logic to get LINK status of active PHYs should be implemented by application, by calling EnetPhy_isLinked() API from PHY driver. Please refer to "Enet Lwip CPSW Example" and "Enet Lwip ICSSG Example" for implementation.
- Each PHY register read consumes around 100 CPU cycles as tested on AM243x platform. Which implies, polling one PHY link status at every 100 ms periodicity would take 0.1% of CPU. It scales with number of PHYs to be monitored.

# Limitations
- Only Clause 22 frame format is support. Clause 45 frame format is not supported.
- MDIO state machine is disabled and hence MDIO LINK_INT and USER_INT interrupts are NOT available.
- Testing has been done by keeping "link status polling task" running at highest priority task in application.

# Not supported MDIO IOCTL APIs in MDIO Manual mode

  Affected API      | Change
 -------------------|----------
ENET_MDIO_IOCTL_GET_VERSION | Supported
ENET_MDIO_IOCTL_PRINT_REGS | Not Supported if MDIO is configured in Manual Mode
ENET_MDIO_IOCTL_IS_ALIVE | Supported
ENET_MDIO_IOCTL_IS_LINKED | Supported
ENET_MDIO_IOCTL_IS_POLL_ENABLED | Not Supported if MDIO is configured in Manual Mode
ENET_MDIO_IOCTL_C22_READ | Supported
ENET_MDIO_IOCTL_C22_WRITE | Supported
ENET_MDIO_IOCTL_C45_READ | Not Supported if MDIO is configured in Manual Mode
ENET_MDIO_IOCTL_C45_WRITE | Not Supported if MDIO is configured in Manual Mode
ENET_MDIO_IOCTL_C22_ASYNC_READ_TRIGGER | Not Supported if MDIO is configured in Manual Mode
ENET_MDIO_IOCTL_C22_ASYNC_READ_COMPLETE | Not Supported if MDIO is configured in Manual Mode
ENET_MDIO_IOCTL_C22_ASYNC_WRITE_TRIGGER | Not Supported if MDIO is configured in Manual Mode
ENET_MDIO_IOCTL_C22_ASYNC_WRITE_COMPLETE | Not Supported if MDIO is configured in Manual Mode
ENET_MDIO_IOCTL_C45_ASYNC_READ_TRIGGER | Not Supported if MDIO is configured in Manual Mode
ENET_MDIO_IOCTL_C45_ASYNC_READ_COMPLETE | Not Supported if MDIO is configured in Manual Mode
ENET_MDIO_IOCTL_C45_ASYNC_WRITE_TRIGGER | Not Supported if MDIO is configured in Manual Mode
ENET_MDIO_IOCTL_C45_ASYNC_WRITE_COMPLETE | Not Supported if MDIO is configured in Manual Mode
MDIO_IOCTL_HANDLE_INTR | Not Supported if MDIO is configured in Manual Mode
ENET_MDIO_IOCTL_ENABLE_STATE_MACHINE | Not Supported if MDIO is configured in Manual Mode