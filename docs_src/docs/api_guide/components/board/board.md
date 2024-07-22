# Board Peripheral Drivers {#BOARD_DRIVERS_PAGE}

[TOC]

This module has information related to the different board level peripherals that are supported

It consists of below sub-modules

\cond SOC_AM64X || SOC_AM243X || SOC_AM263X || SOC_AM263PX || SOC_AM261X
- \subpage BOARD_ETHPHY_PAGE
\endcond
\cond !SOC_AM65X
- \subpage BOARD_FLASH_PAGE
\cond SOC_AM64X || SOC_AM243X
- \subpage BOARD_RAM_PAGE
\endcond
- \subpage BOARD_EEPROM_PAGE
- \subpage BOARD_LED_PAGE
\endcond
\cond SOC_AM65X
- \subpage BOARD_LED_PAGE
\endcond