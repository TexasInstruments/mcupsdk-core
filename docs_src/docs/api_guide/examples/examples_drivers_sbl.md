#  Secondary Bootloader (SBL) {#EXAMPLES_DRIVERS_SBL}

This page lists all the examples related to secondary bootloader and bootloader driver.

To know more about SBL and overall boot flow please take a look at \ref BOOTFLOW_GUIDE
\cond SOC_AM64X || SOC_AM243X
   -# \subpage EXAMPLES_DRIVERS_SBL_NULL
   -# \subpage EXAMPLES_DRIVERS_SBL_OSPI
   -# \subpage EXAMPLES_DRIVERS_SBL_OSPI_MULTI_PARTITION
   -# \subpage EXAMPLES_DRIVERS_SBL_PCIE_HOME
   -# \subpage EXAMPLES_DRIVERS_SBL_SD
   -# \subpage EXAMPLES_DRIVERS_SBL_UART
   -# \subpage EXAMPLES_DRIVERS_SBL_UART_UNIFLASH
   -# \subpage EXAMPLES_DRIVERS_SBL_JTAG_UNIFLASH
   -# \subpage EXAMPLES_DRIVERS_SBL_DFU_UNIFLASH
   -# \subpage EXAMPLES_DRIVERS_SBL_DFU
   \cond SOC_AM64X
   -# \subpage EXAMPLES_DRIVERS_SBL_OSPI_LINUX
   -# \subpage EXAMPLES_DRIVERS_SBL_EMMC_LINUX
   \endcond
\endcond

\cond SOC_AM273X || SOC_AWR294X
   -# \subpage EXAMPLES_DRIVERS_SBL_NULL
   -# \subpage EXAMPLES_DRIVERS_SBL_QSPI
   -# \subpage EXAMPLES_DRIVERS_SBL_UART
   -# \subpage EXAMPLES_DRIVERS_SBL_UART_UNIFLASH
   -# \subpage EXAMPLES_DRIVERS_SBL_JTAG_UNIFLASH
    \cond SOC_AM273X
   -# \subpage EXAMPLES_DRIVERS_SBL_CAN_UNIFLASH
    \endcond
\endcond

\cond SOC_AM263X
   -# \subpage EXAMPLES_DRIVERS_SBL_NULL
   -# \subpage EXAMPLES_DRIVERS_SBL_QSPI
   -# \subpage EXAMPLES_DRIVERS_SBL_UART
   -# \subpage EXAMPLES_DRIVERS_SBL_CAN
   -# \subpage EXAMPLES_DRIVERS_SBL_CAN_UNIFLASH
   -# \subpage EXAMPLES_DRIVERS_SBL_UART_UNIFLASH
   -# \subpage EXAMPLES_DRIVERS_SBL_JTAG_UNIFLASH
   -# \subpage EXAMPLES_DRIVERS_SBL_SD
\endcond

\cond SOC_AM263PX
   -# \subpage EXAMPLES_DRIVERS_SBL_NULL
   -# \subpage EXAMPLES_DRIVERS_SBL_OSPI
   -# \subpage EXAMPLES_DRIVERS_SBL_UART
   -# \subpage EXAMPLES_DRIVERS_SBL_CAN
   -# \subpage EXAMPLES_DRIVERS_SBL_CAN_UNIFLASH
   -# \subpage EXAMPLES_DRIVERS_SBL_UART_UNIFLASH
   -# \subpage EXAMPLES_DRIVERS_SBL_JTAG_UNIFLASH
   -# \subpage EXAMPLES_DRIVERS_SBL_SD
\endcond

\cond SOC_AWR294X
   -# \subpage EXAMPLES_DRIVERS_SBL_QSPI_ENET
\endcond
