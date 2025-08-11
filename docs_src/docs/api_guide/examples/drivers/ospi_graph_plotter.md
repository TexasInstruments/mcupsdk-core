# OSPI Graph Plotter {#EXAMPLES_DRIVERS_OSPI_GRAPH_PLOTTER}

[TOC]

# Introduction

This example is used to plot the ospi phy tuning sweep on a graph. 

# Supported Combinations {#EXAMPLES_DRIVERS_OSPI_GRAPH_PLOTTER_COMBOS}


\cond SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/ospi/ospi_graph_plotter

\endcond

\cond SOC_AM261X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/ospi/ospi_graph_plotter

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
- Use the python script provided with the example to plot the graph -> python phy_tuning_graph_plotter.py path/to/generated/bin/file .

# See Also

\ref DRIVERS_OSPI_PAGE

# Sample Output

\code
 Sweeping... !!!
Writing to file: graph.bin
All tests have passed!!
\endcode