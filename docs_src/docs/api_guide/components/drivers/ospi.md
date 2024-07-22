# OSPI {#DRIVERS_OSPI_PAGE}

[TOC]

The Octal Serial Peripheral Interface (OSPI) module is a kind of Serial Peripheral Interface (SPI) module
which allows single, dual, quad or octal read and write access to external flash devices.
The OSPI module is used to transfer data, either in a memory mapped direct mode (for example a
processor wishing to execute code directly from external flash memory), or in an indirect mode where the
module is set-up to silently perform some requested operation, signaling its completion via interrupts or
status registers.

## Features Supported

- Support for single, dual, quad (QSPI mode) or octal I/O instructions.
- Supports dual Quad-SPI mode for fast boot applications.
- Memory mapped ‘direct’ mode of operation for performing flash data transfers and executing code from flash memory.
- Programmable delays between transactions.
- Legacy mode allowing software direct access to low level transmit and receive FIFOs, bypassing the higher layer processes.
- An independent reference clock to decouple bus clock from SPI clock – allows slow system clocks.
- Programmable baud rate generator to generate OSPI clocks.
- Supports BOOT mode.
- Handling ECC errors for flash devices with embedded correction engine.

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

- OSPI instance name
- Input clock frequency to be used for OSPI module
- Input clock divider which decides the baud-rate at which the flash will be read
- Chip Select
\cond !(SOC_AM263PX || SOC_AM261X)
- Enabling of various features like DMA, PHY mode(not supported yet), XIP(not supported yet)
\endcond
\cond SOC_AM263PX || SOC_AM261X
- Enabling of various features like DMA and PHY mode.
    - PHY configuration allows to
        -  Configuring window tuning parameters.
        -  Set PHY control mode (master/bypass)
        -  Determine if master delay line locks in half/full cycle of delay.

        \imageStyle{ospi_phy_configuration.png, width:80%}
        \image html ospi_phy_configuration.png "OSPI PHY Configurations Syscfg"

        - Tuning window parameters,contains various parameters for OSPI tuning
            - Min and Max values for read delays.
            - TxDLL low start and end, defines txDLL window for finding rxLow.
            - TxDLL high start and end, defines txDLL winodw for finding backup rxLow.
            - RxDLL low search start, minimum value for rxLow.
            - RxDLL low search end, maximum value for rxLow.
            - RxDLL high search start, minimum value for rxHigh.
            - RxDLL high search end, maximum value for rxHigh.
            - TxDLL low search start, minimum value for txLow.
            - TxDLL low search end, maximum value for txLow.
            - TxDLL high search start, minimum value for txHigh.
            - TxDLL high search end, maximum value for txHigh.
            - TxDLL search offset, increment in txDLL value while searching rxLow/rxHigh values.
            - RxDL & TxDLL Search Step, step size for finding rx and tx low/high values.

        \image html ospi_window_params.png "OSPI WINDOW PARAMS"

\endcond
- In advanced config, you can choose various parameters like frame format, decoder chip select, read dummy cycles etc.
- Pinmux configurations for the OSPI instance

## OSPI Phy Tuning Algorithm

The OSPI tuning algorithm works as follows:
- **Step 1 Find Golden Primary RxLow**\n
	To find the RxDLL boundaries, we fix a valid TxDLL and search through RxDLL range, rdDelay values.\n
    As we are not sure of a valid TxDLL we use a window of TxDLL values to find the RxDLL boundaries.

 					Rx_DLL
    		          ▲
    		          │   ++++++++++++++++
    		      127 │     ++++++++++++++
    		          │   x   ++++++++++++
    		          │   xx   +++++++++++
    		          │   xxx   ++++++++++
    		          │   xxxx   +++++++++
    		          │   xxxxx   ++++++++
    		          │ │ xxx│xx   +++++++
    		          │ │ xxx│xxx   ++++++
    		          │ │ xxx│xxxx   +++++
    		          │ │ xxx│xxxxx   ++++
    		          │ │ xxx│xxxxxx   +++
    		 Search   │ │ xxx│xxxxxxx   ++
    		 Rx_Low ──┼─┤►xxx│xxxxxxxx   +
    		          │ │    │
    		         ─┼─┼────┼------────►  Tx_DLL
    		         0│ │    │           127
    		            │    │
    		            │    │

    		        Tx_Low   Tx_Low
    		        Start    End
\n
- **Step 2 Find Golden Secondary RxLow**\n
	Search for one more rxLow at different txDl, To find Secondary rxHigh we use the txDLL + Search_offset value of rxLow.\n
\n
- **Step 3 Select minimum of primary rxLow and secondary rxLow Value**\n
	- Pick minimum value of rxDLL between rxLow and sec_rxLow.
	- Pick Minimum value of rdDelay(read_delay) between rxLow and sec_rxLow.\n

	Primary\n Search   | Secondary\n Search  | Final Point
	:---------------:  | :-----------------: | :--------------------------------
	Fail               | Fail              	 | Return Fail
	Fail               | Pass              	 | Return Fail
	Pass               | Fail              	 | Return Fail
	Pass               | Pass              	 | RxDll = Min(Primary, Secondary) \n RdDelay = Min(Primary, Secondary) \n TxDll = Primary
\n
- **Step 4 Find Golden Primary Rx High**\n
	To find rxHigh we use the txDLL values of rxLow.\n
    Start the rdDelay (Read delay) from maximum and decrement it.\n
    As these are valid values and rxHigh rdDelay is always >= rxLow rdDelay.

			        Rx_DLL
		              ▲
		          127 │   ▲+++++++++++++++
		    Search    │   │ ++++++++++++++
		   Rx_High────┼──►│   ++++++++++++
		   on Fixed   │   │x   +++++++++++
		    Tx_DLL    │   │xx   ++++++++++
		              │   │xxx   +++++++++
		              │   │xxxx   ++++++++
		              │   ▼xxxxx   +++++++
		              │   Xxxxxxx   ++++++
		              │   Xxxxxxxx   +++++
		              │   Xxxxxxxxx   ++++
		              │   Xxxxxxxxxx   +++
		              │   Xxxxxxxxxxx   ++
		              │   Xxxxxxxxxxxx   +
		              │
		             ─┼------------───►  Tx_DLL
		             0│                  127
\n
- **Step 5 Find Golden Secondary Rx High**\n
	To find Secondary rxHigh we use the txDLL + Search_offset value of rxLow.\n
    Start the rdDelay (Read delay) from maximum and decrement it.\n
    As these are valid values and rxHigh rdDelay is always >= rxLow rdDelay.\n
\n
- **Step 6 Select maximum of primary and secondary rxHigh value**\n
	- Compare the Primary and Secondary point.\n
    - Pick the point which has passing maximum rxDll.\n

	Primary\n Search   | Secondary\n Search  | Final Point
	:---------------:  | :-----------------: | :--------------------------------
	Fail               | Fail              	 | Return Fail
	Fail               | Pass              	 | Return Fail
	Pass               | Fail              	 | Return Fail
	Pass               | Pass              	 | If(secondary.rxDll > primary.rxDll) \n Pick Secondary search point \n Else \n Pick Primary search point
\n
- **Step 7 Do a backup Search in case rxLow and rx High has same read delay values**\n
    Check a different point if the rxLow and rxHigh are on the same rdDelay.\n
    This avoids mistaking the metastability gap for an rxDLL boundary.\n

    - **Find backup rx Low**
	    Find the rxDLL boundaries using the TxDLL window at the higher end .\n
        We start the window_end and decrement the TxDLL value until we find the valid point.

        		   Rx_DLL
        		    ▲
        		    │   ++++++++++++++++
        		127 │   ++++++++++++++++
        		    │   ++++++++++++++++
        		    │    +++++++++++++++
        		    │     +++++++++│++++│
        		    │      ++++++++│++++│
        		    │   x   +++++++│++++│
        		    │   xx   ++++++│++++│
        		    │   xxx   +++++│++++│
        		    │   xxxx   ++++│++++│
        		    │   xxxxx   +++│++++│
        		    │   xxxxxx   ++│++++│
        		    │   xxxxxxx   +│++++│         Search
        		    │   xxxxxxxx   │++++◄───────  Rx_Low
        		    │              │    │
        		   ─┼──────────────┼────┤► Tx_DLL
        		   0│              │    │   127
        		                   │    │
        		           Tx_High        Tx_High
        		           Start          End

    - **Find backup sec rxLow**\n
        Search for one more rxLow at different txDll, we use the txDLL - Search_offset value of rxLow.\n

    - **Select minimum of backup rxLow and backup sec rxLow**
        - Pick minimum value of rxDLL between rxLow and sec_rxLow.
	    - Pick Minimum value of rdDelay(read_delay) between rxLow and sec_rxLow.\n

    - **Find backup primary rxHigh search**
        Find rxDLL Max\n
        Start the rdDelay (Read delay) from maximum and decrement it.\n

                    Rx_DLL
            127 ▲
                │   +++++++++++++++▲                Search Rx_High
                │   +++++++++++++++│◄────────────   on Fixed Tx_DLL
                │   +++++++++++++++│
                │    ++++++++++++++│
                │     +++++++++++++│
                │      ++++++++++++│
                │   x   +++++++++++▼
                │   xx   +++++++++++
                │   xxx   ++++++++++
                │   xxxx   +++++++++
                │   xxxxx   ++++++++
                │   xxxxxx   +++++++
                │   xxxxxxx   ++++++
                │   xxxxxxxx    ++++
                │
               ─┼────────────────────► Tx_DLL
               0│                       127

    - **Find backup secondary rxHigh search**\n
        Search for one more rxHigh at different txDll, we use the txDLL - Search_offset value of rxLow.\n

    - **Select maximum of backup primary and  secondary rxhigh**\n
        Compare the Primary and Secondary point.\n
        Pick the point which has passing maximum rxDll.\n

        Primary\n Search   | Secondary\n Search  | Final Point
	    :---------------:  | :-----------------: | :--------------------------------
	    Fail               | Fail              	 | Return Fail
	    Fail               | Pass              	 | Return Fail
	    Pass               | Fail              	 | Return Fail
	    Pass               | Pass              	 | If(secondary.rxDll > primary.rxDll) \n Pick Secondary search point \n Else \n Pick Primary search point
\n
- **Step 8 Find golden TxLow**
    Look for txDLL boundaries at 1/4 of rxDLL window.\n
    Find txDLL Min.\n

                              Rx_DLL
                         127 ▲
                             │   ++++++++++++++++
                  Rx_High    │     ++++++++++++++
                      ───────┼──►x   ++++++++++++
                             │   xx   +++++++++++
                             │   xxx   ++++++++++
                             │   xxxx   +++++++++
               Fix Rx_DLL    │   xxxxx   ++++++++
              1/4 between    │   xxxxxx   +++++++
              Rx_High and    │   xxxxxxx   ++++++
                Rx_Low       │   xxxxxxxx   +++++
                       ──────┼─► ◄───┬──►    ++++
                             │   xxxx│xxxxx   +++
                   Rx_Low    │   xxxx│xxxxxx   ++
                       ──────┼──►xxxx│xxxxxxx   +
                             │       │
                            ─┼───────┼───────────►  Tx_DLL
                            0│       │          127
                                     │
                                Search Tx_Low
\n
- **Step 9 Find golden TxHigh**\n
    Find txDLL Max.\n
    Start the rdDelay (Read delay) from maximum and decrement it.\n

                Rx_DLL
                   127 ▲
                       │   +++++++++++++++++
            Rx_High    │     +++++++++++++++
                ───────┼──►x   +++++++++++++
                       │   xx   ++++++++++++
                       │   xxx   +++++++++++
                       │   xxxx   ++++++++++
         Fix Rx_DLL    │   xxxxx   +++++++++
        1/4 between    │   xxxxxx   ++++++++
        Rx_High and    │   xxxxxxx   +++++++
           Rx_Low      │   xxxxxxxx   ++++++
                 ──────┼─► xxxxxxxxx   ◄─┬─►
                       │   xxxxxxxxxx   +│++
             Rx_Low    │   xxxxxxxxxxx   │++
                 ──────┼──►xxxxxxxxxxxx  │++
                       │                 │
                      ─┼─────────────────┼─►  Tx_DLL
                      0│                 │127
                                      Search Tx_Max
\n
- **Step 10 Do a backup Search in case rxLow and rx High has same read delay values**\n
    Check a different point if the txLow and txHigh are on the same rdDelay.\n
    This avoids mistaking the metastability gap for a txDLL boundary.\n

    - **Find a backup primary txLow**\n
        Look for txDLL boundaries at 3/4 of rxDLL window.\n
        Find txDLL Min.\n

                     Rx_DLL
                    127 ▲
                        │
               Rx_High──┼──►+++++++++++++++++
            Fix Rx_DLL  │   +++++++++++++++++
               3/4 of   │   +++++++++++++++++
              Rx_High  ─┼─► ◄───┬───►++++++++
            and Rx_Low  │     ++│++++++++++++
                        │      +│++++++++++++
                        │   x   │++++++++++++
                        │   xx  │++++++++++++
                        │   xxx │ +++++++++++
                        │   xxxx│  ++++++++++
                        │   xxxx│   +++++++++
                        │   xxxx│x   ++++++++
                        │   xxxx│xx   +++++++
                Rx_Low──┼──►xxxx│xxx   ++++++
                        │       │
                       ─┼───────┼────────────► Tx_DLL
                       0│       │               127
                           Search Tx_Min

    - **Find a backup primary txHigh**\n
        Find txDLL Max.\n
        Start the rdDelay (Read delay) from maximum and decrement it.\n

                     Rx_DLL
                      127
                        ▲
                        │
               Rx_High──┼──►+++++++++++++++++
                        │   +++++++++++++++++
             Fix Rx_DLL │   +++++++++++++++++
             3/4 of ────┼─► +++++++◄────┬───►
              Rx_High   │     ++++++++++│++++
               and      │      +++++++++│++++
              Rx_Low    │   x   ++++++++│++++
                        │   xx   +++++++│++++
                        │   xxx   ++++++│++++
                        │   xxxx   +++++│++++
                        │   xxxxx   ++++│++++
                        │   xxxxxx   +++│++++
                        │   xxxxxxx   ++│++++
                Rx_Low──┼──►xxxxxxxx   +│++++
                        │               │
                       ─┼───────────────┼────► Tx_DLL
                       0│               │       127
                                     Search Tx_Max
\n
- **Step 11 Find bottom left and top right corners**\n
    These are theoretical corners. They may not actually be "good" points.\n
    But the longest diagonal of the shmoo will be between these corners.\n
\n
- **Step 12 Find the tuning point**
    - **Step1** Find the equation of diagonal between topRight(rxHigh,txHigh) and bottomLeft(rxLow,txLow) points.
    - **Step2** Find gapLow, last point along the slope of bottom left read delay region.
    - **Step3** If top right and bottom left have same read delay,  put tuning point in the middle and adjust for temperature.
    - **Step4** If top right and bottom left have different read delays, find gapHigh, the starting point along the slope of top left read delay region.
    - **Step5** Find len1 = gapLow - topLeft, len2 = topRight - gapHigh.
    - **Step6** Choose the read Delay region with maximum length.
    - **Step7** Place the Phy tuning point in the corner farthest from the gap.



## Features not Supported

- Interrupt mode is not supported yet.

## Example Usage

Include the below file to access the APIs
\snippet Ospi_sample.c include

Instance Open Example
\snippet Ospi_sample.c open

Instance Close Example
\snippet Ospi_sample.c close

## API

\ref DRV_OSPI_MODULE