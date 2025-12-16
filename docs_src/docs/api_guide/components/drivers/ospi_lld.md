# OSPI Low Level Driver {#DRIVERS_OSPI_LLD_PAGE}

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
- Support DMA Read.
- Interrupt mode is supported.


## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

- OSPI instance name
- Input clock frequency to be used for OSPI module
- Input clock divider which decides the baud-rate at which the flash will be read
- Chip Select
\cond !(SOC_AM263PX)
- Enabling of various features like DMA, PHY mode(not supported yet), XIP(not supported yet)
\endcond
\cond SOC_AM263PX
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

\cond !(SOC_AM64X) && !(SOC_AM243X)
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
\endcond

\cond SOC_AM64X || SOC_AM243X
The OSPI DQS tuning algorithm works as follows:
- **Step 1: Select the diagonal**
    - Select the diagonal from (0,0) to (127,127) to perform tuning point search. This diagonal represents a linear path through the delay space with equal increments in both X and Y dimensions.
    - If the search fails to find a valid tuning point along this initial diagonal, the algorithm shifts the diagonal up by 10 points (increasing Y-offset while maintaining the same slope) to perform another tuning point search.
    - If the search still fails, the diagonal is shifted right by 10 points (increasing X-offset while maintaining the same slope) for another tuning point search.
    - These steps are repeated in an alternating pattern (up shift, then right shift) up to a maximum shift of 70 units from the first diagonal. Each shifted diagonal maintains a 45-degree slope, ensuring systematic coverage of the delay space.
    - This methodical search pattern optimizes the chances of finding a valid tuning point while balancing thoroughness with efficiency in the calibration process.

    \imageStyle{ospi_diagonal_selection.png, width:40%}
    \image html ospi_diagonal_selection.png "Diagonal Selection"
\n
- **Step 2: Select valid read delays**
    - Set the read delay to its minimum value (usually 0) and perform a systematic search in steps of 16 to identify regions where valid reads are possible.
    - At each step, perform validation read operations to check if the current delay value can successfully read data without errors.
    - A "passing point" simply indicates that a particular read delay value works correctly - it confirms that a valid operating region exists at that delay setting.
    - Once any passing point is found, increment the read delay in steps of 1 to map out the entire valid region.
    - Continue testing across the full range of delay values to identify all valid regions.
    - If no passing points are found across the entire range of read delay values, report failure and shift the diagonal (adjust both sample and read delays together) to find a new potential operating point.
    - The calibration process aims to identify the optimal read delay values.

    \imageStyle{ospi_read_delay_selection.png, width:40%}
    \image html ospi_read_delay_selection.png "Read Delay Selection"
\n
- **Step 3: Find the corner points**
    - Set the read delay to the minimum valid value found in Step 2 and find the first passing point along the diagonal from the diagonal start point by incrementing rxDLL and txDLL delay in steps of 1. This point represents the lower boundary of your valid operating window.
    - Next, set the read delay to the maximum valid value found in Step 2 and search for a passing point along the diagonal from the diagonal end point by decrementing rxDLL and txDLL delay in steps of 1. This point represents the upper boundary of your valid operating window. The goal is to identify the complete range of delay values where the memory interface operates reliably.

    \imageStyle{ospi_corner_point_selection.png, width:40%}
    \image html ospi_corner_point_selection.png "Corner Point Selection"
    \n
    - **Case 1: Only one read delay value**
        - If only one read delay was identified in Step 2, find the corner points of the line with the highest number of consecutive points. These corner points represent the beginning and end of a stable sampling window where data can be reliably read.
        - Check if the length of the line (the distance between corner points) is greater than the minimum pass length. The minimum pass length is a threshold that ensures the sampling window is wide enough for reliable operation. If the line length exceeds this threshold, proceed with tuning point search; otherwise, report a calibration failure since the stable window is too narrow for reliable operation.

        \imageStyle{ospi_only_one_read_delay.png, width:40%}
        \image html ospi_only_one_read_delay.png "Corner Point Selection for One Read Delay"
    \n
    - **Case 2: Two different read delay values**
        - For two different read delay regions, refine the corner points to find points with minimum consecutive passing points as defined. This process involves testing multiple adjacent points in the delay/sample space to ensure stability rather than using single isolated points.
        - For each valid point found, identify the first failing point for the read delay and further refine to find the point with minimum consecutive failing points. This creates a clear boundary between passing and failing regions, improving tuning reliability.
        - If no point with consecutive passing point is found, eliminate the read delay region as it's considered unstable for reliable operation and continue with remaining regions.
        - For minimum read delay, search points on the diagonal in the upward direction (increasing rxDLL and txDLL delays simultaneously); for maximum read delay, search in the downward direction (decreasing rxDLL and txDLL delays). This diagonal search optimizes for finding stable operating regions.
        - After finding the refined corner points, identify the corner points of the line with the largest number of passing points. This ensures selection of the most robust operating region with maximum margin.
        - Calculate the length of both lines in terms of delay steps. If either length exceeds the minimum pass length, proceed to find the tuning point within this region; otherwise, report calibration failure.

        \imageStyle{ospi_two_different_read_delay.png, width:40%}
        \image html ospi_two_different_read_delay.png "Corner Point Selection for Two Different Read Delays"
\n
- **Step 4: Tuning point selection**
    - Select the read delay region with the larger length and calculate its midpoint (midpoint1). This region represents the most stable range of delay values where reads succeed consistently.
    - Find the corner points of the line defined by consecutive passing points perpendicular to the line at midpoint1. This perpendicular line crosses the stable region and helps identify the optimal sampling point by measuring across the width of the passing region.
    - Calculate the midpoint (midpoint2) of the perpendicular line. This point represents the center of the stable region's width at the position determined by midpoint1. The algorithm then performs a radius check to verify if all points within a circle of specified radius around midpoint2 are passing. This ensures the selected point has sufficient margin in all directions.
    - If the radius check succeeds, assign midpoint2 as the tuning point and return success. This means we've found a delay value with adequate margins, providing the best reliability for data transfers.

    \imageStyle{ospi_tuning_point_selection.png, width:40%}
    \image html ospi_tuning_point_selection.png "Tuning Point Selection at Midpoint"
    \n
    - If the radius check fails, divide the perpendicular line into two segments at midpoint1, select the larger segment, calculate its midpoint (midpoint3), and perform radius check. If successful, assign midpoint3 as the tuning point. This segmentation approach systematically narrows down the search area to find the optimal tuning point that satisfies the radius constraints.

    \imageStyle{ospi_tuning_point_selection_2.png, width:40%}
    \image html ospi_tuning_point_selection_2.png "Tuning Point Selection at Midpoint of Larger Segment"
    \n
    - If radius checks fail for both midpoint2 and midpoint3, check if the length of the line in the second read delay region exceeds the minimum pass length     . If yes, repeat the tuning point selection steps for this region, applying the same methodology to find a viable tuning point.
    - The second region analysis follows the same principles but operates on a different segment of the tuning space, providing an alternative area to find a stable operating point.
    - If no passing points are found after exhausting all possible regions and segments, the algorithm reports a tuning failure and selects a different diagonal for analysis. This diagonal selection follows a predefined pattern to ensure comprehensive coverage of the tuning space.
    - Each diagonal represents a different combination of read and write delay parameters, offering multiple opportunities to find a stable operating region.\n

\endcond

## OSPI LLD Migration Guide {#OSPI_LLD_MIGRATION_GUIDE}

### 11.02.00 Changes

<table>
    <tr>
        <th> API
        <th> Remarks
    </tr>
    <tr>
        <td>\ref OSPI_lld_phyTuneGrapher
        <td>Param arrays[4][128][128] updated to arrays[5][128][128]
    </tr>
</table>

## Features not Supported

## Example Usage

Include the below file to access the APIs
\snippet Ospi_lld_sample.c include

Instance Open Example
\snippet Ospi_lld_sample.c open

Non-Blocking Example ISR Register
\snippet Ospi_lld_sample.c isr_call

Non-Blocking Transfer Example
\snippet Ospi_lld_sample.c transfer_nonblocking

Instance Close Example
\snippet Ospi_lld_sample.c close

## API

\ref DRV_OSPI_LLD_MODULE