# Timer {#KERNEL_DPL_TIMER_PAGE}

[TOC]

## Features Supported
- APIs to setup timer configuration
- APIs to start and stop the timer
- APIs to get the current timer count
- APIs to check for counter overflow and clear timer overflow interrupt

## Features NOT Supported

NA

## Important Usage Guidelines
\cond SOC_AM64X || SOC_AM243X
- Support configuration for DM timer
\endcond
\cond SOC_AWR294X || SOC_AM273X || SOC_AM263X || SOC_AM263PX || SOC_AM261X
- Support configuration for RTI timer
- Value for 'inputPreScaler' should be set to 1
 - RTI timer does not have a clock prescaler. However, RTI supports two 32 bit counters (UC and FRC) per block, out of which UC can be used as a prescaler to run FRC.
 - RTI based TimerP driver makes use of both UC and FRC to achieve the required tick period and generate interrupt. So in this design RTICLK cannot be prescaled. RTI TimerP driver assumes prescaler is 1 and the value provided in 'inputPreScaler' is not used.
\endcond
- Supports continues and oneshot mode
- When configuring the timer period in micro second the 'periodInNsec' should be set to 0


## Example Usage

Include the below file to access the APIs,
\snippet TimerP_sample.c include

Example usage to initialize the timer
\snippet TimerP_sample.c initialize

Example usage to start the timer
\snippet TimerP_sample.c start

Example usage to get the current count value
\snippet TimerP_sample.c curCount

Example usage to stop the timer
\snippet TimerP_sample.c stop

## API

\ref KERNEL_DPL_TIMER