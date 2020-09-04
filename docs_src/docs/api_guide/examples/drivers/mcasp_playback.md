# MCASP Playback {#EXAMPLES_DRIVERS_MCASP_PLAYBACK}

[TOC]

# Introduction

This example demonstrates the McASP configured for an audio playback mode of operation. @VAR_BOARD_NAME_LOWER is used with @VAR_BOARD_NAME_LOWER daughter card. An audio source (e.g. cellphone with AUX cable) is connected to Line-In-1 of the daughter card and audio output device (e.g headphones) are connected to the Line-Out-4 of the daughter card. The audio signal from the source can be heard on a single channel at the output.

- Mcasp is configured in I2S mode
- Transfers data on single channel (left)

# Supported Combinations {#EXAMPLES_DRIVERS_MCASP_PLAYBACK_COMBOS}

\cond SOC_AM273X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | c66ss0 nortos
 Toolchain      | ti-arm-clang, ti-c6000
 Board          | AM273x EVM + AM273x audio daughter card
 Example folder | examples/drivers/mcasp/mcasp_playback

\endcond
# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Connect the MIPI header on AM273x EVM to audio daughter card with SAMTEC cable
- Connect audio devices as mentioned in the introduction
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref DRIVERS_MCASP_PAGE

# Sample Output

Shown below is a sample output when the application is run,
Please note that application prints in both CCS and UART console.
In UART console you need to enter 8 characters.

CCS Console:

\code
[MCASP] Audio playback example started.
Enter your response on UART terminal
Stop the demo? (y/n) :
\endcode

UART Console:
\code
[MCASP] Audio playback example started.
Enter your response on UART terminal
Stop the demo? (y/n) : y
\endcode
