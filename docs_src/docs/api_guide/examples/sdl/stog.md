# SDL STOG Example {#EXAMPLES_SDL_STOG}

[TOC]

# Introduction
\cond SOC_AM64X || SOC_AM243X
This example demonstrates the usage of the TOG module. The example shows how to listen for TOG error events using the ESM, and also forcing an error in order to demonstrate application notification of a timeout event.
\endcond

\cond SOC_AM263PX
This example demonstrates the usage of the TOG module. The example shows how to listen for TOG error events using the interrupt, and also forcing an error in order to demonstrate application notification of a timeout event.
\endcond

The example demonstrates:

1. Setting a timeout for TOG instance
2. Error injection for an TOG instance
3. Error event detection for a timeout event

Use Cases
---------
* The TOG is programmed with a timeout that is small enough to trigger a timeout event on any access. Error event is triggered and application is notified of the event.


# Supported Combinations {#EXAMPLES_SDL_STOG_COMBOS}

\cond SOC_AM64X || SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | m4fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/sdl/stog_r5f/

\endcond

\cond SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | r5fss1-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/sdl/stog/

\endcond




# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
\cond SOC_AM64X || SOC_AM243X
- **STOG_r5f**  STOG example for r5f core, First R5F binary should be loaded and run followed by M4F binary.
\endcond

# See Also

\ref SDL_STOG_PAGE

# Sample Output

\cond SOC_AM64X || SOC_AM243X

\code{.c}
For M4F CORE

TOG Sample Example

Init MCU ESM complete

Init MAIN ESM complete

SDL_TOG_setIntrEnable complete

SDL_TOG_init.timeout complete

SDL_TOG_start complete

SDL_TOG_stop complete

All tests have passed.

\endcode

\code{.c}
FOR R5F CORE

TOG Sample Example

 Init MCU ESM complete

 Init MAIN ESM complete

SDL_TOG_setIntrEnable complete

SDL_TOG_init.timeout complete

SDL_TOG_start complete

Waiting for reading END_POINT_ACCESS by M4F core...

SDL_TOG_stop complete

All tests have passed.

\endcode
\endcond

\cond SOC_AM263PX

\code{.c}

TOG Sample Example

SDL_TOG_setIntrEnable complete

SDL_TOG_init.timeout complete

SDL_TOG_start complete

Data Abort exception

TOG Interrupt received

SDL_TOG_stop complete

All tests have passed.

\endcode
\endcond
