# CCM {#SDL_CCM_PAGE}

[TOC]

## Features Supported

The Compute Compare Module (CCM) is the R5F sub-system’s implementation for the lockstep error detection logic. The R5F sub-system, when bootstrapped to lockstep mode, has a logic that compares the outputs of the two cores and asserts an interrupt whenever an error is detected. There is also a logic to test the comparison logic.

The CCM module provides APIs to:

* Initialize the R5F CCM for the operating modes and verify the config

* Perform Self-test of the supported operating modes (normal and error forcing)

* Inject an error

* Read back the static config registers

* Clear errors that have occurred due to error injection or actual fault


## SysConfig Features

- None

## Features NOT Supported

- None

## Important Usage Guidelines

- None

## Example Usage

The following shows an example of SDL R5F CCM API usage by the application.

Initialize the R5F CCM instance
\code{.c}
    retVal = SDL_CCM_init(SDL_R5SS0_CCM);
    if (result != SDL_PASS)
    {
        DebugP_log("CCM_Test_init: Error result = %d\n", result);
        retValue = -1;
    }
    else
    {
        DebugP_log("\nCCM_Test_init: CCM Init complete \n");
    }
\endcode

Verify the configuration
\code{.c}
    result = SDL_CCM_verifyConfig(0);
    if (result != SDL_PASS)
    {
        DebugP_log("R5F-CPU example application CCM_Test_init: Error result = %d\n", result);
        retValue = -1;
    }
    else
    {
        DebugP_log("\nR5F-CPU example application CCM_Test_init: CCM Init complete \n");
    }
\endcode

Perform a self-test:
\code{.c}
    result = SDL_CCM_selfTest(SDL_R5SS0_CCM,
	                          SDL_CCM_MONITOR_TYPE_OUTPUT_COMPARE_BLOCK,
                              SDL_CCM_SELFTEST_TYPE_NORMAL, 0U,
                              10000000);

    if (result != SDL_PASS )
    {
        DebugP_log("\n R5F-CPU example application CCM self test failed");
        retVal = -1;
    }
    else
    {
        DebugP_log("\n R5F-CPU example application CCM Self Test complete");
    }
\endcode

Inject an error:
\code{.c}
    result = SDL_CCM_injectError(SDL_R5SS0_CCM, SDL_CCM_MONITOR_TYPE_OUTPUT_COMPARE_BLOCK);

    if (result != SDL_PASS ) {
        DebugP_log("\n R5F-CPU example application CCM inject failed");
       retVal = -1;
    } else {
        DebugP_log("\n R5F-CPU example application CCM inject Test complete");
    }
\endcode

## API

\ref SDL_CCM_MODULE
