# ECC : Error Correcting Code {#SDL_ECC_PAGE}

[TOC]

To increase functional and system reliability, the memories in many device modules and subsystems are protected by Error Correcting Code (ECC), which performs Single Error Correction (SEC) and Double Error Detection (DED). Detected errors are reported via ESM. Single bit errors are corrected, and double bit errors are detected. The ECC Aggregator is connected to these memory and interconnect components which have the ECC. The ECC aggregator provides access to control and monitor the ECC protected memories in a module or subsystem.

SDL provides support for ECC aggregator configuration. Each ECC aggregator instance can be independently configured through the same SDL API by passing a different instance. The safety manual also defines test-for-diagnostics for the various IPs with ECC/parity support. The SDL also provides the support for executing ECC aggregator self-tests, using the error injection feature of the ECC aggregator.
The ECC aggregators should be configured at startup, after running BIST.

## Features Supported

The SDL provides support for the ECC through:

* ECC Configuration API
* ECC self-test API
* ECC error injection API
* ECC static register readback API
* ECC error status APIs

	The SDL ECC module requires a mapping of certain aggregator registers into the address space of the R5F Core. In these cases, the ECC module will use the DPL API SDL_DPL_addrTranslate() to get the address. The application must provide the mapped address through this call. This mapping is required for any ECC aggregator that is used which has an address which is not in the 32-bit address space.
The mapping is expected to always be valid because it may be needed at runtime to get information about ECC errors that may be encountered.

\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
	There are over 13 ECC aggregators on the device each supporting multiple memories and interconnects.
\endcond

\cond SOC_AM273X || SOC_AWR294X
	There are over 8 ECC aggregators on the device each supporting multiple memories and interconnects.
\endcond

\cond SOC_AM64X || SOC_AWR243X
	There are over 40 ECC aggregators on the device each supporting multiple memories and interconnects.
\endcond

## Error Injection for Various RAM ID types

	There are two types of ECC aggregator RAM IDs supported on the device (wrapper and interconnect). The wrapper types are used for memories where local computations are performed for particular processing cores in the device, and the interconnect types are utilized for interconnect bus signals between cores or to/from peripherals.
For wrapper RAM ID types, after injecting an error, the memory associated with that RAM ID needs to be accessed in order to trigger the error interrupt event. It is the application's responsibility to trigger the error event through memory access after injecting the error.


## SysConfig Features

- None

## Features NOT Supported

- None

## Important Usage Guidelines

- None

## Example Usage of R5F ATCM0

The following shows an example of SDL R5F ECC API usage by the application for Error Injection Tests and Exception handling.

Include the below file to access the APIs

\code{.c}
#include <sdl/sdl_ecc.h>
#include <sdl/sdl_exception.h>
#include <sdl/r5/v0/interrupt.h>
#include "ecc_main.h"
\endcode

Below are the macros specifies the RAM address, ECC aggregator and ECC aggregator RAMID for inject the ECC error
\code{.c}
#define SDL_EXAMPLE_ECC_RAM_ADDR                    (0x00000510u) // R5F ATCM0 RAM address
#define SDL_EXAMPLE_ECC_AGGR                        SDL_R5FSS0_CORE0_ECC_AGGR
#define SDL_EXAMPLE_ECC_RAM_ID                      SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_ATCM0_BANK0_RAM_ID
\endcode

ESM callback function

\cond SOC_AM273X || SOC_AWR294X
\code{.c}
int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
                                           int32_t grpChannel,
                                           int32_t intSrc,
                                           void *arg)
{

    SDL_ECC_MemType eccmemtype;
    SDL_Ecc_AggrIntrSrc eccIntrSrc;
    SDL_ECC_ErrorInfo_t eccErrorInfo;
    int32_t retVal;
    printf("\r\nESM Call back function called : instType 0x%x, " \
                "grpChannel 0x%x, intSrc 0x%x \r\n",
                esmInstType, grpChannel, intSrc);
    printf("\r\nTake action \r\n");

    retVal = SDL_ECC_getESMErrorInfo(esmInstType, intSrc, &eccmemtype, &eccIntrSrc);

    /* Any additional customer specific actions can be added here */
    retVal = SDL_ECC_getErrorInfo(eccmemtype, eccIntrSrc, &eccErrorInfo);

    printf("\r\nECC Error Call back function called : eccMemType %d, errorSrc 0x%x, " \
               "ramId %d, bitErrorOffset 0x%04x%04x, bitErrorGroup %d\r\n",
               eccmemtype, eccIntrSrc, eccErrorInfo.memSubType, (uint32_t)(eccErrorInfo.bitErrorOffset >> 32),
               (uint32_t)(eccErrorInfo.bitErrorOffset & 0x00000000FFFFFFFF), eccErrorInfo.bitErrorGroup);

    if (eccErrorInfo.injectBitErrCnt != 0)
    {
        SDL_ECC_clearNIntrPending(eccmemtype, eccErrorInfo.memSubType, eccIntrSrc, SDL_ECC_AGGR_ERROR_SUBTYPE_INJECT, eccErrorInfo.injectBitErrCnt);
    }
    else
    {
        SDL_ECC_clearNIntrPending(eccmemtype, eccErrorInfo.memSubType, eccIntrSrc, SDL_ECC_AGGR_ERROR_SUBTYPE_NORMAL, eccErrorInfo.bitErrCnt);
    }

    retVal = SDL_ECC_ackIntr(eccmemtype, eccIntrSrc);

    esmError = true;

    return retVal;
}
\endcode
\endcond

\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
\code{.c}
int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInst,
                                            SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,
                                            uint32_t index,
                                            uint32_t intSrc,
                                            uintptr_t *arg)
{


    int32_t retVal = 0;
    uint32_t rd_data = 0;


    printf("\r\nESM Call back function called : instType 0x%x, intType 0x%x, " \
                "grpChannel 0x%x, index 0x%x, intSrc 0x%x \r\n",
                esmInst, esmIntrType, grpChannel, index, intSrc);
    printf("\r\nTake action \r\n");
    if(esmIntrType == 1u){
        printf("\r\nHigh Priority Interrupt Executed\r\n");
        /* Clear DED MSS_CTRL register*/
        SDL_REG32_WR(0x50D18094u, 0x01);
        rd_data = SDL_REG32_RD(0x50D18094u);
        printf("\r\nRead data of DED MSS_CTRL register is 0x%u\r\n",rd_data);
        /* Clear DED RAW MSS_CTRL register*/
        SDL_REG32_WR(0x50D18098u, 0x01);
        rd_data = SDL_REG32_RD(0x50D18098u);
        printf("\r\nRead data of DED RAW MSS_CTRL register is 0x%u\r\n",rd_data);
    }
    else{
        printf("\r\nLow Priority Interrupt Executed\r\n");
        /* Clear SEC MSS_CTRL register*/
        SDL_REG32_WR(0x50D18088u, 0x01);
        rd_data = SDL_REG32_RD(0x50D18088u);
        printf("\r\nRead data of SEC MSS_CTRL register is  0x%u\r\n",rd_data);
        /* Clear SEC RAW MSS_CTRL register*/
        SDL_REG32_WR(0x50D18084u, 0x01);
        rd_data = SDL_REG32_RD(0x50D18084u);
        printf("\r\nRead data of SEC RAW MSS_CTRL register is 0x%u\r\n",rd_data);
    }

    esmError = true;

    return retVal;
}
\endcode
\endcond

This is the list of exception handle and the parameters
\code{.c}
const SDL_R5ExptnHandlers ECC_Test_R5ExptnHandlers =
{
    .udefExptnHandler = &SDL_EXCEPTION_undefInstructionExptnHandler,
    .swiExptnHandler = &SDL_EXCEPTION_swIntrExptnHandler,
    .pabtExptnHandler = &SDL_EXCEPTION_prefetchAbortExptnHandler,
    .dabtExptnHandler = &SDL_EXCEPTION_dataAbortExptnHandler,
    .irqExptnHandler = &SDL_EXCEPTION_irqExptnHandler,
    .fiqExptnHandler = &SDL_EXCEPTION_fiqExptnHandler,
    .udefExptnHandlerArgs = ((void *)0u),
    .swiExptnHandlerArgs = ((void *)0u),
    .pabtExptnHandlerArgs = ((void *)0u),
    .dabtExptnHandlerArgs = ((void *)0u),
    .irqExptnHandlerArgs = ((void *)0u),
};
\endcode

Below are the functions used to print the which exception is occured
\code{.c}
void ECC_Test_undefInstructionExptnCallback(void)
{
    printf("\r\nUndefined Instruction exception\r\n");
}

void ECC_Test_swIntrExptnCallback(void)
{
    printf("\r\nSoftware interrupt exception\r\n");
}

void ECC_Test_prefetchAbortExptnCallback(void)
{
    printf("\r\nPrefetch Abort exception\r\n");
}
void ECC_Test_dataAbortExptnCallback(void)
{
    printf("\r\nData Abort exception\r\n");
}
void ECC_Test_irqExptnCallback(void)
{
    printf("\r\nIrq exception\r\n");
}

void ECC_Test_fiqExptnCallback(void)
{
    printf("\r\nFiq exception\r\n");
}
\endcode

Initilize Exception handler
\code{.c}
void ECC_Test_exceptionInit(void)
{

    SDL_EXCEPTION_CallbackFunctions_t exceptionCallbackFunctions =
            {
             .udefExptnCallback = ECC_Test_undefInstructionExptnCallback,
             .swiExptnCallback = ECC_Test_swIntrExptnCallback,
             .pabtExptnCallback = ECC_Test_prefetchAbortExptnCallback,
             .dabtExptnCallback = ECC_Test_dataAbortExptnCallback,
             .irqExptnCallback = ECC_Test_irqExptnCallback,
             .fiqExptnCallback = ECC_Test_fiqExptnCallback,
            };

    /* Initialize SDL exception handler */
    SDL_EXCEPTION_init(&exceptionCallbackFunctions);
    /* Register SDL exception handler */
    Intc_RegisterExptnHandlers(&ECC_Test_R5ExptnHandlers);

    return;
}
\endcode

This structure defines the elements of ECC  Init configuration
\code{.c}
static SDL_ECC_MemSubType ECC_Test_R5FSS0_CORE0_subMemTypeList[SDL_R5FSS0_CORE0_MAX_MEM_SECTIONS] =
{
     SDL_EXAMPLE_ECC_RAM_ID,
};

static SDL_ECC_InitConfig_t ECC_Test_R5FSS0_CORE0_ECCInitConfig =
{
    .numRams = SDL_R5FSS0_CORE0_MAX_MEM_SECTIONS,
    /**< Number of Rams ECC is enabled  */
    .pMemSubTypeList = &(ECC_Test_R5FSS0_CORE0_subMemTypeList[0]),
    /**< Sub type list  */
};
\endcode

\cond SOC_AM273X || SOC_AWR294X
Event BitMap for ECC ESM callback for MSS
\code{.c}
SDL_ESM_NotifyParams ECC_TestparamsMSS[SDL_ESM_MAX_MSS_EXAMPLE_AGGR] =
{
     {
          /* Event BitMap for ECC ESM callback for R5FA Single bit*/
          .groupNumber = SDL_INTR_GROUP_NUM_1,
          .errorNumber = SDL_ESMG1_ATCM0_SERR,
          .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
          .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
          .callBackFunction = &SDL_ESM_applicationCallbackFunction,
     }
};
\endcode
\endcond
\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
\code{.c}
SDL_ESM_config ECC_Test_esmInitConfig_MAIN =
{
     .esmErrorConfig = {1u, 8u}, /* Self test error config */
     .enableBitmap = {0x00000000u, 0x00018000u, 0x00000000u, 0x00000000u,
                      0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u},
      /**< All events enable: except clkstop events for unused clocks
       *   and PCIE events */
       /* CCM_1_SELFTEST_ERR and _R5FSS0COMPARE_ERR_PULSE_0 */
     .priorityBitmap = {0x00000000u, 0x000010000u, 0x00000000u, 0x00000000u,
                        0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u },
     /**< All events high priority: except clkstop events for unused clocks
      *   and PCIE events */
     .errorpinBitmap = {0x00000000u, 0x00018000u, 0x00000000u, 0x00000000u,
                        0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u},
     /**< All events high priority: except clkstop for unused clocks
      *   and PCIE events */
};
\endcode
\endcond

Enabling the ECC module
\code{.c}
SDL_ECC_UTILS_enableECCATCM();
\endcode

Enabling the Event bus
\code{.c}
SDL_UTILS_enable_event_bus();
\endcode

Initialize ECC memory for the ECC aggregator
\code{.c}
result = SDL_ECC_initMemory(SDL_EXAMPLE_ECC_AGGR, SDL_EXAMPLE_ECC_RAM_ID);
\endcode

\cond SOC_AM273X || SOC_AWR294X
Initialize ESM module
\code{.c}
result = SDL_ESM_init(SDL_ESM_INST_MSS_ESM, &ECC_TestparamsMSS[u8ParamCount],NULL,NULL);
\endcode

Writing '000' will ungate the ESM Group3 (0 to 7) errors  for dounle bit ATCM
\code{.c}
SDL_REG32_WR(SDL_MSS_CTRL_ESM_GATING4, (0x0 << (((SDL_ESMG3_ATCM0_UERR ) % 8)*4)));
\endcode
\endcond

\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
Initialize ESM module
\code{.c}
result = SDL_ESM_init(SDL_ESM_INST_MAIN_ESM0, &ECC_Test_esmInitConfig_MAIN, SDL_ESM_applicationCallbackFunction, ptr);
\endcode
\endcond

Initialize ECC parameters for single and double bit error injection
\code{.c}
result = SDL_ECC_init(SDL_EXAMPLE_ECC_AGGR, &ECC_Test_R5FSS0_CORE0_ECCInitConfig);
\endcode

Execute ECC R5F ATCM0 single bit inject test
\code{.c}
int32_t ECC_Test_run_R5FSS0_CORE0_ATCM0_BANK0_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

	DebugP_log("\r\nR5FSS0 CORE0 ATCM0 BANK0 Single bit error inject: starting \r\n");

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(SDL_EXAMPLE_ECC_RAM_ADDR);

    /* Run one shot test for R5FSS0 CORE0 ATCM0 BANK0 1 bit error */
    injectErrorConfig.flipBitMask = 0x02;
    result = SDL_ECC_injectError(SDL_EXAMPLE_ECC_AGGR,
                                 SDL_EXAMPLE_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                 &injectErrorConfig);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];

        DebugP_log("\r\nR5FSS0 CORE0 ATCM0 BANK0 Single bit error inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete\r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}
\endcode

\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
Enabling the ECC module
\code{.c}
SDL_ECC_UTILS_enableECCATCM();
\endcode

Enabling the Event bus
\code{.c}
SDL_UTILS_enable_event_bus();
\endcode
\endcond

Execute ECC R5F ATCM0 double bit inject test
\code{.c}
int32_t ECC_Test_run_R5FSS0_CORE0_ATCM0_BANK0_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

	DebugP_log("\r\nR5FSS0 CORE0 ATCM0 BANK0 Double bit error inject: starting \r\n");

    /* Run one shot test for R5FSS0 CORE0 ATCM0 BANK0 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(SDL_EXAMPLE_ECC_RAM_ADDR);

    injectErrorConfig.flipBitMask = 0x30002;
    result = SDL_ECC_injectError(SDL_EXAMPLE_ECC_AGGR,
                                 SDL_EXAMPLE_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);

    if (result != SDL_PASS ) {
       retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nR5FSS0 CORE0 ATCM0 BANK0 Double bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p\r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}
\endcode

## Example Usage of R5F BTCM

The following shows an example of SDL R5F ECC API usage by the application for Error Injection Tests and Exception handling.

Include the below file to access the APIs

\code{.c}
#include <sdl/sdl_ecc.h>
#include <sdl/sdl_exception.h>
#include <sdl/r5/v0/interrupt.h>
#include "ecc_main.h"
\endcode

Below are the macros specifies the RAM address, ECC aggregator and ECC aggregator RAMID for inject the ECC error
\code{.c}
#if SDL_B0TCM0_BANK0
#define SDL_R5FSS0_CORE0_MAX_MEM_SECTIONS           (1u)
#define SDL_EXAMPLE_ECC_RAM_ADDR                    (0x00080010u) /* R5F BTCM RAM address */
#define SDL_EXAMPLE_ECC_AGGR                        SDL_R5FSS0_CORE0_ECC_AGGR
#define SDL_EXAMPLE_ECC_RAM_ID                      SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B0TCM0_BANK0_RAM_ID
#endif

#if SDL_B0TCM0_BANK1
#define SDL_R5FSS0_CORE0_MAX_MEM_SECTIONS           (1u)
#define SDL_EXAMPLE_ECC_RAM_ADDR                    (0x00081510u) /* R5F BTCM RAM address */
#define SDL_EXAMPLE_ECC_AGGR                        SDL_R5FSS0_CORE0_ECC_AGGR
#define SDL_EXAMPLE_ECC_RAM_ID                      SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B0TCM0_BANK1_RAM_ID
#endif

#if SDL_B1TCM0_BANK0
#define SDL_R5FSS0_CORE0_MAX_MEM_SECTIONS           (1u)
#define SDL_EXAMPLE_ECC_RAM_ADDR                    (0x00082510u) /* R5F BTCM RAM address */
#define SDL_EXAMPLE_ECC_AGGR                        SDL_R5FSS0_CORE0_ECC_AGGR
#define SDL_EXAMPLE_ECC_RAM_ID                      SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B1TCM0_BANK0_RAM_ID
#endif

#if SDL_B1TCM0_BANK1
#define SDL_R5FSS0_CORE0_MAX_MEM_SECTIONS           (1u)
#define SDL_EXAMPLE_ECC_RAM_ADDR                    (0x00083510u) /* R5F BTCM RAM address */
#define SDL_EXAMPLE_ECC_AGGR                        SDL_R5FSS0_CORE0_ECC_AGGR
#define SDL_EXAMPLE_ECC_RAM_ID                      SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B1TCM0_BANK1_RAM_ID
#endif
\endcode

ESM callback function

\cond SOC_AM273X || SOC_AWR294X
\code{.c}
int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
                                           int32_t grpChannel,
                                           int32_t intSrc,
                                           void *arg)
{

    SDL_ECC_MemType eccmemtype;
    SDL_Ecc_AggrIntrSrc eccIntrSrc;
    SDL_ECC_ErrorInfo_t eccErrorInfo;
    int32_t retVal;
    printf("\r\nESM Call back function called : instType 0x%x, " \
                "grpChannel 0x%x, intSrc 0x%x \r\n",
                esmInstType, grpChannel, intSrc);
    printf("\r\nTake action \r\n");

    retVal = SDL_ECC_getESMErrorInfo(esmInstType, intSrc, &eccmemtype, &eccIntrSrc);

    /* Any additional customer specific actions can be added here */
    retVal = SDL_ECC_getErrorInfo(eccmemtype, eccIntrSrc, &eccErrorInfo);

    printf("\r\nECC Error Call back function called : eccMemType %d, errorSrc 0x%x, " \
               "ramId %d, bitErrorOffset 0x%04x%04x, bitErrorGroup %d\r\n",
               eccmemtype, eccIntrSrc, eccErrorInfo.memSubType, (uint32_t)(eccErrorInfo.bitErrorOffset >> 32),
               (uint32_t)(eccErrorInfo.bitErrorOffset & 0x00000000FFFFFFFF), eccErrorInfo.bitErrorGroup);

    if (eccErrorInfo.injectBitErrCnt != 0)
    {
        SDL_ECC_clearNIntrPending(eccmemtype, eccErrorInfo.memSubType, eccIntrSrc, SDL_ECC_AGGR_ERROR_SUBTYPE_INJECT, eccErrorInfo.injectBitErrCnt);
    }
    else
    {
        SDL_ECC_clearNIntrPending(eccmemtype, eccErrorInfo.memSubType, eccIntrSrc, SDL_ECC_AGGR_ERROR_SUBTYPE_NORMAL, eccErrorInfo.bitErrCnt);
    }

    retVal = SDL_ECC_ackIntr(eccmemtype, eccIntrSrc);

    esmError = true;

    return retVal;
}
\endcode
\endcond

\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
\code{.c}
int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInst,
                                            SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,
                                            uint32_t index,
                                            uint32_t intSrc,
                                            uintptr_t *arg)
{


    int32_t retVal = 0;
    uint32_t rd_data = 0;


    printf("\r\nESM Call back function called : instType 0x%x, intType 0x%x, " \
                "grpChannel 0x%x, index 0x%x, intSrc 0x%x \r\n",
                esmInst, esmIntrType, grpChannel, index, intSrc);
    printf("\r\nTake action \r\n");
    if(esmIntrType == 1u){
        printf("\r\nHigh Priority Interrupt Executed\r\n");
        /* Clear DED MSS_CTRL register*/
        SDL_REG32_WR(0x50D18094u, 0x06);
        rd_data = SDL_REG32_RD(0x50D18094u);
        printf("\r\nRead data of DED MSS_CTRL register is 0x%u\r\n",rd_data);
        /* Clear DED RAW MSS_CTRL register*/
        SDL_REG32_WR(0x50D18098u, 0x06);
        rd_data = SDL_REG32_RD(0x50D18098u);
        printf("\r\nRead data of DED RAW MSS_CTRL register is 0x%u\r\n",rd_data);
    }
    else{
        printf("\r\nLow Priority Interrupt Executed\r\n");
        /* Clear SEC MSS_CTRL register*/
        SDL_REG32_WR(0x50D18088u, 0x6);
        rd_data = SDL_REG32_RD(0x50D18088u);
        printf("\r\nRead data of SEC MSS_CTRL register is  0x%u\r\n",rd_data);
        /* Clear SEC RAW MSS_CTRL register*/
        SDL_REG32_WR(0x50D18084u, 0x6);
        rd_data = SDL_REG32_RD(0x50D18084u);
        printf("\r\nRead data of SEC RAW MSS_CTRL register is 0x%u\r\n",rd_data);
    }

    esmError = true;

    return retVal;
}
\endcode
\endcond

This is the list of exception handle and the parameters
\code{.c}
const SDL_R5ExptnHandlers ECC_Test_R5ExptnHandlers =
{
    .udefExptnHandler = &SDL_EXCEPTION_undefInstructionExptnHandler,
    .swiExptnHandler = &SDL_EXCEPTION_swIntrExptnHandler,
    .pabtExptnHandler = &SDL_EXCEPTION_prefetchAbortExptnHandler,
    .dabtExptnHandler = &SDL_EXCEPTION_dataAbortExptnHandler,
    .irqExptnHandler = &SDL_EXCEPTION_irqExptnHandler,
    .fiqExptnHandler = &SDL_EXCEPTION_fiqExptnHandler,
    .udefExptnHandlerArgs = ((void *)0u),
    .swiExptnHandlerArgs = ((void *)0u),
    .pabtExptnHandlerArgs = ((void *)0u),
    .dabtExptnHandlerArgs = ((void *)0u),
    .irqExptnHandlerArgs = ((void *)0u),
};
\endcode

Below are the functions used to print the which exception is occured
\code{.c}
void ECC_Test_undefInstructionExptnCallback(void)
{
    printf("\r\nUndefined Instruction exception\r\n");
}

void ECC_Test_swIntrExptnCallback(void)
{
    printf("\r\nSoftware interrupt exception\r\n");
}

void ECC_Test_prefetchAbortExptnCallback(void)
{
    printf("\r\nPrefetch Abort exception\r\n");
}
void ECC_Test_dataAbortExptnCallback(void)
{
    printf("\r\nData Abort exception\r\n");
}
void ECC_Test_irqExptnCallback(void)
{
    printf("\r\nIrq exception\r\n");
}

void ECC_Test_fiqExptnCallback(void)
{
    printf("\r\nFiq exception\r\n");
}
\endcode

Initilize Exception handler
\code{.c}
void ECC_Test_exceptionInit(void)
{

    SDL_EXCEPTION_CallbackFunctions_t exceptionCallbackFunctions =
            {
             .udefExptnCallback = ECC_Test_undefInstructionExptnCallback,
             .swiExptnCallback = ECC_Test_swIntrExptnCallback,
             .pabtExptnCallback = ECC_Test_prefetchAbortExptnCallback,
             .dabtExptnCallback = ECC_Test_dataAbortExptnCallback,
             .irqExptnCallback = ECC_Test_irqExptnCallback,
             .fiqExptnCallback = ECC_Test_fiqExptnCallback,
            };

    /* Initialize SDL exception handler */
    SDL_EXCEPTION_init(&exceptionCallbackFunctions);
    /* Register SDL exception handler */
    Intc_RegisterExptnHandlers(&ECC_Test_R5ExptnHandlers);

    return;
}
\endcode

This structure defines the elements of ECC  Init configuration
\code{.c}
static SDL_ECC_MemSubType ECC_Test_R5FSS0_CORE0_subMemTypeList[SDL_R5FSS0_CORE0_MAX_MEM_SECTIONS] =
{
     SDL_EXAMPLE_ECC_RAM_ID,
};

static SDL_ECC_InitConfig_t ECC_Test_R5FSS0_CORE0_ECCInitConfig =
{
    .numRams = SDL_R5FSS0_CORE0_MAX_MEM_SECTIONS,
    /**< Number of Rams ECC is enabled  */
    .pMemSubTypeList = &(ECC_Test_R5FSS0_CORE0_subMemTypeList[0]),
    /**< Sub type list  */
};
\endcode

\cond SOC_AM273X || SOC_AWR294X
Event BitMap for ECC ESM callback for MSS
\code{.c}
/* Event BitMap for ECC ESM callback for MSS */
SDL_ESM_NotifyParams ECC_TestparamsMSS[SDL_ESM_MAX_MSS_EXAMPLE_AGGR] =
{
    {
		/* Event BitMap for ECC ESM callback for R5FA Single bit*/
		.groupNumber = SDL_INTR_GROUP_NUM_1,
		.errorNumber = SDL_ESMG1_B0TCM0_SERR,
		.setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
		.enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
		.callBackFunction = &SDL_ESM_applicationCallbackFunction,
    },
	{
		/* Event BitMap for ECC ESM callback for R5FA Single bit*/
		.groupNumber = SDL_INTR_GROUP_NUM_1,
		.errorNumber = SDL_ESMG1_B1TCM0_SERR,
		.setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
		.enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
		.callBackFunction = &SDL_ESM_applicationCallbackFunction,
    }

};
\endcode
\endcond

\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
\code{.c}
SDL_ESM_config ECC_Test_esmInitConfig_MAIN =
{
     .esmErrorConfig = {1u, 8u}, /* Self test error config */
     .enableBitmap = {0x00000000u, 0x00018000u, 0x00000000u, 0x00000000u,
                      0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u},
      /**< All events enable: except clkstop events for unused clocks
       *   and PCIE events */
       /* CCM_1_SELFTEST_ERR and _R5FSS0COMPARE_ERR_PULSE_0 */
     .priorityBitmap = {0x00000000u, 0x00010000u, 0x00000000u, 0x00000000u,
                        0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u },
     /**< All events high priority: except clkstop events for unused clocks
      *   and PCIE events */
     .errorpinBitmap = {0x00000000u, 0x00018000u, 0x00000000u, 0x00000000u,
                        0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u},
     /**< All events high priority: except clkstop for unused clocks
      *   and PCIE events */
};
\endcode
\endcond

Enabling the ECC module
\code{.c}
#if (SDL_B0TCM0_BANK0) || (SDL_B0TCM0_BANK1)
    /*Enabling the B0TCM ECC module*/
	SDL_ECC_UTILS_enableECCB0TCM();
#endif
#if (SDL_B1TCM0_BANK0) || (SDL_B1TCM0_BANK1)
	    /*Enabling the B0TCM ECC module*/
	SDL_ECC_UTILS_enableECCB1TCM();
#endif
\endcode

Enabling the Event bus
\code{.c}
SDL_UTILS_enable_event_bus();
\endcode

Initialize ECC memory for the ECC aggregator
\code{.c}
result = SDL_ECC_initMemory(SDL_EXAMPLE_ECC_AGGR, SDL_EXAMPLE_ECC_RAM_ID);
\endcode

\cond SOC_AM273X || SOC_AWR294X
Initialize ESM module
\code{.c}
#if (SDL_B0TCM0_BANK0) || (SDL_B0TCM0_BANK1)
        result = SDL_ESM_init(SDL_ESM_INST_MSS_ESM, &ECC_TestparamsMSS[0],NULL,NULL);
		/*Writing '000' will ungate the ESM_GRP3_ERROR_7 for dounle bit BTCM*/
        SDL_REG32_WR(SDL_MSS_CTRL_ESM_GATING4, (0x0 << (((SDL_ESMG3_B0TCM0_UERR ) % 8)*4)));
#endif
#if (SDL_B1TCM0_BANK0) || (SDL_B1TCM0_BANK1)
        result = SDL_ESM_init(SDL_ESM_INST_MSS_ESM, &ECC_TestparamsMSS[1],NULL,NULL);
		/*Writing '000' will ungate the ESM_GRP3_ERROR_7 for dounle bit BTCM*/
        SDL_REG32_WR(SDL_MSS_CTRL_ESM_GATING4, (0x0 << (((SDL_ESMG3_B1TCM0_UERR ) % 8)*4)));
#endif
\endcode
\endcond

\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
Initialize ESM module
\code{.c}
result = SDL_ESM_init(SDL_ESM_INST_MAIN_ESM0, &ECC_Test_esmInitConfig_MAIN, SDL_ESM_applicationCallbackFunction, ptr);
\endcode
\endcond

Initialize ECC parameters for single and double bit error injection
\code{.c}
result = SDL_ECC_init(SDL_EXAMPLE_ECC_AGGR, &ECC_Test_R5FSS0_CORE0_ECCInitConfig);
\endcode

Execute ECC R5F BTCM single bit inject test
\code{.c}
int32_t ECC_Test_run_R5FSS0_CORE0_BTCM_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

	DebugP_log("\r\nR5FSS0 CORE0 BTCM Single bit error inject: starting \r\n");

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(SDL_EXAMPLE_ECC_RAM_ADDR);

    /* Run one shot test for R5FSS0 CORE0 BTCM 1 bit error */
    injectErrorConfig.flipBitMask = 0x02;
    result = SDL_ECC_injectError(SDL_EXAMPLE_ECC_AGGR,
                                 SDL_EXAMPLE_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                 &injectErrorConfig);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];

        DebugP_log("\r\nR5FSS0 CORE0 BTCM Single bit error inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete\r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}
\endcode

Execute ECC R5F BTCM double bit inject test
\code{.c}
int32_t ECC_Test_run_R5FSS0_CORE0_BTCM_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

	DebugP_log("\r\nR5FSS0 CORE0 BTCM Double bit error inject: starting \r\n");

    /* Run one shot test for R5FSS0 CORE0 BTCM 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(SDL_EXAMPLE_ECC_RAM_ADDR);

    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_EXAMPLE_ECC_AGGR,
                                 SDL_EXAMPLE_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);

    if (result != SDL_PASS ) {
       retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nR5FSS0 CORE0 BTCM Double bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p\r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}
\endcode


## Example Usage of MSS L2

The following shows an example of SDL MSS L2 API usage by the application for Error Injection Tests and Exception handling.

Include the below file to access the APIs

\code{.c}
#include <sdl/sdl_ecc.h>
#include "ecc_main.h"
\endcode

Below are the macros specifies the RAM address, ECC aggregator and ECC aggregator RAMID for inject the ECC error
\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
\code{.c}
#define SDL_EXAMPLE_ECC_RAM_ADDR                    (0x70100008u) /* MSS_L2_SLV2 address */
#define SDL_EXAMPLE_ECC_AGGR                        SDL_SOC_ECC_AGGR
#define SDL_EXAMPLE_ECC_RAM_ID                      SDL_SOC_ECC_AGGR_MSS_L2_SLV2_ECC_RAM_ID

#define SDL_MSS_L2_MEM_INIT_ADDR                    (0x50D00240u)
#define SDL_MSS_L2_MEM_INIT_DONE_ADDR               (0x50D00244u)
#define SDL_ECC_AGGR_ERROR_STATUS1_ADDR             (0x53000020u)
#define SDL_ECC_MSS_L2_BANK_MEM_INIT                (0xcu) /* Bank 3 */
\endcode
\endcond

\cond SOC_AM273X || SOC_AWR294X
\code{.c}
#define SDL_EXAMPLE_ECC_RAM_ADDR                    (0x10280000u) /* MSS_L2 RAMB address */
#define SDL_EXAMPLE_ECC_AGGR                        SDL_MSS_ECC_AGG_MSS
#define SDL_EXAMPLE_ECC_RAM_ID                      SDL_MSS_ECC_AGG_MSS_MSS_L2RAMB_ECC_RAM_ID

#define SDL_MSS_L2_MEM_INIT_ADDR					(0x02120034u)
#define SDL_MSS_L2_MEM_INIT_DONE_ADDR				(0x02120038u)
#define SDL_ECC_AGGR_ERROR_STATUS1_ADDR				(0x02F7c020u)
#define SDL_ECC_MSS_L2_BANK_MEM_INIT                (0x2u) /* Bank 2 */
\endcode
\endcond

ESM callback function
\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
\code{.c}
int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInst,
                                            SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,
                                            uint32_t index,
                                            uint32_t intSrc,
                                            uintptr_t *arg)
{

    SDL_ECC_MemType eccmemtype;
    SDL_Ecc_AggrIntrSrc eccIntrSrc;
    SDL_ECC_ErrorInfo_t eccErrorInfo;
    int32_t retVal;


    printf("\r\nESM Call back function called : instType 0x%x, intType 0x%x, " \
                "grpChannel 0x%x, index 0x%x, intSrc 0x%x\r\n",
                esmInst, esmIntrType, grpChannel, index, intSrc);
    printf(" \r\nTake action \r\n");
    if(esmIntrType == 1u){
        printf("\r\nHigh Priority Interrupt Executed\r\n");
    }
    else{
        printf("\r\nLow Priority Interrupt Executed\r\n");
    }
    retVal = SDL_ECC_getESMErrorInfo(esmInst, intSrc, &eccmemtype, &eccIntrSrc);

    /* Any additional customer specific actions can be added here */
    retVal = SDL_ECC_getErrorInfo(eccmemtype, eccIntrSrc, &eccErrorInfo);

    printf("\r\nECC Error Call back function called : eccMemType %d, errorSrc 0x%x, " \
               "ramId %d, bitErrorOffset 0x%04x%04x, bitErrorGroup %d\r\n",
               eccmemtype, eccIntrSrc, eccErrorInfo.memSubType, (uint32_t)(eccErrorInfo.bitErrorOffset >> 32),
               (uint32_t)(eccErrorInfo.bitErrorOffset & 0x00000000FFFFFFFF), eccErrorInfo.bitErrorGroup);

    if (eccErrorInfo.injectBitErrCnt != 0)
    {
        SDL_ECC_clearNIntrPending(eccmemtype, eccErrorInfo.memSubType, eccIntrSrc, SDL_ECC_AGGR_ERROR_SUBTYPE_INJECT, eccErrorInfo.injectBitErrCnt);
    }
    else
    {
        SDL_ECC_clearNIntrPending(eccmemtype, eccErrorInfo.memSubType, eccIntrSrc, SDL_ECC_AGGR_ERROR_SUBTYPE_NORMAL, eccErrorInfo.bitErrCnt);
    }

    retVal = SDL_ECC_ackIntr(eccmemtype, eccIntrSrc);

    esmError = true;

    return retVal;
}
\endcode
\endcond

\cond SOC_AM273X || SOC_AWR294X
\code{.c}
int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
                                           int32_t grpChannel,
                                           int32_t intSrc,
                                           void *arg)
{

    SDL_ECC_MemType eccmemtype;
    SDL_Ecc_AggrIntrSrc eccIntrSrc;
    SDL_ECC_ErrorInfo_t eccErrorInfo;
    int32_t retVal;
    printf("\r\nESM Call back function called : instType 0x%x, " \
                "grpChannel 0x%x, intSrc 0x%x \r\n",
                esmInstType, grpChannel, intSrc);
    printf("\r\nTake action \r\n");

    retVal = SDL_ECC_getESMErrorInfo(esmInstType, intSrc, &eccmemtype, &eccIntrSrc);

    /* Any additional customer specific actions can be added here */
    retVal = SDL_ECC_getErrorInfo(eccmemtype, eccIntrSrc, &eccErrorInfo);

    printf("\r\nECC Error Call back function called : eccMemType %d, errorSrc 0x%x, " \
               "ramId %d, bitErrorOffset 0x%04x%04x, bitErrorGroup %d\r\n",
               eccmemtype, eccIntrSrc, eccErrorInfo.memSubType, (uint32_t)(eccErrorInfo.bitErrorOffset >> 32),
               (uint32_t)(eccErrorInfo.bitErrorOffset & 0x00000000FFFFFFFF), eccErrorInfo.bitErrorGroup);

    if (eccErrorInfo.injectBitErrCnt != 0)
    {
        SDL_ECC_clearNIntrPending(eccmemtype, eccErrorInfo.memSubType, eccIntrSrc, SDL_ECC_AGGR_ERROR_SUBTYPE_INJECT, eccErrorInfo.injectBitErrCnt);
    }
    else
    {
        SDL_ECC_clearNIntrPending(eccmemtype, eccErrorInfo.memSubType, eccIntrSrc, SDL_ECC_AGGR_ERROR_SUBTYPE_NORMAL, eccErrorInfo.bitErrCnt);
    }

    retVal = SDL_ECC_ackIntr(eccmemtype, eccIntrSrc);

    esmError = true;

    return retVal;
}
\endcode
\endcond


This structure defines the elements of ECC  Init configuration
\code{.c}
static SDL_ECC_MemSubType ECC_Test_MSS_L2_subMemTypeList[SDL_MSS_L2_MAX_MEM_SECTIONS] =
{
     SDL_EXAMPLE_ECC_RAM_ID,
};

static SDL_ECC_InitConfig_t ECC_Test_MSS_L2_ECCInitConfig =
{
    .numRams = SDL_MSS_L2_MAX_MEM_SECTIONS,
    /**< Number of Rams ECC is enabled  */
    .pMemSubTypeList = &(ECC_Test_MSS_L2_subMemTypeList[0]),
    /**< Sub type list  */
};
\endcode

Event BitMap for ECC ESM callback for MSS
\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
\code{.c}
SDL_ESM_config ECC_Test_esmInitConfig_MAIN =
{
     .esmErrorConfig = {1u, 8u}, /* Self test error config */
     .enableBitmap = {0x00180000u, 0x00000000u, 0x00000000u, 0x00000000u,
                      0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u},
      /**< All events enable: except clkstop events for unused clocks
       *   and PCIE events */
       /* CCM_1_SELFTEST_ERR and _R5FSS0COMPARE_ERR_PULSE_0 */
     .priorityBitmap = {0x00180000u, 0x000000000u, 0x00000000u, 0x00000000u,
                        0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u },
     /**< All events high priority: except clkstop events for unused clocks
      *   and PCIE events */
     .errorpinBitmap = {0x00180000u, 0x00000000u, 0x00000000u, 0x00000000u,
                        0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u},
     /**< All events high priority: except clkstop for unused clocks
      *   and PCIE events */
};
\endcode
\endcond

\cond SOC_AM273X || SOC_AWR294X
\code{.c}
/* Event BitMap for ECC ESM callback for MSS L2*/
SDL_ESM_NotifyParams ECC_TestparamsMSS[SDL_ESM_MAX_MSS_EXAMPLE_AGGR] =
{
     {
          /* Event BitMap for ECC ESM callback for MSS Single bit*/
          .groupNumber = SDL_INTR_GROUP_NUM_1,
          .errorNumber = SDL_ESMG1_ECCAGGMSS_SERR,
          .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
          .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
          .callBackFunction = &SDL_ESM_applicationCallbackFunction,
     },
     {
          /* Event BitMap for ECC ESM callback for MSS Double bit*/
          .groupNumber = SDL_INTR_GROUP_NUM_1,
          .errorNumber = SDL_ESMG1_ECCAGGMSS_UERR,
          .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_HIGH,
          .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
          .callBackFunction = &SDL_ESM_applicationCallbackFunction,
     },
};
\endcode
\endcond

Initialization of MSS L2 memory
\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
\code{.c}
    /* Clear Done memory*/
    SDL_REG32_WR(SDL_MSS_L2_MEM_INIT_DONE_ADDR, 0xfu);

    /* Initialization of MSS L2 memory*/
    SDL_REG32_WR(SDL_MSS_L2_MEM_INIT_ADDR, SDL_ECC_MSS_L2_BANK_MEM_INIT);

    while(SDL_REG32_RD(SDL_MSS_L2_MEM_INIT_DONE_ADDR)!=SDL_ECC_MSS_L2_BANK_MEM_INIT);

    /* Clear Done memory after MEM init*/
    SDL_REG32_WR(SDL_MSS_L2_MEM_INIT_DONE_ADDR, SDL_ECC_MSS_L2_BANK_MEM_INIT);
\endcode
\endcond

\cond SOC_AM273X || SOC_AWR294X
\code{.c}
	SDL_REG32_WR(SDL_MSS_L2_MEM_INIT_ADDR, SDL_ECC_MSS_L2_BANK_MEM_INIT);

    while(SDL_REG32_RD(SDL_MSS_L2_MEM_INIT_DONE_ADDR)!=SDL_ECC_MSS_L2_BANK_MEM_INIT);

    /* Clear Done memory after MEM init*/
    SDL_REG32_WR(SDL_MSS_L2_MEM_INIT_DONE_ADDR, SDL_ECC_MSS_L2_BANK_MEM_INIT);
\endcode
\endcond

Clearing any old interrupt presented
\code{.c}
SDL_REG32_WR(SDL_ECC_AGGR_ERROR_STATUS1_ADDR, 0xF0Fu);
\endcode

Initialize ECC memory for the ECC aggregator
\code{.c}
result = SDL_ECC_initMemory(SDL_EXAMPLE_ECC_AGGR, SDL_EXAMPLE_ECC_RAM_ID);
\endcode

\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
Initialize ESM module
\code{.c}
    result = SDL_ESM_init(SDL_ESM_INST_MAIN_ESM0, &ECC_Test_esmInitConfig_MAIN, SDL_ESM_applicationCallbackFunction, ptr);
\endcode
\endcond

Initialize ECC parameters for single and double bit error injection
\code{.c}
result = SDL_ECC_init(SDL_EXAMPLE_ECC_AGGR, &ECC_Test_MSS_ECCInitConfig);
\endcode

\cond SOC_AM273X || SOC_AWR294X
Initialize ESM module
\code{.c}
	result = SDL_ESM_init(SDL_ESM_INST_MSS_ESM, &ECC_TestparamsMSS[0],NULL,NULL);
	/* Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation */
	esmError = false;
\endcode
\endcond

Execute ECC MSS L2 single bit inject test
\code{.c}
int32_t ECC_Test_run_MSS_L2RAMB_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

	DebugP_log("\r\nMSS L2 RAMB Single bit error inject: starting \r\n");

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(SDL_EXAMPLE_ECC_RAM_ADDR);

    /* Run one shot test for MSS L2 RAMB 1 bit error */
    injectErrorConfig.flipBitMask = 0x002;
    result = SDL_ECC_injectError(SDL_EXAMPLE_ECC_AGGR,
                                 SDL_EXAMPLE_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                 &injectErrorConfig);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];

        DebugP_log("\r\nMSS L2 RAMB Single bit error inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete\r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}
\endcode

\cond SOC_AM273X || SOC_AWR294X
Initialize ESM module
\code{.c}
	result = SDL_ESM_init(SDL_ESM_INST_MSS_ESM, &ECC_TestparamsMSS[1],NULL,NULL);
	/*Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation*/
	esmError = false;
\endcode
\endcond

Initialization of MSS L2 memory
\code{.c}
    SDL_REG32_WR(SDL_MSS_L2_MEM_INIT_ADDR, SDL_ECC_MSS_L2_BANK_MEM_INIT);

    while(SDL_REG32_RD(SDL_MSS_L2_MEM_INIT_DONE_ADDR)!=SDL_ECC_MSS_L2_BANK_MEM_INIT);

    /* Clear Done memory after MEM init*/
    SDL_REG32_WR(SDL_MSS_L2_MEM_INIT_DONE_ADDR, SDL_ECC_MSS_L2_BANK_MEM_INIT);
\endcode

Clearing any old interrupt presented
\code{.c}
	SDL_REG32_WR(SDL_ECC_AGGR_ERROR_STATUS1_ADDR, 0xF0Fu);
\endcode

\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
Initialize ECC Memory
\code{.c}
	SDL_ECC_initMemory(SDL_EXAMPLE_ECC_AGGR, SDL_EXAMPLE_ECC_RAM_ID);
\endcode
\endcond


Execute ECC MSS L2 double bit inject test
\code{.c}
int32_t ECC_Test_run_MSS_L2RAMB_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

	DebugP_log("\r\nMSS L2 RAMB Double bit error inject: starting \r\n");

    /* Run one shot test for MSS L2 RAMB 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(SDL_EXAMPLE_ECC_RAM_ADDR);

    injectErrorConfig.flipBitMask = 0x30002;
    result = SDL_ECC_injectError(SDL_EXAMPLE_ECC_AGGR,
                                 SDL_EXAMPLE_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);

    /* Access the memory where injection is expected */
    testLocationValue = injectErrorConfig.pErrMem[0];

    if (result != SDL_PASS ) {
       retVal = -1;
    } else {
        DebugP_log("\r\nMSS L2 RAMB Double bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p\r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}
\endcode

## Example Usage of MSS TPTC

The following shows an example of SDL MSS TPTC API usage by the application for Error Injection Tests and Exception handling.

Include the below file to access the APIs

\code{.c}
#include <drivers/edma.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <sdl/sdl_ecc.h>
#include "ecc_main.h"
\endcode

Below are the macros specifies the ECC aggregator and ECC aggregator RAMID for inject the ECC error
\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
\code{.c}
#define SDL_MSS_MAX_MEM_SECTIONS        (1u)
#define SDL_ECC_SEC						(1U)
#define SDL_ECC_DED						(2U)

#define SDL_EXAMPLE_ECC_AGGR            SDL_SOC_ECC_AGGR
#define SDL_EXAMPLE_ECC_RAM_ID          SDL_SOC_ECC_AGGR_TPTC_A0_ECC_RAM_ID
\endcode
\endcond

\cond SOC_AM273X || SOC_AWR294X
\code{.c}
#define SDL_ESM_MAX_MSS_EXAMPLE_AGGR    (2u)
#define SDL_MSS_MAX_MEM_SECTIONS        (1u)
#define SDL_INTR_GROUP_NUM_1            (1U)
#define SDL_INTR_PRIORITY_LVL_LOW       (0U)
#define SDL_INTR_PRIORITY_LVL_HIGH      (1U)
#define SDL_ENABLE_ERR_PIN              (1U)
#define SDL_ECC_SEC						(1U)
#define SDL_ECC_DED						(2U)

#define SDL_EXAMPLE_ECC_AGGR            SDL_MSS_ECC_AGG_MSS
#define SDL_EXAMPLE_ECC_RAM_ID          SDL_MSS_ECC_AGG_MSS_MSS_TPTC_A0_ECC_RAM_ID
\endcode
\endcond

ESM callback function
\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
\code{.c}
int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInst,
                                            SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,
                                            uint32_t index,
                                            uint32_t intSrc,
                                            uintptr_t *arg)
{

    SDL_ECC_MemType eccmemtype;
    SDL_Ecc_AggrIntrSrc eccIntrSrc;
    SDL_ECC_ErrorInfo_t eccErrorInfo;
    int32_t retVal;


    printf("\r\nESM Call back function called : instType 0x%x, intType 0x%x, " \
                "grpChannel 0x%x, index 0x%x, intSrc 0x%x\r\n",
                esmInst, esmIntrType, grpChannel, index, intSrc);
    printf(" \r\nTake action \r\n");
    if(esmIntrType == 1u){
        printf("\r\nHigh Priority Interrupt Executed\r\n");
    }
    else{
        printf("\r\nLow Priority Interrupt Executed\r\n");
    }
    retVal = SDL_ECC_getESMErrorInfo(esmInst, intSrc, &eccmemtype, &eccIntrSrc);

    /* Any additional customer specific actions can be added here */
    retVal = SDL_ECC_getErrorInfo(eccmemtype, eccIntrSrc, &eccErrorInfo);

    printf("\r\nECC Error Call back function called : eccMemType %d, errorSrc 0x%x, " \
               "ramId %d, bitErrorOffset 0x%04x%04x, bitErrorGroup %d\r\n",
               eccmemtype, eccIntrSrc, eccErrorInfo.memSubType, (uint32_t)(eccErrorInfo.bitErrorOffset >> 32),
               (uint32_t)(eccErrorInfo.bitErrorOffset & 0x00000000FFFFFFFF), eccErrorInfo.bitErrorGroup);

    if (eccErrorInfo.injectBitErrCnt != 0)
    {
        SDL_ECC_clearNIntrPending(eccmemtype, eccErrorInfo.memSubType, eccIntrSrc, SDL_ECC_AGGR_ERROR_SUBTYPE_INJECT, eccErrorInfo.injectBitErrCnt);
    }
    else
    {
        SDL_ECC_clearNIntrPending(eccmemtype, eccErrorInfo.memSubType, eccIntrSrc, SDL_ECC_AGGR_ERROR_SUBTYPE_NORMAL, eccErrorInfo.bitErrCnt);
    }

    retVal = SDL_ECC_ackIntr(eccmemtype, eccIntrSrc);

    esmError = true;

    return retVal;
}
\endcode
\endcond

\cond SOC_AM273X || SOC_AWR294X
\code{.c}
int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
                                           int32_t grpChannel,
                                           int32_t intSrc,
                                           void *arg)
{

    SDL_ECC_MemType eccmemtype;
    SDL_Ecc_AggrIntrSrc eccIntrSrc;
    SDL_ECC_ErrorInfo_t eccErrorInfo;
    int32_t retVal;
    printf("\r\nESM Call back function called : instType 0x%x, " \
                "grpChannel 0x%x, intSrc 0x%x \r\n",
                esmInstType, grpChannel, intSrc);
    printf("\r\nTake action \r\n");

    retVal = SDL_ECC_getESMErrorInfo(esmInstType, intSrc, &eccmemtype, &eccIntrSrc);

    /* Any additional customer specific actions can be added here */
    retVal = SDL_ECC_getErrorInfo(eccmemtype, eccIntrSrc, &eccErrorInfo);

    printf("\r\nECC Error Call back function called : eccMemType %d, errorSrc 0x%x, " \
               "ramId %d, bitErrorOffset 0x%04x%04x, bitErrorGroup %d\r\n",
               eccmemtype, eccIntrSrc, eccErrorInfo.memSubType, (uint32_t)(eccErrorInfo.bitErrorOffset >> 32),
               (uint32_t)(eccErrorInfo.bitErrorOffset & 0x00000000FFFFFFFF), eccErrorInfo.bitErrorGroup);

    if (eccErrorInfo.injectBitErrCnt != 0)
    {
        SDL_ECC_clearNIntrPending(eccmemtype, eccErrorInfo.memSubType, eccIntrSrc, SDL_ECC_AGGR_ERROR_SUBTYPE_INJECT, eccErrorInfo.injectBitErrCnt);
    }
    else
    {
        SDL_ECC_clearNIntrPending(eccmemtype, eccErrorInfo.memSubType, eccIntrSrc, SDL_ECC_AGGR_ERROR_SUBTYPE_NORMAL, eccErrorInfo.bitErrCnt);
    }

    retVal = SDL_ECC_ackIntr(eccmemtype, eccIntrSrc);

    esmError = true;

    return retVal;
}
\endcode
\endcond


This structure defines the elements of ECC  Init configuration
\code{.c}
static SDL_ECC_MemSubType ECC_Test_MSS_subMemTypeList[SDL_MSS_MAX_MEM_SECTIONS] =
{
     SDL_EXAMPLE_ECC_RAM_ID,
};

static SDL_ECC_InitConfig_t ECC_Test_MSS_ECCInitConfig =
{
    .numRams = SDL_MSS_MAX_MEM_SECTIONS,
    /**< Number of Rams ECC is enabled  */
    .pMemSubTypeList = &(ECC_Test_MSS_subMemTypeList[0]),
    /**< Sub type list  */
};
\endcode

Event BitMap for ECC ESM callback for MSS
\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
\code{.c}
SDL_ESM_config ECC_Test_esmInitConfig_MAIN =
{
     .esmErrorConfig = {1u, 8u}, /* Self test error config */
     .enableBitmap = {0x00180000u, 0x00000000u, 0x00000000u, 0x00000000u,
                      0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u},
      /**< All events enable: except clkstop events for unused clocks
       *   and PCIE events */
       /* CCM_1_SELFTEST_ERR and _R5FSS0COMPARE_ERR_PULSE_0 */
     .priorityBitmap = {0x00180000u, 0x000000000u, 0x00000000u, 0x00000000u,
                        0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u },
     /**< All events high priority: except clkstop events for unused clocks
      *   and PCIE events */
     .errorpinBitmap = {0x00180000u, 0x00000000u, 0x00000000u, 0x00000000u,
                        0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u},
     /**< All events high priority: except clkstop for unused clocks
      *   and PCIE events */
};
\endcode
\endcond

\cond SOC_AM273X || SOC_AWR294X
\code{.c}
/* Event BitMap for ECC ESM callback for MSS TPTC*/
SDL_ESM_NotifyParams ECC_TestparamsMSS[SDL_ESM_MAX_MSS_EXAMPLE_AGGR] =
{
     {
          /* Event BitMap for ECC ESM callback for MSS Single bit*/
          .groupNumber = SDL_INTR_GROUP_NUM_1,
          .errorNumber = SDL_ESMG1_ECCAGGMSS_SERR,
          .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
          .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
          .callBackFunction = &SDL_ESM_applicationCallbackFunction,
     },
     {
          /* Event BitMap for ECC ESM callback for MSS Double bit*/
          .groupNumber = SDL_INTR_GROUP_NUM_1,
          .errorNumber = SDL_ESMG1_ECCAGGMSS_UERR,
          .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_HIGH,
          .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
          .callBackFunction = &SDL_ESM_applicationCallbackFunction,
     },
};
\endcode
\endcond

Initialization of EDMA and ECC injection
\code{.c}
   /* EDMA transfer with ECC single bit injection*/
	testResult = edma_interrupt_transfer(CONFIG_EDMA0, SDL_ECC_SEC, 0u, EDMA_MSS_TPCC_A_EVT_FREE_0);
\endcode

Initialize EDMA
\code{.c}
	EDMA_Init();
    DebugP_log("\r\n[EDMA] Interrupt Transfer Test Started...\r\n");
    baseAddr = EDMA_getBaseAddr(gEdmaHandle[edmaConfigNum]);

    regionId = EDMA_getRegionId(gEdmaHandle[edmaConfigNum]);

    dmaCh = channelEvent;
    status = EDMA_allocDmaChannel(gEdmaHandle[edmaConfigNum], &dmaCh);

    tcc = channelEvent;
    status = EDMA_allocTcc(gEdmaHandle[edmaConfigNum], &tcc);

    param = channelEvent;
    status = EDMA_allocParam(gEdmaHandle[edmaConfigNum], &param);

\endcode
Initialize ECC memory for the ECC aggregator
\code{.c}
result = SDL_ECC_initMemory(SDL_EXAMPLE_ECC_AGGR, SDL_EXAMPLE_ECC_RAM_ID);
\endcode

\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
Initialize ESM module
\code{.c}
    result = SDL_ESM_init(SDL_ESM_INST_MAIN_ESM0, &ECC_Test_esmInitConfig_MAIN, SDL_ESM_applicationCallbackFunction, ptr);
\endcode
\endcond

\cond SOC_AM273X || SOC_AWR294X
Initialize ESM module
\code{.c}
	result = SDL_ESM_init(SDL_ESM_INST_MSS_ESM, &ECC_TestparamsMSS[counter],NULL,NULL);
\endcode
\endcond

Initialize ECC parameters for single and double bit error injection
\code{.c}
result = SDL_ECC_init(SDL_EXAMPLE_ECC_AGGR, &ECC_Test_MSS_ECCInitConfig);
\endcode

Execute ECC MSS TPTC single bit inject test
\code{.c}
int32_t ECC_Test_run_MSS_TPTC_A0_1Bit_InjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;

    DebugP_log("\n MSS TPTC_A0 Single bit error inject: test starting");

    injectErrorConfig.pErrMem = (uint32_t *)(0x0u);

    /* Run one shot test for MSS TPTC_A0 1 bit error */
    injectErrorConfig.flipBitMask = 0x02;
    result = SDL_ECC_injectError(SDL_EXAMPLE_ECC_AGGR,
                                 SDL_EXAMPLE_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                 &injectErrorConfig);

    if (result != SDL_PASS ) {
        DebugP_log("\n MSS TPTC_A0 Single bit error inject at pErrMem 0x%p test failed",
                    injectErrorConfig.pErrMem);
        retVal = -1;
    } else {

        DebugP_log("\n MSS TPTC_A0 Single bit error inject at pErrMem 0x%p",
                   injectErrorConfig.pErrMem);
    }


    return retVal;
}
\endcode

Execute ECC MSS TPTC double bit inject test
\code{.c}
int32_t ECC_Test_run_MSS_TPTC_A0_2Bit_InjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;

    DebugP_log("\n MSS TPTC_A0 Double bit error inject: starting");

    /* Run one shot test for MSS TPTC_A0 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x0u);

    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_EXAMPLE_ECC_AGGR,
                                 SDL_EXAMPLE_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);

    if (result != SDL_PASS ) {
        DebugP_log("\n MSS TPTC_A0 Double bit error inject: at pErrMem 0x%p: fixed location once test failed",
                    injectErrorConfig.pErrMem);
       retVal = -1;
    } else {

        DebugP_log("\n MSS TPTC_A0 Double bit error inject at pErrMem 0x%p ",
                   injectErrorConfig.pErrMem);
    }

    return retVal;
}
\endcode

Initialize the source address with a pattern and initialize dst address with zero/another pattern (optional)
\code{.c}

    srcBuffPtr = (uint8_t *) gEdmaTestSrcBuff;
    dstBuffPtr = (uint8_t *) gEdmaTestDstBuff;
    for(loopCnt = 0U; loopCnt < EDMA_TEST_BUFFER_SIZE; loopCnt++)
    {
        srcBuffPtr[loopCnt] = (uint8_t)loopCnt;
        dstBuffPtr[loopCnt] = 0;
    }
    CacheP_wb((void *)srcBuffPtr, EDMA_TEST_BUFFER_SIZE, CacheP_TYPE_ALL);
    CacheP_wb((void *)dstBuffPtr, EDMA_TEST_BUFFER_SIZE, CacheP_TYPE_ALL);

    /* Request channel */
    EDMA_configureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
         dmaCh, tcc, param, queueType);

    /* Program Param Set */
    EDMA_ccPaRAMEntry_init(&edmaParam);
    edmaParam.srcAddr       = (uint32_t) SOC_virtToPhy(srcBuffPtr);
    edmaParam.destAddr      = (uint32_t) SOC_virtToPhy(dstBuffPtr);
    edmaParam.aCnt          = (uint16_t) EDMA_TEST_A_COUNT;
    edmaParam.bCnt          = (uint16_t) EDMA_TEST_B_COUNT;
    edmaParam.cCnt          = (uint16_t) EDMA_TEST_C_COUNT;
    edmaParam.bCntReload    = (uint16_t) EDMA_TEST_B_COUNT;
    edmaParam.srcBIdx       = (int16_t) EDMA_PARAM_BIDX(EDMA_TEST_A_COUNT);
    edmaParam.destBIdx      = (int16_t) EDMA_PARAM_BIDX(EDMA_TEST_A_COUNT);
    edmaParam.srcCIdx       = (int16_t) EDMA_TEST_A_COUNT;
    edmaParam.destCIdx      = (int16_t) EDMA_TEST_A_COUNT;
    edmaParam.linkAddr      = 0xFFFFU;
    edmaParam.srcBIdxExt    = (int8_t) EDMA_PARAM_BIDX_EXT(EDMA_TEST_A_COUNT);
    edmaParam.destBIdxExt   = (int8_t) EDMA_PARAM_BIDX_EXT(EDMA_TEST_A_COUNT);
    edmaParam.opt          |=
        (EDMA_OPT_TCINTEN_MASK | EDMA_OPT_ITCINTEN_MASK |
         ((((uint32_t)tcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));
    EDMA_setPaRAM(baseAddr, param, &edmaParam);

    status = SemaphoreP_constructBinary(&gEdmaTestDoneSem, 0);

    /* Register interrupt */
    intrObj.tccNum = tcc;
    intrObj.cbFxn  = &EDMA_regionIsrFxn;
    intrObj.appData = (void *) &gEdmaTestDoneSem;
    status = EDMA_registerIntr(gEdmaHandle[edmaConfigNum], &intrObj);

    /*
     * Transfer is done in A sync mode
     * Number of triggers required are B_COUNT * C_COUNT
     */
    for(loopCnt = 0; loopCnt < (EDMA_TEST_B_COUNT * EDMA_TEST_C_COUNT); loopCnt++)
    {
        EDMA_enableTransferRegion(
            baseAddr, regionId, dmaCh, EDMA_TRIG_MODE_MANUAL);

        SemaphoreP_pend(&gEdmaTestDoneSem, SystemP_WAIT_FOREVER);
    }

    /* Invalidate destination buffer and compare with src buffer */
    CacheP_inv((void *)dstBuffPtr, EDMA_TEST_BUFFER_SIZE, CacheP_TYPE_ALL);
    for(loopCnt = 0; loopCnt < EDMA_TEST_BUFFER_SIZE; loopCnt++)
    {
        if(srcBuffPtr[loopCnt] != dstBuffPtr[loopCnt])
        {
            DebugP_log("Error matching value at src and dst address %d\r\n", loopCnt);
            status = SystemP_FAILURE;
			result = SDL_EFAIL;
            break;
        }
    }

    status = EDMA_unregisterIntr(gEdmaHandle[edmaConfigNum], &intrObj);
    SemaphoreP_destruct(&gEdmaTestDoneSem);

    /* Free channel */
    EDMA_freeChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
        dmaCh, EDMA_TRIG_MODE_MANUAL, tcc, EDMA_TEST_EVT_QUEUE_NO);

    /* Free the EDMA resources managed by driver. */
    status = EDMA_freeDmaChannel(gEdmaHandle[edmaConfigNum], &dmaCh);
    status = EDMA_freeTcc(gEdmaHandle[edmaConfigNum], &tcc);
    status = EDMA_freeParam(gEdmaHandle[edmaConfigNum], &param);

    if(status == SystemP_SUCCESS)
    {
        DebugP_log("\r\n[EDMA] Interrupt Transfer Test Completed!!\r\n");
        if(esmError == TRUE)
        {
            DebugP_log("\r\nAll tests have passed!!\r\n");
            esmError = false;
        }
        else
        {
            result = SDL_EFAIL;
            DebugP_log("\r\nESM interrupt is not occurred.... Test is failed!!\r\n");
        }
    }
    else
    {
        result = SDL_EFAIL;
        DebugP_log("\r\nSome tests have failed!!\r\n");
    }
    EDMA_Deinit();

\endcode


\cond SOC_AM273X || SOC_AWR294X
## Example Usage of DSS MAILBOX

Include the below file to access the APIs

\code{.c}
#include "ecc_main.h"
\endcode

Initialize ECC memory for the ECC aggregator
\code{.c}
result = SDL_ECC_initMemory(SDL_DSS_ECC_AGG, SDL_DSS_ECC_AGG_DSS_MAILBOX_ECC_RAM_ID);
\endcode

Initialize ESM module
\code{.c}
result = SDL_ESM_init(SDL_ESM_INST_DSS_ESM, &ECC_TestparamsDSS[u8ParamCount],NULL,NULL);
\endcode

Initialize ECC parameters for single and double bit error injection
\code{.c}
result = SDL_ECC_init(SDL_DSS_ECC_AGG, &ECC_Test_DSSECCInitConfig);
\endcode

Execute ECC DSS MAILBOX single bit inject test
\code{.c}
int32_t ECC_Test_run_DSS_MAILBOX_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;
    uint32_t ecc_ctrl = 0, ecc_sts = 0 ;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

	SDL_edc_ctlRegs *pEccAggrRegs = ((SDL_edc_ctlRegs *)((uintptr_t)0x060A0000u));

    DebugP_log("\n DSS MAILBOX Single bit error inject: test starting");

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x83100000u);

	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	ecc_sts = SDL_REG32_RD(0x060A0040);
	DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values before ECC single bit error injection",
                   ecc_ctrl, ecc_sts);

    /* Run one shot test for DSS MAILBOX 1 bit error */
    injectErrorConfig.flipBitMask = 0x10;
    result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                 SDL_DSS_ECC_AGG_DSS_MAILBOX_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_N_ROW_REPEAT,
                                 &injectErrorConfig);

	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
    testLocationValue = injectErrorConfig.pErrMem[0];
    ecc_sts = SDL_REG32_RD(0x060A0040);

    DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values after ECC single bit error injection",
                       ecc_ctrl, ecc_sts);

    if (result != SDL_PASS ) {
        DebugP_log("\n DSS MAILBOX Single bit error inject at pErrMem 0x%p test failed",
                    injectErrorConfig.pErrMem);
        retVal = -1;
    } else {

        DebugP_log("\n DSS MAILBOX Single bit error inject at pErrMem 0x%p",
                   injectErrorConfig.pErrMem, testLocationValue);
    }


    return retVal;
}
\endcode

Execute ECC DSS MAILBOX double bit inject test
\code{.c}
int32_t ECC_Test_run_DSS_MAILBOX_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;
    uint32_t ecc_ctrl = 0, ecc_sts = 0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

	SDL_edc_ctlRegs *pEccAggrRegs = ((SDL_edc_ctlRegs *)((uintptr_t)0x060A0000u));

    DebugP_log("\n DSS MAILBOX Double bit error inject: starting");

    /* Run one shot test for DSS MAILBOX 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x83100000u);

	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	ecc_sts = SDL_REG32_RD(0x060A0140);
	DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values before ECC double bit error injection",
                   ecc_ctrl, ecc_sts);

    injectErrorConfig.flipBitMask = 0x101;
    result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                 SDL_DSS_ECC_AGG_DSS_MAILBOX_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_N_ROW_REPEAT,
                                 &injectErrorConfig);

	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	/* Access the memory where injection is expected */
    testLocationValue = injectErrorConfig.pErrMem[0];
    ecc_sts = SDL_REG32_RD(0x060A0140);

    DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values after ECC double bit error injection",
                       ecc_ctrl, ecc_sts);

    if (result != SDL_PASS ) {
        DebugP_log("\n DSS MAILBOX Double bit error inject: at pErrMem 0x%p: fixed location once test failed",
                    injectErrorConfig.pErrMem);
       retVal = -1;
    } else {

        DebugP_log("\n DSS MAILBOX Double bit error inject at pErrMem 0x%p ",
                   injectErrorConfig.pErrMem);
    }

    return retVal;
}
\endcode

## Example Usage of DSS TPTC

The following shows an example of SDL DSS TPTC API usage by the application for Error Injection Tests and Exception handling.

Include the below file to access the APIs

\code{.c}
#include <drivers/edma.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <sdl/sdl_ecc.h>
#include "ecc_main.h"
\endcode

Below are the macros specifies the ECC aggregator and ECC aggregator RAMID for inject the ECC error
\code{.c}
#define SDL_ESM_MAX_DSS_EXAMPLE_AGGR	(2u)

#define SDL_INTR_GROUP_NUM              (1U)
#define SDL_INTR_PRIORITY_LVL_LOW       (0U)
#define SDL_INTR_PRIORITY_LVL_HIGH      (1U)
#define SDL_ENABLE_ERR_PIN              (1U)
#define SDL_ECC_SEC                     (1U)
#define SDL_ECC_DED                     (2U)

#define SDL_DSS_MAX_MEM_SECTIONS        (1u)

#define SDL_EXAMPLE_ECC_AGGR            SDL_DSS_ECC_AGG
#define SDL_EXAMPLE_ECC_RAM_ID          SDL_DSS_ECC_AGG_DSS_TPTC_A0_FIFO_ECC_RAM_ID
\endcode

ESM callback function
\code{.c}
int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
                                           int32_t grpChannel,
                                           int32_t intSrc,
                                           void *arg)
{

    SDL_ECC_MemType eccmemtype;
    SDL_Ecc_AggrIntrSrc eccIntrSrc;
    SDL_ECC_ErrorInfo_t eccErrorInfo;
    int32_t retVal;
    printf("\r\nESM Call back function called : instType 0x%x, " \
                "grpChannel 0x%x, intSrc 0x%x \r\n",
                esmInstType, grpChannel, intSrc);
    printf("\r\nTake action \r\n");

    retVal = SDL_ECC_getESMErrorInfo(esmInstType, intSrc, &eccmemtype, &eccIntrSrc);

    /* Any additional customer specific actions can be added here */
    retVal = SDL_ECC_getErrorInfo(eccmemtype, eccIntrSrc, &eccErrorInfo);

    printf("\r\nECC Error Call back function called : eccMemType %d, errorSrc 0x%x, " \
               "ramId %d, bitErrorOffset 0x%04x%04x, bitErrorGroup %d\r\n",
               eccmemtype, eccIntrSrc, eccErrorInfo.memSubType, (uint32_t)(eccErrorInfo.bitErrorOffset >> 32),
               (uint32_t)(eccErrorInfo.bitErrorOffset & 0x00000000FFFFFFFF), eccErrorInfo.bitErrorGroup);

    if (eccErrorInfo.injectBitErrCnt != 0)
    {
        SDL_ECC_clearNIntrPending(eccmemtype, eccErrorInfo.memSubType, eccIntrSrc, SDL_ECC_AGGR_ERROR_SUBTYPE_INJECT, eccErrorInfo.injectBitErrCnt);
    }
    else
    {
        SDL_ECC_clearNIntrPending(eccmemtype, eccErrorInfo.memSubType, eccIntrSrc, SDL_ECC_AGGR_ERROR_SUBTYPE_NORMAL, eccErrorInfo.bitErrCnt);
    }

    retVal = SDL_ECC_ackIntr(eccmemtype, eccIntrSrc);

    esmError = true;

    return retVal;
}
\endcode


This structure defines the elements of ECC  Init configuration
\code{.c}
static SDL_ECC_MemSubType ECC_Test_DSSsubMemTypeList[SDL_DSS_MAX_MEM_SECTIONS] =
{
     SDL_EXAMPLE_ECC_RAM_ID,
};

static SDL_ECC_InitConfig_t ECC_Test_DSSECCInitConfig =
{
    .numRams = SDL_DSS_MAX_MEM_SECTIONS,
    /**< Number of Rams ECC is enabled  */
    .pMemSubTypeList = &(ECC_Test_DSSsubMemTypeList[0]),
    /**< Sub type list  */
};
\endcode

Event BitMap for ECC ESM callback for DSS
\code{.c}
/* Event BitMap for ECC ESM callback for DSS TPTC*/
SDL_ESM_NotifyParams ECC_TestparamsDSS[SDL_ESM_MAX_DSS_EXAMPLE_AGGR] =
{
    {
	   /* Event BitMap for ECC ESM callback for DSS Single bit*/
	   .groupNumber = SDL_INTR_GROUP_NUM,
	   .errorNumber = SDL_DSS_ESMG1_DSS_ECC_AGG_SERR,
	   .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
	   .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
	   .callBackFunction = &SDL_ESM_applicationCallbackFunction,
    },
    {
	   /* Event BitMap for ECC ESM callback for DSS Double bit*/
	   .groupNumber = SDL_INTR_GROUP_NUM,
	   .errorNumber = SDL_DSS_ESMG1_DSS_ECC_AGG_UERR,
	   .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_HIGH,
	   .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
	   .callBackFunction = &SDL_ESM_applicationCallbackFunction,
    },

};
\endcode

Initialization of EDMA and ECC injection
\code{.c}
   /* EDMA transfer with ECC single bit injection*/
	testResult = edma_interrupt_transfer(CONFIG_EDMA0, SDL_ECC_SEC, 0u, EDMA_DSS_TPCC_A_EVT_FREE_0);
\endcode

Initialize EDMA
\code{.c}
	EDMA_Init();
    DebugP_log("\r\n[EDMA] Interrupt Transfer Test Started...\r\n");
    baseAddr = EDMA_getBaseAddr(gEdmaHandle[edmaConfigNum]);

    regionId = EDMA_getRegionId(gEdmaHandle[edmaConfigNum]);

    dmaCh = channelEvent;
    status = EDMA_allocDmaChannel(gEdmaHandle[edmaConfigNum], &dmaCh);

    tcc = channelEvent;
    status = EDMA_allocTcc(gEdmaHandle[edmaConfigNum], &tcc);

    param = channelEvent;
    status = EDMA_allocParam(gEdmaHandle[edmaConfigNum], &param);

\endcode
Initialize ECC memory for the ECC aggregator
\code{.c}
result = SDL_ECC_initMemory(SDL_EXAMPLE_ECC_AGGR, SDL_EXAMPLE_ECC_RAM_ID);
\endcode

Initialize ESM module
\code{.c}
	result = SDL_ESM_init(SDL_ESM_INST_DSS_ESM, &ECC_TestparamsDSS[counter],NULL,NULL);
\endcode

Initialize ECC parameters for single and double bit error injection
\code{.c}
result = SDL_ECC_init(SDL_DSS_ECC_AGG, &ECC_Test_DSSECCInitConfig);
\endcode

Execute ECC DSS TPTC single bit inject test
\code{.c}
int32_t ECC_Test_run_DSS_TPTC_A0_1Bit_InjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;

    DebugP_log("\r\nDSS TPTC_A0 Single bit error inject: test starting\r\n");

    injectErrorConfig.pErrMem = (uint32_t *)(0x0u);

    /* Run one shot test for DSS TPTC_A0 1 bit error */
    injectErrorConfig.flipBitMask = 0x02;
    result = SDL_ECC_injectError(SDL_EXAMPLE_ECC_AGGR,
                                 SDL_EXAMPLE_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                 &injectErrorConfig);

    if (result != SDL_PASS ) {
        DebugP_log("\r\nDSS TPTC_A0 Single bit error inject at pErrMem 0x%p test failed\r\n",
                    injectErrorConfig.pErrMem);
        retVal = -1;
    } else {

        DebugP_log("\r\nDSS TPTC_A0 Single bit error inject at pErrMem 0x%p\r\n",
                   injectErrorConfig.pErrMem);
    }


    return retVal;
}
\endcode

Execute ECC DSS TPTC double bit inject test
\code{.c}
int32_t ECC_Test_run_DSS_TPTC_A0_2Bit_InjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;

    DebugP_log("\r\nDSS TPTC_A0 Double bit error inject: starting\r\n");

    /* Run one shot test for DSS TPTC_A0 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x0u);

    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_EXAMPLE_ECC_AGGR,
                                 SDL_EXAMPLE_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);

    if (result != SDL_PASS ) {
        DebugP_log("\r\nDSS TPTC_A0 Double bit error inject: at pErrMem 0x%p: fixed location once test failed\r\n",
                    injectErrorConfig.pErrMem);
       retVal = -1;
    } else {

        DebugP_log("\r\nDSS TPTC_A0 Double bit error inject at pErrMem 0x%p\r\n ",
                   injectErrorConfig.pErrMem);
    }

    return retVal;
}
\endcode

Initialize the source address with a pattern and initialize dst address with zero/another pattern (optional)
\code{.c}

    srcBuffPtr = (uint8_t *) gEdmaTestSrcBuff;
    dstBuffPtr = (uint8_t *) gEdmaTestDstBuff;
    for(loopCnt = 0U; loopCnt < EDMA_TEST_BUFFER_SIZE; loopCnt++)
    {
        srcBuffPtr[loopCnt] = (uint8_t)loopCnt;
        dstBuffPtr[loopCnt] = 0;
    }
    CacheP_wb((void *)srcBuffPtr, EDMA_TEST_BUFFER_SIZE, CacheP_TYPE_ALL);
    CacheP_wb((void *)dstBuffPtr, EDMA_TEST_BUFFER_SIZE, CacheP_TYPE_ALL);

    /* Request channel */
    EDMA_configureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
         dmaCh, tcc, param, queueType);

    /* Program Param Set */
    EDMA_ccPaRAMEntry_init(&edmaParam);
    edmaParam.srcAddr       = (uint32_t) SOC_virtToPhy(srcBuffPtr);
    edmaParam.destAddr      = (uint32_t) SOC_virtToPhy(dstBuffPtr);
    edmaParam.aCnt          = (uint16_t) EDMA_TEST_A_COUNT;
    edmaParam.bCnt          = (uint16_t) EDMA_TEST_B_COUNT;
    edmaParam.cCnt          = (uint16_t) EDMA_TEST_C_COUNT;
    edmaParam.bCntReload    = (uint16_t) EDMA_TEST_B_COUNT;
    edmaParam.srcBIdx       = (int16_t) EDMA_PARAM_BIDX(EDMA_TEST_A_COUNT);
    edmaParam.destBIdx      = (int16_t) EDMA_PARAM_BIDX(EDMA_TEST_A_COUNT);
    edmaParam.srcCIdx       = (int16_t) EDMA_TEST_A_COUNT;
    edmaParam.destCIdx      = (int16_t) EDMA_TEST_A_COUNT;
    edmaParam.linkAddr      = 0xFFFFU;
    edmaParam.srcBIdxExt    = (int8_t) EDMA_PARAM_BIDX_EXT(EDMA_TEST_A_COUNT);
    edmaParam.destBIdxExt   = (int8_t) EDMA_PARAM_BIDX_EXT(EDMA_TEST_A_COUNT);
    edmaParam.opt          |=
        (EDMA_OPT_TCINTEN_MASK | EDMA_OPT_ITCINTEN_MASK |
         ((((uint32_t)tcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));
    EDMA_setPaRAM(baseAddr, param, &edmaParam);

    status = SemaphoreP_constructBinary(&gEdmaTestDoneSem, 0);

    /* Register interrupt */
    intrObj.tccNum = tcc;
    intrObj.cbFxn  = &EDMA_regionIsrFxn;
    intrObj.appData = (void *) &gEdmaTestDoneSem;
    status = EDMA_registerIntr(gEdmaHandle[edmaConfigNum], &intrObj);

    /*
     * Transfer is done in A sync mode
     * Number of triggers required are B_COUNT * C_COUNT
     */
    for(loopCnt = 0; loopCnt < (EDMA_TEST_B_COUNT * EDMA_TEST_C_COUNT); loopCnt++)
    {
        EDMA_enableTransferRegion(
            baseAddr, regionId, dmaCh, EDMA_TRIG_MODE_MANUAL);

        SemaphoreP_pend(&gEdmaTestDoneSem, SystemP_WAIT_FOREVER);
    }

    /* Invalidate destination buffer and compare with src buffer */
    CacheP_inv((void *)dstBuffPtr, EDMA_TEST_BUFFER_SIZE, CacheP_TYPE_ALL);
    for(loopCnt = 0; loopCnt < EDMA_TEST_BUFFER_SIZE; loopCnt++)
    {
        if(srcBuffPtr[loopCnt] != dstBuffPtr[loopCnt])
        {
            DebugP_log("Error matching value at src and dst address %d\r\n", loopCnt);
            status = SystemP_FAILURE;
			result = SDL_EFAIL;
            break;
        }
    }

    status = EDMA_unregisterIntr(gEdmaHandle[edmaConfigNum], &intrObj);
    SemaphoreP_destruct(&gEdmaTestDoneSem);

    /* Free channel */
    EDMA_freeChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
        dmaCh, EDMA_TRIG_MODE_MANUAL, tcc, EDMA_TEST_EVT_QUEUE_NO);

    /* Free the EDMA resources managed by driver. */
    status = EDMA_freeDmaChannel(gEdmaHandle[edmaConfigNum], &dmaCh);
    status = EDMA_freeTcc(gEdmaHandle[edmaConfigNum], &tcc);
    status = EDMA_freeParam(gEdmaHandle[edmaConfigNum], &param);

    if(status == SystemP_SUCCESS)
    {
        DebugP_log("\r\n[EDMA] Interrupt Transfer Test Completed!!\r\n");
        if(esmError == TRUE)
        {
            DebugP_log("\r\nAll tests have passed!!\r\n");
            esmError = false;
        }
        else
        {
            result = SDL_EFAIL;
            DebugP_log("\r\nESM interrupt is not occurred.... Test is failed!!\r\n");
        }
    }
    else
    {
        result = SDL_EFAIL;
        DebugP_log("\r\nSome tests have failed!!\r\n");
    }
    EDMA_Deinit();

\endcode

\endcond

## Example Usage of MCAN

Include the below file to access the APIs

\code{.c}
#include "ecc_main.h"
\endcode

Below are the macros specifies the RAM address, ECC aggregator and ECC aggregator RAMID for inject the ECC error
\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
\code{.c}
#define SDL_EXAMPLE_ECC_RAM_ADDR                    (0x52600000u) /*MCAN0 address*/
#define SDL_EXAMPLE_ECC_AGGR                        SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR
#define SDL_EXAMPLE_ECC_RAM_ID                      SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_ID
\endcode
\endcond

\cond SOC_AM273X || SOC_AWR294X
\code{.c}
#define SDL_ESM_MAX_MCANA_EXAMPLE_AGGR              (2u)
#define SDL_INTR_GROUP_NUM_1                        (1U)
#define SDL_INTR_GROUP_NUM_2                        (2U)
#define SDL_INTR_GROUP_NUM_3                        (3U)
#define SDL_INTR_PRIORITY_LVL_LOW                   (0U)
#define SDL_INTR_PRIORITY_LVL_HIGH                  (1U)
#define SDL_ENABLE_ERR_PIN                          (1U)

#define SDL_EXAMPLE_ECC_RAM_ADDR                    (0x02040000u) /* MCANA RAM address */
#define SDL_EXAMPLE_ECC_AGGR                        SDL_MSS_MCANA_ECC
#define SDL_EXAMPLE_ECC_RAM_ID                      SDL_MSS_MCANA_ECC_MSS_MCANA_ECC_RAM_ID
\endcode
\endcond

This structure defines the elements of ECC  Init configuration
\code{.c}
static SDL_ECC_MemSubType ECC_Test_MCANA_subMemTypeList[SDL_MCANA_MAX_MEM_SECTIONS] =
{
     SDL_EXAMPLE_ECC_RAM_ID,
};

static SDL_ECC_InitConfig_t ECC_Test_MCANA_ECCInitConfig =
{
    .numRams = SDL_MCANA_MAX_MEM_SECTIONS,
    /**< Number of Rams ECC is enabled  */
    .pMemSubTypeList = &(ECC_Test_MCANA_subMemTypeList[0]),
    /**< Sub type list  */
};
\endcode

ESM callback function
\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
\code{.c}
int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInst,
                                            SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,
                                            uint32_t index,
                                            uint32_t intSrc,
                                            uintptr_t *arg)
{

    SDL_ECC_MemType eccmemtype;
    SDL_Ecc_AggrIntrSrc eccIntrSrc;
    SDL_ECC_ErrorInfo_t eccErrorInfo;
    int32_t retVal;


    printf("\r\nESM Call back function called : instType 0x%x, intType 0x%x, " \
                "grpChannel 0x%x, index 0x%x, intSrc 0x%x\r\n",
                esmInst, esmIntrType, grpChannel, index, intSrc);
    printf("\r\nTake action\r\n");
    if(esmIntrType == 1u){
        printf("\r\nHigh Priority Interrupt Executed\r\n");
    }
    else{
        printf("\r\nLow Priority Interrupt Executed\r\n");
    }
    retVal = SDL_ECC_getESMErrorInfo(esmInst, intSrc, &eccmemtype, &eccIntrSrc);

    /* Any additional customer specific actions can be added here */
    retVal = SDL_ECC_getErrorInfo(eccmemtype, eccIntrSrc, &eccErrorInfo);

    printf("\r\nECC Error Call back function called : eccMemType %d, errorSrc 0x%x, " \
               "ramId %d, bitErrorOffset 0x%04x%04x, bitErrorGroup %d\r\n",
               eccmemtype, eccIntrSrc, eccErrorInfo.memSubType, (uint32_t)(eccErrorInfo.bitErrorOffset >> 32),
               (uint32_t)(eccErrorInfo.bitErrorOffset & 0x00000000FFFFFFFF), eccErrorInfo.bitErrorGroup);

    if (eccErrorInfo.injectBitErrCnt != 0)
    {
        SDL_ECC_clearNIntrPending(eccmemtype, eccErrorInfo.memSubType, eccIntrSrc, SDL_ECC_AGGR_ERROR_SUBTYPE_INJECT, eccErrorInfo.injectBitErrCnt);
    }
    else
    {
        SDL_ECC_clearNIntrPending(eccmemtype, eccErrorInfo.memSubType, eccIntrSrc, SDL_ECC_AGGR_ERROR_SUBTYPE_NORMAL, eccErrorInfo.bitErrCnt);
    }

    retVal = SDL_ECC_ackIntr(eccmemtype, eccIntrSrc);

    esmError = true;

    return retVal;
}
\endcode
\endcond

\cond SOC_AM273X || SOC_AWR294X
\code{.c}
int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
                                           int32_t grpChannel,
                                           int32_t intSrc,
                                           void *arg)
{

    SDL_ECC_MemType eccmemtype;
    SDL_Ecc_AggrIntrSrc eccIntrSrc;
    SDL_ECC_ErrorInfo_t eccErrorInfo;
    int32_t retVal;
    printf("\r\nESM Call back function called : instType 0x%x, " \
                "grpChannel 0x%x, intSrc 0x%x \r\n",
                esmInstType, grpChannel, intSrc);
    printf("\r\nTake action \n");

    retVal = SDL_ECC_getESMErrorInfo(esmInstType, intSrc, &eccmemtype, &eccIntrSrc);

    /* Any additional customer specific actions can be added here */
    retVal = SDL_ECC_getErrorInfo(eccmemtype, eccIntrSrc, &eccErrorInfo);

    printf("\r\nECC Error Call back function called : eccMemType %d, errorSrc 0x%x, " \
               "ramId %d, bitErrorOffset 0x%04x%04x, bitErrorGroup %d\r\n",
               eccmemtype, eccIntrSrc, eccErrorInfo.memSubType, (uint32_t)(eccErrorInfo.bitErrorOffset >> 32),
               (uint32_t)(eccErrorInfo.bitErrorOffset & 0x00000000FFFFFFFF), eccErrorInfo.bitErrorGroup);

    if (eccErrorInfo.injectBitErrCnt != 0)
    {
        SDL_ECC_clearNIntrPending(eccmemtype, eccErrorInfo.memSubType, eccIntrSrc, SDL_ECC_AGGR_ERROR_SUBTYPE_INJECT, eccErrorInfo.injectBitErrCnt);
    }
    else
    {
        SDL_ECC_clearNIntrPending(eccmemtype, eccErrorInfo.memSubType, eccIntrSrc, SDL_ECC_AGGR_ERROR_SUBTYPE_NORMAL, eccErrorInfo.bitErrCnt);
    }

    retVal = SDL_ECC_ackIntr(eccmemtype, eccIntrSrc);

    esmError = true;

    return retVal;
}
\endcode
\endcond

Event BitMap for ECC ESM callback for MCAN
\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
\code{.c}
SDL_ESM_config ECC_Test_esmInitConfig_MAIN =
{
     .esmErrorConfig = {1u, 8u}, /* Self test error config */
     .enableBitmap = {0x0000000cu, 0x00000000u, 0x00000000u, 0x00000000u,
                      0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u},
      /**< All events enable: except clkstop events for unused clocks
       *   and PCIE events */
       /* CCM_1_SELFTEST_ERR and _R5FSS0COMPARE_ERR_PULSE_0 */
     .priorityBitmap = {0x00000008u, 0x000000000u, 0x00000000u, 0x00000000u,
                        0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u },
     /**< All events high priority: except clkstop events for unused clocks
      *   and PCIE events */
     .errorpinBitmap = {0x0000000cu, 0x00000000u, 0x00000000u, 0x00000000u,
                        0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u},
     /**< All events high priority: except clkstop for unused clocks
      *   and PCIE events */
};
\endcode
\endcond

\cond SOC_AM273X || SOC_AWR294X
\code{.c}
/* Event BitMap for ECC ESM callback for MSS MCAN*/
SDL_ESM_NotifyParams ECC_TestparamsMSS[SDL_ESM_MAX_MSS_EXAMPLE_AGGR] =
{
     {
          /* Event BitMap for ECC ESM callback for MSS Single bit*/
          .groupNumber = SDL_INTR_GROUP_NUM_1,
          .errorNumber = SDL_ESMG1_MCANA_SERR,
          .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
          .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
          .callBackFunction = &SDL_ESM_applicationCallbackFunction,
     },
     {
          /* Event BitMap for ECC ESM callback for MSS Double bit*/
          .groupNumber = SDL_INTR_GROUP_NUM_1,
          .errorNumber = SDL_ESMG1_MCANA_SERR,
          .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_HIGH,
          .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
          .callBackFunction = &SDL_ESM_applicationCallbackFunction,
     },
};
\endcode
\endcond

Initialize ECC memory for the ECC aggregator
\code{.c}
result = SDL_ECC_initMemory(SDL_EXAMPLE_ECC_AGGR, SDL_EXAMPLE_ECC_RAM_ID);
\endcode

\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
Initialize ESM module
\code{.c}
result = SDL_ESM_init(SDL_ESM_INST_MAIN_ESM0, &ECC_Test_esmInitConfig_MAIN, SDL_ESM_applicationCallbackFunction, ptr);
\endcode
\endcond

Initialize ECC parameters for single and double bit error injection
\code{.c}
result = SDL_ECC_init(SDL_EXAMPLE_ECC_AGGR, &ECC_Test_MCANA_ECCInitConfig);
\endcode

\cond SOC_AM273X || SOC_AWR294X
Initialize ESM module
\code{.c}
result = SDL_ESM_init(SDL_ESM_INST_MSS_ESM, &ECC_TestparamsMCANA[0],NULL,NULL);
\endcode
\endcond

Write some data to the RAM memory before injecting
\code{.c}
for(i=1;i<=num_of_iterations;i++){
  wr_data = (i)<<24 | (i)<<16 | (i)<<8 | i;
  SDL_REG32_WR(addr+i*16, wr_data);
}
\endcode

Execute ECC MCAN single bit inject test
\code{.c}
int32_t ECC_Test_run_MCANA_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

	DebugP_log("\r\nMCANA Single bit error inject: starting \r\n");

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(SDL_EXAMPLE_ECC_RAM_ADDR);

    /* Run one shot test for MCANA  1 bit error */
    injectErrorConfig.flipBitMask = 0x002;
    result = SDL_ECC_injectError(SDL_EXAMPLE_ECC_AGGR,
                                 SDL_EXAMPLE_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                 &injectErrorConfig);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];

        DebugP_log("\r\nMCANA Single bit error inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete\r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}
\endcode

Read data from the RAM memory after injecting
\code{.c}
for(i=1;i<=num_of_iterations;i++){
  rd_data = SDL_REG32_RD(addr+i*16);
  DebugP_log("\r\nRead data =  0x%p\r\n",rd_data);
}
\endcode

\cond SOC_AM273X || SOC_AWR294X
Initialize ESM module
\code{.c}
result = SDL_ESM_init(SDL_ESM_INST_MSS_ESM, &ECC_TestparamsMCANA[1],NULL,NULL);
\endcode
\endcond

Write some data to the RAM memory before injecting
\code{.c}
for(i=1;i<=num_of_iterations;i++){
  wr_data = (i)<<24 | (i)<<16 | (i)<<8 | i;
  SDL_REG32_WR(addr+i*16, wr_data);
}
\endcode

Execute ECC MCAN double bit inject test
\code{.c}
int32_t ECC_Test_run_MCANA_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

	DebugP_log("\r\nMCANA double bit error inject: starting \r\n");

    /* Run one shot test for MCANA  2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(SDL_EXAMPLE_ECC_RAM_ADDR);

    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_EXAMPLE_ECC_AGGR,
                                 SDL_EXAMPLE_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);

    /* Access the memory where injection is expected */
    testLocationValue = injectErrorConfig.pErrMem[0];

    if (result != SDL_PASS ) {
       retVal = -1;
    } else {
        DebugP_log("\r\nMCANA Double bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p\r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}
\endcode

Read data from the RAM memory after injecting
\code{.c}
for(i=1;i<=num_of_iterations;i++){
  rd_data = SDL_REG32_RD(addr+i*16);
  DebugP_log("\r\nRead data =  0x%p\r\n",rd_data);
}
\endcode

\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
## Example Usage of ICSSM

Include the below file to access the APIs

\code{.c}
#include "ecc_main.h"
\endcode

Below are the macros specifies the RAM address, ECC aggregator and ECC aggregator RAMID for inject the ECC error
\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
\code{.c}
#define SDL_ICSSM_DRAM0								(1U)
#define	SDL_ICSSM_DRAM1								(0U)
#define	SDL_ICSSM_PR1_PDSP0_IRAM					(0U)
#define	SDL_ICSSM_PR1_PDSP1_IRAM					(0U)
#define SDL_ICSSM_RAM								(0U)

#define SDL_ICSSM_MAX_MEM_SECTIONS           		(1u)

#if SDL_ICSSM_DRAM0
#define SDL_EXAMPLE_ECC_RAM_ADDR                    (0x48000000u) /* ICSSM DRAM0 RAM address */
#define SDL_EXAMPLE_ECC_AGGR                        SDL_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR
#define SDL_EXAMPLE_ECC_RAM_ID                      SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_DRAM0_ECC_RAM_ID
#endif

#if SDL_ICSSM_DRAM1
#define SDL_EXAMPLE_ECC_RAM_ADDR                    (0x48002000u) /* ICSSM DRAM1 RAM address */
#define SDL_EXAMPLE_ECC_AGGR                        SDL_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR
#define SDL_EXAMPLE_ECC_RAM_ID                      SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_DRAM1_ECC_RAM_ID
#endif

#if SDL_ICSSM_PR1_PDSP0_IRAM
#define SDL_EXAMPLE_ECC_RAM_ADDR                    (0x48034000u) /* ICSSM PR1 PDSP0 IRAM RAM address */
#define SDL_EXAMPLE_ECC_AGGR                        SDL_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR
#define SDL_EXAMPLE_ECC_RAM_ID                      SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_PR1_PDSP0_IRAM_ECC_RAM_ID
#endif

#if SDL_ICSSM_PR1_PDSP1_IRAM
#define SDL_EXAMPLE_ECC_RAM_ADDR                    (0x48038000u) /* ICSSM PR1 PDSP1 IRAM RAM address */
#define SDL_EXAMPLE_ECC_AGGR                        SDL_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR
#define SDL_EXAMPLE_ECC_RAM_ID                      SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_PR1_PDSP1_IRAM_ECC_RAM_ID
#endif

#if SDL_ICSSM_RAM
#define SDL_EXAMPLE_ECC_RAM_ADDR                    (0x48010000u) /* ICSSM RAM RAM address */
#define SDL_EXAMPLE_ECC_AGGR                        SDL_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR
#define SDL_EXAMPLE_ECC_RAM_ID                      SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_RAM_ECC_RAM_ID
#endif
\endcode
\endcond

This structure defines the elements of ECC  Init configuration
\code{.c}
static SDL_ECC_MemSubType ECC_Test_ICSSM_subMemTypeList[SDL_ICSSM_MAX_MEM_SECTIONS] =
{
    SDL_EXAMPLE_ECC_RAM_ID,
};

static SDL_ECC_InitConfig_t ECC_Test_ICSSM_ECCInitConfig =
{
    .numRams = SDL_ICSSM_MAX_MEM_SECTIONS,
    /**< Number of Rams ECC is enabled  */
    .pMemSubTypeList = &(ECC_Test_ICSSM_subMemTypeList[0]),
    /**< Sub type list  */
};
\endcode

ESM callback function
\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
\code{.c}
int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInst,
                                            SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,
                                            uint32_t index,
                                            uint32_t intSrc,
                                            uintptr_t *arg)
{

    SDL_ECC_MemType eccmemtype;
    SDL_Ecc_AggrIntrSrc eccIntrSrc;
    SDL_ECC_ErrorInfo_t eccErrorInfo;
    int32_t retVal;


    printf("\r\nESM Call back function called : instType 0x%x, intType 0x%x, " \
                "grpChannel 0x%x, index 0x%x, intSrc 0x%x\r\n",
                esmInst, esmIntrType, grpChannel, index, intSrc);
    printf("\r\nTake action\r\n");
    if(esmIntrType == 1u){
        printf("\r\nHigh Priority Interrupt Executed\r\n");
    }
    else{
        printf("\r\nLow Priority Interrupt Executed\r\n");
    }
    retVal = SDL_ECC_getESMErrorInfo(esmInst, intSrc, &eccmemtype, &eccIntrSrc);

    /* Any additional customer specific actions can be added here */
    retVal = SDL_ECC_getErrorInfo(eccmemtype, eccIntrSrc, &eccErrorInfo);

    printf("\r\nECC Error Call back function called : eccMemType %d, errorSrc 0x%x, " \
               "ramId %d, bitErrorOffset 0x%04x%04x, bitErrorGroup %d\r\n",
               eccmemtype, eccIntrSrc, eccErrorInfo.memSubType, (uint32_t)(eccErrorInfo.bitErrorOffset >> 32),
               (uint32_t)(eccErrorInfo.bitErrorOffset & 0x00000000FFFFFFFF), eccErrorInfo.bitErrorGroup);

    if (eccErrorInfo.injectBitErrCnt != 0)
    {
        SDL_ECC_clearNIntrPending(eccmemtype, eccErrorInfo.memSubType, eccIntrSrc, SDL_ECC_AGGR_ERROR_SUBTYPE_INJECT, eccErrorInfo.injectBitErrCnt);
    }
    else
    {
        SDL_ECC_clearNIntrPending(eccmemtype, eccErrorInfo.memSubType, eccIntrSrc, SDL_ECC_AGGR_ERROR_SUBTYPE_NORMAL, eccErrorInfo.bitErrCnt);
    }

    retVal = SDL_ECC_ackIntr(eccmemtype, eccIntrSrc);

    esmError = true;

    return retVal;
}
\endcode
\endcond

Event BitMap for ECC ESM callback for ICSSM
\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
\code{.c}
SDL_ESM_config ECC_Test_esmInitConfig_MAIN =
{
     .esmErrorConfig = {1u, 8u}, /* Self test error config */
     .enableBitmap = {0x00000000u, 0x00000000u, 0x00006000u, 0x00000000u,
                      0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u},
      /**< All events enable: except clkstop events for unused clocks
       *   and PCIE events */
       /* CCM_1_SELFTEST_ERR and _R5FSS0COMPARE_ERR_PULSE_0 */
     .priorityBitmap = {0x00000000u, 0x00000000u, 0x00002000u, 0x00000000u,
                        0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u },
     /**< All events high priority: except clkstop events for unused clocks
      *   and PCIE events */
     .errorpinBitmap = {0x00000000u, 0x00000000u, 0x00006000u, 0x00000000u,
                        0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u},
     /**< All events high priority: except clkstop for unused clocks
      *   and PCIE events */
};
\endcode
\endcond

Initialize ECC memory for the ECC aggregator
\code{.c}
result = SDL_ECC_initMemory(SDL_EXAMPLE_ECC_AGGR, SDL_EXAMPLE_ECC_RAM_ID);
\endcode

\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
Initialize ESM module
\code{.c}
result = SDL_ESM_init(SDL_ESM_INST_MAIN_ESM0, &ECC_Test_esmInitConfig_MAIN, SDL_ESM_applicationCallbackFunction, ptr);
\endcode
\endcond

Initialize ECC parameters for single and double bit error injection
\code{.c}
result = SDL_ECC_init(SDL_EXAMPLE_ECC_AGGR, &ECC_Test_ICSSM_ECCInitConfig);
\endcode

Execute ECC ICSSM single bit inject test
\code{.c}
int32_t ECC_Test_run_ICSSM_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

	DebugP_log("\r\nICSSM Single bit error inject: starting \r\n");

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(SDL_EXAMPLE_ECC_RAM_ADDR);

    /* Run one shot test for ICSSM 1 bit error */
    injectErrorConfig.flipBitMask = 0x02;
    result = SDL_ECC_injectError(SDL_EXAMPLE_ECC_AGGR,
                                 SDL_EXAMPLE_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                 &injectErrorConfig);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];

        DebugP_log("\r\nICSSM Single bit error inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete\r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}
\endcode

Execute ECC ICSSM double bit inject test
\code{.c}
int32_t ECC_Test_run_ICSSM_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

	DebugP_log("\r\nICSSM Double bit error inject: starting \r\n");

    /* Run one shot test for ICSSM 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(SDL_EXAMPLE_ECC_RAM_ADDR);

    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_EXAMPLE_ECC_AGGR,
                                 SDL_EXAMPLE_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);

    if (result != SDL_PASS ) {
       retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nICSSM Double bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p\r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}
\endcode
\endcond

## Example Usage of TCM Parity

Include the below file to access the APIs

\code{.c}
#include "parity_main.h"
\endcode

Below are the macros specifies the values need to set for TCM parity Error forcing
\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
\code{.c}
#define SDL_R5F0ATCM0							(0x7U)
#define SDL_R5F0B0TCM0							(0x700U)
#define	SDL_R5F0B1TCM0							(0x70000U)

#define SDL_R5F0ATCM1							(0x70U)
#define SDL_R5F0B0TCM1							(0x7000U)
#define	SDL_R5F0B1TCM1							(0x700000U)

#define SDL_R5F1ATCM0							(0x7U)
#define SDL_R5F1B0TCM0							(0x700U)
#define	SDL_R5F1B1TCM0							(0x70000U)

#define SDL_R5F1ATCM1							(0x70U)
#define SDL_R5F1B0TCM1							(0x7000U)
#define	SDL_R5F1B1TCM1							(0x700000U)
\endcode
\endcond
\cond (SOC_AM273X)||(SOC_AM273X)
\code{.c}
#define SDL_ESM_MAX_EXAMPLE                (6u)
#define SDL_INTR_GROUP_NUM_2               (2U)
#define SDL_INTR_PRIORITY_LVL_LOW          (0U)
#define SDL_INTR_PRIORITY_LVL_HIGH         (1U)
#define SDL_ENABLE_ERR_PIN                 (1U)

#define SDL_ATCM0_MASK							(0x7U)
#define SDL_ATCM1_MASK							(0x70U)
#define SDL_B0TCM0_MASK							(0x700U)
#define	SDL_B0TCM1_MASK							(0x7000U)
#define SDL_B1TCM0_MASK							(0x70000U)
#define	SDL_B1TCM1_MASK							(0x700000U)
\endcode
\endcond

ESM callback function
\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
\code{.c}
int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInst,
                                            SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,
                                            uint32_t index,
                                            uint32_t intSrc,
                                            uintptr_t *arg)
{

    int32_t retVal = 0;

    printf("\r\nESM Call back function called : instType 0x%x, intType 0x%x, " \
                "grpChannel 0x%x, index 0x%x, intSrc 0x%x \r\n",
                esmInst, esmIntrType, grpChannel, index, intSrc);
    printf("\r\nTake action \r\n");
	/* Clears all the status registers */
	SDL_clearStatusRegs(SetValue);

	esmError = true;

    return retVal;
}
\endcode
\endcond
\cond (SOC_AM273X)||(SOC_AM273X)
\code{.c}
int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
                                           int32_t grpChannel,
                                           int32_t intSrc,
                                           void *arg)
{

    printf("\r\nESM Call back function called : instType 0x%x, " \
                "grpChannel 0x%x, intSrc 0x%x \r\n",
                esmInstType, grpChannel, intSrc);
    printf("\r\nTake action \r\n");

	esmError = true;

    return 0;
}
\endcode
\endcond

Event BitMap for ESM callback for TCM Parity
\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
\code{.c}
SDL_ESM_config Test_esmInitConfig_MAIN =
{
     .esmErrorConfig = {1u, 8u}, /* Self test error config */
     .enableBitmap = {0x0003C000u, 0x00000000u, 0x00000000u, 0x00000000u,
                      0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u},
      /**< All events enable: except clkstop events for unused clocks
       *   and PCIE events */
     .priorityBitmap = {0x0000C000u, 0x000000000u, 0x00000000u, 0x00000000u,
                        0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u },
     /**< All events high priority: except clkstop events for unused clocks
      *   and PCIE events */
     .errorpinBitmap = {0x0003C000u, 0x00000000u, 0x00000000u, 0x00000000u,
                        0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u},
     /**< All events high priority: except clkstop for unused clocks
      *   and PCIE events */
};
\endcode
\endcond
\cond (SOC_AM273X)||(SOC_AWR294X)
\code{.c}
SDL_ESM_NotifyParams ECC_TestparamsMSS[SDL_ESM_MAX_EXAMPLE] =
{
	/* ATCM */
    {
		/* Event BitMap for ESM callback for ATCM0 Single bit*/
		.groupNumber = SDL_INTR_GROUP_NUM_2,
		.errorNumber = SDL_ESMG2_ATCM0_PARITY_ERR,
		.setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
		.enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
		.callBackFunction = &SDL_ESM_applicationCallbackFunction,
    },
	{
		/* Event BitMap for ESM callback for ATCM1 Single bit*/
		.groupNumber = SDL_INTR_GROUP_NUM_2,
		.errorNumber = SDL_ESMG2_ATCM1_PARITY_ERR,
		.setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_HIGH,
		.enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
		.callBackFunction = &SDL_ESM_applicationCallbackFunction,
    },
	/* B0TCM */
	{
		/* Event BitMap for ESM callback for B0TCM0 Single bit*/
		.groupNumber = SDL_INTR_GROUP_NUM_2,
		.errorNumber = SDL_ESMG2_B0TCM0_PARITY_ERR,
		.setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
		.enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
		.callBackFunction = &SDL_ESM_applicationCallbackFunction,
    },
	{
		/* Event BitMap for ESM callback for ATCM1 Single bit*/
		.groupNumber = SDL_INTR_GROUP_NUM_2,
		.errorNumber = SDL_ESMG2_B0TCM1_PARITY_ERR,
		.setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_HIGH,
		.enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
		.callBackFunction = &SDL_ESM_applicationCallbackFunction,
    },
	/* B1TCM */
	{
		/* Event BitMap for ESM callback for B0TCM0 Single bit*/
		.groupNumber = SDL_INTR_GROUP_NUM_2,
		.errorNumber = SDL_ESMG2_B1TCM0_PARITY_ERR,
		.setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
		.enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
		.callBackFunction = &SDL_ESM_applicationCallbackFunction,
    },
	{
		/* Event BitMap for ESM callback for ATCM1 Single bit*/
		.groupNumber = SDL_INTR_GROUP_NUM_2,
		.errorNumber = SDL_ESMG2_B1TCM1_PARITY_ERR,
		.setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_HIGH,
		.enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
		.callBackFunction = &SDL_ESM_applicationCallbackFunction,
    },

};
\endcode
\endcond

\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
Initialize ESM module
\code{.c}
result = SDL_ESM_init(SDL_ESM_INST_MAIN_ESM0, &ECC_Test_esmInitConfig_MAIN, SDL_ESM_applicationCallbackFunction, ptr);
\endcode
\endcond
\cond (SOC_AM273X)||(SOC_AWR294X)
Initialize ESM module
\code{.c}
result = SDL_ESM_init(SDL_ESM_INST_MSS_ESM, &ECC_TestparamsMSS[0],NULL,NULL);
\endcode
\endcond

Execute TCM Parity Error injection for B0TCM0 for R5FSS0 core 0
\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
\code{.c}
	if (retVal == 0)
    {
		DebugP_log("\r\nTCM PARITY:B0TCM0 Started\r\n");
        retValue = SDL_ECC_TCMParity(SDL_R5SS0_CPU0_TCM,\
								 SDL_R5FSS0_CORE0__B0TCM0,\
								 SDL_R5F0B0TCM0);
		DebugP_log("\r\nTCM Parity Status for B0TCM0 = 0x%x\r\n", retValue);
		if (retValue != 0)
		{
			retVal = 0;
			DebugP_log("\r\nB0TCM0 Parity : Completed\r\n");
		}
		else{
			retVal = -1;
			DebugP_log("\r\nB0TCM0 Parity : Failed\r\n");
		}
	}
\endcode
\endcond

\cond (SOC_AM273X)||(SOC_AWR294X)
\code{.c}
	if (retVal == 0)
    {
		DebugP_log("\r\nMSS TCM PARITY: ATCM0 Started\r\n");
        result = SDL_ECC_tcmParity(SDL_TCM_PARITY_ATCM0,\
								 SDL_ATCM0_MASK);
		/* Wait until ESM interrupt happens */
		while(esmError !=true);
		esmError = false;
		if (result != SDL_PASS)
		{
			retVal = -1;
			DebugP_log("\r\nMSS ATCM0 Parity : Failed\r\n");
		}
		else{
			retVal = 0;
			DebugP_log("\r\nMSS ATCM0 Parity : Completed\r\n");
		}
	}
\endcode
\endcond

## Example Usage of DMA Parity

Include the below file to access the APIs

\code{.c}
#include "parity_main.h"
\endcode

ESM callback function
\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
\code{.c}
int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInst,
                                            SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,
                                            uint32_t index,
                                            uint32_t intSrc,
                                            uintptr_t *arg)
{

    int32_t retVal = 0;

    printf("\r\nESM Call back function called : instType 0x%x, " \
                "grpChannel 0x%x, intSrc 0x%x \r\n",
                esmInst, grpChannel, intSrc);
    printf("\r\nTake action \r\n");
	/* Disable parity for TPCC0 */
	SDL_REG32_WR(0x50D18180, 0x1000);

	esmError = true;

    return retVal;
}
\endcode
\endcond
\cond SOC_AM273X || SOC_AWR294X
\code{.c}
int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
                                           int32_t grpChannel,
                                           int32_t intSrc,
                                           void *arg)
{

    printf("\r\nESM Call back function called : instType 0x%x, " \
                "grpChannel 0x%x, intSrc 0x%x \r\n",
                esmInstType, grpChannel, intSrc);
    printf("\r\nTake action \r\n");

#if defined(R5F_INPUTS)
	/* Disable parity for TPCC0 */
	SDL_REG32_WR(0x02120160, 0x00);
#endif
#if defined(C66_INPUTS)
	/* Disable parity for TPCCA */
	SDL_REG32_WR(0x060200BC, 0x4);
	/* Disable parity for TPCCB */
	SDL_REG32_WR(0x060200C0, 0x4);
	/* Disable parity for TPCCC */
	SDL_REG32_WR(0x060200C4, 0x4);
#endif
	esmError = true;

    return 0;
}
\endcode
\endcond

Event BitMap for ESM callback for TCM Parity
\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
\code{.c}
SDL_ESM_config Test_esmInitConfig_MAIN =
{
     .esmErrorConfig = {1u, 8u}, /* Self test error config */
     .enableBitmap = {0x00000000u, 0x80000000u, 0x00000010u, 0x00000000u,
                      0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u},
      /**< All events enable: except clkstop events for unused clocks
       *   and PCIE events */
     .priorityBitmap = {0x00000000u, 0x80000000u, 0x00000010u, 0x00000000u,
                        0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u },
     /**< All events high priority: except clkstop events for unused clocks
      *   and PCIE events */
     .errorpinBitmap = {0x00000000u, 0x80000000u, 0x00000010u, 0x00000000u,
                        0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u},
     /**< All events high priority: except clkstop for unused clocks
      *   and PCIE events */
};
\endcode
\endcond
\cond SOC_AM273X || SOC_AWR294X
\code{.c}
SDL_ESM_NotifyParams ECC_Testparams[SDL_ESM_MSS_MAX_EXAMPLE] =
{
	/* MSS TPTC */
    {
		/* Event BitMap for ESM callback for TPCC_A Single bit*/
		.groupNumber = SDL_INTR_GROUP_NUM_1,
		.errorNumber = SDL_ESMG1_MSS_TPCC_A_INTAGG_ERR,
		.setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
		.enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
		.callBackFunction = &SDL_ESM_applicationCallbackFunction,
    },
	/* MSS TPTC */
    {
		/* Event BitMap for ESM callback for TPCC_B Single bit*/
		.groupNumber = SDL_INTR_GROUP_NUM_1,
		.errorNumber = SDL_ESMG1_MSS_TPCC_B_INTAGG_ERR,
		.setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_HIGH,
		.enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
		.callBackFunction = &SDL_ESM_applicationCallbackFunction,
    },


};
SDL_ESM_NotifyParams ECC_TestparamsDSS[SDL_ESM_DSS_MAX_EXAMPLE] =
{
	/* DSS TPTCA */
    {
		/* Event BitMap for ESM callback for TPCC_A Single bit*/
		.groupNumber = SDL_INTR_GROUP_NUM_1,
		.errorNumber = SDL_DSS_ESMG1_DSS_TPCC_A_INTAGG_ERR,
		.setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
		.enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
		.callBackFunction = &SDL_ESM_applicationCallbackFunction,
    },
	/* DSS TPTCB */
    {
		/* Event BitMap for ESM callback for TPCC_B Single bit*/
		.groupNumber = SDL_INTR_GROUP_NUM_1,
		.errorNumber = SDL_DSS_ESMG1_DSS_TPCC_B_INTAGG_ERR,
		.setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_HIGH,
		.enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
		.callBackFunction = &SDL_ESM_applicationCallbackFunction,
    },
	/* DSS TPTCC */
    {
		/* Event BitMap for ESM callback for TPCC_C Single bit*/
		.groupNumber = SDL_INTR_GROUP_NUM_1,
		.errorNumber = SDL_DSS_ESMG1_DSS_TPCC_C_INTAGG_ERR,
		.setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
		.enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
		.callBackFunction = &SDL_ESM_applicationCallbackFunction,
    },
};
\endcode
\endcond

\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
Initialize ESM module
\code{.c}
result = SDL_ESM_init(SDL_ESM_INST_MAIN_ESM0, &Test_esmInitConfig_MAIN, SDL_ESM_applicationCallbackFunction, ptr);
\endcode
\endcond

\cond SOC_AM273X || SOC_AWR294X
Initialize ESM module
\code{.c}
result = SDL_ESM_init(SDL_ESM_INST_MSS_ESM, &ECC_Testparams[0],NULL,NULL);
\endcode
\endcond

Execute TPCC Parity Error injection
\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
\code{.c}
	if (retVal == 0)
    {
		DebugP_log("\r\nTPCC PARITY : TPCC0 Started\r\n");
        retVal = SDL_ECC_TPCCParity(SDL_TPCC0, \
								 0x11);
		DebugP_log("\r\nParam Register = %x\r\n", retVal);

		/* wait for delay */
		for (i =0 ; i < 200 ; i = i + 1 );

		/* Wait until ESM interrupt happens */
		while(esmError !=true);

		if(esmError == true)
		{
			DebugP_log("\r\nTPCC PARITY : TPCC0 Completed\r\n");
			esmError = false;
		}
		else{
			result = SDL_EFAIL;
		}
	}
\endcode
\endcond
\cond SOC_AM273X || SOC_AWR294X
\code{.c}
	if (retVal == 0)
    {
		DebugP_log("\r\nMSS TPCCA Parity \r\n");
        paramstatus = SDL_ECC_TPCCParity(SDL_TPCC0A, \
								 0x11);
		DebugP_log("\r\nParam Register = %x\r\n", paramstatus);

		/* Wait until ESM interrupt happens */
		while(esmError !=true);
		/*wait for delay */
		for (i =0 ; i < 200 ; i = i + 1 );

		if(esmError == true)
		{
			DebugP_log("\r\nMSS TPCCA Parity : Completed\r\n");
			esmError = false;
		}
		else{
			retVal = -1;
			DebugP_log("\r\nMSS TPCCA Parity : Failed\r\n");
		}
	}
\endcode
\endcond

\cond SOC_AM273X
## Example Usage of DSS L2 Parity

Include the below file to access the APIs

\code{.c}
#include "ecc_main.h"
\endcode

Below are the macros specifies the ESM init and DSS DSP parity registers used to inject parity error
\code{.c}
#define SDL_ESM_MAX_DSS_EXAMPLE_AGGR				(1u)

#define SDL_INTR_GROUP_NUM                          (1U)
#define SDL_INTR_PRIORITY_LVL_LOW                   (0U)
#define SDL_INTR_PRIORITY_LVL_HIGH                  (1U)
#define SDL_ENABLE_ERR_PIN                          (1U)

#define SDL_INJECT_PARITY                           (0x01u)

#define SDL_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB1     (0x06020074u)
#define SDL_INITIAL_VALUE                           (0x11u)
\endcode

ESM callback function
\code{.c}
int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
										   int32_t grpChannel,
										   int32_t intSrc,
										   void *arg)
{

    DebugP_log("\r\nESM Call back function called : instType 0x%x, " \
                "grpChannel 0x%x, intSrc 0x%x \r\n",
                esmInstType, grpChannel, intSrc);
    DebugP_log(" \r\nTake action \r\n");
    if(esmInstType == 1u){
        DebugP_log("\r\nHigh Priority Interrupt Executed\r\n");
    }
    else{
        DebugP_log("\r\nLow Priority Interrupt Executed\r\n");
    }

    /*
     *Disable the parity by clearing DSS_CTRL.DSS_DSP_L2RAM_PARITY_CTRL.DSS_DSP_L2RAM_PARITY_CTRL_ENABLE
     *Disable register field
     */
    SDL_REG32_WR(SDL_DSS_DSP_L2RAM_PARITY_CTRL, SDL_ECC_DSS_L2RAM_PARITY_ERROR_CLEAR);

	DebugP_log("\r\nclearing DSS_CTRL.DSS_DSP_L2RAM_PARITY_CTRL.DSS_DSP_L2RAM_PARITY_CTRL_ENABLE\r\n");

    esmError = true;

    return 0;
}
\endcode

Event BitMap for Parity ESM callback for DSS L2 RAM
\code{.c}
SDL_ESM_NotifyParams Parity_TestparamsDSS[SDL_ESM_MAX_DSS_EXAMPLE_AGGR] =
{
     {
           /* Event BitMap for ECC ESM callback for DSS Single bit*/
           .groupNumber = SDL_INTR_GROUP_NUM,
           .errorNumber = SDL_DSS_ESMG1_DSS_DSP_L2_PARITY_ERR_VB0_EVEN,
           .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
           .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
           .callBackFunction = &SDL_ESM_applicationCallbackFunction,
      },
};
\endcode

Enable the parity
\code{.c}
SDL_REG32_WR(SDL_DSS_DSP_L2RAM_PARITY_CTRL, SDL_ECC_DSS_L2RAM_PARITY_ENABLE);
\endcode

DSS L2 parity init*/
\code{.c}
SDL_ECC_dss_l2_parity_init();
\endcode

Initialize ESM module
\code{.c}
result = SDL_ESM_init(SDL_ESM_INST_DSS_ESM, &Parity_TestparamsDSS[counter],NULL,NULL);
\endcode

On a parity error from a particular bank, reading the register DSS_CTRL.DSS_DSP_L2RAM_PARITY_ERR_STATUS_VBx gives the address location
\code{.c}
injectErrAdd = SDL_REG32_RD(SDL_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB1);
\endcode

DSS L2 parity error inject
\code{.c}
SDL_ECC_dss_l2_parity_errorInject(SDL_INJECT_PARITY, injectErrAdd, SDL_INITIAL_VALUE);
\endcode

Waiting for ESM Interrupt
\code{.c}
do
{
	timeOutCnt += 1;
	if (timeOutCnt > maxTimeOutMilliSeconds)
	{
		result = SDL_EFAIL;
		break;
	}
} while (esmError == false);
\endcode

## Example Usage of DSS L1 Parity

Include the below file to access the APIs

\code{.c}
#include "ecc_main.h"
\endcode

Below are the macros specifies the ESM init and DSS DSP L1 parity registers used to inject parity error
\code{.c}
#define SDL_ESM_MAX_DSS_EXAMPLE_AGGR                (1u)

#define SDL_INTR_GROUP_NUM                          (1U)
#define SDL_INTR_PRIORITY_LVL_LOW                   (0U)
#define SDL_INTR_PRIORITY_LVL_HIGH                  (1U)
#define SDL_ENABLE_ERR_PIN                          (1U)

#define SDL_SEC                                     (0x01u)

#define SDL_DSS_L2_ORIGIN                           (0x00800000u)
#define SDL_DSS_L1P_ORIGIN                          (0x00E00000U)

#define SDL_DSS_INTH_INT_ID_IDMAINT1                (14)

#define SDL_DSS_ICFGF_L1PCFG                        (0x01840020U)
#define SDL_DSS_ICFGF_L1DCFG                        (0x01840040U)
\endcode

ESM Callback Function
\code{.c}
int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
										   int32_t grpChannel,
										   int32_t intSrc,
										   void *arg)
{

    printf("\r\nESM Call back function called : instType 0x%x, " \
                "grpChannel 0x%x, intSrc 0x%x \r\n",
                esmInstType, grpChannel, intSrc);
    printf(" \r\nTake action \r\n");
    if(esmInstType == 1u){
        printf("\r\nHigh Priority Interrupt Executed\r\n");
    }
    else{
        printf("\r\nLow Priority Interrupt Executed\r\n");
    }

    esmError = true;

    return 0;
}
\endcode

IDMA Callback Function
\code{.c}
int32_t SDL_IDMA1_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
                                           int32_t grpChannel,
                                           int32_t intSrc,
                                           void *arg)
{
    idmaTransferComplete = true;

    return 0;
}
\endcode

Event BitMap for ESM callback for DSP L1 Parity Errors
\code{.c}
SDL_ESM_NotifyParams Parity_TestparamsDSS[SDL_ESM_MAX_DSS_EXAMPLE_AGGR] =
{
     {
           /* Event BitMap for Parity ESM callback for DSS Single bit*/
           .groupNumber = SDL_INTR_GROUP_NUM,
           .errorNumber = SDL_DSS_ESMG1_DSS_DSP_L1P_PARITY,
           .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
           .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
           .callBackFunction = &SDL_ESM_applicationCallbackFunction,
      },
};
\endcode

Dummy Function
\code{.c}
#pragma CODE_SECTION(EDC_dummyFunction, ".func");
int32_t EDC_dummyFunction(void)
{
    int32_t a = 0;
    int32_t b = 4;
    int32_t i;
    for (i = 0; i <= 3; i++)
    {
        b = b + 7;
        a = a + b;
    }
    return a;
}
\endcode

Disable the L1P and L1D Cache
\code{.c}
SDL_REG32_WR(SDL_DSS_ICFGF_L1PCFG, 0x00); /*L1PCFG is set to 0*/
SDL_REG32_WR(SDL_DSS_ICFGF_L1DCFG, 0x00); /*L1DCFG is set to 0*/
\endcode

Initialize ESM module
\code{.c}
result = SDL_ESM_init(SDL_ESM_INST_DSS_ESM, &Parity_TestparamsDSS[counter],NULL,NULL);
\endcode

Configuring the IDMA
\code{.c}
/* Configuring the ESM */
intrParams.callbackArg = (uintptr_t)SDL_ESM_INST_DSS_ESM;

/* Configuring the IDMA1 DED interrupt */
intrParams.intNum = SDL_DSS_INTH_INT_ID_IDMAINT1;
intrParams.callback = (pSDL_DPL_InterruptCallbackFunction)SDL_IDMA1_applicationCallbackFunction;

/* Register call back function for vector Config Interrupt */
SDL_DPL_registerInterrupt(&intrParams, &SDL_Parity_CfgHwiPHandle);

/*Enable the IDMA 1 interrupt*/
SDL_DPL_enableInterrupt(SDL_DSS_INTH_INT_ID_IDMAINT1);
\endcode

Enable L1P EDC command
\code{.c}
retVal = SDL_ECC_dss_l1p_edc_CMD_EN();
\endcode

IDMA transfer
\code{.c}
SDL_ECC_IDMA1_transfer(SDL_DSS_L2_ORIGIN, SDL_DSS_L1P_ORIGIN);
\endcode

Call Dummy function
\code{.c}
EDC_dummyFunction()
\endcode

Suspend L1P EDC command
\code{.c}
retVal = SDL_ECC_dss_l1p_CMD_SUSP();
\endcode

Read and Toggle single bit
\code{.c}
rd_data = SDL_REG32_RD(SDL_DSS_L2_ORIGIN);
rd_data = rd_data ^ SDL_SEC;
SDL_REG32_WR(SDL_DSS_L2_ORIGIN, rd_data);
\endcode

IDMA transfer
\code{.c}
SDL_ECC_IDMA1_transfer(SDL_DSS_L2_ORIGIN, SDL_DSS_L1P_ORIGIN);
\endcode

Enable L1P EDC command
\code{.c}
retVal = SDL_ECC_dss_l1p_edc_CMD_EN();
\endcode

IDMA transfer
\code{.c}
SDL_ECC_IDMA1_transfer(SDL_DSS_L2_ORIGIN, SDL_DSS_L1P_ORIGIN);
\endcode

Waiting for ESM Interrupt
\code{.c}
do
{
	timeOutCnt += 1;
	if (timeOutCnt > maxTimeOutMilliSeconds)
	{
		result = SDL_EFAIL;
		break;
	}
} while (esmError == false);
\endcode

## Example Usage of DSS L2 EDC

Include the below file to access the APIs

\code{.c}
#include "ecc_main.h"
\endcode

Below are the macros specifies the ESM init and DSS DSP L1 parity registers used to inject parity error
\code{.c}
#define SDL_ESM_MAX_DSS_EXAMPLE_AGGR				(2u)

#define SDL_INTR_GROUP_NUM                          (1U)
#define SDL_INTR_PRIORITY_LVL_LOW                   (0U)
#define SDL_INTR_PRIORITY_LVL_HIGH                  (1U)
#define SDL_ENABLE_ERR_PIN                          (1U)

#define SDL_EDC_SEC                       			(0x01u)
#define SDL_EDC_DED                       			(0x03u)

#define SDL_DSS_L2_ORIGIN                           (0x00800000u)
#define SDL_DSS_INTH_INT_ID_IDMAINT1                (14)

#define SDL_DSS_ICFGF_L1PCFG                        (0x01840020U)
#define SDL_DSS_ICFGF_L1DCFG                        (0x01840040U)
\endcode

ESM Callback Function
\code{.c}
int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
										   int32_t grpChannel,
										   int32_t intSrc,
										   void *arg)
{

    printf("\r\nESM Call back function called : instType 0x%x, " \
                "grpChannel 0x%x, intSrc 0x%x \r\n",
                esmInstType, grpChannel, intSrc);
    printf(" \r\nTake action \r\n");
    if(esmInstType == 1u){
        printf("\r\nHigh Priority Interrupt Executed\r\n");
    }
    else{
        printf("\r\nLow Priority Interrupt Executed\r\n");
    }


	/*Clear Error Detection Address Register*/
    SDL_REG32_WR(SDL_DSP_ICFG_L2EDADDR, 0x00u);

    esmError = true;

    return 0;
}
\endcode

IDMA Callback Function
\code{.c}
int32_t SDL_IDMA1_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
                                           int32_t grpChannel,
                                           int32_t intSrc,
                                           void *arg)
{
    printf("\r\nIDMA1 call back function called. \r\n");

    idmaTransferComplete = true;

    return 0;
}
\endcode

Event BitMap for ESM callback for DSP EDC Errors
\code{.c}
SDL_ESM_NotifyParams EDC_TestparamsDSS[SDL_ESM_MAX_DSS_EXAMPLE_AGGR] =
{
     {
           /* Event BitMap for EDC ESM callback for DSS Single bit*/
           .groupNumber = SDL_INTR_GROUP_NUM,
           .errorNumber = SDL_DSS_ESMG1_DSS_DSP_L2_SEC_ERR,
           .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
           .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
           .callBackFunction = &SDL_ESM_applicationCallbackFunction,
      },
      {
           /* Event BitMap for EDC ESM callback for DSS Double bit*/
           .groupNumber = SDL_INTR_GROUP_NUM,
           .errorNumber = SDL_DSS_ESMG1_DSS_DSP_L2_DED_ERR,
           .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_HIGH,
           .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
           .callBackFunction = &SDL_ESM_applicationCallbackFunction,
      },

};
\endcode

Dummy Function
\code{.c}
#pragma CODE_SECTION(EDC_dummyFunction, ".func");
int32_t EDC_dummyFunction(void)
{
    int32_t a = 0;
    int32_t b = 4;
    int32_t i;
    for (i = 0; i <= 3; i++)
    {
        b = b + 7;
        a = a + b;
    }
    return a;
}
\endcode

Disable the L1P and L1D Cache
\code{.c}
SDL_REG32_WR(SDL_DSS_ICFGF_L1PCFG, 0x00); /*L1PCFG is set to 0*/
SDL_REG32_WR(SDL_DSS_ICFGF_L1DCFG, 0x00); /*L1DCFG is set to 0*/
\endcode

Initialize ESM module
\code{.c}
result = SDL_ESM_init(SDL_ESM_INST_DSS_ESM, &Parity_TestparamsDSS[counter],NULL,NULL);
\endcode

Configuring the IDMA
\code{.c}
/* Configuring the ESM */
intrParams.callbackArg = (uintptr_t)SDL_ESM_INST_DSS_ESM;

/* Configuring the IDMA1 DED interrupt */
intrParams.intNum = SDL_DSS_INTH_INT_ID_IDMAINT1;
intrParams.callback = (pSDL_DPL_InterruptCallbackFunction)SDL_IDMA1_applicationCallbackFunction;

/* Register call back function for vector Config Interrupt */
SDL_DPL_registerInterrupt(&intrParams, &SDL_Parity_CfgHwiPHandle);

/*Enable the IDMA 1 interrupt*/
SDL_DPL_enableInterrupt(SDL_DSS_INTH_INT_ID_IDMAINT1);
\endcode

Enable L2 EDC command
\code{.c}
retVal = SDL_ECC_dss_l2_edc_CMD_EN();
\endcode

IDMA transfer
\code{.c}
SDL_ECC_IDMA1_transfer(SDL_DSS_L2_ORIGIN, SDL_DSS_L2_ORIGIN);
\endcode

Call Dummy function
\code{.c}
EDC_dummyFunction();
\endcode

Suspend L2 EDC command
\code{.c}
retVal = SDL_ECC_dss_l2_CMD_SUSP();
\endcode

Read and Toggle single bit
\code{.c}
rd_data = SDL_REG32_RD(SDL_DSS_L2_ORIGIN);
rd_data = rd_data ^ SDL_SEC;
SDL_REG32_WR(SDL_DSS_L2_ORIGIN, rd_data);
\endcode

Read and Toggle double bit
\code{.c}
rd_data = SDL_REG32_RD(SDL_DSS_L2_ORIGIN);
rd_data = rd_data ^ SDL_EDC_DED;
SDL_REG32_WR(SDL_DSS_L2_ORIGIN, rd_data);
\endcode

Enable L2 EDC command
\code{.c}
retVal = SDL_ECC_dss_l2_edc_CMD_EN();
\endcode

Call Dummy function
\code{.c}
EDC_dummyFunction();
\endcode

Waiting for ESM Interrupt
\code{.c}
do
{
	timeOutCnt += 1;
	if (timeOutCnt > maxTimeOutMilliSeconds)
	{
		result = SDL_EFAIL;
		break;
	}
} while (esmError == false);
\endcode

## Example Usage of DSS EDC Errors

Include the below file to access the APIs

\code{.c}
#include "ecc_main.h"
\endcode

Below are the macros specifies the ESM init and DSS DSP EDC registers used to inject EDC error
\code{.c}
#define SDL_ESM_MAX_DSS_EXAMPLE_AGGR				(2u)

#define SDL_INTR_GROUP_NUM                          (1U)
#define SDL_INTR_PRIORITY_LVL_LOW                   (0U)
#define SDL_INTR_PRIORITY_LVL_HIGH                  (1U)
#define SDL_ENABLE_ERR_PIN                          (1U)

#define SDL_DSS_ECC_NC_ITERRUPT_ID					(111)
#define SDL_DSS_ECC_C_ITERRUPT_ID					(112)

#define SDL_ENABLE_L1D_DATA_MASK_FLG                (0x01u)
#define SDL_ENABLE_L1D_TAG_MASK_FLG                 (0x02u)
#define SDL_ENABLE_L2_TAG_MASK_FLG                  (0x04u)
#define SDL_ENABLE_L2_SNOP_MASK_FLG                 (0x08u)
#define SDL_ENABLE_L2_MPPA_MASK_FLG                 (0x10u)
#define SDL_ENABLE_L2_LRU_MASK_FLG                  (0x20u)
#define SDL_ENABLE_L1P_TAG_MASK_FLG                 (0x40u)

/* To test above memories one by one then assign any of the above macro to SDL_MEMORY_ENABLE macro*/
#define SDL_MEMORY_ENABLE							SDL_ENABLE_L1D_DATA_MASK_FLG
\endcode

ESM Callback Function
\code{.c}
int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
										   int32_t grpChannel,
										   int32_t intSrc,
										   void *arg)
{

	DebugP_log("\r\nESM Call back function called : instType 0x%x, " \
                "grpChannel 0x%x, intSrc 0x%x \r\n",
                esmInstType, grpChannel, intSrc);
    DebugP_log("\r\nTake action \r\n");

	/* Write to DSP_ICFG__EDCINTMASK REGISTER and DSP_ICFG__EDCINTFLG REGISTER
	 * TO disable PROPOGATION OF EXCEPTION
	 */

    SDL_REG32_WR(SDL_DSP_ICFG_EDCINTMASK, SDL_DSP_ICFG_DISABLE);
    SDL_REG32_WR(SDL_DSP_ICFG_EDCINTFLG, SDL_DSP_ICFG_DISABLE);

    esmError = true;

    return 0;
}
\endcode

ECC DED Callback Function
\code{.c}
int32_t SDL_ECC_DED_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
                                           int32_t grpChannel,
                                           int32_t intSrc,
                                           void *arg)
{

    DebugP_log("\r\nECC DED Call back function called : instType 0x%x, " \
                "grpChannel 0x%x, intSrc 0x%x \r\n",
                esmInstType, grpChannel, intSrc);
    DebugP_log("\r\nTake action \r\n");

    /* Write to DSP_ICFG__EDCINTMASK REGISTER and DSP_ICFG__EDCINTFLG REGISTER
     * TO disable PROPOGATION OF EXCEPTION
     */

    SDL_REG32_WR(SDL_DSP_ICFG_EDCINTMASK, SDL_DSP_ICFG_DISABLE);
    SDL_REG32_WR(SDL_DSP_ICFG_EDCINTFLG, SDL_DSP_ICFG_DISABLE);

    esmError = true;

    return 0;
}
\endcode

ECC SEC Callback Function
\code{.c}
int32_t SDL_ECC_SEC_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
                                           int32_t grpChannel,
                                           int32_t intSrc,
                                           void *arg)
{

    DebugP_log("\r\nECC SEC Call back function called : instType 0x%x, " \
                "grpChannel 0x%x, intSrc 0x%x \r\n",
                esmInstType, grpChannel, intSrc);
    DebugP_log("\r\nTake action \r\n");

    /* Write to DSP_ICFG__EDCINTMASK REGISTER and DSP_ICFG__EDCINTFLG REGISTER
     * TO disable PROPOGATION OF EXCEPTION
     */

    SDL_REG32_WR(SDL_DSP_ICFG_EDCINTMASK, SDL_DSP_ICFG_DISABLE);
    SDL_REG32_WR(SDL_DSP_ICFG_EDCINTFLG, SDL_DSP_ICFG_DISABLE);

    esmError = true;

    return 0;
}
\endcode

Event BitMap for ESM callback for DSP EDC Errors
\code{.c}
SDL_ESM_NotifyParams EDC_TestparamsDSS[SDL_ESM_MAX_DSS_EXAMPLE_AGGR] =
{
    {
		/* Event BitMap for EDC ESM callback for DSS Single bit*/
		.groupNumber = SDL_INTR_GROUP_NUM,
		.errorNumber = SDL_DSS_ESMG1_DSS_DSP_EDC_SEC_ERR,
		.setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
		.enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
		.callBackFunction = &SDL_ESM_applicationCallbackFunction,
    },
	{
		/* Event BitMap for EDC ESM callback for DSS Double bit*/
		.groupNumber = SDL_INTR_GROUP_NUM,
		.errorNumber = SDL_DSS_ESMG1_DSS_DSP_EDC_DED_ERR,
		.setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_HIGH,
		.enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
		.callBackFunction = &SDL_ESM_applicationCallbackFunction,
	},
};
\endcode

Initialize ESM module
\code{.c}
result = SDL_ESM_init(SDL_ESM_INST_DSS_ESM, &EDC_TestparamsDSS[counter],NULL,NULL);
\endcode

Configuring the ESM
\code{.c}
intrParams.callbackArg = (uintptr_t)SDL_ESM_INST_DSS_ESM;
\endcode

Configuring the ECC DED interrupt, Register call back function for vector Config Interrupt and Enable the DED interrupt
\code{.c}
intrParams.intNum = SDL_DSS_ECC_NC_ITERRUPT_ID;
intrParams.callback = (pSDL_DPL_InterruptCallbackFunction)SDL_ECC_DED_applicationCallbackFunction;

SDL_DPL_registerInterrupt(&intrParams, &SDL_EDC_CfgHwiPHandle);

SDL_DPL_enableInterrupt(SDL_DSS_ECC_NC_ITERRUPT_ID);
\endcode

Configuring the ECC SEC interrupt, Register call back function for vector Config Interrupt and Enable the SEC interrupt
\code{.c}
intrParams.intNum = SDL_DSS_ECC_C_ITERRUPT_ID;
intrParams.callback = (pSDL_DPL_InterruptCallbackFunction)SDL_ECC_SEC_applicationCallbackFunction;

SDL_DPL_registerInterrupt(&intrParams, &SDL_EDC_CfgHwiPHandle);

SDL_DPL_enableInterrupt(SDL_DSS_ECC_C_ITERRUPT_ID);
\endcode

Write to DSP_ICFG__EDCINTMASK and DSP_ICFG__EDCINTFLG registers to enable and propagate an DSS DSP Memory
\code{.c}
SDL_ECC_DSP_Aggregated_EDC_Errors(SDL_MEMORY_ENABLE);
\endcode

Waiting for ESM Interrupt
\code{.c}
do
{
	timeOutCnt += 1;
	if (timeOutCnt > maxTimeOutMilliSeconds)
	{
		result = SDL_EFAIL;
		break;
	}
} while (esmError == false);
\endcode

Read MASK register value after ESM callback
\code{.c}
readValue = SDL_REG32_RD(SDL_DSP_ICFG_EDCINTMASK);
\endcode

Read FLG register value after ESM callback
\code{.c}
readValue = SDL_REG32_RD(SDL_DSP_ICFG_EDCINTFLG);
\endcode
\endcond

## API

\ref SDL_ECC_AGGR_API
