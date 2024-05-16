/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 *  \file v0/gpmc.h
 *
 *  \brief GPMC Driver API/interface file.
 */

/**
 *  \defgroup DRV_GPMC_MODULE APIs for GPMC
 *  \ingroup DRV_MODULE
 *
 *  This file containing the GPMC API.
 *
 *  The GPMC header file should be included in an application as follows:
 *  \code
 *  #include <drivers/gpmc.h>
 *  \endcode
 *
 *  @{
 */

#ifndef _GPMC__H_
#define _GPMC__H_


/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <kernel/dpl/SystemP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/CacheP.h>
#include <kernel/dpl/HwiP.h>
#include <drivers/hw_include/csl_types.h>
#include <drivers/hw_include/cslr_gpmc.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
/**
*  \name GPMC CHIP select
*
*   Macros which can be used as to select CHIP select instance for GPMC.
*
* @{
*/
#define GPMC_CHIP_SELECT_CS0                            (0U)
#define GPMC_CHIP_SELECT_CS1                            (1U)
#define GPMC_CHIP_SELECT_CS2                            (2U)
#define GPMC_CHIP_SELECT_CS3                            (3U)
/** @} */

/**
*  \name GPMC Interupt status flags
*
*   Macros which can be used as 'interupt' parameter to GPMC_interuptStatusGet,
*   GPMC_interuptStatusClear APIs.
*
* @{
*/
#define GPMC_FIFOEVENT_STATUS                         (0U)
#define GPMC_TERMINALCOUNT_STATUS                     (1U)
#define GPMC_WAIT0EDGEDETECTION_STATUS                (2U)
#define GPMC_WAIT1EDGEDETECTION_STATUS                (3U)
/** @} */

/**
*  \name GPMC Interupt Flags
*
*  Macros which can be used as 'interupt' parameter to GPMC_enableInterupt,
*  #GPMC_disableInterupt APIs.
*
* @{
*/
#define GPMC_FIFOEVENT_INT                            (0U)
#define GPMC_TERMINALCOUNT_INT                        (1U)
#define GPMC_WAIT0EDGEDETECTION_INT                   (2U)
#define GPMC_WAIT1EDGEDETECTION_INT                   (3U)
/** @} */


/**
*  \name GPMC Interupt Flags
*
* Macros which can be used as 'mode' parameter to GPMC_prefetchPostWriteConfigEnable API.
*
* @{
*/
#define GPMC_PREFETCH_ACCESSMODE_READ                 (0U)
#define GPMC_PREFETCH_ACCESSMODE_WRITE                (1U)
/** @} */


/**
*  \name GPMC ECC algorithm
*
* Macros which can be used to set GPMC ECC engine algorithm.
*
* @{
*/
#define GPMC_ECC_ALGORITHM_HAMMINGCODE                (0U)
#define GPMC_ECC_ALGORITHM_BCH                        (1U)
/** @} */

/**
*  \name GPMC ECC error correvel level.
*
* Macros which can be used to set GPMC ECC engine error correction level.
*
* @{
*/
#define GPMC_ECC_BCH_ERRCORRCAP_UPTO_4BITS            (0U)
#define GPMC_ECC_BCH_ERRCORRCAP_UPTO_8BITS            (1U)
#define GPMC_ECC_BCH_ERRCORRCAP_UPTO_16BITS           (2U)
/** @} */


/**
*  \name GPMC ECC Result Register
*
* Macros which can be used to select ECC result register for GPMC ECC engine.
*
* @{
*/
#define GPMC_ECCPOINTER_RESULT_1                      (1U)
#define GPMC_ECCPOINTER_RESULT_2                      (2U)
#define GPMC_ECCPOINTER_RESULT_3                      (3U)
#define GPMC_ECCPOINTER_RESULT_4                      (4U)
#define GPMC_ECCPOINTER_RESULT_5                      (5U)
#define GPMC_ECCPOINTER_RESULT_6                      (6U)
#define GPMC_ECCPOINTER_RESULT_7                      (7U)
#define GPMC_ECCPOINTER_RESULT_8                      (8U)
#define GPMC_ECCPOINTER_RESULT_9                      (9U)
/** @} */

/**
*  \name GPMC ECC Size Config
*
* Macros which can be used to set length of used and unsued bytes in the ECC value.
* @{
*/
#define GPMC_ECC_SIZE_0                               (0U)
#define GPMC_ECC_SIZE_1                               (1U)
/** @} */

/**
*  \name GPMC ECC Result Register Size
*
* Macros which can be used to set ECC size for ECC result register.
* @{
*/
#define GPMC_ECC_RESULT_1                             (1U)
#define GPMC_ECC_RESULT_2                             (2U)
#define GPMC_ECC_RESULT_3                             (3U)
#define GPMC_ECC_RESULT_4                             (4U)
#define GPMC_ECC_RESULT_5                             (5U)
#define GPMC_ECC_RESULT_6                             (6U)
#define GPMC_ECC_RESULT_7                             (7U)
#define GPMC_ECC_RESULT_8                             (8U)
#define GPMC_ECC_RESULT_9                             (9U)
/** @} */

/**
*  \name GPMC BCH RESULT
*
* Macros which can be used to get BCH syndrome polynomial in GPMC_eccBchResultGet API.
* @{
*/
#define GPMC_BCH_RESULT0                             (0U)
#define GPMC_BCH_RESULT1                             (1U)
#define GPMC_BCH_RESULT2                             (2U)
#define GPMC_BCH_RESULT3                             (3U)
#define GPMC_BCH_RESULT4                             (4U)
#define GPMC_BCH_RESULT5                             (5U)
#define GPMC_BCH_RESULT6                             (6U)
/** @} */

/**
*  \name GPMC CHIP Select SIZE
*
* Macros which can be used to set chip select address size.
* @{
*/
#define GPMC_CS_MASK_ADDR_SIZE_256MB                            (0x00U)
#define GPMC_CS_MASK_ADDR_SIZE_128MB                            (0x08U)
#define GPMC_CS_MASK_ADDR_SIZE_64MB                             (0x0cU)
#define GPMC_CS_MASK_ADDR_SIZE_32MB                             (0x0eU)
#define GPMC_CS_MASK_ADDR_SIZE_16MB                             (0x0fU)
/** @} */

/**
 * \brief  This macro used to make the conf value which is used to configure the
 *         CS signal timing configuration.
 *
 * \param  CSWrOffTime    CS# de-assertion time from start cycle time for write
 *                        accesses in GPMC_FCLK cycles.
 *
 * \param  CSRdOffTime    CS# de-assertion time from start cycle time for read
 *                        accesses in GPMC_FCLK cycles
 *
 * \param  CSExtDelayFlag Flag to indicate whether to add half GPMC_FCLK delay
 *                        to CS or not. \n
 *                        This can take one of the following values : \n
 *                        CSL_GPMC_CONFIG2_CSEXTRADELAY_DELAYED : CS# Timing control
 *                                                                 signal is delayed of
 *                                                                 half GPMC_FCLK cycle. \n
 *                        CSL_GPMC_CONFIG2_CSEXTRADELAY_NOTDELAYED : CS# Timing control
 *                                                                    signal is not delayed. \n
 *
 * \param  CSOnTime       CS# assertion time from start cycle time in GPMC_FCLK
 *                        cycles.
 *
 */
#define GPMC_CS_TIMING_CONFIG(CSWrOffTime, CSRdOffTime, CSExtDelayFlag, \
                              CSOnTime)                                 \
    ((uint32_t) (((CSWrOffTime) << CSL_GPMC_CONFIG2_CSWROFFTIME_SHIFT)    \
                 & CSL_GPMC_CONFIG2_CSWROFFTIME_MASK) |                   \
     (((CSRdOffTime) << CSL_GPMC_CONFIG2_CSRDOFFTIME_SHIFT)               \
      & CSL_GPMC_CONFIG2_CSRDOFFTIME_MASK) |                              \
     (((CSExtDelayFlag) << CSL_GPMC_CONFIG2_CSEXTRADELAY_SHIFT)           \
      & CSL_GPMC_CONFIG2_CSEXTRADELAY_MASK) |                             \
     (((CSOnTime) << CSL_GPMC_CONFIG2_CSONTIME_SHIFT)                     \
      & CSL_GPMC_CONFIG2_CSONTIME_MASK))


/**
 * \brief  This macro used to make the conf value which is used to configure the
 *         ADV# signal timing configuration.
 *
 * \param  ADVAADMuxWrOffTime    ADV# de-assertion time in GPMC_FCLK cycles for
 *                               first address phase when using the AAD-Mux
 *                               prorocol.
 *
 * \param  ADVAADMuxRdOffTime    ADV# de-assertion time in GPMC_FCLK cycles for
 *                               first address phase when using the AAD-Mux
 *                               prorocol.
 *
 * \param  ADVWrOffTime          ADV# de-assertion time in GPMC_FCLK cycles from
 *                               start cycle time for write accesses
 *
 * \param  ADVRdOffTime          ADV# de-assertion time in GPMC_FCLK cycles from
 *                               start cycle time for write accesses
 *
 * \param  ADVExtDelayFlag       Flag to indicate whether to add half GPMC_FCLK
 *                               delay to ADV or not.  \n
 *                               This can take one of the following values :   \n
 *                               CSL_GPMC_CONFIG3_ADVEXTRADELAY_DELAYED : ADV# Timing control
 *                                                                        signal is delayed of
 *                                                                        half GPMC_FCLK cycle. \n
 *                               CSL_GPMC_CONFIG3_ADVEXTRADELAY_NOTDELAYED : ADV# Timing control
 *                                                                          signal is not delayed. \n
 *
 * \param  ADVAADMuxOnTime       ADV# assertion time in GPMC_FCLK cycles for
 *                               first address phase when using the
 *                               AAD-Multiplexed protocol.
 *
 * \param  ADVOnTime             ADV# assertion time from start cycle time in
 *                               GPMC_FCLK cycles.
 *
 */
#define GPMC_ADV_TIMING_CONFIG(ADVAADMuxWrOffTime, ADVAADMuxRdOffTime,      \
                               ADVWrOffTime, ADVRdOffTime, ADVExtDelayFlag, \
                               ADVAADMuxOnTime,                             \
                               ADVOnTime)                                   \
    ((uint32_t) (((ADVAADMuxWrOffTime) <<                                   \
                  CSL_GPMC_CONFIG3_ADVAADMUXWROFFTIME_SHIFT) &                \
                 CSL_GPMC_CONFIG3_ADVAADMUXWROFFTIME_MASK) |                  \
     (((ADVAADMuxRdOffTime) << CSL_GPMC_CONFIG3_ADVAADMUXRDOFFTIME_SHIFT) &   \
      CSL_GPMC_CONFIG3_ADVAADMUXRDOFFTIME_MASK) |                             \
     (((ADVWrOffTime) << CSL_GPMC_CONFIG3_ADVWROFFTIME_SHIFT) &               \
      CSL_GPMC_CONFIG3_ADVWROFFTIME_MASK) |                                   \
     (((ADVRdOffTime) << CSL_GPMC_CONFIG3_ADVRDOFFTIME_SHIFT) &               \
      CSL_GPMC_CONFIG3_ADVRDOFFTIME_MASK) |                                   \
     (((ADVExtDelayFlag) << CSL_GPMC_CONFIG3_ADVEXTRADELAY_SHIFT) &           \
      CSL_GPMC_CONFIG3_ADVEXTRADELAY_MASK) |                                  \
     (((ADVAADMuxOnTime) << CSL_GPMC_CONFIG3_ADVAADMUXONTIME_SHIFT) &         \
      CSL_GPMC_CONFIG3_ADVAADMUXONTIME_MASK) |                                \
     (((ADVOnTime) << CSL_GPMC_CONFIG3_ADVONTIME_SHIFT) &                     \
      CSL_GPMC_CONFIG3_ADVONTIME_MASK))


/**
 * \brief  This macro used to make the conf value which is used to configure the
 *         WE# and OE# signal timing configuration.\n
 *
 * \param  WEOffTime             WE# de-assertion time in GPMC_FCLK cycles from
 *                               start cycle time.
 *
 * \param  WEExtDelayFlag        Flag to indicate whether to add half GPMC_FCLK
 *                               delay to WE or not. \n
 *                               This can take one of the following values :  \n
 *                               CSL_GPMC_CONFIG4_WEEXTRADELAY_DELAYED : WE# Timing control
 *                                                                       signal is delayed of
 *                                                                       half GPMC_FCLK cycle.  \n
 *                               CSL_GPMC_CONFIG4_WEEXTRADELAY_NOTDELAYED : WE# Timing control
 *                                                                          signal is not delayed. \n
 * \param  WEOnTime              WE# assertion time in GPMC_FCLK cycles from
 *                               start cycle time.
 *
 * \param  OEAADMuxOffTime       OE# de-assertion time in GPMC_FCLK cycles for
 *                               first address phase when using the AAD-Mux
 *                               prorocol.
 *
 * \param  OEOffTime             OE# de-assertion time in GPMC_FCLK cycles from
 *                               start cycle time.
 *
 * \param  OEExtDelayFlag        Flag to indicate whether to add half GPMC_FCLK
 *                               delay to OE or not. \n
 *                               This can take one of the following values :
 *                               CSL_GPMC_CONFIG4_OEEXTRADELAY_DELAYED : OE# Timing control
 *                                                                      signal is delayed of
 *                                                                      half GPMC_FCLK cycle. \n
 *                               CSL_GPMC_CONFIG4_OEEXTRADELAY_NOTDELAYED : OE# Timing control
 *                                                                          signal is not delayed. \n
 *
 * \param  OEAADMuxOnTime        OE# assertion time in GPMC_FCLK cycles for
 *                               first address phase when using the AAD-Mux
 *                               prorocol.
 *
 * \param  OEOnTime              OE# assertion time in GPMC_FCLK cycles from
 *                               start cycle time.
 *
 *
 */
#define GPMC_WE_OE_TIMING_CONFIG(WEOffTime, WEExtDelayFlag, WEOnTime,           \
                                 OEAADMuxOffTime, OEOffTime, OEExtDelayFlag,    \
                                 OEAADMuxOnTime, OEOnTime)                      \
    ((uint32_t) (((WEOffTime) <<                                                \
                  CSL_GPMC_CONFIG4_WEOFFTIME_SHIFT) &                             \
                 CSL_GPMC_CONFIG4_WEOFFTIME_MASK) |                               \
     (((WEExtDelayFlag) <<                                                      \
       CSL_GPMC_CONFIG4_WEEXTRADELAY_SHIFT) & CSL_GPMC_CONFIG4_WEEXTRADELAY_MASK) | \
     (((WEOnTime) <<                                                            \
       CSL_GPMC_CONFIG4_WEONTIME_SHIFT) & CSL_GPMC_CONFIG4_WEONTIME_MASK) |         \
     (((OEAADMuxOffTime) <<                                                     \
       CSL_GPMC_CONFIG4_OEAADMUXOFFTIME_SHIFT) &                                  \
      CSL_GPMC_CONFIG4_OEAADMUXOFFTIME_MASK) |                                    \
     (((OEOffTime) <<                                                           \
       CSL_GPMC_CONFIG4_OEOFFTIME_SHIFT) & CSL_GPMC_CONFIG4_OEOFFTIME_MASK) |       \
     (((OEExtDelayFlag) <<                                                      \
       CSL_GPMC_CONFIG4_OEEXTRADELAY_SHIFT) & CSL_GPMC_CONFIG4_OEEXTRADELAY_MASK) | \
     (((OEAADMuxOnTime) <<                                                      \
       CSL_GPMC_CONFIG4_OEAADMUXONTIME_SHIFT) &                                   \
      CSL_GPMC_CONFIG4_OEAADMUXONTIME_MASK) |                                     \
     (((OEOnTime) <<                                                            \
       CSL_GPMC_CONFIG4_OEONTIME_SHIFT) & CSL_GPMC_CONFIG4_OEONTIME_MASK))



/**
 * \brief  This macro used to make the conf value which is used to configure the
 *         read access and cycle time timing configuration.\n
 *
 * \param  rdCycleTime           Total read cycle time in GPMC_FCLK cycles.
 *
 * \param  wrCycleTime           Total write cycle time in GPMC_FCLK cycles.
 *
 * \param  rdAccessTime          Read access time (Delay between start cycle
 *                               time and first data valid) in GPMC_FCLK cycles.
 *
 * \param  pageBurstAccessTime   Page burest access time (Delay between
 *                               successive words in a multiple access)in
 *                               GPMC_FCLK cycles.
 *
 */
#define GPMC_RDACCESS_CYCLETIME_TIMING_CONFIG(rdCycleTime, wrCycleTime,         \
                                              rdAccessTime,                     \
                                              pageBurstAccessTime)              \
    ((uint32_t) (((rdCycleTime) <<                                              \
                  CSL_GPMC_CONFIG5_RDCYCLETIME_SHIFT) &                           \
                 CSL_GPMC_CONFIG5_RDCYCLETIME_MASK) |                             \
     (((wrCycleTime) <<                                                         \
       CSL_GPMC_CONFIG5_WRCYCLETIME_SHIFT) & CSL_GPMC_CONFIG5_WRCYCLETIME_MASK) |   \
     (((rdAccessTime) <<                                                        \
       CSL_GPMC_CONFIG5_RDACCESSTIME_SHIFT) & CSL_GPMC_CONFIG5_RDACCESSTIME_MASK) | \
     (((pageBurstAccessTime) <<                                                 \
       CSL_GPMC_CONFIG5_PAGEBURSTACCESSTIME_SHIFT) &                              \
      CSL_GPMC_CONFIG5_PAGEBURSTACCESSTIME_MASK))



/**
 * \brief  This macro used to make the conf value which is used to configure the
 *         cycle to cycle and bus turn around time timing configuration.\n
 *
 * \param  cycle2CycleDelay       Cycle to cycle delay (Chip select high pulse
 *                                delay between two successive accesses)in
 *                                GPMC_FCLK cycles.
 *
 * \param  cycle2CycleDelaySameCSCfg Specified whether to add the cycle to cycle
 *                                delay between two successive accesses or not
 *                                (to the same chip-select). \n
 *                                This can take one of the following values : \n
 *                                CSL_GPMC_CONFIG6_CYCLE2CYCLESAMECSEN_C2CDELAY : To add the delay. \n
 *                                CSL_GPMC_CONFIG6_CYCLE2CYCLESAMECSEN_NOC2CDELAY : Don't add the delay. \n
 *
 * \param  cycle2CycleDelayDiffCSCfg Specified whether to add the cycle to cycle
 *                                delay between two successive accesses or not
 *                                (to the diffrent chip-select). \n
 *                                This can take one of the following values : \n
 *                                CSL_GPMC_CONFIG6_CYCLE2CYCLEDIFFCSEN_C2CDELAY : To add the delay. \n
 *                                CSL_GPMC_CONFIG6_CYCLE2CYCLEDIFFCSEN_NOC2CDELAY : Don't add the delay. \n
 *
 * \param  busTAtime             Bus turn aroung time between two successive
 *                               accesses to the same chip-select
 *                               (read to write) or to a diff chip-select in
 *                               GPMC_FCLK cycles.
 *
 *
 */
#define GPMC_CYCLE2CYCLE_BUSTURNAROUND_TIMING_CONFIG(cycle2CycleDelay,          \
                                                     cycle2CycleDelaySameCSCfg, \
                                                     cycle2CycleDelayDiffCSCfg, \
                                                     busTAtime)                 \
    ((uint32_t) (((cycle2CycleDelay) <<                                         \
                  CSL_GPMC_CONFIG6_CYCLE2CYCLEDELAY_SHIFT) &                      \
                 CSL_GPMC_CONFIG6_CYCLE2CYCLEDELAY_MASK) |                        \
     (((cycle2CycleDelaySameCSCfg) <<                                           \
       CSL_GPMC_CONFIG6_CYCLE2CYCLESAMECSEN_SHIFT) &                              \
      CSL_GPMC_CONFIG6_CYCLE2CYCLESAMECSEN_MASK) |                                \
     (((cycle2CycleDelayDiffCSCfg) <<                                           \
       CSL_GPMC_CONFIG6_CYCLE2CYCLEDIFFCSEN_SHIFT) &                              \
      CSL_GPMC_CONFIG6_CYCLE2CYCLEDIFFCSEN_MASK) |                                \
     (((busTAtime) <<                                                           \
       CSL_GPMC_CONFIG6_BUSTURNAROUND_SHIFT) & CSL_GPMC_CONFIG6_BUSTURNAROUND_MASK))

/** \brief size in bytes*/
#define GPMC_DMA_COPY_LOWER_LIMIT                   (512U)
/** \brief GPMC CHIP select base address shift value. */
#define GPMC_CS_BASE_ADDR_SHIFT                      (24U)
/** \brief GPMC ECC Wrap mode for ECC value computation. */
#define GPMC_ECC_WRAP_MODE1                          (1)
/** \brief Invalid value for params initialisation. */
#define GPMC_CMD_INVALID                             (0xFFFFFFFFU)

#define GPMC_MEM_TYPE_NAND                           (0)
#define GPMC_MEM_TYPE_PSRAM                          (1)
#define GPMC_MEM_TYPE_NORLIKE                        (1)    //Same as pSRAM

/** \brief A handle that is returned from a #GPMC_open() call */
typedef void* GPMC_Handle;

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */
/**
 *  \brief GPMC ECC engine algoritms.
 *
 *
 */
typedef enum GPMC_v1_nandEccAlgo_s
{
    GPMC_NAND_ECC_ALGO_NONE  = 0x00U,
    GPMC_NAND_ECC_ALGO_HAMMING_1BIT,
    GPMC_NAND_ECC_ALGO_BCH_4BIT,
    GPMC_NAND_ECC_ALGO_BCH_8BIT,
    GPMC_NAND_ECC_ALGO_BCH_16BIT

} GPMC_nandEccAlgo;

/**
 *  \brief GPMC driver operating modes.
 *
 *
 */
typedef enum GPMC_OperatingMode_s {

    GPMC_OPERATING_MODE_BLOCKING = 0,
    /**< Interupt based blocking mode */
    GPMC_OPERATING_MODE_CALLBACK,
    /**< Interupt based callback mode*/
    GPMC_OPERATING_MODE_POLLING
    /**< Non interupt base blocking mode*/

} GPMC_OperatingMode;

/**
 *  \brief GPMC data transfer modes.
 *
 *
 */
typedef enum GPMC_TransferMode_s {

    GPMC_TRANSFER_MODE_BLOCKING = 0,
    /**<
    *  GPMC driver transaction blocks execution. This mode can only be used
    *  when called within a Task context and is interrupt based.
    */
    GPMC_TRANSFER_MODE_CALLBACK,
    /**<
    *  GPMC driver transaction does not block code execution and will call a
    *  callback function. This mode can be used in a Task, Swi, or Hwi context.
    */
} GPMC_TransferMode;

/**
 * \brief Type of the GPMC transaction for read and write.
 *
 */
typedef enum GPMC_TransactionType_e {
    GPMC_TRANSACTION_TYPE_READ = 0,
    /**< Read transaction */
    GPMC_TRANSACTION_TYPE_WRITE,
    /**< Write transaction */
    GPMC_TRANSACTION_TYPE_READ_CMDREG,
    /**< Read transaction with command register */
    GPMC_TRANSACTION_TYPE_WRITE_CMDREG
    /**< Write transaction with command register */
} GPMC_TransactionType;

/**
 * \brief  Transaction status codes that are set by the GPMC driver.
 *
 */
typedef enum GPMC_TransactionStatus_s {
    GPMC_TRANSFER_COMPLETED = 0,
    GPMC_TRANSFER_STARTED,
    GPMC_TRANSFER_CANCELED,
    GPMC_TRANSFER_FAILED,
    GPMC_TRANSFER_CSN_DEASSERT,
    GPMC_TRANSFER_TIMEOUT,
} GPMC_TransactionStatus;


/**
 *  \brief Data structure to set NAND command parameters.
 *
 */
typedef struct
{
    uint32_t cmdCycle1;
    /**< NAND command cycle 1. */
    uint32_t cmdCycle2;
    /**< NAND command cycle 2. */
    uint32_t rowAddress;
    /**< NAND row address */
    uint32_t numRowAddrCycles;
    /**< Number of row address cycles.  */
    uint32_t colAddress;
    /**< NAND column address. */
    uint32_t numColAddrCycles;
    /**< Number of column address cycles. */
    uint32_t waitTimeout;
    /**< Wait timeout for the command. */
    uint32_t checkReadypin;
    /**< Check WAIT PIN status or WAIT PIN interupt status. */
} GPMC_nandCmdParams;


/**
 *  \brief Data structure to set transaction type parameters.
 *
 */
typedef struct GPMC_Transaction_s {

	GPMC_TransactionType    transType;
    /**< Transaction type : GPMC_TransactionType */
    uint32_t                count;
    /**< Number of bytes for this transaction */
    void                    *Buf;
    /**< void * to a buffer to receive/send data */
    void                    *arg;
    /**< Argument to be passed to the callback function */
    uint32_t                transferTimeout;
    /**< Timeout of the specific transfer */
    GPMC_TransactionStatus status;
    /**< Status code set by GPMC read or write transaction */
} GPMC_Transaction;

/**
 *  \brief      The definition of a callback function used by the GPMC driver
 *              when used in GPMC_OPERATING_MODE_CALLBACK mode.
 *
 *  \param      GPMC_Handle          GPMC_Handle
 *  \param      GPMC_Transaction*    GPMC_Transaction*
 */
typedef void (*GPMC_CallbackFxn)(GPMC_Handle handle,
                                 GPMC_Transaction * transaction);


/**
 *  \brief  GPMC driver instance parameters.
 *
 *  GPMC Parameters are used to with the GPMC_open() call. Default values for
 *  these parameters are set using GPMC_Params_init().
 *
 *  If NULL is passed for the parameters, GPMC_open() uses default parameters.
 *
 */
typedef struct GPMC_Params_s
{
    /** Index of Channel used by GPMC DMA Driver. This index will be set by SysCfg according to the DMA driver chosen.
     * The GPMC driver uses this index to do an GPMC_dmaOpen() inside the GPMC_open() if the DMA mode is enabled.
     */
    int32_t                 gpmcDmaChIndex;
    /**< device type, NAND or NOR like */
    uint32_t                devType;
    /**< device size, 8-bit, 16-bit or 32-bit bus width */
    uint32_t                devSize;
    /**< Signals timing latencies scalar factor */
    uint32_t                chipSel;
    /**< Enable GPMC interrupt. */
    uint32_t                intrEnable;

    uint32_t                dmaEnable;
    /**< Enable GPMC DMA mode. */
    GPMC_TransferMode       transferMode;
    /**< Polling, blocking or callback mode . Refer \ref GPMC_TransferMode */
    GPMC_CallbackFxn        transferCallBckFunc;
    /**< External Callback function pointer*/
    uint32_t                memDevice;
    /**< Flag to indicate memory device */
} GPMC_Params;

/**
 *  \brief GPMC Address Region
 *
 *  GPMC Address Region will be a part of the attributes. It is used while using DMA Copy
 *  to check if the destination address is a region not accessible to DMA. This data is usually SOC
 *  specific and is filled by SysConfig.
 *
 */
typedef struct
{
    uint32_t regionStartAddr;
    /**< Start address of the region */
    uint32_t regionSize;
    /**< Size of the region */

} GPMC_AddrRegion;

/**
 *  \brief GPMC driver object
 *
 */
typedef struct GPMC_Object_s {

    GPMC_Handle                     handle;
    /**< Instance handle */
    GPMC_CallbackFxn                transferCallbackFxn;
    /**< Callback function pointer */
    GPMC_Params                     params;
    /**< Driver user configurable params structure */
    GPMC_OperatingMode              operMode;
    /**< GPMC Driver operating mode */
    /*
     * State variables
     */
    uint32_t                        isOpen;
    /**< Flag to indicate if the instance is already open */
    HwiP_Object                     hwi;
    /**< Interrupt object */
    SemaphoreP_Object               mutex;
    /**< Driver lock object */
    SemaphoreP_Object               transferComplete;
    /**< Transfer Sync Semaphore object */
    GPMC_Transaction                *transaction;
    /**< Pointer to current transaction struct */
    void* gpmcDmaHandle;
    /**< DMA configuration handle */
} GPMC_Object;

/**
 *  \brief  GPMC timing parameters
 */
typedef struct GPMC_timingParams_s
{
    uint32_t    csOnTime;
    /* Chip Select assertion time. */
    uint32_t    csRdOffTime;
    /**< Chip Select Read de-assertion time. */
    uint32_t    csWrOffTime;
    /**< Chip Select Write de-assertion time. */
    uint32_t    advOnTime;
    /**< ADV Assertion time. */
    uint32_t    advRdOffTime;
    /**< ADV Read de-assertion time. */
    uint32_t    advWrOffTime;
    /**< ADV Write de-assertion time. */
    uint32_t    advAadMuxOnTime;
    /**< ADV Assertion time in an AADMultiplexed access. */
    uint32_t    advAadMuxRdOffTime;
    /**< ADV Read de-assertion time in an AADMultiplexed access. */
    uint32_t    advAadMuxWrOffTime;
    /**< ADV Write de-assertion time in an AADMultiplexed access. */
    uint32_t    weOnTtime;
    /**< WE assertion time. */
    uint32_t    weOffTime;
    /**< WE de-assertion time. */
    uint32_t    oeOnTime;
    /**< OE assertion time. */
    uint32_t    oeOffTime;
    /**< OE de-assertion time. */
    uint32_t    oeAadMuxOnTime;
    /**< OE assertion time in an AADMultiplexed access. */
    uint32_t    oeAadMuxOffTime;
    /**< OE de-assertion time in an AADMultiplexed access. */
    uint32_t    pageBurstAccess;
    /**< Multiple access word delay. */
    uint32_t    rdAccessTime;
    /**< Start-cycle to first valid delay. */
    uint32_t    wrAcessTime;
    /**< Delay from StartAccessTime to the CLK rising edge. */
    uint32_t    rdCycleTime;
    /**< Total read cycle time. */
    uint32_t    wrCycleTime;
    /**< Total write cycle time. */
    uint32_t    wrDataOnMuxBusTime;
    /**< Write Data on Mux Bus Time. */
    uint32_t    cycle2CycleDelay;
    /**< Chip Select high pulse delay between two successive accesses. */
    uint32_t    cycleDelaySameChipSel;
    /**< Value to control adding of cycle2cycle delay between two successive
         accesses to the same chip select. */
    uint32_t    cycleDelayDiffChipSel;
    /**< Value to control adding of cycle2cycle delay between two successive
         accesses to the different chip select. */
    uint32_t    busTurnAroundTime;
    /**< Bus turn around time between two successive
         accesses to the same chip-select (read to write)
         or to a different chip-select(read to read and read to write)
         in GPMC_FCLK cycles. */
} GPMC_timingParams;

/**
 *  \brief  GPMC instance attributes
 */
typedef struct GPMC_HwAttrs_s
{

    uint32_t                gpmcBaseAddr;
    /**< GPMC Peripheral base address. */
    uint32_t                dataBaseAddr;
    /**< GPMC flash base address. */
    uint32_t                elmBaseAddr;
    /**< Error Location Module base address for ECC computation. */
    uint32_t                inputClkFreq;
    /**< GPMC module input clock frequency. */
    uint32_t                intrNum;
    /**< GPMC Peripheral interupt number. */
    uint32_t                intrPriority;
    /**< Interupt priority*/
    uint32_t                clkDivider;
    /**< GPMC FCLK divider*/
    uint32_t                addrDataMux;
    /**< Address and data multiplexed protocol */
    uint32_t                timeLatency;
    /**< Current Active chip select in use by the memory controller */
    uint32_t                chipSelBaseAddr;
    /**< Chip select base address (A29 - A24) */
    uint32_t                chipSelAddrSize;
    /**< Chip select address mask size */
    uint32_t                waitPinNum;
    /**< Wait pin number */
    uint32_t                waitPinPol;
    /**< Wait pin polarity  */
    GPMC_timingParams       timingParams;
    /**< Structure holding the timing parameters for the GPMC instance. */
    GPMC_nandEccAlgo        eccAlgo;
    /**< ECC algorithm supported by the controller. */
    uint32_t                readType;
    /**< Read operation mode */
    uint32_t                writeType;
    /**< Write operation mode */
    uint32_t                csExDelay;
    /**< Chip Select Extra Delay flag. */
    uint32_t                accessType;
    /**< Prefetch/Postwrite engine optimised access. */
    uint32_t                optimisedAccess;
    /**< Cycle optimisation for prefetch/post write engine. */
    uint32_t                cycleOptimisation;
    /**< Memory access type */
    const GPMC_AddrRegion   *dmaRestrictedRegions;
    /**< Pointer to array of GPMC_AddrRegion data structures filled by SysConfig. The
    array should be terminated by a { 0xFFFFFFFFU, 0U } entry. It is used while
    using DMA copy to check if the destination address is a region not accessible to DMA
    and switch to CPU copy */

} GPMC_HwAttrs;

/**
 *  \brief  GPMC Driver configuration structure
 */
typedef struct GPMC_Config_s {

    const GPMC_HwAttrs * attrs;
    /**< Pointer to driver instance specific attributes */
    GPMC_Object *object;
    /**< Pointer to driver specific data object */

} GPMC_Config;


/** \brief Externally defined driver configuration array */
extern GPMC_Config gGpmcConfig[];
/** \brief Externally defined driver configuration array size */
extern uint32_t gGpmcConfigNum;

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */
/**
 *  \brief  Function to initialize the \ref GPMC_Params struct to its defaults
 *
 *  \param  params Pointer to \ref GPMC_Params structure for initialization
 */
static inline void GPMC_Params_init(GPMC_Params *params);

/**
 *  \brief  This function initializes the GPMC module
 */
void GPMC_init(void);
/**
 *  \brief  This function de-initializes the GPMC module
 */
void GPMC_deinit(void);

/**
 *  \brief  This function opens a given GPMC peripheral
 *
 *  \pre    GPMC controller has been initialized using #GPMC_init()
 *
 *  \param  index       Index of config to use in the #GPMC_Config array
 *  \param  prms  Pointer to parameters #GPMC_Params to open the driver instance.
 *
 *  \return A #GPMC_Handle on success or a NULL on an error or if it has been
 *          opened already
 *
 *  \sa     #GPMC_init()
 *  \sa     #GPMC_close()
 */
GPMC_Handle GPMC_open(uint32_t index, const GPMC_Params *prms);

/**
 *  \brief  Function to close a GPMC peripheral specified by the GPMC handle
 *
 *  \pre    #GPMC_open() has to be called first
 *
 *  \param  handle      #GPMC_Handle returned from #GPMC_open()
 *
 *  \sa     #GPMC_open()
 */
void GPMC_close(GPMC_Handle handle);

/**
 *  \brief  This function returns the input clk frequency GPMC was programmed at
 *
 *  \pre    GPMC controller has been opened using #GPMC_open()
 *
 *  \param  handle  An #GPMC_Handle returned from an #GPMC_open()
 *
 *  \return GPMC RCLK in Hertz
 */
uint32_t GPMC_getInputClk(GPMC_Handle handle);

/**
 *  \brief  This function returns the handle of an open GPMC Instance from the instance index
 *
 *  \pre    GPMC controller has been opened using #GPMC_open()
 *
 *  \param  driverInstanceIndex Index of config to use in the #GPMC_Config array
 *
 *  \return An #GPMC_Handle if it has been opened already or NULL otherwise
 *
 *  \sa     #GPMC_init()
 *  \sa     #GPMC_open()
 */
GPMC_Handle GPMC_getHandle(uint32_t driverInstanceIndex);

/**
 *  \brief  Function to initialise #GPMC_nandCmdParams structure to default values.
 *
 *
 *  \param  cmdParams Pointer to #GPMC_nandCmdParams structure
 *
 */
void GPMC_writeNandCommandParamsInit(GPMC_nandCmdParams *cmdParams);

/**
 *  \brief  Function to write NAND command parameters.
 *
 *  \param  handle  An #GPMC_Handle returned from an #GPMC_open()
 *
 *  \param  cmdParams Pointer to #GPMC_nandCmdParams structure
 *
 *  \return SystemP_SUCCESS or SystemP_FAILURE
 *
 */
int32_t GPMC_writeNandCommand(GPMC_Handle handle, GPMC_nandCmdParams *cmdParams);

/**
 *  \brief  Function to initialise #GPMC_Transaction structure to default values.
 *
 *
 *  \param  trans Pointer to #GPMC_Transaction structure
 *
 */
void GPMC_transactionInit(GPMC_Transaction *trans);

/**
 *  \brief  Function to read data from NAND flash using DMA or CPU
 *          prefetch/post write engine.
 *
 *  \param  handle  An #GPMC_Handle returned from an #GPMC_open()
 *
 *  \param  trans Pointer to #GPMC_Transaction structure
 *
 *  \return SystemP_SUCCESS or SystemP_FAILURE
 */
int32_t GPMC_nandReadData(GPMC_Handle handle, GPMC_Transaction *trans);

/**
 *  \brief  Function to write data to NANDflash using CPU prefetch/post write engine.
 *
 *  \param  handle  An #GPMC_Handle returned from an #GPMC_open()
 *
 *  \param  trans Pointer to #GPMC_Transaction structure
 *
 *  \return SystemP_SUCCESS or SystemP_FAILURE
 */
int32_t GPMC_nandWriteData(GPMC_Handle handle, GPMC_Transaction *trans);

/**
 *  \brief  Function to set device width for GPMC instance connected to external
 *          device.
 *
 *  \param  handle  An #GPMC_Handle returned from an #GPMC_open()
 *
 *
 *  \return SystemP_SUCCESS or SystemP_FAILURE
 */
int32_t GPMC_setDeviceSize(GPMC_Handle handle);

/**
 *  \brief  Function to set device type (NANDLIKE OR NORLIKE) for GPMC instance connected
 *          to external device.
 *
 *  \param  handle  An #GPMC_Handle returned from an #GPMC_open()
 *
 *
 *  \return SystemP_SUCCESS or SystemP_FAILURE
 */
int32_t GPMC_setDeviceType(GPMC_Handle handle);

/**
 *  \brief  Function to configure GPMC timing parameters.
 *
 *  \param  handle  An #GPMC_Handle returned from an #GPMC_open()
 *
 *
 *  \return SystemP_SUCCESS or SystemP_FAILURE
 */
int32_t GPMC_configureTimingParameters(GPMC_Handle handle);

/**
 *  \brief  Function to set ECC used and unused bytes size in nibbles.
 *
 *  \param  handle  An #GPMC_Handle returned from an #GPMC_open()
 *
 *  \param  eccSize Param to set #GPMC_ECC_SIZE_0 (used bytes) or #GPMC_ECC_SIZE_1 unused bytes
 *
 *  \param  eccSizeVal  ECC used or ununsed bytes value in nibbles.
 *
 *
 *  \return SystemP_SUCCESS or SystemP_FAILURE
 */
int32_t GPMC_eccValueSizeSet(GPMC_Handle handle, uint32_t eccSize, uint32_t eccSizeVal);

/**
 *  \brief  Function to configure ELM module for error correction.
 *
 *  \param  handle  An #GPMC_Handle returned from an #GPMC_open()
 *
 *  \param  numSectors Number of 512 bytes sector in a NAND page.
 *
 *
 *  \return SystemP_SUCCESS or SystemP_FAILURE
 */
int32_t GPMC_eccBchConfigureElm(GPMC_Handle handle, uint8_t numSectors);

/**
 *  \brief  Function to configure GPMC ECC engine for BCH algorithm
 *
 *  \param  handle  An #GPMC_Handle returned from an #GPMC_open()
 *
 *  \param  eccSteps Number of 512 bytes sectors to process BCH syndrome polynomial computation.
 *
 *
 *  \return SystemP_SUCCESS or SystemP_FAILURE
 */
int32_t GPMC_eccEngineBCHConfig (GPMC_Handle handle, uint32_t eccSteps);

/**
 *  \brief  Function to enable GPMC ECC engine.
 *
 *  \param  handle  An #GPMC_Handle returned from an #GPMC_open()
 *
 *  \return SystemP_SUCCESS or SystemP_FAILURE
 */
int32_t GPMC_eccEngineEnable(GPMC_Handle handle);

/**
 *  \brief  Function to clear GPMC ECC result register.
 *
 *  \param  handle  An #GPMC_Handle returned from an #GPMC_open()
 *
 */
void GPMC_eccResultRegisterClear(GPMC_Handle handle);

/**
 *  \brief  Function to fill BCH syndrome value per sector to ELM module.
 *
 *  \param  handle  An #GPMC_Handle returned from an #GPMC_open()
 *
 *  \param  sector  Sector number
 *
 *  \param  bchData Pointer to syndrome polynomial value.
 *
 *
 *  \return SystemP_SUCCESS or SystemP_FAILURE
 */
int32_t GPMC_eccBchFillSyndromeValue(GPMC_Handle handle, uint32_t sector, uint32_t *bchData);

/**
 *  \brief  Function to start error processing for a sector by ELM module.
 *
 *  \param  handle  An #GPMC_Handle returned from an #GPMC_open()
 *
 *  \param  sector  Sector number
 *
 *
 *
 *  \return SystemP_SUCCESS or SystemP_FAILURE
 */
int32_t GPMC_eccBchStartErrorProcessing(GPMC_Handle handle, uint8_t sector);

/**
 *  \brief  Function to get error processing status for a sector by ELM module.
 *
 *  \param  handle  An #GPMC_Handle returned from an #GPMC_open()
 *
 *  \param  sector  Sector number
 *
 *  \return SystemP_SUCCESS or SystemP_FAILURE
 */
int32_t GPMC_eccBchCheckErrorProcessingStatus(GPMC_Handle handle, uint32_t sector);

/**
 *  \brief  Function to get number of errors per sector by ELM module.
 *
 *  \param  handle  An #GPMC_Handle returned from an #GPMC_open()
 *
 *  \param  sector  Sector number
 *
 *  \param  errCount  Pointer to store number of errors in a sector
 *
 *  \param  errLoc  Pointer to store error locations in a sector
 *
 *  \return SystemP_SUCCESS or SystemP_FAILURE
 */
int32_t GPMC_eccBchSectorGetError(GPMC_Handle handle, uint32_t sector, uint32_t *errCount, uint32_t *errLoc);

/**
 *  \brief  Function to compute BCH syndrome polynomial for NAND write operation.
 *
 *  \param  handle  An #GPMC_Handle returned from an #GPMC_open()
 *
 *  \param  sector  Sector number
 *
 *  \param  pEccdata  Pointer to store BCH syndrome polynomial.
 *
 *  \return SystemP_SUCCESS or SystemP_FAILURE
 */
int32_t GPMC_eccCalculateBchSyndromePolynomial(GPMC_Handle handle, uint8_t *pEccdata, uint32_t sector);

/**
 *  \brief  Function to get BCH syndrome polynomial per sector NAND read operation.
 *
 *  \param  handle  An #GPMC_Handle returned from an #GPMC_open()
 *
 *  \param  sector  Sector number
 *
 *  \param  bchData  Pointer to store BCH syndrome polynomial.
 *
 *  \return SystemP_SUCCESS or SystemP_FAILURE
 */
int32_t GPMC_eccGetBchSyndromePolynomial(GPMC_Handle handle, uint32_t sector, uint32_t *bchData);

/**
 *  \brief  Function to configure GPMC PREFETCH read and POST write engine.
 *
 *  \param  handle  An #GPMC_Handle returned from an #GPMC_open()
 *
 *
 *  \return SystemP_SUCCESS or SystemP_FAILURE
 */
int32_t GPMC_configurePrefetchPostWriteEngine(GPMC_Handle handle);

/**
 *  \brief  Function to disable WRITE protect line.
 *
 *  \param  handle  An #GPMC_Handle returned from an #GPMC_open()
 *
 *
 *  \return SystemP_SUCCESS or SystemP_FAILURE
 */
int32_t GPMC_disableFlashWriteProtect(GPMC_Handle handle);

/**
 *  \brief  Function to disable WRITE protect line.
 *
 *  \param  handle  An #GPMC_Handle returned from an #GPMC_open()
 *
 *
 *  \return SystemP_SUCCESS or SystemP_FAILURE
 */
int32_t GPMC_enableFlashWriteProtect(GPMC_Handle handle);

/**
 *  \brief  Function to create correct address based on bus width.
 *
 *  \param  busWidth  Bus width for the NOR Flash device.
 *
 *  \param  blkAddr  Base address for NOR Flash.
 *
 *  \param  offset  Base address for NOR Flash.
 *
 *
 *  \return Pointer to Address
 */
uint8_t *GPMC_norMakeAddr(uint8_t busWidth,uint32_t blkAddr,uint32_t offset);

/**
 *  \brief  Function to create correct NOR cmd based on bus width.
 *
 *  \param  busWidth  Bus width for the NOR Flash device.
 *
 *  \param  cmd  Command to be sent on Data Lines.
 *
 *  \param  cmdBuf  Buffer pointer to store the modified cmd.
 *
 */
void GPMC_norMakeCmd(uint8_t busWidth, uint32_t cmd, void *cmdBuf);

/**
 *  \brief  Function to read data from norlike device.
 *
 *  \param  handle  An #GPMC_Handle returned from an #GPMC_open()
 *
 *  \param  offset  Offset of read address from GPMC data baseaddress.
 *
 *  \param  buf  Pointer to buffer for storing read data.
 *
 *  \param  len  Number of bytes to read.
 *
 *
 *  \return SystemP_SUCCESS or SystemP_FAILURE
 */
int32_t GPMC_norReadData(GPMC_Handle handle, uint32_t offset,uint8_t *buf, uint32_t len);

/**
 *  \brief  Function to write data to norlike device.
 *
 *  \param  handle  An #GPMC_Handle returned from an #GPMC_open()
 *
 *  \param  offset  Offset of write address from GPMC data baseaddress.
 *
 *  \param  buf  Pointer to buffer for storing read data.
 *
 *  \param  len  Number of bytes to write.
 *
 *
 *  \return SystemP_SUCCESS or SystemP_FAILURE
 */
int32_t GPMC_norWriteData(GPMC_Handle handle,uint32_t offset,uint8_t *buf, uint32_t len);

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */
static inline void GPMC_Params_init(GPMC_Params *params)
{
    if (params != NULL)
    {
        params->dmaEnable = FALSE;
        params->intrEnable = FALSE;
        params->chipSel = 0;
        params->devType = CSL_GPMC_CONFIG1_DEVICETYPE_NANDLIKE;
        params->devType = CSL_GPMC_CONFIG1_DEVICESIZE_EIGHTBITS;
        params->gpmcDmaChIndex = -1;
        params->transferMode = GPMC_TRANSFER_MODE_BLOCKING;
        params->transferCallBckFunc = NULL;
    }
}

#ifdef __cplusplus
}
#endif

#endif  /* _GPMC__H_ */

/** @} */

