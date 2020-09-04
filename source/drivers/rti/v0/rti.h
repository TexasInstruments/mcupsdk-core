/*
 *  Copyright (C) 2022 Texas Instruments Incorporated
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
 *  \defgroup DRV_RTI_MODULE APIs for RTI
 *  \ingroup DRV_MODULE
 *
 *  This module contains APIs to program and use the RTI module.
 *
 *  @{
 */

/**
 *  \file v0/rti.h
 *
 *  \brief RTI Driver API/interface file.
 */

#ifndef RTI_H_
#define RTI_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <drivers/hw_include/cslr_rti.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
 * \anchor RTI_StallModes
 * \name RTI Global Control Stall Modes
 *
 * Macro to select the stall mode in RTI Global Control.
 *
 * The counters can be either stopped or running while in debug halting mode.
 *
 * @{
 */
/** \brief The counters will be stopped while in debug halting mode */
#define    RTI_GC_STALL_MODE_ON     (CSL_RTI_RTIGCTRL_COS_STOPPED)
/** \brief The counters will be running while in debug halting mode */
#define    RTI_GC_STALL_MODE_OFF    (CSL_RTI_RTIGCTRL_COS_RUNNING)
/** @} */

/**
 * \anchor RTI_TmrCntBlkIndexes
 * \name RTI Timer Counter Block Indexes
 *
 * RTI Timer contains two counter blocks
 *
 * @{
 */
/** \brief RTI Timer counter block 0 */
#define RTI_TMR_CNT_BLK_INDEX_0        (0U)
/** \brief RTI Timer counter block 1 */
#define RTI_TMR_CNT_BLK_INDEX_1        (1U)
/** \brief RTI Timer counter block maximum */
#define RTI_TMR_CNT_BLK_INDEX_MAX      (RTI_TMR_CNT_BLK_INDEX_1)
/** @} */

/**
 * \anchor RTI_TmrCmpBlkIndexes
 * \name RTI Timer Compare Block Indexes
 *
 * RTI Timer contains four compare blocks
 *
 * @{
 */
/** \brief RTI Timer compare block 0 */
#define RTI_TMR_CMP_BLK_INDEX_0        (0U)
/** \brief RTI Timer compare block 1 */
#define RTI_TMR_CMP_BLK_INDEX_1        (1U)
/** \brief RTI Timer compare block 2 */
#define RTI_TMR_CMP_BLK_INDEX_2        (2U)
/** \brief RTI Timer compare block 3 */
#define RTI_TMR_CMP_BLK_INDEX_3        (3U)
/** \brief RTI Timer compare block maximum */
#define RTI_TMR_CMP_BLK_INDEX_MAX      (RTI_TMR_CMP_BLK_INDEX_3)
/** @} */

/**
 * \anchor RTI_TmrPrescalerValues
 * \name RTI Timer Prescaler Value Ranges
 *
 * @{
 */
/** \brief Minimum possible prescaler value for RTI Timer UP counter. */
#define RTI_TMR_MIN_PRESCALER_VAL           (1U)
/** \brief Maximum possible prescaler value for RTI Timer UP counter(2^32). */
#define RTI_TMR_MAX_PRESACLER_VAL           (0xFFFFFFFFU)
/** @} */

/**
 * \anchor RTI_TmrIntFlags
 * \name RTI Timer Interrupt Set/Indication Flags
 *
 *  @{
 */
/** \brief Comapre 0 match interrupt */
#define RTI_TMR_INT_INT0_FLAG        (CSL_RTI_RTISETINT_SETINT0_MASK)
/** \brief Comapre 1 match interrupt */
#define RTI_TMR_INT_INT1_FLAG        (CSL_RTI_RTISETINT_SETINT1_MASK)
/** \brief Comapre 2 match interrupt */
#define RTI_TMR_INT_INT2_FLAG        (CSL_RTI_RTISETINT_SETINT2_MASK)
/** \brief Comapre 3 match interrupt */
#define RTI_TMR_INT_INT3_FLAG        (CSL_RTI_RTISETINT_SETINT3_MASK)

/** \brief Comapre 0 match DMA request */
#define RTI_TMR_INT_DMA0_FLAG        (CSL_RTI_RTISETINT_SETDMA0_MASK)
/** \brief Comapre 1 match DMA request */
#define RTI_TMR_INT_DMA1_FLAG        (CSL_RTI_RTISETINT_SETDMA1_MASK)
/** \brief Comapre 2 match DMA request */
#define RTI_TMR_INT_DMA2_FLAG        (CSL_RTI_RTISETINT_SETDMA2_MASK)
/** \brief Comapre 3 match DMA request */
#define RTI_TMR_INT_DMA3_FLAG        (CSL_RTI_RTISETINT_SETDMA3_MASK)

/** \brief Timebase interrupt: detection of a missing external clock edge */
#define RTI_TMR_INT_TB_FLAG          (CSL_RTI_RTISETINT_SETTBINT_MASK)
/** \brief Free running counter 0 overflow */
#define RTI_TMR_INT_OVL0_FLAG        (CSL_RTI_RTISETINT_SETOVL0INT_MASK)
/** \brief Free running counter 1 overflow */
#define RTI_TMR_INT_OVL1_FLAG        (CSL_RTI_RTISETINT_SETOVL1INT_MASK)

/** \brief All interrupt status mask */
#define RTI_TMR_INT_STATUS_ALL       (RTI_TMR_INT_INT0_FLAG |   \
                                      RTI_TMR_INT_INT1_FLAG |   \
                                      RTI_TMR_INT_INT2_FLAG |   \
                                      RTI_TMR_INT_INT3_FLAG |   \
                                      RTI_TMR_INT_TB_FLAG   |   \
                                      RTI_TMR_INT_OVL0_FLAG |   \
                                      RTI_TMR_INT_OVL1_FLAG)
/** \brief All interrupt request mask */
#define RTI_TMR_INT_REQ_ALL          (RTI_TMR_INT_INT0_FLAG |   \
                                      RTI_TMR_INT_INT1_FLAG |   \
                                      RTI_TMR_INT_INT2_FLAG |   \
                                      RTI_TMR_INT_INT3_FLAG |   \
                                      RTI_TMR_INT_DMA0_FLAG |   \
                                      RTI_TMR_INT_DMA1_FLAG |   \
                                      RTI_TMR_INT_DMA2_FLAG |   \
                                      RTI_TMR_INT_DMA3_FLAG |   \
                                      RTI_TMR_INT_TB_FLAG   |   \
                                      RTI_TMR_INT_OVL0_FLAG |   \
                                      RTI_TMR_INT_OVL1_FLAG)
/** @} */

/**
 * \anchor RTI_TmrIntAutoClrValues
 * \name RTI Timer Interrupt Auto Clear Enable/Disable Flags
 *
 * @{
 */
/** \brief Compare interrupt Auto Clear Enable*/
#define RTI_TMR_INT_AUTO_CLR_ENABLE_FLAG  (0x0000000FU)
/** \brief Compare interrupt Auto Clear Disable */
#define RTI_TMR_INT_AUTO_CLR_DISABLE_FLAG  (0x00000005U)
/** @} */

/**
 * \anchor RTI_TmrNTUIds
 * \name RTI Timer NTU IDs
 *
 * RTI Timer Network Time (NTU) IDs
 *
 * @{
 */
/** \brief RTI Timer NTU 0*/
#define RTI_TMR_NTU_0        (0x0U)
/** \brief RTI Timer NTU 1*/
#define RTI_TMR_NTU_1        (0x5U)
/** \brief RTI Timer NTU 2*/
#define RTI_TMR_NTU_2        (0xAU)
/** \brief RTI Timer NTU 3*/
#define RTI_TMR_NTU_3        (0xFU)
/** @} */

/**
 * \anchor RTI_TmrCapEvent
 * \name RTI Timer Capture Event Indexes
 *
 * RTI Timer Capture Event source numbers
 *
 * @{
 */
/** \brief RTI Timer Capture Event source 0*/
#define RTI_TMR_CAPTURE_EVT_0        (0U)
/** \brief RTI Timer Capture Event source 1*/
#define RTI_TMR_CAPTURE_EVT_1        (1U)
/** \brief RTI Timer Capture Event source maximum*/
#define RTI_TMR_CAPTURE_EVT_MAX      (RTI_TMR_CAPTURE_EVT_1)
/** @} */

/**
 * \anchor RTI_TmrClkSrc
 * \name RTI Timer Clock Source
 *
 * RTI Timer clock source of conter block 0
 *
 * @{
 */
/** \brief Use counter block 0 up counter as clock source of the free runnung counter*/
#define RTI_TMR_CLK_SRC_COUNTER     (0U)
/** \brief Use one of NTU as clock source of the free runnung counter*/
#define RTI_TMR_CLK_SRC_NTU         (1U)
/** @} */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 * \brief   Set the stall mode in RTI Global Control.
 *
 * \param   baseAddr       Base Address of the RTI instance.
 *
 * \param   stallMode    Stall mode in RTI global control
 * 'stallMode' can take any of @ref RTI_StallModes
 *
 * \return  status         Success of the stall mode configuration
 *                                - Success: SystemP_SUCCESS
 *                                - Fail   : SystemP_FAILURE
 *
 **/
int32_t RTIG_setStallMode(uint32_t baseAddr, uint32_t stallMode);

/**
 * \brief   Start the timer.
 *
 * \param   baseAddr        Base Address of the RTI instance.
 * \param   cntIndex        Conter Block index
 *
 * \return  status          Success of the operation
 *                                - Success: SystemP_SUCCESS
 *                                - Fail   : SystemP_FAILURE
 *
 * \note    The timer must be configured before it is started/enabled.
 *
 **/
int32_t RTI_counterEnable(uint32_t baseAddr, uint32_t cntIndex);

/**
 * \brief   Stop the timer.
 *
 * \param   baseAddr        Base Address of the RTI instance.
 * \param   cntIndex        Conter Block index
 *
 * \return  status          Success of the operation
 *                                - Success: SystemP_SUCCESS
 *                                - Fail   : SystemP_FAILURE
 **/
int32_t RTI_counterDisable(uint32_t baseAddr, uint32_t cntIndex);

/**
 * \brief   Clear Timer Counter block.
 *
 * \param   baseAddr        Base Address of the RTI instance.
 * \param   cntIndex        Conter Block index
 *
 * \return  status          Success of the operation
 *                                - Success: SystemP_SUCCESS
 *                                - Fail   : SystemP_FAILURE
 *
 * \note    The timer counter will be stopped and then cleared. The function may be invoked
 *          at device initialization prior to configuration
 *
 **/
int32_t RTI_counterClear(uint32_t baseAddr, uint32_t cntIndex);

/**
 * \brief   Clear Timer Compare block
 *
 * \param   baseAddr        Base Address of the RTI instance.
 * \param   cmpIndex        Comapre Block index
 *
 * \return  status          Success of the operation
 *                                - Success: SystemP_SUCCESS
 *                                - Fail   : SystemP_FAILURE
 *
 **/
int32_t RTI_compareClear(uint32_t baseAddr, uint32_t cmpIndex);

/**
 * \brief   Get/Read the counter value from the counter registers.
 *
 * \param   baseAddr        Base Address of the RTI instance.
 * \param   cntIndex        Conter Block index
 * \param   counterLow      Pointer to the vaue of the UP counter
 * \param   counterHigh     Pointer to the vaue of the Free running counter
 *
 * \return  status          Success of the operation
 *                                - Success: SystemP_SUCCESS
 *                                - Fail   : SystemP_FAILURE
 *
 * \note:   Value can be read from the counter register when the counter is
 *          stopped or when it is running.
 **/
int32_t RTI_counterGet(uint32_t baseAddr, uint32_t cntIndex, uint32_t *counterLow, uint32_t *counterHigh);

/**
 * \brief   Configure Capture operation
 *
 * \param   baseAddr        Base Address of the RTI instance.
 * \param   cntIndex        Conter Block index
 * \param   cntrCapSrc      Capture source
 *
 * \return  status          Success of the operation
 *                                - Success: SystemP_SUCCESS
 *                                - Fail   : SystemP_FAILURE
 *
 **/
int32_t RTI_captureConfig(uint32_t baseAddr, uint32_t cntIndex, uint32_t cntrCapSrc);

/**
 * \brief   Get/Read the counter value from the capture registers.
 *
 * \param   baseAddr        Base Address of the RTI instance.
 * \param   cntIndex        Conter Block index
 * \param   counterLow      Pointer to the vaue of the capture UP counter
 * \param   counterHigh     Pointer to the vaue of the capture Free running counter
 *
 * \return  status          Success of the operation
 *                                - Success: SystemP_SUCCESS
 *                                - Fail   : SystemP_FAILURE
 *
 **/
int32_t RTI_captureCounterGet(uint32_t baseAddr, uint32_t cntIndex, uint32_t *counterLow, uint32_t *counterHigh);

/**
 * \brief   Confiure Compare operation
 *
 * \param   baseAddr        Base Address of the RTI instance.
 * \param   cmpIndex        Comapre Block index
 * \param   cntBlkIndex     Index of source counter block
 * \param   cmpVal          Initial compare value
 * \param   period          Period of the free running counter in terms of counts
 *
 * \return  status          Success of the operation
 *                                - Success: SystemP_SUCCESS
 *                                - Fail   : SystemP_FAILURE
 **/
int32_t RTI_compareEventConfig(uint32_t baseAddr, uint32_t cmpIndex, uint32_t cntBlkIndex, uint32_t cmpVal, uint32_t period);

/**
 * \brief   Confiure Compare operation
 *
 * \param   baseAddr        Base Address of the RTI instance.
 * \param   cntBlkIndex     Counter Block index
 * \param   clkSrc          Select clock source of the free running counter of Counter Block 0
 * \param   ntu             Select the NTU clock
 * \param   prescaler       Compare value of the up counter
 *
 * \return  status          Success of the operation
 *                                - Success: SystemP_SUCCESS
 *                                - Fail   : SystemP_FAILURE
 **/
int32_t RTI_counterConfigure(uint32_t baseAddr, uint32_t cntBlkIndex, uint32_t clkSrc, uint32_t ntu, uint32_t prescaler);

/**
 * \brief   Get the compare match register contents.
 *
 * \param   baseAddr        Base Address of the RTI instance.
 * \param   cmpIndex        Comapre Block index
 *
 * \return  This API returns the match register contents.
 *
 **/
uint32_t RTI_compareGet(uint32_t baseAddr, uint32_t cmpIndex);

/**
 * \brief   Confiure Compare Clear operation
 *
 * \param   baseAddr        Base Address of the RTI instance.
 * \param   cmpIndex        Compare Block index
 * \param   cmpClearVal     Compare Clear value
 *
 * \return  status          Success of the operation
 *                                - Success: SystemP_SUCCESS
 *                                - Fail   : SystemP_FAILURE
 **/
int32_t RTI_compareClearConfig(uint32_t baseAddr, uint32_t cmpIndex, uint32_t cmpClearVal);

/**
 * \brief   Read the status of INTFLAG register.
 *
 * \param   baseAddr        Base Address of the RTI instance.
 *
 * \return  This API returns the status of INTFLAG register.
 *
 **/
uint32_t RTI_intStatusGet(uint32_t baseAddr);

/**
 * \brief   Clear the status of interrupt events.
 *
 * \param   baseAddr        Base Address of the RTI instance.
 * \param   intFlags        Variable used to clear the events.
 *
 * 'intFlags' can take any of @ref RTI_TmrIntFlags
 *
 * \return  status          Success of the operation
 *                                - Success: SystemP_SUCCESS
 *                                - Fail   : SystemP_FAILURE
 **/
int32_t RTI_intStatusClear(uint32_t baseAddr, uint32_t intFlags);

/**
 * \brief   Enable the Timer interrupts.
 *
 * \param   baseAddr        Base Address of the RTI instance.
 * \param   intFlags        Variable used to enable the interrupts.
 *
 * 'intFlags' can take @ref RTI_TmrIntFlags
 *
 * \return  status          Success of the operation
 *                                - Success: SystemP_SUCCESS
 *                                - Fail   : SystemP_FAILURE
 *
 **/
int32_t RTI_intEnable(uint32_t baseAddr, uint32_t intFlags);

/**
 * \brief   Disable the Timer interrupts.
 *
 * \param   baseAddr        Base Address of the RTI instance.
 * \param   intFlags        Variable used to disable the interrupts.
 *
 * 'intFlags' can take @ref RTI_TmrIntFlags
 *
 * \return  status          Success of the operation
 *                                - Success: SystemP_SUCCESS
 *                                - Fail   : SystemP_FAILURE
 **/
int32_t RTI_intDisable(uint32_t baseAddr, uint32_t intFlags);

/**
 * \brief   Enable the Compare interrupt auto clear.
 *
 * \param   baseAddr        Base Address of the RTI instance.
 * \param   cmpIndex        Comapre Block index
 *
 * \return  status          Success of the operation
 *                                - Success: SystemP_SUCCESS
 *                                - Fail   : SystemP_FAILURE
 **/
int32_t RTI_intAutoClearEnable(uint32_t baseAddr, uint32_t cmpIndex);

/**
 * \brief   Disable the Compare interrupt auto clear.
 *
 * \param   baseAddr        Base Address of the RTI instance.
 * \param   cmpIndex        Comapre Block index
 *
 * \return  status          Success of the operation
 *                                - Success: SystemP_SUCCESS
 *                                - Fail   : SystemP_FAILURE
 **/
int32_t RTI_intAutoClearDisable(uint32_t baseAddr, uint32_t cmpIndex);

#ifdef __cplusplus
}
#endif

#endif /* #ifndef RTI_H_ */

/** @} */
