/*
 *  Copyright (C) 2023 Texas Instruments Incorporated
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
 *  \defgroup DRV_ELM_MODULE APIs for ELM
 *  \ingroup DRV_MODULE
 *
 *  This module contains APIs to program and use the ELM module. The APIs
 *  can be used by other drivers to get access to ELM and also by
 *  application to initiate data transfer operation.
 *
 *  @{
 */

/**
 *  \file v0/elm.h
 *
 *  \brief ELM Driver API/interface file.
 */

#ifndef ELM_H
#define ELM_H

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <drivers/hw_include/cslr_elm.h>

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
*                           MACRO DEFINITIONS
*******************************************************************************/
/*****************************************************************************/

/**
 *  \brief Macros which can be used as 'configVal' parameter
 *         to ELM_autoGatingConfig API
 */
/** OCP clock is free-running */
#define ELM_AUTOGATING_OCP_FREE                       (0)
/** Automatic internal OCP clock gating strategy is applied
    based on the OCP interface activity */
#define ELM_AUTOGATING_OCP_GATING                     (1)

/**
 *  \brief Macros which can be used as 'configVal' parameter
 *         to ELM_idleModeSelect API.
 */
/** For selecting force-idle mode */
#define ELM_IDLEMODE_FORCEIDLE                        (0)
/** For selecting no-idle mode */
#define ELM_IDLEMODE_NOIDLE                           (1)
/** For selecting smart-idle mode */
#define ELM_IDLEMODE_SMARTIDLE                        (2)

/**
 *  \brief Macros which can be used as 'configVal' parameter
 *         to ELM_clockActivityOCPConfig API.
 */
/** OCP Clock is maintained during wake up period */
#define ELM_CLOCKACTIVITYOCP_OCP_ON                   (1)
/** OCP clock is switch off */
#define ELM_CLOCKACTIVITYOCP_OCP_OFF                  (0)

/**
 *  \brief Macros which can be used as 'flag' parameter to ELM_interuptStatusGet API.
 */
/** Error-location status for syndrome polynomial 0 */
#define ELM_LOC_VALID_0_STATUS                        (0U)
/** Error-location status for syndrome polynomial 1 */
#define ELM_LOC_VALID_1_STATUS                        (1U)
/** Error-location status for syndrome polynomial 2 */
#define ELM_LOC_VALID_2_STATUS                        (2U)
/** Error-location status for syndrome polynomial 3 */
#define ELM_LOC_VALID_3_STATUS                        (3U)
/** Error-location status for syndrome polynomial 4 */
#define ELM_LOC_VALID_4_STATUS                        (4U)
/** Error-location status for syndrome polynomial 5 */
#define ELM_LOC_VALID_5_STATUS                        (5U)
/** Error-location status for syndrome polynomial 6 */
#define ELM_LOC_VALID_6_STATUS                        (6U)
/** Error-location status for syndrome polynomial 7 */
#define ELM_LOC_VALID_7_STATUS                        (7U)
/** Error-location status for a full page */
#define ELM_PAGE_VALID_STATUS                         (8U)

/**
 *  \brief Macros which can be used as 'flag' parameter to ELM_interuptConfig API.
 */
/** Enables ELM interrupts */
#define ELM_INT_ENALBLE                               (1U)
/** Disables ELM interrupts */
#define ELM_INT_DISALBLE                              (0U)

/**
 *  \brief Macros which can be used as 'bchECCLevel' parameter to
 *         ELM_errorCorrectionLevelSet API.
 */
/** BCH level for Upto 4 bits error correction */
#define ELM_ECC_BCH_LEVEL_4BITS                       (0U)
/** BCH level for Upto 8 bits error correction */
#define ELM_ECC_BCH_LEVEL_8BITS                       (1U)
/** BCH level for Upto 16 bits error correction */
#define ELM_ECC_BCH_LEVEL_16BITS                      (2U)

/**
 *  \brief Macros which can be used as 'mode' parameter to ELM_setSectorMode API.
 */
/** Enables continuous mode of ELM module */
#define ELM_MODE_CONTINUOUS                           (0U)
/** Enables page mode of ELM module */
#define ELM_MODE_PAGE                                 (1U)

/**
 *  \brief Macros which can be used as 'sectorNum' parameter to ELM_setSectorMode API.
 */
/** For selecting syndrome polynomial 0 as part of the page in page mode */
#define ELM_PAGEMODE_SECTOR_0                         (0U)
/** For selecting syndrome polynomial 1 as part of the page in page mode */
#define ELM_PAGEMODE_SECTOR_1                         (1U)
/** For selecting syndrome polynomial 2 as part of the page in page mode */
#define ELM_PAGEMODE_SECTOR_2                         (2U)
/** For selecting syndrome polynomial 3 as part of the page in page mode */
#define ELM_PAGEMODE_SECTOR_3                         (3U)
/** For selecting syndrome polynomial 4 as part of the page in page mode */
#define ELM_PAGEMODE_SECTOR_4                         (4U)
/** For selecting syndrome polynomial 5 as part of the page in page mode */
#define ELM_PAGEMODE_SECTOR_5                         (5U)
/** For selecting syndrome polynomial 6 as part of the page in page mode */
#define ELM_PAGEMODE_SECTOR_6                         (6U)
/** For selecting syndrome polynomial 7 as part of the page in page mode */
#define ELM_PAGEMODE_SECTOR_7                         (7U)

/**
 *  \brief Macros which can be used as 'synFrgmtId' parameter to
 *         ELM_setSyndromeFragment API
 */
/** Selects syndrome fragment 0 for error-location processing */
#define ELM_SYNDROME_FRGMT_0                          (0U)
/** Selects syndrome fragment 1 for error-location processing */
#define ELM_SYNDROME_FRGMT_1                          (1U)
/** Selects syndrome fragment 2 for error-location processing */
#define ELM_SYNDROME_FRGMT_2                          (2U)
/** Selects syndrome fragment 3 for error-location processing */
#define ELM_SYNDROME_FRGMT_3                          (3U)
/** Selects syndrome fragment 4 for error-location processing */
#define ELM_SYNDROME_FRGMT_4                          (4U)
/** Selects syndrome fragment 5 for error-location processing */
#define ELM_SYNDROME_FRGMT_5                          (5U)
/** Selects syndrome fragment 6 for error-location processing */
#define ELM_SYNDROME_FRGMT_6                          (6U)

/**
 *  \brief Macros which can be used as 'errNum' parameter to
 *         ELM_errorLocationBitAddrGet API.
 */
/** Indicates 0th error of ELM */
#define ELM_ERROR_NUM_0                               (0U)
/** Indicates 1st error of ELM */
#define ELM_ERROR_NUM_1                               (1U)
/** Indicates 2nd error of ELM */
#define ELM_ERROR_NUM_2                               (2U)
/** Indicates 3rd error of ELM */
#define ELM_ERROR_NUM_3                               (3U)
/** Indicates 4th error of ELM */
#define ELM_ERROR_NUM_4                               (4U)
/** Indicates 5th error of ELM */
#define ELM_ERROR_NUM_5                               (5U)
/** Indicates 6th error of ELM */
#define ELM_ERROR_NUM_6                               (6U)
/** Indicates 7th error of ELM */
#define ELM_ERROR_NUM_7                               (7U)
/** Indicates 8th error of ELM */
#define ELM_ERROR_NUM_8                               (8U)
/** Indicates 9th error of ELM */
#define ELM_ERROR_NUM_9                               (9U)
/** Indicates 10th error of ELM */
#define ELM_ERROR_NUM_10                              (10U)
/** Indicates 11th error of ELM */
#define ELM_ERROR_NUM_11                              (11U)
/** Indicates 12th error of ELM */
#define ELM_ERROR_NUM_12                              (12U)
/** Indicates 13th error of ELM */
#define ELM_ERROR_NUM_13                              (13U)
/** Indicates 14th error of ELM */
#define ELM_ERROR_NUM_14                              (14U)
/** Indicates 15th error of ELM */
#define ELM_ERROR_NUM_15                              (15U)

/** Sets a bit of ELM register to high */
#define ELM_BIT_SET_HIGH        (1U)
/** Sets a bit of ELM register to low */
#define ELM_BIT_SET_LOW         (0U)

/** Enables a bit of ELM register */
#define ELM_BIT_ENABLE          (1U)
/** Disables a bit of ELM register */
#define ELM_BIT_DISABLE         (0U)

/** Handle to access ELM registers. */
#define hElmCfg  ((CSL_elmRegs *)CSL_ELM_0_CFG_REGS)

/** ELM Handle */
typedef CSL_elmRegs * elmHandle;

/*****************************************************************************
**                    FUNCTION PROTOTYPES
*****************************************************************************/

/**
* \brief   This function reads the IP revision code of ELM.\n
*
* \param   baseAddr       Memory address of the ELM.\n
*
* \return  ipRev          IP revision code of ELM.\n
*/
uint32_t ELM_revisionGet(uint32_t baseAddr);

/**
* \brief   This function configs the Internal OCP clock gating strategy.\n
*
* \param   baseAddr       Memory address of the ELM.\n
*
* \param   configVal      Config value for OCP clk gating.\n
*                         This can take one of the following values :\n
*                         ELM_AUTOGATING_OCP_FREE   -- OCP clock is free-running\n
*                         ELM_AUTOGATING_OCP_GATING -- Automatic internal OCP
*                                                      clock gating strategy is
*                                                      applied based on the OCP
*                                                      interface activity.\n
*
* \return  none.\n
*/
void ELM_autoGatingConfig(uint32_t baseAddr, uint32_t configVal);

/**
* \brief   This function sets the idle mode for ELM.\n
*
* \param   baseAddr       Memory address of the ELM.\n
*
* \param   mode           Idle mode.\n
*                         This can take one of the following values :\n
*                         ELM_IDLEMODE_FORCEIDLE -- for force-idle. \n
*                         ELM_IDLEMODE_NOIDLE    -- for no-idle. \n
*                         ELM_IDLEMODE_SMARTIDLE -- for smart-idle.\n
*
* \return  none.\n
*/
void ELM_idleModeSelect(uint32_t baseAddr, uint32_t mode);

/**
* \brief   This function Resets the ELM.\n
*
* \param   baseAddr   Memory address of the ELM.\n
*
* \return  None.\n
*/
void ELM_moduleReset(uint32_t baseAddr);

/**
* \brief   This function sets the OCP Clock activity when module is in IDLE
*          mode.\n
*
* \param   baseAddr       Memory address of the ELM.\n
*
* \param   configVal      Config value.\n
*                         This can take one of the following values :\n
*                         ELM_CLOCKACTIVITYOCP_OCP_OFF -- OCP clock is swith off\n
*                         ELM_CLOCKACTIVITYOCP_OCP_ON  -- OCP Clock is
*                                                         maintained during
*                                                         wake up period.\n
*
* \return  none.\n
*/
void ELM_clockActivityOCPConfig(uint32_t baseAddr, uint32_t configVal);

/**
* \brief   This function gets the software resets status of ELM.\n
*
* \param   baseAddr   Memory address of the ELM.\n
*
* \return  status     Reset status : \n
*                       0 : Module reset is on-going.\n
*                       1 : Module reset is completed.\n
*/
uint32_t ELM_moduleResetStatusGet(uint32_t baseAddr);

/**
* \brief   This function gets Interrupt status.\n
*
* \param   baseAddr       Memory address of the ELM.\n
*
* \param   flag           Flag for which interrupt status has to get.\n
*                         This can take one of the following values :\n
*                         ELM_LOC_VALID_0_STATUS         : Error-location
*                                                          status for syndrome
*                                                          polynomial 0.\n
*                         ELM_LOC_VALID_1_STATUS         : Error-location
*                                                          status for syndrome
*                                                          polynomial 1.\n
*                         ELM_LOC_VALID_2_STATUS         : Error-location
*                                                          status for syndrome
*                                                          polynomial 2.\n
*                         ELM_LOC_VALID_3_STATUS         : Error-location
*                                                          status for syndrome
*                                                          polynomial 3.\n
*                         ELM_LOC_VALID_4_STATUS         : Error-location
*                                                          status for syndrome
*                                                          polynomial 4.\n
*                         ELM_LOC_VALID_5_STATUS         : Error-location
*                                                          status for syndrome
*                                                          polynomial 5.\n
*                         ELM_LOC_VALID_6_STATUS         : Error-location
*                                                          status for syndrome
*                                                          polynomial 6.\n
*                         ELM_LOC_VALID_7_STATUS         : Error-location
*                                                          status for syndrome
*                                                          polynomial 7.\n
*                         ELM_PAGE_VALID_STATUS          : Error-location
*                                                          status for a full
*                                                          page. \n
*
* \return  intSts         Interrupt Status. Return value meaning depends on the
*                         param flag. \n
*                         If flag is ELM_LOC_VALID_i_STATUS where i = 0 to 7,
*                         then\n
*                            0 : No syndrome processed or process in progress.\n
*                            1 : Error-location process completed.\n
*
*                         If flag is ELM_PAGE_VALID_STATUS, then\n
*                            0 : Error locations invalid for all polynomials
*                                enabled.\n
*                            1 : All error locations valid.\n
*
*/
uint32_t ELM_interuptStatusGet(uint32_t baseAddr, uint32_t flag);

/**
* \brief   This function clears the interrupt.\n
*
* \param   baseAddr       Memory address of the ELM.\n
*
* \param   flag           Flag for which interrupt status has to clear.\n
*                         This can take one of the following values :\n
*                         ELM_LOC_VALID_0_STATUS        : Error-location
*                                                         interrupt for syndrome
*                                                         polynomial 0.\n
*                         ELM_LOC_VALID_1_STATUS        : Error-location
*                                                         interrupt for syndrome
*                                                         polynomial 1.\n
*                         ELM_LOC_VALID_2_STATUS        : Error-location
*                                                         interrupt for syndrome
*                                                         polynomial 2.\n
*                         ELM_LOC_VALID_3_STATUS        : Error-location
*                                                         interrupt for syndrome
*                                                         polynomial 3.\n
*                         ELM_LOC_VALID_4_STATUS        : Error-location
*                                                         interrupt for syndrome
*                                                         polynomial 4.\n
*                         ELM_LOC_VALID_5_STATUS        : Error-location
*                                                         interrupt for syndrome
*                                                         polynomial 5.\n
*                         ELM_LOC_VALID_6_STATUS        : Error-location
*                                                         interrupt for syndrome
*                                                         polynomial 6.\n
*                         ELM_LOC_VALID_7_STATUS        : Error-location
*                                                         interrupt for syndrome
*                                                         polynomial 7.\n
*                         ELM_PAGE_VALID_STATUS         : Error-location
*                                                         interrupt for a full
*                                                         page.\n
*
* \return  None. \n
*
*/
void ELM_interuptStatusClear(uint32_t baseAddr, uint32_t flag);

/**
* \brief   This function configs i.e enables or disables the interrupts.\n
*
* \param   baseAddr       Memory address of the ELM.\n
*
* \param   intflag        intFlag for which interrupt has to config.\n
*                         This can take one of the following values :\n
*                         ELM_LOC_VALID_0_STATUS        : Error-location
*                                                         interrupt for syndrome
*                                                         polynomial 0.\n
*                         ELM_LOC_VALID_1_STATUS        : Error-location
*                                                         interrupt for syndrome
*                                                         polynomial 1.\n
*                         ELM_LOC_VALID_2_STATUS        : Error-location
*                                                         interrupt for syndrome
*                                                         polynomial 2.\n
*                         ELM_LOC_VALID_3_STATUS        : Error-location
*                                                         interrupt for syndrome
*                                                         polynomial 3.\n
*                         ELM_LOC_VALID_4_STATUS        : Error-location
*                                                         interrupt for syndrome
*                                                         polynomial 4.\n
*                         ELM_LOC_VALID_5_STATUS        : Error-location
*                                                         interrupt for syndrome
*                                                         polynomial 5.\n
*                         ELM_LOC_VALID_6_STATUS        : Error-location
*                                                         interrupt for syndrome
*                                                         polynomial 6.\n
*                         ELM_LOC_VALID_7_STATUS        : Error-location
*                                                         interrupt for syndrome
*                                                         polynomial 7.\n
*                         ELM_PAGE_VALID_STATUS         : Error-location
*                                                         interrupt for a full
*                                                         page. \n
*
* \param   configVal     Config Value.\n
*                        This can take one of the following values :\n
*                        ELM_INT_ENALBLE                : To enble the interrupt\n
*                        ELM_INT_DISALBLE               : To disable the
*                                                         interrupt.\n
* \return  None. \n
*
*/
void ELM_interuptConfig(uint32_t baseAddr, uint32_t intFlag, uint32_t configVal);

/**
* \brief   This function sets the Error correction level for BCH alogorithm.\n
*
* \param   baseAddr       Memory address of the ELM.\n
*
* \param   bchECCLevel    BCH error correction level.\n
*                         This can take one of the following values :\n
*                         ELM_ECC_BCH_LEVEL_16BITS    : For Upto 16 bits error
*                                                       correction.\n
*                         ELM_ECC_BCH_LEVEL_8BITS     : For Upto 8 bits error
*                                                       correction.\n
*                         ELM_ECC_BCH_LEVEL_4BITS     : For Upto 4 bits error
*                                                       correction.\n
*
* \return  None. \n
*
*/
void ELM_errorCorrectionLevelSet(uint32_t baseAddr, uint32_t eccLevel);

/**
* \brief   This function sets the size of the buffers for which
*          the error-location engine is used.\n
*
* \param   baseAddr   Memory address of the ELM.\n
*
* \param   eccSize    eccSize in number of nibbles (i.e 4-bits entities).\n
*
* \return  None. \n
*
*/
void ELM_setECCSize(uint32_t baseAddr, uint32_t eccSize);

/**
* \brief   This function sets mode of the ELM module.\n
*
* \param   baseAddr       Memory address of the ELM.\n
* \param   mode           mode of the ELM.\n
*                         This can take one of the following values :\n
*                         ELM_MODE_PAGE       -- For page mode.\n
*                         ELM_MODE_CONTINUOUS -- For continuous mode.\n
*
* \param   sectorNum      Sector number or syndrome polynomial number which has
*                         to select as part of the page in page mode.\n
*                         This can take one of the following values :\n
*                         ELM_PAGEMODE_SECTOR_0 -- For selecting syndrome
*                                                  polynomial 0 as part of the
*                                                  page in page mode.\n
*                         ELM_PAGEMODE_SECTOR_1 -- For selecting syndrome
*                                                  polynomial 1 as part of the
*                                                  page in page mode.\n
*                         ELM_PAGEMODE_SECTOR_2 -- For selecting syndrome
*                                                  polynomial 2 as part of the
*                                                  page in page mode.\n
*                         ELM_PAGEMODE_SECTOR_3 -- For selecting syndrome
*                                                  polynomial 3 as part of the
*                                                  page in page mode.\n
*                         ELM_PAGEMODE_SECTOR_4 -- For selecting syndrome
*                                                  polynomial 4 as part of the
*                                                  page in page mode.\n
*                         ELM_PAGEMODE_SECTOR_5 -- For selecting syndrome
*                                                  polynomial 5 as part of the
*                                                  page in page mode.\n
*                         ELM_PAGEMODE_SECTOR_6 -- For selecting syndrome
*                                                  polynomial 6 as part of the
*                                                  page in page mode.\n
*                         ELM_PAGEMODE_SECTOR_7 -- For selecting syndrome
*                                                  polynomial 7 as part of the
*                                                  page in page mode.\n

* \return  None. \n
*
*/
void ELM_setSectorMode(uint32_t baseAddr, uint32_t mode, uint32_t sectorNum);

/**
* \brief   This function sets the fragments of syndrome polynomial for
*          error-location processing.\n
*
* \param   baseAddr       Memory address of the ELM.\n
*
* \param   synFrgmtId     Syndrome fragment ID.\n
*                         This can take one of the following values :\n
*                         ELM_SYNDROME_FRGMT_0 - For syndrome fragment 0.\n
*                         ELM_SYNDROME_FRGMT_1 - For syndrome fragment 1.\n
*                         ELM_SYNDROME_FRGMT_2 - For syndrome fragment 2.\n
*                         ELM_SYNDROME_FRGMT_3 - For syndrome fragment 3.\n
*                         ELM_SYNDROME_FRGMT_4 - For syndrome fragment 4.\n
*                         ELM_SYNDROME_FRGMT_5 - For syndrome fragment 5.\n
*                         ELM_SYNDROME_FRGMT_6 - For syndrome fragment 6.\n
* \param   synFrgmtVal    Syndrome fragment value.\n
*
* \param   sector         528 bytes sector num.\n
*
* \return  None. \n
*
*/
void ELM_setSyndromeFragment(uint32_t baseAddr, uint32_t synFrgmtId,
                         uint32_t synFrgmtVal, uint32_t sector);

/**
* \brief   This function starts the error-location processing for the polynomial
*          set in the syndrome fragment registers.\n
*
* \param   baseAddr   Memory address of the ELM.\n
*
* \param   sector     528 bytes sector num.\n
*
* \return  None. \n
*
*/
void ELM_errorLocationProcessingStart(uint32_t baseAddr, uint32_t sector);

/**
* \brief   This function gets the error-location processing status.\n
*
* \param   baseAddr   Memory address of the ELM.\n
*
* \param   sector     Page sector num.\n
*
* \return  status. \n
*          0 : ECC error-location process failed.\n
*          1 : ECC error-location process passed.\n
*
*/
uint32_t ELM_errorLocationProcessingStatusGet(uint32_t baseAddr, uint32_t sector);

/**
* \brief   This function gets the number of errors detected and located
*          during error-location processing.\n
*
* \param   baseAddr   Memory address of the ELM.\n
*
* \param   sector     Page sector num, 0 - 7.\n
*
* \return  Number of errors detected and located.\n
*
*/
uint32_t ELM_getNumError(uint32_t baseAddr, uint32_t sector);

/**
* \brief   This function gets the Error-location bit address for the error
*          number passed.\n
*
* \param   baseAddr       Memory address of the ELM.\n
*
* \param   errNum         Error number.\n
*                         This can take one of the following values :\n
*                         ELM_ERROR_NUM_0 - For 0th error.\n
*                         ELM_ERROR_NUM_1 - For 1th error.\n
*                         ELM_ERROR_NUM_2 - For 2th error.\n
*                         ELM_ERROR_NUM_3 - For 3th error.\n
*                         ELM_ERROR_NUM_4 - For 4th error.\n
*                         ELM_ERROR_NUM_5 - For 5th error.\n
*                         ELM_ERROR_NUM_6 - For 6th error.\n
*                         ELM_ERROR_NUM_7 - For 7th error.\n
*                         ELM_ERROR_NUM_8 - For 8th error.\n
*                         ELM_ERROR_NUM_9 - For 9th error.\n
*                         ELM_ERROR_NUM_10 - For 10th error.\n
*                         ELM_ERROR_NUM_11 - For 11th error.\n
*                         ELM_ERROR_NUM_12 - For 12th error.\n
*                         ELM_ERROR_NUM_13 - For 13th error.\n
*                         ELM_ERROR_NUM_14 - For 14th error.\n
*                         ELM_ERROR_NUM_15 - For 15th error.\n
*
* \param   sector      Page sector num, 0 - 7.\n
*
* \return  Bit address for the error number.\n
*
*/
uint32_t ELM_errorLocationBitAddrGet(uint32_t baseAddr, uint32_t errNum, uint32_t sector);


#ifdef __cplusplus
}
#endif

#endif /* ELM_H */
