/*!
 *  \file CUST_drivers.c
 *
 *  \brief
 *  Provides initialization of custom drivers.
 *
 *  \author
 *  KUNBUS GmbH
 *
 *  \date
 *  2022-03-16
 *
 *  \copyright
 *  Copyright (c) 2022, KUNBUS GmbH<br /><br />
 *  All rights reserved.<br />
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:<br />
 *  <ol>
 *  <li>Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.</li>
 *  <li>Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.</li>
 *  <li>Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.</li>
 *  </ol>
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "ti_board_open_close.h"
#include "ti_drivers_open_close.h"

#include <drivers/CUST_drivers.h>

extern PRUICSS_Config gPruIcssConfig[];
extern ETHPHY_Config  gEthPhyConfig[];
extern ETHPHY_Handle  gEthPhyHandle[];

#define WRITE_STACK_SIZE_BYTE     1024
#define WRITE_FLASH_STACK_SIZE    (WRITE_STACK_SIZE_BYTE/sizeof(configSTACK_DEPTH_TYPE))

typedef struct CUST_DRIVERS_SPermWriteParam
{
    void*        handle;
    uint32_t     offset;
    uint8_t*     pData;
    uint32_t     length;
}CUST_DRIVERS_SPermWriteParam_t;

static void CUST_DRIVERS_flashWriteTask  (void *pArg_p);
static void CUST_DRIVERS_eepromWriteTask (void *pArg_p);

static OSAL_TASK_EPriority_t    eepromTaskPrio_s;
static OSAL_TASK_EPriority_t    flashTaskPrio_s;

static char aOutStream_s[0x200] = {0};

static uint32_t writeOperationPendingCnt_s = 0;
static bool     writeOperationPending_s    = false;

static CUST_DRIVERS_SPermWriteParam_t  writeParam_s;
static void*                           flashTaskHandle_s = NULL;
static StackType_t                     writeTaskStack_s[WRITE_FLASH_STACK_SIZE] __attribute__((aligned(32), section(".threadstack"))) = {0};

/*!
* <!-- Description: -->
*
* \brief
* Custom drivers initialization.
*
*  \return     uint32_t                      Error code.
*
*  \retval     CUST_DRIVERS_eERR_NOERROR     Success.
*  \retval     CUST_DRIVERS_eERR_PHY         PHY driver initialization failed.
*  \retval     CUST_DRIVERS_eERR_UART        UART driver initialization failed.
*  \retval     CUST_DRIVERS_eERR_LED         LED driver initialization failed.
*  \retval     CUST_DRIVERS_eERR_FLASH       FLASH driver initialization failed.
*
*/
uint32_t CUST_DRIVERS_init(CUST_DRIVERS_SInit_t* pParams_p)
{
    uint32_t error = (uint32_t) CUST_DRIVERS_eERR_NOERROR;
    CUST_PHY_SParams_t phyParams = { .pPruIcssCfg = CUST_DRIVERS_getPruIcssCfg(pParams_p->pruIcss.instance),
                                     .pEthPhy0Cfg = CUST_DRIVERS_getEthPhyCfg (pParams_p->pruIcss.ethPhy.instance_0),
                                     .pEthPhy1Cfg = CUST_DRIVERS_getEthPhyCfg (pParams_p->pruIcss.ethPhy.instance_1)
                                   };

    eepromTaskPrio_s = pParams_p->eeprom.taskPrio;
    flashTaskPrio_s  = pParams_p->flash.taskPrio;

    Drivers_open();

    OSAL_registerPrintOut(NULL, PRINTF_CALLBACK_FUNCTION);

    if (CUST_UART_eERR_NOERROR != CUST_UART_init())
    {
        error = (uint32_t) CUST_DRIVERS_eERR_UART;
        goto initErr;
    }

    if (CUST_PHY_eERR_NOERROR != CUST_PHY_init(&phyParams))
    {
        error = (uint32_t) CUST_DRIVERS_eERR_PHY;
        goto initErr;
    }

    if (SystemP_SUCCESS != Board_driversOpen())
    {
        error = (uint32_t) CUST_DRIVERS_eERR_GENERALERROR;
        goto initErr;
    }

    if (CUST_LED_eERR_NOERROR != CUST_LED_init())
    {
        error = (uint32_t) CUST_DRIVERS_eERR_LED;
        goto initErr;
    }

    if (PERMANENT_DATA_MEMORY_TYPE == CUST_DRIVERS_PRM_eTYPE_FLASH)
    {
        if (CUST_FLASH_eERR_NOERROR != CUST_FLASH_init())
        {
            error = (uint32_t) CUST_DRIVERS_eERR_FLASH;
            goto initErr;
        }
    }
    else if (PERMANENT_DATA_MEMORY_TYPE == CUST_DRIVERS_PRM_eTYPE_EEPROM)
    {
        if (CUST_EEPROM_eERR_NOERROR != CUST_EEPROM_init())
        {
            error = (uint32_t) CUST_DRIVERS_eERR_EEPROM;
            goto initErr;
        }
    }
    else
        error = (uint32_t) CUST_DRIVERS_eERR_NO_PERMANENT_STORAGE;

initErr:
    return error;
}

/*!
* <!-- Description: -->
*
* \brief
* Custom drivers deinitialization.
*
*  \return     uint32_t                        Error code.
*
*  \retval     CUST_DRIVERS_eERR_NOERROR       Success.
*  \retval     CUST_DRIVERS_eERR_PHY           PHY driver deinitialization failed.
*  \retval     CUST_DRIVERS_eERR_UART          UART driver deinitialization failed.
*  \retval     CUST_DRIVERS_eERR_LED           LED driver deinitialization failed.
*  \retval     CUST_DRIVERS_eERR_FLASH         FLASH driver deinitialization failed.
*
*/
uint32_t CUST_DRIVERS_deinit(void)
{
    uint32_t error = (uint32_t) CUST_DRIVERS_eERR_NOERROR;

    if (CUST_LED_eERR_NOERROR != CUST_LED_deInit())
    {
        error = (uint32_t) CUST_DRIVERS_eERR_LED;
        goto deinitErr;
    }

    if (CUST_FLASH_eERR_NOERROR != CUST_FLASH_deInit())
    {
        error = (uint32_t) CUST_DRIVERS_eERR_FLASH;
        goto deinitErr;
    }

    if (CUST_PHY_eERR_NOERROR != CUST_PHY_deInit())
    {
        error = (uint32_t) CUST_DRIVERS_eERR_PHY;
        goto deinitErr;
    }

    Board_driversClose();

    if (CUST_UART_eERR_NOERROR != CUST_UART_deInit())
    {
        error = (uint32_t) CUST_DRIVERS_eERR_UART;
        goto deinitErr;
    }

    Drivers_close();

deinitErr:
    return error;
}

/*!
* <!-- Description: -->
*
* \brief
* Provides pointer to specific SysConfig PRU-ICSS block configuration defined by instance.
*
*  <!-- Parameters and return values: -->
*
*  \param[in]  instance_p      Instance number
*  \return     pointer to requested PRU-ICSS block configuration
*
*/
PRUICSS_Config* CUST_DRIVERS_getPruIcssCfg (uint32_t instance_p)
{
    PRUICSS_Config* pPruIcssCfg = NULL;

    pPruIcssCfg = &gPruIcssConfig[instance_p];

    return pPruIcssCfg;
}

/*!
* <!-- Description: -->
*
* \brief
* Provides pointer to specific SysConfig ETHPHY configuration defined by instance.
*
*  <!-- Parameters and return values: -->
*
*  \param[in]  instance_p      Instance number
*  \return     pointer to requested ETHPHY configuration
*
*/
ETHPHY_Config*  CUST_DRIVERS_getEthPhyCfg (uint32_t instance_p)
{
    ETHPHY_Config* pEthPhyCfg = NULL;

    pEthPhyCfg = &gEthPhyConfig[instance_p];

    return pEthPhyCfg;
}

/*!
* <!-- Description: -->
*
* \brief
* Provides pointer to specific ETHPHY handler defined by instance.
*
*  <!-- Parameters and return values: -->
*
*  \param[in]  instance_p      Instance number
*  \return     requested ETHPHY handler
*
*/
ETHPHY_Handle CUST_DRIVERS_getEthPhyHandle (uint32_t instance_p)
{
    ETHPHY_Handle ethPhyHandle = NULL;

    ethPhyHandle = gEthPhyHandle[instance_p];

    return ethPhyHandle;
}

/*!
* <!-- Description: -->
*
* \brief
* Provides MDIO manual mode SysConfig setting.
*
*  <!-- Parameters and return values: -->
*
*  \return     MDIO manual mode ON/OFF setting
*
*  \retval     true   MDIO Manual Mode is ON.
*  \retval     false  MDIO Manual Mode is OFF.
*/
bool CUST_DRIVERS_getMdioManualMode (void)
{
    bool result = false;

#if (defined MDIO_MANUAL_MODE_ENABLED)
    result = true;
#endif

    return result;
}

/*!
* <!-- Description: -->
*
* \brief
* Provides MDIO manual mode base address as defined in SysConfig.
*
*  <!-- Parameters and return values: -->
*
*  \return     MDIO manual mode base address
*
*  \retval     0      MDIO Manual Mode Base Address is not defined.
*  \retval     other  MDIO Manual Mode Base Address as defined.
*/
uint32_t CUST_DRIVERS_getMdioManualModeBaseAddress (void)
{
    uint32_t result = 0;

#if (defined MDIO_MANUAL_MODE_ENABLED) && (defined MDIO_MANUAL_MODE_BASE_ADDRESS)
    result = MDIO_MANUAL_MODE_BASE_ADDRESS;
#endif

    return result;
}

/*!
* <!-- Description: -->
*
* \brief
* UART printf output function.
*
* \details
* Printing of specific string to UART output.
*
*
*  <!-- Parameters and return values: -->
*
*  \param[in]  pContext_p      Call context
*  \param[in]  pFormat_p       Format string.
*  \param[in]  argptr_p        Parameter list.
*
*
*/
void CUST_DRIVERS_UART_printf(void* pContext_p, const char* __restrict pFormat_p, va_list argptr_p)
{
    /* @cppcheck_justify{unusedVariable} false-positive: variable is used */
    //cppcheck-suppress unusedVariable
    int32_t transferOK;
    /* @cppcheck_justify{unusedVariable} false-positive: variable is used */
    //cppcheck-suppress unusedVariable
    UART_Transaction transaction;

    UART_Transaction_init(&transaction);

    OSAL_MEMORY_memset(aOutStream_s, 0, sizeof(aOutStream_s));
    (void)vsnprintf(aOutStream_s, sizeof(aOutStream_s), pFormat_p, argptr_p);

    transaction.count = strlen(aOutStream_s);
    transaction.buf = (void *)aOutStream_s;
    transaction.args = NULL;
    transferOK = UART_write(gUartHandle[PRINTF_UART_CALLBACK_INSTANCE], &transaction);

    (void)transferOK;
}

/*!
* <!-- Description: -->
*
* \brief
* DebugLog printf output function.
*
* \details
* Printing of specific string to CCS console output.
*
*
*  <!-- Parameters and return values: -->
*
*  \param[in]  pContext_p      Call context
*  \param[in]  pFormat_p       Format string.
*  \param[in]  argptr_p        Parameter list.
*
*
*/
void CUST_DRIVERS_LOG_printf(void* pContext_p, const char* __restrict pFormat_p, va_list argptr_p)
{
    OSAL_MEMORY_memset(aOutStream_s, 0, sizeof(aOutStream_s));
    (void)vsnprintf(aOutStream_s, sizeof(aOutStream_s), pFormat_p, argptr_p);

    DebugP_log(aOutStream_s);
}

/*!
* <!-- Description: -->
*
* \brief
* Provides handler to permanent data memory.
*
*  <!-- Parameters and return values: -->
*
*  \param[in]  type_p          Defines memory type (FLASH, EEPROM, ...).
*  \param[in]  instance_p      Instance number of permanent data memory.
*
*  \return     void*           Handler to permanent data memory.
*
*/
void* CUST_DRIVERS_PRM_getHandle (uint32_t type_p, uint32_t instance_p)
{
    void*   pHandle = NULL;

    switch(type_p)
    {
        case CUST_DRIVERS_PRM_eTYPE_FLASH:
            pHandle = Flash_getHandle(instance_p);
            break;
        case CUST_DRIVERS_PRM_eTYPE_EEPROM:
            pHandle = gEepromHandle[instance_p];
            break;
        default:
            pHandle = NULL;
            break;
    }

    return pHandle;
}


/*!
* <!-- Description: -->
*
* \brief
* Reads permanent data.
*
*  <!-- Parameters and return values: -->
*
*  \param[in]  handler_p       Permanent data memory handler.
*  \param[in]  type_p          Permanent data memory type.
*  \param[in]  offset_p        Permanent data memory offset where data need to be written.
*  \param[in]  pBuf_p          Pointer to read buffer.
*  \param[in]  length_p        Length of data to be read.
*
*  \return     uint32_t                                 Error code.
*
*  \retval     CUST_DRIVERS_eERR_NOERROR                Success.
*  \retval     CUST_DRIVERS_eERR_GENERALERROR           Failed.
*  \retval     CUST_DRIVERS_eERR_FLASH_HANDLE_INVALID   FLASH handle is invalid.
*  \retval     CUST_DRIVERS_eERR_FLASH_READ             FLASH read error.
*  \retval     CUST_DRIVERS_eERR_EEPROM_HANDLE_INVALID  EEPROM handle is invalid.
*  \retval     CUST_DRIVERS_eERR_EEPROM                 EEPROM driver error.
*
*
*/
uint32_t CUST_DRIVERS_PRM_read (void* handler_p, uint32_t type_p, uint32_t offset_p, uint8_t* pBuf_p, uint32_t length_p)
{
    int32_t  ret = SystemP_FAILURE;
    uint32_t err = (uint32_t) CUST_DRIVERS_eERR_GENERALERROR;

    switch(type_p)
    {
        case CUST_DRIVERS_PRM_eTYPE_FLASH:
        {
            if (NULL == handler_p)
            {
                err = (uint32_t) CUST_DRIVERS_eERR_FLASH_HANDLE_INVALID;
                break;
            }

            ret = Flash_read ((Flash_Handle) handler_p, offset_p, pBuf_p, length_p);

            if (SystemP_SUCCESS != ret)
            {
                err = (uint32_t) CUST_DRIVERS_eERR_FLASH_READ;
                break;
            }

            err = (uint32_t) CUST_DRIVERS_eERR_NOERROR;
        }break;
        case CUST_DRIVERS_PRM_eTYPE_EEPROM:
        {
            if (NULL == handler_p)
            {
                err = (uint32_t) CUST_DRIVERS_eERR_EEPROM_HANDLE_INVALID;
                break;
            }

            ret = EEPROM_read ((EEPROM_Handle) handler_p, offset_p, pBuf_p, length_p);

            if (SystemP_SUCCESS != ret)
            {
                err = (uint32_t) CUST_DRIVERS_eERR_EEPROM_READ;
                break;
            }

            err = (uint32_t) CUST_DRIVERS_eERR_NOERROR;
        }break;
        default:
            break;
    }

    return err;
}

/*!
* <!-- Description: -->
*
* \brief
* Writes permanent data.
*
*  <!-- Parameters and return values: -->
*
*  \param[in]  handler_p       Permanent data memory handler.
*  \param[in]  type_p          Permanent data memory type.
*  \param[in]  offset_p        Permanent data memory offset where data need to be written.
*  \param[in]  pBuf_p          Pointer to write buffer.
*  \param[in]  length_p        Length of data to be read.
*  \param[in]  blocking_p      Waits until operation is finished.
*
*  \return     uint32_t                             Error code.
*
*  \retval     CUST_DRIVERS_eERR_NOERROR            Success.
*  \retval     CUST_DRIVERS_eERR_GENERALERROR       Failed.
*  \retval     CUST_DRIVERS_eERR_FLASH_DATA_INVALID Pointer to data to be flashed is invalid.
*  \retval     CUST_DRIVERS_eERR_EEPROM_WRITE       EEPROM write error.
*
*/
uint32_t CUST_DRIVERS_PRM_write (void* handler_p, uint32_t type_p, uint32_t offset_p, uint8_t* pBuf_p, uint32_t length_p, bool blocking_p)
{
    uint32_t err = (uint32_t) CUST_DRIVERS_eERR_GENERALERROR;

    switch(type_p)
    {
        case CUST_DRIVERS_PRM_eTYPE_FLASH:
        {
            writeOperationPendingCnt_s++;

            // Wait until last write to flash is done
            while(true == writeOperationPending_s)
            {
                OSAL_SCHED_sleep(100);
            }

            // allocate memory for data to be written
            writeParam_s.handle = handler_p;
            writeParam_s.offset = offset_p;
            writeParam_s.pData  = OSAL_MEMORY_calloc(length_p, 1);
            writeParam_s.length = length_p;

            if (NULL == writeParam_s.pData)
            {
                OSAL_printf("Func: %s, Line: %lu: Memory allocation of %lu bytes failed.\r\n", __func__, __LINE__, length_p);

                writeOperationPendingCnt_s--;
                err = (uint32_t) CUST_DRIVERS_eERR_FLASH_DATA_INVALID;
                break;
            }

            // Make a copy of the permanent data so that the following calls can make their changes
            OSAL_MEMORY_memcpy(writeParam_s.pData, pBuf_p, length_p);

            writeOperationPending_s = true;

            // Start writing the flash in a thread so that the main task is not blocked
            flashTaskHandle_s = OSAL_SCHED_startTask (CUST_DRIVERS_flashWriteTask,
                                                      &writeParam_s,
                                                      flashTaskPrio_s,
                                                      (uint8_t*) writeTaskStack_s,
                                                      WRITE_STACK_SIZE_BYTE,
                                                      OSAL_OS_START_TASK_FLG_NONE,
                                                      "WriteFlash");

            if (true == blocking_p)
            {
                while(true == writeOperationPending_s)
                {
                    OSAL_SCHED_sleep(100);
                }
            }

            err = (uint32_t) CUST_DRIVERS_eERR_NOERROR;
        }break;
        case CUST_DRIVERS_PRM_eTYPE_EEPROM:
        {
            writeOperationPendingCnt_s++;

            // Wait until last write to eeprom is done
            while(true == writeOperationPending_s)
            {
                OSAL_SCHED_sleep(100);
            }

            // allocate memory for data to be written
            writeParam_s.handle = handler_p;
            writeParam_s.offset = offset_p;
            writeParam_s.pData  = OSAL_MEMORY_calloc(length_p, 1);
            writeParam_s.length = length_p;

            if (NULL == writeParam_s.pData)
            {
                OSAL_printf("Func: %s, Line: %lu: Memory allocation of %lu bytes failed.\r\n", __func__, __LINE__, length_p);

                writeOperationPendingCnt_s--;
                err = (uint32_t) CUST_DRIVERS_eERR_EEPROM_DATA_INVALID;
                break;
            }

            // Make a copy of the permanent data so that the following calls can make their changes
            OSAL_MEMORY_memcpy(writeParam_s.pData, pBuf_p, length_p);

            writeOperationPending_s = true;

            // Start writing the flash in a thread so that the main task is not blocked
            flashTaskHandle_s = OSAL_SCHED_startTask (CUST_DRIVERS_eepromWriteTask,
                                                      &writeParam_s,
                                                      eepromTaskPrio_s,
                                                      (uint8_t*) writeTaskStack_s,
                                                      WRITE_STACK_SIZE_BYTE,
                                                      OSAL_OS_START_TASK_FLG_NONE,
                                                      "WriteEeprom");

            if (true == blocking_p)
            {
                while(true == writeOperationPending_s)
                {
                    OSAL_SCHED_sleep(100);
                }
            }

            err = (uint32_t) CUST_DRIVERS_eERR_NOERROR;
        }break;
        default:
            break;
    }

    return err;
}

/*!
* <!-- Description: -->
*
* \brief
* Provides permanent data write operation status.
*
*  <!-- Parameters and return values: -->
*
*  \return     bool        Status of permanent data write operation.
*
*  \retval     false       Not pending.
*  \retval     true        Pending.
*
*/
bool CUST_DRIVERS_PRM_isWritePending (void)
{
    bool ret = true;

    if ((writeOperationPending_s    == false) &&
        (writeOperationPendingCnt_s == 0))
    {
        ret = false;
    }

    return ret;
}

/*!
* <!-- Description: -->
*
* \brief
* Task function responsible for writing of data to the FLASH.
*
*  <!-- Parameters and return values: -->
*
*  \param[in]  pArg_p          Pointer to task argument of CUST_DRIVERS_SPermWriteParam_t type.
*
*/
static void CUST_DRIVERS_flashWriteTask (void *pArg_p)
{
    CUST_DRIVERS_SPermWriteParam_t* pParam = (CUST_DRIVERS_SPermWriteParam_t*) pArg_p;

    Flash_Attrs *pAttr;
    int32_t      err;
    uint32_t     iOffset;
    uint32_t     block;
    uint32_t     page;

    if (NULL == pParam->handle)
    {
        OSAL_error (__func__, __LINE__, CUST_DRIVERS_eERR_FLASH_HANDLE_INVALID, true, 0);
        goto laError;
    }

    if (NULL == pParam->pData)
    {
        OSAL_error (__func__, __LINE__, CUST_DRIVERS_eERR_FLASH_DATA_INVALID, true, 0);
        goto laError;
    }

    if (0 == pParam->length)
    {
        OSAL_error (__func__, __LINE__, CUST_DRIVERS_eERR_FLASH_LENGTH_INVALID, true, 0);
        goto laError;
    }

    pAttr = ((Flash_Config*) pParam->handle)->attrs;

    for (iOffset = pParam->offset; iOffset <= pParam->offset + pParam->length; iOffset += pAttr->blockSize)
    {
        err = Flash_offsetToBlkPage ((Flash_Handle) pParam->handle, iOffset, &block, &page);
        if (SystemP_SUCCESS != err)
        {
            OSAL_error (__func__, __LINE__, CUST_DRIVERS_eERR_FLASH_OFFSET, true, 0);
            goto laError;
        }

        err = Flash_eraseBlk ((Flash_Handle) pParam->handle, block);
        if (SystemP_SUCCESS != err)
        {
            OSAL_error (__func__, __LINE__, CUST_DRIVERS_eERR_FLASH_ERASE, true, 0);
            goto laError;
        }
    }

    err = Flash_write ((Flash_Handle) pParam->handle, pParam->offset, pParam->pData, pParam->length);
    if (SystemP_SUCCESS != err)
    {
        OSAL_error (__func__, __LINE__, CUST_DRIVERS_eERR_FLASH_WRITE, true, 0);
        goto laError;
    }

    OSAL_MEMORY_free(pParam->pData);

laError:
    writeOperationPending_s = false;
    writeOperationPendingCnt_s--;

    OSAL_SCHED_exitTask(NULL);
}

/*!
* <!-- Description: -->
*
* \brief
* Task function responsible for writing of data to the EEPROM.
*
*  <!-- Parameters and return values: -->
*
*  \param[in]  pArg_p          Pointer to task argument of CUST_DRIVERS_SPermWriteParam_t type.
*
*/
static void CUST_DRIVERS_eepromWriteTask (void *pArg_p)
{
    CUST_DRIVERS_SPermWriteParam_t* pParam = (CUST_DRIVERS_SPermWriteParam_t*) pArg_p;

    int32_t      err;

    if (NULL == pParam->handle)
    {
        OSAL_error (__func__, __LINE__, CUST_DRIVERS_eERR_EEPROM_HANDLE_INVALID, true, 0);
        goto laError;
    }

    if (NULL == pParam->pData)
    {
        OSAL_error (__func__, __LINE__, CUST_DRIVERS_eERR_EEPROM_DATA_INVALID, true, 0);
        goto laError;
    }

    if (0 == pParam->length)
    {
        OSAL_error (__func__, __LINE__, CUST_DRIVERS_eERR_EEPROM_LENGTH_INVALID, true, 0);
        goto laError;
    }

    err = EEPROM_write ((EEPROM_Handle) pParam->handle, pParam->offset, pParam->pData, pParam->length);
    if (SystemP_SUCCESS != err)
    {
        OSAL_error (__func__, __LINE__, CUST_DRIVERS_eERR_EEPROM_WRITE, true, 0);
        goto laError;
    }

    OSAL_MEMORY_free(pParam->pData);

laError:
    writeOperationPending_s = false;
    writeOperationPendingCnt_s--;

    OSAL_SCHED_exitTask(NULL);
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Set industrial LEDs controlled by TPIC2810
 *
 *  \details
 *  .
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pattern             LED pattern.
 *
 */
void CUST_DRIVERS_LED_setIndustrialLeds (uint32_t value_p)
{
    static uint32_t ledValue_s = 0;

    LED_Handle handle = NULL;
    LED_Attrs* pAttrs = NULL;
    int32_t    status;
    int32_t    ledCnt;

    handle = gLedHandle[INDUSTRIAL_LEDS_INSTANCE];

    if (NULL == handle)
    {
        return;
    }

    pAttrs = (LED_Attrs*) LED_getAttrs(INDUSTRIAL_LEDS_INSTANCE);

    if( ledValue_s == value_p)
    {
        return;
    }

    ledValue_s = value_p;

    for(ledCnt = 0U; ledCnt < pAttrs->numLedPerGroup; ledCnt++)
    {
        if(value_p & 1)
        {
            status = LED_on(handle, ledCnt);
        }
        else
        {
            status = LED_off(handle, ledCnt);
        }

        if(SystemP_SUCCESS != status)
        {
            return;
        }

        value_p >>= 1;
    }
}
