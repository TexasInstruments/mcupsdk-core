/*
 *  Copyright (C) 2022-23 Texas Instruments Incorporated
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
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON AN2
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*==========================================================================
 *                             Include Files
 *==========================================================================*/
#include <stdlib.h>
#include <stdint.h>
#include <kernel/dpl/SemaphoreP.h>
#include <drivers/hw_include/cslr_soc.h>
#include <kernel/dpl/HwiP.h>
#include <drivers/hsmclient.h>
#include <string.h>
#include <kernel/dpl/DebugP.h>

/*==========================================================================
 *                           Macros
 *==========================================================================*/
/**
 * @brief
 *  This is the version number for the IPC Export Interface. The version number
 *  should be the same between the HSM Boot ROM and the R5 SBL. In the case of a
 *  mismatch the HSM Boot ROM will generate an error.
 */
#define HSMCLIENT_IPC_EXPORT_VERSION          (0x1U)

/*==========================================================================
 *                       Section-1 structure Declarations
 *==========================================================================*/
/**
 * @brief
 *       hsmclient IPC  Message Type for HSMRt load function.
 *
 * @details
 *  The enumeration describes the messages which are supported by the
 *  IPC Module executing in the HSM Runtime state.
 */
typedef enum Hsmclient_ipcExportMsgType_e
{
    /**
     * @brief   Load the HSM Runtime
     *      R5 SBL -> HSM
     *  Payload = ipcLoadHSM
     */
    Hsmclient_ipcExportMsgType_LOAD_HSM        = 0x9980A1D4U,
    /**
     * @brief   HSM Runtime Load Status
     *      HSM -> R5 SBL
     *  Payload = ipcLoadHSMResult
     */
    Hsmclient_ipcExportMsgType_LOAD_HSM_RESULT = 0xA70915DEU

}Hsmclient_ipcExportMsgType;

/**
 * @brief  IPC Load HSM Runtime
 *
 * @details
 *  This is the format of the message which is sent to load the
 *  HSM runtime. The R5 SBL will populate this message and will send
 *  to the HSM Boot ROM.
 */
typedef struct Hsmclient_ipcExportHeader_t
{
    /**
     * @brief   Version of the IPC Exported Interface.
     * \ref HSMCLIENT_IPC_EXPORT_VERSION
     */
    uint32_t            version;
    /**
     * @brief   IPC Message Type which is being sent.
     */
    Hsmclient_ipcExportMsgType   msgType;
    /**
     * @brief   This is the 16bit checksum which will need to be added
     * to ensure correctness of the data being passed. The checksum
     * is computed on the header + payload.
     */
    uint16_t            checksum;
}Hsmclient_ipcExportHeader;

/**
 * @brief
 *  IPC Load HSM Runtime
 *
 * @details
 *  This is the format of the message which is sent to load the
 *  HSM runtime. The R5 SBL will populate this message and will send
 *  to the HSM Boot ROM.
 */
typedef struct Hsmclient_ipcLoadHSM_t
{
    /**
     * @brief   This is header which is added to all the messages
     */
    Hsmclient_ipcExportHeader    header;
    /**
     * @brief   This is the load address where the HSM runtime image
     * is located. The image should always have the X509 certificate
     * followed by the actual HSM runtime binary.
     */
    uint32_t            imgLoadAddress;
}Hsmclient_ipcLoadHSM;

/**
 * @brief
 *  IPC Load HSM Status
 *
 * @details
 *  The enumeration describes the status of the HSM Runtime loading
 *  status. This is sent back to the R5 SBL
 */
typedef enum Hsmclient_ipcLoadHSMStatus_e
{
    /**
     * @brief   HSM Runtime was loaded successfully
     */
    Hsmclient_ipcLoadHSMStatus_SUCCESS = 0x4A43AB6CU,

    /**
     * @brief   HSM Runtime could not be loaded
     */
    Hsmclient_ipcLoadHSMStatus_FAILURE = 0x7021AE4BU
}Hsmclient_ipcLoadHSMStatus;

/**
 * @brief  IPC Load HSM Result
 *
 * @details
 *  This is the format of the message which is sent from the HSM Boot ROM
 *  to the R5 SBL to indicate the result of the loading request.
 */
typedef struct Hsmclient_ipcLoadHSMResult_t
{
    /**
     * @brief   This is header which is added to all the messages
     */
    Hsmclient_ipcExportHeader    header;
    /**
     * @brief   Status of the HSM Boot ROM Loading status
     */
    Hsmclient_ipcLoadHSMStatus    status;
}Hsmclient_ipcLoadHSMResult;

/*==========================================================================
 *                          Section-1 Global Variables
 *==========================================================================*/

/* variable foer hsm RT firmware download complete status */
volatile uint32_t gHsmRtDownloadComplete = 0;
CSL_mss_ctrlRegs * ptrMSSCtrlRegs = (CSL_mss_ctrlRegs *)CSL_MSS_CTRL_U_BASE;

/*==========================================================================
 *                          static Function Declarations
 *==========================================================================*/

/**
 *  @brief   This is a utility function which is used to compute the checksum
 *           on the provided buffer.
 *
 *  @param   ptrBuffer Buffer pointer.
 *
 *  @param   sizeMsg   message size in bytes.
 *
 *  @return  Computed checksum
 */
static uint16_t Hsmclient_computeIPCChecksum(uint8_t* ptrBuffer, uint32_t sizeMsg);
/**
 *  @brief   mailbox Rx ISR
 *
 *  @param   args loadHSMResult passing as arument
 *
 */
static void Hsmclient_mboxRxISR(void *args);

/*==========================================================================
 *                      static Function Definitions
 *==========================================================================*/

static uint16_t Hsmclient_computeIPCChecksum(uint8_t* ptrBuffer, uint32_t sizeMsg)
{
    uint8_t*    ptrData;
    uint32_t    index    = 0U;
    uint32_t    checksum = 0U;

    /* Checksum includes the header */
    ptrData = (uint8_t*)ptrBuffer;

    /* Cycle through the entire message */
    while (index < sizeMsg)
    {
        checksum = checksum + ptrData[index];
        index    = index + 1U;
    }
    checksum = (checksum & 0xFFFFU) + (checksum >> 16U);
    checksum = (checksum & 0xFFFFU) + (checksum >> 16U);
    checksum = ~checksum;

    return (uint16_t)checksum;

}

static void Hsmclient_mboxRxISR(void *args)
{
    Hsmclient_ipcLoadHSMResult *ploadHSMResult = (Hsmclient_ipcLoadHSMResult *) args;

    /* clear interrupt from the processor */
    CSL_FINSR (ptrMSSCtrlRegs->R5SS0_CORE0_MBOX_READ_REQ, 24, 24, 1U);

    /* Copy the HSM Result */
    memcpy ((void*)ploadHSMResult, (void*)CSL_MBOX_SRAM_U_BASE, sizeof(Hsmclient_ipcLoadHSMResult));

    /* clear Read done */
    CSL_FINSR (ptrMSSCtrlRegs->R5SS0_CORE0_MBOX_READ_DONE, 24, 24, 1U);

    /*HSMRT down load completed*/
    gHsmRtDownloadComplete = 1;

}

/*==============================================================================*
 *                          public Function defination.
 *==============================================================================*/

int32_t Hsmclient_loadHSMRtFirmware(HsmClient_t *NotifyClient, const uint8_t *pHSMRt_firmware)
{
    int32_t  status   = SystemP_SUCCESS;
    Hsmclient_ipcLoadHSM         loadHSMImage;
    Hsmclient_ipcLoadHSMResult   loadHSMResult = {{0}};
    uint16_t            orgChecksum;
    HwiP_Params hwiParams;
    HwiP_Object hwiObjReadReq;
    uint8_t *ptrMessage = (uint8_t *) CSL_HSM_MBOX_SRAM_U_BASE;

    if (pHSMRt_firmware != NULL)
    {
        /* clear any pending Interrupt */
        HwiP_clearInt(CSLR_R5FSS0_CORE0_INTR_MBOX_READ_REQ);

        /* register interrupt for Rx Mailbox */
        HwiP_Params_init(&hwiParams);
        hwiParams.intNum = CSLR_R5FSS0_CORE0_INTR_MBOX_READ_REQ;
        hwiParams.callback = Hsmclient_mboxRxISR;
        hwiParams.args = &loadHSMResult;
        hwiParams.isPulse = 0;

        status |= HwiP_construct(
            &hwiObjReadReq,
            &hwiParams);

        /* Populate the ipcExportMsgType_LOAD_HSM message header: */
        loadHSMImage.header.version  = HSMCLIENT_IPC_EXPORT_VERSION;
        loadHSMImage.header.msgType  = Hsmclient_ipcExportMsgType_LOAD_HSM;
        loadHSMImage.header.checksum = 0U;
        loadHSMImage.imgLoadAddress  = (uint32_t)((uint8_t *)pHSMRt_firmware);
        /* Compute the checksum: */
        loadHSMImage.header.checksum = Hsmclient_computeIPCChecksum ((uint8_t*)&loadHSMImage, sizeof(loadHSMImage));

        /* Copy the message: */
        memcpy ((void *)ptrMessage, (void *)&loadHSMImage, sizeof(Hsmclient_ipcLoadHSM));

        /* raise interrupt to the processor */
        CSL_FINSR (ptrMSSCtrlRegs->R5SS0_CORE0_MBOX_WRITE_DONE, 24, 24, 1U);

        /* Wait until hsmRt firmware download completes */
        while(gHsmRtDownloadComplete != 1)
        {
            ; /* wait until hsmRt download completes */
        }
        orgChecksum = loadHSMResult.header.checksum;
        loadHSMResult.header.checksum = 0U;
        /* Compute the checksum: */
        loadHSMResult.header.checksum = Hsmclient_computeIPCChecksum ((uint8_t*)&loadHSMResult, sizeof(loadHSMResult));
        /* Check for checksum match and firmware load status signature */
        if ((loadHSMResult.header.checksum != orgChecksum) || (loadHSMResult.status != Hsmclient_ipcLoadHSMStatus_SUCCESS))
        {
            /* Error: Invalid checksum */
            status = SystemP_FAILURE;
        }

        HwiP_destruct(&hwiObjReadReq);

        if(status == SystemP_SUCCESS)
        {
            /* once loaded hsmrt firmware wait for bootnotify message  */
            status = HsmClient_waitForBootNotify(NotifyClient,SystemP_WAIT_FOREVER);
        }
    }
    else
    {
        /* Error: Invalid load address */
        status = SystemP_FAILURE;
    }
    return status;
}
