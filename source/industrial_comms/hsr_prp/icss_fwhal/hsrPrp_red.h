/**
 * \file  hsrPrp_red.h
 *
 * \brief Include file for hsrPrp_red.c
 *
 * \par
 *  Copyright (C) 2021 Texas Instruments Incorporated
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
* \defgroup INDUSTRIAL_COMMS_HSR_PRP_FWHAL_MODULE APIs for HSR/PRP FWHAL
* \ingroup INDUSTRIAL_COMMS_MODULE
*  HSR/PRP FWHAL(Firmware and Hardware Abstraction Layer) APIs implement the key interface between HSR/PRP firmware.
*
*  @{
*/

/**
* \defgroup COMMON_API HSR/PRP Common Driver APIs
* \ingroup INDUSTRIAL_COMMS_HSR_PRP_FWHAL_MODULE
*/

/**
* \defgroup HSR_API HSR Driver APIs
* \ingroup INDUSTRIAL_COMMS_HSR_PRP_FWHAL_MODULE
*/

/**
* \defgroup PRP_API PRP Driver APIs
* \ingroup INDUSTRIAL_COMMS_HSR_PRP_FWHAL_MODULE
*/

#ifndef RED_H_
#define RED_H_

#ifdef __cplusplus
extern "C"
{
#endif


/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "hsrPrp_firmwareOffsets.h"
#include "hsrPrp_red_common.h"
#include "hsrPrp_handle.h"
#ifdef ICSS_PROTOCOL_PRP
#include "hsrPrp_red_prp.h"
#endif /* ICSS_PROTOCOL_PRP */
#ifdef ICSS_PROTOCOL_HSR
#include "hsrPrp_red_hsr.h"
#endif /* ICSS_PROTOCOL_HSR */

#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/TaskP.h>


/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define RED_MANUFACTURER  "Texas Instruments"
#define RED_VERSION_NAME  "1.0.4.0553"
#define RED_NODE_NAME     "TI"


#ifdef ICSS_PROTOCOL_PRP
#define RED_TAG_SIZE     PRP_RCT_SIZE
#define RED_LAN_A_MAGIC  PRP_LAN_A_MAGIC
#define RED_LAN_B_MAGIC  PRP_LAN_B_MAGIC
#endif /* ICSS_PROTOCOL_PRP */

#ifdef ICSS_PROTOCOL_HSR
#define RED_TAG_SIZE     HSR_TAG_SIZE
#define RED_LAN_A_MAGIC  HSR_LAN_A_MAGIC
#define RED_LAN_B_MAGIC  HSR_LAN_B_MAGIC
#endif /* ICSS_PROTOCOL_HSR */

//Note the count take in account CRC
#define RED_RX_FRAME_BYTE_CNT_MIN    ( 64 )  /**< RX Frame total byte count minimum value */
#define RED_RX_FRAME_BYTE_CNT_MAX  ( 1528 )  /**< RX Frame total byte count maximum value (VLAN + HSR TAG*/

/*VLAN*/
#define VLAN_TAG_TPID_BIG_ENDIAN      ( 0x0081 )  /**< PRP-1 RCT Suffix in Big Endian format*/
#define VLAN_TAG_SIZE                  ( 4 )

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
/** @addtogroup COMMON_API
 @{ */
/**
 *  \brief Assigns key function pointers based on macro
 *
 *
 */
void       RedProtocolInit(void);
/**
 *  \brief Sets Clear Node Table flag to be used by PRU
 *  \param  hsrPrphandle
 *
 */
void       RedNodeTableClear(hsrPrpHandle *hsrPrphandle);
/**
 *  \brief Increments statistics counter.
 *
 *  \param offset Counter
 *  \param pruicssHandle Pointer to PRU ICSS Handle, parent structure containing all switch information
 *
 */
void       RedIncrementCounter(uint32_t offset, PRUICSS_Handle      pruicssHandle);
/**
 *  \internal
 *  \brief Updates TLV1 of a Supervision frame
 *
 *  \param hsrPrphandle
 *  \param type TLV1.type set value
 *
 */
void       RedSupFrameUpdateTlv(hsrPrpHandle *hsrPrphandle, uint8_t type);
/**
 *  \brief Enqueues redundancy frames to transmit
 *
 *  \param icssEmachandle
 *  \param txArg
 *  \param userArg
 *
 *  \return RED_OK on success, RED_ERR otherwise
 *
 */
int32_t RedTxPacket(void *icssEmachandle, ICSS_EMAC_TxArgument *txArg, void *userArg);
/**
 *  \brief Copies data from L3 receive queues
 *
 *  \param icssEmachandle
 *  \param rxVoidArg
 *  \param userArg
 *
 *  \return RED_OK(0) on success, RED_ERR(1) otherwise
 *
 */
int32_t RedRxPktGet(void *icssEmachandle, ICSS_EMAC_RxArgument *rxVoidArg, void *userArg);
/**
 *  \brief Function to create RED Supervision task
 *  \param  hsrPrphandle
 *  \param  pruicssHandle Pointer to PRU ICSS Handle, parent structure containing all switch information
 *  \return RED_OK(0) if success, RED_ERR(1) otherwise
 */
RED_STATUS RedLifeCheckTaskCreate(hsrPrpHandle *hsrPrphandle,
                                  PRUICSS_Handle      pruicssHandle);
/**
 *  \brief Function to delete RED Supervision task
 *  \param  hsrPrphandle
 */
void       RedLifeCheckTaskDelete(hsrPrpHandle *hsrPrphandle);

/**
 *  \brief Function to start RedProtocolStart
 *  \param  hsrPrphandle
 *  \param  pruicssHandle Pointer to PRU ICSS Handle, parent structure containing all switch information
 */
RED_STATUS RedProtocolStart(hsrPrpHandle *hsrPrphandle,
                            PRUICSS_Handle      pruicssHandle);
/**
 *  \brief Configure IEP to modify max frame size (to acccount for LRE header)
 *
 *  \param  pruicssHandle Pointer to PRU ICSS Handle, parent structure containing all switch information
 *
 */
void   RedProtocolConfigure(PRUICSS_Handle      pruicssHandle);
/**
 *  \brief Delete Node Table check Task
 *
 *  \param  hsrPrphandle
 *
 *  \param  pruicssHandle Pointer to PRU ICSS Handle, parent structure containing all switch information
 *
 */
void       RedProtocolStop(hsrPrpHandle *hsrPrphandle,
                           PRUICSS_Handle      pruicssHandle);

/** @} */

/**
* @}
*/


#ifdef __cplusplus
}
#endif

#endif /* RED_H_ */
