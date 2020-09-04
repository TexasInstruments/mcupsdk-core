/*
 *  Copyright (c) 2021, KUNBUS GmbH
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
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

#ifndef INC_PROT__IOL_DEFINITION_H__
#define INC_PROT__IOL_DEFINITION_H__

///@cond (!USER_DOC)

#ifndef IOL_ONLY_TYPE_INCLUDE
#include "IOL_Port_Definition.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
\addtogroup group_iol_definition IO-Link Definitions
\{

\brief IO-Link definitions for Master and Device.
*/

/** \brief Revision ID Code for specification version 1.0 */
#define IOL_REVISION_1_0   0x10   
/** \brief Revision ID Code for specification version 1.1 */
#define IOL_REVISION_1_1   0x11 

/** \brief Number of bits per byte in IOL context */
#define IOL_T_TBYTE_BIT                 11

/** \brief Convert bits to microseconds using baud rate */
#define IOL_BIT_USEC(nbit, baud)    ((1000*1000*8/baud+1)*(nbit)/8)
#define IOL_BAUD_COM1       4800
#define IOL_BAUD_COM2       38400
#define IOL_BAUD_COM3       230400


/** \brief Seed value for checksums */
#define IOL_CHECKSUM_SEED  0x52  
/** \brief Status bit in CKS byte */
#define IOL_CKS_PDSTATUS_FLAG   0x40  
/** \brief Event bit in CKS byte */
#define IOL_CKS_EVENT_FLAG      0x80

/** \brief Event status mask */
#define IOL_EVENT_STATUS_MASK   0x3F


/** \brief Flow control for ISDU start */
#define IOL_ISDU_FLOW_START     0x10 
/** \brief Flow control for ISDU idle 1 */
#define IOL_ISDU_FLOW_IDLE1     0x11  
/** \brief Flow control for ISDU idle 2 */
#define IOL_ISDU_FLOW_IDLE2     0x12
/** \brief Flow control for ISDU abort */
#define IOL_ISDU_FLOW_ABORT     0x1F 


/** \brief Used for TMSeq calculation (see blow) */
#define IOL_MSEQ_BITS_MASTER(bytes)    ((bytes)*(IOL_T_TBYTE_BIT + IOL_T_T1_BIT) - IOL_T_T1_BIT)
/** \brief Used for TMSeq calculation (see blow) */
#define IOL_MSEQ_BITS_DEVICE(bytes)    ((bytes)*(IOL_T_TBYTE_BIT + IOL_T_T2_BIT) - IOL_T_T2_BIT)

/** \brief Calculate TMSeq in bits. See A.3.6 of \ref section_iol_spec_1_1 */
#define IOL_MSEQ_BITS(txbytes, rxbytes) (IOL_MSEQ_BITS_MASTER(txbytes) + IOL_T_TA_BIT \
                                        + IOL_MSEQ_BITS_DEVICE(rxbytes))

/** \brief Number of bits which the Device can answer before maximum M-Sequence time */
#define IOL_STARTUP_JITTER      (IOL_T_T1_BIT+IOL_T_T2_BIT+IOL_T_TA_BIT)


/** \brief ISDU timeout in milliseconds */
#define IOL_ISDU_TIMEOUT_MS     5500 

/** \brief Calculate PD length IOL Code for n bits. See B.1.6 of \ref section_iol_spec_1_1 */
#define IOL_PD_BITS(n)      ( ((n)<=16) ? (n) : (0x80 | (((n)+7)/8-1) ) )

/** \brief Calculate bits number from PD Length. see B.1.6 of \ref section_iol_spec_1_1 */
#define IOL_BITS_PD(n)      ( ((n) > 0x80) ? ((((n) & 0x1F) + 1) << 3) : ((n) & 0x1F))

/** \brief Calculate PD length IOL Codes for n bytes. See B.1.6 of \ref section_iol_spec_1_1 */
#define IOL_PD_BYTES(n)     ( ((n)>=3) ? (0x80 | (n-1)) : n*8)

/** \brief Calculate bytes number from PD Length. see B.1.6 of \ref section_iol_spec_1_1. */
#define IOL_BYTES_PD(n)     ( ((n) > 0x80) ? (((n) & 0x1F) + 1) : (((n) & 0x1F) == 0) ? 0 : (((((n) - 1) & 0x1F) >> 3) + 1))

/** \brief Calculate IOL cycle time (macro recommended for constant values). See B.1.3 of \ref section_iol_spec_1_1 */

#define IOLW_CYCLE_MS(n)    (( (n) < 5) ?      0x00 |   0 : /* Free Running */ \
                            ( (n) < 315 ) ?    0x40 | (int)((n+4)/5) : \
                                               0x40 | 0x3F) // 315 ms

#define IOL_CYCLE_MS(n)     (( (n) < 0.4) ?    0x00 |   4 : /* 400 us */ \
                            ( (n) < 6.4) ?     0x00 | (int)((n)/0.1) : \
                            ( (n) < 32 ) ?     0x40 | (int)((n-6.4)/0.4) : \
                            ( (n) < 132.8 ) ?  0x80 | (int)((n-32)/1.6) : \
                                               0x80 | 0x3F) // 132,8 ms

#define IOL_IMA_MS(n)       ((n) < (255*1.664)) ?   0x0100 | (INT8U)((n)/1.664) : \
                            ((n) < (255*5)) ?       0x0200 | (INT8U)((n)/5) : \
                            ((n) < (255*1000)) ?    0x0300 | (INT8U)((n)/1000) : \
                                                    0x0400 | (INT8U)((n)/60/1000)

/** \brief Macro to indicate that the Master uses the mincycletime  */
#define IOL_MINCYCLETIME    0

/** \brief Calculate IOL cycle time (macro recommended for constant values). See B.1.3 of \ref section_iol_spec_1_1 */
#define IOL_CYCLE_US(n)     IOL_CYCLE_MS(n / 1000)

/** \brief Return larger value of a,b */
#define IOL_LARGER(a,b)     ((a) > (b) ? a : b)

#define IOL_DS_INDEX                    3
#define IOL_DS_SUBINDEX_Command         1
#define IOL_DS_SUBINDEX_StateProperty   2
#define IOL_DS_SUBINDEX_StorageSize     3
#define IOL_DS_SUBINDEX_Checksum        4
#define IOL_DS_SUBINDEX_IndexList       5

#define IOL_DS_UPLOAD_FLAG              0x80
#define IOL_DS_STATE_INACTIVE           (0<<1)
#define IOL_DS_STATE_UPLOAD             (1<<1)
#define IOL_DS_STATE_DOWNLOAD           (2<<1)
#define IOL_DS_STATE_LOCKED             (3<<1)
#define IOL_DS_STATE_MASK               (3<<1)


/** Unique ID consists of Vendor/Device ID and 4 byte unique ID */
#define IOL_UNIQUEID_SIZE   9

/** \} */

///@endcond 

#ifdef __cplusplus
}
#endif


#endif
