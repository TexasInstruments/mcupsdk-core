/******************************************************************************
 *
 *
 * TEXAS INSTRUMENTS TEXT FILE LICENSE
 *
 *  Copyright (c) 2018-2019 Texas Instruments Incorporated
 *
 * All rights reserved not granted herein.
 * 
 * Limited License.  
 *
 * Texas Instruments Incorporated grants a world-wide, royalty-free, non-exclusive 
 * license under copyrights and patents it now or hereafter owns or controls to 
 * make, have made, use, import, offer to sell and sell ("Utilize") this software 
 * subject to the terms herein.  With respect to the foregoing patent license, 
 * such license is granted  solely to the extent that any such patent is necessary 
 * to Utilize the software alone.  The patent license shall not apply to any 
 * combinations which include this software, other than combinations with devices 
 * manufactured by or for TI ("TI Devices").  No hardware patent is licensed hereunder.
 *
 * Redistributions must preserve existing copyright notices and reproduce this license 
 * (including the above copyright notice and the disclaimer and (if applicable) source 
 * code license limitations below) in the documentation and/or other materials provided 
 * with the distribution.
 * 
 * Redistribution and use in binary form, without modification, are permitted provided 
 * that the following conditions are met:
 *	No reverse engineering, decompilation, or disassembly of this software is 
 *  permitted with respect to any software provided in binary form.
 *	Any redistribution and use are licensed by TI for use only with TI Devices.
 *	Nothing shall obligate TI to provide you with source code for the software 
 *  licensed and provided to you in object code.
 * 
 * If software source code is provided to you, modification and redistribution of the 
 * source code are permitted provided that the following conditions are met:
 *	Any redistribution and use of the source code, including any resulting derivative 
 *  works, are licensed by TI for use only with TI Devices.
 *	Any redistribution and use of any object code compiled from the source code
 *  and any resulting derivative works, are licensed by TI for use only with TI Devices.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of its suppliers 
 * may be used to endorse or promote products derived from this software without 
 * specific prior written permission.
 *
 * DISCLAIMER.
 *
 * THIS SOFTWARE IS PROVIDED BY TI AND TI’S LICENSORS "AS IS" AND ANY EXPRESS OR IMPLIED 
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY 
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL TI AND TI’S 
 * LICENSORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE 
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * file : icss_rx_int_pacing_mmap.h
 *
 * brief: This file contains VLAN/Multicast filtering feature memory map
 *
 */
#ifndef ICSS_RX_INT_PACING_MM_H
#define ICSS_RX_INT_PACING_MM_H

/* Interrupt pacing offsets */
/* 0x1FC wasted */
/*************************************************************************************/
/* 1 byte | 0 : Interrupt Pacing disabled | 1 : Interrupt Pacing enabled */
#define INTR_PAC_STATUS_OFFSET_PRU1             0x1FAE
/* 1 byte | 0 : Interrupt Pacing disabled | 1 : Interrupt Pacing enabled */
#define INTR_PAC_STATUS_OFFSET_PRU0             0x1FAF
/* Interrupt Pacing disabled, Adaptive logic disabled */
#define INTR_PAC_DIS_ADP_LGC_DIS                0x0
/* Interrupt Pacing enabled, Adaptive logic disabled */
#define INTR_PAC_ENA_ADP_LGC_DIS                0x1
/* Interrupt Pacing enabled, Adaptive logic enabled */
#define INTR_PAC_ENA_ADP_LGC_ENA                0x2

/* 4 bytes | previous TS from eCAP TSCNT for PRU 0 */
#define INTR_PAC_PREV_TS_OFFSET_PRU0            0x1FB0
/* 4 bytes | timer expiration value for PRU 0 */
#define INTR_PAC_TMR_EXP_OFFSET_PRU0            0x1FB4

/* 4 bytes | timer expiration value for PRU 1 */
#define INTR_PAC_PREV_TS_OFFSET_PRU1            0x1FB8
/* 4 bytes | previous TS from eCAP TSCNT for PRU 1 */
#define INTR_PAC_TMR_EXP_OFFSET_PRU1            0x1FBC
#define INTR_PAC_PREV_TS_RESET_VAL              0x0

/* eCAP offset in PRU-ICSS memory */
#define eCAP_BASE_OFFSET                        0x30000

#endif /* ICSS_RX_INT_PACING_MM_H */
