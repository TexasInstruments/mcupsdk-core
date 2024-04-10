/*
*  TEXAS INSTRUMENTS TEXT FILE LICENSE
* 
*   Copyright (c) 2017 Texas Instruments Incorporated
* 
*  All rights reserved not granted herein.
*  
*  Limited License.  
* 
*  Texas Instruments Incorporated grants a world-wide, royalty-free, non-exclusive 
*  license under copyrights and patents it now or hereafter owns or controls to 
*  make, have made, use, import, offer to sell and sell ("Utilize") this software 
*  subject to the terms herein.  With respect to the foregoing patent license, 
*  such license is granted  solely to the extent that any such patent is necessary 
*  to Utilize the software alone.  The patent license shall not apply to any 
*  combinations which include this software, other than combinations with devices 
*  manufactured by or for TI (“TI Devices”).  No hardware patent is licensed hereunder.
* 
*  Redistributions must preserve existing copyright notices and reproduce this license 
*  (including the above copyright notice and the disclaimer and (if applicable) source 
*  code license limitations below) in the documentation and/or other materials provided 
*  with the distribution.
*  
*  Redistribution and use in binary form, without modification, are permitted provided 
*  that the following conditions are met:
* 	No reverse engineering, decompilation, or disassembly of this software is 
*   permitted with respect to any software provided in binary form.
* 	Any redistribution and use are licensed by TI for use only with TI Devices.
* 	Nothing shall obligate TI to provide you with source code for the software 
*   licensed and provided to you in object code.
*  
*  If software source code is provided to you, modification and redistribution of the 
*  source code are permitted provided that the following conditions are met:
* 	Any redistribution and use of the source code, including any resulting derivative 
*   works, are licensed by TI for use only with TI Devices.
* 	Any redistribution and use of any object code compiled from the source code
*   and any resulting derivative works, are licensed by TI for use only with TI Devices.
* 
*  Neither the name of Texas Instruments Incorporated nor the names of its suppliers 
*  may be used to endorse or promote products derived from this software without 
*  specific prior written permission.
* 
*  DISCLAIMER.
* 
*  THIS SOFTWARE IS PROVIDED BY TI AND TI’S LICENSORS "AS IS" AND ANY EXPRESS OR IMPLIED 
*  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY 
*  AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL TI AND TI’S 
*  LICENSORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
*  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE 
*  GOODS OR SERVICES* LOSS OF USE, DATA, OR PROFITS* OR BUSINESS INTERRUPTION) HOWEVER 
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
*  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/ 

/*
* file:   icss_emacFwVersion.h
*
* brief:  Firmware versioning information 
*/

#ifndef ICSS_EMAC_FW_VERSION__H
#define ICSS_EMAC_FW_VERSION__H

#include <ti/drv/icss_emac/icss_emacDrv.h>

#ifdef __cplusplus
extern "C" {
#endif

/* FIRMWARE versioning information, must remain in sync with versioning of firmware release as specified in ti/drv/icss_emac/firmware/icss_dualemac/src/firmware_version.h */
#define FIRMWARE_DEVICE_ICSS_REV1    ((uint32_t)0U)
#define FIRMWARE_DEVICE_ICSS_REV2    ((uint32_t)1U)

#define FIRMWARE_PROTOCOL_TYPE_ETHERNET_SWITCH    ((uint32_t)0x12)
#define FIRMWARE_VERSION_INTERNAL     ((uint32_t)1U)
#define FIRMWARE_VERSION_RELEASE      ((uint32_t)0U)

#define FIRMWARE_VERSION_MAJOR        ((uint32_t)3U)
#define FIRMWARE_VERSION_MINOR        ((uint32_t)2U)
#define FIRMWARE_VERSION_BUILD        ((uint32_t)13U)


#if defined(icev2AM335x) || defined(idkAM437x) || defined(iceAMIC110)
#define ICSS_FIRMWARE_RELEASE_1     ((FIRMWARE_DEVICE_ICSS_REV1 << 8) | (FIRMWARE_PROTOCOL_TYPE_ETHERNET_SWITCH << 0))
#else
#define ICSS_FIRMWARE_RELEASE_1     ((FIRMWARE_DEVICE_ICSS_REV2 << 8) | (FIRMWARE_PROTOCOL_TYPE_ETHERNET_SWITCH << 0))

#endif

#define ICSS_FIRMWARE_RELEASE_2     ((FIRMWARE_VERSION_RELEASE << 31) | (FIRMWARE_VERSION_MAJOR << 24) | (FIRMWARE_VERSION_MINOR << 16) | (FIRMWARE_VERSION_BUILD << 0))


#ifdef __cplusplus
}
#endif

#endif  /* ICSS_EMAC_FW_VERSION__H */

