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

#ifndef __firmware_version_h
#define __firmware_version_h 1

#ifdef __cplusplus
extern "C"
{
#endif


// ICSS_FIRMWARE_RELEASE_1:
// bit 31..16 reserved
// bit15..8	device number
#define		FIRMWARE_DEVICE_AM335x		0	// AM335x
#define		FIRMWARE_DEVICE_AM437x		1	// AM437x
#define		FIRMWARE_DEVICE_AM571x		2	// AM571x
#define		FIRMWARE_DEVICE_AM572x		3	// AM571x
#define		FIRMWARE_DEVICE_K2G			4	// K2G

// bit7..0	protocol type
#define		FIRMWARE_PROTOCOL_TYPE_PROFIBUS_SLAVE		0x00
#define		FIRMWARE_PROTOCOL_TYPE_ETHERCAT_SLAVE		0x01
#define		FIRMWARE_PROTOCOL_TYPE_PROFINET_DEVICE		0x02
#define		FIRMWARE_PROTOCOL_TYPE_SERCOS_SLAVE			0x03
#define		FIRMWARE_PROTOCOL_TYPE_OPENMAC_SLAVE		0x04
#define		FIRMWARE_PROTOCOL_TYPE_ETHERNET 			0x05
#define		FIRMWARE_PROTOCOL_TYPE_ENETIP_SLAVE			0x06
#define		FIRMWARE_PROTOCOL_TYPE_PROFINET_IRT_DEVICE	0x07

// ICSS_FIRMWARE_RELEASE_2:
// bit31		release or internal version
#define		FIRMWARE_VERSION_RELEASE	0
#define		FIRMWARE_VERSION_INTERNAL	1
// change next for official SDK release
#define 	FIRMWARE_RELEASE_TYPE   FIRMWARE_VERSION_INTERNAL
// bit30..24	major version number
#define		FIRMWARE_VERSION_MAJOR		0x00
// bit23..16		minor version number
#define		FIRMWARE_VERSION_MINOR			0x0F
// bit15..0		build number
#define		FIRMWARE_VERSION_BUILD			0x10



//ICSS_FIRMWARE_FEATURE_MASK:

#define FEATURE_MODE    0x01<<1   //MODE SWITCH
#define FEATURE_STORM_PREVENTION    0x01<<3 //ENABLED
#define FEATURE_REDUNDANCY    0x01<<4  //MRP
#define FEATURE_SYNCHRONIZATION    0x02<<8 //PTCP
#define FEATURE_NUMBER_OF_QUEUES    0x04<<12 //4 Queues
#define FEATURE_PROFINET    0x01<<16   //PROFINET SUPPORTED
#define FEATURE_DCP_FILTER  0x01<<17  //ENABLED


#define		ICSS_FIRMWARE_RELEASE_1	((FIRMWARE_DEVICE_AM572x << 8) | (FIRMWARE_DEVICE_AM571x << 8)|(FIRMWARE_DEVICE_K2G << 8)| (FIRMWARE_PROTOCOL_TYPE_PROFINET_IRT_DEVICE << 0))
#define		ICSS_FIRMWARE_RELEASE_2 ((FIRMWARE_RELEASE_TYPE << 31) | (FIRMWARE_VERSION_MAJOR << 24) | (FIRMWARE_VERSION_MINOR << 16) | (FIRMWARE_VERSION_BUILD << 0))
#define     ICSS_FIRMWARE_FEATURE_MASK (FEATURE_MODE| FEATURE_STORM_PREVENTION| FEATURE_REDUNDANCY|FEATURE_SYNCHRONIZATION|FEATURE_NUMBER_OF_QUEUES|FEATURE_PROFINET|FEATURE_DCP_FILTER)
#define     ICSS_FIRMWARE_RESERVED      0x00000000

#ifdef __cplusplus
}
#endif

#endif
