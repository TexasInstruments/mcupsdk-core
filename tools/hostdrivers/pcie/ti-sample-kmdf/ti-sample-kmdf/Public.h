/*
 *  Copyright (C) 2023-2024 Texas Instruments Incorporated
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

/*++

Module Name:

    public.h

Abstract:

    This module contains the common declarations shared by driver
    and user applications.

Environment:

    user and kernel

--*/

//
// Define an Interface Guid so that apps can find the device and talk to it.
//

DEFINE_GUID (GUID_DEVINTERFACE_tisamplekmdf,
    0xe2ff65d7,0x9f73,0x40c5,0xbd,0xac,0x53,0xb1,0xb7,0x2a,0x18,0x9f);
// {e2ff65d7-9f73-40c5-bdac-53b1b72a189f}

//
// Device type           -- in the "User Defined" range."
//
#define FILEIO_TYPE 40001

//
// The IOCTL function codes from 0x800 to 0xFFF are for customer use.
//
#define IOCTL_TISAMPLEKMDF_TEST_DMA \
    CTL_CODE( FILEIO_TYPE, 0x900, METHOD_BUFFERED, FILE_ANY_ACCESS  )
#define IOCTL_TISAMPLEKMDF_TEST_MSI \
    CTL_CODE( FILEIO_TYPE, 0x901, METHOD_BUFFERED, FILE_ANY_ACCESS  )
#define IOCTL_TISAMPLEKMDF_TEST_BARS \
    CTL_CODE( FILEIO_TYPE, 0x902, METHOD_BUFFERED, FILE_ANY_ACCESS  )

//
//  Structure to go with IOCTL_TISAMPLEKMDF_TEST_DMA.
//  The Data part is of variable length, determined by
//  the input buffer length passed to DeviceIoControl.
//
typedef struct _TISAMPLEKMDF_TEST_DMA
{
    ULONG pattern;
    ULONG result[16 * 1024];
} TISAMPLEKMDF_TEST_DMA, * PTISAMPLEKMDF_TEST_DMA;

//
//  Structure to go with IOCTL_TISAMPLEKMDF_TEST_MSI.
//
typedef struct _TISAMPLEKMDF_TEST_MSI
{
    ULONG result;
} TISAMPLEKMDF_TEST_MSI, * PTISAMPLEKMDF_TEST_MSI;

//
//  Structure to go with IOCTL_TISAMPLEKMDF_TEST_BARS.
//
typedef struct _TISAMPLEKMDF_TEST_BARS
{
    ULONG result;
} TISAMPLEKMDF_TEST_BARS, * PTISAMPLEKMDF_TEST_BARS;