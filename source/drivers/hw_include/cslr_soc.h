/*
 *  Copyright (C) 2020-2024 Texas Instruments Incorporated
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
 *
 */
#ifndef CSLR_SOC_TOP_H_
#define CSLR_SOC_TOP_H_

#include <stdint.h>


#ifdef __cplusplus
extern "C"
{
#endif

#if defined (SOC_AM64X) || defined (SOC_AM243X)
#include <drivers/hw_include/am64x_am243x/cslr_soc.h>
#endif

#if defined (SOC_AM263X)
#include <drivers/hw_include/am263x/cslr_soc.h>
#endif
#if defined (SOC_AM263PX)
#include <drivers/hw_include/am263px/cslr_soc.h>
#endif

#if defined (SOC_AM261X)
#include <drivers/hw_include/am261x/cslr_soc.h>
#endif

#if defined (SOC_AM273X)
#include <drivers/hw_include/am273x/cslr_soc.h>
#endif

#if defined (SOC_AWR294X)
#include <drivers/hw_include/awr294x/cslr_soc.h>
#endif

#if defined (SOC_AM62X)
#include <drivers/hw_include/am62x/cslr_soc.h>
#endif

#if defined (SOC_AM65X)
#include <drivers/hw_include/am65x/cslr_soc.h>
#endif

#ifdef __cplusplus
}
#endif

#endif
