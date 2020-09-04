/********************************************************************
 * Copyright (C) 2003-2019 Texas Instruments Incorporated.
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
*********************************************************************
* file: cslr_xge_cpsw_ss_s.h
*
* Brief: This file contains the Register Description for xge_cpsw_ss_s
*
*********************************************************************/
#ifndef CSLR_XGE_CPSW_SS_S_TOP_H
#define CSLR_XGE_CPSW_SS_S_TOP_H

#ifdef __cplusplus
extern "C"
{
#endif



#if defined(SOC_K2K)

#include <xge/V0/cslr_xge_cpsw_ss_s.h>

#elif defined(SOC_K2H)

#include <xge/V0/cslr_xge_cpsw_ss_s.h>

#elif defined(SOC_K2E)

#include <xge/V1/cslr_xge_cpsw_ss_s.h>

#elif defined(SOC_K2L)

#include <xge/V1/cslr_xge_cpsw_ss_s.h>

#elif defined(SOC_K2G)

#include <xge/V2/cslr_xge_cpsw_ss_s.h>

#elif defined(SOC_AM65XX) || defined(SOC_J721E) || defined (SOC_J74202) || defined (SOC_J721S2) || defined (SOC_J7200) || defined(SOC_AM64X) || defined(SOC_AM243X) || defined (SOC_AM62X)

#include <xge/V4/cslr_xge_cpsw_ss_s.h>

#elif defined(SOC_AM273X) || defined (SOC_AWR294X)

#include <xge/V5/cslr_xge_cpsw_ss_s.h>

#elif defined(SOC_AM263X)

#include <xge/V6/cslr_xge_cpsw_ss_s.h>

#endif /* SOC_XXXXX */


#ifdef __cplusplus
}
#endif

#endif /* CSLR_XGE_CPSW_H_ */
