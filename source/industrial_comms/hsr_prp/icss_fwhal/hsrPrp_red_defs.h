/**
 * \file hsrPrp_red_defs.h
 * \brief Contains standard definitions for Node types
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
 * \par
 */


#ifndef RED_DEFS_H_
#define RED_DEFS_H_

#ifdef __cplusplus
extern "C"
{
#endif


/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define IEC62439_CONST_REM_NODE_TYPE_DANP               (0)
#define IEC62439_CONST_REM_NODE_TYPE_REDBOXP            (1)
#define IEC62439_CONST_REM_NODE_TYPE_VDANP              (2)
#define IEC62439_CONST_REM_NODE_TYPE_DANH               (3)
#define IEC62439_CONST_REM_NODE_TYPE_REDBOXH            (4)
#define IEC62439_CONST_REM_NODE_TYPE_VDANH              (5)
#define IEC62439_CONST_REM_NODE_TYPE_SAN                (6)

#define IEC62439_CONST_DUPLICATE_ACCEPT                 (1)
#define IEC62439_CONST_DUPLICATE_DISCARD                (2)

#define IEC62439_CONST_TRANSPARENT_RECEPTION_REMOVE_RCT (1)
#define IEC62439_CONST_TRANSPARENT_RECEPTION_PASS_RCT   (2)


#ifdef __cplusplus
}
#endif

#endif /* RED_DEFS_H_ */
