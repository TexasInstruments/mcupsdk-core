/*
 *  Copyright (C) 2002-2020 Texas Instruments Incorporated
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

/**
 *   \file  csl_types.h
 *
 *   \brief  This file contains the Register Desciptions for CSL types
 */

#ifndef CSL_TYPES_H
#define CSL_TYPES_H

#include <stdint.h>


#ifdef __cplusplus
extern "C"
{
#endif


/**
 *  \anchor CSL_Type_t
 *  \name CSL Types
 *
 *  CSL data types.
 *
 *  @{
 */
#ifndef TRUE
#define TRUE		(1U)
#define FALSE		(0U)
#endif

/* Define CSL_TRUE/CSL_FALSE to go with uint32_t */
#ifndef CSL_TRUE
#define CSL_TRUE    (1U)
#endif
#ifndef CSL_FALSE
#define CSL_FALSE   (0U)
#endif

/* @} */


/** \brief CSL error type */
typedef int32_t CSL_ErrType_t;

/**
 *  \anchor CSL_ErrType_t
 *  \name CSL Error Types
 *
 *  CSL function return error codes.
 *
 *  @{
 */
#define CSL_PASS                        ( (int32_t) (0))
#define CSL_EFAIL                       (-(int32_t) (1))
#define CSL_EBADARGS                    (-(int32_t) (2))
#define CSL_EINVALID_PARAMS             (-(int32_t) (3))
#define CSL_ETIMEOUT                    (-(int32_t) (4))
#define CSL_EOUT_OF_RANGE               (-(int32_t) (5))
#define CSL_EUNSUPPORTED_CMD            (-(int32_t) (6))
#define CSL_EUNSUPPORTED_OPS            (-(int32_t) (7))
#define CSL_EALLOC                      (-(int32_t) (8))
/* @} */

/** \brief Define NULL if not defined */
#ifndef NULL
#define NULL            (0)
#endif

#ifndef NULL_PTR
#define NULL_PTR ((void *)0)
#endif


#ifdef __cplusplus
}
#endif

#endif /* CSL_TYPES_H */
