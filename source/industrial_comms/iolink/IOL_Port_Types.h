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

#ifndef INC_PROT__IOL_PORT_TYPES_H__
#define INC_PROT__IOL_PORT_TYPES_H__

/**
\addtogroup group_iol_port_types IO-Link port types
\{
*/

#include <stdint.h>


#ifndef NULL
#define NULL  ((void*)0)
#endif

#ifndef FALSE
#define FALSE ((TBOOL)0)
#endif

#ifndef TRUE
#define TRUE  ((TBOOL)1)
#endif

typedef char CHAR8; /**< \brief 8 bit character data type */

typedef uint8_t TBOOL; /**< \brief Boolean data type (at least 1 bit) */

typedef int8_t INT8S; /**< \brief 8 bit signed integer */

typedef int16_t INT16S; /**< \brief 16 bit signed integer */

typedef int32_t INT32S; /**< \brief 32 bit signed integer */

typedef uint8_t INT8U; /**< \brief 8 bit unsigned integer */

typedef uint16_t INT16U; /**< \brief 16 bit unsigned integer */

typedef uint32_t INT32U; /**< \brief 32 bit unsigned integer */

typedef float FLOAT32; /**< \brief 32 bit float */

typedef double FLOAT64; /**< \brief 64 bit float */

/** \} */

#endif
