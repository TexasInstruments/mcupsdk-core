/*
 * Copyright (c) 2001-2003 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 */
#ifndef LWIP_ARCH_CC_H
#define LWIP_ARCH_CC_H

#ifdef __cplusplus
extern "C"
{
#endif


/* std.h functions required */
#include <stdint.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/nortos/dpl/common/printf.h>

/* Disable lwIP's private definition of 'struct timeval' */
#define LWIP_TIMEVAL_PRIVATE 0
#include <sys/select.h>

/* Define byte order of the system */
#ifndef BYTE_ORDER
#define BYTE_ORDER LITTLE_ENDIAN
#endif

/* Use lwip provided errors as ti compiler is too granular*/
#define LWIP_PROVIDE_ERRNO  1

#define LWIP_RAND() ((u32_t)rand())

/* Setup Packing Macros */
#define PACK_STRUCT_BEGIN
#define PACK_STRUCT_STRUCT __attribute__ ((__packed__))
#define PACK_STRUCT_END
#define PACK_STRUCT_FIELD(x) x

/* Different handling for unit test, normally not needed */
#ifdef LWIP_NOASSERT_ON_ERROR
#define LWIP_ERROR(message, expression, handler) do { if (!(expression)) { \
  handler;}} while(0)
#endif

/** Platform specific assertion handling.\n
 * Note the default implementation pulls in printf, fflush and abort, which may
 * in turn pull in a lot of standard libary code. In resource-constrained
 * systems, this should be defined to something less resource-consuming.
 */
#ifdef LWIP_PLATFORM_ASSERT
#undef LWIP_PLATFORM_ASSERT
#endif
#ifndef LWIP_PLATFORM_ASSERT
#define LWIP_PLATFORM_ASSERT(x) do { \
			  DebugP_log(x);  \
              DebugP_assert(0); } while(0)
#include <stdio.h>
#include <stdlib.h>
#endif

/** Platform specific diagnostic output.\n
 * Note the default implementation pulls in printf, which may
 * in turn pull in a lot of standard libary code. In resource-constrained
 * systems, this should be defined to something less resource-consuming.
 */
#ifdef LWIP_PLATFORM_DIAG
#undef LWIP_PLATFORM_DIAG
#endif
#ifndef LWIP_PLATFORM_DIAG
#define LWIP_PLATFORM_DIAG(x) do { \
			  DebugP_log x; } while(0)
#include <stdio.h>
#include <stdlib.h>
#endif

#ifdef printf
#undef printf
#endif
#define printf printf_

struct sio_status_s;
typedef struct sio_status_s sio_status_t;
#define sio_fd_t sio_status_t*
#define __sio_fd_t_defined

typedef uint32_t sys_prot_t;

#define strnlen strnlen_
size_t strnlen_ (register const char *s, size_t maxlen);
#ifdef __cplusplus
}
#endif

#endif /* LWIP_ARCH_CC_H */
