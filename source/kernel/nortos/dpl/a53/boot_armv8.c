/*
 *  Copyright (C) 2018-2021 Texas Instruments Incorporated
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

#include <stdlib.h>
#include <errno.h>
#include "common_armv8.h"

#ifdef SMP_FREERTOS
#include <kernel/dpl/CacheP.h>
#endif

/* below are set in linker command file */
extern unsigned int __bss_start__, __bss_end__;
extern char __heap_start__, __heap_end__;
extern unsigned int __data_load__, __data_start__, __data_end__;

extern int main();
extern void __mmu_init(void);

#ifdef SMP_FREERTOS
volatile int32_t mmuInitDone = -1;
volatile int32_t bssInitDone = -1;
#endif

int __system_start(void)
{
    volatile unsigned int * bs;
    volatile unsigned int * be;
    unsigned int * dl;
    unsigned int * ds;
    unsigned int * de;

#if defined (SMP_FREERTOS)

    /* Initialization of bss section is done only once (From Core0) */
    if (0 == Armv8_getCoreId())
    {
        /* initialize .bss to zero */
        bs = & __bss_start__;
        be = & __bss_end__;
        while (bs < be)
        {
            *bs = 0;
            bs++;
        }

        /* relocate the .data section */
        dl = & __data_load__;
        ds = & __data_start__;
        de = & __data_end__;
        if (dl != ds)
        {
            while (ds < de)
            {
                *ds = *dl;
                dl++;
                ds++;
            }
        }

        /* Set flag to indicate bss initialization is done by Core0 */
        bssInitDone = 1;
    }
    else
    {
        /* Core1 should wait until bss initialization is done by Core0 */
        while(bssInitDone != 1)
        {
            ;
        }
    }

#else
    /* initialize .bss to zero */
    bs = & __bss_start__;
    be = & __bss_end__;
    while (bs < be)
    {
        *bs = 0;
        bs++;
    }

    /* relocate the .data section */
    dl = & __data_load__;
    ds = & __data_start__;
    de = & __data_end__;
    if (dl != ds)
    {
        while (ds < de)
        {
            *ds = *dl;
            dl++;
            ds++;
        }
    }
#endif

#if defined (SMP_FREERTOS)

    /* Wait for MMU init to be done by Core0 when running SMP FreeRTOS */
    /* This is done to synchronise between Core0 and Core1 so that MMU initialization is done by Core 0 */
    if (1 == Armv8_getCoreId())
    {
        while(mmuInitDone != 1)
        {
            ;
        }
    }

#endif

    /* initialize mmu and cache */
    __mmu_init();

#if defined (SMP_FREERTOS)

    /* Set flag to indicate the MMU initialization completion by Core0 */
    if (0 == Armv8_getCoreId())
    {
        mmuInitDone = 1;
        CacheP_wb((void *)&mmuInitDone, sizeof(mmuInitDone), CacheP_TYPE_ALL);
    }

#endif

    main();

    Armv8_exit();

    return(0);
}

/* override _sbrk_r with ,custom implementation and use __heap_start and __heap_end as marker to
   define default malloc heap limit
 */
void *_mysbrk (int  incr)
{
	static char *heap_end;		/* Previous end of heap or 0 if none */
	char        *prev_heap_end;

	if (0 == heap_end) {
		heap_end = &__heap_start__;			/* Initialize first time round */
	}

	prev_heap_end  = heap_end;
	heap_end      += incr;
	if( heap_end < (&__heap_end__))
    {

	}
    else
    {
		errno = ENOMEM;
		return (char*)-1;
	}
	return (void *) prev_heap_end;
}

void *_sbrk_r(struct _reent *ptr, ptrdiff_t incr)
{
    char *ret;
    errno = 0;
    if ( (ret = (char *)(_mysbrk (incr))) == (void *) -1 && errno != 0)
    {
        ptr->_errno = errno;
    }
    return ret;
}
