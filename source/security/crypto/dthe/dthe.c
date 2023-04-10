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

/**
 *  \file   dthe.c
 *
 *  \brief  This file contains the implementation of Dthe driver
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <security/crypto/dthe/dthe.h>
#include <drivers/hw_include/cslr.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void DTHE_init(void)
{
    return ;
}

void DTHE_deinit(void)
{
    return ;
}

DTHE_Handle DTHE_open(uint32_t index)
{
    DTHE_Return_t status = DTHE_RETURN_SUCCESS;
    DTHE_Handle     handle  = NULL;
    DTHE_Config     *config = NULL;
    DTHE_Attrs      *attrs  = NULL;
    /* Check instance */
    if(index >= gDtheConfigNum)
    {
        status = DTHE_RETURN_FAILURE;
    }
    else
    {
        config = &gDtheConfig[index];
        DebugP_assert(NULL != config->attrs);
        attrs = config->attrs;
        if(TRUE == attrs->isOpen)
        {
            /* Handle is already opened */
            status = DTHE_RETURN_FAILURE;
            attrs->faultStatus = status;
        }
    }

    if(DTHE_RETURN_SUCCESS == status)
    {
        attrs->isOpen = TRUE;
        handle = (DTHE_Handle) config;
        attrs->faultStatus = status;
    }
    else
    {
        /* status ireturned here will be failure as the handle is not
         * instantiated when the DTHE is already open (attrs->isOpen
         * is set to TRUE)
         */
        status = DTHE_close(handle);
    }
    return (handle);
}

DTHE_Return_t DTHE_close(DTHE_Handle handle)
{
    DTHE_Return_t  status       = DTHE_RETURN_FAILURE;
    DTHE_Config         *config = NULL;
    DTHE_Attrs          *attrs  = NULL;
    if(handle != NULL)
    {
        config = (DTHE_Config *) handle;
		if(config->attrs->isOpen != (uint32_t)FALSE)
		{
			attrs = config->attrs;
			DebugP_assert(NULL != attrs);
			attrs->isOpen = FALSE;
			/* To disable module */
			handle = NULL;
			status  = DTHE_RETURN_SUCCESS;
		}
	}

    return (status);
}

