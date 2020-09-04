/*!
 *  \example appPhyReset.c
 *
 *  \brief
 *  EtherNet/IP&trade; Adapter Example, application PHY reset.
 *
 *  \author
 *  KUNBUS GmbH
 *
 *  \date
 *  2021-07-23
 *
 *  \copyright
 *  Copyright (c) 2021, KUNBUS GmbH<br /><br />
 *  All rights reserved.<br />
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:<br />
 *  <ol>
 *  <li>Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.</li>
 *  <li>Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.</li>
 *  <li>Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.</li>
 *  </ol>
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
 
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#ifdef __LINUX_USER_SPACE
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <time.h>
#include <pthread.h>
#include <errno.h>
#endif

#include <appPhyReset.h>

#ifdef __LINUX_USER_SPACE
    static uint32_t setGpio (const char* cpGpioName_p, uint8_t ucValue_p);
#endif


//*************************************************************************************************
//| Function: 
//|
//! \brief
//!
//! \detailed
//!
//!
//!
//! \ingroup
//-------------------------------------------------------------------------------------------------
uint32_t APP_initPhyResetGpio (
    void)

{
#ifdef __LINUX_USER_SPACE
    int iFd_l;
    int iRv_l;
    uint32_t i32uErr_l;
#define MAX_TEMP_BUF    5
    char acBuffer_l[MAX_TEMP_BUF];


    iFd_l = open ("/sys/class/gpio/export", O_WRONLY | O_SYNC);
    if (iFd_l == -1)
    {
        OSAL_printf ("\nUnable to open: /sys/class/gpio/export");
        i32uErr_l = 1;
        goto laError;
    }


    acBuffer_l[0] = '6';
    acBuffer_l[1] = '9';
    iRv_l = write (iFd_l, &acBuffer_l, 2);
    // There may be an error, if GPIO is already exported. We will see it if try to change direction

    close (iFd_l);

    iFd_l = open ("/sys/class/gpio/gpio69/direction", O_WRONLY | O_SYNC);
    if (iFd_l == -1)
    {
        OSAL_printf ("\nUnable to open: /sys/class/gpio/gpio69/direction");
        i32uErr_l = 2;
        goto laError;
    }


    acBuffer_l[0] = 'o';
    acBuffer_l[1] = 'u';
    acBuffer_l[2] = 't';
    iRv_l = write (iFd_l, &acBuffer_l, 3);
    if (iRv_l != 3)
    {
        OSAL_printf ("\nError in setting GPIO direction");
        i32uErr_l = 3;
        goto laError;
    }

    close (iFd_l);

    return (0);

    //-------------------------------------------------------------------------------------------------
laError:

    return (i32uErr_l);
#else

    return (0);
#endif
}

//*************************************************************************************************
//| Function: 
//|
//! \brief
//!
//! \detailed
//!
//!
//!
//! \ingroup
//-------------------------------------------------------------------------------------------------
#ifdef __LINUX_USER_SPACE
uint32_t setGpio (
    const char *cpGpioName_p, 
    uint8_t ucValue_p)

{
    int iFd_l = -1;
    int iRv_l;
    uint32_t i32uErr_l;


    iFd_l = open (cpGpioName_p, O_WRONLY | O_SYNC);
    if (iFd_l == -1)
    {
        OSAL_printf ("\nUnable to open: %s", cpGpioName_p);
        i32uErr_l = 1;
        goto laError;
    }

    iRv_l = write (iFd_l, &ucValue_p, 1);
    if (iRv_l != 1)
    {
        OSAL_printf ("\nUnable to write to: %s", cpGpioName_p);
        i32uErr_l = 2;
        goto laError;
    }

    close (iFd_l);

    return (0);

//-------------------------------------------------------------------------------------------------
laError:

    if (iFd_l != -1)
    {
        close (iFd_l);
    }

    return (i32uErr_l);
}
#endif

//*************************************************************************************************
//| Function: 
//|
//! \brief
//!
//! \detailed
//!
//!
//!
//! \ingroup
//-------------------------------------------------------------------------------------------------
void APP_pruResetPhy (
    void* ctxt,
    uint8_t idx,
    uint8_t bReset)

{
#ifdef __LINUX_USER_SPACE
    if (bReset)
    {
        setGpio ("/sys/class/gpio/gpio69/value", '0');
    }
    else
    {
        setGpio ("/sys/class/gpio/gpio69/value", '1');
    }
#endif
}

//*************************************************************************************************
