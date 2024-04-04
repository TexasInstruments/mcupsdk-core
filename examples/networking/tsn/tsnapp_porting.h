/*
 *  Copyright (c) Texas Instruments Incorporated 2023
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

#ifndef __TSNAPP_PORTING_H__
#define __TSNAPP_PORTING_H__
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/DebugP.h>
#include <core/enet_types.h>
#include <core/enet_mod_tas.h>
#include <core/enet_types.h>
#include <enet_apputils.h>
#include <enet_ethutils.h>

#ifndef DISABLE_FAT_FS

#include <ff_stdio.h>

#define INTERFACE_CONFFILE_PATH "/sd0/conffiles/interface.conf"
#define UNICONF_DBFILE_PATH     "/sd0/uniconfdb/example.bin"

typedef FF_Stat_t       EnetApp_fsInfo_t;
#define FSTAT(fn, st)   ff_stat((fn), (st))
#define FSSTAT_OK       (0)
#define NETCONF_YANG_SCHEMA_DIR "/sd0/schemas/xmlsafe"

#endif //DISABLE_FAT_FS

#define EnetApp_sleep           ClockP_usleep
#define EnetApp_yield           TaskP_yield
#define ENDLINE "\r\n"
#define USE_CRLF

static inline char EnetTsnApp_getChar(void)
{
    char ch;

    DebugP_scanf("%c", &ch);

    return ch;
}

static inline int32_t EnetTsnApp_getNum(void)
{
    int32_t num;

    DebugP_scanf("%d", &num);

    return num;
}

#endif
