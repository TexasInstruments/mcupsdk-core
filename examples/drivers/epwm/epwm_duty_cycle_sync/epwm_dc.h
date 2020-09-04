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

#ifndef _EPWM_DC_H_
#define _EPWM_DC_H_

#include <stdint.h>
#include <drivers/hw_include/hw_types.h>
#include <drivers/epwm.h>

/* Status return values */
#define EPWM_DC_SOK         ( 0 )   /* no error */
#define EPWM_DC_INV_PRMS    ( -1 )  /* error, invalid parameters */

/* EPWM IDs */
#define EPWM_ID_0           ( 0 )
#define EPWM_ID_1           ( 1 )
#define EPWM_ID_2           ( 2 )

/* EPWM configuration parameters */
typedef struct _EPwmCfgPrms_t
{
    uint32_t    epwmId;             /* EPWM ID */
    uint32_t    epwmBaseAddr;       /* EPWM base address */
    uint32_t    epwmFclkFreq;       /* EPWM functional clock frequency */
    uint32_t    hspClkDiv;          /* EPWM High-Speed Time-base Clock Prescale Bits */
    uint32_t    clkDiv;             /* EPWM Time-base Clock Prescale Bits */
    uint32_t    epwmOutFreq;        /* EPWM output frequency */
    float       epwmDutyCycle;      /* EPWM init duty cycle, 0.0-100.0*/
    uint32_t    epwmTbCounterDir;   /* EPWM counter direction (Up, Down, Up/Down) */
    
    /* TB sync in config */
    Bool        cfgTbSyncIn;        /* config TB sync in flag (true/false) */
    float 	    tbPhsValue;         /* cfgTbSyncIn==TRUE: timer phase value to load on Sync In event, 0.0-100.0*/
    uint32_t 	tbSyncInCounterDir; /* cfgTbSyncIn==TRUE: counter direction on Sync In event */
    
    /* TB sync out config */
    Bool        cfgTbSyncOut;       /* config TB sync output flag (true/false) */
    uint32_t 	tbSyncOutMode;      /* cfgTbSyncOut==TRUE: Sync Out mode */
    
    /* AQ config */
    EPWM_AqActionCfg aqCfg;
    
    /* DB config */
    Bool        cfgDb;              /* config DB flag (true/false) */
    EPWM_DeadbandCfg dbCfg;         /* Deadband config */
    
    /* ET config */
    Bool        cfgEt;              /* config ET module */
    uint32_t    intSel;             /* ET interrupt select */
    uint32_t    intPrd;             /* ET interrupt period */
} EPwmCfgPrms_t;

/* EPWM object */
typedef struct _EPwmObj_t
{
    EPwmCfgPrms_t epwmCfgPrms;      /* EPWM configuration parameters */
    uint32_t    epwmPrdVal;         /* EPWM period value */
} EPwmObj_t;

/* EPWM Handle */
typedef EPwmObj_t * Epwm_Handle;

/* Initialize EPWM */
Epwm_Handle epwmInit(
    EPwmCfgPrms_t *pEpwmCfgPrms, 
    EPwmObj_t *pEpwmObj
);

/* Update EPWM outputs */
int32_t epwmUpdateOut(
    Epwm_Handle hEpwm,
    float amp
);

#endif /* _EPWM_DC_H_ */
