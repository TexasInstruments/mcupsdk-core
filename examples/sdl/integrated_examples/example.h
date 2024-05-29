/*
 *   Copyright (c) 2024 Texas Instruments Incorporated
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

#ifndef EXAMPLE_H
#define EXAMPLE_H

#ifdef __cplusplus
extern "C"
{
#endif

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */

/* ========================================================================== */
/*                 External Function Declarations                             */
/* ========================================================================== */

/* Just a readable define running forever; e.g. while (FOREVER)       */
#define FOREVER 1

/* Create some states for the example. Nothing sophisticated          */
/* but will allow us to sequence or block or free run                 */
#define EXAMPLE_STATE_STARTUP       0u  /* system is starting up          */
#define EXAMPLE_STATE_STARTUP_DONE  1u  /* system passed startup checks   */
#define EXAMPLE_STATE_BLOCK         2u  /* Tasks must blocked             */
#define EXAMPLE_STATE_FREERUN       3u  /* Tasks are free to run          */
#define EXAMPLE_STATE_SHUTDOWN      4u  /* shut down the system           */
#define EXAMPLE_STATE_RUNDIAGS      5u  /* system is running diagnsotics  */

/* state example is operating in */
extern uint32_t exampleState;           /* created in main.c */

/* Holds mcan stats */
struct s_mcanstats{
  uint32_t   polled;  /* number times we called the polling routine */
  uint32_t   txcount; /* Bytes transmitted                          */
  uint32_t   rxcount; /* Bytes received                             */
  uint32_t   txerr;   /* Errors on transmit                         */
  uint32_t   rxerr;   /* Errors on receive                          */
  uint32_t   badid;   /* Comparison id not matching                 */
  uint32_t   baddata; /* Comparison data not matching               */
};
extern struct s_mcanstats  mcanstats;

extern void mcan_loopback_polling_main(void *args);
extern void McanConfigureDriver(void *args);

/* We want to send data for a few minutes. We will send this much (in bytes) and then stop. */
#define EXAMPLE_MAX_DATA_TO_SEND   (20000000u)

/* time base in ticks for running diagnostics - ~ every 2 minutes  as our ticks are 1ms */
#define  DIAG_TIMER_VALUE (2000u*60u)
/* Our tasks */
extern void main_task(void *args);
extern void mcan_task(void *args);

/* Task Priorities */
/* Task Priorities. Running MAIN at highest priority for all tasks as it checks SDL */
#define MCAN_TASK_PRI  (configMAX_PRIORITIES-3)
#define MAIN_TASK_PRI  (configMAX_PRIORITIES-2)

#ifdef __cplusplus
}
#endif

#endif /* EXAMPLE_H */

/* Nothing past this point */
