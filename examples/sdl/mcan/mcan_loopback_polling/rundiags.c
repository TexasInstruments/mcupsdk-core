/*
 *  Copyright (C) 2023 Texas Instruments Incorporated
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
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include "sdlexample.h"


/*
** bool runningDiags(void)
**
** Return true if running diagnostics and false if not.
*/
bool runningDiags(void)
{
  bool      diags;


  if (sdlstats.rundiagnostics)
  {
    diags = true;
  }
  else
  {
    diags = false;
  }

  return diags;

}


/*
** void performDiags(void *args)
**
** This function performs the runtime diagnostics.
*/
int32_t performDiags(void *args)
{
  int32_t   sdlResult;
  uint32_t  delay;

  /* check RTI */
  for (delay=0; delay < 20000; delay++);
  DebugP_log("    RTI UC1...... ");
  sdlstats.diagrunning = DIAGRUNNING_RTI;
  sdlResult =  RTI_Test();
  if (sdlResult != SDL_PASS)
  {
    DebugP_log("FAILED. \r\n");
    return sdlResult;
  }
  DebugP_log("PASSED. \r\n");

 /* MCRC_AUTOMODE check*/
  for (delay=0; delay < 20000; delay++);
  DebugP_log("    MCRC AUTO mode test... ");
  sdlstats.diagrunning = DIAGRUNNING_MCRC;
  sdlResult = MCRCAuto_test();
  if (sdlResult != SDL_PASS)
  {
    DebugP_log("FAILED. \r\n");
    return sdlResult;
  }
  DebugP_log("PASSED. \r\n");

  /* check ECC for MCAN  */
  DebugP_log("    ECC MCAN0.... ");
  sdlstats.diagrunning = DIAGRUNNING_ECCMCAN0;
  sdlResult = ECC_test();
  if (sdlResult != SDL_PASS)
  {
    DebugP_log("FAILED. \r\n");
    return sdlResult;
  }
  DebugP_log("PASSED. \r\n");
  for (delay=0; delay < 20000; delay++);

  /* check DCC */
  DebugP_log("    DCC UC1...... ");
  sdlstats.diagrunning = DIAGRUNNING_DCC;
  sdlResult = DCC_test();
  if (sdlResult != SDL_PASS)
  {
    DebugP_log("FAILED. \r\n");
    return sdlResult;
  }
  DebugP_log("PASSED. \r\n");
  for (delay=0; delay < 20000; delay++);

  /* check TCM Parity */
  DebugP_log("    TCM Parity... ");
  sdlstats.diagrunning = DIAGRUNNING_TCMPARITY;
  sdlResult = ParityTCM_test();
  if (sdlResult != SDL_PASS)
  {
    DebugP_log("FAILED. \r\n");
    return sdlResult;
  }
  DebugP_log("PASSED. \r\n");

  for (delay=0; delay < 20000; delay++);

  DebugP_log("    Bus Safety... ");
  sdlstats.diagrunning = DIAGRUNNING_ECCBUS;
  sdlResult =  ecc_bus_safety_testExecute();
  if (sdlResult != SDL_PASS)
  {
    DebugP_log("FAILED. \r\n");
    return sdlResult;
  }
  DebugP_log("PASSED. \r\n");

  for (delay=0; delay < 20000; delay++);

  DebugP_log("    Lockstep mode test... ");
  sdlstats.diagrunning = DIAGRUNNING_CCM;
  sdlResult =  CCM_Test(NULL);
  if (sdlResult != SDL_PASS)
  {
    DebugP_log("FAILED. \r\n");
    return sdlResult;
  }
  DebugP_log("PASSED. \r\n");
  for (delay=0; delay < 20000; delay++);
  sdlstats.diagrunning = DIAGRUNNING_NONE;

  return sdlResult;

}
