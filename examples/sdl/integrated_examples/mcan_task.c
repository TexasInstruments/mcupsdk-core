/*
 *  Copyright (C) 2023-2024 Texas Instruments Incorporated
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
#include "ti_drivers_config.h"
#include "ti_board_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "example.h"
#include <dpl_interface.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>


/* Some stats for the MCAN we can collect here. Polling routine will fill in. */
struct s_mcanstats mcanstats;

/*
** void mcan_task(void *args)
**
** This is our MCAN task within the system and will do the MCAN Polling.
** It will send and receive messages using the MCAN in loopback mode.
**
** This task should not be started unless system checks pass.
*/

void mcan_task(void *args)
{
  /* clear stats for MCAN */
  mcanstats.polled   = 0;
  mcanstats.txcount  = 0;
  mcanstats.rxcount  = 0;
  mcanstats.txerr    = 0;
  mcanstats.rxerr    = 0;
  mcanstats.badid    = 0;
  mcanstats.baddata  = 0;

  while (FOREVER)
  {
    switch (exampleState)
    {
      case EXAMPLE_STATE_FREERUN:
            /* generate an initial print of the stats for visual effect */
            if (mcanstats.polled == 0)
            {
              DebugP_log("[MAIN] Polled: 0 TxBytes: 0  RxBytes:  0  TxErr: 0  RxErr: 0  BadID: 0 MismatchData: 0\r\n");
            }
            /* we can poll in this state */
            mcanstats.polled++;
            mcan_loopback_polling_main(NULL);
            /* for every 1000 calls, lets print some stats */
            if ((mcanstats.polled%1000) == 0)
            {
              DebugP_log("[MCAN] Polled: %d TxBytes: %d  RxBytes:%d  TxErr: %d  RxErr: %d  BadID: %d  MismatchData: %d\r\n",
                          mcanstats.polled, mcanstats.txcount,  mcanstats.rxcount, mcanstats.txerr, mcanstats.rxerr, mcanstats.badid, mcanstats.baddata);
            }
            break;

        default:
            /* system not ready - Yield */
            vTaskDelay(10);
            break;
    }
  }

}
