/*
 *  Copyright (C) 2022-23 Texas Instruments Incorporated
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
#include "ti_drivers_config.h"
#include "ti_board_config.h"
#include "ti_board_open_close.h"
#include "ti_drivers_open_close.h"
#include <drivers/spinlock.h>
#include <kernel/dpl/ClockP.h>
#include <drivers/hw_include/cslr.h>

#define lockNum1 0
#define lockNum2 1
#define DELAY_TIME 1

int main(void)
{
    System_init();
    Board_init();

    Drivers_open();
    Board_driversOpen();

    int loopcount = 5;

    while(loopcount--)
    {
        int32_t  status;

        /* Spin till lock is acquired */
        while(1U)
        {
            status = Spinlock_lock(CSL_SPINLOCK0_BASE, lockNum1);
            if(status == SPINLOCK_LOCK_STATUS_FREE)
            {
                break;  /* Free and taken */
            }
        }

        /*
         * enter critical section
         */

        DebugP_log("hello core 1 \r\n");
        ClockP_sleep(DELAY_TIME);
        Spinlock_unlock(CSL_SPINLOCK0_BASE, lockNum2);
    }

    Board_driversClose();
    Drivers_close();
    
    Board_deinit();
    System_deinit();
    return 0;
}
