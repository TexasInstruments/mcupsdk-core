
//! [include]
#include <kernel/dpl/CycleCounterP.h>
#include <kernel/dpl/DebugP.h>
//! [include]
 


void samples()
{
    {
//! [usage]
        uint32_t cycleCountBefore, cycleCountAfter, cpuCycles;

        /* enable and reset CPU cycle coutner */
        CycleCounterP_reset();

        cycleCountBefore = CycleCounterP_getCount32();

        /* call functions to profile */

        cycleCountAfter = CycleCounterP_getCount32();

        /* Check for overflow and wrap around. 
         *
         * This logic will only work for one overflow.
         * If multiple overflows happen during the profile period, 
         * then CPU cycles count will be wrong, 
         */
        if(cycleCountAfter > cycleCountBefore)
        {
            cpuCycles = cycleCountAfter - cycleCountBefore;
        }
        else
        {
            cpuCycles = (0xFFFFFFFFU - cycleCountBefore) + cycleCountAfter;
        }
        DebugP_log("CPU cycles:%u\n", cpuCycles);
//! [usage]         
    }


}