
//! [include]
#include <kernel/dpl/TimerP.h>
//! [include]

void samples(uint32_t timerBaseAddr)
{
{
//! [initialize]
    TimerP_Params timerParams;
    /* setup timer but dont start it */
    TimerP_Params_init(&timerParams);
    timerParams.inputPreScaler    = 1u;
    timerParams.inputClkHz        = 250u*1000u*1000u;
    timerParams.periodInUsec      = 10u*1000u;
    timerParams.oneshotMode       = 0;
    timerParams.enableOverflowInt = 1;
    TimerP_setup(timerBaseAddr, &timerParams);
//! [initialize]
}
{
//! [start]
    /* start the tick timer */
    TimerP_start(timerBaseAddr);
//! [start]
}
{
//! [curCount]
    uint32_t count;

    /* start the tick timer */
    count = TimerP_getCount(timerBaseAddr);
    (void) count; /* kill warning of variable set but not used */
//! [curCount]
}
{
//! [stop]
    /* stop the tick timer */
    TimerP_stop(timerBaseAddr);
//! [stop]
}
}