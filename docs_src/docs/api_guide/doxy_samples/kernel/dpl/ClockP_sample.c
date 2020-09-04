
//! [include]
#include <kernel/dpl/ClockP.h>
//! [include]

//! [callback]
uint32_t gOneShotCount = 0;
uint32_t gPeriodicCount = 0;

void myClockCallback(ClockP_Object *obj, void *arg)
{
    uint32_t *value = (uint32_t*)arg;

    (*value)++; /* increment number of time's this callback is called */
}
//! [callback]

void samples()
{
{
//! [oneshot mode]
    ClockP_Params clockParams;
    ClockP_Object clockObj;

    ClockP_Params_init(&clockParams);
    clockParams.timeout = ClockP_usecToTicks(10*1000);
    clockParams.start = 1;
    clockParams.callback = myClockCallback;
    clockParams.args = &gOneShotCount; /* pass address of counter which is incremented in the callback */

    ClockP_construct(&clockObj, &clockParams);
//! [oneshot mode]
}
{
//! [periodic mode]
    ClockP_Params clockParams;
    ClockP_Object clockObj;

    ClockP_Params_init(&clockParams);
    clockParams.timeout = ClockP_usecToTicks(100*1000);
    clockParams.period = clockParams.timeout;
    clockParams.start = 1;
    clockParams.callback = myClockCallback;
    clockParams.args = &gPeriodicCount; /* pass address of counter which is incremented in the callback */

    ClockP_construct(&clockObj, &clockParams);
//! [periodic mode]
}
{
//! [time]
    uint64_t curTimeInUsecs;

    curTimeInUsecs = ClockP_getTimeUsec();

    // code or functions to profile
    // func1()
    // func2()

    curTimeInUsecs = ClockP_getTimeUsec() - curTimeInUsecs;

//! [time]
}
}