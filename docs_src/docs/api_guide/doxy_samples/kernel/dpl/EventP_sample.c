
//! [include]
#include <kernel/dpl/EventP.h>
#include <kernel/dpl/DebugP.h>
//! [include]

//! [define]
/* Event objects, typically these are global's */
EventP_Object gMyEvent;

/* Event bits */
/* Event bit mask upto 24 bits */
#define EVENT_BIT_PING (0x000001u)
#define EVENT_BIT_PONG (0x000002u)
//! [define]

void create(void *args)
{
    int32_t status;
//! [create]
    status = EventP_construct(&gMyEvent);
    DebugP_assert(SystemP_SUCCESS == status);
//! [create]
}

//! [task1]
void task1(void *args)
{
    int32_t status;
    uint32_t eventBits;

    /* Set PING bit from task1 */
    status = EventP_setBits(&gMyEvent, EVENT_BIT_PING);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Reading the Event Bits */
    status = EventP_getBits(&gMyEvent, &eventBits);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Wait for PONG but to be set from task2. PING bit was already set */
    status = EventP_waitBits(&gMyEvent, (EVENT_BIT_PING | EVENT_BIT_PONG), 0, 1, SystemP_WAIT_FOREVER, &eventBits);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Clearing the PING and PONG bits */
    status = EventP_clearBits(&gMyEvent, (EVENT_BIT_PING | EVENT_BIT_PONG));
    DebugP_assert(SystemP_SUCCESS == status);
}
//! [task1]


//! [task2]
void task2(void *args)
{
    int32_t status;
    uint32_t eventBits;

    /* Wait for PING but to be set from task1*/
    status = EventP_waitBits(&gMyEvent, EVENT_BIT_PING, 0, 1, SystemP_WAIT_FOREVER, &eventBits);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Set PONG bit from task2 */
    status = EventP_setBits(&gMyEvent, EVENT_BIT_PONG);
    DebugP_assert(SystemP_SUCCESS == status);

}
//! [task2]
