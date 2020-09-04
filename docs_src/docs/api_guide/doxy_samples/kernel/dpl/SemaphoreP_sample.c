
//! [include]
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/ClockP.h>
//! [include]

//! [define]
/* semaphore objects, typically these are global's */
SemaphoreP_Object gBinarySem;
SemaphoreP_Object gMutexSem;
SemaphoreP_Object gCountingSem;

/* resource to protect with counting semaphore */
#define NUM_RESOURCES   (10u)
uint32_t gResource[NUM_RESOURCES];

//! [define]

//! [callback]
/* peripheral ISR which post's the binary semaphore */
void myISR(void *args)
{
    SemaphoreP_post(&gBinarySem);
}
//! [callback]

void samples()
{
{
//! [mutex]
    SemaphoreP_constructMutex(&gMutexSem);

    /* wait forever for the mutex to be available, lock or enter the critical section */
    SemaphoreP_pend(&gMutexSem, SystemP_WAIT_FOREVER);

    /* mutual exclusion, critical section */

    /* unlock the mutex, exit critical section */
    SemaphoreP_post(&gMutexSem);
//! [mutex]
}
{
//! [binary]
    int32_t status;

    SemaphoreP_constructBinary(&gBinarySem, 0);

    // initialize peripheral
    // register myISR
    // enable peripheral

    /* wait for 10ms for the semaphore to be post by the peripheral */
    status = SemaphoreP_pend(&gBinarySem, ClockP_usecToTicks(10*1000));
    if(status==SystemP_SUCCESS)
    {
        /* success */
    }
    else
    if(status==SystemP_TIMEOUT)
    {
        /* failed due to timeout */
    }
    else
    {
        /* other failure */
    }

//! [binary]
}
{
//! [counting]
    uint32_t resourceId = 0;

    SemaphoreP_constructCounting(&gCountingSem, NUM_RESOURCES, NUM_RESOURCES);

    /* wait for a resource to be available */
    SemaphoreP_pend(&gCountingSem, SystemP_WAIT_FOREVER);

    /* access the resource */

    /* release resoource */
    resourceId = (resourceId+1)%NUM_RESOURCES;
    SemaphoreP_post(&gCountingSem);

//! [counting]
}
}