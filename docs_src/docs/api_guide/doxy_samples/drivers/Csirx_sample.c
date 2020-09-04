
#include <stdio.h>
//! [include]
#include <drivers/csirx.h>
//! [include]

#define CONFIG_CSIRX0   (0)
CSIRX_Handle gCsirxHandle[1];

//! [buffer]
/* max buffer size for ping and ping */
#define MY_CSIRX__FRAME_LINES_MAX    (4U)
#define MY_CSIRX__BYTES_PER_LINE_MAX (128U)

/* ping pong buffer, MUST be placed in a memory accessible to both CPU and CSIRX HW */
uint8_t gMyCsirxBuf[2][MY_CSIRX__FRAME_LINES_MAX*MY_CSIRX__BYTES_PER_LINE_MAX] __attribute__((aligned(64), section(".bss.dss_l3")));
//! [buffer]

//! [isr]

/* flag to check if ISR has occured, you can also use semaphore's instead of global variables to block on a interrupt instead */
bool gMyCsirxContextIntrDone = false;
uint8_t gMyCsirxContextId = 0; /* the context that is used to receive CSIRX frames */

/* interrupt callback, we simply count the interrupts in the callback */
/* this ISR is specified via SysConfig */
void my_csirx_commonCallback(CSIRX_Handle handle, void *arg, struct CSIRX_CommonIntr_s *irq)
{
    if(irq->isContextIntr[gMyCsirxContextId])
    {
        CSIRX_ContextIntr contextIntrStatus;

        CSIRX_contextGetPendingIntr(handle, gMyCsirxContextId, &contextIntrStatus);
        CSIRX_contextClearAllIntr(handle, gMyCsirxContextId);

        if(contextIntrStatus.isFrameEndCodeDetect)
        {
            gMyCsirxContextIntrDone = true;
        }
    }
}
//! [isr]

void csirx_operation(void)
{
//! [operation]
    uint32_t bufId = 0;

    /* After the Drivers_open or Drivers_csirxInstanceOpen, CSIRX is setup and configured, and the interface is enabled, however the context is not enabled */
    /* Enable the required context(s) by doing below, here we also setup the data output addresses before enabling the CSIRX context */
    CSIRX_contextSetPingPongAddress(gCsirxHandle[CONFIG_CSIRX0], gMyCsirxContextId, (uint32_t)gMyCsirxBuf[0], (uint32_t)gMyCsirxBuf[1] );
    CSIRX_contextEnable(gCsirxHandle[CONFIG_CSIRX0], gMyCsirxContextId);

    while(1)
    {
        /* wait for one frame to be captured */
        while( gMyCsirxContextIntrDone != true )
        {
        }
        gMyCsirxContextIntrDone = false;

        /* process data in buffer gMyCsirxBuf[bufId] */

        bufId = bufId ^ 1; /* switch between 0 and 1, i.e ping and pong */
    }

    /* when done disable CSIRX context */
    CSIRX_contextDisable(gCsirxHandle[CONFIG_CSIRX0], gMyCsirxContextId);

//! [operation]
}
