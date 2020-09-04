
#include <stdio.h>
#include <kernel/dpl/DebugP.h>
#include <drivers/hw_include/cslr_soc.h>
#include <stdint.h>
//! [include]
#include <drivers/adcbuf.h>
//! [include]

#define CONFIG_ADCBUF0            (0U)

ADCBuf_Handle gADCBufHandle;

void open_adcbuf(void)
{
//! [open_adcbuf]
    ADCBuf_Params params;

    ADCBuf_Params_init(&params);
    gADCBufHandle = ADCBuf_open(CONFIG_ADCBUF0, &params);
    DebugP_assert(gADCBufHandle != NULL);
//! [open_adcbuf]
}

void close_adcbuf(void)
{
//! [close_adcbuf]
    ADCBuf_close(gADCBufHandle);
//! [close_adcbuf]
}
