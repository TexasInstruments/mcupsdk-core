
//! [include]
#include <stdio.h>
#include <kernel/dpl/HwiP.h>
//! [include]

//! [isr]
void myISR(void *args)
{
    /* my ISR */
}
//! [isr]

void samples()
{
{
//! [register]
    HwiP_Params hwiParams;
    HwiP_Object hwiObj;

    HwiP_Params_init(&hwiParams);
    /* for R5F, interrupt #10 at VIM */
    hwiParams.intNum = 10; 
    /* for M4F, external interrupt #10 at NVIC is 
       16 internal interrupts + external interrupt number at NVIC 
       i.e hwiParams.intNum = 16 + 10; 
     */
    hwiParams.callback = myISR;
    hwiParams.args = NULL;

    HwiP_construct(&hwiObj, &hwiParams);
//! [register]
}
{
//! [disable]
    uintptr_t oldIntState;

    oldIntState = HwiP_disable();

    /* critical section */

    HwiP_restore(oldIntState);
//! [disable]
}
}