
//! [include]
#include <drivers/rti.h>
//! [include]

void samples(uint32_t rtiBaseAddr)
{
{
//! [start]
    /* start the rti counter */
    RTI_counterEnable(rtiBaseAddr, RTI_TMR_CNT_BLK_INDEX_0);
//! [start]
}
{
//! [stop]
    /* stop the rti counter */
    RTI_counterDisable(rtiBaseAddr, RTI_TMR_CNT_BLK_INDEX_0);
//! [stop]
}
}