#include <stdio.h>
//! [include]
#include <drivers/qos.h>
/* QoS configuration data file */
#include "drivers/qos/v0/soc/am64x/qos_data.h"
//! [include]

void init(void)
{
//! [init]
    QOS_init(gQosData, gQosCount);
//! [init]
}