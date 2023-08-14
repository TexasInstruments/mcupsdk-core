
//! [include]
#include <drivers/mpu_firewall.h>
//! [include]

void samples()
{
//! [mpu]

    MPU_FIREWALL_RegionParams mpuParams;

    /* Initialize the MPU Param structure */
    MPU_FIREWALL_RegionParams_init(&mpuParams);

    /* Set the region parameters */
    mpuParams.id = 4;
    mpuParams.regionNumber = 0;

    /* Program the parameters for the region */
    MPU_FIREWALL_getRegion(&mpuParams);

//! [mpu]
}