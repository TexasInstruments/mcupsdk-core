
#include <stdio.h>
#include <kernel/dpl/DebugP.h>
#include <drivers/hw_include/cslr_soc.h>
#include <stdint.h>
//! [include]
#include <drivers/adc.h>
//! [include]

uint32_t adcBaseAddr = CSL_ADC0_BASE;

void init_adc(void)
{
//! [init_adc]
    /* Use internal calibration */
    uint32_t errOffset = 1;
    /* Starts internal calibration */
    uint32_t calibration = 1;

    ADCInit(adcBaseAddr, TRUE, errOffset, calibration);
//! [init_adc]
}

void get_fifo_threshold(void)
{
//! [get_fifo_threshold]
    /* Get threshold level for FIFO 0 */
    uint32_t level = ADCGetCPUFIFOThresholdLevel(adcBaseAddr, ADC_FIFO_NUM_0);

    DebugP_log("Number of words after which interrupt to CPU will be generated for FIFO0 is : %i\r\n", level);
//! [get_fifo_threshold]
}

void check_adc_poweredup(void)
{
//! [adc_powered_up]
    /* Check if ADC is powered up or not */
    uint32_t status = AdcIsPoweredUp(adcBaseAddr);

    if(TRUE == status)
    {
        DebugP_log("ADC is powered up\r\n");
    }

    else
    {
        DebugP_log("ADC is not powered up\r\n");
    }

//! [adc_powered_up]
}
