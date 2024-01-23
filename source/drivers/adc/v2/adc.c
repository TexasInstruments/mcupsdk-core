/*
 * Copyright (C) 2024 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <drivers/adc.h>
#include <stdint.h>

//*****************************************************************************
//
// ADC_setMode
//
//*****************************************************************************
void ADC_setMode(uint32_t base, ADC_Resolution resolution,
                 ADC_SignalMode signalMode)
{
    //
    // Apply the resolution and signalMode to the specified ADC.
    //
    HW_WR_REG16(base + CSL_ADC_ADCCTL2,
        ((HW_RD_REG16(base + CSL_ADC_ADCCTL2) &
        ~(CSL_ADC_ADCCTL2_RESOLUTION_MASK | CSL_ADC_ADCCTL2_SIGNALMODE_MASK)) |
        (((uint16_t)resolution<<CSL_ADC_ADCCTL2_RESOLUTION_SHIFT) |
        (((uint16_t)signalMode)<<CSL_ADC_ADCCTL2_SIGNALMODE_SHIFT))));
}

//*****************************************************************************
//
// ADC_setPPBTripLimits
//
//*****************************************************************************
void
ADC_setPPBTripLimits(uint32_t base, ADC_PPBNumber ppbNumber,
                     int32_t tripHiLimit, int32_t tripLoLimit)
{
    uint32_t ppbHiOffset;
    uint32_t ppbLoOffset;

    //
    // Check the arguments.
    //
    DebugP_assert((tripHiLimit <= 65535) && (tripHiLimit >= -65536));
    DebugP_assert((tripLoLimit <= 65535) && (tripLoLimit >= -65536));

    //
    // Get the offset to the appropriate trip limit registers.
    //
    ppbHiOffset = (ADC_ADCPPBx_STEP * (uint32_t)ppbNumber) +
                  CSL_ADC_ADCPPB1TRIPHI;
    ppbLoOffset = (ADC_ADCPPBx_STEP * (uint32_t)ppbNumber) +
                  CSL_ADC_ADCPPB1TRIPLO;

    //
    // Set the trip high limit.
    //
    HW_WR_REG32(base + ppbHiOffset,
        ((HW_RD_REG32(base + ppbHiOffset) & ~ADC_ADCPPBTRIP_MASK) |
        ((uint32_t)tripHiLimit & ADC_ADCPPBTRIP_MASK)));

    //
    // Set the trip low limit.
    //
    HW_WR_REG32(base + ppbLoOffset,
        ((HW_RD_REG32(base + ppbLoOffset) & ~ADC_ADCPPBTRIP_MASK) |
        ((uint32_t)tripLoLimit & ADC_ADCPPBTRIP_MASK)));
}


//*****************************************************************************
//
// ADC_configureRepeater
//
//*****************************************************************************
void
ADC_configureRepeater(uint32_t base, uint16_t repInstance,
                      ADC_RepeaterConfig *config)
{
    uint32_t regOffset;
    //
    // Check the arguments.
    //
    DebugP_assert(repInstance <= 2U);
    DebugP_assert(config->repCount <= 127U);

    regOffset = base + (repInstance * (ADC_REPxCTL_STEP));

    //
    // Configure repeater mode, trigger and syncin source.
    //
    HW_WR_REG32(
        regOffset + CSL_ADC_REP1CTL,
        (HW_RD_REG32(regOffset + CSL_ADC_REP1CTL) &
              ~((uint32_t)ADC_REPCTL_MASK))  | (((uint32_t)config->repMode) |
                ((uint32_t)config->repTrigger << CSL_ADC_REP1CTL_TRIGGER_SHIFT) |
                ((uint32_t)config->repSyncin << CSL_ADC_REP1CTL_SYNCINSEL_SHIFT)));

    //
    // Configure repeater count.
    //
    HW_WR_REG16(regOffset + CSL_ADC_REP1N,
        (HW_RD_REG16( regOffset + CSL_ADC_REP1N ) &
        ~(CSL_ADC_REP1N_NSEL_MASK)) | config->repCount
    );

    //
    // Configure repeater phase.
    //
    HW_WR_REG16(regOffset + CSL_ADC_REP1PHASE,
        (HW_RD_REG16( regOffset + CSL_ADC_REP1PHASE ) &
        ~(CSL_ADC_REP1PHASE_PHASE_MASK)) | config->repPhase
    );
    
    //
    // Configure repeater spread.
    //
    HW_WR_REG16(regOffset + CSL_ADC_REP1SPREAD,
        (HW_RD_REG16( regOffset + CSL_ADC_REP1SPREAD ) &
        ~(CSL_ADC_REP1SPREAD_SPREAD_MASK)) | config->repSpread
    );
}