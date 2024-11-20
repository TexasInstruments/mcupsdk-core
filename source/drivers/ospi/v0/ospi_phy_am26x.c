/*
 *  Copyright (C) 2021-2024 Texas Instruments Incorporated
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <string.h>
#include <drivers/ospi.h>
#include <drivers/ospi/v0/lld/ospi_lld.h>
#include <drivers/ospi/v0/cslr_ospi.h>
#include <drivers/hw_include/cslr.h>

int32_t OSPI_phyReadAttackVector(OSPI_Handle handle, uint32_t offset)
{
    int32_t status = SystemP_SUCCESS;
    OSPILLD_Handle hOspi;
    if(NULL != handle)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        hOspi = &obj->ospilldObject;
        status = OSPI_lld_phyReadAttackVector(hOspi,offset);
    }

    return status;
}

void OSPI_phyGetTuningData(uint32_t *tuningData, uint32_t *tuningDataSize)
{
    OSPI_lld_phyGetTuningData(tuningData,tuningDataSize);
}

/*
 *
  Shown below is an approximate txDLL vs rxDLL graph of a typical PHY.
  Regions P1-P2-BL and Q1-Q2-R2-TR-R1 are the passing regions.
  Each region corresponds to a different value of read data capture delay.
  The gap between the regions can move away or towards origin depending on various factors (like temperature).

  There can be just one region also. Or the orientation of the gap will be opposite.

  The tuning/calibration algorithm can work correctly only if we have a general idea of this graph.

  The grapher function blindly sweeps through all the tx and rx DLL values for rdDelays 0,1,2,3 and writes this
  data to the array passed into this function. This has to be a [4][128][128] array.

  This data can be then saved by reading the SOC memory region and saving it as binary data. Python script can be
  written to process this to give the PHY scatter graph of the particular flash.

 RX
  |
  |     R1
  |     _______________________________ TR
  |     |                               |
  |     | Q1                            |
  |   P1 \                              |
  |       \                             |
  |     |\ \                            |
  |     | \ \                           |
  |     |  \ \                          |
  |     |   \ \                         |
  |     |    \ \                        |
  |     |     \ \                       |
  |     |      \ \                      |
  |     |       \ \ Q2                  |
  |     |________\ \____________________|R2
  |     BL     P2
  |_________________________________________ TX

*/

int32_t OSPI_phyTuneGrapher(OSPI_Handle handle, uint32_t flashOffset, uint8_t arrays[4][128][128])
{
    int32_t status = SystemP_SUCCESS;
    OSPILLD_Handle hOspi;
    if(NULL!= handle)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        hOspi = &obj->ospilldObject;
        status = OSPI_lld_phyTuneGrapher(hOspi,flashOffset,arrays);
    }

    return status;
}

int32_t OSPI_phyTuneDDR(OSPI_Handle handle, uint32_t flashOffset)
{
    int32_t status = SystemP_SUCCESS;

    OSPILLD_Handle hOspi;
    if(NULL!= handle)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        hOspi = &obj->ospilldObject;
        status = OSPI_lld_phyTuneDDR(hOspi,flashOffset);
    }
    return status;
}

int32_t OSPI_phyTuneSDR(OSPI_Handle handle, uint32_t flashOffset)
{
    int32_t status = SystemP_SUCCESS;

    OSPILLD_Handle hOspi;
    if(NULL!= handle)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        hOspi = &obj->ospilldObject;
        status = OSPI_lld_phyTuneSDR(hOspi,flashOffset);
    }
    return status;
}
