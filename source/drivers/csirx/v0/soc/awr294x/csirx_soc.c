/*
 * Copyright (C) 2021 Texas Instruments Incorporated
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

#include <drivers/csirx.h>
#include <drivers/hw_include/cslr_soc.h>

CSIRX_HwAttrs gCsirxHwAttrs_csirxA_r5f =
{
    .csirxRegs = CSL_RSS_CSI2A_U_BASE,
    .rcssCtrlRegs = CSL_RSS_CTRL_U_BASE,
    .hwInstId = 0,
    .ctrlClockHz = 96000000,
    .interconnectClockHz = 200000000,
    .commonIntNum = CSL_MSS_INTR_RSS_CSI2A_INT,
    .combinedEndOfLineIntNum = CSL_MSS_INTR_RSS_CSI2A_EOL_INT,
    .combinedEndOfFrameIntNum = CSL_MSS_INTR_RSS_CSI2A_EOF_INT,
    .startOfFrameIntr0IntNum = CSL_MSS_INTR_RSS_CSI2A_SOF_INT0,
    .startOfFrameIntr1IntNum = CSL_MSS_INTR_RSS_CSI2A_SOF_INT1,
    .contextEndOfLineIntNum = {
        CSL_MSS_INTR_RSS_CSI2A_EOL_CNTX0_INT,
        CSL_MSS_INTR_RSS_CSI2A_EOL_CNTX1_INT,
        CSL_MSS_INTR_RSS_CSI2A_EOL_CNTX2_INT,
        CSL_MSS_INTR_RSS_CSI2A_EOL_CNTX3_INT,
        CSL_MSS_INTR_RSS_CSI2A_EOL_CNTX4_INT,
        CSL_MSS_INTR_RSS_CSI2A_EOL_CNTX5_INT,
        CSL_MSS_INTR_RSS_CSI2A_EOL_CNTX6_INT,
        CSL_MSS_INTR_RSS_CSI2A_EOL_CNTX7_INT,
    },
};

CSIRX_HwAttrs gCsirxHwAttrs_csirxA_c66 =
{
    .csirxRegs = CSL_RSS_CSI2A_U_BASE,
    .rcssCtrlRegs = CSL_RSS_CTRL_U_BASE,
    .hwInstId = 0,
    .ctrlClockHz = 96000000,
    .interconnectClockHz = 200000000,
    .commonIntNum = CSL_DSS_INTR_RSS_CSI2A_INT,
    .combinedEndOfLineIntNum = CSL_DSS_INTR_RSS_CSI2A_EOL_INT,
    .combinedEndOfFrameIntNum = CSL_MSS_INTR_RSS_CSI2A_EOF_INT,
    .startOfFrameIntr0IntNum = CSL_DSS_INTR_RSS_CSI2A_SOF_INT0,
    .startOfFrameIntr1IntNum = CSL_DSS_INTR_RSS_CSI2A_SOF_INT1,
    .contextEndOfLineIntNum = {
        CSIRX_INTERRUPT_NOT_CONNECTED_ID,
        CSIRX_INTERRUPT_NOT_CONNECTED_ID,
        CSIRX_INTERRUPT_NOT_CONNECTED_ID,
        CSIRX_INTERRUPT_NOT_CONNECTED_ID,
        CSIRX_INTERRUPT_NOT_CONNECTED_ID,
        CSIRX_INTERRUPT_NOT_CONNECTED_ID,
        CSIRX_INTERRUPT_NOT_CONNECTED_ID,
        CSIRX_INTERRUPT_NOT_CONNECTED_ID,
    },
};
