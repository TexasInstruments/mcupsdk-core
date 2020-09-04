/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
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

/**
 *  \file mibspi_soc.c
 *
 *  \brief File containing MIBSPI soc APIs implementation for version V0.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <string.h>
#include <drivers/mibspi/v0/mibspi.h>

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
static void MIBSPI_socMemInitMSSSPIA(void);
static void MIBSPI_socMemInitMSSSPIB(void);
static void MIBSPI_socMemInitRCSSSPIA(void);
static void MIBSPI_socMemInitRCSSSPIB(void);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
static void MIBSPI_socMemInitRCSSSPIA(void)
{
    CSL_rcss_ctrlRegs *rcssCtrl = (CSL_rcss_ctrlRegs *)CSL_RCSS_CTRL_U_BASE;

    /* Clear MEMINIT DONE before initiating MEMINIT */
    CSL_FINS(rcssCtrl->RCSS_SPIA_MEMINIT_DONE, RCSS_CTRL_RCSS_SPIA_MEMINIT_DONE_RCSS_SPIA_MEMINIT_DONE_MEM0_DONE, 1);
    while (CSL_FEXT(rcssCtrl->RCSS_SPIA_MEMINIT_DONE, RCSS_CTRL_RCSS_SPIA_MEMINIT_DONE_RCSS_SPIA_MEMINIT_DONE_MEM0_DONE) != 0);

    CSL_FINS(rcssCtrl->RCSS_SPIA_MEMINIT, RCSS_CTRL_RCSS_SPIA_MEMINIT_RCSS_SPIA_MEMINIT_MEM0_INIT, 1);

    while (CSL_FEXT(rcssCtrl->RCSS_SPIA_MEMINIT_DONE, RCSS_CTRL_RCSS_SPIA_MEMINIT_DONE_RCSS_SPIA_MEMINIT_DONE_MEM0_DONE) != 1);
    CSL_FINS(rcssCtrl->RCSS_SPIA_MEMINIT_DONE, RCSS_CTRL_RCSS_SPIA_MEMINIT_DONE_RCSS_SPIA_MEMINIT_DONE_MEM0_DONE, 1);
    while (CSL_FEXT(rcssCtrl->RCSS_SPIA_MEMINIT_DONE, RCSS_CTRL_RCSS_SPIA_MEMINIT_DONE_RCSS_SPIA_MEMINIT_DONE_MEM0_DONE) != 0);

}

static void MIBSPI_socMemInitRCSSSPIB(void)
{
    CSL_rcss_ctrlRegs *rcssCtrl = (CSL_rcss_ctrlRegs *)CSL_RCSS_CTRL_U_BASE;

    /* Clear MEMINIT DONE before initiating MEMINIT */
    CSL_FINS(rcssCtrl->RCSS_SPIB_MEMINIT_DONE, RCSS_CTRL_RCSS_SPIB_MEMINIT_DONE_RCSS_SPIB_MEMINIT_DONE_MEM0_DONE, 1);
    while (CSL_FEXT(rcssCtrl->RCSS_SPIB_MEMINIT_DONE, RCSS_CTRL_RCSS_SPIB_MEMINIT_DONE_RCSS_SPIB_MEMINIT_DONE_MEM0_DONE) != 0);

    CSL_FINS(rcssCtrl->RCSS_SPIB_MEMINIT, RCSS_CTRL_RCSS_SPIB_MEMINIT_RCSS_SPIB_MEMINIT_MEM0_INIT, 1);

    while (CSL_FEXT(rcssCtrl->RCSS_SPIB_MEMINIT_DONE, RCSS_CTRL_RCSS_SPIB_MEMINIT_DONE_RCSS_SPIB_MEMINIT_DONE_MEM0_DONE) != 1);
    CSL_FINS(rcssCtrl->RCSS_SPIB_MEMINIT_DONE, RCSS_CTRL_RCSS_SPIB_MEMINIT_DONE_RCSS_SPIB_MEMINIT_DONE_MEM0_DONE, 1);
    while (CSL_FEXT(rcssCtrl->RCSS_SPIB_MEMINIT_DONE, RCSS_CTRL_RCSS_SPIB_MEMINIT_DONE_RCSS_SPIB_MEMINIT_DONE_MEM0_DONE) != 0);

}

static void MIBSPI_socMemInitMSSSPIA(void)
{
    CSL_mss_ctrlRegs *mssCtrl = (CSL_mss_ctrlRegs *)CSL_MSS_CTRL_U_BASE;


    /* Clear MEMINIT DONE before initiating MEMINIT */
    CSL_FINS(mssCtrl->MSS_SPIA_MEM_INIT_DONE, MSS_CTRL_MSS_SPIA_MEM_INIT_DONE_MSS_SPIA_MEM_INIT_DONE_MEM0_DONE, 1);
    while (CSL_FEXT(mssCtrl->MSS_SPIA_MEM_INIT_DONE, MSS_CTRL_MSS_SPIA_MEM_INIT_DONE_MSS_SPIA_MEM_INIT_DONE_MEM0_DONE) != 0);

    CSL_FINS(mssCtrl->MSS_SPIA_MEM_INIT, MSS_CTRL_MSS_SPIA_MEM_INIT_MSS_SPIA_MEM_INIT_MEM0_INIT, 1);

    while (CSL_FEXT(mssCtrl->MSS_SPIA_MEM_INIT_DONE, MSS_CTRL_MSS_SPIA_MEM_INIT_DONE_MSS_SPIA_MEM_INIT_DONE_MEM0_DONE) != 1);
    CSL_FINS(mssCtrl->MSS_SPIA_MEM_INIT_DONE, MSS_CTRL_MSS_SPIA_MEM_INIT_DONE_MSS_SPIA_MEM_INIT_DONE_MEM0_DONE, 1);
    while (CSL_FEXT(mssCtrl->MSS_SPIA_MEM_INIT_DONE, MSS_CTRL_MSS_SPIA_MEM_INIT_DONE_MSS_SPIA_MEM_INIT_DONE_MEM0_DONE) != 0);
}

static void MIBSPI_socMemInitMSSSPIB(void)
{
    CSL_mss_ctrlRegs *mssCtrl = (CSL_mss_ctrlRegs *)CSL_MSS_CTRL_U_BASE;

    /* Clear MEMINIT DONE before initiating MEMINIT */
    CSL_FINS(mssCtrl->MSS_SPIB_MEM_INIT_DONE, MSS_CTRL_MSS_SPIB_MEM_INIT_DONE_MSS_SPIB_MEM_INIT_DONE_MEM0_DONE, 1);
    while (CSL_FEXT(mssCtrl->MSS_SPIB_MEM_INIT_DONE, MSS_CTRL_MSS_SPIB_MEM_INIT_DONE_MSS_SPIB_MEM_INIT_DONE_MEM0_DONE) != 0);

    CSL_FINS(mssCtrl->MSS_SPIB_MEM_INIT, MSS_CTRL_MSS_SPIB_MEM_INIT_MSS_SPIB_MEM_INIT_MEM0_INIT, 1);

    while (CSL_FEXT(mssCtrl->MSS_SPIB_MEM_INIT_DONE, MSS_CTRL_MSS_SPIB_MEM_INIT_DONE_MSS_SPIB_MEM_INIT_DONE_MEM0_DONE) != 1);
    CSL_FINS(mssCtrl->MSS_SPIB_MEM_INIT_DONE, MSS_CTRL_MSS_SPIB_MEM_INIT_DONE_MSS_SPIB_MEM_INIT_DONE_MEM0_DONE, 1);
    while (CSL_FEXT(mssCtrl->MSS_SPIB_MEM_INIT_DONE, MSS_CTRL_MSS_SPIB_MEM_INIT_DONE_MSS_SPIB_MEM_INIT_DONE_MEM0_DONE) != 0);

}

void MIBSPI_socMemInit(uint32_t index)
{
    switch(index)
    {
        case MIBSPI_INST_ID_MSS_SPIA:
            MIBSPI_socMemInitMSSSPIA();
            break;
        case MIBSPI_INST_ID_MSS_SPIB:
            MIBSPI_socMemInitMSSSPIB();
            break;
        case MIBSPI_INST_ID_RCSS_SPIA:
            MIBSPI_socMemInitRCSSSPIA();
            break;
        case MIBSPI_INST_ID_RCSS_SPIB:
            MIBSPI_socMemInitRCSSSPIB();
            break;
    }
    return;
}
