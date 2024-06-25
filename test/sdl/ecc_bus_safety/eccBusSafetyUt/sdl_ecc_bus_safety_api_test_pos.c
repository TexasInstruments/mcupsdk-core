/* Copyright (c) 2022-23 Texas Instruments Incorporated
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
 *
 */

 /**
 *  \file     sdl_ecc_bus_safety_api_test_pos.c
 *
 *  \brief    This file contains ECC BUS SAFETY  API positive unit test code..
 *
 *  \details  Ecc Bus Safety unit tests
 **/

#include "ecc_bus_safety_main.h"

int32_t sdl_ecc_bus_safety_posTest(void)
{
    int32_t  testStatus = SDL_APP_TEST_PASS;
#if !defined (SOC_AM263PX) || defined (SOC_AM261X)
    SDL_ECC_BUS_SAFETY_staticRegs regs;
#endif
    uint32_t writeData = 0x1234567U;

#if defined (SOC_AM273X) ||  defined (SOC_AWR294X)
#if defined (SUBSYS_DSS)
    uint32_t status =0 ;
#endif
#endif

#if defined (SUBSYS_MSS)
    uint32_t status =0 ;
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD ) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
#if defined (SOC_AM273X) ||  defined (SOC_AWR294X)
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
#endif

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

#if defined (SOC_AM273X) || defined (SOC_AWR294X)
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_WR) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
#endif
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_MCRC) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_QSPI) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

#if defined (SOC_AM273X) ||  defined (SOC_AWR294X)
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_SWBUF) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_TO_MDO) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_SCRP) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
#endif
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5A_AHB) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5B_AHB) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
#endif

#if defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_RD) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_RD) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_S) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_S) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5C_AHB) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5D_AHB) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_WR) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_WR) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_MBOX) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_MCRC) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_QSPI) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_STM_STIM) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_SCRP0) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_SCRP1) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_ICSSM_PDSP0) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_ICSSM_PDSP1) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_ICSSM_S) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_DAP) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secExecute(SDL_ECC_BUS_SAFETY_MSS_MBOX,SDL_MBOX_SRAM_U_BASE, writeData) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secExecute(SDL_ECC_BUS_SAFETY_MSS_MCRC,0U, writeData) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secExecute(SDL_ECC_BUS_SAFETY_MSS_MCRC,0U, 0) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* Test cases for SDL_ECC_BUS_SAFETY_MSS_secExecute */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secExecute(SDL_ECC_BUS_SAFETY_MSS_CPSW,0U, 0U) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_dedExecute(SDL_ECC_BUS_SAFETY_MSS_MBOX,SDL_MBOX_SRAM_U_BASE, writeData) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_dedExecute(SDL_ECC_BUS_SAFETY_MSS_MCRC,0U, writeData) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_dedExecute(SDL_ECC_BUS_SAFETY_MSS_MCRC,0U, 0) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* Test cases for SDL_ECC_BUS_SAFETY_MSS_dedExecute */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_dedExecute(SDL_ECC_BUS_SAFETY_MSS_CPSW,0U, 0U) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

#endif

#if defined (SOC_AM273X) || defined (SOC_AWR294X)
#if defined (SUBSYS_MSS)

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD ) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_WR) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5A_AHB) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5B_AHB) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
#endif
#endif


#if defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_RD) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_RD) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_S) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_S) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5C_AHB) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5D_AHB) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_WR) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_WR) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
#endif



#if defined (SOC_AM273X) || defined (SOC_AWR294X)
#if defined (SUBSYS_MSS)
    /* Test cases for SDL_ECC_BUS_SAFETY_MSS_secErrorClear with different input parameter */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_CPSW) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* Test cases for SDL_ECC_BUS_SAFETY_MSS_secErrorClear with different input parameter */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_L2_A) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* Test cases for SDL_ECC_BUS_SAFETY_MSS_secErrorClear with different input parameter */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_L2_B) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* Test cases for SDL_ECC_BUS_SAFETY_MSS_secErrorClear with different input parameter */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_L2_B) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* Test cases for SDL_ECC_BUS_SAFETY_MSS_secErrorClear with different input parameter */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_GPADC) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* Test cases for SDL_ECC_BUS_SAFETY_MSS_secErrorClear with different input parameter */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_DMM_SLV) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* Test cases for SDL_ECC_BUS_SAFETY_MSS_redErrorClear with different input parameter */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_DMM) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* Test cases for SDL_ECC_BUS_SAFETY_MSS_secExecute */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secExecute(SDL_ECC_BUS_SAFETY_MSS_PCR,0U, 0U) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* Test cases for SDL_ECC_BUS_SAFETY_MSS_secExecute */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secExecute(SDL_ECC_BUS_SAFETY_MSS_PCR2,0U, 0U) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* Test cases for SDL_ECC_BUS_SAFETY_MSS_secExecute */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secExecute(SDL_ECC_BUS_SAFETY_MSS_CPSW,0U, 0U) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* Test cases for SDL_ECC_BUS_SAFETY_MSS_dedExecute */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_dedExecute(SDL_ECC_BUS_SAFETY_MSS_PCR,0U, 0U) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* Test cases for SDL_ECC_BUS_SAFETY_MSS_dedExecute */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_dedExecute(SDL_ECC_BUS_SAFETY_MSS_PCR2,0U, 0U) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* Test cases for SDL_ECC_BUS_SAFETY_MSS_dedExecute */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_dedExecute(SDL_ECC_BUS_SAFETY_MSS_CPSW,0U, 0U) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* Test cases for SDL_ECC_BUS_SAFETY_MSS_readStaticRegs */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if(SDL_ECC_BUS_SAFETY_MSS_readStaticRegs(SDL_ECC_BUS_SAFETY_MSS_GPADC,&regs ) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_WR) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5A_AHB) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5B_AHB) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secExecute(SDL_ECC_BUS_SAFETY_MSS_MBOX,SDL_MSS_MBOX_U_BASE, writeData) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secExecute(SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD,0U, writeData) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secExecute(SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD,0U, writeData) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secExecute(SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD,0U, writeData) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secExecute(SDL_ECC_BUS_SAFETY_MSS_QSPI,SDL_MSS_QSPI_U_BASE, writeData) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secExecute(SDL_ECC_BUS_SAFETY_MSS_MCRC,SDL_MSS_MCRC_U_BASE, writeData) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secExecute(SDL_ECC_BUS_SAFETY_MSS_SWBUF,SDL_MSS_SWBUF_U_BASE, writeData) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secExecute(SDL_ECC_BUS_SAFETY_MSS_TO_MDO,SDL_MSS_TO_MDO_U_BASE, writeData) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secExecute(SDL_ECC_BUS_SAFETY_MSS_TO_MDO,0XCA000000, writeData) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secExecute(SDL_ECC_BUS_SAFETY_DAP_R232,0U, writeData) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_dedExecute(SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD,0U, writeData) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_dedExecute(SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD,0U, writeData) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_dedExecute(SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD,0U, writeData) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_dedExecute(SDL_ECC_BUS_SAFETY_MSS_MBOX,SDL_MSS_MBOX_U_BASE, writeData) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_dedExecute(SDL_ECC_BUS_SAFETY_MSS_QSPI,SDL_MSS_QSPI_U_BASE, writeData) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_dedExecute(SDL_ECC_BUS_SAFETY_MSS_MCRC,SDL_MSS_MCRC_U_BASE, writeData) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_dedExecute(SDL_ECC_BUS_SAFETY_MSS_SWBUF,SDL_MSS_SWBUF_U_BASE, writeData) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_dedExecute(SDL_ECC_BUS_SAFETY_MSS_TO_MDO,SDL_MSS_TO_MDO_U_BASE, writeData) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_dedExecute(SDL_ECC_BUS_SAFETY_DAP_R232,0U, writeData) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_redExecute(SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_redExecute(SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD, SDL_ECC_BUS_SAFETY_FI_MAIN, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_redExecute(SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD, SDL_ECC_BUS_SAFETY_FI_GLOBAL_MAIN, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_redExecute(SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD, SDL_ECC_BUS_SAFETY_FI_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_redExecute(SDL_ECC_BUS_SAFETY_MSS_QSPI, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_redExecute(SDL_ECC_BUS_SAFETY_MSS_MCRC, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_redExecute(SDL_ECC_BUS_SAFETY_MSS_SWBUF, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_redExecute(SDL_ECC_BUS_SAFETY_MSS_SCRP, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_redExecute(SDL_ECC_BUS_SAFETY_MSS_TO_MDO, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_redExecute(SDL_ECC_BUS_SAFETY_DAP_R232, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

#endif
#endif


#if defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secExecute(SDL_ECC_BUS_SAFETY_MSS_MBOX,SDL_MBOX_SRAM_U_BASE, writeData) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secExecute(SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD,0U, writeData) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_dedExecute(SDL_ECC_BUS_SAFETY_MSS_MBOX,SDL_MBOX_SRAM_U_BASE, writeData) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_dedExecute(SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD,0U, writeData) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_redExecute(SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_redExecute(SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD, SDL_ECC_BUS_SAFETY_FI_MAIN, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_redExecute(SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD, SDL_ECC_BUS_SAFETY_FI_GLOBAL_MAIN, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_redExecute(SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD, SDL_ECC_BUS_SAFETY_FI_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

#endif

#if defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_getSecErrorStatus(SDL_ECC_BUS_SAFETY_MSS_MBOX,&status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_getDedErrorStatus(SDL_ECC_BUS_SAFETY_MSS_MBOX,&status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_getRedErrorStatus(SDL_ECC_BUS_SAFETY_MSS_MBOX,&status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_getRedErrorStatus(SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD,&status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_getRedErrorStatus(SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR,&status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_getRedErrorStatus(SDL_ECC_BUS_SAFETY_MSS_MBOX,&status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_L2_A) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* Test cases for SDL_ECC_BUS_SAFETY_MSS_secErrorClear with different input parameter */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_L2_B) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* Test cases for SDL_ECC_BUS_SAFETY_MSS_secErrorClear with different input parameter */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_L2_B) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* Test cases for SDL_ECC_BUS_SAFETY_MSS_secErrorClear with different input parameter */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_L2_C) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* Test cases for SDL_ECC_BUS_SAFETY_MSS_secErrorClear with different input parameter */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_L2_D) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* Test cases for SDL_ECC_BUS_SAFETY_MSS_secErrorClear with different input parameter */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_MMC) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    #if !defined(SOC_AM263PX)
    /* Test cases for SDL_ECC_BUS_SAFETY_MSS_secErrorClear with different input parameter */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_GPMC) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    #endif

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5A_AHB) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5B_AHB) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* Test cases for SDL_ECC_BUS_SAFETY_MSS_redErrorClear with different input parameter */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_MAIN_VBUSP) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_getRedErrorStatus(SDL_ECC_BUS_SAFETY_MSS_CR5B_AHB,&status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_getDedErrorStatus(SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD,&status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_getDedErrorStatus(SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR,&status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_getDedErrorStatus(SDL_ECC_BUS_SAFETY_MSS_MBOX,&status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_getRedErrorStatus(SDL_ECC_BUS_SAFETY_MSS_CR5A_AHB,&status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_getRedErrorStatus(SDL_ECC_BUS_SAFETY_MSS_CR5B_AHB,&status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_getRedErrorStatus(SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD,&status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_getRedErrorStatus(SDL_ECC_BUS_SAFETY_MSS_CR5A_AHB,&status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* Test cases for SDL_ECC_BUS_SAFETY_MSS_redErrorClear with different input parameter */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_PERI_VBUSP) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
     if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_getRedErrorStatus(SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD,&status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* Test cases for SDL_ECC_BUS_SAFETY_MSS_secExecute */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secExecute(SDL_ECC_BUS_SAFETY_MSS_CPSW,0U, 0U) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_getRedErrorStatus(SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD,&status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* Test cases for SDL_ECC_BUS_SAFETY_MSS_dedExecute */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_dedExecute(SDL_ECC_BUS_SAFETY_MSS_CPSW,0U, 0U) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    #if !defined(SOC_AM263PX)
    /* Test cases for SDL_ECC_BUS_SAFETY_MSS_readStaticRegs */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if(SDL_ECC_BUS_SAFETY_MSS_readStaticRegs(SDL_ECC_BUS_SAFETY_MSS_GPMC,&regs ) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    #endif

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_getRedErrorStatus(SDL_ECC_BUS_SAFETY_MSS_MBOX,&status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    #if !defined(SOC_AM263PX)
    /* Test cases for SDL_ECC_BUS_SAFETY_MSS_readStaticRegs */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if(SDL_ECC_BUS_SAFETY_MSS_readStaticRegs(SDL_ECC_BUS_SAFETY_MSS_GPMC,&regs ) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    #endif

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_getRedErrorStatus(SDL_ECC_BUS_SAFETY_MSS_QSPI,&status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_getRedErrorStatus(SDL_ECC_BUS_SAFETY_MSS_MCRC,&status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_getRedErrorStatus(SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR,&status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_getRedErrorStatus(SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD,&status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

#endif
#if defined (SOC_AM273X) ||defined (SOC_AWR294X)
#if defined (SUBSYS_MSS)
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_getSecErrorStatus(SDL_ECC_BUS_SAFETY_MSS_MBOX,&status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_getDedErrorStatus(SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD,&status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_getDedErrorStatus(SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR,&status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_getDedErrorStatus(SDL_ECC_BUS_SAFETY_MSS_MBOX,&status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_getRedErrorStatus(SDL_ECC_BUS_SAFETY_DAP_R232,&status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_getRedErrorStatus(SDL_ECC_BUS_SAFETY_MSS_SWBUF,&status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_getRedErrorStatus(SDL_ECC_BUS_SAFETY_MSS_SCRP,&status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_getRedErrorStatus(SDL_ECC_BUS_SAFETY_MSS_TO_MDO,&status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_getRedErrorStatus(SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD,&status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_getRedErrorStatus(SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR,&status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_getRedErrorStatus(SDL_ECC_BUS_SAFETY_MSS_MBOX,&status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_getRedErrorStatus(SDL_ECC_BUS_SAFETY_MSS_QSPI,&status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_getRedErrorStatus(SDL_ECC_BUS_SAFETY_MSS_MCRC,&status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_getRedErrorStatus(SDL_ECC_BUS_SAFETY_MSS_SWBUF,&status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_getRedErrorStatus(SDL_ECC_BUS_SAFETY_MSS_SCRP,&status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_getRedErrorStatus(SDL_ECC_BUS_SAFETY_MSS_TO_MDO,&status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_getRedErrorStatus(SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR,&status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_getRedErrorStatus(SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD,&status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_getRedErrorStatus(SDL_ECC_BUS_SAFETY_DAP_R232,&status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
#endif
#endif
#if defined (SOC_AM273X) ||  (SOC_AWR294X)
#if defined (SUBSYS_DSS)

    /* Test cases for SDL_ECC_BUS_SAFETY_DSS_secErrorClear with different input parameter */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_DSS_DSP_MDMA) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_DSS_L3_BANKA) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_DSS_L3_BANKB) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_DSS_L3_BANKC) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_DSS_DSP_SDMA) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_RD) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_RD) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_RD) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_RD) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_RD) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_RD) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_RD) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_RD) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_RD) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_RD) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_WR) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_WR) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_WR) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_WR) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_WR) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_WR) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_WR) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_WR) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_WR) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_WR) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_DSS_CBUFF_FIFO) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_DSS_MCRC) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_DSS_PCR) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_DSS_HWA_DMA0) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_DSS_HWA_DMA1) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_DSS_MBOX) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
#endif
#endif
#if defined(SOC_AM273X)
#if defined (SUBSYS_DSS)
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_DSS_MDO_FIFO) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_DSS_L3_BANKD) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
#endif
#endif
#if defined (SOC_AM273X) || defined (SOC_AWR294X)
#if defined (SUBSYS_DSS)
    /* Test cases for SDL_ECC_BUS_SAFETY_DSS_secExecute */
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_secExecute(SDL_ECC_BUS_SAFETY_DSS_L3_BANKA,SDL_DSS_L3_BANKA_ADDRESS, writeData) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_secExecute(SDL_ECC_BUS_SAFETY_DSS_L3_BANKA,SDL_DSS_L3_BANKA_ADDRESS_END, writeData) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_secExecute(SDL_ECC_BUS_SAFETY_DSS_L3_BANKA,SDL_DSS_L3_BANKA_ADDRESS+4U, writeData) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_secExecute(SDL_ECC_BUS_SAFETY_DSS_CBUFF_FIFO,SDL_DSS_CBUFF_FIFO_U_BASE, writeData) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_secExecute(SDL_ECC_BUS_SAFETY_DSS_PCR,0U, 0U) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_secExecute(SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_RD,0U, 0U) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_secExecute(SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_RD,0U, 0U) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_secExecute(SDL_ECC_BUS_SAFETY_DSS_MBOX,SDL_DSS_MAILBOX_U_BASE, writeData) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_secExecute(SDL_ECC_BUS_SAFETY_DSS_MBOX,SDL_DSS_MAILBOX_U_BASE, writeData) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    /* Test cases for SDL_ECC_BUS_SAFETY_DSS_dedExecute */
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_dedExecute(SDL_ECC_BUS_SAFETY_DSS_L3_BANKA,SDL_DSS_L3_BANKA_ADDRESS, writeData) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_dedExecute(SDL_ECC_BUS_SAFETY_DSS_L3_BANKA,SDL_DSS_L3_BANKA_ADDRESS_END, writeData) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_dedExecute(SDL_ECC_BUS_SAFETY_DSS_L3_BANKA,SDL_DSS_L3_BANKA_ADDRESS+4U, writeData) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_dedExecute(SDL_ECC_BUS_SAFETY_DSS_CBUFF_FIFO,SDL_DSS_CBUFF_FIFO_U_BASE, writeData) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_dedExecute(SDL_ECC_BUS_SAFETY_DSS_PCR,0U, 0U) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_dedExecute(SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_RD,0U, 0U) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_dedExecute(SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_RD,0U, 0U) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_dedExecute(SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_RD,0U, 0U) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_dedExecute(SDL_ECC_BUS_SAFETY_DSS_MBOX,SDL_DSS_MAILBOX_U_BASE, writeData) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* Test cases for SDL_ECC_BUS_SAFETY_DSS_redExecute */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_redExecute(SDL_ECC_BUS_SAFETY_DSS_L3_BANKA,SDL_ECC_BUS_SAFETY_FI_MAIN, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_redExecute(SDL_ECC_BUS_SAFETY_DSS_L3_BANKA,SDL_ECC_BUS_SAFETY_FI_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_redExecute(SDL_ECC_BUS_SAFETY_DSS_L3_BANKA,SDL_ECC_BUS_SAFETY_FI_GLOBAL_MAIN, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_redExecute(SDL_ECC_BUS_SAFETY_DSS_L3_BANKA,SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* Test cases for SDL_ECC_BUS_SAFETY_DSS_redExecute */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if(SDL_ECC_BUS_SAFETY_DSS_readStaticRegs(SDL_ECC_BUS_SAFETY_DSS_MBOX,&regs ) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
     /* Test cases for SDL_ECC_BUS_SAFETY_DSS_getSecErrorStatus */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_getSecErrorStatus(SDL_ECC_BUS_SAFETY_DSS_L3_BANKA,&status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* Test cases for SDL_ECC_BUS_SAFETY_DSS_dedErrorClear */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_dedErrorClear(SDL_ECC_BUS_SAFETY_DSS_L3_BANKA) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* Test cases for SDL_ECC_BUS_SAFETY_DSS_getDedErrorStatus */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_getDedErrorStatus(SDL_ECC_BUS_SAFETY_DSS_L3_BANKA,&status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* Test cases for SDL_ECC_BUS_SAFETY_DSS_redErrorClear */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_redErrorClear(SDL_ECC_BUS_SAFETY_DSS_L3_BANKA) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_redErrorClear(SDL_ECC_BUS_SAFETY_DSS_L3_BANKA) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* Test cases for SDL_ECC_BUS_SAFETY_DSS_redErrorClear */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_getRedErrorStatus(SDL_ECC_BUS_SAFETY_DSS_L3_BANKA,&status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_getRedErrorStatus(SDL_ECC_BUS_SAFETY_DSS_CBUFF_FIFO,&status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_getRedErrorStatus(SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_RD,&status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_getRedErrorStatus(SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_WR,&status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_getRedErrorStatus(SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_RD,&status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_getRedErrorStatus(SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_RD,&status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_getRedErrorStatus(SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_WR,&status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_getRedErrorStatus(SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_WR,&status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_getRedErrorStatus(SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_WR,&status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_getRedErrorStatus(SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_RD,&status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_getRedErrorStatus(SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_WR,&status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_getRedErrorStatus(SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_WR,&status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_getRedErrorStatus(SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_RD,&status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
#if defined (SOC_AWR294X)
    SDL_ECC_BUS_SAFETY_DSS_AWR2944_AWR2943_Detect();
#endif
#endif
#endif
#if defined (SOC_AWR294X) || defined (SOC_AM273X)
#if defined (SUBSYS_DSS)
    writeData = status;
    status = testStatus;
#endif
#endif
    return (testStatus);

}
