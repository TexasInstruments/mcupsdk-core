/*
 *  Copyright (C) 2022 Texas Instruments Incorporated
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

#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <kernel/dpl/DebugP.h>
#include <drivers/soc.h>
#include <drivers/pcie.h>
#include <string.h>

/******** Buffer *******/
#define BUF_SIZE            60

/* Number of Interrupts */
#define MSI_NUM_ITER        (10u)

/* MSI interrupt number */
/* This is the interrupt number identified by the 5bit LSB of MSI data send from EP to RC */
#define MSI_IRQ_NUM         (5u)

uint32_t dst_buf[BUF_SIZE]  __attribute__((aligned(4096)));

void Pcie_bufInit(void)
{
    int i = 0;

    for (i = 0; i < BUF_SIZE; i++)
    {
        dst_buf[i] = 0xFFFFFFFF;
    }

    return;
}

void pcie_msi_irq_ep_main (void *args)
{
    int32_t status;
    Pcie_MsiParams msiParams;
    Pcie_sendMsiParams params;
    uint32_t i;

    Drivers_open();
    Board_driversOpen();

    DebugP_log("Device in EP mode\r\n");

    Pcie_bufInit();

    for (i = 0; i < MSI_NUM_ITER; i++)
    {
        /* Check if the buffer (0xC0DEC0DE) is received from RC */
        do
        {
            CacheP_inv (dst_buf, BUF_SIZE * sizeof(uint32_t), CacheP_TYPE_ALL);
        } while ((dst_buf[BUF_SIZE-2] != 0xC0DEC0DE) || (dst_buf[BUF_SIZE-1] != i));

        if (i == 0)
        {
            memset (&msiParams, 0, sizeof(msiParams));
            status = Pcie_getMsiRegs (gPcieHandle[CONFIG_PCIE0], PCIE_LOCATION_LOCAL, &msiParams);
        }

        /* Loopback buffer to RC */
        {
            void *transBufAddr = (void *)(CONFIG_PCIE0_OB_REGION0_LOWER);

            memcpy(transBufAddr, dst_buf, BUF_SIZE * sizeof(uint32_t));
        }

        /* Send MSI to RC */
        if (status == SystemP_SUCCESS && msiParams.enable == 1)
        {
            params.addr = CONFIG_PCIE0_OB_REGION1_LOWER;
            params.data = msiParams.data;
            params.intNum = MSI_IRQ_NUM;

            status = Pcie_epSendMsiIrq(gPcieHandle[CONFIG_PCIE0], MSI_IRQ_NUM, params);

            if(status != SystemP_SUCCESS)
            {
                break;
            }
        }

    }

    Board_driversClose();
    Drivers_close();

    return;
}
