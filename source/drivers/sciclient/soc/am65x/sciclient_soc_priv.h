/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
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
 *  \file am65x/sciclient_soc_priv.h
 *
 *  \brief Private AM65xx specific SOC file
 */

#ifndef SCICLIENT_SOC_PRIV_H_
#define SCICLIENT_SOC_PRIV_H_

#include <drivers/sciclient/soc/am65x/sciclient_irq_rm.h>
#include <drivers/sciclient/sciclient_default_boardcfg/am65x/sciclient_defaultBoardcfg_hex.h>
#include <drivers/sciclient/sciclient_default_boardcfg/am65x/sciclient_defaultBoardcfg_rm_hex.h>
#include <drivers/sciclient/sciclient_default_boardcfg/am65x/sciclient_defaultBoardcfg_pm_hex.h>
#include <drivers/sciclient/sciclient_default_boardcfg/am65x/sciclient_defaultBoardcfg_security_hex.h>

/**
 *  \anchor Sciclient_RomThreadIds
 *  \name Sciclient load firmware ROM Thread Ids.
 *  @{
 */

/** Thread ID's for each message type, R5 Out Bound Normal Priority    */
#define SCICLIENT_ROM_R5_TX_NORMAL_THREAD                           (4U)
/** Thread ID's for each message type, R5 In Bound Normal Priority    */
#define SCICLIENT_ROM_R5_RX_NORMAL_THREAD                           (5U)

/** @} */

/** CPU ID of the R5F */
#define SCICLIENT_HOST_ID            (TISCI_HOST_ID_R5_1)

#endif /* SCICLIENT_SOC_PRIV_H_ */
