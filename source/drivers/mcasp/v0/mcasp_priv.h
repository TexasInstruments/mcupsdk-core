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
 *  \file v0/mcasp_priv.h
 *
 *  \brief MCASP Driver internal file.
 */

#ifndef MCASP_PRIV_H_
#define MCASP_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <drivers/mcasp.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                          Function Declarations */
/* ========================================================================== */

/**
 * \brief   Function for doing the edma configuration for the instance being opened
 *
 * \param config pointer to #MCASP_Config structure of instance being opened.
 * \param edmaInst EDMA instance number used for EDMA configuration
 *
 */
void MCASP_openDma(MCASP_Config *config, uint32_t edmaInst);

/**
 * \brief   Function for releasing the edma configuration for the instance being closed
 *
 * \param config pointer to #MCASP_Config structure of instance being closed.
 * \param edmaInst EDMA instance number used for EDMA configuration
 *
 */
void MCASP_closeDma(MCASP_Config *config, uint32_t edmaInst);

/**
 * \brief   Function for enabling the dma transfer for transmission
 *
 * \param config pointer to #MCASP_Config.
 *
 */
int32_t MCASP_enableDmaTx(MCASP_Config *config);

/**
 * \brief   Function for disabling the dma transfer for reception
 *
 * \param config pointer to #MCASP_Config.
 *
 */
int32_t MCASP_enableDmaRx(MCASP_Config *config);

/**
 * \brief   Function for enabling the dma transfer for transmission
 *
 * \param config pointer to #MCASP_Config.
 *
 */
void MCASP_disableDmaTx(MCASP_Config *config);

/**
 * \brief   Function for disabling the dma transfer for reception
 *
 * \param config pointer to #MCASP_Config.
 *
 */
void MCASP_disableDmaRx(MCASP_Config *config);

#ifdef __cplusplus
}
#endif

#endif /* #ifndef MCASP_PRIV_H_ */

/** @} */
