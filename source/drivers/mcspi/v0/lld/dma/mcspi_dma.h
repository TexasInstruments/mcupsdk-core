/*
 *  Copyright (C) 2021-23 Texas Instruments Incorporated
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
 *  \file mcspi_dma.h
 *
 *  \brief MCSPI DMA LLD header file.
 */

#ifndef MCSPI_DMA_H_
#define MCSPI_DMA_H_

#include <stdint.h>
#include <drivers/mcspi.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**
 *  \defgroup MCSPI_DMA_LLD APIs for MCSPI DMA mode
 *  \ingroup DRV_MCSPI_LLD_MODULE
 *
 *  This module contains APIs to program and use DMA drivers available in the SoC with MCSPI.
 *
 *  @{
 */

/**
 * \brief API to open an MCSPI DMA channel
 *
 * This API will open a DMA Channel using the appropriate DMA driver callbacks and the registered via Sysconfig
 *
 * \param mcspiDmaHandle    [in] #MCSPILLD_Handle returned from #MCSPI_open()
 *
 * \return Handle to the MCSPI DMA Config Object
 */
int32_t MCSPI_lld_dmaInit(MCSPI_DmaHandle mcspiDmaHandle);

/**
 * \brief API to close an MCSPI DMA channel
 *
 * This API will open a DMA Channel using the appropriate DMA driver callbacks registered via Sysconfig
 *
 * \param hMcspi    [in] #MCSPILLD_Handle returned from #MCSPI_open()
 * \param chCfg     [in] Pointer to #MCSPI_ChConfig. This parameter can't be NULL
 *
 * \return MCSPI_STATUS_SUCCESS on success, else failure
 */
int32_t MCSPI_lld_dmaDeInit(MCSPILLD_Handle hMcspi, const MCSPI_ChConfig *chCfg, uint32_t chCnt);

/**
 * \brief API to init a DMA Channel opened
 *
 * This API will open a DMA Channel using the appropriate DMA driver callbacks registered via Sysconfig
 *
 * \param hMcspi    [in] #MCSPILLD_Handle returned from #MCSPI_open()
 *
 * \return MCSPI_STATUS_SUCCESS on success, else failure
 */
int32_t MCSPI_lld_dmaChInit(MCSPILLD_Handle hMcspi, uint32_t chCnt);

/**
 * \brief API to do a DMA transfer using a specific DMA driver - UDMA, EDMA etc
 *
 * Typically this callback is hidden from the end application and is implemented
 * when a new DMA driver needs to be supported.
 *
 * \param hMcspi    [in] #MCSPILLD_Handle returned from #MCSPI_open()
 * \param chObj         [in] Pointer to #MCSPI_ChObject. This parameter can't be NULL
 * \param transaction   [in] Pointer to #MCSPI_Transaction. This parameter can't be NULL
 *
 * \return MCSPI_STATUS_SUCCESS on success, else failure
 */
int32_t MCSPI_lld_dmaTransfer(MCSPILLD_Handle hMcspi, MCSPI_ChObject *chObj, const MCSPI_Transaction *transaction);

/**
 * \brief API to Stop DMA using a specific DMA driver - UDMA, EDMA etc
 *
 * Typically this callback is hidden from the end application and is implemented
 * when a new DMA driver needs to be supported.
 *
 * \param hMcspi    [in] #MCSPILLD_Handle returned from #MCSPI_open()
 * \param chObj         [in] Pointer to #MCSPI_ChObject. This parameter can't be NULL
 * \param chNum         [in] Channel number
 *
 * \return void
 */
void MCSPI_lld_dmaStop(MCSPILLD_Handle hMcspi, MCSPI_ChObject *chObj, uint32_t chNum);

/**
 *  \defgroup MCSPI_DMA APIs for MCSPI DMA mode
 *  \ingroup DRV_MCSPI_MODULE
 *
 *  This module contains APIs to program and use DMA drivers available in the SoC with MCSPI.
 *
 *  @{
 */

/**
 * \brief Driver implementation to open a specific DMA driver channel - UDMA, EDMA etc
 *
 * Typically this callback is hidden from the end application and is implemented
 * when a new DMA driver needs to be supported.
 *
 * \param mcspiDmaArgs   [in] DMA specific arguments, obtained from the config
 *
 * \return MCSPI_STATUS_SUCCESS on success, else failure
 */
typedef int32_t (*MCSPI_dmaOpenFxn)(void *mcspiDmaArgs);

/**
 * \brief Driver implementation to close a specific DMA driver channel - UDMA, EDMA etc
 *
 * Typically this callback is hidden from the end application and is implemented
 * when a new DMA driver needs to be supported.
 *
 * \param MCSPI_DmaHandle   [in] MCSPI DMA Object handle returned from \ref MCSPI_dmaOpen
 * \param mcspiDmaArgs      [in] DMA specific arguments, obtained from the config
 *
 * \return MCSPI_STATUS_SUCCESS on success, else failure
 */
typedef int32_t (*MCSPI_dmaCloseFxn)(MCSPI_Handle handle, const MCSPI_ChConfig *chCfg);

/**
 * \brief Driver implementation to init a DMA channel params using a specific DMA driver - UDMA, EDMA etc
 *
 * Typically this callback is hidden from the end application and is implemented
 * when a new DMA driver needs to be supported.
 *
 * \param handle    [in] #MCSPI_Handle returned from #MCSPI_open()
 * \param chCfg     [in] Pointer to #MCSPI_ChConfig. This parameter can't be NULL
 * \param dmaChCfg  [in] Pointer to #MCSPI_DmaChConfig. This parameter can't be NULL
 *
 * \return MCSPI_STATUS_SUCCESS on success, else failure
 */
typedef int32_t (*MCSPI_dmaChInitFxn)(MCSPI_Handle handle,
                        const MCSPI_ChConfig *chCfg,
                        const MCSPI_DmaChConfig *dmaChCfg);

/**
 * \brief Driver implementation to do a DMA transfer using a specific DMA driver - UDMA, EDMA etc
 *
 * Typically this callback is hidden from the end application and is implemented
 * when a new DMA driver needs to be supported.
 *
 * \param obj           [in] Pointer to MCSPI object
 * \param chObj         [in] Pointer to #MCSPI_ChObject. This parameter can't be NULL
 * \param attrs         [in] Pointer to #MCSPI_Attrs. This parameter can't be NULL
 * \param transaction   [in] Pointer to #MCSPI_Transaction. This parameter can't be NULL
 *
 * \return MCSPI_STATUS_SUCCESS on success, else failure
 */
typedef int32_t (*MCSPI_dmaTransferControllerFxn)(MCSPI_Object *obj,
                                MCSPI_ChObject *chObj,
                                const MCSPI_Attrs *attrs,
                                MCSPI_Transaction *transaction);
/**
 * \brief Driver implementation to Stop DMA using a specific DMA driver - UDMA, EDMA etc
 *
 * Typically this callback is hidden from the end application and is implemented
 * when a new DMA driver needs to be supported.
 *
 * \param obj           [in] Pointer to MCSPI object
 * \param attrs         [in] Pointer to #MCSPI_Attrs. This parameter can't be NULL
 * \param chObj         [in] Pointer to #MCSPI_ChObject. This parameter can't be NULL
 * \param chNum         [in] Channel number
 *
 * \return MCSPI_STATUS_SUCCESS on success, else failure
 */
typedef int32_t (*MCSPI_dmaStopFxn)(MCSPI_Object *obj,
                                const MCSPI_Attrs *attrs,
                                MCSPI_ChObject *chObj,
                                uint32_t chNum);

/**
 * \brief Driver implementation callbacks
 */
typedef struct MCSPI_DmaFxns_s
{
	MCSPI_dmaOpenFxn                dmaOpenFxn;
	MCSPI_dmaCloseFxn               dmaCloseFxn;
	MCSPI_dmaChInitFxn              dmaChInitFxn;
    MCSPI_dmaTransferControllerFxn      dmaTransferControllerFxn;
    MCSPI_dmaStopFxn                dmaStopFxn;

} MCSPI_DmaFxns;

#if defined (DMA_VERSION_MCSPI_UDMA)
MCSPI_DmaFxns gMcspiDmaUdmaFxns;
#endif
#if defined (DMA_VERSION_MCSPI_EDMA)
MCSPI_DmaFxns gMcspiDmaEdmaFxns;
#endif

/**
 * \brief MCSPI DMA Configuration, these are filled by SysCfg based on the DMA driver that is selected
 */
typedef struct MCSPI_DmaConfig_s
{
	MCSPI_DmaFxns *fxns;
	/* Registered callbacks for a particular DMA driver. This will be set by Sysconfig depending on the DMA driver selected*/
	void *mcspiDmaArgs;
	/* Arguments specific to a DMA driver. This will be typecasted to the specific DMA driver args struct
	 * when used by the appropriate callback. This struct will be defined in the specific DMA driver header file.
	 * Allocation of this struct will be done statically using Sysconfig code generation in the example code
	 */
} MCSPI_DmaConfig;

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* MCSPI_DMA_H_ */