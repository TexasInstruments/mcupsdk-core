/*
 * Copyright (C) 2025 Texas Instruments Incorporated
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


 #ifndef MMCSD_EDMA_H_
 #define MMCSD_EDMA_H_    
 
 #include <stdint.h> 
 #include <drivers/mmcsd.h>
 #include <drivers/mmcsd/v1/lld/mmcsd_lld.h>
 #include <drivers/mmcsd/v1/lld/internal/mmcsd_parse.h>
 #include <drivers/edma/v0/edma.h>
 
 #ifdef __cplusplus
 extern "C"
 {
 #endif
 
 /**
  *  \defgroup MMCSD_DMA_LLD APIs for MMCSD DMA mode
  *  \ingroup DRV_MMCSD_LLD_MODULE
  *
  *  This module contains APIs to program and use DMA drivers available in the SoC with MMCSD.
  *
  *  @{
  */
 
typedef struct MMCSD_EdmaChConfig_s
{
  uint32_t        edmaTccRx;
  /**< EDMA TCC used for MMCSD RX transfer */
  uint32_t        edmaTccTx;
  /**< EDMA TCC used for MMCSD TX transfer */
  uint32_t        edmaTccDummy;
  /**< EDMA TCC used for MMCSD Dummy transfer */
  uint32_t        edmaRxChId;
  /**< EDMA Channel used for MMCSD RX transfer */
  uint32_t        edmaTxChId;
  /**< EDMA Channel used for MMCSD TX transfer */
  uint32_t        edmaRxParam;
  /**< EDMA Param ID used for MMCSD RX transfer */
  uint32_t        edmaTxParam;
  /**< EDMA Param ID used for MMCSD TX transfer */
  uint32_t        edmaDummyParam;
  /**< EDMA Param ID used for MMCSD TX Dummy transfer */
  uint32_t        edmaRegionId;
  /**< EDMA Region used for MMCSD transfer */
  uint32_t        edmaBaseAddr;
  /**< EDMA Base address used for MMCSD transfer */
  Edma_IntrObject edmaIntrObjRx;
  /**< EDMA MMCSD RX Interrupt object */
  Edma_IntrObject edmaIntrObjTx;
  /**< EDMA MMCSD TX Interrupt object */
  Edma_IntrObject edmaIntrObjDummy;
  /**< EDMA MMCSD Dummy Interrupt object */
  uint32_t        isIntEnabled;
  /**< Flag to indicate whether interrupt is enabled */
} MMCSD_EdmaChConfig;

  /**
  * \brief API to open an MMCSD DMA channel
  *
  * This API will open a DMA Channel using the appropriate DMA driver callbacks and the registered via Sysconfig
  *
  * \param mmcsdHandle    [in] #MMCSDILLD_Handle returned from #MMCSD_open()
  *
  * \return Handle to the MMCSD DMA Config Object
  */
 int32_t MMCSD_edmaChInit(MMCSDLLD_Handle mmcsdHandle);
 
  /**
  * \brief API to close an MMCSD DMA channel
  *
  * This API will close the previously opened DMA Channel.
  *
  * \param mmcsdHandle    [in] #MMCSDILLD_Handle returned from #MMCSD_open()
  *
  * \return Status of the operation
  */
 int32_t MMCSD_edmaChDeinit(MMCSDLLD_Handle mmcsdHandle);
 
/**
* \brief API to do a DMA transfer using EDMA driver
*
* This API will perform a DMA transfer using the EDMA driver.
*
* \param mmcsdHandle    [in] #MMCSDILLD_Handle returned from #MMCSD_open()
* \param src            [in] Pointer to source buffer
* \param dst            [in] Pointer to destination buffer
* \param size           [in] Size of the data to be transferred
* \param timeout        [in] Timeout value for the transfer
*
* \return Status of the operation
*/
 
int32_t MMCSD_edmaTransfer(MMCSDLLD_Handle mmcsdHandle, MMCSDLLD_Transaction *trans);
 
 #ifdef __cplusplus
 }
 #endif
 
 #endif