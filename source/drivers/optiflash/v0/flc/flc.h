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

#ifndef IG__FLC_H__
#define IG__FLC_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


#include <drivers/hw_include/cslr_soc.h>

/**
 * @brief function returned success
 *
 */
#define FLC_API_STS_SUCCESS                     ((FLC_API_STS_t)(0))
/**
 * @brief NULL pointer was passed
 *
 */
#define FLC_API_STS_ERROR_NULL_PTR              ((FLC_API_STS_t)(-1))
/**
 * @brief Illegal core ID was passed
 *
 */
#define FLC_API_STS_ERROR_ILLEGAL_CORE_ID       ((FLC_API_STS_t)(-2))
/**
 * @brief Illegal FLC region ID was passed
 *
 */
#define FLC_API_STS_ERROR_ILLEGAL_REGION_ID     ((FLC_API_STS_t)(-3))
/**
 * @brief Unknown Interrupt passed
 *
 */
#define FLC_API_STS_ERROR_UNKNOWN_INTERRUPT     ((FLC_API_STS_t)(-4))

/**
 * @brief FLC Region 0 ID
 *
 */
#define FLC_REGION1 ((FLC_regionId)(0U))
/**
 * @brief FLC Region 1 ID
 *
 */
#define FLC_REGION2 ((FLC_regionId)(1U))
/**
 * @brief FLC Region 2 ID
 *
 */
#define FLC_REGION3 ((FLC_regionId)(2U))
/**
 * @brief FLC Region 3 ID
 *
 */
#define FLC_REGION4 ((FLC_regionId)(3U))
/**
 * @brief MAX FLC regions supported
 *
 */
#define FLC_MAX_REGION (4U)

/**
 * @brief FLC Done Interrupt
 *
 */
#define FLC_INTERRUPT_DONE          ((FLC_Interrupt)(0U))
/**
 * @brief FLC Write Error Interrupt
 *
 */
#define FLC_INTERRUPT_WRITE_ERROR   ((FLC_Interrupt)(1))
/**
 * @brief FLC read error interrupt
 *
 */
#define FLC_INTERRUPT_READ_ERROR    ((FLC_Interrupt)(2))

/**
 * @brief handle to the FLC instance which is used later on to configure it.
 *
 */
typedef void* FLC_Handle;
/**
 * @brief return value of the API function
 * @see FLC_API_STS_SUCCESS
 * @see FLC_API_STS_ERROR_NULL_PTR
 * @see FLC_API_STS_ERROR_ILLEGAL_CORE_ID
 * @see FLC_API_STS_ERROR_ILLEGAL_REGION_ID
 * @see FLC_API_STS_ERROR_UNKNOWN_INTERRUPT
 */
typedef int FLC_API_STS_t;
/**
 * @brief FLC region ID type
 * @see FLC_REGION1
 * @see FLC_REGION2
 * @see FLC_REGION3
 * @see FLC_REGION4
 */
typedef uint32_t FLC_regionId;
/**
 * @brief FLC interrupt type
 * @see FLC_Interrupt_Done
 * @see FLC_Interrupt_WriteError
 * @see FLC_Interrupt_ReadError
 */
typedef uint32_t FLC_Interrupt;

/**
 * @brief FLC region information.
 *
 * Use this struct to configure the each FLC region
 *
 */
typedef struct
{
    uint32_t sourceStartAddress;        ///< source start address. Data at this address is copied
    uint32_t destinationStartAddress;   ///< destiantion start address, this address is also written
    uint32_t sourceEndAddress;          ///< source end address, upto this address data is copied
    FLC_regionId regionId;              ///< id of flc region which is to be configured
    uint32_t baseAddress;               ///< baseaddress of the ip
} FLC_RegionInfo;

/**
 * @brief configure a FLC region
 *
 * @param regionInfo [in] pointer to a FLC_RegionInfo struct which store the configuration for a particular region
 * @return FLC_API_STS_t
 */
FLC_API_STS_t FLC_configureRegion(FLC_RegionInfo *regionInfo);

/**
 * @brief start FLC copy from source to destination
 *
 * @param regionInfo [in] pointer to region info struct
 * @return FLC_API_STS_t
 */
FLC_API_STS_t FLC_startRegion(FLC_RegionInfo * const regionInfo);

/**
 * @brief check if region is done copying
 *
 * This function sends info of all 4 regions, where bit 0 gives info for region 1, bit 1 for region 2, etc. 1 represents FLC copy is complete.
 *
 * @param regionInfo [in] pointer to region info struct
 * @param status [out] pointer to an unsigned int to store status value
 * @return FLC_API_STS_t
 */
FLC_API_STS_t FLC_isRegionDone(FLC_RegionInfo * const regionInfo, uint32_t *status);

/**
 * @brief check if a read error occured in last FLC copy
 *
 * @param regionInfo [in] pointer to region info struct
 * @param status [out] pointer to an unsigned int to store status value
 * @return FLC_API_STS_t
 */
FLC_API_STS_t FLC_wasReadError(FLC_RegionInfo * const regionInfo, uint32_t *status);

/**
 * @brief check if a write error occured in last FLC copy
 *
 * @param regionInfo [in] pointer to region info struct
 * @return FLC_API_STS_t
 */
FLC_API_STS_t FLC_wasWriteError(FLC_RegionInfo * const regionInfo, uint32_t *status);

/**
 * @brief clear any write error if occured
 *
 * @param regionInfo [in] pointer to region info struct
 * @return FLC_API_STS_t
 */
FLC_API_STS_t FLC_clearWriteError(FLC_RegionInfo * const regionInfo);

/**
 * @brief clear any read error if occured
 *
 * @param regionInfo [in] pointer to region info struct
 * @return FLC_API_STS_t
 */
FLC_API_STS_t FLC_clearReadError(FLC_RegionInfo * const regionInfo);

/**
 * @brief return IRQ mask.
 *
 * Refer to FLC IRQMASK register for more details
 *
 * @param regionInfo [in] pointer to region info struct
 * @param status [out] pointer to uint32_t variable in which the status is stored
 * @return FLC_API_STS_t
 */
FLC_API_STS_t FLC_readIRQMask(FLC_RegionInfo * const regionInfo, uint32_t *status);

/**
 * @brief reutrn FLC IRQ Status register value.
 *
 * Refer to FLC IRQ Status.
 *
 * @param regionInfo [in] pointer to region info struct
 * @param status [out] pointer to uint32_t variable in which the status is stored
 * @return FLC_API_STS_t
 */
FLC_API_STS_t FLC_readIRQStatus(FLC_RegionInfo * const regionInfo, uint32_t *status);

/**
 * @brief  Set Interrupt for different FLC events.
 *
 * @param regionInfo [in] pointer to region info struct
 * @param intr interrupt which needs to be set.
 * @return FLC_API_STS_t
 * @see FLC_Interrupt
 */
FLC_API_STS_t FLC_enableInterrupt(FLC_RegionInfo * const regionInfo, FLC_Interrupt intr);

/**
 * @brief Clear Interrupt for different FLC events.
 *
 * @param regionInfo [in] pointer to region info struct
 * @param intr interrupt which needs to be set.
 * @return FLC_API_STS_t
 * @see FLC_Interrupt
 */
FLC_API_STS_t FLC_clearInterrupt(FLC_RegionInfo * const regionInfo, FLC_Interrupt intr);

/**
 * @brief Clear Interrupt for different FLC events.
 *
 * @param regionInfo [in] pointer to region info struct
 * @param intr interrupt which needs to be set.
 * @return FLC_API_STS_t
 * @see FLC_Interrupt
 */
FLC_API_STS_t FLC_disableInterrupt(FLC_RegionInfo * const regionInfo, FLC_Interrupt intr);

/**
 * @brief Disable a region
 *
 * @param regionInfo [in] pointer to region info struct
 * @return FLC_API_STS_t
 */
FLC_API_STS_t FLC_disable(FLC_RegionInfo * const regionInfo);

#ifdef __cplusplus
}
#endif

#endif //IG__FLC_H__