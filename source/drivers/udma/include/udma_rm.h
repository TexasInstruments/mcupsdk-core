/*
 *  Copyright (C) 2018-2021 Texas Instruments Incorporated
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
 *  \ingroup DRV_UDMA_MODULE
 *  \defgroup DRV_UDMA_RM_MODULE UDMA RM API
 *            This is UDMA driver resource manager related configuration
 *            parameters and API
 *
 *  @{
 */

/**
 *  \file udma_rm.h
 *
 *  \brief UDMA RM related parameters and API.
 */

#ifndef UDMA_RM_H_
#define UDMA_RM_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** \brief Macro used to specify that Resource ID is invalid. */
#define UDMA_RM_RES_ID_INVALID          ((uint32_t) 0xFFFF0005U)

/** \brief Macro used to specify - reserve minimum required number of resources for an instance */
#define UDMA_RM_SHARED_RES_CNT_MIN      ((uint32_t) 0xFFFF0006U)

/** \brief Macro used to specify - reserve all the remaining unreserved resources for an instance */
#define UDMA_RM_SHARED_RES_CNT_REST     ((uint32_t) 0xFFFF0007U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 *  \brief UDMA resource manager shared resource parameters.
 */
typedef struct
{
    uint32_t                resId;
    /**< UDMA Resource Id */
    uint32_t                startResrvCnt;
    /**< No. of resources from the start left reserved and can't be used by UDMA
     *   Ex: Core Interrupts(IR Interrupts) used by IPC, CPSW9G etc */
    uint32_t                endResrvCnt;
    /**< No. of resources from the end left reserved and can't be used by UDMA
     *   Ex: Core Interrupts(IR Interrupts) used by IPC, CPSW9G etc */
    uint32_t                numInst;
    /**< No. of instances for which the available resources should be split.
     *   -This can be no. of UDMA Instances (#UDMA_NUM_INST_ID) incase of
     *    resources like Gloable Events, IR Intr, VINT etc.
     *   -This can be no.of cores (#UDMA_NUM_CORE) incase of splitting
     *    resources that are assigned #TISCI_HOST_ID_ALL between different cores*/
    uint32_t                minReq;
     /**< Minimum no. of resources required per instance.
      *   This is validated with the unresrved number of resources.
      *   ie, UDMA Driver will return error (#UDMA_EBADARGS) when:
      *     (numInst * minReq) > (total_num_res -  startResrvCnt - endResrvCnt)
      *      where, total_num_res = range_num returned by \ref Sciclient_rmGetResourceRange
      *
      *   For example,
      *   When numInst = 2; minReq = 50;
      *   range_num = 110; startResrvCnt = 10; endResrvCnt = 10;
      *
      *   no.of unreserved resources = 110-10-10 = 90
      *   But, total requirment = minReq*numInst = 50*2 = 100
      *   Since the requirment cant be met, UDMA Driver will return #UDMA_EBADARGS
      *
      *   In this case either more no.of resources should be reserved in
      *   Sciclient_deafaultBoardCfg_rm.c
      *   OR
      *   Adjustments should be made in minReq/startResrvCnt/endResrvCnt.
      */
    uint32_t                instShare[UDMA_RM_SHARED_RES_MAX_INST];
    /**< No. of resources for each instance.
     *   This can be:
     *   - #UDMA_RM_SHARED_RES_CNT_MIN - Reserves 'minReq' no.of resources for the instance
     *       ie, final_share = minReq
     *   - #UDMA_RM_SHARED_RES_CNT_REST - Reserves all the remaining unreserved resources.
     *       ie, final_share = total_num_res -  startResrvCnt - endResrvCnt - sum(other_instance_share)
     *       If more than one instance uses #UDMA_RM_SHARED_RES_CNT_REST,
     *       the remaining will be split equally.
     *   - Any specific number such that:-
     *       sum of instance shares is less than unresrved number of resources
     *       ie, "sum(instShare[]) < (total_num_res -  startResrvCnt - endResrvCnt)"
     *       Else will return error (#UDMA_EINVALID_PARAMS)
     *       Note: While calculating 'sum(instShare[])', 'minReq' count
     *       will be used for shares with #UDMA_RM_SHARED_RES_CNT_MIN or
     *       #UDMA_RM_SHARED_RES_CNT_REST
     *
     *   For example,
     *   When numInst = 4; minReq = 50; range_num = 410;
     *   startResrvCnt = 7; endResrvCnt = 3;
     *   (Here, no.of unreserved resources = 410-7-3 = 400)
     *
     *   Case 1: instShare[] = {UDMA_RM_SHARED_RES_CNT_MIN,
     *                          UDMA_RM_SHARED_RES_CNT_REST,
     *                          UDMA_RM_SHARED_RES_CNT_REST,
     *                          100U}
     *
     *       sum(instShare[]) = 50+50+50+100 = 250
     *       This is less than no.of unreserved resources(400)
     *
     *       Therefore, final_share will be {50,125,125,100}
     *
     *   Case 2: instShare[] = {UDMA_RM_SHARED_RES_CNT_MIN,
     *                          UDMA_RM_SHARED_RES_CNT_REST,
     *                          250U,
     *                          100U}
     *
     *       sum(instShare[]) = 50+50+250+100 = 450
     *       Since this is greater than no.of unreserved resources(400),
     *       UDMA Driver will return #UDMA_EINVALID_PARAMS
     */
} Udma_RmSharedResPrms;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
/**
 *  \brief Returns the RM Shared Resource default parameters structure for the
 *  requested resource.
 *  User can use this API to get the default parameters for a resource
 *  and override as per need.
 *
 *  \param resId       [IN] \ref Udma_RmResId
 *
 *  \return Pointer to default RM Shared Resource parameters
 *          #Udma_RmSharedResPrms
 *          Note: Returns NULL_PTR if the requested \ref Udma_RmResId
 *          dosen't have an entry in the array of default RM Shared Resource
 *          parameters structure
 */
Udma_RmSharedResPrms *Udma_rmGetSharedResPrms(uint32_t resId);

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                  Internal/Private Structure Declarations                   */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif /* #ifndef UDMA_RM_H_ */

/** @} */
