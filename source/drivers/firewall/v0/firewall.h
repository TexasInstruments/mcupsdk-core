/*
 *  Copyright (C) 2022-23 Texas Instruments Incorporated
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
 *  \defgroup DRV_FIREWALL_MODULE APIs for FIREWALL
 *  \ingroup DRV_MODULE
 *
 *  This module contains APIs to program and use the firewall module. The APIs
 *  can be used by others to get access to firewall module.
 *
 *  @{
 */

/**
 *  \file v0/firewall.h
 *
 *  \brief Firewall Driver API/interface file.
 */

#ifndef FIREWALL_H_
#define FIREWALL_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <string.h>
#include <drivers/soc.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/SemaphoreP.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
 * \brief Maximun number of privilege ID slots
 */
#define FWL_MAX_PRIVID_SLOTS                    (3U)

/**
 * \brief Firewall Limits
 */
#define FWL_MAX_ID                              (35U)
#define FWL_MAX_REGION                          (16U)

/**
 *  \anchor Firewall_PrivId
 *  \name Master Privilege ID's
 *
 *  Privilege ID's of Master
 *
 *  @{
 */
#define PRIVID_NONE                             (0U)
#define PRIVID_ALLOW_EVERYONE                   (195U)
#define PRIVID_DMSC                             (202U)
#define PRIVID_BLOCK_EVERYONE                   (197U)
#define PRIVID_SPROXY_PRIVATE                   (11U)
#define PRIVID_A53_NON_SECURE                   (1U)
#define PRIVID_A53_SECURE                       (1U)
#define PRIVID_M4_0_NON_SECURE                  (100U)
#define PRIVID_MAIN_0_R5_0_NONSECURE            (212U)
#define PRIVID_MAIN_0_R5_0_SECURE               (212U)
#define PRIVID_MAIN_0_R5_1_NONSECURE            (213U)
#define PRIVID_MAIN_0_R5_1_SECURE               (213U)
#define PRIVID_MAIN_1_R5_0_NONSECURE            (214U)
#define PRIVID_MAIN_1_R5_0_SECURE               (214U)
#define PRIVID_MAIN_1_R5_1_NONSECURE            (215U)
#define PRIVID_MAIN_1_R5_1_SECURE               (215U)
#define PRIVID_MAIN_0_ICSSG_0                   (136U)
/** @} */

/**
 *  \anchor Firewall_ControlWord
 *  \name Firewall Control Mask
 *
 *  Control mask of word of Firewall
 *
 *  @{
 */
#define FWL_CONTROL_ENABLE_MASK                (0xFU)
#define FWL_CONTROL_DISABLE                    (0x0U)
#define FWL_CONTROL_ENABLE                     (0xAU)
#define FWL_CONTROL_LOCK                       (0x10U)
#define FWL_CONTROL_BG                         (0x100U)
#define FWL_CONTROL_CACHE_MODE                 (0x200U)
/** @} */

/**
 *  \anchor Firewall_PermWord
 *  \name Firewall Permission Mask
 *
 *  Permission mask of Firewall
 *
 *  @{
 */
#define FWL_PERM_SEC_MASK                     (0x00FFU)
#define FWL_PERM_NSEC_MASK                    (0xFF00U)
#define FWL_PERM_PRIV_MASK                    (0x0F0FU)
#define FWL_PERM_USER_MASK                    (0xF0F0U)

#define FWL_PERM_WRITE_MASK                   (0x1111U)
#define FWL_PERM_READ_MASK                    (0x2222U)
#define FWL_PERM_CACHE_MASK                   (0x4444U)
#define FWL_PERM_DEBUG_MASK                   (0x8888U)
#define FWL_PERM_DENY_ALL                     (0x0000U)
/** @} */

/**
*  \anchor Firewall_PermMacro
*  \name Macros for Firewall Permission's
*
*  Macros for Firewall permission's
*
*  @{
*/
#define FWL_PERM_RW_ALL                        (FWL_PERM_WRITE_MASK | \
						FWL_PERM_READ_MASK | \
						FWL_PERM_CACHE_MASK | \
						FWL_PERM_DEBUG_MASK)

#define FWL_PERM_RO_ALL                        (FWL_PERM_READ_MASK | \
						FWL_PERM_CACHE_MASK | \
						FWL_PERM_DEBUG_MASK)

#define FWL_PERM_WO_ALL                        (FWL_PERM_WRITE_MASK | \
						FWL_PERM_CACHE_MASK | \
						FWL_PERM_DEBUG_MASK)

#define FWL_PERM_SEC_RW                        (FWL_PERM_SEC_MASK & \
						FWL_PERM_RW_ALL)

#define FWL_PERM_NSEC_RW                       (FWL_PERM_NSEC_MASK & \
						FWL_PERM_RW_ALL)

#define FWL_PERM_PRIV_RW                       (FWL_PERM_PRIV_MASK & \
						FWL_PERM_RW_ALL)

#define FWL_PERM_USER_RW                       (FWL_PERM_USER_MASK & \
						FWL_PERM_RW_ALL)

#define FWL_PERM_SEC_RO                        (FWL_PERM_SEC_MASK & \
						FWL_PERM_RO_ALL)

#define FWL_PERM_NSEC_RO                       (FWL_PERM_NSEC_MASK & \
						FWL_PERM_RO_ALL)

#define FWL_PERM_PRIV_RO                       (FWL_PERM_PRIV_MASK & \
						FWL_PERM_RO_ALL)

#define FWL_PERM_USER_RO                       (FWL_PERM_USER_MASK & \
						FWL_PERM_RO_ALL)
/** @} */

/* ========================================================================== */
/*                             Structure Definitions                          */
/* ========================================================================== */


/**
 * \brief Firewall Region Cfg specifies the MMR configuration for the specified
 *  firewall ID.
 */
typedef struct
{
    /** region Index of the firewall ID */
    uint16_t        regionIndex;
    /** Contents of the firewall CONTROL register */
	uint32_t		control;
    /** Contents of the firewall CONTROL register */
	uint32_t		permissions[FWL_MAX_PRIVID_SLOTS];
    /** Start Address of the Region */
	uint64_t        startAddr;
    /** End Address of the Region */
	uint64_t        endAddr;
} Firewall_RegionCfg;

/**
 * \brief Firewall atributes
 *
 * Firewall attributes are used within #Firewall_open().
 * The attributes are used to initialize Firewall ID MMR as
 * per the attributes
 *
 */
typedef struct {
    /* Id of Firewall to be used */
    uint32_t    firewallId;
    /* Number of regions in the firewall */
    uint32_t    totalRegions;
    /* Number of regions initialized */
    uint32_t    initRegions;
    /* Region configuration */
    Firewall_RegionCfg *regionInfo;
}Firewall_Attrs;


/**
 *  \brief A handle that is initialized in #Firewall_open() call
 */
typedef void *Firewall_Handle;

/**
 *  \brief Firewall driver Object
 */
typedef struct
{
    /** Firewall instance handle */
    Firewall_Handle handle;
    /** Flag for Firewall configuration done */
    uint32_t cfgDone;
} Firewall_Object;

/**
 * \brief Firewall Instance Configuration.
 *  Pointer to this object is returned as handle by driver open.
 */
typedef struct
{
    /** Pointer to driver specific attributes */
    Firewall_Attrs       *attrs;
    /**< Pointer to driver specific data object */
    Firewall_Object      *object;

} Firewall_Config;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief  This function initializes the firewall module
 */
void Firewall_init(void);

/**
 *  \brief  This function de-initializes the firewall module
 */
void Firewall_deinit(void);

/**
 * \brief This function opens a given Firewall peripheral
 *
 * \pre Firewall controlled has been initialized using #Firewall_init()
 *
 * \param handle #Firewall_Handle initialized in #Firewall_open()
 *
 * \return #SystemP_SUCCESS on success, #SystemP_FAILURE otherwise
 */
int32_t Firewall_open(Firewall_Handle handle);

/**
 * \brief Function to close Firewall peripheral specified by firewall handle
 *
 * \pre #Firewall_open() has to be called first
 *
 * \param handle #Firewall_Handle initialized in #Firewall_open()
 */
void Firewall_close(Firewall_Handle handle);

/**
 * \brief Configure firewall for a single region
 *
 * \param firewallId Firewall id of the region
 * \param region Firewall region configuration parameters
 *
 * \return #SystemP_SUCCESS on success, #SystemP_FAILURE otherwise
 */
int32_t Firewall_configureSingleRegion(uint32_t firewallId, Firewall_RegionCfg *region);

/**
 * \brief Intialize a firewall with multiple regions
 *
 * \param handle #Firewall_Handle initialized in #Firewall_open()
 *
 * \return #SystemP_SUCCESS on success, #SystemP_FAILURE otherwise
 */
int32_t Firewall_configureRegion(Firewall_Handle handle);



#ifdef __cplusplus
}
#endif

#endif /* #ifndef FIREWALL_H_ */

/** @} */