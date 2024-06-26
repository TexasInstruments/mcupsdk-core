/*
 *  Copyright (c) Texas Instruments Incorporated 2020
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

/*!
 * \file     enet_apputils.h
 *
 * \brief    This file contains the function prototypes of the Enet
 *           application utility functions used in the Enet examples.
 *
 * NOTE: This library is meant only for Enet examples. Customers are not
 * encouraged to use this layer as these are very specific to the examples
 * written and the API behaviour and signature can change at any time to
 * suit the examples.
 */

#ifndef ENET_APPUTILS_H_
#define ENET_APPUTILS_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <enet.h>
#include <enet_cfg.h>
#if defined (ENET_SOC_HOSTPORT_DMA_TYPE_UDMA)
#include <drivers/udma.h>
#endif
#include <include/core/enet_rm.h>
#include <include/per/cpsw.h>
#if (ENET_ENABLE_PER_ICSSG == 1)
#include <include/per/icssg.h>
#endif
#include <include/mod/cpsw_stats.h>
#include <include/mod/cpsw_macport.h>

#include <include/core/enet_dma.h>

#include "enet_ethutils.h"

#if (defined(SOC_AM64X) || defined(SOC_AM243X))
#include "enet_ioctlutils.h"
#include "enet_apputils_k3.h"
#include "enet_udmautils.h"
#endif

#if defined (SOC_AM273X) || defined(SOC_AWR294X) || defined (SOC_AM263X) || defined (SOC_AM263PX) || defined(SOC_AM261X)
#include "enet_ioctlutils.h"
#include "enet_cpdmautils.h"
#endif

#if defined(__KLOCWORK__)
#include <stdlib.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** \brief Size of an array */
#define ENETAPPUTILS_ROUND_UP(x, y)     ((((x) + ((y) - 1U)) / (y)) * (y))
#define ENETAPPUTILS_ALIGN(x)           (ENETAPPUTILS_ROUND_UP((x), 128U))

/* \brief Octets of payload per console row */
#define OCTETS_PER_ROW                  (16U)
/** \brief Max frame length */
#define ETH_MAX_FRAME_LEN               (1522U)

/* \brief Support macros for MMR lock/unlock functions */
#define MMR_KICK0_UNLOCK_VAL            (0x68EF3490U)
#define MMR_KICK1_UNLOCK_VAL            (0xD172BC5AU)
#define MMR_KICK_LOCK_VAL               (0x00000000U)

#define CSL_MMR_KICK_UNLOCKED_MASK          (0x00000001U)
#define CSL_MMR_KICK_UNLOCKED_SHIFT         (0x0U)

#define ENET_CTRL_RGMII_ID_SHIFT            (4U)
#define ENET_CTRL_RGMII_ID_INTTXDLY         (0U)
#define ENET_CTRL_RGMII_ID_NODELAY          (1U)

#define ENET_UTILS_MCU2_0_UART_INSTANCE     (2U)
#define ENET_MAX_NUM_MAC_PER_PHER           (3U)

#if defined(__KLOCWORK__)
#define EnetAppUtils_assert(cond)       do { if (!(cond)) abort(); } while (0)
#else
#    if ENET_CFG_IS_ON(TRACE_DISABLE_INFOSTRING)
#        define EnetAppUtils_assert(cond)                                     \
           (EnetAppUtils_assertLocal((bool) (cond), (const char *) " ",       \
                    (const char *) __FILE__, (int32_t) __LINE__))
#    else
#        define EnetAppUtils_assert(cond)                                     \
           (EnetAppUtils_assertLocal((bool) (cond), (const char *) # cond,    \
                    (const char *) __FILE__, (int32_t) __LINE__))

#    endif
#endif

#define ENET_UTILS_CACHELINE_SIZE        (ENETDMA_CACHELINE_ALIGNMENT)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 *  \brief  Lock/unlock enum
 */
typedef enum
{
    ENETAPPUTILS_LOCK_MMR = 0U,
    /**< lock control MMR */
    ENETAPPUTILS_UNLOCK_MMR,
    /**< unlock control MMR */
} EnetAppUtils_MmrLockState;

/**
 *  \brief CTRL MMR enum for MCU and Main domain
 */
typedef enum
{
    ENETAPPUTILS_MMR_LOCK0 = 0U,
    /**< control MMR lock 0*/
    ENETAPPUTILS_MMR_LOCK1,
    /**< control MMR lock 1*/
    ENETAPPUTILS_MMR_LOCK2,
    /**< control MMR lock 2*/
    ENETAPPUTILS_MMR_LOCK3,
    /**< control MMR lock 3*/
    ENETAPPUTILS_MMR_LOCK4,
    /**< control MMR lock 4*/
    ENETAPPUTILS_MMR_LOCK5,
    /**< control MMR lock 5*/
    ENETAPPUTILS_MMR_LOCK6,
    /**< control MMR lock 6*/
    ENETAPPUTILS_MMR_LOCK7,
    /**< control MMR lock 7*/
} EnetAppUtils_CtrlMmrType;

typedef struct EnetApp_HandleInfo_s
{
    Enet_Handle hEnet;
#if !(defined(SOC_AM273X) || defined(SOC_AWR294X) || defined (SOC_AM263X) || defined (SOC_AM263PX) || defined(SOC_AM261X))
    Udma_DrvHandle hUdmaDrv;
#endif
} EnetApp_HandleInfo;

typedef struct EnetApp_GetDmaHandleInArgs_s
{
    Enet_Type enetType;

    uint32_t instId;
    /*! Enet CPDMA event callback function - this function will be called when
     *  the registered packets are transmitted on TX channel */
    EnetDma_PktNotifyCb notifyCb;
    /*! Argument to be used for the callback routines (it should mean something
     *  to layer into which the callback calls) */
    void *cbArg;
} EnetApp_GetDmaHandleInArgs;


#if defined (ENET_SOC_HOSTPORT_DMA_TYPE_CPDMA)
typedef struct EnetApp_GetTxDmaHandleOutArgs_s
{
    /* TX channel handle */
    EnetDma_TxChHandle hTxCh;
    /* TX channel number */
    uint32_t txChNum;
    /* max numTxPkts for the channel */
    uint32_t maxNumTxPkts;
} EnetApp_GetTxDmaHandleOutArgs;


typedef struct EnetApp_GetRxDmaHandleOutArgs_s
{
    /* RX channel handle */
    EnetDma_RxChHandle hRxCh;
    /* RX channel number */
    uint32_t rxChNum;
    /* mac Address valid */
    uint32_t numValidMacAddress;
    /* MAC address. It's port's MAC address in Dual-MAC or
     * host port's MAC addres in Switch */
    uint8_t macAddr[ENET_MAX_NUM_MAC_PER_PHER][ENET_MAC_ADDR_LEN];
    /* max numTxPkts for the channel */
    uint32_t maxNumRxPkts;
} EnetApp_GetRxDmaHandleOutArgs;
#endif

#if defined (ENET_SOC_HOSTPORT_DMA_TYPE_UDMA)
typedef struct EnetApp_GetTxDmaHandleOutArgs_s
{
    Enet_Type enetType;

    uint32_t instId;
    /* TX channel handle */
    EnetDma_TxChHandle hTxCh;
    /* TX channel number */
    uint32_t txChNum;
    /* max numTxPkts for the channel */
    uint32_t maxNumTxPkts;
    /* ch uses global event */
    bool useGlobalEvt;
} EnetApp_GetTxDmaHandleOutArgs;

typedef struct EnetApp_GetRxDmaHandleOutArgs_s
{
    Enet_Type enetType;

    uint32_t instId;
    /* RX channel handle */
    EnetDma_RxChHandle hRxCh;
    /* RX start flow index */
    uint32_t rxFlowStartIdx;
    /* RX flow index */
    uint32_t rxFlowIdx;
    /* mac Address valid */
    uint32_t numValidMacAddress;
    /* MAC address. It's port's MAC address in Dual-MAC or
     * host port's MAC addres in Switch */
    uint8_t macAddr[ENET_MAX_NUM_MAC_PER_PHER][ENET_MAC_ADDR_LEN];
    /* max numTxPkts for the channel */
    uint32_t maxNumRxPkts;
    /* flow uses global event */
    bool useGlobalEvt;
    /* flow is default flow */
    bool useDefaultFlow;
    /*! UDMAP receive flow packet size based free buffer queue enable configuration
     * to be programmed into the rx_size_thresh_en field of the RFLOW_RFC register.
     * See the UDMAP section of the TRM for more information on this setting.
     * Configuration of the optional size thresholds when this configuration is
     * enabled is done by sending the @ref tisci_msg_rm_udmap_flow_size_thresh_cfg_req
     * message to System Firmware for the receive flow allocated by this request.
     * This parameter can be no greater than
     * @ref TISCI_MSG_VALUE_RM_UDMAP_RX_FLOW_SIZE_THRESH_MAX */
    uint8_t  sizeThreshEn;
    /* Channel id associated with the rx flow. Relevant only for ICSSG as for CPSW rx chIdx is
     * always 0
     */
    uint32_t chIdx;
} EnetApp_GetRxDmaHandleOutArgs;
#endif

typedef struct EnetApp_GetMacAddrOutArgs_s
{
    /* mac Address valid */
    uint32_t macAddressCnt;

    /* MAC address. It's port's MAC address in Dual-MAC or
     * host port's MAC address in Switch */
    uint8_t macAddr[ENET_MAX_NUM_MAC_PER_PHER][ENET_MAC_ADDR_LEN];
} EnetApp_GetMacAddrOutArgs;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/*!
 * \brief Perform cache writeback operation
 *
 * Performs cache write-back operation based on whether cache is coherent
 * or not. This internally uses the OSAL API.
 *
 * \param addr  Start address of the cache line/s
 * \param size  Size (in bytes) of the memory to be written back
 *
 */
void EnetAppUtils_cacheWb(const void *addr,
                          int32_t size);

/*!
 * \brief Perform cache invalidate operation
 *
 * Perform cache invalidate operation based on whether cache is
 * coherent or not. This internally uses the OSAL API.
 *
 * \param addr  Start address of the cache line/s
 * \param size  Size (in bytes) of the memory to invalidate
 *
 */
void EnetAppUtils_cacheInv(const void *addr,
                           int32_t size);

/*!
 * \brief Perform cache writeback and invalidate operation
 *
 * Performs cache write-back and invalidation operation based on whether
 * cache is coherent or not. This internally uses the OSAL API.
 *
 * \param addr  Start address of the cache line/s
 * \param size  Size (in bytes) of the memory to writeback and invalidate
 */
void EnetAppUtils_cacheWbInv(const void *addr,
                             int32_t size);

/*!
 * \brief Prints the va_list on UART or console.
 *
 * Print the passed va_list on UART or console.  This API is required by apps
 * implementing functions which originally take a variadic argument, hence
 * the body of such functions require helper APIs that takes va_list.
 */
void EnetAppUtils_vprint(const char *pcString,
                         va_list args);

/*!
 * \brief Prints the string on UART or console
 */
void EnetAppUtils_print(const char *pcString,
                        ...);

/*!
 * \brief Input utility used to accept a character from terminal
 */
char EnetAppUtils_getChar(void);

/*!
 * \brief Input utility used to accept a number from terminal
 */
int32_t EnetAppUtils_getNum(void);

/*!
 * \brief Input utility used to accept a hexa decimal number from terminal
 */
uint32_t EnetAppUtils_getHex(void);

/*!
 * \brief Rand number function
 */
uint32_t EnetAppUtils_randFxn(uint32_t min,
                              uint32_t max);

/*!
 * \brief Wait for emulator connection
 */
void EnetAppUtils_waitEmuConnect(void);

/*!
 * \brief Wait function utility. This uses timer created in EnetAppUtils_timerInit
 *        function for busy wait for app passed wait time.
 */
void EnetAppUtils_wait(uint32_t waitTime);

/**
 *  \brief Returns if print can be supported for a platform/build
 *
 *  \return TRUE if print can be supported for a platform. Else FALSE
 */
uint32_t EnetAppUtils_isPrintSupported(void);

#if (ENET_ENABLE_PER_ICSSG == 1)
/**
 * \brief Utils function for printing "non-zero" MAC port statistics for ICSSG
 */
void EnetAppUtils_printIcssgMacPortStats(IcssgStats_MacPort *st,
                                         bool printFixedCounters);

/**
 * \brief Utils function for printing "non-zero" ICSSG PA statistics.
 */
void EnetAppUtils_printIcssgPaStats(IcssgStats_Pa *st);
#endif

/**
 *  \brief Utils function for printing "non-zero" MAC port statistics for CPSW2G
 */
void EnetAppUtils_printMacPortStats2G(CpswStats_MacPort_2g *st);

/**
 *  \brief Utils function for printing "non-zero" Host port statistics for CPSW2G
 */
void EnetAppUtils_printHostPortStats2G(CpswStats_HostPort_2g *st);

/**
 *  \brief Utils function for printing "non-zero" MAC port statistics for CPSW9G
 */
void EnetAppUtils_printMacPortStats9G(CpswStats_MacPort_Ng *st);

/**
 *  \brief Utils function for printing "non-zero" Host port statistics for CPSW9G
 */
void EnetAppUtils_printHostPortStats9G(CpswStats_HostPort_Ng *st);

/**
 *  \brief Utils function for printing Ethernet frame with src MAC, dest MAC,
 *         VLAN type
 */
void EnetAppUtils_printFrame(EthFrame *frame,
                             uint32_t len);

/**
 *  \brief Utils function for printing MAC address
 */
void EnetAppUtils_printMacAddr(uint8_t macAddr[]);

/**
 *  \brief Validates the state of all packets in the given
 *         queue with the expectedState and changes it to
 *         newState.
 */
void EnetAppUtils_validatePacketState(EnetDma_PktQ *pQueue,
                                      uint32_t expectedState,
                                      uint32_t newState);

/**
 *  \brief lock/unlock the MCU MMR CNTR register
 */
EnetAppUtils_MmrLockState EnetAppUtils_mcuMmrCtrl(EnetAppUtils_CtrlMmrType mmrNum,
                                                  EnetAppUtils_MmrLockState lock);

/**
 *  \brief Lock/unlock the Main MMR CNTR register
 */
EnetAppUtils_MmrLockState EnetAppUtils_mainMmrCtrl(EnetAppUtils_CtrlMmrType mmrNum,
                                                   EnetAppUtils_MmrLockState lock);

/**
 *  \brief Set loopback configuration for RGMII mode
 */
void EnetAppUtils_setNoPhyCfgRgmii(EnetMacPort_Interface *interface,
                                   EnetPhy_Cfg *phyCfg);

/**
 *  \brief Set loopback configuration for RMII mode
 */
void EnetAppUtils_setNoPhyCfgRmii(EnetMacPort_Interface *interface,
                                   EnetPhy_Cfg *phyCfg);

/**
 *  \brief Set loopback configuration for SGMII mode
 */
void EnetAppUtils_setNoPhyCfgSgmii(EnetMacPort_Interface *interface,
                                   CpswMacPort_Cfg *macCfg,
                                   EnetPhy_Cfg *phyCfg);

/**
 *  \brief Utils assert function
 */
#if !defined(__KLOCWORK__)
void EnetAppUtils_assertLocal(bool condition,
                              const char *str,
                              const char *fileName,
                              int32_t lineNum);
#endif

int32_t EnetAppUtils_showRxFlowStats(EnetDma_RxChHandle hRxFlow);

int32_t EnetAppUtils_showTxChStats(EnetDma_TxChHandle hTxCh);

int32_t EnetAppUtils_showRxChStats(EnetDma_RxChHandle hRxCh);

/**
 *  \brief Utility function to init resource config structure part of Cpsw_Cfg
 */
void EnetAppUtils_initResourceConfig(Enet_Type enetType,
                                     uint32_t instId,
                                     uint32_t selfCoreId,
                                     EnetRm_ResCfg *resCfg);

/*!
 * \brief Converts given character in Hexadecimal to Decimal value
 *
 * \param hex  Hexadecimal character to be converted
 * \return decimal value if a valid hexadecimal character
 * \return -1 if invalid hexadecimal character
 */
int8_t  EnetAppUtils_hex2Num(char hex);

/*!
 * \brief Converts given MAC address in string format to byte array
 *
 * \param txt  Pointer to MAC address string
 * \param addr Pointer to array of size 6 bytes to store octets of the MAC address
 *
 * \return ENET_SOK if input is a valid MAC address format
 *         ENET_EFAIL if input is invalid MAC address format
 */
int32_t EnetAppUtils_macAddrAtoI(const char *txt, uint8_t *addr);

/*!
 * \brief Converts given IP address in string format to byte array
 *
 * \param txt  Pointer to IP address string
 * \param addr Pointer to array of size 4 bytes to store octets of the IPv4 address
 *
 * \return ENET_SOK if input is a valid IPv4 address format
 *         ENET_EFAIL if input is invalid IPv4 address format
 */
int32_t EnetAppUtils_ipAddrAtoI(const char* txt, uint8_t *addr);

/**
 *  \brief Utility function to enable CPSW module clocks via SCI_CLIENT
 */
void EnetAppUtils_enableClocks(Enet_Type enetType, uint32_t instId);

/**
 *  \brief Utility function to disable CPSW module clocks via SCI_CLIENT
 */
void EnetAppUtils_disableClocks(Enet_Type enetType, uint32_t instId);

int32_t EnetAppUtils_showRxFlowStats(EnetDma_RxChHandle hRxFlow);

int32_t EnetAppUtils_showTxChStats(EnetDma_TxChHandle hTxCh);

void EnetApp_getEnetInstInfo(uint32_t enetInstanceId, Enet_Type *enetType, uint32_t *instId);

void EnetApp_getEnetInstMacInfo(Enet_Type enetType, uint32_t instId,
                             Enet_MacPort macPortList[],   uint8_t *numMacPorts);
void     EnetApp_acquireHandleInfo(Enet_Type enetType, uint32_t instId,
                                   EnetApp_HandleInfo *handleInfo);

void     EnetApp_coreAttach(Enet_Type enetType, uint32_t instId,
                            uint32_t coreId,
                            EnetPer_AttachCoreOutArgs *attachInfo);

void     EnetApp_coreDetach(Enet_Type enetType, uint32_t instId,
                            uint32_t coreId,
                            uint32_t coreKey);

void     EnetApp_releaseHandleInfo(Enet_Type enetType, uint32_t instId);

bool EnetApp_isPortLinked(Enet_Handle hEnet);

void EnetApp_closeTxDma(uint32_t enetTxDmaChId,
                        Enet_Handle hEnet,
                        uint32_t coreKey,
                        uint32_t coreId,
                        EnetDma_PktQ *fqPktInfoQ,
                        EnetDma_PktQ *cqPktInfoQ);

void EnetApp_closeRxDma(uint32_t enetRxDmaChId,
                        Enet_Handle hEnet,
                        uint32_t coreKey,
                        uint32_t coreId,
                        EnetDma_PktQ *fqPktInfoQ,
                        EnetDma_PktQ *cqPktInfoQ);

void EnetApp_getMacAddress(uint32_t enetRxDmaChId,
                            EnetApp_GetMacAddrOutArgs *outArgs);

void EnetApp_getRxDmaHandle(uint32_t enetRxDmaChId,
                            const EnetApp_GetDmaHandleInArgs *inArgs,
                            EnetApp_GetRxDmaHandleOutArgs *outArgs);

void EnetApp_getTxDmaHandle(uint32_t enetTxDmaChId,
                            const EnetApp_GetDmaHandleInArgs *inArgs,
                            EnetApp_GetTxDmaHandleOutArgs *outArgs);

int32_t EnetAppUtils_addAllPortMcastMembership(Enet_Handle hEnet,
                                               uint8_t *mcastMacAddr);

int32_t EnetAppUtils_delAllPortMcastMembership(Enet_Handle hEnet,
                                               uint8_t *mcastMacAddr);

void EnetApp_getNonPtpRxDmaInfo(Enet_Type enetType,
                                uint32_t instId,
                                uint32_t nonPtpRxFlowId[],
                                uint8_t *nonPtpRxFlowNum);

void EnetApp_getNonPtpTxDmaInfo(Enet_Type enetType,
                                uint32_t instId,
                                uint32_t nonPtpTxFlowId[],
                                uint8_t *nonPtpTxFlowNum);

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */



#ifdef __cplusplus
}
#endif

#endif  /* ENET_APPUTILS_H_ */
