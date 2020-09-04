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

/**
 *  \defgroup DRV_PCIE_MODULE APIs for PCIE
 *  \ingroup DRV_MODULE
 *
 *  This module contains APIs to program and use the PCIE module. The APIs
 *  can be used by other drivers to get access to PCIE and also by
 *  applications
 *
 *  @{
 */

/**
 *  \file pcie.h
 *
 *  \brief PCIE Driver API/interface file.
 */

#ifndef PCIE_H_
#define PCIE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ========================================================================== */
/*                             Macros                                         */
/* ========================================================================== */

/**
 * \brief Maximum PCIe devices supported by the driver
*/
#define PCIE_MAX_PERIPHS    (4U)

/**
 * \brief Maximum PCIe MSI interrupts supported
 *
 */
#define PCIE_MAX_MSI_IRQ    (32U)

/**
 * \brief Maxmium number of MSIx interrupts supported
 *
 */
#define PCIE_MAX_MSIX_IRQ   (2048U)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/**
 * \brief These are the possible values for PCIe mode
 */
typedef enum
{
    PCIE_EP_MODE = 0,    /**< Required when setting the PCIe Mode to End Point using the @ref Pcie_setInterfaceMode function */
    PCIE_LEGACY_EP_MODE, /**< Required when setting the PCIe Mode to Legacy End Point using the @ref Pcie_setInterfaceMode function */
    PCIE_RC_MODE         /**< Required when setting the PCIe Mode to Root Complex using the @ref Pcie_setInterfaceMode function */
} Pcie_Mode;

/**
 * \brief Enumeration for PCIE generations
 */
typedef enum
{
    PCIE_GEN1 = 1,      /**< PCIe Gen1 for 2.5 GT/s speed */
    PCIE_GEN2 = 2,      /**< PCIe Gen2 for 5.0 GT/s speed */
    PCIE_GEN3 = 3,      /**< PCIe Gen3 for 8.0 GT/s speed */
} Pcie_Gen;

/**
 * \brief Driver handle returned by #Pcie_open() call
 */
typedef void *Pcie_Handle;

/**
 * \brief The Pcie_DeviceCfg is used to specify device level
 * configuration of the driver instance.
 */
typedef struct
{
    /** Device-dependant configuration base address. */
    void              *cfgBase;
    /** Base address of the "data" area (remote device memory) */
    void              *dataBase;
    /** Reserved part of real data area due to memory mapping. */
    uint32_t           dataReserved;
    /** Driver version-dependant device params. Not all HW will have these (put NULL). */
    void              *devParams;
} Pcie_DeviceCfgBaseAddr;

/**
 * \brief These are the possible values for Prefetch BAR configuration
 */
typedef enum
{
    PCIE_BAR_NON_PREF = 0,    /**< Non Prefetchable Region*/
    PCIE_BAR_PREF             /**< Prefetchable Region*/
} Pcie_BarPref;

/**
 * \brief These are the possible values for Type BAR configuration
 */
typedef enum
{
  PCIE_BAR_TYPE32 = 0,    /**< 32 bits BAR */
  PCIE_BAR_RSVD,          /**< Reserved */
  PCIE_BAR_TYPE64         /**< 64 bits BAR */
} Pcie_BarType;

/**
 * \brief These are the possible values for Memory BAR configuration
 */
typedef enum
{
    PCIE_BAR_MEM_MEM = 0,    /**< Memory BAR */
    PCIE_BAR_MEM_IO          /**< IO BAR */
} Pcie_BarMem;

/**
 * \brief  Enum to select PCIe ATU(Address translation unit) region
 * direction(Inbound or Outbound).
 * This enum is used while configuring inbound or outbound region.
 */
typedef enum Pcie_AtuRegionDir
{
    PCIE_ATU_REGION_DIR_OUTBOUND, /**< Select PCIe outbound region. */
    PCIE_ATU_REGION_DIR_INBOUND   /**< Select PCIe inbound region. */
} Pcie_AtuRegionDir;

/**
 * \brief  This enum is used to select PCIe TLP(Transaction layer packet) type
 * while configuring inbound or outbound region.
 */
typedef enum Pcie_TlpType
{
    PCIE_TLP_TYPE_MEM, /**< MEM type is selected while doing memory transfer */
    PCIE_TLP_TYPE_IO,  /**< IO type is selected while doing I/O transfer */
    PCIE_TLP_TYPE_CFG  /**< CFG type is selected while doing configuration
                         * access */
} Pcie_TlpType;

/**
 * \brief  Enum to select address or BAR match mode.
 */
typedef enum Pcie_AtuRegionMatchMode
{
    PCIE_ATU_REGION_MATCH_MODE_ADDR, /**< Inbound packets are filtered by address match mode */
    PCIE_ATU_REGION_MATCH_MODE_BAR   /**< Inbound packets are filtered by BAR match mode */
} Pcie_AtuRegionMatchMode;

/**
 * \brief Pcie_ObAtuCfg specifies the Outbound ATU configurations for PCIe
 */
typedef struct
{
    /** ATU region Index */
    uint32_t regionIndex;
    /** Transaction layer packet type \ref Pcie_TlpType */
    Pcie_TlpType tlpType;
    /** Lower 32bit of Base address */
    uint32_t lowerBaseAddr;
    /** Upper 32bit of Base address */
    uint32_t upperBaseAddr;
    /** Region window size */
    uint32_t regionWindowSize;
    /** Lower 32bit of Target address */
    uint32_t lowerTargetAddr;
    /** Upper 32bit of Target address */
    uint32_t upperTargetAddr;
} Pcie_ObAtuCfg;

/**
 * \brief Pcie_IbAtuCfg specifies the Inbound ATU configurations for PCIe
 */
typedef struct
{
    /** ATU region Index */
    uint32_t regionIndex;
    /** Transaction layer packet type \ref Pcie_TlpType */
    Pcie_TlpType tlpType;
    /** Lower 32bit of Base address */
    uint32_t lowerBaseAddr;
    /** Upper 32bit of Base address */
    uint32_t upperBaseAddr;
    /** Region window size */
    uint32_t regionWindowSize;
    /** Lower 32bit of Target address */
    uint32_t lowerTargetAddr;
    /** Upper 32bit of Target address */
    uint32_t upperTargetAddr;
    /** BAR aperture for the Inbound ATU */
    uint32_t barAperture;
    /** BAR config for the Inbound ATU */
    uint32_t barCfg;
} Pcie_IbAtuCfg;

/**
 * \brief Function pointer for the PCIe MSI ISR
 */
typedef void (*Pcie_MsiIsr)(void *arg, uint32_t msiData);

/**
 * \brief Function pointer for the PCIe MSIx ISR
 */
typedef void (*Pcie_MsixIsr)(void *arg, uint32_t msixData);

/**
 * \brief Pcie_RegisterMsiIsrParams specifies the parameters to
 * register an ISR for MSI
 */
typedef struct {
    /** ISR for MSI */
    Pcie_MsiIsr isr;
    /** Arguements for MSI ISR */
    void *arg;
    /** Interrupt number (Note: This is not the core interrupt number.
     *  This is the PCIe MSI interrupt number maintained independently
     * from core interrupt.
     * The range of interrupts is between 0 to \ref PCIE_MAX_MSI_IRQ - 1 )
     * This interrupt number should be the 5 MSB of the 32 bit MSI data
     * sent to RC from EP */
    uint32_t intNum;
} Pcie_RegisterMsiIsrParams;

/**
 * \brief Pcie_RegisterMsixIsrParams specifies the parameters to
 * register an ISR for MSIX
 */
typedef struct {
    /** ISR for MSIX */
    Pcie_MsixIsr isr;
    /** Arguements for MSIX ISR */
    void *arg;
    /** Interrupt number (Note: This is not the core interrupt number.
     *  This is the PCIe MSIX interrupt number maintained independently
     * from core interrupt.
     * The range of interrupts is between 0 to \ref PCIE_MAX_MSIX_IRQ - 1 )
     * This interrupt number should be the 5 MSB of the 32 bit MSIX data
     * sent to RC from EP */
    uint32_t intNum;
} Pcie_RegisterMsixIsrParams;

/**
 * \brief PCIe MSI Isr control structure
 */
typedef struct {
    /** PCIe MSI Isr */
    Pcie_MsiIsr isr[PCIE_MAX_MSI_IRQ];
    /** PCIe MSI Isr arguments*/
    void *isrArgs[PCIE_MAX_MSI_IRQ];
} Pcie_MsiIsrCtrl;

/**
 * \brief ISR and arguement list for MSIx
 */
typedef struct {
    /** PCIe MSIx Isr */
    Pcie_MsixIsr isr[PCIE_MAX_MSIX_IRQ];
    /** PCIe MSIx Isr arguments*/
    void *isrArgs[PCIE_MAX_MSIX_IRQ];
} Pcie_MsixIsrCtrl;

/**
 * \brief PCIe MSIx table entry
 */
typedef struct {
    /* Address to write to for MSIx interrupt */
	uintptr_t addr;
    /* Data to write to for MSIx interrupt */
	uint32_t data;
    /* Indentifies if a function is restricted from sending MSIx */
	uint32_t vector_ctrl;
} Pcie_MsixTblEntry;

typedef struct
{
    Pcie_MsixTblEntry tbl[PCIE_MAX_MSIX_IRQ];
} Pcie_MsixTbl;

/**
 * \brief PCIe atributes
 *
 * PCIe attributes are used within #Pcie_open().
 * The attributes are used to initialize PCIe device instance as
 * per the attributes
 *
 */
typedef struct
{
    /** PCIe device number */
    uint32_t deviceNum;
    /** PCIe operation mode (RC or EP) */
    Pcie_Mode operationMode;
    /** PCIe operation speed (GEN1, GEN2 or GEN3) */
    Pcie_Gen gen;
    /** Number of lanes for the instance */
    uint32_t numLanes;
    /** PCIe Outbound ATU config params */
    Pcie_ObAtuCfg *obAtu;
    /** Number of PCIe Outbound configurations */
    uint32_t obAtuNum;
    /** PCIe Inbound ATU config params */
    Pcie_IbAtuCfg *ibAtu;
    /** Number of PCIe Inbound configurations */
    uint32_t ibAtuNum;
    /** Global Event number for MSI */
    uint32_t msiGlobalEventNum;
    /** Ring number used for MSI */
    uint32_t msiRingNum;
    /** Core interrupt number for MSI */
    uint32_t msiIntNum;
    /** Flag to indicate MSI enable */
    uint32_t msiIrqEnableFlag;
    /** ISR list for MSI */
    Pcie_MsiIsrCtrl *msiIsrCtrl;
    /** Ring memory for Ring accelerator used for MSI */
    uint8_t *msiRingMem;
    /** Global Event number for MSIx */
    uint32_t msixGlobalEventNum;
    /** Ring number used for MSIx */
    uint32_t msixRingNum;
    /** Core interrupt number for MSIx */
    uint32_t msixIntNum;
    /** Flag to indicate MSIx enable */
    uint32_t msixIrqEnableFlag;
    /** EP MSIx interrupt table */
    Pcie_MsixTbl *epMsixTbl;
    /** ISR list for MSIx */
    Pcie_MsixIsrCtrl *msixIsrCtrl;
    /** Ring memory for Ring accelerator used for MSIx */
    uint8_t *msixRingMem;
} Pcie_Attrs;

/**
 * \brief PCIe driver object
 */
typedef struct
{
    /** PCIe instance handle */
    Pcie_Handle handle;
    /** PCIe instance base addresses */
    Pcie_DeviceCfgBaseAddr *bases;
    /** Flag for PCIe configuration done */
    uint32_t cfgDone;
} Pcie_Object;

/**
 * \brief PCIe device configuration.
 *
 * The structure contains the base addresses and parameters for
 * the PCIe instances
 *
 */
typedef struct
{
    /** PCIe base addreses for PCIe instances */
    Pcie_DeviceCfgBaseAddr *basesPtr[PCIE_MAX_PERIPHS];
} Pcie_DeviceCfg;

/**
 * \brief PCIe configuration for initalization
 *
 * The Pcie_InitCfg is used to specify the initialization
 * parameters for PCIe device instance
 */
typedef struct
{
    /** Device Configuration */
    Pcie_DeviceCfg dev;
} Pcie_InitCfg;


/**
 * \brief PCIE global configuration array
 *
 * This structure needs to be defined before calling #Pcie_init() and
 * must not be changed by user thereafter
 */
typedef struct
{
    Pcie_Object *object;
    Pcie_Attrs *attrs;
} Pcie_Config;

/**
 * \brief Enumeration for PCIe access type remote/local
 *
 * Selects whether to read/write local or remote PCIe registers.
 * PCIe configuration registers are accessible locally and remotely
 *
 * For PCIe application registers the access is always local
 */
typedef enum
{
    PCIE_LOCATION_LOCAL,     /**< Access the local PCIe peripheral */
    PCIE_LOCATION_REMOTE     /**< Access the remote PCIe peripheral */
} Pcie_Location;

/**
 * \brief Enumeration for possible values for encoding LTSSM state
 */
typedef enum
{
    PCIE_LTSSM_DETECT_QUIET=0,      /**< 0x00 */
    PCIE_LTSSM_DETECT_ACT,          /**< 0x01 */
    PCIE_LTSSM_POLL_ACTIVE,         /**< 0x02 */
    PCIE_LTSSM_POLL_COMPLIANCE,     /**< 0x03 */
    PCIE_LTSSM_POLL_CONFIG,         /**< 0x04 */
    PCIE_LTSSM_PRE_DETECT_QUIET,    /**< 0x05 */
    PCIE_LTSSM_DETECT_WAIT,         /**< 0x06 */
    PCIE_LTSSM_CFG_LINKWD_START,    /**< 0x07 */
    PCIE_LTSSM_CFG_LINKWD_ACEPT,    /**< 0x08 */
    PCIE_LTSSM_CFG_LANENUM_WAIT,    /**< 0x09 */
    PCIE_LTSSM_CFG_LANENUM_ACEPT,   /**< 0x0a */
    PCIE_LTSSM_CFG_COMPLETE,        /**< 0x0b */
    PCIE_LTSSM_CFG_IDLE,            /**< 0x0c */
    PCIE_LTSSM_RCVRY_LOCK,          /**< 0x0d */
    PCIE_LTSSM_RCVRY_SPEED,         /**< 0x0e */
    PCIE_LTSSM_RCVRY_RCVRCFG,       /**< 0x0f */
    PCIE_LTSSM_RCVRY_IDLE,          /**< 0x10 */
    PCIE_LTSSM_L0,                  /**< 0x11 */
    PCIE_LTSSM_L0S,                 /**< 0x12 */
    PCIE_LTSSM_L123_SEND_EIDLE,     /**< 0x13 */
    PCIE_LTSSM_L1_IDLE,             /**< 0x14 */
    PCIE_LTSSM_L2_IDLE,             /**< 0x15 */
    PCIE_LTSSM_L2_WAKE,             /**< 0x16 */
    PCIE_LTSSM_DISABLED_ENTRY,      /**< 0x17 */
    PCIE_LTSSM_DISABLED_IDLE,       /**< 0x18 */
    PCIE_LTSSM_DISABLED,            /**< 0x19 */
    PCIE_LTSSM_LPBK_ENTRY,          /**< 0x1a */
    PCIE_LTSSM_LPBK_ACTIVE,         /**< 0x1b */
    PCIE_LTSSM_LPBK_EXIT,           /**< 0x1c */
    PCIE_LTSSM_LPBK_EXIT_TIMEOUT,   /**< 0x1d */
    PCIE_LTSSM_HOT_RESET_ENTRY,     /**< 0x1e */
    PCIE_LTSSM_HOT_RESET,           /**< 0x1f */
    PCIE_LTSSM_RCVRY_EQ0,           /**< 0x20 */
    PCIE_LTSSM_RCVRY_EQ1,           /**< 0x21 */
    PCIE_LTSSM_RCVRY_EQ2,           /**< 0x22 */
    PCIE_LTSSM_RCVRY_EQ3            /**< 0x23 */
} Pcie_LtssmState;

/**
 * \brief PCIe BAR configuration info
 *
 * The pcieBarCfg is used to configure a 32bits BAR Register
 * or the lower 32bits of a 64bits BAR register. \n
 * This should NOT be used to configure BAR masks.\n
 * This should NOT be used to configure the Upper 32bits of
 * a 64bits BAR register.
 */
typedef struct Pcie_BarCfg_s {
    /** Local or remote peripheral */
    Pcie_Location location;
    /** PCIe mode */
    Pcie_Mode mode;
    /** Base Address (32bits) */
    uint32_t base;
    /** Prefetch */
    Pcie_BarPref prefetch;
    /** Type */
    Pcie_BarType type;
    /** Memory Space */
    Pcie_BarMem memSpace;
    /** BAR index */
    uint8_t idx;
    /** BARxC */
    uint8_t barxc;
    /** BARxA */
    uint8_t barxa;
} Pcie_BarCfg;

/**
 * \brief Inbound traslation configuration info
 * The Pcie_IbTransCfg is used to configure the Inbound Translation Registers
 */
typedef struct Pcie_IbTransCfg_s {
    /** Inbound Translation BAR match */
    uint8_t    ibBar;
    /** Low Inbound Start address (32bits) */
    uint32_t   ibStartAddrLo;
    /** High Inbound Start address (32bits) */
    uint32_t   ibStartAddrHi;
    /** Inbound Translation Address Offset (32bits) */
    uint32_t   ibOffsetAddr;
    /** Identifies the translation region (0-3) */
    uint8_t    region;
} Pcie_IbTransCfg;

/**
 *  \brief This Structure defines the ATU region parameters
 *
 *  These parameters are used for configuring inbound or outbound
 *  ATU(Address translation unit) region
 */
typedef struct Pcie_AtuRegionParams_s
{
    Pcie_AtuRegionDir       regionDir;
    /**< Region direction Inbound or Outbound
     * Values given by enum #Pcie_AtuRegionDir
     */
    Pcie_TlpType            tlpType;
    /**< TLP(transaction layer packet) type
     * Values given by enum #Pcie_TlpType
     */
    uint32_t                 enableRegion;
    /**< Region enable or disable */
    Pcie_AtuRegionMatchMode matchMode;
    /**< Region match mode Address match or BAR match
     * Values given by enum #Pcie_AtuRegionMatchMode
     */
    uint32_t                 barNumber;
    /**< BAR number with which the region is associated
     *   Possible values for EP : 0 to 5 for 32bit and 0 to 2 for 64bit
     *   Possible values for RC : 0 to 1 for 32bit and 0 for 64bit
     */
    uint32_t                 lowerBaseAddr;
    /**< Lower base address : should be 4K aligned
     *   For outbound configuration this contains outbound region offset
     *   For inbound  configuration this contains inbound PCIe start address
     */
    uint32_t                 upperBaseAddr;
    /**< Upper base address
     *   Higher 32 bits in case of 64 bit addressing
     *   Configured to 0 for 32 bit addressing
     */
    uint32_t                 regionWindowSize;
    /**< Region window size
     *   For outbound configuration this contains outbound window size
     *   For inbound  configuration this contains PCIe inbound window size
     */
    uint32_t                 lowerTargetAddr;
    /**< Lower Target address: should be 4K aligned
     *   For outbound configuration this contains outbound PCIe start offset
     *   For inbound  configuration this contains destination address
     */
    uint32_t                 upperTargetAddr;
    /**< Upper target address
     *   Higher 32 bits in case of 64 bit addressing
     *   Configured to 0 for 32 bit addressing
     */
} Pcie_AtuRegionParams;

/** \brief Externally defined driver configuration array */
extern Pcie_Config gPcieConfig[];
/** \brief Externally defined driver configuration array size */
extern uint32_t gPcieConfigNum;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 * \brief This function initializes the PCIe module
 */
void Pcie_init(void);

/**
 * \brief This function opens a given PCIe peripheral
 *
 * \pre PCIe controlled has been initialized using #Pcie_init()
 *
 * \param index     Index  of config to use in the *Pcie_config* array
 *
 * \return A #Pcie_Handle on success or a NULL on an error or if it has been
 *          opened already
 */
Pcie_Handle Pcie_open(uint32_t index);

/**
 * \brief Function to close PCIe peripheral specified by PCIe handle
 *
 * \pre #Pcie_open() has to be called first
 *
 * \param handle #Pcie_Handle retuned from #Pcie_open()
 */
void Pcie_close(Pcie_Handle handle);

/**
 * \brief Get the device base address info for the PCIe peripheral
 *
 * \param handle #Pcie_Handle returned from #Pcie_open()
 *
 * \return Pointer to the base address info structure if success, else returns NULL
 */
Pcie_DeviceCfgBaseAddr *Pcie_handleGetBases (Pcie_Handle handle);

/**
 * \brief Set interfac mode (RC/EP)
 *
 * \param handle Pcie_Handle for the instance
 * \param mode Interface mode (EP/EC)
 *
 * \return #SystemP_SUCCESS on successful; else error on failure
 */
int32_t Pcie_setInterfaceMode (Pcie_Handle handle, Pcie_Mode mode);

/**
 *  \brief  Pcie_getMemSpaceReserved returns amount of reserved space
 * between beginning of hardware's data area and the base returned
 * by @ref Pcie_getMemSpaceRange.
 *
 * \param handle Pcie_Handle for the instance
 * \param resSize Pointer to return reserved space
 *
 *  \retval  #SystemP_SUCCESS on successful; else error on failure
 */
int32_t Pcie_getMemSpaceReserved (Pcie_Handle handle, uint32_t *resSize);

/**
 * \brief Returns the PCIe Internal Address Range for the
 * memory space. This range is used for accessing memory.
 *
 * \param handle Pcie_Handle for the instance
 * \param base Pointer to return base address
 * \param size Pointer to return size
 *
 * \return #SystemP_SUCCESS on successful; else error on failure
 */
int32_t Pcie_getMemSpaceRange (Pcie_Handle handle, void **base, uint32_t *size);

/**
 * \brief Configure a BAR Register (32 bits)
 *
 * \param handle Pcie_Handle for the instance
 * \param barCfg Bar configuration parameters
 *
 * \return #SystemP_SUCCESS on successful; else error on failure
 */
int32_t Pcie_cfgBar (Pcie_Handle handle, const Pcie_BarCfg *barCfg);

/**
 * \brief Configure address translation registers
 *
 * \param handle Pcie_Handle for the instance
 * \param location Local or remote configuration space
 * \param atuRegionIndex Address translation region index
 * \param atuRegionParams Address translation region parameters
 *
 * \return #SystemP_SUCCESS on successful; else error on failure
 */
int32_t Pcie_atuRegionConfig (Pcie_Handle handle, Pcie_Location location,
            uint32_t atuRegionIndex, const Pcie_AtuRegionParams *atuRegionParams);

/**
 * \brief Get vendor ID and device ID of Pcie Device
 *
 * \param handle Pcie_Handle returned by #Pcie_open()
 * \param location Local or remote configuration space
 * \param vendorId Pointer to return vendor ID
 * \param deviceId Pointer to return device ID
 *
 * \return #SystemP_SUCCESS if successful; else error on failure
 */
int32_t Pcie_getVendorId(Pcie_Handle handle, Pcie_Location location,
                                        uint32_t *vendorId, uint32_t *deviceId);

/**
 * \brief Wait for PCIe link training to complete
 *
 * \param handle Pcie_Handle returned by #Pcie_open()
 *
 * \return #SystemP_SUCCESS if successful; else error on failure
 */
int32_t Pcie_waitLinkUp(Pcie_Handle handle);

/**
 * \brief Verify if the link parameters is established as configured
 *
 * \pre Link training needs to be completed
 *
 * \param handle Pcie_Handle returned by #Pcie_open()
 *
 * \return #SystemP_SUCCESS if successful; else error on failure
 */
int32_t Pcie_checkLinkParams(Pcie_Handle handle);

/**
 * \brief Enable/disable PCIe link training
 *
 * \param handle Pcie_Handle returned by #Pcie_open()
 * \param enable Enable(1) / disable (0) link training
 *
 * \return #SystemP_SUCCESS if successful; else error on failure
 */
int32_t Pcie_LtssmCtrl (Pcie_Handle handle, uint8_t enable);

/**
 * \brief Set number of PCIe lanes as configured
 *
 * \param handle Pcie_Handle returned by #Pcie_open()
 *
 * \return #SystemP_SUCCESS if successful; else error on failure
 */
int32_t Pcie_setLanes (Pcie_Handle handle);

/**
 * \brief Configure Pcie for EP (End Point) operation.
 * PCIe mode setting is NOT done here (\ref Pcie_setInterfaceMode)
 *
 * \param handle Pcie_Handle returned by #Pcie_open()
 *
 * \return #SystemP_SUCCESS if successful; else error on failure
 */
int32_t Pcie_cfgEP (Pcie_Handle handle);

/**
 * \brief Configure Pcie for RC (Root Complex) operation.
 * PCIe mode setting is NOT done here (\ref Pcie_setInterfaceMode)
 *
 * \param handle Pcie_Handle returned by #Pcie_open()
 *
 * \return #SystemP_SUCCESS if successful; else error on failure
 */
int32_t Pcie_cfgRC (Pcie_Handle handle);

#ifdef __cplusplus
}
#endif

#endif  /* PCIE_H_ */

/** @} */
