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

#ifndef CSIRX_V0_H_
#define CSIRX_V0_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <kernel/dpl/SystemP.h>
#include <kernel/dpl/HwiP.h>


/**
 *  \addtogroup DRV_CSIRX_DPHY_MODULE
 *
 *  @{
 */


/** @} */

/**
 *  \addtogroup DRV_CSIRX_COMPLEXIO_MODULE
 *
 *  @{
 */

/** \brief Max possible data lanes */
#define CSIRX_DATA_LANES_MAX    (4U)

/**
 *  \anchor CSIRX_COMPLEXIO_POWER_COMMAND
 *  \name Complex IO power commands
 *  @{
 */

/*! \brief Power off ComplexIO */
#define CSIRX_COMPLEXIO_POWER_COMMAND_OFF (0U)

/*! \brief Power on ComplexIO */
#define CSIRX_COMPLEXIO_POWER_COMMAND_ON  (1U)

/*! \brief Put ComplexIO in ULP (Ultra Low Power) state */
#define CSIRX_COMPLEXIO_POWER_COMMAND_ULP (2U)

/** @} */

/**
 *  \anchor CSIRX_COMPLEXIO_POWER_STATUS
 *  \name Complex IO power status
 *  @{
 */

/*! \brief Complex IO power status OFF */
#define CSIRX_COMPLEXIO_POWER_STATUS_OFF (0U)

/*! \brief Complex IO power status ON */
#define CSIRX_COMPLEXIO_POWER_STATUS_ON  (1U)

/*! \brief Complex IO power status ULP (Ultra Low Power) */
#define CSIRX_COMPLEXIO_POWER_STATUS_ULP (2U)

/** @} */

/**
 *  \anchor CSIRX_LANE_POLARITY
 *  \name Lane polarity
 *  @{
 */

/*! \brief Lane polarity +/- */
#define CSIRX_LANE_POLARITY_PLUS_MINUS  (0U)

/*! \brief Lane polarity -/+ */
#define CSIRX_LANE_POLARITY_MINUS_PLUS  (1U)

/** @} */

/**
 *  \anchor CSIRX_LANE_POSITION
 *  \name Lane position
 *  @{
 */

/*! \brief Lane not used */
#define CSIRX_LANE_POSITION_LANE_NOT_USED  (0U)

/*! \brief Physical lane position #1 */
#define CSIRX_LANE_POSITION_1              (1U)

/*! \brief Physical lane position #2 */
#define CSIRX_LANE_POSITION_2              (2U)

/*! \brief Physical lane position #3 */
#define CSIRX_LANE_POSITION_3              (3U)

/*! \brief Physical lane position #4 */
#define CSIRX_LANE_POSITION_4              (4U)

/*! \brief Physical lane position #5 */
#define CSIRX_LANE_POSITION_5              (5U)

/** @} */


/** @} */

/**
 *  \addtogroup DRV_CSIRX_COMMON_MODULE
 *
 *  @{
 */

/*! \brief Special value of \ref CSIRX_CommonConfig::stopStateFsmTimeoutInNanoSecs
       for setting maximum timeout, this is also the CSIRX IP reset state. */
#define CSIRX_STOP_STATE_FSM_TIMEOUT_MAX  (200000U)

/**
 *  \anchor CSIRX_ENDIANNESS
 *  \name Data endianess
 *  @{
 */

/*! \brief All little endian except legacy YUV422 8b and YUV420, which are
 *         big endian. */
#define CSIRX_ENDIANNESS_NATIVE_MIPI_CSI2 (0U)

/*! \brief All little endian */
#define CSIRX_ENDIANNESS_LITTLE_ENDIAN    (1U)

/*! \brief Endianness max for error checking purposes */
#define CSIRX_ENDIANNESS_MAX              (1U)

/** @}*/

/**
 *  \anchor CSIRX_BURST_SIZE
 *  \name Burst size defines
 *  @{
 */

/*! \brief Burst size 1x64 OCP writes */
#define CSIRX_BURST_SIZE_1X64   (0U)

/*! \brief Burst size 2x64 OCP writes */
#define CSIRX_BURST_SIZE_2X64   (1U)

/*! \brief Burst size 4x64 OCP writes */
#define CSIRX_BURST_SIZE_4X64   (2U)

/*! \brief Burst size 8x64 OCP writes */
#define CSIRX_BURST_SIZE_8X64   (3U)

/*! \brief Burst size max */
#define CSIRX_BURST_SIZE_MAX    (3U)

/** @} */


/** @} */

/**
 *  \addtogroup DRV_CSIRX_CONTEXT_MODULE
 *
 *  @{
 */

/** \brief Max possible CSIRX contexts */
#define CSIRX_CONTEXTS_MAX                (8U)

/*! \brief Ping-pong addresses and line offset alignment. Can be used for
       aligning ping-pong buffers and enforcing line offset alignment in bytes */
#define CSIRX_PING_PONG_ADDRESS_LINEOFFSET_ALIGNMENT_IN_BYTES (32U)

/*! \brief Special value for contiguous storage. */
#define CSIRX_LINEOFFSET_CONTIGUOUS_STORAGE  (0U)

/*! \brief Special value of \ref CSIRX_ContextConfig::numFramesToAcquire
       for setting infinite frames, this is also the CSIRX IP reset state. */
#define CSIRX_NUM_FRAMES_TO_ACQUIRE_INFINITE (0U)

/**
 *  \anchor CSIRX_PINGPONG_STATUS
 *  \name Ping or Pong done status
 *  @{
 */

/** \brief Ping is done */
#define CSIRX_CONTEXT_PINGPONG_STATUS_PING_DONE     (0U)
/** \brief Pong is done */
#define CSIRX_CONTEXT_PINGPONG_STATUS_PONG_DONE     (1U)

/** @} */

/**
 *  \anchor CSIRX_PING_PONG_SWITCHING_MODE
 *  \name Ping pong switching mode
 *  @{
 */

/** \brief Switch ping pong address on end of frame */
#define CSIRX_PING_PONG_FRAME_SWITCHING  (0U)

/** \brief Switch ping pong address on end of line */
#define CSIRX_PING_PONG_LINE_SWITCHING   (1U)

/** @} */

/**
 *  \anchor CSIRX_TRANSCODE_FORMAT
 *  \name Transcode format
 *  @{
 */

/*! \brief Transcoding disabled. */
#define CSIRX_TRANSCODE_FORMAT_NO_TRANSCODE     (0U)

/*! \brief Compress input RAW10 using A-law. Output is regular RAW8. */
#define CSIRX_TRANSCODE_FORMAT_IN_RAW10_ALAW_OUT_RAW8 (3U)

/*! \brief Regular RAW8 input and regular RAW8 output */
#define CSIRX_TRANSCODE_FORMAT_IN_RAW8_OUT_RAW8 (4U)

/*! \brief Input uncompressed RAW10, output RAW10 + EXP16 */
#define CSIRX_TRANSCODE_FORMAT_IN_RAW10_OUT_RAW10_EXP16 (5U)

/*! \brief Input uncompressed RAW10, output regular packed RAW10 */
#define CSIRX_TRANSCODE_FORMAT_IN_RAW10_OUT_RAW10_PACKED (6U)

/*! \brief Input uncompressed RAW12, output RAW12 + EXP16 */
#define CSIRX_TRANSCODE_FORMAT_IN_RAW12_OUT_RAW12_EXP16 (7U)

/*! \brief Input uncompressed RAW12, output regular packed RAW12 */
#define CSIRX_TRANSCODE_FORMAT_IN_RAW12_OUT_RAW12_PACKED (8U)

/*! \brief Input uncompressed RAW14, output uncompressed RAW14 */
#define CSIRX_TRANSCODE_FORMAT_IN_RAW14_OUT_RAW14 (9U)

/** @} */

/**
 *  \anchor CSIRX_DATA_FORMAT
 *  \name Data format
 *  @{
 */

/*! \brief Others except NULL and BLANKING */
#define CSIRX_FORMAT_OTHERS_EXCEPT_NULL_AND_BLANKING        (0x000U)

/*! \brief Embedded 8-bit non image (for example JPEG) as per MIPI spec */
#define CSIRX_FORMAT_EMBEDDED_8_BIT_NON_IMAGE               (0x012U)

/*! \brief YUV420 8-bit as per MIPI spec */
#define CSIRX_FORMAT_YUV420_8_BIT                           (0x018U)

/*! \brief YUV420 10-bit as per MIPI spec */
#define CSIRX_FORMAT_YUV420_10_BIT                          (0x019U)

/*! \brief YUV420 8-bit legacy as per MIPI spec */
#define CSIRX_FORMAT_YUV420_8_BIT_LEGACY                    (0x01AU)

/*! \brief YUV420 8-bit CSPS (Chroma Shifted Pixel Sampling) as per MIPI spec */
#define CSIRX_FORMAT_YUV420_8_BIT_CSPS                      (0x01CU)

/*! \brief YUV420 10-bit CSPS (Chroma Shifted Pixel Sampling)  as per MIPI spec */
#define CSIRX_FORMAT_YUV420_10_BIT_CSPS                     (0x01DU)

/*! \brief YUV422 8-bit as per MIPI spec */
#define CSIRX_FORMAT_YUV422_8_BIT                           (0x01EU)

/*! \brief YUV422 10-bit as per MIPI spec */
#define CSIRX_FORMAT_YUV422_10_BIT                          (0x01FU)

/*! \brief RGB565 as per MIPI spec */
#define CSIRX_FORMAT_RGB565                                 (0x022U)

/*! \brief RGB888 as per MIPI spec */
#define CSIRX_FORMAT_RGB888                                 (0x024U)

/*! \brief RAW6 as per MIPI spec */
#define CSIRX_FORMAT_RAW6                                   (0x028U)

/*! \brief RAW7 as per MIPI spec */
#define CSIRX_FORMAT_RAW7                                   (0x029U)

/*! \brief RAW8 as per MIPI spec */
#define CSIRX_FORMAT_RAW8                                   (0x02AU)

/*! \brief RAW19 as per MIPI spec */
#define CSIRX_FORMAT_RAW10                                  (0x02BU)

/*! \brief RAW12 as per MIPI spec */
#define CSIRX_FORMAT_RAW12                                  (0x02CU)

/*! \brief RAW14 as per MIPI spec */
#define CSIRX_FORMAT_RAW14                                  (0x02DU)

/*! \brief RGB666 EXP32_24: EXP32 = Data expansion to 32 bits,
       padding with \ref CSIRX_ContextConfig::alpha. Most significant
       8-bits of output 32-bit will be equal to the least significant 8-bits of
       \ref CSIRX_ContextConfig::alpha */
#define CSIRX_FORMAT_RGB666_EXP32_24                        (0x033U)

/*! \brief User Defined 8-bit data type 1 */
#define CSIRX_FORMAT_USER_DEFINED_8_BIT_DATA_TYPE_1         (0x040U)

/*! \brief User Defined 8-bit data type 2 */
#define CSIRX_FORMAT_USER_DEFINED_8_BIT_DATA_TYPE_2         (0x041U)

/*! \brief User Defined 8-bit data type 3 */
#define CSIRX_FORMAT_USER_DEFINED_8_BIT_DATA_TYPE_3         (0x042U)

/*! \brief User Defined 8-bit data type 4 */
#define CSIRX_FORMAT_USER_DEFINED_8_BIT_DATA_TYPE_4         (0x043U)

/*! \brief User Defined 8-bit data type 5 */
#define CSIRX_FORMAT_USER_DEFINED_8_BIT_DATA_TYPE_5         (0x044U)

/*! \brief User Defined 8-bit data type 6 */
#define CSIRX_FORMAT_USER_DEFINED_8_BIT_DATA_TYPE_6         (0x045U)

/*! \brief User Defined 8-bit data type 7 */
#define CSIRX_FORMAT_USER_DEFINED_8_BIT_DATA_TYPE_7         (0x046U)

/*! \brief User Defined 8-bit data type 8 */
#define CSIRX_FORMAT_USER_DEFINED_8_BIT_DATA_TYPE_8         (0x047U)

/*! \brief RAW6 with data expansion to 8-bits, padding with zeros */
#define CSIRX_FORMAT_RAW6_EXP8                              (0x068U)

/*! \brief RAW7 with data expansion to 8-bits, padding with zeros */
#define CSIRX_FORMAT_RAW7_EXP8                              (0x069U)

/*! \brief User Defined 8-bit data type 1 with data expansion to 8-bits,
       padding with zeros */
#define CSIRX_FORMAT_USER_DEFINED_8_BIT_DATA_TYPE_1_EXP8    (0x080U)

/*! \brief User Defined 8-bit data type 2 with data expansion to 8-bits,
       padding with zeros */
#define CSIRX_FORMAT_USER_DEFINED_8_BIT_DATA_TYPE_2_EXP8    (0x081U)

/*! \brief User Defined 8-bit data type 3 with data expansion to 8-bits,
       padding with zeros */
#define CSIRX_FORMAT_USER_DEFINED_8_BIT_DATA_TYPE_3_EXP8    (0x082U)

/*! \brief User Defined 8-bit data type 4 with data expansion to 8-bits,
       padding with zeros */
#define CSIRX_FORMAT_USER_DEFINED_8_BIT_DATA_TYPE_4_EXP8    (0x083U)

/*! \brief User Defined 8-bit data type 5 with data expansion to 8-bits,
       padding with zeros */
#define CSIRX_FORMAT_USER_DEFINED_8_BIT_DATA_TYPE_5_EXP8    (0x084U)

/*! \brief User Defined 8-bit data type 6 with data expansion to 8-bits,
       padding with zeros */
#define CSIRX_FORMAT_USER_DEFINED_8_BIT_DATA_TYPE_6_EXP8    (0x085U)

/*! \brief User Defined 8-bit data type 7 with data expansion to 8-bits,
       padding with zeros */
#define CSIRX_FORMAT_USER_DEFINED_8_BIT_DATA_TYPE_7_EXP8    (0x086U)

/*! \brief User Defined 8-bit data type 8 with data expansion to 8-bits,
       padding with zeros */
#define CSIRX_FORMAT_USER_DEFINED_8_BIT_DATA_TYPE_8_EXP8    (0x087U)

/*! \brief RGB444 with data expansion to 16-bits. Most significant
       4-bits of 16-bit output will be equal to the least significant 4-bits of
       \ref CSIRX_ContextConfig::alpha */
#define CSIRX_FORMAT_RGB444_EXP16                           (0x0A0U)

/*! \brief RGB555 with data expansion to 16-bits. Most significant 1-bit of
       16-bit output will be equal to the least significant 1-bit of
       \ref CSIRX_ContextConfig::alpha */
#define CSIRX_FORMAT_RGB555_EXP16                           (0x0A1U)

/*! \brief RAW10 with data expansion to 16-bits. Most significant
       6-bits of 16-bit output will be equal to the least significant 6-bits of
       \ref CSIRX_ContextConfig::alpha */
#define CSIRX_FORMAT_RAW10_EXP16                            (0x0ABU)

/*! \brief RAW12 with data expansion to 16-bits. Most significant
       4-bits of 16-bit output will be equal to the least significant 4-bits of
       \ref CSIRX_ContextConfig::alpha */
#define CSIRX_FORMAT_RAW12_EXP16                            (0x0ACU)

/*! \brief RAW14 with data expansion to 16-bits. Most significant
       2-bits of 16-bit output will be equal to the least significant 2-bits of
       \ref CSIRX_ContextConfig::alpha */
#define CSIRX_FORMAT_RAW14_EXP16                            (0x0ADU)

/*! \brief RGB666 with data expansion to 32-bits. Most significant
       14-bits of 32-bit output will be equal to the least significant 14-bits of
       \ref CSIRX_ContextConfig::alpha */
#define CSIRX_FORMAT_RGB666_EXP32                           (0x0E3U)

/*! \brief RGB888 with data expansion to 32-bits. Most significant
       8-bits of 32-bit output will be equal to the least significant 8-bits of
       \ref CSIRX_ContextConfig::alpha */
#define CSIRX_FORMAT_RGB888_EXP32                           (0x0E4U)

/** @} */

/**
 *  \anchor CSIRX_USER_DEFINED_FORMAT
 *  \name User defined data format
 *  @{
 */

/*! \brief RAW6 */
#define CSIRX_USER_DEFINED_FORMAT_RAW6  (0U)

/*! \brief RAW8 */
#define CSIRX_USER_DEFINED_FORMAT_RAW7  (1U)

/*! \brief RAW8, not valid if \ref CSIRX_ContextConfig::format is any of
       the defines CSIRX_FORMAT_USER_DEFINED_8_BIT_DATA_TYPE_x_EXP8 (x=1..8)
       in \ref CSIRX_DATA_FORMAT */
#define CSIRX_USER_DEFINED_FORMAT_RAW8  (2U)

/** @} */


/** @} */

/**
 *  \addtogroup DRV_CSIRX_INSTANCE_MODULE
 *
 *  @{
 */

/*! \brief Interrupt number ID to indicate, this interrupt line is not connected in this CPU */
#define CSIRX_INTERRUPT_NOT_CONNECTED_ID        (0xFFFFU)

/**
 *  \anchor CSIRX_FIFODEPTH
 *  \name FIFO depth
 *  @{
 */

/*! \brief FIFO depth is 8 times 68-bits */
#define CSIRX_FIFODEPTH_8X68   (2U)

/*! \brief FIFO depth is 16 times 68-bits */
#define CSIRX_FIFODEPTH_16X68  (3U)

/*! \brief FIFO depth is 32 times 68-bits */
#define CSIRX_FIFODEPTH_32X68  (4U)

/*! \brief FIFO depth is 64 times 68-bits */
#define CSIRX_FIFODEPTH_64X68  (5U)

/*! \brief FIFO depth is 128 times 68-bits */
#define CSIRX_FIFODEPTH_128X68 (6U)

/*! \brief FIFO depth is 256 times 68-bits */
#define CSIRX_FIFODEPTH_256X68 (7U)

/** @} */

/** \brief CSIRX driver handle */
typedef struct CSIRX_Config_s      *CSIRX_Handle;

/** @} */

/**
 *  \addtogroup DRV_CSIRX_COMMON_MODULE
 *
 *  @{
 */

/*! \brief Common (across contexts) interrupts.

       This represents both status and configuration
       (enable/disable) structures. The description of the fields is in terms of
       status. For configuration they should be interpreted as "true if you want
       to enable the detection of this condition, false otherwise".

       For interrupt status, the interrupts can be cleared by issuing
       \ref CSIRX_commonClearAllIntr API
  */
struct CSIRX_CommonIntr_s;

/**
 * \brief Common interrupt callback
 *
 * The driver clear the pending interrupts internally so callback need not clear it
 *
 * \param handle [in] CSIRX driver handle
 * \param arg [in] User supplied argument
 * \param irq [in] Interrupt status, see \ref CSIRX_CommonIntr
 */
typedef void (*CSIRX_CommonCallback)(CSIRX_Handle handle, void *arg, struct CSIRX_CommonIntr_s *irq);

/**
 * \brief Generic Interrupt callback
 *
 * \param handle [in] CSIRX driver handle
 * \param arg [in] User supplied argument
 */
typedef void (*CSIRX_Callback)(CSIRX_Handle handle, void *arg);

/** @} */

/**
 *  \addtogroup DRV_CSIRX_CONTEXT_MODULE
 *
 *  @{
 */

/**
 * \brief Context interrupt callback
 *
 * \param handle [in] CSIRX driver handle
 * \param arg [in] User supplied argument
 * \param contextId [in] Context ID which generated this interrupt
 */
typedef void (*CSIRX_ContextCallback)(CSIRX_Handle handle, void *arg, uint8_t contextId);

/** @} */

/**
 *  \addtogroup DRV_CSIRX_DPHY_MODULE
 *
 *  @{
 */

/** \brief DPHY configuration */
typedef struct CSIRX_DphyConfig_s
{
    /** \brief DDR clock speed in Hz */
    uint32_t ddrClockInHz;

    /** \brief Set to true if wanting to enable clock missing detector */
    bool     isClockMissingDetectionEnabled;

    /** \brief Trigger escape codes */
    uint8_t  triggerEscapeCode[4];
} CSIRX_DphyConfig;

/** @} */

/**
 *  \addtogroup DRV_CSIRX_COMPLEXIO_MODULE
 *
 *  @{
 */

/*! \brief Lane configuration */
typedef struct CSIRX_ComplexioLaneConfig_s
{
    /*! \brief polarity, one of \ref CSIRX_LANE_POLARITY */
    uint8_t polarity;

    /*! \brief position, one of \ref CSIRX_LANE_POSITION */
    uint8_t position;
} CSIRX_ComplexioLaneConfig;

/*! \brief Complex IO per lane interrupt

    This represents both status and configuration
    structures. The description of the fields is in terms of status. For
    configuration they should be interpreted as "true if you want to enable
    the detection of this condition, false otherwise".
 */
typedef struct CSIRX_ComplexioLaneIntr_s
{
    /*! \brief true if lane transitioned to ULPM (Ultra Low Power Mode), else
           false */
    bool isStateTransitionToULPM;

    /*! \brief true if control error happened, else false */
    bool isControlError;

    /*! \brief true if escape entry error happened, else false */
    bool isEscapeEntryError;

    /*! \brief true if Start of Transmission (SOT) Synchronization error
           happened, else false */
    bool isStartOfTransmissionSyncError;

    /*! \brief true if Start of Transmission (SOT) error happened, else false */
    bool isStartOfTransmissionError;
} CSIRX_ComplexioLaneIntr;

/*! \brief Complex IO all (logical) lanes interrupts.

    This represents both status
    and configuration (enable/disable) structures. The description of the
    fields is in terms of status. For configuration they should be interpreted
    as "true if you want to enable the detection of this condition, false
    otherwise". For status, the interrupts can be cleared by issuing
    \ref CSIRX_complexioClearAllIntr API
  */
typedef struct CSIRX_ComplexioLanesIntr_s
{
    /*! \brief true if all lanes transitioned to ULPM (Ultra Low Power Mode) */
    bool isAllLanesEnterULPM;

    /*! \brief true if at least one of the active lanes has exit
           ULPM (Ultra Low Power Mode) */
    bool isAllLanesExitULPM;

    /*! \brief data lanes interrupt configuration */
    CSIRX_ComplexioLaneIntr dataLane[CSIRX_DATA_LANES_MAX];

    /*! \brief clock lane interrupt configuration */
    CSIRX_ComplexioLaneIntr clockLane;
} CSIRX_ComplexioLanesIntr;

/*! \brief All lanes configuration */
typedef struct CSIRX_ComplexioLanesConfig_s
{
    /*! \brief data lanes configuration */
    CSIRX_ComplexioLaneConfig dataLane[CSIRX_DATA_LANES_MAX];

    /*! \brief clock lanes configuration */
    CSIRX_ComplexioLaneConfig clockLane;
} CSIRX_ComplexioLanesConfig;

/*! \brief Complex IO configuration */
typedef struct CSIRX_ComplexioConfig
{
    /*! \brief lanes configuration */
    CSIRX_ComplexioLanesConfig lanesConfig;

    /*! \brief enable/disable lanes interrupts */
    CSIRX_ComplexioLanesIntr   enableIntr;

    /*! \brief If true, automatically switches between ULP and ON states based on
           ULPM signals from complex IO */
    bool isPowerAuto;
} CSIRX_ComplexioConfig;

/** @} */

/**
 *  \addtogroup DRV_CSIRX_COMMON_MODULE
 *
 *  @{
 */

/*! \brief Common (across contexts) interrupts.

       This represents both status and configuration
       (enable/disable) structures. The description of the fields is in terms of
       status. For configuration they should be interpreted as "true if you want
       to enable the detection of this condition, false otherwise".

       For interrupt status, the interrupts can be cleared by issuing
       \ref CSIRX_commonClearAllIntr API
  */
typedef struct CSIRX_CommonIntr_s
{
    /*! \brief reserved, keep this as 0 */
    bool isOcpError;

    /*! \brief true if short packet was received other than the MIPI sync events: \n
           Frame Start Code (0x0) \n
           Frame End   Code (0x1) \n
           Line  Start Code (0x2) \n
           Line  End   Code (0x3) \n
           Sync codes 0x4 to 0x7 are reserved by MIPI. Therefore, data type between
           0x8 and x0F only will be applicable to this. */
    bool isGenericShortPacketReceive;

    /*! \brief if true, ECC has been used to do the correction of
           1-bit error (short packet only). Influenced by
           \ref CSIRX_CommonConfig::isHeaderErrorCheckEnabled. */
    bool isOneBitShortPacketErrorCorrect;

    /*! \brief if true, more than 1-bit error that cannot be ECC corrected and was
           detected in the short packet or long packet header. Influenced by
           \ref CSIRX_CommonConfig::isHeaderErrorCheckEnabled. */
    bool isMoreThanOneBitShortPacketErrorCannotCorrect;

    /*! \brief if true, one or more of complex IO errors defined
           in \ref CSIRX_ComplexioLanesIntr happened. They can be queried using
           \ref CSIRX_complexioGetPendingIntr API. */
    bool isComplexioError;

    /*! \brief if true, it indicates data input rate is higher than the data
           output rate resulting in the receive FIFO overflowing. In case of an
           overflow, the module properly finishes the burst that has been started
           and doesn't issue any new OCP transactions on the master port.
           A reset of the module is required
           to restart correctly */
    bool isFifoOverflow;

    /*! \brief if any of the context entries is true, one or more of context interrupts
           defined in \ref CSIRX_ContextIntr happened for the respective context.
           The IRQs for a specific context can be queried using
           \ref CSIRX_contextGetPendingIntr API and they can be cleared using
           \ref CSIRX_contextClearAllIntr API  */
    bool isContextIntr[CSIRX_CONTEXTS_MAX];
} CSIRX_CommonIntr;

/**
 * \brief Callbacks and callback arguments for various common interrupt event
 *
 * Context specific callback be registered using \ref CSIRX_ContextConfig
 */
typedef struct CSIRX_CommonIntrCallbacks_s
{
    /*!  \brief Combined End of Line call back function definition.
     *
     *    This is called when
     *    one or more context whose \ref CSIRX_ContextConfig::isEndOfLinePulseEnabled
     *    is true(enabled) triggers the end of line pulse
     */
    CSIRX_Callback combinedEndOfLineCallback;

    /*! \brief Argument for CSIRX_CommonIntrCallbacks::combinedEndOfLineCallback */
    void *combinedEndOfLineCallbackArgs;

    /*!
     *  \brief  Combined End of Frame call back function definition.
     *
     *          This is called when
     *          one or more context whose \ref CSIRX_ContextConfig::isEndOfFramePulseEnabled
     *          is true(enaled) triggers the end of frame pulse
     */
    CSIRX_Callback combinedEndOfFrameCallback;

    /*! \brief Argument for CSIRX_CommonIntrCallbacks::combinedEndOfFrameCallback */
    void *combinedEndOfFrameCallbackArgs;

   /*!
    *  \brief  Common interrupt call back function definition.
    *
    *          This is called when any of the
    *          conditions in \ref CSIRX_CommonIntr are enabled and they happen.
    *          This API only provides the common interrupt status, the one associated with
    *          the physical interrupt line. User in their supplied call back function can do
    *          further parsing of common interrupt status to then query for example the
    *          complex IO and context interrupt using the following APIs: \n
    *          \ref CSIRX_complexioGetPendingIntr, \ref CSIRX_contextGetPendingIntr \n
    *          User must also clear the above ones using:
    *          \ref CSIRX_complexioClearAllIntr, \ref CSIRX_contextClearAllIntr \n
    *          Note the Isr inside the driver has responsibility of clearing all
    *          of the common interrupts. However, separate public APIs to query
    *          (\ref CSIRX_commonGetPendingIntr) and clear (\ref CSIRX_commonClearAllIntr)
    *          common interrupts are provided when user gives this call back function
    *          to be NULL, indicating a desire to poll the common interrupt.
    */
    CSIRX_CommonCallback commonCallback;

    /*! \brief Argument for CSIRX_CommonIntrCallbacks::commonCallback */
    void *commonCallbackArgs;

    /*!
     *  \brief  Start of Frame (SOF) Interrupt 0 call back function definition.
     *
     *          This is
     *          called when the context for which SOF Interrupt 0 is to be generated gets a
     *          SOF packet. Note this is independent of the
     *          \ref CSIRX_ContextIntr::isFrameStartCodeDetect configuration.
     */
    CSIRX_ContextCallback startOfFrameIntr0Callback;

    /*! \brief Argument for CSIRX_CommonIntrCallbacks::startOfFrameIntr0Callback */
    void *startOfFrameIntr0CallbackArgs;

    /*!
     *  \brief  Start of Frame (SOF) Interrupt 1 call back function definition.
     *
     *          This is
     *          called when the context for which SOF Interrupt 1 is to be generated gets a
     *          SOF packet. Note this is independent of the
     *          \ref CSIRX_ContextIntr::isFrameStartCodeDetect configuration.
     */
    CSIRX_ContextCallback startOfFrameIntr1Callback;

    /*! \brief Argument for CSIRX_CommonIntrCallbacks::startOfFrameIntr1Callback */
    void *startOfFrameIntr1CallbackArgs;

} CSIRX_CommonIntrCallbacks;

/**
 * \brief Common to all context configuration
 */
typedef struct CSIRX_CommonConfig_s {

    /*! \brief If true, when \ref CSIRX_commonDisable  is issued,
           the interface stops after full frames are received in all active
           contexts */
    bool isSoftStoppingOnInterfaceDisable;

    /*! \brief If true, enables the Error Correction Code check for the
           received header (short and long packets for all virtual channel ids).
           Influences \ref CSIRX_CommonIntr::isOneBitShortPacketErrorCorrect
           and \ref CSIRX_CommonIntr::isMoreThanOneBitShortPacketErrorCannotCorrect */
    bool isHeaderErrorCheckEnabled;

    /*! \brief If true, enables sign extension of RAW10/12/14 for all contexts
           whose \ref CSIRX_ContextConfig::format is among those with EXP16
           output */
    bool isSignExtensionEnabled;

    /*! \brief If false, then burst size is determined by \ref CSIRX_CommonConfig::burstSize,
           otherwise (if true), the burst size is 16x64 OCP writes */
    bool isBurstSizeExpand;

    /*! \brief If true, writes are non posted */
    bool isNonPostedWrites;

    /*! \brief If true, automatic OCP clock gatingbased on OCP activity is enabled.
           If false, OCP clock is free running */
    bool isOcpAutoIdle;

    /*! \brief stop state FSM timeout in Nano seconds. The maximum timeout possible
           is equal to 1/CSIRX_INTERCONNECT_CLOCK_HZ * 8191 * 16 * 4 seconds.
           e.g for CSIRX_INTERCONNECT_CLOCK_HZ = 200 MHz, max timeout is 2.621 ms
           For setting the maximum timeout possible, a special
           define \ref CSIRX_STOP_STATE_FSM_TIMEOUT_MAX is provided. This is
           also the CSIRX IP reset default */
    uint32_t stopStateFsmTimeoutInNanoSecs;

    /*! \brief Sets the DMA burst size on the interconnect to one of
           \ref CSIRX_BURST_SIZE. Only effective if \ref isBurstSizeExpand is
           false */
    uint8_t burstSize;

    /*! \brief One of \ref CSIRX_ENDIANNESS. */
    uint8_t endianness;

    /*! \brief Context Id for generation of SOF Interrupt 0 */
    uint8_t startOfFrameIntr0ContextId;

    /*! \brief Context Id for generation of SOF Interrupt 1 */
    uint8_t startOfFrameIntr1ContextId;

    /*! \brief Context Id for generation of EOF Interrupt 0 */
    uint8_t endOfFrameIntr0ContextId;

    /*! \brief Context Id for generation of EOF Interrupt 1 */
    uint8_t endOfFrameIntr1ContextId;

    /*! \brief Common interrupt to enable or disable */
    CSIRX_CommonIntr enableIntr;

    /*! \brief Common interrupt callbacks */
    CSIRX_CommonIntrCallbacks intrCallbacks;

} CSIRX_CommonConfig;

/** @} */

/**
 *  \addtogroup DRV_CSIRX_CONTEXT_MODULE
 *
 *  @{
 */

/*! \brief ping-pong related configuration for each context. Note specified ping
       and pong addresses can be identical which will effectively disable double
       buffering. */
typedef struct CSIRX_ContextPingPongConfig_s
{
    /*! \brief ping address in which the frame/line data is received,
           must have 5 LSBs 0. User must provision for sufficient size to cover
           multiple lines/frames depending on configuration */
    uint32_t pingAddress;

    /*! \brief pong address in which the frame/line data is received,
           must have 5 LSBs 0. User must provision for sufficient size to cover
           multiple lines/frames depending on configuration */
    uint32_t pongAddress;

    /*! \brief line offset as defined in the TRM (CSI2_CTX_DAT_OFST::OFST). Special value
           \ref CSIRX_LINEOFFSET_CONTIGUOUS_STORAGE for contiguous storage, otherwise
           line offset sets the destination offset between the first pixel of
           the previous line and the first pixel of the current line. Units in
           bytes. This is a 17-bit signed number (line offset
           can be negative) and must must have 5 LSBs as 0 */
    int32_t  lineOffset;

    /*! \brief ping-pong switching mode, one of \ref CSIRX_PING_PONG_SWITCHING_MODE.
           If line based, ping-pong switch happens every
           \ref CSIRX_ContextPingPongConfig::numLinesForLineBasedPingPongSwitching lines.
           If frame based,ping-pong switch happens every
           \ref CSIRX_ContextPingPongConfig::numFramesForFrameBasedPingPongSwitching */
    uint8_t  pingPongSwitchMode;

    /*! \brief number of frames for frame based ping-pong switching. Typically
           used in interlaced mode (=2). For progressive mode is set to 1 */
    uint8_t  numFramesForFrameBasedPingPongSwitching;

    /*! \brief number of lines for line based ping-pong switching. Typically used
           in radar for multi-chirp (when chirp is a line) processing */
    uint16_t numLinesForLineBasedPingPongSwitching;
} CSIRX_ContextPingPongConfig;

/*! \brief Cropping configuration */
typedef struct CSIRX_ContextCropConfig_s
{

    /*! \brief 13-bit field that indicates pixels to output per line when
           value is between 1 and 8191. Pixels
           \ref horizontalSkip - (image width provided by sensor) are
           output when this value is 0. */
    uint16_t horizontalCount;

    /*! \brief Pixels to skip horizontally between 0 and 8191. */
    uint16_t horizontalSkip;

    /*! \brief 13-bit field that indicates lines to output per frame when
           value is between 1 and 8191. Pixels
           \ref verticalSkip - (image height provided by sensor) are
           output when this value is 0. */
    uint16_t verticalCount;

    /*! \brief Pixels to skip vertically between 0 and 8191. */
    uint16_t verticalSkip;
} CSIRX_ContextCropConfig;

/*! \brief Transcoding configuration */
typedef struct CSIRX_ContextTranscodeConfig_s
{
    /*! \brief Transcoding format, one of \ref CSIRX_TRANSCODE_FORMAT */
    uint8_t transcodeFormat;

    /*! \brief If true, horizontal down scaling by 2 is enabled, else disabled */
    bool    isHorizontalDownscalingBy2Enabled;

    /*! \brief Cropping configuration */
    CSIRX_ContextCropConfig crop;
} CSIRX_ContextTranscodeConfig;

/*! \brief Context interrupts.

       This represents both status and configuration
       (enable/disable) structures. The description of the fields is in terms of
       status. For configuration they should be interpreted as "true if you want
       to enable the detection of this condition, false otherwise". For interrupt status ,
       the interrupts can be cleared by issuing \ref CSIRX_contextClearAllIntr API */
typedef struct CSIRX_ContextIntr_s {

    /*! \brief If true, indicates number of lines specified
           in \ref CSIRX_ContextConfig::numLinesForIntr were received. The interrupt
           repeatability within the frame is decided based on
           \ref CSIRX_ContextConfig::isGenerateIntrEveryNumLinesForIntr configuration.
     */
    bool isNumLines;

    /*! \brief If true, indicates \ref CSIRX_ContextConfig::numFramesToAcquire
           frames have been acquired. */
    bool isFramesToAcquire;

    /*! \brief If true, long packet payoad check-sum mismatched. Influenced by
           \ref CSIRX_ContextConfig::isPayloadChecksumEnable */
    bool isPayloadChecksumMismatch;

    /*! \brief If true, triggers when Line Start sync Code is detected */
    bool isLineStartCodeDetect;

    /*! \brief If true, triggers when Line End sync Code is detected */
    bool isLineEndCodeDetect;

    /*! \brief If true, triggers when Frame Start sync Code is detected */
    bool isFrameStartCodeDetect;

    /*! \brief If true, triggers when Frame End sync Code is detected */
    bool isFrameEndCodeDetect;

    /*! \brief If true, 1-bit error was detected and corrected in long packet.
           Influenced by \ref CSIRX_CommonConfig::isHeaderErrorCheckEnabled */
    bool isLongPacketOneBitErrorCorrect;

} CSIRX_ContextIntr;

/*! \brief Context configuration */
typedef struct CSIRX_ContextConfig_s {

    /*! \brief Virtual channel Id as per MIPI spec */
    uint8_t  virtualChannelId;

    /*! \brief Data format, one of \ref CSIRX_DATA_FORMAT */
    uint16_t format;

    /*! \brief  Selects the pixel format of USER_DEFINED in \ref CSIRX_DATA_FORMAT
           configuration. One of \ref CSIRX_USER_DEFINED_FORMAT */
    uint8_t  userDefinedMapping;

    /*! \brief number of frames to acquire. Special value
           \ref CSIRX_NUM_FRAMES_TO_ACQUIRE_INFINITE for infinite frames */
    uint16_t numFramesToAcquire;

    /*! \brief see description of \ref CSIRX_ContextIntr::isNumLines */
    uint16_t numLinesForIntr;

    /*! \brief controls the padding for *_EXP16, *_EXP32 and *_EXP32_24 data formats
           of the \ref format field */
    uint16_t alpha;

    /*! \brief If true, enables byte swapping of payload data when it is multiples
           of 16-bits. Byte swapping is performed before pixel reconstruction.
           It doesn't affect short packets, long packet header or footers or CRC
           calculation. */
    bool isByteSwapEnabled;

    /*! \brief If true, data is received as per \ref format and the long packet
           code transmitted in the MIPI stream is ignored. If disabled, data is
           received as per \ref format and the long packet code transmitted in
           the MIPI stream is used. */
    bool isGenericEnabled;

    /*! \brief if enabled, end of frame pulse is generated at the end of the frame.
           This controls the combined End of Frame interrupt. */
    bool isEndOfFramePulseEnabled;

    /*! \brief if enabled, end of line pulse is generated at the end of the line.
           This controls the combined End of Line IRQ and the context End of Line
           interrupt. */
    bool isEndOfLinePulseEnabled;

    /*! \brief If true, enables checksum checking of long packet payload.
           Influenced by \ref CSIRX_ContextIntr::isPayloadChecksumMismatch */
    bool isPayloadChecksumEnable;

    /*! \brief see description of CSIRX_ContextIntr::isNumLines */
    bool isGenerateIntrEveryNumLinesForIntr;

    /*! \brief Transcode configuration */
    CSIRX_ContextTranscodeConfig transcodeConfig;

    /*! \brief ping-pong configuration */
    CSIRX_ContextPingPongConfig pingPongConfig;

    /*! \brief Context interrupts to enable/disable */
    CSIRX_ContextIntr enableIntr;

   /*!
     *  \brief  Context End of Line interrupt call back function definition - NOT SUPPORTED AS OF NOW
    *
    *          This is called
    *          when the context's end of line is received and the last memory write
    *          of the received line has landed in memory.
    *          NOTE: this is generated independent
    *          of the \ref CSIRX_ContextIntr::isLineEndCodeDetect configuration.
    *          This is only relevant when
    *          \ref CSIRX_ContextPingPongConfig::pingPongSwitchMode
    *          is \ref CSIRX_PING_PONG_LINE_SWITCHING
    */
    CSIRX_ContextCallback eolCallback;

    /*! \brief Arguments for CSIRX_ContextConfig::eolCallback */
    void *eolCallbackArgs;

} CSIRX_ContextConfig;

/** @} */

/**
 *  \addtogroup DRV_CSIRX_INSTANCE_MODULE
 *
 *  @{
 */

/*! \brief CSIRX Instance information */
typedef struct CSIRX_Info_s {

    /*! \brief 4-bit Major Revision ID + 4-bit minor revision ID*/
    uint8_t revisionId;

    /*! \brief Output FIFO depth, one of \ref CSIRX_FIFODEPTH */
    uint8_t fifoDepth;

    /*! \brief Number of contexts */
    uint8_t numContexts;

} CSIRX_Info;

/** \brief Context specific state information, not to be used by end users */
typedef struct  {

    /** \brief Current programmed ping address */
    uint32_t pingAddress;

    /** \brief Current programmed pong address */
    uint32_t pongAddress;

} CSIRX_ContextObject;

/*! \brief CSIRX Instance Object. Used internally, not to be used by end users */
typedef struct CSIRX_Object_s {

    /** \brief 0: instance is not opened, 1: instance is open */
    bool isOpen;

    /** \brief Context specific state information */
    CSIRX_ContextObject context[CSIRX_CONTEXTS_MAX];

    /** \brief NOT to used by end users */
    uint32_t startOfFrameIntr0ContextId;

    /** \brief NOT to used by end users */
    uint32_t startOfFrameIntr1ContextId;

    /** \brief NOT to used by end users */
    uint32_t endOfFrameIntr0ContextId;

    /** \brief NOT to used by end users */
    uint32_t endOfFrameIntr1ContextId;

    /** \brief NOT to used by end users */
    CSIRX_CommonIntrCallbacks intrCallbacks;

    /** \brief NOT to used by end users */
    HwiP_Object commonIntrObj;

    /** \brief NOT to used by end users */
    HwiP_Object combinedEndOfLineIntrObj;

    /** \brief NOT to used by end users */
    HwiP_Object combinedEndOfFrameIntrObj;

    /** \brief NOT to used by end users */
    HwiP_Object startOfFrameIntr0IntrObj;

    /** \brief NOT to used by end users */
    HwiP_Object startOfFrameIntr1IntrObj;

} CSIRX_Object;

/*! \brief CSIRX HW Attributes. Generated when using sysconfig */
typedef struct CSIRX_HwAttrs_s {
    /*! \brief CSIRX IP registers */
    uint32_t csirxRegs;

    /*! \brief Additional CSIRX control registers, located in SOC top level CTRL regs */
    uint32_t rcssCtrlRegs;

    /*! \brief HW instance ID, 0: CSI2A, 1: CSI2B as so on */
    uint32_t hwInstId;

    /*! \brief CSI interface control clock in Hz. In AM723x, this is 96000000 Hz */
    uint32_t ctrlClockHz;

    /*! \brief CSI interconnect control clock in Hz. In AM723x, this is 200000000 Hz */
    uint32_t interconnectClockHz;

    /*! \brief CPU Interrupt number, if interrupt not connected, set to CSIRX_INTERRUPT_NOT_CONNECTED_ID */
    uint16_t commonIntNum;

    /*! \brief CPU Interrupt number, if interrupt not connected, set to CSIRX_INTERRUPT_NOT_CONNECTED_ID */
    uint16_t combinedEndOfLineIntNum;

    /*! \brief CPU Interrupt number, if interrupt not connected, set to CSIRX_INTERRUPT_NOT_CONNECTED_ID */
    uint16_t combinedEndOfFrameIntNum;

    /*! \brief CPU Interrupt number, if interrupt not connected, set to CSIRX_INTERRUPT_NOT_CONNECTED_ID */
    uint16_t startOfFrameIntr0IntNum;

    /*! \brief CPU Interrupt number, if interrupt not connected, set to CSIRX_INTERRUPT_NOT_CONNECTED_ID */
    uint16_t startOfFrameIntr1IntNum;

    /*! \brief CPU Interrupt number, if interrupt not connected, set to CSIRX_INTERRUPT_NOT_CONNECTED_ID */
    uint16_t contextEndOfLineIntNum[CSIRX_CONTEXTS_MAX];

} CSIRX_HwAttrs;

/*! \brief CSIRX Instance Config Object */
typedef struct CSIRX_Config_s {

    /*! \brief Instance Object */
    CSIRX_Object         *object;

    /*! \brief Instance HW Attributes */
    CSIRX_HwAttrs  const *hwAttrs;
} CSIRX_Config;


/** \brief Array of CSIRX instance enabled via SysConfig */
extern CSIRX_Config gCsirxConfig[];

/** \brief Number of CSIRX instances enabled via SysConfig */
extern uint32_t gCsirxConfigNum;

/** @} */

/**
 *  \addtogroup DRV_CSIRX_INSTANCE_MODULE
 *
 *  @{
 */

/**
 * \brief Initialize CSIRX driver.
 *
 * This does not touch the HW itself, it only initialized internal data structures.
 * Called by SysConfig as part of System_init()
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t CSIRX_init(void);

/**
 * \brief De-Initialize CSIRX driver.
 *
 * This does not touch the HW itself, it only de-initialized internal data structures.
 * Called by SysConfig as part of System_deinit()
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t CSIRX_deinit(void);

/**
 * \brief Open CSIRX driver.
 *
 * \param instanceId [in] CSIRX config to open. This acts as a index into \ref gCsirxConfig[]
 *
 * \return Handle to CSIRX driver for given instance. NULL in case of failure
 */
CSIRX_Handle CSIRX_open(uint32_t instanceId);

/**
 * \brief Get CSIRX instance info
 *
 * \param handle [in] CSIRX driver handle
 * \param info [out] Instance information
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t CSIRX_getInfo(CSIRX_Handle handle, CSIRX_Info *info);

/**
 * \brief Reset CSIRX instance
 *
 * \param handle [in] CSIRX driver handle
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t CSIRX_reset(CSIRX_Handle handle);

/**
 * \brief Close CSIRX instance
 *
 * \param handle [in] CSIRX driver handle
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t CSIRX_close(CSIRX_Handle handle);

/** @} */

/**
 *  \addtogroup DRV_CSIRX_DPHY_MODULE
 *
 *  @{
 */

/**
 * \brief Sets default values for configuration
 *
 * \param config [in] configuration
 */
void CSIRX_DphyConfig_init(CSIRX_DphyConfig *config);

/**
 * \brief Configure DPHY
 *
 * \param handle [in] CSIRX driver handle
 * \param config [in] DPHY configuration
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t CSIRX_dphySetConfig(CSIRX_Handle handle, CSIRX_DphyConfig *config);

/**
 * \brief Check if DPHY control clock reset is done
 *
 * \param handle [in] CSIRX driver handle
 * \param isDone [out] true: reset is done, false: reset is not done
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t CSIRX_dphyIsControlClockResetDone(CSIRX_Handle handle, bool *isDone);

/**
 * \brief Check if DPHY byte clock reset is done
 *
 * \param handle [in] CSIRX driver handle
 * \param isDone [out] true: reset is done, false: reset is not done
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t CSIRX_dphyIsByteClockResetDone(CSIRX_Handle handle, bool *isDone);

/**
 * \brief Check if DPHY clock missing detector error
 *
 * This API is only
 * meaningful if \ref CSIRX_DphyConfig::isClockMissingDetectionEnabled was
 * configured to true i.e clock missing detector was enabled.
 *
 * \param handle [in] CSIRX driver handle
 * \param isError [out] true: error is detected, false: no error
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t CSIRX_dphyIsClockMissingDetectorError(CSIRX_Handle handle, bool *isError);

/** @} */

/**
 *  \addtogroup DRV_CSIRX_COMPLEXIO_MODULE
 *
 *  @{
 */

/**
 * \brief Sets default values for configuration
 *
 * \param config [in] configuration
 */
void CSIRX_ComplexioConfig_init(CSIRX_ComplexioConfig *config);

/**
 * \brief Configure Complex IO
 *
 * \param handle [in] CSIRX driver handle
 * \param config [in] Complex IO configuration
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t CSIRX_complexioSetConfig(CSIRX_Handle handle, CSIRX_ComplexioConfig *config);

/**
 * \brief Celar all pending complex IO interrupts
 *
 * \param handle [in] CSIRX driver handle
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t CSIRX_complexioClearAllIntr(CSIRX_Handle handle);

/**
 * \brief Get all pending complex IO interrupts
 *
 * \param handle [in] CSIRX driver handle
 * \param intrStatus [out] Interrupt status
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t CSIRX_complexioGetPendingIntr(CSIRX_Handle handle, CSIRX_ComplexioLanesIntr *intrStatus);

/**
 * \brief Query about ComplexIO power status.
 *
 * \sa CSIRX_complexioPowerOn, CSIRX_complexioPowerOff, CSIRX_complexioUltraLowPower
 *
 * \param handle [in] CSIRX driver handle
 * \param powerStatus [out] One of \ref CSIRX_COMPLEXIO_POWER_STATUS
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t CSIRX_complexioGetPowerStatus(CSIRX_Handle handle, uint8_t *powerStatus);

/**
 * \brief  Set Complex IO power command
 *
 * \param handle [in] CSIRX driver handle
 * \param powerCommand [out] one of \ref CSIRX_COMPLEXIO_POWER_COMMAND
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t CSIRX_complexioSetPowerCommand(CSIRX_Handle handle, uint8_t powerCommand);

/**
 * \brief Deaasert complex IO reset
 *
 * \param handle [in] CSIRX driver handle
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t CSIRX_complexioDeassertReset(CSIRX_Handle handle);

/**
 * \brief Assert Force Rx Mode on complex IO
 *
 * \param handle [in] CSIRX driver handle
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t CSIRX_complexioAssertForceRxModeOn(CSIRX_Handle handle);

/**
 * \brief De-assert Force Rx Mode on complex IO
 *
 * \param handle [in] CSIRX driver handle
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t CSIRX_complexioDeassertForceRxModeOn(CSIRX_Handle handle);

/**
 * \brief Check if complex IO reset is done
 *
 * \param handle [in] CSIRX driver handle
 * \param isDone [out] true: reset is done, false: reset is not done
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t CSIRX_complexioIsResetDone(CSIRX_Handle handle, bool *isDone);

/**
 * \brief Check if force RX mode on is deasserted
 *
 * \param handle [in] CSIRX driver handle
 * \param isDeasserted [out] true: deassert is done, false: deassert is not done
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t CSIRX_complexioIsDeassertForceRxModeOn(CSIRX_Handle handle, bool *isDeasserted);

/**
 * \brief Power ON complex IO
 *
 * \param handle [in] CSIRX driver handle
 *
 * \return SystemP_SUCCESS on success, else failure
 */
static inline int32_t CSIRX_complexioPowerOn(CSIRX_Handle handle)
{
    return CSIRX_complexioSetPowerCommand(handle, CSIRX_COMPLEXIO_POWER_COMMAND_ON);
}

/**
 * \brief Power OFF complex IO
 *
 * \param handle [in] CSIRX driver handle
 *
 * \return SystemP_SUCCESS on success, else failure
 */
static inline int32_t CSIRX_complexioPowerOff(CSIRX_Handle handle)
{
    return CSIRX_complexioSetPowerCommand(handle, CSIRX_COMPLEXIO_POWER_COMMAND_OFF);
}

/**
 * \brief Put complex IO in ULP (Ultra Low Power) state
 *
 * \param handle [in] CSIRX driver handle
 *
 * \return SystemP_SUCCESS on success, else failure
 */
static inline int32_t CSIRX_complexioUltraLowPower(CSIRX_Handle handle)
{
    return CSIRX_complexioSetPowerCommand(handle, CSIRX_COMPLEXIO_POWER_COMMAND_ULP);
}


/** @} */

/**
 *  \addtogroup DRV_CSIRX_COMMON_MODULE
 *
 *  @{
 */

/**
 * \brief Sets default values for configuration
 *
 * \param config [in] configuration
 */
void CSIRX_CommonConfig_init(CSIRX_CommonConfig *config);

/**
 * \brief Configure common to all context settings
 *
 * \param handle [in] CSIRX driver handle
 * \param config [in] Common to all context configuration
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t CSIRX_commonSetConfig(CSIRX_Handle handle, CSIRX_CommonConfig *config);

/**
 * \brief Clear all pending common to all context interrupts
 *
 * \param handle [in] CSIRX driver handle
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t CSIRX_commonClearAllIntr(CSIRX_Handle handle);

/**
 * \brief Get all pending common to all context interrupts
 *
 * \param handle [in] CSIRX driver handle
 * \param intrStatus [out] Interrupt status
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t CSIRX_commonGetPendingIntr(CSIRX_Handle handle, CSIRX_CommonIntr *intrStatus);

/**
 * \brief Get generic short packet header
 *
 * \param handle [in] CSIRX driver handle
 * \param shortPacket [out] Generic short packet header
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t CSIRX_commonGetGenericShortPacket(CSIRX_Handle handle, uint32_t *shortPacket);

/**
 * \brief Enable CSIRX interface
 *
 * \param handle [in] CSIRX driver handle
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t CSIRX_commonEnable(CSIRX_Handle handle);

/**
 * \brief Disable CSIRX interface
 *
 * \param handle [in] CSIRX driver handle
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t CSIRX_commonDisable(CSIRX_Handle handle);

/** @} */

/**
 *  \addtogroup DRV_CSIRX_CONTEXT_MODULE
 *
 *  @{
 */


/**
 * \brief Sets default values for configuration
 *
 * \param config [in] configuration
 */
void CSIRX_ContextConfig_init(CSIRX_ContextConfig *config);


/**
 * \brief Configure context
 *
 * \param handle [in] CSIRX driver handle
 * \param contextId [in] Context ID
 * \param config [in] Context configuration
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t CSIRX_contextSetConfig(CSIRX_Handle handle, uint8_t contextId, CSIRX_ContextConfig *config);

/**
 * \brief Set context ping and pong address
 *
 * \param handle [in] CSIRX driver handle
 * \param contextId [in] Context ID
 * \param pingAddress [in] ping address, MUST have 5 LSBs as zero
 * \param pongAddress [in] pong address, MUST have 5 LSBs as zero
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t CSIRX_contextSetPingPongAddress(CSIRX_Handle handle, uint8_t contextId, uint32_t pingAddress, uint32_t pongAddress);

/**
 * \brief Set context line offset
 *
 * \param handle [in] CSIRX driver handle
 * \param contextId [in] Context ID
 * \param lineOffset [in] See \ref CSIRX_ContextPingPongConfig::lineOffset , units of bytes
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t CSIRX_contextSetLineOffset(CSIRX_Handle handle, uint8_t contextId, int32_t lineOffset);


/**
 * \brief Enable context
 *
 * \param handle [in] CSIRX driver handle
 * \param contextId [in] Context ID
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t CSIRX_contextEnable(CSIRX_Handle handle, uint8_t contextId);

/**
 * \brief Disable context
 *
 * \param handle [in] CSIRX driver handle
 * \param contextId [in] Context ID
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t CSIRX_contextDisable(CSIRX_Handle handle, uint8_t contextId);

/**
 * \brief Clear all pending context interrupts
 *
 * \param handle [in] CSIRX driver handle
 * \param contextId [in] Context ID
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t CSIRX_contextClearAllIntr(CSIRX_Handle handle, uint8_t contextId);

/**
 * \brief Get all pending context interrupts
 *
 * \param handle [in] CSIRX driver handle
 * \param contextId [in] Context ID
 * \param intrStatus [out] Interrupt status
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t CSIRX_contextGetPendingIntr(CSIRX_Handle handle, uint8_t contextId, CSIRX_ContextIntr *intrStatus);

/**
 * \brief Get current frame number as decoded from within CSIRX long packet
 *
 * \param handle [in] CSIRX driver handle
 * \param contextId [in] Context ID
 * \param frameNumber [out] Frame number
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t CSIRX_contextGetFrameNumber(CSIRX_Handle handle, uint8_t contextId, uint16_t *frameNumber);

/**
 * \brief Get current completed receive address
 *
 * \param handle [in] CSIRX driver handle
 * \param contextId [in] Context ID
 * \param bufAddress [out] Received frame address
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t CSIRX_contextGetRecvAddress(CSIRX_Handle handle, uint8_t contextId, uint32_t *bufAddress);

/**
 * \brief Get status if ping or pong buffer write is done
 *
 * \param handle [in] CSIRX driver handle
 * \param contextId [in] Context ID
 * \param pingPongStatus [out] One of \ref CSIRX_PINGPONG_STATUS
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t CSIRX_contextGetPingPongStatus(CSIRX_Handle handle, uint8_t contextId, uint8_t *pingPongStatus);

/** @} */

/**
 *  \addtogroup DRV_CSIRX_DEBUG_MODULE
 *
 *  @{
 */

/**
 * \brief Set short packet header
 *
 * \param handle [in] CSIRX driver handle
 * \param shortPacket [in] short packet header
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t CSIRX_debugModeSetShortPacket(CSIRX_Handle handle, uint32_t shortPacket);

/**
 * \brief Set long packet header
 *
 * \param handle [in] CSIRX driver handle
 * \param longPacketHeader [in] Long packet header
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t CSIRX_debugModeSetLongPacketHeader(CSIRX_Handle handle, uint32_t longPacketHeader);

/**
 * \brief Set long packet payload
 *
 * \param handle [in] CSIRX driver handle
 * \param payload [in] long packet payload
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t CSIRX_debugModeSetLongPacketPayload(CSIRX_Handle handle, uint32_t payload);

/**
 * \brief Enable debug mode
 *
 * \param handle [in] CSIRX driver handle
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t CSIRX_debugModeEnable(CSIRX_Handle handle);

/**
 * \brief Disable debug mode
 *
 * \param handle [in] CSIRX driver handle
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t CSIRX_debugModeDisable(CSIRX_Handle handle);

/**
 * \brief Generate frames in debug mode
 *
 * \ref CSIRX_debugModeEnable needs to be called before this API
 *
 * \param handle [in] CSIRX driver handle
 * \param txFormat [in] see \ref CSIRX_DATA_FORMAT
 * \param virtualChannelId [in] 0..3
 * \param numOfFrames [in] Number of frames of debug data to generate
 * \param numLinesPerFrame [in] Lines per frame
 * \param numBytesPerLine [in] Bytes per line
 */
void CSIRX_debugModeGenerateFrames(CSIRX_Handle handle,
                                  uint16_t txFormat,
                                  uint8_t virtualChannelId,
                                  uint32_t numOfFrames,
                                  uint32_t numLinesPerFrame,
                                  uint32_t numBytesPerLine);

/** @} */

#ifdef __cplusplus
}
#endif

#endif

/**
 *  \defgroup DRV_CSIRX_MODULE APIs for CSIRX
 *  \ingroup DRV_MODULE
 *
 *  The CSI driver is divided into below sub module APIs.
 *  See also \ref DRIVERS_CSIRX_PAGE for more details.
 */

/**
 *  \defgroup DRV_CSIRX_DPHY_MODULE CSIRX Dphy APIs
 *  \ingroup DRV_CSIRX_MODULE
 *
 *  APIs related to CSIRX D-Phy programming
 */

/**
 *  \defgroup DRV_CSIRX_COMPLEXIO_MODULE CSIRX Complex IO APIs
 *  \ingroup DRV_CSIRX_MODULE
 *
 *  APIs related to CSIRX complex IO lanes
 */

/**
 *  \defgroup DRV_CSIRX_COMMON_MODULE CSIRX Common APIs
 *  \ingroup DRV_CSIRX_MODULE
 *
 *  APIs common across contexts
 */

/**
 *  \defgroup DRV_CSIRX_CONTEXT_MODULE CSIRX Context APIs
 *  \ingroup DRV_CSIRX_MODULE
 *
 *  APIs specific to a context
 */

/**
 *  \defgroup DRV_CSIRX_DEBUG_MODULE CSIRX Debug APIs
 *  \ingroup DRV_CSIRX_MODULE
 *
 *  APIs related to CSIRX debug features. Typically not used by applications.
 */

/**
 *  \defgroup DRV_CSIRX_INSTANCE_MODULE CSIRX Instance APIs
 *  \ingroup DRV_CSIRX_MODULE
 *
 *  APIs to open/close top level CSIRX instance's
 */
