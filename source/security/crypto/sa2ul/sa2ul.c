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
 *  \file   sa2ul.c
 *
 *  \brief  This file contains the implementation of SA2UL ( Ultra lite Security Accelerator) driver
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <string.h>
#include <stddef.h>
#include <drivers/hw_include/cslr.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/CacheP.h>
#include <security/crypto/sa2ul/sa2ul.h>
#include <drivers/hw_include/am64x_am243x/cslr_soc_baseaddress.h>
#include <drivers/sciclient.h>
#include <drivers/hw_include/am64x_am243x/cslr_main_ctrl_mmr.h>
#include <drivers/hw_include/am64x_am243x/cslr_soc_baseaddress.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** \brief Check address aligned */
#define SA2UL_IS_ALIGNED_PTR(ptr, align)                ((((uint32_t)ptr) & ((align)-1)) == 0)

#define SA2UL_IS_HMAC(alg)                              ((alg & 0x10u) == 0)

/** \brief Number of items in MCE data array */
#define SA2UL_MCE_DATA_NUM                              (6)

/** \brief Pack 2 instructions as 3 bytes */
#define MCE_PACK2(op1, f21, f11, f01, op2, f22, f12, f02) \
        ((op1) << 4) | ((f21) << 2) | ((f11) >> 1), \
        (((f11) & 1) << 7) | ((f01) << 4) | (op2), \
        ((f22) << 6) | ((f12) << 3) | (f02)


/** \brief CSL macros */
#define CSL_SA2UL_SCCTL1_OWNER_MASK                     (0x80000000u)
#define CSL_SA2UL_SCCTL1_OWNER_SHIFT                    (31u)

#define CSL_SA2UL_SCCTL1_EVICT_DONE_MASK                (0x40000000u)
#define CSL_SA2UL_SCCTL1_EVICT_DONE_SHIFT               (30u)

#define CSL_SA2UL_SCCTL1_FETCH_EVICT_CONTROL_MASK       (0xff0000u)
#define CSL_SA2UL_SCCTL1_FETCH_EVICT_CONTROL_SHIFT      (16u)

#define CSL_SA2UL_SCCTL1_FETCH_EVICT_SIZE_MASK          (0xff0000u)
#define CSL_SA2UL_SCCTL1_FETCH_EVICT_SIZE_SHIFT         (16u)

#define CSL_SA2UL_SCCTL1_SCID_MASK                      (0xffffu)
#define CSL_SA2UL_SCCTL1_SCID_SHIFT                     (0u)

#define CSL_SA2UL_SCCTL2_OVERWRITE_FLOWID_MASK          (0x80000000u)
#define CSL_SA2UL_SCCTL2_OVERWRITE_FLOWID_SHIFT         (31u)

#define CSL_SA2UL_SCCTL2_PRIVID_MASK                    (0xff0000u)
#define CSL_SA2UL_SCCTL2_PRIVID_SHIFT                   (16u)

#define CSL_SA2UL_SCCTL2_PRIV_MASK                      (0x300u)
#define CSL_SA2UL_SCCTL2_PRIV_SHIFT                     (8u)

#define CSL_SA2UL_SCCTL2_ALLOW_PROMOTE_MASK             (0x80u)
#define CSL_SA2UL_SCCTL2_ALLOW_PROMOTE_SHIFT            (7u)

#define CSL_SA2UL_SCCTL2_ALLOW_DEMOTE_MASK              (0x40u)
#define CSL_SA2UL_SCCTL2_ALLOW_DEMOTE_SHIFT             (6u)

#define CSL_SA2UL_SCCTL2_ALLOW_NON_SECURE_MASK          (0x20u)
#define CSL_SA2UL_SCCTL2_ALLOW_NON_SECURE_SHIFT         (5u)

#define CSL_SA2UL_SCCTL2_SECURE_MASK                    (0x1u)
#define CSL_SA2UL_SCCTL2_SECURE_SHIFT                   (0u)

#define CSL_SA2UL_SCPTRH_FLOWID_MASK                    (0x3fff0000u)
#define CSL_SA2UL_SCPTRH_FLOWID_SHIFT                   (16u)

#define CSL_SA2UL_AUTHCTX1_MODESEL_MASK                 (0x80000000u)
#define CSL_SA2UL_AUTHCTX1_MODESEL_SHIFT                (31u)

#define CSL_SA2UL_AUTHCTX1_DEFAULT_NEXT_ENGINE_ID_MASK  (0x1f000000u)
#define CSL_SA2UL_AUTHCTX1_DEFAULT_NEXT_ENGINE_ID_SHIFT (24u)

#define CSL_SA2UL_AUTHCTX1_SW_CONTROL_MASK              (0xff0000u)
#define CSL_SA2UL_AUTHCTX1_SW_CONTROL_SHIFT             (16u)

#define CSL_SA2UL_ENCRCTL_MODESEL_MASK                  (0x80000000u)
#define CSL_SA2UL_ENCRCTL_MODESEL_SHIFT                 (31u)

#define CSL_SA2UL_ENCRCTL_USE_DKEK_MASK                 (0x40000000u)
#define CSL_SA2UL_ENCRCTL_USE_DKEK_SHIFT                (30u)

#define CSL_SA2UL_ENCRCTL_DEFAULT_NEXT_ENGINE_ID_MASK   (0x1f000000u)
#define CSL_SA2UL_ENCRCTL_DEFAULT_NEXT_ENGINE_ID_SHIFT  (24u)

#define CSL_SA2UL_ENCRCTL_TRAILER_EVERY_CHUNK_MASK      (0x800000u)
#define CSL_SA2UL_ENCRCTL_TRAILER_EVERY_CHUNK_SHIFT     (23u)

#define CSL_SA2UL_ENCRCTL_TRAILER_AT_END_MASK           (0x400000u)
#define CSL_SA2UL_ENCRCTL_TRAILER_AT_END_SHIFT          (22u)

#define CSL_SA2UL_ENCRCTL_PKT_DATA_SECTION_UPDATE_MASK  (0x200000u)
#define CSL_SA2UL_ENCRCTL_PKT_DATA_SECTION_UPDATE_SHIFT (21u)

#define CSL_SA2UL_ENCRCTL_ENCRYPT_DECRYPT_MASK          (0x100000u)
#define CSL_SA2UL_ENCRCTL_ENCRYPT_DECRYPT_SHIFT         (20u)

#define CSL_SA2UL_ENCRCTL_BLK_SIZE_MASK                 (0x70000u)
#define CSL_SA2UL_ENCRCTL_BLK_SIZE_SHIFT                (16u)

#define CSL_SA2UL_ENCRCTL_SOP_OFFSET_MASK               (0xF00u)
#define CSL_SA2UL_ENCRCTL_SOP_OFFSET_SHIFT              (8u)

#define CSL_SA2UL_ENCRCTL_MIDDLE_OFFSET_MASK            (0xF0u)
#define CSL_SA2UL_ENCRCTL_MIDDLE_OFFSET_SHIFT           (4u)

#define CSL_SA2UL_ENCRCTL_EOP_OFFSET_MASK               (0xFu)
#define CSL_SA2UL_ENCRCTL_EOP_OFFSET_SHIFT              (0u)

#define CSL_SA2UL_SWWORD0_BYP_CMD_LBL_LEN_MASK          (0x80000000u)
#define CSL_SA2UL_SWWORD0_BYP_CMD_LBL_LEN_SHIFT         (31u)

#define CSL_SA2UL_SWWORD0_CPPI_DST_INFO_PRESENT_MASK    (0x40000000u)
#define CSL_SA2UL_SWWORD0_CPPI_DST_INFO_PRESENT_SHIFT   (30u)

#define CSL_SA2UL_SWWORD0_ENGINE_ID_MASK                (0x3e000000u)
#define CSL_SA2UL_SWWORD0_ENGINE_ID_SHIFT               (25u)

#define CSL_SA2UL_SWWORD0_CMD_LBL_PRESENT_MASK          (0x1000000u)
#define CSL_SA2UL_SWWORD0_CMD_LBL_PRESENT_SHIFT         (24u)

#define CSL_SA2UL_SWWORD0_CMD_LBL_OFFSET_MASK           (0xf00000u)
#define CSL_SA2UL_SWWORD0_CMD_LBL_OFFSET_SHIFT          (20u)

#define CSL_SA2UL_SWWORD0_FRAGMENT_MASK                 (0x80000u)
#define CSL_SA2UL_SWWORD0_FRAGMENT_SHIFT                (19u)

#define CSL_SA2UL_SWWORD0_NO_PAYLOAD_MASK               (0x40000u)
#define CSL_SA2UL_SWWORD0_NO_PAYLOAD_SHIFT              (18u)

#define CSL_SA2UL_SWWORD0_TEARDOWN_MASK                 (0x20000u)
#define CSL_SA2UL_SWWORD0_TEARDOWN_SHIFT                (17u)

#define CSL_SA2UL_SWWORD0_EVICT_MASK                    (0x10000u)
#define CSL_SA2UL_SWWORD0_EVICT_SHIFT                   (16u)

#define CSL_SA2UL_SWWORD0_SCID_MASK                     (0xffffu)
#define CSL_SA2UL_SWWORD0_SCID_SHIFT                    (0u)

#define CSL_SA2UL_SCPTRH_EGRESS_CPPI_STATUS_LEN_MASK    (0xff000000u)
#define CSL_SA2UL_SCPTRH_EGRESS_CPPI_STATUS_LEN_SHIFT   (24u)

#define CSL_SA2UL_INPSIINFO_EGRESS_CPPI_DEST_QUEUE_NUM_MASK  (0xffff0000u)
#define CSL_SA2UL_INPSIINFO_EGRESS_CPPI_DEST_QUEUE_NUM_SHIFT (16u)

#define CSL_SA2UL_INPSIINFO_NONSEC_CRYPTO_MASK          (0x8u)
#define CSL_SA2UL_INPSIINFO_NONSEC_CRYPTO_SHIFT         (3u)

#define CSL_SA2UL_INPSIINFO_DEMOTE_MASK                 (0x4u)
#define CSL_SA2UL_INPSIINFO_DEMOTE_SHIFT                (2u)

#define CSL_SA2UL_INPSIINFO_PROMOTE_MASK                (0x2u)
#define CSL_SA2UL_INPSIINFO_PROMOTE_SHIFT               (1u)

#define CSL_SA2UL_CMDLBLHDR2_OPTION1_CTX_OFFSET_MASK    (0xf80000u)
#define CSL_SA2UL_CMDLBLHDR2_OPTION1_CTX_OFFSET_SHIFT   (19u)

#define CSL_SA2UL_CMDLBLHDR2_OPTION1_LEN_MASK           (0x70000u)
#define CSL_SA2UL_CMDLBLHDR2_OPTION1_LEN_SHIFT          (16u)

#define CSL_SA2UL_CMDLBLHDR2_OPTION2_CTX_OFFSET_MASK    (0xf800u)
#define CSL_SA2UL_CMDLBLHDR2_OPTION2_CTX_OFFSET_SHIFT   (11u)

#define CSL_SA2UL_CMDLBLHDR2_OPTION2_LEN_MASK           (0x700u)
#define CSL_SA2UL_CMDLBLHDR2_OPTION2_LEN_SHIFT          (8u)

#define CSL_SA2UL_CMDLBLHDR2_OPTION3_CTX_OFFSET_MASK    (0xf8u)
#define CSL_SA2UL_CMDLBLHDR2_OPTION3_CTX_OFFSET_SHIFT   (3u)

#define CSL_SA2UL_CMDLBLHDR2_OPTION3_LEN_MASK           (0x7u)
#define CSL_SA2UL_CMDLBLHDR2_OPTION3_LEN_SHIFT          (0u)

#define CSL_SA2UL_CMDLBLHDR2_SOP_BYPASS_LEN_MASK        (0xff000000u)
#define CSL_SA2UL_CMDLBLHDR2_SOP_BYPASS_LEN_SHIFT       (24u)

#define CSL_SA2UL_CMDLBLHDR1_LEN_TO_BE_PROCESSESED_MASK (0xffffu)
#define CSL_SA2UL_CMDLBLHDR1_LEN_TO_BE_PROCESSESED_SHIFT (0u)

#define CSL_SA2UL_CMDLBLHDR1_CMD_LABEL_LEN_MASK         (0xff0000u)
#define CSL_SA2UL_CMDLBLHDR1_CMD_LABEL_LEN_SHIFT        (16u)

#define CSL_SA2UL_CMDLBLHDR1_NEXT_ENGINE_SELECT_CODE_MASK  (0xff000000u)
#define CSL_SA2UL_CMDLBLHDR1_NEXT_ENGINE_SELECT_CODE_SHIFT (24u)

#define SA2UL_PSIL_DST_THREAD_OFFSET                    (0x8000U)

#define SA2UL_AES_128_KEY_SIZE_IN_BITS                  (128U)
#define SA2UL_AES_192_KEY_SIZE_IN_BITS                  (192U)
#define SA2UL_AES_256_KEY_SIZE_IN_BITS                  (256U)
/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 * \brief Extended packet info for SA2UL
 */
struct SA2UL_ExtendedPktInfo
{
    uint32_t timestamp;
    uint32_t swWord0;
    uint32_t scptrL;
    uint32_t scptrH;
} __attribute__((__packed__));

/**
 * \brief Protocol-specific word in pkts sent to SA2UL
 */
struct SA2UL_PSDataTx
{
    uint32_t inPsiInfo;
    uint32_t cmdLblHdr1;
    uint32_t cmdLblHdr2;
    uint32_t optionWords[52/4];
} __attribute__((__packed__));

/**
 * \brief Protocol specific data for packets received from SA2UL
 */
struct SA2UL_PSDataRx
{
    uint32_t trailerData[16];
} __attribute__((__packed__));

/**
 * \brief Host descriptor structure for SA2UL TX with extended pkt info and
 *        PS words, 128 bytes in size.
 */
struct SA2UL_HostDescrTx
{
    /* Host packet descriptor */
    CSL_PktdmaCppi5HMPD pd;

    /* Extended packet info (16 bytes) for SA2UL */
    struct SA2UL_ExtendedPktInfo exPktInfo;

    /* Protocol-specific word for SA2UL */
    struct SA2UL_PSDataTx psData;
} __attribute__((__packed__));

/**
 * \brief Host descriptor structure for SA2UL Rx with extended pkt info and
 *        PS words, 128 bytes in size.
 */
struct SA2UL_HostDescrRx
{
    /* Host packet descriptor */
    CSL_PktdmaCppi5HMPD pd;

    /* Extended packet info (16 bytes) for SA2UL */
    struct SA2UL_ExtendedPktInfo exPktInfo;

    /* Protocol-specific word for SA2UL */
    struct SA2UL_PSDataRx psData;
} __attribute__((__packed__));

/*!
 * \brief Encoded block sizes for encryption algos
 */
static const uint8_t SA2UL_EncBlksizeEncoded[SA2UL_ENC_ALG_MAX] = { 1u, 0u };

/*!
 * \brief Encryption mode control engine instructions for AES-256-CBC decryption
 */
static const uint8_t gSa2ulMceAes256CbcDecr[] =
{
    MCE_PACK2(
    /* PROC     256,   AES_Key, Reg0 */
        8,      2,      1,      0,
    /* OUTSET   Reg3,   Reg1,   Reg2 */
        10,     3,      1,      2),
    MCE_PACK2(
    /* WAIT     Reg2,   Reg1,   Crypto_OUT ^ Src2 */
        9,      2,      1,      7,
    /* CP       Reg1,   -,      Reg0 */
        4,      1,      0,      0),
    MCE_PACK2(
    /* OUT      -,      -,      - */
        12,     0,      0,      0,
        0,      0,      0,      0)
};

/*!
 * \brief Encryption mode control engine instructions for AES-256-CBC encryption
 */
static const uint8_t gSa2ulMceAes256CbcEncr[] =
{
    MCE_PACK2(
    /* XOR      Reg0,  Reg1,  Reg0 */
        1,      0,      1,      0,
    /* PROC     256,    AES_Key, Reg0 */
        8,      2,      1,      0),
    MCE_PACK2(
    /* OUTSET   Reg2, Crypto_OUT, Crypto_OUT */
        10,     2,      4,      4,
    /* WAIT_OUT  Reg1,   ZERO,  Crypto_OUT */
        11,     1,      7,      6)
};

/*!
 * \brief Encryption mode control engine instructions for AES-256-ECB
 */
static const uint8_t gSa2ulMceAes256Ecb[] =
{
    MCE_PACK2(
    /* PROC     256,   AES_Key, Reg0 */
        8,      2,      1,      0,
    /* OUTSET   Reg0,   Reg0,   Crypto_OUT */
        10,     0,      0,      4),
    MCE_PACK2(
    /* WAIT_OUT Reg1,   ZERO,   Reg1 */
        11,     1,      7,      1,
        0,      0,      0,      0)
};

/*!
 * \brief Encryption mode control engine instructions for AES-128-CBC decryption
 */
static const uint8_t gSa2ulMceAes128CbcDecr[] =
{
    MCE_PACK2(
    /* PROC     128,    AES_Key,Reg0 */
        8,      0,      1,      0,
    /* OUTSET   Reg3,   Reg1,   Reg2 */
        10,     3,      1,      2),
    MCE_PACK2(
    /* WAIT     Reg2,   Reg1,   Crypto_OUT ^ Src2 */
        9,      2,      1,      7,
    /* CP       Reg1,   -,      Reg0 */
        4,      1,      0,      0),
    MCE_PACK2(
    /* OUT      -,      -,      - */
        12,     0,      0,      0,
        0,      0,      0,      0)
};

/*!
 * \brief Encryption mode control engine instructions for AES-128-CBC encryption
 */
static const uint8_t gSa2ulMceAes128CbcEncr[] =
{
    MCE_PACK2(
    /* XOR      Reg0,   Reg1,   Reg0 */
        1,      0,      1,      0,
    /* PROC     128,    AES_Key,Reg0 */
        8,      0,      1,      0),
    MCE_PACK2(
    /* OUTSET   Reg2,  Crypto_OUT, Crypto_OUT */
        10,     2,      4,      4,
    /* WAIT_OUT Reg1,   ZERO,   Crypto_OUT */
        11,     1,      7,      6)
};

/*!
 * \brief Encryption mode control engine instructions for AES-128-ECB
 */
static const uint8_t gSa2ulMceAes128Ecb[] =
{
    MCE_PACK2(
    /* PROC     128,   AES_Key, Reg0 */
        8,      0,      1,      0,
    /* OUTSET   Reg0,   Reg0,   Crypto_OUT */
        10,     0,      0,      4),
    MCE_PACK2(
    /* WAIT_OUT Reg1,   ZERO,   Reg1 */
        11,     1,      7,      1,
        0,      0,      0,      0)
};

/*!
 * \brief Encryption mode control engine instructions for different modes
 */
static const SA2UL_MCEData gSa2ulMceDataArray[SA2UL_MCE_DATA_NUM] =
{
    {
        /* AES-256-ECB */
        0u, 0u, 0u,
        sizeof(gSa2ulMceAes256Ecb),
        gSa2ulMceAes256Ecb
    },
    {
        /* AES-256-CBC encryption */
        0u, 0u, 0u,
        sizeof(gSa2ulMceAes256CbcEncr),
        gSa2ulMceAes256CbcEncr
    },
    {
        /* AES-256-CBC decryption */
        0u, 0u, 0u,
        sizeof(gSa2ulMceAes256CbcDecr),
        gSa2ulMceAes256CbcDecr
    },
    {
        /* AES-128-ECB */
        0u, 0u, 0u,
        sizeof(gSa2ulMceAes128Ecb),
        gSa2ulMceAes128Ecb
    },
    {
        /* AES-128-CBC encryption */
        0u, 0u, 0u,
        sizeof(gSa2ulMceAes128CbcEncr),
        gSa2ulMceAes128CbcEncr
    },
    {
        /* AES-128-CBC decryption */
        0u, 0u, 0u,
        sizeof(gSa2ulMceAes128CbcDecr),
        gSa2ulMceAes128CbcDecr
    }
};

enum
{
    MCE_DATA_ARRAY_INDEX_AES_256_ECB = 0,
    MCE_DATA_ARRAY_INDEX_AES_256_CBC_ENCRYPT,
    MCE_DATA_ARRAY_INDEX_AES_256_CBC_DECRYPT,
    MCE_DATA_ARRAY_INDEX_AES_128_ECB ,
    MCE_DATA_ARRAY_INDEX_AES_128_CBC_ENCRYPT,
    MCE_DATA_ARRAY_INDEX_AES_128_CBC_DECRYPT
};

static const uint32_t gSa2ulHashSizeBytes[] =
{
    0u,  /* NULL */
    16u, /* MD5 */
    20u, /* SHA1 */
    28u, /* SHA2_224 */
    32u, /* SHA2_256 */
    48u, /* SHA2_384 */
    64u, /* SHA2_512 */
    0u   /* NULL */
};

static const uint32_t gSa2ulHashBlkSizeBits[] =
{
    0u, /* NULL */
    512u, /* MD5 */
    512u, /* SHA1 */
    512u, /* SHA2_224 */
    512u, /* SHA2_256 */
    1024u, /* SHA2_384 */
    1024u, /* SHA2_512 */
    0u     /* NULL */
};

const uint32_t gSa2ulAesTe4[] = {
    0x63636363U, 0x7c7c7c7cU, 0x77777777U, 0x7b7b7b7bU,
    0xf2f2f2f2U, 0x6b6b6b6bU, 0x6f6f6f6fU, 0xc5c5c5c5U,
    0x30303030U, 0x01010101U, 0x67676767U, 0x2b2b2b2bU,
    0xfefefefeU, 0xd7d7d7d7U, 0xababababU, 0x76767676U,
    0xcacacacaU, 0x82828282U, 0xc9c9c9c9U, 0x7d7d7d7dU,
    0xfafafafaU, 0x59595959U, 0x47474747U, 0xf0f0f0f0U,
    0xadadadadU, 0xd4d4d4d4U, 0xa2a2a2a2U, 0xafafafafU,
    0x9c9c9c9cU, 0xa4a4a4a4U, 0x72727272U, 0xc0c0c0c0U,
    0xb7b7b7b7U, 0xfdfdfdfdU, 0x93939393U, 0x26262626U,
    0x36363636U, 0x3f3f3f3fU, 0xf7f7f7f7U, 0xccccccccU,
    0x34343434U, 0xa5a5a5a5U, 0xe5e5e5e5U, 0xf1f1f1f1U,
    0x71717171U, 0xd8d8d8d8U, 0x31313131U, 0x15151515U,
    0x04040404U, 0xc7c7c7c7U, 0x23232323U, 0xc3c3c3c3U,
    0x18181818U, 0x96969696U, 0x05050505U, 0x9a9a9a9aU,
    0x07070707U, 0x12121212U, 0x80808080U, 0xe2e2e2e2U,
    0xebebebebU, 0x27272727U, 0xb2b2b2b2U, 0x75757575U,
    0x09090909U, 0x83838383U, 0x2c2c2c2cU, 0x1a1a1a1aU,
    0x1b1b1b1bU, 0x6e6e6e6eU, 0x5a5a5a5aU, 0xa0a0a0a0U,
    0x52525252U, 0x3b3b3b3bU, 0xd6d6d6d6U, 0xb3b3b3b3U,
    0x29292929U, 0xe3e3e3e3U, 0x2f2f2f2fU, 0x84848484U,
    0x53535353U, 0xd1d1d1d1U, 0x00000000U, 0xededededU,
    0x20202020U, 0xfcfcfcfcU, 0xb1b1b1b1U, 0x5b5b5b5bU,
    0x6a6a6a6aU, 0xcbcbcbcbU, 0xbebebebeU, 0x39393939U,
    0x4a4a4a4aU, 0x4c4c4c4cU, 0x58585858U, 0xcfcfcfcfU,
    0xd0d0d0d0U, 0xefefefefU, 0xaaaaaaaaU, 0xfbfbfbfbU,
    0x43434343U, 0x4d4d4d4dU, 0x33333333U, 0x85858585U,
    0x45454545U, 0xf9f9f9f9U, 0x02020202U, 0x7f7f7f7fU,
    0x50505050U, 0x3c3c3c3cU, 0x9f9f9f9fU, 0xa8a8a8a8U,
    0x51515151U, 0xa3a3a3a3U, 0x40404040U, 0x8f8f8f8fU,
    0x92929292U, 0x9d9d9d9dU, 0x38383838U, 0xf5f5f5f5U,
    0xbcbcbcbcU, 0xb6b6b6b6U, 0xdadadadaU, 0x21212121U,
    0x10101010U, 0xffffffffU, 0xf3f3f3f3U, 0xd2d2d2d2U,
    0xcdcdcdcdU, 0x0c0c0c0cU, 0x13131313U, 0xececececU,
    0x5f5f5f5fU, 0x97979797U, 0x44444444U, 0x17171717U,
    0xc4c4c4c4U, 0xa7a7a7a7U, 0x7e7e7e7eU, 0x3d3d3d3dU,
    0x64646464U, 0x5d5d5d5dU, 0x19191919U, 0x73737373U,
    0x60606060U, 0x81818181U, 0x4f4f4f4fU, 0xdcdcdcdcU,
    0x22222222U, 0x2a2a2a2aU, 0x90909090U, 0x88888888U,
    0x46464646U, 0xeeeeeeeeU, 0xb8b8b8b8U, 0x14141414U,
    0xdedededeU, 0x5e5e5e5eU, 0x0b0b0b0bU, 0xdbdbdbdbU,
    0xe0e0e0e0U, 0x32323232U, 0x3a3a3a3aU, 0x0a0a0a0aU,
    0x49494949U, 0x06060606U, 0x24242424U, 0x5c5c5c5cU,
    0xc2c2c2c2U, 0xd3d3d3d3U, 0xacacacacU, 0x62626262U,
    0x91919191U, 0x95959595U, 0xe4e4e4e4U, 0x79797979U,
    0xe7e7e7e7U, 0xc8c8c8c8U, 0x37373737U, 0x6d6d6d6dU,
    0x8d8d8d8dU, 0xd5d5d5d5U, 0x4e4e4e4eU, 0xa9a9a9a9U,
    0x6c6c6c6cU, 0x56565656U, 0xf4f4f4f4U, 0xeaeaeaeaU,
    0x65656565U, 0x7a7a7a7aU, 0xaeaeaeaeU, 0x08080808U,
    0xbabababaU, 0x78787878U, 0x25252525U, 0x2e2e2e2eU,
    0x1c1c1c1cU, 0xa6a6a6a6U, 0xb4b4b4b4U, 0xc6c6c6c6U,
    0xe8e8e8e8U, 0xddddddddU, 0x74747474U, 0x1f1f1f1fU,
    0x4b4b4b4bU, 0xbdbdbdbdU, 0x8b8b8b8bU, 0x8a8a8a8aU,
    0x70707070U, 0x3e3e3e3eU, 0xb5b5b5b5U, 0x66666666U,
    0x48484848U, 0x03030303U, 0xf6f6f6f6U, 0x0e0e0e0eU,
    0x61616161U, 0x35353535U, 0x57575757U, 0xb9b9b9b9U,
    0x86868686U, 0xc1c1c1c1U, 0x1d1d1d1dU, 0x9e9e9e9eU,
    0xe1e1e1e1U, 0xf8f8f8f8U, 0x98989898U, 0x11111111U,
    0x69696969U, 0xd9d9d9d9U, 0x8e8e8e8eU, 0x94949494U,
    0x9b9b9b9bU, 0x1e1e1e1eU, 0x87878787U, 0xe9e9e9e9U,
    0xcecececeU, 0x55555555U, 0x28282828U, 0xdfdfdfdfU,
    0x8c8c8c8cU, 0xa1a1a1a1U, 0x89898989U, 0x0d0d0d0dU,
    0xbfbfbfbfU, 0xe6e6e6e6U, 0x42424242U, 0x68686868U,
    0x41414141U, 0x99999999U, 0x2d2d2d2dU, 0x0f0f0f0fU,
    0xb0b0b0b0U, 0x54545454U, 0xbbbbbbbbU, 0x16161616U
};

/* for 128-bit blocks, AES never uses more than 10 rcon values */
static const uint32_t gSa2ulAesRcon[] =
{
	0x01000000, 0x02000000, 0x04000000, 0x08000000,
	0x10000000, 0x20000000, 0x40000000, 0x80000000,
	0x1B000000, 0x36000000
};
/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */
static void SA2UL_ringAccelWriteDescr(SA2UL_Config *config, uint32_t ringNum, uint64_t descr);
static uint64_t SA2UL_ringAccelReadDescr(SA2UL_Config *config,uint32_t ringNum);
static uint32_t SA2UL_storageQinsert (SA2UL_Object *object,uint64_t val);
static int32_t SA2UL_getMceIndex(SA2UL_ContextObject *ctxObj, uint8_t *aesKeyInvFlag);
static void SA2UL_aesInvKey(uint32_t *invKey, uint32_t *cipherKey, int32_t keyBits);
static int32_t SA2UL_aesKeyExpandEnc(uint32_t *rk, uint32_t *cipherKey, int32_t keyBits);
static void SA2UL_u32LeToU8(uint8_t *dest, const uint32_t *src, uint32_t len);
static void SA2UL_u8LeToU32(uint32_t *dest, const uint8_t *src, uint32_t len);
static void SA2UL_64bEndianSwap(uint32_t *dest, const uint32_t *src, uint32_t len);
static int32_t SA2UL_setupTxChannel(SA2UL_Config  *config);
static int32_t SA2UL_setupRxChannel(SA2UL_Config  *config);
static int32_t SA2UL_dmaInit(SA2UL_Config *config);
static int32_t SA2UL_configInstance(SA2UL_Config *config);
static int32_t SA2UL_checkOpenParams(const SA2UL_Params *prms);
static int32_t SA2UL_pushBuffer(SA2UL_ContextObject *pCtxObj,const uint8_t  *input, uint32_t ilen, uint8_t  *output);
static int32_t SA2UL_popBuffer(SA2UL_ContextObject *pCtxObj, uint64_t *doneBuf, uint32_t *doneBufSize, uint8_t *dataTransferDone);
static int32_t SA2UL_hwInit(SA2UL_Attrs  *attrs);
static uint32_t SA2UL_hwDeInit(SA2UL_Attrs  *attrs);
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void SA2UL_init(void)
{
    return ;
}
void SA2UL_deinit(void)
{
    return ;
}

SA2UL_Handle SA2UL_open(uint32_t index, const SA2UL_Params *params)
{
    uint32_t retVal           = SystemP_SUCCESS;
    SA2UL_Handle     handle   = NULL;
    SA2UL_Config     *config  = NULL;
    SA2UL_Object     *object  = NULL;

    /* Check instance */
    if(index >= gSa2ulConfigNum)
    {
        retVal = SystemP_FAILURE;
    }
    else
    {
        config = &gSa2ulConfig[index];
    }

    if(SystemP_SUCCESS == retVal)
    {
        DebugP_assert(NULL != config->object);
        DebugP_assert(NULL != config->attrs);
        object = config->object;
        if(TRUE == object->isOpen)
        {
            /* Handle is already opened */
            retVal = SystemP_FAILURE;
        }
    }

    if(SystemP_SUCCESS == retVal)
    {
        /* Init state */
        object->handle = (SA2UL_Handle) config;
        if(NULL != params)
        {
            memcpy(&object->prms, params, sizeof(SA2UL_Params));
        }
        else
        {
            /* Init with default if NULL is passed */
            SA2UL_Params_init(&object->prms);
        }

        /* Check open parameters */
        retVal = SA2UL_checkOpenParams(&object->prms);
    }

    if(SystemP_SUCCESS == retVal)
    {
        /* Configure the SA2UL instance parameters HwInit*/
        retVal = SA2UL_hwInit(config->attrs);
        if(SystemP_SUCCESS == retVal)
        {
            /* Initialize DMA */
            retVal = SA2UL_dmaInit(config);
        }
    }

    if(SystemP_SUCCESS == retVal)
    {
        object->isOpen = TRUE;
        handle = (SA2UL_Handle) config;
    }

    /* Free-up resources in case of error */
    if(SystemP_SUCCESS != retVal)
    {
        if(NULL != config)
        {
            SA2UL_close((SA2UL_Handle) config);
        }
    }

    return (handle);
}

void SA2UL_close(SA2UL_Handle handle)
{
    SA2UL_Object *object;
    SA2UL_Config *config;
    const SA2UL_Attrs *attrs;
    config = (SA2UL_Config *) handle;
    if((NULL != config) && (config->object != NULL) && (config->object->isOpen != (uint32_t)FALSE))
    {
        object = config->object;
        attrs = config->attrs;
        Udma_chClose(object->txChHandle);
        Udma_chClose(object->rxChHandle[0]);
        Udma_chClose(object->rxChHandle[1]);
        SA2UL_hwDeInit(config->attrs);
        DebugP_assert(NULL != object);
        DebugP_assert(NULL != attrs);
        object->isOpen = FALSE;
        /* TO module disable */
        handle = NULL;
        SA2UL_hwDeInit(config->attrs);
    }
    return;
}

static int32_t SA2UL_aesKeyExpandEnc(uint32_t *rk, uint32_t *cipherKey, int32_t keyBits)
{
	int32_t i = 0;
	uint32_t temp;
	uint32_t *Te4 = (uint32_t *)&gSa2ulAesTe4;

	/* Caution: cipher key is little Endian */
	rk[0] = cipherKey[0];
	rk[1] = cipherKey[1];
	rk[2] = cipherKey[2];
	rk[3] = cipherKey[3];

	if (keyBits == SA2UL_AES_128_KEY_SIZE_IN_BITS) {
		for (;;) {
			temp  = rk[3];
			rk[4] = rk[0] ^
				(Te4[(temp >> 16) & 0xff] & 0xff000000) ^
				(Te4[(temp >>  8) & 0xff] & 0x00ff0000) ^
				(Te4[(temp      ) & 0xff] & 0x0000ff00) ^
				(Te4[(temp >> 24)       ] & 0x000000ff) ^
				gSa2ulAesRcon[i];
			rk[5] = rk[1] ^ rk[4];
			rk[6] = rk[2] ^ rk[5];
			rk[7] = rk[3] ^ rk[6];
			if (++i == 10) {
				return 10;   /* Nr is fixed and set to 10 */
			}
			rk += 4;
		}
	}

	/* Cipher key expansion for keyBits > 128 */
	rk[4] = cipherKey[4];
	rk[5] = cipherKey[5];
	if (keyBits == SA2UL_AES_192_KEY_SIZE_IN_BITS) {
		for (;;) {
			temp = rk[ 5];
			rk[ 6] = rk[ 0] ^
				(Te4[(temp >> 16) & 0xff] & 0xff000000) ^
				(Te4[(temp >>  8) & 0xff] & 0x00ff0000) ^
				(Te4[(temp      ) & 0xff] & 0x0000ff00) ^
				(Te4[(temp >> 24)       ] & 0x000000ff) ^
				gSa2ulAesRcon[i];
			rk[ 7] = rk[ 1] ^ rk[ 6];
			rk[ 8] = rk[ 2] ^ rk[ 7];
			rk[ 9] = rk[ 3] ^ rk[ 8];
			if (++i == 8) {
				return 12;
			}
			rk[10] = rk[ 4] ^ rk[ 9];
			rk[11] = rk[ 5] ^ rk[10];
			rk += 6;
		}
	}

	/* Cipher key expansion for keyBits > 192 */
	rk[6] = cipherKey[6];
	rk[7] = cipherKey[7];
	if (keyBits == SA2UL_AES_256_KEY_SIZE_IN_BITS) {
		for (;;) {
			temp = rk[ 7];
			rk[ 8] = rk[ 0] ^
				(Te4[(temp >> 16) & 0xff] & 0xff000000) ^
				(Te4[(temp >>  8) & 0xff] & 0x00ff0000) ^
				(Te4[(temp      ) & 0xff] & 0x0000ff00) ^
				(Te4[(temp >> 24)       ] & 0x000000ff) ^
				gSa2ulAesRcon[i];
			rk[ 9] = rk[ 1] ^ rk[ 8];
			rk[10] = rk[ 2] ^ rk[ 9];
			rk[11] = rk[ 3] ^ rk[10];
			if (++i == 7) {
				return 14;
			}
			temp = rk[11];
			rk[12] = rk[ 4] ^
				(Te4[(temp >> 24)       ] & 0xff000000) ^
				(Te4[(temp >> 16) & 0xff] & 0x00ff0000) ^
				(Te4[(temp >>  8) & 0xff] & 0x0000ff00) ^
				(Te4[(temp      ) & 0xff] & 0x000000ff);
			rk[13] = rk[ 5] ^ rk[12];
			rk[14] = rk[ 6] ^ rk[13];
			rk[15] = rk[ 7] ^ rk[14];

			rk += 8;
		}
	}

	return 0;
}

static void SA2UL_aesInvKey(uint32_t *invKey, uint32_t *cipherKey, int32_t keyBits)
{
	int32_t i = 0;
	uint32_t roundKey[60];
	uint32_t* rk = &roundKey[0];
	uint32_t temp;
	uint32_t *Te4 = (uint32_t *)&gSa2ulAesTe4;
	uint32_t *pInvRK = &roundKey[40];
	int32_t keySize = keyBits >> 5;  /* Key size in 32-bit words */

	if (keyBits == SA2UL_AES_128_KEY_SIZE_IN_BITS) {
		SA2UL_aesKeyExpandEnc(rk, cipherKey, 128);
	}

	/* Cipher key expansion for keyBits > 128 */
	if (keyBits == SA2UL_AES_192_KEY_SIZE_IN_BITS) {
		/* Input the cipher key as big Endian byte stream */
		rk[0] = cipherKey[0];
		rk[1] = cipherKey[1];
		rk[2] = cipherKey[2];
		rk[3] = cipherKey[3];
		rk[4] = cipherKey[4];
		rk[5] = cipherKey[5];
		for (;;) {
			temp = rk[ 5];
			rk[ 6] = rk[ 0] ^
				(Te4[(temp >> 16) & 0xff] & 0xff000000) ^
				(Te4[(temp >>  8) & 0xff] & 0x00ff0000) ^
				(Te4[(temp      ) & 0xff] & 0x0000ff00) ^
				(Te4[(temp >> 24)       ] & 0x000000ff) ^
				gSa2ulAesRcon[i];
			rk[ 7] = rk[ 1] ^ rk[ 6];
			rk[ 8] = rk[ 2] ^ rk[ 7];
			rk[ 9] = rk[ 3] ^ rk[ 8];
			rk[10] = rk[ 4] ^ rk[ 9];
			rk[11] = rk[ 5] ^ rk[10];
			if (++i == 8) {
				break;
			}
			rk += 6;
		}
		pInvRK = &roundKey[48];
	}

	/* Cipher key expansion for keyBits > 192 */
	if (keyBits == SA2UL_AES_256_KEY_SIZE_IN_BITS) {
		SA2UL_aesKeyExpandEnc(rk, cipherKey, 256);
		pInvRK = &roundKey[52];
	}


	/* Output the inverse key as little-endian */
	for (i = 0; i < keySize; i++)
	{
		invKey[i] = pInvRK[i];
	}
}

int32_t SA2UL_contextAlloc(SA2UL_Handle handle, SA2UL_ContextObject *ctxObj, const SA2UL_ContextParams *ctxPrms)
{
    uint32_t retVal = SystemP_SUCCESS;
    SA2UL_SecCtx sc;
    int32_t mcDataIndex;
    uint8_t aesKeyInvFlag;
    uint64_t authLen;
    SA2UL_Object *saObj;
    SA2UL_Config  *saCfg;
    SA2UL_Attrs   *saAttrs;
    saCfg = (SA2UL_Config *) handle;
    saObj = saCfg->object;
    saAttrs = saCfg->attrs;

    if((NULL == handle) || (NULL == ctxObj) || (NULL == ctxPrms))
    {
        retVal =  SystemP_FAILURE;
    }
    else
    {
        if(!SA2UL_IS_ALIGNED_PTR(ctxObj, SA2UL_CACHELINE_ALIGNMENT))
        {
            retVal =  SystemP_FAILURE;
        }
        if (SystemP_SUCCESS == retVal)
        {
            memcpy(&ctxObj->ctxPrms, ctxPrms, sizeof(SA2UL_ContextParams));

            /* Increment global context-ID */
            ctxObj->secCtxId = saObj->contextId;
            saObj->contextId++;
            ctxObj->txBytesCnt = 0u;
            ctxObj->rxBytesCnt = 0u;
            ctxObj->computationStatus = 0u;
            ctxObj->sa2ulErrCnt = 0u;

            memset(&sc, 0, sizeof(SA2UL_SecCtx));

            authLen = (ctxObj->ctxPrms.inputLen << 3) ;

            if(ctxObj->ctxPrms.opType == SA2UL_OP_AUTH)
            {
                if(SA2UL_IS_HMAC(ctxObj->ctxPrms.hashAlg))
                {
                    /* HMAC context not supported by ROM driver
                    HMAC for ROM is done with SHA512 context */
                    retVal = SystemP_FAILURE;
                }
                if (SystemP_SUCCESS == retVal)
                {
                    sc.u.auth.authCtx1 =
                        CSL_FMK(SA2UL_AUTHCTX1_MODESEL, 0u) |
                        CSL_FMK(SA2UL_AUTHCTX1_DEFAULT_NEXT_ENGINE_ID, SA2UL_ENGINE_CODE_DEFAULT_EGRESS_PORT) |
                        CSL_FMK(SA2UL_AUTHCTX1_SW_CONTROL, 0x40u | ctxObj->ctxPrms.hashAlg);

                    /* Authentication length in bits for basic hash, length in bits */
                    sc.u.auth.authenticationLengthLo = authLen;
                    sc.u.auth.authenticationLengthHi = (uint32_t)(authLen >> 32);

                    sc.scctl.scctl1 =
                        CSL_FMK(SA2UL_SCCTL1_OWNER, 1u) |
                        CSL_FMK(SA2UL_SCCTL1_EVICT_DONE, 1u) |
                        CSL_FMK(SA2UL_SCCTL1_FETCH_EVICT_CONTROL, 0x91u);
                }
            }
            else if(ctxObj->ctxPrms.opType == SA2UL_OP_ENC)
            {
                if((ctxObj->ctxPrms.encAlg > SA2UL_ENC_ALG_MAX) ||
                    (ctxObj->ctxPrms.encMode > SA2UL_ENC_MODE_MAX) ||
                    (ctxObj->ctxPrms.encKeySize > SA2UL_ENC_KEYSIZE_MAX))
                {
                    retVal = SystemP_FAILURE;
                }
                if (SystemP_SUCCESS == retVal)
                {
                    mcDataIndex = SA2UL_getMceIndex(ctxObj , &aesKeyInvFlag);
                    if (SystemP_FAILURE != mcDataIndex)
                    {
                        sc.u.enc.encrCtl =
                        CSL_FMK(SA2UL_ENCRCTL_MODESEL, 0u) |
                        CSL_FMK(SA2UL_ENCRCTL_USE_DKEK, 0u) |
                        CSL_FMK(SA2UL_ENCRCTL_DEFAULT_NEXT_ENGINE_ID, SA2UL_ENGINE_CODE_DEFAULT_EGRESS_PORT) |
                        CSL_FMK(SA2UL_ENCRCTL_TRAILER_EVERY_CHUNK, 0u) |
                        CSL_FMK(SA2UL_ENCRCTL_TRAILER_AT_END, 0u) |
                        CSL_FMK(SA2UL_ENCRCTL_PKT_DATA_SECTION_UPDATE, 1u) |
                        CSL_FMK(SA2UL_ENCRCTL_ENCRYPT_DECRYPT, ctxObj->ctxPrms.encDirection) |
                        CSL_FMK(SA2UL_ENCRCTL_BLK_SIZE, SA2UL_EncBlksizeEncoded[ctxObj->ctxPrms.encAlg]) |
                        CSL_FMK(SA2UL_ENCRCTL_SOP_OFFSET, gSa2ulMceDataArray[mcDataIndex].sopOffset) |
                        CSL_FMK(SA2UL_ENCRCTL_MIDDLE_OFFSET, gSa2ulMceDataArray[mcDataIndex].middleOffset) |
                        CSL_FMK(SA2UL_ENCRCTL_EOP_OFFSET, gSa2ulMceDataArray[mcDataIndex].eopOffset);

                        SA2UL_u8LeToU32(sc.u.enc.modeCtrlInstrs,
                            gSa2ulMceDataArray[mcDataIndex].mcInstrs,
                            gSa2ulMceDataArray[mcDataIndex].nMCInstrs);
                        sc.u.enc.hwCtrlWord = 0;

                        /* Copy key */
                        SA2UL_u8LeToU32(sc.u.enc.encKeyValue, ctxObj->ctxPrms.key, SA2UL_MAX_KEY_SIZE_BYTES);

                        if (aesKeyInvFlag == (uint8_t)TRUE)
                        {
                            /* Invert the key in context */
                            SA2UL_aesInvKey(sc.u.enc.encKeyValue, sc.u.enc.encKeyValue, SA2UL_ENC_KEYSIZE_BITS(ctxObj->ctxPrms.encKeySize));
                        }
                        /* Copy IV */
                        SA2UL_u8LeToU32(sc.u.enc.encAux2, ctxObj->ctxPrms.iv,SA2UL_MAX_IV_SIZE_BYTES);

                        sc.scctl.scctl1 =
                            CSL_FMK(SA2UL_SCCTL1_OWNER, 1u) |
                            CSL_FMK(SA2UL_SCCTL1_EVICT_DONE, 1u) |
                            CSL_FMK(SA2UL_SCCTL1_FETCH_EVICT_CONTROL, 0x8Du);
                    }
                }
            }
            else
            {
                /* Not implemented at present */
                retVal = SystemP_FAILURE;
            }
            if (SystemP_SUCCESS == retVal)
            {
                sc.scctl.scctl2 =
                CSL_FMK(SA2UL_SCCTL2_PRIVID, saAttrs->privId) |
                CSL_FMK(SA2UL_SCCTL2_PRIV, saAttrs->priv) |
                CSL_FMK(SA2UL_SCCTL2_SECURE, saAttrs->secure);

                SA2UL_64bEndianSwap((uint32_t*)&ctxObj->secCtx, (uint32_t*)&sc, sizeof(sc));

                /* Perform cache writeback */
                CacheP_wb(&ctxObj->secCtx, sizeof(sc), CacheP_TYPE_ALLD);

                /* Perform cache writeback */
                CacheP_inv(&ctxObj->secCtx, sizeof(sc), CacheP_TYPE_ALLD);

                ctxObj->handle = (SA2UL_Config *)handle;
            }
        }
    }
    return (retVal);
}

int32_t SA2UL_contextFree(SA2UL_ContextObject *pCtxObj)
{
    uint32_t retVal = SystemP_SUCCESS;

    if(NULL == pCtxObj)
    {
        retVal = SystemP_FAILURE;
    }
    else
    {
        if(pCtxObj->totalLengthInBytes != pCtxObj->txBytesCnt)
        {
            retVal = SystemP_FAILURE;
        }
        else
        {
            memset(&pCtxObj->secCtx, 0, sizeof(SA2UL_SecCtx));
        }
    }

    return (retVal);
}

int32_t SA2UL_contextProcess(SA2UL_ContextObject *pCtxObj,const uint8_t  *input, uint32_t ilen, uint8_t  *output)
{
    uint32_t retVal             = SystemP_SUCCESS;
    uint64_t doneBufAddr        = 0;
    uint32_t donedataLen        = 0;
    uint8_t  doneFlag           = 0;
    uint32_t processIterations  = 0;
    uint32_t numChunks          = 0;
    uint32_t remainingBytes     = 0;
    uint32_t maxLength          = 0;
    uint8_t *ptrInput           = (uint8_t *)input;
    uint8_t *ptrOutput          = output;

    if(NULL == pCtxObj || NULL == input || NULL == output)
    {
        retVal = SystemP_FAILURE;
    }
    else
    {
        if( pCtxObj->ctxPrms.opType ==  SA2UL_OP_ENC)
        {
            numChunks      = ilen / SA2UL_MAX_INPUT_LENGTH_ENC;
            remainingBytes = ilen % SA2UL_MAX_INPUT_LENGTH_ENC;
            maxLength      = SA2UL_MAX_INPUT_LENGTH_ENC;
        }
        else if( pCtxObj->ctxPrms.opType ==  SA2UL_OP_AUTH)
        {
            numChunks      = ilen / SA2UL_MAX_INPUT_LENGTH_AUTH;
            remainingBytes = ilen % SA2UL_MAX_INPUT_LENGTH_AUTH;
            maxLength      = SA2UL_MAX_INPUT_LENGTH_AUTH;
        }
        else
        {
            retVal = SystemP_FAILURE;
        }

        if(SystemP_SUCCESS == retVal)
        {

            for(processIterations = 0; processIterations < numChunks;processIterations++)
            {
                retVal      = SA2UL_pushBuffer(pCtxObj,ptrInput, maxLength, ptrOutput);
                if(SystemP_SUCCESS == retVal)
                {
                    /* Consider timeout */
                    while((doneBufAddr != (uint64_t)ptrInput) && ( donedataLen != maxLength) && (doneFlag != (uint8_t)TRUE))
                    {
                        retVal = SA2UL_popBuffer(pCtxObj, &doneBufAddr, &donedataLen, &doneFlag);
                    }
                }
                ptrInput    = ptrInput + ((processIterations + 1) * maxLength);
                ptrOutput   = ptrOutput + ((processIterations + 1) * maxLength);
            }
            if(SystemP_SUCCESS == retVal)
            {
                if(remainingBytes != 0)
                {
                    retVal = SA2UL_pushBuffer(pCtxObj, ptrInput, remainingBytes, ptrOutput);
                    if(SystemP_SUCCESS == retVal)
                    {
                        /* Consider timeout */
                        while((doneBufAddr != (uint64_t)ptrInput) && ( donedataLen != remainingBytes ) && (doneFlag != (uint8_t)TRUE))
                        {
                            retVal = SA2UL_popBuffer(pCtxObj, &doneBufAddr, &donedataLen, &doneFlag);
                        }
                    }
                }
            }
        }
    }
    return (retVal);
}

static uint32_t SA2UL_storageQremove(SA2UL_Object *object,uint64_t *val)
{
    uint32_t retVal = SystemP_FAILURE;

    if(object->storageQueueFree != SA2UL_RING_N_ELEMS){
        *val = object->storageQueue[object->storageQueueTail];
        retVal = object->storageQueueTail;
        object->storageQueueTail = (object->storageQueueTail + 1) % SA2UL_RING_N_ELEMS;
        object->storageQueueFree++;
    }

    return (retVal);
}

static uint64_t SA2UL_ringAccelReadDescr(SA2UL_Config *config,uint32_t ringNum)
{
    uint64_t                descr;
    SA2UL_Attrs             *attrs;
    SA2UL_Object            *object;
    attrs  = (SA2UL_Attrs *)config->attrs;
    object = (SA2UL_Object *)config->object;

    if((ringNum == object->ringaccChnls[attrs->txRingNumInt]) ||
       (ringNum == object->ringaccChnls[attrs->txRingNumInt]))
    {
        Udma_ringDequeueRaw(object->txRingHandle,&descr);
    }
    else if((ringNum == object->ringaccChnls[attrs->rxRingNumInt]) ||
            (ringNum == object->ringaccChnls[attrs->rxRingNumInt]))
    {
        Udma_ringDequeueRaw(object->rxRingHandle,&descr);
    }
    else
    {
        SA2UL_storageQremove(object,&descr);
    }

    return (descr);
}

static uint32_t SA2UL_getStorageRingOcc(SA2UL_Object *object, uint32_t ringNum)
{
    return (SA2UL_RING_N_ELEMS - object->storageQueueFree);
}

static uint32_t SA2UL_storageQinsert(SA2UL_Object *object,uint64_t val)
{
    uint32_t retVal = SystemP_FAILURE;

    if(object->storageQueueFree)
    {
        object->storageQueue[object->storageQueueHead] = val;
        retVal = object->storageQueueHead;
        object->storageQueueHead = (object->storageQueueHead + 1) % SA2UL_RING_N_ELEMS;
        object->storageQueueFree--;
    }

    return (retVal);
}

static void SA2UL_ringAccelWriteDescr(SA2UL_Config *config, uint32_t ringNum, uint64_t descr)
{
    SA2UL_Attrs             *attrs;
    SA2UL_Object            *object;
    attrs  = (SA2UL_Attrs *)config->attrs;
    object = (SA2UL_Object *)config->object;

    if((ringNum == object->ringaccChnls[attrs->txRingNumInt]) ||
       (ringNum == object->ringaccChnls[attrs->txRingNumInt]))
    {
        Udma_ringQueueRaw(object->txRingHandle,descr);
    }
    else if((ringNum == object->ringaccChnls[attrs->rxRingNumInt]) ||
            (ringNum == object->ringaccChnls[attrs->rxRingNumInt]))
    {
        Udma_ringQueueRaw(object->rxRingHandle,descr);
    }
    else
    {
        SA2UL_storageQinsert(object, descr);
    }
}

static int32_t SA2UL_setupRxChannel(SA2UL_Config *config)
{
    Udma_ChPrms         chPrms;
    Udma_ChRxPrms       rxPrms;
    Udma_RingHandle     ringHandle;
    int32_t             retVal = UDMA_SOK;
    const SA2UL_Attrs  *attrs;
    SA2UL_Object       *object;

    DebugP_assert(NULL != config->attrs);
    DebugP_assert(NULL != config->object);
    attrs       = config->attrs;
    object      = config->object;

    if(!SA2UL_IS_ALIGNED_PTR(attrs->rxRingMemAddr, SA2UL_CACHELINE_ALIGNMENT))
    {
        retVal = UDMA_EFAIL;
    }
    if (UDMA_SOK == retVal)
    {
        /* RX channel parameters */
        UdmaChPrms_init(&chPrms, UDMA_CH_TYPE_RX_MAPPED);
        chPrms.peerChNum            = attrs->rxPsil0ThreadId;
        chPrms.mappedChGrp          = attrs->udmaSaRxGroupNum;
        chPrms.fqRingPrms.ringMem   = (uint64_t*)attrs->rxRingMemAddr;
        chPrms.fqRingPrms.elemCnt   = attrs->ringCnt;
        chPrms.fqRingPrms.mode      = TISCI_MSG_VALUE_RM_RING_MODE_RING;
        chPrms.fqRingPrms.asel      = UDMA_RINGACC_ASEL_ENDPOINT_PHYSADDR;

        /* Open RX channel for receive from SA */
        object->rxChHandle[1]       = &object->udmaRxChObj[1];
        retVal = Udma_chOpen(object->drvHandle, object->rxChHandle[1], UDMA_CH_TYPE_RX_MAPPED, &chPrms);
        if(UDMA_SOK == retVal)
        {
            UdmaChRxPrms_init(&rxPrms, UDMA_CH_TYPE_RX_MAPPED);
            rxPrms.dmaPriority       = TISCI_MSG_VALUE_RM_UDMAP_CH_SCHED_PRIOR_MEDHIGH;
            rxPrms.fetchWordSize     = attrs->descSize >> 2;
            rxPrms.flowEInfoPresent  = 1;
            rxPrms.flowPsInfoPresent = 1;
            retVal = Udma_chConfigRx(object->rxChHandle[1], &rxPrms);
        }
        else
        {
            retVal = UDMA_EFAIL;
            DebugP_logError("error in Rx-1 Udma_chOpen()  \n");
        }
    }

    if (UDMA_SOK == retVal)
    {
        /* Update the Rx Ring numbers */
        ringHandle                   = Udma_chGetFqRingHandle(object->rxChHandle[1]);
        object->rxFreeRingHandle     = ringHandle;
        object->ringaccChnls[attrs->rxRingNumInt] = Udma_ringGetNum(ringHandle);
        object->rxFlowHandle         = Udma_chGetDefaultFlowHandle(object->rxChHandle[1]);
        object->udmapRxFlownum       = Udma_flowGetNum(object->rxFlowHandle);

        /* Update the Rx Ring numbers */
        ringHandle                   = Udma_chGetCqRingHandle(object->rxChHandle[1]);
        object->rxRingHandle         = ringHandle;
        object->ringaccChnls[attrs->rxRingNumInt] = Udma_ringGetNum(ringHandle);

        /* Create the channel for the second thread with same flow as other thread */
        UdmaChPrms_init(&chPrms, UDMA_CH_TYPE_RX_MAPPED);
        chPrms.peerChNum             = attrs->rxPsil1ThreadId;
        chPrms.mappedChGrp           = attrs->udmaSaRxGroupNum;
        chPrms.fqRingPrms.elemCnt    = attrs->ringCnt;
        chPrms.fqRingPrms.mode       = TISCI_MSG_VALUE_RM_RING_MODE_RING;
        chPrms.fqRingPrms.asel       = UDMA_RINGACC_ASEL_ENDPOINT_PHYSADDR;
        chPrms.fqRingPrms.ringMem    = (uint64_t*)attrs->rxRingMemAddr;
        object->rxChHandle[0]        = &object->udmaRxChObj[0];
    }
    retVal = Udma_chOpen(object->drvHandle, object->rxChHandle[0], UDMA_CH_TYPE_RX_MAPPED, &chPrms);
    if(UDMA_SOK == retVal)
    {
        UdmaChRxPrms_init(&rxPrms, UDMA_CH_TYPE_RX_MAPPED);
        rxPrms.dmaPriority       = TISCI_MSG_VALUE_RM_UDMAP_CH_SCHED_PRIOR_MEDHIGH;
        rxPrms.fetchWordSize     = attrs->descSize >> 2;
        rxPrms.configDefaultFlow = TRUE;
        retVal = Udma_chConfigRx(object->rxChHandle[0], &rxPrms);
    }
    else
    {
        retVal = UDMA_EFAIL;
        DebugP_logError("error in Rx-0 Udma_chOpen()  \n");
    }

    /* Enable the channel after everything is setup */
    if(UDMA_SOK == retVal)
    {
        retVal = Udma_chEnable(object->rxChHandle[1]);
        if(UDMA_SOK == retVal)
        {
            retVal = Udma_chEnable(object->rxChHandle[0]);
        }

        if(retVal != UDMA_SOK)
        {
            retVal = UDMA_EFAIL;
            DebugP_logError("error in Udma_chEnable()  \n");
        }

    }
    if(UDMA_SOK == retVal)
    {
        /* Update Rx channels */
        object->udmapRxChnum[0]      = Udma_chGetNum(object->rxChHandle[0]);
        object->udmapRxChnum[1]      = Udma_chGetNum(object->rxChHandle[1]);

        /* (SW) storage ring */
        object->ringaccChnls[attrs->swRingNumInt] = SA2UL_SW_RING_NUM;
    }

    return (retVal);
}

static int32_t SA2UL_setupTxChannel(SA2UL_Config *config)
{
    Udma_ChPrms         chPrms;
    Udma_ChTxPrms       txPrms;
    Udma_RingHandle     ringHandle;
    int32_t             retVal = UDMA_SOK;
    const SA2UL_Attrs   *attrs;
    SA2UL_Object        *object;

    DebugP_assert(NULL != config->attrs);
    DebugP_assert(NULL != config->object);
    attrs = config->attrs;
    object = config->object;

    if(!SA2UL_IS_ALIGNED_PTR(attrs->txRingMemAddr, SA2UL_CACHELINE_ALIGNMENT))
    {
        retVal = UDMA_EFAIL;
    }
    if(UDMA_SOK == retVal)
    {
        UdmaChPrms_init(&chPrms, UDMA_CH_TYPE_TX_MAPPED);
        chPrms.mappedChGrp           = attrs->udmaSaTxGroupNum;
        chPrms.peerChNum             = attrs->txPsilThreadId | SA2UL_PSIL_DST_THREAD_OFFSET;
        chPrms.fqRingPrms.ringMem    = (uint64_t*)attrs->txRingMemAddr;
        chPrms.fqRingPrms.elemCnt    = attrs->ringCnt;

        /* this is the dual ring mode */
        chPrms.fqRingPrms.mode       = TISCI_MSG_VALUE_RM_RING_MODE_RING;

        /* Open TX channel for transmit */
        object->txChHandle           = &object->udmaTxChObj;
        object->drvHandle            = (void *)attrs->udmaHandle;
    }

    retVal = Udma_chOpen(object->drvHandle, object->txChHandle, UDMA_CH_TYPE_TX_MAPPED, &chPrms);
    if(UDMA_SOK == retVal)
    {
        UdmaChTxPrms_init(&txPrms, UDMA_CH_TYPE_TX_MAPPED);
        txPrms.dmaPriority       = TISCI_MSG_VALUE_RM_UDMAP_CH_SCHED_PRIOR_MEDHIGH;
        txPrms.fetchWordSize     = attrs->descSize >> 2;
        retVal = Udma_chConfigTx(object->txChHandle, &txPrms);
        if(UDMA_SOK == retVal)
        {
            retVal = Udma_chEnable(object->txChHandle);
            object->udmapTxChnum = Udma_chGetNum(object->txChHandle);
        }
    }
    else
    {
        retVal = UDMA_EFAIL;
        DebugP_logError("error in Tx Udma_chOpen()  \n");
    }

    if(retVal == UDMA_SOK)
    {
        /* Update the Tx Ring numbers */
        ringHandle               = Udma_chGetFqRingHandle(object->txChHandle);
        object->txRingHandle     = ringHandle;
        object->ringaccChnls[attrs->txRingNumInt] = Udma_ringGetNum(ringHandle);
        ringHandle               = Udma_chGetCqRingHandle(object->txChHandle);
        object->txComplRingHandle = ringHandle;
        object->ringaccChnls[attrs->txRingNumInt] = Udma_ringGetNum(ringHandle);
    }

    return (retVal);
}

static int32_t SA2UL_dmaInit(SA2UL_Config *config)
{
    uint32_t retVal  = SystemP_SUCCESS;
    SA2UL_Object       *object;
    SA2UL_Attrs        *attrs;
    uint8_t            cnt;
    uint64_t           phys;
    DebugP_assert(NULL != config->object);
    object = config->object;
    attrs  = config->attrs;

    object->contextId            = attrs->contextIdStart;
    object->storageQueueFree     = SA2UL_RING_N_ELEMS;

    /* setup Tx Channel */
    retVal = SA2UL_setupTxChannel(config);
    if(retVal != SystemP_SUCCESS)
    {
        DebugP_logError("error in creating the dma tx channel \n");
        retVal = SystemP_FAILURE;
    }

    /* setup Tx Channel */
    retVal = SA2UL_setupRxChannel(config);
    if(retVal != SystemP_SUCCESS)
    {
        DebugP_logError("error in creating the dma Rx channel \n");
        retVal = SystemP_FAILURE;
    }

    if(retVal == SystemP_SUCCESS)
    {
        phys = attrs->descMemAddr;
        for (cnt=0; cnt<(attrs->descMemAddr / attrs->descMemSize); cnt++)
        {
            if(cnt == (SA2UL_RING_N_ELEMS * 2))
            {
                break;
            }
            SA2UL_ringAccelWriteDescr(config,object->ringaccChnls[attrs->swRingNumInt], phys);
            phys += attrs->descSize;
        }
    }

    return (retVal);
}

static void SA2UL_u32LeToU8(uint8_t *dest, const uint32_t *src, uint32_t len)
{
    uint32_t i, t;

    for (i=0; i<len; i+=4) {
        t = *src++;
        *dest++ = t >> 24;
        *dest++ = t >> 16;
        *dest++ = t >> 8;
        *dest++ = t;
    }
}

static void SA2UL_u8LeToU32(uint32_t *dest, const uint8_t *src, uint32_t len)
{
    uint32_t i, t = 0;

    for (i=0; i<len; i++) {
        t = (t << 8) | src[i];
        if ((i & 3) == 3) {
            *dest++ = t;
            t = 0;
        }
    }
    if ((i & 3) != 0) {
        *dest = t << ((4-(i&3)) << 3);
    }
}

static void SA2UL_64bEndianSwap(uint32_t *dest, const uint32_t *src, uint32_t len)
{
    uint32_t tmp[4], *t;
    uint32_t i, j;

    t = &tmp[0];
    for (i=0; i<len; i+=16) {
        for (j=0; j<4; j++) *t++ = *src++;
        for (j=0; j<4; j++) *dest++ = *--t;
    }
}

static int32_t SA2UL_getMceIndex(SA2UL_ContextObject *ctxObj, uint8_t *aesKeyInvFlag)
{
    uint32_t retVal  = SystemP_FAILURE;
    *aesKeyInvFlag = (uint8_t) FALSE;
    if((ctxObj->ctxPrms.encAlg == SA2UL_ENC_ALG_AES) && (ctxObj->ctxPrms.encKeySize == SA2UL_ENC_KEYSIZE_256))
    {
        if(ctxObj->ctxPrms.encMode == SA2UL_ENC_MODE_ECB)
        {
            if(ctxObj->ctxPrms.encDirection == SA2UL_ENC_DIR_DECRYPT)
                *aesKeyInvFlag = (uint8_t)TRUE;
            retVal = MCE_DATA_ARRAY_INDEX_AES_256_ECB;
        }
        else if(ctxObj->ctxPrms.encMode == SA2UL_ENC_MODE_CBC)
        {
            if(ctxObj->ctxPrms.encDirection == SA2UL_ENC_DIR_ENCRYPT)
            {
                retVal = MCE_DATA_ARRAY_INDEX_AES_256_CBC_ENCRYPT;
            }
            else
            {
                *aesKeyInvFlag = (uint8_t)TRUE;
                retVal = MCE_DATA_ARRAY_INDEX_AES_256_CBC_DECRYPT;
            }
        }
    }
    else if((ctxObj->ctxPrms.encAlg == SA2UL_ENC_ALG_AES) && (ctxObj->ctxPrms.encKeySize == SA2UL_ENC_KEYSIZE_128))
    {
        if(ctxObj->ctxPrms.encMode == SA2UL_ENC_MODE_ECB)
        {
            if(ctxObj->ctxPrms.encDirection == SA2UL_ENC_DIR_DECRYPT)
                *aesKeyInvFlag = (uint8_t)TRUE;
            retVal = MCE_DATA_ARRAY_INDEX_AES_128_ECB;
        }
        else if(ctxObj->ctxPrms.encMode == SA2UL_ENC_MODE_CBC)
        {
            if(ctxObj->ctxPrms.encDirection == SA2UL_ENC_DIR_ENCRYPT)
            {
                retVal = MCE_DATA_ARRAY_INDEX_AES_128_CBC_ENCRYPT;
            }
            else
            {
                *aesKeyInvFlag = (uint8_t)TRUE;
                retVal = MCE_DATA_ARRAY_INDEX_AES_128_CBC_DECRYPT;
            }
        }
    }

    return (retVal);
}

static int32_t SA2UL_pushBuffer(SA2UL_ContextObject *pCtxObj,const uint8_t  *input, uint32_t ilen, uint8_t  *output)
{
    uint32_t retVal = SystemP_SUCCESS;
    struct SA2UL_HostDescrTx *txDescr;
    struct SA2UL_HostDescrRx *rxDescr;
    uint64_t phys, authLen;
    uint32_t lenTBP;
    SA2UL_Object *object;
    SA2UL_Attrs  *attrs;
    SA2UL_Config *config;
    config = (SA2UL_Config *)pCtxObj->handle;
    object = (SA2UL_Object *)config->object;
    attrs  = (SA2UL_Attrs *)config->attrs;

    if((pCtxObj->txBytesCnt + ilen) > pCtxObj->totalLengthInBytes)
        retVal = SystemP_FAILURE;
    if(SystemP_SUCCESS == retVal)
    {
        if(ilen > CSL_FEXT(0xffffffffu, UDMAP_CPPI5_PD_DESCINFO_PKTLEN))
        {
            /* Packet length must not be greater than the the field
            * size in Host pkt descriptor */
            retVal = SystemP_FAILURE;
        }

        lenTBP = CSL_FEXT(0xffffffffu, SA2UL_CMDLBLHDR1_LEN_TO_BE_PROCESSESED);
        if(ilen > lenTBP)
        {
            if(pCtxObj->ctxPrms.opType == SA2UL_OP_ENC)
            {
                /* Packet length must not be greater than the the field
                * size in command label for encryption */
                retVal = SystemP_FAILURE;
            }
        }
        else
        {
            lenTBP = ilen;
        }
    }
    /* Check the occupancy of storage queue */
    if(SA2UL_getStorageRingOcc(object, attrs->swRingNumInt) < 2u)
    {
        retVal = SystemP_FAILURE;
    }
    if(SystemP_SUCCESS == retVal)
    {
        phys = SA2UL_ringAccelReadDescr(config, object->ringaccChnls[attrs->swRingNumInt]);
        txDescr = (struct SA2UL_HostDescrTx *)(phys);

        phys = SA2UL_ringAccelReadDescr(config, object->ringaccChnls[attrs->swRingNumInt]);
        rxDescr = (struct SA2UL_HostDescrRx *)(phys);

        memset(txDescr, 0, sizeof(struct SA2UL_HostDescrTx));
        txDescr->pd.descInfo =
            /* Host descriptor type */
            CSL_FMK(UDMAP_CPPI5_PD_DESCINFO_DTYPE, CSL_UDMAP_CPPI5_PD_DESCINFO_DTYPE_VAL_HOST) |
            /* extended info is present */
            CSL_FMK(UDMAP_CPPI5_PD_DESCINFO_EINFO, CSL_UDMAP_CPPI5_PD_DESCINFO_EINFO_VAL_IS_PRESENT) |
            /* 12 bytes of protocol specific info in desc */
            CSL_FMK(UDMAP_CPPI5_PD_DESCINFO_PSWCNT, 12u >> 2) |
            /* packet length */
            CSL_FMK(UDMAP_CPPI5_PD_DESCINFO_PKTLEN, ilen);

        txDescr->pd.pktInfo1 =
            CSL_FMK(UDMAP_CPPI5_PD_PKTINFO1_FLOWID, object->udmapRxFlownum);

        txDescr->pd.pktInfo2 =
            CSL_FMK(UDMAP_CPPI5_PD_PKTINFO2_RETQ, object->ringaccChnls[attrs->txRingNumInt]);

        txDescr->exPktInfo.swWord0 =
            CSL_FMK(SA2UL_SWWORD0_CPPI_DST_INFO_PRESENT, 1u) |
            CSL_FMK(SA2UL_SWWORD0_CMD_LBL_PRESENT, 1u) |
            CSL_FMK(SA2UL_SWWORD0_CMD_LBL_OFFSET, 0u) |
            CSL_FMK(SA2UL_SWWORD0_SCID, pCtxObj->secCtxId);

        txDescr->psData.cmdLblHdr1 =
            CSL_FMK(SA2UL_CMDLBLHDR1_LEN_TO_BE_PROCESSESED, lenTBP) |
            CSL_FMK(SA2UL_CMDLBLHDR1_CMD_LABEL_LEN, 8u) |
            CSL_FMK(SA2UL_CMDLBLHDR1_NEXT_ENGINE_SELECT_CODE,
                    SA2UL_ENGINE_CODE_DEFAULT_EGRESS_PORT);

        if(pCtxObj->ctxPrms.opType == SA2UL_OP_AUTH)
        {
            txDescr->exPktInfo.swWord0 |= (CSL_FMK(SA2UL_SWWORD0_ENGINE_ID,
            SA2UL_ENGINE_CODE_AUTHENTICATION_MODULE_P1));

            if((pCtxObj->txBytesCnt + ilen) == pCtxObj->totalLengthInBytes)
            {
                /* Evict and teardown security context after last packet */
                txDescr->exPktInfo.swWord0 |= (CSL_FMK(SA2UL_SWWORD0_TEARDOWN, 1u) | CSL_FMK(SA2UL_SWWORD0_EVICT, 1u));
            }
            else
            {
                /* Set the fragment bit */
                txDescr->exPktInfo.swWord0 |= (CSL_FMK(SA2UL_SWWORD0_BYP_CMD_LBL_LEN, 1u) | CSL_FMK(SA2UL_SWWORD0_FRAGMENT, 1u));
            }

            if(pCtxObj->ctxPrms.inputLen > CSL_FEXT(0xffffffffu, SA2UL_CMDLBLHDR1_LEN_TO_BE_PROCESSESED))
            {
                /* Update protocol specific data length, add 8 */
                CSL_FINS(txDescr->pd.descInfo,
                    UDMAP_CPPI5_PD_DESCINFO_PSWCNT, 20u >> 2);

                /* Update command header data length, add 8 */
                CSL_FINS(txDescr->psData.cmdLblHdr1,
                    SA2UL_CMDLBLHDR1_CMD_LABEL_LEN, 16u);

                /* Add option 1 to replace the Authnentication length
                * parameter in context */
                txDescr->psData.cmdLblHdr2 =
                    CSL_FMK(SA2UL_CMDLBLHDR2_OPTION1_CTX_OFFSET, 1u) |
                    CSL_FMK(SA2UL_CMDLBLHDR2_OPTION1_LEN, 1u);

                /* Authentication length in bits */
                authLen = ((uint64_t)pCtxObj->ctxPrms.inputLen) << 3;

                /* Add one block length of additional processing size
                * for HMAC */
                if(SA2UL_IS_HMAC(pCtxObj->ctxPrms.hashAlg)) {
                    authLen += gSa2ulHashBlkSizeBits[pCtxObj->ctxPrms.hashAlg & 7u];
                }

            txDescr->psData.optionWords[0] = (uint32_t)(authLen >> 32);
            txDescr->psData.optionWords[1] = (uint32_t)authLen;
            }
        }

        else
        {
            txDescr->exPktInfo.swWord0 |= (CSL_FMK(SA2UL_SWWORD0_ENGINE_ID, SA2UL_ENGINE_CODE_ENCRYPTION_MODULE_P1));
            if((pCtxObj->txBytesCnt + ilen) == pCtxObj->ctxPrms.inputLen)
            {
                /* Evict and teardown security context after last packet*/
                txDescr->exPktInfo.swWord0 |= (CSL_FMK(SA2UL_SWWORD0_TEARDOWN, 1u) | CSL_FMK(SA2UL_SWWORD0_EVICT, 1u));
            }
        }
        /* Load SecContec pointer in extended PktInfo */
        phys = (uint64_t)(&pCtxObj->secCtx);
        txDescr->exPktInfo.scptrL = (uint32_t)phys;
        txDescr->exPktInfo.scptrH = ((uint32_t)(phys >> 32));
        if(pCtxObj->ctxPrms.opType == SA2UL_OP_AUTH)
        {
            txDescr->exPktInfo.scptrH |= (CSL_FMK(SA2UL_SCPTRH_EGRESS_CPPI_STATUS_LEN, gSa2ulHashSizeBytes[pCtxObj->ctxPrms.hashAlg & 7u]));
        }

        /* Egress CPPI Destination Queue */
        txDescr->psData.inPsiInfo = CSL_FMK(SA2UL_INPSIINFO_EGRESS_CPPI_DEST_QUEUE_NUM, object->ringaccChnls[attrs->rxRingNumInt]);

        txDescr->pd.bufPtr = (uint64_t)input;
        txDescr->pd.bufInfo1 = ilen;
        txDescr->pd.orgBufPtr = (uint64_t)input;
        txDescr->pd.orgBufLen = ilen;

        /* Create a Rx descriptor */
        memset(rxDescr, 0, sizeof(struct SA2UL_HostDescrRx));
        rxDescr->pd.descInfo =
        /* Host descriptor type */
        CSL_FMK(UDMAP_CPPI5_PD_DESCINFO_DTYPE, CSL_PKTDMA_CPPI5_PD_DESCINFO_DTYPE_VAL_HOST);
        rxDescr->pd.bufPtr = (uint64_t)output;
        rxDescr->pd.bufInfo1 = ilen;
        rxDescr->pd.orgBufPtr = (uint64_t)output;
        rxDescr->pd.orgBufLen = ilen;
    }
    /* Check ring accelerator accupancy */
    if((Udma_ringGetForwardRingOcc(object->rxRingHandle) == SA2UL_RING_N_ELEMS) ||
        (Udma_ringGetForwardRingOcc(object->txRingHandle) == SA2UL_RING_N_ELEMS))
    {
        retVal = SystemP_FAILURE;
    }
    if(SystemP_SUCCESS == retVal)
    {
        /* Perform cache writeback */
        CacheP_wb(rxDescr, attrs->descSize, CacheP_TYPE_ALLD);

        /* Perform cache writeback */
        CacheP_inv(rxDescr, attrs->descSize, CacheP_TYPE_ALLD);

        /* Perform cache writeback */
        CacheP_wb(txDescr, attrs->descSize, CacheP_TYPE_ALLD);

        /* Perform cache writeback */
        CacheP_inv(txDescr, attrs->descSize, CacheP_TYPE_ALLD);

        /* Push the RX descriptor to RX free ring */
        phys = (uint64_t)(rxDescr);
        SA2UL_ringAccelWriteDescr(config,object->ringaccChnls[attrs->rxRingNumInt], phys);

        /* Push the TX descriptor to TX send ring*/
        phys = (uint64_t)(txDescr);
        SA2UL_ringAccelWriteDescr(config,object->ringaccChnls[attrs->txRingNumInt], phys);

        pCtxObj->txBytesCnt += ilen;

        /* Perform cache writeback */
        CacheP_wb(output, ilen, CacheP_TYPE_ALLD);

        /* Perform cache writeback */
        CacheP_inv(output, ilen, CacheP_TYPE_ALLD);
    }
    return (retVal);
}

static int32_t SA2UL_popBuffer(SA2UL_ContextObject *pCtxObj, uint64_t *doneBuf, uint32_t *doneBufSize, uint8_t *dataTransferDone)
{
    uint32_t retVal = SystemP_SUCCESS;
    struct SA2UL_HostDescrTx *txDescr;
    struct SA2UL_HostDescrRx *rxDescr;
    uint64_t phys;
    uint32_t reg;
    SA2UL_Object *object;
    SA2UL_Attrs  *attrs;
    SA2UL_Config *config;
    config = (SA2UL_Config *)pCtxObj->handle;
    attrs  = (SA2UL_Attrs  *)config->attrs;
    object = (SA2UL_Object *)config->object;
    *dataTransferDone = (uint8_t)FALSE;

    /* Check the occupancy of Tx complete and RX done queues  */
    if((Udma_ringGetReverseRingOcc(object->rxRingHandle) == 0) || \
       (Udma_ringGetReverseRingOcc(object->txRingHandle) == 0))
    {
        retVal = SystemP_FAILURE;
    }
    if(SystemP_SUCCESS == retVal)
    {
        /* Read the TX done descriptor from TX free ring for cleanup */
        phys = SA2UL_ringAccelReadDescr(config,(uint32_t)object->ringaccChnls[attrs->txRingNumInt]);
        SA2UL_ringAccelWriteDescr(config,(uint32_t)object->ringaccChnls[attrs->swRingNumInt], phys);
        txDescr = (struct SA2UL_HostDescrTx *)(phys);

        *doneBuf = txDescr->pd.bufPtr;
        *doneBufSize = txDescr->pd.bufInfo1;

        /* Read the RX done descriptor from RX done ring */
        phys = SA2UL_ringAccelReadDescr(config,object->ringaccChnls[attrs->rxRingNumInt]);
        SA2UL_ringAccelWriteDescr(config,object->ringaccChnls[attrs->swRingNumInt], phys);
        rxDescr = (struct SA2UL_HostDescrRx *)(phys);

        /* Check for protocol-specific flags for SA2UL errors */
        if(0u != CSL_FEXT(rxDescr->pd.pktInfo1,
                    UDMAP_CPPI5_PD_PKTINFO1_PSFLGS))
                pCtxObj->sa2ulErrCnt ++;

        reg = CSL_FEXT(rxDescr->pd.descInfo, PKTDMA_CPPI5_PD_DESCINFO_PKTLEN);
        pCtxObj->rxBytesCnt += reg;

        if(pCtxObj->rxBytesCnt == pCtxObj->ctxPrms.inputLen)
        {
            if(pCtxObj->ctxPrms.opType == SA2UL_OP_AUTH)
            {
                /* Check for protocol-specific data for SA2UL hash ouput */
                reg = CSL_FEXT(rxDescr->pd.descInfo,
                        PKTDMA_CPPI5_PD_DESCINFO_PSWCNT);

                if((reg << 2) == gSa2ulHashSizeBytes[pCtxObj->ctxPrms.hashAlg & 7u])
                {

                    if(pCtxObj->sa2ulErrCnt == 0u)
                    {
                        /* Copy final hash value from last descriptor */
                        SA2UL_u32LeToU8(pCtxObj->computedHash, &rxDescr->psData.trailerData[0], gSa2ulHashSizeBytes[pCtxObj->ctxPrms.hashAlg & 7u]);

                        pCtxObj->computationStatus = 0;
                    }
                }
            }
            else
            {
                pCtxObj->computationStatus = 0;
            }

            *dataTransferDone =  (uint8_t)TRUE;
        }
    }

    return (retVal);
}

static int32_t SA2UL_hwInit(SA2UL_Attrs  *attrs)
{
    uint32_t retVal           = SystemP_SUCCESS;
    uint32_t                    reg;
    CSL_Cp_aceRegs *pSaRegs = (CSL_Cp_aceRegs *)attrs->saBaseAddr;
    /* Is sha enabled in efuses */
    reg = CSL_FEXT(CSL_REG_RD(&pSaRegs->MMR.EFUSE_EN), CP_ACE_EFUSE_EN_ENABLE);
    if((reg & 1u) == 0u)
    {
        retVal = SystemP_FAILURE;
    }
    else
    {
        /* Enable specific SA2UL engine modules */
        reg = CSL_REG_RD(&pSaRegs->UPDATES.ENGINE_ENABLE);
        CSL_FINS(reg, CP_ACE_UPDATES_ENGINE_ENABLE_CTX_EN, 1u);
        CSL_FINS(reg, CP_ACE_UPDATES_ENGINE_ENABLE_CDMA_IN_EN, 1u);
        CSL_FINS(reg, CP_ACE_UPDATES_ENGINE_ENABLE_CDMA_OUT_EN, 1u);
        CSL_FINS(reg, CP_ACE_UPDATES_ENGINE_ENABLE_PKA_EN, 1u);
        CSL_FINS(reg, CP_ACE_UPDATES_ENGINE_ENABLE_TRNG_EN, 1u);
        CSL_FINS(reg, CP_ACE_UPDATES_ENGINE_ENABLE_AUTHSS_EN, 1u);
        CSL_FINS(reg, CP_ACE_UPDATES_ENGINE_ENABLE_ENCSS_EN, 1u);
        CSL_REG_WR(&pSaRegs->UPDATES.ENGINE_ENABLE, reg);

        reg = CSL_CP_ACE_CMD_STATUS_CTXCACH_EN_MASK       |
              CSL_CP_ACE_CMD_STATUS_CDMA_IN_PORT_EN_MASK  |
              CSL_CP_ACE_CMD_STATUS_CDMA_OUT_PORT_EN_MASK |
              CSL_CP_ACE_CMD_STATUS_PKA_EN_MASK           |
              CSL_CP_ACE_CMD_STATUS_TRNG_EN_MASK          |
              CSL_CP_ACE_CMD_STATUS_ENCSS_EN_MASK         |
              CSL_CP_ACE_CMD_STATUS_AUTHSS_EN_MASK;

        /* Consider timeout */
        while ((reg & CSL_REG_RD(&pSaRegs->MMR.CMD_STATUS)) != reg)
        {
        }
        /* incase timeout */
        if((reg & CSL_REG_RD(&pSaRegs->MMR.CMD_STATUS)) != reg)
        {
            retVal = SystemP_FAILURE;
        }
    }
    return (retVal);

}

static uint32_t SA2UL_hwDeInit(SA2UL_Attrs  *attrs)
{
    uint32_t retVal           = SystemP_SUCCESS;
    uint32_t                    reg;
    CSL_Cp_aceRegs *pSaRegs = (CSL_Cp_aceRegs *)attrs->saBaseAddr;
    /* Is sha enabled in efuses */
    reg = CSL_FEXT(CSL_REG_RD(&pSaRegs->MMR.EFUSE_EN), CP_ACE_EFUSE_EN_ENABLE);
    if((reg & 1u) == 0u)
    {
        retVal = SystemP_FAILURE;
    }
    else
    {
        /* Enable specific SA2UL engine modules */
        reg = CSL_REG_RD(&pSaRegs->UPDATES.ENGINE_ENABLE);
        CSL_FINS(reg, CP_ACE_UPDATES_ENGINE_ENABLE_CTX_EN, 0u);
        CSL_FINS(reg, CP_ACE_UPDATES_ENGINE_ENABLE_CDMA_IN_EN, 0u);
        CSL_FINS(reg, CP_ACE_UPDATES_ENGINE_ENABLE_CDMA_OUT_EN, 0u);
        CSL_FINS(reg, CP_ACE_UPDATES_ENGINE_ENABLE_PKA_EN, 0u);
        CSL_FINS(reg, CP_ACE_UPDATES_ENGINE_ENABLE_TRNG_EN, 0u);
        CSL_FINS(reg, CP_ACE_UPDATES_ENGINE_ENABLE_AUTHSS_EN, 0u);
        CSL_FINS(reg, CP_ACE_UPDATES_ENGINE_ENABLE_ENCSS_EN, 0u);
        CSL_REG_WR(&pSaRegs->UPDATES.ENGINE_ENABLE, reg);

        reg = CSL_CP_ACE_CMD_STATUS_CTXCACH_EN_MASK       |
              CSL_CP_ACE_CMD_STATUS_CDMA_IN_PORT_EN_MASK  |
              CSL_CP_ACE_CMD_STATUS_CDMA_OUT_PORT_EN_MASK |
              CSL_CP_ACE_CMD_STATUS_PKA_EN_MASK           |
              CSL_CP_ACE_CMD_STATUS_TRNG_EN_MASK          |
              CSL_CP_ACE_CMD_STATUS_ENCSS_EN_MASK         |
              CSL_CP_ACE_CMD_STATUS_AUTHSS_EN_MASK;
    }

    return (retVal);
}

static int32_t SA2UL_checkOpenParams(const SA2UL_Params *prms)
{
    int32_t     retVal = SystemP_SUCCESS;

    return (retVal);
}

int32_t SA2UL_rngSetup(SA2UL_Handle handle)
{
    uint32_t val = 0, updated_bits = 0, retVal = SystemP_SUCCESS;
    SA2UL_Config     *config  = (SA2UL_Config *)handle;
    CSL_Cp_aceRegs   *pSaRegs = (CSL_Cp_aceRegs *)config->attrs->saBaseAddr;
    CSL_Cp_aceTrngRegs *pTrngRegs = &pSaRegs->TRNG;
    CSL_main_ctrl_mmr_cfg0Regs *pMainMmrCtrl =(CSL_main_ctrl_mmr_cfg0Regs *) CSL_CTRL_MMR0_CFG0_BASE;

    CSL_REG_WR(&pTrngRegs->TRNG_CONTROL, 0U);
    CSL_REG_WR(&pTrngRegs->TRNG_CONTROL, 0U);

    /* Initialize TRNG_CONFIG to 0 */
	val = ((uint32_t) 0U);
	val |= ((((uint32_t) 5U) << CSL_CP_ACE_TRNG_CONFIG_SAMPLE_CYCLES_SHIFT) & (CSL_CP_ACE_TRNG_CONFIG_SAMPLE_CYCLES_MASK));
    CSL_REG_WR(&pTrngRegs->TRNG_CONFIG, val);

    /* Leave the ALARMCNT register at its reset value */
	val = ((uint32_t) 0xFFU);
    CSL_REG_WR(&pTrngRegs->TRNG_ALARMCNT, val);

    /* write zeros to ALARMMASK and ALARMSTOP registers */
	val = ((uint32_t) 0U);
	CSL_REG_WR(&pTrngRegs->TRNG_ALARMMASK, val);
	CSL_REG_WR(&pTrngRegs->TRNG_ALARMSTOP, val);

    /* We have 8 FRO's in the RNG */
	val = ((uint32_t) 0xFFU);
    CSL_REG_WR(&pTrngRegs->TRNG_FROENABLE, val);

    /* Enable TRNG Section 5.2.5 */
	val = ((((uint32_t) 1U) << CSL_CP_ACE_TRNG_CONTROL_ENABLE_TRNG_SHIFT));
    CSL_REG_WR(&pTrngRegs->TRNG_CONTROL, val);

    /* Initial Seed values:
    *JTAGID, JTAG_USER_ID and DIE_ID0 to 3, In future additional device specific ids will add for rest of fields */
    pTrngRegs->TRNG_PS_AI_0  = (uint32_t)pMainMmrCtrl->JTAGID;
    pTrngRegs->TRNG_PS_AI_1  = (uint32_t)pMainMmrCtrl->JTAG_USER_ID;
    pTrngRegs->TRNG_PS_AI_2  = (uint32_t)pMainMmrCtrl->DIE_ID0;
    pTrngRegs->TRNG_PS_AI_3  = (uint32_t)pMainMmrCtrl->DIE_ID1;
    pTrngRegs->TRNG_PS_AI_4  = (uint32_t)pMainMmrCtrl->DIE_ID2;
    pTrngRegs->TRNG_PS_AI_5  = (uint32_t)pMainMmrCtrl->DIE_ID3;
    pTrngRegs->TRNG_PS_AI_6  = 0x2111ecc9;
    pTrngRegs->TRNG_PS_AI_7  = 0x122111ec;
    pTrngRegs->TRNG_PS_AI_8  = 0x6574b6d7;
    pTrngRegs->TRNG_PS_AI_9  = 0x02cd76ac;
    pTrngRegs->TRNG_PS_AI_10 = 0x76acadd3;
    pTrngRegs->TRNG_PS_AI_11 = 0x74b6d702;
    val = CSL_REG_RD(&pTrngRegs->TRNG_CONTROL);

    /* We always request the maximum number of blocks possible */
	updated_bits |= CSL_CP_ACE_TRNG_CONTROL_DATA_BLOCKS_MASK;
	/* Set the request data bit */
	updated_bits |= CSL_CP_ACE_TRNG_CONTROL_REQUEST_DATA_MASK;

	val |= updated_bits;

	CSL_REG_WR(&pTrngRegs->TRNG_CONTROL, val);

    return (retVal);
}

int32_t SA2UL_rngRead(SA2UL_Handle handle, uint32_t *out)
{
    int32_t retVal = SystemP_SUCCESS;
    uint32_t val = 0U, mask = 0U;
    uint32_t ready = 0U;
    SA2UL_Config        *config         = (SA2UL_Config *)handle;
    CSL_Cp_aceRegs      *pSaRegs        = (CSL_Cp_aceRegs *)config->attrs->saBaseAddr;
    CSL_Cp_aceTrngRegs  *pTrngRegs      = &pSaRegs->TRNG;

    val = CSL_REG_RD(&pTrngRegs->TRNG_CONTROL);

    mask = ((((uint32_t) 1U) << CSL_CP_ACE_TRNG_CONTROL_ENABLE_TRNG_SHIFT));

	if ((val & mask) == mask)
    {
		retVal = SystemP_SUCCESS;
	}

    else
    {
		retVal = SystemP_FAILURE;
        DebugP_assert(SystemP_SUCCESS == retVal);
	}

    if(SystemP_SUCCESS == retVal)
    {
        /* Check if random data is available */
		val = CSL_REG_RD(&pTrngRegs->TRNG_STATUS);
		ready =  (val & CSL_CP_ACE_TRNG_STATUS_READY_MASK) >> CSL_CP_ACE_TRNG_STATUS_READY_SHIFT;
		while (ready != 1U)
        {
            val = CSL_REG_RD(&pTrngRegs->TRNG_STATUS);
            ready =  (val & CSL_CP_ACE_TRNG_STATUS_READY_MASK) >> CSL_CP_ACE_TRNG_STATUS_READY_SHIFT;
        }
        /* If data is available, read it into the output buffer */
		out[0]  = CSL_REG_RD(&pTrngRegs->TRNG_INPUT_0);
        out[1]  = CSL_REG_RD(&pTrngRegs->TRNG_INPUT_1);
        out[2]  = CSL_REG_RD(&pTrngRegs->TRNG_INPUT_2);
        out[3]  = CSL_REG_RD(&pTrngRegs->TRNG_INPUT_3);

		/*Set the INTACK and go back*/
		CSL_REG_WR(&pTrngRegs->TRNG_STATUS, (CSL_CP_ACE_TRNG_INTACK_READY_ACK_MASK << CSL_CP_ACE_TRNG_INTACK_READY_ACK_SHIFT));
	}
    return (retVal);
}