/********************************************************************
 * Copyright (C) 2019 Texas Instruments Incorporated.
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
 *
 *  Name        : cslr_ale.h
*/
#ifndef CSLR_ALE_H_
#define CSLR_ALE_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include <drivers/hw_include/cslr.h>
#include <stdint.h>

/**************************************************************************
* Hardware Region  :
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t MOD_VER;                   /* Module and Version */
    volatile uint32_t ALE_STATUS;                /* ALE Status */
    volatile uint32_t ALE_CONTROL;               /* ALE Control */
    volatile uint32_t ALE_CTRL2;                 /* ALE Control 2 */
    volatile uint32_t ALE_PRESCALE;              /* ALE Prescale */
    volatile uint32_t ALE_AGING_CTRL;            /* ALE Aging Control */
    volatile uint8_t  Resv_28[4];
    volatile uint32_t ALE_NXT_HDR;               /* ALE Next Header */
    volatile uint32_t ALE_TBLCTL;                /* ALE Table Control */
    volatile uint8_t  Resv_52[16];
    volatile uint32_t ALE_TBLW2;                 /* ALE LUT Table word 2 */
    volatile uint32_t ALE_TBLW1;                 /* ALE LUT Table word 1 */
    volatile uint32_t ALE_TBLW0;                 /* ALE LUT Table word 0 */
    volatile uint32_t I0_ALE_PORTCTL0[9];        /* ALE Port Control X */
    volatile uint8_t  Resv_144[44];
    volatile uint32_t ALE_UVLAN_MEMBER;          /* ALE Unknown VLAN Member Mask Register */
    volatile uint32_t ALE_UVLAN_URCAST;          /* ALE Unknown VLAN Unregistered Multicast Flood Mask Register */
    volatile uint32_t ALE_UVLAN_RMCAST;          /* ALE Unknown VLAN Registered Multicast Flood Mask Register */
    volatile uint32_t ALE_UVLAN_UNTAG;           /* ALE Unknown VLAN force Untagged Egress Mask Register */
    volatile uint8_t  Resv_184[24];
    volatile uint32_t ALE_STAT_DIAG;             /* ALE Statistic Output Diagnostic Register */
    volatile uint32_t ALE_OAM_LB_CTRL;           /* ALE OAM Loopback Control */
    volatile uint32_t ALE_MSK_MUX0;              /* ALE Mask Mux 0 */
    volatile uint32_t I1_ALE_MSK_MUX1[7];        /* ALE Mask Mux X */
    volatile uint8_t  Resv_252[28];
    volatile uint32_t EGRESSOP;                  /* Egress Operation */
    volatile uint32_t POLICECFG0;                /* Policing Config 0 */
    volatile uint32_t POLICECFG1;                /* Policing Config 1 */
    volatile uint32_t POLICECFG2;                /* Policing Config 2 */
    volatile uint32_t POLICECFG3;                /* Policing Config 3 */
    volatile uint32_t POLICECFG4;                /* Policing Config 4 */
    volatile uint8_t  Resv_280[4];
    volatile uint32_t POLICECFG6;                /* Policing Config 6 */
    volatile uint32_t POLICECFG7;                /* Policing Config 7 */
    volatile uint32_t POLICETBLCTL;              /* Policing Table Control */
    volatile uint32_t POLICECONTROL;             /* Policing Control */
    volatile uint32_t POLICETESTCTL;             /* Policing Test Control */
    volatile uint32_t POLICEHSTAT;               /* Policing Hit Status */
    volatile uint8_t  Resv_308[4];
    volatile uint32_t THREADMAPDEF;              /* THREAD Mapping Default Value */
    volatile uint32_t THREADMAPCTL;              /* THREAD Mapping Control */
    volatile uint32_t THREADMAPVAL;              /* THREAD Mapping Value */
} CSL_AleRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_ALE_MOD_VER                                                        (0x00000000U)
#define CSL_ALE_ALE_STATUS                                                     (0x00000004U)
#define CSL_ALE_ALE_CONTROL                                                    (0x00000008U)
#define CSL_ALE_ALE_CTRL2                                                      (0x0000000CU)
#define CSL_ALE_ALE_PRESCALE                                                   (0x00000010U)
#define CSL_ALE_ALE_AGING_CTRL                                                 (0x00000014U)
#define CSL_ALE_ALE_NXT_HDR                                                    (0x0000001CU)
#define CSL_ALE_ALE_TBLCTL                                                     (0x00000020U)
#define CSL_ALE_ALE_TBLW2                                                      (0x00000034U)
#define CSL_ALE_ALE_TBLW1                                                      (0x00000038U)
#define CSL_ALE_ALE_TBLW0                                                      (0x0000003CU)
#define CSL_ALE_I0_ALE_PORTCTL0(I0_ALE_PORTCTL0)                               (0x00000040U+((I0_ALE_PORTCTL0)*0x4U))
#define CSL_ALE_ALE_UVLAN_MEMBER                                               (0x00000090U)
#define CSL_ALE_ALE_UVLAN_URCAST                                               (0x00000094U)
#define CSL_ALE_ALE_UVLAN_RMCAST                                               (0x00000098U)
#define CSL_ALE_ALE_UVLAN_UNTAG                                                (0x0000009CU)
#define CSL_ALE_ALE_STAT_DIAG                                                  (0x000000B8U)
#define CSL_ALE_ALE_OAM_LB_CTRL                                                (0x000000BCU)
#define CSL_ALE_ALE_MSK_MUX0                                                   (0x000000C0U)
#define CSL_ALE_I1_ALE_MSK_MUX1(I1_ALE_MSK_MUX1)                               (0x000000C4U+((I1_ALE_MSK_MUX1)*0x4U))
#define CSL_ALE_EGRESSOP                                                       (0x000000FCU)
#define CSL_ALE_POLICECFG0                                                     (0x00000100U)
#define CSL_ALE_POLICECFG1                                                     (0x00000104U)
#define CSL_ALE_POLICECFG2                                                     (0x00000108U)
#define CSL_ALE_POLICECFG3                                                     (0x0000010CU)
#define CSL_ALE_POLICECFG4                                                     (0x00000110U)
#define CSL_ALE_POLICECFG6                                                     (0x00000118U)
#define CSL_ALE_POLICECFG7                                                     (0x0000011CU)
#define CSL_ALE_POLICETBLCTL                                                   (0x00000120U)
#define CSL_ALE_POLICECONTROL                                                  (0x00000124U)
#define CSL_ALE_POLICETESTCTL                                                  (0x00000128U)
#define CSL_ALE_POLICEHSTAT                                                    (0x0000012CU)
#define CSL_ALE_THREADMAPDEF                                                   (0x00000134U)
#define CSL_ALE_THREADMAPCTL                                                   (0x00000138U)
#define CSL_ALE_THREADMAPVAL                                                   (0x0000013CU)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* MOD_VER */

#define CSL_ALE_MOD_VER_MODULE_ID_MASK                                         (0xFFFF0000U)
#define CSL_ALE_MOD_VER_MODULE_ID_SHIFT                                        (0x00000010U)
#define CSL_ALE_MOD_VER_MODULE_ID_MAX                                          (0x0000FFFFU)

#define CSL_ALE_MOD_VER_RTL_VERSION_MASK                                       (0x0000F800U)
#define CSL_ALE_MOD_VER_RTL_VERSION_SHIFT                                      (0x0000000BU)
#define CSL_ALE_MOD_VER_RTL_VERSION_MAX                                        (0x0000001FU)

#define CSL_ALE_MOD_VER_MAJOR_REVISION_MASK                                    (0x00000700U)
#define CSL_ALE_MOD_VER_MAJOR_REVISION_SHIFT                                   (0x00000008U)
#define CSL_ALE_MOD_VER_MAJOR_REVISION_MAX                                     (0x00000007U)

#define CSL_ALE_MOD_VER_CUSTOM_REVISION_MASK                                   (0x000000C0U)
#define CSL_ALE_MOD_VER_CUSTOM_REVISION_SHIFT                                  (0x00000006U)
#define CSL_ALE_MOD_VER_CUSTOM_REVISION_MAX                                    (0x00000003U)

#define CSL_ALE_MOD_VER_MINOR_REVISION_MASK                                    (0x0000003FU)
#define CSL_ALE_MOD_VER_MINOR_REVISION_SHIFT                                   (0x00000000U)
#define CSL_ALE_MOD_VER_MINOR_REVISION_MAX                                     (0x0000003FU)

/* ALE_STATUS */

#define CSL_ALE_ALE_STATUS_UREGANDREGMSK12_MASK                                (0x80000000U)
#define CSL_ALE_ALE_STATUS_UREGANDREGMSK12_SHIFT                               (0x0000001FU)
#define CSL_ALE_ALE_STATUS_UREGANDREGMSK12_MAX                                 (0x00000001U)

#define CSL_ALE_ALE_STATUS_UREGANDREGMSK08_MASK                                (0x40000000U)
#define CSL_ALE_ALE_STATUS_UREGANDREGMSK08_SHIFT                               (0x0000001EU)
#define CSL_ALE_ALE_STATUS_UREGANDREGMSK08_MAX                                 (0x00000001U)

#define CSL_ALE_ALE_STATUS_POLCNTDIV8_MASK                                     (0x0000FF00U)
#define CSL_ALE_ALE_STATUS_POLCNTDIV8_SHIFT                                    (0x00000008U)
#define CSL_ALE_ALE_STATUS_POLCNTDIV8_MAX                                      (0x000000FFU)

#define CSL_ALE_ALE_STATUS_RAMDEPTH128_MASK                                    (0x00000080U)
#define CSL_ALE_ALE_STATUS_RAMDEPTH128_SHIFT                                   (0x00000007U)
#define CSL_ALE_ALE_STATUS_RAMDEPTH128_MAX                                     (0x00000001U)

#define CSL_ALE_ALE_STATUS_RAMDEPTH32_MASK                                     (0x00000040U)
#define CSL_ALE_ALE_STATUS_RAMDEPTH32_SHIFT                                    (0x00000006U)
#define CSL_ALE_ALE_STATUS_RAMDEPTH32_MAX                                      (0x00000001U)

#define CSL_ALE_ALE_STATUS_KLUENTRIES_MASK                                     (0x0000001FU)
#define CSL_ALE_ALE_STATUS_KLUENTRIES_SHIFT                                    (0x00000000U)
#define CSL_ALE_ALE_STATUS_KLUENTRIES_MAX                                      (0x0000001FU)

/* ALE_CONTROL */

#define CSL_ALE_ALE_CONTROL_ENABLE_ALE_MASK                                    (0x80000000U)
#define CSL_ALE_ALE_CONTROL_ENABLE_ALE_SHIFT                                   (0x0000001FU)
#define CSL_ALE_ALE_CONTROL_ENABLE_ALE_MAX                                     (0x00000001U)

#define CSL_ALE_ALE_CONTROL_CLEAR_TABLE_MASK                                   (0x40000000U)
#define CSL_ALE_ALE_CONTROL_CLEAR_TABLE_SHIFT                                  (0x0000001EU)
#define CSL_ALE_ALE_CONTROL_CLEAR_TABLE_MAX                                    (0x00000001U)

#define CSL_ALE_ALE_CONTROL_AGE_OUT_NOW_MASK                                   (0x20000000U)
#define CSL_ALE_ALE_CONTROL_AGE_OUT_NOW_SHIFT                                  (0x0000001DU)
#define CSL_ALE_ALE_CONTROL_AGE_OUT_NOW_MAX                                    (0x00000001U)

#define CSL_ALE_ALE_CONTROL_MIRROR_DP_MASK                                     (0x0F000000U)
#define CSL_ALE_ALE_CONTROL_MIRROR_DP_SHIFT                                    (0x00000018U)
#define CSL_ALE_ALE_CONTROL_MIRROR_DP_MAX                                      (0x0000000FU)

#define CSL_ALE_ALE_CONTROL_UPD_BW_CTRL_MASK                                   (0x00E00000U)
#define CSL_ALE_ALE_CONTROL_UPD_BW_CTRL_SHIFT                                  (0x00000015U)
#define CSL_ALE_ALE_CONTROL_UPD_BW_CTRL_MAX                                    (0x00000007U)

#define CSL_ALE_ALE_CONTROL_MIRROR_TOP_MASK                                    (0x000F0000U)
#define CSL_ALE_ALE_CONTROL_MIRROR_TOP_SHIFT                                   (0x00000010U)
#define CSL_ALE_ALE_CONTROL_MIRROR_TOP_MAX                                     (0x0000000FU)

#define CSL_ALE_ALE_CONTROL_UPD_STATIC_MASK                                    (0x00008000U)
#define CSL_ALE_ALE_CONTROL_UPD_STATIC_SHIFT                                   (0x0000000FU)
#define CSL_ALE_ALE_CONTROL_UPD_STATIC_MAX                                     (0x00000001U)

#define CSL_ALE_ALE_CONTROL_UVLAN_NO_LEARN_MASK                                (0x00002000U)
#define CSL_ALE_ALE_CONTROL_UVLAN_NO_LEARN_SHIFT                               (0x0000000DU)
#define CSL_ALE_ALE_CONTROL_UVLAN_NO_LEARN_MAX                                 (0x00000001U)

#define CSL_ALE_ALE_CONTROL_MIRROR_MEN_MASK                                    (0x00001000U)
#define CSL_ALE_ALE_CONTROL_MIRROR_MEN_SHIFT                                   (0x0000000CU)
#define CSL_ALE_ALE_CONTROL_MIRROR_MEN_MAX                                     (0x00000001U)

#define CSL_ALE_ALE_CONTROL_MIRROR_DEN_MASK                                    (0x00000800U)
#define CSL_ALE_ALE_CONTROL_MIRROR_DEN_SHIFT                                   (0x0000000BU)
#define CSL_ALE_ALE_CONTROL_MIRROR_DEN_MAX                                     (0x00000001U)

#define CSL_ALE_ALE_CONTROL_MIRROR_SEN_MASK                                    (0x00000400U)
#define CSL_ALE_ALE_CONTROL_MIRROR_SEN_SHIFT                                   (0x0000000AU)
#define CSL_ALE_ALE_CONTROL_MIRROR_SEN_MAX                                     (0x00000001U)

#define CSL_ALE_ALE_CONTROL_EN_HOST_UNI_FLOOD_MASK                             (0x00000100U)
#define CSL_ALE_ALE_CONTROL_EN_HOST_UNI_FLOOD_SHIFT                            (0x00000008U)
#define CSL_ALE_ALE_CONTROL_EN_HOST_UNI_FLOOD_MAX                              (0x00000001U)

#define CSL_ALE_ALE_CONTROL_LEARN_NO_VLANID_MASK                               (0x00000080U)
#define CSL_ALE_ALE_CONTROL_LEARN_NO_VLANID_SHIFT                              (0x00000007U)
#define CSL_ALE_ALE_CONTROL_LEARN_NO_VLANID_MAX                                (0x00000001U)

#define CSL_ALE_ALE_CONTROL_ENABLE_VID0_MODE_MASK                              (0x00000040U)
#define CSL_ALE_ALE_CONTROL_ENABLE_VID0_MODE_SHIFT                             (0x00000006U)
#define CSL_ALE_ALE_CONTROL_ENABLE_VID0_MODE_MAX                               (0x00000001U)

#define CSL_ALE_ALE_CONTROL_ENABLE_OUI_DENY_MASK                               (0x00000020U)
#define CSL_ALE_ALE_CONTROL_ENABLE_OUI_DENY_SHIFT                              (0x00000005U)
#define CSL_ALE_ALE_CONTROL_ENABLE_OUI_DENY_MAX                                (0x00000001U)

#define CSL_ALE_ALE_CONTROL_ENABLE_BYPASS_MASK                                 (0x00000010U)
#define CSL_ALE_ALE_CONTROL_ENABLE_BYPASS_SHIFT                                (0x00000004U)
#define CSL_ALE_ALE_CONTROL_ENABLE_BYPASS_MAX                                  (0x00000001U)

#define CSL_ALE_ALE_CONTROL_BCAST_MCAST_CTL_MASK                               (0x00000008U)
#define CSL_ALE_ALE_CONTROL_BCAST_MCAST_CTL_SHIFT                              (0x00000003U)
#define CSL_ALE_ALE_CONTROL_BCAST_MCAST_CTL_MAX                                (0x00000001U)

#define CSL_ALE_ALE_CONTROL_ALE_VLAN_AWARE_MASK                                (0x00000004U)
#define CSL_ALE_ALE_CONTROL_ALE_VLAN_AWARE_SHIFT                               (0x00000002U)
#define CSL_ALE_ALE_CONTROL_ALE_VLAN_AWARE_MAX                                 (0x00000001U)

#define CSL_ALE_ALE_CONTROL_ENABLE_AUTH_MODE_MASK                              (0x00000002U)
#define CSL_ALE_ALE_CONTROL_ENABLE_AUTH_MODE_SHIFT                             (0x00000001U)
#define CSL_ALE_ALE_CONTROL_ENABLE_AUTH_MODE_MAX                               (0x00000001U)

#define CSL_ALE_ALE_CONTROL_ENABLE_RATE_LIMIT_MASK                             (0x00000001U)
#define CSL_ALE_ALE_CONTROL_ENABLE_RATE_LIMIT_SHIFT                            (0x00000000U)
#define CSL_ALE_ALE_CONTROL_ENABLE_RATE_LIMIT_MAX                              (0x00000001U)

/* ALE_CTRL2 */

#define CSL_ALE_ALE_CTRL2_TRK_EN_DST_MASK                                      (0x80000000U)
#define CSL_ALE_ALE_CTRL2_TRK_EN_DST_SHIFT                                     (0x0000001FU)
#define CSL_ALE_ALE_CTRL2_TRK_EN_DST_MAX                                       (0x00000001U)

#define CSL_ALE_ALE_CTRL2_TRK_EN_SRC_MASK                                      (0x40000000U)
#define CSL_ALE_ALE_CTRL2_TRK_EN_SRC_SHIFT                                     (0x0000001EU)
#define CSL_ALE_ALE_CTRL2_TRK_EN_SRC_MAX                                       (0x00000001U)

#define CSL_ALE_ALE_CTRL2_TRK_EN_PRI_MASK                                      (0x20000000U)
#define CSL_ALE_ALE_CTRL2_TRK_EN_PRI_SHIFT                                     (0x0000001DU)
#define CSL_ALE_ALE_CTRL2_TRK_EN_PRI_MAX                                       (0x00000001U)

#define CSL_ALE_ALE_CTRL2_TRK_EN_IVLAN_MASK                                    (0x08000000U)
#define CSL_ALE_ALE_CTRL2_TRK_EN_IVLAN_SHIFT                                   (0x0000001BU)
#define CSL_ALE_ALE_CTRL2_TRK_EN_IVLAN_MAX                                     (0x00000001U)

#define CSL_ALE_ALE_CTRL2_TRK_EN_SIP_MASK                                      (0x02000000U)
#define CSL_ALE_ALE_CTRL2_TRK_EN_SIP_SHIFT                                     (0x00000019U)
#define CSL_ALE_ALE_CTRL2_TRK_EN_SIP_MAX                                       (0x00000001U)

#define CSL_ALE_ALE_CTRL2_TRK_EN_DIP_MASK                                      (0x01000000U)
#define CSL_ALE_ALE_CTRL2_TRK_EN_DIP_SHIFT                                     (0x00000018U)
#define CSL_ALE_ALE_CTRL2_TRK_EN_DIP_MAX                                       (0x00000001U)

#define CSL_ALE_ALE_CTRL2_DROP_BADLEN_MASK                                     (0x00800000U)
#define CSL_ALE_ALE_CTRL2_DROP_BADLEN_SHIFT                                    (0x00000017U)
#define CSL_ALE_ALE_CTRL2_DROP_BADLEN_MAX                                      (0x00000001U)

#define CSL_ALE_ALE_CTRL2_NODROP_SRCMCST_MASK                                  (0x00400000U)
#define CSL_ALE_ALE_CTRL2_NODROP_SRCMCST_SHIFT                                 (0x00000016U)
#define CSL_ALE_ALE_CTRL2_NODROP_SRCMCST_MAX                                   (0x00000001U)

#define CSL_ALE_ALE_CTRL2_DEFNOFRAG_MASK                                       (0x00200000U)
#define CSL_ALE_ALE_CTRL2_DEFNOFRAG_SHIFT                                      (0x00000015U)
#define CSL_ALE_ALE_CTRL2_DEFNOFRAG_MAX                                        (0x00000001U)

#define CSL_ALE_ALE_CTRL2_DEFLMTNXTHDR_MASK                                    (0x00100000U)
#define CSL_ALE_ALE_CTRL2_DEFLMTNXTHDR_SHIFT                                   (0x00000014U)
#define CSL_ALE_ALE_CTRL2_DEFLMTNXTHDR_MAX                                     (0x00000001U)

#define CSL_ALE_ALE_CTRL2_TRK_BASE_MASK                                        (0x00070000U)
#define CSL_ALE_ALE_CTRL2_TRK_BASE_SHIFT                                       (0x00000010U)
#define CSL_ALE_ALE_CTRL2_TRK_BASE_MAX                                         (0x00000007U)

#define CSL_ALE_ALE_CTRL2_MIRROR_MIDX_MASK                                     (0x000003FFU)
#define CSL_ALE_ALE_CTRL2_MIRROR_MIDX_SHIFT                                    (0x00000000U)
#define CSL_ALE_ALE_CTRL2_MIRROR_MIDX_MAX                                      (0x000003FFU)

/* ALE_PRESCALE */

#define CSL_ALE_ALE_PRESCALE_ALE_PRESCALE_MASK                                 (0x000FFFFFU)
#define CSL_ALE_ALE_PRESCALE_ALE_PRESCALE_SHIFT                                (0x00000000U)
#define CSL_ALE_ALE_PRESCALE_ALE_PRESCALE_MAX                                  (0x000FFFFFU)

/* ALE_AGING_CTRL */

#define CSL_ALE_ALE_AGING_CTRL_PRESCALE_2_DISABLE_MASK                         (0x80000000U)
#define CSL_ALE_ALE_AGING_CTRL_PRESCALE_2_DISABLE_SHIFT                        (0x0000001FU)
#define CSL_ALE_ALE_AGING_CTRL_PRESCALE_2_DISABLE_MAX                          (0x00000001U)

#define CSL_ALE_ALE_AGING_CTRL_PRESCALE_1_DISABLE_MASK                         (0x40000000U)
#define CSL_ALE_ALE_AGING_CTRL_PRESCALE_1_DISABLE_SHIFT                        (0x0000001EU)
#define CSL_ALE_ALE_AGING_CTRL_PRESCALE_1_DISABLE_MAX                          (0x00000001U)

#define CSL_ALE_ALE_AGING_CTRL_ALE_AGING_TIMER_MASK                            (0x00FFFFFFU)
#define CSL_ALE_ALE_AGING_CTRL_ALE_AGING_TIMER_SHIFT                           (0x00000000U)
#define CSL_ALE_ALE_AGING_CTRL_ALE_AGING_TIMER_MAX                             (0x00FFFFFFU)

/* ALE_NXT_HDR */

#define CSL_ALE_ALE_NXT_HDR_IP_NXT_HDR3_MASK                                   (0xFF000000U)
#define CSL_ALE_ALE_NXT_HDR_IP_NXT_HDR3_SHIFT                                  (0x00000018U)
#define CSL_ALE_ALE_NXT_HDR_IP_NXT_HDR3_MAX                                    (0x000000FFU)

#define CSL_ALE_ALE_NXT_HDR_IP_NXT_HDR2_MASK                                   (0x00FF0000U)
#define CSL_ALE_ALE_NXT_HDR_IP_NXT_HDR2_SHIFT                                  (0x00000010U)
#define CSL_ALE_ALE_NXT_HDR_IP_NXT_HDR2_MAX                                    (0x000000FFU)

#define CSL_ALE_ALE_NXT_HDR_IP_NXT_HDR1_MASK                                   (0x0000FF00U)
#define CSL_ALE_ALE_NXT_HDR_IP_NXT_HDR1_SHIFT                                  (0x00000008U)
#define CSL_ALE_ALE_NXT_HDR_IP_NXT_HDR1_MAX                                    (0x000000FFU)

#define CSL_ALE_ALE_NXT_HDR_IP_NXT_HDR0_MASK                                   (0x000000FFU)
#define CSL_ALE_ALE_NXT_HDR_IP_NXT_HDR0_SHIFT                                  (0x00000000U)
#define CSL_ALE_ALE_NXT_HDR_IP_NXT_HDR0_MAX                                    (0x000000FFU)

/* ALE_TBLCTL */

#define CSL_ALE_ALE_TBLCTL_TABLEWR_MASK                                        (0x80000000U)
#define CSL_ALE_ALE_TBLCTL_TABLEWR_SHIFT                                       (0x0000001FU)
#define CSL_ALE_ALE_TBLCTL_TABLEWR_MAX                                         (0x00000001U)

#define CSL_ALE_ALE_TBLCTL_TABLEIDX_MASK                                       (0x000003FFU)
#define CSL_ALE_ALE_TBLCTL_TABLEIDX_SHIFT                                      (0x00000000U)
#define CSL_ALE_ALE_TBLCTL_TABLEIDX_MAX                                        (0x000003FFU)

/* ALE_TBLW2 */

#define CSL_ALE_ALE_TBLW2_TABLEWRD2_MASK                                       (0x000007FFU)
#define CSL_ALE_ALE_TBLW2_TABLEWRD2_SHIFT                                      (0x00000000U)
#define CSL_ALE_ALE_TBLW2_TABLEWRD2_MAX                                        (0x000007FFU)

/* ALE_TBLW1 */

#define CSL_ALE_ALE_TBLW1_TABLEWRD1_MASK                                       (0xFFFFFFFFU)
#define CSL_ALE_ALE_TBLW1_TABLEWRD1_SHIFT                                      (0x00000000U)
#define CSL_ALE_ALE_TBLW1_TABLEWRD1_MAX                                        (0xFFFFFFFFU)

/* ALE_TBLW0 */

#define CSL_ALE_ALE_TBLW0_TABLEWRD0_MASK                                       (0xFFFFFFFFU)
#define CSL_ALE_ALE_TBLW0_TABLEWRD0_SHIFT                                      (0x00000000U)
#define CSL_ALE_ALE_TBLW0_TABLEWRD0_MAX                                        (0xFFFFFFFFU)

/* I0_ALE_PORTCTL0 */

#define CSL_ALE_I0_ALE_PORTCTL0_I0_REG_P0_BCAST_LIMIT_MASK                     (0xFF000000U)
#define CSL_ALE_I0_ALE_PORTCTL0_I0_REG_P0_BCAST_LIMIT_SHIFT                    (0x00000018U)
#define CSL_ALE_I0_ALE_PORTCTL0_I0_REG_P0_BCAST_LIMIT_MAX                      (0x000000FFU)

#define CSL_ALE_I0_ALE_PORTCTL0_I0_REG_P0_MCAST_LIMIT_MASK                     (0x00FF0000U)
#define CSL_ALE_I0_ALE_PORTCTL0_I0_REG_P0_MCAST_LIMIT_SHIFT                    (0x00000010U)
#define CSL_ALE_I0_ALE_PORTCTL0_I0_REG_P0_MCAST_LIMIT_MAX                      (0x000000FFU)

#define CSL_ALE_I0_ALE_PORTCTL0_I0_REG_P0_DROP_DOUBLE_VLAN_MASK                (0x00008000U)
#define CSL_ALE_I0_ALE_PORTCTL0_I0_REG_P0_DROP_DOUBLE_VLAN_SHIFT               (0x0000000FU)
#define CSL_ALE_I0_ALE_PORTCTL0_I0_REG_P0_DROP_DOUBLE_VLAN_MAX                 (0x00000001U)

#define CSL_ALE_I0_ALE_PORTCTL0_I0_REG_P0_DROP_DUAL_VLAN_MASK                  (0x00004000U)
#define CSL_ALE_I0_ALE_PORTCTL0_I0_REG_P0_DROP_DUAL_VLAN_SHIFT                 (0x0000000EU)
#define CSL_ALE_I0_ALE_PORTCTL0_I0_REG_P0_DROP_DUAL_VLAN_MAX                   (0x00000001U)

#define CSL_ALE_I0_ALE_PORTCTL0_I0_REG_P0_MACONLY_CAF_MASK                     (0x00002000U)
#define CSL_ALE_I0_ALE_PORTCTL0_I0_REG_P0_MACONLY_CAF_SHIFT                    (0x0000000DU)
#define CSL_ALE_I0_ALE_PORTCTL0_I0_REG_P0_MACONLY_CAF_MAX                      (0x00000001U)

#define CSL_ALE_I0_ALE_PORTCTL0_I0_REG_P0_DIS_PAUTHMOD_MASK                    (0x00001000U)
#define CSL_ALE_I0_ALE_PORTCTL0_I0_REG_P0_DIS_PAUTHMOD_SHIFT                   (0x0000000CU)
#define CSL_ALE_I0_ALE_PORTCTL0_I0_REG_P0_DIS_PAUTHMOD_MAX                     (0x00000001U)

#define CSL_ALE_I0_ALE_PORTCTL0_I0_REG_P0_MACONLY_MASK                         (0x00000800U)
#define CSL_ALE_I0_ALE_PORTCTL0_I0_REG_P0_MACONLY_SHIFT                        (0x0000000BU)
#define CSL_ALE_I0_ALE_PORTCTL0_I0_REG_P0_MACONLY_MAX                          (0x00000001U)

#define CSL_ALE_I0_ALE_PORTCTL0_I0_REG_P0_TRUNKEN_MASK                         (0x00000400U)
#define CSL_ALE_I0_ALE_PORTCTL0_I0_REG_P0_TRUNKEN_SHIFT                        (0x0000000AU)
#define CSL_ALE_I0_ALE_PORTCTL0_I0_REG_P0_TRUNKEN_MAX                          (0x00000001U)

#define CSL_ALE_I0_ALE_PORTCTL0_I0_REG_P0_TRUNKNUM_MASK                        (0x00000300U)
#define CSL_ALE_I0_ALE_PORTCTL0_I0_REG_P0_TRUNKNUM_SHIFT                       (0x00000008U)
#define CSL_ALE_I0_ALE_PORTCTL0_I0_REG_P0_TRUNKNUM_MAX                         (0x00000003U)

#define CSL_ALE_I0_ALE_PORTCTL0_I0_REG_P0_MIRROR_SP_MASK                       (0x00000080U)
#define CSL_ALE_I0_ALE_PORTCTL0_I0_REG_P0_MIRROR_SP_SHIFT                      (0x00000007U)
#define CSL_ALE_I0_ALE_PORTCTL0_I0_REG_P0_MIRROR_SP_MAX                        (0x00000001U)

#define CSL_ALE_I0_ALE_PORTCTL0_I0_REG_P0_NO_SA_UPDATE_MASK                    (0x00000020U)
#define CSL_ALE_I0_ALE_PORTCTL0_I0_REG_P0_NO_SA_UPDATE_SHIFT                   (0x00000005U)
#define CSL_ALE_I0_ALE_PORTCTL0_I0_REG_P0_NO_SA_UPDATE_MAX                     (0x00000001U)

#define CSL_ALE_I0_ALE_PORTCTL0_I0_REG_P0_NO_LEARN_MASK                        (0x00000010U)
#define CSL_ALE_I0_ALE_PORTCTL0_I0_REG_P0_NO_LEARN_SHIFT                       (0x00000004U)
#define CSL_ALE_I0_ALE_PORTCTL0_I0_REG_P0_NO_LEARN_MAX                         (0x00000001U)

#define CSL_ALE_I0_ALE_PORTCTL0_I0_REG_P0_VID_INGRESS_CHECK_MASK               (0x00000008U)
#define CSL_ALE_I0_ALE_PORTCTL0_I0_REG_P0_VID_INGRESS_CHECK_SHIFT              (0x00000003U)
#define CSL_ALE_I0_ALE_PORTCTL0_I0_REG_P0_VID_INGRESS_CHECK_MAX                (0x00000001U)

#define CSL_ALE_I0_ALE_PORTCTL0_I0_REG_P0_DROP_UN_TAGGED_MASK                  (0x00000004U)
#define CSL_ALE_I0_ALE_PORTCTL0_I0_REG_P0_DROP_UN_TAGGED_SHIFT                 (0x00000002U)
#define CSL_ALE_I0_ALE_PORTCTL0_I0_REG_P0_DROP_UN_TAGGED_MAX                   (0x00000001U)

#define CSL_ALE_I0_ALE_PORTCTL0_I0_REG_P0_PORTSTATE_MASK                       (0x00000003U)
#define CSL_ALE_I0_ALE_PORTCTL0_I0_REG_P0_PORTSTATE_SHIFT                      (0x00000000U)
#define CSL_ALE_I0_ALE_PORTCTL0_I0_REG_P0_PORTSTATE_MAX                        (0x00000003U)

/* ALE_UVLAN_MEMBER */

#define CSL_ALE_ALE_UVLAN_MEMBER_UVLAN_MEMBER_LIST_MASK                        (0x000001FFU)
#define CSL_ALE_ALE_UVLAN_MEMBER_UVLAN_MEMBER_LIST_SHIFT                       (0x00000000U)
#define CSL_ALE_ALE_UVLAN_MEMBER_UVLAN_MEMBER_LIST_MAX                         (0x000001FFU)

/* ALE_UVLAN_URCAST */

#define CSL_ALE_ALE_UVLAN_URCAST_UVLAN_UNREG_MCAST_FLOOD_MASK_MASK             (0x000001FFU)
#define CSL_ALE_ALE_UVLAN_URCAST_UVLAN_UNREG_MCAST_FLOOD_MASK_SHIFT            (0x00000000U)
#define CSL_ALE_ALE_UVLAN_URCAST_UVLAN_UNREG_MCAST_FLOOD_MASK_MAX              (0x000001FFU)

/* ALE_UVLAN_RMCAST */

#define CSL_ALE_ALE_UVLAN_RMCAST_UVLAN_REG_MCAST_FLOOD_MASK_MASK               (0x000001FFU)
#define CSL_ALE_ALE_UVLAN_RMCAST_UVLAN_REG_MCAST_FLOOD_MASK_SHIFT              (0x00000000U)
#define CSL_ALE_ALE_UVLAN_RMCAST_UVLAN_REG_MCAST_FLOOD_MASK_MAX                (0x000001FFU)

/* ALE_UVLAN_UNTAG */

#define CSL_ALE_ALE_UVLAN_UNTAG_UVLAN_FORCE_UNTAGGED_EGRESS_MASK               (0x000001FFU)
#define CSL_ALE_ALE_UVLAN_UNTAG_UVLAN_FORCE_UNTAGGED_EGRESS_SHIFT              (0x00000000U)
#define CSL_ALE_ALE_UVLAN_UNTAG_UVLAN_FORCE_UNTAGGED_EGRESS_MAX                (0x000001FFU)

/* ALE_STAT_DIAG */

#define CSL_ALE_ALE_STAT_DIAG_PBCAST_DIAG_MASK                                 (0x00008000U)
#define CSL_ALE_ALE_STAT_DIAG_PBCAST_DIAG_SHIFT                                (0x0000000FU)
#define CSL_ALE_ALE_STAT_DIAG_PBCAST_DIAG_MAX                                  (0x00000001U)

#define CSL_ALE_ALE_STAT_DIAG_PORT_DIAG_MASK                                   (0x00000F00U)
#define CSL_ALE_ALE_STAT_DIAG_PORT_DIAG_SHIFT                                  (0x00000008U)
#define CSL_ALE_ALE_STAT_DIAG_PORT_DIAG_MAX                                    (0x0000000FU)

#define CSL_ALE_ALE_STAT_DIAG_STAT_DIAG_MASK                                   (0x0000000FU)
#define CSL_ALE_ALE_STAT_DIAG_STAT_DIAG_SHIFT                                  (0x00000000U)
#define CSL_ALE_ALE_STAT_DIAG_STAT_DIAG_MAX                                    (0x0000000FU)

/* ALE_OAM_LB_CTRL */

#define CSL_ALE_ALE_OAM_LB_CTRL_OAM_LB_CTRL_MASK                               (0x000001FFU)
#define CSL_ALE_ALE_OAM_LB_CTRL_OAM_LB_CTRL_SHIFT                              (0x00000000U)
#define CSL_ALE_ALE_OAM_LB_CTRL_OAM_LB_CTRL_MAX                                (0x000001FFU)

/* ALE_MSK_MUX0 */

#define CSL_ALE_ALE_MSK_MUX0_VLAN_MASK_MUX_0_MASK                              (0x00000003U)
#define CSL_ALE_ALE_MSK_MUX0_VLAN_MASK_MUX_0_SHIFT                             (0x00000000U)
#define CSL_ALE_ALE_MSK_MUX0_VLAN_MASK_MUX_0_MAX                               (0x00000003U)

/* I1_ALE_MSK_MUX1 */

#define CSL_ALE_I1_ALE_MSK_MUX1_I1_REG_VLAN_MASK_MUX_1_MASK                    (0x00000003U)
#define CSL_ALE_I1_ALE_MSK_MUX1_I1_REG_VLAN_MASK_MUX_1_SHIFT                   (0x00000000U)
#define CSL_ALE_I1_ALE_MSK_MUX1_I1_REG_VLAN_MASK_MUX_1_MAX                     (0x00000003U)

/* EGRESSOP */

#define CSL_ALE_EGRESSOP_EGRESS_OP_MASK                                        (0xFF000000U)
#define CSL_ALE_EGRESSOP_EGRESS_OP_SHIFT                                       (0x00000018U)
#define CSL_ALE_EGRESSOP_EGRESS_OP_MAX                                         (0x000000FFU)

#define CSL_ALE_EGRESSOP_EGRESS_TRK_MASK                                       (0x00E00000U)
#define CSL_ALE_EGRESSOP_EGRESS_TRK_SHIFT                                      (0x00000015U)
#define CSL_ALE_EGRESSOP_EGRESS_TRK_MAX                                        (0x00000007U)

#define CSL_ALE_EGRESSOP_TTL_CHECK_MASK                                        (0x00100000U)
#define CSL_ALE_EGRESSOP_TTL_CHECK_SHIFT                                       (0x00000014U)
#define CSL_ALE_EGRESSOP_TTL_CHECK_MAX                                         (0x00000001U)

#define CSL_ALE_EGRESSOP_DEST_PORTS_MASK                                       (0x000001FFU)
#define CSL_ALE_EGRESSOP_DEST_PORTS_SHIFT                                      (0x00000000U)
#define CSL_ALE_EGRESSOP_DEST_PORTS_MAX                                        (0x000001FFU)

/* POLICECFG0 */

#define CSL_ALE_POLICECFG0_PORT_MEN_MASK                                       (0x80000000U)
#define CSL_ALE_POLICECFG0_PORT_MEN_SHIFT                                      (0x0000001FU)
#define CSL_ALE_POLICECFG0_PORT_MEN_MAX                                        (0x00000001U)

#define CSL_ALE_POLICECFG0_TRUNKID_MASK                                        (0x40000000U)
#define CSL_ALE_POLICECFG0_TRUNKID_SHIFT                                       (0x0000001EU)
#define CSL_ALE_POLICECFG0_TRUNKID_MAX                                         (0x00000001U)

#define CSL_ALE_POLICECFG0_PORT_NUM_MASK                                       (0x1E000000U)
#define CSL_ALE_POLICECFG0_PORT_NUM_SHIFT                                      (0x00000019U)
#define CSL_ALE_POLICECFG0_PORT_NUM_MAX                                        (0x0000000FU)

#define CSL_ALE_POLICECFG0_PRI_MEN_MASK                                        (0x00080000U)
#define CSL_ALE_POLICECFG0_PRI_MEN_SHIFT                                       (0x00000013U)
#define CSL_ALE_POLICECFG0_PRI_MEN_MAX                                         (0x00000001U)

#define CSL_ALE_POLICECFG0_PRI_VAL_MASK                                        (0x00070000U)
#define CSL_ALE_POLICECFG0_PRI_VAL_SHIFT                                       (0x00000010U)
#define CSL_ALE_POLICECFG0_PRI_VAL_MAX                                         (0x00000007U)

#define CSL_ALE_POLICECFG0_ONU_MEN_MASK                                        (0x00008000U)
#define CSL_ALE_POLICECFG0_ONU_MEN_SHIFT                                       (0x0000000FU)
#define CSL_ALE_POLICECFG0_ONU_MEN_MAX                                         (0x00000001U)

#define CSL_ALE_POLICECFG0_ONU_INDEX_MASK                                      (0x000003FFU)
#define CSL_ALE_POLICECFG0_ONU_INDEX_SHIFT                                     (0x00000000U)
#define CSL_ALE_POLICECFG0_ONU_INDEX_MAX                                       (0x000003FFU)

/* POLICECFG1 */

#define CSL_ALE_POLICECFG1_DST_MEN_MASK                                        (0x80000000U)
#define CSL_ALE_POLICECFG1_DST_MEN_SHIFT                                       (0x0000001FU)
#define CSL_ALE_POLICECFG1_DST_MEN_MAX                                         (0x00000001U)

#define CSL_ALE_POLICECFG1_DST_INDEX_MASK                                      (0x03FF0000U)
#define CSL_ALE_POLICECFG1_DST_INDEX_SHIFT                                     (0x00000010U)
#define CSL_ALE_POLICECFG1_DST_INDEX_MAX                                       (0x000003FFU)

#define CSL_ALE_POLICECFG1_SRC_MEN_MASK                                        (0x00008000U)
#define CSL_ALE_POLICECFG1_SRC_MEN_SHIFT                                       (0x0000000FU)
#define CSL_ALE_POLICECFG1_SRC_MEN_MAX                                         (0x00000001U)

#define CSL_ALE_POLICECFG1_SRC_INDEX_MASK                                      (0x000003FFU)
#define CSL_ALE_POLICECFG1_SRC_INDEX_SHIFT                                     (0x00000000U)
#define CSL_ALE_POLICECFG1_SRC_INDEX_MAX                                       (0x000003FFU)

/* POLICECFG2 */

#define CSL_ALE_POLICECFG2_OVLAN_MEN_MASK                                      (0x80000000U)
#define CSL_ALE_POLICECFG2_OVLAN_MEN_SHIFT                                     (0x0000001FU)
#define CSL_ALE_POLICECFG2_OVLAN_MEN_MAX                                       (0x00000001U)

#define CSL_ALE_POLICECFG2_OVLAN_INDEX_MASK                                    (0x03FF0000U)
#define CSL_ALE_POLICECFG2_OVLAN_INDEX_SHIFT                                   (0x00000010U)
#define CSL_ALE_POLICECFG2_OVLAN_INDEX_MAX                                     (0x000003FFU)

#define CSL_ALE_POLICECFG2_IVLAN_MEN_MASK                                      (0x00008000U)
#define CSL_ALE_POLICECFG2_IVLAN_MEN_SHIFT                                     (0x0000000FU)
#define CSL_ALE_POLICECFG2_IVLAN_MEN_MAX                                       (0x00000001U)

#define CSL_ALE_POLICECFG2_IVLAN_INDEX_MASK                                    (0x000003FFU)
#define CSL_ALE_POLICECFG2_IVLAN_INDEX_SHIFT                                   (0x00000000U)
#define CSL_ALE_POLICECFG2_IVLAN_INDEX_MAX                                     (0x000003FFU)

/* POLICECFG3 */

#define CSL_ALE_POLICECFG3_ETHERTYPE_MEN_MASK                                  (0x80000000U)
#define CSL_ALE_POLICECFG3_ETHERTYPE_MEN_SHIFT                                 (0x0000001FU)
#define CSL_ALE_POLICECFG3_ETHERTYPE_MEN_MAX                                   (0x00000001U)

#define CSL_ALE_POLICECFG3_ETHERTYPE_INDEX_MASK                                (0x03FF0000U)
#define CSL_ALE_POLICECFG3_ETHERTYPE_INDEX_SHIFT                               (0x00000010U)
#define CSL_ALE_POLICECFG3_ETHERTYPE_INDEX_MAX                                 (0x000003FFU)

#define CSL_ALE_POLICECFG3_IPSRC_MEN_MASK                                      (0x00008000U)
#define CSL_ALE_POLICECFG3_IPSRC_MEN_SHIFT                                     (0x0000000FU)
#define CSL_ALE_POLICECFG3_IPSRC_MEN_MAX                                       (0x00000001U)

#define CSL_ALE_POLICECFG3_IPSRC_INDEX_MASK                                    (0x000003FFU)
#define CSL_ALE_POLICECFG3_IPSRC_INDEX_SHIFT                                   (0x00000000U)
#define CSL_ALE_POLICECFG3_IPSRC_INDEX_MAX                                     (0x000003FFU)

/* POLICECFG4 */

#define CSL_ALE_POLICECFG4_IPDST_MEN_MASK                                      (0x80000000U)
#define CSL_ALE_POLICECFG4_IPDST_MEN_SHIFT                                     (0x0000001FU)
#define CSL_ALE_POLICECFG4_IPDST_MEN_MAX                                       (0x00000001U)

#define CSL_ALE_POLICECFG4_IPDST_INDEX_MASK                                    (0x03FF0000U)
#define CSL_ALE_POLICECFG4_IPDST_INDEX_SHIFT                                   (0x00000010U)
#define CSL_ALE_POLICECFG4_IPDST_INDEX_MAX                                     (0x000003FFU)

/* POLICECFG6 */

#define CSL_ALE_POLICECFG6_PIR_IDLE_INC_VAL_MASK                               (0xFFFFFFFFU)
#define CSL_ALE_POLICECFG6_PIR_IDLE_INC_VAL_SHIFT                              (0x00000000U)
#define CSL_ALE_POLICECFG6_PIR_IDLE_INC_VAL_MAX                                (0xFFFFFFFFU)

/* POLICECFG7 */

#define CSL_ALE_POLICECFG7_CIR_IDLE_INC_VAL_MASK                               (0xFFFFFFFFU)
#define CSL_ALE_POLICECFG7_CIR_IDLE_INC_VAL_SHIFT                              (0x00000000U)
#define CSL_ALE_POLICECFG7_CIR_IDLE_INC_VAL_MAX                                (0xFFFFFFFFU)

/* POLICETBLCTL */

#define CSL_ALE_POLICETBLCTL_WRITE_ENABLE_MASK                                 (0x80000000U)
#define CSL_ALE_POLICETBLCTL_WRITE_ENABLE_SHIFT                                (0x0000001FU)
#define CSL_ALE_POLICETBLCTL_WRITE_ENABLE_MAX                                  (0x00000001U)

#define CSL_ALE_POLICETBLCTL_POL_TBL_IDX_MASK                                  (0x0000007FU)
#define CSL_ALE_POLICETBLCTL_POL_TBL_IDX_SHIFT                                 (0x00000000U)
#define CSL_ALE_POLICETBLCTL_POL_TBL_IDX_MAX                                   (0x0000007FU)

/* POLICECONTROL */

#define CSL_ALE_POLICECONTROL_POLICING_EN_MASK                                 (0x80000000U)
#define CSL_ALE_POLICECONTROL_POLICING_EN_SHIFT                                (0x0000001FU)
#define CSL_ALE_POLICECONTROL_POLICING_EN_MAX                                  (0x00000001U)

#define CSL_ALE_POLICECONTROL_RED_DROP_EN_MASK                                 (0x20000000U)
#define CSL_ALE_POLICECONTROL_RED_DROP_EN_SHIFT                                (0x0000001DU)
#define CSL_ALE_POLICECONTROL_RED_DROP_EN_MAX                                  (0x00000001U)

#define CSL_ALE_POLICECONTROL_YELLOW_DROP_EN_MASK                              (0x10000000U)
#define CSL_ALE_POLICECONTROL_YELLOW_DROP_EN_SHIFT                             (0x0000001CU)
#define CSL_ALE_POLICECONTROL_YELLOW_DROP_EN_MAX                               (0x00000001U)

#define CSL_ALE_POLICECONTROL_YELLOWTHRESH_MASK                                (0x07000000U)
#define CSL_ALE_POLICECONTROL_YELLOWTHRESH_SHIFT                               (0x00000018U)
#define CSL_ALE_POLICECONTROL_YELLOWTHRESH_MAX                                 (0x00000007U)

#define CSL_ALE_POLICECONTROL_POLMCHMODE_MASK                                  (0x00C00000U)
#define CSL_ALE_POLICECONTROL_POLMCHMODE_SHIFT                                 (0x00000016U)
#define CSL_ALE_POLICECONTROL_POLMCHMODE_MAX                                   (0x00000003U)

#define CSL_ALE_POLICECONTROL_PRIORITY_THREAD_EN_MASK                          (0x00200000U)
#define CSL_ALE_POLICECONTROL_PRIORITY_THREAD_EN_SHIFT                         (0x00000015U)
#define CSL_ALE_POLICECONTROL_PRIORITY_THREAD_EN_MAX                           (0x00000001U)

#define CSL_ALE_POLICECONTROL_MAC_ONLY_DEF_DIS_MASK                            (0x00100000U)
#define CSL_ALE_POLICECONTROL_MAC_ONLY_DEF_DIS_SHIFT                           (0x00000014U)
#define CSL_ALE_POLICECONTROL_MAC_ONLY_DEF_DIS_MAX                             (0x00000001U)

/* POLICETESTCTL */

#define CSL_ALE_POLICETESTCTL_POL_CLRALL_HIT_MASK                              (0x80000000U)
#define CSL_ALE_POLICETESTCTL_POL_CLRALL_HIT_SHIFT                             (0x0000001FU)
#define CSL_ALE_POLICETESTCTL_POL_CLRALL_HIT_MAX                               (0x00000001U)

#define CSL_ALE_POLICETESTCTL_POL_CLRALL_REDHIT_MASK                           (0x40000000U)
#define CSL_ALE_POLICETESTCTL_POL_CLRALL_REDHIT_SHIFT                          (0x0000001EU)
#define CSL_ALE_POLICETESTCTL_POL_CLRALL_REDHIT_MAX                            (0x00000001U)

#define CSL_ALE_POLICETESTCTL_POL_CLRALL_YELLOWHIT_MASK                        (0x20000000U)
#define CSL_ALE_POLICETESTCTL_POL_CLRALL_YELLOWHIT_SHIFT                       (0x0000001DU)
#define CSL_ALE_POLICETESTCTL_POL_CLRALL_YELLOWHIT_MAX                         (0x00000001U)

#define CSL_ALE_POLICETESTCTL_POL_CLRSEL_ALL_MASK                              (0x10000000U)
#define CSL_ALE_POLICETESTCTL_POL_CLRSEL_ALL_SHIFT                             (0x0000001CU)
#define CSL_ALE_POLICETESTCTL_POL_CLRSEL_ALL_MAX                               (0x00000001U)

#define CSL_ALE_POLICETESTCTL_POL_TEST_IDX_MASK                                (0x0000007FU)
#define CSL_ALE_POLICETESTCTL_POL_TEST_IDX_SHIFT                               (0x00000000U)
#define CSL_ALE_POLICETESTCTL_POL_TEST_IDX_MAX                                 (0x0000007FU)

/* POLICEHSTAT */

#define CSL_ALE_POLICEHSTAT_POL_HIT_MASK                                       (0x80000000U)
#define CSL_ALE_POLICEHSTAT_POL_HIT_SHIFT                                      (0x0000001FU)
#define CSL_ALE_POLICEHSTAT_POL_HIT_MAX                                        (0x00000001U)

#define CSL_ALE_POLICEHSTAT_POL_REDHIT_MASK                                    (0x40000000U)
#define CSL_ALE_POLICEHSTAT_POL_REDHIT_SHIFT                                   (0x0000001EU)
#define CSL_ALE_POLICEHSTAT_POL_REDHIT_MAX                                     (0x00000001U)

#define CSL_ALE_POLICEHSTAT_POL_YELLOWHIT_MASK                                 (0x20000000U)
#define CSL_ALE_POLICEHSTAT_POL_YELLOWHIT_SHIFT                                (0x0000001DU)
#define CSL_ALE_POLICEHSTAT_POL_YELLOWHIT_MAX                                  (0x00000001U)

/* THREADMAPDEF */

#define CSL_ALE_THREADMAPDEF_DEFTHREAD_EN_MASK                                 (0x00008000U)
#define CSL_ALE_THREADMAPDEF_DEFTHREAD_EN_SHIFT                                (0x0000000FU)
#define CSL_ALE_THREADMAPDEF_DEFTHREAD_EN_MAX                                  (0x00000001U)

#define CSL_ALE_THREADMAPDEF_DEFTHREADVAL_MASK                                 (0x0000003FU)
#define CSL_ALE_THREADMAPDEF_DEFTHREADVAL_SHIFT                                (0x00000000U)
#define CSL_ALE_THREADMAPDEF_DEFTHREADVAL_MAX                                  (0x0000003FU)

/* THREADMAPCTL */

#define CSL_ALE_THREADMAPCTL_CLASSINDEX_MASK                                   (0x0000007FU)
#define CSL_ALE_THREADMAPCTL_CLASSINDEX_SHIFT                                  (0x00000000U)
#define CSL_ALE_THREADMAPCTL_CLASSINDEX_MAX                                    (0x0000007FU)

/* THREADMAPVAL */

#define CSL_ALE_THREADMAPVAL_THREAD_EN_MASK                                    (0x00008000U)
#define CSL_ALE_THREADMAPVAL_THREAD_EN_SHIFT                                   (0x0000000FU)
#define CSL_ALE_THREADMAPVAL_THREAD_EN_MAX                                     (0x00000001U)

#define CSL_ALE_THREADMAPVAL_THREADVAL_MASK                                    (0x0000003FU)
#define CSL_ALE_THREADMAPVAL_THREADVAL_SHIFT                                   (0x00000000U)
#define CSL_ALE_THREADMAPVAL_THREADVAL_MAX                                     (0x0000003FU)

#ifdef __cplusplus
}
#endif
#endif
