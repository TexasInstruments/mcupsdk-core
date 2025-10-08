/*
 *   Copyright (c) 2022-23 Texas Instruments Incorporated
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
 */

#ifndef SDL_ECC_BUS_SAFETY_HW_H_
#define SDL_ECC_BUS_SAFETY_HW_H_

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************************************
* Register Definitions
****************************************************************************************************/
/* MSS */
#if defined (SOC_AM273X) || defined (SOC_AWR294X)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_CTRL                                       (0x000001A0U)
#define SDL_MSS_CTRL_NERROR_MASK                                               (0x000006DCU)
#elif defined (SOC_AM263X) || defined (SOC_AM263PX)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_CTRL                             (0x00018200U)
#elif defined (SOC_AM261X)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_CTRL                             (0x000187DCU)
#endif

/* DSS */
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_CTRL                             (0x00000800U)

/****************************************************************************************************
* Field Definition Macros
****************************************************************************************************/

/* BUS_SAFETY_CTRL */

#define SDL_CTRL_BUS_SAFETY_CTRL_ENABLE_MASK                                               (0x00000007U)
#define SDL_CTRL_BUS_SAFETY_CTRL_ENABLE_SHIFT                                              (0x00000000U)
#define SDL_CTRL_BUS_SAFETY_CTRL_ENABLE_RESETVAL                                           (0x00000000U)
#define SDL_CTRL_BUS_SAFETY_CTRL_ENABLE_MAX                                                (0x00000007U)

/* BUS_SAFETY_CTRL */

#define SDL_CTRL_BUS_SAFETY_CTRL_BUS_SAFETY_CTRL_ENABLE_MASK                               (0x00000007U)
#define SDL_CTRL_BUS_SAFETY_CTRL_BUS_SAFETY_CTRL_ENABLE_SHIFT                              (0x00000000U)
#define SDL_CTRL_BUS_SAFETY_CTRL_BUS_SAFETY_CTRL_ENABLE_RESETVAL                           (0x00000007U)
#define SDL_CTRL_BUS_SAFETY_CTRL_BUS_SAFETY_CTRL_ENABLE_MAX                                (0x00000007U)

#define SDL_CTRL_BUS_SAFETY_CTRL_BUS_SAFETY_CTRL_ERR_CLEAR_MASK                            (0x00000100U)
#define SDL_CTRL_BUS_SAFETY_CTRL_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT                           (0x00000008U)
#define SDL_CTRL_BUS_SAFETY_CTRL_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL                        (0x00000000U)
#define SDL_CTRL_BUS_SAFETY_CTRL_BUS_SAFETY_CTRL_ERR_CLEAR_MAX                             (0x00000001U)

/* BUS_SAFETY_FI */

#define SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_GLOBAL_MAIN_MASK                              (0x00000001U)
#define SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT                             (0x00000000U)
#define SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL                          (0x00000000U)
#define SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_GLOBAL_MAIN_MAX                               (0x00000001U)

#define SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_GLOBAL_SAFE_MASK                              (0x00000002U)
#define SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT                             (0x00000001U)
#define SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL                          (0x00000000U)
#define SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_GLOBAL_SAFE_MAX                               (0x00000001U)

#define SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_SEC_MASK                                      (0x00000010U)
#define SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_SEC_SHIFT                                     (0x00000004U)
#define SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_SEC_RESETVAL                                  (0x00000000U)
#define SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_SEC_MAX                                       (0x00000001U)

#define SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_DED_MASK                                      (0x00000020U)
#define SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_DED_SHIFT                                     (0x00000005U)
#define SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_DED_RESETVAL                                  (0x00000000U)
#define SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_DED_MAX                                       (0x00000001U)

#define SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_DATA_MASK                                     (0x0000FF00U)
#define SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_DATA_SHIFT                                    (0x00000008U)
#define SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_DATA_RESETVAL                                 (0x00000000U)
#define SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_DATA_MAX                                      (0x000000FFU)

#define SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_MAIN_MASK                                     (0x00FF0000U)
#define SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_MAIN_SHIFT                                    (0x00000010U)
#define SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_MAIN_RESETVAL                                 (0x00000000U)
#define SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_MAIN_MAX                                      (0x000000FFU)

#define SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_SAFE_MASK                                     (0xFF000000U)
#define SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_SAFE_SHIFT                                    (0x00000018U)
#define SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_SAFE_RESETVAL                                 (0x00000000U)
#define SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_SAFE_MAX                                      (0x000000FFU)

/* BUS_SAFETY_ERR */

#define SDL_CTRL_BUS_SAFETY_ERR_BUS_SAFETY_ERR_SEC_MASK                                    (0x00FF0000U)
#define SDL_CTRL_BUS_SAFETY_ERR_BUS_SAFETY_ERR_SEC_SHIFT                                   (0x00000010U)
#define SDL_CTRL_BUS_SAFETY_ERR_BUS_SAFETY_ERR_SEC_RESETVAL                                (0x00000000U)
#define SDL_CTRL_BUS_SAFETY_ERR_BUS_SAFETY_ERR_SEC_MAX                                     (0x000000FFU)

#define SDL_CTRL_BUS_SAFETY_ERR_BUS_SAFETY_ERR_DED_MASK                                    (0xFF000000U)
#define SDL_CTRL_BUS_SAFETY_ERR_BUS_SAFETY_ERR_DED_SHIFT                                   (0x00000018U)
#define SDL_CTRL_BUS_SAFETY_ERR_BUS_SAFETY_ERR_DED_RESETVAL                                (0x00000000U)
#define SDL_CTRL_BUS_SAFETY_ERR_BUS_SAFETY_ERR_DED_MAX                                     (0x000000FFU)

#if defined (SOC_AM261X)

/* BUS_SAFETY_ERR_STAT_CMD */

#define SDL_CTRL_BUS_SAFETY_ERR_STAT_CMD_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK                 (0x00000001U)
#define SDL_CTRL_BUS_SAFETY_ERR_STAT_CMD_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT                (0x00000000U)
#define SDL_CTRL_BUS_SAFETY_ERR_STAT_CMD_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL             (0x00000000U)
#define SDL_CTRL_BUS_SAFETY_ERR_STAT_CMD_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX                  (0x00000001U)

/* BUS_SAFETY_ERR_STAT_READ */

#define SDL_CTRL_BUS_SAFETY_ERR_STAT_READ_BUS_SAFETY_ERR_STAT_READ_STAT_MASK               (0x00000002U)
#define SDL_CTRL_BUS_SAFETY_ERR_STAT_READ_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT              (0x00000001U)
#define SDL_CTRL_BUS_SAFETY_ERR_STAT_READ_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL           (0x00000000U)
#define SDL_CTRL_BUS_SAFETY_ERR_STAT_READ_BUS_SAFETY_ERR_STAT_READ_STAT_MAX                (0x00000001U)

/* BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_CTRL_BUS_SAFETY_ERR_STAT_WRITE_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK             (0x00000004U)
#define SDL_CTRL_BUS_SAFETY_ERR_STAT_WRITE_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT            (0x00000002U)
#define SDL_CTRL_BUS_SAFETY_ERR_STAT_WRITE_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL         (0x00000000U)
#define SDL_CTRL_BUS_SAFETY_ERR_STAT_WRITE_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX              (0x00000001U)

/* BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_CTRL_BUS_SAFETY_ERR_STAT_WRITERESP_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK     (0x00000008U)
#define SDL_CTRL_BUS_SAFETY_ERR_STAT_WRITERESP_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT    (0x00000003U)
#define SDL_CTRL_BUS_SAFETY_ERR_STAT_WRITERESP_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_CTRL_BUS_SAFETY_ERR_STAT_WRITERESP_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX      (0x00000001U)

#else
/* BUS_SAFETY_ERR_STAT_CMD */

#define SDL_CTRL_BUS_SAFETY_ERR_STAT_CMD_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK                 (0xFFFFFFFFU)
#define SDL_CTRL_BUS_SAFETY_ERR_STAT_CMD_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT                (0x00000000U)
#define SDL_CTRL_BUS_SAFETY_ERR_STAT_CMD_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL             (0x00000000U)
#define SDL_CTRL_BUS_SAFETY_ERR_STAT_CMD_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX                  (0xFFFFFFFFU)

/* BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_CTRL_BUS_SAFETY_ERR_STAT_WRITE_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK             (0xFFFFFFFFU)
#define SDL_CTRL_BUS_SAFETY_ERR_STAT_WRITE_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT            (0x00000000U)
#define SDL_CTRL_BUS_SAFETY_ERR_STAT_WRITE_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL         (0x00000000U)
#define SDL_CTRL_BUS_SAFETY_ERR_STAT_WRITE_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX              (0xFFFFFFFFU)

/* BUS_SAFETY_ERR_STAT_READ */

#define SDL_CTRL_BUS_SAFETY_ERR_STAT_READ_BUS_SAFETY_ERR_STAT_READ_STAT_MASK               (0xFFFFFFFFU)
#define SDL_CTRL_BUS_SAFETY_ERR_STAT_READ_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT              (0x00000000U)
#define SDL_CTRL_BUS_SAFETY_ERR_STAT_READ_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL           (0x00000000U)
#define SDL_CTRL_BUS_SAFETY_ERR_STAT_READ_BUS_SAFETY_ERR_STAT_READ_STAT_MAX                (0xFFFFFFFFU)

/* BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_CTRL_BUS_SAFETY_ERR_STAT_WRITERESP_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK     (0xFFFFFFFFU)
#define SDL_CTRL_BUS_SAFETY_ERR_STAT_WRITERESP_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT    (0x00000000U)
#define SDL_CTRL_BUS_SAFETY_ERR_STAT_WRITERESP_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_CTRL_BUS_SAFETY_ERR_STAT_WRITERESP_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX      (0xFFFFFFFFU)
#endif


#ifdef __cplusplus
}
#endif
#endif
