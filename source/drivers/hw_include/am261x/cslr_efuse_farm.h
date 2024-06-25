/********************************************************************
 * Copyright (C) 2020 Texas Instruments Incorporated.
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
 *  Name        : cslr_efuse_farm.h
*/
#ifndef CSLR_EFUSE_FARM_H_
#define CSLR_EFUSE_FARM_H_

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
    volatile uint32_t INSTRUCTION;
    volatile uint32_t ADDRESS;
    volatile uint32_t DATAREGISTERUPPER;
    volatile uint32_t DATAREGISTERLOWER;
    volatile uint32_t SYSTEMCONFIG;
    volatile uint32_t SYSTEMSTATUS;
    volatile uint32_t ACCUMULATOR;
    volatile uint32_t BOUNDARY;
    volatile uint32_t KEYFLAG;
    volatile uint32_t KEY;
    volatile uint32_t RELEASE;
    volatile uint32_t PINS;
    volatile uint32_t CRA;
    volatile uint32_t READ;
    volatile uint32_t PROGRAM;
    volatile uint32_t ERROR;
} CSL_efuse_farmRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_EFUSE_FARM_INSTRUCTION                                             (0x00000000U)
#define CSL_EFUSE_FARM_ADDRESS                                                 (0x00000004U)
#define CSL_EFUSE_FARM_DATAREGISTERUPPER                                       (0x00000008U)
#define CSL_EFUSE_FARM_DATAREGISTERLOWER                                       (0x0000000CU)
#define CSL_EFUSE_FARM_SYSTEMCONFIG                                            (0x00000010U)
#define CSL_EFUSE_FARM_SYSTEMSTATUS                                            (0x00000014U)
#define CSL_EFUSE_FARM_ACCUMULATOR                                             (0x00000018U)
#define CSL_EFUSE_FARM_BOUNDARY                                                (0x0000001CU)
#define CSL_EFUSE_FARM_KEYFLAG                                                 (0x00000020U)
#define CSL_EFUSE_FARM_KEY                                                     (0x00000024U)
#define CSL_EFUSE_FARM_RELEASE                                                 (0x00000028U)
#define CSL_EFUSE_FARM_PINS                                                    (0x0000002CU)
#define CSL_EFUSE_FARM_CRA                                                     (0x00000030U)
#define CSL_EFUSE_FARM_READ                                                    (0x00000034U)
#define CSL_EFUSE_FARM_PROGRAM                                                 (0x00000038U)
#define CSL_EFUSE_FARM_ERROR                                                   (0x0000003CU)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* INSTRUCTION */

#define CSL_EFUSE_FARM_INSTRUCTION_ADDHOLDDWI_MASK                             (0x0000FFFFU)
#define CSL_EFUSE_FARM_INSTRUCTION_ADDHOLDDWI_SHIFT                            (0x00000000U)
#define CSL_EFUSE_FARM_INSTRUCTION_ADDHOLDDWI_RESETVAL                         (0x00000000U)
#define CSL_EFUSE_FARM_INSTRUCTION_ADDHOLDDWI_MAX                              (0x0000FFFFU)

#define CSL_EFUSE_FARM_INSTRUCTION_NU1_MASK                                    (0x00FF0000U)
#define CSL_EFUSE_FARM_INSTRUCTION_NU1_SHIFT                                   (0x00000010U)
#define CSL_EFUSE_FARM_INSTRUCTION_NU1_RESETVAL                                (0x00000000U)
#define CSL_EFUSE_FARM_INSTRUCTION_NU1_MAX                                     (0x000000FFU)

#define CSL_EFUSE_FARM_INSTRUCTION_HOLDECI_MASK                                (0x1F000000U)
#define CSL_EFUSE_FARM_INSTRUCTION_HOLDECI_SHIFT                               (0x00000018U)
#define CSL_EFUSE_FARM_INSTRUCTION_HOLDECI_RESETVAL                            (0x00000000U)
#define CSL_EFUSE_FARM_INSTRUCTION_HOLDECI_MAX                                 (0x0000001FU)

#define CSL_EFUSE_FARM_INSTRUCTION_NU2_MASK                                    (0xE0000000U)
#define CSL_EFUSE_FARM_INSTRUCTION_NU2_SHIFT                                   (0x0000001DU)
#define CSL_EFUSE_FARM_INSTRUCTION_NU2_RESETVAL                                (0x00000000U)
#define CSL_EFUSE_FARM_INSTRUCTION_NU2_MAX                                     (0x00000007U)

#define CSL_EFUSE_FARM_INSTRUCTION_RESETVAL                                    (0x00000000U)

/* ADDRESS */

#define CSL_EFUSE_FARM_ADDRESS_ROW_MASK                                        (0x000007FFU)
#define CSL_EFUSE_FARM_ADDRESS_ROW_SHIFT                                       (0x00000000U)
#define CSL_EFUSE_FARM_ADDRESS_ROW_RESETVAL                                    (0x00000000U)
#define CSL_EFUSE_FARM_ADDRESS_ROW_MAX                                         (0x000007FFU)

#define CSL_EFUSE_FARM_ADDRESS_BLOCKSEL_MASK                                   (0x0000F800U)
#define CSL_EFUSE_FARM_ADDRESS_BLOCKSEL_SHIFT                                  (0x0000000BU)
#define CSL_EFUSE_FARM_ADDRESS_BLOCKSEL_RESETVAL                               (0x00000000U)
#define CSL_EFUSE_FARM_ADDRESS_BLOCKSEL_MAX                                    (0x0000001FU)

#define CSL_EFUSE_FARM_ADDRESS_NU1_MASK                                        (0xFFFF0000U)
#define CSL_EFUSE_FARM_ADDRESS_NU1_SHIFT                                       (0x00000010U)
#define CSL_EFUSE_FARM_ADDRESS_NU1_RESETVAL                                    (0x00000000U)
#define CSL_EFUSE_FARM_ADDRESS_NU1_MAX                                         (0x0000FFFFU)

#define CSL_EFUSE_FARM_ADDRESS_RESETVAL                                        (0x00000000U)

/* DATAREGISTERUPPER */

#define CSL_EFUSE_FARM_DATAREGISTERUPPER_REDUNDANTROW_MASK                     (0x00000001U)
#define CSL_EFUSE_FARM_DATAREGISTERUPPER_REDUNDANTROW_SHIFT                    (0x00000000U)
#define CSL_EFUSE_FARM_DATAREGISTERUPPER_REDUNDANTROW_RESETVAL                 (0x00000000U)
#define CSL_EFUSE_FARM_DATAREGISTERUPPER_REDUNDANTROW_MAX                      (0x00000001U)

#define CSL_EFUSE_FARM_DATAREGISTERUPPER_READPROTECT_MASK                      (0x00000002U)
#define CSL_EFUSE_FARM_DATAREGISTERUPPER_READPROTECT_SHIFT                     (0x00000001U)
#define CSL_EFUSE_FARM_DATAREGISTERUPPER_READPROTECT_RESETVAL                  (0x00000000U)
#define CSL_EFUSE_FARM_DATAREGISTERUPPER_READPROTECT_MAX                       (0x00000001U)

#define CSL_EFUSE_FARM_DATAREGISTERUPPER_WRITEPROTECT_MASK                     (0x00000004U)
#define CSL_EFUSE_FARM_DATAREGISTERUPPER_WRITEPROTECT_SHIFT                    (0x00000002U)
#define CSL_EFUSE_FARM_DATAREGISTERUPPER_WRITEPROTECT_RESETVAL                 (0x00000000U)
#define CSL_EFUSE_FARM_DATAREGISTERUPPER_WRITEPROTECT_MAX                      (0x00000001U)

#define CSL_EFUSE_FARM_DATAREGISTERUPPER_NU1_MASK                              (0xFFFFFFF8U)
#define CSL_EFUSE_FARM_DATAREGISTERUPPER_NU1_SHIFT                             (0x00000003U)
#define CSL_EFUSE_FARM_DATAREGISTERUPPER_NU1_RESETVAL                          (0x00000000U)
#define CSL_EFUSE_FARM_DATAREGISTERUPPER_NU1_MAX                               (0x1FFFFFFFU)

#define CSL_EFUSE_FARM_DATAREGISTERUPPER_RESETVAL                              (0x00000000U)

/* DATAREGISTERLOWER */

#define CSL_EFUSE_FARM_DATAREGISTERLOWER_REDUNDANTROW_MASK                     (0x0000003FU)
#define CSL_EFUSE_FARM_DATAREGISTERLOWER_REDUNDANTROW_SHIFT                    (0x00000000U)
#define CSL_EFUSE_FARM_DATAREGISTERLOWER_REDUNDANTROW_RESETVAL                 (0x00000000U)
#define CSL_EFUSE_FARM_DATAREGISTERLOWER_REDUNDANTROW_MAX                      (0x0000003FU)

#define CSL_EFUSE_FARM_DATAREGISTERLOWER_REDUNDANCYENABLE_MASK                 (0x00000040U)
#define CSL_EFUSE_FARM_DATAREGISTERLOWER_REDUNDANCYENABLE_SHIFT                (0x00000006U)
#define CSL_EFUSE_FARM_DATAREGISTERLOWER_REDUNDANCYENABLE_RESETVAL             (0x00000000U)
#define CSL_EFUSE_FARM_DATAREGISTERLOWER_REDUNDANCYENABLE_MAX                  (0x00000001U)

#define CSL_EFUSE_FARM_DATAREGISTERLOWER_NU1_MASK                              (0x3FFFFF80U)
#define CSL_EFUSE_FARM_DATAREGISTERLOWER_NU1_SHIFT                             (0x00000007U)
#define CSL_EFUSE_FARM_DATAREGISTERLOWER_NU1_RESETVAL                          (0x00000000U)
#define CSL_EFUSE_FARM_DATAREGISTERLOWER_NU1_MAX                               (0x007FFFFFU)

#define CSL_EFUSE_FARM_DATAREGISTERLOWER_NU2_MASK                              (0x40000000U)
#define CSL_EFUSE_FARM_DATAREGISTERLOWER_NU2_SHIFT                             (0x0000001EU)
#define CSL_EFUSE_FARM_DATAREGISTERLOWER_NU2_RESETVAL                          (0x00000000U)
#define CSL_EFUSE_FARM_DATAREGISTERLOWER_NU2_MAX                               (0x00000001U)

#define CSL_EFUSE_FARM_DATAREGISTERLOWER_DISABLELOADFUSESCAN_MASK              (0x80000000U)
#define CSL_EFUSE_FARM_DATAREGISTERLOWER_DISABLELOADFUSESCAN_SHIFT             (0x0000001FU)
#define CSL_EFUSE_FARM_DATAREGISTERLOWER_DISABLELOADFUSESCAN_RESETVAL          (0x00000000U)
#define CSL_EFUSE_FARM_DATAREGISTERLOWER_DISABLELOADFUSESCAN_MAX               (0x00000001U)

#define CSL_EFUSE_FARM_DATAREGISTERLOWER_RESETVAL                              (0x00000000U)

/* SYSTEMCONFIG */

#define CSL_EFUSE_FARM_SYSTEMCONFIG_INTAUTOGATECTL_MASK                        (0x00000001U)
#define CSL_EFUSE_FARM_SYSTEMCONFIG_INTAUTOGATECTL_SHIFT                       (0x00000000U)
#define CSL_EFUSE_FARM_SYSTEMCONFIG_INTAUTOGATECTL_RESETVAL                    (0x00000001U)
#define CSL_EFUSE_FARM_SYSTEMCONFIG_INTAUTOGATECTL_MAX                         (0x00000001U)

#define CSL_EFUSE_FARM_SYSTEMCONFIG_SWRESET_MASK                               (0x00000002U)
#define CSL_EFUSE_FARM_SYSTEMCONFIG_SWRESET_SHIFT                              (0x00000001U)
#define CSL_EFUSE_FARM_SYSTEMCONFIG_SWRESET_RESETVAL                           (0x00000000U)
#define CSL_EFUSE_FARM_SYSTEMCONFIG_SWRESET_MAX                                (0x00000001U)

#define CSL_EFUSE_FARM_SYSTEMCONFIG_WAKEUPGENCTL_MASK                          (0x00000004U)
#define CSL_EFUSE_FARM_SYSTEMCONFIG_WAKEUPGENCTL_SHIFT                         (0x00000002U)
#define CSL_EFUSE_FARM_SYSTEMCONFIG_WAKEUPGENCTL_RESETVAL                      (0x00000000U)
#define CSL_EFUSE_FARM_SYSTEMCONFIG_WAKEUPGENCTL_MAX                           (0x00000001U)

#define CSL_EFUSE_FARM_SYSTEMCONFIG_IDLEREQACK_MASK                            (0x00000018U)
#define CSL_EFUSE_FARM_SYSTEMCONFIG_IDLEREQACK_SHIFT                           (0x00000003U)
#define CSL_EFUSE_FARM_SYSTEMCONFIG_IDLEREQACK_RESETVAL                        (0x00000002U)
#define CSL_EFUSE_FARM_SYSTEMCONFIG_IDLEREQACK_MAX                             (0x00000003U)

#define CSL_EFUSE_FARM_SYSTEMCONFIG_SENSITIVITY_MASK                           (0x00000020U)
#define CSL_EFUSE_FARM_SYSTEMCONFIG_SENSITIVITY_SHIFT                          (0x00000005U)
#define CSL_EFUSE_FARM_SYSTEMCONFIG_SENSITIVITY_RESETVAL                       (0x00000000U)
#define CSL_EFUSE_FARM_SYSTEMCONFIG_SENSITIVITY_MAX                            (0x00000001U)

#define CSL_EFUSE_FARM_SYSTEMCONFIG_REACTION_MASK                              (0x00000040U)
#define CSL_EFUSE_FARM_SYSTEMCONFIG_REACTION_SHIFT                             (0x00000006U)
#define CSL_EFUSE_FARM_SYSTEMCONFIG_REACTION_RESETVAL                          (0x00000000U)
#define CSL_EFUSE_FARM_SYSTEMCONFIG_REACTION_MAX                               (0x00000001U)

#define CSL_EFUSE_FARM_SYSTEMCONFIG_NU1_MASK                                   (0x00000080U)
#define CSL_EFUSE_FARM_SYSTEMCONFIG_NU1_SHIFT                                  (0x00000007U)
#define CSL_EFUSE_FARM_SYSTEMCONFIG_NU1_RESETVAL                               (0x00000000U)
#define CSL_EFUSE_FARM_SYSTEMCONFIG_NU1_MAX                                    (0x00000001U)

#define CSL_EFUSE_FARM_SYSTEMCONFIG_CLKCTL_MASK                                (0x00000F00U)
#define CSL_EFUSE_FARM_SYSTEMCONFIG_CLKCTL_SHIFT                               (0x00000008U)
#define CSL_EFUSE_FARM_SYSTEMCONFIG_CLKCTL_RESETVAL                            (0x00000000U)
#define CSL_EFUSE_FARM_SYSTEMCONFIG_CLKCTL_MAX                                 (0x0000000FU)

#define CSL_EFUSE_FARM_SYSTEMCONFIG_PWRMGMTCTRL_MASK                           (0x00003000U)
#define CSL_EFUSE_FARM_SYSTEMCONFIG_PWRMGMTCTRL_SHIFT                          (0x0000000CU)
#define CSL_EFUSE_FARM_SYSTEMCONFIG_PWRMGMTCTRL_RESETVAL                       (0x00000000U)
#define CSL_EFUSE_FARM_SYSTEMCONFIG_PWRMGMTCTRL_MAX                            (0x00000003U)

#define CSL_EFUSE_FARM_SYSTEMCONFIG_NU2_MASK                                   (0xFFFFC000U)
#define CSL_EFUSE_FARM_SYSTEMCONFIG_NU2_SHIFT                                  (0x0000000EU)
#define CSL_EFUSE_FARM_SYSTEMCONFIG_NU2_RESETVAL                               (0x00000000U)
#define CSL_EFUSE_FARM_SYSTEMCONFIG_NU2_MAX                                    (0x0003FFFFU)

#define CSL_EFUSE_FARM_SYSTEMCONFIG_RESETVAL                                   (0x00000011U)

/* SYSTEMSTATUS */

#define CSL_EFUSE_FARM_SYSTEMSTATUS_RESETDONE_MASK                             (0x00000001U)
#define CSL_EFUSE_FARM_SYSTEMSTATUS_RESETDONE_SHIFT                            (0x00000000U)
#define CSL_EFUSE_FARM_SYSTEMSTATUS_RESETDONE_RESETVAL                         (0x00000001U)
#define CSL_EFUSE_FARM_SYSTEMSTATUS_RESETDONE_MAX                              (0x00000001U)

#define CSL_EFUSE_FARM_SYSTEMSTATUS_NU1_MASK                                   (0xFFFFFFFEU)
#define CSL_EFUSE_FARM_SYSTEMSTATUS_NU1_SHIFT                                  (0x00000001U)
#define CSL_EFUSE_FARM_SYSTEMSTATUS_NU1_RESETVAL                               (0x00000000U)
#define CSL_EFUSE_FARM_SYSTEMSTATUS_NU1_MAX                                    (0x7FFFFFFFU)

#define CSL_EFUSE_FARM_SYSTEMSTATUS_RESETVAL                                   (0x00000001U)

/* ACCUMULATOR */

#define CSL_EFUSE_FARM_ACCUMULATOR_ACCUMULATOR_MASK                            (0x00FFFFFFU)
#define CSL_EFUSE_FARM_ACCUMULATOR_ACCUMULATOR_SHIFT                           (0x00000000U)
#define CSL_EFUSE_FARM_ACCUMULATOR_ACCUMULATOR_RESETVAL                        (0x00000000U)
#define CSL_EFUSE_FARM_ACCUMULATOR_ACCUMULATOR_MAX                             (0x00FFFFFFU)

#define CSL_EFUSE_FARM_ACCUMULATOR_NU1_MASK                                    (0xFF000000U)
#define CSL_EFUSE_FARM_ACCUMULATOR_NU1_SHIFT                                   (0x00000018U)
#define CSL_EFUSE_FARM_ACCUMULATOR_NU1_RESETVAL                                (0x00000000U)
#define CSL_EFUSE_FARM_ACCUMULATOR_NU1_MAX                                     (0x000000FFU)

#define CSL_EFUSE_FARM_ACCUMULATOR_RESETVAL                                    (0x00000000U)

/* BOUNDARY */

#define CSL_EFUSE_FARM_BOUNDARY_ENABLE_MASK                                    (0x00000001U)
#define CSL_EFUSE_FARM_BOUNDARY_ENABLE_SHIFT                                   (0x00000000U)
#define CSL_EFUSE_FARM_BOUNDARY_ENABLE_RESETVAL                                (0x00000000U)
#define CSL_EFUSE_FARM_BOUNDARY_ENABLE_MAX                                     (0x00000001U)

#define CSL_EFUSE_FARM_BOUNDARY_SYSWSREADSTATES_MASK                           (0x0000001EU)
#define CSL_EFUSE_FARM_BOUNDARY_SYSWSREADSTATES_SHIFT                          (0x00000001U)
#define CSL_EFUSE_FARM_BOUNDARY_SYSWSREADSTATES_RESETVAL                       (0x00000000U)
#define CSL_EFUSE_FARM_BOUNDARY_SYSWSREADSTATES_MAX                            (0x0000000FU)

#define CSL_EFUSE_FARM_BOUNDARY_SYSREPAIREN_MASK                               (0x00000060U)
#define CSL_EFUSE_FARM_BOUNDARY_SYSREPAIREN_SHIFT                              (0x00000005U)
#define CSL_EFUSE_FARM_BOUNDARY_SYSREPAIREN_RESETVAL                           (0x00000000U)
#define CSL_EFUSE_FARM_BOUNDARY_SYSREPAIREN_MAX                                (0x00000003U)

#define CSL_EFUSE_FARM_BOUNDARY_SYSDIEDAUTOLOADEN_MASK                         (0x00000080U)
#define CSL_EFUSE_FARM_BOUNDARY_SYSDIEDAUTOLOADEN_SHIFT                        (0x00000007U)
#define CSL_EFUSE_FARM_BOUNDARY_SYSDIEDAUTOLOADEN_RESETVAL                     (0x00000000U)
#define CSL_EFUSE_FARM_BOUNDARY_SYSDIEDAUTOLOADEN_MAX                          (0x00000001U)

#define CSL_EFUSE_FARM_BOUNDARY_EFCFDI_MASK                                    (0x00000100U)
#define CSL_EFUSE_FARM_BOUNDARY_EFCFDI_SHIFT                                   (0x00000008U)
#define CSL_EFUSE_FARM_BOUNDARY_EFCFDI_RESETVAL                                (0x00000000U)
#define CSL_EFUSE_FARM_BOUNDARY_EFCFDI_MAX                                     (0x00000001U)

#define CSL_EFUSE_FARM_BOUNDARY_NU1_MASK                                       (0x001FFE00U)
#define CSL_EFUSE_FARM_BOUNDARY_NU1_SHIFT                                      (0x00000009U)
#define CSL_EFUSE_FARM_BOUNDARY_NU1_RESETVAL                                   (0x00000000U)
#define CSL_EFUSE_FARM_BOUNDARY_NU1_MAX                                        (0x00000FFFU)

#define CSL_EFUSE_FARM_BOUNDARY_DSIABLERDROW_MASK                              (0x00200000U)
#define CSL_EFUSE_FARM_BOUNDARY_DSIABLERDROW_SHIFT                             (0x00000015U)
#define CSL_EFUSE_FARM_BOUNDARY_DSIABLERDROW_RESETVAL                          (0x00000000U)
#define CSL_EFUSE_FARM_BOUNDARY_DSIABLERDROW_MAX                               (0x00000001U)

#define CSL_EFUSE_FARM_BOUNDARY_NU2_MASK                                       (0xFFC00000U)
#define CSL_EFUSE_FARM_BOUNDARY_NU2_SHIFT                                      (0x00000016U)
#define CSL_EFUSE_FARM_BOUNDARY_NU2_RESETVAL                                   (0x00000000U)
#define CSL_EFUSE_FARM_BOUNDARY_NU2_MAX                                        (0x000003FFU)

#define CSL_EFUSE_FARM_BOUNDARY_RESETVAL                                       (0x00000000U)

/* KEYFLAG */

#define CSL_EFUSE_FARM_KEYFLAG_KEYFLAG_MASK                                    (0x00000001U)
#define CSL_EFUSE_FARM_KEYFLAG_KEYFLAG_SHIFT                                   (0x00000000U)
#define CSL_EFUSE_FARM_KEYFLAG_KEYFLAG_RESETVAL                                (0x00000000U)
#define CSL_EFUSE_FARM_KEYFLAG_KEYFLAG_MAX                                     (0x00000001U)

#define CSL_EFUSE_FARM_KEYFLAG_NU1_MASK                                        (0xFFFFFFFEU)
#define CSL_EFUSE_FARM_KEYFLAG_NU1_SHIFT                                       (0x00000001U)
#define CSL_EFUSE_FARM_KEYFLAG_NU1_RESETVAL                                    (0x00000000U)
#define CSL_EFUSE_FARM_KEYFLAG_NU1_MAX                                         (0x7FFFFFFFU)

#define CSL_EFUSE_FARM_KEYFLAG_RESETVAL                                        (0x00000000U)

/* KEY */

#define CSL_EFUSE_FARM_KEY_KEYCODE_MASK                                        (0xFFFFFFFFU)
#define CSL_EFUSE_FARM_KEY_KEYCODE_SHIFT                                       (0x00000000U)
#define CSL_EFUSE_FARM_KEY_KEYCODE_RESETVAL                                    (0x00000000U)
#define CSL_EFUSE_FARM_KEY_KEYCODE_MAX                                         (0xFFFFFFFFU)

#define CSL_EFUSE_FARM_KEY_RESETVAL                                            (0x00000000U)

/* RELEASE */

#define CSL_EFUSE_FARM_RELEASE_DAYEFUSE_MASK                                   (0x0000001FU)
#define CSL_EFUSE_FARM_RELEASE_DAYEFUSE_SHIFT                                  (0x00000000U)
#define CSL_EFUSE_FARM_RELEASE_DAYEFUSE_RESETVAL                               (0x00000005U)
#define CSL_EFUSE_FARM_RELEASE_DAYEFUSE_MAX                                    (0x0000001FU)

#define CSL_EFUSE_FARM_RELEASE_MONTHEFUSE_MASK                                 (0x000001E0U)
#define CSL_EFUSE_FARM_RELEASE_MONTHEFUSE_SHIFT                                (0x00000005U)
#define CSL_EFUSE_FARM_RELEASE_MONTHEFUSE_RESETVAL                             (0x00000001U)
#define CSL_EFUSE_FARM_RELEASE_MONTHEFUSE_MAX                                  (0x0000000FU)

#define CSL_EFUSE_FARM_RELEASE_YEAREFUSE_MASK                                  (0x0000FE00U)
#define CSL_EFUSE_FARM_RELEASE_YEAREFUSE_SHIFT                                 (0x00000009U)
#define CSL_EFUSE_FARM_RELEASE_YEAREFUSE_RESETVAL                              (0x00000007U)
#define CSL_EFUSE_FARM_RELEASE_YEAREFUSE_MAX                                   (0x0000007FU)

#define CSL_EFUSE_FARM_RELEASE_DAYODP_MASK                                     (0x001F0000U)
#define CSL_EFUSE_FARM_RELEASE_DAYODP_SHIFT                                    (0x00000010U)
#define CSL_EFUSE_FARM_RELEASE_DAYODP_RESETVAL                                 (0x00000000U)
#define CSL_EFUSE_FARM_RELEASE_DAYODP_MAX                                      (0x0000001FU)

#define CSL_EFUSE_FARM_RELEASE_MONTHODP_MASK                                   (0x01E00000U)
#define CSL_EFUSE_FARM_RELEASE_MONTHODP_SHIFT                                  (0x00000015U)
#define CSL_EFUSE_FARM_RELEASE_MONTHODP_RESETVAL                               (0x00000000U)
#define CSL_EFUSE_FARM_RELEASE_MONTHODP_MAX                                    (0x0000000FU)

#define CSL_EFUSE_FARM_RELEASE_YEARODP_MASK                                    (0xFE000000U)
#define CSL_EFUSE_FARM_RELEASE_YEARODP_SHIFT                                   (0x00000019U)
#define CSL_EFUSE_FARM_RELEASE_YEARODP_RESETVAL                                (0x00000000U)
#define CSL_EFUSE_FARM_RELEASE_YEARODP_MAX                                     (0x0000007FU)

#define CSL_EFUSE_FARM_RELEASE_RESETVAL                                        (0x00000E25U)

/* PINS */

#define CSL_EFUSE_FARM_PINS_SYSWSREADSTATES_MASK                               (0x0000000FU)
#define CSL_EFUSE_FARM_PINS_SYSWSREADSTATES_SHIFT                              (0x00000000U)
#define CSL_EFUSE_FARM_PINS_SYSWSREADSTATES_RESETVAL                           (0x00000007U)
#define CSL_EFUSE_FARM_PINS_SYSWSREADSTATES_MAX                                (0x0000000FU)

#define CSL_EFUSE_FARM_PINS_SYSREPAIREN_MASK                                   (0x00000030U)
#define CSL_EFUSE_FARM_PINS_SYSREPAIREN_SHIFT                                  (0x00000004U)
#define CSL_EFUSE_FARM_PINS_SYSREPAIREN_RESETVAL                               (0x00000000U)
#define CSL_EFUSE_FARM_PINS_SYSREPAIREN_MAX                                    (0x00000003U)

#define CSL_EFUSE_FARM_PINS_SYSDIEDAUTOLOADEN_MASK                             (0x00000040U)
#define CSL_EFUSE_FARM_PINS_SYSDIEDAUTOLOADEN_SHIFT                            (0x00000006U)
#define CSL_EFUSE_FARM_PINS_SYSDIEDAUTOLOADEN_RESETVAL                         (0x00000000U)
#define CSL_EFUSE_FARM_PINS_SYSDIEDAUTOLOADEN_MAX                              (0x00000001U)

#define CSL_EFUSE_FARM_PINS_EFCFCLRZ_MASK                                      (0x00000080U)
#define CSL_EFUSE_FARM_PINS_EFCFCLRZ_SHIFT                                     (0x00000007U)
#define CSL_EFUSE_FARM_PINS_EFCFCLRZ_RESETVAL                                  (0x00000001U)
#define CSL_EFUSE_FARM_PINS_EFCFCLRZ_MAX                                       (0x00000001U)

#define CSL_EFUSE_FARM_PINS_EFCREADY_MASK                                      (0x00000100U)
#define CSL_EFUSE_FARM_PINS_EFCREADY_SHIFT                                     (0x00000008U)
#define CSL_EFUSE_FARM_PINS_EFCREADY_RESETVAL                                  (0x00000000U)
#define CSL_EFUSE_FARM_PINS_EFCREADY_MAX                                       (0x00000001U)

#define CSL_EFUSE_FARM_PINS_NU1_MASK                                           (0xFFFFFE00U)
#define CSL_EFUSE_FARM_PINS_NU1_SHIFT                                          (0x00000009U)
#define CSL_EFUSE_FARM_PINS_NU1_RESETVAL                                       (0x00000040U)
#define CSL_EFUSE_FARM_PINS_NU1_MAX                                            (0x007FFFFFU)

#define CSL_EFUSE_FARM_PINS_RESETVAL                                           (0x00008087U)

/* CRA */

#define CSL_EFUSE_FARM_CRA_DATA_MASK                                           (0x0000003FU)
#define CSL_EFUSE_FARM_CRA_DATA_SHIFT                                          (0x00000000U)
#define CSL_EFUSE_FARM_CRA_DATA_RESETVAL                                       (0x00000000U)
#define CSL_EFUSE_FARM_CRA_DATA_MAX                                            (0x0000003FU)

#define CSL_EFUSE_FARM_CRA_NU1_MASK                                            (0xFFFFFFC0U)
#define CSL_EFUSE_FARM_CRA_NU1_SHIFT                                           (0x00000006U)
#define CSL_EFUSE_FARM_CRA_NU1_RESETVAL                                        (0x00000000U)
#define CSL_EFUSE_FARM_CRA_NU1_MAX                                             (0x03FFFFFFU)

#define CSL_EFUSE_FARM_CRA_RESETVAL                                            (0x00000000U)

/* READ */

#define CSL_EFUSE_FARM_READ_COMBINEDMARGIN_MASK                                (0x00000003U)
#define CSL_EFUSE_FARM_READ_COMBINEDMARGIN_SHIFT                               (0x00000000U)
#define CSL_EFUSE_FARM_READ_COMBINEDMARGIN_RESETVAL                            (0x00000000U)
#define CSL_EFUSE_FARM_READ_COMBINEDMARGIN_MAX                                 (0x00000003U)

#define CSL_EFUSE_FARM_READ_READDATABIT_MASK                                   (0x00000004U)
#define CSL_EFUSE_FARM_READ_READDATABIT_SHIFT                                  (0x00000002U)
#define CSL_EFUSE_FARM_READ_READDATABIT_RESETVAL                               (0x00000000U)
#define CSL_EFUSE_FARM_READ_READDATABIT_MAX                                    (0x00000001U)

#define CSL_EFUSE_FARM_READ_NU1_MASK                                           (0x00000008U)
#define CSL_EFUSE_FARM_READ_NU1_SHIFT                                          (0x00000003U)
#define CSL_EFUSE_FARM_READ_NU1_RESETVAL                                       (0x00000000U)
#define CSL_EFUSE_FARM_READ_NU1_MAX                                            (0x00000001U)

#define CSL_EFUSE_FARM_READ_READCLKPULSEWIDTH_MASK                             (0x000000F0U)
#define CSL_EFUSE_FARM_READ_READCLKPULSEWIDTH_SHIFT                            (0x00000004U)
#define CSL_EFUSE_FARM_READ_READCLKPULSEWIDTH_RESETVAL                         (0x00000000U)
#define CSL_EFUSE_FARM_READ_READCLKPULSEWIDTH_MAX                              (0x0000000FU)

#define CSL_EFUSE_FARM_READ_NU2_MASK                                           (0xFFFFFF00U)
#define CSL_EFUSE_FARM_READ_NU2_SHIFT                                          (0x00000008U)
#define CSL_EFUSE_FARM_READ_NU2_RESETVAL                                       (0x00000000U)
#define CSL_EFUSE_FARM_READ_NU2_MAX                                            (0x00FFFFFFU)

#define CSL_EFUSE_FARM_READ_RESETVAL                                           (0x00000000U)

/* PROGRAM */

#define CSL_EFUSE_FARM_PROGRAM_WRITECLKPULSEWIDTH_MASK                         (0x000001FFU)
#define CSL_EFUSE_FARM_PROGRAM_WRITECLKPULSEWIDTH_SHIFT                        (0x00000000U)
#define CSL_EFUSE_FARM_PROGRAM_WRITECLKPULSEWIDTH_RESETVAL                     (0x00000000U)
#define CSL_EFUSE_FARM_PROGRAM_WRITECLKPULSEWIDTH_MAX                          (0x000001FFU)

#define CSL_EFUSE_FARM_PROGRAM_WRITEITERATION_MASK                             (0x00001E00U)
#define CSL_EFUSE_FARM_PROGRAM_WRITEITERATION_SHIFT                            (0x00000009U)
#define CSL_EFUSE_FARM_PROGRAM_WRITEITERATION_RESETVAL                         (0x00000000U)
#define CSL_EFUSE_FARM_PROGRAM_WRITEITERATION_MAX                              (0x0000000FU)

#define CSL_EFUSE_FARM_PROGRAM_SHORTSWITCH_MASK                                (0x00002000U)
#define CSL_EFUSE_FARM_PROGRAM_SHORTSWITCH_SHIFT                               (0x0000000DU)
#define CSL_EFUSE_FARM_PROGRAM_SHORTSWITCH_RESETVAL                            (0x00000000U)
#define CSL_EFUSE_FARM_PROGRAM_SHORTSWITCH_MAX                                 (0x00000001U)

#define CSL_EFUSE_FARM_PROGRAM_CLKSTALLPULSEWIDTH_MASK                         (0x3FFFC000U)
#define CSL_EFUSE_FARM_PROGRAM_CLKSTALLPULSEWIDTH_SHIFT                        (0x0000000EU)
#define CSL_EFUSE_FARM_PROGRAM_CLKSTALLPULSEWIDTH_RESETVAL                     (0x00000000U)
#define CSL_EFUSE_FARM_PROGRAM_CLKSTALLPULSEWIDTH_MAX                          (0x0000FFFFU)

#define CSL_EFUSE_FARM_PROGRAM_CMPDISABLE_MASK                                 (0x40000000U)
#define CSL_EFUSE_FARM_PROGRAM_CMPDISABLE_SHIFT                                (0x0000001EU)
#define CSL_EFUSE_FARM_PROGRAM_CMPDISABLE_RESETVAL                             (0x00000000U)
#define CSL_EFUSE_FARM_PROGRAM_CMPDISABLE_MAX                                  (0x00000001U)

#define CSL_EFUSE_FARM_PROGRAM_NU1_MASK                                        (0x80000000U)
#define CSL_EFUSE_FARM_PROGRAM_NU1_SHIFT                                       (0x0000001FU)
#define CSL_EFUSE_FARM_PROGRAM_NU1_RESETVAL                                    (0x00000000U)
#define CSL_EFUSE_FARM_PROGRAM_NU1_MAX                                         (0x00000001U)

#define CSL_EFUSE_FARM_PROGRAM_RESETVAL                                        (0x00000000U)

/* ERROR */

#define CSL_EFUSE_FARM_ERROR_STATUSCODE_MASK                                   (0x0000001FU)
#define CSL_EFUSE_FARM_ERROR_STATUSCODE_SHIFT                                  (0x00000000U)
#define CSL_EFUSE_FARM_ERROR_STATUSCODE_RESETVAL                               (0x00000000U)
#define CSL_EFUSE_FARM_ERROR_STATUSCODE_MAX                                    (0x0000001FU)

#define CSL_EFUSE_FARM_ERROR_INSTRUCTIONDONE_MASK                              (0x00000020U)
#define CSL_EFUSE_FARM_ERROR_INSTRUCTIONDONE_SHIFT                             (0x00000005U)
#define CSL_EFUSE_FARM_ERROR_INSTRUCTIONDONE_RESETVAL                          (0x00000000U)
#define CSL_EFUSE_FARM_ERROR_INSTRUCTIONDONE_MAX                               (0x00000001U)

#define CSL_EFUSE_FARM_ERROR_NU1_MASK                                          (0xFFFFFFC0U)
#define CSL_EFUSE_FARM_ERROR_NU1_SHIFT                                         (0x00000006U)
#define CSL_EFUSE_FARM_ERROR_NU1_RESETVAL                                      (0x00000000U)
#define CSL_EFUSE_FARM_ERROR_NU1_MAX                                           (0x03FFFFFFU)

#define CSL_EFUSE_FARM_ERROR_RESETVAL                                          (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
