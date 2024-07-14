/*
 * Copyright (C) 2024 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of
 * its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef CSL_WIZ8M_SB_REFCLK100P0MHZ_32B_8GBPS_PCIE_1L1C_C
#define CSL_WIZ8M_SB_REFCLK100P0MHZ_32B_8GBPS_PCIE_1L1C_C

#include <stdint.h>
#include <drivers/hw_include/soc_config.h>
#include <drivers/hw_include/cslr.h>

void csl_wiz8m_sb_refclk100p0MHz_32b_8Gbps_PCIE_1l1c(uint32_t base_addr)
  {
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0004),15, 8, (uint32_t)0x02);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0060),15, 8, (uint32_t)0x98);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0088),31,24, (uint32_t)0x98);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x00d0),31,24, (uint32_t)0x45);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x00e8),15, 8, (uint32_t)0x0e);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0248),15, 8, (uint32_t)0x05);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0258),23,16, (uint32_t)0x83);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x026c), 7, 0, (uint32_t)0x83);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0270),23,16, (uint32_t)0x81);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x039c),23,16, (uint32_t)0x3b);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0a04), 7, 0, (uint32_t)0x03);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0b38),23,16, (uint32_t)0x70);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x1404),31,24, (uint32_t)0x0f);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x1408), 7, 0, (uint32_t)0x4f);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x140c),15, 8, (uint32_t)0x0f);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x140c),23,16, (uint32_t)0x4f);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x1414),23,16, (uint32_t)0x07);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x1418),23,16, (uint32_t)0x7f);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x141c),15, 8, (uint32_t)0x0f);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x144c), 7, 0, (uint32_t)0x9a);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x1488),23,16, (uint32_t)0x32);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x1494),31,24, (uint32_t)0x80);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x1498),15, 8, (uint32_t)0x0f);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x1498),23,16, (uint32_t)0x4f);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x007c),31,24, (uint32_t)0x01);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0b90),15, 8, (uint32_t)0x02);
  }
#endif
