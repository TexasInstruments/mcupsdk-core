/*
 * Copyright (C) 2024 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of
 * its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef CSL_WIZ8M_SB_REFCLK100P0MHZ_16B_5P0GBPS_USB_1L1C_SR_C
#define CSL_WIZ8M_SB_REFCLK100P0MHZ_16B_5P0GBPS_USB_1L1C_SR_C

#include <stdint.h>
#include <ti/csl/soc.h>
#include <ti/csl/csl.h>

void csl_wiz8m_sb_refclk100p0MHz_16b_5p0Gbps_USB_1l1c_sr(uint32_t base_addr)
  {
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0000),31,24, (uint32_t)0x17);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0004),15, 8, (uint32_t)0x02);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0004), 7, 0, (uint32_t)0x0e);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0008),23,16, (uint32_t)0x2e);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0008),31,24, (uint32_t)0x2e);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0060), 7, 0, (uint32_t)0x4b);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0060),15, 8, (uint32_t)0x98);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0060),23,16, (uint32_t)0x60);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x00d0),31,24, (uint32_t)0x45);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x00e8),15, 8, (uint32_t)0x0e);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0220), 7, 0, (uint32_t)0x34);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0220),15, 8, (uint32_t)0x34);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0220),31,24, (uint32_t)0x37);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0224), 7, 0, (uint32_t)0x37);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0224),15, 8, (uint32_t)0x37);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0228),23,16, (uint32_t)0x37);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0228),31,24, (uint32_t)0x37);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x022c), 7, 0, (uint32_t)0x37);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x022c),15, 8, (uint32_t)0x37);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0230),15, 8, (uint32_t)0x2a);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0230),23,16, (uint32_t)0x2a);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0240),23,16, (uint32_t)0x10);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0240),31,24, (uint32_t)0x34);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0244), 7, 0, (uint32_t)0x40);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0244),23,16, (uint32_t)0x34);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0248),15, 8, (uint32_t)0x0d);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0258),15, 8, (uint32_t)0x16);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0258),23,16, (uint32_t)0x84);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0258),31,24, (uint32_t)0xf2);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x025c), 7, 0, (uint32_t)0x21);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0260), 7, 0, (uint32_t)0x27);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0260),15, 8, (uint32_t)0x04);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0268),15, 8, (uint32_t)0x04);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0288),15, 8, (uint32_t)0x2c);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0330),31,24, (uint32_t)0xa0);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0338),23,16, (uint32_t)0x03);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0338),31,24, (uint32_t)0x00);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x033c), 7, 0, (uint32_t)0x00);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0344),31,24, (uint32_t)0x18);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x034c), 7, 0, (uint32_t)0x18);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x039c),23,16, (uint32_t)0x3b);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0a04), 7, 0, (uint32_t)0x03);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0a14),31,24, (uint32_t)0x3c);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0a18),15, 8, (uint32_t)0x3c);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0a38), 7, 0, (uint32_t)0x3e);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0a38),15, 8, (uint32_t)0x3e);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0ae0), 7, 0, (uint32_t)0x07);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0b6c),23,16, (uint32_t)0xcd);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0b6c),31,24, (uint32_t)0x04);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0b98),23,16, (uint32_t)0x03);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x1400), 7, 0, (uint32_t)0x3f);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x1404),23,16, (uint32_t)0x6f);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x1404),31,24, (uint32_t)0x6f);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x140c), 7, 0, (uint32_t)0x6f);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x140c),15, 8, (uint32_t)0x6f);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x1410),15, 8, (uint32_t)0x27);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x1414), 7, 0, (uint32_t)0x0c);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x1414),23,16, (uint32_t)0x07);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x1418),23,16, (uint32_t)0x40);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x141c), 7, 0, (uint32_t)0x00);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x141c),15, 8, (uint32_t)0x1f);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x1428),31,24, (uint32_t)0x08);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x1434),31,24, (uint32_t)0x00);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x1444), 7, 0, (uint32_t)0x94);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x1460),31,24, (uint32_t)0x7f);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x1464), 7, 0, (uint32_t)0x43);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x1464),23,16, (uint32_t)0x6f);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x1464),31,24, (uint32_t)0x43);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x1484),23,16, (uint32_t)0x8f);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x1498), 7, 0, (uint32_t)0x4f);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x1498),23,16, (uint32_t)0x4f);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x007c),31,24, (uint32_t)0x0d);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0b90),15, 8, (uint32_t)0x0f);
  }
#endif
