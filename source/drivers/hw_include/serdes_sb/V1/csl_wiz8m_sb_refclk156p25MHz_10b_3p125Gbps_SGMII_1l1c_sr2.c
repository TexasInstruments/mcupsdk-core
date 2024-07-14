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

#ifndef CSL_WIZ8M_SB_REFCLK156P25MHZ_10B_3P125GBPS_SGMII_1L1C_SR2_C
#define CSL_WIZ8M_SB_REFCLK156P25MHZ_10B_3P125GBPS_SGMII_1L1C_SR2_C

#include <stdint.h>
#include <ti/csl/soc.h>
#include <ti/csl/csl.h>

void csl_wiz8m_sb_refclk156p25MHz_10b_3p125Gbps_SGMII_1l1c_sr2(uint32_t base_addr)
  {
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0000),31,24, (uint32_t)0x12);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0004), 7, 0, (uint32_t)0x04);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0004),15, 8, (uint32_t)0x02);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0008),23,16, (uint32_t)0x39);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0008),31,24, (uint32_t)0x39);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x005c),31,24, (uint32_t)0x90);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0060), 7, 0, (uint32_t)0x7b);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0060),23,16, (uint32_t)0x80);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x00d4), 7, 0, (uint32_t)0x12);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0200), 7, 0, (uint32_t)0x00);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0200),23,16, (uint32_t)0x11);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0220), 7, 0, (uint32_t)0x3c);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0220),15, 8, (uint32_t)0x3c);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0220),23,16, (uint32_t)0x1e);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0230),15, 8, (uint32_t)0x32);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0230),23,16, (uint32_t)0x32);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0240),23,16, (uint32_t)0x10);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0240),31,24, (uint32_t)0x34);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0244), 7, 0, (uint32_t)0x40);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0244),23,16, (uint32_t)0x3c);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0248),15, 8, (uint32_t)0x0f);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x024c),31,24, (uint32_t)0x01);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0258),15, 8, (uint32_t)0x13);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0258),31,24, (uint32_t)0x0b);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x025c), 7, 0, (uint32_t)0x22);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x025c),23,16, (uint32_t)0x08);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0260),31,24, (uint32_t)0x20);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0264),15, 8, (uint32_t)0x00);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0268), 7, 0, (uint32_t)0x03);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0268),15, 8, (uint32_t)0x04);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0278),31,24, (uint32_t)0xe8);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x027c), 7, 0, (uint32_t)0x04);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0288),15, 8, (uint32_t)0x2c);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x028c),31,24, (uint32_t)0x00);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0290), 7, 0, (uint32_t)0x00);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0330),15, 8, (uint32_t)0x00);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0330),31,24, (uint32_t)0x40);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0344),31,24, (uint32_t)0x18);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0a34),23,16, (uint32_t)0x10);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0a38), 7, 0, (uint32_t)0x6e);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0a38),15, 8, (uint32_t)0x6e);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0a50),23,16, (uint32_t)0x19);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0a50),31,24, (uint32_t)0x92);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0a54), 7, 0, (uint32_t)0x91);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0a58),15, 8, (uint32_t)0xc0);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0a58),31,24, (uint32_t)0x60);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0a5c), 7, 0, (uint32_t)0x84);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0a5c),15, 8, (uint32_t)0x14);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0a5c),23,16, (uint32_t)0x07);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0a60), 7, 0, (uint32_t)0x4b);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0a60),23,16, (uint32_t)0xb0);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0a60),31,24, (uint32_t)0xd0);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0a64), 7, 0, (uint32_t)0x10);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0a64),15, 8, (uint32_t)0x30);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0a68),15, 8, (uint32_t)0xc3);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0ad4), 7, 0, (uint32_t)0x03);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0ae0), 7, 0, (uint32_t)0x07);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0ae0),15, 8, (uint32_t)0x01);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0ae0),31,24, (uint32_t)0x01);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0ae8),23,16, (uint32_t)0x02);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0b00), 7, 0, (uint32_t)0x01);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0b6c),23,16, (uint32_t)0xa0);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0b6c),31,24, (uint32_t)0x01);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0b38),23,16, (uint32_t)0x70);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0a48),23,16, (uint32_t)0x09);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0a48),31,24, (uint32_t)0x71);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0a5c),23,16, (uint32_t)0x87);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0a5c),31,24, (uint32_t)0x75);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0a74), 7, 0, (uint32_t)0x37);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0a74),15, 8, (uint32_t)0x32);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0a74),23,16, (uint32_t)0x14);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0a74),31,24, (uint32_t)0x23);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x1400), 7, 0, (uint32_t)0x0f);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x1404),31,24, (uint32_t)0x01);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x140c),15, 8, (uint32_t)0x01);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x1434),31,24, (uint32_t)0x00);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x143c),15, 8, (uint32_t)0x43);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x1448),23,16, (uint32_t)0x01);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x144c), 7, 0, (uint32_t)0x9c);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x1458), 7, 0, (uint32_t)0x43);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x1460),15, 8, (uint32_t)0x43);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x1470),23,16, (uint32_t)0x43);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x1474),15, 8, (uint32_t)0x43);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x1484),23,16, (uint32_t)0x8f);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x1490),15, 8, (uint32_t)0x27);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x1498),15, 8, (uint32_t)0x01);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x007c),31,24, (uint32_t)0x0d);
  CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0b90),15, 8, (uint32_t)0x0f);
  }
#endif
