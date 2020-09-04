/*
 * Copyright (C) 2021 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
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
 */


#ifndef MENU_HDSL_REG_INTO_DDR_H_
#define MENU_HDSL_REG_INTO_DDR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <drivers/hw_include/hw_types.h>

#define HWREG(x)                                                               \
        (*((volatile uint32_t *)(x)))
#define HWREGB(x)                                                              \
        (*((volatile uint8_t *)(x)))
#define HWREGH(x)                                                              \
        (*((volatile uint16_t *)(x)))

/* for DDR trace*/
#define DDR_START_OFFSET 0x80000000
#define DDR_LIMIT 0x3FFFFFFF
#define DDR_END_OFFSET 0xBFFFFFF

#define NUM_RESOURCES 5000 //4210752 (0x3fff_ffff div 0xff)
/* UDMA TR packet descriptor memory size - with one TR */
#define UDMA_TEST_TRPD_SIZE             (UDMA_GET_TRPD_TR15_SIZE(1U))

#define ENDAT_EN (0x1 << 26)
#define ENDAT_TX_CFG (0x10 | (23 << 16))
#define ENDAT_RX_CFG (0x10 | (2 << 16) | 7 | 0x08)
#define CTR_EN (1 << 3)
#define MAX_WAIT 20000
/* ISR for v-frame-reception */
static void HDSL_IsrFxn(void);
/*
 * @brief It will test DDR traces into DDR
 * \n Here user will decide how many copies of HDSL_Interface_Register
 * \n will be copied into DDR
 */
void traces_into_ddr();
/*
 * @brief It will test DDR traces into DDR using GPIO pins
 * \ connect J17 and J16 pint on AM64x board using wire
 * \n Set the J17 header pin on board as LOW (Active low)
 * \n wait for 1 second and print traces on UART
 * \n Set the J17 header pin on board as HIGH
 */
void TC_input_start_copy();
int HDSL_sync_calculation(void);
#ifdef __cplusplus
}
#endif

#endif