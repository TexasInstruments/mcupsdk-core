/********************************************************************
 * Copyright (C) 2013-2014 Texas Instruments Incorporated.
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
#ifndef CSLR_QSPI_H
#define CSLR_QSPI_H

#ifdef __cplusplus
extern "C"
{
#endif
#include <drivers/hw_include/cslr.h>
#include <drivers/hw_include/tistdtypes.h>


/**************************************************************************
* Register Overlay Structure for MMR
**************************************************************************/
typedef struct {
    volatile Uint32 PID;
    volatile Uint8  RSVD0[12];
    volatile Uint32 SYSCONFIG;
    volatile Uint8  RSVD1[12];
    volatile Uint32 INTR_STATUS_RAW_SET;
    volatile Uint32 INTR_STATUS_ENABLED_CLEAR;
    volatile Uint32 INTR_ENABLE_SET_REG;
    volatile Uint32 INTR_ENABLE_CLEAR_REG;
    volatile Uint32 INTC_EOI_REG;
    volatile Uint8  RSVD2[12];
    volatile Uint32 SPI_CLOCK_CNTRL_REG;
    volatile Uint32 SPI_DC_REG;
    volatile Uint32 SPI_CMD_REG;
    volatile Uint32 SPI_STATUS_REG;
    volatile Uint32 SPI_DATA_REG;
    volatile Uint32 SPI_SETUP0_REG;
    volatile Uint32 SPI_SETUP1_REG;
    volatile Uint32 SPI_SETUP2_REG;
    volatile Uint32 SPI_SETUP3_REG;
    volatile Uint32 SPI_SWITCH_REG;
    volatile Uint32 SPI_DATA_REG_1;
    volatile Uint32 SPI_DATA_REG_2;
    volatile Uint32 SPI_DATA_REG_3;
    volatile Uint8  RSVD3[12];
} CSL_QspiRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

/* PID */
#define CSL_QSPI_PID                                            (0x0U)

/* SYSCONFIG */
#define CSL_QSPI_SYSCONFIG                                      (0x10U)

/* This register contains the raw interrupt status as defined in HL0.8 */
#define CSL_QSPI_INTR_STATUS_RAW_SET                            (0x20U)

/* Interrupt Status Enabled/Clear Register This register contains the enabled
 * interrupt status as defined in HL0.8 */
#define CSL_QSPI_INTR_STATUS_ENABLED_CLEAR                      (0x24U)

/* Interrupt Enable/Set Register This register contains the enable status as
 * defined in HL0.8 */
#define CSL_QSPI_INTR_ENABLE_SET_REG                            (0x28U)

/* Interrupt Enable/Clear Register This register contains the enable status as
 * defined in HL0.8 */
#define CSL_QSPI_INTR_ENABLE_CLEAR_REG                          (0x2CU)

/* INTC EOI Register This register contains the EOI vector register contents
 * as defined by HL0.8 */
#define CSL_QSPI_INTC_EOI_REG                                   (0x30U)

/* SPI Clock Control (SPICC) Register SPICC controls the SPI clock generation
 * The SPICC controls the SPI clock generation. The input for clock division
 * is the input SPI_CLK signal. The output clock will be divided by DCLK_DIV+1
 * to provide the output SPI interface clock as well as controlling the main
 * portion of the design. Note that loading a value or 0 input DCLK_DIV will
 * force the input SPI_CLK to be used directly for the SPI interface clock.
 * The value in DCLK_DIV is only loaded when CLKEN transitions from a 0 to 1
 * state. This register can only be written to when the SPI is not busy, as
 * defined by SPISR[0]. */
#define CSL_QSPI_SPI_CLOCK_CNTRL_REG                            (0x40U)

/* SPI Device Control (SPIDC) Register The SPIDC controls the different modes
 * for each output chip select The SPIDC controls the different modes for each
 * output chip select. NOTE: The combination of [CKPn, CKPHn] creates the “SPI
 * mode”. Most serial Flash devices only support SPI modes 0 and 3. SPI
 * devices transmit and receive data on opposite edge’s of the SPI clock. Note
 * that changing the clock polarity also swaps the transmit/receive clock edge
 * relationship. If a slave device states that it receives data on the rising
 * edge and transmits on the falling edge of the clock, then it can only
 * support mode 0 or 3 (CKPn = 0, CKPHn = 0 OR CKPn = 1, CKPHn = 1). For
 * details concerning modes and transmit/receive waveforms, please see Figure
 * 6. This register can only be written to when the SPI is not busy, as
 * defined by SPISR[0]. */
#define CSL_QSPI_SPI_DC_REG                                     (0x44U)

/* SPI Command Register (SPICR) Sets up the SPI command Since the SPI
 * convention is to always shift in new data (LSB position) while shifting out
 * data from the MSB position the ‘read’, ‘write’ and ‘read dual’ commands
 * actually just initiate a transfer. Write commands will assert the
 * spi_dout_oe_n signal to enable the tristate buffer for spi_dout_o. Read
 * commands will deassert the spi_dout_oe_n signal to disable the tristate
 * buffer for spi_dout_o. Executing any of these commands will initiate the
 * next word transfer (except for the case of “read dual”). The ‘read dual’
 * command will also initiate a transfer like read or write. However, this
 * command is used to communicate with serial Flash devices which support dual
 * read output. Dual read output mode uses the spi_dout signal as an input
 * along with the spi_din input (spi_dout is therefore a bidirectional
 * signal). WLEN transfers must be even. The spi_din input will contain the
 * odd number bytes and the spi_dout bidirectional/input will contain the even
 * number bytes. This read mode effectively doubles the read bandwidth of the
 * design. This command can only be used on those devices which support a dual
 * read command. The ‘read quad’ command is similar to the read dual command,
 * except it uses a 6 pin interface. The In particular, transfers are started
 * by writing to byte 2 of this register (only byte 2 will start a transfer).
 * Writing a “reserved” value to the CMD register will terminate the frame
 * transfer. This can be used to abort a frame if desired. Reserved commands
 * are CMD = 00 and CMD_MODE=1 AND CMD=11. This register can only be written
 * to when the SPI is not busy, as defined by SPISR[0]. If the SPI clock has
 * not been enabled (SPICC[31] = 0), then writing to this register will NOT
 * start the SPI transfer. The transfer will not be queued to start upon the
 * activation of the clock. The transfer will ONLY start if SPICC[31] = 1 when
 * this register is written. */
#define CSL_QSPI_SPI_CMD_REG                                    (0x48U)

/* SPI Status Register (SPISR) The SPI Status Register contains indicators to
 * allow the user to monitor the progression of a frame transfer The SPI
 * Status Register contains indicators to allow the user to monitor the
 * progression of a frame transfer. The word complete (WC) and frame complete
 * (FC) bits are used as stimulus for generating interrupts. Setting the
 * corresponding interrupt enable bit in the SPI Command Register will allow
 * these events to generate an interrupt. The WDCNT field will reset itself
 * when transitioning from the LOAD state to the SHIFT state. THE WC and FC
 * fields will be reset every time the user writes to the SPI Command Register
 * or the SPI Status Register is read. The BUSY bit of this register
 * (SPISR[0]) is used to block write access to the SPICC, SPIDC, SPICR and
 * SPIDR registers. This is done to keep control parameters from changing the
 * interface protocol signals while a transfer is in progress. If a write is
 * made while BUSY is active, the write will simply not occur. BUSY is set
 * immediately when SPICR[18:16] are written and BUSY will be cleared when the
 * current word has completed transfer (on SPISR[1]). However, if the SPI
 * clock enable is not set (SPICC[31] = 0) and a command is executed
 * (SPICR[18:16] is written to start a command), BUSY will NOT be set and the
 * command will not be executed. BUSY will ONLY go high if SPICC[31] = 1 and
 * SPICR[18:16] is written. */
#define CSL_QSPI_SPI_STATUS_REG                                 (0x4CU)

/* SPI Data Register (SPIDR) Data received in the data register is shifted
 * into the LSB position and the contents of the register are shifted to the
 * left. The register is cleared between reads or writes Data received in the
 * data register is shifted into the LSB position and the contents of the
 * register are shifted to the left. The register is cleared between reads or
 * writes. When writing data to the register it should be right justified so
 * pre-shifting is not required. The word length field will determine the
 * location of the most significant bit and the bit position that will be
 * shifted out first during a write. In order to shift out byte data the word
 * length should be set to ‘7’ and the data byte should be written to the
 * lower byte of the data register. By setting the word length to ‘7’ the data
 * register will look like a pseudo 8-bit shift register. The word length
 * setting does not affect the VBUSP read or write operations. This register
 * can only be written to when the SPI is not busy, as defined by SPISR[0]. */
#define CSL_QSPI_SPI_DATA_REG                                   (0x50U)

/* Memory Mapped SPI Setup0 Register The Memory Mapped SPI Setup0 Register
 * contains the read/write command setup for the Memory Mapped Protocol
 * Translator (effecting chip select 0 output). The Memory Mapped SPI Setup
 * Register contains the read/write command setup for the Memory Mapped
 * Protocol Translator, request 0 input (effecting chip select 0 output). This
 * corresponds to MAddrSpace = 001. Note that by default (reset), the device
 * uses a write command of 2, read command of 3 and number of address bytes of
 * 3. This default covers most serial Flash devices, but can be changed. */
#define CSL_QSPI_SPI_SETUP0_REG                                 (0x54U)

/* Memory Mapped SPI Switch Register The Memory Mapped SPI Switch Register
 * allows the CPU to switch control of the core SPI module’s configuration
 * port between the configuration VBUSP port and the Memory Mapped Protocol
 * Translator The Memory Mapped SPI Switch Register allows the CPU to switch
 * control of the core SPI module’s configuration port between the
 * configuration VBUSP port and the Memory Mapped Protocol Translator. In
 * addition, an interrupt enable field is defined which is used to enable or
 * disable word count interrupt generation in memory mapped mode. */
#define CSL_QSPI_SPI_SWITCH_REG                                 (0x64U)

/* Memory Mapped SPI Setup1 Register The Memory Mapped SPI Setup1 Register
 * contains the read/write command setup for the Memory Mapped Protocol
 * Translator (effecting chip select 1 output). The Memory Mapped SPI Setup
 * Register contains the read/write command setup for the Memory Mapped
 * Protocol Translator, request 1 input (effecting chip select 1 output). This
 * corresponds to MAddrSpace = 010. Note that by default (reset), the device
 * uses a write command of 2, read command of 3 and number of address bytes of
 * 3. This default covers most serial Flash devices, but can be changed. */
#define CSL_QSPI_SPI_SETUP1_REG                                 (0x58U)

/* Memory Mapped SPI Setup2 Register The Memory Mapped SPI Setup2 Register
 * contains the read/write command setup for the Memory Mapped Protocol
 * Translator (effecting chip select 0 output). The Memory Mapped SPI Setup
 * Register contains the read/write command setup for the Memory Mapped
 * Protocol Translator, request 2 input (effecting chip select 2 output). This
 * corresponds to MAddrSpace = 011. Note that by default (reset), the device
 * uses a write command of 2, read command of 3 and number of address bytes of
 * 3. This default covers most serial Flash devices, but can be changed. */
#define CSL_QSPI_SPI_SETUP2_REG                                 (0x5CU)

/* Memory Mapped SPI Setup3 Register The Memory Mapped SPI Setup3 Register
 * contains the read/write command setup for the Memory Mapped Protocol
 * Translator (effecting chip select 0 output). The Memory Mapped SPI Setup
 * Register contains the read/write command setup for the Memory Mapped
 * Protocol Translator, request 3 input (effecting chip select 3 output). This
 * corresponds to MAddrSpace = 100, 101, 110 and 111.. Note that by default
 * (reset), the device uses a write command of 2, read command of 3 and number
 * of address bytes of 3. This default covers most serial Flash devices, but
 * can be changed. */
#define CSL_QSPI_SPI_SETUP3_REG                                 (0x60U)

/* SPI Data1 Register (SPIDR1) Data received in the data register is shifted
 * into the LSB position and the contents of the register are shifted to the
 * left. The register is cleared between reads or writes. This acts as the 2nd
 * 32 bit register of the 128 bit shift register in/out Data received in the
 * data register is shifted into the LSB position and the contents of the
 * register are shifted to the left. The register is cleared between reads or
 * writes. When writing data to the register it should be right justified so
 * pre-shifting is not required. The word length field will determine the
 * location of the most significant bit and the bit position that will be
 * shifted out first during a write. In order to shift out byte data the word
 * length should be set to ‘7’ and the data byte should be written to the
 * lower byte of the data register. By setting the word length to ‘7’ the data
 * register will look like a pseudo 8-bit shift register. The word length
 * setting does not affect the VBUSP read or write operations. */
#define CSL_QSPI_SPI_DATA_REG_1                                 (0x68U)

/* SPI Data2 Register (SPIDR2) Data received in the data register is shifted
 * into the LSB position and the contents of the register are shifted to the
 * left. The register is cleared between reads or writes. This acts as the 2nd
 * 32 bit register of the 128 bit shift register in/out Data received in the
 * data register is shifted into the LSB position and the contents of the
 * register are shifted to the left. The register is cleared between reads or
 * writes. When writing data to the register it should be right justified so
 * pre-shifting is not required. The word length field will determine the
 * location of the most significant bit and the bit position that will be
 * shifted out first during a write. In order to shift out byte data the word
 * length should be set to ‘7’ and the data byte should be written to the
 * lower byte of the data register. By setting the word length to ‘7’ the data
 * register will look like a pseudo 8-bit shift register. The word length
 * setting does not affect the VBUSP read or write operations. */
#define CSL_QSPI_SPI_DATA_REG_2                                 (0x6CU)

/* SPI Data3 Register (SPIDR3) Data received in the data register is shifted
 * into the LSB position and the contents of the register are shifted to the
 * left. The register is cleared between reads or writes. This acts as the 2nd
 * 32 bit register of the 128 bit shift register in/out Data received in the
 * data register is shifted into the LSB position and the contents of the
 * register are shifted to the left. The register is cleared between reads or
 * writes. When writing data to the register it should be right justified so
 * pre-shifting is not required. The word length field will determine the
 * location of the most significant bit and the bit position that will be
 * shifted out first during a write. In order to shift out byte data the word
 * length should be set to ‘7’ and the data byte should be written to the
 * lower byte of the data register. By setting the word length to ‘7’ the data
 * register will look like a pseudo 8-bit shift register. The word length
 * setting does not affect the VBUSP read or write operations. */
#define CSL_QSPI_SPI_DATA_REG_3                                 (0x70U)


/**************************************************************************
* Field Definition Macros
**************************************************************************/

/* PID */

#define CSL_QSPI_PID_MINOR_MASK                                 (0x0000003FU)
#define CSL_QSPI_PID_MINOR_SHIFT                                (0U)
#define CSL_QSPI_PID_MINOR_RESETVAL                             (0x00000000U)
#define CSL_QSPI_PID_MINOR_MAX                                  (0x0000003fU)

#define CSL_QSPI_PID_CUSTOM_MASK                                (0x000000C0U)
#define CSL_QSPI_PID_CUSTOM_SHIFT                               (6U)
#define CSL_QSPI_PID_CUSTOM_RESETVAL                            (0x00000000U)
#define CSL_QSPI_PID_CUSTOM_MAX                                 (0x00000003U)

#define CSL_QSPI_PID_MAJOR_MASK                                 (0x00000700U)
#define CSL_QSPI_PID_MAJOR_SHIFT                                (8U)
#define CSL_QSPI_PID_MAJOR_RESETVAL                             (0x00000000U)
#define CSL_QSPI_PID_MAJOR_MAX                                  (0x00000007U)

#define CSL_QSPI_PID_RTL_VERSION_MASK                           (0x0000F800U)
#define CSL_QSPI_PID_RTL_VERSION_SHIFT                          (11U)
#define CSL_QSPI_PID_RTL_VERSION_RESETVAL                       (0x00000000U)
#define CSL_QSPI_PID_RTL_VERSION_MAX                            (0x0000001fU)

#define CSL_QSPI_PID_FUNC_MASK                                  (0x0FFF0000U)
#define CSL_QSPI_PID_FUNC_SHIFT                                 (16U)
#define CSL_QSPI_PID_FUNC_RESETVAL                              (0x00000f40U)
#define CSL_QSPI_PID_FUNC_MAX                                   (0x00000fffU)

#define CSL_QSPI_PID_RSVD_MASK                                  (0x30000000U)
#define CSL_QSPI_PID_RSVD_SHIFT                                 (28U)
#define CSL_QSPI_PID_RSVD_RESETVAL                              (0x00000000U)
#define CSL_QSPI_PID_RSVD_MAX                                   (0x00000003U)

#define CSL_QSPI_PID_SCHEME_MASK                                (0xC0000000U)
#define CSL_QSPI_PID_SCHEME_SHIFT                               (30U)
#define CSL_QSPI_PID_SCHEME_RESETVAL                            (0x00000001U)
#define CSL_QSPI_PID_SCHEME_MAX                                 (0x00000003U)

#define CSL_QSPI_PID_RESETVAL                                   (0x4f400000U)

/* SYSCONFIG */

#define CSL_QSPI_SYSCONFIG_RSVD_1_MASK                          (0x00000003U)
#define CSL_QSPI_SYSCONFIG_RSVD_1_SHIFT                         (0U)
#define CSL_QSPI_SYSCONFIG_RSVD_1_RESETVAL                      (0x00000000U)
#define CSL_QSPI_SYSCONFIG_RSVD_1_MAX                           (0x00000003U)

#define CSL_QSPI_SYSCONFIG_IDLE_MODE_MASK                       (0x0000000CU)
#define CSL_QSPI_SYSCONFIG_IDLE_MODE_SHIFT                      (2U)
#define CSL_QSPI_SYSCONFIG_IDLE_MODE_RESETVAL                   (0x00000002U)
#define CSL_QSPI_SYSCONFIG_IDLE_MODE_FORCE_IDLE                 (0x00000000U)
#define CSL_QSPI_SYSCONFIG_IDLE_MODE_NO_IDLE                    (0x00000001U)
#define CSL_QSPI_SYSCONFIG_IDLE_MODE_SMART_IDLE                 (0x00000002U)
#define CSL_QSPI_SYSCONFIG_IDLE_MODE_SMART_IDLE_WAKEUP_CAPABLE  (0x00000003U)

#define CSL_QSPI_SYSCONFIG_RSVD_3_MASK                          (0x00000030U)
#define CSL_QSPI_SYSCONFIG_RSVD_3_SHIFT                         (4U)
#define CSL_QSPI_SYSCONFIG_RSVD_3_RESETVAL                      (0x00000002U)
#define CSL_QSPI_SYSCONFIG_RSVD_3_MAX                           (0x00000003U)

#define CSL_QSPI_SYSCONFIG_RSVD_2_MASK                          (0xFFFFFFC0U)
#define CSL_QSPI_SYSCONFIG_RSVD_2_SHIFT                         (6U)
#define CSL_QSPI_SYSCONFIG_RSVD_2_RESETVAL                      (0x00000000U)
#define CSL_QSPI_SYSCONFIG_RSVD_2_MAX                           (0x03ffffffU)

#define CSL_QSPI_SYSCONFIG_RESETVAL                             (0x00000028U)

/* INTR_STATUS_RAW_SET */

#define CSL_QSPI_INTR_STATUS_RAW_SET_FIRQ_RAW_MASK              (0x00000001U)
#define CSL_QSPI_INTR_STATUS_RAW_SET_FIRQ_RAW_SHIFT             (0U)
#define CSL_QSPI_INTR_STATUS_RAW_SET_FIRQ_RAW_RESETVAL          (0x00000000U)
#define CSL_QSPI_INTR_STATUS_RAW_SET_FIRQ_RAW_READ_INACTIVE     (0x00000000U)
#define CSL_QSPI_INTR_STATUS_RAW_SET_FIRQ_RAW_READ_ACTIVE       (0x00000001U)

#define CSL_QSPI_INTR_STATUS_RAW_SET_WIRQ_RAW_MASK              (0x00000002U)
#define CSL_QSPI_INTR_STATUS_RAW_SET_WIRQ_RAW_SHIFT             (1U)
#define CSL_QSPI_INTR_STATUS_RAW_SET_WIRQ_RAW_RESETVAL          (0x00000000U)
#define CSL_QSPI_INTR_STATUS_RAW_SET_WIRQ_RAW_READ_INACTIVE     (0x00000000U)
#define CSL_QSPI_INTR_STATUS_RAW_SET_WIRQ_RAW_READ_ACTIVE       (0x00000001U)

#define CSL_QSPI_INTR_STATUS_RAW_SET_RSVD_MASK                  (0xFFFFFFFCU)
#define CSL_QSPI_INTR_STATUS_RAW_SET_RSVD_SHIFT                 (2U)
#define CSL_QSPI_INTR_STATUS_RAW_SET_RSVD_RESETVAL              (0x00000000U)
#define CSL_QSPI_INTR_STATUS_RAW_SET_RSVD_MAX                   (0x3fffffffU)

#define CSL_QSPI_INTR_STATUS_RAW_SET_RESETVAL                   (0x00000000U)

/* INTR_STATUS_ENABLED_CLEAR */

#define CSL_QSPI_INTR_STATUS_ENABLED_CLEAR_FIRQ_ENA_MASK        (0x00000001U)
#define CSL_QSPI_INTR_STATUS_ENABLED_CLEAR_FIRQ_ENA_SHIFT       (0U)
#define CSL_QSPI_INTR_STATUS_ENABLED_CLEAR_FIRQ_ENA_RESETVAL    (0x00000000U)
#define CSL_QSPI_INTR_STATUS_ENABLED_CLEAR_FIRQ_ENA_INACTIVE    (0x00000000U)
#define CSL_QSPI_INTR_STATUS_ENABLED_CLEAR_FIRQ_ENA_ACTIVE      (0x00000001U)

#define CSL_QSPI_INTR_STATUS_ENABLED_CLEAR_WIRQ_ENA_MASK        (0x00000002U)
#define CSL_QSPI_INTR_STATUS_ENABLED_CLEAR_WIRQ_ENA_SHIFT       (1U)
#define CSL_QSPI_INTR_STATUS_ENABLED_CLEAR_WIRQ_ENA_RESETVAL    (0x00000000U)
#define CSL_QSPI_INTR_STATUS_ENABLED_CLEAR_WIRQ_ENA_INACTIVE    (0x00000000U)
#define CSL_QSPI_INTR_STATUS_ENABLED_CLEAR_WIRQ_ENA_ACTIVE      (0x00000001U)

#define CSL_QSPI_INTR_STATUS_ENABLED_CLEAR_RSVD_MASK            (0xFFFFFFFCU)
#define CSL_QSPI_INTR_STATUS_ENABLED_CLEAR_RSVD_SHIFT           (2U)
#define CSL_QSPI_INTR_STATUS_ENABLED_CLEAR_RSVD_RESETVAL        (0x00000000U)
#define CSL_QSPI_INTR_STATUS_ENABLED_CLEAR_RSVD_MAX             (0x3fffffffU)

#define CSL_QSPI_INTR_STATUS_ENABLED_CLEAR_RESETVAL             (0x00000000U)

/* INTR_ENABLE_SET_REG */

#define CSL_QSPI_INTR_ENABLE_SET_REG_FIRQ_ENA_SET_MASK          (0x00000001U)
#define CSL_QSPI_INTR_ENABLE_SET_REG_FIRQ_ENA_SET_SHIFT         (0U)
#define CSL_QSPI_INTR_ENABLE_SET_REG_FIRQ_ENA_SET_RESETVAL      (0x00000000U)
#define CSL_QSPI_INTR_ENABLE_SET_REG_FIRQ_ENA_SET_INACTIVE      (0x00000000U)
#define CSL_QSPI_INTR_ENABLE_SET_REG_FIRQ_ENA_SET_ACTIVE        (0x00000001U)

#define CSL_QSPI_INTR_ENABLE_SET_REG_WIRQ_ENA_SET_MASK          (0x00000002U)
#define CSL_QSPI_INTR_ENABLE_SET_REG_WIRQ_ENA_SET_SHIFT         (1U)
#define CSL_QSPI_INTR_ENABLE_SET_REG_WIRQ_ENA_SET_RESETVAL      (0x00000000U)
#define CSL_QSPI_INTR_ENABLE_SET_REG_WIRQ_ENA_SET_INACTIVE      (0x00000000U)
#define CSL_QSPI_INTR_ENABLE_SET_REG_WIRQ_ENA_SET_ACTIVE        (0x00000001U)

#define CSL_QSPI_INTR_ENABLE_SET_REG_RSVD_MASK                  (0xFFFFFFFCU)
#define CSL_QSPI_INTR_ENABLE_SET_REG_RSVD_SHIFT                 (2U)
#define CSL_QSPI_INTR_ENABLE_SET_REG_RSVD_RESETVAL              (0x00000000U)
#define CSL_QSPI_INTR_ENABLE_SET_REG_RSVD_MAX                   (0x3fffffffU)

#define CSL_QSPI_INTR_ENABLE_SET_REG_RESETVAL                   (0x00000000U)

/* INTR_ENABLE_CLEAR_REG */

#define CSL_QSPI_INTR_ENABLE_CLEAR_REG_FIRQ_ENA_CLR_MASK        (0x00000001U)
#define CSL_QSPI_INTR_ENABLE_CLEAR_REG_FIRQ_ENA_CLR_SHIFT       (0U)
#define CSL_QSPI_INTR_ENABLE_CLEAR_REG_FIRQ_ENA_CLR_RESETVAL    (0x00000000U)
#define CSL_QSPI_INTR_ENABLE_CLEAR_REG_FIRQ_ENA_CLR_INACTIVE    (0x00000000U)
#define CSL_QSPI_INTR_ENABLE_CLEAR_REG_FIRQ_ENA_CLR_ACTIVE      (0x00000001U)

#define CSL_QSPI_INTR_ENABLE_CLEAR_REG_WIRQ_ENA_CLR_MASK        (0x00000002U)
#define CSL_QSPI_INTR_ENABLE_CLEAR_REG_WIRQ_ENA_CLR_SHIFT       (1U)
#define CSL_QSPI_INTR_ENABLE_CLEAR_REG_WIRQ_ENA_CLR_RESETVAL    (0x00000000U)
#define CSL_QSPI_INTR_ENABLE_CLEAR_REG_WIRQ_ENA_CLR_INACTIVE    (0x00000000U)
#define CSL_QSPI_INTR_ENABLE_CLEAR_REG_WIRQ_ENA_CLR_ACTIVE      (0x00000001U)

#define CSL_QSPI_INTR_ENABLE_CLEAR_REG_RSVD_MASK                (0xFFFFFFFCU)
#define CSL_QSPI_INTR_ENABLE_CLEAR_REG_RSVD_SHIFT               (2U)
#define CSL_QSPI_INTR_ENABLE_CLEAR_REG_RSVD_RESETVAL            (0x00000000U)
#define CSL_QSPI_INTR_ENABLE_CLEAR_REG_RSVD_MAX                 (0x3fffffffU)

#define CSL_QSPI_INTR_ENABLE_CLEAR_REG_RESETVAL                 (0x00000000U)

/* INTC_EOI_REG */

#define CSL_QSPI_INTC_EOI_REG_EOI_VECTOR_MASK                   (0xFFFFFFFFU)
#define CSL_QSPI_INTC_EOI_REG_EOI_VECTOR_SHIFT                  (0U)
#define CSL_QSPI_INTC_EOI_REG_EOI_VECTOR_RESETVAL               (0x00000000U)
#define CSL_QSPI_INTC_EOI_REG_EOI_VECTOR_MAX                    (0xffffffffU)

#define CSL_QSPI_INTC_EOI_REG_RESETVAL                          (0x00000000U)

/* SPI_CLOCK_CNTRL_REG */

#define CSL_QSPI_SPI_CLOCK_CNTRL_REG_DCLK_DIV_MASK              (0x0000FFFFU)
#define CSL_QSPI_SPI_CLOCK_CNTRL_REG_DCLK_DIV_SHIFT             (0U)
#define CSL_QSPI_SPI_CLOCK_CNTRL_REG_DCLK_DIV_RESETVAL          (0x00000000U)
#define CSL_QSPI_SPI_CLOCK_CNTRL_REG_DCLK_DIV_MAX               (0x0000ffffU)

#define CSL_QSPI_SPI_CLOCK_CNTRL_REG_RSVD_MASK                  (0x7FFF0000U)
#define CSL_QSPI_SPI_CLOCK_CNTRL_REG_RSVD_SHIFT                 (16U)
#define CSL_QSPI_SPI_CLOCK_CNTRL_REG_RSVD_RESETVAL              (0x00000000U)
#define CSL_QSPI_SPI_CLOCK_CNTRL_REG_RSVD_MAX                   (0x00007fffU)

#define CSL_QSPI_SPI_CLOCK_CNTRL_REG_CLKEN_MASK                 (0x80000000U)
#define CSL_QSPI_SPI_CLOCK_CNTRL_REG_CLKEN_SHIFT                (31U)
#define CSL_QSPI_SPI_CLOCK_CNTRL_REG_CLKEN_RESETVAL             (0x00000000U)
#define CSL_QSPI_SPI_CLOCK_CNTRL_REG_CLKEN_DCLOCK_ON            (0x00000001U)
#define CSL_QSPI_SPI_CLOCK_CNTRL_REG_CLKEN_DCLOCK_OFF           (0x00000000U)

#define CSL_QSPI_SPI_CLOCK_CNTRL_REG_RESETVAL                   (0x00000000U)

/* SPI_DC_REG */

#define CSL_QSPI_SPI_DC_REG_CKP0_MASK                           (0x00000001U)
#define CSL_QSPI_SPI_DC_REG_CKP0_SHIFT                          (0U)
#define CSL_QSPI_SPI_DC_REG_CKP0_RESETVAL                       (0x00000000U)
#define CSL_QSPI_SPI_DC_REG_CKP0_DATA_ACTIVE                    (0x00000001U)
#define CSL_QSPI_SPI_DC_REG_CKP0_DATA_INACTIVE                  (0x00000000U)

#define CSL_QSPI_SPI_DC_REG_CSP0_MASK                           (0x00000002U)
#define CSL_QSPI_SPI_DC_REG_CSP0_SHIFT                          (1U)
#define CSL_QSPI_SPI_DC_REG_CSP0_RESETVAL                       (0x00000000U)
#define CSL_QSPI_SPI_DC_REG_CSP0_ACTIVE_LOW                     (0x00000000U)
#define CSL_QSPI_SPI_DC_REG_CSP0_ACTIVE_HIGH                    (0x00000001U)

#define CSL_QSPI_SPI_DC_REG_CKPH0_MASK                          (0x00000004U)
#define CSL_QSPI_SPI_DC_REG_CKPH0_SHIFT                         (2U)
#define CSL_QSPI_SPI_DC_REG_CKPH0_RESETVAL                      (0x00000000U)
#define CSL_QSPI_SPI_DC_REG_CKPH0_CKP_0_SHIFT_OUT_FALLING_EDGE  (0x00000000U)
#define CSL_QSPI_SPI_DC_REG_CKPH0_CKP_0_SHIFT_OUT_RISING_EDGE   (0x00000001U)
#define CSL_QSPI_SPI_DC_REG_CKPH0_CKP_1_SHIFT_OUT_FALLING_EDGE  (0x00000001U)
#define CSL_QSPI_SPI_DC_REG_CKPH0_CKP_1_SHIFT_OUT_RISING_EDGE   (0x00000000U)

#define CSL_QSPI_SPI_DC_REG_DD0_MASK                            (0x00000018U)
#define CSL_QSPI_SPI_DC_REG_DD0_SHIFT                           (3U)
#define CSL_QSPI_SPI_DC_REG_DD0_RESETVAL                        (0x00000000U)
#define CSL_QSPI_SPI_DC_REG_DD0_CS_TO_DATA_DELAY_0              (0x00000000U)
#define CSL_QSPI_SPI_DC_REG_DD0_CS_TO_DATA_DELAY_1              (0x00000001U)
#define CSL_QSPI_SPI_DC_REG_DD0_CS_TO_DATA_DELAY_2              (0x00000002U)
#define CSL_QSPI_SPI_DC_REG_DD0_CS_TO_DATA_DELAY_3              (0x00000003U)

#define CSL_QSPI_SPI_DC_REG_RSVD_0_MASK                         (0x000000E0U)
#define CSL_QSPI_SPI_DC_REG_RSVD_0_SHIFT                        (5U)
#define CSL_QSPI_SPI_DC_REG_RSVD_0_RESETVAL                     (0x00000000U)
#define CSL_QSPI_SPI_DC_REG_RSVD_0_MAX                          (0x00000007U)

#define CSL_QSPI_SPI_DC_REG_CKP1_MASK                           (0x00000100U)
#define CSL_QSPI_SPI_DC_REG_CKP1_SHIFT                          (8U)
#define CSL_QSPI_SPI_DC_REG_CKP1_RESETVAL                       (0x00000000U)
#define CSL_QSPI_SPI_DC_REG_CKP1_DATA_ACTIVE                    (0x00000001U)
#define CSL_QSPI_SPI_DC_REG_CKP1_DATA_INACTIVE                  (0x00000000U)

#define CSL_QSPI_SPI_DC_REG_CSP1_MASK                           (0x00000200U)
#define CSL_QSPI_SPI_DC_REG_CSP1_SHIFT                          (9U)
#define CSL_QSPI_SPI_DC_REG_CSP1_RESETVAL                       (0x00000000U)
#define CSL_QSPI_SPI_DC_REG_CSP1_ACTIVE_LOW                     (0x00000000U)
#define CSL_QSPI_SPI_DC_REG_CSP1_ACTIVE_HIGH                    (0x00000001U)

#define CSL_QSPI_SPI_DC_REG_CKPH1_MASK                          (0x00000400U)
#define CSL_QSPI_SPI_DC_REG_CKPH1_SHIFT                         (10U)
#define CSL_QSPI_SPI_DC_REG_CKPH1_RESETVAL                      (0x00000000U)
#define CSL_QSPI_SPI_DC_REG_CKPH1_CKP_0_SHIFT_OUT_FALLING_EDGE  (0x00000000U)
#define CSL_QSPI_SPI_DC_REG_CKPH1_CKP_0_SHIFT_OUT_RISING_EDGE   (0x00000001U)
#define CSL_QSPI_SPI_DC_REG_CKPH1_CKP_1_SHIFT_OUT_FALLING_EDGE  (0x00000001U)
#define CSL_QSPI_SPI_DC_REG_CKPH1_CKP_1_SHIFT_OUT_RISING_EDGE   (0x00000000U)

#define CSL_QSPI_SPI_DC_REG_DD1_MASK                            (0x00001800U)
#define CSL_QSPI_SPI_DC_REG_DD1_SHIFT                           (11U)
#define CSL_QSPI_SPI_DC_REG_DD1_RESETVAL                        (0x00000000U)
#define CSL_QSPI_SPI_DC_REG_DD1_CS_TO_DATA_DELAY_0              (0x00000000U)
#define CSL_QSPI_SPI_DC_REG_DD1_CS_TO_DATA_DELAY_1              (0x00000001U)
#define CSL_QSPI_SPI_DC_REG_DD1_CS_TO_DATA_DELAY_2              (0x00000002U)
#define CSL_QSPI_SPI_DC_REG_DD1_CS_TO_DATA_DELAY_3              (0x00000003U)

#define CSL_QSPI_SPI_DC_REG_RSVD_1_MASK                         (0x0000E000U)
#define CSL_QSPI_SPI_DC_REG_RSVD_1_SHIFT                        (13U)
#define CSL_QSPI_SPI_DC_REG_RSVD_1_RESETVAL                     (0x00000000U)
#define CSL_QSPI_SPI_DC_REG_RSVD_1_MAX                          (0x00000007U)

#define CSL_QSPI_SPI_DC_REG_CKP2_MASK                           (0x00010000U)
#define CSL_QSPI_SPI_DC_REG_CKP2_SHIFT                          (16U)
#define CSL_QSPI_SPI_DC_REG_CKP2_RESETVAL                       (0x00000000U)
#define CSL_QSPI_SPI_DC_REG_CKP2_DATA_ACTIVE                    (0x00000001U)
#define CSL_QSPI_SPI_DC_REG_CKP2_DATA_INACTIVE                  (0x00000000U)

#define CSL_QSPI_SPI_DC_REG_CSP2_MASK                           (0x00020000U)
#define CSL_QSPI_SPI_DC_REG_CSP2_SHIFT                          (17U)
#define CSL_QSPI_SPI_DC_REG_CSP2_RESETVAL                       (0x00000000U)
#define CSL_QSPI_SPI_DC_REG_CSP2_ACTIVE_LOW                     (0x00000000U)
#define CSL_QSPI_SPI_DC_REG_CSP2_ACTIVE_HIGH                    (0x00000001U)

#define CSL_QSPI_SPI_DC_REG_CKPH2_MASK                          (0x00040000U)
#define CSL_QSPI_SPI_DC_REG_CKPH2_SHIFT                         (18U)
#define CSL_QSPI_SPI_DC_REG_CKPH2_RESETVAL                      (0x00000000U)
#define CSL_QSPI_SPI_DC_REG_CKPH2_CKP_0_SHIFT_OUT_FALLING_EDGE  (0x00000000U)
#define CSL_QSPI_SPI_DC_REG_CKPH2_CKP_0_SHIFT_OUT_RISING_EDGE   (0x00000001U)
#define CSL_QSPI_SPI_DC_REG_CKPH2_CKP_1_SHIFT_OUT_FALLING_EDGE  (0x00000001U)
#define CSL_QSPI_SPI_DC_REG_CKPH2_CKP_1_SHIFT_OUT_RISING_EDGE   (0x00000000U)

#define CSL_QSPI_SPI_DC_REG_DD2_MASK                            (0x00180000U)
#define CSL_QSPI_SPI_DC_REG_DD2_SHIFT                           (19U)
#define CSL_QSPI_SPI_DC_REG_DD2_RESETVAL                        (0x00000000U)
#define CSL_QSPI_SPI_DC_REG_DD2_CS_TO_DATA_DELAY_0              (0x00000000U)
#define CSL_QSPI_SPI_DC_REG_DD2_CS_TO_DATA_DELAY_1              (0x00000001U)
#define CSL_QSPI_SPI_DC_REG_DD2_CS_TO_DATA_DELAY_2              (0x00000002U)
#define CSL_QSPI_SPI_DC_REG_DD2_CS_TO_DATA_DELAY_3              (0x00000003U)

#define CSL_QSPI_SPI_DC_REG_RSVD_2_MASK                         (0x00E00000U)
#define CSL_QSPI_SPI_DC_REG_RSVD_2_SHIFT                        (21U)
#define CSL_QSPI_SPI_DC_REG_RSVD_2_RESETVAL                     (0x00000000U)
#define CSL_QSPI_SPI_DC_REG_RSVD_2_MAX                          (0x00000007U)

#define CSL_QSPI_SPI_DC_REG_CKP3_MASK                           (0x01000000U)
#define CSL_QSPI_SPI_DC_REG_CKP3_SHIFT                          (24U)
#define CSL_QSPI_SPI_DC_REG_CKP3_RESETVAL                       (0x00000000U)
#define CSL_QSPI_SPI_DC_REG_CKP3_DATA_ACTIVE                    (0x00000001U)
#define CSL_QSPI_SPI_DC_REG_CKP3_DATA_INACTIVE                  (0x00000000U)

#define CSL_QSPI_SPI_DC_REG_CSP3_MASK                           (0x02000000U)
#define CSL_QSPI_SPI_DC_REG_CSP3_SHIFT                          (25U)
#define CSL_QSPI_SPI_DC_REG_CSP3_RESETVAL                       (0x00000000U)
#define CSL_QSPI_SPI_DC_REG_CSP3_ACTIVE_LOW                     (0x00000000U)
#define CSL_QSPI_SPI_DC_REG_CSP3_ACTIVE_HIGH                    (0x00000001U)

#define CSL_QSPI_SPI_DC_REG_CKPH3_MASK                          (0x04000000U)
#define CSL_QSPI_SPI_DC_REG_CKPH3_SHIFT                         (26U)
#define CSL_QSPI_SPI_DC_REG_CKPH3_RESETVAL                      (0x00000000U)
#define CSL_QSPI_SPI_DC_REG_CKPH3_CKP_0_SHIFT_OUT_FALLING_EDGE  (0x00000000U)
#define CSL_QSPI_SPI_DC_REG_CKPH3_CKP_0_SHIFT_OUT_RISING_EDGE   (0x00000001U)
#define CSL_QSPI_SPI_DC_REG_CKPH3_CKP_1_SHIFT_OUT_FALLING_EDGE  (0x00000001U)
#define CSL_QSPI_SPI_DC_REG_CKPH3_CKP_1_SHIFT_OUT_RISING_EDGE   (0x00000000U)

#define CSL_QSPI_SPI_DC_REG_DD3_MASK                            (0x18000000U)
#define CSL_QSPI_SPI_DC_REG_DD3_SHIFT                           (27U)
#define CSL_QSPI_SPI_DC_REG_DD3_RESETVAL                        (0x00000000U)
#define CSL_QSPI_SPI_DC_REG_DD3_CS_TO_DATA_DELAY_0              (0x00000000U)
#define CSL_QSPI_SPI_DC_REG_DD3_CS_TO_DATA_DELAY_1              (0x00000001U)
#define CSL_QSPI_SPI_DC_REG_DD3_CS_TO_DATA_DELAY_2              (0x00000002U)
#define CSL_QSPI_SPI_DC_REG_DD3_CS_TO_DATA_DELAY_3              (0x00000003U)

#define CSL_QSPI_SPI_DC_REG_RSVD_3_MASK                         (0xE0000000U)
#define CSL_QSPI_SPI_DC_REG_RSVD_3_SHIFT                        (29U)
#define CSL_QSPI_SPI_DC_REG_RSVD_3_RESETVAL                     (0x00000000U)
#define CSL_QSPI_SPI_DC_REG_RSVD_3_MAX                          (0x00000007U)

#define CSL_QSPI_SPI_DC_REG_RESETVAL                            (0x00000000U)

/* SPI_CMD_REG */

#define CSL_QSPI_SPI_CMD_REG_FLEN_MASK                          (0x00000FFFU)
#define CSL_QSPI_SPI_CMD_REG_FLEN_SHIFT                         (0U)
#define CSL_QSPI_SPI_CMD_REG_FLEN_RESETVAL                      (0x00000000U)
#define CSL_QSPI_SPI_CMD_REG_FLEN_MAX                           (0x00000fffU)

#define CSL_QSPI_SPI_CMD_REG_RSVD_MASK                          (0x00003000U)
#define CSL_QSPI_SPI_CMD_REG_RSVD_SHIFT                         (12U)
#define CSL_QSPI_SPI_CMD_REG_RSVD_RESETVAL                      (0x00000000U)
#define CSL_QSPI_SPI_CMD_REG_RSVD_MAX                           (0x00000003U)

#define CSL_QSPI_SPI_CMD_REG_WIRQ_MASK                          (0x00004000U)
#define CSL_QSPI_SPI_CMD_REG_WIRQ_SHIFT                         (14U)
#define CSL_QSPI_SPI_CMD_REG_WIRQ_RESETVAL                      (0x00000000U)
#define CSL_QSPI_SPI_CMD_REG_WIRQ_WORD_COUNT_IRQ_ENABLE         (0x00000001U)
#define CSL_QSPI_SPI_CMD_REG_WIRQ_WORD_COUNT_IRQ_DISABLE        (0x00000000U)

#define CSL_QSPI_SPI_CMD_REG_FIRQ_MASK                          (0x00008000U)
#define CSL_QSPI_SPI_CMD_REG_FIRQ_SHIFT                         (15U)
#define CSL_QSPI_SPI_CMD_REG_FIRQ_RESETVAL                      (0x00000000U)
#define CSL_QSPI_SPI_CMD_REG_FIRQ_FRAME_COUNT_IRQ_ENABLE        (0x00000001U)
#define CSL_QSPI_SPI_CMD_REG_FIRQ_FRAME_COUNT_IRQ_DISABLE       (0x00000000U)

#define CSL_QSPI_SPI_CMD_REG_CMD_MASK                           (0x00070000U)
#define CSL_QSPI_SPI_CMD_REG_CMD_SHIFT                          (16U)
#define CSL_QSPI_SPI_CMD_REG_CMD_RESETVAL                       (0x00000000U)
#define CSL_QSPI_SPI_CMD_REG_CMD_FOUR_PIN_READ_SINGLE           (0x00000001U)
#define CSL_QSPI_SPI_CMD_REG_CMD_FOUR_PIN_WRITE_SINGLE          (0x00000002U)
#define CSL_QSPI_SPI_CMD_REG_CMD_FOUR_PIN_READ_DUAL             (0x00000003U)
#define CSL_QSPI_SPI_CMD_REG_CMD_THREE_PIN_READ_SINGLE          (0x00000005U)
#define CSL_QSPI_SPI_CMD_REG_CMD_THREE_PIN_WRITE_SINGLE         (0x00000006U)
#define CSL_QSPI_SPI_CMD_REG_CMD_SIX_PIN_READ_QUAD              (0x00000007U)

#ifdef CSL_MODIFICATION
#define CSL_QSPI_SPI_CMD_REG_WLEN_MASK                          (0x00F80000U)
#else
#define CSL_QSPI_SPI_CMD_REG_WLEN_MASK                          (0x03F80000U)
#endif
#define CSL_QSPI_SPI_CMD_REG_WLEN_SHIFT                         (19U)
#define CSL_QSPI_SPI_CMD_REG_WLEN_RESETVAL                      (0x00000000U)
#define CSL_QSPI_SPI_CMD_REG_WLEN_MAX                           (0x0000001fU)

#define CSL_QSPI_SPI_CMD_REG_RSVD_2_MASK                        (0x0F000000U)
#define CSL_QSPI_SPI_CMD_REG_RSVD_2_SHIFT                       (24U)
#define CSL_QSPI_SPI_CMD_REG_RSVD_2_RESETVAL                    (0x00000000U)
#define CSL_QSPI_SPI_CMD_REG_RSVD_2_MAX                         (0x0000000fU)

#define CSL_QSPI_SPI_CMD_REG_CSNUM_MASK                         (0x30000000U)
#define CSL_QSPI_SPI_CMD_REG_CSNUM_SHIFT                        (28U)
#define CSL_QSPI_SPI_CMD_REG_CSNUM_RESETVAL                     (0x00000000U)
#define CSL_QSPI_SPI_CMD_REG_CSNUM_CS_0                         (0x00000000U)
#define CSL_QSPI_SPI_CMD_REG_CSNUM_CS_1                         (0x00000001U)
#define CSL_QSPI_SPI_CMD_REG_CSNUM_CS_2                         (0x00000002U)
#define CSL_QSPI_SPI_CMD_REG_CSNUM_CS_3                         (0x00000003U)

#define CSL_QSPI_SPI_CMD_REG_RSVD_3_MASK                        (0xC0000000U)
#define CSL_QSPI_SPI_CMD_REG_RSVD_3_SHIFT                       (30U)
#define CSL_QSPI_SPI_CMD_REG_RSVD_3_RESETVAL                    (0x00000000U)
#define CSL_QSPI_SPI_CMD_REG_RSVD_3_MAX                         (0x00000003U)

#define CSL_QSPI_SPI_CMD_REG_RESETVAL                           (0x00000000U)

/* SPI_STATUS_REG */

#define CSL_QSPI_SPI_STATUS_REG_BUSY_MASK                       (0x00000001U)
#define CSL_QSPI_SPI_STATUS_REG_BUSY_SHIFT                      (0U)
#define CSL_QSPI_SPI_STATUS_REG_BUSY_RESETVAL                   (0x00000000U)
#define CSL_QSPI_SPI_STATUS_REG_BUSY_IDLE                       (0x00000000U)
#define CSL_QSPI_SPI_STATUS_REG_BUSY_BUSY                       (0x00000001U)

#define CSL_QSPI_SPI_STATUS_REG_WC_MASK                         (0x00000002U)
#define CSL_QSPI_SPI_STATUS_REG_WC_SHIFT                        (1U)
#define CSL_QSPI_SPI_STATUS_REG_WC_RESETVAL                     (0x00000000U)
#define CSL_QSPI_SPI_STATUS_REG_WC_WORD_TRANSFER_NOT_COMPLETE   (0x00000000U)
#define CSL_QSPI_SPI_STATUS_REG_WC_WORD_TRANSFER_COMPLETE       (0x00000001U)

#define CSL_QSPI_SPI_STATUS_REG_FC_MASK                         (0x00000004U)
#define CSL_QSPI_SPI_STATUS_REG_FC_SHIFT                        (2U)
#define CSL_QSPI_SPI_STATUS_REG_FC_RESETVAL                     (0x00000000U)
#define CSL_QSPI_SPI_STATUS_REG_FC_FRAME_TRANSFER_NOT_COMPLETE  (0x00000000U)
#define CSL_QSPI_SPI_STATUS_REG_FC_FRAME_TRANSFER_COMPLETE      (0x00000001U)

#define CSL_QSPI_SPI_STATUS_REG_RSVD_MASK                       (0x0000FFF8U)
#define CSL_QSPI_SPI_STATUS_REG_RSVD_SHIFT                      (3U)
#define CSL_QSPI_SPI_STATUS_REG_RSVD_RESETVAL                   (0x00000000U)
#define CSL_QSPI_SPI_STATUS_REG_RSVD_MAX                        (0x00001fffU)

#define CSL_QSPI_SPI_STATUS_REG_WDCNT_MASK                      (0x0FFF0000U)
#define CSL_QSPI_SPI_STATUS_REG_WDCNT_SHIFT                     (16U)
#define CSL_QSPI_SPI_STATUS_REG_WDCNT_RESETVAL                  (0x00000000U)
#define CSL_QSPI_SPI_STATUS_REG_WDCNT_MAX                       (0x00000fffU)

#define CSL_QSPI_SPI_STATUS_REG_RSVD_2_MASK                     (0xF0000000U)
#define CSL_QSPI_SPI_STATUS_REG_RSVD_2_SHIFT                    (28U)
#define CSL_QSPI_SPI_STATUS_REG_RSVD_2_RESETVAL                 (0x00000000U)
#define CSL_QSPI_SPI_STATUS_REG_RSVD_2_MAX                      (0x0000000fU)

#define CSL_QSPI_SPI_STATUS_REG_RESETVAL                        (0x00000000U)

/* SPI_DATA_REG */

#define CSL_QSPI_SPI_DATA_REG_DATA_MASK                         (0xFFFFFFFFU)
#define CSL_QSPI_SPI_DATA_REG_DATA_SHIFT                        (0U)
#define CSL_QSPI_SPI_DATA_REG_DATA_RESETVAL                     (0x00000000U)
#define CSL_QSPI_SPI_DATA_REG_DATA_MAX                          (0xffffffffU)

#define CSL_QSPI_SPI_DATA_REG_RESETVAL                          (0x00000000U)

/* SPI_SETUP0_REG */

#define CSL_QSPI_SPI_SETUP0_REG_RCMD_MASK                       (0x000000FFU)
#define CSL_QSPI_SPI_SETUP0_REG_RCMD_SHIFT                      (0U)
#define CSL_QSPI_SPI_SETUP0_REG_RCMD_RESETVAL                   (0x00000003U)
#define CSL_QSPI_SPI_SETUP0_REG_RCMD_MAX                        (0x000000ffU)

#define CSL_QSPI_SPI_SETUP0_REG_NUM_A_BYTES_MASK                (0x00000300U)
#define CSL_QSPI_SPI_SETUP0_REG_NUM_A_BYTES_SHIFT               (8U)
#define CSL_QSPI_SPI_SETUP0_REG_NUM_A_BYTES_RESETVAL            (0x00000002U)
#define CSL_QSPI_SPI_SETUP0_REG_NUM_A_BYTES_ONE_BYTE            (0x00000000U)
#define CSL_QSPI_SPI_SETUP0_REG_NUM_A_BYTES_TWO_BYTES           (0x00000001U)
#define CSL_QSPI_SPI_SETUP0_REG_NUM_A_BYTES_THREE_BYTES         (0x00000002U)
#define CSL_QSPI_SPI_SETUP0_REG_NUM_A_BYTES_FOUR_BYTES          (0x00000003U)

#define CSL_QSPI_SPI_SETUP0_REG_NUM_D_BYTES_MASK                (0x00000C00U)
#define CSL_QSPI_SPI_SETUP0_REG_NUM_D_BYTES_SHIFT               (10U)
#define CSL_QSPI_SPI_SETUP0_REG_NUM_D_BYTES_RESETVAL            (0x00000000U)
#define CSL_QSPI_SPI_SETUP0_REG_NUM_D_BYTES_USE_NUM_D_BITS      (0x00000000U)
#define CSL_QSPI_SPI_SETUP0_REG_NUM_D_BYTES_USE_8_BITS          (0x00000001U)
#define CSL_QSPI_SPI_SETUP0_REG_NUM_D_BYTES_USE_16_BITS         (0x00000002U)
#define CSL_QSPI_SPI_SETUP0_REG_NUM_D_BYTES_USE_24_BITS         (0x00000003U)

#define CSL_QSPI_SPI_SETUP0_REG_READ_TYPE_MASK                  (0x00003000U)
#define CSL_QSPI_SPI_SETUP0_REG_READ_TYPE_SHIFT                 (12U)
#define CSL_QSPI_SPI_SETUP0_REG_READ_TYPE_RESETVAL              (0x00000000U)
#define CSL_QSPI_SPI_SETUP0_REG_READ_TYPE_NORMAL_READ           (0x00000000U)
#define CSL_QSPI_SPI_SETUP0_REG_READ_TYPE_DUAL_READ             (0x00000001U)
#define CSL_QSPI_SPI_SETUP0_REG_READ_TYPE_NORMAL_READ_TYPE      (0x00000002U)
#define CSL_QSPI_SPI_SETUP0_REG_READ_TYPE_QUAD_READ             (0x00000003U)

#define CSL_QSPI_SPI_SETUP0_REG_RSVD_1_MASK                     (0x0000C000U)
#define CSL_QSPI_SPI_SETUP0_REG_RSVD_1_SHIFT                    (14U)
#define CSL_QSPI_SPI_SETUP0_REG_RSVD_1_RESETVAL                 (0x00000000U)
#define CSL_QSPI_SPI_SETUP0_REG_RSVD_1_MAX                      (0x00000003U)

#define CSL_QSPI_SPI_SETUP0_REG_WCMD_MASK                       (0x00FF0000U)
#define CSL_QSPI_SPI_SETUP0_REG_WCMD_SHIFT                      (16U)
#define CSL_QSPI_SPI_SETUP0_REG_WCMD_RESETVAL                   (0x00000002U)
#define CSL_QSPI_SPI_SETUP0_REG_WCMD_MAX                        (0x000000ffU)

#define CSL_QSPI_SPI_SETUP0_REG_RSVD_2_MASK                     (0xE0000000U)
#define CSL_QSPI_SPI_SETUP0_REG_RSVD_2_SHIFT                    (29U)
#define CSL_QSPI_SPI_SETUP0_REG_RSVD_2_RESETVAL                 (0x00000000U)
#define CSL_QSPI_SPI_SETUP0_REG_RSVD_2_MAX                      (0x00000007U)

#define CSL_QSPI_SPI_SETUP0_REG_NUM_D_BITS_MASK                 (0x1F000000U)
#define CSL_QSPI_SPI_SETUP0_REG_NUM_D_BITS_SHIFT                (24U)
#define CSL_QSPI_SPI_SETUP0_REG_NUM_D_BITS_RESETVAL             (0x00000000U)
#define CSL_QSPI_SPI_SETUP0_REG_NUM_D_BITS_MAX                  (0x0000001fU)

#define CSL_QSPI_SPI_SETUP0_REG_RESETVAL                        (0x00020203U)

/* SPI_SWITCH_REG */

#define CSL_QSPI_SPI_SWITCH_REG_MMPT_S_MASK                     (0x00000001U)
#define CSL_QSPI_SPI_SWITCH_REG_MMPT_S_SHIFT                    (0U)
#define CSL_QSPI_SPI_SWITCH_REG_MMPT_S_RESETVAL                 (0x00000000U)
#define CSL_QSPI_SPI_SWITCH_REG_MMPT_S_SEL_CFG_PORT             (0x00000000U)
#define CSL_QSPI_SPI_SWITCH_REG_MMPT_S_SEL_MM_PORT              (0x00000001U)

#define CSL_QSPI_SPI_SWITCH_REG_MM_INT_EN_MASK                  (0x00000002U)
#define CSL_QSPI_SPI_SWITCH_REG_MM_INT_EN_SHIFT                 (1U)
#define CSL_QSPI_SPI_SWITCH_REG_MM_INT_EN_RESETVAL              (0x00000000U)
#define CSL_QSPI_SPI_SWITCH_REG_MM_INT_EN_MM_MODE_INTR_DISABLED  (0x00000000U)
#define CSL_QSPI_SPI_SWITCH_REG_MM_INT_EN_MM_MODE_INTR_ENABLED  (0x00000001U)

#define CSL_QSPI_SPI_SWITCH_REG_RSVD_MASK                       (0xFFFFFFFCU)
#define CSL_QSPI_SPI_SWITCH_REG_RSVD_SHIFT                      (2U)
#define CSL_QSPI_SPI_SWITCH_REG_RSVD_RESETVAL                   (0x00000000U)
#define CSL_QSPI_SPI_SWITCH_REG_RSVD_MAX                        (0x3fffffffU)

#define CSL_QSPI_SPI_SWITCH_REG_RESETVAL                        (0x00000000U)

/* SPI_SETUP1_REG */

#define CSL_QSPI_SPI_SETUP1_REG_RCMD_MASK                       (0x000000FFU)
#define CSL_QSPI_SPI_SETUP1_REG_RCMD_SHIFT                      (0U)
#define CSL_QSPI_SPI_SETUP1_REG_RCMD_RESETVAL                   (0x00000003U)
#define CSL_QSPI_SPI_SETUP1_REG_RCMD_MAX                        (0x000000ffU)

#define CSL_QSPI_SPI_SETUP1_REG_NUM_A_BYTES_MASK                (0x00000300U)
#define CSL_QSPI_SPI_SETUP1_REG_NUM_A_BYTES_SHIFT               (8U)
#define CSL_QSPI_SPI_SETUP1_REG_NUM_A_BYTES_RESETVAL            (0x00000002U)
#define CSL_QSPI_SPI_SETUP1_REG_NUM_A_BYTES_ONE_BYTE            (0x00000000U)
#define CSL_QSPI_SPI_SETUP1_REG_NUM_A_BYTES_TWO_BYTES           (0x00000001U)
#define CSL_QSPI_SPI_SETUP1_REG_NUM_A_BYTES_THREE_BYTES         (0x00000002U)
#define CSL_QSPI_SPI_SETUP1_REG_NUM_A_BYTES_FOUR_BYTES          (0x00000003U)

#define CSL_QSPI_SPI_SETUP1_REG_NUM_D_BYTES_MASK                (0x00000C00U)
#define CSL_QSPI_SPI_SETUP1_REG_NUM_D_BYTES_SHIFT               (10U)
#define CSL_QSPI_SPI_SETUP1_REG_NUM_D_BYTES_RESETVAL            (0x00000000U)
#define CSL_QSPI_SPI_SETUP1_REG_NUM_D_BYTES_USE_NUM_D_BITS      (0x00000000U)
#define CSL_QSPI_SPI_SETUP1_REG_NUM_D_BYTES_USE_8_BITS          (0x00000001U)
#define CSL_QSPI_SPI_SETUP1_REG_NUM_D_BYTES_USE_16_BITS         (0x00000002U)
#define CSL_QSPI_SPI_SETUP1_REG_NUM_D_BYTES_USE_24_BITS         (0x00000003U)

#define CSL_QSPI_SPI_SETUP1_REG_READ_TYPE_MASK                  (0x00003000U)
#define CSL_QSPI_SPI_SETUP1_REG_READ_TYPE_SHIFT                 (12U)
#define CSL_QSPI_SPI_SETUP1_REG_READ_TYPE_RESETVAL              (0x00000000U)
#define CSL_QSPI_SPI_SETUP1_REG_READ_TYPE_NORMAL_READ           (0x00000000U)
#define CSL_QSPI_SPI_SETUP1_REG_READ_TYPE_DUAL_READ             (0x00000001U)
#define CSL_QSPI_SPI_SETUP1_REG_READ_TYPE_NORMAL_READ_TYPE      (0x00000002U)
#define CSL_QSPI_SPI_SETUP1_REG_READ_TYPE_QUAD_READ             (0x00000003U)

#define CSL_QSPI_SPI_SETUP1_REG_RSVD_1_MASK                     (0x0000C000U)
#define CSL_QSPI_SPI_SETUP1_REG_RSVD_1_SHIFT                    (14U)
#define CSL_QSPI_SPI_SETUP1_REG_RSVD_1_RESETVAL                 (0x00000000U)
#define CSL_QSPI_SPI_SETUP1_REG_RSVD_1_MAX                      (0x00000003U)

#define CSL_QSPI_SPI_SETUP1_REG_WCMD_MASK                       (0x00FF0000U)
#define CSL_QSPI_SPI_SETUP1_REG_WCMD_SHIFT                      (16U)
#define CSL_QSPI_SPI_SETUP1_REG_WCMD_RESETVAL                   (0x00000002U)
#define CSL_QSPI_SPI_SETUP1_REG_WCMD_MAX                        (0x000000ffU)

#define CSL_QSPI_SPI_SETUP1_REG_RSVD_2_MASK                     (0xE0000000U)
#define CSL_QSPI_SPI_SETUP1_REG_RSVD_2_SHIFT                    (29U)
#define CSL_QSPI_SPI_SETUP1_REG_RSVD_2_RESETVAL                 (0x00000000U)
#define CSL_QSPI_SPI_SETUP1_REG_RSVD_2_MAX                      (0x00000007U)

#define CSL_QSPI_SPI_SETUP1_REG_NUM_D_BITS_MASK                 (0x1F000000U)
#define CSL_QSPI_SPI_SETUP1_REG_NUM_D_BITS_SHIFT                (24U)
#define CSL_QSPI_SPI_SETUP1_REG_NUM_D_BITS_RESETVAL             (0x00000000U)
#define CSL_QSPI_SPI_SETUP1_REG_NUM_D_BITS_MAX                  (0x0000001fU)

#define CSL_QSPI_SPI_SETUP1_REG_RESETVAL                        (0x00020203U)

/* SPI_SETUP2_REG */

#define CSL_QSPI_SPI_SETUP2_REG_RCMD_MASK                       (0x000000FFU)
#define CSL_QSPI_SPI_SETUP2_REG_RCMD_SHIFT                      (0U)
#define CSL_QSPI_SPI_SETUP2_REG_RCMD_RESETVAL                   (0x00000003U)
#define CSL_QSPI_SPI_SETUP2_REG_RCMD_MAX                        (0x000000ffU)

#define CSL_QSPI_SPI_SETUP2_REG_NUM_A_BYTES_MASK                (0x00000300U)
#define CSL_QSPI_SPI_SETUP2_REG_NUM_A_BYTES_SHIFT               (8U)
#define CSL_QSPI_SPI_SETUP2_REG_NUM_A_BYTES_RESETVAL            (0x00000002U)
#define CSL_QSPI_SPI_SETUP2_REG_NUM_A_BYTES_ONE_BYTE            (0x00000000U)
#define CSL_QSPI_SPI_SETUP2_REG_NUM_A_BYTES_TWO_BYTES           (0x00000001U)
#define CSL_QSPI_SPI_SETUP2_REG_NUM_A_BYTES_THREE_BYTES         (0x00000002U)
#define CSL_QSPI_SPI_SETUP2_REG_NUM_A_BYTES_FOUR_BYTES          (0x00000003U)

#define CSL_QSPI_SPI_SETUP2_REG_NUM_D_BYTES_MASK                (0x00000C00U)
#define CSL_QSPI_SPI_SETUP2_REG_NUM_D_BYTES_SHIFT               (10U)
#define CSL_QSPI_SPI_SETUP2_REG_NUM_D_BYTES_RESETVAL            (0x00000000U)
#define CSL_QSPI_SPI_SETUP2_REG_NUM_D_BYTES_USE_NUM_D_BITS      (0x00000000U)
#define CSL_QSPI_SPI_SETUP2_REG_NUM_D_BYTES_USE_8_BITS          (0x00000001U)
#define CSL_QSPI_SPI_SETUP2_REG_NUM_D_BYTES_USE_16_BITS         (0x00000002U)
#define CSL_QSPI_SPI_SETUP2_REG_NUM_D_BYTES_USE_24_BITS         (0x00000003U)

#define CSL_QSPI_SPI_SETUP2_REG_READ_TYPE_MASK                  (0x00003000U)
#define CSL_QSPI_SPI_SETUP2_REG_READ_TYPE_SHIFT                 (12U)
#define CSL_QSPI_SPI_SETUP2_REG_READ_TYPE_RESETVAL              (0x00000000U)
#define CSL_QSPI_SPI_SETUP2_REG_READ_TYPE_NORMAL_READ           (0x00000000U)
#define CSL_QSPI_SPI_SETUP2_REG_READ_TYPE_DUAL_READ             (0x00000001U)
#define CSL_QSPI_SPI_SETUP2_REG_READ_TYPE_NORMAL_READ_TYPE      (0x00000002U)
#define CSL_QSPI_SPI_SETUP2_REG_READ_TYPE_QUAD_READ             (0x00000003U)

#define CSL_QSPI_SPI_SETUP2_REG_RSVD_1_MASK                     (0x0000C000U)
#define CSL_QSPI_SPI_SETUP2_REG_RSVD_1_SHIFT                    (14U)
#define CSL_QSPI_SPI_SETUP2_REG_RSVD_1_RESETVAL                 (0x00000000U)
#define CSL_QSPI_SPI_SETUP2_REG_RSVD_1_MAX                      (0x00000003U)

#define CSL_QSPI_SPI_SETUP2_REG_WCMD_MASK                       (0x00FF0000U)
#define CSL_QSPI_SPI_SETUP2_REG_WCMD_SHIFT                      (16U)
#define CSL_QSPI_SPI_SETUP2_REG_WCMD_RESETVAL                   (0x00000002U)
#define CSL_QSPI_SPI_SETUP2_REG_WCMD_MAX                        (0x000000ffU)

#define CSL_QSPI_SPI_SETUP2_REG_RSVD_2_MASK                     (0xE0000000U)
#define CSL_QSPI_SPI_SETUP2_REG_RSVD_2_SHIFT                    (29U)
#define CSL_QSPI_SPI_SETUP2_REG_RSVD_2_RESETVAL                 (0x00000000U)
#define CSL_QSPI_SPI_SETUP2_REG_RSVD_2_MAX                      (0x00000007U)

#define CSL_QSPI_SPI_SETUP2_REG_NUM_D_BITS_MASK                 (0x1F000000U)
#define CSL_QSPI_SPI_SETUP2_REG_NUM_D_BITS_SHIFT                (24U)
#define CSL_QSPI_SPI_SETUP2_REG_NUM_D_BITS_RESETVAL             (0x00000000U)
#define CSL_QSPI_SPI_SETUP2_REG_NUM_D_BITS_MAX                  (0x0000001fU)

#define CSL_QSPI_SPI_SETUP2_REG_RESETVAL                        (0x00020203U)

/* SPI_SETUP3_REG */

#define CSL_QSPI_SPI_SETUP3_REG_RCMD_MASK                       (0x000000FFU)
#define CSL_QSPI_SPI_SETUP3_REG_RCMD_SHIFT                      (0U)
#define CSL_QSPI_SPI_SETUP3_REG_RCMD_RESETVAL                   (0x00000003U)
#define CSL_QSPI_SPI_SETUP3_REG_RCMD_MAX                        (0x000000ffU)

#define CSL_QSPI_SPI_SETUP3_REG_NUM_A_BYTES_MASK                (0x00000300U)
#define CSL_QSPI_SPI_SETUP3_REG_NUM_A_BYTES_SHIFT               (8U)
#define CSL_QSPI_SPI_SETUP3_REG_NUM_A_BYTES_RESETVAL            (0x00000002U)
#define CSL_QSPI_SPI_SETUP3_REG_NUM_A_BYTES_ONE_BYTE            (0x00000000U)
#define CSL_QSPI_SPI_SETUP3_REG_NUM_A_BYTES_TWO_BYTES           (0x00000001U)
#define CSL_QSPI_SPI_SETUP3_REG_NUM_A_BYTES_THREE_BYTES         (0x00000002U)
#define CSL_QSPI_SPI_SETUP3_REG_NUM_A_BYTES_FOUR_BYTES          (0x00000003U)

#define CSL_QSPI_SPI_SETUP3_REG_NUM_D_BYTES_MASK                (0x00000C00U)
#define CSL_QSPI_SPI_SETUP3_REG_NUM_D_BYTES_SHIFT               (10U)
#define CSL_QSPI_SPI_SETUP3_REG_NUM_D_BYTES_RESETVAL            (0x00000000U)
#define CSL_QSPI_SPI_SETUP3_REG_NUM_D_BYTES_USE_NUM_D_BITS      (0x00000000U)
#define CSL_QSPI_SPI_SETUP3_REG_NUM_D_BYTES_USE_8_BITS          (0x00000001U)
#define CSL_QSPI_SPI_SETUP3_REG_NUM_D_BYTES_USE_16_BITS         (0x00000002U)
#define CSL_QSPI_SPI_SETUP3_REG_NUM_D_BYTES_USE_24_BITS         (0x00000003U)

#define CSL_QSPI_SPI_SETUP3_REG_READ_TYPE_MASK                  (0x00003000U)
#define CSL_QSPI_SPI_SETUP3_REG_READ_TYPE_SHIFT                 (12U)
#define CSL_QSPI_SPI_SETUP3_REG_READ_TYPE_RESETVAL              (0x00000000U)
#define CSL_QSPI_SPI_SETUP3_REG_READ_TYPE_NORMAL_READ           (0x00000000U)
#define CSL_QSPI_SPI_SETUP3_REG_READ_TYPE_DUAL_READ             (0x00000001U)
#define CSL_QSPI_SPI_SETUP3_REG_READ_TYPE_NORMAL_READ_TYPE      (0x00000002U)
#define CSL_QSPI_SPI_SETUP3_REG_READ_TYPE_QUAD_READ             (0x00000003U)

#define CSL_QSPI_SPI_SETUP3_REG_RSVD_1_MASK                     (0x0000C000U)
#define CSL_QSPI_SPI_SETUP3_REG_RSVD_1_SHIFT                    (14U)
#define CSL_QSPI_SPI_SETUP3_REG_RSVD_1_RESETVAL                 (0x00000000U)
#define CSL_QSPI_SPI_SETUP3_REG_RSVD_1_MAX                      (0x00000003U)

#define CSL_QSPI_SPI_SETUP3_REG_WCMD_MASK                       (0x00FF0000U)
#define CSL_QSPI_SPI_SETUP3_REG_WCMD_SHIFT                      (16U)
#define CSL_QSPI_SPI_SETUP3_REG_WCMD_RESETVAL                   (0x00000002U)
#define CSL_QSPI_SPI_SETUP3_REG_WCMD_MAX                        (0x000000ffU)

#define CSL_QSPI_SPI_SETUP3_REG_RSVD_2_MASK                     (0xE0000000U)
#define CSL_QSPI_SPI_SETUP3_REG_RSVD_2_SHIFT                    (29U)
#define CSL_QSPI_SPI_SETUP3_REG_RSVD_2_RESETVAL                 (0x00000000U)
#define CSL_QSPI_SPI_SETUP3_REG_RSVD_2_MAX                      (0x00000007U)

#define CSL_QSPI_SPI_SETUP3_REG_NUM_D_BITS_MASK                 (0x1F000000U)
#define CSL_QSPI_SPI_SETUP3_REG_NUM_D_BITS_SHIFT                (24U)
#define CSL_QSPI_SPI_SETUP3_REG_NUM_D_BITS_RESETVAL             (0x00000000U)
#define CSL_QSPI_SPI_SETUP3_REG_NUM_D_BITS_MAX                  (0x0000001fU)

#define CSL_QSPI_SPI_SETUP3_REG_RESETVAL                        (0x00020203U)

/* SPI_DATA_REG_1 */

#define CSL_QSPI_SPI_DATA_REG_1_DATA_MASK                       (0xFFFFFFFFU)
#define CSL_QSPI_SPI_DATA_REG_1_DATA_SHIFT                      (0U)
#define CSL_QSPI_SPI_DATA_REG_1_DATA_RESETVAL                   (0x00000000U)
#define CSL_QSPI_SPI_DATA_REG_1_DATA_MAX                        (0xffffffffU)

#define CSL_QSPI_SPI_DATA_REG_1_RESETVAL                        (0x00000000U)

/* SPI_DATA_REG_2 */

#define CSL_QSPI_SPI_DATA_REG_2_DATA_MASK                       (0xFFFFFFFFU)
#define CSL_QSPI_SPI_DATA_REG_2_DATA_SHIFT                      (0U)
#define CSL_QSPI_SPI_DATA_REG_2_DATA_RESETVAL                   (0x00000000U)
#define CSL_QSPI_SPI_DATA_REG_2_DATA_MAX                        (0xffffffffU)

#define CSL_QSPI_SPI_DATA_REG_2_RESETVAL                        (0x00000000U)

/* SPI_DATA_REG_3 */

#define CSL_QSPI_SPI_DATA_REG_3_DATA_MASK                       (0xFFFFFFFFU)
#define CSL_QSPI_SPI_DATA_REG_3_DATA_SHIFT                      (0U)
#define CSL_QSPI_SPI_DATA_REG_3_DATA_RESETVAL                   (0x00000000U)
#define CSL_QSPI_SPI_DATA_REG_3_DATA_MAX                        (0xffffffffU)

#define CSL_QSPI_SPI_DATA_REG_3_RESETVAL                        (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
