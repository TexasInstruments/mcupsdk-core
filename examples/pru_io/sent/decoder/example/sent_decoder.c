/*
 *  Copyright (C) 2023 Texas Instruments Incorporated
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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <sent_decoder.h>

#include <sent_decoder_pru0_bin.h>
#include <sent_decoder_pru1_bin.h>



/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/*Look up Table array*/
uint8_t crc_lut_arr[] = {0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, 0x9, 0xa, 0xb, 0xc, 0xd, 0xe, 0xf, 0xd,
 0xc, 0xf, 0xe, 0x9, 0x8, 0xb, 0xa, 0x5, 0x4, 0x7, 0x6, 0x1, 0x0, 0x3, 0x2, 0x7, 0x6, 0x5, 0x4, 0x3, 0x2,
 0x1, 0x0, 0xf, 0xe, 0xd, 0xc, 0xb, 0xa, 0x9, 0x8, 0xa, 0xb, 0x8, 0x9, 0xe, 0xf, 0xc, 0xd, 0x2, 0x3, 0x0,
 0x1, 0x6, 0x7, 0x4, 0x5, 0xe, 0xf, 0xc, 0xd, 0xa, 0xb, 0x8, 0x9, 0x6, 0x7, 0x4, 0x5, 0x2, 0x3, 0x0, 0x1,
 0x3, 0x2, 0x1, 0x0, 0x7, 0x6, 0x5, 0x4, 0xb, 0xa, 0x9, 0x8, 0xf, 0xe, 0xd, 0xc, 0x9, 0x8, 0xb, 0xa, 0xd,
 0xc, 0xf, 0xe, 0x1, 0x0, 0x3, 0x2, 0x5, 0x4, 0x7, 0x6, 0x4, 0x5, 0x6, 0x7, 0x0, 0x1, 0x2, 0x3, 0xc, 0xd,
 0xe, 0xf, 0x8, 0x9, 0xa, 0xb, 0x1, 0x0, 0x3, 0x2, 0x5, 0x4, 0x7, 0x6, 0x9, 0x8, 0xb, 0xa, 0xd, 0xd, 0xf,
 0xe, 0xc, 0xd, 0xe, 0xf, 0x8, 0x9, 0xa, 0xb, 0x4, 0x5, 0x6, 0x7, 0x0, 0x1, 0x2, 0x3, 0x6, 0x7, 0x4, 0x5,
 0x2, 0x3, 0x0, 0x1, 0xe, 0xf, 0xc, 0xd, 0xa, 0xb, 0x8, 0x9, 0xb, 0xa, 0x9, 0x8, 0xf, 0xe, 0xd, 0xc, 0x3,
 0x2, 0x1, 0x0, 0x7, 0x6, 0x5, 0x4, 0xf, 0xe, 0xd, 0xc, 0xb, 0xa, 0x9, 0x8, 0x7, 0x6, 0x5, 0x4, 0x3, 0x2,
 0x1, 0x0, 0x2, 0x3, 0x0, 0x1, 0x6, 0x7, 0x4, 0x5, 0xa, 0xb, 0x8, 0x9, 0xe, 0xf, 0xc, 0xd, 0x8, 0x9, 0xa,
 0xb, 0xc, 0xd, 0xe, 0xf, 0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x5, 0x4, 0x7, 0x6, 0x1, 0x0, 0x3, 0x2,
 0xd, 0xc, 0xf, 0xe, 0x9, 0x8, 0xb, 0xa};

Sent_Obj gSentDataHandle[8];
uint32_t ch_flag_offset[] = {CH0_DATA_READY_FLAG_OFFSET, CH1_DATA_READY_FLAG_OFFSET, CH2_DATA_READY_FLAG_OFFSET, CH3_DATA_READY_FLAG_OFFSET,
                             CH4_DATA_READY_FLAG_OFFSET, CH5_DATA_READY_FLAG_OFFSET, CH6_DATA_READY_FLAG_OFFSET, CH7_DATA_READY_FLAG_OFFSET};

uint32_t ch_data_offset[] = {CH0_DATA_OFFSET, CH1_DATA_OFFSET, CH2_DATA_OFFSET, CH3_DATA_OFFSET, CH4_DATA_OFFSET, CH5_DATA_OFFSET, CH6_DATA_OFFSET, CH7_DATA_OFFSET};

uint32_t ch_flag_mask[] = {CH0_FLAG_MASK, CH1_FLAG_MASK, CH2_FLAG_MASK, CH3_FLAG_MASK, CH4_FLAG_MASK, CH5_FLAG_MASK, CH6_FLAG_MASK, CH7_FLAG_MASK};
uint32_t num_frames_recvd[8];
/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

/**
 * @brief   Initialize CRC4 Look up Table in PRU DMEM.
 *          The CRC polynomial is x^4 + x^3 + x^2 + 1 with seed value of 0101
 *
 * @param   none
 *
 * @return  void
 */
static void sent_init_crc4_lut(void);

/**
 * @brief   Initialises the PRU cores and respective Data RAMs
 *
 * @param   void
 *
 * @return  none
 */
void pruicss_init(void)
{
    gPruIcss0Handle = PRUICSS_open(CONFIG_PRU_ICSS0);
    PRUICSS_setGpMuxSelect(gPruIcss0Handle, PRUICSS_PRU0, PRUICSS_GP_MUX_SEL_MODE_GP);
    PRUICSS_setGpMuxSelect(gPruIcss0Handle, PRUICSS_PRU1, PRUICSS_GP_MUX_SEL_MODE_GP);

    /*PRU0 Init*/
    PRUICSS_initMemory(gPruIcss0Handle, PRUICSS_DATARAM(PRUICSS_PRU0));
    PRUICSS_disableCore(gPruIcss0Handle, PRUICSS_PRU0);

    /*PRU1 Init*/
    PRUICSS_initMemory(gPruIcss0Handle, PRUICSS_DATARAM(PRUICSS_PRU1));
    PRUICSS_disableCore(gPruIcss0Handle, PRUICSS_PRU1);

}

/**
 * @brief   Loads PRU Firmware and enables the core
 *
 * @param   void
 *
 * @return  none
 */
void pruicss_load_run_fw(void)
{
    /*PRU0 Load and Run*/
    PRUICSS_disableCore(gPruIcss0Handle, PRUICSS_PRU0);
    PRUICSS_disableCore(gPruIcss0Handle, PRUICSS_PRU1);

    /*Load firmware. Set buffer = write to Pru memory */
    if(PRUICSS_writeMemory(gPruIcss0Handle, PRUICSS_IRAM_PRU(PRUICSS_PRU0), 0,
                                     (uint32_t *) SentDecoderFirmwarePru0_0,
                                        sizeof(SentDecoderFirmwarePru0_0)))
    {
        if(PRUICSS_writeMemory(gPruIcss0Handle, PRUICSS_IRAM_PRU(PRUICSS_PRU1), 0,
                                        (uint32_t *) SentDecoderFirmwarePru1_0,
                                            sizeof(SentDecoderFirmwarePru1_0)))
        {
            /*Run firmware */
            PRUICSS_enableCore(gPruIcss0Handle, PRUICSS_PRU1);
            PRUICSS_enableCore(gPruIcss0Handle, PRUICSS_PRU0);
        }

    }
}

/**
 * @brief   Configures I2C controlled SOC mux to configure GPIO mode for ICSS
 *
 * @param   void
 *
 * @return  none
 */
#ifdef SOC_AM263X
 void enable_board_mux(void)
{
    /*Required to configure SOC MUX to select ICSS pins for input. 1- PWM XBAR, 0- ICSS Pins*/
    SOC_selectIcssGpiMux(0, 0x0);

    I2C_Transaction i2cTransaction;
    uint8_t buffer[2];

    I2C_Transaction_init(&i2cTransaction);
    i2cTransaction.writeBuf     = buffer;
    i2cTransaction.writeCount   = 2U;
    i2cTransaction.targetAddress = 0x20;

    /* Configure MDIO sel pin */
    /* Set output to high */
    buffer[0] = 0x02U ; /* Port 0 */
    buffer[1] = (0x03 << 2); /* Pin 2 & 3 */
    I2C_transfer(I2C_getHandle(CONFIG_I2C2), &i2cTransaction);

    /* set pin to output */
    buffer[0] = 0x6U;
    buffer[1] = ~(0x3 << 2);
    I2C_transfer(I2C_getHandle(CONFIG_I2C2), &i2cTransaction);
}
#endif
void sent_main(void *args)
{
    int status;
    uint32_t flag = 0;
    uint32_t i = 0;
    uint32_t data_ready_offset = 0;

    Drivers_open(); // check return status
    status = Board_driversOpen();
    DebugP_assert(SystemP_SUCCESS == status);

    /*Enable GPIO Mode for ICSS*/
#ifdef SOC_AM263X
    enable_board_mux();
#endif
    /*Initialise ICSS*/
    pruicss_init();
    /*Load Firmware and Run*/
    pruicss_load_run_fw();

    /*Generate CRC look up table*/
    sent_init_crc4_lut();

    DebugP_log("\n\r******************SENT DECODER Application**********************\n\r");
    DebugP_log("\r\nSENT PRU-ICSS firmware loaded and running\n\n\n");

    uint32_t *data_ready_status = ((uint32_t *)(((PRUICSS_HwAttrs const *)(gPruIcss0Handle->hwAttrs))->sharedDramBase));

    while(1)
    {
        /*Loop for all channels*/
        for (i = 0; i < 8; i++)
        {
            /*Extract SENT frame*/
            data_ready_offset = (i/4)*4;
            flag = HW_RD_REG32((uint32_t)data_ready_status + data_ready_offset);
            if((flag & ch_flag_mask[i]))
            {
                num_frames_recvd[i]++;
                HW_WR_REG8((uint32_t)data_ready_status + ch_flag_offset[i], 0x0);
                /*Extract SENT frame*/
                gSentDataHandle[i].ConfigTickTime = HW_RD_REG16((uint32_t)data_ready_status + (ch_data_offset[i] + CONFIG_TICK_TIME_OFFSET));
                gSentDataHandle[i].StatusComBit = HW_RD_REG16((uint32_t)data_ready_status + (ch_data_offset[i] + STATUS_COM_BIT_OFFSET));
                gSentDataHandle[i].Data0 = HW_RD_REG8((uint32_t)data_ready_status + (ch_data_offset[i] + DATA0_OFFSET));
                gSentDataHandle[i].Data1 = HW_RD_REG8((uint32_t)data_ready_status + (ch_data_offset[i] + DATA1_OFFSET));
                gSentDataHandle[i].Data2 = HW_RD_REG8((uint32_t)data_ready_status + (ch_data_offset[i] + DATA2_OFFSET));
                gSentDataHandle[i].Data3 = HW_RD_REG8((uint32_t)data_ready_status + (ch_data_offset[i] + DATA3_OFFSET));
                gSentDataHandle[i].Data4 = HW_RD_REG8((uint32_t)data_ready_status + (ch_data_offset[i] + DATA4_OFFSET));
                gSentDataHandle[i].Data5 = HW_RD_REG8((uint32_t)data_ready_status + (ch_data_offset[i] + DATA5_OFFSET));
                gSentDataHandle[i].CRC = HW_RD_REG8((uint32_t)data_ready_status + (ch_data_offset[i] + CRC_OFFSET));
                gSentDataHandle[i].error_status = HW_RD_REG16((uint32_t)data_ready_status + (ch_data_offset[i] + ERROR_STATUS_OFFSET));
                #ifdef _DEBUG_
                DebugP_log("\n\r******************CHANNEL%d SENT DATA**********************\n\r",i);
                DebugP_log("\n\rNumber of Frames received: \t %d", num_frames_recvd[i]);
                DebugP_log("\n\rChannel0 calculated tick period: \t %d ns", gSentDataHandle[i].ConfigTickTime);
                DebugP_log("\n\rchannel0 StatusComm Data: \t %02x", gSentDataHandle[i].StatusComBit);
                DebugP_log("\n\rChannel0:  Data0 Data1 Data2 Data3 Data4 Data5 CRC");
                DebugP_log("\n\rValue: \t  %02x \t %02x \t %02x \t %02x \t %02x \t %02x \t %02x \t", gSentDataHandle[i].Data0, gSentDataHandle[i].Data1,
                                    gSentDataHandle[i].Data2, gSentDataHandle[i].Data3, gSentDataHandle[i].Data4, gSentDataHandle[i].Data5, gSentDataHandle[i].CRC);
                DebugP_log("\n\r**********************************************************\n\r");
                #endif
            }
        }

    }

    Board_driversClose();
    Drivers_close();
}

static void sent_init_crc4_lut(void)
{
    uint32_t *lut_crc4 = ((uint32_t *)((((PRUICSS_HwAttrs const *)(gPruIcss0Handle->hwAttrs))->pru1DramBase) +
                       CRC4_LUT_OFFSET));
    memcpy(lut_crc4, crc_lut_arr, sizeof(crc_lut_arr));

}
