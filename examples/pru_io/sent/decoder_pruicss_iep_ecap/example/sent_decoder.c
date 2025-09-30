/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
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

#include <stdio.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

#include <drivers/pruicss.h>
#include <sent_decoder.h>

/* TODO: Move following macro generation to SysConfig */
#define ENABLE_ENHANCED_SERIAL_MESSAGE

#if defined(ENABLE_ENHANCED_SERIAL_MESSAGE)
#include <sent_decoder_using_iep_capture_enhanced_serial_msg_pru0_bin.h> // > PRUFirmware array
#elif defined(ENABLE_SHORT_SERIAL_MESSAGE)
#include <sent_decoder_using_iep_capture_short_serial_msg_pru0_bin.h> // > PRUFirmware array
#else
#include <sent_decoder_using_iep_capture_pru0_bin.h> // > PRUFirmware array
#endif


/**
 *  @brief PRU Core
 *  Wil come from sysconfig
 */
#define PRUICSS_PRUx                PRUICSS_PRU0

/*
 *  This is an example project to show R5F
 *  loading PRU firmware.
 */

/** \brief Global Structure pointer holding PRUSS1 memory Map. */

PRUICSS_Handle gPruIcss0Handle;

/*Look up Table array for CRC4*/
uint8_t crc4_lut_arr[] = {0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, 0x9, 0xa, 0xb, 0xc, 0xd, 0xe, 0xf, 0xd,
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


#ifdef ENABLE_ENHANCED_SERIAL_MESSAGE
/*Look up Table array for CRC6*/
uint8_t crc6_lut_arr[] = { 
0, 25, 50, 43, 61, 36, 15, 22, 35, 58, 17, 8, 30, 7, 44, 53,
31, 6, 45, 52, 34, 59, 16, 9, 60, 37, 14, 23, 1, 24, 51, 42,
62, 39, 12, 21, 3, 26, 49, 40, 29, 4, 47, 54, 32, 57, 18, 11,
33, 56, 19, 10, 28, 5, 46, 55, 2, 27, 48, 41, 63, 38, 13, 20 };
#endif

Sent_Obj gSentDataHandle[NUM_SENT_CHANNELS];

#ifdef ENABLE_SHORT_SERIAL_MESSAGE
Sent_ShortSerialMessage gSentSerialMsgDataHandle[NUM_SENT_CHANNELS];
#endif

#ifdef ENABLE_ENHANCED_SERIAL_MESSAGE
Sent_EnhancedSerialMessage gSentEnhancedSerialMsgDataHandle[NUM_SENT_CHANNELS];
#endif 

uint32_t ch_flag_offset[] = {CH0_DATA_READY_FLAG_OFFSET, CH1_DATA_READY_FLAG_OFFSET, CH2_DATA_READY_FLAG_OFFSET, CH3_DATA_READY_FLAG_OFFSET, CH4_DATA_READY_FLAG_OFFSET, CH5_DATA_READY_FLAG_OFFSET};
uint32_t ch_data_offset[] = {CH0_DATA_OFFSET, CH1_DATA_OFFSET, CH2_DATA_OFFSET, CH3_DATA_OFFSET, CH4_DATA_OFFSET, CH5_DATA_OFFSET};

#if defined(ENABLE_SHORT_SERIAL_MESSAGE) || defined (ENABLE_ENHANCED_SERIAL_MESSAGE)
uint32_t ch_serial_message_data_offset[] = {CH0_SERIAL_MSG_DATA_BASE, CH1_SERIAL_MSG_DATA_BASE, CH2_SERIAL_MSG_DATA_BASE, CH3_SERIAL_MSG_DATA_BASE, CH4_SERIAL_MSG_DATA_BASE, CH5_SERIAL_MSG_DATA_BASE};
#endif

uint32_t num_frames_recvd[NUM_SENT_CHANNELS] = {0, 0, 0, 0, 0, 0};

#if defined(ENABLE_SHORT_SERIAL_MESSAGE) || defined (ENABLE_ENHANCED_SERIAL_MESSAGE)
uint32_t num_serial_messages_recvd[NUM_SENT_CHANNELS] = {0, 0, 0, 0, 0, 0};
#endif 

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

#ifdef ENABLE_ENHANCED_SERIAL_MESSAGE
/**
 * @brief   Initialize CRC6 Look up Table in PRU DMEM.
 *          The CRC polynomial is x^6 + x^4 + x^3 + 1 with seed value of 10101
 *
 * @param   none
 *
 * @return  void
 */
static void sent_init_crc6_lut(void);
#endif

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
                                     (uint32_t *) SentDecoderFirmwarePru_0,
                                        sizeof(SentDecoderFirmwarePru_0)))
    {
            /*Run firmware */
        PRUICSS_enableCore(gPruIcss0Handle, PRUICSS_PRU0);

    }
}

/**
 * @brief   Configures I2C controlled SOC mux to configure GPIO mode for ICSS
 *
 * @param   void
 *
 * @return  none
 */
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
#ifdef SOC_AM261X
void Drivers_socTimeSyncXbar1Open1()
{
    /*
    * TIME SYNC 1 XBAR
    */
    SOC_xbarSelectTimesyncXbar1InputSource(CSL_SOC_TIMESYNC_XBAR1_U_BASE, SOC_TIMESYNC_XBAR1_ICSS_MODULE_2, SOC_TIMESYNC_XBAR1_GPIO_INT_XBAR_OUT_8);
    SOC_xbarSelectTimesyncXbar1InputSource(CSL_SOC_TIMESYNC_XBAR1_U_BASE, SOC_TIMESYNC_XBAR1_ICSS_MODULE_3, SOC_TIMESYNC_XBAR1_GPIO_INT_XBAR_OUT_9);
    SOC_xbarSelectTimesyncXbar1InputSource(CSL_SOC_TIMESYNC_XBAR1_U_BASE, SOC_TIMESYNC_XBAR1_ICSS_MODULE_4, SOC_TIMESYNC_XBAR1_GPIO_INT_XBAR_OUT_10);
    SOC_xbarSelectTimesyncXbar1InputSource(CSL_SOC_TIMESYNC_XBAR1_U_BASE, SOC_TIMESYNC_XBAR1_ICSS_MODULE_5, SOC_TIMESYNC_XBAR1_GPIO_INT_XBAR_OUT_11);
    SOC_xbarSelectTimesyncXbar1InputSource(CSL_SOC_TIMESYNC_XBAR1_U_BASE, SOC_TIMESYNC_XBAR1_ICSS_MODULE_6, SOC_TIMESYNC_XBAR1_GPIO_INT_XBAR_OUT_12);
    SOC_xbarSelectTimesyncXbar1InputSource(CSL_SOC_TIMESYNC_XBAR1_U_BASE, SOC_TIMESYNC_XBAR1_ICSS_MODULE_7, SOC_TIMESYNC_XBAR1_GPIO_INT_XBAR_OUT_13);
}
#endif
void sent_main(void *args)
{
    int status;
    uint32_t flag = 0;
    uint32_t i = 0;
#ifdef ENABLE_ENHANCED_SERIAL_MESSAGE
    uint32_t word0;
    uint32_t word1;
#endif

    Drivers_open(); // check return status
    status = Board_driversOpen();
    DebugP_assert(SystemP_SUCCESS == status);

    /*Enable GPIO Mode for ICSS*/
#ifdef SOC_AM263X
    enable_board_mux();
#endif
#ifdef SOC_AM261X
    Drivers_socTimeSyncXbar1Open1();
#endif
    /*Initialise ICSS*/
    pruicss_init();
    /*Load Firmware and Run*/
    pruicss_load_run_fw();

    /*Generate CRC look up tables*/
    sent_init_crc4_lut();
#ifdef ENABLE_ENHANCED_SERIAL_MESSAGE
    sent_init_crc6_lut();
#endif    

     DebugP_log("\n\r******************SENT DECODER Application**********************\n\r");
     DebugP_log("\r\nSENT PRU-ICSS firmware loaded and running\n\n\n");

     uint32_t *data_ready_status = ((uint32_t *)(((PRUICSS_HwAttrs const *)(gPruIcss0Handle->hwAttrs))->sharedDramBase));

     while(1)
     {
        /*Loop for all channels*/
        for (i = 0; i < NUM_SENT_CHANNELS; i++)
        {
            /*Extract SENT frame*/
            flag = HW_RD_REG8((uint32_t)data_ready_status + ch_flag_offset[i]);
            if(flag)
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
                DebugP_log("\n\r********************** CHANNEL%d SENT DATA **************************\n\r", i);
                DebugP_log("\n\rNumber of Frames received:        %d", num_frames_recvd[i]);
                DebugP_log("\n\rChannel %d calculated tick period: %d ns", i, gSentDataHandle[i].ConfigTickTime);
                DebugP_log("\n\rChannel %d StatusComm Data:        %02x", i, gSentDataHandle[i].StatusComBit);
                DebugP_log("\n\rChannel %d:\t  Data0\t Data1\t Data2\t Data3\t Data4\t Data5\t CRC\t", i);
                DebugP_log("\n\rValue:   \t  %02x\t %02x\t %02x\t %02x\t %02x\t %02x\t %02x\t", gSentDataHandle[i].Data0, gSentDataHandle[i].Data1,
                                    gSentDataHandle[i].Data2, gSentDataHandle[i].Data3, gSentDataHandle[i].Data4, gSentDataHandle[i].Data5, gSentDataHandle[i].CRC);
                DebugP_log("\n\r********************************************************************\n\r");
#endif
#ifdef ENABLE_SHORT_SERIAL_MESSAGE
                if(flag & SHORT_SERIAL_MESSAGE_DATA_READY)
                {
                    num_serial_messages_recvd[i]++;
                    
                    gSentSerialMsgDataHandle[i].MessageId = ((HW_RD_REG8((uint32_t)data_ready_status + (ch_serial_message_data_offset[i] + SHORT_SERIAL_MESSAGE_BYTE1))) & SHORT_SERIAL_MESSAGE_BYTE1_ID_MASK) >> SHORT_SERIAL_MESSAGE_BYTE1_ID_SHIFT ;
                    gSentSerialMsgDataHandle[i].Data0 = ((HW_RD_REG8((uint32_t)data_ready_status + (ch_serial_message_data_offset[i] + SHORT_SERIAL_MESSAGE_BYTE1))) & SHORT_SERIAL_MESSAGE_BYTE1_DATA0_MASK) >> SHORT_SERIAL_MESSAGE_BYTE1_DATA0_SHIFT ;
                    gSentSerialMsgDataHandle[i].Data1 = ((HW_RD_REG8((uint32_t)data_ready_status + (ch_serial_message_data_offset[i] + SHORT_SERIAL_MESSAGE_BYTE0))) & SHORT_SERIAL_MESSAGE_BYTE0_DATA1_MASK) >> SHORT_SERIAL_MESSAGE_BYTE0_DATA1_SHIFT ;
                    gSentSerialMsgDataHandle[i].CRC = ((HW_RD_REG8((uint32_t)data_ready_status + (ch_serial_message_data_offset[i] + SHORT_SERIAL_MESSAGE_BYTE0))) & SHORT_SERIAL_MESSAGE_BYTE0_CRC_MASK) >> SHORT_SERIAL_MESSAGE_BYTE0_CRC_SHIFT ;
#ifdef _DEBUG_                
                    DebugP_log("\n\r*********** CHANNEL%d SENT SHORT SERIAL MESSAGE DATA *****************\n\r", i);
                    DebugP_log("\n\rNumber of Serial Messages received: \t %d", num_serial_messages_recvd[i]);
                    DebugP_log("\n\rChannel %d:\t MessageID\t Data0\t Data1\t CRC", i);
                    DebugP_log("\n\rValue:   \t %02x\t\t %02x\t %02x\t %02x\t", gSentSerialMsgDataHandle[i].MessageId, gSentSerialMsgDataHandle[i].Data0, gSentSerialMsgDataHandle[i].Data1, gSentSerialMsgDataHandle[i].CRC);
                    DebugP_log("\n\r********************************************************************\n\r");
#endif
                }
#endif
#ifdef ENABLE_ENHANCED_SERIAL_MESSAGE
                if(flag & ENHANCED_SERIAL_MESSAGE_DATA_READY)
                {
                    num_serial_messages_recvd[i]++;

                    word0 =  HW_RD_REG32((uint32_t)data_ready_status + (ch_serial_message_data_offset[i] + ENHANCED_SERIAL_MESSAGE_WORD0));
                    word1 =  HW_RD_REG32((uint32_t)data_ready_status + (ch_serial_message_data_offset[i] + ENHANCED_SERIAL_MESSAGE_WORD1));
                    gSentEnhancedSerialMsgDataHandle[i].CRC = (word0 & ENHANCED_SERIAL_MESSAGE_WORD0_CRC_MASK) >> ENHANCED_SERIAL_MESSAGE_WORD0_CRC_SHIFT;
                    gSentEnhancedSerialMsgDataHandle[i].ConfigBit = (word1 & ENHANCED_SERIAL_MESSAGE_WORD1_CONFIG_MASK) >> ENHANCED_SERIAL_MESSAGE_WORD1_CONFIG_SHIFT;
                    /* Read lower 12 bits of data */
                    gSentEnhancedSerialMsgDataHandle[i].Data = (word0 & ENHANCED_SERIAL_MESSAGE_WORD0_DATA_MASK) >> ENHANCED_SERIAL_MESSAGE_WORD0_DATA_SHIFT;

                    if(gSentEnhancedSerialMsgDataHandle[i].ConfigBit)
                    {
                        /* Config Bit = 1, Data is 16 bits, Message ID is 4 bits*/

                        /* Read upper 4 bits of data */
                        gSentEnhancedSerialMsgDataHandle[i].Data |= ((word1 & ENHANCED_SERIAL_MESSAGE_WORD1_C1_DATA_MASK) >> ENHANCED_SERIAL_MESSAGE_WORD1_C1_DATA_SHIFT) << 12;

                        gSentEnhancedSerialMsgDataHandle[i].MessageId = ((word1 & ENHANCED_SERIAL_MESSAGE_WORD1_C1_ID_MASK) >> ENHANCED_SERIAL_MESSAGE_WORD1_C1_ID_SHIFT);
                    }
                    else
                    {
                        /* Config Bit = 0, Data is 12 bits, Message ID is 8 bits*/
                        gSentEnhancedSerialMsgDataHandle[i].MessageId = ((word1 & ENHANCED_SERIAL_MESSAGE_WORD1_C0_ID_LOW_MASK) >> ENHANCED_SERIAL_MESSAGE_WORD1_C0_ID_LOW_SHIFT) |
                        ((((word1 & ENHANCED_SERIAL_MESSAGE_WORD1_C0_ID_HIGH_MASK) >> ENHANCED_SERIAL_MESSAGE_WORD1_C0_ID_HIGH_SHIFT))<<4);
                    }
#ifdef _DEBUG_ 
                    DebugP_log("\n\r********* CHANNEL%d SENT ENHANCED SERIAL MESSAGE DATA ***************\n\r", i);
                    DebugP_log("\n\rNumber of Serial Messages received: \t %d", num_serial_messages_recvd[i]);
                    if(gSentEnhancedSerialMsgDataHandle[i].ConfigBit)
                    {
                        DebugP_log("\n\rConfig Bit = 1, Data is 16 bits, Message ID is 4 bits");
                    }
                    else
                    {
                        DebugP_log("\n\rConfig Bit = 0, Data is 12 bits, Message ID is 8 bits");
                    }
                    DebugP_log("\n\rChannel %d:\t MessageID\t Data\t CRC", i);
                    DebugP_log("\n\rValue:   \t %02x\t\t %04x\t %02x\t", gSentEnhancedSerialMsgDataHandle[i].MessageId, gSentEnhancedSerialMsgDataHandle[i].Data, gSentEnhancedSerialMsgDataHandle[i].CRC);
                    DebugP_log("\n\r********************************************************************\n\r");
#endif
                }
#endif                
            }
        }
     }

    Board_driversClose();
    Drivers_close();
}

static void sent_init_crc4_lut(void)
{
    uint32_t *lut_crc4 = ((uint32_t *)((((PRUICSS_HwAttrs const *)(gPruIcss0Handle->hwAttrs))->pru0DramBase) +
                       CRC4_LUT_OFFSET));
    memcpy(lut_crc4, crc4_lut_arr, sizeof(crc4_lut_arr));
}

#ifdef ENABLE_ENHANCED_SERIAL_MESSAGE

static void sent_init_crc6_lut(void)
{
    uint32_t *lut_crc6 = ((uint32_t *)((((PRUICSS_HwAttrs const *)(gPruIcss0Handle->hwAttrs))->pru0DramBase) +
                       CRC6_LUT_OFFSET));
    memcpy(lut_crc6, crc6_lut_arr, sizeof(crc6_lut_arr));
}
#endif
