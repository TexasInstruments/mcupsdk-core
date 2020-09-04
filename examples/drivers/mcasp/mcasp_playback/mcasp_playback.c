/*
 *  Copyright (C) 2022 Texas Instruments Incorporated
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

/* This example demonstrates the MibSPI RX and TX operation configured
 * in blocking, interrupt mode of operation.
 *
 * This example sends a known data in the TX mode of length
 * APP_MIBSPI_MSGSIZE and then receives the same in RX mode.
 * Digital playback mode is enabled to receive data.
 *
 * When transfer is completed, TX and RX buffer data are compared.
 * If data is matched, test result is passed otherwise failed.
 */
#include <kernel/dpl/DebugP.h>
#include <drivers/i2c.h>
#include <drivers/gpio.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <kernel/dpl/ClockP.h>
#include <drivers/mcasp.h>
#include <drivers/pinmux.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* audio buffer settings */
#define APP_MCASP_AUDIO_BUFF_COUNT  (4U)
#define APP_MCASP_AUDIO_BUFF_SIZE   (512U)

/* Create buffers for transmit and Receive */
uint8_t gMcaspAudioBufferTx[APP_MCASP_AUDIO_BUFF_COUNT][APP_MCASP_AUDIO_BUFF_SIZE] __attribute__((aligned(256)));
uint8_t gMcaspAudioBufferRx[APP_MCASP_AUDIO_BUFF_COUNT][APP_MCASP_AUDIO_BUFF_SIZE] __attribute__((aligned(256)));

/* Create transaction objects for transmit and Receive */
MCASP_Transaction   gMcaspAudioTxnTx[APP_MCASP_AUDIO_BUFF_COUNT] = {0};
MCASP_Transaction   gMcaspAudioTxnRx[APP_MCASP_AUDIO_BUFF_COUNT] = {0};

/* AM273x Audio Daughter Card CODEC configuration */
#define APP_MCASP_CODEC_LINEIN_ADDR (0x48U)

/* CODEC I2C pinmux configuration */
static Pinmux_PerCfg_t gCodecI2cPinMuxCfg[] = {
        /* MSS_I2CA pin config */
    /* MSS_I2CA_SCL -> PAD_BG (Uf) */
    {
        PIN_PAD_BG,
        ( PIN_MODE(10) | PIN_PULL_DISABLE )
    },
    /* MSS_I2CA_SDA -> PAD_BF (V16) */
    {
        PIN_PAD_BF,
        ( PIN_MODE(10) | PIN_PULL_DISABLE )
    },
};

void mcasp_i2c_test(void);
void mcasp_i2c_read(uint8_t deviceAddress, uint8_t regAddress);
void mcasp_codec_config(void);

void mcasp_playback_main(void *args)
{
    int32_t         status = SystemP_SUCCESS;
    uint32_t        i;
    MCASP_Handle    mcaspHandle;
    char            valueChar;

    /* Handle MSS_I2CA pinmux config in app to workaround SysConfig bug */
    Pinmux_config(gCodecI2cPinMuxCfg, PINMUX_DOMAIN_ID_MAIN);

    Drivers_open();
    Board_driversOpen();
    mcasp_codec_config();

    DebugP_log("[MCASP] Audio playback example started.\r\n");

    mcaspHandle = MCASP_getHandle(CONFIG_MCASP0);

    /* Prepare and submit audio transaction transmit objects */
    for (i = 0U; i < APP_MCASP_AUDIO_BUFF_COUNT; i++)
    {
        gMcaspAudioTxnTx[i].buf = (void*) &gMcaspAudioBufferTx[i][0];
        gMcaspAudioTxnTx[i].count = APP_MCASP_AUDIO_BUFF_SIZE/4;
        gMcaspAudioTxnTx[i].timeout = 0xFFFFFF;
        MCASP_submitTx(mcaspHandle, &gMcaspAudioTxnTx[i]);
    }

    /* Prepare and submit audio transaction receive objects */
    for (i = 0U; i < APP_MCASP_AUDIO_BUFF_COUNT; i++)
    {
        gMcaspAudioTxnRx[i].buf = (void*) &gMcaspAudioBufferRx[i][0];
        gMcaspAudioTxnRx[i].count = APP_MCASP_AUDIO_BUFF_SIZE/4;
        gMcaspAudioTxnRx[i].timeout = 0xFFFFFF;
        MCASP_submitRx(mcaspHandle, &gMcaspAudioTxnRx[i]);
    }

    /* Trigger McASP receive operation */
    status = MCASP_startTransferRx(mcaspHandle);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Trigger McASP transmit operation */
    status = MCASP_startTransferTx(mcaspHandle);
    DebugP_assert(status == SystemP_SUCCESS);

    DebugP_log("Enter your response on UART terminal");

    do
    {
        DebugP_log("\r\nStop the demo? (y/n) : ");
        status = DebugP_scanf("%c", &valueChar);
        DebugP_assert(status == SystemP_SUCCESS);
    } while (valueChar != 'y');

    Board_driversClose();
    Drivers_close();
}

void mcasp_codec_config(void)
{
    uint8_t         txBuffer[2];
    uint8_t         deviceAddress;
    I2C_Handle      i2cHandle;
    I2C_Transaction i2cTransaction;
    uint32_t        gpioBaseAddr, pinNum;
    int32_t         status = SystemP_SUCCESS;

    /* set Line-Out DAC reset line to high */
    gpioBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_DAC_RST_BASE_ADDR);
    pinNum       = GPIO_DAC_RST_PIN;
    GPIO_setDirMode(gpioBaseAddr, pinNum, GPIO_DAC_RST_DIR);
    GPIO_pinWriteHigh(gpioBaseAddr, pinNum);

    /* set Line-Out DAC AMUTEIN line to high */
    gpioBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_DAC_AMUTEIN_BASE_ADDR);
    pinNum       = GPIO_DAC_AMUTEIN_PIN;
    GPIO_setDirMode(gpioBaseAddr, pinNum, GPIO_DAC_AMUTEIN_DIR);
    GPIO_pinWriteHigh(gpioBaseAddr, pinNum);

    /* set Line-In ADC reset line to high */
    gpioBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_LINEIN_RST_BASE_ADDR);
    pinNum       = GPIO_LINEIN_RST_PIN;
    GPIO_setDirMode(gpioBaseAddr, pinNum, GPIO_LINEIN_RST_DIR);
    GPIO_pinWriteHigh(gpioBaseAddr, pinNum);

    i2cHandle = gI2cHandle[CONFIG_I2C0];
    deviceAddress = APP_MCASP_CODEC_LINEIN_ADDR;

    status = I2C_probe(i2cHandle, deviceAddress);
    if(status == SystemP_SUCCESS)
    {
        /* Wake-up device by I2C write into P0_R2 using internal AREG */
        I2C_Transaction_init(&i2cTransaction);
        i2cTransaction.writeBuf   = txBuffer;
        i2cTransaction.writeCount = 2;
        i2cTransaction.slaveAddress = deviceAddress;
        txBuffer[0] = 0x2; /* sleep reg */
        txBuffer[1] = 0x81; /* active mode */
        I2C_transfer(i2cHandle, &i2cTransaction);

        /* Powerdown MICBIAS and ADC channels on fault detection (overtemperature, and so forth) */
        I2C_Transaction_init(&i2cTransaction);
        i2cTransaction.writeBuf   = txBuffer;
        i2cTransaction.writeCount = 2;
        i2cTransaction.slaveAddress = deviceAddress;
        txBuffer[0] = 0x28; /* sleep reg */
        txBuffer[1] = 0x10; /* active mode */
        I2C_transfer(i2cHandle, &i2cTransaction);

        /* Set ASI mode to 32-bit I2S */
        I2C_Transaction_init(&i2cTransaction);
        i2cTransaction.writeBuf   = txBuffer;
        i2cTransaction.writeCount = 2;
        i2cTransaction.slaveAddress = deviceAddress;
        txBuffer[0] = 0x7; /* ASI_CFG0 reg */
        txBuffer[1] = 0x70; /* 32-bit I2S mode */
        I2C_transfer(i2cHandle, &i2cTransaction);

        /* set CH1 input mode to line-in */
        I2C_Transaction_init(&i2cTransaction);
        i2cTransaction.writeBuf   = txBuffer;
        i2cTransaction.writeCount = 2;
        i2cTransaction.slaveAddress = deviceAddress;
        txBuffer[0] = 0x3C; /* CH1_CFG0 reg */
        txBuffer[1] = 0xB0; /* Line-in dc-coupled input */
        I2C_transfer(i2cHandle, &i2cTransaction);

        /* activate input channel */
        I2C_Transaction_init(&i2cTransaction);
        i2cTransaction.writeBuf   = txBuffer;
        i2cTransaction.writeCount = 2;
        i2cTransaction.slaveAddress = deviceAddress;
        txBuffer[0] = 0x73; /* IN_CH_EN reg */
        txBuffer[1] = 0x80; /* input channel 1 enable only */
        I2C_transfer(i2cHandle, &i2cTransaction);

        /* activate output channel */
        I2C_Transaction_init(&i2cTransaction);
        i2cTransaction.writeBuf   = txBuffer;
        i2cTransaction.writeCount = 2;
        i2cTransaction.slaveAddress = deviceAddress;
        txBuffer[0] = 0x74; /* ASI_OUT_CH_EN reg */
        txBuffer[1] = 0x80; /* output channel 1 enable only */
        I2C_transfer(i2cHandle, &i2cTransaction);

        /* Power-up ADC,MICBIAS and PLL by I2C write into P0_R117 */
        I2C_Transaction_init(&i2cTransaction);
        i2cTransaction.writeBuf   = txBuffer;
        i2cTransaction.writeCount = 2;
        i2cTransaction.slaveAddress = deviceAddress;
        txBuffer[0] = 0x75; /* PWR_CFG reg */
        txBuffer[1] = 0xE0; /* power up ADC ch and PLL and BIAS */
        I2C_transfer(i2cHandle, &i2cTransaction);
    }
    else
    {
        DebugP_log("Unable to find codec at 0x%x\r\n", deviceAddress);
    }
}

void mcasp_txcb (MCASP_Handle handle,
                          MCASP_Transaction *transaction)
{
    MCASP_submitRx(handle, transaction);
}

void mcasp_rxcb (MCASP_Handle handle,
                          MCASP_Transaction *transaction)
{
    MCASP_submitTx(handle, transaction);
}
