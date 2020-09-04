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

#include <drivers/csirx.h>
#include <drivers/hw_include/cslr_csirx.h>
#include <float.h>
#include <math.h>
#include <string.h>

#if defined (SOC_AM273X)
#include <drivers/hw_include/am273x/cslr_rcss_ctrl.h>
#endif
#if defined (SOC_AWR294X)
#include <drivers/hw_include/awr294x/cslr_rss_ctrl.h>
#endif

/* Max HW instances that are possible */
#define CSIRX_HW_INSTANCES_MAX  (2U)

#define CSIRX_PING_PONG_LINEOFFSET_ALIGNMENT_MASK    \
        (CSIRX_PING_PONG_ADDRESS_LINEOFFSET_ALIGNMENT_IN_BYTES - 1U)


int32_t CSIRX_contextSetConfig (CSIRX_Handle handle, uint8_t contextId, CSIRX_ContextConfig *contextConfig)
{
    int32_t status = SystemP_SUCCESS;
    CSIRX_Config *config = (CSIRX_Config *)handle;

    if(    config == NULL
        || config->object == NULL
        || config->hwAttrs == NULL
        || config->hwAttrs->csirxRegs == 0
        || config->hwAttrs->rcssCtrlRegs == 0
        || contextId >= CSIRX_CONTEXTS_MAX
        || contextConfig == NULL
      )
    {
        status = SystemP_FAILURE;
    }
    else
    {
        if(    contextConfig->virtualChannelId >= 4
            || (contextConfig->pingPongConfig.lineOffset & CSIRX_PING_PONG_LINEOFFSET_ALIGNMENT_MASK) != 0
            || (contextConfig->transcodeConfig.transcodeFormat == CSIRX_TRANSCODE_FORMAT_NO_TRANSCODE
                    && contextConfig->transcodeConfig.isHorizontalDownscalingBy2Enabled )
            || (contextConfig->transcodeConfig.crop.horizontalCount > 0x1FFF)
            || (contextConfig->transcodeConfig.crop.horizontalSkip > 0x1FFF)
            || (contextConfig->transcodeConfig.crop.verticalCount > 0x1FFF)
            || (contextConfig->transcodeConfig.crop.verticalSkip > 0x1FFF)
          )
        {
            status = SystemP_FAILURE;
        }
        else
        {
            CSL_csirxRegs *regs = (CSL_csirxRegs *) config->hwAttrs->csirxRegs;
            CSL_csirxContextRegs *ctxRegs = &regs->ctx[contextId];
            CSL_csirxContextTranscodeRegs *ctxTranscodeRegs = &regs->ctxTranscode[contextId];

            uint32_t regVal;

            regVal =  CSL_FMK(CSI2_CTX0_CTRL1_BYTESWAP, contextConfig->isByteSwapEnabled )
                    | CSL_FMK(CSI2_CTX0_CTRL1_GENERIC, contextConfig->isGenericEnabled )
                    | CSL_FMK(CSI2_CTX0_CTRL1_HSCALE, contextConfig->transcodeConfig.isHorizontalDownscalingBy2Enabled )
                    | CSL_FMK(CSI2_CTX0_CTRL1_TRANSCODE, contextConfig->transcodeConfig.transcodeFormat )
                    | CSL_FMK(CSI2_CTX0_CTRL1_COUNT, contextConfig->numFramesToAcquire )
                    | CSL_FMK(CSI2_CTX0_CTRL1_COUNT_UNLOCK, 1 )
                    | CSL_FMK(CSI2_CTX0_CTRL1_LINE_MODULO, contextConfig->isGenerateIntrEveryNumLinesForIntr )
                    | CSL_FMK(CSI2_CTX0_CTRL1_FEC_NUMBER, contextConfig->pingPongConfig.numFramesForFrameBasedPingPongSwitching )
                    | CSL_FMK(CSI2_CTX0_CTRL1_EOF_EN, contextConfig->isEndOfFramePulseEnabled )
                    | CSL_FMK(CSI2_CTX0_CTRL1_EOL_EN, contextConfig->isEndOfLinePulseEnabled )
                    | CSL_FMK(CSI2_CTX0_CTRL1_CS_EN, contextConfig->isPayloadChecksumEnable )
                    ;

            ctxRegs->CTX_CTRL1 = regVal;

            regVal =  CSL_FMK(CSI2_CTX0_CTRL2_USER_DEF_MAPPING, contextConfig->userDefinedMapping )
                    | CSL_FMK(CSI2_CTX0_CTRL2_VIRTUAL_ID, contextConfig->virtualChannelId )
                    | CSL_FMK(CSI2_CTX0_CTRL2_FORMAT, contextConfig->format )
                    ;

            ctxRegs->CTX_CTRL2 = regVal;

            regVal =  CSL_FMK(CSI2_CTX0_CTRL3_ALPHA, contextConfig->alpha )
                    | CSL_FMK(CSI2_CTX0_CTRL3_LINE_NUMBER, contextConfig->numLinesForIntr )
                    ;

            ctxRegs->CTX_CTRL3 = regVal;

            ctxRegs->CTX_DAT_OFST = contextConfig->pingPongConfig.lineOffset;

            regVal =  ( ( contextConfig->enableIntr.isNumLines & 0x1 ) << 7 )
                    | ( ( contextConfig->enableIntr.isFramesToAcquire & 0x1 ) << 6 )
                    | ( ( contextConfig->enableIntr.isPayloadChecksumMismatch & 0x1 ) << 5 )
                    | ( ( contextConfig->enableIntr.isLineStartCodeDetect & 0x1 ) << 2 )
                    | ( ( contextConfig->enableIntr.isLineEndCodeDetect & 0x1 ) << 3 )
                    | ( ( contextConfig->enableIntr.isFrameStartCodeDetect & 0x1 ) << 0 )
                    | ( ( contextConfig->enableIntr.isFrameEndCodeDetect & 0x1 ) << 1 )
                    | ( ( contextConfig->enableIntr.isLongPacketOneBitErrorCorrect & 0x1 ) << 8 )
                    ;

            ctxRegs->CTX_IRQENABLE = regVal;
            ctxRegs->CTX_IRQSTATUS = 0xFFFFFFFF; /* clear all pending interrupts */

            regVal =  CSL_FMK(CSI2_CTX0_TRANSCODEH_HCOUNT , contextConfig->transcodeConfig.crop.horizontalCount )
                    | CSL_FMK(CSI2_CTX0_TRANSCODEH_HSKIP , contextConfig->transcodeConfig.crop.horizontalSkip )
                    ;

            ctxTranscodeRegs->CTX_TRANSCODEH = regVal;

            regVal =  CSL_FMK(CSI2_CTX0_TRANSCODEV_VCOUNT , contextConfig->transcodeConfig.crop.verticalCount )
                    | CSL_FMK(CSI2_CTX0_TRANSCODEV_VSKIP , contextConfig->transcodeConfig.crop.verticalSkip )
                    ;

            ctxTranscodeRegs->CTX_TRANSCODEV = regVal;

            #if defined (SOC_AM273X)
            {
                CSL_rcss_ctrlRegs *rcssCtrlRegs = (CSL_rcss_ctrlRegs *) config->hwAttrs->rcssCtrlRegs;

                regVal = ( (contextConfig->pingPongConfig.pingPongSwitchMode & 0x1) << 16)
                    | (contextConfig->pingPongConfig.numLinesForLineBasedPingPongSwitching & 0xFFFF)
                    ;

                if(config->hwAttrs->hwInstId == 0)
                {
                    rcssCtrlRegs->RCSS_CSI2A_CTX_LINE_PING_PONG[contextId] = regVal;
                }
                else
                {
                    rcssCtrlRegs->RCSS_CSI2B_CTX_LINE_PING_PONG[contextId] = regVal;
                }
            }
            #endif
            #if defined (SOC_AWR294X)
            {
                CSL_rss_ctrlRegs *rcssCtrlRegs = (CSL_rss_ctrlRegs *) config->hwAttrs->rcssCtrlRegs;

                regVal = ( (contextConfig->pingPongConfig.pingPongSwitchMode & 0x1) << 16)
                    | (contextConfig->pingPongConfig.numLinesForLineBasedPingPongSwitching & 0xFFFF);
                rcssCtrlRegs->RSS_CSI2A_CTX_LINE_PING_PONG[contextId] = regVal;
            }
            #endif

            status = CSIRX_contextSetPingPongAddress(handle, contextId,
                            contextConfig->pingPongConfig.pingAddress, contextConfig->pingPongConfig.pongAddress);

        }
    }
    return status;
}

int32_t CSIRX_contextSetPingPongAddress (CSIRX_Handle handle, uint8_t contextId, uint32_t pingAddress, uint32_t pongAddress)
{
    int32_t status = SystemP_SUCCESS;
    CSIRX_Config *config = (CSIRX_Config *)handle;

    if(    config == NULL
        || config->object == NULL
        || config->hwAttrs == NULL
        || config->hwAttrs->csirxRegs == 0
        || contextId >= CSIRX_CONTEXTS_MAX
        || (pingAddress & CSIRX_PING_PONG_LINEOFFSET_ALIGNMENT_MASK) != 0
        || (pongAddress & CSIRX_PING_PONG_LINEOFFSET_ALIGNMENT_MASK) != 0
      )
    {
        status = SystemP_FAILURE;
    }
    else
    {
        CSL_csirxRegs *regs = (CSL_csirxRegs *) config->hwAttrs->csirxRegs;

        regs->ctx[contextId].CTX_DAT_PING_ADDR = pingAddress;
        regs->ctx[contextId].CTX_DAT_PONG_ADDR = pongAddress;

        config->object->context[contextId].pingAddress = pingAddress;
        config->object->context[contextId].pongAddress = pongAddress;
    }
    return status;
}

int32_t CSIRX_contextSetLineOffset(CSIRX_Handle handle, uint8_t contextId, int32_t lineOffset)
{
    int32_t status = SystemP_SUCCESS;
    CSIRX_Config *config = (CSIRX_Config *)handle;

    if(    config == NULL
        || config->object == NULL
        || config->hwAttrs == NULL
        || config->hwAttrs->csirxRegs == 0
        || contextId >= CSIRX_CONTEXTS_MAX
      )
    {
        status = SystemP_FAILURE;
    }
    else
    {
        CSL_csirxRegs *regs = (CSL_csirxRegs *) config->hwAttrs->csirxRegs;

        regs->ctx[contextId].CTX_DAT_OFST = lineOffset;
    }
    return status;
}

static int32_t CSIRX_contextEnableOrDisable(CSIRX_Handle handle, uint8_t contextId, bool isEnable)
{
    int32_t status = SystemP_SUCCESS;
    CSIRX_Config *config = (CSIRX_Config *)handle;

    if(    config == NULL
        || config->object == NULL
        || config->hwAttrs == NULL
        || config->hwAttrs->csirxRegs == 0
        || contextId >= CSIRX_CONTEXTS_MAX
      )
    {
        status = SystemP_FAILURE;
    }
    else
    {
        CSL_csirxRegs *regs = (CSL_csirxRegs *) config->hwAttrs->csirxRegs;

        CSL_REG32_FINS(&regs->ctx[contextId].CTX_CTRL1, CSI2_CTX0_CTRL1_CTX_EN, isEnable);
    }
    return status;
}

int32_t CSIRX_contextEnable(CSIRX_Handle handle, uint8_t contextId)
{
    return CSIRX_contextEnableOrDisable(handle, contextId, 1);
}

int32_t CSIRX_contextDisable(CSIRX_Handle handle, uint8_t contextId)
{
    return CSIRX_contextEnableOrDisable(handle, contextId, 0);
}

int32_t CSIRX_contextClearAllIntr (CSIRX_Handle handle, uint8_t contextId)
{
    int32_t status = SystemP_SUCCESS;
    CSIRX_Config *config = (CSIRX_Config *)handle;

    if(    config == NULL
        || config->object == NULL
        || config->hwAttrs == NULL
        || config->hwAttrs->csirxRegs == 0
        || contextId >= CSIRX_CONTEXTS_MAX
      )
    {
        status = SystemP_FAILURE;
    }
    else
    {
        CSL_csirxRegs *regs = (CSL_csirxRegs *) config->hwAttrs->csirxRegs;

        regs->ctx[contextId].CTX_IRQSTATUS = regs->ctx[contextId].CTX_IRQSTATUS;
    }
    return status;
}

int32_t CSIRX_contextGetPendingIntr(CSIRX_Handle handle, uint8_t contextId, CSIRX_ContextIntr *intrStatus)
{
    int32_t status = SystemP_SUCCESS;
    CSIRX_Config *config = (CSIRX_Config *)handle;

    if(    config == NULL
        || config->object == NULL
        || config->hwAttrs == NULL
        || config->hwAttrs->csirxRegs == 0
        || contextId >= CSIRX_CONTEXTS_MAX
        || intrStatus == NULL
      )
    {
        status = SystemP_FAILURE;
    }
    else
    {
        CSL_csirxRegs *regs = (CSL_csirxRegs *) config->hwAttrs->csirxRegs;
        uint32_t regVal = regs->ctx[contextId].CTX_IRQSTATUS;

        intrStatus->isNumLines                      = ( regVal >> 7 ) & 0x1;
        intrStatus->isFramesToAcquire               = ( regVal >> 6 ) & 0x1;
        intrStatus->isPayloadChecksumMismatch       = ( regVal >> 5 ) & 0x1;
        intrStatus->isLineStartCodeDetect           = ( regVal >> 2 ) & 0x1;
        intrStatus->isLineEndCodeDetect             = ( regVal >> 3 ) & 0x1;
        intrStatus->isFrameStartCodeDetect          = ( regVal >> 0 ) & 0x1;
        intrStatus->isFrameEndCodeDetect            = ( regVal >> 1 ) & 0x1;
        intrStatus->isLongPacketOneBitErrorCorrect  = ( regVal >> 8 ) & 0x1;
    }
    return status;
}

int32_t CSIRX_contextGetFrameNumber (CSIRX_Handle handle, uint8_t contextId, uint16_t *frameNumber)
{
    int32_t status = SystemP_SUCCESS;
    CSIRX_Config *config = (CSIRX_Config *)handle;

    if(    config == NULL
        || config->object == NULL
        || config->hwAttrs == NULL
        || config->hwAttrs->csirxRegs == 0
        || contextId >= CSIRX_CONTEXTS_MAX
        || frameNumber == NULL
      )
    {
        status = SystemP_FAILURE;
    }
    else
    {
        CSL_csirxRegs *regs = (CSL_csirxRegs *) config->hwAttrs->csirxRegs;

        *frameNumber = CSL_REG32_FEXT(&regs->ctx[contextId].CTX_CTRL2, CSI2_CTX0_CTRL2_FRAME);
    }
    return status;
}

int32_t CSIRX_contextGetRecvAddress (CSIRX_Handle handle, uint8_t contextId, uint32_t *bufAddress)
{
    int32_t status = SystemP_SUCCESS;
    CSIRX_Config *config = (CSIRX_Config *)handle;

    if(    config == NULL
        || config->object == NULL
        || config->hwAttrs == NULL
        || config->hwAttrs->csirxRegs == 0
        || contextId >= CSIRX_CONTEXTS_MAX
        || bufAddress == NULL
      )
    {
        status = SystemP_FAILURE;
    }
    else
    {
        CSL_csirxRegs *regs = (CSL_csirxRegs *) config->hwAttrs->csirxRegs;

        uint8_t pingPongStatus  = CSL_REG32_FEXT(&regs->ctx[contextId].CTX_CTRL1, CSI2_CTX0_CTRL1_PING_PONG);

        if(pingPongStatus == CSIRX_CONTEXT_PINGPONG_STATUS_PING_DONE)
        {
            *bufAddress = config->object->context[contextId].pingAddress;
        }
        else
        {
            *bufAddress = config->object->context[contextId].pongAddress;
        }
    }
    return status;
}

int32_t CSIRX_contextGetPingPongStatus (CSIRX_Handle handle, uint8_t contextId, uint8_t *pingPongStatus)
{
    int32_t status = SystemP_SUCCESS;
    CSIRX_Config *config = (CSIRX_Config *)handle;

    if(    config == NULL
        || config->object == NULL
        || config->hwAttrs == NULL
        || config->hwAttrs->csirxRegs == 0
        || contextId >= CSIRX_CONTEXTS_MAX
        || pingPongStatus == NULL
      )
    {
        status = SystemP_FAILURE;
    }
    else
    {
        CSL_csirxRegs *regs = (CSL_csirxRegs *) config->hwAttrs->csirxRegs;

        *pingPongStatus  = CSL_REG32_FEXT(&regs->ctx[contextId].CTX_CTRL1, CSI2_CTX0_CTRL1_PING_PONG);
    }
    return status;
}

static int32_t CSIRX_debugModeSetPacketHeader(CSIRX_Handle handle, uint32_t packetHeader)
{
    int32_t status = SystemP_SUCCESS;
    CSIRX_Config *config = (CSIRX_Config *)handle;

    if(    config == NULL
        || config->object == NULL
        || config->hwAttrs == NULL
        || config->hwAttrs->csirxRegs == 0
      )
    {
        status = SystemP_FAILURE;
    }
    else
    {
        CSL_csirxRegs *regs = (CSL_csirxRegs *) config->hwAttrs->csirxRegs;

        regs->DBG_H = packetHeader;
    }
    return status;
}

int32_t CSIRX_debugModeSetShortPacket (CSIRX_Handle handle, uint32_t shortPacket)
{
    return CSIRX_debugModeSetPacketHeader(handle, shortPacket);
}

int32_t CSIRX_debugModeSetLongPacketHeader (CSIRX_Handle handle, uint32_t longPacketHeader)
{
    return CSIRX_debugModeSetPacketHeader(handle, longPacketHeader);
}

int32_t CSIRX_debugModeSetLongPacketPayload(CSIRX_Handle handle, uint32_t payload)
{
    int32_t status = SystemP_SUCCESS;
    CSIRX_Config *config = (CSIRX_Config *)handle;

    if(    config == NULL
        || config->object == NULL
        || config->hwAttrs == NULL
        || config->hwAttrs->csirxRegs == 0
      )
    {
        status = SystemP_FAILURE;
    }
    else
    {
        CSL_csirxRegs *regs = (CSL_csirxRegs *) config->hwAttrs->csirxRegs;

        regs->DBG_P = payload;
    }
    return status;
}

static int32_t CSIRX_debugModeEnableOrDisable(CSIRX_Handle handle, bool isEnable)
{
    int32_t status = SystemP_SUCCESS;
    CSIRX_Config *config = (CSIRX_Config *)handle;

    if(    config == NULL
        || config->object == NULL
        || config->hwAttrs == NULL
        || config->hwAttrs->csirxRegs == 0
      )
    {
        status = SystemP_FAILURE;
    }
    else
    {
        CSL_csirxRegs *regs = (CSL_csirxRegs *) config->hwAttrs->csirxRegs;

        CSL_REG32_FINS(&regs->CTRL, CSI2_CTRL_DBG_EN, isEnable);
    }
    return status;
}

int32_t CSIRX_debugModeEnable(CSIRX_Handle handle)
{
    return CSIRX_debugModeEnableOrDisable(handle, 1);
}

int32_t CSIRX_debugModeDisable(CSIRX_Handle handle)
{
    return CSIRX_debugModeEnableOrDisable(handle, 0);
}

int32_t CSIRX_dphySetConfig (CSIRX_Handle handle, CSIRX_DphyConfig *dphyConfig)
{
    int32_t status = SystemP_SUCCESS;
    CSIRX_Config *config = (CSIRX_Config *)handle;


    if(    config == NULL
        || config->object == NULL
        || config->hwAttrs == NULL
        || config->hwAttrs->csirxRegs == 0
        || config->hwAttrs->ctrlClockHz == 0
        || dphyConfig == NULL
      )
    {
        status = SystemP_FAILURE;
    }
    else
    {
        CSL_csirxRegs *regs = (CSL_csirxRegs *) config->hwAttrs->csirxRegs;
        uint32_t ths_settle, ths_term, tclk_term, tclk_settle, ctrl_clock_div_factor;

        ths_term   = (uint32_t)ceil(12.5e-9 * (float)dphyConfig->ddrClockInHz) - 1U;
        if (ths_term > CSL_CSI2_PHY_CFG_REG0_THS_TERM_MAX)
        {
            status = SystemP_FAILURE;
        }

        ths_settle = (uint32_t)ceil(90.0e-9 * (float)dphyConfig->ddrClockInHz) + 3U;
        if (ths_settle > CSL_CSI2_PHY_CFG_REG0_THS_SETTLE_MAX)
        {
            status = SystemP_FAILURE;
        }

        tclk_term   = (uint32_t)ceil(9.5e-9 * (float)config->hwAttrs->ctrlClockHz) - 1U;
        if (tclk_term > CSL_CSI2_PHY_CFG_REG1_TCLK_TERM_MAX)
        {
            status = SystemP_FAILURE;
        }

        ctrl_clock_div_factor = (uint32_t)ceil(15.0e-9 * (float)config->hwAttrs->ctrlClockHz) - 1U;
        if (ctrl_clock_div_factor > CSL_CSI2_PHY_CFG_REG1_CTRLCLK_DIV_FACT_MAX)
        {
            status = SystemP_FAILURE;
        }

        tclk_settle = CSL_MAX(3U, ((uint32_t)ceil(155.0e-9 * (float)config->hwAttrs->ctrlClockHz) -
                                1U));
        if (tclk_settle > CSL_CSI2_PHY_CFG_REG1_TCLK_SETTLE_MAX)
        {
            status = SystemP_FAILURE;
        }

        if(status == SystemP_SUCCESS)
        {
            /* REG0 */
            CSL_REG32_FINS(&regs->PHY_CFG_REG0,
                            CSI2_PHY_CFG_REG0_THS_TERM,
                            ths_term);
            CSL_REG32_FINS(&regs->PHY_CFG_REG0,
                            CSI2_PHY_CFG_REG0_THS_SETTLE,
                            ths_settle);
            CSL_REG32_FINS(&regs->PHY_CFG_REG0,
                            CSI2_PHY_CFG_REG0_HS_CLK_CONFIG,
                            dphyConfig->isClockMissingDetectionEnabled);

            /* REG1 */
            CSL_REG32_FINS(&regs->PHY_CFG_REG1,
                            CSI2_PHY_CFG_REG1_TCLK_TERM,
                            tclk_term);
            CSL_REG32_FINS(&regs->PHY_CFG_REG1,
                            CSI2_PHY_CFG_REG1_CTRLCLK_DIV_FACT,
                            ctrl_clock_div_factor);
            CSL_REG32_FINS(&regs->PHY_CFG_REG1,
                            CSI2_PHY_CFG_REG1_TCLK_SETTLE,
                            tclk_settle);

            /* REG2 */
            CSL_REG32_FINS(&regs->PHY_CFG_REG2,
                            CSI2_PHY_CFG_REG2_RXTRIGGERESC0,
                            dphyConfig->triggerEscapeCode[0]);
            CSL_REG32_FINS(&regs->PHY_CFG_REG2,
                            CSI2_PHY_CFG_REG2_RXTRIGGERESC1,
                            dphyConfig->triggerEscapeCode[1]);
            CSL_REG32_FINS(&regs->PHY_CFG_REG2,
                            CSI2_PHY_CFG_REG2_RXTRIGGERESC2,
                            dphyConfig->triggerEscapeCode[2]);
            CSL_REG32_FINS(&regs->PHY_CFG_REG2,
                            CSI2_PHY_CFG_REG2_RXTRIGGERESC3,
                            dphyConfig->triggerEscapeCode[3]);
        }
    }
    return status;
}

int32_t CSIRX_dphyIsControlClockResetDone (CSIRX_Handle handle, bool *isDone)
{
    int32_t status = SystemP_SUCCESS;
    CSIRX_Config *config = (CSIRX_Config *)handle;

    if(    config == NULL
        || config->object == NULL
        || config->hwAttrs == NULL
        || config->hwAttrs->csirxRegs == 0
        || isDone == NULL
      )
    {
        status = SystemP_FAILURE;
    }
    else
    {
        CSL_csirxRegs *regs = (CSL_csirxRegs *) config->hwAttrs->csirxRegs;

        *isDone = (bool)CSL_REG32_FEXT(&regs->PHY_CFG_REG1, CSI2_PHY_CFG_REG1_RESETDONECTRLCLK);
    }
    return status;
}

int32_t CSIRX_dphyIsByteClockResetDone (CSIRX_Handle handle, bool *isDone)
{
    int32_t status = SystemP_SUCCESS;
    CSIRX_Config *config = (CSIRX_Config *)handle;

    if(    config == NULL
        || config->object == NULL
        || config->hwAttrs == NULL
        || config->hwAttrs->csirxRegs == 0
        || isDone == NULL
      )
    {
        status = SystemP_FAILURE;
    }
    else
    {
        CSL_csirxRegs *regs = (CSL_csirxRegs *) config->hwAttrs->csirxRegs;

        *isDone = (bool)CSL_REG32_FEXT(&regs->PHY_CFG_REG1, CSI2_PHY_CFG_REG1_RESETDONERXBYTECLK);
    }
    return status;
}

int32_t CSIRX_dphyIsClockMissingDetectorError (CSIRX_Handle handle, bool *isError)
{
    int32_t status = SystemP_SUCCESS;
    CSIRX_Config *config = (CSIRX_Config *)handle;

    if(    config == NULL
        || config->object == NULL
        || config->hwAttrs == NULL
        || config->hwAttrs->csirxRegs == 0
        || isError == NULL
      )
    {
        status = SystemP_FAILURE;
    }
    else
    {
        CSL_csirxRegs *regs = (CSL_csirxRegs *) config->hwAttrs->csirxRegs;

        *isError = (bool)CSL_REG32_FEXT(&regs->PHY_CFG_REG1, CSI2_PHY_CFG_REG1_CLK_MISS_DET);
    }
    return status;
}

#if defined (SOC_AM273X)
static void CSIRX_complexioSetPullUpPullDown(uint32_t baseAddr, uint32_t lanePos, uint32_t isEnable)
{
    uint32_t addr;

    /* calculate the address of the lane register implied by the lane position */
    addr = baseAddr +
           ((uint32_t)lanePos * CSL_RCSS_CTRL_CSI2_INTER_LANE_CFG_ADDR_DISTANCE);

    /* DX0 pull-up */
    CSL_REG32_FINS(
        addr,
        RCSS_CTRL_RCSS_CSI2A_LANE0_CFG_RCSS_CSI2A_LANE0_CFG_DX0_ENBPU,
        (uint32_t)isEnable);

    /* DX0 pull-down */
    CSL_REG32_FINS(
        addr,
        RCSS_CTRL_RCSS_CSI2A_LANE0_CFG_RCSS_CSI2A_LANE0_CFG_DX0_ENBPD,
        (uint32_t)isEnable);

    /* DY0 pull-up */
    CSL_REG32_FINS(
        addr,
        RCSS_CTRL_RCSS_CSI2A_LANE0_CFG_RCSS_CSI2A_LANE0_CFG_DY0_ENBPU,
        (uint32_t)isEnable);

    /* DY0 pull-down */
    CSL_REG32_FINS(
        addr,
        RCSS_CTRL_RCSS_CSI2A_LANE0_CFG_RCSS_CSI2A_LANE0_CFG_DY0_ENBPD,
        (uint32_t)isEnable);
}
#endif
#if defined (SOC_AWR294X)
static void CSIRX_complexioSetPullUpPullDown(uint32_t baseAddr, uint32_t lanePos, uint32_t isEnable)
{
    uint32_t addr;

    /* calculate the address of the lane register implied by the lane position */
    addr = baseAddr +
           ((uint32_t)lanePos * CSL_RSS_CTRL_CSI2_INTER_LANE_CFG_ADDR_DISTANCE);

    /* DX0 pull-up */
    CSL_REG32_FINS(
        addr,
        RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DX0_ENBPU,
        (uint32_t)isEnable);

    /* DX0 pull-down */
    CSL_REG32_FINS(
        addr,
        RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DX0_ENBPD,
        (uint32_t)isEnable);

    /* DY0 pull-up */
    CSL_REG32_FINS(
        addr,
        RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DY0_ENBPU,
        (uint32_t)isEnable);

    /* DY0 pull-down */
    CSL_REG32_FINS(
        addr,
        RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DY0_ENBPD,
        (uint32_t)isEnable);
}
#endif

static void CSIRX_complexioSetLaneIntr( uint32_t *regVal, uint32_t lanePos, CSIRX_ComplexioLaneIntr *laneIntr)
{
    uint32_t pos0;

    pos0 = ((uint32_t)lanePos - 1U) * CSL_CSI2_COMPLEXIO_IRQENABLE_INTER_LANE_SHIFT_DISTANCE;

    *regVal |=   ( (uint32_t)laneIntr->isControlError                << (pos0 + CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRCONTROL1_SHIFT  ) )
               | ( (uint32_t)laneIntr->isEscapeEntryError            << (pos0 + CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRESC1_SHIFT      ) )
               | ( (uint32_t)laneIntr->isStartOfTransmissionError     << (pos0 + CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRSOTHS1_SHIFT    ) )
               | ( (uint32_t)laneIntr->isStartOfTransmissionSyncError << (pos0 + CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRSOTSYNCHS1_SHIFT) )
               | ( (uint32_t)laneIntr->isStateTransitionToULPM       << (pos0 + CSL_CSI2_COMPLEXIO1_IRQENABLE_STATEULPM1_SHIFT   ) )
               ;
}

static void CSIRX_complexioGetLanePendingIntr(uint32_t regVal, uint8_t lanePos, CSIRX_ComplexioLaneIntr *laneStatus)
{
    uint32_t pos0 = ((uint32_t)lanePos - 1U) *
                        CSL_CSI2_COMPLEXIO_IRQSTATUS_INTER_LANE_SHIFT_DISTANCE;

    laneStatus->isStateTransitionToULPM         = ( regVal >> (pos0 + CSL_CSI2_COMPLEXIO1_IRQSTATUS_STATEULPM1_SHIFT   ) ) & 0x1;
    laneStatus->isControlError                  = ( regVal >> (pos0 + CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRCONTROL1_SHIFT  ) ) & 0x1;
    laneStatus->isEscapeEntryError              = ( regVal >> (pos0 + CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRESC1_SHIFT      ) ) & 0x1;
    laneStatus->isStartOfTransmissionSyncError  = ( regVal >> (pos0 + CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRSOTSYNCHS1_SHIFT) ) & 0x1;
    laneStatus->isStartOfTransmissionError      = ( regVal >> (pos0 + CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRSOTHS1_SHIFT    ) ) & 0x1;
}

static void CSIRX_complexioSetLanePendingIntrNone(CSIRX_ComplexioLaneIntr *laneStatus)
{
    laneStatus->isStateTransitionToULPM         = false;
    laneStatus->isControlError                  = false;
    laneStatus->isEscapeEntryError              = false;
    laneStatus->isStartOfTransmissionSyncError  = false;
    laneStatus->isStartOfTransmissionError      = false;
}

int32_t CSIRX_complexioSetConfig (CSIRX_Handle handle, CSIRX_ComplexioConfig *complexioConfig)
{
    int32_t status = SystemP_SUCCESS;
    CSIRX_Config *config = (CSIRX_Config *)handle;

    if(    config == NULL
        || config->object == NULL
        || config->hwAttrs == NULL
        || config->hwAttrs->csirxRegs == 0
        || config->hwAttrs->rcssCtrlRegs == 0
        || complexioConfig == NULL
      )
    {
        status = SystemP_FAILURE;
    }
    else
    {
        uint32_t i, j, lanePos, notUsedCount, laneEnable, regVal;

        /* various error checks */

        if (   ( complexioConfig->lanesConfig.clockLane.position == CSIRX_LANE_POSITION_LANE_NOT_USED )
            || ( complexioConfig->lanesConfig.clockLane.position > (CSIRX_DATA_LANES_MAX+1) )
           )
        {
            status = SystemP_FAILURE;
        }

        /* Data lane should be in range and at least one data lane should be connected */
        notUsedCount = 0;
        for(i = 0; i < CSIRX_DATA_LANES_MAX; i++)
        {
            lanePos = complexioConfig->lanesConfig.dataLane[i].position;
            if (lanePos > (CSIRX_DATA_LANES_MAX+1) )
            {
                status = SystemP_FAILURE;
            }
            if (lanePos == CSIRX_LANE_POSITION_LANE_NOT_USED)
            {
                notUsedCount++;
            }
        }
        if(notUsedCount == CSIRX_DATA_LANES_MAX)
        {
            status = SystemP_FAILURE;
        }

        /* Compare clock lanes with all other data lanes */
        for(i = 0; i < CSIRX_DATA_LANES_MAX; i++)
        {
            lanePos = complexioConfig->lanesConfig.dataLane[i].position;
            if(    (lanePos != CSIRX_LANE_POSITION_LANE_NOT_USED)
                && (complexioConfig->lanesConfig.clockLane.position == lanePos))
            {
                status = SystemP_FAILURE;
            }
        }

        /* Compare data lanes among themselves */
        for(i = 0; i < CSIRX_DATA_LANES_MAX; i++)
        {
            for(j = (i + 1U); j < CSIRX_DATA_LANES_MAX; j++)
            {
                if(  ( complexioConfig->lanesConfig.dataLane[i].position != CSIRX_LANE_POSITION_LANE_NOT_USED)
                  && ( complexioConfig->lanesConfig.dataLane[j].position != CSIRX_LANE_POSITION_LANE_NOT_USED)
                  && ( complexioConfig->lanesConfig.dataLane[i].position == complexioConfig->lanesConfig.dataLane[j].position)
                  )
                {
                    status = SystemP_FAILURE;
                }
            }
        }

        if(status == SystemP_SUCCESS)
        {
            CSL_csirxRegs *regs = (CSL_csirxRegs *) config->hwAttrs->csirxRegs;

            /* RSS_CTRL lane config and lane enable */
            laneEnable = 0U;
            if (complexioConfig->lanesConfig.clockLane.position != CSIRX_LANE_POSITION_LANE_NOT_USED)
            {
                laneEnable |= ((uint32_t)1U <<
                            ((uint32_t)complexioConfig->lanesConfig.clockLane.position - 1U));
            }
            for(i = 0; i < CSIRX_DATA_LANES_MAX; i++)
            {
                lanePos = complexioConfig->lanesConfig.dataLane[i].position;
                if (lanePos != CSIRX_LANE_POSITION_LANE_NOT_USED)
                {
                    laneEnable |= ((uint32_t)1U << ((uint32_t)lanePos - 1U));
                }
            }

            #if defined (SOC_AM273X)
            {
                CSL_rcss_ctrlRegs *rcssCtrlRegs = (CSL_rcss_ctrlRegs *) config->hwAttrs->rcssCtrlRegs;

                /* lane config */
                for(lanePos = 0; lanePos < (CSIRX_DATA_LANES_MAX+1); lanePos++)
                {
                    if(config->hwAttrs->hwInstId==0)
                    {
                        CSIRX_complexioSetPullUpPullDown(
                                            (uint32_t)&rcssCtrlRegs->RCSS_CSI2A_LANE0_CFG,
                                            lanePos,
                                            ( (laneEnable >> (uint32_t)lanePos) & 1U )
                                            );
                    }
                    else
                    {
                        CSIRX_complexioSetPullUpPullDown(
                                            (uint32_t)&rcssCtrlRegs->RCSS_CSI2B_LANE0_CFG,
                                            lanePos,
                                            ( (laneEnable >> (uint32_t)lanePos) & 1U )
                                            );
                    }
                }

                /* lane enable */
                if(config->hwAttrs->hwInstId==0)
                {
                    CSL_REG32_FINS(&rcssCtrlRegs->RCSS_CSI2A_CFG,
                                RCSS_CTRL_RCSS_CSI2A_CFG_RCSS_CSI2A_CFG_LANE_ENABLE,
                                (uint32_t)laneEnable);
                }
                else
                {
                    CSL_REG32_FINS(&rcssCtrlRegs->RCSS_CSI2B_CFG,
                                RCSS_CTRL_RCSS_CSI2B_CFG_RCSS_CSI2B_CFG_LANE_ENABLE,
                                (uint32_t)laneEnable);
                }
            }
            #endif

            #if defined (SOC_AWR294X)
            {
                CSL_rss_ctrlRegs *rcssCtrlRegs = (CSL_rss_ctrlRegs *) config->hwAttrs->rcssCtrlRegs;

                /* lane config */
                for(lanePos = 0; lanePos < (CSIRX_DATA_LANES_MAX+1); lanePos++)
                {
                    CSIRX_complexioSetPullUpPullDown(
                                        (uint32_t)&rcssCtrlRegs->RSS_CSI2A_LANE0_CFG,
                                        lanePos,
                                        ( (laneEnable >> (uint32_t)lanePos) & 1U ));
                }

                /* lane enable */
                CSL_REG32_FINS(&rcssCtrlRegs->RSS_CSI2A_CFG,
                            RSS_CTRL_RSS_CSI2A_CFG_RSS_CSI2A_CFG_LANE_ENABLE,
                            (uint32_t)laneEnable);
            }
            #endif

            /* power auto */
            CSL_REG32_FINS(& regs->COMPLEXIO_CFG1, CSI2_COMPLEXIO_CFG1_PWR_AUTO,
                        (uint32_t) complexioConfig->isPowerAuto);

            /* lane config::clock lane */
            CSL_REG32_FINS(& regs->COMPLEXIO_CFG1, CSI2_COMPLEXIO_CFG1_CLOCK_POSITION,
                        (uint32_t)complexioConfig->lanesConfig.clockLane.position);
            CSL_REG32_FINS(& regs->COMPLEXIO_CFG1, CSI2_COMPLEXIO_CFG1_CLOCK_POL,
                        (uint32_t)complexioConfig->lanesConfig.clockLane.polarity);

            /* lane config::data lanes */
            CSL_REG32_FINS(& regs->COMPLEXIO_CFG1, CSI2_COMPLEXIO_CFG1_DATA1_POSITION,
                        (uint32_t)complexioConfig->lanesConfig.dataLane[0].position);
            CSL_REG32_FINS(& regs->COMPLEXIO_CFG1, CSI2_COMPLEXIO_CFG1_DATA1_POL,
                        (uint32_t)complexioConfig->lanesConfig.dataLane[0].polarity);
            CSL_REG32_FINS(& regs->COMPLEXIO_CFG1, CSI2_COMPLEXIO_CFG1_DATA2_POSITION,
                        (uint32_t)complexioConfig->lanesConfig.dataLane[1].position);
            CSL_REG32_FINS(& regs->COMPLEXIO_CFG1, CSI2_COMPLEXIO_CFG1_DATA2_POL,
                        (uint32_t)complexioConfig->lanesConfig.dataLane[1].polarity);
            CSL_REG32_FINS(& regs->COMPLEXIO_CFG1, CSI2_COMPLEXIO_CFG1_DATA3_POSITION,
                        (uint32_t)complexioConfig->lanesConfig.dataLane[2].position);
            CSL_REG32_FINS(& regs->COMPLEXIO_CFG1, CSI2_COMPLEXIO_CFG1_DATA3_POL,
                        (uint32_t)complexioConfig->lanesConfig.dataLane[2].polarity);
            CSL_REG32_FINS(& regs->COMPLEXIO_CFG1, CSI2_COMPLEXIO_CFG1_DATA4_POSITION,
                        (uint32_t)complexioConfig->lanesConfig.dataLane[3].position);
            CSL_REG32_FINS(& regs->COMPLEXIO_CFG1, CSI2_COMPLEXIO_CFG1_DATA4_POL,
                        (uint32_t)complexioConfig->lanesConfig.dataLane[3].polarity);

            /* interrupts */
            regVal =    ((uint32_t) complexioConfig->enableIntr.isAllLanesEnterULPM << CSL_CSI2_COMPLEXIO1_IRQENABLE_STATEALLULPMENTER_SHIFT)
                      | ((uint32_t) complexioConfig->enableIntr.isAllLanesExitULPM << CSL_CSI2_COMPLEXIO1_IRQENABLE_STATEALLULPMEXIT_SHIFT);

            /* clock lane */
            if (complexioConfig->lanesConfig.clockLane.position != CSIRX_LANE_POSITION_LANE_NOT_USED)
            {
                CSIRX_complexioSetLaneIntr(&regVal,
                                            complexioConfig->lanesConfig.clockLane.position,
                                            &complexioConfig->enableIntr.clockLane
                                        );
            }

            /* data lanes */
            for(i = 0; i < CSIRX_DATA_LANES_MAX; i++)
            {
                lanePos = complexioConfig->lanesConfig.dataLane[i].position;
                if (lanePos != CSIRX_LANE_POSITION_LANE_NOT_USED)
                {
                    CSIRX_complexioSetLaneIntr(&regVal,
                                                lanePos,
                                                &complexioConfig->enableIntr.dataLane[i]
                                            );
                }
            }

            /* write the IRQ reg */
            CSL_REG32_WR( &regs->COMPLEXIO1_IRQENABLE, regVal);
        }
    }
    return status;
}

int32_t CSIRX_complexioClearAllIntr (CSIRX_Handle handle)
{
    int32_t status = SystemP_SUCCESS;
    CSIRX_Config *config = (CSIRX_Config *)handle;

    if(    config == NULL
        || config->object == NULL
        || config->hwAttrs == NULL
        || config->hwAttrs->csirxRegs == 0
      )
    {
        status = SystemP_FAILURE;
    }
    else
    {
        CSL_csirxRegs *regs = (CSL_csirxRegs *) config->hwAttrs->csirxRegs;

        regs->COMPLEXIO1_IRQSTATUS = regs->COMPLEXIO1_IRQSTATUS;
    }
    return status;
}

int32_t CSIRX_complexioGetPendingIntr (CSIRX_Handle handle, CSIRX_ComplexioLanesIntr *intrStatus)
{
    int32_t status = SystemP_SUCCESS;
    CSIRX_Config *config = (CSIRX_Config *)handle;

    if(    config == NULL
        || config->object == NULL
        || config->hwAttrs == NULL
        || config->hwAttrs->csirxRegs == 0
        || intrStatus == NULL
      )
    {
        status = SystemP_FAILURE;
    }
    else
    {
        CSL_csirxRegs *regs = (CSL_csirxRegs *) config->hwAttrs->csirxRegs;
        uint32_t regVal;
        uint8_t lanePos;

        regVal = regs->COMPLEXIO1_IRQSTATUS;

        intrStatus->isAllLanesEnterULPM = ( regVal >> 25 ) & 0x1;
        intrStatus->isAllLanesExitULPM = ( regVal >> 26 ) & 0x1;

        /* clock lane */
        lanePos = (uint8_t)CSL_REG32_FEXT(&regs->COMPLEXIO_CFG1, CSI2_COMPLEXIO_CFG1_CLOCK_POSITION);
        if (lanePos != CSIRX_LANE_POSITION_LANE_NOT_USED)
        {
            CSIRX_complexioGetLanePendingIntr(regVal, lanePos, &intrStatus->clockLane);
        }
        else
        {
            CSIRX_complexioSetLanePendingIntrNone(&intrStatus->clockLane);
        }

        /* Data lane 1 */
        lanePos = (uint8_t)CSL_REG32_FEXT(&regs->COMPLEXIO_CFG1, CSI2_COMPLEXIO_CFG1_DATA1_POSITION);
        if (lanePos != CSIRX_LANE_POSITION_LANE_NOT_USED)
        {
            CSIRX_complexioGetLanePendingIntr(regVal, lanePos, &intrStatus->dataLane[0]);
        }
        else
        {
            CSIRX_complexioSetLanePendingIntrNone(&intrStatus->dataLane[0]);
        }

        /* Data lane 2 */
        lanePos = (uint8_t)CSL_REG32_FEXT(&regs->COMPLEXIO_CFG1, CSI2_COMPLEXIO_CFG1_DATA2_POSITION);
        if (lanePos != CSIRX_LANE_POSITION_LANE_NOT_USED)
        {
            CSIRX_complexioGetLanePendingIntr(regVal, lanePos, &intrStatus->dataLane[1]);
        }
        else
        {
            CSIRX_complexioSetLanePendingIntrNone(&intrStatus->dataLane[1]);
        }

        /* Data lane 3 */
        lanePos = (uint8_t)CSL_REG32_FEXT(&regs->COMPLEXIO_CFG1, CSI2_COMPLEXIO_CFG1_DATA3_POSITION);
        if (lanePos != CSIRX_LANE_POSITION_LANE_NOT_USED)
        {
            CSIRX_complexioGetLanePendingIntr(regVal, lanePos, &intrStatus->dataLane[2]);
        }
        else
        {
            CSIRX_complexioSetLanePendingIntrNone(&intrStatus->dataLane[2]);
        }

        /* Data lane 4 */
        lanePos = (uint8_t)CSL_REG32_FEXT(&regs->COMPLEXIO_CFG1, CSI2_COMPLEXIO_CFG1_DATA4_POSITION);
        if (lanePos != CSIRX_LANE_POSITION_LANE_NOT_USED)
        {
            CSIRX_complexioGetLanePendingIntr(regVal, lanePos, &intrStatus->dataLane[3]);
        }
        else
        {
            CSIRX_complexioSetLanePendingIntrNone(&intrStatus->dataLane[3]);
        }
    }
    return status;
}

int32_t CSIRX_complexioGetPowerStatus (CSIRX_Handle handle, uint8_t *powerStatus)
{
    int32_t status = SystemP_SUCCESS;
    CSIRX_Config *config = (CSIRX_Config *)handle;

    if(    config == NULL
        || config->object == NULL
        || config->hwAttrs == NULL
        || config->hwAttrs->csirxRegs == 0
        || powerStatus == NULL
      )
    {
        status = SystemP_FAILURE;
    }
    else
    {
        CSL_csirxRegs *regs = (CSL_csirxRegs *) config->hwAttrs->csirxRegs;

        *powerStatus = (uint8_t)CSL_REG32_FEXT(& regs->COMPLEXIO_CFG1, CSI2_COMPLEXIO_CFG1_PWR_STATUS);
    }
    return status;
}

int32_t CSIRX_complexioSetPowerCommand (CSIRX_Handle handle, uint8_t powerCommand)
{
    int32_t status = SystemP_SUCCESS;
    CSIRX_Config *config = (CSIRX_Config *)handle;

    if(    config == NULL
        || config->object == NULL
        || config->hwAttrs == NULL
        || config->hwAttrs->csirxRegs == 0
      )
    {
        status = SystemP_FAILURE;
    }
    else
    {
        CSL_csirxRegs *regs = (CSL_csirxRegs *) config->hwAttrs->csirxRegs;

        CSL_REG32_FINS(& regs->COMPLEXIO_CFG1, CSI2_COMPLEXIO_CFG1_PWR_CMD, (uint32_t)powerCommand);
    }
    return status;
}

int32_t CSIRX_complexioDeassertReset (CSIRX_Handle handle)
{
    int32_t status = SystemP_SUCCESS;
    CSIRX_Config *config = (CSIRX_Config *)handle;

    if(    config == NULL
        || config->object == NULL
        || config->hwAttrs == NULL
        || config->hwAttrs->csirxRegs == 0
      )
    {
        status = SystemP_FAILURE;
    }
    else
    {
        CSL_csirxRegs *regs = (CSL_csirxRegs *) config->hwAttrs->csirxRegs;

        CSL_REG32_FINS(& regs->COMPLEXIO_CFG1, CSI2_COMPLEXIO_CFG1_RESET_CTRL, 1U);
    }
    return status;
}

int32_t CSIRX_complexioAssertForceRxModeOn (CSIRX_Handle handle)
{
    int32_t status = SystemP_SUCCESS;
    CSIRX_Config *config = (CSIRX_Config *)handle;

    if(    config == NULL
        || config->object == NULL
        || config->hwAttrs == NULL
        || config->hwAttrs->csirxRegs == 0
      )
    {
        status = SystemP_FAILURE;
    }
    else
    {
        CSL_csirxRegs *regs = (CSL_csirxRegs *) config->hwAttrs->csirxRegs;

        CSL_REG32_FINS(& regs->TIMING, CSI2_TIMING_FORCE_RX_MODE_IO1, 1U );
    }
    return status;
}

int32_t CSIRX_complexioDeassertForceRxModeOn (CSIRX_Handle handle)
{
    int32_t status = SystemP_SUCCESS;
    CSIRX_Config *config = (CSIRX_Config *)handle;

    if(    config == NULL
        || config->object == NULL
        || config->hwAttrs == NULL
        || config->hwAttrs->csirxRegs == 0
      )
    {
        status = SystemP_FAILURE;
    }
    else
    {
        CSL_csirxRegs *regs = (CSL_csirxRegs *) config->hwAttrs->csirxRegs;

        CSL_REG32_FINS(& regs->TIMING, CSI2_TIMING_FORCE_RX_MODE_IO1, 0U );
    }
    return status;
}

int32_t CSIRX_complexioIsResetDone (CSIRX_Handle handle, bool *isDone)
{
    int32_t status = SystemP_SUCCESS;
    CSIRX_Config *config = (CSIRX_Config *)handle;

    if(    config == NULL
        || config->object == NULL
        || config->hwAttrs == NULL
        || config->hwAttrs->csirxRegs == 0
        || isDone == NULL
      )
    {
        status = SystemP_FAILURE;
    }
    else
    {
        CSL_csirxRegs *regs = (CSL_csirxRegs *) config->hwAttrs->csirxRegs;

        *isDone = (bool)CSL_REG32_FEXT(& regs->COMPLEXIO_CFG1, CSI2_COMPLEXIO_CFG1_RESET_DONE);
    }
    return status;
}

int32_t CSIRX_complexioIsDeassertForceRxModeOn (CSIRX_Handle handle, bool *isDeasserted)
{
    int32_t status = SystemP_SUCCESS;
    CSIRX_Config *config = (CSIRX_Config *)handle;

    if(    config == NULL
        || config->object == NULL
        || config->hwAttrs == NULL
        || config->hwAttrs->csirxRegs == 0
        || isDeasserted == NULL
      )
    {
        status = SystemP_FAILURE;
    }
    else
    {
        CSL_csirxRegs *regs = (CSL_csirxRegs *) config->hwAttrs->csirxRegs;

        *isDeasserted = (bool)( CSL_REG32_FEXT(& regs->TIMING, CSI2_TIMING_FORCE_RX_MODE_IO1) == 0U );
    }
    return status;
}

static void CSIRX_commonIsr(void *args)
{
    CSIRX_Config *config = (CSIRX_Config *) args;

    if(    config == NULL
        || config->object == NULL
        || config->hwAttrs == NULL
        || config->hwAttrs->csirxRegs == 0
      )
    {
        /* invalid arguments, do nothing */
    }
    else
    {
        CSIRX_CommonIntrCallbacks *intrCallbacks = &config->object->intrCallbacks;
        CSIRX_CommonIntr intrStatus;

        CSIRX_commonGetPendingIntr( config, &intrStatus);
        CSIRX_commonClearAllIntr( config );

        if(intrCallbacks->commonCallback != NULL)
        {
            intrCallbacks->commonCallback( config, intrCallbacks->commonCallbackArgs, &intrStatus);
        }
    }
}

static void CSIRX_commonCombinedEndOfLineIsr(void *args)
{
    CSIRX_Config *config = (CSIRX_Config *) args;

    if(    config == NULL
        || config->object == NULL
      )
    {
        /* invalid arguments, do nothing */
    }
    else
    {
        CSIRX_CommonIntrCallbacks *intrCallbacks = &config->object->intrCallbacks;

        if(intrCallbacks->combinedEndOfLineCallback != NULL)
        {
            intrCallbacks->combinedEndOfLineCallback( config, intrCallbacks->combinedEndOfLineCallbackArgs);
        }
    }
}

static void CSIRX_commonCombinedEndOfFrameIsr(void *args)
{
    CSIRX_Config *config = (CSIRX_Config *) args;

    if(    config == NULL
        || config->object == NULL
      )
    {
        /* invalid arguments, do nothing */
    }
    else
    {
        CSIRX_CommonIntrCallbacks *intrCallbacks = &config->object->intrCallbacks;

        if(intrCallbacks->combinedEndOfFrameCallback != NULL)
        {
            intrCallbacks->combinedEndOfFrameCallback( config, intrCallbacks->combinedEndOfFrameCallbackArgs);
        }
    }
}

static void CSIRX_commonStartOfFrameIntr0Isr(void *args)
{
    CSIRX_Config *config = (CSIRX_Config *) args;

    if(    config == NULL
        || config->object == NULL
      )
    {
        /* invalid arguments, do nothing */
    }
    else
    {
        CSIRX_CommonIntrCallbacks *intrCallbacks = &config->object->intrCallbacks;

        if(intrCallbacks->startOfFrameIntr0Callback != NULL)
        {
            intrCallbacks->startOfFrameIntr0Callback( config,
                    intrCallbacks->startOfFrameIntr0CallbackArgs,
                    config->object->startOfFrameIntr0ContextId
                );
        }
    }
}

static void CSIRX_commonStartOfFrameIntr1Isr(void *args)
{
    CSIRX_Config *config = (CSIRX_Config *) args;

    if(    config == NULL
        || config->object == NULL
      )
    {
        /* invalid arguments, do nothing */
    }
    else
    {
        CSIRX_CommonIntrCallbacks *intrCallbacks = &config->object->intrCallbacks;

        if(intrCallbacks->startOfFrameIntr1Callback != NULL)
        {
            intrCallbacks->startOfFrameIntr1Callback( config,
                    intrCallbacks->startOfFrameIntr1CallbackArgs,
                    config->object->startOfFrameIntr1ContextId
                );
        }
    }
}

static int32_t CSIRX_commonRegisterIntr(CSIRX_Config *config)
{
    int32_t status = SystemP_SUCCESS;
    HwiP_Params hwiParams;
    CSIRX_CommonIntrCallbacks *intrCallbacks = &config->object->intrCallbacks;
    const CSIRX_HwAttrs *hwAttrs = config->hwAttrs;

    if(     hwAttrs->commonIntNum != CSIRX_INTERRUPT_NOT_CONNECTED_ID
        &&  intrCallbacks->commonCallback != NULL
      )
    {
        HwiP_Params_init(&hwiParams);

        hwiParams.intNum   = hwAttrs->commonIntNum;
        hwiParams.callback = CSIRX_commonIsr;
        hwiParams.args     = config;

        status |= HwiP_construct(&config->object->commonIntrObj, &hwiParams);
    }

    if(     hwAttrs->combinedEndOfLineIntNum != CSIRX_INTERRUPT_NOT_CONNECTED_ID
        &&  intrCallbacks->combinedEndOfLineCallback != NULL
      )
    {
        HwiP_Params_init(&hwiParams);

        hwiParams.intNum   = hwAttrs->combinedEndOfLineIntNum;
        hwiParams.callback = CSIRX_commonCombinedEndOfLineIsr;
        hwiParams.args     = config;

        status |= HwiP_construct(&config->object->combinedEndOfLineIntrObj, &hwiParams);
    }

    if(     hwAttrs->combinedEndOfFrameIntNum != CSIRX_INTERRUPT_NOT_CONNECTED_ID
        &&  intrCallbacks->combinedEndOfFrameCallback != NULL
      )
    {
        HwiP_Params_init(&hwiParams);

        hwiParams.intNum   = hwAttrs->combinedEndOfFrameIntNum;
        hwiParams.callback = CSIRX_commonCombinedEndOfFrameIsr;
        hwiParams.args     = config;

        status |= HwiP_construct(&config->object->combinedEndOfFrameIntrObj, &hwiParams);
    }

    if(     hwAttrs->startOfFrameIntr0IntNum != CSIRX_INTERRUPT_NOT_CONNECTED_ID
        &&  intrCallbacks->startOfFrameIntr0Callback != NULL
      )
    {
        HwiP_Params_init(&hwiParams);

        hwiParams.intNum   = hwAttrs->startOfFrameIntr0IntNum;
        hwiParams.callback = CSIRX_commonStartOfFrameIntr0Isr;
        hwiParams.args     = config;

        status |= HwiP_construct(&config->object->startOfFrameIntr0IntrObj, &hwiParams);
    }

    if(     hwAttrs->startOfFrameIntr1IntNum != CSIRX_INTERRUPT_NOT_CONNECTED_ID
        &&  intrCallbacks->startOfFrameIntr1Callback != NULL
      )
    {
        HwiP_Params_init(&hwiParams);

        hwiParams.intNum   = hwAttrs->startOfFrameIntr1IntNum;
        hwiParams.callback = CSIRX_commonStartOfFrameIntr1Isr;
        hwiParams.args     = config;

        status |= HwiP_construct(&config->object->startOfFrameIntr1IntrObj, &hwiParams);
    }

    return status;
}

static int32_t CSIRX_commonUnRegisterIntr(CSIRX_Config *config)
{
    int32_t status = SystemP_SUCCESS;
    CSIRX_CommonIntrCallbacks *intrCallbacks = &config->object->intrCallbacks;
    const CSIRX_HwAttrs *hwAttrs = config->hwAttrs;

    if(     hwAttrs->commonIntNum != CSIRX_INTERRUPT_NOT_CONNECTED_ID
        &&  intrCallbacks->commonCallback != NULL
      )
    {
        HwiP_destruct(&config->object->commonIntrObj);
    }

    if(     hwAttrs->combinedEndOfLineIntNum != CSIRX_INTERRUPT_NOT_CONNECTED_ID
        &&  intrCallbacks->combinedEndOfLineCallback != NULL
      )
    {
        HwiP_destruct(&config->object->combinedEndOfLineIntrObj);
    }

    if(     hwAttrs->combinedEndOfFrameIntNum != CSIRX_INTERRUPT_NOT_CONNECTED_ID
        &&  intrCallbacks->combinedEndOfFrameCallback != NULL
      )
    {
        HwiP_destruct(&config->object->combinedEndOfFrameIntrObj);
    }

    if(     hwAttrs->startOfFrameIntr0IntNum != CSIRX_INTERRUPT_NOT_CONNECTED_ID
        &&  intrCallbacks->startOfFrameIntr0Callback != NULL
      )
    {
        HwiP_destruct(&config->object->startOfFrameIntr0IntrObj);
    }

    if(     hwAttrs->startOfFrameIntr1IntNum != CSIRX_INTERRUPT_NOT_CONNECTED_ID
        &&  intrCallbacks->startOfFrameIntr1Callback != NULL
      )
    {
        HwiP_destruct(&config->object->startOfFrameIntr1IntrObj);
    }

    return status;
}

int32_t CSIRX_commonSetConfig (CSIRX_Handle handle, CSIRX_CommonConfig *commonConfig)
{
    int32_t status = SystemP_SUCCESS;
    CSIRX_Config *config = (CSIRX_Config *)handle;

    if(    config == NULL
        || config->object == NULL
        || config->hwAttrs == NULL
        || config->hwAttrs->csirxRegs == 0
        || commonConfig == NULL
      )
    {
        status = SystemP_FAILURE;
    }
    if(status == SystemP_SUCCESS)
    {
        if( (commonConfig->isBurstSizeExpand == (bool)true)
               &&
            (
                ! ( (commonConfig->burstSize == CSIRX_BURST_SIZE_1X64) ||
                    (commonConfig->burstSize == CSIRX_BURST_SIZE_2X64) ||
                    (commonConfig->burstSize == CSIRX_BURST_SIZE_4X64) ||
                    (commonConfig->burstSize == CSIRX_BURST_SIZE_8X64)
                  )
            )
          )
        {
            status = SystemP_FAILURE;
        }
        if(! ( (commonConfig->endianness == CSIRX_ENDIANNESS_NATIVE_MIPI_CSI2 ) ||
               (commonConfig->endianness == CSIRX_ENDIANNESS_LITTLE_ENDIAN )
             )
          )
        {
            status = SystemP_FAILURE;
        }
        if(     commonConfig->startOfFrameIntr0ContextId >= CSIRX_CONTEXTS_MAX
            ||  commonConfig->startOfFrameIntr1ContextId >= CSIRX_CONTEXTS_MAX
            ||  commonConfig->endOfFrameIntr0ContextId >= CSIRX_CONTEXTS_MAX
            ||  commonConfig->endOfFrameIntr1ContextId >= CSIRX_CONTEXTS_MAX
            )
        {
            status = SystemP_FAILURE;
        }
    }
    if(status == SystemP_SUCCESS)
    {
        CSL_csirxRegs *regs = (CSL_csirxRegs *) config->hwAttrs->csirxRegs;

        uint32_t regSysconfig, regCtrl, regTiming, regIrqEnable, regRcssCsiCfg;
        uint32_t temp, stopCount, interconnectClockCount, X16, X4, contextId;
        uint32_t regRcssCsiCfgAddr = 0;

        #if defined (SOC_AM273X)
        {
            CSL_rcss_ctrlRegs *rcssCtrlRegs = (CSL_rcss_ctrlRegs *)config->hwAttrs->rcssCtrlRegs;

            if(config->hwAttrs->hwInstId==0)
            {
                regRcssCsiCfgAddr = (uint32_t)&rcssCtrlRegs->RCSS_CSI2A_CFG;
            }
            else
            {
                regRcssCsiCfgAddr = (uint32_t)&rcssCtrlRegs->RCSS_CSI2B_CFG;
            }
        }
        #endif
        #if defined (SOC_AWR294X)
        {
            CSL_rss_ctrlRegs *rcssCtrlRegs = (CSL_rss_ctrlRegs *)config->hwAttrs->rcssCtrlRegs;
            regRcssCsiCfgAddr = (uint32_t)&rcssCtrlRegs->RSS_CSI2A_CFG;
        }
        #endif

        regSysconfig  = regs->SYSCONFIG;
        regCtrl       = regs->CTRL;
        regTiming     = regs->TIMING;
        regIrqEnable  = regs->IRQENABLE;
        regRcssCsiCfg = CSL_REG32_RD(regRcssCsiCfgAddr);

        /* Configure fields into the local copies of registers */
        CSL_FINS(regSysconfig, CSI2_SYSCONFIG_AUTO_IDLE, (uint32_t)commonConfig->isOcpAutoIdle);

        /* TODO: do we need a device based #ifdef here? Will know after silicon team
        * investigates more.
        *
        * Although MSTANDBY_MODE is marked as a reserved field, its reset default value
        * of 0 is not functional in the in the silicon, force it to the mode
        * which DV tests have been using i.e 2 = smart standby */
        CSL_FINS(regSysconfig, CSI2_SYSCONFIG_MSTANDBY_MODE, (uint32_t)2U);

        CSL_FINS(regCtrl, CSI2_CTRL_BURST_SIZE_EXPAND, (uint32_t)commonConfig->isBurstSizeExpand);
        CSL_FINS(regCtrl, CSI2_CTRL_BURST_SIZE, (uint32_t)commonConfig->burstSize);
        CSL_FINS(regCtrl, CSI2_CTRL_NON_POSTED_WRITE, (uint32_t)commonConfig->isNonPostedWrites);
        CSL_FINS(regCtrl, CSI2_CTRL_ENDIANNESS, (uint32_t)commonConfig->endianness);
        CSL_FINS(regCtrl, CSI2_CTRL_FRAME, (uint32_t)commonConfig->isSoftStoppingOnInterfaceDisable);
        CSL_FINS(regCtrl, CSI2_CTRL_ECC_EN, (uint32_t)commonConfig->isHeaderErrorCheckEnabled);

        #if defined (SOC_AM273X)
        CSL_FINS(regRcssCsiCfg,
                RCSS_CTRL_RCSS_CSI2A_CFG_RCSS_CSI2A_CFG_SIGN_EXT_EN,
                (uint32_t)commonConfig->isSignExtensionEnabled);

        /* End of Frame interrupt 0 */
        CSL_FINS(regRcssCsiCfg,
                RCSS_CTRL_RCSS_CSI2A_CFG_RCSS_CSI2A_CFG_EOF_INTR0_SEL,
                (uint32_t)commonConfig->endOfFrameIntr0ContextId);
        config->object->endOfFrameIntr0ContextId = commonConfig->endOfFrameIntr0ContextId;

        /* End of Frame interrupt 1 */
        CSL_FINS(regRcssCsiCfg,
                RCSS_CTRL_RCSS_CSI2A_CFG_RCSS_CSI2A_CFG_EOF_INTR1_SEL,
                (uint32_t)commonConfig->endOfFrameIntr1ContextId);
        config->object->startOfFrameIntr1ContextId = commonConfig->startOfFrameIntr1ContextId;

        /* Start of Frame interrupt 0 */
        CSL_FINS(regRcssCsiCfg,
                RCSS_CTRL_RCSS_CSI2A_CFG_RCSS_CSI2A_CFG_SOF_INTR0_SEL,
                (uint32_t)commonConfig->startOfFrameIntr0ContextId);
        config->object->startOfFrameIntr0ContextId = commonConfig->startOfFrameIntr0ContextId;

        /* Start of Frame interrupt 1 */
        CSL_FINS(regRcssCsiCfg,
                RCSS_CTRL_RCSS_CSI2A_CFG_RCSS_CSI2A_CFG_SOF_INTR1_SEL,
                (uint32_t)commonConfig->startOfFrameIntr1ContextId);
        config->object->startOfFrameIntr1ContextId = commonConfig->startOfFrameIntr1ContextId;
        #endif
        #if defined (SOC_AWR294X)
        CSL_FINS(regRcssCsiCfg,
                RSS_CTRL_RSS_CSI2A_CFG_RSS_CSI2A_CFG_SIGN_EXT_EN,
                (uint32_t)commonConfig->isSignExtensionEnabled);

        /* End of Frame interrupt 0 */
        CSL_FINS(regRcssCsiCfg,
                RSS_CTRL_RSS_CSI2A_CFG_RSS_CSI2A_CFG_EOF_INTR0_SEL,
                (uint32_t)commonConfig->endOfFrameIntr0ContextId);
        config->object->endOfFrameIntr0ContextId = commonConfig->endOfFrameIntr0ContextId;

        /* End of Frame interrupt 1 */
        CSL_FINS(regRcssCsiCfg,
                RSS_CTRL_RSS_CSI2A_CFG_RSS_CSI2A_CFG_EOF_INTR1_SEL,
                (uint32_t)commonConfig->endOfFrameIntr1ContextId);
        config->object->startOfFrameIntr1ContextId = commonConfig->startOfFrameIntr1ContextId;

        /* Start of Frame interrupt 0 */
        CSL_FINS(regRcssCsiCfg,
                RSS_CTRL_RSS_CSI2A_CFG_RSS_CSI2A_CFG_SOF_INTR0_SEL,
                (uint32_t)commonConfig->startOfFrameIntr0ContextId);
        config->object->startOfFrameIntr0ContextId = commonConfig->startOfFrameIntr0ContextId;

        /* Start of Frame interrupt 1 */
        CSL_FINS(regRcssCsiCfg,
                RSS_CTRL_RSS_CSI2A_CFG_RSS_CSI2A_CFG_SOF_INTR1_SEL,
                (uint32_t)commonConfig->startOfFrameIntr1ContextId);
        config->object->startOfFrameIntr1ContextId = commonConfig->startOfFrameIntr1ContextId;
        #endif

        /* Stop state timeout related programming */
        temp = (float)commonConfig->stopStateFsmTimeoutInNanoSecs * (float)1e-9 *
                (float)config->hwAttrs->interconnectClockHz;

        /* check for overflow, error will be high in this case */
        if((temp + ((float)10.0 * (float)FLT_EPSILON)) > (float)(0xFFFFFFFFU))
        {
            interconnectClockCount = 0xFFFFFFFFU;
        }
        else
        {
            interconnectClockCount = (uint32_t) temp;
        }

        /* limit to max */
        if(   ( commonConfig->stopStateFsmTimeoutInNanoSecs == CSIRX_STOP_STATE_FSM_TIMEOUT_MAX)
           || ( interconnectClockCount > (16U * 4U * CSL_CSI2_TIMING_STOP_STATE_COUNTER_IO1_RESETVAL) )
           )
        {
            X16 = 1U;
            X4 = 1U;
            stopCount = CSL_CSI2_TIMING_STOP_STATE_COUNTER_IO1_RESETVAL;
        }
        else
        {
            if(interconnectClockCount <= CSL_CSI2_TIMING_STOP_STATE_COUNTER_IO1_RESETVAL)
            {
                X16 = 0U;
                X4 = 0U;
                stopCount = interconnectClockCount;
            }
            else if(interconnectClockCount <= (4U * CSL_CSI2_TIMING_STOP_STATE_COUNTER_IO1_RESETVAL))
            {
                X16 = 0U;
                X4 = 1U;
                stopCount = interconnectClockCount / 4U;
            }
            else /* interconnectClockCount <= (16U * 4U * CSL_CSI2_TIMING_STOP_STATE_COUNTER_IO1_RESETVAL)) */
            {
                X16 = 1U;
                X4 = 1U;
                stopCount = interconnectClockCount / (4U * 16U);
            }
        }

        CSL_FINS(regTiming, CSI2_TIMING_STOP_STATE_X16_IO1,     X16);
        CSL_FINS(regTiming, CSI2_TIMING_STOP_STATE_X4_IO1,      X4);
        CSL_FINS(regTiming, CSI2_TIMING_STOP_STATE_COUNTER_IO1, (uint32_t)stopCount);

        regIrqEnable |=
              ( (uint32_t)commonConfig->enableIntr.isComplexioError <<
                        CSL_CSI2_IRQENABLE_COMPLEXIO1_ERR_IRQ_SHIFT )
            | ( (uint32_t)commonConfig->enableIntr.isMoreThanOneBitShortPacketErrorCannotCorrect <<
                        CSL_CSI2_IRQENABLE_ECC_NO_CORRECTION_IRQ_SHIFT )
            | ( (uint32_t)commonConfig->enableIntr.isOneBitShortPacketErrorCorrect <<
                        CSL_CSI2_IRQENABLE_ECC_CORRECTION_IRQ_SHIFT )
            | ( (uint32_t)commonConfig->enableIntr.isFifoOverflow <<
                        CSL_CSI2_IRQENABLE_FIFO_OVF_IRQ_SHIFT )
            | ( (uint32_t)commonConfig->enableIntr.isOcpError <<
                        CSL_CSI2_IRQENABLE_OCP_ERR_IRQ_SHIFT )
            | ( (uint32_t)commonConfig->enableIntr.isGenericShortPacketReceive <<
                        CSL_CSI2_IRQENABLE_SHORT_PACKET_IRQ_SHIFT )
            ;

        for(contextId = 0; contextId < CSIRX_CONTEXTS_MAX; contextId++)
        {
            regIrqEnable |= ( (uint32_t)commonConfig->enableIntr.isContextIntr[contextId] <<
                                ( CSL_CSI2_IRQENABLE_CONTEXT0_SHIFT +
                                    ((uint32_t)contextId * CSL_CSI2_INTER_CONTEXT_IRQENABLE_SHIFT_DISTANCE)
                                )
                            );
        }

        /* Write the registers that were constructed in local memory */
        regs->SYSCONFIG = regSysconfig;
        regs->CTRL      = regCtrl;
        regs->TIMING    = regTiming;
        regs->IRQENABLE = regIrqEnable;
        CSL_REG32_WR(regRcssCsiCfgAddr, regRcssCsiCfg);

        config->object->intrCallbacks = commonConfig->intrCallbacks;

        /* register ISRs */
        status = CSIRX_commonRegisterIntr(config);
    }
    return status;
}

int32_t CSIRX_commonClearAllIntr (CSIRX_Handle handle)
{
    int32_t status = SystemP_SUCCESS;
    CSIRX_Config *config = (CSIRX_Config *)handle;

    if(    config == NULL
        || config->object == NULL
        || config->hwAttrs == NULL
        || config->hwAttrs->csirxRegs == 0
      )
    {
        status = SystemP_FAILURE;
    }
    else
    {
        CSL_csirxRegs *regs = (CSL_csirxRegs *) config->hwAttrs->csirxRegs;

        regs->IRQSTATUS = regs->IRQSTATUS;
    }
    return status;
}

int32_t CSIRX_commonGetPendingIntr (CSIRX_Handle handle, CSIRX_CommonIntr *intrStatus)
{
    int32_t status = SystemP_SUCCESS;
    CSIRX_Config *config = (CSIRX_Config *)handle;

    if(    config == NULL
        || config->object == NULL
        || config->hwAttrs == NULL
        || config->hwAttrs->csirxRegs == 0
        || intrStatus == NULL
      )
    {
        status = SystemP_FAILURE;
    }
    else
    {
        CSL_csirxRegs *regs = (CSL_csirxRegs *) config->hwAttrs->csirxRegs;
        uint32_t regVal = regs->IRQSTATUS;

        intrStatus->isOcpError                                    = ( regVal >> 14 ) & 0x1;
        intrStatus->isGenericShortPacketReceive                   = ( regVal >> 13 ) & 0x1;
        intrStatus->isOneBitShortPacketErrorCorrect               = ( regVal >> 12 ) & 0x1;
        intrStatus->isMoreThanOneBitShortPacketErrorCannotCorrect = ( regVal >> 11 ) & 0x1;
        intrStatus->isComplexioError                              = ( regVal >>  9 ) & 0x1;
        intrStatus->isFifoOverflow                                = ( regVal >>  8 ) & 0x1;
        intrStatus->isContextIntr[0]                              = ( regVal >>  0 ) & 0x1;
        intrStatus->isContextIntr[1]                              = ( regVal >>  1 ) & 0x1;
        intrStatus->isContextIntr[2]                              = ( regVal >>  2 ) & 0x1;
        intrStatus->isContextIntr[3]                              = ( regVal >>  3 ) & 0x1;
        intrStatus->isContextIntr[4]                              = ( regVal >>  4 ) & 0x1;
        intrStatus->isContextIntr[5]                              = ( regVal >>  5 ) & 0x1;
        intrStatus->isContextIntr[6]                              = ( regVal >>  6 ) & 0x1;
        intrStatus->isContextIntr[7]                              = ( regVal >>  7 ) & 0x1;
    }
    return status;
}

int32_t CSIRX_commonGetGenericShortPacket (CSIRX_Handle handle, uint32_t *shortPacket)
{
    int32_t status = SystemP_SUCCESS;
    CSIRX_Config *config = (CSIRX_Config *)handle;

    if(    config == NULL
        || config->object == NULL
        || config->hwAttrs == NULL
        || config->hwAttrs->csirxRegs == 0
        || shortPacket == NULL
      )
    {
        status = SystemP_FAILURE;
    }
    else
    {
        CSL_csirxRegs *regs = (CSL_csirxRegs *) config->hwAttrs->csirxRegs;

        *shortPacket = regs->SHORT_PACKET;
    }
    return status;
}

int32_t CSIRX_commonEnable (CSIRX_Handle handle)
{
    int32_t status = SystemP_SUCCESS;
    CSIRX_Config *config = (CSIRX_Config *)handle;

    if(    config == NULL
        || config->object == NULL
        || config->hwAttrs == NULL
        || config->hwAttrs->csirxRegs == 0
      )
    {
        status = SystemP_FAILURE;
    }
    else
    {
        CSL_csirxRegs *regs = (CSL_csirxRegs *) config->hwAttrs->csirxRegs;

        CSL_REG32_FINS(&regs->CTRL, CSI2_CTRL_IF_EN, 1U);
    }
    return status;
}

int32_t CSIRX_commonDisable (CSIRX_Handle handle)
{
    int32_t status = SystemP_SUCCESS;
    CSIRX_Config *config = (CSIRX_Config *)handle;

    if(    config == NULL
        || config->object == NULL
        || config->hwAttrs == NULL
        || config->hwAttrs->csirxRegs == 0
      )
    {
        status = SystemP_FAILURE;
    }
    else
    {
        CSL_csirxRegs *regs = (CSL_csirxRegs *) config->hwAttrs->csirxRegs;

        CSL_REG32_FINS(&regs->CTRL, CSI2_CTRL_IF_EN, 0U);
    }
    return status;
}

int32_t CSIRX_init()
{
    int32_t status = SystemP_SUCCESS;
    uint32_t i;

    for(i=0; i < gCsirxConfigNum; i++)
    {
        if( gCsirxConfig[i].object != NULL )
        {
            gCsirxConfig[i].object->isOpen = false;
        }
    }
    return status;
}

int32_t CSIRX_deinit()
{
    int32_t status = SystemP_SUCCESS;

    return status;
}

CSIRX_Handle CSIRX_open (uint32_t instanceId)
{
    CSIRX_Config *config = NULL;

    if( instanceId < gCsirxConfigNum )
    {
        config = &gCsirxConfig[instanceId];

        if(    config->object == NULL
            || config->hwAttrs == NULL
            || config->hwAttrs->csirxRegs == 0
            || config->hwAttrs->rcssCtrlRegs == 0
            || config->hwAttrs->hwInstId >= CSIRX_HW_INSTANCES_MAX
            || config->hwAttrs->ctrlClockHz == 0
            || config->object->isOpen == true
            )
        {
            /* something is not setup correctly or driver instance is open already */
            config = NULL;
        }
        if( config != NULL )
        {
            CSL_csirxRegs *regs = (CSL_csirxRegs *) config->hwAttrs->csirxRegs;
            uint32_t i;

            config->object->isOpen = true;

            /* disable all contexts and disable CSIRX interface itself */
            CSL_REG32_FINS(&regs->CTRL, CSI2_CTRL_IF_EN, 0U);
            for(i=0; i < CSIRX_CONTEXTS_MAX; i++)
            {
                CSL_REG32_FINS(&regs->ctx[i].CTX_CTRL1, CSI2_CTX0_CTRL1_CTX_EN, 0);
            }

            /* disable all interrupts */
            regs->COMPLEXIO1_IRQENABLE = 0;
            for(i=0; i < CSIRX_CONTEXTS_MAX; i++)
            {
                regs->ctx[i].CTX_IRQENABLE = 0;
            }
            regs->IRQENABLE = 0;

            /* clear all interrupts */
            regs->COMPLEXIO1_IRQSTATUS = 0xFFFFFFFFU;
            for(i=0; i < CSIRX_CONTEXTS_MAX; i++)
            {
                regs->ctx[i].CTX_IRQSTATUS = 0xFFFFFFFFU;
            }
            regs->IRQSTATUS = 0xFFFFFFFFU;

            /* clear SW state */
            for(i=0; i < CSIRX_CONTEXTS_MAX; i++)
            {
                config->object->context[i].pingAddress = 0;
                config->object->context[i].pongAddress = 0;
            }
        }
    }
    return (CSIRX_Handle)config;
}

int32_t CSIRX_close (CSIRX_Handle handle)
{
    int32_t status = SystemP_SUCCESS;
    CSIRX_Config *config = (CSIRX_Config *)handle;

    if(    config == NULL
        || config->object == NULL
        || config->hwAttrs == NULL
        || config->hwAttrs->csirxRegs == 0
      )
    {
        status = SystemP_FAILURE;
    }
    else
    {
        CSL_csirxRegs *regs = (CSL_csirxRegs *) config->hwAttrs->csirxRegs;
        uint32_t i;

        /* it is assumed that CSIRX is shutdown gracefully using the CSIRX APIs */
        config->object->isOpen = false;

        /* disable all interrupts */
        regs->COMPLEXIO1_IRQENABLE = 0;
        for(i=0; i < CSIRX_CONTEXTS_MAX; i++)
        {
            regs->ctx[i].CTX_IRQENABLE = 0;
        }
        regs->IRQENABLE = 0;

        /* clear all interrupts */
        regs->COMPLEXIO1_IRQSTATUS = 0xFFFFFFFFU;
        for(i=0; i < CSIRX_CONTEXTS_MAX; i++)
        {
            regs->ctx[i].CTX_IRQSTATUS = 0xFFFFFFFFU;
        }
        regs->IRQSTATUS = 0xFFFFFFFFU;

        /* disable all contexts and disable CSIRX interface itself */
        CSL_REG32_FINS(&regs->CTRL, CSI2_CTRL_IF_EN, 0U);
        for(i=0; i < CSIRX_CONTEXTS_MAX; i++)
        {
            CSL_REG32_FINS(&regs->ctx[i].CTX_CTRL1, CSI2_CTX0_CTRL1_CTX_EN, 0);
        }

        CSIRX_commonUnRegisterIntr(config);
    }
    return status;
}

int32_t CSIRX_getInfo (CSIRX_Handle handle, CSIRX_Info *info)
{
    int32_t status = SystemP_SUCCESS;
    CSIRX_Config *config = (CSIRX_Config *)handle;

    if(    config == NULL
        || config->object == NULL
        || config->hwAttrs == NULL
        || config->hwAttrs->csirxRegs == 0
        || info == NULL
      )
    {
        status = SystemP_FAILURE;
    }
    else
    {
        CSL_csirxRegs *regs = (CSL_csirxRegs *) config->hwAttrs->csirxRegs;

        info->revisionId  = CSL_REG32_FEXT(&regs->REVISION, CSI2_REVISION_REV);
        info->fifoDepth   = CSL_REG32_FEXT(&regs->GNQ, CSI2_GNQ_FIFODEPTH);
        info->numContexts = 1 << CSL_REG32_FEXT(&regs->GNQ, CSI2_GNQ_NBCONTEXTS);
    }
    return status;
}

int32_t CSIRX_reset (CSIRX_Handle handle)
{
    int32_t status = SystemP_SUCCESS;
    CSIRX_Config *config = (CSIRX_Config *)handle;

    if(    config == NULL
        || config->object == NULL
        || config->hwAttrs == NULL
        || config->hwAttrs->csirxRegs == 0
      )
    {
        status = SystemP_FAILURE;
    }
    else
    {
        CSL_csirxRegs *regs = (CSL_csirxRegs *) config->hwAttrs->csirxRegs;
        uint32_t resetAttempts, isResetDone;

        /* Issue soft reset and check for reset done-ness for a maximum of
         * 5 attempts */
        CSL_REG32_FINS( &regs->SYSCONFIG, CSI2_SYSCONFIG_SOFT_RESET, 1U);

        resetAttempts = 0;
        do
        {
            isResetDone = CSL_REG32_FEXT( &regs->SYSSTATUS, CSI2_SYSSTATUS_RESET_DONE);
            resetAttempts++;
        } while( (isResetDone == (uint32_t)0U) && (resetAttempts < 5U));

        /* attempted out, generate error code */
        if(isResetDone == (uint32_t)0U)
        {
            status = SystemP_FAILURE;
        }

        /* reset RCSS CTRL CSIRX config */
        #if defined (SOC_AM273X)
        {
            CSL_rcss_ctrlRegs *rcssCtrlRegs = (CSL_rcss_ctrlRegs *) config->hwAttrs->rcssCtrlRegs;

            if(config->hwAttrs->hwInstId==0)
            {
                rcssCtrlRegs->RCSS_CSI2A_CFG         = CSL_RCSS_CTRL_RCSS_CSI2A_CFG_RESETVAL;
                rcssCtrlRegs->RCSS_CSI2A_PARITY_CTRL = CSL_RCSS_CTRL_RCSS_CSI2A_PARITY_CTRL_RESETVAL;
                rcssCtrlRegs->RCSS_CSI2A_LANE0_CFG   = CSL_RCSS_CTRL_RCSS_CSI2A_LANE0_CFG_RESETVAL;
                rcssCtrlRegs->RCSS_CSI2A_LANE1_CFG   = CSL_RCSS_CTRL_RCSS_CSI2A_LANE1_CFG_RESETVAL;
                rcssCtrlRegs->RCSS_CSI2A_LANE2_CFG   = CSL_RCSS_CTRL_RCSS_CSI2A_LANE2_CFG_RESETVAL;
                rcssCtrlRegs->RCSS_CSI2A_LANE3_CFG   = CSL_RCSS_CTRL_RCSS_CSI2A_LANE3_CFG_RESETVAL;
                rcssCtrlRegs->RCSS_CSI2A_LANE4_CFG   = CSL_RCSS_CTRL_RCSS_CSI2A_LANE4_CFG_RESETVAL;
            }
            else
            {
                rcssCtrlRegs->RCSS_CSI2B_CFG         = CSL_RCSS_CTRL_RCSS_CSI2B_CFG_RESETVAL;
                rcssCtrlRegs->RCSS_CSI2B_PARITY_CTRL = CSL_RCSS_CTRL_RCSS_CSI2B_PARITY_CTRL_RESETVAL;
                rcssCtrlRegs->RCSS_CSI2B_LANE0_CFG   = CSL_RCSS_CTRL_RCSS_CSI2B_LANE0_CFG_RESETVAL;
                rcssCtrlRegs->RCSS_CSI2B_LANE1_CFG   = CSL_RCSS_CTRL_RCSS_CSI2B_LANE1_CFG_RESETVAL;
                rcssCtrlRegs->RCSS_CSI2B_LANE2_CFG   = CSL_RCSS_CTRL_RCSS_CSI2B_LANE2_CFG_RESETVAL;
                rcssCtrlRegs->RCSS_CSI2B_LANE3_CFG   = CSL_RCSS_CTRL_RCSS_CSI2B_LANE3_CFG_RESETVAL;
                rcssCtrlRegs->RCSS_CSI2B_LANE4_CFG   = CSL_RCSS_CTRL_RCSS_CSI2B_LANE4_CFG_RESETVAL;
            }
        }
        #endif
        #if defined (SOC_AWR294X)
        {
            CSL_rss_ctrlRegs *rcssCtrlRegs = (CSL_rss_ctrlRegs *) config->hwAttrs->rcssCtrlRegs;
            rcssCtrlRegs->RSS_CSI2A_CFG         = CSL_RSS_CTRL_RSS_CSI2A_CFG_RESETVAL;
            rcssCtrlRegs->RSS_CSI2A_PARITY_CTRL = CSL_RSS_CTRL_RSS_CSI2A_PARITY_CTRL_RESETVAL;
            rcssCtrlRegs->RSS_CSI2A_LANE0_CFG   = CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RESETVAL;
            rcssCtrlRegs->RSS_CSI2A_LANE1_CFG   = CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RESETVAL;
            rcssCtrlRegs->RSS_CSI2A_LANE2_CFG   = CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RESETVAL;
            rcssCtrlRegs->RSS_CSI2A_LANE3_CFG   = CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RESETVAL;
            rcssCtrlRegs->RSS_CSI2A_LANE4_CFG   = CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RESETVAL;
        }
        #endif
    }
    return status;
}

void CSIRX_CommonConfig_init(CSIRX_CommonConfig *config)
{
    memset( config, 0, sizeof(CSIRX_CommonConfig) );

    config->isSoftStoppingOnInterfaceDisable = false;
    config->isHeaderErrorCheckEnabled = false;
    config->isSignExtensionEnabled = false;
    config->isBurstSizeExpand = false;
    config->isNonPostedWrites = false;
    config->isOcpAutoIdle = false;
    config->stopStateFsmTimeoutInNanoSecs = CSIRX_STOP_STATE_FSM_TIMEOUT_MAX;
    config->burstSize = CSIRX_BURST_SIZE_8X64;
    config->endianness = CSIRX_ENDIANNESS_LITTLE_ENDIAN;
    config->startOfFrameIntr0ContextId = 0;
    config->startOfFrameIntr1ContextId = 1;
    config->endOfFrameIntr0ContextId = 0;
    config->endOfFrameIntr1ContextId = 1;
}

void CSIRX_ComplexioConfig_init(CSIRX_ComplexioConfig *config)
{
    memset( config, 0, sizeof(CSIRX_ComplexioConfig) );
}

void CSIRX_ContextConfig_init(CSIRX_ContextConfig *config)
{
    memset( config, 0, sizeof(CSIRX_ContextConfig) );

    config->transcodeConfig.transcodeFormat = CSIRX_TRANSCODE_FORMAT_NO_TRANSCODE;
    config->pingPongConfig.pingPongSwitchMode = CSIRX_PING_PONG_FRAME_SWITCHING;
    config->pingPongConfig.numFramesForFrameBasedPingPongSwitching = 1;
    config->pingPongConfig.numLinesForLineBasedPingPongSwitching = 1;
}

void CSIRX_DphyConfig_init(CSIRX_DphyConfig *config)
{
    memset( config, 0, sizeof(CSIRX_DphyConfig) );
}

/*  In debug mode, the line end codes are not detected due to a silicon problem
 *  but generating two line ends instead of one as a workaround would result in
 *  detection of one line end code. However silicon team has not determined
 *  if other logic is sound, so the define below has been provided to
 *  disable/enable it */
#define CSIRX_DEBUG_MODE_ENABLE_LINE_END_WORKAROUND

/*  short packet synthesis from virtual channel (VC), data type (DT),
    and data field (DF) */
#define CSIRX_DEBUG_MODE_SP(VC, DF, DT) (0x00000000U | (((DF) & 0xFFFFU) << 8U) | \
                                        ((VC) << 6U) | (DT))

/*! Frame Start Code synthesis from virtual channel (VC) and frame number (N) */
#define CSIRX_DEBUG_MODE_FSC(VC, N) CSIRX_DEBUG_MODE_SP((VC), (N), 0x0U)

/*! Frame End Code synthesis from virtual channel (VC) and frame number (N) */
#define CSIRX_DEBUG_MODE_FEC(VC, N) CSIRX_DEBUG_MODE_SP((VC), (N), 0x1U)

/*! Line Start Code synthesis from virtual channel (VC) and line number (N) */
#define CSIRX_DEBUG_MODE_LSC(VC, N) CSIRX_DEBUG_MODE_SP((VC), (N), 0x2U)

/*! Line End Code synthesis from virtual channel (VC) and line number (N) */
#define CSIRX_DEBUG_MODE_LEC(VC, N) CSIRX_DEBUG_MODE_SP((VC), (N), 0x3U)

/*! Packet Header synthesis from virtual channel (VC), data type (DT) and word count (WC) */
#define CSIRX_DEBUG_MODE_PH(VC, DT, WC) (0x00000000U | ((WC) << 8) | ((VC) << 6U) | (DT))

/* test payload, circularly addressed, length can be arbitrary */
uint32_t gCsirxDebugModeTestPayload[] =
{
    0x11223344,
    0x55667788,
    0x99aabbcc,
    0xddeeff11,
    0x44332211,
    0x88776655,
    0xccbbaa99,
    0x11ffeedd,
};

void CSIRX_debugModeGenerateLines(CSIRX_Handle handle,
                                 uint16_t txFormat,
                                 uint8_t virtualChannelId,
                                 uint32_t lineStartPayloadIndx,
                                 uint32_t lineStartCodelineNumber,
                                 uint32_t numOfLines,
                                 uint32_t numBytesPerLine)
{
    uint32_t numLines;
    uint32_t lineNumber, wordCounter;
    uint32_t testPayloadIndx, wordCount, testPayloadSize;
    uint32_t payload;

    DebugP_assert( (numBytesPerLine % sizeof(uint32_t) ) == 0 );
    lineNumber = lineStartCodelineNumber;

    testPayloadSize = sizeof(gCsirxDebugModeTestPayload)/sizeof(uint32_t);
    wordCount = numBytesPerLine/sizeof(uint32_t);
    testPayloadIndx = (lineStartPayloadIndx * wordCount) % testPayloadSize;

    for(numLines = 0; numLines < numOfLines; numLines++)
    {
        /* Send Line start Code (short packet) */
        CSIRX_debugModeSetShortPacket(handle, CSIRX_DEBUG_MODE_LSC(virtualChannelId, lineNumber));

        /* Send Long Packet Header (PH), 4 bytes payload length */
        CSIRX_debugModeSetLongPacketHeader(handle,
            CSIRX_DEBUG_MODE_PH(virtualChannelId, txFormat, numBytesPerLine));

        /* Send Long Packet Payload */
        for(wordCounter = 0; wordCounter < wordCount; wordCounter++)
        {
            payload = gCsirxDebugModeTestPayload[testPayloadIndx++];
            CSIRX_debugModeSetLongPacketPayload(handle, payload);
            if(testPayloadIndx == testPayloadSize)
            {
                testPayloadIndx = 0;
            }
        }

        /* Send Long Packet Footer (PF) */
        /* It seems in debug mode the PF is automatically generated */

        /* Send Line End Code (short packet) */
        CSIRX_debugModeSetShortPacket(handle,
             CSIRX_DEBUG_MODE_LEC(virtualChannelId, lineNumber));
#ifdef CSIRX_DEBUG_MODE_ENABLE_LINE_END_WORKAROUND
        CSIRX_debugModeSetShortPacket(handle,
             CSIRX_DEBUG_MODE_LEC(virtualChannelId, lineNumber));
#endif

        lineNumber++;
    }
}

void CSIRX_debugModeGenerateFrames(CSIRX_Handle handle,
                                  uint16_t txFormat,
                                  uint8_t virtualChannelId,
                                  uint32_t numOfFrames,
                                  uint32_t numLinesPerFrame,
                                  uint32_t numBytesPerLine)
{
    uint32_t frameCounter;
    uint16_t lineStartCodeLineNumber, lineStartPayloadIndx;

    lineStartPayloadIndx = 0U;
    lineStartCodeLineNumber = 1U;
    for(frameCounter = 1U; frameCounter <= numOfFrames; frameCounter++)
    {
        /* Send Frame start Code (short packet) */
        CSIRX_debugModeSetShortPacket(handle,
             CSIRX_DEBUG_MODE_FSC(virtualChannelId, frameCounter) );

        CSIRX_debugModeGenerateLines(handle,
                                    txFormat,
                                    virtualChannelId,
                                    lineStartPayloadIndx,
                                    lineStartCodeLineNumber,
                                    numLinesPerFrame,
                                    numBytesPerLine);
        lineStartPayloadIndx += numLinesPerFrame;
        lineStartCodeLineNumber += numLinesPerFrame;

        /* Send Frame end Code (short packet) */
        CSIRX_debugModeSetShortPacket(handle,
             CSIRX_DEBUG_MODE_FEC(virtualChannelId, frameCounter));
    }
}
