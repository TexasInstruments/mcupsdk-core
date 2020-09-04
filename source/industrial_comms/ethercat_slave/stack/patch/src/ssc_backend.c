/*!
* \file ssc_backend.c
*
* \brief
* Beckhoff SSC Integration: Callback Backend.
*
* \author
* KUNBUS GmbH
*
* \date
* 2021-05-19
*
* \copyright
* Copyright (c) 2021, KUNBUS GmbH<br /><br />
* All rights reserved.<br />
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:<br />
* <ol>
* <li>Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.</li>
* <li>Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.</li>
* <li>Neither the name of the copyright holder nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.</li>
* </ol>
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#include <ssc.h>
#include <ecat_def.h>
#include <ecatslv.h>
#include <aoeappl.h>

#include <ssc_kbStack.h>

#define __SSC_BACKEND_IMPLEMENTATION    1
#include "ssc_backend.h"

#define SSC_TRACETIMER  1

#define SETDWORD(b0, b1, b2, b3) \
    (((b0)<< 0)|((b1)<< 8)|((b2)<<16)|((b3)<<24))

#define SETBYTE(b0, b1, b2, b3, b4, b5, b6, b7) \
    ((b0)<<0)|((b1)<<1)|((b2)<<2)|((b3)<<3)|((b4)<<4)|((b5)<<5)|((b6)<<6)|((b7)<<7)

typedef struct SSC_LED_sLine
{
    uint32_t ledLine[3];
} SSC_LED_sLine_t;

typedef struct SSC_LED_sRunState
{
    SSC_LED_sLine_t     run;
    SSC_LED_sLine_t     errOff;
    SSC_LED_sLine_t     errOn;
    SSC_LED_sLine_t     errFlick;
    SSC_LED_sLine_t     errBlink;
    SSC_LED_sLine_t     errSingleFlash;
    SSC_LED_sLine_t     errDoubleFlash;

    uint8_t             errOffLen;
    uint8_t             errOnLen;
    uint8_t             errFlickLen;
    uint8_t             errBlinkLen;

    uint8_t             errSingleFlashLen;
    uint8_t             errDoubleFlashLen;
    uint16_t            rsvd;
} SSC_LED_sRunState_t;

static SSC_LED_sRunState_t  SSC_LED_runOff_s =
{
    { /* run */
        SETDWORD(SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0)), 0, 0
    },
    { /* err Off */
        SETDWORD(SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0)), 0, 0
    },
    { /* err On */
        SETDWORD(SETBYTE(1,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0)), 0, 0
    },
    { /* err Flick */
        SETDWORD(SETBYTE(1,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0)), 0, 0
    },
    { /* err Blink */
        SETDWORD(SETBYTE(1,1,1,1,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0)), 0, 0
    },
    { /* err Single Flash */
        SETDWORD(SETBYTE(1,1,1,1,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0)), 0, 0
    },
    { /* err Double Flash */
        SETDWORD(SETBYTE(1,1,1,1,0,0,0,0),SETBYTE(1,1,1,1,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0)), 0, 0
    },
    1,  /* err Off Len */
    1,  /* err On Len */
    2,  /* err Flick Len */
    8,  /* err Blink Len */
    24, /* err SingleFlash Len */
    32, /* err DoubleFlash Len */
    0,  /* rsvd */
};

static SSC_LED_sRunState_t  SSC_LED_runOn_s =
{
    { /* run */
        SETDWORD(SETBYTE(1,1,1,1,1,1,1,1),SETBYTE(1,1,1,1,1,1,1,1),SETBYTE(1,1,1,1,1,1,1,1),SETBYTE(1,1,1,1,1,1,1,1)), 0, 0
    },
    { /* err Off */
        SETDWORD(SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0)), 0, 0
    },
    { /* err On */
        SETDWORD(SETBYTE(1,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0)), 0, 0
    },
    { /* err Flick */
        SETDWORD(SETBYTE(1,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0)), 0, 0
    },
    { /* err Blink */
        SETDWORD(SETBYTE(1,1,1,1,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0)), 0, 0
    },
    { /* err Single Flash */
        SETDWORD(SETBYTE(1,1,1,1,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0)), 0, 0
    },
    { /* err Double Flash */
        SETDWORD(SETBYTE(1,1,1,1,0,0,0,0),SETBYTE(1,1,1,1,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0)), 0, 0
    },
    1,  /* err Off Len */
    1,  /* err On Len */
    2,  /* err Flick Len */
    8,  /* err Blink Len */
    24, /* err SingleFlash Len */
    32, /* err DoubleFlash Len */
    0,  /* rsvd */
    };

static SSC_LED_sRunState_t  SSC_LED_runFlick_s =
{
    { /* run */
        SETDWORD(SETBYTE(1,0,1,0,1,0,1,0),SETBYTE(1,0,1,0,1,0,1,0),SETBYTE(1,0,1,0,1,0,1,0),SETBYTE(1,0,1,0,1,0,1,0)), 0, 0
    },
    { /* err Off */
        SETDWORD(SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0)), 0, 0
    },
    { /* err On */
        SETDWORD(SETBYTE(1,1,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0)), 0, 0
    },
    { /* err Flick */
        SETDWORD(SETBYTE(0,1,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0)), 0, 0
    },
    { /* err Blink */
        SETDWORD(SETBYTE(1,1,1,1,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0)), 0, 0
    },
    { /* err Single Flash */
        SETDWORD(SETBYTE(1,1,1,1,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0)), 0, 0
    },
    { /* err Double Flash */
        SETDWORD(SETBYTE(1,1,1,1,0,0,0,0),SETBYTE(1,1,1,1,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0)), 0, 0
    },
    2,  /* err Off Len */
    2,  /* err On Len */
    2,  /* err Flick Len */
    8,  /* err Blink Len */
    24, /* err SingleFlash Len */
    32, /* err DoubleFlash Len */
    0,  /* rsvd */
};

static SSC_LED_sRunState_t  SSC_LED_runBlink_s =
{
    { /* run */
        SETDWORD(SETBYTE(1,1,1,1,0,0,0,0),SETBYTE(1,1,1,1,0,0,0,0),SETBYTE(1,1,1,1,0,0,0,0),SETBYTE(1,1,1,1,0,0,0,0)), 0, 0
    },
    { /* err Off */
        SETDWORD(SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0)), 0, 0
    },
    { /* err On */
        SETDWORD(SETBYTE(1,1,1,1,1,1,1,1),SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0)), 0, 0
    },
    { /* err Flick */
        SETDWORD(SETBYTE(1,0,1,0,1,0,1,1),SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0)), 0, 0
    },
    { /* err Blink */
        SETDWORD(SETBYTE(0,0,0,0,1,1,1,1),SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0)), 0, 0
    },
    { /* err Single Flash */
        SETDWORD(SETBYTE(0,0,0,0,1,1,1,1),SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0)), 0, 0
    },
    { /* err Double Flash */
        SETDWORD(SETBYTE(0,0,0,0,1,1,1,1),SETBYTE(0,0,0,0,1,1,1,1),SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0)), 0, 0
    },
    8,  /* err Off Len */
    8,  /* err On Len */
    8,  /* err Flick Len */
    8,  /* err Blink Len */
    24, /* err SingleFlash Len */
    32, /* err DoubleFlash Len */
    0,  /* rsvd */
};

static SSC_LED_sRunState_t  SSC_LED_runSingleFlash_s =
{
    { /* run */
        SETDWORD(SETBYTE(1,1,1,1,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(1,1,1,1,0,0,0,0)),
        SETDWORD(SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(1,1,1,1,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0)),
        SETDWORD(SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(1,1,1,1,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0))
    },
    { /* err Off */
        SETDWORD(SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0)), 0, 0
    },
    { /* err On */
        SETDWORD(SETBYTE(1,1,1,1,1,1,1,1),SETBYTE(1,1,1,1,1,1,1,1),SETBYTE(1,1,1,1,1,1,1,1),SETBYTE(0,0,0,0,0,0,0,0)), 0, 0
    },
    { /* err Flick */
        SETDWORD(SETBYTE(1,0,1,0,1,0,1,0),SETBYTE(1,0,1,0,1,0,1,0),SETBYTE(1,0,1,0,1,0,1,0),SETBYTE(0,0,0,0,0,0,0,0)), 0, 0
    },
    { /* err Blink */
        SETDWORD(SETBYTE(0,0,0,0,1,1,1,1),SETBYTE(0,0,0,0,1,1,1,1),SETBYTE(0,0,0,0,1,1,1,1),SETBYTE(0,0,0,0,0,0,0,0)), 0, 0
    },
    { /* err Single Flash */
        SETDWORD(SETBYTE(0,0,0,0,1,1,1,1),SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0)), 0, 0
    },
    { /* err Double Flash */
        SETDWORD(SETBYTE(0,0,0,0,1,1,1,1),SETBYTE(0,0,0,0,1,1,1,1),SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0)),
        SETDWORD(SETBYTE(0,0,0,0,1,1,1,1),SETBYTE(0,0,0,0,1,1,1,1),SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0)),
        SETDWORD(SETBYTE(0,0,0,0,1,1,1,1),SETBYTE(0,0,0,0,1,1,1,1),SETBYTE(0,0,0,0,0,0,0,0),SETBYTE(0,0,0,0,0,0,0,0))
    },
    24, /* err Off Len */
    24, /* err On Len */
    24, /* err Flick Len */
    24, /* err Blink Len */
    24, /* err SingleFlash Len */
    96, /* err DoubleFlash Len */
    0,  /* rsvd */
};

#if DC_SUPPORTED
#define SSCCHECK_DC_OVERRUN_IN_MS   4000 /**<\brief cycle time in ms to check the DC system time for overrun*/
#endif

/*! <!-- Description: -->
 *
 *  \brief
 *  Main Loop Application callback
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_mainLoop(void)
{
    if (SSC_callbacks_g.cbMainLoop)
    {
        SSC_callbacks_g.cbMainLoop(SSC_callbacks_g.pMainLoopCtxt);
    }
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Input Mapping callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pData_p process data.
 *
 *  <!-- References: -->
 *
 *  \sa SSC_outputMapping, SSC_generateMapping, SSC_application, SSC_ackErrorInd, SSC_stopOutputHandler, SSC_startOutputHandler,
 *      SSC_stopInputHandler, SSC_startInputHandler, SSC_stopMailboxHandler, SSC_startMailboxHandler, SSC_resetOutputs, SSC_EOE_receive
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_inputMapping(uint16_t* pData_p)
{
    if (SSC_callbacks_g.cbInputMapping)
    {
        SSC_callbacks_g.cbInputMapping(SSC_callbacks_g.pInputMappingCtxt, pData_p);
    }
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Input Mapping callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pData_p     process data
 *
 *  <!-- References: -->
 *
 *  \sa SSC_inputMapping, SSC_generateMapping, SSC_application, SSC_ackErrorInd, SSC_stopOutputHandler, SSC_startOutputHandler,
 *      SSC_stopInputHandler, SSC_startInputHandler, SSC_stopMailboxHandler, SSC_startMailboxHandler, SSC_resetOutputs, SSC_EOE_receive
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_outputMapping(uint16_t* pData_p)
{
    if (SSC_callbacks_g.cbOutputMapping)
    {
        SSC_callbacks_g.cbOutputMapping(SSC_callbacks_g.pOutputMappingCtxt, pData_p);
    }
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Application callback
 *
 *  <!-- References: -->
 *
 *  \sa SSC_inputMapping, SSC_generateMapping, SSC_outputMapping, SSC_ackErrorInd, SSC_stopOutputHandler, SSC_startOutputHandler,
 *      SSC_stopInputHandler, SSC_startInputHandler, SSC_stopMailboxHandler, SSC_startMailboxHandler, SSC_resetOutputs, SSC_EOE_receive
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_application(void)
{
    if (SSC_callbacks_g.cbApplication)
    {
        SSC_callbacks_g.cbApplication(SSC_callbacks_g.pApplicationCtxt);
    }
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Generate Mapping  callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[out] pInputSize_p    Input Mapping Size
 *  \param[out] pOutputSize_p   Output Mapping Size
 *  \return     uint16_t        SSC error code
 *
 *  <!-- References: -->
 *
 *  \sa SSC_inputMapping, SSC_application, SSC_outputMapping, SSC_ackErrorInd, SSC_stopOutputHandler, SSC_startOutputHandler,
 *      SSC_stopInputHandler, SSC_startInputHandler, SSC_stopMailboxHandler, SSC_startMailboxHandler, SSC_resetOutputs, SSC_EOE_receive
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
uint16_t SSC_generateMapping(uint16_t *pInputSize_p, uint16_t *pOutputSize_p)
{
    uint16_t    retVal  = (uint16_t)~0;

    if (SSC_callbacks_g.cbGenerateMapping)
    {
        retVal = SSC_callbacks_g.cbGenerateMapping(SSC_callbacks_g.pGenerateMappingCtxt, pInputSize_p, pOutputSize_p);
    }
    return retVal;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Start Mailbox Handler callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \return     uint16_t        SSC error code
 *
 *  <!-- References: -->
 *
 *  \sa SSC_inputMapping, SSC_application, SSC_outputMapping, SSC_ackErrorInd, SSC_stopOutputHandler, SSC_startOutputHandler,
 *      SSC_stopInputHandler, SSC_startInputHandler, SSC_stopMailboxHandler, SSC_generateMapping, SSC_resetOutputs, SSC_EOE_receive
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
uint16_t SSC_startMailboxHandler(void)
{
    uint16_t retVal = (uint16_t)~0;

    if (SSC_callbacks_g.cbStartMailboxHandler)
    {
        retVal = SSC_callbacks_g.cbStartMailboxHandler(SSC_callbacks_g.pStartMailboxHandlerCtxt);
    }
    return retVal;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Stop Mailbox Handler callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \return     uint16_t        SSC error code
 *
 *  <!-- References: -->
 *
 *  \sa SSC_inputMapping, SSC_application, SSC_outputMapping, SSC_ackErrorInd, SSC_stopOutputHandler, SSC_startOutputHandler,
 *      SSC_stopInputHandler, SSC_startInputHandler, SSC_startMailboxHandler, SSC_generateMapping, SSC_resetOutputs, SSC_EOE_receive
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
uint16_t SSC_stopMailboxHandler(void)
{
    uint16_t retVal = (uint16_t)~0;

    if (SSC_callbacks_g.cbStopMailboxHandler)
    {
        retVal = SSC_callbacks_g.cbStopMailboxHandler(SSC_callbacks_g.pStopMailboxHandlerCtxt);
    }
    return retVal;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Start Input Handler callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pIntMask_p      Interrupt Mask
 *  \return     uint16_t        SSC error code
 *
 *  <!-- References: -->
 *
 *  \sa SSC_inputMapping, SSC_application, SSC_outputMapping, SSC_ackErrorInd, SSC_stopOutputHandler, SSC_startOutputHandler,
 *      SSC_stopInputHandler, SSC_stopMailboxHandler, SSC_startMailboxHandler, SSC_generateMapping, SSC_resetOutputs, SSC_EOE_receive
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
uint16_t SSC_startInputHandler(uint16_t *pIntMask_p)
{
    uint16_t retVal = (uint16_t)~0;
    if (SSC_callbacks_g.cbStartInputHandler)
    {
        retVal = SSC_callbacks_g.cbStartInputHandler(SSC_callbacks_g.pStartInputHandlerCtxt, pIntMask_p);
    }
    return retVal;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Stop InputHandler callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \return     uint16_t        SSC error code
 *
 *  <!-- References: -->
 *
 *  \sa SSC_inputMapping, SSC_application, SSC_outputMapping, SSC_ackErrorInd, SSC_stopOutputHandler, SSC_startOutputHandler,
 *      SSC_startInputHandler, SSC_stopMailboxHandler, SSC_startMailboxHandler, SSC_generateMapping, SSC_resetOutputs, SSC_EOE_receive
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
uint16_t SSC_stopInputHandler(void)
{
    uint16_t retVal = (uint16_t)~0;

    if (SSC_callbacks_g.cbStopInputHandler)
    {
        retVal = SSC_callbacks_g.cbStopInputHandler(SSC_callbacks_g.pStopInputHandlerCtxt);
    }
    return retVal;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Start OutputHandler callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \return     uint16_t        SSC error code
 *
 *  <!-- Example: -->
 *
 *  <!-- References: -->
 *
 *  \sa SSC_inputMapping, SSC_application, SSC_outputMapping, SSC_ackErrorInd, SSC_stopOutputHandler, SSC_stopInputHandler,
 *      SSC_startInputHandler, SSC_stopMailboxHandler, SSC_startMailboxHandler, SSC_generateMapping, SSC_resetOutputs, SSC_EOE_receive
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
uint16_t SSC_startOutputHandler(void)
{
    uint16_t retVal = (uint16_t)~0;
    if (SSC_callbacks_g.cbStartOutputHandler)
    {
        retVal = SSC_callbacks_g.cbStartOutputHandler(SSC_callbacks_g.pStartOutputHandlerCtxt);
    }
    return retVal;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Stop OutputHandler callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \return     uint16_t        SSC error code
 *
 *  <!-- References: -->
 *
 *  \sa SSC_inputMapping, SSC_application, SSC_outputMapping, SSC_ackErrorInd, SSC_startOutputHandler, SSC_stopInputHandler,
 *      SSC_startInputHandler, SSC_stopMailboxHandler, SSC_startMailboxHandler, SSC_generateMapping, SSC_resetOutputs, SSC_EOE_receive
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
uint16_t SSC_stopOutputHandler(void)
{
    uint16_t retVal = (uint16_t)~0;

    if (SSC_callbacks_g.cbStopOutputHandler)
    {
        retVal = SSC_callbacks_g.cbStopOutputHandler(SSC_callbacks_g.pStopOutputHandlerCtxt);
    }
    return retVal;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Acknowledge Error Indication callback
 *
 *  <!-- Parameters and return values: -->
 *  \param[in]  stateTrans_p    State transission causing error
 *
 *  <!-- References: -->
 *
 *  \sa SSC_inputMapping, SSC_application, SSC_outputMapping, SSC_stopOutputHandler, SSC_startOutputHandler, SSC_stopInputHandler,
 *      SSC_startInputHandler, SSC_stopMailboxHandler, SSC_startMailboxHandler, SSC_generateMapping, SSC_resetOutputs, SSC_EOE_receive
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_ackErrorInd(uint16_t stateTrans_p)
{
    if (SSC_callbacks_g.cbAckErrorInd)
    {
        SSC_callbacks_g.cbAckErrorInd(SSC_callbacks_g.pAckErrorIndCtxt, stateTrans_p);
    }
}

/*! <!-- Description: -->
 *
 *  \brief
 *  EoE Receive callback
 *
 *  <!-- Parameters and return values: -->
 *  \param[in]  pData_p     Received data buffer
 *  \param[in]  length_p    size of received data
 *
 *  <!-- Example: -->
 *
 *  <!-- References: -->
 *
 *  \sa SSC_inputMapping, SSC_application, SSC_outputMapping, SSC_stopOutputHandler, SSC_startOutputHandler, SSC_stopInputHandler,
 *      SSC_startInputHandler, SSC_stopMailboxHandler, SSC_startMailboxHandler, SSC_generateMapping, SSC_resetOutputs, SSC_ackErrorInd
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_EOE_receive(uint16_t* pData_p, uint16_t length_p)
{
    if (SSC_callbacks_g.cbEoeReceive)
    {
        SSC_callbacks_g.cbEoeReceive(SSC_callbacks_g.pEoeReceiveCtxt, pData_p, length_p);
    }
}

/*! <!-- Description: -->
 *
 *  \brief
 *  EoE Settings Indicator callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pMac_p              MAC address.
 *  \param[in]  pIp_p               IP address
 *  \param[in]  pSubNet_p           SubNet mask
 *  \param[in]  pDefaultGateway_p   Default Gateway
 *  \param[in]  pDnsIp_p            IP of DNS server
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_EOE_settingInd(uint16_t* pMac_p, uint16_t* pIp_p, uint16_t* pSubNet_p, uint16_t* pDefaultGateway_p,
                        uint16_t* pDnsIp_p)
{
    if (SSC_callbacks_g.cbEoeSettingInd)
    {
        SSC_callbacks_g.cbEoeSettingInd(SSC_callbacks_g.pEoeSettingIndCtxt, pMac_p, pIp_p, pSubNet_p,
                                        pDefaultGateway_p, pDnsIp_p);
    }
}

/*! <!-- Description: -->
 *
 *  \brief
 *  FoE Read callback
 *
 *  <!-- Parameters and return values: -->
 *  \param[in]  pName_p         Pointer to the name of the file.
 *  \param[in]  nameSize_p      Length of the file name
 *  \param[in]  password_p      Password for the file read
 *  \param[in]  maxblockSize_p  Maximum FoE block size
 *  \param[in]  pData_p         Pointer to the response data
 *  \return     size of the data to be sent, busy or an error code.
 *
 *  <!-- References: -->
 *
 *  \sa SSC_FOW_write
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
uint16_t SSC_FOE_read(uint16_t* pName_p, uint16_t nameSize_p, uint32_t password_p, uint16_t maxblockSize_p, uint16_t* pData_p)
{
    uint16_t ret = (uint16_t)~0;

    if (SSC_callbacks_g.cbFoeRead)
    {
        ret = SSC_callbacks_g.cbFoeRead(SSC_callbacks_g.pFoeReadCtxt, pName_p, nameSize_p, password_p, maxblockSize_p, pData_p);
    }

    return ret;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  FoE read data
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  fileOffset_p            Offset of the next file data to be sent.
 *  \param[in]  foeMaxSendBlockSize_p   max Foe SendBlock Size
 *  \param[im]  pData_p                 Buffer for file data to be sent
 *  \return     size of the data to be sent, busy or an error code 0 (all data was sent before)
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
uint16_t SSC_FOE_readData(uint32_t fileOffset_p, uint16_t foeMaxSendBlockSize_p, uint16_t* pData_p)
{
    uint16_t ret = (uint16_t)~0;

    if (SSC_callbacks_g.cbFoeReadData)
    {
        ret = SSC_callbacks_g.cbFoeReadData(SSC_callbacks_g.pFoeReadDataCtxt, fileOffset_p, foeMaxSendBlockSize_p, pData_p);
    }

    return ret;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  FoE write data
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pName_p         Pointer to the name of the file.
 *  \param[in]  nameSize_p      Length of the file name
 *  \param[in]  password_p      Password for the file read
 *  \return     size of the data to be sent, busy or an error code.
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
uint16_t SSC_FOE_write(uint16_t* pName_p, uint16_t nameSize_p, uint32_t password_p)
{
    uint16_t ret = (uint16_t)~0;

    if (SSC_callbacks_g.cbFoeWrite)
    {
        ret = SSC_callbacks_g.cbFoeWrite(SSC_callbacks_g.pFoeWriteCtxt, pName_p, nameSize_p, password_p);
    }

    return ret;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  FoE write data
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pData_p             Received file data.
 *  \param[in]  size_p              Length of received file data
 *  \param[in]  dataFollowing_p     true: data to follow, false: end of transfer
 *  \return     okay, busy or an error code
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
uint16_t SSC_FOE_writeData(uint16_t* pData_p, uint16_t size_p, uint8_t dataFollowing_p)
{
    uint16_t ret = (uint16_t)~0;

    if (SSC_callbacks_g.cbFoeWriteData)
    {
        ret = SSC_callbacks_g.cbFoeWriteData(SSC_callbacks_g.pFoeWriteDataCtxt, pData_p, size_p, dataFollowing_p);
    }

    return ret;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  FoE error
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  errorCode_p error code.
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_FOE_error(uint32_t errorCode_p)
{
    if (SSC_callbacks_g.cbFoeError)
    {
        SSC_callbacks_g.cbFoeError(SSC_callbacks_g.pFoeErrorCtxt, errorCode_p);
    }
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Bootloader start
 *
 *  <!-- Parameters and return values: -->
 *  \param[in]  state_p     state to enter
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_BL_start(uint8_t state_p)
{
    if (SSC_callbacks_g.cbBlStart)
    {
        SSC_callbacks_g.cbBlStart(SSC_callbacks_g.pBlStartCtxt, state_p);
    }
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Bootloader stop
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_BL_stop(void)
{
    if (SSC_callbacks_g.cbBlStop)
    {
        SSC_callbacks_g.cbBlStop(SSC_callbacks_g.pBlStopCtxt);
    }
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Bootloader finish
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_BL_finish(void)
{
    if (SSC_callbacks_g.cbBlFinish)
    {
        SSC_callbacks_g.cbBlFinish(SSC_callbacks_g.pBlFinishCtxt);
    }
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Set LED callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  runLed_p	Run LED state
 *  \param[in]  errLed_p    Error LED state
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_setLedOut(uint8_t runLed_p, uint8_t errLed_p)
{
    if (SSC_callbacks_g.cbSetLedOut)
    {
        SSC_callbacks_g.cbSetLedOut(SSC_callbacks_g.pSetLedOutCtxt, runLed_p, errLed_p);
    }
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Send SoE Frame to master
 *
 *  \details
 *  Read SoE frame from the registered function and send it to the Master.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pSendMbx_p  SoE Mailbox memory.
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_SOE_send(void* pSendMbx_p)
{
    if (SSC_callbacks_g.cbSoeSendCb)
    {
        SSC_callbacks_g.cbSoeSendCb(SSC_callbacks_g.pSoeSendCtxt, pSendMbx_p);
    }
}

/*! <!-- Description: -->
 *
 *  \brief
 *  SoE Frame received from master.
 *
 *  \details
 *   Call the registered SoE Recv function to process the SoE input from master.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pRecvMbx_p      SoE Mailbox memory.
 *  \return     result          Mailbox error code.
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
uint8_t SSC_SOE_recv(void* pRecvMbx_p)
{
    uint8_t result = 0;
    if (SSC_callbacks_g.cbSoeRecvCb)
    {
        result = SSC_callbacks_g.cbSoeRecvCb(SSC_callbacks_g.pSoeRecvCtxt, pRecvMbx_p);
    }
    return result;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Lock Global Mutex
 *
 *  <!-- Parameters and return values: -->
 *  \param[in]  lock_p  0: unlock Mutex, 1: lock Mutex
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_MTX_global(uint8_t lock_p)
{
    if (SSC_callbacks_g.cbGlobalMutex)
    {
        SSC_callbacks_g.cbGlobalMutex(SSC_callbacks_g.pGlobalMutexCtxt, lock_p);
    }
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Get Timer Register
 *
 *  <!-- Parameters and return values: -->
 *  \return Timer value
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
uint32_t SSC_getTimerRegister(void)
{
    uint32_t    retVal = (uint32_t)~0;

    if (SSC_callbacks_g.cbGetTimerRegister)
    {
        retVal = SSC_callbacks_g.cbGetTimerRegister(SSC_callbacks_g.pGetTimerRegisterCtxt);
    }
    return retVal;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Get Timer Register with overflow flag
 *
 *  <!-- Parameters and return values: -->
 *  \param[out]  overflow    overflow flag
 *  \return Timer value
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
uint32_t SSC_get_timer_register_overflow(bool* overflow)
{
    uint32_t    retVal = (uint32_t)~0;

    if (SSC_callbacks_g.cbGetTimerRegisterOvl)
    {
        retVal = SSC_callbacks_g.cbGetTimerRegisterOvl(SSC_callbacks_g.pGetTimerRegisterOvlCtxt, overflow);
    }
    return retVal;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Clear Timer Register
 *
 *  \details
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_clearTimerRegister(void)
{
    if (SSC_callbacks_g.cbClearTimerRegister)
    {
        SSC_callbacks_g.cbClearTimerRegister(SSC_callbacks_g.pClearTimerRegisterCtxt);
    }
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Read EEPROM call.
 *
 *  <!-- Parameters and return values: -->
 *  \param[in]  wordaddr_p  EEProm address to read from
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
uint16_t SSC_EEP_read(uint32_t wordaddr_p)
{
    uint16_t retVal = (uint16_t)~0;

    if (SSC_callbacks_g.cbEscEepromRead)
    {
        retVal = SSC_callbacks_g.cbEscEepromRead(SSC_callbacks_g.pEscEepromReadCtxt, wordaddr_p);
    }
    return  retVal;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Write EEPROM call.
 *
 *  <!-- Parameters and return values: -->
 *  \param[in]  wordaddr_p  EEProm address to write to
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
uint16_t SSC_EEP_write(uint32_t wordaddr_p)
{
    uint16_t retVal = (uint16_t)~0;

    if (SSC_callbacks_g.cbEscEepromWrite)
    {
        retVal = SSC_callbacks_g.cbEscEepromWrite(SSC_callbacks_g.pEscEepromWriteCtxt, wordaddr_p);
    }
    return  retVal;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Reload EEPROM call.
 *
 *  <!-- Parameters and return values: -->
 *  \return uint16_t    error code
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
uint16_t SSC_EEP_reload(void)
{
    uint16_t retVal = (uint16_t)~0;
    if (SSC_callbacks_g.cbEscEepromReload)
    {
        retVal = SSC_callbacks_g.cbEscEepromReload(SSC_callbacks_g.pEscEepromReloadCtxt);
    }
    return  retVal;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Restore EEPROM call.
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_EEP_store(void)
{
    if (SSC_callbacks_g.cbEscEepromStore)
    {
        SSC_callbacks_g.cbEscEepromStore(SSC_callbacks_g.pEscEepromStoreCtxt);
    }
}

static void SSC_LED_updateRunState(uint8_t alStatus_p, uint8_t* pLedState_p)
{
    if (!pLedState_p)
    {
        return;
    }

    switch (alStatus_p & STATE_MASK)
    {
    case STATE_INIT:    pLedState_p[0] = LED_OFF; break;
    case STATE_PREOP:   pLedState_p[0] = LED_BLINKING; break;
    case STATE_SAFEOP:  pLedState_p[0] = LED_SINGLEFLASH; break;
    case STATE_OP:      pLedState_p[0] = LED_ON; break;
    case STATE_BOOT:    pLedState_p[0] = LED_FLICKERING; break;
    default:            pLedState_p[0] = LED_OFF; break;
    }
}

static uint8_t SSC_LED_state(uint32_t stepIndex_p, SSC_LED_sLine_t* pDefinition_p)
{
    uint8_t ledOutState = 0;

    if (pDefinition_p && stepIndex_p < 96)
    {
        uint32_t stateWord  = pDefinition_p->ledLine[stepIndex_p/32];
        uint8_t  bitInWord  = (stepIndex_p%32);

        ledOutState = ((stateWord & (1 << bitInWord)) == (1 << bitInWord))?1:0;
    }

    return ledOutState;
}

static void SSC_LED_stateWork(bool outputUpdate_p, uint16_t increments_p, uint8_t alStatus_p,
                              uint8_t* pEcatRunLedState_p, uint8_t* pEcatErrLedState_p)
{
    uint8_t                 runLed      = 0;
    uint8_t                 errLed      = 0;
    static uint32_t         globSteps   = 0;
    SSC_LED_sRunState_t*    pRunState   = NULL;
    SSC_LED_sLine_t*        pRunLine    = NULL;
    SSC_LED_sLine_t*        pErrLine    = NULL;
    uint8_t                 modulo      = 0;

    if (outputUpdate_p)
    {
        runLed  = 1;
        errLed  = 0;
        goto Exit;
    }

    if (!pEcatRunLedState_p || !pEcatErrLedState_p)
    {
        /* fatal error */
        runLed  = 0;
        errLed  = 1;
        goto Exit;
    }

    SSC_LED_updateRunState(alStatus_p, pEcatRunLedState_p);

    switch (pEcatRunLedState_p[0])
    {
    case LED_OFF:           pRunState = &SSC_LED_runOff_s; break;
    case LED_FLICKERING:    pRunState = &SSC_LED_runFlick_s; break;
    case LED_BLINKING:      pRunState = &SSC_LED_runBlink_s; break;
    case LED_SINGLEFLASH:   pRunState = &SSC_LED_runSingleFlash_s; break;
    case LED_ON:            pRunState = &SSC_LED_runOn_s; break;
    default: break;
    }

    if (!pRunState)
    {
        /* fatal error */
        runLed  = 0;
        errLed  = 1;
        goto Exit;
    }

    pRunLine = &pRunState->run;

    switch(pEcatErrLedState_p[0])
    {
    case LED_OFF:           pErrLine    = &pRunState->errOff;
                            modulo      = pRunState->errOffLen; break;
    case LED_FLICKERING:    pErrLine    = &pRunState->errFlick;
                            modulo      = pRunState->errFlickLen; break;
    case LED_BLINKING:      pErrLine    = &pRunState->errBlink;
                            modulo      = pRunState->errBlinkLen; break;
    case LED_SINGLEFLASH:   pErrLine    = &pRunState->errSingleFlash;
                            modulo      = pRunState->errSingleFlashLen; break;
    case LED_DOUBLEFLASH:   pErrLine    = &pRunState->errDoubleFlash;
                            modulo      = pRunState->errDoubleFlashLen; break;
    case LED_ON:            pErrLine    = &pRunState->errOn;
                            modulo      = pRunState->errOnLen; break;
    default:
        break;
    }

    if (!pRunLine || ! pErrLine || !modulo)
    {
        /* fatal error */
        runLed  = 0;
        errLed  = 1;
        goto Exit;
    }

    /* find cur Step */
    globSteps   += increments_p;
    globSteps   %= modulo;

    /* extract LED setting */
    runLed  = SSC_LED_state(globSteps, pRunLine);
    errLed  = SSC_LED_state(globSteps, pErrLine);

Exit:
    SSC_setLedOut(runLed, errLed);
}

void SSC_checkTimer(bool ecatWaitForAlControlRes_p, int16_t* pEsmTimeoutCnt_p,
                    bool ecatOutputUpdateRunning_p, uint8_t alStatus_p,
                    uint8_t* pEcatRunLedState_p, uint8_t* pEcatErrLedState_p,
                    uint16_t* pEePromStoreTimeoutValue_p, uint16_t* pEePromStoreTimeoutCnt_p, cbEepStore_t cbStoreEep_p,
                    bool dc32Bit_p, uint64_t* pDcTimeStamp_p, uint32_t* pDcChkOverrunCnt_p)
{
    uint16_t            expiredTicks    = 0;
    uint16_t            expiredMsecs    = 0;
    uint32_t            currentCounter;
    static uint64_t     lastCounter     = ~0;
    static uint64_t     lastExpTicks    = ~0;

    uint32_t            deltaTmCounter  = 0;
    uint32_t            deltaTiCounter  = 0;

    bool                crtOverFlow     = false;

#if (defined SSC_TRACETIMER) && (1==SSC_TRACETIMER)
    static uint16_t     maxExticks      = 0;
    static uint32_t     maxDelta        = 0;
#endif

    /* only enter down when msec limit is passed */
    currentCounter = SSC_get_timer_register_overflow(&crtOverFlow);

    if (crtOverFlow)
    {
        currentCounter += 0x100000000;
    }

    if ((uint64_t)~0 == lastCounter)
    {
        lastCounter     = currentCounter;
        return;
    }

    deltaTmCounter  = currentCounter - lastCounter;
    deltaTiCounter  = currentCounter - lastExpTicks;

    if (deltaTmCounter < SSC_ECAT_TIMER_INC_P_MS())
    {
        /* do nothing before msec */
        return;
    }

    lastCounter     = (currentCounter&0xFFFFFFFF); /* limit last counter to real 32Bit */

    expiredMsecs    = deltaTmCounter / SSC_ECAT_TIMER_INC_P_MS();
    expiredTicks    = (deltaTiCounter / SSC_ECAT_TIMER_INC_P_MS());
    expiredTicks   /= 50; /* reduce ticks to each 50 msec */

#if (defined SSC_TRACETIMER) && (1==SSC_TRACETIMER)
    if (deltaTmCounter > maxDelta)
    {
        OSAL_printf("%s:MaxD:%d (%d)\r\n", __func__, deltaTmCounter, expiredMsecs);
        maxDelta    = deltaTmCounter;
    }
    if (expiredTicks > maxExticks)
    {
        OSAL_printf("%s:MaxET:%d\r\n", __func__, expiredTicks);
        maxExticks  = expiredTicks;
    }
#endif

    /*decrement the state transition timeout counter*/
    if(ecatWaitForAlControlRes_p && pEsmTimeoutCnt_p && (pEsmTimeoutCnt_p[0] > 0))
    {
        if (pEsmTimeoutCnt_p[0] < expiredMsecs)
        {
            pEsmTimeoutCnt_p[0]     = 0;
        }
        else
        {
            pEsmTimeoutCnt_p[0]    -= expiredMsecs;
        }
    }

#if !ESC_SM_WD_SUPPORTED
#if MAX_PD_OUTPUT_SIZE > 0
    /*The SyncManager watchdog is not supported, the local watchdog is used*/
    ECAT_CheckWatchdog();
#endif	 //#if MAX_PD_OUTPUT_SIZE > 0
#endif //#if !ESC_SM_WD_SUPPORTED

    if (expiredTicks)
    {
        /* local implement LED stateMachine */
        SSC_LED_stateWork(ecatOutputUpdateRunning_p, expiredTicks, alStatus_p, pEcatRunLedState_p, pEcatErrLedState_p);

        lastExpTicks    = (currentCounter&0xFFFFFFFF); /* limit last counter to real 32Bit */
    }

#if DC_SUPPORTED
    DC_CheckWatchdog();
#endif
#if ESC_EEPROM_EMULATION
    /*ECATCHANGE_START(V5.12) EEPROM3*/

    if (pEePromStoreTimeoutValue_p && (pEePromStoreTimeoutValue_p[0] > 0) && pEePromStoreTimeoutCnt_p)
    {
        pEePromStoreTimeoutCnt_p[0]++;

        if (pEePromStoreTimeoutValue_p[0] <= pEePromStoreTimeoutCnt_p[0])
        {
            pEePromStoreTimeoutValue_p[0]   = 0;
            pEePromStoreTimeoutCnt_p[0]     = 0;

            if (cbStoreEep_p != NULL)
            {
                cbStoreEep_p();
            }
        }
    }
    /*ECATCHANGE_END(V5.12) EEPROM3*/

#endif //#if ESC_EEPROM_EMULATION
#if COE_SUPPORTED
#if !_PIC18
/*ECATCHANGE_START(V5.12) COE4*/

#if DC_SUPPORTED
    /* Increment the counter every ms between two updates based on the system time (32Bit overrun is handled in COE_SyncTimeStamp) */
    if (!dc32Bit_p || (pDcTimeStamp_p && ((pDcTimeStamp_p[0] & 0xFFFFFFFF) <= 4293000000)))
    {
#endif
        /* the timestamp is stored in ns */
        pDcTimeStamp_p[0]   = pDcTimeStamp_p[0] + 1000000;

#if DC_SUPPORTED
    }
    else if(dc32Bit_p && pDcChkOverrunCnt_p)
    {
        /* in case of a 32Bit DC and almost expired time stamp check for a DC overrun*/
        pDcChkOverrunCnt_p[0] = SSCCHECK_DC_OVERRUN_IN_MS;
    }

    if (pDcChkOverrunCnt_p)
    {
        pDcChkOverrunCnt_p[0]++;
    }
#endif
/*ECATCHANGE_END(V5.12) COE4*/
#endif //#if !_PIC18
#endif
}

UINT16  AOEAPPL_AmsInd(AmsCmd MBXMEM *pCmd)
{
    uint16_t retVal = (uint16_t)~0;
    if(SSC_callbacks_g.cbAoeRecvCb != NULL)
    {
        retVal = SSC_callbacks_g.cbAoeRecvCb(SSC_callbacks_g.pAoeRecvCtxt, pCmd);
    }
    return retVal;
}

//*************************************************************************************************
