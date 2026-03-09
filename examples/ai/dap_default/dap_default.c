/*
 *  Copyright (C) 2026 Texas Instruments Incorporated
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

/*
 * This example demonstrates basic DAP (Device Agent Protocol) usage with
 * MCU+SDK UART driver in interrupt/callback mode.
 *
 * The DAP module enables communication with TI Edge AI Studio for:
 * - Sensor data streaming
 * - Model configuration
 * - Property read/write
 * - Pipeline configuration
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <string.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/SystemP.h>
#include <kernel/dpl/ClockP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "Dap.h"
/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** \brief Default number of samples to stream */
#define DEFAULT_SAMPLE_COUNT        (50U)

/** \brief Temperature sample size (1 x INT16 = 2 bytes) */
#define TEMP_SAMPLE_SIZE            (2U)

/* ========================================================================== */
/*                 Private Variable Declarations                              */
/* ========================================================================== */

/* -------------------------------------------------------------------------- */
/*                 DAP Interface Configuration Data                           */
/* -------------------------------------------------------------------------- */

/* Sensor JSON strings (pre-formatted) */

/* type=3: scalar, dataFormat=5: INT16 */
static const char gSensor0Json[] =
    "{\"name\":\"temperature\",\"type\":3,\"dataFormat\":5,\"sequenceNumbers\":true,\"labels\":\"value\"}";

/* Model JSON strings (pre-formatted) */
static const char gModel0Json[] =
    "{\"name\":\"anomaly_detector\",\"task\":\"classification\",\"projectId\":\"demo-001\"}";

/* Property structures */
static Dap_Interface_PropertyInfoType gSamplesProperty = {
    .NamePtr = "samples", .Type = DAP_DATA_FORMAT_UINT16, .Value = {.U16 = DEFAULT_SAMPLE_COUNT}};

/* DAP Interface Configuration */
static const Dap_InterfaceConfigType gDapInterfaceConfig = {
    /* Sensors */
    .SensorList  = {gSensor0Json, NULL, NULL, NULL, NULL, NULL, NULL, NULL},
    .SensorCount = 1U,

    /* Models */
    .ModelList  = {gModel0Json, NULL, NULL, NULL},
    .ModelCount = 1U,

    /* Properties */
    .PropertyList  = {&gSamplesProperty, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
                      NULL, NULL, NULL, NULL, NULL, NULL, NULL},
    .PropertyCount = 1U,

    /* Inference values */
    .InfValueList  = {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL},
    .InfValueCount = 0U};

/* DAP instance */
Dap_InstanceType gDapInstance;

/* ========================================================================== */
/*                 Private Function Prototypes                                */
/* ========================================================================== */

void Dap_Link_TxCallback(UART_Handle handle, UART_Transaction *trans);
void Dap_Link_RxCallback(UART_Handle handle, UART_Transaction *trans);

/* ========================================================================== */
/*                 Main Function                                              */
/* ========================================================================== */

void dap_default_main(void *args)
{
    int32_t status = 0U;
    boolean  streamStarted = FALSE;
    /* Sample sizes for sensors: sensor0=accel (6 bytes), sensor1=temp (2 bytes) */
    const uint16_t sampleSizes[DAP_INTERFACE_MAX_SENSORS] = {TEMP_SAMPLE_SIZE, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
    uint16_t dummyCounter = 0U;

    /* Open drivers - UART is configured via SysConfig with callback mode */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("[DAP] Example started...\r\n");

    /* DAP initialization */
    Dap_InitParamsType dapParams;
    Dap_InitParamsSetDefault(&dapParams);

    /* Configure link layer to use the SysConfig UART instance */
    dapParams.LinkParams.UartInstanceIndex = CONFIG_UART0;
    dapParams.LinkParams.LinkTxMode = DAP_LINK_MODE_INTERRUPT;
    dapParams.LinkParams.LinkRxMode = DAP_LINK_MODE_INTERRUPT;
    dapParams.InterfaceConfigPtr    = &gDapInterfaceConfig;

    status = Dap_Init(&gDapInstance, &dapParams);
    if (status != DAP_ERROR_NONE)
    {
        DebugP_log("[DAP] Init failed with status: %u\r\n", status);
        goto cleanup;
    }

    status = Dap_Open(&gDapInstance);
    if (status != DAP_ERROR_NONE)
    {
        DebugP_log("[DAP] Open failed with status: %u\r\n", status);
        goto cleanup;
    }

    DebugP_log("[DAP] Initialized successfully. Waiting for commands...\r\n");

    /* Main processing loop */
    while (1)
    {
        status = Dap_Process(&gDapInstance);

        if (status != DAP_ERROR_NONE)
        {
            DebugP_log("[DAP] Process error: %u\r\n", status);
            break;
        }

        /* Check streaming status */
        boolean isStreaming = FALSE;
        (void)Dap_IsStreaming(&gDapInstance, &isStreaming);

        if (isStreaming == TRUE)
        {
            /* Start stream on first iteration */
            if (streamStarted == FALSE)
            {
                uint32_t totalSamples = (uint32_t)gSamplesProperty.Value.U16;

                status = Dap_StartSensorStream(&gDapInstance, sampleSizes, sizeof(sampleSizes), totalSamples,DAP_DATA_CHANNEL_SENSOR_SIGNAL);
                if (status == DAP_ERROR_NONE)
                {
                    streamStarted = TRUE;
                    dummyCounter = 0U;
                }
            }

            /* Stream dummy data */
            if (streamStarted == TRUE)
            {
                /* Dummy temperature data (sensor 0): single INT16 big-endian */
                uint8_t tempBytes[TEMP_SAMPLE_SIZE];
                uint16_t tempValue = 25000U + dummyCounter;  /* ~25C in raw format */
                tempBytes[0] = (uint8_t)(tempValue >> 8);
                tempBytes[1] = (uint8_t)(tempValue & 0xFFU);

                (void)Dap_StreamSensorSample(&gDapInstance, 1U, tempBytes, TEMP_SAMPLE_SIZE);

                dummyCounter++;
            }
        }
        else
        {
            /* Reset stream state when not streaming */
            streamStarted = FALSE;
        }
    }

cleanup:
    DebugP_log("[DAP] Example finished.\r\n");

    Board_driversClose();
    Drivers_close();
}

/* ========================================================================== */
/*                 UART Callback Implementations                              */
/* ========================================================================== */

/*
 * These callbacks are configured in SysConfig for the UART instance
 * used by DAP. The names must match what SysConfig expects:
 *   - writeCallbackFxn = Dap_Link_TxCallback
 *   - readCallbackFxn = Dap_Link_RxCallback
 *   - readMode = UART_TRANSFER_MODE_CALLBACK
 *   - writeMode = UART_TRANSFER_MODE_CALLBACK
 *
 * The transaction args are set by the link layer to point to the
 * Dap_Link_InstanceType, which is used to continue frame processing.
 */

void Dap_Link_TxCallback(UART_Handle handle, UART_Transaction *trans)
{
    if (trans != NULL && trans->args != NULL)
    {
        /*
         * The transaction args points to the link layer instance.
         * Call ContinueTransmit to reset TX state after transmission completes.
         */
        Dap_Link_InstanceType *linkPtr = (Dap_Link_InstanceType *)trans->args;
        Dap_Link_ContinueTransmit(linkPtr);
    }
}

void Dap_Link_RxCallback(UART_Handle handle, UART_Transaction *trans)
{
    if (trans != NULL && trans->args != NULL)
    {
        /*
         * The transaction args points to the link layer instance.
         * Call ContinueReceive to process received bytes and continue
         * building the DAP frame.
         */
        Dap_Link_InstanceType *linkPtr = (Dap_Link_InstanceType *)trans->args;
        uint32_t bytesReceived         = trans->count;

        Dap_Link_ContinueReceive(linkPtr, bytesReceived);
    }
}
