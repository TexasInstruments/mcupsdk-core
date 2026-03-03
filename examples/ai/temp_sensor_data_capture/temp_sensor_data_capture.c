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

/**
 * \file temp_sensor_data_capture.c
 * \brief Temperature Sensor Data Capture Example with DAP
 *
 * This example demonstrates temperature/humidity data capture using the
 * TIDA-010997 booster pack HDC3020 sensor with Edge AI Studio integration via DAP.
 */

#include <string.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include <drivers/i2c.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "Dap.h"
#include "tida010997.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** \brief Default number of samples to stream */
#define DEFAULT_SAMPLE_COUNT        (50U)

/** \brief Sample delay in microseconds (100ms = 10Hz to match sensor rate) */
#define SAMPLE_DELAY_US             (100U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void Dap_Link_TxCallback(UART_Handle handle, UART_Transaction *trans);
void Dap_Link_RxCallback(UART_Handle handle, UART_Transaction *trans);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* -------------------------------------------------------------------------- */
/*                 DAP Interface Configuration Data                           */
/* -------------------------------------------------------------------------- */

/* Sensor JSON strings (pre-formatted for Edge AI Studio) */
/* type=3: SCALAR, dataFormat=5: INT16 (raw sensor value) */
static const char gTempSensorJson[] =
    "{\"name\":\"Temperature1\",\"type\":3,\"dataFormat\":5,\"sequenceNumbers\":true,\"labels\":\"value\"}";

/* Property structures */
static Dap_Interface_PropertyInfoType gSamplesProperty = {
    .NamePtr = "samples",
    .Type    = DAP_DATA_FORMAT_UINT16,
    .Value   = {.U16 = DEFAULT_SAMPLE_COUNT}
};

/* DAP Interface Configuration */
static const Dap_InterfaceConfigType gDapInterfaceConfig = {
    /* Sensors */
    .SensorList  = {gTempSensorJson, NULL, NULL, NULL, NULL, NULL, NULL, NULL},
    .SensorCount = 1U,

    /* Models (none for data capture) */
    .ModelList  = {NULL, NULL, NULL, NULL},
    .ModelCount = 0U,

    /* Properties */
    .PropertyList  = {&gSamplesProperty, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
                      NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL},
    .PropertyCount = 1U,

    /* Inference values (none for data capture) */
    .InfValueList  = {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL},
    .InfValueCount = 0U
};

/* DAP instance */
static Dap_InstanceType gDapInstance;

/* HDC3020 sensor instance */
static TIDA010997_Sensor_Instance_t gHdc3020Sensor;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/**
 * \brief Main entry point for temperature sensor data capture example
 */
void temp_sensor_data_capture_main(void *args)
{
    int32_t                       status;
    I2C_Handle                     i2cHandle;
    Dap_InitParamsType             dapParams;
    TIDA010997_Sensor_InitParams_t sensorParams;
    boolean                        isStreaming;
    boolean                        streamStarted = FALSE;
    const uint16_t                 sampleSizes[DAP_INTERFACE_MAX_SENSORS] = {2U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};

    /* Open drivers - UART and I2C configured via SysConfig */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("[TEMP] TIDA-010997 Temperature Sensor Data Capture Example\r\n");

    /* Get I2C handle */
    i2cHandle = gI2cHandle[CONFIG_I2C0];

    /* Initialize HDC3020 sensor using TIDA010997 board driver */
    TIDA010997_Sensor_initParamsDefault(&sensorParams, TIDA010997_SENSOR_TYPE_HDC3020);
    sensorParams.i2cHandle = i2cHandle;
    sensorParams.config.hdc3020.autoMeasurement = true;
    sensorParams.config.hdc3020.measurementRate = 4U;  /* 10Hz */
    sensorParams.config.hdc3020.lowPowerMode    = 0U;  /* LPM_0 */

    status = TIDA010997_Sensor_init(&gHdc3020Sensor, &sensorParams);
    if (status != TIDA010997_SENSOR_OK)
    {
        DebugP_log("[TEMP] HDC3020 init failed: %d\r\n", status);
        DebugP_log("[TEMP] Make sure TIDA-010997 booster pack is connected\r\n");
        goto cleanup;
    }

    DebugP_log("[TEMP] HDC3020 sensor initialized at 10Hz\r\n");

    /* Wait for sensor to stabilize */
    ClockP_usleep(1000000U);  /* 1 second */

    /* Initialize DAP */
    Dap_InitParamsSetDefault(&dapParams);
    dapParams.LinkParams.UartInstanceIndex = CONFIG_UART0;
    dapParams.LinkParams.LinkTxMode        = DAP_LINK_MODE_INTERRUPT;
    dapParams.LinkParams.LinkRxMode        = DAP_LINK_MODE_INTERRUPT;
    dapParams.InterfaceConfigPtr           = &gDapInterfaceConfig;

    status = Dap_Init(&gDapInstance, &dapParams);
    if (status != DAP_ERROR_NONE)
    {
        DebugP_log("[TEMP] DAP Init failed: %u\r\n", status);
        goto cleanup;
    }

    status = Dap_Open(&gDapInstance);
    if (status != DAP_ERROR_NONE)
    {
        DebugP_log("[TEMP] DAP Open failed: %u\r\n", status);
        goto cleanup;
    }

    DebugP_log("[TEMP] DAP initialized. Waiting for Edge AI Studio...\r\n");

    /* Main processing loop */
    while (1)
    {
        status = Dap_Process(&gDapInstance);

        if (status != DAP_ERROR_NONE)
        {
            DebugP_log("[TEMP] DAP Process error: %u\r\n", status);
        }

        /* Check streaming status */
        isStreaming = FALSE;
        (void)Dap_IsStreaming(&gDapInstance, &isStreaming);

        if (isStreaming == TRUE)
        {
            /* Start stream on first iteration */
            if (streamStarted == FALSE)
            {
                uint32_t totalSamples = (uint32_t)gSamplesProperty.Value.U16;

                status = Dap_StartSensorStream(&gDapInstance, sampleSizes, sizeof(sampleSizes), totalSamples);
                if (status == DAP_ERROR_NONE)
                {
                    streamStarted = TRUE;
                }
            }

            /* Stream samples one at a time */
            if (streamStarted == TRUE)
            {
                TIDA010997_Sensor_Data_t dataArray[2];
                uint32_t                 dataCount = 2U;
                uint8_t                  tempBytes[2];

                status = TIDA010997_Sensor_getAllData(&gHdc3020Sensor, dataArray, &dataCount);

                if ((status == TIDA010997_SENSOR_OK) && (dataCount >= 1U))
                {
                    /* Send raw temperature value in big-endian format (2 bytes) */
                    uint16_t tempRaw = (uint16_t)(dataArray[0].rawValue & 0xFFFFU);
                    tempBytes[0] = (uint8_t)(tempRaw >> 8);
                    tempBytes[1] = (uint8_t)(tempRaw & 0xFFU);

                    (void)Dap_StreamSensorSample(&gDapInstance, 0U, tempBytes, 2U);
                }

                /* Delay to control sample rate */
                ClockP_usleep(SAMPLE_DELAY_US);
            }
        }
        else
        {
            /* Reset stream state when not streaming */
            streamStarted = FALSE;
        }
    }

cleanup:
    /* De-initialize sensor */
    TIDA010997_Sensor_deInit(&gHdc3020Sensor);

    DebugP_log("[TEMP] Example finished.\r\n");

    Board_driversClose();
    Drivers_close();
}

/* ========================================================================== */
/*                 UART Callback Implementations                              */
/* ========================================================================== */

/*
 * These callbacks are configured in SysConfig for the UART instance
 * used by DAP. The transaction args are set by the link layer to point
 * to the Dap_Link_InstanceType.
 */

void Dap_Link_TxCallback(UART_Handle handle, UART_Transaction *trans)
{
    if ((trans != NULL) && (trans->args != NULL))
    {
        Dap_Link_InstanceType *linkPtr = (Dap_Link_InstanceType *)trans->args;

        /* Reset TX state after transmission completes */
        Dap_Link_ContinueTransmit(linkPtr);
    }
}

void Dap_Link_RxCallback(UART_Handle handle, UART_Transaction *trans)
{
    if ((trans != NULL) && (trans->args != NULL))
    {
        Dap_Link_InstanceType *linkPtr    = (Dap_Link_InstanceType *)trans->args;
        uint32_t               bytesReceived = trans->count;

        /* Continue receiving DAP frame via link layer */
        Dap_Link_ContinueReceive(linkPtr, bytesReceived);
    }
}
