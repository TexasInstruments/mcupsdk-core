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
 * \file imu_sensor_data_capture.c
 * \brief IMU Sensor Data Capture Example with DAP
 *
 * This example demonstrates 3-axis accelerometer data capture using the
 * TIDA-010997 booster pack BMI270 sensor with Edge AI Studio integration via DAP.
 */

#include <string.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include <drivers/mcspi.h>
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

/** \brief Sample delay in microseconds (1ms = 1000Hz for faster streaming) */
#define SAMPLE_DELAY_US             (1000U)

/** \brief Size of accelerometer sample in bytes (3 x INT16 = 6 bytes) */
#define ACCEL_SAMPLE_SIZE           (6U)

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
/* type=4: vector, dataFormat=5: INT16 (raw sensor values) */
/* shape=[3] indicates 3 values per sample (X, Y, Z) */
static const char gAccelSensorJson[] =
    "{\"name\":\"3-axis Accelerometer\",\"type\":4,\"dataFormat\":2,\"columns\":3,\"sequenceNumbers\":true,\"labels\":[\"x\", \"y\", \"z\"]}";

/* Property structures */
static Dap_Interface_PropertyInfoType gSamplesProperty = {
    .NamePtr = "samples",
    .Type    = DAP_DATA_FORMAT_UINT16,
    .Value   = {.U16 = DEFAULT_SAMPLE_COUNT}
};

/* DAP Interface Configuration */
static const Dap_InterfaceConfigType gDapInterfaceConfig = {
    /* Sensors */
    .SensorList  = {gAccelSensorJson, NULL, NULL, NULL, NULL, NULL, NULL, NULL},
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

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/**
 * \brief Main entry point for IMU sensor data capture example
 */
void imu_sensor_data_capture_main(void *args)
{
    int32_t                        status;
    MCSPI_Handle                   spiHandle;
    Dap_InitParamsType             dapParams;
    TIDA010997_Sensor_InitParams_t sensorParams;
    TIDA010997_Sensor_Instance_t   bmi270Sensor;
    boolean                        isStreaming;
    boolean                        streamStarted = FALSE;
    const uint16_t                 sampleSizes[DAP_INTERFACE_MAX_SENSORS] = {ACCEL_SAMPLE_SIZE, 0U, 0U, 0U, 0U, 0U, 0U, 0U};

    /* Open drivers - UART and SPI configured via SysConfig */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("[IMU] TIDA-010997 BMI270 IMU Data Capture Example\r\n");

    /* Get SPI handle */
    spiHandle = gMcspiHandle[CONFIG_MCSPI0];

    /* Initialize BMI270 sensor using TIDA010997 board driver */
    TIDA010997_Sensor_initParamsDefault(&sensorParams, TIDA010997_SENSOR_TYPE_BMI270);
    sensorParams.spiHandle = spiHandle;
    sensorParams.config.bmi270.accelRange  = 2U;    /* 8g */
    sensorParams.config.bmi270.gyroRange   = 0U;    /* 2000 dps */
    sensorParams.config.bmi270.accelOdr    = 0x08U; /* 100 Hz */
    sensorParams.config.bmi270.gyroOdr     = 0x08U; /* 100 Hz */
    sensorParams.config.bmi270.accelEnable = true;
    sensorParams.config.bmi270.gyroEnable  = false; /* Accel only */

    status = TIDA010997_Sensor_init(&bmi270Sensor, &sensorParams);
    if (status != TIDA010997_SENSOR_OK)
    {
        DebugP_log("[IMU] BMI270 init failed: %d\r\n", status);
        DebugP_log("[IMU] Make sure TIDA-010997 booster pack is connected\r\n");
        goto cleanup;
    }

    DebugP_log("[IMU] BMI270 sensor initialized (Accel: +/-2g, 100Hz)\r\n");

    /* Wait for sensor to stabilize */
    ClockP_usleep(100000U);  /* 100ms */

    /* Initialize DAP */
    Dap_InitParamsSetDefault(&dapParams);
    dapParams.LinkParams.UartInstanceIndex = CONFIG_UART0;
    dapParams.LinkParams.LinkTxMode        = DAP_LINK_MODE_INTERRUPT;
    dapParams.LinkParams.LinkRxMode        = DAP_LINK_MODE_INTERRUPT;
    dapParams.InterfaceConfigPtr           = &gDapInterfaceConfig;

    status = Dap_Init(&gDapInstance, &dapParams);
    if (status != DAP_ERROR_NONE)
    {
        DebugP_log("[IMU] DAP Init failed: %u\r\n", status);
        goto cleanup;
    }

    status = Dap_Open(&gDapInstance);
    if (status != DAP_ERROR_NONE)
    {
        DebugP_log("[IMU] DAP Open failed: %u\r\n", status);
        goto cleanup;
    }

    DebugP_log("[IMU] DAP initialized. Waiting for Edge AI Studio...\r\n");

    /* Main processing loop */
    while (1)
    {
        status = Dap_Process(&gDapInstance);

        if (status != DAP_ERROR_NONE)
        {
            DebugP_log("[IMU] DAP Process error: %u\r\n", status);
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

                status = Dap_StartSensorStream(&gDapInstance, sampleSizes,sizeof(sampleSizes), totalSamples);
                if (status == DAP_ERROR_NONE)
                {
                    streamStarted = TRUE;
                }
            }

            /* Stream samples one at a time */
            if (streamStarted == TRUE)
            {
                TIDA010997_Sensor_Data_t dataArray[3];
                uint32_t                 dataCount = 3U;
                uint8_t                  sensorBytes[ACCEL_SAMPLE_SIZE] = {0U};

                status = TIDA010997_Sensor_getAllData(&bmi270Sensor, dataArray, &dataCount);

                if ((status == TIDA010997_SENSOR_OK) && (dataCount >= 3U))
                {
                    /* Pack X, Y, Z as big-endian INT16 (2 bytes each = 6 bytes total) */
                    /* dataArray order: ACCEL_X, ACCEL_Y, ACCEL_Z */
                    uint16_t accelX = (uint16_t)(dataArray[0].rawValue & 0xFFFFU);
                    uint16_t accelY = (uint16_t)(dataArray[1].rawValue & 0xFFFFU);
                    uint16_t accelZ = (uint16_t)(dataArray[2].rawValue & 0xFFFFU);

                    sensorBytes[0] = (uint8_t)(accelX >> 8);
                    sensorBytes[1] = (uint8_t)(accelX & 0xFFU);
                    sensorBytes[2] = (uint8_t)(accelY >> 8);
                    sensorBytes[3] = (uint8_t)(accelY & 0xFFU);
                    sensorBytes[4] = (uint8_t)(accelZ >> 8);
                    sensorBytes[5] = (uint8_t)(accelZ & 0xFFU);
                }

                /* Stream sample (real data or zeros if sensor failed) */
                (void)Dap_StreamSensorSample(&gDapInstance, 0U, sensorBytes, ACCEL_SAMPLE_SIZE);

                /* Short delay to allow sensor read without blocking stream */
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
    DebugP_log("[IMU] Example finished.\r\n");

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
