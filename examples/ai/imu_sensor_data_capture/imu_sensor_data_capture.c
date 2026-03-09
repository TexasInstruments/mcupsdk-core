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
 * \brief IMU Sensor Data Capture Example with Dual-Mode DAP Support
 *
 * This example demonstrates dual-mode operation for the TIDA-010997 booster pack
 * BMI270 IMU sensor with Edge AI Studio integration via DAP protocol.
 *
 * Supported Modes (controlled by Edge AI Studio):
 * - DATA_ACQUISITION: Stream raw 3-axis accelerometer data for dataset collection
 * - SENSOR_INFERENCE: Run on-device inference and stream classification results
 *
 * The application:
 * 1. Initializes BMI270 sensor via SPI
 * 2. Waits for Edge AI Studio to configure pipeline mode
 * 3. In DATA_ACQUISITION mode: Streams raw sensor samples (6 bytes per sample)
 * 4. In SENSOR_INFERENCE mode: Collects frames, runs feature extraction and
 *    TVM model inference, streams results (class + probabilities)
 *
 * Inference Classes:
 * - Class 0: Jerk
 * - Class 1: Smooth
 */

#include <stdlib.h>
#include <string.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include <drivers/mcspi.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* Feature extraction library */
#include "feature_extract.h"

/* TVM-generated model interface */
#include "tvmgen_default.h"

/* DAP protocol for Edge AI Studio communication */
#include "Dap.h"

/* TIDA-010997 boosterpack sensor driver */
#include "tida010997.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** \brief Default number of samples to stream */
#define DEFAULT_SAMPLE_COUNT            (128U)

/** \brief Size of inference result in bytes (class + 2 probabilities) */
#define INFERENCE_RESULT_SIZE           (3U)

/** \brief Size of raw accelerometer sample in bytes (3 x INT16 = 6 bytes) */
#define ACCEL_SAMPLE_SIZE               (6U)

/** \brief Number of accelerometer axes */
#define NUM_AXES                        (3U)

/** \brief Total raw input size (FE_VARIABLES * FE_FRAME_SIZE) */
#define RAW_INPUT_SIZE                  (FE_VARIABLES * FE_FRAME_SIZE)

/** \brief Total model input size */
#define MODEL_INPUT_SIZE                (1 * FE_STACKING_CHANNELS * FE_STACKING_FRAME_WIDTH * 1)

/** \brief Sample delay in microseconds */
#define SAMPLE_DELAY_US                 (1000U)

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* UART callbacks for DAP link layer */
void Dap_Link_TxCallback(UART_Handle handle, UART_Transaction *trans);
void Dap_Link_RxCallback(UART_Handle handle, UART_Transaction *trans);

/* ========================================================================== */
/*                 DAP Interface Configuration (Inline)                       */
/* ========================================================================== */

/**
 * Accelerometer sensor JSON configuration for Edge AI Studio
 * - type=4: vector sensor
 * - dataFormat=2: INT16
 * - columns=3: X, Y, Z axes
 */
static const char gSensorAccJson[] =
     "{\"name\":\"3-axis Accelerometer\",\"type\":4,\"dataFormat\":2,\"columns\":3,\"sequenceNumbers\":true,\"labels\":[\"x\", \"y\", \"z\"]}";

/**
 * Model JSON configuration for Edge AI Studio
 * - task: TimeSeries_Generic
 * - classIdToName: Maps class indices to labels
 */
static const char gModelJson[] = "{\"name\": \"TimeSeries_Generic_13k_t\",\"task\":\"TimeSeries_Generic\",\"projectId\" :\"Project_Name\",\"classIdToName\":{\"1\": \"Smooth\", \"0\": \"Jerk\"}}";
/** \brief Property: Number of samples for streaming */
static Dap_Interface_PropertyInfoType gPropertySamples = {
    .NamePtr = "samples",
    .Type    = DAP_DATA_FORMAT_UINT16,
    .Value   = {.U16 = DEFAULT_SAMPLE_COUNT}
};

/** \brief Inference value: Predicted class index */
static Dap_Interface_InfValueInfoType gInfValue1 = {
    .NamePtr = "inference_result",
    .Format  = DAP_DATA_FORMAT_INT8
};

/** \brief Inference value: Class 0 (Jerk) probability */
static Dap_Interface_InfValueInfoType gInfValue2 = {
    .NamePtr = "class0_probability",
    .Format  = DAP_DATA_FORMAT_INT8
};

/** \brief Inference value: Class 1 (Smooth) probability */
static Dap_Interface_InfValueInfoType gInfValue3 = {
    .NamePtr = "class1_probability",
    .Format  = DAP_DATA_FORMAT_INT8
};

/** \brief DAP Interface Configuration */
static const Dap_InterfaceConfigType gDapInterfaceConfig = {
    /* Sensors */
    .SensorList  = {gSensorAccJson, NULL, NULL, NULL, NULL, NULL, NULL, NULL},
    .SensorCount = 1U,

    /* Models */
    .ModelList  = {gModelJson, NULL, NULL, NULL},
    .ModelCount = 1U,

    /* Properties */
    .PropertyList  = {&gPropertySamples, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
                      NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL},
    .PropertyCount = 1U,

    /* Inference values for streaming results */
    .InfValueList  = {&gInfValue1, &gInfValue2, &gInfValue3, NULL, NULL, NULL, NULL, NULL},
    .InfValueCount = 3U
};

/** \brief Sample sizes for DAP streaming (3 bytes for inference result) */
static const uint16_t gSampleSizes[DAP_INTERFACE_MAX_SENSORS] = {
    INFERENCE_RESULT_SIZE, 0U, 0U, 0U, 0U, 0U, 0U, 0U
};

/** \brief Sample sizes for raw sensor data streaming (6 bytes for 3-axis accel) */
static const uint16_t gRawSampleSizes[DAP_INTERFACE_MAX_SENSORS] = {
    ACCEL_SAMPLE_SIZE, 0U, 0U, 0U, 0U, 0U, 0U, 0U
};

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/** \brief DAP instance */
static Dap_InstanceType gDapInstance;

/** \brief BMI270 sensor instance */
static TIDA010997_Sensor_Instance_t gBmi270Sensor;

/* -------------------------------------------------------------------------- */
/*                 Feature Extraction Buffers                                 */
/* -------------------------------------------------------------------------- */

/** \brief Scratch buffer for feature extraction */
static float gScratchBuffer[FE_FRAME_SIZE * 4];

/** \brief Raw accelerometer data buffer (3 axes × 128 samples = 384 floats) */
static float gRawInputData[RAW_INPUT_SIZE];

/** \brief History buffer for feature extraction initialization */
static float gHistoryBuffer[MODEL_INPUT_SIZE];

/** \brief Model input buffer (quantized int8) */
static model_input_t gModelInput[MODEL_INPUT_SIZE];

/** \brief Model output buffer */
static model_output_t gModelOutput[FE_NN_OUT_SIZE];

/* -------------------------------------------------------------------------- */
/*                 Test Vector Support                                        */
/* -------------------------------------------------------------------------- */

/** \brief Test vectors from test_vector.c */
extern float raw_input_test[];
extern float model_test_input[];
extern model_output_t golden_output[];

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/**
 * \brief Collect accelerometer samples into raw data buffer
 *
 * Collects FE_FRAME_SIZE samples from each axis into gRawInputData buffer.
 * Data is organized as [X0..X127, Y0..Y127, Z0..Z127]
 *
 * \param[in] sensorPtr Pointer to BMI270 sensor instance
 *
 * \return 0 on success, non-zero on failure
 */
static int32_t collectAccelFrame(TIDA010997_Sensor_Instance_t *sensorPtr)
{
    TIDA010997_Sensor_Data_t dataArray[3];
    uint32_t dataCount;
    int32_t status;
    uint32_t sampleIdx;

    for (sampleIdx = 0U; sampleIdx < FE_FRAME_SIZE; sampleIdx++)
    {
        dataCount = 3U;
        status = TIDA010997_Sensor_getAllData(sensorPtr, dataArray, &dataCount);

        if ((status == TIDA010997_SENSOR_OK) && (dataCount >= 3U))
        {
            /* Store X, Y, Z in separate channel blocks */
            gRawInputData[0 * FE_FRAME_SIZE + sampleIdx] = (float)dataArray[0].rawValue;  /* X */
            gRawInputData[1 * FE_FRAME_SIZE + sampleIdx] = (float)dataArray[1].rawValue;  /* Y */
            gRawInputData[2 * FE_FRAME_SIZE + sampleIdx] = (float)dataArray[2].rawValue;  /* Z */

        }
        else
        {
            return -1;
        }

        /* Delay between samples to match sensor ODR */
        ClockP_usleep(SAMPLE_DELAY_US);
    }

    return 0;
}

/**
 * \brief Run feature extraction on raw data
 *
 * Processes gRawInputData through feature extraction pipeline
 * and produces quantized int8 output in gModelInput.
 *
 * \param[in] feHandle Feature extraction handle
 */
static void runFeatureExtraction(feature_extraction_handle feHandle)
{
    feHandle->input_buffer = &gRawInputData[0];
    feHandle->output_buffer = &gModelInput[0];
    feHandle->test_feature_extraction = false;

    FE_initFeatureExtract(feHandle);
    FE_runFeatureExtract(feHandle);
}

/**
 * \brief Run model inference
 *
 * Runs TVM model on extracted features and stores output in gModelOutput.
 */
static void runInference(void)
{
    struct tvmgen_default_inputs inputs = {(void *)&gModelInput};
    struct tvmgen_default_outputs outputs = {(void *)&gModelOutput};

    tvmgen_default_run(&inputs, &outputs);

#ifdef TVMGEN_DEFAULT_TI_NPU
    extern volatile int32_t tvmgen_default_finished;
    uint32_t timeout = 1000000U;

    while (!tvmgen_default_finished && (timeout > 0U))
    {
        timeout--;
    }

    if (timeout == 0U)
    {
        DebugP_log("[IMU] ERROR: NPU inference timed out!\r\n");
    }
#endif
}

/**
 * \brief Get predicted class from model output
 *
 * \return Predicted class index (0=Jerk, 1=Smooth)
 */
static int8_t getPredictedClass(void)
{
    /* argmax: higher score wins */
    if (gModelOutput[0] > gModelOutput[1])
    {
        return 0;  /* Jerk */
    }
    else
    {
        return 1;  /* Smooth */
    }
}

/**
 * \brief Run test with test vectors to validate implementation
 *
 * \param[in] feHandle Feature extraction handle
 *
 * \return 0 if all tests pass, non-zero otherwise
 */
static int32_t runTestVectorValidation(feature_extraction_handle feHandle)
{
    int error = 0;

    DebugP_log("[IMU] Running test vector validation...\r\n");

    /* Use test vectors */
    feHandle->input_buffer = &raw_input_test[0];
    feHandle->history_buffer = &model_test_input[0];
    feHandle->output_buffer = &gModelInput[0];
    feHandle->test_feature_extraction = false;

    FE_initFeatureExtract(feHandle);
    FE_runFeatureExtract(feHandle);

    /* Run inference */
    runInference();

    /* Compare with golden output */
    error = FE_compareModelOutput(golden_output, gModelOutput);

    if (error == 0)
    {
        DebugP_log("[IMU] Test vector validation PASSED!\r\n");
    }
    else
    {
        DebugP_log("[IMU] Test vector validation FAILED! Mismatches: %d\r\n", error);
    }

    DebugP_log("[IMU] Model output: [%d, %d], Golden: [%d, %d]\r\n",
               gModelOutput[0], gModelOutput[1],
               golden_output[0], golden_output[1]);

    return error;
}

/**
 * \brief Main entry point for IMU sensor data capture example
 *
 * This function initializes the BMI270 sensor, feature extraction, TVM model,
 * and DAP protocol. It then enters a main loop that:
 * - Processes DAP commands from Edge AI Studio
 * - Checks pipeline mode (DATA_ACQUISITION or SENSOR_INFERENCE)
 * - Streams raw sensor data or inference results accordingly
 */
void imu_sensor_data_capture_main(void *args)
{
    int32_t status;
    MCSPI_Handle spiHandle;
    Dap_InitParamsType dapParams;
    TIDA010997_Sensor_InitParams_t sensorParams;
    feature_extraction fe;
    feature_extraction_handle feHandle = &fe;
    bool isStreaming;
    bool streamStarted = false;
    uint32_t inferenceCount = 0U;
    int8_t predictedClass;
    int8_t inferenceResult[INFERENCE_RESULT_SIZE];

    /* Open drivers - UART and SPI configured via SysConfig */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("==========================================================\r\n");
    DebugP_log("[IMU] IMU Sensor Data Capture Example (Dual-Mode DAP)\r\n");
    DebugP_log("==========================================================\r\n");

    /* Get SPI handle for BMI270 communication */
    spiHandle = gMcspiHandle[CONFIG_MCSPI0];

    /* -------------------------------------------------------------------- */
    /*                 Initialize Feature Extraction                        */
    /* -------------------------------------------------------------------- */

    feHandle->scratch_buffer = &gScratchBuffer[0];
    FE_allocFeatureExtract(feHandle);

    /* Initialize history buffer to zeros */
    memset(gHistoryBuffer, 0, sizeof(gHistoryBuffer));
    feHandle->history_buffer = &gHistoryBuffer[0];

    DebugP_log("[IMU] Feature extraction initialized\r\n");
    DebugP_log("[IMU]   Variables: %d (axes)\r\n", FE_VARIABLES);
    DebugP_log("[IMU]   Frame size: %d samples\r\n", FE_FRAME_SIZE);
    DebugP_log("[IMU]   Model input size: %d\r\n", MODEL_INPUT_SIZE);
    DebugP_log("[IMU]   Model output size: %d classes\r\n", FE_NN_OUT_SIZE);

    /* -------------------------------------------------------------------- */
    /*                 Run Test Vector Validation                           */
    /* -------------------------------------------------------------------- */

    status = runTestVectorValidation(feHandle);
    if (status != 0)
    {
        DebugP_log("[IMU] WARNING: Test vector validation failed!\r\n");
    }

    /* -------------------------------------------------------------------- */
    /*                 Initialize BMI270 Accelerometer                      */
    /* -------------------------------------------------------------------- */

    TIDA010997_Sensor_initParamsDefault(&sensorParams, TIDA010997_SENSOR_TYPE_BMI270);
    sensorParams.spiHandle = spiHandle;
    sensorParams.config.bmi270.accelRange  = 2U;    /* +/-8g range */
    sensorParams.config.bmi270.gyroRange   = 0U;    /* 2000 dps (unused) */
    sensorParams.config.bmi270.accelOdr    = 0x08U; /* 100 Hz ODR */
    sensorParams.config.bmi270.gyroOdr     = 0x08U; /* 100 Hz (unused) */
    sensorParams.config.bmi270.accelEnable = true;
    sensorParams.config.bmi270.gyroEnable  = false; /* Accelerometer only */

    status = TIDA010997_Sensor_init(&gBmi270Sensor, &sensorParams);
    if (status != TIDA010997_SENSOR_OK)
    {
        DebugP_log("[IMU] ERROR: BMI270 initialization failed: %d\r\n", status);
        DebugP_log("[IMU] Ensure TIDA-010997 booster pack is connected\r\n");
        goto cleanup;
    }

    DebugP_log("[IMU] BMI270 sensor initialized (Accel: +/-8g, 100Hz)\r\n");

    /* Wait for sensor to stabilize */
    ClockP_usleep(100000U);  /* 100ms */

    /* -------------------------------------------------------------------- */
    /*                 Initialize DAP Protocol                              */
    /* -------------------------------------------------------------------- */

    Dap_InitParamsSetDefault(&dapParams);
    dapParams.LinkParams.UartInstanceIndex = CONFIG_UART0;
    dapParams.LinkParams.LinkTxMode        = DAP_LINK_MODE_INTERRUPT;
    dapParams.LinkParams.LinkRxMode        = DAP_LINK_MODE_INTERRUPT;
    dapParams.InterfaceConfigPtr           = &gDapInterfaceConfig;

    status = Dap_Init(&gDapInstance, &dapParams);
    if (status != DAP_ERROR_NONE)
    {
        DebugP_log("[IMU] ERROR: DAP Init failed: %d\r\n", status);
        goto cleanup;
    }

    status = Dap_Open(&gDapInstance);
    if (status != DAP_ERROR_NONE)
    {
        DebugP_log("[IMU] ERROR: DAP Open failed: %d\r\n", status);
        goto cleanup;
    }

    DebugP_log("[IMU] DAP initialized. Waiting for Edge AI Studio...\r\n");
    DebugP_log("[IMU] Supported modes: DATA_ACQUISITION, SENSOR_INFERENCE\r\n");
    DebugP_log("==========================================================\r\n");

    /* -------------------------------------------------------------------- */
    /*                 Main Processing Loop                                 */
    /* -------------------------------------------------------------------- */

    while (1)
    {
        /* Process DAP commands from Edge AI Studio */
        status = Dap_Process(&gDapInstance);
        if (status != DAP_ERROR_NONE)
        {
            DebugP_log("[IMU] DAP Process error: %d\r\n", status);
        }

        /* Check streaming status */
        isStreaming = false;
        (void)Dap_IsStreaming(&gDapInstance, &isStreaming);

        

        if (isStreaming == true)
        {
            /* Get current pipeline mode from DAP */
            Dap_PipelineConfigType pipelineConfig;
            Dap_GetPipelineConfig(&gDapInstance, &pipelineConfig);

            if (pipelineConfig.Mode == DAP_PIPELINE_MODE_DATA_ACQUISITION)
            {
                /* -------------------------------------------------------- */
                /* DATA ACQUISITION MODE: Stream raw sensor samples         */
                /* -------------------------------------------------------- */
                if (streamStarted == false)
                {
                    uint32_t totalSamples = (uint32_t)gPropertySamples.Value.U16;
                    status = Dap_StartSensorStream(&gDapInstance, gRawSampleSizes,
                                                   sizeof(gRawSampleSizes), totalSamples,
                                                   DAP_DATA_CHANNEL_SENSOR_SIGNAL);
                    if (status == DAP_ERROR_NONE)
                    {
                        streamStarted = true;
                        inferenceCount = 0U;
                        DebugP_log("[IMU] Data acquisition started (%lu samples)\r\n", totalSamples);
                    }
                }

                if (streamStarted == true)
                {
                    /* Read and stream single raw sample */
                    TIDA010997_Sensor_Data_t dataArray[3];
                    uint32_t dataCount = 3U;
                    uint8_t sensorBytes[ACCEL_SAMPLE_SIZE];

                    status = TIDA010997_Sensor_getAllData(&gBmi270Sensor, dataArray, &dataCount);
                    if ((status == TIDA010997_SENSOR_OK) && (dataCount >= 3U))
                    {
                        /* Pack as big-endian INT16 */
                        uint16_t accelX = (uint16_t)(dataArray[0].rawValue & 0xFFFFU);
                        uint16_t accelY = (uint16_t)(dataArray[1].rawValue & 0xFFFFU);
                        uint16_t accelZ = (uint16_t)(dataArray[2].rawValue & 0xFFFFU);

                        sensorBytes[0] = (uint8_t)(accelX >> 8);
                        sensorBytes[1] = (uint8_t)(accelX & 0xFFU);
                        sensorBytes[2] = (uint8_t)(accelY >> 8);
                        sensorBytes[3] = (uint8_t)(accelY & 0xFFU);
                        sensorBytes[4] = (uint8_t)(accelZ >> 8);
                        sensorBytes[5] = (uint8_t)(accelZ & 0xFFU);

                        Dap_StreamSensorSample(&gDapInstance, 0U, sensorBytes, ACCEL_SAMPLE_SIZE);
                    }
                    ClockP_usleep(SAMPLE_DELAY_US);
                }
            }
            else if (pipelineConfig.Mode == DAP_PIPELINE_MODE_SENSOR_INFERENCE)
            {
                /* -------------------------------------------------------- */
                /* SENSOR INFERENCE MODE: Run inference and stream results  */
                /* -------------------------------------------------------- */
                if (streamStarted == false)
                {
                    streamStarted = true;
                    inferenceCount = 0U;
                    DebugP_log("[IMU] Inference streaming started\r\n");
                }

                /* Step 1: Collect accelerometer frame (128 samples × 3 axes) */
                status = collectAccelFrame(&gBmi270Sensor);

                if (status == 0)
                {
                    /* Step 2: Run feature extraction */
                    runFeatureExtraction(feHandle);

                    /* Step 3: Run model inference */
                    runInference();

                    /* Step 4: Get predicted class */
                    predictedClass = getPredictedClass();

                    /* Step 5: Pack inference result [class, prob0, prob1] */
                    inferenceResult[0] = predictedClass;
                    inferenceResult[1] = gModelOutput[0];  /* Jerk score */
                    inferenceResult[2] = gModelOutput[1];  /* Smooth score */

                    status = Dap_StartSensorStream(&gDapInstance,
                                               gSampleSizes,
                                               sizeof(gSampleSizes),
                                               1, DAP_DATA_CHANNEL_INF_VALUE);

                    /* Step 6: Stream to Edge AI Studio */
                    status = Dap_StreamSensorSample(&gDapInstance,
                                                    0U,
                                                    (const uint8_t *)inferenceResult,
                                                    INFERENCE_RESULT_SIZE);

                    if (status != DAP_ERROR_NONE)
                    {
                        DebugP_log("[IMU] Stream sample failed: %d\r\n", status);
                    }

                    inferenceCount++;
                }
                else
                {
                    DebugP_log("[IMU] Sensor read failed\r\n");
                }
                ClockP_usleep(1000U);
            }
        }
        else
        {
            /* Streaming stopped - reset state */
            if (streamStarted == true)
            {
                DebugP_log("[IMU] Streaming stopped. Total samples/inferences: %lu\r\n", inferenceCount);
                streamStarted = false;
            }

            /* Idle - small delay to avoid busy loop */
            ClockP_usleep(1000U);
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

/**
 * \brief UART transmit complete callback for DAP link layer
 */
void Dap_Link_TxCallback(UART_Handle handle, UART_Transaction *trans)
{
    if ((trans != NULL) && (trans->args != NULL))
    {
        Dap_Link_InstanceType *linkPtr = (Dap_Link_InstanceType *)trans->args;
        Dap_Link_ContinueTransmit(linkPtr);
    }
}

/**
 * \brief UART receive complete callback for DAP link layer
 */
void Dap_Link_RxCallback(UART_Handle handle, UART_Transaction *trans)
{
    if ((trans != NULL) && (trans->args != NULL))
    {
        Dap_Link_InstanceType *linkPtr = (Dap_Link_InstanceType *)trans->args;
        uint32_t bytesReceived = trans->count;
        Dap_Link_ContinueReceive(linkPtr, bytesReceived);
    }
}
