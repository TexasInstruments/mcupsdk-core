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
 * This example shows the use of feature_extract library files to process
 * fan blade anomaly detection dataset and perform AI inference using a pre-trained model.
 * The feature extraction library extracts relevant features from raw input
 * data and prepares the input for the model.
 */

#include <stdlib.h>
#include <math.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

#include "feature_extract.h"
#include "tvmgen_default.h"

float scratch_buffer[FE_FRAME_SIZE * 4];

/* test case from test vector */
extern float raw_input_test[];
extern float model_test_input[];
extern model_output_t golden_output[];

/* NN model output */
#define ANOMALY 1
#define NORMAL 0
int test_result;
#define TEST_FEATURE_EXTRACT

model_input_t model_input[1 * FE_STACKING_CHANNELS * FE_STACKING_FRAME_WIDTH * 1];
#if defined(TEST_FEATURE_EXTRACT)
model_input_t test_feature_extraction[1 * FE_STACKING_CHANNELS * FE_STACKING_FRAME_WIDTH * 1];
#endif

model_output_t model_output[FE_NN_OUT_SIZE];

void fan_blade_anomalydetection_main(void *args)
{
    feature_extraction fe;
    feature_extraction_handle fe_handle = &fe;
    int error = 0;

    /* Open drivers */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("Fan Blade Anomaly Detection Example Started ...\r\n");

    fe_handle->scratch_buffer = &scratch_buffer[0];
    FE_allocFeatureExtract(fe_handle);

    fe_handle->input_buffer = &raw_input_test[0];
    fe_handle->history_buffer = &model_test_input[0];

#if defined(TEST_FEATURE_EXTRACT)
    fe_handle->output_buffer = &test_feature_extraction[0];
    fe_handle->test_feature_extraction = true;
    FE_initFeatureExtract(fe_handle);
#endif
    fe_handle->output_buffer = &model_input[0];
    fe_handle->test_feature_extraction = false;
    FE_initFeatureExtract(fe_handle);

    FE_runFeatureExtract(fe_handle);

#if defined(TEST_FEATURE_EXTRACT)
    error = FE_compareModelInput(test_feature_extraction, model_input);
    DebugP_log("Feature extraction mismatches %d\r\n", error);
#endif

    struct tvmgen_default_inputs inputs = {(void *)&model_input};
    struct tvmgen_default_outputs outputs = {(void *)&model_output};

#ifdef TVMGEN_DEFAULT_TI_NPU
    extern void TI_NPU_init();
    TI_NPU_init();
#endif
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
        DebugP_log("ERROR: NPU inference timed out!\r\n");
    }
#endif

    int output_index = 0;
    float reconstruction_error = 0;
    for (output_index = 0; output_index < FE_NN_OUT_SIZE; output_index++)
    {
        float difference = fabsf(golden_output[output_index] - model_output[output_index]);
        if (difference > 4)
        {
            error++;
            DebugP_log("Difference at index[%d] = %d\r\n", output_index, (int)(difference));
        }

        /* calculate the reconstruction error */
        float diff = model_input[output_index] - model_output[output_index];
        reconstruction_error += diff * diff;
    }
    reconstruction_error /= FE_NN_OUT_SIZE;
    if (reconstruction_error > RECONSTRUCTION_ERROR_THRESHOLD)
    {
        test_result = ANOMALY;
    }
    else
    {
        test_result = NORMAL;
    }

    DebugP_log("Golden vectors matched: %d not matched: %d\r\n", FE_NN_OUT_SIZE - error, error);
    DebugP_log("Reconstruction error is %d, Threshold is %d\r\n", (int)reconstruction_error, (int)RECONSTRUCTION_ERROR_THRESHOLD);
    DebugP_log("The test sample is a %s sample\r\n", test_result == ANOMALY ? "Anomaly" : "Normal");
    DebugP_log("All tests have passed!!\r\n");
    Board_driversClose();
    Drivers_close();
}
