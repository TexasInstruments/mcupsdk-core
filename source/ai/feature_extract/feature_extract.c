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


#ifdef __TMS320C2000__
    #include "dsp.h"
    #include "fpu32/fpu_fft_hann.h"
    #include "fpu32/C28x_FPU_FastRTS.h"
#endif

#ifndef __TMS320C2000__
    #include "fpu_fft_hann.h"
#endif

#include "feature_extract.h"
#include "tvmgen_default.h"

/* Standard library includes */
#include <string.h>
#include <stdlib.h>
#include <math.h>

#if defined(FE_FFT)
extern void FE_cfft(float *fft_input_buffer, float *fft_output_buffer, uint16_t frame_size, uint16_t fft_stages);
#endif

/* ============================================================================
 * CONSTANTS & DATA
 * ============================================================================ */
#if defined(FE_WIN)
#if FE_FRAME_SIZE == 32
    const float hann[] = HANN32;
#elif FE_FRAME_SIZE == 64
    const float hann[] = HANN64;
#elif FE_FRAME_SIZE == 128
    const float hann[] = HANN128;
#elif FE_FRAME_SIZE == 256
    const float hann[] = HANN256;
#elif FE_FRAME_SIZE == 512
    const float hann[] = HANN512;
#elif FE_FRAME_SIZE == 1024
    const float hann[] = HANN1024;
#elif FE_FRAME_SIZE == 2048
    const float hann[] = HANN2048;
#else
    const float hann[] = {0};
#endif
#else
    const float hann[] = {0};
#endif

#if defined(FE_FFT)
void FE_fft(float *fft_input_buffer, float *fft_output_buffer, uint16_t frame_size, uint16_t fft_stages){
    FE_cfft(fft_input_buffer, fft_output_buffer, frame_size, fft_stages);
    return;
}
#endif

#if defined(FE_WIN)
void FE_getHanningWindow(float *hanning_window, uint16_t frame_size)
{
    if (hanning_window == NULL)
        return;
    memcpy(hanning_window, hann, frame_size * sizeof(float) / 2);
    return;
}

void FE_windowing(float *input_buffer, float *hanning_window, uint16_t frame_size)
{
    if (input_buffer == NULL || hanning_window == NULL || frame_size == 0)
        return;

    uint16_t index;
    uint16_t mask = frame_size - 1;
    for (index = 0; index < frame_size; index++)
    {
        /* Select correct window index using symmetric Hann window */
        uint16_t window_idx = (index < (frame_size >> 1)) ? index : (mask - index);
        input_buffer[index] = hanning_window[window_idx] * input_buffer[index];
    }
    return;
}
#endif

#if defined(FE_BIN)
void FE_bin(float *input_buffer, uint16_t feature_size_per_frame, uint16_t minimum_bin_size, uint16_t bin_size, uint16_t normalize_bin, uint16_t array_len)
{
    if (input_buffer == NULL || feature_size_per_frame == 0 || bin_size == 0)
        return;

    float bin_sum;
    uint16_t bin_idx, idx, bin_start, bin_end;

    for (bin_idx = 0; bin_idx < feature_size_per_frame; bin_idx++)
    {
        bin_sum = 0.0f;
        bin_start = minimum_bin_size + (bin_idx * bin_size);
        bin_end = bin_start + bin_size;

        /* Sum values in current bin with bounds checking */
        for (idx = bin_start; idx < bin_end; idx++)
        {
            bin_sum += input_buffer[idx];
        }

        /* Normalize or store raw sum */
        if (normalize_bin == 1)
        {
            input_buffer[bin_idx] = bin_sum / bin_size;
        }
        else
        {
            input_buffer[bin_idx] = bin_sum;
        }
    }
    return;
}
#endif

#if defined(FE_NORMALIZE)
void FE_fftNormalize(float *input_buffer, uint16_t frame_size)
{
    if (input_buffer == NULL || frame_size == 0)
        return;

    uint16_t idx;
    float normalization_factor = 1.0f / frame_size;
    for (idx = 0; idx < frame_size; idx++)
    {
        input_buffer[idx] *= normalization_factor;
    }
    return;
}
#endif

#if defined(FE_LOG)
void FE_log(float *input_buffer, uint16_t feature_size_per_frame, float log_mul, float log_threshold, float log_base)
{
    if (input_buffer == NULL || feature_size_per_frame == 0)
        return;

    uint16_t i;
    float scaler = log_mul;

    /* Calculate scaling factor */
    if (log_base > 0.0f)
        scaler /= logf(log_base);

    for (i = 0; i < feature_size_per_frame; i++)
    {
        float log_input = log_threshold + input_buffer[i];

        /* Ensure positive value for logarithm */
        if (log_input < 0.0f)
            log_input = -log_input;

        input_buffer[i] = scaler * logf(log_input);
    }
    return;
}
#endif

int FE_compareModelInput(model_input_t* expected_array, model_input_t* obtained_array)
{
    int error = 0, idx = 0;
    for(idx=0;idx<FE_STACKING_CHANNELS*FE_STACKING_FRAME_WIDTH;idx++){
    #if defined(SKIP_NORMALIZE)
        if(abs(expected_array[idx] - obtained_array[idx]) > 1)
            error++;
    #else
        if(fabsf(expected_array[idx] - obtained_array[idx]) > 1)
            error++;
    #endif
    }
    return error;
}

int FE_compareModelOutput(model_output_t* expected_array, model_output_t* obtained_array)
{
    int error = 0, idx = 0;
    for(idx=0;idx<FE_NN_OUT_SIZE;idx++){
    #if defined(OUTPUT_INT)
        if(abs(expected_array[idx] - obtained_array[idx]) > 1)
            error++;
    #else
        if(fabsf(expected_array[idx] - obtained_array[idx]) > 1)
            error++;
    #endif
    }
    return error;
}

void FE_batchNormalization_npu(float *input_buffer, int8_t *output_buffer, float bias_val, float scale_val, int32_t shift_val, uint16_t feature_size)
{
    if (input_buffer == NULL || output_buffer == NULL || feature_size == 0)
        return;

    uint16_t index;
    for (index = 0; index < feature_size; index++)
    {
        /* Apply batch normalization: (input + bias) * scale >> shift */
        float normalized = input_buffer[index] + bias_val;
        int32_t scaled = (int32_t)floorf(normalized * scale_val);
        int32_t shifted = scaled >> shift_val;

        /* Clamp to int8_t range [-128, 127] */
        if (shifted > 127)
            output_buffer[index] = 127;
        else if (shifted < -128)
            output_buffer[index] = -128;
        else
            output_buffer[index] = (int8_t)shifted;
    }
    return;
}

void FE_batchNormalization_cpu(float *input_buffer, model_input_t *output_buffer, float scale_val, int32_t zero_point_val, uint16_t feature_size)
{
    if (input_buffer == NULL || output_buffer == NULL || feature_size == 0)
        return;

    uint16_t index;
    for (index = 0; index < feature_size; index++)
    {
        /* Apply batch normalization: round(input / q_scale) + q_zero_point */
        float scaled = input_buffer[index] * scale_val;
        int32_t round = (int32_t)floorf(scaled + 0.5f);
        int32_t shifted = round + zero_point_val;

        /* Clamp to model_input_t range */
#if defined(SKIP_NORMALIZE)
        /* int8_t range [-128, 127] */
        if (shifted > 127)
            output_buffer[index] = 127;
        else if (shifted < -128)
            output_buffer[index] = -128;
        else
            output_buffer[index] = (model_input_t)shifted;
#else
        /* uint8_t range [0, 255] */
        if (shifted > 255)
            output_buffer[index] = 255;
        else if (shifted < 0)
            output_buffer[index] = 0;
        else
            output_buffer[index] = (model_input_t)shifted;
#endif
    }
    return;
}

void FE_allocFeatureExtract(feature_extraction_handle fe)
{
    if (fe == NULL)
        return;
    fe->size_required_by_library = FE_FRAME_SIZE * 4 * sizeof(float);
    return;
}

void FE_initFeatureExtract(feature_extraction_handle fe)
{
    if (fe == NULL || fe->history_buffer == NULL || fe->output_buffer == NULL)
        return;

    void* history_buffer = fe->history_buffer;
    void* output_buffer = fe->output_buffer;
    uint16_t total_feature_size_per_frame = fe->test_feature_extraction ? 0 : FE_FEATURE_SIZE_PER_FRAME;

    int num_channel = 0;

    for (num_channel = 0; num_channel < FE_STACKING_CHANNELS; num_channel++)
    {
        float *bn_input = (float *)history_buffer + (num_channel * FE_STACKING_FRAME_WIDTH);
        model_input_t *bn_output = (model_input_t *)output_buffer + ((num_channel * FE_STACKING_FRAME_WIDTH) + total_feature_size_per_frame);
#ifdef SKIP_NORMALIZE
    #ifdef TVMGEN_DEFAULT_BIAS_LEN
        float bias_val = tvmgen_default_bias_data[num_channel % TVMGEN_DEFAULT_BIAS_LEN];
        float scale_val = tvmgen_default_scale_data[num_channel % TVMGEN_DEFAULT_SCALE_LEN];
        int32_t shift_val = tvmgen_default_shift_data[num_channel % TVMGEN_DEFAULT_SHIFT_LEN];
        FE_batchNormalization_npu(bn_input, bn_output, bias_val, scale_val, shift_val, FE_STACKING_FRAME_WIDTH - total_feature_size_per_frame);
    #else
        float scale_val = tvmgen_default_input_reciprocal_scale_data[0];
        int32_t zero_point_val = tvmgen_default_input_zero_point_data[0];
        FE_batchNormalization_cpu(bn_input, bn_output, scale_val, zero_point_val, FE_STACKING_FRAME_WIDTH - total_feature_size_per_frame);
    #endif
#else
        memcpy(bn_output, bn_input, (FE_STACKING_FRAME_WIDTH - total_feature_size_per_frame) * sizeof(float));
#endif
    }
    return;
}


void FE_runFeatureExtract(feature_extraction_handle fe)
{
    if (fe == NULL || fe->input_buffer == NULL || fe->output_buffer == NULL || fe->scratch_buffer == NULL)
        return;

    void* input_buffer = fe->input_buffer;
    void* output_buffer = fe->output_buffer;
    float* scratch_buffer = fe->scratch_buffer;

    model_input_t *output_ptr = (model_input_t *)output_buffer;

    int num_channel = 0;
    for (num_channel = 0; num_channel < (FE_VARIABLES); num_channel++)
    {
        /* Copy input frame to scratch buffer */
        memcpy(scratch_buffer, ((float*)input_buffer) + (num_channel * FE_FRAME_SIZE), FE_FRAME_SIZE * sizeof(float));

#if defined(FE_RAW)
        /* Mean removal preprocessing */
        float accumulated_sum = 0.0f;
        int idx;
        for (idx = 0; idx < FE_FRAME_SIZE; idx++)
        {
            accumulated_sum += scratch_buffer[idx];
        }
        float mean = accumulated_sum / FE_FRAME_SIZE;

        for (idx = 0; idx < FE_FRAME_SIZE; idx++)
        {
            scratch_buffer[idx] -= mean;
        }
#endif

#if defined(FE_WIN)
        /* Apply Hanning window */
        float *window_input = &scratch_buffer[0];
        float *hanning_window = &scratch_buffer[FE_FRAME_SIZE];
        FE_getHanningWindow(hanning_window, FE_FRAME_SIZE);
        FE_windowing(window_input, hanning_window, FE_FRAME_SIZE);
#endif

#if defined(FE_FFT)
        /* Perform Complex FFT */
        float *fft_input = &scratch_buffer[0];
        float *fft_output = &scratch_buffer[FE_FRAME_SIZE * 2];
        FE_fft(fft_input, fft_output, FE_FRAME_SIZE, FE_FFT_STAGES);
#endif

#if defined(FE_NORMALIZE)
        /* Normalize FFT output */
        float *fft_normalize_input = &scratch_buffer[0];
        FE_fftNormalize(fft_normalize_input, FE_FRAME_SIZE);
#endif

#if defined(FE_DC_REM)
        /* Remove DC component */
        float *buffer = &scratch_buffer[0];
        uint16_t output_size = FE_FRAME_SIZE / 2 + 1;
        memcpy(buffer, buffer + 1, output_size * sizeof(float));
        buffer[FE_FRAME_SIZE / 2] = 0.0f;
#endif

#if defined(FE_BIN)
        /* Frequency binning */
        float *bin_input = &scratch_buffer[0];
        FE_bin(bin_input, FE_FEATURE_SIZE_PER_FRAME, FE_MIN_FFT_BIN, FE_FFT_BIN_SIZE, FE_BIN_NORMALIZE, FE_FRAME_SIZE / 2);
#endif

#if defined(FE_LOG)
        /* Log scaling */
        float *log_input = &scratch_buffer[0];
        FE_log(log_input, FE_FEATURE_SIZE_PER_FRAME, FE_LOG_MUL, FE_LOG_TOL, FE_LOG_BASE);
#endif

#if defined(FE_CONCAT)
        /* Frame concatenation with sliding window */
        uint16_t starting_index = num_channel * FE_FEATURE_SIZE_PER_FRAME * FE_NUM_FRAME_CONCAT;
        model_input_t *sliding_window = &output_ptr[0];
        memcpy(&sliding_window[starting_index], 
               &sliding_window[starting_index + FE_FEATURE_SIZE_PER_FRAME], 
               (FE_NUM_FRAME_CONCAT - 1) * FE_FEATURE_SIZE_PER_FRAME * sizeof(model_input_t));
        
        float *bn_input = &scratch_buffer[0];
        model_input_t *bn_output = &output_ptr[((num_channel + 1) * (FE_FEATURE_SIZE_PER_FRAME * FE_NUM_FRAME_CONCAT)) - FE_FEATURE_SIZE_PER_FRAME];
        
#if defined(SKIP_NORMALIZE)
    #ifdef TVMGEN_DEFAULT_BIAS_LEN
        float bias_val = tvmgen_default_bias_data[num_channel % TVMGEN_DEFAULT_BIAS_LEN];
        float scale_val = tvmgen_default_scale_data[num_channel % TVMGEN_DEFAULT_SCALE_LEN];
        int32_t shift_val = tvmgen_default_shift_data[num_channel % TVMGEN_DEFAULT_SHIFT_LEN];
        FE_batchNormalization_npu(bn_input, bn_output, bias_val, scale_val, shift_val, FE_FEATURE_SIZE_PER_FRAME);
    #else
        float scale_val = tvmgen_default_input_reciprocal_scale_data[0];
        int32_t zero_point_val = tvmgen_default_input_zero_point_data[0];
        FE_batchNormalization_cpu(bn_input, bn_output, scale_val, zero_point_val, FE_FEATURE_SIZE_PER_FRAME);
    #endif
#else
        memcpy(bn_output, bn_input, FE_FEATURE_SIZE_PER_FRAME * sizeof(model_input_t));
#endif

#else
        /* Direct output without concatenation */
        memcpy(&output_ptr[num_channel * FE_STACKING_FRAME_WIDTH], &scratch_buffer[0], FE_STACKING_FRAME_WIDTH * sizeof(model_input_t));
#endif
    }
    return;
}
