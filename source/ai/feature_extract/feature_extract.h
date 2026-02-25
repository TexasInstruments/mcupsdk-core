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
 * \addtogroup ai_fe_group AI - Feature Extract APIs
 * 
 * @{
 */

/**
 * \file feature_extract.h
  * 
  * \brief
  * Header file contains enumerations, structure definitions and functions
  * declarations for Edge AI Feature Extract Module interface
  */

#ifndef FEATURE_EXTRACT_H_
#define FEATURE_EXTRACT_H_

// Peripheral includes
#include "math.h"
#if defined(__TMS320C2000__) || defined(__C29__)
#include "driverlib.h"
#else
#include "stdint.h"
#include "stdbool.h"
#endif

#include "user_input_config.h"
#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief Model Input
 * 
 * Model Input type
 */
#if defined(SKIP_NORMALIZE)
typedef int8_t model_input_t;
#else
typedef float model_input_t;
#endif

/**
 * \brief Model Output
 * 
 * Model Output type
 */
#if defined(OUTPUT_INT)
typedef int8_t model_output_t;
#else
typedef float model_output_t;
#endif

/**
 * \struct feature_extraction
 * \brief Feature extraction context structure
 * 
 * Contains buffers and configuration for feature extraction operations
 */
typedef struct {
    void* input_buffer;                 /**< Pointer to input buffer address on which feature extraction will be performed */
    void* output_buffer;                /**< Pointer to output buffer address to store extracted features */
    void* history_buffer;               /**< Pointer to history buffer address if need to initialize history */
    void* scratch_buffer;               /**< Pointer to scratch buffer address for computation */
    bool test_feature_extraction;       /**< Toggle to test feature extraction from test vector */
    int32_t size_required_by_library;   /**< Size required by FE Library (used by scratch buffer) */
} feature_extraction;

typedef feature_extraction* feature_extraction_handle;

/**
 * \brief Calculates required memory allocation for feature extraction
 * 
 * \param fe Feature extraction handle
 * 
 * Sets the required memory size for feature extraction operations
 * Size is calculated as 2x frame size in float precision
 * \return None
 * 
 */
void FE_allocFeatureExtract(feature_extraction_handle fe);

/**
 * \brief Initializes feature extraction state and buffers
 * 
 * \param fe Feature extraction handle
 * 
 * Initializes history buffer and output buffer for feature extraction
 * Handles both normalized and quantized output modes
 * Processes each channel separately for multi-channel data
 * \return None
 * 
 */
void FE_initFeatureExtract(feature_extraction_handle fe);

/**
 * \brief Main feature extraction processing function
 * 
 * \param fe Feature extraction handle containing all buffers and state
 * 
 * Performs complete feature extraction pipeline including:
 * - Raw signal processing (optional mean removal)
 * - Windowing (optional)
 * - FFT computation (optional)
 * - FFT normalization (optional)
 * - DC removal (optional)
 * - Frequency binning (optional)
 * - Log scaling (optional)
 * - Frame concatenation (optional)
 * - Batch normalization or direct copy (based on configuration)
 * 
 * Processes each channel independently and supports both normalized
 * float output and quantized int8_t output modes
 * \return None
 * 
 */
void FE_runFeatureExtract(feature_extraction_handle fe);

/**
 * \brief Compares feature extraction buffers
 * 
 * \param expected_array Expected output from feature extraction
 * \param obtained_array Obtained output from feature extraction
 * 
 * Takes the difference of the two arrays and returns the number of
 * elements that were not matched.
 * \return number of mismatches in two array
 * 
 */
int FE_compareModelInput(model_input_t* expected_array, model_input_t* obtained_array);

/**
 * \brief Compares nn model buffers
 * 
 * \param expected_array Expected output from nn model
 * \param obtained_array Obtained output from nn model
 * 
 * Takes the difference of the two arrays and returns the number of
 * elements that were not matched.
 * \return number of mismatches in two array
 * 
 */
int FE_compareModelOutput(model_output_t* expected_array, model_output_t* obtained_array);

/** @} */
#ifdef __cplusplus
}
#endif  /* extern "C" */

#endif /* FEATURE_EXTRACT_H_ */
