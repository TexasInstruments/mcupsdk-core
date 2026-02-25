/*****************************************************************************/
/* Copyright (C) 2026 Texas Instruments Incorporated                         */
/* http://www.ti.com/                                                        */
/*                                                                           */
/*  Redistribution and  use in source  and binary forms, with  or without    */
/*  modification,  are permitted provided  that the  following conditions    */
/*  are met:                                                                 */
/*                                                                           */
/*     Redistributions  of source  code must  retain the  above copyright    */
/*     notice, this list of conditions and the following disclaimer.         */
/*                                                                           */
/*     Redistributions in binary form  must reproduce the above copyright    */
/*     notice, this  list of conditions  and the following  disclaimer in    */
/*     the  documentation  and/or   other  materials  provided  with  the    */
/*     distribution.                                                         */
/*                                                                           */
/*     Neither the  name of Texas Instruments Incorporated  nor the names    */
/*     of its  contributors may  be used to  endorse or  promote products    */
/*     derived  from   this  software  without   specific  prior  written    */
/*     permission.                                                           */
/*                                                                           */
/*  THIS SOFTWARE  IS PROVIDED BY THE COPYRIGHT  HOLDERS AND CONTRIBUTORS    */
/*  "AS IS"  AND ANY  EXPRESS OR IMPLIED  WARRANTIES, INCLUDING,  BUT NOT    */
/*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR    */
/*  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT    */
/*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,    */
/*  SPECIAL,  EXEMPLARY,  OR CONSEQUENTIAL  DAMAGES  (INCLUDING, BUT  NOT    */
/*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,    */
/*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY    */
/*  THEORY OF  LIABILITY, WHETHER IN CONTRACT, STRICT  LIABILITY, OR TORT    */
/*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE    */
/*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.     */
/*                                                                           */
/*****************************************************************************/

#ifndef TVMGEN_DEFAULT_H_
#define TVMGEN_DEFAULT_H_
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Symbol defined when running model on the host processor */
#define TVMGEN_DEFAULT_TI_NPU_SOFT
#ifdef TVMGEN_DEFAULT_TI_NPU
	#error Conflicting definition for where model should run.
#endif

/* The generated model library expects the following inputs/outputs:
 * Inputs:
 *    Tensor[(1, 10, 128, 1), int8]
 * Outputs:
 *    Tensor[(1, 1), float32]
 */

/* Input feature normalization parameters:
 *   input_int = clip(((int32_t)((input_float + bias) * scale)) >> shift, min, max)
 *   where (min, max) = (-128, 127) if int8 type, (0, 255) if uint8 type
 */
#define TVMGEN_DEFAULT_BIAS_LEN 10
#define TVMGEN_DEFAULT_SCALE_LEN 10
#define TVMGEN_DEFAULT_SHIFT_LEN 10
extern const float tvmgen_default_bias_data[] __attribute__((weak)) = {-3.3617374e+01, -3.6871988e-01, -2.4863745e+02,  4.1961697e+01,
 -3.4365898e+01, -1.9320020e+03,  7.4103447e+01, -7.5090698e+01,
 -4.6432278e+01, -3.7818821e+01};
extern const float tvmgen_default_scale_data[] __attribute__((weak)) = { 228., -128., -157., -174.,  205.,  128.,  207.,  154.,  131., -155.};
extern const int32_t tvmgen_default_shift_data[] __attribute__((weak)) = {12,  6, 16,  9, 11, 14, 11,  9, 10, 11};


/*!
 * \brief Input tensor pointers for TVM module "default" 
 */
struct tvmgen_default_inputs {
  void* onnx__Add_0;
};

/*!
 * \brief Output tensor pointers for TVM module "default" 
 */
struct tvmgen_default_outputs {
  void* output;
};

/*!
 * \brief entrypoint function for TVM module "default"
 * \param inputs Input tensors for the module 
 * \param outputs Output tensors for the module 
 */
int32_t tvmgen_default_run(
  struct tvmgen_default_inputs* inputs,
  struct tvmgen_default_outputs* outputs
);

#ifdef __cplusplus
}
#endif

#endif // TVMGEN_DEFAULT_H_
