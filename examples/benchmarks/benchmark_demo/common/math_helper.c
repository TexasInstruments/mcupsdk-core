/* ----------------------------------------------------------------------
* Copyright (C) 2010-2012 ARM Limited. All rights reserved.
*
* $Date:        17. January 2013
* $Revision: 	V1.4.0  b
*
* Project: 	    CMSIS DSP Library
*
* Title:	    math_helper.c
*
* Description:	Definition of all helper functions required.
*
* Target Processor: Cortex-M4/Cortex-M3
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*   - Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   - Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in
*     the documentation and/or other materials provided with the
*     distribution.
*   - Neither the name of ARM LIMITED nor the names of its contributors
*     may be used to endorse or promote products derived from this
*     software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
* -------------------------------------------------------------------- */

/* ----------------------------------------------------------------------
*		Include standard header files
* -------------------------------------------------------------------- */
#include<math.h>

/* ----------------------------------------------------------------------
*		Include project header files
* -------------------------------------------------------------------- */
#include "math_helper.h"

/**
 * @brief  Caluclation of SNR
 * @param[in]  pRef 	Pointer to the reference buffer
 * @param[in]  pTest	Pointer to the test buffer
 * @param[in]  buffSize	total number of samples
 * @return     SNR
 * The function Caluclates signal to noise ratio for the reference output
 * and test output
 */

float arm_snr_f32(float *pRef, float *pTest, uint32_t buffSize)
{
  float EnergySignal = 0.0, EnergyError = 0.0;
  uint32_t i;
  float SNR;
  int temp;
  int *test;

  for (i = 0; i < buffSize; i++)
    {
 	  /* Checking for a NAN value in pRef array */
	  test =   (int *)(&pRef[i]);
      temp =  *test;

	  if (temp == 0x7FC00000)
	  {
	  		return(0);
	  }

	  /* Checking for a NAN value in pTest array */
	  test =   (int *)(&pTest[i]);
      temp =  *test;

	  if (temp == 0x7FC00000)
	  {
	  		return(0);
	  }
      EnergySignal += pRef[i] * pRef[i];
      EnergyError += (pRef[i] - pTest[i]) * (pRef[i] - pTest[i]);
    }

	/* Checking for a NAN value in EnergyError */
	test =   (int *)(&EnergyError);
    temp =  *test;

    if (temp == 0x7FC00000)
    {
  		return(0);
    }


  SNR = 10 * log10 (EnergySignal / EnergyError);

  return (SNR);

}
