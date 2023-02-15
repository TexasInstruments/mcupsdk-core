/*
 *  Copyright (C) 2023 Texas Instruments Incorporated
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
 *
 * This example performs HWA based BFP Compression and Decompression operation
 * along the chirp dimension on complex 1D FFT input data.
 *
 * The input to the compression engine is a complex range-FFT data. This data is
 * arranged as [Chirps ] [Antennas ] [Samples ] dimension.
 * The HWA PaRAM sets are initialized with proper configuration for Compression and
 * Decompression operation.
 *
 * HWA is triggered to perform the Compression operation and Decompression operation.
 * The HWA decompressed output is then compared to the ideal decompressed data.
 * The example also demonstrates how BFP mantissa bitwidth can be configured depending
 * on the desired compression ratio.
 *
 */

#include <string.h>
#include <math.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/CycleCounterP.h>
#include <drivers/hwa.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* test vector */
#include "hwa_bfp_input_testvector.h"
#include "hwa_bfp_output_testvector.h"

#define SAMPLES_PER_BLOCK               8U
#define NUM_SAMPLES                     32U
#define NUM_CHIRPS                      16U
#define NUM_ANTENNAS                    4U
#define COMP_DATA_IN_HWA_SRC_ADDR       CSL_DSS_HWA_DMA0_RAM_BANK0_BASE
#define COMP_DATA_OUT_HWA_DST_ADDR      CSL_DSS_HWA_DMA0_RAM_BANK4_BASE
#define DECOMP_DATA_IN_HWA_SRC_ADDR     COMP_DATA_OUT_HWA_DST_ADDR
#define DECOMP_DATA_OUT_HWA_DST_ADDR    CSL_DSS_HWA_DMA0_RAM_BANK0_BASE
#define MAX_ALLOWED_ERROR 0

HWA_Handle  gHwaHandle;
uint32_t testBFPnumBlocks;

int32_t HWACompression_config(float compRatio)
{
    int32_t     errCode = SystemP_SUCCESS;
    uint8_t     paramsetIdx = 0;
    HWA_ParamConfig     HWATestParamConfig[2];
    HWA_CommonConfig    commonConfig;
    uint8_t     decrImagBitw = 0;
    uint8_t     scaleFacBW = 4U;
    uint8_t     BFPMantissaBW;


    memset(HWATestParamConfig, 0, sizeof(HWATestParamConfig));

    uint32_t cmpInputBytesPerBlock = sizeof(uint32_t) * SAMPLES_PER_BLOCK;
    uint32_t cmpOutputBytesPerBlock = (ceil(cmpInputBytesPerBlock * compRatio / 4.0))* 4U; /* Word aligned */
    float acheivedCompRatio = (float)cmpOutputBytesPerBlock/(float)cmpInputBytesPerBlock;
    uint16_t cmpOuputSamplesPerBlock = cmpOutputBytesPerBlock/sizeof(uint32_t);

#if defined (SOC_AWR294X)
    HWA_Object  *ptrHWADriver = (HWA_Object *)gHwaHandle;
    if(ptrHWADriver->isES2P0Device == true)
    {
        decrImagBitw = 1;
        DebugP_log("Performing Asymmetric Compression\r\n");
    }
    else
    {
        decrImagBitw = 0;
        DebugP_log("Performing Symmetric Compression\r\n");
    }
#else
    decrImagBitw = 0;
    DebugP_log("Performing Symmetric Compression\r\n");
#endif

    BFPMantissaBW = floor((cmpOutputBytesPerBlock*8U - scaleFacBW + decrImagBitw*SAMPLES_PER_BLOCK)/(SAMPLES_PER_BLOCK * 2U));
    DebugP_log("Achieved Compression ratio: %.3f, BFPMantissaBW: %d\r\n", acheivedCompRatio, BFPMantissaBW);

    {{
    /****************************/
    /* BFP Compression Paramset */
    /****************************/
    paramsetIdx = 0;
    HWATestParamConfig[paramsetIdx].triggerMode = HWA_TRIG_MODE_SOFTWARE;
    HWATestParamConfig[paramsetIdx].accelMode = HWA_ACCELMODE_COMPRESS;

    /* SRC configuration */
    HWATestParamConfig[paramsetIdx].source.srcAddr = (uint32_t)HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(COMP_DATA_IN_HWA_SRC_ADDR);
    HWATestParamConfig[paramsetIdx].source.srcAcnt = SAMPLES_PER_BLOCK - 1;
    HWATestParamConfig[paramsetIdx].source.srcAIdx = sizeof(uint32_t);
    HWATestParamConfig[paramsetIdx].source.srcBcnt = testBFPnumBlocks - 1;
    HWATestParamConfig[paramsetIdx].source.srcBIdx = cmpInputBytesPerBlock;
    HWATestParamConfig[paramsetIdx].source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX; //complex data
    HWATestParamConfig[paramsetIdx].source.srcWidth = HWA_SAMPLES_WIDTH_16BIT; //16-bit
    HWATestParamConfig[paramsetIdx].source.srcSign = HWA_SAMPLES_SIGNED; //signed
    HWATestParamConfig[paramsetIdx].source.srcConjugate = HWA_FEATURE_BIT_DISABLE; //no conjugate
    HWATestParamConfig[paramsetIdx].source.srcScale = 0;

    /* DST configuration */
    HWATestParamConfig[paramsetIdx].dest.dstAddr = (uint32_t)HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(COMP_DATA_OUT_HWA_DST_ADDR);
    HWATestParamConfig[paramsetIdx].dest.dstAcnt = cmpOuputSamplesPerBlock- 1;
    HWATestParamConfig[paramsetIdx].dest.dstAIdx = sizeof(uint32_t);
    HWATestParamConfig[paramsetIdx].dest.dstBIdx = cmpOutputBytesPerBlock;
    HWATestParamConfig[paramsetIdx].dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
    HWATestParamConfig[paramsetIdx].dest.dstWidth = HWA_SAMPLES_WIDTH_16BIT;
    HWATestParamConfig[paramsetIdx].dest.dstSign = HWA_SAMPLES_UNSIGNED;
    HWATestParamConfig[paramsetIdx].dest.dstConjugate = HWA_FEATURE_BIT_DISABLE; //no conjugate
    HWATestParamConfig[paramsetIdx].dest.dstScale = 0;
    HWATestParamConfig[paramsetIdx].dest.dstSkipInit = 0; // no skipping

#if defined (SOC_AWR294X)
    if(ptrHWADriver->isES2P0Device == true)
    {
        HWATestParamConfig[paramsetIdx].accelModeArgs.compressMode.cmpRoundEn = HWA_FEATURE_BIT_DISABLE;
        HWATestParamConfig[paramsetIdx].accelModeArgs.compressMode.decrImagBitw = decrImagBitw;
        HWATestParamConfig[paramsetIdx].accelModeArgs.compressMode.selLfsr = HWA_FEATURE_BIT_DISABLE;
    }
#endif

    HWATestParamConfig[paramsetIdx].accelModeArgs.compressMode.ditherEnable = HWA_FEATURE_BIT_DISABLE;
    HWATestParamConfig[paramsetIdx].accelModeArgs.compressMode.compressDecompress = HWA_CMP_DCMP_COMPRESS;
    HWATestParamConfig[paramsetIdx].accelModeArgs.compressMode.method = HWA_COMPRESS_METHOD_BFP;
    HWATestParamConfig[paramsetIdx].accelModeArgs.compressMode.passSelect = HWA_COMPRESS_PATHSELECT_BOTHPASSES;
    HWATestParamConfig[paramsetIdx].accelModeArgs.compressMode.headerEnable = HWA_FEATURE_BIT_ENABLE;
    HWATestParamConfig[paramsetIdx].accelModeArgs.compressMode.scaleFactorBW = scaleFacBW; //log2(sample bits)
    HWATestParamConfig[paramsetIdx].accelModeArgs.compressMode.BFPMantissaBW = BFPMantissaBW;

    errCode = HWA_configParamSet(gHwaHandle, paramsetIdx, &HWATestParamConfig[paramsetIdx], NULL);
    if (errCode != SystemP_SUCCESS)
    {
        DebugP_log("Error: HWA_configParamSet(%d) (Software triggered, AccelMode compress) returned %d\r\n", paramsetIdx, errCode);
        DebugP_log("Feature : API HWA_configParamSet() [Software triggered, AccelMode compress] FAIL\r\n");
        DebugP_assert(errCode == SystemP_SUCCESS);
        return errCode;
    }
    }}

    {{
    /****************************/
    /* BFP Decompression Paramset */
    /****************************/
    paramsetIdx++;
    HWATestParamConfig[paramsetIdx].triggerMode = HWA_TRIG_MODE_IMMEDIATE;
    HWATestParamConfig[paramsetIdx].accelMode = HWA_ACCELMODE_COMPRESS;

    /* SRC configuration */
    HWATestParamConfig[paramsetIdx].source.srcAddr = (uint32_t)HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DECOMP_DATA_IN_HWA_SRC_ADDR);
    HWATestParamConfig[paramsetIdx].source.srcAcnt = cmpOuputSamplesPerBlock - 1;
    HWATestParamConfig[paramsetIdx].source.srcAIdx = sizeof(uint32_t);
    HWATestParamConfig[paramsetIdx].source.srcBcnt = testBFPnumBlocks - 1;
    HWATestParamConfig[paramsetIdx].source.srcBIdx = cmpOutputBytesPerBlock;
    HWATestParamConfig[paramsetIdx].source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
    HWATestParamConfig[paramsetIdx].source.srcWidth = HWA_SAMPLES_WIDTH_16BIT;
    HWATestParamConfig[paramsetIdx].source.srcSign = HWA_SAMPLES_UNSIGNED;
    HWATestParamConfig[paramsetIdx].source.srcConjugate = HWA_FEATURE_BIT_DISABLE; //no conjugate
    HWATestParamConfig[paramsetIdx].source.srcScale = 0;

    /* DST configuration */
    HWATestParamConfig[paramsetIdx].dest.dstAddr = (uint32_t)HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DECOMP_DATA_OUT_HWA_DST_ADDR);
    HWATestParamConfig[paramsetIdx].dest.dstAcnt = SAMPLES_PER_BLOCK - 1;
    HWATestParamConfig[paramsetIdx].dest.dstAIdx = sizeof(uint32_t);
    HWATestParamConfig[paramsetIdx].dest.dstBIdx = cmpInputBytesPerBlock;
    HWATestParamConfig[paramsetIdx].dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
    HWATestParamConfig[paramsetIdx].dest.dstWidth = HWA_SAMPLES_WIDTH_16BIT;
    HWATestParamConfig[paramsetIdx].dest.dstSign = HWA_SAMPLES_SIGNED;
    HWATestParamConfig[paramsetIdx].dest.dstConjugate = HWA_FEATURE_BIT_DISABLE; //no conjugate
    HWATestParamConfig[paramsetIdx].dest.dstScale = 0;
    HWATestParamConfig[paramsetIdx].dest.dstSkipInit = 0; // no skipping

    HWATestParamConfig[paramsetIdx].accelModeArgs.compressMode.ditherEnable = HWA_FEATURE_BIT_DISABLE;
    HWATestParamConfig[paramsetIdx].accelModeArgs.compressMode.compressDecompress = HWA_CMP_DCMP_DECOMPRESS;
    HWATestParamConfig[paramsetIdx].accelModeArgs.compressMode.method = HWA_COMPRESS_METHOD_BFP;
    HWATestParamConfig[paramsetIdx].accelModeArgs.compressMode.passSelect = HWA_COMPRESS_PATHSELECT_BOTHPASSES;
    HWATestParamConfig[paramsetIdx].accelModeArgs.compressMode.headerEnable = HWA_FEATURE_BIT_ENABLE;
    HWATestParamConfig[paramsetIdx].accelModeArgs.compressMode.scaleFactorBW = scaleFacBW;
    HWATestParamConfig[paramsetIdx].accelModeArgs.compressMode.BFPMantissaBW = BFPMantissaBW;

#if defined (SOC_AWR294X)
    if(ptrHWADriver->isES2P0Device == true)
    {
        //Below registers only work for ES2.0 samples
        HWATestParamConfig[paramsetIdx].accelModeArgs.compressMode.cmpRoundEn = HWA_FEATURE_BIT_DISABLE;
        HWATestParamConfig[paramsetIdx].accelModeArgs.compressMode.decrImagBitw = decrImagBitw;
        HWATestParamConfig[paramsetIdx].accelModeArgs.compressMode.selLfsr = HWA_FEATURE_BIT_DISABLE;
    }
#endif

    errCode = HWA_configParamSet(gHwaHandle, paramsetIdx, &HWATestParamConfig[paramsetIdx], NULL);
    if (errCode != 0)
    {
        DebugP_log("Error: HWA_configParamSet(%d) (Immediate triggered, AccelMode decompress) returned %d\r\n", paramsetIdx, errCode);
        DebugP_log("Feature : API HWA_configParamSet() [Immediate triggered, AccelMode decompress] FAIL\r\n");
        DebugP_assert(errCode == SystemP_SUCCESS);
        return errCode;
    }
    }}

    /*****************/
    /* COMMON CONFIG */
    /*****************/

    memset((void *)&commonConfig, 0, sizeof(HWA_CommonConfig));
    commonConfig.configMask = HWA_COMMONCONFIG_MASK_STATEMACHINE_CFG;
#if defined (SOC_AWR294X)
    commonConfig.configMask |= HWA_COMMONCONFIG_MASK_CMP_LFSRSEED0;
    commonConfig.compressConfig.cmpLfsrSeed0 = 123;
#endif

    commonConfig.paramStartIdx = 0 ;
    commonConfig.paramStopIdx = paramsetIdx;
    commonConfig.numLoops = 1;

    errCode = HWA_configCommon(gHwaHandle, &commonConfig);

    return errCode;
}

int32_t compareResults(cmplx16ImRe_t *ideal, cmplx16ImRe_t *obtained, int32_t numSamplesToCompare)
{

    uint32_t i;
    int32_t diffReal, diffImag;
    int32_t result = 1;
    for (i = 0; i < numSamplesToCompare; i++)
    {
        diffReal = ((ideal[i].real - obtained[i].real) > 0) ? (ideal[i].real - obtained[i].real) : -(ideal[i].real - obtained[i].real);
        diffImag = ((ideal[i].imag - obtained[i].imag) > 0) ? (ideal[i].imag - obtained[i].imag) : -(ideal[i].imag - obtained[i].imag);
        if (diffReal > MAX_ALLOWED_ERROR || diffImag > MAX_ALLOWED_ERROR)
        {
            if (diffReal > 2*MAX_ALLOWED_ERROR || diffImag > 2*MAX_ALLOWED_ERROR)
            {
                DebugP_log("Mismatch at idx %d: Ideal %d + i(%d), Obtained %d + i(%d)\r\n", i, ideal[i].real, ideal[i].imag, obtained[i].real, obtained[i].imag);
                result = 0;
            }
            else{
                DebugP_log("High error seen at idx %d: Ideal %d + i(%d), Obtained %d + i(%d)\r\n", i, ideal[i].real, ideal[i].imag, obtained[i].real, obtained[i].imag);
            }
        }
    }
    return result;
}

int32_t runCmpTest(uint8_t* input, cmplx16ImRe_t* idealResult, float compRatio)
{
    static uint16_t testIdx =0;
    int32_t  status = SystemP_SUCCESS, testPass=0;

    DebugP_log("\r\n\n................................\r\n");
    DebugP_log("Running test %d\r\n", testIdx++);
    DebugP_log("NumSamples: %d, NumChirps: %d\r\n", NUM_SAMPLES, NUM_CHIRPS);
    DebugP_log("CompRatio: %.2f, SamplesPerBlock: %d\r\n", compRatio, SAMPLES_PER_BLOCK);

    testBFPnumBlocks = (NUM_SAMPLES * NUM_CHIRPS * NUM_ANTENNAS)/SAMPLES_PER_BLOCK;

    /* Configure HWA */
    status = HWACompression_config(compRatio);
    DebugP_assert(SystemP_SUCCESS == status);
    DebugP_log("HWA cofiguration done successfully\r\n");

    /* copy the entire input data to HWA MEMBANK */
    memcpy((uint8_t*)((uint32_t)COMP_DATA_IN_HWA_SRC_ADDR), (uint8_t*)input, sizeof(uint32_t) * NUM_SAMPLES * NUM_CHIRPS * NUM_ANTENNAS);

    /* Enable the HWA */
    status = HWA_enable(gHwaHandle, 1);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Trigger the HWA */
    HWA_setSoftwareTrigger(gHwaHandle, HWA_TRIG_MODE_SOFTWARE);

    /* Wait for completion */
    ClockP_usleep(1 * 100U);

    DebugP_log("Compression and Decompression Done. Comparing Results.. \r\n");

    /* Compare the output result */
    testPass = compareResults((cmplx16ImRe_t *)((uint32_t)DECOMP_DATA_OUT_HWA_DST_ADDR), (cmplx16ImRe_t*)idealResult, SAMPLES_PER_BLOCK * testBFPnumBlocks);

    if (testPass != 1)
    {
        DebugP_log("Error: Compression/decompression BFP output found incorrect: error %d\r\n", testPass);
    }
    else
    {
        DebugP_log("Debug: Compression/decompression BFP output generated by HWA found correct\r\n");
    }

    /* Disable HWA */
    status = HWA_enable(gHwaHandle, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    return status;
}

void hwa_bfp_compression(void *args)
{
    int32_t  status = SystemP_SUCCESS;


    DebugP_log("\r\n\r\n ------- HWA BFP compression/decompression tests ------- \r\n\r\n");

    /* Initialization */
    Drivers_open();
    Board_driversOpen();
    CycleCounterP_reset(); /* Reset CPU cycle counter */

    /* Open HWA driver */
    gHwaHandle = HWA_open(0, NULL, &status);
    if(gHwaHandle == NULL)
    {
        DebugP_log("Error: Unable to open HWA instance. Error: %d\r\n", status);
        DebugP_assert(gHwaHandle == NULL);
    }
    DebugP_log("HWA Instance has been opened successfully\r\n");

#if defined (SOC_AWR294X)
    HWA_Object  *ptrHWADriver = (HWA_Object *)gHwaHandle;
    if(ptrHWADriver->isES2P0Device == true)
    {
        runCmpTest((uint8_t*)(&gHWATest_compressBFP_input1[0]), (cmplx16ImRe_t*)(&gHWATest_decompBFP_output1_2p1[0]), compRatio[0]);
        runCmpTest((uint8_t*)(&gHWATest_compressBFP_input1[0]), (cmplx16ImRe_t*)(&gHWATest_decompBFP_output2_2p1[0]), compRatio[1]);
        runCmpTest((uint8_t*)(&gHWATest_compressBFP_input1[0]), (cmplx16ImRe_t*)(&gHWATest_decompBFP_output3_2p1[0]), compRatio[2]);
    }
    else
    {
        runCmpTest((uint8_t*)(&gHWATest_compressBFP_input1[0]), (cmplx16ImRe_t*)(&gHWATest_decompBFP_output1[0]), compRatio[0]);
        runCmpTest((uint8_t*)(&gHWATest_compressBFP_input1[0]), (cmplx16ImRe_t*)(&gHWATest_decompBFP_output2[0]), compRatio[1]);
        runCmpTest((uint8_t*)(&gHWATest_compressBFP_input1[0]), (cmplx16ImRe_t*)(&gHWATest_decompBFP_output3[0]), compRatio[2]);
    }
#else

    runCmpTest((uint8_t*)(&gHWATest_compressBFP_input1[0]), (cmplx16ImRe_t*)(&gHWATest_decompBFP_output1[0]), compRatio[0]);
    runCmpTest((uint8_t*)(&gHWATest_compressBFP_input1[0]), (cmplx16ImRe_t*)(&gHWATest_decompBFP_output2[0]), compRatio[1]);
    runCmpTest((uint8_t*)(&gHWATest_compressBFP_input1[0]), (cmplx16ImRe_t*)(&gHWATest_decompBFP_output3[0]), compRatio[2]);

#endif

    /* HWA driver close */
    HWA_controlPeripheralSuspendMode(gHwaHandle, 0);
    status += HWA_close(gHwaHandle);
    DebugP_assert(SystemP_SUCCESS == status);

    if(status == SystemP_SUCCESS)
    {
        DebugP_log("\r\n\nHWA BFP Compression/Decompression Test Completed!!\r\n");
    }

    Board_driversClose();
    Drivers_close();
}
