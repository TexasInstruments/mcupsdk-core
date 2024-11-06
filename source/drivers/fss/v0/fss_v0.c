
/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
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


#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <drivers/hw_include/hw_types.h>
#include <drivers/hw_include/csl_types.h>
#include <kernel/dpl/DebugP.h>
#include "fss.h"

#define HWREG(x)                                                               \
        (*((volatile uint32_t *)(x)))

#define MSS_OSPI_BOOT_CONFIG_MASK              (0x840ul)
#define MSS_OSPI_BOOT_CONFIG_SEG               (0x844ul)

int32_t FSS_addressBitMask(FSS_Handle handle, uint32_t bitMask, uint8_t segment)
{
    int32_t ret = SystemP_FAILURE;

    if(handle != NULL)
    {
        FSS_Config *config = (FSS_Config *)handle;
        if(bitMask != 0)
        {
            HWREG(config->ipBaseAddress + MSS_OSPI_BOOT_CONFIG_MASK) = ~(bitMask >> 12);
            HWREG(config->ipBaseAddress + MSS_OSPI_BOOT_CONFIG_SEG) = (segment * (bitMask + 1)) >> 12;
        }
        else
        {
            HWREG(config->ipBaseAddress + MSS_OSPI_BOOT_CONFIG_MASK) = 0;
            HWREG(config->ipBaseAddress + MSS_OSPI_BOOT_CONFIG_SEG) = 0;
        }
        ret = SystemP_SUCCESS;
    }

    return ret;
}

int32_t FSS_selectRegionA(FSS_Handle handle)
{
    int32_t ret = SystemP_FAILURE;
    if(handle != NULL)
    {
        FSS_Config *config = (FSS_Config *)handle;
        if((config->extFlashSize & (config->extFlashSize - 1)) == 0)
        {
            ret = FSS_addressBitMask(handle, (config->extFlashSize / 2) - 1, 0);
        }
    }
    return ret;
}

int32_t FSS_selectRegionB(FSS_Handle handle)
{
    int32_t ret = SystemP_FAILURE;
    if(handle != NULL)
    {
        FSS_Config *config = (FSS_Config *)handle;
        if((config->extFlashSize & (config->extFlashSize - 1)) == 0)
        {
            ret = FSS_addressBitMask(handle, (config->extFlashSize / 2) - 1, 1);
        }
    }
    return ret;
}

int32_t FSS_disableAddressRemap(FSS_Handle handle)
{
    return FSS_addressBitMask(handle, 0, 0);
}


#define MAX_SIZE 103

static void FSS_ECCConvertBytesToBits(uint8_t* bytes, int* bits, size_t size)
{
    for (size_t i = 0; i < size; i++)
    {
        for (int j = 0; j < 8; j++)
        {
            bits[i * 8 + j] = (bytes[i] >> (7 - j)) & 1;
        }
    }
}

static uint32_t FSS_ECCCalculateRegionSize(uint32_t sizeInBytes)
{
    // If sizeInBytes is less than 4KB, return 1
    if (sizeInBytes < 4096)
    {
        return 1;
    }
    // Calculate the region size based on the provided logic
    return ((sizeInBytes -1) / 4096) + 1;
}

static uint8_t FSS_generateECCSyndrom(int *lbit)
{
    int ecc0, ecc1, ecc2, ecc3, ecc4, ecc5, ecc6, ecc7;

    ecc0 = (lbit[0] ^ lbit[1] ^ lbit[2] ^ lbit[4] ^ lbit[5] ^ lbit[7] ^ lbit[10] ^ lbit[11] ^ lbit[12] ^ lbit[14] ^ lbit[17] ^ lbit[18] ^ lbit[21] ^ lbit[23] ^ lbit[24] ^ lbit[26] ^ lbit[27] ^ lbit[29] ^ lbit[32] ^ lbit[33] ^ lbit[36] ^ lbit[38] ^ lbit[39] ^ lbit[41] ^ lbit[44] ^ lbit[46] ^ lbit[47] ^ lbit[50] ^ lbit[51] ^ lbit[53] ^ lbit[56] ^ lbit[57] ^ lbit[58] ^ lbit[60] ^ lbit[63] ^ lbit[64] ^ lbit[67] ^ lbit[69] ^ lbit[70] ^ lbit[72] ^ lbit[75] ^ lbit[77] ^ lbit[78] ^ lbit[81] ^ lbit[82] ^ lbit[84] ^ lbit[87] ^ lbit[88] ^ lbit[91] ^ lbit[93] ^ lbit[94] ^ lbit[97] ^ lbit[98] ^ lbit[100]);

    ecc1 = (lbit[0] ^ lbit[1] ^ lbit[3] ^ lbit[4] ^ lbit[6] ^ lbit[8] ^ lbit[10] ^ lbit[11] ^ lbit[13] ^ lbit[15] ^ lbit[17] ^ lbit[19] ^ lbit[21] ^ lbit[23] ^ lbit[25] ^ lbit[26] ^ lbit[28] ^ lbit[30] ^ lbit[32] ^ lbit[34] ^ lbit[36] ^ lbit[38] ^ lbit[40] ^ lbit[42] ^ lbit[44] ^ lbit[46] ^ lbit[48] ^ lbit[50] ^ lbit[52] ^ lbit[54] ^ lbit[56] ^ lbit[57] ^ lbit[59] ^ lbit[61] ^ lbit[63] ^ lbit[65] ^ lbit[67] ^ lbit[69] ^ lbit[71] ^ lbit[73] ^ lbit[75] ^ lbit[77] ^ lbit[79] ^ lbit[81] ^ lbit[83] ^ lbit[85] ^ lbit[87] ^ lbit[89] ^ lbit[91] ^ lbit[93] ^ lbit[95] ^ lbit[97] ^ lbit[99] ^ lbit[101]);

    ecc2 = (lbit[0] ^ lbit[2] ^ lbit[3] ^ lbit[5] ^ lbit[6] ^ lbit[9] ^ lbit[10] ^ lbit[12] ^ lbit[13] ^ lbit[16] ^ lbit[17] ^ lbit[20] ^ lbit[21] ^ lbit[24] ^ lbit[25] ^ lbit[27] ^ lbit[28] ^ lbit[31] ^ lbit[32] ^ lbit[35] ^ lbit[36] ^ lbit[39] ^ lbit[40] ^ lbit[43] ^ lbit[44] ^ lbit[47] ^ lbit[48] ^ lbit[51] ^ lbit[52] ^ lbit[55] ^ lbit[56] ^ lbit[58] ^ lbit[59] ^ lbit[62] ^ lbit[63] ^ lbit[66] ^ lbit[67] ^ lbit[70] ^ lbit[71] ^ lbit[74] ^ lbit[75] ^ lbit[78] ^ lbit[79] ^ lbit[82] ^ lbit[83] ^ lbit[86] ^ lbit[87] ^ lbit[90] ^ lbit[91] ^ lbit[94] ^ lbit[95] ^ lbit[98] ^ lbit[99] ^ lbit[102]);

    ecc3 = (lbit[1] ^ lbit[2] ^ lbit[3] ^ lbit[7] ^ lbit[8] ^ lbit[9] ^ lbit[10] ^ lbit[14] ^ lbit[15] ^ lbit[16] ^ lbit[17] ^ lbit[22] ^ lbit[23] ^ lbit[24] ^ lbit[25] ^ lbit[29] ^ lbit[30] ^ lbit[31] ^ lbit[32] ^ lbit[37] ^ lbit[38] ^ lbit[39] ^ lbit[40] ^ lbit[45] ^ lbit[46] ^ lbit[47] ^ lbit[48] ^ lbit[53] ^ lbit[54] ^ lbit[55] ^ lbit[56] ^ lbit[60] ^ lbit[61] ^ lbit[62] ^ lbit[63] ^ lbit[68] ^ lbit[69] ^ lbit[70] ^ lbit[71] ^ lbit[76] ^ lbit[77] ^ lbit[78] ^ lbit[79] ^ lbit[84] ^ lbit[85] ^ lbit[86] ^ lbit[87] ^ lbit[92] ^ lbit[93] ^ lbit[94] ^ lbit[95] ^ lbit[100] ^ lbit[101] ^ lbit[102]);

    ecc4 = (lbit[4] ^ lbit[5] ^ lbit[6] ^ lbit[7] ^ lbit[8] ^ lbit[9] ^ lbit[10] ^ lbit[18] ^ lbit[19] ^ lbit[20] ^ lbit[21] ^ lbit[22] ^ lbit[23] ^ lbit[24] ^ lbit[25] ^ lbit[33] ^ lbit[34] ^ lbit[35] ^ lbit[36] ^ lbit[37] ^ lbit[38] ^ lbit[39] ^ lbit[40] ^ lbit[49] ^ lbit[50] ^ lbit[51] ^ lbit[52] ^ lbit[53] ^ lbit[54] ^ lbit[55] ^ lbit[56] ^ lbit[64] ^ lbit[65] ^ lbit[66] ^ lbit[67] ^ lbit[68] ^ lbit[69] ^ lbit[70] ^ lbit[71] ^ lbit[80] ^ lbit[81] ^ lbit[82] ^ lbit[83] ^ lbit[84] ^ lbit[85] ^ lbit[86] ^ lbit[87] ^ lbit[96] ^ lbit[97] ^ lbit[98] ^ lbit[99] ^ lbit[100] ^ lbit[101] ^ lbit[102]);

    ecc5 = (lbit[11] ^ lbit[12] ^ lbit[13] ^ lbit[14] ^ lbit[15] ^ lbit[16] ^ lbit[17] ^ lbit[18] ^ lbit[19] ^ lbit[20] ^ lbit[21] ^ lbit[22] ^ lbit[23] ^ lbit[24] ^ lbit[25] ^ lbit[41] ^ lbit[42] ^ lbit[43] ^ lbit[44] ^ lbit[45] ^ lbit[46] ^ lbit[47] ^ lbit[48] ^ lbit[49] ^ lbit[50] ^ lbit[51] ^ lbit[52] ^ lbit[53] ^ lbit[54] ^ lbit[55] ^ lbit[56] ^ lbit[72] ^ lbit[73] ^ lbit[74] ^ lbit[75] ^ lbit[76] ^ lbit[77] ^ lbit[78] ^ lbit[79] ^ lbit[80] ^ lbit[81] ^ lbit[82] ^ lbit[83] ^ lbit[84] ^ lbit[85] ^ lbit[86] ^ lbit[87]);

    ecc6 = (lbit[26] ^ lbit[27] ^ lbit[28] ^ lbit[29] ^ lbit[30] ^ lbit[31] ^ lbit[32] ^ lbit[33] ^ lbit[34] ^ lbit[35] ^ lbit[36] ^ lbit[37] ^ lbit[38] ^ lbit[39] ^ lbit[40] ^ lbit[41] ^ lbit[42] ^ lbit[43] ^ lbit[44] ^ lbit[45] ^ lbit[46] ^ lbit[47] ^ lbit[48] ^ lbit[49] ^ lbit[50] ^ lbit[51] ^ lbit[52] ^ lbit[53] ^ lbit[54] ^ lbit[55] ^ lbit[56] ^ lbit[88] ^ lbit[89] ^ lbit[90] ^ lbit[91] ^ lbit[92] ^ lbit[93] ^ lbit[94] ^ lbit[95] ^ lbit[96] ^ lbit[97] ^ lbit[98] ^ lbit[99] ^ lbit[100] ^ lbit[101] ^ lbit[102]);

    ecc7 = (lbit[57] ^ lbit[58] ^ lbit[59] ^ lbit[60] ^ lbit[61] ^ lbit[62] ^ lbit[63] ^ lbit[64] ^ lbit[65] ^ lbit[66] ^ lbit[67] ^ lbit[68] ^ lbit[69] ^ lbit[70] ^ lbit[71] ^ lbit[72] ^ lbit[73] ^ lbit[74] ^ lbit[75] ^ lbit[76] ^ lbit[77] ^ lbit[78] ^ lbit[79] ^ lbit[80] ^ lbit[81] ^ lbit[82] ^ lbit[83] ^ lbit[84] ^ lbit[85] ^ lbit[86] ^ lbit[87] ^ lbit[88] ^ lbit[89] ^ lbit[90] ^ lbit[91] ^ lbit[92] ^ lbit[93] ^ lbit[94] ^ lbit[95] ^ lbit[96] ^ lbit[97] ^ lbit[98] ^ lbit[99] ^ lbit[100] ^ lbit[101] ^ lbit[102]);

    int binword1 = (ecc3 << 3) | (ecc2 << 2) | (ecc1 << 1) | ecc0;
    int binword2 = (ecc7 << 3) | (ecc6 << 2) | (ecc5 << 1) | ecc4;

    char* ecc=(char *)malloc(3* sizeof(char));
    sprintf(ecc, "%01X%01X", binword2, binword1);

    return (uint8_t)strtol(ecc,NULL,16);

}

void FSS_enableECC()
{
    HW_WR_FIELD32(CSL_FSS_FSAS_GENREGS_REGS_BASE + CSL_FSS_FSAS_GENREGS_SYSCONFIG, CSL_FSS_FSAS_GENREGS_SYSCONFIG_ECC_EN, 1);
    HW_WR_FIELD32(CSL_FSS_FSAS_GENREGS_REGS_BASE + CSL_FSS_FSAS_GENREGS_SYSCONFIG, CSL_FSS_FSAS_GENREGS_SYSCONFIG_ECC_DISABLE_ADR,1);
}

void FSS_disableECC()
{
    HW_WR_FIELD32(CSL_FSS_FSAS_GENREGS_REGS_BASE + CSL_FSS_FSAS_GENREGS_SYSCONFIG, CSL_FSS_FSAS_GENREGS_SYSCONFIG_ECC_EN, 0);
}

// Set ECC Region Start using region index
void FSS_setECCRegionStart(uint32_t regionIndex, uint32_t ecc_reg_start)
{
    uint32_t regOffset = CSL_FSS_FSAS_GENREGS_ECC_REGCTRL_ECC_RGSTRT(regionIndex);
    HW_WR_FIELD32(CSL_FSS_FSAS_GENREGS_REGS_BASE + regOffset, CSL_FSS_FSAS_GENREGS_ECC_REGCTRL_ECC_RGSTRT_R_START, (ecc_reg_start)/0x1000);
}

/*Set ECC Region Size using region index*/
void FSS_setECCRegionSize(uint32_t regionIndex, uint32_t size_in_bytes)
{
    uint32_t ecc_reg_size=FSS_ECCCalculateRegionSize(size_in_bytes);
    uint32_t regOffset = CSL_FSS_FSAS_GENREGS_ECC_REGCTRL_ECC_RGSIZ(regionIndex);
    HW_WR_FIELD32(CSL_FSS_FSAS_GENREGS_REGS_BASE + regOffset, CSL_FSS_FSAS_GENREGS_ECC_REGCTRL_ECC_RGSIZ_R_SIZE, ecc_reg_size);
}

int32_t FSS_configECCMRegion(FSS_ECCRegionConfig *parameter)
{
    int32_t status = SystemP_FAILURE;
    if(NULL != parameter)
    {
        uint32_t regOffset = CSL_FSS_FSAS_GENREGS_ECC_REGCTRL_ECC_RGSTRT(parameter->regionIndex);
        uint32_t ecc_reg_size =FSS_ECCCalculateRegionSize(parameter->size);
        uint32_t regOffset1 = CSL_FSS_FSAS_GENREGS_ECC_REGCTRL_ECC_RGSIZ(parameter->regionIndex);
        HW_WR_FIELD32(CSL_FSS_FSAS_GENREGS_REGS_BASE + regOffset, CSL_FSS_FSAS_GENREGS_ECC_REGCTRL_ECC_RGSTRT_R_START, (parameter->startAddress)/0x1000);
        HW_WR_FIELD32(CSL_FSS_FSAS_GENREGS_REGS_BASE + regOffset1, CSL_FSS_FSAS_GENREGS_ECC_REGCTRL_ECC_RGSIZ_R_SIZE, ecc_reg_size);
        HW_WR_FIELD32(CSL_FSS_FSAS_GENREGS_REGS_BASE + regOffset1, CSL_FSS_FSAS_GENREGS_ECC_REGCTRL_ECC_RGSIZ_R_SIZE, ecc_reg_size);
        status = SystemP_SUCCESS;
    }
    return status;
}

int32_t FSS_ConfigEccm(uint32_t numRegion, FSS_ECCRegionConfig *parameter)
{
    int32_t status = SystemP_FAILURE;
    if(parameter != NULL)
    {
        FSS_disableECC();
        for(int i=0; i < numRegion; i++)
        {
            status = FSS_configECCMRegion(&parameter[i]);
        }
        FSS_enableECC();
        status = SystemP_SUCCESS;
    }
    return status;
}

static uint32_t FSS_appendECC(uint8_t *input_buffer, uint32_t sizet, uint8_t *output_buffer,uint8_t gmac[])
{
    size_t input_index = 0, output_index = 0;
    while (input_index < sizet)
    {
        uint8_t bytes[32];
        memcpy(bytes, &input_buffer[input_index], 32);

        // ECC P1 calculation
        int comb_b1_bits[64], comb_b2_bits[32], comb_b3_bits[8];

        // Combine bytes 0 to 7
        uint8_t comb1[8] = { bytes[7], bytes[6], bytes[5], bytes[4], bytes[3], bytes[2], bytes[1], bytes[0] };
        FSS_ECCConvertBytesToBits(comb1, comb_b1_bits, 8);

        // Combine bytes 8 to 11
        uint8_t comb2[4] = { bytes[11], bytes[10], bytes[9], bytes[8] };
        FSS_ECCConvertBytesToBits(comb2, comb_b2_bits, 4);

        // Byte 12
        FSS_ECCConvertBytesToBits(bytes + 12, comb_b3_bits, 1);
        memmove(comb_b3_bits, comb_b3_bits + 1, 7 * sizeof(int)); // Remove MSB from comb_b3_bits

        int final_arr[MAX_SIZE];
        memcpy(final_arr, comb_b3_bits, 7 * sizeof(int));
        memcpy(final_arr + 7, comb_b2_bits, 32 * sizeof(int));
        memcpy(final_arr + 39, comb_b1_bits, 64 * sizeof(int));

        // Reverse final_arr
        for (int i = 0; i < MAX_SIZE / 2; i++)
        {
            int temp = final_arr[i];
            final_arr[i] = final_arr[MAX_SIZE - i - 1];
            final_arr[MAX_SIZE - i - 1] = temp;
        }

        uint8_t ecc_p1 = FSS_generateECCSyndrom(final_arr);
        //printf("%02X\n", ecc_p1);


        // ECC P2 calculation
        int comb_b4_bits[8], comb_b5_bits[64], comb_b6_bits[32], comb_b7_bits[8];

        // Byte 12
        FSS_ECCConvertBytesToBits(bytes + 12, comb_b4_bits, 1);
        memmove(comb_b4_bits, comb_b4_bits, 1 * sizeof(int)); // Remove all but MSB from comb_b4_bits

        // Combine bytes 13 to 20
        uint8_t comb5[8] = { bytes[20], bytes[19], bytes[18], bytes[17], bytes[16], bytes[15], bytes[14], bytes[13] };
        FSS_ECCConvertBytesToBits(comb5, comb_b5_bits, 8);

        // Combine bytes 21 to 24
        uint8_t comb6[4] = { bytes[24], bytes[23], bytes[22], bytes[21] };
        FSS_ECCConvertBytesToBits(comb6, comb_b6_bits, 4);

        // Byte 25
        FSS_ECCConvertBytesToBits(bytes + 25, comb_b7_bits, 1);
        memmove(comb_b7_bits, comb_b7_bits + 2, 6 * sizeof(int)); // Remove 2 MSBs from comb_b7_bits

        memcpy(final_arr, comb_b7_bits, 6 * sizeof(int));
        memcpy(final_arr + 6, comb_b6_bits, 32 * sizeof(int));
        memcpy(final_arr + 38, comb_b5_bits, 64 * sizeof(int));
        memcpy(final_arr + 102, comb_b4_bits, 1 * sizeof(int));

        // Reverse final_arr
        for (int i = 0; i < MAX_SIZE / 2; i++)
        {
            int temp = final_arr[i];
            final_arr[i] = final_arr[MAX_SIZE - i - 1];
            final_arr[MAX_SIZE - i - 1] = temp;
        }

        uint8_t ecc_p2 = FSS_generateECCSyndrom(final_arr);
        //printf("%02X\n", ecc_p2);

            // ECC P3 calculation
            int comb_b8_bits[8], comb_b9_bits[64], comb_b10_bits[32], comb_b11_bits[8];
            FSS_ECCConvertBytesToBits(bytes + 25, comb_b8_bits, 1);
            memmove(comb_b8_bits, comb_b8_bits , 2 * sizeof(int)); // Remove 2 MSBs from comb_b8_bits

            uint8_t comb9[8] = {gmac[1], gmac[0], bytes[31], bytes[30], bytes[29], bytes[28], bytes[27], bytes[26]};
            FSS_ECCConvertBytesToBits(comb9, comb_b9_bits, 8);

            uint8_t comb10[4] = {gmac[5], gmac[4], gmac[3], gmac[2]};
            FSS_ECCConvertBytesToBits(comb10, comb_b10_bits, 4);

            FSS_ECCConvertBytesToBits(&gmac[6], comb_b11_bits, 1);
            memmove(comb_b11_bits, comb_b11_bits + 3, 5 * sizeof(int)); // Remove 3 MSBs from comb_b11_bits

            memcpy(final_arr, comb_b11_bits, 5 * sizeof(int));
            memcpy(final_arr + 5, comb_b10_bits, 32 * sizeof(int));
            memcpy(final_arr + 37, comb_b9_bits, 64 * sizeof(int));
            memcpy(final_arr + 101, comb_b8_bits, 2 * sizeof(int));

            // Reverse final_arr
            for (int i = 0; i < MAX_SIZE / 2; i++)
            {
                int temp = final_arr[i];
                final_arr[i] = final_arr[MAX_SIZE - i - 1];
                final_arr[MAX_SIZE - i - 1] = temp;
            }

            uint8_t ecc_p3 = FSS_generateECCSyndrom(final_arr);
            //printf("%02X\n", ecc_p3);
            int comb_b12_bits[8], comb_b13_bits[64], comb_b14_bits[32], comb_b15_bits[8];
            FSS_ECCConvertBytesToBits(&gmac[6], comb_b12_bits, 1);
            memmove(comb_b12_bits, comb_b12_bits , 3 * sizeof(int)); // Remove 2 MSBs from comb_b8_bits

            uint8_t comb13[8] = {gmac[14], gmac[13], gmac[12], gmac[11], gmac[10], gmac[9], gmac[8], gmac[7]};
            FSS_ECCConvertBytesToBits(comb13, comb_b13_bits, 8);

            uint8_t comb14[4] = {0, 0, 0, gmac[15]};
            FSS_ECCConvertBytesToBits(comb14, comb_b14_bits, 4);

            uint8_t comb15[1]={0};
            FSS_ECCConvertBytesToBits(comb15, comb_b15_bits, 1);
            memmove(comb_b15_bits, comb_b15_bits + 3, 5 * sizeof(int)); // Remove 3 MSBs from comb_b11_bits

            memcpy(final_arr, comb_b15_bits, 5 * sizeof(int));
            memcpy(final_arr + 5, comb_b14_bits, 32 * sizeof(int));
            memcpy(final_arr + 37, comb_b13_bits, 64 * sizeof(int));
            memcpy(final_arr + 101, comb_b12_bits, 2 * sizeof(int));

            // Reverse final_arr
            for (int i = 0; i < MAX_SIZE / 2; i++)
            {
                int temp = final_arr[i];
                final_arr[i] = final_arr[MAX_SIZE - i - 1];
                final_arr[MAX_SIZE - i - 1] = temp;
            }

            uint8_t ecc_p4 = FSS_generateECCSyndrom(final_arr);

            // Append ECC to the block and write to output buffer
            memcpy(&output_buffer[output_index], bytes, 32);

            output_buffer[output_index+32]= ecc_p1;
            output_buffer[output_index+33]= ecc_p2;
            output_buffer[output_index+34]= ecc_p3;
            output_buffer[output_index+35]= ecc_p4;

            output_index +=36;
            input_index += 32;
    }
    return output_index;
}