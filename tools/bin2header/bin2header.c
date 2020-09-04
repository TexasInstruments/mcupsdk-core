/**
 * bin2header.c
 *
*/
/*
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **/
#include <stdio.h>
#include <stdlib.h>

char *license_header = " * \n\
 * \n\
 * Copyright (c) 2014 Texas Instruments Incorporated ALL RIGHTS RESERVED\n\
 * \n\
*/\n\n";


int main(int argc, char *argv[])
{
    FILE *in_fp, *out_fp;
    long file_size = 0, itr = 0;
    int mem_width = 1;


    if(argc < 4)
    {
        printf("Usage : bin2header.exe	input_file output_file out_array_name [1/4]\n");
        return -1;
    }

    if(argc > 4)
    {
        mem_width = atoi(argv[4]);
    }

    if(mem_width != 1)
    {
        mem_width = 4;
    }

    in_fp = fopen(argv[1], "rb");

    if(!in_fp)
    {
        printf("Error opening input image file!\n");
        return -1;
    }

    out_fp = fopen(argv[2], "w");

    if(!out_fp)
    {
        printf("Error opening/creating out image file!\n");
        return -1;
    }

    /* Calculate the size of the input image and rewind to the begin of file */
    fseek(in_fp, 0, SEEK_END);
    file_size = ftell(in_fp);
    printf("file size: %d\n", file_size);
    rewind(in_fp);


    /* Insert the header first */
    fprintf(out_fp, "/* \n * %s\n" , argv[2]);
    fprintf(out_fp, "%s", license_header);

    if(mem_width == 1)
    {
        fprintf(out_fp, "const unsigned char %s[]= {\n", argv[3]);
    }

    else
    {
        fprintf(out_fp, "const unsigned int %s[]= {\n", argv[3]);
    }

    for(itr = 1; itr <= file_size ; itr += mem_width)
    {
        if(mem_width == 1)
        {
            unsigned char temp;
            fread(&temp, sizeof(temp), 1, in_fp);

            if(itr == file_size)
            {
                fprintf(out_fp, "0x%.2x };", temp);
            }

            else
            {
                fprintf(out_fp, "0x%.2x,", temp);
            }

            if((itr % 10) == 0)
            {
                fprintf(out_fp, "\n", temp);
            }
        }

        else
        {
            unsigned int temp;
            fread(&temp, sizeof(temp), 1, in_fp);

            if(itr >= file_size - 4)
            {
                fprintf(out_fp, "0x%.8x };\n", temp);
            }

            else
            {
                fprintf(out_fp, "0x%.8x,\n", temp);
            }

        }

    }

    return 0;

}