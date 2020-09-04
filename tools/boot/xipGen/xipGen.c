/*
 *  Copyright (C) 2018-2021 Texas Instruments Incorporated
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <getopt.h>



/* default values, can be over-ridden using command line args */
#define XIP_REGION_START    (0x60000000U)
#define XIP_REGION_SIZE_MB  (256U)

/* reasonable default, dont except larger numbers here */
#define MAX_SECTIONS        (10000U)
#define MAX_FILE_NAME       (4*1024)

#define RPRC_HEADER_TAG     (0x43525052U)

/* RPRC file header */
typedef struct {

    uint32_t magicWord; /* MUST be equal to RPRC_HEADER_TAG */
    uint32_t entryPoint;
    uint32_t rsv0;
    uint32_t sectionCount;
    uint32_t version;

} rprc_header_t;

/* RPRC section header */
typedef struct {

    uint32_t runAddress;
    uint32_t rsv0;
    uint32_t size;
    uint32_t rsv1;
    uint32_t rsv2;
} rprc_section_t;

typedef struct {

    void *inputBuf;
    uint32_t inputBufSize;

    rprc_header_t nonXipRprcHeader;
    rprc_header_t nonXipRprcMergedHeader;

    rprc_section_t nonXipRprcSections[MAX_SECTIONS];
    rprc_section_t nonXipRprcMergedSections[MAX_SECTIONS];

    rprc_header_t xipRprcHeader;
    rprc_header_t xipRprcMergedHeader;

    rprc_section_t xipRprcSections[MAX_SECTIONS];
    rprc_section_t xipRprcMergedSections[MAX_SECTIONS];

    uint32_t xipAddrStart;
    uint32_t xipRegionSize;

    char inputFileName[MAX_FILE_NAME];
    char outputFileNameNonXip[MAX_FILE_NAME];
    char outputFileNameXip[MAX_FILE_NAME];

    uint32_t verbose;

} xip_ctrl_t;

xip_ctrl_t gXipCtrl;

/* reads input file into gXipCtrl.inputBuf and sets gXipCtrl.inputBufSize to input file size */
void readInputFile(char *filename)
{
    uint32_t size, readSize;
    FILE *fp = fopen(filename, "rb");

    if(fp==NULL)
    {
        printf("ERROR: [%s] input file not found !!!\n", filename);
        exit(0);
    }

    /* get file size */
    fseek(fp, 0, SEEK_END);
    size = ftell(fp);
    fseek(fp, 0, SEEK_SET);

    /* alloc memory for input file */
    gXipCtrl.inputBuf = malloc(size);
    if(gXipCtrl.inputBuf == NULL)
    {
        printf("ERROR: Unable to allocate %d bytes for reading input file !!!\n", size);
        exit(0);
    }

    /* rad all the input data in one go */
    readSize = fread(gXipCtrl.inputBuf, 1, size, fp);
    if(readSize!=size)
    {
        printf("ERROR: Unable to read %d bytes from input file !!!\n", size);
        exit(0);
    }

    /* update file size, we will use this later for error checking */
    gXipCtrl.inputBufSize = size;

    fclose(fp);
}

/* get a little endian 32b word give a uint8_t * pointer */
uint32_t getWord32(uint8_t *ptr)
{
    return ((uint32_t)ptr[3] << 24) | ((uint32_t)ptr[2] << 16) | ((uint32_t)ptr[1] << 8) | ((uint32_t)ptr[0] << 0);
}

/* set nice defaults in RPRC header */
void rprcHeaderInit(rprc_header_t *header)
{
    header->magicWord = RPRC_HEADER_TAG;
    header->entryPoint = 0;
    header->rsv0 = 0;
    header->sectionCount = 0;
    header->version = 0;
}

/* check if given section falls in XIP address range */
uint32_t isXipSection(rprc_section_t *sectionHeader)
{
    if(sectionHeader->runAddress >= gXipCtrl.xipAddrStart &&
        (sectionHeader->runAddress + sectionHeader->size) <= (gXipCtrl.xipAddrStart + gXipCtrl.xipRegionSize)
        )
    {
        return 1;
    }
    return 0;
}

/* parse input RPRC file and count the XIP and non-XIP sections */
void parseInputFile(char *filename)
{
    rprc_header_t rprcHeader;

    if(gXipCtrl.verbose)
    {
        printf("[%s] Parsing file ...\n", filename);
    }

    readInputFile(filename);

    uint8_t *ptr = gXipCtrl.inputBuf;
    uint32_t value = getWord32(ptr);

    /* check for RPRC header tag */
    if(value != RPRC_HEADER_TAG)
    {
        printf("ERROR: Invalid input file. RPRC header not found 0x%08x != 0x%08x !!!\n", value, RPRC_HEADER_TAG);
        exit(0);
    }

    /* parse RPRC input file and count XIP and non-XIP sections, also make the
       XIP and non-XIP headers that will go into the output files
     */
    rprcHeaderInit(&gXipCtrl.nonXipRprcHeader);
    rprcHeaderInit(&gXipCtrl.xipRprcHeader);

    memcpy(&rprcHeader, ptr, sizeof(rprc_header_t));
    ptr += sizeof(rprc_header_t);

    gXipCtrl.nonXipRprcHeader.version = rprcHeader.version;
    gXipCtrl.nonXipRprcHeader.entryPoint = rprcHeader.entryPoint;
    gXipCtrl.xipRprcHeader.version = rprcHeader.version;

    for(uint32_t i  = 0; i < rprcHeader.sectionCount; i++)
    {
        rprc_section_t sectionHeader;

        memcpy(&sectionHeader, ptr, sizeof(rprc_section_t));
        ptr += sizeof(rprc_section_t);

        if( isXipSection(&sectionHeader) )
        {
            /* we collect the XIP section into a array, since in next step, we merge adjoning sections
               This will help in simplifying the flashing process
             */
            gXipCtrl.xipRprcSections[gXipCtrl.xipRprcHeader.sectionCount] = sectionHeader;

            /* convert to offset in flash */
            gXipCtrl.xipRprcSections[gXipCtrl.xipRprcHeader.sectionCount].runAddress = sectionHeader.runAddress - gXipCtrl.xipAddrStart;

            gXipCtrl.xipRprcHeader.sectionCount++;
        }
        else
        {
            gXipCtrl.nonXipRprcSections[gXipCtrl.nonXipRprcHeader.sectionCount] = sectionHeader;

            gXipCtrl.nonXipRprcHeader.sectionCount++;
        }

        if(gXipCtrl.verbose)
        {
            printf("Found section %d with adddress 0x%08x, size 0x%08x bytes\n",
                i,
                sectionHeader.runAddress,
                sectionHeader.size
                );
        }

        ptr += sectionHeader.size;

        if( ptr > (uint8_t*)( (uint32_t*)gXipCtrl.inputBuf + gXipCtrl.inputBufSize) )
        {
            printf("ERROR: Invalid input file. Some data missing in file\n");
            exit(0);
        }
    }

    if(gXipCtrl.verbose)
    {
        printf("[%s] Found %d sections. %d are non-XIP sections, %d are XIP sections, NON-XIP entryPoint = 0x%08x\n",
            filename,
            rprcHeader.sectionCount,
            gXipCtrl.nonXipRprcHeader.sectionCount,
            gXipCtrl.xipRprcHeader.sectionCount,
            gXipCtrl.nonXipRprcHeader.entryPoint
            );
    }
}

/* check if 'cur' section is adjacent to 'mergedCur' section, if yes then we can merge these sections into one */
uint32_t isRprcSectionMerged(rprc_section_t *cur, rprc_section_t *mergedCur)
{
    if( (mergedCur->runAddress + mergedCur->size) == (cur->runAddress) )
    {
        return 1;
    }
    return 0;
}

/* we merge the XIP section stored in gXipCtrl.xipRprcSections and keep the merged result in
   gXipCtrl.xipRprcMergedSections. gXipCtrl.xipRprcMergedHeader holds the header after merging

   Merging helps us flash in contigous chunks when possible.
   Better to merge here now than in flash writer later.
 */
void mergeXipSections()
{
    uint32_t mergedSectionCount = 0;

    rprcHeaderInit(&gXipCtrl.xipRprcMergedHeader);

    /* nothing to merge */
    if(gXipCtrl.xipRprcHeader.sectionCount == 0 )
        return; /* nothing to do */

    /* first section will be un merged */
    gXipCtrl.xipRprcMergedSections[mergedSectionCount] = gXipCtrl.xipRprcSections[mergedSectionCount];
    mergedSectionCount++;

    for(uint32_t i = 1; i < gXipCtrl.xipRprcHeader.sectionCount; i++)
    {
        rprc_section_t *cur, *mergedCur;

        mergedCur = &gXipCtrl.xipRprcMergedSections[mergedSectionCount-1];
        cur = &gXipCtrl.xipRprcSections[i];

        if( isRprcSectionMerged(cur, mergedCur) )
        {
            if(gXipCtrl.verbose)
            {
                printf("Merging section @ 0x%08x of size 0x%08x, with section @ 0x%08x of size 0x%08x\n",
                    cur->runAddress,
                    cur->size,
                    mergedCur->runAddress,
                    mergedCur->size
                    );
            }
            /* add new section size to the merged section size */
            mergedCur->size += cur->size;
        }
        else
        {
            /* nothing to merge, treat as new section */
            gXipCtrl.xipRprcMergedSections[mergedSectionCount] = gXipCtrl.xipRprcSections[i];
            mergedSectionCount++;
        }
    }

    gXipCtrl.xipRprcMergedHeader.sectionCount = mergedSectionCount;
}

void mergeNonXipSections()
{
    uint32_t mergedSectionCount = 0;

    rprcHeaderInit(&gXipCtrl.nonXipRprcMergedHeader);

    /* nothing to merge */
    if(gXipCtrl.nonXipRprcHeader.sectionCount == 0 )
        return; /* nothing to do */

    /* first section will be un merged */
    gXipCtrl.nonXipRprcMergedSections[mergedSectionCount] = gXipCtrl.nonXipRprcSections[mergedSectionCount];
    mergedSectionCount++;

    for(uint32_t i = 1; i < gXipCtrl.nonXipRprcHeader.sectionCount; i++)
    {
        rprc_section_t *cur, *mergedCur;

        mergedCur = &gXipCtrl.nonXipRprcMergedSections[mergedSectionCount-1];
        cur = &gXipCtrl.nonXipRprcSections[i];

        if( isRprcSectionMerged(cur, mergedCur) )
        {
            if(gXipCtrl.verbose)
            {
                printf("Merging section @ 0x%08x of size 0x%08x, with section @ 0x%08x of size 0x%08x\n",
                    cur->runAddress,
                    cur->size,
                    mergedCur->runAddress,
                    mergedCur->size
                    );
            }
            /* add new section size to the merged section size */
            mergedCur->size += cur->size;
        }
        else
        {
            /* nothing to merge, treat as new section */
            gXipCtrl.nonXipRprcMergedSections[mergedSectionCount] = gXipCtrl.nonXipRprcSections[i];
            mergedSectionCount++;
        }
    }

    gXipCtrl.nonXipRprcMergedHeader.sectionCount = mergedSectionCount;
    gXipCtrl.nonXipRprcMergedHeader.version = gXipCtrl.nonXipRprcHeader.version;
    gXipCtrl.nonXipRprcMergedHeader.entryPoint = gXipCtrl.nonXipRprcHeader.entryPoint;
}

/* writer section header to given file */
void writeSectionHeader(FILE *fp, char *filename, uint32_t sectionNum, rprc_section_t *section)
{
    uint32_t wrSize;

    if(gXipCtrl.verbose)
    {
        printf("Writing section header  #%d, adddress 0x%08x, size 0x%08x bytes to file [%s]\n",
            sectionNum,
            section->runAddress,
            section->size,
            filename
            );
    }

    wrSize = fwrite(section, 1, sizeof(rprc_section_t), fp);
    if(wrSize!=sizeof(rprc_section_t))
    {
        printf("ERROR: Unable to write %d section header to output file [%s]\n", sectionNum, filename);
        exit(0);
    }
}

/* writer RPRC header to given file */
void writeRprcHeader(FILE *fp, char *filename, rprc_header_t *header)
{
    uint32_t wrSize;

    wrSize = fwrite(header, 1, sizeof(rprc_header_t),  fp);
    if(wrSize!=sizeof(rprc_header_t))
    {
        printf("ERROR: Unable to write RPRC header to output file [%s]\n", filename);
        exit(0);
    }
}

/* create output RPRC files, one for non-XIP section and one for XIP sections.
   The XIP output file has its section mergeded when two sections are exactly next to each other
 */
void createOutputFiles(char *nonXipOutFilename, char *xipOutFilename)
{
    uint8_t *ptr = gXipCtrl.inputBuf;
    rprc_header_t rprcHeader;
    uint32_t wrSize;
    uint32_t mergedXipSectionCount = 0, xipSectionCount = 0, sectionNum = 0;
    uint32_t mergedNonXipSectionCount = 0, nonXipSectionCount = 0;

    FILE *fp_nonXip = fopen(nonXipOutFilename, "wb");
    FILE *fp_xip = fopen(xipOutFilename, "wb");

    if(fp_nonXip == NULL)
    {
        printf("ERROR: Unable to open output file [%s]\n", nonXipOutFilename);
        exit(0);
    }
    if(fp_xip == NULL)
    {
        printf("ERROR: Unable to open output file [%s]\n", xipOutFilename);
        exit(0);
    }

    /* we dont need to check the header input again, but we still do it */
    uint32_t value = getWord32(ptr);
    if(value != RPRC_HEADER_TAG)
    {
        printf("ERROR: Invalid input file. RPRC header not found !!!\n");
        exit(0);
    }

    mergeXipSections();
    mergeNonXipSections();

    /* write RPRC header which is already computed during input parsing */
    writeRprcHeader(fp_nonXip, nonXipOutFilename, &gXipCtrl.nonXipRprcMergedHeader);
    writeRprcHeader(fp_xip, xipOutFilename, &gXipCtrl.xipRprcMergedHeader);

    /* read input header */
    memcpy(&rprcHeader, ptr, sizeof(rprc_header_t));
    ptr += sizeof(rprc_header_t);

    for(uint32_t i  = 0; i < rprcHeader.sectionCount; i++)
    {
        rprc_section_t sectionHeader;
        FILE *fp;
        char *filename;

        memcpy(&sectionHeader, ptr, sizeof(rprc_section_t));
        ptr += sizeof(rprc_section_t);

        if( isXipSection(&sectionHeader) )
        {
            /* XIP section, we need to XIP file but first check if this is a merged section
               We should write header for the first section only in a group of merged sections
             */

            fp = fp_xip;
            filename = xipOutFilename;
            sectionNum = xipSectionCount;

            if(xipSectionCount==0)
            {
                /* This is first section, write the first header, the header is computed during mergeXipSections() */
                rprc_section_t *mergedSection = &gXipCtrl.xipRprcMergedSections[mergedXipSectionCount];
                writeSectionHeader(fp, filename, mergedXipSectionCount, mergedSection);
                mergedXipSectionCount++;
            }
            else
            {
                rprc_section_t *cur, *prev;

                /* we know xipSectionCount is > 0, so doing a xipSectionCount-1 is valid */
                prev = &gXipCtrl.xipRprcSections[xipSectionCount-1];
                cur = &gXipCtrl.xipRprcSections[xipSectionCount];

                if( isRprcSectionMerged(cur, prev) )
                {
                    /* do nothing we have written the header already */
                }
                else
                {
                    /* This is distinct section and not merged, so write this new section header */
                    /* Do bounds checking */
                    if(mergedXipSectionCount < gXipCtrl.xipRprcMergedHeader.sectionCount)
                    {
                        /* this is a new section header */
                        rprc_section_t *mergedSection = &gXipCtrl.xipRprcMergedSections[mergedXipSectionCount];
                        writeSectionHeader(fp, filename, mergedXipSectionCount, mergedSection);
                        mergedXipSectionCount++;
                    }
                }
            }
            xipSectionCount++;
        }
        else
        {
            /* if non-XIP, nothing special to do, just write header and data to non-XIP output file */
            fp = fp_nonXip;
            filename = nonXipOutFilename;
            sectionNum = nonXipSectionCount;

            if(nonXipSectionCount==0)
            {
                /* This is first section, write the first header, the header is computed during mergeXipSections() */
                rprc_section_t *mergedSection = &gXipCtrl.nonXipRprcMergedSections[mergedNonXipSectionCount];
                writeSectionHeader(fp, filename, mergedNonXipSectionCount, mergedSection);
                mergedNonXipSectionCount++;
            }
            else
            {
                rprc_section_t *cur, *prev;

                /* we know xipSectionCount is > 0, so doing a xipSectionCount-1 is valid */
                prev = &gXipCtrl.nonXipRprcSections[nonXipSectionCount-1];
                cur = &gXipCtrl.nonXipRprcSections[nonXipSectionCount];

                if( isRprcSectionMerged(cur, prev) )
                {
                    /* do nothing we have written the header already */
                }
                else
                {
                    /* This is distinct section and not merged, so write this new section header */
                    /* Do bounds checking */
                    if(mergedNonXipSectionCount < gXipCtrl.nonXipRprcMergedHeader.sectionCount)
                    {
                        /* this is a new section header */
                        rprc_section_t *mergedSection = &gXipCtrl.nonXipRprcMergedSections[mergedNonXipSectionCount];
                        writeSectionHeader(fp, filename, mergedNonXipSectionCount, mergedSection);
                        mergedNonXipSectionCount++;
                    }
                }
            }
            nonXipSectionCount++;
        }

        /* write section data to XIP or non-XIP file based on section address range decided earlier */
        if(gXipCtrl.verbose)
        {
            printf("Writing section data with adddress 0x%08x, size 0x%08x bytes to file [%s]\n",
                sectionHeader.runAddress,
                sectionHeader.size,
                filename
                );
        }

        wrSize = fwrite(ptr, 1, sectionHeader.size, fp);
        if(wrSize!=sectionHeader.size)
        {
            printf("ERROR: Unable to write %d section data to output file [%s]\n", sectionNum, filename);
            exit(0);
        }
        ptr += sectionHeader.size;

        if( ptr > (uint8_t*)( (uint32_t*)gXipCtrl.inputBuf + gXipCtrl.inputBufSize) )
        {
            printf("ERROR: Invalid input file. Some data missing in file\n");
            exit(0);
        }
    }

    fclose(fp_xip);
    fclose(fp_nonXip);
}

void showUsage()
{
    printf("\n");
    printf("XIP Image Creation Tool  - (c) Texas Instruments 2021, created on %s\n", __DATE__);
    printf("\n");
    printf("Usage: xipGen [options]\n");
    printf("\n");
    printf("Description, \n"
            "   This tool, splits a input RPRC application file, into two RPRC files, \n"
            "   - First RPRC file, containing non-XIP sections. The application entry point is assumed \n"
            "     to be in non-XIP region.\n"
            "   - Second RPRC file, containing XIP sections. The RPRC entry point is set to 0 and \n"
            "     adjacent XIP sections are merged \n"
            "   \n"
            "   Note, the final number of sections in both RPRC files taken together can be less \n"
            "   than the sections in the input file due to section merging in the XIP RPRC file.\n"
            "   \n"
            "   The non-XIP RPRC file should be flashed and booted via SBL as usual\n"
            "   \n"
            "   The XIP RPRC file should be flashed via the SDK flash writer, the flash writer will \n"
            "   flash sections at the flash address mentioned in the RPRC sections\n"
            );
    printf("\n");
    printf("Options, \n");
    printf("--input, -i : input RPRC file, \n");
    printf("--output, -o : output RPRC file of non-XIP sections, \n");
    printf("--output-xip, -x : output RPRC file of XIP sections, \n");
    printf("--flash-start-addr, -f : XIP flash address space start, specified in hex. If not specified 0x60000000 is used \n");
    printf("--flash-size, -s : XIP flash address space size in units of mega bytes, specified as integer. If not specified 256 MB is used \n");
    printf("--verbose, -v : Verbose prints are enabled during the tool execution \n");
    printf("--help, -h : Shows this help \n");
    printf("\n");
    exit(0);
}

void setDefaults()
{
    gXipCtrl.xipAddrStart = XIP_REGION_START;
    gXipCtrl.xipRegionSize = XIP_REGION_SIZE_MB*0x100000;
    gXipCtrl.verbose = 0;
    strcpy(gXipCtrl.inputFileName, "");
    strcpy(gXipCtrl.outputFileNameXip, "");
    strcpy(gXipCtrl.outputFileNameNonXip, "");
}

int parseArgs(int argc, char **argv)
{
    static struct option long_options[] = {
                   {"input",     required_argument, 0,  'i' },
                   {"output",  required_argument,       0,  'o' },
                   {"output-xip",  required_argument, 0,  'x' },
                   {"flash-start-addr", required_argument,       0,  'f' },
                   {"flash-size",  required_argument, 0, 's'},
                   {"verbose",  no_argument, 0, 'v'},
                   {"help",  no_argument, 0, 'h'},
                   {0, 0, 0,  0 }
               };

    int c;
    int option_index;

    setDefaults();

    if(argc == 1)
    {
        showUsage();
    }

    while(1)
    {
        c = getopt_long(argc, argv, "i:o:x:f:s:hv",
                        long_options, &option_index);
        if (c == -1)
        {
            break;
        }

        switch(c)
        {
            case 'h':
                showUsage();
                break;
            case 'i':
                if(optarg!=NULL)
                    strcpy(gXipCtrl.inputFileName, optarg);
                break;
            case 'o':
                if(optarg!=NULL)
                    strcpy(gXipCtrl.outputFileNameNonXip, optarg);
                break;
            case 'x':
                if(optarg!=NULL)
                    strcpy(gXipCtrl.outputFileNameXip, optarg);
                break;
            case 'f':
                if(optarg!=NULL)
                {
                    uint32_t addr = strtoul(optarg, NULL, 16);

                    if(addr > 0)
                    {
                        gXipCtrl.xipAddrStart = addr;
                    }
                }
                break;
            case 's':
                if(optarg!=NULL)
                {
                    uint32_t size = atoi(optarg);

                    if(size > 0 && size < XIP_REGION_SIZE_MB)
                    {
                        gXipCtrl.xipRegionSize = size*0x100000;
                    }
                }
                break;
            case 'v':
                gXipCtrl.verbose = 1;
                break;
            case '?':
                /* unknown option passed */
                break;
        }
    }

    if(gXipCtrl.inputFileName[0] == 0 ||
        gXipCtrl.outputFileNameNonXip[0] == 0 ||
        gXipCtrl.outputFileNameXip[0] == 0
        )
    {
        printf("ERROR: input or output files not specified\n");
        exit(0);
    }
}

int main(int argc, char **argv)
{
    parseArgs(argc, argv);
    parseInputFile(gXipCtrl.inputFileName);
    createOutputFiles(gXipCtrl.outputFileNameNonXip, gXipCtrl.outputFileNameXip);
    parseInputFile(gXipCtrl.outputFileNameNonXip);
    parseInputFile(gXipCtrl.outputFileNameXip);

    return 0;
}
