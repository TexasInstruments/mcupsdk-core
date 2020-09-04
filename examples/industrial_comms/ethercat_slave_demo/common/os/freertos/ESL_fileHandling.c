/*!
 *  \file ESL_fileHandling.c
 *
 *  \brief
 *  File Handling for FreeRTOS.
 *
 *  \author
 *  KUNBUS GmbH
 *
 *  \date
 *  2021-05-18
 *
 *  \copyright
 *  Copyright (c) 2021, KUNBUS GmbH<br /><br />
 *  All rights reserved.<br />
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:<br />
 *  <ol>
 *  <li>Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.</li>
 *  <li>Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.</li>
 *  <li>Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.</li>
 *  </ol>
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <osal.h>
#include <ESL_fileHandling.h>

#include <project.h>
#include <string.h>

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Compare file content
 *
 *  \details
 *  This function compares two files. if the files are different, the return value
 *  is 1 if they are identical the return value is 0
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pFnameA_p   File A
 *  \param[in]  pFnameB_p   File B
 *  \return     ErrorCode p Closer description of ErrorCode, if required.
 *
 *  \remarks
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *  #include <theHeader.h>
 *
 *  // required variables
 *  char* fa="filea.bin";
 *  char* fb="fileb.bin";
 *  int32_t retVal;
 *
 *  // the Call
 *  retVal = EC_SLV_APP_FILE_diff(const char* pFnameA_p, const char* pFnameB_p);
 *  \endcode
 *
 *  <!-- Group: -->
 *
 *  \ingroup EC_SLV_APP
 *
 * */
int32_t EC_SLV_APP_FILE_diff(const char* pFnameA_p, const char* pFnameB_p)
{
    uint32_t    retVal  = 0;
    FILE*       filepA  = NULL;
    FILE*       filepB  = NULL;
    uint8_t     aBufferA[512];
    uint8_t     aBufferB[512];

    if ((filepA = fopen(pFnameA_p, "r")) == NULL)
    {
        retVal = 1;
        goto finish;
    }

    if ((filepB = fopen(pFnameB_p, "r")) == NULL)
    {
        retVal = 1;
        goto finish;
    }

    while (!feof(filepA) && !feof(filepB))
    {
        int32_t lena, lenb;

        lena = fread(aBufferA, 1, sizeof(aBufferA), filepA);
        lenb = fread(aBufferB, 1, sizeof(aBufferB), filepB);

        if (lena != lenb)
        {
            // length is different
            retVal = 1;
            break;
        }

        if (lena < 0)
        {
            break;
        }

        if (OSAL_MEMORY_memcmp(aBufferA, aBufferB, lena) != 0)
        {
            // content is different
            retVal = 1;
            break;
        }
    }

finish:

    if (filepA)
    {
        fclose(filepA);
    }

    if (filepB)
    {
        fclose(filepB);
    }

    return retVal;
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Copy File
 *
 *  \details
 *  This function copies a file. If the file is already available it is not overwritten.
 *  This saves time and conserves the flash memory.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pFromName_p     Source file name
 *  \param[in]  pToName_p       Target file name
 *  \return     ErrorCode
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *
 *  // required variables
 *  uint32_t retVal = 0;
 *  char* fa="filea.bin";
 *  char* fb="fileb.bin";
 *
 *  // the Call
 *  retVal = EC_SLV_APP_FILE_copy(fa, fb);
 *  \endcode
 *
 *  <!-- References: -->
 *
 *  <!-- Group: -->
 *
 *  \ingroup EC_SLV_APP
 *
 * */
uint32_t EC_SLV_APP_FILE_copy(const char* pFromName_p, const char* pToName_p)
{
    EC_API_EError_t error   = EC_API_eERR_NONE;
    FILE*           readFd  = NULL;
    FILE*           writeFd = NULL;

    if (EC_SLV_APP_FILE_diff(pFromName_p, pToName_p) != 0)
    {
        long    length          = 0;
        uint8_t trans[0x1000]   = {0};
        long    read            = 0;
        long    written         = 0;

        readFd = fopen(pFromName_p, "r");

        if (NULL == readFd)
        {
            error  = EC_API_eERR_FW_ERR_OPEN_FAILED;
            goto Exit;
        }

        fseek (readFd, 0, SEEK_END);   // non-portable
        length = ftell(readFd);
        fseek (readFd, 0, SEEK_SET);

        remove(pToName_p);
        writeFd = fopen(pToName_p, "wb");

        if (!writeFd)
        {
            error  = EC_API_eERR_FW_ERR_OPEN_FAILED;
            goto Exit;
        }

        while (!feof(readFd))
        {
            OSAL_MEMORY_memset(trans, 0, sizeof(trans));
            read = fread(trans, sizeof(uint8_t), sizeof(trans), readFd);
            written+= fwrite(trans, sizeof(uint8_t), read, writeFd);
        }

        if (written != length)
        {
            fclose(writeFd);
            writeFd = NULL;
            remove(pToName_p);
            error  = EC_API_eERR_FW_ERR_MEM_OVERFLOW;
        }
    }

Exit:
    if (readFd)
    {
        fclose(readFd);
    }
    if (writeFd)
    {
        fclose(writeFd);
    }

    return error;
}
