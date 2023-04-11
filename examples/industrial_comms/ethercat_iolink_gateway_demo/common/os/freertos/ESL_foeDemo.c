/*!
 *  \file ESL_foeDemo.c
 *
 *  \brief
 *  EtherCAT<sup>&reg;</sup> File Transfer over EtherCAT Example
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
#include <ESL_foeDemo.h>
#include <string.h>

/*---------------------------------------------
-    Error Codes
-----------------------------------------------*/
#define    ECAPP_ERROR_CLASS(x)                 (0x38020000|((x)&0xffff))

#define    ECAT_FOE_ERRCODE_NOTDEFINED          ECAPP_ERROR_CLASS(0x8000)   /**< \brief Not defined*/
#define    ECAT_FOE_ERRCODE_NOTFOUND            ECAPP_ERROR_CLASS(0x8001)   /**< \brief The file requested by an FoE upload service could not be found on the server*/
#define    ECAT_FOE_ERRCODE_ACCESS              ECAPP_ERROR_CLASS(0x8002)   /**< \brief Read or write access to this file not allowed (e.g. due to local control).*/
#define    ECAT_FOE_ERRCODE_DISKFULL            ECAPP_ERROR_CLASS(0x8003)   /**< \brief Disk to store file is full or memory allocation exceeded*/
#define    ECAT_FOE_ERRCODE_ILLEGAL             ECAPP_ERROR_CLASS(0x8004)   /**< \brief Illegal FoE operation, e.g. service identifier invalid*/
#define    ECAT_FOE_ERRCODE_PACKENO             ECAPP_ERROR_CLASS(0x8005)   /**< \brief FoE packet number invalid*/
#define    ECAT_FOE_ERRCODE_EXISTS              ECAPP_ERROR_CLASS(0x8006)   /**< \brief The file which is requested to be downloaded does already exist*/
#define    ECAT_FOE_ERRCODE_NOUSER              ECAPP_ERROR_CLASS(0x8007)   /**< \brief No User*/
#define    ECAT_FOE_ERRCODE_BOOTSTRAPONLY       ECAPP_ERROR_CLASS(0x8008)   /**< \brief FoE only supported in Bootstrap*/
#define    ECAT_FOE_ERRCODE_NOTINBOOTSTRAP      ECAPP_ERROR_CLASS(0x8009)   /**< \brief This file may not be accessed in BOOTSTRAP state*/
#define    ECAT_FOE_ERRCODE_NORIGHTS            ECAPP_ERROR_CLASS(0x800A)   /**< \brief Password invalid*/
#define    ECAT_FOE_ERRCODE_PROGERROR           ECAPP_ERROR_CLASS(0x800B)   /**< \brief Generic programming error. Should only be returned if  error reason cannot be distinguished*/
#define    ECAT_FOE_ERRCODE_INVALID_CHECKSUM    ECAPP_ERROR_CLASS(0x800C)   /**< \brief checksum included in the file is invalid*/
#define    ECAT_FOE_ERRCODE_INVALID_FIRMWARE    ECAPP_ERROR_CLASS(0x800D)   /**< \brief The hardware does not support the downloaded firmware*/
#define    ECAT_FOE_ERRCODE_NO_FILE             ECAPP_ERROR_CLASS(0x800F)   /**< \brief Do not use (identical with 0x8001)*/
#define    ECAT_FOE_ERRCODE_NO_FILE_HEADER      ECAPP_ERROR_CLASS(0x8010)   /**< \brief Missing file header of error in file header*/
#define    ECAT_FOE_ERRCODE_FLASH_ERROR         ECAPP_ERROR_CLASS(0x8011)   /**< \brief Flash cannot be accessed*/

typedef struct EC_SLV_APP_FoE_SInternalHandle
{
    FILE*       pFileHandle;
    char        aFilename[MAX_FILE_NAME_SIZE]; // 0 terminated absolute filename
    uint32_t    password;
    uint8_t     isRead;
} EC_SLV_APP_FoE_SInternalHandle_t;

static EC_SLV_APP_FoE_SInternalHandle_t     EC_SLV_APP_FoE_InternalHandle_s     = { 0 };

/*! <!-- Description: -->
 *
 *  \brief
 *  Read file from device.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pContext_p      context handle
 *  \param[in]  pData_p         data buffer.
 *  \param[in]  size_p          read length parameter.
 *  \param[in]  fileOffset_p    File offset.
 *  \return     ErrorCode       FoE error code.
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *  uint16_t*   pData;
 *  uint16_t    length;
 *  uint32_t    fileOffset;
 *
 *  // the Call
 *  retVal = EC_SLV_APP_FoE_fileRead(NULL, pData, length, fileOffset);
 *  \endcode
 *
 *  <!-- Group: -->
 *
 *  \ingroup EC_SLV_APP
 *
 * */
uint32_t EC_SLV_APP_FoE_fileRead(void* pContext_p, uint16_t* pData_p, uint16_t size_p, uint32_t fileOffset_p)
{
    size_t      readLen;
    uint32_t    retVal;

    OSALUNREF_PARM(pContext_p);

    if (EC_SLV_APP_FoE_InternalHandle_s.pFileHandle == NULL)
    {
        // no file
        retVal = ECAT_FOE_ERRCODE_PACKENO;
        goto Exit;
    }

    if (fseek(EC_SLV_APP_FoE_InternalHandle_s.pFileHandle, fileOffset_p, SEEK_SET) != 0)
    {
        retVal = ECAT_FOE_ERRCODE_ILLEGAL;
        goto Exit;
    }

    if (feof(EC_SLV_APP_FoE_InternalHandle_s.pFileHandle))
    {
        retVal = EC_API_eERR_NONE; // end of file
        goto Exit;
    }

    readLen = fread(pData_p, 1, size_p, EC_SLV_APP_FoE_InternalHandle_s.pFileHandle);
    if (ferror(EC_SLV_APP_FoE_InternalHandle_s.pFileHandle) != 0)
    {
        retVal = ECAT_FOE_ERRCODE_ILLEGAL;
        goto Exit;
    }
    retVal = readLen;

Exit:
    return retVal;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Write file from device.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pContext_p      context
 *  \param[in]  pData_p   		Data Buffer.
 *  \param[in]  size_p    		write length parameter.
 *  \return     ErrorCode       FoE error code.
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *  // required variables
 *  uint16_t* pData;
 *  uint16_t  length;
 *
 *  // the Call
 *  retVal = EC_SLV_APP_FoE_fileWrite(pData, length);
 *  \endcode
 *
 *  <!-- Group: -->
 *
 *  \ingroup EC_SLV_APP
 *
 * */
uint32_t EC_SLV_APP_FoE_fileWrite(void *pContext_p, uint16_t* pData_p, uint16_t size_p)
{
    size_t      writeLen;
    uint32_t    retVal;

    OSALUNREF_PARM(pContext_p);

    if (EC_SLV_APP_FoE_InternalHandle_s.pFileHandle == NULL)
    {
        // no file
        retVal = ECAT_FOE_ERRCODE_ILLEGAL;
        goto Exit;
    }

    writeLen = fwrite(pData_p, 1, size_p, EC_SLV_APP_FoE_InternalHandle_s.pFileHandle);
    if (writeLen != size_p)
    {
        retVal = ECAT_FOE_ERRCODE_DISKFULL;
        goto Exit;
    }

    retVal = writeLen;
Exit:
    return retVal;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Close file.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pContext_p  context
 *  \param[in]  errorCode_p Error code handed by the Stack.
 *  \return     ErrorCode   FoE error code.
 *
 *  <!-- Example: -->
 *  // required variables
 *  uint32_t errorCode;
 *  void* pCtxt;
 *
 *  EC_SLV_APP_FoE_fileClose(pCtxt, errorCode);
 *
 *  <!-- Group: -->
 *
 *  \ingroup EC_SLV_APP
 *
 * */
uint32_t EC_SLV_APP_FoE_fileClose(void* pContext_p, uint32_t errorCode_p)
{
    uint32_t    retVal  = 0;

    OSALUNREF_PARM(pContext_p);

    if (EC_SLV_APP_FoE_InternalHandle_s.pFileHandle)
    {
        fclose(EC_SLV_APP_FoE_InternalHandle_s.pFileHandle);

        // copy write file
        if (errorCode_p == 0 && !EC_SLV_APP_FoE_InternalHandle_s.isRead)
        {
            uint32_t error = EC_SLV_APP_FILE_copy(KBFOE_TMPFILE, EC_SLV_APP_FoE_InternalHandle_s.aFilename);

            if (error  != EC_API_eERR_NONE)
            {
                retVal = ECAT_FOE_ERRCODE_DISKFULL;
            }
        }
    }

    OSAL_MEMORY_memset(&EC_SLV_APP_FoE_InternalHandle_s, 0, sizeof(EC_SLV_APP_FoE_SInternalHandle_t));

    return retVal;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Open file from device.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pContext_p          context
 *  \param[in]  pName_p             Filename.
 *  \param[in]  nameLen_p           Filename length.
 *  \param[in]  isRead_p            Access rigths.
 *  \param[in]  password_p          Access code.
 *  \return     ErrorCode           FoE Error code.
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *  // required variables
 *  uin16t_t    length;
 *  const char* pName;
 *  bool access;
 *  uint32_t password;
 *
 *  // the Call
 *  EC_SLV_APP_FoeFileOpen(fileName, length, access, password)
 *  \endcode
 *
 *  <!-- Group: -->
 *
 *  \ingroup EC_SLV_APP
 *
 * */
uint32_t EC_SLV_APP_FoE_fileOpen(void* pContext_p, const char* pName_p, uint16_t nameLen_p, bool isRead_p, uint32_t password_p)
{
    const char* pFmode_p    = NULL;
    uint32_t    retVal;

    EC_SLV_APP_FoE_fileClose(pContext_p, ~0);

    // store filename
    strcpy(EC_SLV_APP_FoE_InternalHandle_s.aFilename, KBFOE_PREFIX);
    OSAL_MEMORY_memcpy(EC_SLV_APP_FoE_InternalHandle_s.aFilename + sizeof(KBFOE_PREFIX) - 1, pName_p, nameLen_p);

    EC_SLV_APP_FoE_InternalHandle_s.aFilename[nameLen_p + (sizeof(KBFOE_PREFIX) - 1)] = '\0';
    EC_SLV_APP_FoE_InternalHandle_s.password    = password_p;
    EC_SLV_APP_FoE_InternalHandle_s.isRead      = isRead_p;

    // open file
    if (isRead_p)
    {
        pFmode_p = "rb";
        EC_SLV_APP_FoE_InternalHandle_s.pFileHandle = fopen(EC_SLV_APP_FoE_InternalHandle_s.aFilename, pFmode_p);
    }
    else
    {
        pFmode_p = "wb";
        EC_SLV_APP_FoE_InternalHandle_s.pFileHandle = fopen(KBFOE_TMPFILE, pFmode_p);
    }

    if (EC_SLV_APP_FoE_InternalHandle_s.pFileHandle == NULL)
    {
        retVal = ECAT_FOE_ERRCODE_DISKFULL;
        goto Exit;
    }

    retVal = EC_API_eERR_NONE;
Exit:
    return retVal;
}

//*************************************************************************************************
