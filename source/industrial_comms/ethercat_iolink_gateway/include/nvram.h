/*!
 *  \file
 *
 *  \brief
 *  Access to non-volatile memory (NVRAM)
 *
 *  \author
 *  KUNBUS GmbH
 *
 *  \date
 *  2022-07
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

#ifndef NVRAM_H_
#define NVRAM_H_

// this uses littleFS
#include "lfs.h"

#ifdef __cplusplus
extern "C"
{
#endif

// set logging level
#define NVR_LOGLEVEL 1

#if NVR_LOGLEVEL > 2
#define NVR_LOG_DEBUG(fmt, ...) OSAL_printf("%s: " fmt "\r\n", __func__, ##__VA_ARGS__)
#else
#define NVR_LOG_DEBUG(fmt, ...)
#endif

#if NVR_LOGLEVEL > 1
#define NVR_LOG_INFO(fmt, ...) OSAL_printf("%s: " fmt "\r\n", __func__, ##__VA_ARGS__)
#else
#define NVR_LOG_INFO(fmt, ...)
#endif

#if NVR_LOGLEVEL > 0
#define NVR_LOG_ERROR(fmt, ...) OSAL_printf("%s: ERROR: " fmt "\r\n", __func__, ##__VA_ARGS__)
#else
#define NVR_LOG_ERROR(fmt, ...)
#endif

// driver finish function type
typedef int32_t (*NVR_drvFini_t)(void);

// write-callback function type
typedef void(*NVR_writeCallback_t)(int32_t status, char* msg);

typedef enum NVR_err {
    NVR_ERR_OK = 0,
    NVR_ERR_BUSY,
    NVR_ERR_ERROR,
    NVR_ERR_NOTAVAIL,
    NVR_ERR_IO,
    NVR_ERR_INVAL
} NVR_err_t;

typedef enum NVR_type {
    NVR_TYPE_NONE = 0,
    NVR_TYPE_GW_PARAM,
    NVR_TYPE_IOL_PORT,
} NVR_type_t;

enum {
    NVR_MODE_OVERWRITE = 1,
    NVR_MODE_APPEND = 2
};

/******************** Public Interface Functions **********************/

extern NVR_err_t NVR_init(struct lfs_config *plfscfg);
extern NVR_err_t NVR_fini(int32_t (*drv_fini)(void));
extern NVR_err_t NVR_registerCallback(NVR_writeCallback_t cb);
extern NVR_err_t NVR_read(const char * const name, uint32_t * const pLen, const int32_t offset, void * const pData);
extern NVR_err_t NVR_write(const char * const name, const int32_t mode, const uint32_t len, const void * const pData);
extern NVR_err_t NVR_delete(const char * const name);
extern NVR_err_t NVR_erase(void);
extern NVR_err_t NVR_bootcount(const char * const name, uint32_t * const pCount);
extern NVR_err_t NVR_list(const char * const path);
extern uint32_t NVR_crc32(uint32_t crc, const void *buffer, size_t size);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* NVRAM_H_ */
