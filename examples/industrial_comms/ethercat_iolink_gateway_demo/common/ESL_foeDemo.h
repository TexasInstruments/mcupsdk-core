/*
 *  Copyright (c) 2020, Kunbus GmbH
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
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

#if !(defined __ESL_FOEDEMO_H__)
#define __ESL_FOEDEMO_H__		1

#include <stdint.h>
#include "project.h"

#include "ESL_fileHandling.h"
#include <ecSlvApi_Error.h>
#include <stdio.h>
#include <stdbool.h>

#define MAX_FILE_NAME_SIZE    64
#define KBFOE_TMPFILE   "/tmp/foefile.bin"
#define KBFOE_PREFIX    "/tmp/"

#if (defined __cplusplus)
extern "C" {
#endif

extern uint32_t EC_SLV_APP_FoE_fileOpen     (void*          pContext_p
                                            ,const char*    pName_p
                                            ,uint16_t       nameLen_p
                                            ,bool           isRead_p
                                            ,uint32_t       password_p);
extern uint32_t EC_SLV_APP_FoE_fileClose    (void*          pContext_p
                                            ,uint32_t       errorCode_p);
extern uint32_t EC_SLV_APP_FoE_fileRead     (void*          pContext_p
                                            ,uint16_t*      pData_p
                                            ,uint16_t       size_p
                                            ,uint32_t       fileOffset_p);
extern uint32_t EC_SLV_APP_FoE_fileWrite    (void*          pContext_p
                                            ,uint16_t*      pData_p
                                            ,uint16_t       size_p);

#if (defined __cplusplus)
}
#endif

/** @} */
#endif /* __ESL_FOEDEMO_H__ */

