/*!
* \file pru_syncManager.h
*
* \brief
* PRU Integration: SyncManager interface.
*
* \author
* KUNBUS GmbH
*
* \date
* 2021-05-20
*
* \copyright
* Copyright (c) 2021, KUNBUS GmbH<br /><br />
* All rights reserved.<br />
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:<br />
* <ol>
* <li>Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.</li>
* <li>Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.</li>
* <li>Neither the name of the copyright holder nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.</li>
* </ol>
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#if !(defined __PRU_SYNCMANAGER_H__)
#define __PRU_SYNCMANAGER_H__		1

#include <osal.h>

#define    MAX_SYNC_MAN                         8
#define    SIZEOF_SM_REGISTER                   8

typedef struct SYNCMAN_SProperties {
    uint16_t    physical_start_addr;
    uint16_t    length;
} SYNCMAN_SProperties_t;

#if (defined __cplusplus)
extern "C" {
#endif

extern void     PRU_SM_zeroInit             (void);
extern uint16_t PRU_SM_endAddress           (uint8_t                    syncManNo_p);
extern uint16_t PRU_SM_startAddress         (uint8_t                    syncManNo_p);
extern uint16_t PRU_SM_length               (uint8_t                    syncManNo_p);
extern int16_t  PRU_SM_getIndex             (uint16_t                   address_p
                                            ,uint16_t                   length_p);
extern uint8_t  PRU_SM_PDI_configOngoing    (void);
extern void     PRU_SM_setProperties        (uint8_t                    syncMan_p
                                            ,uint16_t                   address_p
                                            ,uint16_t                   length_p);

#if (defined __cplusplus)
}
#endif

#endif /* __PRU_SYNCMANAGER_H__ */
