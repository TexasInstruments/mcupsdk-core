/*!
 *  \file CMN_CPU_api.h
 *
 *  \brief
 *  Common CPU load monitoring FreeRTOS support
 *
 *  \author
 *  KUNBUS GmbH
 *
 *  \date
 *  2022-06-10
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

#if !(defined PROTECT_CMN_CPU_API_H)
#define PROTECT_CMN_CPU_API_H     1

#define CMN_CPU_API_MAX_TASKS_NUM       32
#define CMN_CPU_API_LOAD_NAME_MAX_CHARS 32

typedef enum CMN_CPU_API_EOutput
{
    CMN_CPU_API_eOUT_NONE,
    CMN_CPU_API_eOUT_OSAL_PRINTF,
    CMN_CPU_API_eOUT_DEBUGP
}CMN_CPU_API_EOutput_t;

typedef struct CMN_CPU_API_SLoad
{
    void*       taskHandle;
    char        name[CMN_CPU_API_LOAD_NAME_MAX_CHARS + 1];
    uint32_t    cpuLoad;
    bool        exists;
}CMN_CPU_API_SLoad_t;

typedef struct CMN_CPU_API_SData
{
    CMN_CPU_API_SLoad_t   cpu;
    CMN_CPU_API_SLoad_t   tasks[CMN_CPU_API_MAX_TASKS_NUM];
    uint32_t              tasksNum;
}CMN_CPU_API_SData_t;

typedef struct CMN_CPU_API_SParams
{
    OSAL_TASK_EPriority_t taskPrio;
    CMN_CPU_API_EOutput_t output;
}CMN_CPU_API_SParams_t;

#if (defined __cplusplus)
extern "C" {
#endif

extern void                 CMN_CPU_API_startMonitor   (CMN_CPU_API_SParams_t* pParams_p);
extern void                 CMN_CPU_API_generateReport (CMN_CPU_API_EOutput_t out_p);
extern CMN_CPU_API_SData_t* CMN_CPU_API_getData        (void);

#if (defined __cplusplus)
}
#endif

#endif /* PROTECT_CMN_CPU_API_H */
