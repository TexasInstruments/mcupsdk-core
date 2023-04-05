/*!
 *  \file CMN_app.c
 *
 *  \brief
 *  Common application FreeRTOS support.
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
#include <board.h>

#include <CMN_app.h>

static TaskP_Object CMN_APP_mainHandle_s;
static TaskP_Params CMN_APP_mainParam_s;

static StackType_t CMN_APP_aMainStack_s[CMN_APP_MAIN_STACK_SIZE] __attribute__((aligned(32), section(".threadstack"))) = {0};

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Creates main application thread.
 *
 *  <!-- Parameters and return values: -->
 *  \param[in]  cbTask_p     Function pointer to main task function of application.
 *  \param[in]  pArg_p       Pointer to main task arguments.
 *  \param[in]  prio_p       Specifies priority level of application main task.
 *
 *
 */
void CMN_APP_mainCreate (CMN_APP_CBTask_t               cbTask_p
                        ,void*                          pArg_p
                        ,uint32_t                       prio_p)
{
    OSAL_TASK_EPriority_t prio = (OSAL_TASK_EPriority_t) prio_p;
    uint32_t err;

    TaskP_Params_init(&CMN_APP_mainParam_s);

    CMN_APP_mainParam_s.name      = "cmn_app_task";
    CMN_APP_mainParam_s.stackSize = CMN_APP_MAIN_STACK_SIZE_IN_BYTES;
    CMN_APP_mainParam_s.stack     = (uint8_t*)CMN_APP_aMainStack_s;
    CMN_APP_mainParam_s.priority  = TaskP_PRIORITY_LOWEST + prio;
    CMN_APP_mainParam_s.taskMain  = (TaskP_FxnMain) cbTask_p;
    CMN_APP_mainParam_s.args      = pArg_p;

    err = TaskP_construct(&CMN_APP_mainHandle_s, &CMN_APP_mainParam_s);

    if (SystemP_SUCCESS != err)
    {
        OSAL_printf("Error setting create thread of %s (%ld)\r\n", CMN_APP_mainParam_s.name, err);
        OSAL_error(__func__, __LINE__, OSAL_STACK_INIT_ERROR, true, 0);
    }
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Frees memory allocated by task in time of creation.
 *
 *  \detail
 *  Needs to be called at the end of each created task.
 *
 */
void CMN_APP_mainExit (void)
{
    TaskP_destruct(NULL);
}
