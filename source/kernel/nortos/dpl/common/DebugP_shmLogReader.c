/*
 *  Copyright (C) 2018-2023 Texas Instruments Incorporated
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

#include <drivers/hw_include/cslr_soc.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>

#define DEBUG_SHM_LOG_READER_TASK_POLLING_TIME_IN_MSEC  (10u)
#define DEBUG_SHM_LOG_READER_LINE_BUF_SIZE  (130u)

typedef struct {

    uint8_t isCoreShmLogInialized[CSL_CORE_ID_MAX];
    DebugP_ShmLog *shmLog;
    uint8_t numCores;
    char lineBuf[DEBUG_SHM_LOG_READER_LINE_BUF_SIZE+UNSIGNED_INTEGERVAL_THREE]; /* +3 to add \r\n and null char at end of string in worst case */

} DebugP_ShmLogReaderCtrl;

DebugP_ShmLogReaderCtrl gDebugShmLogReaderCtrl;

uint32_t DebugP_shmLogReaderGetString(DebugP_ShmLog *shmLog,char *buf, uint32_t buf_size);

void DebugP_shmLogReaderTaskMain(void *args);

extern void DebugP_shmLogReaderTaskCreate(void);

void DebugP_shmLogReaderInit(DebugP_ShmLog *shmLog, uint16_t numCores)
{
    uint16_t i;

    DebugP_assertNoLog(numCores <= CSL_CORE_ID_MAX);

    for(i=0; i<CSL_CORE_ID_MAX; i++)
    {
        gDebugShmLogReaderCtrl.isCoreShmLogInialized[i] = 0;
    }
    gDebugShmLogReaderCtrl.shmLog = shmLog;
    gDebugShmLogReaderCtrl.numCores = (uint8_t)numCores;

    DebugP_shmLogReaderTaskCreate();
}

uint32_t DebugP_shmLogReaderGetString(DebugP_ShmLog *shmLog,
                char *buf, uint32_t buf_size)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t num_bytes, wr_idx, rd_idx;

    num_bytes = 0;
    wr_idx = shmLog->wrIndex;
    rd_idx = shmLog->rdIndex;
    if((wr_idx >= DebugP_SHM_LOG_SIZE )|| (rd_idx >= DebugP_SHM_LOG_SIZE))
    {
        status = SystemP_FAILURE;
    }
    if(status == SystemP_SUCCESS)
    {
        if(rd_idx > wr_idx)
        {
            num_bytes = (DebugP_SHM_LOG_SIZE - rd_idx) + wr_idx;
        }
        else
        {
            num_bytes = wr_idx - rd_idx;
        }
        if(num_bytes > 0U)
        {
            volatile uint8_t *src;
            uint32_t break_loop, copy_bytes, idx;
            uint8_t cur_char;

            src = shmLog->buffer;
            idx = 0;
            for(copy_bytes = 0U; copy_bytes < num_bytes; copy_bytes ++)
            {
                cur_char = src[rd_idx];

                rd_idx++;
                if(rd_idx >=  DebugP_SHM_LOG_SIZE)
                {
                    rd_idx = 0;
                }

                break_loop = 0;

                /* pick only user viewable characters */
                if(    ((cur_char >= (uint8_t)' ' )&& (cur_char <= (uint8_t)'~'))/* all alphabets, numbers, special char's like .,- etc */
                    || (cur_char == (uint8_t)'\r') /* other valid char's */
                    || (cur_char == (uint8_t)'\n')
                    || (cur_char == (uint8_t)'\t')
                    )
                {
                    buf[idx] = (char)cur_char;
                    idx ++;
                }
                if(cur_char==(uint8_t)'\n')
                {
                    /* add null termination for string and break, we have +3 additional bytes of \r\n null for worst case */
                    buf[idx] = (char)0;
					idx = idx + 1U;
                    break_loop = 1;
                }
                else
                if (idx >= buf_size)
                {
                    /* buffer size exceeded, we have +3 additional bytes of \r\n null, add these and break */
                    buf[idx] = (char)'\r';
					idx = idx + 1U;
                    buf[idx] = (char)'\n';
					idx = idx + 1U;
                    buf[idx] = (char)0;
					idx = idx + 1U;
                    break_loop = 1;
                }
				else { /*MISRAC */}
                if ( 1U == break_loop)
                {
                    break;
                }
				else { /*MISRAC */}
            }
            num_bytes = idx;

            shmLog->rdIndex = rd_idx;
            /* dummy read to ensure data is written to memory */
            rd_idx = shmLog->rdIndex;
        }
    }
    return num_bytes;
}

void DebugP_shmLogRead(void) 
{
	uint32_t i;

	for(i=0; i<gDebugShmLogReaderCtrl.numCores; i++)
	{
		DebugP_ShmLog *shmLog = &gDebugShmLogReaderCtrl.shmLog[i];

		if(gDebugShmLogReaderCtrl.isCoreShmLogInialized[i]==0U)
		{
			if(shmLog->isValid == DebugP_SHM_LOG_IS_VALID)
			{
				gDebugShmLogReaderCtrl.isCoreShmLogInialized[i] = 1;
				/* clear isValid flag */
				shmLog->isValid = 0;
			}
		}
		if((gDebugShmLogReaderCtrl.isCoreShmLogInialized[i]!=0U))
		{
			uint32_t strLen;

			do
			{
				strLen = DebugP_shmLogReaderGetString(shmLog,
								gDebugShmLogReaderCtrl.lineBuf,
								DEBUG_SHM_LOG_READER_LINE_BUF_SIZE);
				if(strLen > 0U)
				{
					DebugP_log(gDebugShmLogReaderCtrl.lineBuf);
				}
			} while(strLen != 0U);
		}
	}
}

void DebugP_shmLogReaderTaskMain(void *args)
{
    uint32_t pollingTicks = ClockP_usecToTicks(DEBUG_SHM_LOG_READER_TASK_POLLING_TIME_IN_MSEC*1000U);
    uint32_t elaspedTicks, sleepTicks;
    uint32_t loop = 1;
    sleepTicks = pollingTicks;
    while(loop!=0U)
    {
        uint32_t i;

        elaspedTicks = ClockP_getTicks();
        ClockP_usleep( ClockP_ticksToUsec(sleepTicks) );

        for(i=0; i<gDebugShmLogReaderCtrl.numCores; i++)
        {
            DebugP_ShmLog *shmLog = &gDebugShmLogReaderCtrl.shmLog[i];

            if(gDebugShmLogReaderCtrl.isCoreShmLogInialized[i]==0U)
            {
                if(shmLog->isValid == DebugP_SHM_LOG_IS_VALID)
                {
                    gDebugShmLogReaderCtrl.isCoreShmLogInialized[i] = 1;
                    /* clear isValid flag */
                    shmLog->isValid = 0;
                }
            }
            if(gDebugShmLogReaderCtrl.isCoreShmLogInialized[i]!=0U)
            {
                uint32_t strLen;

                do
                {
                    strLen = DebugP_shmLogReaderGetString(shmLog,
                                    gDebugShmLogReaderCtrl.lineBuf,
                                    DEBUG_SHM_LOG_READER_LINE_BUF_SIZE);
                    if(strLen > 0U)
                    {
                        DebugP_log(gDebugShmLogReaderCtrl.lineBuf);
                    }
                } while(strLen!=0U);
            }
        }

        elaspedTicks = ClockP_getTicks() - elaspedTicks;
        if( elaspedTicks >= pollingTicks)
        {
            sleepTicks = 0;
        }
        else
        {
            sleepTicks = (pollingTicks - elaspedTicks);
        }
    }
    /* the loop will never exit */
}

