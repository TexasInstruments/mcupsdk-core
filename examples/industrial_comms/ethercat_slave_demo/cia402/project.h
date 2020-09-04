/*!
 *  \example project.h
 *
 *  \brief
 *  Brief description of purpose and functionality.
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

#if !(defined __PROJECT_H__)
#define __PROJECT_H__		1

#include <stdio.h>

/* EtherCAT Slave configuration*/

#define KBEC_MAX_PD_LEN                     1024
#define KBEC_DEFAULT_PD_LEN                 256

#define ECAT_REVISION                       0x00010000
#define ECAT_DEVICE_SERIALNUMBER            0xFFFFFFFF //replace with serial number from uboot

#define ESC_EE_PDI_CONTROL                  0x0C80 // 0x80 = On Chip PDI Type
#define ESC_EE_PDI_CONFIG                   0x88E0 // ?? copy from TI

#define COE_SDO_INFO_SUPPORTED              0x02
#define COE_PDO_ASSIGN_SUPPORTED            0x04
#define COE_PDO_CONFIG_SUPPORTED            0x08
#define COE_PDO_UPLOAD_SUPPORTED            0x10
#define COE_SDO_COMPLETE_ACCESS_SUPPORTED   0x20

#define ET1100_BOOT_MBXOUT_START            0x1000
#define ET1100_BOOT_MBXOUT_DEF_LENGTH       256

#define ET1100_BOOT_MBXIN_START             0x1200
#define ET1100_BOOT_MBXIN_DEF_LENGTH        256

#define ET1100_MBXOUT_START                 0x1000
#define ET1100_MBXOUT_DEF_LENGTH            256
#define ET1100_MBXOUT_CONTROLREG            0x26
#define ET1100_MBXOUT_ENABLE                1

#define ET1100_MBXIN_START                  ET1100_MBXOUT_START + ET1100_MBXOUT_DEF_LENGTH
#define ET1100_MBXIN_DEF_LENGTH             256
#define ET1100_MBXIN_CONTROLREG             0x22
#define ET1100_MBXIN_ENABLE                 1

#define ET1100_OUTPUT_START                 ET1100_MBXIN_START + ET1100_MBXIN_DEF_LENGTH
#define ET1100_OUTPUT_CONTROLREG            0x64
#define ET1100_OUTPUT_DEF_LENGTH            (KBEC_MAX_PD_LEN * 3) //in Bytes
#define ET1100_OUTPUT_ENABLE                1

#define ET1100_INPUT_START                  ET1100_OUTPUT_START + ET1100_OUTPUT_DEF_LENGTH
#define ET1100_INPUT_CONTROLREG             0x20
#define ET1100_INPUT_DEF_LENGTH             (KBEC_MAX_PD_LEN * 3) //in Bytes
#define ET1100_INPUT_ENABLE                 1

/* pru/ecat address settings of shared ram */
#define MIN_PD_READ_ADDRESS                 0x1000      // ti register offset
#define MIN_PD_WRITE_ADDRESS                0x1000
#define MIN_MBX_WRITE_ADDRESS               0x1000
#define MIN_MBX_READ_ADDRESS                0x1000

/*#define   OS_TICKS_IN_MILLI_SEC   10 // 100us tick */
#define OS_TICKS_IN_MILLI_SEC               1 /* 1000us tick */
/* task priorities */
#ifndef BSP_OS_MAX_TASKS
#define BSP_OS_MAX_TASKS                    20
#endif

#define KBECSLV_PRIO_PDI                    OSAL_TASK_ePRIO_ECPDI
#define KBECSLV_PRIO_SYNC0                  OSAL_TASK_ePRIO_ECSync
#define KBECSLV_PRIO_SYNC1                  OSAL_TASK_ePRIO_ECSync
#define KBECSLV_PRIO_EOE                    OSAL_TASK_ePRIO_ECEoE
#define KBECSLV_PRIO_LED                    OSAL_TASK_ePRIO_ECLED

#define OSPIFLASH_APP_STARTMAGIC            ((void*)0xEE11AA44)

#if (defined __cplusplus)
extern "C" {
#endif

/* extern void func(void); */

#if (defined __cplusplus)
}
#endif

#endif /* __PROJECT_H__ */
