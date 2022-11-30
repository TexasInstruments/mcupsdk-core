/* Copyright (C) 2023 Texas Instruments Incorporated
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
 *
 */

 /**
 *  \file     crc_main.h
 *
 *  \brief    This file contains crc test code defines.
 *
 **/

/*===========================================================================*/
/*                         Include files                                     */
/*===========================================================================*/
/*===========================================================================*/
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <kernel/dpl/DebugP.h>
#include <drivers/crc.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <drivers/hw_include/awr294x/cslr_soc_baseaddress.h>


#if !defined(CRC_MAIN_H)
#define CRC_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif



/*===========================================================================*/
/*                         Declarations                                      */
/*===========================================================================*/

/* Define the test interface */
typedef struct crcTest_s
{
    int32_t  (*testFunction)(void);   /* The code that runs the test */
    char      *name;                  /* The test name */
    int32_t    testStatus;            /* Test Status */
} crcTest_t;

/*===========================================================================*/
/*                         Macros                                            */
/*===========================================================================*/
#define CSL_APP_TEST_NOT_RUN        (-(int32_t) (2))
#define CSL_APP_TEST_FAILED         (-(int32_t) (1))
#define CSL_APP_TEST_PASS           ( (int32_t) (0))
#define CSL_PASS                    ( (int32_t) (0))
#define CSL_EBADARGS                (-(int32_t) (2))
#define CSL_EFAIL                   (-(int32_t) (1))
//#define CONFIG_CRC0_BASE_ADDR 		(CSL_MSS_MCRC_U_BASE)
#define APP_USER_DATA_SIZE          ((uint32_t) 36U)
#define APP_CRC_PATTERN_SIZE        ((uint32_t) 2U)
#define APP_CRC_PATTERN_CNT         (APP_USER_DATA_SIZE/APP_CRC_PATTERN_SIZE)
#define APP_CRC_SECT_CNT            ((uint32_t) 1U)
#define CRC_OPERATION_MODE_SEMICPU  (CRC_CTRL2_CH1_MODE_SEMICPU)


/*===========================================================================*/
/*                         Internal function declarations                    */
/*===========================================================================*/


/*===========================================================================*/
/*                         External function declarations                    */
/*===========================================================================*/
extern int32_t crc_posTest(void);
extern int32_t crc_negTest(void);
extern void testcase_main(void);


#ifdef __cplusplus
}

#endif /*extern "C" */

#endif /* CRC_MAIN_H */
/* Nothing past this point */
