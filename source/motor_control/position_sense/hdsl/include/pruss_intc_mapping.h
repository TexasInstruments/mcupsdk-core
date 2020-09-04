/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
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

#ifndef PRUSS_INTC_MAPPING_H_
#define PRUSS_INTC_MAPPING_H_

#ifdef __cplusplus
extern "C" {
#endif

#define ICSS1_PRUSS0_HOST_INTR5     56

/* ICSS0 event map */
#define HIPERFACE_EVENT0	 16
#define HIPERFACE_EVENT1	 18
#define HIPERFACE_EVENT2	 19
#define HIPERFACE_EVENT3	 20
#define HIPERFACE_EVENT4     21

#define HDSL_APP_CHANNEL0                0
#define HDSL_APP_CHANNEL1                1
#define HDSL_APP_CHANNEL2                2
#define HDSL_APP_CHANNEL3                3
#define HDSL_APP_CHANNEL4                4
#define HDSL_APP_CHANNEL5                5
#define HDSL_APP_CHANNEL6                6
#define HDSL_APP_CHANNEL7                7
#define HDSL_APP_CHANNEL8                8
#define HDSL_APP_CHANNEL9                9

#define PRU_EVTOUT0             2
#define PRU_EVTOUT3             5
#define PRU_EVTOUT5             7
#define PRU_EVTOUT6             8

#define PRU_EVTOUT5_HOSTEN_MASK     0x003C
#define PRU_EVTOUT6_HOSTEN_MASK     0x0100

#define SYS_EVT_POLARITY_HIGH       1

#define SYS_EVT_TYPE_EDGE           1
#define SYS_EVT_TYPE_LEVEL          0

/* ICSS0 / ICSS_L */
/* Is HDSL_APP_CHANNEL0->PRU0 & HDSL_APP_CHANNEL1->PRU1 reqd ?, seems not reqd */
#define PRU_ICSS0_INTC_INITDATA { \
        { HIPERFACE_EVENT0, HIPERFACE_EVENT2, HIPERFACE_EVENT3, HIPERFACE_EVENT4, 0xFF  }, \
        { \
            {HIPERFACE_EVENT0,     HDSL_APP_CHANNEL2,   SYS_EVT_POLARITY_HIGH,  SYS_EVT_TYPE_LEVEL}, \
            {HIPERFACE_EVENT2,     HDSL_APP_CHANNEL3,   SYS_EVT_POLARITY_HIGH,  SYS_EVT_TYPE_LEVEL}, \
            {HIPERFACE_EVENT3,     HDSL_APP_CHANNEL4,   SYS_EVT_POLARITY_HIGH,  SYS_EVT_TYPE_LEVEL}, \
            {HIPERFACE_EVENT4,     HDSL_APP_CHANNEL5,   SYS_EVT_POLARITY_HIGH,  SYS_EVT_TYPE_LEVEL}, \
			{0xFF, 0xFF, 0xFF, 0xFF }, \
        }, \
        { \
            {HDSL_APP_CHANNEL2, PRU_EVTOUT0}, \
            {HDSL_APP_CHANNEL3, HDSL_APP_CHANNEL3}, \
            {HDSL_APP_CHANNEL4, HDSL_APP_CHANNEL4}, \
            {HDSL_APP_CHANNEL5, PRU_EVTOUT3},\
            {0xFF,0xFF} \
        }, \
        PRU_EVTOUT5_HOSTEN_MASK }

/* Is HDSL_APP_CHANNEL0->PRU0 & HDSL_APP_CHANNEL1->PRU1 reqd ?, seems not reqd */
#define PRU_ICSS1_INTC_INITDATA { \
        { ICSS1_PRUSS0_HOST_INTR5, 0xFF },\
        { \
            {ICSS1_PRUSS0_HOST_INTR5, HDSL_APP_CHANNEL8,SYS_EVT_POLARITY_HIGH ,SYS_EVT_TYPE_EDGE }, \
            {0xFF,0xFF,0xFF,0xFF} \
        }, \
        { \
            {HDSL_APP_CHANNEL8, PRU_EVTOUT6}, \
            {0xFF,0xFF} \
        }, \
        PRU_EVTOUT6_HOSTEN_MASK }

#ifdef __cplusplus
}
#endif

#endif
