/*
 * Copyright (C) 2021 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#ifndef HDSL_INTERFACE_H_
#define HDSL_INTERFACE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

#include <stdbool.h>

/**
 * \brief HDSL Interface
 */
struct hdslInterface
{
    volatile uint8_t SYS_CTRL;    /**< System control */
    volatile uint8_t SYNC_CTRL;   /**< Synchronization control */
    volatile uint8_t resvd0;      /**< Reserved 0 */
    volatile uint8_t MASTER_QM;   /**< Quality monitoring */
    volatile uint8_t EVENT_H;     /**< High bytes event */
    volatile uint8_t EVENT_L;     /**< Low bytes event */
    volatile uint8_t MASK_H;      /**< High byte event mask */
    volatile uint8_t MASK_L;      /**< Low byte event mask */
    volatile uint8_t MASK_SUM;    /**< Summary mask */
    volatile uint8_t EDGES;       /**< Cable bit sampling time control */
    volatile uint8_t DELAY;       /**< Run time delay of system cable and signal strength */
    volatile uint8_t VERSION;     /**< Version */
    volatile uint8_t resvd1;      /**< Reserved 1 */
    volatile uint8_t ENC_ID2;     /**< Encoder ID, byte 2 */
    volatile uint8_t ENC_ID1;     /**< Encoder ID, byte 1 */
    volatile uint8_t ENC_ID0;     /**< Encoder ID, byte 0 */
    volatile uint8_t POS4;        /**< Fast position, byte 4 */
    volatile uint8_t POS3;        /**< Fast position, byte 3 */
    volatile uint8_t POS2;        /**< Fast position, byte 2 */
    volatile uint8_t POS1;        /**< Fast position, byte 1 */
    volatile uint8_t POS0;        /**< Fast position, byte 0 */
    volatile uint8_t VEL2;        /**< Speed, byte 2 */
    volatile uint8_t VEL1;        /**< Speed, byte 1 */
    volatile uint8_t VEL0;        /**< Speed, byte 0 */
    volatile uint8_t SUMMARY;     /**< Summarized slave status */
    volatile uint8_t VPOS4;       /**< Safe position, byte 4 */
    volatile uint8_t VPOS3;       /**< Safe position, byte 3 */
    volatile uint8_t VPOS2;       /**< Safe position, byte 2 */
    volatile uint8_t VPOS1;       /**< Safe position, byte 1 */
    volatile uint8_t VPOS0;       /**< Safe position, byte 0 */
    volatile uint8_t VPOSCRC_H;   /**< CRC of Safe position, byte 1 */
    volatile uint8_t VPOSCRC_L;   /**< CRC of Safe position, byte 0 */
    volatile uint8_t PC_BUFFER0;  /**< Parameters channel buffer, byte 0 */
    volatile uint8_t PC_BUFFER1;  /**< Parameters channel buffer, byte 1 */
    volatile uint8_t PC_BUFFER2;  /**< Parameters channel buffer, byte 2 */
    volatile uint8_t PC_BUFFER3;  /**< Parameters channel buffer, byte 3 */
    volatile uint8_t PC_BUFFER4;  /**< Parameters channel buffer, byte 4 */
    volatile uint8_t PC_BUFFER5;  /**< Parameters channel buffer, byte 5 */
    volatile uint8_t PC_BUFFER6;  /**< Parameters channel buffer, byte 6 */
    volatile uint8_t PC_BUFFER7;  /**< Parameters channel buffer, byte 7 */
    volatile uint8_t PC_ADD_H;    /**< Long message address, byte 1 */
    volatile uint8_t PC_ADD_L;    /**< Long message address, byte 0 */
    volatile uint8_t PC_OFF_H;    /**< Long message address offset, byte 1 */
    volatile uint8_t PC_OFF_L;    /**< Long message address offset, byte 0 */
    volatile uint8_t PC_CTRL;     /**< Parameters channel control */
    volatile uint8_t PIPE_S;      /**< Sensor hub channel status */
    volatile uint8_t PIPE_D;      /**< Sensor hub channel data */
    volatile uint8_t PC_DATA;     /**< Short message parameters channel data */
    volatile uint8_t resvd2;      /**< Reserved 2 */
    volatile uint8_t resvd3;      /**< Reserved 3 */
    volatile uint8_t resvd4;      /**< Reserved 4 */
    volatile uint8_t resvd5;      /**< Reserved 5 */
    volatile uint8_t resvd6;      /**< Reserved 6 */
    volatile uint8_t resvd7;      /**< Reserved 7 */
    volatile uint8_t resvd8;      /**< Reserved 8 */
    volatile uint8_t S_PC_DATA;   /**< Response of Short message parameters channel Read for safe1 channel */
    volatile uint8_t ACC_ERR_CNT; /**< Fast position error counter */
    volatile uint8_t MAXACC;      /**< Fast position acceleration boundary */
    volatile uint8_t MAXDEV_H;    /**< Fast position estimator deviation high byte */
    volatile uint8_t MAXDEV_L;    /**< Fast position estimator deviation low byte */
    volatile uint8_t resvd10;     /**< Reserved 10 */
    volatile uint8_t resvd11;     /**< Reserved 11 */
    volatile uint8_t resvd12;     /**< Reserved 12 */
    volatile uint8_t DUMMY;       /**< Dummy, no data */
    volatile uint8_t SLAVE_REG_CTRL;    /**< Short message control */
    volatile uint8_t ACC_ERR_CNT_THRES; /**< Fast position error counter threshold */
    volatile uint8_t MAXDEV_H_THRES;    /**< Fast position estimator deviation high byte threshold */
    volatile uint8_t MAXDEV_L_THRES;    /**< Fast position estimator deviation low byte threshold */
    /*Safe 2 Interface */
    volatile uint8_t version;
    volatile uint8_t ENC2_ID;
    volatile uint8_t STATUS2;
    volatile uint8_t VPOS24;
    volatile uint8_t VPOS23;
    volatile uint8_t VPOS22;
    volatile uint8_t VPOS21;
    volatile uint8_t VPOS20;
    volatile uint8_t VPOSCRC2_H;
    volatile uint8_t VPOSCRC2_L;
    volatile uint8_t DUMMY2;
	/* Online Status*/
	volatile uint16_t ONLINE_STATUS_D;
	volatile uint16_t ONLINE_STATUS_1;
	volatile uint16_t ONLINE_STATUS_2;
};



#ifdef __cplusplus
}
#endif

#endif
