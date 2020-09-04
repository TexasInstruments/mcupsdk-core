/*
 *  Copyright (c) 2021, KUNBUS GmbH
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

#ifndef INC_PROT__IOL_PORT_DEFINITION_H__
#define INC_PROT__IOL_PORT_DEFINITION_H__

#include <string.h>

/**
\addtogroup group_iol_port_definition IO-Link Port Definitions
\{
*/

/** \brief This define sets the size of the checksum lookup table. 

If 1, there is a 256 byte lookup table used. if 0, there is only a 32 byte
lookup table used. The bigger lookup table is faster, the smaller needs
less code. The checksum had to be computed twice (Master+Device) during
an IO-Link cycle. Because of this it is recommended to use the full
lookup table */
#define IOL_USE_FULL_CHECKSUM_LOOKUP    1

/** \brief Prefix for IO-Link function declaration  */
#ifdef __cplusplus
#define IOL_FUNC_DECL extern "C"
#else
#define IOL_FUNC_DECL extern
#endif

/** \brief Activates (1) or deactivates (0) the logger.

The logger should only be used for debugging and be disabled on
the release compilation.
*/
#define IOL_ENABLE_LOGGER               0

/** \brief Activates (1) or deactivates (0) the assertions.

Assertions can be used during development. If the software is
tested, they can be disabled to save code memory.
*/
#define IOL_ENABLE_ASSERTIONS           0

/** \brief Is true if the log function (#IOL_Port_vLog()) is needed,
otherwise false. */
#define IOL_IS_LOG_FUNCTION_NEEDED      (IOL_ENABLE_LOGGER || IOL_ENABLE_ASSERTIONS)

/** \brief UART frame transmission delay of Master (ports).

See A.3.3 of \ref section_iol_spec_1_1.
This is the maximum time between two Master Tx UART bytes. Depending
on the hardware implementation (using of FIFO or DMA) this value can
be set to 0. The value is only used to calculate the frame monitor and
mincycletime.

For the Device implementation this value should always be set to the maximum
allowed value in the specification.
*/
#define IOL_T_T1_BIT                    0

/** \brief UART frame transmission delay of Devices.

See A.3.4 of \ref section_iol_spec_1_1.
This is the maximum time between two Device Tx UART bytes. Depending
on the hardware implementation (using of FIFO or DMA) this value can
be set to 0. The value is only used to calculate the frame monitor and
mincycletime.

For the Master implementation this value should always be set to the maximum
allowed value in the specification.
*/
#define IOL_T_T2_BIT                    3

/** \brief Response time of Devices.

See A.3.5 of \ref section_iol_spec_1_1.
This value is the delay after the last Master UART byte until the Device
starts with the response.
*/
#define IOL_T_TA_BIT                    11

/** \brief Master Message Delay.

See 7.3.2.3 of \ref section_iol_spec_1_1.
This is the number of bit times, the Master has to wait, before trying
another baud rate.
*/
#define IOL_T_TDMT_BIT                  28


/** \brief Receive enable delay.

See 5.3.3.3 of \ref section_iol_spec_1_1.
This delay is the wait time after the wake-up pulse, before
the communication is starting.
*/
#define IOL_T_REN_US                    500

/** \} */

#endif
