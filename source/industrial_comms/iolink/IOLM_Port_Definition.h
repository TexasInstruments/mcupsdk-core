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

#ifndef INC_PROT__IOLM_PORT_DEFINITION_H__
#define INC_PROT__IOLM_PORT_DEFINITION_H__

/**
 \addtogroup group_iolm_port_definition IO-Link Master port definitions
 \{
 */

/** \brief Number of ports, which are operated by the IO-Link Master */
#define IOLM_PORT_COUNT             8

/** \brief Interval of the SysTick timer in milliseconds

 The SysTick is used for the internal software timers. The function
 #IOLM_Port_s32SysTickVal() requests a timer value. This parameter
 defines the tick increments of the timer. it should be between
 1ms and 10ms.

 */
#define IOLM_SYSTICK_INTERVAL_MS    1

/** \brief Retry interval of wake-up sequence.

 see 7.3.2.2 of \ref section_iol_spec_1_1. */
#define IOLM_WAKE_RETRY_TIMEOUT     40

/** \brief Mainloop time

 Master processing time is the maximum time between receiving data (rx complete interrupt)
 and the next mainloop processing in microseconds.

 This value should be set to the maximum interval between two #IOLM_API_vRun calls.

 If #IOLM_IRQ_PROCESSING is enabled, this value equals the min cycle time of the
 Mmster. If not, the min cycle time of the Master is this processing time + the message
 sequence time on the line. */
#define IOLM_MASTER_PROCESSING_TIME     10

/** \brief Wake-up sequence retry interval should be 500 to 1000ms

See table 40  of \ref section_iol_spec_1_1 */
#define IOLM_MASTER_RETRY_INTERVAL      700

/** \brief Process cyclic frames in IRQ context and not in mainloop

 If this value is set, the interpretation of the rx frame is done in
 the function #IOLM_API_PL_vTransferInd() and the transmit frame
 is prepared in the function #IOLM_API_vTimerExpired(). This
 functions normally run in interrupt context. With this mode are
 short cycle times possible (400 us) but with the cost of a higher
 interrupt load. It ensures also that always the most recent
 Process Data is available in both directions.

 If this value is set to zero, the frame processing and preparation
 is done in the mainloop. Interrupts are only used to indicate events
 and trigger the start of data. This mode decreases the jitter on the
 line, allows more Devices, but with the cost of slower cycle times
 (>= 1ms) and a output Process Data delay on very slow sensors
 (cycletime > 20ms). This is because the tx frame is prepared if the last
 frame is received and transfered on the next cycle interrupt.
 */
#define IOLM_IRQ_PROCESSING         0

/** \brief Enable the Data Storage mechanism */
#define IOLM_DS_ENABLE              1

/** \brief Increase the MSeq timeout by n microseconds

 This is an additional time between the the receive of the last byte in a frame and start of the
 next frame. Increasing this, reduces the min cycle time, but also makes the communication
 more reliable on multi port implementation or heavy interrupt load.
 */
#define IOLM_ADDITIONAL_MSEQ_TIMEOUT 10

/** \brief Cycle jitter value

 This value is to adjust the cycle jitter into a valid range.
 According specification a cycle time with +0% to +10% is allowed.
 The stack tries to reach the 0%. If the output driver jitters, it is
 possible that the cycle time is below 0%. For this an additional time
 in microseconds can be added by this value. There are values for COM1,
 COM2, COM3. The value can also be negative (useful for COM3)
 */
#define IOLM_CYCLE_JITTER_US \
    IOL_BIT_USEC(0, IOL_BAUD_COM1), \
    IOL_BIT_USEC(0, IOL_BAUD_COM2), \
    IOL_BIT_USEC(0, IOL_BAUD_COM3)

/** \brief Number of Process Data buffers

 Depending on the consistency requirements, different values can be selected. Valid values:
 - 1: Input data is only valid during IOLM_Port_AL_vNewInput() call. Output data
 has to be set in IOLM_Port_AL_vPDCycle().
 - 2: Ping Pong Buffer. Input data can be read every time and is valid at least
 one IOL cycle. Output data can be set once during a cycle.
 - 3: Input data is valid until next call of IOLM_API_AL_eGetInput().
 Output data can be set every time.

 */
#define IOLM_PD_BUFFER_COUNT        3

/** \brief PD cycle indication as fast as possible

 If this value is set, the functions #IOLM_Port_AL_vPDCycle and
 #IOLM_Port_AL_vNewInput() are called in the receive functions of the stack.
 This can be in interrupt context. If
 the user application needs a lot of time, this can delay the IO-Link cycle.
 It should be used if the user application needs the Process Data as fast
 as possible.

 If the value is set to 0, the functions are called in the following
 mainloop call and not in interrupt context. It guarantees that the time
 critical operations are handled before the indication to the user application.
 */
#define IOLM_PD_CYCLE_IRQ           0

/** \brief Wake-up pulse timer length in microseconds (pulse needs to be 75-85 us).

 This value is used to generate the wake-up pulse by a timer.
 It does not include the additional hardware delays and maybe additional
 jitter because of other interrupts. It is recommended to use a hardware
 timer with output compare unit to generate this pulse if
 #IOLM_Port_PL_eWakeUp() returns #IOLM_eWakeupMode_Stack.
 */
#define IOLM_WAKEUP_PULSE_TIMER_US  80


#define IOLM_THREAD_PRIO_MAIN       5
#define IOLM_THREAD_PRIO_HIGH       6

#define IOLM_SMI_MAX_DIAG_ENTRIES   10
#define IOLM_SMIEXT_ENABLED         1

#define IOLM_SMI_CLIENT_APP         1


#define IOLM_SMI_MASTER_ID      0x00, 0x00, 0x00, 0x05

/** \} */

#endif
