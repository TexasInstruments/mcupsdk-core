; Copyright (C) 2022 Texas Instruments Incorporated
;
; Redistribution and use in source and binary forms, with or without
; modification, are permitted provided that the following conditions
 ; are met:
 ;
 ;   Redistributions of source code must retain the above copyright
 ;   notice, this list of conditions and the following disclaimer.
 ;
 ;   Redistributions in binary form must reproduce the above copyright
 ;   notice, this list of conditions and the following disclaimer in the
 ;   documentation and/or other materials provided with the
 ;   distribution.
 ;
 ;   Neither the name of Texas Instruments Incorporated nor the names of
 ;   its contributors may be used to endorse or promote products derived
 ;   from this software without specific prior written permission.
 ;
 ; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 ; "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 ; LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 ; A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 ; OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 ; SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 ; LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 ; DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 ; THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 ; (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 ; OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ;

	.if	!$isdefed("__tamagawa_constants_h")
__tamagawa_constants_h	.set	1

NON_EEPROM_TX_FRAME_SIZE      .set    0x0A  ;Tx Frame size for Non-EEPROM command = 10 bits
EEPROM_READ_TX_FRAME_SIZE     .set    0x1E  ;Tx Frame size for EEPROM Read command = 30 bits
EEPROM_WRITE_TX_FRAME_SIZE    .set    0x00  ;Tx Frame size for EEPROM Write command = 0 bits (for Continuous FIFO loading)

NON_EEPROM_CMD_ID     .set    0x00  ;non-EEPROM command id
EEPROM_READ_CMD_ID    .set    0x08  ;EEPROM Read command ID
EEPROM_WRITE_CMD_ID    .set    0x04 ;EEPROM Write command ID

BIT_CLEAR_MASK    .set    0x07  ;Mask to clear the bits for Tx frame size
CHANNEL_CHECK_MASK    .set    0x03  ;Mask to check the channels selected
POLLING_MASK    .set    0x1C    ;Mask to poll the required bits in Tx Continuous FIFO loading
THREE_BYTES_FIFO_LEVEL    .set    0x0C  ;Mask to check whether the Tx FIFO level is at 3 bytes

	.endif
