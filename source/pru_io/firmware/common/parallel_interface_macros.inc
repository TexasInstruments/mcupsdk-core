; Copyright (C) 2022 Texas Instruments Incorporated - http://www.ti.com/
;
; Redistribution and use in source and binary forms, with or without
; modification, are permitted provided that the following conditions
; are met:
;
; Redistributions of source code must retain the above copyright
; notice, this list of conditions and the following disclaimer.
;
; Redistributions in binary form must reproduce the above copyright
; notice, this list of conditions and the following disclaimer in the
; documentation and/or other materials provided with the
; distribution.
;
; Neither the name of Texas Instruments Incorporated nor the names of
; its contributors may be used to endorse or promote products derived
; from this software without specific prior written permission.
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

;************************************************************************************
;   File:     parallel_interface_macros.inc
;
;   Brief:    Parallel Interface related macro definitions
;
;   Includes:
;            icss_pin_macros.inc
;            time_macros.inc
;************************************************************************************
    .if    !$defined("__parallel_interface_macros_inc")
__parallel_interface_macros_inc    .set 1

    .include "icss_pin_macros.inc"
    .include "time_macros.inc"

;************************************************************************************
;   Macro: m_read_trigger
;
;   Drive the RD_PIN low for RD_HIGH_LOW_TIME period and then make it high again
;
;   PEAK cycles:
;           ~ cycle - depends on configuration
;
;   Pseudo code:
;       ========== C ==========
;       CLR(RD_PIN);
;       WAIT(RD_HIGH_LOW_TIME);
;       SET(RD_PIN);
;       WAIT(RD_HIGH_LOW_TIME);
;       =======================
;
;   Parameters:
;           RD_HIGH_LOW_TIME - Period to wait, in ns, after setting control (CS, RD) pins
;           RD_PIN           - PRU Read Pin to trigger the adc to send data (similar to SCLK)
;************************************************************************************
m_read_trigger      .macro  RD_HIGH_LOW_TIME, RD_PIN
    m_pru_clr_pin           RD_PIN
    m_wait_nano_sec         RD_HIGH_LOW_TIME
    m_pru_set_pin           RD_PIN
    m_wait_nano_sec         RD_HIGH_LOW_TIME
    .endm

;************************************************************************************
;   Macro: m_read_packet_parallel16_msb
;
;   Read a data packet from parallel interface with 16 data lines mapped to R31[15:0]
;   ENDIAN = MSB => MSB bits are received first and are aligned to higher pin numbers
;   Example: If receiving 10 bits = data[9:0], they should be available on R31[15:6]
;
;   PEAK cycles:
;           ~ cycle - depends on configuration
;
;   Pseudo code:
;       ========== C ==========
;       data_reg = value;
;       =======================
;
;   Parameters:
;           PACKETSIZE       - 1 to 32
;           RD_HIGH_LOW_TIME - Period to wait, in ns, after setting control (CS, RD) pins
;           data_reg         - Register to store read data packet (32 bit)
;           RD_PIN           - PRU Read Pin to trigger the adc to send data (similar to SCLK)
;************************************************************************************
m_read_packet_parallel16_msb    .macro  data_reg, PACKETSIZE, RD_HIGH_LOW_TIME, RD_PIN
    m_read_trigger  RD_HIGH_LOW_TIME, RD_PIN    ; send read signal
    mov             data_reg.w2, r31.w0         ; read data
    ; TODO: If we want to pass data_reg as only 16bit register?

    .if     PACKETSIZE > 16
    m_read_trigger  RD_HIGH_LOW_TIME, RD_PIN    ; send read signal
    mov             data_reg.w0, r31.w0         ; read data
    .endif  ; PACKETSIZE > 16

    lsr             data_reg, data_reg, (32 - PACKETSIZE)
    .endm

;************************************************************************************
;   Macro: m_read_packet_parallel16_lsb
;
;   Read a data packet from parallel interface with 16 data lines mapped to R31[15:0]
;   ENDIAN = LSB => LSB bits are received first and are aligned to lower pin numbers
;   Example: If receiving 10 bits = data[9:0], they should be available on R31[9:0]
;
;   PEAK cycles:
;           ~ cycle - depends on configuration
;
;   Pseudo code:
;       ========== C ==========
;       data_reg = value;
;       =======================
;
;   Parameters:
;           PACKETSIZE       - 1 to 32
;           RD_HIGH_LOW_TIME - Period to wait, in ns, after setting control (CS, RD) pins
;           data_reg         - Register to store read data packet (32 bit)
;           RD_PIN           - PRU Read Pin to trigger the adc to send data (similar to SCLK)
;************************************************************************************
m_read_packet_parallel16_lsb    .macro  data_reg, PACKETSIZE, RD_HIGH_LOW_TIME, RD_PIN
    .if     PACKETSIZE <= 16
    ; .if     $symcmp(PRE_READ_STEP,"CLR_BEFORE_READ") = 0
    ldi             data_reg, 0                 ; clear data_reg
    ; .endif  ; PRE_READ_STEP
    .endif      ; PACKETSIZE <= 16

    m_read_trigger  RD_HIGH_LOW_TIME, RD_PIN    ; send read signal
    mov             data_reg.w0, r31.w0         ; read data

    .if         PACKETSIZE > 16
    m_read_trigger  RD_HIGH_LOW_TIME, RD_PIN    ; send read signal
    mov             data_reg.w2, r31.w0         ; read data
    .endif  ; PACKETSIZE > 16

    .endm

;************************************************************************************
;   Macro: m_read_packet_parallel8_msb
;
;   Read a data packet from parallel interface with 8 data lines mapped to R31[7:0]
;   ENDIAN = MSB => MSB bits are received first and are aligned to higher pin numbers
;   Example: If receiving 4 bits = data[3:0], they should be available on R31[7:4]
;
;   PEAK cycles:
;           ~ cycle - depends on configuration
;
;   Pseudo code:
;       ========== C ==========
;       data_reg = value;
;       =======================
;
;   Parameters:
;           PACKETSIZE       - 1 to 32
;           RD_HIGH_LOW_TIME - Period to wait, in ns, after setting control (CS, RD) pins
;           data_reg         - Register to store read data packet (32 bit)
;           RD_PIN           - PRU Read Pin to trigger the adc to send data (similar to SCLK)
;************************************************************************************
m_read_packet_parallel8_msb    .macro  data_reg, PACKETSIZE, RD_HIGH_LOW_TIME, RD_PIN
    m_read_trigger  RD_HIGH_LOW_TIME, RD_PIN    ; send read signal
    mov             data_reg.b3, r31.b0         ; read data

    .if     PACKETSIZE > 8
    m_read_trigger  RD_HIGH_LOW_TIME, RD_PIN    ; send read signal
    mov             data_reg.b2, r31.b0         ; read data
    .endif  ; PACKETSIZE > 186

    .if     PACKETSIZE > 16
    m_read_trigger  RD_HIGH_LOW_TIME, RD_PIN    ; send read signal
    mov             data_reg.b1, r31.b0         ; read data
    .endif  ; PACKETSIZE > 16

    .if     PACKETSIZE > 24
    m_read_trigger  RD_HIGH_LOW_TIME, RD_PIN    ; send read signal
    mov             data_reg.b0, r31.b0         ; read data
    .endif  ; PACKETSIZE > 24

    lsr             data_reg, data_reg, (32 - PACKETSIZE)
; TODO: options for INTERFACE_REG(R31.bx)=b0,b1, RD_ACTIVE_LEVEL=HIGH,LOW
    .endm

;************************************************************************************
;   Macro: m_read_packet_parallel8_lsb
;
;   Read a data packet from parallel interface with 8 data lines mapped to R31[7:0]
;   ENDIAN = LSB => LSB bits are received first and are aligned to lower pin numbers
;   Example: If receiving 4 bits = data[3:0], they should be available on R31[3:0]
;
;   PEAK cycles:
;           ~ cycle - depends on configuration
;
;   Pseudo code:
;       ========== C ==========
;       data_reg = value;
;       =======================
;
;   Parameters:
;           PACKETSIZE       - 1 to 32
;           RD_HIGH_LOW_TIME - Period to wait, in ns, after setting control (CS, RD) pins
;           data_reg         - Register to store read data packet (32 bit)
;           RD_PIN           - PRU Read Pin to trigger the adc to send data (similar to SCLK)
;************************************************************************************
m_read_packet_parallel8_lsb    .macro  data_reg, PACKETSIZE, RD_HIGH_LOW_TIME, RD_PIN
    .if     PACKETSIZE <= 24
    ; .if     $symcmp(PRE_READ_STEP,"CLR_BEFORE_READ") = 0
    ldi             data_reg, 0                 ; clear data_reg
    ; .endif  ; PRE_READ_STEP
    .endif  ; PACKETSIZE <= 24

    m_read_trigger  RD_HIGH_LOW_TIME, RD_PIN    ; send read signal
    mov             data_reg.b0, r31.b0         ; read data

    .if     PACKETSIZE > 8
    m_read_trigger  RD_HIGH_LOW_TIME, RD_PIN    ; send read signal
    mov             data_reg.b1, r31.b0         ; read data
    .endif  ; PACKETSIZE > 186

    .if     PACKETSIZE > 16
    m_read_trigger  RD_HIGH_LOW_TIME, RD_PIN    ; send read signal
    mov             data_reg.b2, r31.b0         ; read data
    .endif  ; PACKETSIZE > 16

    .if     PACKETSIZE > 24
    m_read_trigger  RD_HIGH_LOW_TIME, RD_PIN    ; send read signal
    mov             data_reg.b3, r31.b0         ; read data
    .endif  ; PACKETSIZE > 24
; TODO: options for INTERFACE_REG(R31.bx)=b0,b1, RD_ACTIVE_LEVEL=HIGH,LOW
    .endm

;************************************************************************************
;   Macro: m_read_packet_parallel16
;
;   Read a data packet from parallel interface with 16 data lines mapped to R31[15:0]
;
;   PEAK cycles:
;           ~ cycle - depends on configuration
;
;   Pseudo code:
;       ========== C ==========
;       data_reg = value;
;       =======================
;
;   Parameters:
;           ENDIAN           - MSB : MSB Bits are transferred first
;                              LSB : LSB Bits are transferred first
;           PACKETSIZE       - 1 to 32
;           RD_HIGH_LOW_TIME - Period to wait, in ns, after setting control (CS, RD) pins
;           data_reg         - Register to store read data packet (32 bit)
;           RD_PIN           - PRU Read Pin to trigger the adc to send data (similar to SCLK)
;************************************************************************************
m_read_packet_parallel16    .macro  ENDIAN, data_reg, PACKETSIZE, RD_HIGH_LOW_TIME, RD_PIN
    .if         $symcmp(ENDIAN,"MSB") = 0
    m_read_packet_parallel16_msb    data_reg, PACKETSIZE, RD_HIGH_LOW_TIME, RD_PIN

    .elseif     $symcmp(ENDIAN,"LSB") = 0
    m_read_packet_parallel16_lsb    data_reg, PACKETSIZE, RD_HIGH_LOW_TIME, RD_PIN
    .endif      ; ENDIAN

    .endm

;************************************************************************************
;   Macro: m_read_packet_parallel8
;
;   Read a data packet from parallel interface with 8 data lines mapped to R31[7:0]
;
;   PEAK cycles:
;           ~ cycle - depends on configuration
;
;   Pseudo code:
;       ========== C ==========
;       data_reg = value;
;       =======================
;
;   Parameters:
;           ENDIAN           - MSB : MSB Bits are transferred first
;                              LSB : LSB Bits are transferred first
;           PACKETSIZE       - 1 to 32
;           RD_HIGH_LOW_TIME - Period to wait, in ns, after setting control (CS, RD) pins
;           data_reg         - Register to store read data packet (32 bit)
;           RD_PIN           - PRU Read Pin to trigger the adc to send data (similar to SCLK)
;************************************************************************************
m_read_packet_parallel8     .macro  ENDIAN, data_reg, PACKETSIZE, RD_HIGH_LOW_TIME, RD_PIN
    .if         $symcmp(ENDIAN,"MSB") = 0
    m_read_packet_parallel8_msb     data_reg, PACKETSIZE, RD_HIGH_LOW_TIME, RD_PIN

    .elseif     $symcmp(ENDIAN,"LSB") = 0
    m_read_packet_parallel8_lsb     data_reg, PACKETSIZE, RD_HIGH_LOW_TIME, RD_PIN
    .endif      ; ENDIAN

    .endm

;************************************************************************************
;   Macro: m_parallel_read_packet
;
;   Read as data packet from parallel interface with following configurations
;    - INTERFACE, ENDIAN, DATAWIDTH
;
;   PEAK cycles:
;           ~ cycle - depends on configuration
;
;   Pseudo code:
;       ========== C ==========
;       data_reg = value;
;       =======================
;
;   Parameters:
;           ENDIAN           - MSB : MSB Bits are transferred first
;                              LSB : LSB Bits are transferred first
;           INTERFACE        - PARALLEL16 : 16 bit paralllel interface
;                              PARALLEL8  : 8 bit paralllel interface
;           DATAWIDTH        - 1 to 32
;           RD_HIGH_LOW_TIME - Period to wait, in ns, after setting control (CS, RD) pins
;           data_reg         - Register to store read data packet (32 bit)
;************************************************************************************
m_parallel_read_packet .macro  ENDIAN, INTERFACE, DATAWIDTH, RD_HIGH_LOW_TIME, data_reg, RD_PIN
    .if     $symcmp(ENDIAN,"MSB") = 0
    .if     $symcmp(INTERFACE,"PARALLEL16") = 0
    ; send read signal
    m_pru_clr_pin           RD_PIN
    m_wait_nano_sec         RD_HIGH_LOW_TIME
    m_pru_set_pin           RD_PIN
    m_wait_nano_sec         RD_HIGH_LOW_TIME
    ; read data
    mov     data_reg.w2, r31.w0

    .if     DATAWIDTH > 16
    ; send read signal
    m_pru_clr_pin           RD_PIN
    m_wait_nano_sec         RD_HIGH_LOW_TIME
    m_pru_set_pin           RD_PIN
    m_wait_nano_sec         RD_HIGH_LOW_TIME
    ; read data
    mov     data_reg.w0, r31.w0
    .endif  ; DATAWIDTH > 16

    lsr     data_reg, data_reg, (32 - DATAWIDTH)

    .elseif     $symcmp(INTERFACE,"PARALLEL8") = 0
    ; send read signal
    m_pru_clr_pin           RD_PIN
    m_wait_nano_sec         RD_HIGH_LOW_TIME
    m_pru_set_pin           RD_PIN
    m_wait_nano_sec         RD_HIGH_LOW_TIME
    ; read data
    mov     data_reg.b3, r31.b0

    .if     DATAWIDTH > 8
    ; send read signal
    m_pru_clr_pin           RD_PIN
    m_wait_nano_sec         RD_HIGH_LOW_TIME
    m_pru_set_pin           RD_PIN
    m_wait_nano_sec         RD_HIGH_LOW_TIME
    ; read data
    mov     data_reg.b2, r31.b0
    .endif  ; DATAWIDTH > 186

    .if     DATAWIDTH > 16
    ; send read signal
    m_pru_clr_pin           RD_PIN
    m_wait_nano_sec         RD_HIGH_LOW_TIME
    m_pru_set_pin           RD_PIN
    m_wait_nano_sec         RD_HIGH_LOW_TIME
    ; read data
    mov     data_reg.b1, r31.b0
    .endif  ; DATAWIDTH > 16

    .if     DATAWIDTH > 24
    ; send read signal
    m_pru_clr_pin           RD_PIN
    m_wait_nano_sec         RD_HIGH_LOW_TIME
    m_pru_set_pin           RD_PIN
    m_wait_nano_sec         RD_HIGH_LOW_TIME
    ; read data
    mov     data_reg.b0, r31.b0
    .endif  ; DATAWIDTH > 24

    lsr     data_reg, data_reg, (32 - DATAWIDTH)
    .endif  ; INTERFACE = PARALLEL8

    .elseif     $symcmp(ENDIAN,"LSB") = 0
    .if     $symcmp(INTERFACE,"PARALLEL16") = 0

    .if     DATAWIDTH <= 16
    ; clear data_reg
    ldi     data_reg, 0
    .endif  ; DATAWIDTH <= 16

    ; send read signal
    m_pru_clr_pin           RD_PIN
    m_wait_nano_sec         RD_HIGH_LOW_TIME
    m_pru_set_pin           RD_PIN
    m_wait_nano_sec         RD_HIGH_LOW_TIME
    ; read data
    mov     data_reg.w0, r31.w0

    .if     DATAWIDTH > 16
    ; send read signal
    m_pru_clr_pin           RD_PIN
    m_wait_nano_sec         RD_HIGH_LOW_TIME
    m_pru_set_pin           RD_PIN
    m_wait_nano_sec         RD_HIGH_LOW_TIME
    ; read data
    mov     data_reg.w2, r31..w0
    .endif  ; DATAWIDTH > 16

    .elseif     $symcmp(INTERFACE,"PARALLEL8") = 0

    .if     DATAWIDTH <= 24
    ; clear data_reg
    ldi     data_reg, 0
    .endif  ; DATAWIDTH <= 24

    ; send read signal
    m_pru_clr_pin           RD_PIN
    m_wait_nano_sec         RD_HIGH_LOW_TIME
    m_pru_set_pin           RD_PIN
    m_wait_nano_sec         RD_HIGH_LOW_TIME
    ; read data
    mov     data_reg.b0, r31.b0

    .if     DATAWIDTH > 8
    ; send read signal
    m_pru_clr_pin           RD_PIN
    m_wait_nano_sec         RD_HIGH_LOW_TIME
    m_pru_set_pin           RD_PIN
    m_wait_nano_sec         RD_HIGH_LOW_TIME
    ; read data
    mov     data_reg.b1, r31.b0
    .endif  ; DATAWIDTH > 186

    .if     DATAWIDTH > 16
    ; send read signal
    m_pru_clr_pin           RD_PIN
    m_wait_nano_sec         RD_HIGH_LOW_TIME
    m_pru_set_pin           RD_PIN
    m_wait_nano_sec         RD_HIGH_LOW_TIME
    ; read data
    mov     data_reg.b2, r31.b0
    .endif  ; DATAWIDTH > 16

    .if     DATAWIDTH > 24
    ; send read signal
    m_pru_clr_pin           RD_PIN
    m_wait_nano_sec         RD_HIGH_LOW_TIME
    m_pru_set_pin           RD_PIN
    m_wait_nano_sec         RD_HIGH_LOW_TIME
    ; read data
    mov     data_reg.b3, r31.b0
    .endif  ; DATAWIDTH > 24

    .endif  ; INTERFACE = PARALLEL8
    .endif  ; ENDIAN    = LSB

    .endm

;************************************************************************************
;   Macro: m_parallel_read_frame
;
;   Read as data frame composed of data packets from parallel interface
;
;   PEAK cycles:
;           ~ cycle - depends on configuration
;
;   Pseudo code:
;       ========== C ==========
;       data_reg = value;
;       =======================
;
;   Parameters:
;           ENDIAN     - MSB : MSB Bits are transferred first
;                        LSB : LSB Bits are transferred first
;           INTERFACE  - PARALLEL16 : 16 bit paralllel interface
;                        PARALLEL8  : 8 bit paralllel interface
;           DATAWIDTH  - 1 to 32
;           RD_HIGH_LOW_TIME   - Period to wait, in ns, after setting control (CS, RD) pins
;           data_reg   - Register to store read data packet (32 bit)
;************************************************************************************
m_parallel_read_frame   .macro  NO_OF_PACKETS, dataRegBaseAddr, tempReg, ENDIAN, INTERFACE, DATAWIDTH, RD_HIGH_LOW_TIME, data_reg
    .if         0
    ; set /CS low
    m_pru_clr_pin       CS_PIN
    m_wait_nano_sec     WAIT_CSRD
    .endif

    ;-----------------------------------------------
    ; Memory Optimized Code
    ;-----------------------------------------------
    .if         0
    .var    packetId
    .asg    tempReg.b0,    packetId

    ; Can we use this loop instruction? noninterruptible loop meaning?
    ; loop EndLoop, NO_OF_PACKETS
    ldi     packetId, 0
READ_FRAME_LOOP:
    m_parallel_read_packet  ENDIAN, INTERFACE, DATAWIDTH, RD_HIGH_LOW_TIME, data_reg
    ; .if         DATAWIDTH <= 8
    ; mvib    *dataRegBaseAddr++, data_reg
    ; .elseif     DATAWIDTH <= 16
    ; mviw    *dataRegBaseAddr++, data_reg
    ; .else
    mvid    *dataRegBaseAddr++, data_reg
    ; .endif  ; DATAWIDTH <= 8
    add     packetId, packetId, 1
    qbne    READ_FRAME_LOOP, packetId, NO_OF_PACKETS
    .endif

    ;-----------------------------------------------
    ; Speed Optimized Code
    ;-----------------------------------------------
    .if         1
    .eval       0, i
    .loop       NO_OF_PACKETS
    .if         0
    m_parallel_read_packet  ENDIAN, INTERFACE, DATAWIDTH, RD_HIGH_LOW_TIME, data_reg
    mvid    *dataRegBaseAddr++, data_reg
    .endif
    .if         1
    ; TODO: check if this works: data_reg_:i:
    m_parallel_read_packet  ENDIAN, INTERFACE, DATAWIDTH, RD_HIGH_LOW_TIME, data_reg_:i:
    .endif
    .eval       i+1, i
    .endloop    ; NO_OF_PACKETS
    .endif

    .if         0
    ; set /CS high
    m_pru_set_pin       CS_PIN
    m_wait_nano_sec     WAIT_CSRD
    .endif

    .endm

    .endif  ; __parallel_interface_macros_inc