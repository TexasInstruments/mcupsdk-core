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
;   File:   pru_ipc_macros.inc
;
;   Register Memory Used:
;           26 Bytes (6.5 regs)
;
;   DRAM Memory Used:
;           (8 + 4*noOfBuffers) Bytes
;************************************************************************************
    .if    !$defined("__pru_ipc_macros_inc")
__pru_ipc_macros_inc    .set 1

PRU_IPC_CONFIG_MEM    .set   PRU_DMEM_ADDR

; ---------------------------------------------
;  Configuration parameters memory map:
;                          PRU_IPC_CONFIG_MEM
;                            0|---------|
;  Last block ID           -  | 2 Bytes |
;  Data Size               -  | 1 Bytes |
;  Number of Buffers       -  | 1 Bytes |
;                            4|---------|
;  Block Size              -  | 2 Bytes |
;  Number of Blocks        -  | 2 Bytes |
;                            8|---------|
;  Buffer 0 - (n-1) base   -  | 4 Bytes |
;       addresses           12|---------|
;          ...                |   ...   |
; ---------------------------------------------

; Defines for memory offsets for configurables:
BLOCKID_OFFSET          .set    0   ; 2 byte length
DATASIZE_OFFSET         .set    2   ; 1 byte length
NOOFBUFFERS_OFFSET      .set    3   ; 1 byte length
BLOCKSIZE_OFFSET        .set    4   ; 2 byte length
NOOFBLOCKS_OFFSET       .set    6   ; 2 byte length
BUFFERBASE_OFFSET       .set    8   ; 4 byte length


;*********************************** STRUCTURES *************************************

;************************************************************************************
;   Structure: s_PruIpc_config
;
;   Strucutre to store the configurable parameters for the Pru Ipc Instance
;
;   Size:
;           5 bytes
;
;   Elements:
;           blockSize      -   Number of data packets per Block
;           noOfBlocks     -   Number of Blocks per Buffer
;           noOfBuffers    -   Maximum Number of Active Buffers
;
;   See Also:
;           dataSize       -   Size of the data packets
;************************************************************************************
s_PruIpc_config         .struct
noOfBuffers             .ubyte   ; max 255 Buffers
blockSize               .ushort  ; max 65535 data packets
noOfBlocks              .ushort  ; max 255 blocks
s_PruIpc_config_len     .endstruct

;************************************************************************************
;   Structure: s_PruIpc_iterator
;
;   Strucutre to store the iterators for the Pru Ipc Instance
;
;   Size:
;           9 bytes
;
;   Elements:
;           packet     -   To track data packet index
;           block      -   To track block index
;           buffer     -   To track buffer index
;           idxInBlock -   To track index inside blocks
;
;   See Also:
;************************************************************************************
s_PruIpc_iterator       .struct
buffer                  .ubyte
block                   .ushort
packet                  .ulong   ; size = block size * num of blocks = 2^16 * 2^16 => min 4 bytes
idxInBlock              .ushort
s_PruIpc_iterator_len   .endstruct

;************************************************************************************
;   Structure: s_PruIpc_register
;
;   Strucutre to store temporary registers for the Pru Ipc Instance
;
;   Size:
;           6 bytes
;
;   Elements:
;           bufferBaseAddr   -   Stores base address for buffer buffers
;           bufferOffset     -   Stores memory offset for TMP.bufferBaseAddr
;
;   See Also:
;************************************************************************************
s_PruIpc_register     .struct
bufferOffset          .ushort   ; max 65536/4 buffer addresses
bufferBaseAddr        .ulong    ; 4 byte addresses
s_PruIpc_register_len .endstruct


;******************** REGISTER BINDINGS for structure instances *********************

;************************************************************************************
;   Structure Instance: CFG
;   Struct used for storing the Pru Ipc configurable parameters
;
;   Registers:
;           5 bytes => R21.b3, R22
;
;   See Also:
;************************************************************************************
CFG         .sassign    R21.b3,  s_PruIpc_config

;************************************************************************************
;   Structure Instance: ITR
;   Struct used for storing the iterators for the Pru Ipc Instance
;
;   Registers:
;           9 bytes => R23.b1, R23.w2, R24, R25.w0
;
;   See Also:
;************************************************************************************
ITR         .sassign    R23.b1,  s_PruIpc_iterator

;************************************************************************************
;   Structure Instance: TMP
;   Struct used for storing temporary registers for the Pru Ipc Instance
;
;   Registers:
;           6 bytes => R25.w2, R26
;
;   See Also:
;************************************************************************************
TMP         .sassign    R25.w2,  s_PruIpc_register


;************************************* MACROS ***************************************

;************************************************************************************
;   Macro: m_pru_ipc_init
;
;   Initialization code when the Pru Ipc is started
;
;   PEAK cycles:
;           11 cycle
;
;   Invokes:
;           None
;
;   Pseudo code:
;           ========== C ==========
;           memcpy(offset(dataSize), PRU_IPC_CONFIG_MEM + DATASIZE_OFFSET, 1);
;           memcpy(offset(CFG.blockSize), PRU_IPC_CONFIG_MEM + BLOCKSIZE_OFFSET, 4);
;           ITR = 0;
;           =======================
;
;   Parameters:
;           dataSize       -   Size of the data packets (must dataSize = bx)
;           baseMemAddr    -   register for storing BASE_MEM_ADDR in
;           BASE_MEM_ADDR  -   Base addr of config memory
;   Returns:
;           None
;
;   See Also:
;************************************************************************************
m_pru_ipc_init   .macro  dataSize, baseMemAddr, BASE_MEM_ADDR
    ldi32   baseMemAddr, BASE_MEM_ADDR
    lbbo    &dataSize,        baseMemAddr, DATASIZE_OFFSET,    1
    lbbo    &CFG.noOfBuffers, baseMemAddr, NOOFBUFFERS_OFFSET, 5     ; can we use s_PruIpc_config_len
    ldi     ITR.packet, 0
    ldi     ITR.block,  0
    ldi     ITR.idxInBlock, 0
    .endm

;************************************************************************************
;   Macro: m_pru_ipc_send
;
;   Sends the data packets to the shared memory for each Buffer
;   Creates interrupt event for r5f core to read data packets.
;
;   PEAK cycles:
;           (15 + 10*noOfBuffers) cycle
;
;   Invokes:
;           None
;
;   Pseudo code:
;           ========================================== C =========================================
;           for (ITR.buffer = 0, TMP.bufferOffset = BUFFERBASE_OFFSET;
;                                       ITR.buffer < CFG.noOfBuffers;
;                                        ITR.buffer++, TMP.bufferOffset+=4) {
;               dataReg = *dataRegBaseAddr++;
;               memcpy(offset(TMP.bufferBaseAddr), PRU_IPC_CONFIG_MEM + TMP.bufferOffset, 4);
;               if (TMP.bufferBaseAddr == 0)     continue;
;               memcpy(offset(dataReg), TMP.bufferBaseAddr + ITR.packet, dataSize);
;           }
;           ITR.packet += dataSize;
;           ITR.idxInBlock++;
;
;           if (ITR.idxInBlock == CFG.blockSize) {
;               memcpy(offset(ITR.block), PRU_IPC_CONFIG_MEM + BLOCKID_OFFSET, 1);
;               r31.b0 = INT_NUM;  // generate interrupt
;               ITR.idxInBlock = 0;
;               ITR.block++;
;               if (ITR.block == CFG.noOfBlocks) {
;                   ITR.block  = 0
;                   ITR.packet = 0
;               }
;           }
;           ======================================================================================
;
;   Parameters:
;           dataRegBaseAddr    - (8 bits) Address of starting ADC data register (&ADC_DATA_REG_1)
;                                   {ADC_DATA_REG_1, ADC_DATA_REG_2, .... ADC_DATA_REG_N}
;                                   => register R1.bx must be used
;           dataReg            -   Data stored in ADC_DATA_REG_X, Can pass ADC_DATA_REG_0
;           dataSize           -   Size of the data packets (must dataSize = bx)
;           INTR_ENABLE        -   Enable interrupt trigger to specified event
;           INTR_NUM           -   Interrupt value to write to R31 to trigger interrupt event
;
;   Returns:
;           None
;
;   See Also:
;************************************************************************************
m_pru_ipc_send   .macro  dataRegBaseAddr, dataReg, dataSize, baseMemAddr, INTR_ENABLE, INTR_NUM
    add     TMP.bufferOffset, baseMemAddr, BUFFERBASE_OFFSET

    loop    BUFFERS_DATA_SENT?, CFG.noOfBuffers
    mvid    dataReg, *dataRegBaseAddr++
    lbbo    &TMP.bufferBaseAddr, baseMemAddr, TMP.bufferOffset, 4  ; load that buffer base address
    qbeq    SKIP_BUFFER?, TMP.bufferBaseAddr, 0                           ; if NULL skip this buffer write
    ; Is this needed to support skip on null pointer?
    sbbo    &dataReg, TMP.bufferBaseAddr, ITR.packet, b0                  ; dataSize = bx
SKIP_BUFFER?:
    add     TMP.bufferOffset, TMP.bufferOffset, 4
BUFFERS_DATA_SENT?:

    add     ITR.packet, ITR.packet, dataSize
    add     ITR.idxInBlock, ITR.idxInBlock, 1

    qbne    SKIP_INTERRUPT_ISSUE?, ITR.idxInBlock, CFG.blockSize
    sbbo    &ITR.block, baseMemAddr, BLOCKID_OFFSET, 2
    .if     INTR_ENABLE = 1
    ldi     r31.b0, INTR_NUM                 ; - interrupt r5f core
    .endif
    ldi     ITR.idxInBlock,  0
    add     ITR.block, ITR.block, 1

    qbgt    SKIP_INDEX_RESET?, ITR.block, CFG.noOfBlocks
    ldi     ITR.block,  0
    ldi     ITR.packet, 0
SKIP_INTERRUPT_ISSUE?:
SKIP_INDEX_RESET?:
    .endm

;************************************************************************************
;   Macro: m_pru_ipc_rcv
;
;   Receives the data packets from the shared memory for each Buffer
;   Handles interrupt event from r5f core, to read data packets.
;
;   PEAK cycles:
;           (15 + 10*noOfBuffers) cycle
;
;   Invokes:
;           None
;
;   Pseudo code:
;           ========================================== C =========================================
;           for (ITR.buffer = 0, TMP.bufferOffset = BUFFERBASE_OFFSET;
;                                       ITR.buffer < CFG.noOfBuffers;
;                                        ITR.buffer++, TMP.bufferOffset+=4) {
;               dataReg = *dataRegBaseAddr++;
;               memcpy(offset(TMP.bufferBaseAddr), PRU_IPC_CONFIG_MEM + TMP.bufferOffset, 4);
;               if (TMP.bufferBaseAddr == 0)     continue;
;               memcpy(TMP.bufferBaseAddr + ITR.packet, offset(dataReg), dataSize);
;           }
;           ITR.packet += dataSize;
;           ITR.idxInBlock++;
;
;           if (ITR.idxInBlock == CFG.blockSize) {
;               memcpy(offset(ITR.block), PRU_IPC_CONFIG_MEM + BLOCKID_OFFSET, 1);
;               r31.b0 = INT_NUM;  // generate interrupt
;               ITR.idxInBlock = 0;
;               ITR.block++;
;               if (ITR.block == CFG.noOfBlocks) {
;                   ITR.block  = 0
;                   ITR.packet = 0
;               }
;           }
;           ======================================================================================
;
;   Parameters:
;           dataRegBaseAddr    - (8 bits) Address of starting ADC data register (&ADC_DATA_REG_1)
;                                   {ADC_DATA_REG_1, ADC_DATA_REG_2, .... ADC_DATA_REG_N}
;                                   => register R1.bx must be used
;           dataReg            -   Data stored in ADC_DATA_REG_X, Can pass ADC_DATA_REG_0
;           dataSize           -   Size of the data packets (must dataSize = bx)
;           INTR_ENABLE        -   Enable interrupt trigger to specified event
;           EVENT_NUM          -   Event number value on which interrupt will be generated
;
;   Returns:
;           None
;
;   See Also:
;************************************************************************************
m_pru_ipc_rcv       .macro  dataRegBaseAddr, dataReg, dataSize, baseMemAddr, temp_reg, INTR_ENABLE, EVENT_NUM
    add     TMP.bufferOffset, baseMemAddr, BUFFERBASE_OFFSET

    loop    BUFFERS_DATA_RCVD?, CFG.noOfBuffers
    lbbo    &TMP.bufferBaseAddr, baseMemAddr, TMP.bufferOffset, 4  ; load that buffer base address
    lbbo    &dataReg, TMP.bufferBaseAddr, ITR.packet, b0           ; dataSize = bx
    mvid    *dataRegBaseAddr++, dataReg
    add     TMP.bufferOffset, TMP.bufferOffset, 4
BUFFERS_DATA_RCVD?:

    add     ITR.packet, ITR.packet, dataSize
    add     ITR.idxInBlock, ITR.idxInBlock, 1

    qbne    SKIP_INTERRUPT_ISSUE?, ITR.idxInBlock, CFG.blockSize
    sbbo    &ITR.block, baseMemAddr, BLOCKID_OFFSET, 2
    .if     INTR_ENABLE = 1
    ldi     temp_reg, EVENT_NUM
    sbco    &temp_reg, ICSS_INTC_CONST, 0x24, 4
    .endif
    ldi     ITR.idxInBlock,  0
    add     ITR.block, ITR.block, 1

    qbgt    SKIP_INDEX_RESET?, ITR.block, CFG.noOfBlocks
    ldi     ITR.block,  0
    ldi     ITR.packet, 0
SKIP_INTERRUPT_ISSUE?:
SKIP_INDEX_RESET?:
    .endm

    .endif  ; __pru_ipc_macros_inc
