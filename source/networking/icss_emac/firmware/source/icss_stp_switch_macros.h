;
;  TEXAS INSTRUMENTS TEXT FILE LICENSE
; 
;   Copyright (c) 2017-2019 Texas Instruments Incorporated
; 
;  All rights reserved not granted herein.
;  
;  Limited License.  
; 
;  Texas Instruments Incorporated grants a world-wide, royalty-free, non-exclusive 
;  license under copyrights and patents it now or hereafter owns or controls to 
;  make, have made, use, import, offer to sell and sell ("Utilize") this software 
;  subject to the terms herein.  With respect to the foregoing patent license, 
;  such license is granted  solely to the extent that any such patent is necessary 
;  to Utilize the software alone.  The patent license shall not apply to any 
;  combinations which include this software, other than combinations with devices 
;  manufactured by or for TI (“TI Devices”).  No hardware patent is licensed hereunder.
; 
;  Redistributions must preserve existing copyright notices and reproduce this license 
;  (including the above copyright notice and the disclaimer and (if applicable) source 
;  code license limitations below) in the documentation and/or other materials provided 
;  with the distribution.
;  
;  Redistribution and use in binary form, without modification, are permitted provided 
;  that the following conditions are met:
; 	No reverse engineering, decompilation, or disassembly of this software is 
;   permitted with respect to any software provided in binary form.
; 	Any redistribution and use are licensed by TI for use only with TI Devices.
; 	Nothing shall obligate TI to provide you with source code for the software 
;   licensed and provided to you in object code.
;  
;  If software source code is provided to you, modification and redistribution of the 
;  source code are permitted provided that the following conditions are met:
; 	Any redistribution and use of the source code, including any resulting derivative 
;   works, are licensed by TI for use only with TI Devices.
; 	Any redistribution and use of any object code compiled from the source code
;   and any resulting derivative works, are licensed by TI for use only with TI Devices.
; 
;  Neither the name of Texas Instruments Incorporated nor the names of its suppliers 
;  may be used to endorse or promote products derived from this software without 
;  specific prior written permission.
; 
;  DISCLAIMER.
; 
;  THIS SOFTWARE IS PROVIDED BY TI AND TI’S LICENSORS "AS IS" AND ANY EXPRESS OR IMPLIED 
;  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY 
;  AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL TI AND TI’S 
;  LICENSORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
;  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE 
;  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
;  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
;  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF vTHIS SOFTWARE, 
;  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
; 
; file:   icss_stp_switch_macros.h
;
; brief:  Macros used for firmware-based ICSS Ethernet switch
;
;
;

    .if !$defined("__icss_stp_switch_macros_hp")	
__icss_stp_switch_macros_hp    .set  1

;*******************************************************************************
; ASM Includes
;*******************************************************************************
    .include "emac_MII_Rcv.h"

;*******************************************************************************
; C Includes
;*******************************************************************************
    .cdecls C,NOLIST
%{
#include "icss_stp_switch.h"
%}

;*******************************************************************************
; Macros
;*******************************************************************************
    
;*******************************************************************************
M_MAC_HASH    .macro    MacAddr01, MacAddr23, MacAddr45, out_byte
;-------------------------------------------------------------------------------
; Description:  Calculate MAC address hash
; Input Parameters: MacAddr01 - register containing first 2 bytes of MAC address
;                   MacAddr23 - register word containing next 2 bytes of MAC address
;                   MacAddr45 - register word containing last 2 bytes of MAC address
; Use of registers: RCV_TEMP_REG_1
; Peak PRU cycles : 4
; output registers : RCV_TEMP_REG_2.b0
;-------------------------------------------------------------------------------
    .asg  RCV_TEMP_REG_1, _Tmp  ; Temporary register
;-------------------------------------------------------------------------------

    ; Hash function is to XOR all MAC bytes together

    ; Hash first two bytes
    MOV   _Tmp.w0, MacAddr01  ; This word technically comes from a struct, so can't use *.b0. Move to register
    MOV   out_byte, _Tmp.b0
    XOR   out_byte, out_byte, _Tmp.b1

    ; Hash next two bytes
    MOV   _Tmp.w0, MacAddr23  ; This word technically comes from a struct, so can't use *.b0. Move to register
    XOR   out_byte, out_byte, _Tmp.b0
    XOR   out_byte, out_byte, _Tmp.b1

    ; Hash last two bytes
    MOV   _Tmp.w0, MacAddr45  ; This word technically comes from a struct, so can't use *.b0. Move to register
    XOR   out_byte, out_byte, _Tmp.b0
    XOR   out_byte, out_byte, _Tmp.b1
    
return?: 
    ; All done
    .unasg _Tmp
    .endm
;-------------------------------------------------------------------------------

;*******************************************************************************
M_FDB_LINEAR_SEARCH   .macro      MacAddr01, MacAddr23, MacAddr45, BucketIndex, BucketLen, OutVal
;-------------------------------------------------------------------------------
; Description:  Linearly search bucket in FDB MAC Table for requested MAC address
; Input Parameters: MacAddr01 - register containing first 2 bytes of MAC address
;                   MacAddr23 - register word containing next 2 bytes of MAC address
;                   MacAddr45 - register word containing last 2 bytes of MAC address
;                   BucketLen - Number of entries to search in the MAC Table
; Use of registers: RCV_TEMP_REG_1/2
; Peak PRU cycles : (11 * BucketLen) + 1
; output registers : OutVal - Search success (1) or fail (0)
;-------------------------------------------------------------------------------
    .asg  RCV_TEMP_REG_1,    _FdbMacAddr  ; Retrieved FDB MAC Address (base)
    .asg  RCV_TEMP_REG_1.w0, _FdbMac01    ; Retreived FDB MAC Address (first 2 bytes)
    .asg  RCV_TEMP_REG_1.w2, _FdbMac23    ; Retreived FDB MAC Address (next 2 bytes)
    .asg  RCV_TEMP_REG_2.w0, _FdbMac45    ; Retreived FDB MAC Address (last 2 bytes)
    .asg  RCV_TEMP_REG_2.w2, _Tmp
    .asg  RCV_TEMP_REG_3,    _TablePtr    ; Table address pointer
;-------------------------------------------------------------------------------

    ; Prep read from MAC table
    LDI32 _TablePtr, ICSS_EMAC_FW_FDB__MAC_TBL_ADDR ; Prepare table pointer for read at table base.
    
    ; Calculate MAC Table index offset
    LSL   _Tmp, BucketIndex, MAC_TBL_SHIFT_VAL_1   ; MAC Table entries are 10 bytes long
    ADD   _TablePtr, _TablePtr, _Tmp          ; .. so multiply the index by 8 and add to ptr
    LSL   _Tmp, BucketIndex, MAC_TBL_SHIFT_VAL_2   ; .. then multiply the index by 2
    ADD   _TablePtr, _TablePtr, _Tmp          ; .. and add to ptr idx * 10
    
    ; Linearly search bucket
    LOOP  linear_search_loop_end?, BucketLen
    LBBO  &_FdbMacAddr, _TablePtr, FDB_MAC_INFO__MAC_ADDR__OFFSET, FDB_MAC_INFO__MAC_ADDR__SIZE

    ; If the MAC adresses don't match, continue
    QBNE  linear_search_continue?, MacAddr01, _FdbMac01
    QBNE  linear_search_continue?, MacAddr23, _FdbMac23
    QBNE  linear_search_continue?, MacAddr45, _FdbMac45

    ; MAC address found in FDB, return Success
    LDI   OutVal, 1
    QBA   return?

linear_search_continue?:
    ; Increase table pointer and loop
    ADD   _TablePtr, _TablePtr, ICSS_EMAC_FW_FDB__MAC_TBL_ENTRY_SIZE
linear_search_loop_end?:

    ; MAC address NOT found in FDB, return Fail
    LDI   OutVal, 0

return?:
    ; All done
    .unasg _FdbMacAddr
    .unasg _FdbMac01
    .unasg _FdbMac23 
    .unasg _FdbMac45
    .unasg _Tmp
    .unasg _TablePtr
    .endm
;-------------------------------------------------------------------------------

;*******************************************************************************
M_FDB_BINARY_SEARCH   .macro      MacAddr01, MacAddr23, MacAddr45, BucketIndex, BucketLen, OutVal
;-------------------------------------------------------------------------------
; Description:  Binary search bucket in FDB MAC Table for requested MAC address
; Input Parameters: MacAddr01 - register containing first 2 bytes of MAC address
;                   MacAddr23 - register word containing next 2 bytes of MAC address
;                   MacAddr45 - register word containing last 2 bytes of MAC address
;                   BucketLen - Number of entries to search in the MAC Table
; Use of registers: 
; Peak PRU cycles : 
; output registers : OutVal - Search success (1) or fail (0)
;-------------------------------------------------------------------------------
    .asg  RCV_TEMP_REG_1,    _FdbMacAddr  ; Retrieved FDB MAC Address (base)
    .asg  RCV_TEMP_REG_1,    _FdbMac0123  ; Retreived FDB MAC Address (first 4 bytes)
    .asg  RCV_TEMP_REG_2.w0, _FdbMac45    ; Retreived FDB MAC Address (last 2 bytes)
    .asg  RCV_TEMP_REG_2.b0, _FdbMac4     ; Retreived FDB MAC Address
    .asg  RCV_TEMP_REG_2.b1, _FdbMac5     ; Retreived FDB MAC Address
    .asg  RCV_TEMP_REG_2.b2, _Low
    .asg  RCV_TEMP_REG_2.b3, _High
    .asg  RCV_TEMP_REG_3,    _TablePtr    ; Table address pointer
    .asg  RCV_TEMP_REG_3,    _TmpMacAddr  ; Table address pointer
    .asg  BucketIndex,       _Tmp
    .asg  BucketLen,         _Mid
;-------------------------------------------------------------------------------

    ; Prepare index boundaries
    MOV   _Low, BucketIndex
    ADD   _High, _Low, BucketLen
    ; BucketIndex and BucketLen should NOT be used anymore
               
    ; Loop a maximum of log2(BucketLen) times
    LMBD  _Tmp, BucketLen, 1
    QBEQ  binary_loop_end?, _Tmp, 32
    ADD   _Tmp, _Tmp, 1
binary_loop_start?:    
    LOOP  binary_loop_end?, _Tmp
    
    ; Start halfway through bucket
    ADD   _Mid, _Low, _High
    LSR   _Mid, _Mid, 1
     
    ; Prep read from MAC table
    LDI32 _TablePtr, ICSS_EMAC_FW_FDB__MAC_TBL_ADDR ; Prepare table pointer for read at table base.
    
    ; Calculate MAC Table index offset
    LSL   _Tmp, _Mid, MAC_TBL_SHIFT_VAL_1   ; MAC Table entries are 10 bytes long
    ADD   _TablePtr, _TablePtr, _Tmp          ; .. so multiply the index by 8 and add to ptr
    LSL   _Tmp, _Mid, MAC_TBL_SHIFT_VAL_2   ; .. then multiply the index by 2
    ADD   _TablePtr, _TablePtr, _Tmp          ; .. and add to ptr idx * 10
    
    ; Read bucket entry
    LBBO  &_FdbMacAddr, _TablePtr, FDB_MAC_INFO__MAC_ADDR__OFFSET, FDB_MAC_INFO__MAC_ADDR__SIZE

    MOV   _TmpMacAddr.w0, MacAddr01
    MOV   _TmpMacAddr.w2, MacAddr23
    
    ; Compare MAC to FDB
    QBEQ  continue0?, _FdbMac0123.b0, _TmpMacAddr.b0
    QBGT  greater?, _FdbMac0123.b0, _TmpMacAddr.b0
    QBLT  lesser?, _FdbMac0123.b0, _TmpMacAddr.b0        
    
continue0?:
    QBEQ  continue1?, _FdbMac0123.b1, _TmpMacAddr.b1
    QBGT  greater?, _FdbMac0123.b1, _TmpMacAddr.b1
    QBLT  lesser?, _FdbMac0123.b1, _TmpMacAddr.b1

continue1?:
    QBEQ  continue2?, _FdbMac0123.b2, _TmpMacAddr.b2
    QBGT  greater?, _FdbMac0123.b2, _TmpMacAddr.b2
    QBLT  lesser?, _FdbMac0123.b2, _TmpMacAddr.b2

continue2?:
    QBEQ  continue3?, _FdbMac0123.b3, _TmpMacAddr.b3
    QBGT  greater?, _FdbMac0123.b3, _TmpMacAddr.b3
    QBLT  lesser?, _FdbMac0123.b3, _TmpMacAddr.b3

continue3?:
    MOV   _TmpMacAddr, MacAddr45    
    QBEQ  continue4?, _FdbMac4, _TmpMacAddr.b0
    QBGT  greater?, _FdbMac4, _TmpMacAddr.b0
    QBLT  lesser?, _FdbMac4, _TmpMacAddr.b0

continue4?:
    QBEQ  equal?, _FdbMac5, _TmpMacAddr.b1
    QBGT  greater?, _FdbMac5, _TmpMacAddr.b1
    QBLT  lesser?, _FdbMac5, _TmpMacAddr.b1    

equal?:
    LDI   OutVal, 1
    QBA   return?
    
greater?:
    ADD   _Low, _Mid, 1
    QBA   check_loop?
    
lesser?:
    SUB _High, _Mid, 1

check_loop?
    ; Break if pointers overlap (end of search)
    QBGT  break?, _High, _Low

binary_loop_end?:
break?:

    ; Address not found in table
    LDI   OutVal, 0

return?:
    ; Re-calculate Table Pointer for return purposes
    LDI32 _TablePtr, ICSS_EMAC_FW_FDB__MAC_TBL_ADDR ; Prepare table pointer for read at table base.
    
    ; Calculate MAC Table index offset
    LSL   _Tmp, _Mid, MAC_TBL_SHIFT_VAL_1   ; MAC Table entries are 10 bytes long
    ADD   _TablePtr, _TablePtr, _Tmp          ; .. so multiply the index by 8 and add to ptr
    LSL   _Tmp, _Mid, MAC_TBL_SHIFT_VAL_2   ; .. then multiply the index by 2
    ADD   _TablePtr, _TablePtr, _Tmp          ; .. and add to ptr idx * 10
    
    ; All done
    .unasg _FdbMacAddr
    .unasg _FdbMac0123 
    .unasg _FdbMac45
    .unasg _FdbMac4
    .unasg _FdbMac5
    .unasg _Low
    .unasg _High
    .unasg _TablePtr
    .unasg _TmpMacAddr
    .unasg _Tmp
    .unasg _Mid
    .endm
;-------------------------------------------------------------------------------

;*******************************************************************************
M_UNICAST_FDB_HASH_LOOKUP    .macro    MacAddr01, MacAddr23, MacAddr45, retVal
;-------------------------------------------------------------------------------
; Description:  Unicast FDB table lookup
; Input Parameters: MacAddr01 - register containing first 2 bytes of MAC address
;                   MacAddr23 - register word containing next 2 bytes of MAC address
;                   MacAddr45 - register word containing last 2 bytes of MAC address
; Use of registers: RCV_TEMP_REG_1/2/3/4
; Peak PRU cycles : (11 * BucketLen) + 19
; output registers : retVal - Lookup success (1) or fail (0)
;-------------------------------------------------------------------------------
    .asg  RCV_TEMP_REG_1.w0, _TmpMacIdx   ; For calculating the MAC Table index
    .asg  RCV_TEMP_REG_2.b0, _HashValue   ; Calculated hash byte
    .asg  RCV_TEMP_REG_2.w0, _HashIndex   ; Shifted hash byte for indexing into Index Table
    .asg  RCV_TEMP_REG_3,    _TablePtr    ; Table address pointer
    .asg  RCV_TEMP_REG_4,    _BucketInfo  ; Bucket information
    .asg  RCV_TEMP_REG_4.w0, _BucketIndex ; Bucket starting point in MAC Table
    .asg  RCV_TEMP_REG_4.w2, _BucketLen  ; Number of entries in bucket
;-------------------------------------------------------------------------------

    ; Calculate hash value of MAC address
    M_MAC_HASH     MacAddr01, MacAddr23, MacAddr45, _HashValue

    ; Read Bucket Info from Index Table
    LDI32 _TablePtr, ICSS_EMAC_FW_FDB__IDX_TBL_ADDR ; Prepare table pointer for read at table base.
    LSL   _HashIndex, _HashValue, IDX_TBL_SHIFT_VAL ; Index Table entries are 4 bytes long
    ADD   _TablePtr, _TablePtr, _HashIndex          ; .. so offset table pointer accordingly
    LBBO  &_BucketInfo, _TablePtr, 0, 4             ; .. and then read 4 bytes.    

    ; Search the MAC Table for requested MAC address
    M_FDB_BINARY_SEARCH MacAddr01, MacAddr23, MacAddr45, _BucketIndex, _BucketLen, retVal

.return?:
    ; All done
    .unasg _TmpMacIdx
    .unasg _HashValue
    .unasg _HashIndex
    .unasg _TablePtr
    .unasg _BucketInfo
    .unasg _BucketIndex
    .unasg _BucketLen
    .endm
;-------------------------------------------------------------------------------

    
;*******************************************************************************
M_RELEASE_PRU_LOCK    .macro    LocksPtr, PruLock
;-------------------------------------------------------------------------------
; Description:  
; Input Parameters: 
; Use of registers: 
; Peak PRU cycles : 
; output registers : 
;-------------------------------------------------------------------------------
    
    LDI32 LocksPtr, ICSS_EMAC_FW_FDB__ARBITRATION_ADDR    
    LBBO  &PruLock, LocksPtr, ICSS_EMAC_FW_FDB__PRU_ARBITRATION__LOCAL_OFFSET, ICSS_EMAC_FW_FDB__PRU_ARBITRATION_SIZE
    .if $defined("PRU0")	
    CLR   PruLock, PruLock, ICSS_EMAC_FW_FDB__ARBITRATION__PRU_0_LOCK
    .else
    CLR   PruLock, PruLock, ICSS_EMAC_FW_FDB__ARBITRATION__PRU_1_LOCK
    .endif    
    SBBO  &PruLock, LocksPtr, ICSS_EMAC_FW_FDB__PRU_ARBITRATION__LOCAL_OFFSET, ICSS_EMAC_FW_FDB__PRU_ARBITRATION_SIZE

    .endm
;-------------------------------------------------------------------------------

;*******************************************************************************
M_TAKE_PRU_LOCK    .macro    LocksPtr, PruLock
;-------------------------------------------------------------------------------
; Description:  
; Input Parameters: 
; Use of registers: 
; Peak PRU cycles : 
; output registers : 
;-------------------------------------------------------------------------------
    
    LBBO  &PruLock, LocksPtr, ICSS_EMAC_FW_FDB__PRU_ARBITRATION__LOCAL_OFFSET, ICSS_EMAC_FW_FDB__PRU_ARBITRATION_SIZE
    
    .if $defined("PRU0")	
    SET   PruLock, PruLock, ICSS_EMAC_FW_FDB__ARBITRATION__PRU_0_LOCK
    .else
    SET   PruLock, PruLock, ICSS_EMAC_FW_FDB__ARBITRATION__PRU_1_LOCK
    .endif
    
    SBBO  &PruLock, LocksPtr, ICSS_EMAC_FW_FDB__PRU_ARBITRATION__LOCAL_OFFSET, ICSS_EMAC_FW_FDB__PRU_ARBITRATION_SIZE
          
    .endm    
;-------------------------------------------------------------------------------

;*******************************************************************************
M_CHECK_HOST_LOCK    .macro    LocksPtr, HostLock, RetVal, Timeout
;-------------------------------------------------------------------------------
; Description:  
; Input Parameters: 
; Use of registers: 
; Peak PRU cycles : 
; output registers : 
;-------------------------------------------------------------------------------

    ; Assume success
    LDI RetVal, 0x1

    ; Read the Host Lock status
    LBBO  &HostLock, LocksPtr, ICSS_EMAC_FW_FDB__HOST_ARBITRATION__LOCAL_OFFSET, ICSS_EMAC_FW_FDB__HOST_ARBITRATION_SIZE

    ; If the host lock isn't taken, continue with the FDB lookup
    QBBC  fin?, HostLock, ICSS_EMAC_FW_FDB__ARBITRATION__HOST_LOCK
    
    ; Loop for Timeout times
    LOOP  timeout?, Timeout
    
    ; Keep checking the host lock status until either timeout or lock is open
    LBBO  &HostLock, LocksPtr, ICSS_EMAC_FW_FDB__HOST_ARBITRATION__LOCAL_OFFSET, ICSS_EMAC_FW_FDB__HOST_ARBITRATION_SIZE
    QBBC  fin?, HostLock, ICSS_EMAC_FW_FDB__ARBITRATION__HOST_LOCK
    
timeout?:
    ; We have timed out, return FAIL (0)
    LDI RetVal, 0x0

fin?:    
          
    .endm    
;-------------------------------------------------------------------------------
    
    
;*******************************************************************************
M_SPIN_LOCK    .macro    RetVal, Timeout
;-------------------------------------------------------------------------------
; Description:  
; Input Parameters: 
; Use of registers: 
; Peak PRU cycles : 
; output registers : 
;-------------------------------------------------------------------------------
    .asg  RCV_TEMP_REG_3, _LocksPtr
    .asg  RCV_TEMP_REG_4.w0, _Locks
    .asg  RCV_TEMP_REG_4.b0, _HostLock
    .asg  RCV_TEMP_REG_4.b1, _PruLock
    
;-------------------------------------------------------------------------------
    ; Prepare FDB Arbitration base pointer
    LDI32 _LocksPtr, ICSS_EMAC_FW_FDB__ARBITRATION_ADDR

    ; Take the PRU Lock
    M_TAKE_PRU_LOCK     _LocksPtr, _PruLock

    ; Check the Host Pointer with 
    M_CHECK_HOST_LOCK   _LocksPtr, _HostLock, RetVal, Timeout
    QBNE  fin?, RetVal, 0

    ; Timed out, release the PRU lock
    M_RELEASE_PRU_LOCK  _LocksPtr, _PruLock

fin?:     
    .unasg _LocksPtr
    .unasg _Locks
    .unasg _HostLock
    .unasg _PruLock
    .endm
;-------------------------------------------------------------------------------

;*******************************************************************************
M_SPIN_UNLOCK    .macro
;-------------------------------------------------------------------------------
; Description:  
; Input Parameters: 
; Use of registers: 
; Peak PRU cycles : 
; output registers : 
;-------------------------------------------------------------------------------
    .asg  RCV_TEMP_REG_3, _LocksPtr
    .asg  RCV_TEMP_REG_4.w0, _Locks
    .asg  RCV_TEMP_REG_4.b1, _PruLock
    
;-------------------------------------------------------------------------------
    ; Prepare FDB Arbitration base pointer
    LDI32 _LocksPtr, ICSS_EMAC_FW_FDB__ARBITRATION_ADDR

    ; Release the PRU Lock
    M_RELEASE_PRU_LOCK  _LocksPtr, _PruLock
    
    .unasg _LocksPtr
    .unasg _Locks
    .unasg _PruLock
    .endm
;-------------------------------------------------------------------------------
        
    
    .endif      ;__icss_stp_switch_macros_hp
