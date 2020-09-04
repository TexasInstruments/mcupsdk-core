
;
; Copyright (C) 2021 Texas Instruments Incorporated
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
; "AS IS" AND ANY EXPgResS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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

	.if	!$isdefed("__firmware_version_h")
__firmware_version_h	.set	1

;  ICSS_FIRMWARE_RELEASE_1:
;  bit 31..16 reserved
;  bit15..8	device number
FIRMWARE_DEVICE_AM335x	.set		0	;  AM335x
FIRMWARE_DEVICE_AM437x	.set		1	;  AM437x
FIRMWARE_DEVICE_AM64x	.set		2	;  AM64x
;  bit7..0	protocol type
FIRMWARE_PROTOCOL_TYPE_PROFIBUS_SLAVE	.set	0x00
FIRMWARE_PROTOCOL_TYPE_ETHERCAT_SLAVE	.set	0x01
FIRMWARE_PROTOCOL_TYPE_PROFINET_DEVICE	.set	0x02
FIRMWARE_PROTOCOL_TYPE_SERCOS_SLAVE	.set		0x03
FIRMWARE_PROTOCOL_TYPE_OPENMAC_SLAVE	.set	0x04
FIRMWARE_PROTOCOL_TYPE_ETHERNET	.set 		0x05
FIRMWARE_PROTOCOL_TYPE_ENETIP_SLAVE	.set		0x06
FIRMWARE_PROTOCOL_TYPE_ENDAT_MASTER	.set		0x07

;  ICSS_FIRMWARE_RELEASE_2:
;  bit31		release or internal version
FIRMWARE_VERSION_RELEASE	.set	0
FIRMWARE_VERSION_INTERNAL	.set	1
;  bit30..24		version number
FIRMWARE_VERSION_REVISION	.set		0x00
;  bit23..16		major number
FIRMWARE_VERSION_MAJOR	.set			0x02
;  bit15..0		minor number
FIRMWARE_VERSION_MINOR	.set			0x0000

ICSS_FIRMWARE_RELEASE_1	.set	((FIRMWARE_DEVICE_AM64x << 8) | (FIRMWARE_PROTOCOL_TYPE_ENDAT_MASTER << 0))
ICSS_FIRMWARE_RELEASE_2	.set ((FIRMWARE_VERSION_RELEASE << 31) | (FIRMWARE_VERSION_REVISION << 24) | (FIRMWARE_VERSION_MAJOR << 16) | (FIRMWARE_VERSION_MINOR << 0))

	.endif
