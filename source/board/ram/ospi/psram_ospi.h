/*
 *  Copyright (C) 2025 Texas Instruments Incorporated
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

#ifndef PSRAM_OSPI_H_
#define PSRAM_OSPI_H_

#ifdef __cplusplus
extern "C" {
#endif
#include <board/ram.h>
#include <drivers/ospi.h>

#define OSPI_FLASH_ATTACK_VECTOR_SIZE       (128U)

static uint8_t gOspiFlashAttackVector[OSPI_FLASH_ATTACK_VECTOR_SIZE] =
{
 0xFEU,   // 0b11111110 @ 0x00000400 1024 bytes
 0xFF,   // 0b11111111 @ 0x00000401 1025 bytes
 // Even Aligned Victim bit: 0, vic time slice : 111110000, agr time slice : 000000011
 0x01,   // 0b00000001 @ 0x00000402 1026 bytes
 0x01,   // 0b00000001 @ 0x00000403 1027 bytes
 0x01,   // 0b00000001 @ 0x00000404 1028 bytes
 0x01,   // 0b00000001 @ 0x00000405 1029 bytes
 0x01,   // 0b00000001 @ 0x00000406 1030 bytes
 0x00,   // 0b00000000 @ 0x00000407 1031 bytes
 0x00,   // 0b00000000 @ 0x00000408 1032 bytes
 0xFE,   // 0b11111110 @ 0x00000409 1033 bytes
 0xFE,   // 0b11111110 @ 0x0000040A 1034 bytes
 // Odd  Aligned Victim bit: 0, vic time slice : 111110000, agr time slice : 000000011
 0x01,   // 0b00000001 @ 0x0000040B 1035 bytes
 0x01,   // 0b00000001 @ 0x0000040C 1036 bytes
 0x01,   // 0b00000001 @ 0x0000040D 1037 bytes
 0x01,   // 0b00000001 @ 0x0000040E 1038 bytes
 0x00,   // 0b00000000 @ 0x0000040F 1039 bytes
 0x00,   // 0b00000000 @ 0x00000410 1040 bytes
 0xFE,   // 0b11111110 @ 0x00000411 1041 bytes
 0xFE,   // 0b11111110 @ 0x00000412 1042 bytes
 0x01,   // 0b00000001 @ 0x00000413 1043 bytes
 // Even Aligned Victim bit: 0, vic time slice : 111110000, agr time slice : 111110011
 0xFF,   // 0b11111111 @ 0x00000414 1044 bytes
 0xFF,   // 0b11111111 @ 0x00000415 1045 bytes
 0xFF,   // 0b11111111 @ 0x00000416 1046 bytes
 0xFF,   // 0b11111111 @ 0x00000417 1047 bytes
 0xFF,   // 0b11111111 @ 0x00000418 1048 bytes
 0x00,   // 0b00000000 @ 0x00000419 1049 bytes
 0x00,   // 0b00000000 @ 0x0000041A 1050 bytes
 0xFE,   // 0b11111110 @ 0x0000041B 1051 bytes
 0xFE,   // 0b11111110 @ 0x0000041C 1052 bytes
 // Odd  Aligned Victim bit: 0, vic time slice : 111110000, agr time slice : 111110011
 0xFF,   // 0b11111111 @ 0x0000041D 1053 bytes
 0xFF,   // 0b11111111 @ 0x0000041E 1054 bytes
 0xFF,   // 0b11111111 @ 0x0000041F 1055 bytes
 0xFF,   // 0b11111111 @ 0x00000420 1056 bytes
 0x00,   // 0b00000000 @ 0x00000421 1057 bytes
 0x00,   // 0b00000000 @ 0x00000422 1058 bytes
 0xFE,   // 0b11111110 @ 0x00000423 1059 bytes
 0xFE,   // 0b11111110 @ 0x00000424 1060 bytes
 0xFF,   // 0b11111111 @ 0x00000425 1061 bytes
 // Even Aligned Victim bit: 0, vic time slice : 111110000, agr time slice : 000001011
 0x01,   // 0b00000001 @ 0x00000426 1062 bytes
 0x01,   // 0b00000001 @ 0x00000427 1063 bytes
 0x01,   // 0b00000001 @ 0x00000428 1064 bytes
 0x01,   // 0b00000001 @ 0x00000429 1065 bytes
 0x01,   // 0b00000001 @ 0x0000042A 1066 bytes
 0xFE,   // 0b11111110 @ 0x0000042B 1067 bytes
 0x00,   // 0b00000000 @ 0x0000042C 1068 bytes
 0xFE,   // 0b11111110 @ 0x0000042D 1069 bytes
 0xFE,   // 0b11111110 @ 0x0000042E 1070 bytes
 // Odd  Aligned Victim bit: 0, vic time slice : 111110000, agr time slice : 000001011
 0x01,   // 0b00000001 @ 0x0000042F 1071 bytes
 0x01,   // 0b00000001 @ 0x00000430 1072 bytes
 0x01,   // 0b00000001 @ 0x00000431 1073 bytes
 0x01,   // 0b00000001 @ 0x00000432 1074 bytes
 0xFE,   // 0b11111110 @ 0x00000433 1075 bytes
 0x00,   // 0b00000000 @ 0x00000434 1076 bytes
 0xFE,   // 0b11111110 @ 0x00000435 1077 bytes
 0xFE,   // 0b11111110 @ 0x00000436 1078 bytes
 0x01,   // 0b00000001 @ 0x00000437 1079 bytes
 // Even Aligned Victim bit: 0, vic time slice : 111110000, agr time slice : 111111011
 0xFF,   // 0b11111111 @ 0x00000438 1080 bytes
 0xFF,   // 0b11111111 @ 0x00000439 1081 bytes
 0xFF,   // 0b11111111 @ 0x0000043A 1082 bytes
 0xFF,   // 0b11111111 @ 0x0000043B 1083 bytes
 0xFF,   // 0b11111111 @ 0x0000043C 1084 bytes
 0xFE,   // 0b11111110 @ 0x0000043D 1085 bytes
 0x00,   // 0b00000000 @ 0x0000043E 1086 bytes
 0xFE,   // 0b11111110 @ 0x0000043F 1087 bytes
 0xFE,   // 0b11111110 @ 0x00000440 1088 bytes
 // Odd  Aligned Victim bit: 0, vic time slice : 111110000, agr time slice : 111111011
 0xFF,   // 0b11111111 @ 0x00000441 1089 bytes
 0xFF,   // 0b11111111 @ 0x00000442 1090 bytes
 0xFF,   // 0b11111111 @ 0x00000443 1091 bytes
 0xFF,   // 0b11111111 @ 0x00000444 1092 bytes
 0xFE,   // 0b11111110 @ 0x00000445 1093 bytes
 0x00,   // 0b00000000 @ 0x00000446 1094 bytes
 0xFE,   // 0b11111110 @ 0x00000447 1095 bytes
 0xFE,   // 0b11111110 @ 0x00000448 1096 bytes
 0xFF,   // 0b11111111 @ 0x00000449 1097 bytes
 // Even Aligned Victim bit: 0, vic time slice : 111110000, agr time slice : 000000111
 0x01,   // 0b00000001 @ 0x0000044A 1098 bytes
 0x01,   // 0b00000001 @ 0x0000044B 1099 bytes
 0x01,   // 0b00000001 @ 0x0000044C 1100 bytes
 0x01,   // 0b00000001 @ 0x0000044D 1101 bytes
 0x01,   // 0b00000001 @ 0x0000044E 1102 bytes
 0x00,   // 0b00000000 @ 0x0000044F 1103 bytes
 0xFE,   // 0b11111110 @ 0x00000450 1104 bytes
 0xFE,   // 0b11111110 @ 0x00000451 1105 bytes
 0xFE,   // 0b11111110 @ 0x00000452 1106 bytes
 // Odd  Aligned Victim bit: 0, vic time slice : 111110000, agr time slice : 000000111
 0x01,   // 0b00000001 @ 0x00000453 1107 bytes
 0x01,   // 0b00000001 @ 0x00000454 1108 bytes
 0x01,   // 0b00000001 @ 0x00000455 1109 bytes
 0x01,   // 0b00000001 @ 0x00000456 1110 bytes
 0x00,   // 0b00000000 @ 0x00000457 1111 bytes
 0xFE,   // 0b11111110 @ 0x00000458 1112 bytes
 0xFE,   // 0b11111110 @ 0x00000459 1113 bytes
 0xFE,   // 0b11111110 @ 0x0000045A 1114 bytes
 0x01,   // 0b00000001 @ 0x0000045B 1115 bytes
 // Even Aligned Victim bit: 0, vic time slice : 111110000, agr time slice : 111110111
 0xFF,   // 0b11111111 @ 0x0000045C 1116 bytes
 0xFF,   // 0b11111111 @ 0x0000045D 1117 bytes
 0xFF,   // 0b11111111 @ 0x0000045E 1118 bytes
 0xFF,   // 0b11111111 @ 0x0000045F 1119 bytes
 0xFF,   // 0b11111111 @ 0x00000460 1120 bytes
 0x00,   // 0b00000000 @ 0x00000461 1121 bytes
 0xFE,   // 0b11111110 @ 0x00000462 1122 bytes
 0xFE,   // 0b11111110 @ 0x00000463 1123 bytes
 0xFE,   // 0b11111110 @ 0x00000464 1124 bytes
 // Odd  Aligned Victim bit: 0, vic time slice : 111110000, agr time slice : 111110111
 0xFF,   // 0b11111111 @ 0x00000465 1125 bytes
 0xFF,   // 0b11111111 @ 0x00000466 1126 bytes
 0xFF,   // 0b11111111 @ 0x00000467 1127 bytes
 0xFF,   // 0b11111111 @ 0x00000468 1128 bytes
 0x00,   // 0b00000000 @ 0x00000469 1129 bytes
 0xFE,   // 0b11111110 @ 0x0000046A 1130 bytes
 0xFE,   // 0b11111110 @ 0x0000046B 1131 bytes
 0xFE,   // 0b11111110 @ 0x0000046C 1132 bytes
 0xFF,   // 0b11111111 @ 0x0000046D 1133 bytes
 // Even Aligned Victim bit: 0, vic time slice : 111110000, agr time slice : 000001111
 0x01,   // 0b00000001 @ 0x0000046E 1134 bytes
 0x01,   // 0b00000001 @ 0x0000046F 1135 bytes
 0x01,   // 0b00000001 @ 0x00000470 1136 bytes
 0x01,   // 0b00000001 @ 0x00000471 1137 bytes
 0x01,   // 0b00000001 @ 0x00000472 1138 bytes
 0xFE,   // 0b11111110 @ 0x00000473 1139 bytes
 0xFE,   // 0b11111110 @ 0x00000474 1140 bytes
 0xFE,   // 0b11111110 @ 0x00000475 1141 bytes
 0xFE,   // 0b11111110 @ 0x00000476 1142 bytes
 // Odd  Aligned Victim bit: 0, vic time slice : 111110000, agr time slice : 000001111
 0x01,   // 0b00000001 @ 0x00000477 1143 bytes
 0x01,   // 0b00000001 @ 0x00000478 1144 bytes
 0x01,   // 0b00000001 @ 0x00000479 1145 bytes
 0x01,   // 0b00000001 @ 0x0000047A 1146 bytes
 0xFE,   // 0b11111110 @ 0x0000047B 1147 bytes
 0xFE,   // 0b11111110 @ 0x0000047C 1148 bytes
 0xFE,   // 0b11111110 @ 0x0000047D 1149 bytes
 0xFE,   // 0b11111110 @ 0x0000047E 1150 bytes
 0x01    // 0b00000001 @ 0x0000047F 1151 bytes
};

typedef struct {
    OSPI_Handle ospiHandle;
    uint8_t dacEnable;
    uint8_t phyEnable;
    uint8_t numAddrBytes;
    uint8_t badBlockCheck;
} Ram_OspiPsramObject;

/* PSRAM specific externs */
extern Ram_Fxns gPsramOspiFxns;

#ifdef __cplusplus
}
#endif

#endif /* PSRAM_XSPI_H_ */