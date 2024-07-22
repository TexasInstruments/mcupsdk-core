/********************************************************************
 * Copyright (C) 2023 Texas Instruments Incorporated.
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
 *
 *  Name        : cslr_controlss_inputxbar.h
*/
#ifndef CSLR_CONTROLSS_INPUTXBAR_H_
#define CSLR_CONTROLSS_INPUTXBAR_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include <drivers/hw_include/cslr.h>
#include <stdint.h>

/**************************************************************************
* Hardware Region  :
**************************************************************************/


/**************************************************************************
    XBAR INPUT Macros
**************************************************************************/

/******************* G0 *********************/
#define INPUT_XBAR_GPIO0        (0)
#define INPUT_XBAR_GPIO1        (1)
#define INPUT_XBAR_GPIO2        (2)
#define INPUT_XBAR_GPIO3        (3)
#define INPUT_XBAR_GPIO4        (4)
#define INPUT_XBAR_GPIO5        (5)
#define INPUT_XBAR_GPIO6        (6)
#define INPUT_XBAR_GPIO7        (7)
#define INPUT_XBAR_GPIO8        (8)
#define INPUT_XBAR_GPIO9        (9)
#define INPUT_XBAR_GPIO10      (10)
#define INPUT_XBAR_GPIO11      (11)
#define INPUT_XBAR_GPIO12      (12)
#define INPUT_XBAR_GPIO13      (13)
#define INPUT_XBAR_GPIO14      (14)
#define INPUT_XBAR_GPIO15      (15)
#define INPUT_XBAR_GPIO16      (16)
#define INPUT_XBAR_GPIO17      (17)
#define INPUT_XBAR_GPIO18      (18)
#define INPUT_XBAR_GPIO19      (19)
#define INPUT_XBAR_GPIO20      (20)
#define INPUT_XBAR_GPIO21      (21)
#define INPUT_XBAR_GPIO22      (22)
#define INPUT_XBAR_GPIO23      (23)
#define INPUT_XBAR_GPIO24      (24)
#define INPUT_XBAR_GPIO25      (25)
#define INPUT_XBAR_GPIO26      (26)
#define INPUT_XBAR_GPIO27      (27)
#define INPUT_XBAR_GPIO28      (28)
#define INPUT_XBAR_GPIO29      (29)
#define INPUT_XBAR_GPIO30      (30)
#define INPUT_XBAR_GPIO31      (31)
#define INPUT_XBAR_GPIO32      (32)
#define INPUT_XBAR_GPIO33      (33)
#define INPUT_XBAR_GPIO34      (34)
#define INPUT_XBAR_GPIO35      (35)
#define INPUT_XBAR_GPIO36      (36)
#define INPUT_XBAR_GPIO37      (37)
#define INPUT_XBAR_GPIO38      (38)
#define INPUT_XBAR_GPIO39      (39)
#define INPUT_XBAR_GPIO40      (40)
#define INPUT_XBAR_GPIO41      (41)
#define INPUT_XBAR_GPIO42      (42)
#define INPUT_XBAR_GPIO43      (43)
#define INPUT_XBAR_GPIO44      (44)
#define INPUT_XBAR_GPIO45      (45)
#define INPUT_XBAR_GPIO46      (46)
#define INPUT_XBAR_GPIO47      (47)
#define INPUT_XBAR_GPIO48      (48)
#define INPUT_XBAR_GPIO49      (49)
#define INPUT_XBAR_GPIO50      (50)
#define INPUT_XBAR_GPIO51      (51)
#define INPUT_XBAR_GPIO52      (52)
#define INPUT_XBAR_GPIO53      (53)
#define INPUT_XBAR_GPIO54      (54)
#define INPUT_XBAR_GPIO55      (55)
#define INPUT_XBAR_GPIO56      (56)
#define INPUT_XBAR_GPIO57      (57)
#define INPUT_XBAR_GPIO58      (58)
#define INPUT_XBAR_GPIO59      (59)
#define INPUT_XBAR_GPIO60      (60)
#define INPUT_XBAR_GPIO61      (61)
#define INPUT_XBAR_GPIO62      (62)
#define INPUT_XBAR_GPIO63      (63)
#define INPUT_XBAR_GPIO64      (64)
#define INPUT_XBAR_GPIO65      (65)
#define INPUT_XBAR_GPIO66      (66)
#define INPUT_XBAR_GPIO67      (67)
#define INPUT_XBAR_GPIO68      (68)
#define INPUT_XBAR_GPIO69      (69)
#define INPUT_XBAR_GPIO70      (70)
#define INPUT_XBAR_GPIO71      (71)
#define INPUT_XBAR_GPIO72      (72)
#define INPUT_XBAR_GPIO73      (73)
#define INPUT_XBAR_GPIO74      (74)
#define INPUT_XBAR_GPIO75      (75)
#define INPUT_XBAR_GPIO76      (76)
#define INPUT_XBAR_GPIO77      (77)
#define INPUT_XBAR_GPIO78      (78)
#define INPUT_XBAR_GPIO79      (79)
#define INPUT_XBAR_GPIO80      (80)
#define INPUT_XBAR_GPIO81      (81)
#define INPUT_XBAR_GPIO82      (82)
#define INPUT_XBAR_GPIO83      (83)
#define INPUT_XBAR_GPIO84      (84)
#define INPUT_XBAR_GPIO85      (85)
#define INPUT_XBAR_GPIO86      (86)
#define INPUT_XBAR_GPIO87      (87)
#define INPUT_XBAR_GPIO88      (88)
#define INPUT_XBAR_GPIO89      (89)
#define INPUT_XBAR_GPIO90      (90)
#define INPUT_XBAR_GPIO91      (91)
#define INPUT_XBAR_GPIO92      (92)
#define INPUT_XBAR_GPIO93      (93)
#define INPUT_XBAR_GPIO94      (94)
#define INPUT_XBAR_GPIO95      (95)
#define INPUT_XBAR_GPIO96      (96)
#define INPUT_XBAR_GPIO97      (97)
#define INPUT_XBAR_GPIO98      (98)
#define INPUT_XBAR_GPIO99      (99)
#define INPUT_XBAR_GPIO100    (100)
#define INPUT_XBAR_GPIO101    (101)
#define INPUT_XBAR_GPIO102    (102)
#define INPUT_XBAR_GPIO103    (103)
#define INPUT_XBAR_GPIO104    (104)
#define INPUT_XBAR_GPIO105    (105)
#define INPUT_XBAR_GPIO106    (106)
#define INPUT_XBAR_GPIO107    (107)
#define INPUT_XBAR_GPIO108    (108)
#define INPUT_XBAR_GPIO109    (109)
#define INPUT_XBAR_GPIO110    (110)
#define INPUT_XBAR_GPIO111    (111)
#define INPUT_XBAR_GPIO112    (112)
#define INPUT_XBAR_GPIO113    (113)
#define INPUT_XBAR_GPIO114    (114)
#define INPUT_XBAR_GPIO115    (115)
#define INPUT_XBAR_GPIO116    (116)
#define INPUT_XBAR_GPIO117    (117)
#define INPUT_XBAR_GPIO118    (118)
#define INPUT_XBAR_GPIO119    (119)
#define INPUT_XBAR_GPIO120    (120)
#define INPUT_XBAR_GPIO121    (121)
#define INPUT_XBAR_GPIO122    (122)
#define INPUT_XBAR_GPIO123    (123)
#define INPUT_XBAR_GPIO124    (124)
#define INPUT_XBAR_GPIO125    (125)
#define INPUT_XBAR_GPIO126    (126)
#define INPUT_XBAR_GPIO127    (127)
#define INPUT_XBAR_GPIO128    (128)
#define INPUT_XBAR_GPIO129    (129)
#define INPUT_XBAR_GPIO130    (130)
#define INPUT_XBAR_GPIO131    (131)
#define INPUT_XBAR_GPIO132    (132)
#define INPUT_XBAR_GPIO133    (133)
#define INPUT_XBAR_GPIO134    (134)
#define INPUT_XBAR_GPIO135    (135)
#define INPUT_XBAR_GPIO136    (136)
#define INPUT_XBAR_GPIO137    (137)
#define INPUT_XBAR_GPIO138    (138)
#define INPUT_XBAR_GPIO139    (139)
#define INPUT_XBAR_GPIO140    (140)
#define INPUT_XBAR_GPIO141    (141)
#define INPUT_XBAR_GPIO142    (142)
#define INPUT_XBAR_GPIO143    (143)

/******************* G1 *********************/
#define INPUT_XBAR_ICSSGPO0_PORT0    (0)
#define INPUT_XBAR_ICSSGPO1_PORT0    (1)
#define INPUT_XBAR_ICSSGPO2_PORT0    (2)
#define INPUT_XBAR_ICSSGPO3_PORT0    (3)
#define INPUT_XBAR_ICSSGPO4_PORT0    (4)
#define INPUT_XBAR_ICSSGPO5_PORT0    (5)
#define INPUT_XBAR_ICSSGPO6_PORT0    (6)
#define INPUT_XBAR_ICSSGPO7_PORT0    (7)
#define INPUT_XBAR_ICSSGPO8_PORT0    (8)
#define INPUT_XBAR_ICSSGPO9_PORT0    (9)
#define INPUT_XBAR_ICSSGPO10_PORT0    (10)
#define INPUT_XBAR_ICSSGPO11_PORT0    (11)
#define INPUT_XBAR_ICSSGPO12_PORT0    (12)
#define INPUT_XBAR_ICSSGPO13_PORT0    (13)
#define INPUT_XBAR_ICSSGPO14_PORT0    (14)
#define INPUT_XBAR_ICSSGPO15_PORT0    (15)
#define INPUT_XBAR_ICSSGPO0_PORT1    (16)
#define INPUT_XBAR_ICSSGPO1_PORT1    (17)
#define INPUT_XBAR_ICSSGPO2_PORT1    (18)
#define INPUT_XBAR_ICSSGPO3_PORT1    (19)
#define INPUT_XBAR_ICSSGPO4_PORT1    (20)
#define INPUT_XBAR_ICSSGPO5_PORT1    (21)
#define INPUT_XBAR_ICSSGPO6_PORT1    (22)
#define INPUT_XBAR_ICSSGPO7_PORT1    (23)
#define INPUT_XBAR_ICSSGPO8_PORT1    (24)
#define INPUT_XBAR_ICSSGPO9_PORT1    (25)
#define INPUT_XBAR_ICSSGPO10_PORT1    (26)
#define INPUT_XBAR_ICSSGPO11_PORT1    (27)
#define INPUT_XBAR_ICSSGPO12_PORT1    (28)
#define INPUT_XBAR_ICSSGPO13_PORT1    (29)
#define INPUT_XBAR_ICSSGPO14_PORT1    (30)
#define INPUT_XBAR_ICSSGPO15_PORT1    (31)

/******************* G2 *********************/
#define INPUT_XBAR_OUTPUTXBAR_OUT0     (0)
#define INPUT_XBAR_OUTPUTXBAR_OUT1     (1)
#define INPUT_XBAR_OUTPUTXBAR_OUT2     (2)
#define INPUT_XBAR_OUTPUTXBAR_OUT3     (3)
#define INPUT_XBAR_OUTPUTXBAR_OUT4     (4)
#define INPUT_XBAR_OUTPUTXBAR_OUT5     (5)
#define INPUT_XBAR_OUTPUTXBAR_OUT6     (6)
#define INPUT_XBAR_OUTPUTXBAR_OUT7     (7)
#define INPUT_XBAR_OUTPUTXBAR_OUT8     (8)
#define INPUT_XBAR_OUTPUTXBAR_OUT9     (9)
#define INPUT_XBAR_OUTPUTXBAR_OUT10   (10)
#define INPUT_XBAR_OUTPUTXBAR_OUT11   (11)
#define INPUT_XBAR_OUTPUTXBAR_OUT12   (12)
#define INPUT_XBAR_OUTPUTXBAR_OUT13   (13)
#define INPUT_XBAR_OUTPUTXBAR_OUT14   (14)
#define INPUT_XBAR_OUTPUTXBAR_OUT15   (15)

/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t PID;
    volatile uint8_t  Resv_256[252];
    volatile uint32_t INPUTXBAR0_GSEL;
    volatile uint32_t INPUTXBAR0_G0;
    volatile uint32_t INPUTXBAR0_G1;
    volatile uint32_t INPUTXBAR0_G2;
    volatile uint8_t  Resv_320[48];
    volatile uint32_t INPUTXBAR1_GSEL;
    volatile uint32_t INPUTXBAR1_G0;
    volatile uint32_t INPUTXBAR1_G1;
    volatile uint32_t INPUTXBAR1_G2;
    volatile uint8_t  Resv_384[48];
    volatile uint32_t INPUTXBAR2_GSEL;
    volatile uint32_t INPUTXBAR2_G0;
    volatile uint32_t INPUTXBAR2_G1;
    volatile uint32_t INPUTXBAR2_G2;
    volatile uint8_t  Resv_448[48];
    volatile uint32_t INPUTXBAR3_GSEL;
    volatile uint32_t INPUTXBAR3_G0;
    volatile uint32_t INPUTXBAR3_G1;
    volatile uint32_t INPUTXBAR3_G2;
    volatile uint8_t  Resv_512[48];
    volatile uint32_t INPUTXBAR4_GSEL;
    volatile uint32_t INPUTXBAR4_G0;
    volatile uint32_t INPUTXBAR4_G1;
    volatile uint32_t INPUTXBAR4_G2;
    volatile uint8_t  Resv_576[48];
    volatile uint32_t INPUTXBAR5_GSEL;
    volatile uint32_t INPUTXBAR5_G0;
    volatile uint32_t INPUTXBAR5_G1;
    volatile uint32_t INPUTXBAR5_G2;
    volatile uint8_t  Resv_640[48];
    volatile uint32_t INPUTXBAR6_GSEL;
    volatile uint32_t INPUTXBAR6_G0;
    volatile uint32_t INPUTXBAR6_G1;
    volatile uint32_t INPUTXBAR6_G2;
    volatile uint8_t  Resv_704[48];
    volatile uint32_t INPUTXBAR7_GSEL;
    volatile uint32_t INPUTXBAR7_G0;
    volatile uint32_t INPUTXBAR7_G1;
    volatile uint32_t INPUTXBAR7_G2;
    volatile uint8_t  Resv_768[48];
    volatile uint32_t INPUTXBAR8_GSEL;
    volatile uint32_t INPUTXBAR8_G0;
    volatile uint32_t INPUTXBAR8_G1;
    volatile uint32_t INPUTXBAR8_G2;
    volatile uint8_t  Resv_832[48];
    volatile uint32_t INPUTXBAR9_GSEL;
    volatile uint32_t INPUTXBAR9_G0;
    volatile uint32_t INPUTXBAR9_G1;
    volatile uint32_t INPUTXBAR9_G2;
    volatile uint8_t  Resv_896[48];
    volatile uint32_t INPUTXBAR10_GSEL;
    volatile uint32_t INPUTXBAR10_G0;
    volatile uint32_t INPUTXBAR10_G1;
    volatile uint32_t INPUTXBAR10_G2;
    volatile uint8_t  Resv_960[48];
    volatile uint32_t INPUTXBAR11_GSEL;
    volatile uint32_t INPUTXBAR11_G0;
    volatile uint32_t INPUTXBAR11_G1;
    volatile uint32_t INPUTXBAR11_G2;
    volatile uint8_t  Resv_1024[48];
    volatile uint32_t INPUTXBAR12_GSEL;
    volatile uint32_t INPUTXBAR12_G0;
    volatile uint32_t INPUTXBAR12_G1;
    volatile uint32_t INPUTXBAR12_G2;
    volatile uint8_t  Resv_1088[48];
    volatile uint32_t INPUTXBAR13_GSEL;
    volatile uint32_t INPUTXBAR13_G0;
    volatile uint32_t INPUTXBAR13_G1;
    volatile uint32_t INPUTXBAR13_G2;
    volatile uint8_t  Resv_1152[48];
    volatile uint32_t INPUTXBAR14_GSEL;
    volatile uint32_t INPUTXBAR14_G0;
    volatile uint32_t INPUTXBAR14_G1;
    volatile uint32_t INPUTXBAR14_G2;
    volatile uint8_t  Resv_1216[48];
    volatile uint32_t INPUTXBAR15_GSEL;
    volatile uint32_t INPUTXBAR15_G0;
    volatile uint32_t INPUTXBAR15_G1;
    volatile uint32_t INPUTXBAR15_G2;
    volatile uint8_t  Resv_1280[48];
    volatile uint32_t INPUTXBAR16_GSEL;
    volatile uint32_t INPUTXBAR16_G0;
    volatile uint32_t INPUTXBAR16_G1;
    volatile uint32_t INPUTXBAR16_G2;
    volatile uint8_t  Resv_1344[48];
    volatile uint32_t INPUTXBAR17_GSEL;
    volatile uint32_t INPUTXBAR17_G0;
    volatile uint32_t INPUTXBAR17_G1;
    volatile uint32_t INPUTXBAR17_G2;
    volatile uint8_t  Resv_1408[48];
    volatile uint32_t INPUTXBAR18_GSEL;
    volatile uint32_t INPUTXBAR18_G0;
    volatile uint32_t INPUTXBAR18_G1;
    volatile uint32_t INPUTXBAR18_G2;
    volatile uint8_t  Resv_1472[48];
    volatile uint32_t INPUTXBAR19_GSEL;
    volatile uint32_t INPUTXBAR19_G0;
    volatile uint32_t INPUTXBAR19_G1;
    volatile uint32_t INPUTXBAR19_G2;
    volatile uint8_t  Resv_1536[48];
    volatile uint32_t INPUTXBAR20_GSEL;
    volatile uint32_t INPUTXBAR20_G0;
    volatile uint32_t INPUTXBAR20_G1;
    volatile uint32_t INPUTXBAR20_G2;
    volatile uint8_t  Resv_1600[48];
    volatile uint32_t INPUTXBAR21_GSEL;
    volatile uint32_t INPUTXBAR21_G0;
    volatile uint32_t INPUTXBAR21_G1;
    volatile uint32_t INPUTXBAR21_G2;
    volatile uint8_t  Resv_1664[48];
    volatile uint32_t INPUTXBAR22_GSEL;
    volatile uint32_t INPUTXBAR22_G0;
    volatile uint32_t INPUTXBAR22_G1;
    volatile uint32_t INPUTXBAR22_G2;
    volatile uint8_t  Resv_1728[48];
    volatile uint32_t INPUTXBAR23_GSEL;
    volatile uint32_t INPUTXBAR23_G0;
    volatile uint32_t INPUTXBAR23_G1;
    volatile uint32_t INPUTXBAR23_G2;
    volatile uint8_t  Resv_1792[48];
    volatile uint32_t INPUTXBAR24_GSEL;
    volatile uint32_t INPUTXBAR24_G0;
    volatile uint32_t INPUTXBAR24_G1;
    volatile uint32_t INPUTXBAR24_G2;
    volatile uint8_t  Resv_1856[48];
    volatile uint32_t INPUTXBAR25_GSEL;
    volatile uint32_t INPUTXBAR25_G0;
    volatile uint32_t INPUTXBAR25_G1;
    volatile uint32_t INPUTXBAR25_G2;
    volatile uint8_t  Resv_1920[48];
    volatile uint32_t INPUTXBAR26_GSEL;
    volatile uint32_t INPUTXBAR26_G0;
    volatile uint32_t INPUTXBAR26_G1;
    volatile uint32_t INPUTXBAR26_G2;
    volatile uint8_t  Resv_1984[48];
    volatile uint32_t INPUTXBAR27_GSEL;
    volatile uint32_t INPUTXBAR27_G0;
    volatile uint32_t INPUTXBAR27_G1;
    volatile uint32_t INPUTXBAR27_G2;
    volatile uint8_t  Resv_2048[48];
    volatile uint32_t INPUTXBAR28_GSEL;
    volatile uint32_t INPUTXBAR28_G0;
    volatile uint32_t INPUTXBAR28_G1;
    volatile uint32_t INPUTXBAR28_G2;
    volatile uint8_t  Resv_2112[48];
    volatile uint32_t INPUTXBAR29_GSEL;
    volatile uint32_t INPUTXBAR29_G0;
    volatile uint32_t INPUTXBAR29_G1;
    volatile uint32_t INPUTXBAR29_G2;
    volatile uint8_t  Resv_2176[48];
    volatile uint32_t INPUTXBAR30_GSEL;
    volatile uint32_t INPUTXBAR30_G0;
    volatile uint32_t INPUTXBAR30_G1;
    volatile uint32_t INPUTXBAR30_G2;
    volatile uint8_t  Resv_2240[48];
    volatile uint32_t INPUTXBAR31_GSEL;
    volatile uint32_t INPUTXBAR31_G0;
    volatile uint32_t INPUTXBAR31_G1;
    volatile uint32_t INPUTXBAR31_G2;
} CSL_controlss_inputxbarRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_CONTROLSS_INPUTXBAR_PID                                            (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR0_GSEL                                (0x00000100U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR0_G0                                  (0x00000104U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR0_G1                                  (0x00000108U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR0_G2                                  (0x0000010CU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR1_GSEL                                (0x00000140U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR1_G0                                  (0x00000144U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR1_G1                                  (0x00000148U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR1_G2                                  (0x0000014CU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR2_GSEL                                (0x00000180U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR2_G0                                  (0x00000184U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR2_G1                                  (0x00000188U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR2_G2                                  (0x0000018CU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR3_GSEL                                (0x000001C0U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR3_G0                                  (0x000001C4U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR3_G1                                  (0x000001C8U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR3_G2                                  (0x000001CCU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR4_GSEL                                (0x00000200U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR4_G0                                  (0x00000204U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR4_G1                                  (0x00000208U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR4_G2                                  (0x0000020CU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR5_GSEL                                (0x00000240U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR5_G0                                  (0x00000244U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR5_G1                                  (0x00000248U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR5_G2                                  (0x0000024CU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR6_GSEL                                (0x00000280U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR6_G0                                  (0x00000284U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR6_G1                                  (0x00000288U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR6_G2                                  (0x0000028CU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR7_GSEL                                (0x000002C0U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR7_G0                                  (0x000002C4U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR7_G1                                  (0x000002C8U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR7_G2                                  (0x000002CCU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR8_GSEL                                (0x00000300U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR8_G0                                  (0x00000304U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR8_G1                                  (0x00000308U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR8_G2                                  (0x0000030CU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR9_GSEL                                (0x00000340U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR9_G0                                  (0x00000344U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR9_G1                                  (0x00000348U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR9_G2                                  (0x0000034CU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR10_GSEL                               (0x00000380U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR10_G0                                 (0x00000384U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR10_G1                                 (0x00000388U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR10_G2                                 (0x0000038CU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR11_GSEL                               (0x000003C0U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR11_G0                                 (0x000003C4U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR11_G1                                 (0x000003C8U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR11_G2                                 (0x000003CCU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR12_GSEL                               (0x00000400U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR12_G0                                 (0x00000404U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR12_G1                                 (0x00000408U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR12_G2                                 (0x0000040CU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR13_GSEL                               (0x00000440U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR13_G0                                 (0x00000444U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR13_G1                                 (0x00000448U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR13_G2                                 (0x0000044CU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR14_GSEL                               (0x00000480U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR14_G0                                 (0x00000484U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR14_G1                                 (0x00000488U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR14_G2                                 (0x0000048CU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR15_GSEL                               (0x000004C0U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR15_G0                                 (0x000004C4U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR15_G1                                 (0x000004C8U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR15_G2                                 (0x000004CCU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR16_GSEL                               (0x00000500U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR16_G0                                 (0x00000504U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR16_G1                                 (0x00000508U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR16_G2                                 (0x0000050CU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR17_GSEL                               (0x00000540U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR17_G0                                 (0x00000544U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR17_G1                                 (0x00000548U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR17_G2                                 (0x0000054CU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR18_GSEL                               (0x00000580U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR18_G0                                 (0x00000584U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR18_G1                                 (0x00000588U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR18_G2                                 (0x0000058CU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR19_GSEL                               (0x000005C0U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR19_G0                                 (0x000005C4U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR19_G1                                 (0x000005C8U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR19_G2                                 (0x000005CCU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR20_GSEL                               (0x00000600U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR20_G0                                 (0x00000604U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR20_G1                                 (0x00000608U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR20_G2                                 (0x0000060CU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR21_GSEL                               (0x00000640U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR21_G0                                 (0x00000644U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR21_G1                                 (0x00000648U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR21_G2                                 (0x0000064CU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR22_GSEL                               (0x00000680U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR22_G0                                 (0x00000684U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR22_G1                                 (0x00000688U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR22_G2                                 (0x0000068CU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR23_GSEL                               (0x000006C0U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR23_G0                                 (0x000006C4U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR23_G1                                 (0x000006C8U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR23_G2                                 (0x000006CCU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR24_GSEL                               (0x00000700U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR24_G0                                 (0x00000704U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR24_G1                                 (0x00000708U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR24_G2                                 (0x0000070CU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR25_GSEL                               (0x00000740U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR25_G0                                 (0x00000744U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR25_G1                                 (0x00000748U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR25_G2                                 (0x0000074CU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR26_GSEL                               (0x00000780U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR26_G0                                 (0x00000784U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR26_G1                                 (0x00000788U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR26_G2                                 (0x0000078CU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR27_GSEL                               (0x000007C0U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR27_G0                                 (0x000007C4U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR27_G1                                 (0x000007C8U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR27_G2                                 (0x000007CCU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR28_GSEL                               (0x00000800U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR28_G0                                 (0x00000804U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR28_G1                                 (0x00000808U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR28_G2                                 (0x0000080CU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR29_GSEL                               (0x00000840U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR29_G0                                 (0x00000844U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR29_G1                                 (0x00000848U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR29_G2                                 (0x0000084CU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR30_GSEL                               (0x00000880U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR30_G0                                 (0x00000884U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR30_G1                                 (0x00000888U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR30_G2                                 (0x0000088CU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR31_GSEL                               (0x000008C0U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR31_G0                                 (0x000008C4U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR31_G1                                 (0x000008C8U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR31_G2                                 (0x000008CCU)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* PID */

#define CSL_CONTROLSS_INPUTXBAR_PID_PID_MINOR_MASK                             (0x0000003FU)
#define CSL_CONTROLSS_INPUTXBAR_PID_PID_MINOR_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_PID_PID_MINOR_RESETVAL                         (0x00000014U)
#define CSL_CONTROLSS_INPUTXBAR_PID_PID_MINOR_MAX                              (0x0000003FU)

#define CSL_CONTROLSS_INPUTXBAR_PID_PID_CUSTOM_MASK                            (0x000000C0U)
#define CSL_CONTROLSS_INPUTXBAR_PID_PID_CUSTOM_SHIFT                           (0x00000006U)
#define CSL_CONTROLSS_INPUTXBAR_PID_PID_CUSTOM_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_PID_PID_CUSTOM_MAX                             (0x00000003U)

#define CSL_CONTROLSS_INPUTXBAR_PID_PID_MAJOR_MASK                             (0x00000700U)
#define CSL_CONTROLSS_INPUTXBAR_PID_PID_MAJOR_SHIFT                            (0x00000008U)
#define CSL_CONTROLSS_INPUTXBAR_PID_PID_MAJOR_RESETVAL                         (0x00000002U)
#define CSL_CONTROLSS_INPUTXBAR_PID_PID_MAJOR_MAX                              (0x00000007U)

#define CSL_CONTROLSS_INPUTXBAR_PID_PID_MISC_MASK                              (0x0000F800U)
#define CSL_CONTROLSS_INPUTXBAR_PID_PID_MISC_SHIFT                             (0x0000000BU)
#define CSL_CONTROLSS_INPUTXBAR_PID_PID_MISC_RESETVAL                          (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_PID_PID_MISC_MAX                               (0x0000001FU)

#define CSL_CONTROLSS_INPUTXBAR_PID_PID_MSB16_MASK                             (0xFFFF0000U)
#define CSL_CONTROLSS_INPUTXBAR_PID_PID_MSB16_SHIFT                            (0x00000010U)
#define CSL_CONTROLSS_INPUTXBAR_PID_PID_MSB16_RESETVAL                         (0x00006180U)
#define CSL_CONTROLSS_INPUTXBAR_PID_PID_MSB16_MAX                              (0x0000FFFFU)

#define CSL_CONTROLSS_INPUTXBAR_PID_RESETVAL                                   (0x61800214U)

/* INPUTXBAR0_GSEL */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR0_GSEL_GSEL_MASK                      (0x00000003U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR0_GSEL_GSEL_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR0_GSEL_GSEL_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR0_GSEL_GSEL_MAX                       (0x00000003U)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR0_GSEL_RESETVAL                       (0x00000000U)

/* INPUTXBAR0_G0 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR0_G0_SEL_MASK                         (0x000000FFU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR0_G0_SEL_SHIFT                        (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR0_G0_SEL_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR0_G0_SEL_MAX                          (0x000000FFU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR0_G0_RESETVAL                         (0x00000000U)

/* INPUTXBAR0_G1 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR0_G1_SEL_MASK                         (0x0000001FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR0_G1_SEL_SHIFT                        (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR0_G1_SEL_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR0_G1_SEL_MAX                          (0x0000001FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR0_G1_RESETVAL                         (0x00000000U)

/* INPUTXBAR0_G2 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR0_G2_SEL_MASK                         (0x0000000FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR0_G2_SEL_SHIFT                        (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR0_G2_SEL_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR0_G2_SEL_MAX                          (0x0000000FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR0_G2_RESETVAL                         (0x00000000U)

/* INPUTXBAR1_GSEL */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR1_GSEL_GSEL_MASK                      (0x00000003U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR1_GSEL_GSEL_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR1_GSEL_GSEL_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR1_GSEL_GSEL_MAX                       (0x00000003U)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR1_GSEL_RESETVAL                       (0x00000000U)

/* INPUTXBAR1_G0 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR1_G0_SEL_MASK                         (0x000000FFU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR1_G0_SEL_SHIFT                        (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR1_G0_SEL_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR1_G0_SEL_MAX                          (0x000000FFU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR1_G0_RESETVAL                         (0x00000000U)

/* INPUTXBAR1_G1 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR1_G1_SEL_MASK                         (0x0000001FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR1_G1_SEL_SHIFT                        (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR1_G1_SEL_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR1_G1_SEL_MAX                          (0x0000001FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR1_G1_RESETVAL                         (0x00000000U)

/* INPUTXBAR1_G2 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR1_G2_SEL_MASK                         (0x0000000FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR1_G2_SEL_SHIFT                        (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR1_G2_SEL_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR1_G2_SEL_MAX                          (0x0000000FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR1_G2_RESETVAL                         (0x00000000U)

/* INPUTXBAR2_GSEL */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR2_GSEL_GSEL_MASK                      (0x00000003U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR2_GSEL_GSEL_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR2_GSEL_GSEL_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR2_GSEL_GSEL_MAX                       (0x00000003U)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR2_GSEL_RESETVAL                       (0x00000000U)

/* INPUTXBAR2_G0 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR2_G0_SEL_MASK                         (0x000000FFU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR2_G0_SEL_SHIFT                        (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR2_G0_SEL_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR2_G0_SEL_MAX                          (0x000000FFU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR2_G0_RESETVAL                         (0x00000000U)

/* INPUTXBAR2_G1 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR2_G1_SEL_MASK                         (0x0000001FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR2_G1_SEL_SHIFT                        (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR2_G1_SEL_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR2_G1_SEL_MAX                          (0x0000001FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR2_G1_RESETVAL                         (0x00000000U)

/* INPUTXBAR2_G2 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR2_G2_SEL_MASK                         (0x0000000FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR2_G2_SEL_SHIFT                        (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR2_G2_SEL_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR2_G2_SEL_MAX                          (0x0000000FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR2_G2_RESETVAL                         (0x00000000U)

/* INPUTXBAR3_GSEL */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR3_GSEL_GSEL_MASK                      (0x00000003U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR3_GSEL_GSEL_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR3_GSEL_GSEL_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR3_GSEL_GSEL_MAX                       (0x00000003U)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR3_GSEL_RESETVAL                       (0x00000000U)

/* INPUTXBAR3_G0 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR3_G0_SEL_MASK                         (0x000000FFU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR3_G0_SEL_SHIFT                        (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR3_G0_SEL_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR3_G0_SEL_MAX                          (0x000000FFU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR3_G0_RESETVAL                         (0x00000000U)

/* INPUTXBAR3_G1 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR3_G1_SEL_MASK                         (0x0000001FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR3_G1_SEL_SHIFT                        (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR3_G1_SEL_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR3_G1_SEL_MAX                          (0x0000001FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR3_G1_RESETVAL                         (0x00000000U)

/* INPUTXBAR3_G2 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR3_G2_SEL_MASK                         (0x0000000FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR3_G2_SEL_SHIFT                        (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR3_G2_SEL_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR3_G2_SEL_MAX                          (0x0000000FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR3_G2_RESETVAL                         (0x00000000U)

/* INPUTXBAR4_GSEL */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR4_GSEL_GSEL_MASK                      (0x00000003U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR4_GSEL_GSEL_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR4_GSEL_GSEL_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR4_GSEL_GSEL_MAX                       (0x00000003U)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR4_GSEL_RESETVAL                       (0x00000000U)

/* INPUTXBAR4_G0 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR4_G0_SEL_MASK                         (0x000000FFU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR4_G0_SEL_SHIFT                        (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR4_G0_SEL_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR4_G0_SEL_MAX                          (0x000000FFU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR4_G0_RESETVAL                         (0x00000000U)

/* INPUTXBAR4_G1 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR4_G1_SEL_MASK                         (0x0000001FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR4_G1_SEL_SHIFT                        (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR4_G1_SEL_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR4_G1_SEL_MAX                          (0x0000001FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR4_G1_RESETVAL                         (0x00000000U)

/* INPUTXBAR4_G2 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR4_G2_SEL_MASK                         (0x0000000FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR4_G2_SEL_SHIFT                        (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR4_G2_SEL_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR4_G2_SEL_MAX                          (0x0000000FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR4_G2_RESETVAL                         (0x00000000U)

/* INPUTXBAR5_GSEL */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR5_GSEL_GSEL_MASK                      (0x00000003U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR5_GSEL_GSEL_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR5_GSEL_GSEL_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR5_GSEL_GSEL_MAX                       (0x00000003U)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR5_GSEL_RESETVAL                       (0x00000000U)

/* INPUTXBAR5_G0 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR5_G0_SEL_MASK                         (0x000000FFU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR5_G0_SEL_SHIFT                        (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR5_G0_SEL_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR5_G0_SEL_MAX                          (0x000000FFU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR5_G0_RESETVAL                         (0x00000000U)

/* INPUTXBAR5_G1 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR5_G1_SEL_MASK                         (0x0000001FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR5_G1_SEL_SHIFT                        (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR5_G1_SEL_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR5_G1_SEL_MAX                          (0x0000001FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR5_G1_RESETVAL                         (0x00000000U)

/* INPUTXBAR5_G2 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR5_G2_SEL_MASK                         (0x0000000FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR5_G2_SEL_SHIFT                        (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR5_G2_SEL_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR5_G2_SEL_MAX                          (0x0000000FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR5_G2_RESETVAL                         (0x00000000U)

/* INPUTXBAR6_GSEL */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR6_GSEL_GSEL_MASK                      (0x00000003U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR6_GSEL_GSEL_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR6_GSEL_GSEL_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR6_GSEL_GSEL_MAX                       (0x00000003U)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR6_GSEL_RESETVAL                       (0x00000000U)

/* INPUTXBAR6_G0 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR6_G0_SEL_MASK                         (0x000000FFU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR6_G0_SEL_SHIFT                        (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR6_G0_SEL_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR6_G0_SEL_MAX                          (0x000000FFU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR6_G0_RESETVAL                         (0x00000000U)

/* INPUTXBAR6_G1 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR6_G1_SEL_MASK                         (0x0000001FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR6_G1_SEL_SHIFT                        (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR6_G1_SEL_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR6_G1_SEL_MAX                          (0x0000001FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR6_G1_RESETVAL                         (0x00000000U)

/* INPUTXBAR6_G2 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR6_G2_SEL_MASK                         (0x0000000FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR6_G2_SEL_SHIFT                        (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR6_G2_SEL_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR6_G2_SEL_MAX                          (0x0000000FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR6_G2_RESETVAL                         (0x00000000U)

/* INPUTXBAR7_GSEL */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR7_GSEL_GSEL_MASK                      (0x00000003U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR7_GSEL_GSEL_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR7_GSEL_GSEL_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR7_GSEL_GSEL_MAX                       (0x00000003U)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR7_GSEL_RESETVAL                       (0x00000000U)

/* INPUTXBAR7_G0 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR7_G0_SEL_MASK                         (0x000000FFU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR7_G0_SEL_SHIFT                        (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR7_G0_SEL_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR7_G0_SEL_MAX                          (0x000000FFU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR7_G0_RESETVAL                         (0x00000000U)

/* INPUTXBAR7_G1 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR7_G1_SEL_MASK                         (0x0000001FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR7_G1_SEL_SHIFT                        (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR7_G1_SEL_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR7_G1_SEL_MAX                          (0x0000001FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR7_G1_RESETVAL                         (0x00000000U)

/* INPUTXBAR7_G2 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR7_G2_SEL_MASK                         (0x0000000FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR7_G2_SEL_SHIFT                        (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR7_G2_SEL_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR7_G2_SEL_MAX                          (0x0000000FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR7_G2_RESETVAL                         (0x00000000U)

/* INPUTXBAR8_GSEL */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR8_GSEL_GSEL_MASK                      (0x00000003U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR8_GSEL_GSEL_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR8_GSEL_GSEL_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR8_GSEL_GSEL_MAX                       (0x00000003U)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR8_GSEL_RESETVAL                       (0x00000000U)

/* INPUTXBAR8_G0 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR8_G0_SEL_MASK                         (0x000000FFU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR8_G0_SEL_SHIFT                        (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR8_G0_SEL_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR8_G0_SEL_MAX                          (0x000000FFU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR8_G0_RESETVAL                         (0x00000000U)

/* INPUTXBAR8_G1 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR8_G1_SEL_MASK                         (0x0000001FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR8_G1_SEL_SHIFT                        (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR8_G1_SEL_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR8_G1_SEL_MAX                          (0x0000001FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR8_G1_RESETVAL                         (0x00000000U)

/* INPUTXBAR8_G2 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR8_G2_SEL_MASK                         (0x0000000FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR8_G2_SEL_SHIFT                        (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR8_G2_SEL_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR8_G2_SEL_MAX                          (0x0000000FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR8_G2_RESETVAL                         (0x00000000U)

/* INPUTXBAR9_GSEL */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR9_GSEL_GSEL_MASK                      (0x00000003U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR9_GSEL_GSEL_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR9_GSEL_GSEL_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR9_GSEL_GSEL_MAX                       (0x00000003U)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR9_GSEL_RESETVAL                       (0x00000000U)

/* INPUTXBAR9_G0 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR9_G0_SEL_MASK                         (0x000000FFU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR9_G0_SEL_SHIFT                        (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR9_G0_SEL_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR9_G0_SEL_MAX                          (0x000000FFU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR9_G0_RESETVAL                         (0x00000000U)

/* INPUTXBAR9_G1 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR9_G1_SEL_MASK                         (0x0000001FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR9_G1_SEL_SHIFT                        (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR9_G1_SEL_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR9_G1_SEL_MAX                          (0x0000001FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR9_G1_RESETVAL                         (0x00000000U)

/* INPUTXBAR9_G2 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR9_G2_SEL_MASK                         (0x0000000FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR9_G2_SEL_SHIFT                        (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR9_G2_SEL_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR9_G2_SEL_MAX                          (0x0000000FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR9_G2_RESETVAL                         (0x00000000U)

/* INPUTXBAR10_GSEL */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR10_GSEL_GSEL_MASK                     (0x00000003U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR10_GSEL_GSEL_SHIFT                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR10_GSEL_GSEL_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR10_GSEL_GSEL_MAX                      (0x00000003U)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR10_GSEL_RESETVAL                      (0x00000000U)

/* INPUTXBAR10_G0 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR10_G0_SEL_MASK                        (0x000000FFU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR10_G0_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR10_G0_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR10_G0_SEL_MAX                         (0x000000FFU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR10_G0_RESETVAL                        (0x00000000U)

/* INPUTXBAR10_G1 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR10_G1_SEL_MASK                        (0x0000001FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR10_G1_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR10_G1_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR10_G1_SEL_MAX                         (0x0000001FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR10_G1_RESETVAL                        (0x00000000U)

/* INPUTXBAR10_G2 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR10_G2_SEL_MASK                        (0x0000000FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR10_G2_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR10_G2_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR10_G2_SEL_MAX                         (0x0000000FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR10_G2_RESETVAL                        (0x00000000U)

/* INPUTXBAR11_GSEL */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR11_GSEL_GSEL_MASK                     (0x00000003U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR11_GSEL_GSEL_SHIFT                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR11_GSEL_GSEL_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR11_GSEL_GSEL_MAX                      (0x00000003U)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR11_GSEL_RESETVAL                      (0x00000000U)

/* INPUTXBAR11_G0 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR11_G0_SEL_MASK                        (0x000000FFU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR11_G0_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR11_G0_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR11_G0_SEL_MAX                         (0x000000FFU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR11_G0_RESETVAL                        (0x00000000U)

/* INPUTXBAR11_G1 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR11_G1_SEL_MASK                        (0x0000001FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR11_G1_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR11_G1_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR11_G1_SEL_MAX                         (0x0000001FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR11_G1_RESETVAL                        (0x00000000U)

/* INPUTXBAR11_G2 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR11_G2_SEL_MASK                        (0x0000000FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR11_G2_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR11_G2_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR11_G2_SEL_MAX                         (0x0000000FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR11_G2_RESETVAL                        (0x00000000U)

/* INPUTXBAR12_GSEL */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR12_GSEL_GSEL_MASK                     (0x00000003U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR12_GSEL_GSEL_SHIFT                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR12_GSEL_GSEL_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR12_GSEL_GSEL_MAX                      (0x00000003U)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR12_GSEL_RESETVAL                      (0x00000000U)

/* INPUTXBAR12_G0 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR12_G0_SEL_MASK                        (0x000000FFU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR12_G0_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR12_G0_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR12_G0_SEL_MAX                         (0x000000FFU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR12_G0_RESETVAL                        (0x00000000U)

/* INPUTXBAR12_G1 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR12_G1_SEL_MASK                        (0x0000001FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR12_G1_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR12_G1_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR12_G1_SEL_MAX                         (0x0000001FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR12_G1_RESETVAL                        (0x00000000U)

/* INPUTXBAR12_G2 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR12_G2_SEL_MASK                        (0x0000000FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR12_G2_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR12_G2_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR12_G2_SEL_MAX                         (0x0000000FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR12_G2_RESETVAL                        (0x00000000U)

/* INPUTXBAR13_GSEL */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR13_GSEL_GSEL_MASK                     (0x00000003U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR13_GSEL_GSEL_SHIFT                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR13_GSEL_GSEL_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR13_GSEL_GSEL_MAX                      (0x00000003U)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR13_GSEL_RESETVAL                      (0x00000000U)

/* INPUTXBAR13_G0 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR13_G0_SEL_MASK                        (0x000000FFU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR13_G0_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR13_G0_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR13_G0_SEL_MAX                         (0x000000FFU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR13_G0_RESETVAL                        (0x00000000U)

/* INPUTXBAR13_G1 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR13_G1_SEL_MASK                        (0x0000001FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR13_G1_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR13_G1_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR13_G1_SEL_MAX                         (0x0000001FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR13_G1_RESETVAL                        (0x00000000U)

/* INPUTXBAR13_G2 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR13_G2_SEL_MASK                        (0x0000000FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR13_G2_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR13_G2_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR13_G2_SEL_MAX                         (0x0000000FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR13_G2_RESETVAL                        (0x00000000U)

/* INPUTXBAR14_GSEL */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR14_GSEL_GSEL_MASK                     (0x00000003U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR14_GSEL_GSEL_SHIFT                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR14_GSEL_GSEL_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR14_GSEL_GSEL_MAX                      (0x00000003U)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR14_GSEL_RESETVAL                      (0x00000000U)

/* INPUTXBAR14_G0 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR14_G0_SEL_MASK                        (0x000000FFU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR14_G0_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR14_G0_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR14_G0_SEL_MAX                         (0x000000FFU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR14_G0_RESETVAL                        (0x00000000U)

/* INPUTXBAR14_G1 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR14_G1_SEL_MASK                        (0x0000001FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR14_G1_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR14_G1_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR14_G1_SEL_MAX                         (0x0000001FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR14_G1_RESETVAL                        (0x00000000U)

/* INPUTXBAR14_G2 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR14_G2_SEL_MASK                        (0x0000000FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR14_G2_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR14_G2_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR14_G2_SEL_MAX                         (0x0000000FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR14_G2_RESETVAL                        (0x00000000U)

/* INPUTXBAR15_GSEL */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR15_GSEL_GSEL_MASK                     (0x00000003U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR15_GSEL_GSEL_SHIFT                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR15_GSEL_GSEL_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR15_GSEL_GSEL_MAX                      (0x00000003U)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR15_GSEL_RESETVAL                      (0x00000000U)

/* INPUTXBAR15_G0 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR15_G0_SEL_MASK                        (0x000000FFU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR15_G0_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR15_G0_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR15_G0_SEL_MAX                         (0x000000FFU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR15_G0_RESETVAL                        (0x00000000U)

/* INPUTXBAR15_G1 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR15_G1_SEL_MASK                        (0x0000001FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR15_G1_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR15_G1_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR15_G1_SEL_MAX                         (0x0000001FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR15_G1_RESETVAL                        (0x00000000U)

/* INPUTXBAR15_G2 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR15_G2_SEL_MASK                        (0x0000000FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR15_G2_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR15_G2_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR15_G2_SEL_MAX                         (0x0000000FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR15_G2_RESETVAL                        (0x00000000U)

/* INPUTXBAR16_GSEL */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR16_GSEL_GSEL_MASK                     (0x00000003U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR16_GSEL_GSEL_SHIFT                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR16_GSEL_GSEL_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR16_GSEL_GSEL_MAX                      (0x00000003U)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR16_GSEL_RESETVAL                      (0x00000000U)

/* INPUTXBAR16_G0 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR16_G0_SEL_MASK                        (0x000000FFU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR16_G0_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR16_G0_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR16_G0_SEL_MAX                         (0x000000FFU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR16_G0_RESETVAL                        (0x00000000U)

/* INPUTXBAR16_G1 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR16_G1_SEL_MASK                        (0x0000001FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR16_G1_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR16_G1_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR16_G1_SEL_MAX                         (0x0000001FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR16_G1_RESETVAL                        (0x00000000U)

/* INPUTXBAR16_G2 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR16_G2_SEL_MASK                        (0x0000000FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR16_G2_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR16_G2_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR16_G2_SEL_MAX                         (0x0000000FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR16_G2_RESETVAL                        (0x00000000U)

/* INPUTXBAR17_GSEL */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR17_GSEL_GSEL_MASK                     (0x00000003U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR17_GSEL_GSEL_SHIFT                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR17_GSEL_GSEL_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR17_GSEL_GSEL_MAX                      (0x00000003U)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR17_GSEL_RESETVAL                      (0x00000000U)

/* INPUTXBAR17_G0 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR17_G0_SEL_MASK                        (0x000000FFU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR17_G0_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR17_G0_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR17_G0_SEL_MAX                         (0x000000FFU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR17_G0_RESETVAL                        (0x00000000U)

/* INPUTXBAR17_G1 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR17_G1_SEL_MASK                        (0x0000001FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR17_G1_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR17_G1_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR17_G1_SEL_MAX                         (0x0000001FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR17_G1_RESETVAL                        (0x00000000U)

/* INPUTXBAR17_G2 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR17_G2_SEL_MASK                        (0x0000000FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR17_G2_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR17_G2_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR17_G2_SEL_MAX                         (0x0000000FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR17_G2_RESETVAL                        (0x00000000U)

/* INPUTXBAR18_GSEL */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR18_GSEL_GSEL_MASK                     (0x00000003U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR18_GSEL_GSEL_SHIFT                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR18_GSEL_GSEL_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR18_GSEL_GSEL_MAX                      (0x00000003U)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR18_GSEL_RESETVAL                      (0x00000000U)

/* INPUTXBAR18_G0 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR18_G0_SEL_MASK                        (0x000000FFU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR18_G0_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR18_G0_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR18_G0_SEL_MAX                         (0x000000FFU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR18_G0_RESETVAL                        (0x00000000U)

/* INPUTXBAR18_G1 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR18_G1_SEL_MASK                        (0x0000001FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR18_G1_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR18_G1_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR18_G1_SEL_MAX                         (0x0000001FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR18_G1_RESETVAL                        (0x00000000U)

/* INPUTXBAR18_G2 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR18_G2_SEL_MASK                        (0x0000000FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR18_G2_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR18_G2_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR18_G2_SEL_MAX                         (0x0000000FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR18_G2_RESETVAL                        (0x00000000U)

/* INPUTXBAR19_GSEL */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR19_GSEL_GSEL_MASK                     (0x00000003U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR19_GSEL_GSEL_SHIFT                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR19_GSEL_GSEL_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR19_GSEL_GSEL_MAX                      (0x00000003U)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR19_GSEL_RESETVAL                      (0x00000000U)

/* INPUTXBAR19_G0 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR19_G0_SEL_MASK                        (0x000000FFU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR19_G0_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR19_G0_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR19_G0_SEL_MAX                         (0x000000FFU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR19_G0_RESETVAL                        (0x00000000U)

/* INPUTXBAR19_G1 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR19_G1_SEL_MASK                        (0x0000001FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR19_G1_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR19_G1_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR19_G1_SEL_MAX                         (0x0000001FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR19_G1_RESETVAL                        (0x00000000U)

/* INPUTXBAR19_G2 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR19_G2_SEL_MASK                        (0x0000000FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR19_G2_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR19_G2_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR19_G2_SEL_MAX                         (0x0000000FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR19_G2_RESETVAL                        (0x00000000U)

/* INPUTXBAR20_GSEL */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR20_GSEL_GSEL_MASK                     (0x00000003U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR20_GSEL_GSEL_SHIFT                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR20_GSEL_GSEL_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR20_GSEL_GSEL_MAX                      (0x00000003U)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR20_GSEL_RESETVAL                      (0x00000000U)

/* INPUTXBAR20_G0 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR20_G0_SEL_MASK                        (0x000000FFU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR20_G0_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR20_G0_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR20_G0_SEL_MAX                         (0x000000FFU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR20_G0_RESETVAL                        (0x00000000U)

/* INPUTXBAR20_G1 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR20_G1_SEL_MASK                        (0x0000001FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR20_G1_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR20_G1_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR20_G1_SEL_MAX                         (0x0000001FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR20_G1_RESETVAL                        (0x00000000U)

/* INPUTXBAR20_G2 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR20_G2_SEL_MASK                        (0x0000000FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR20_G2_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR20_G2_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR20_G2_SEL_MAX                         (0x0000000FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR20_G2_RESETVAL                        (0x00000000U)

/* INPUTXBAR21_GSEL */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR21_GSEL_GSEL_MASK                     (0x00000003U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR21_GSEL_GSEL_SHIFT                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR21_GSEL_GSEL_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR21_GSEL_GSEL_MAX                      (0x00000003U)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR21_GSEL_RESETVAL                      (0x00000000U)

/* INPUTXBAR21_G0 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR21_G0_SEL_MASK                        (0x000000FFU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR21_G0_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR21_G0_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR21_G0_SEL_MAX                         (0x000000FFU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR21_G0_RESETVAL                        (0x00000000U)

/* INPUTXBAR21_G1 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR21_G1_SEL_MASK                        (0x0000001FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR21_G1_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR21_G1_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR21_G1_SEL_MAX                         (0x0000001FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR21_G1_RESETVAL                        (0x00000000U)

/* INPUTXBAR21_G2 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR21_G2_SEL_MASK                        (0x0000000FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR21_G2_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR21_G2_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR21_G2_SEL_MAX                         (0x0000000FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR21_G2_RESETVAL                        (0x00000000U)

/* INPUTXBAR22_GSEL */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR22_GSEL_GSEL_MASK                     (0x00000003U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR22_GSEL_GSEL_SHIFT                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR22_GSEL_GSEL_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR22_GSEL_GSEL_MAX                      (0x00000003U)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR22_GSEL_RESETVAL                      (0x00000000U)

/* INPUTXBAR22_G0 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR22_G0_SEL_MASK                        (0x000000FFU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR22_G0_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR22_G0_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR22_G0_SEL_MAX                         (0x000000FFU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR22_G0_RESETVAL                        (0x00000000U)

/* INPUTXBAR22_G1 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR22_G1_SEL_MASK                        (0x0000001FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR22_G1_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR22_G1_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR22_G1_SEL_MAX                         (0x0000001FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR22_G1_RESETVAL                        (0x00000000U)

/* INPUTXBAR22_G2 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR22_G2_SEL_MASK                        (0x0000000FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR22_G2_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR22_G2_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR22_G2_SEL_MAX                         (0x0000000FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR22_G2_RESETVAL                        (0x00000000U)

/* INPUTXBAR23_GSEL */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR23_GSEL_GSEL_MASK                     (0x00000003U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR23_GSEL_GSEL_SHIFT                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR23_GSEL_GSEL_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR23_GSEL_GSEL_MAX                      (0x00000003U)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR23_GSEL_RESETVAL                      (0x00000000U)

/* INPUTXBAR23_G0 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR23_G0_SEL_MASK                        (0x000000FFU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR23_G0_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR23_G0_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR23_G0_SEL_MAX                         (0x000000FFU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR23_G0_RESETVAL                        (0x00000000U)

/* INPUTXBAR23_G1 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR23_G1_SEL_MASK                        (0x0000001FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR23_G1_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR23_G1_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR23_G1_SEL_MAX                         (0x0000001FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR23_G1_RESETVAL                        (0x00000000U)

/* INPUTXBAR23_G2 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR23_G2_SEL_MASK                        (0x0000000FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR23_G2_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR23_G2_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR23_G2_SEL_MAX                         (0x0000000FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR23_G2_RESETVAL                        (0x00000000U)

/* INPUTXBAR24_GSEL */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR24_GSEL_GSEL_MASK                     (0x00000003U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR24_GSEL_GSEL_SHIFT                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR24_GSEL_GSEL_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR24_GSEL_GSEL_MAX                      (0x00000003U)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR24_GSEL_RESETVAL                      (0x00000000U)

/* INPUTXBAR24_G0 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR24_G0_SEL_MASK                        (0x000000FFU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR24_G0_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR24_G0_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR24_G0_SEL_MAX                         (0x000000FFU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR24_G0_RESETVAL                        (0x00000000U)

/* INPUTXBAR24_G1 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR24_G1_SEL_MASK                        (0x0000001FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR24_G1_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR24_G1_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR24_G1_SEL_MAX                         (0x0000001FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR24_G1_RESETVAL                        (0x00000000U)

/* INPUTXBAR24_G2 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR24_G2_SEL_MASK                        (0x0000000FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR24_G2_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR24_G2_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR24_G2_SEL_MAX                         (0x0000000FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR24_G2_RESETVAL                        (0x00000000U)

/* INPUTXBAR25_GSEL */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR25_GSEL_GSEL_MASK                     (0x00000003U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR25_GSEL_GSEL_SHIFT                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR25_GSEL_GSEL_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR25_GSEL_GSEL_MAX                      (0x00000003U)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR25_GSEL_RESETVAL                      (0x00000000U)

/* INPUTXBAR25_G0 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR25_G0_SEL_MASK                        (0x000000FFU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR25_G0_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR25_G0_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR25_G0_SEL_MAX                         (0x000000FFU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR25_G0_RESETVAL                        (0x00000000U)

/* INPUTXBAR25_G1 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR25_G1_SEL_MASK                        (0x0000001FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR25_G1_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR25_G1_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR25_G1_SEL_MAX                         (0x0000001FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR25_G1_RESETVAL                        (0x00000000U)

/* INPUTXBAR25_G2 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR25_G2_SEL_MASK                        (0x0000000FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR25_G2_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR25_G2_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR25_G2_SEL_MAX                         (0x0000000FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR25_G2_RESETVAL                        (0x00000000U)

/* INPUTXBAR26_GSEL */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR26_GSEL_GSEL_MASK                     (0x00000003U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR26_GSEL_GSEL_SHIFT                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR26_GSEL_GSEL_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR26_GSEL_GSEL_MAX                      (0x00000003U)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR26_GSEL_RESETVAL                      (0x00000000U)

/* INPUTXBAR26_G0 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR26_G0_SEL_MASK                        (0x000000FFU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR26_G0_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR26_G0_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR26_G0_SEL_MAX                         (0x000000FFU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR26_G0_RESETVAL                        (0x00000000U)

/* INPUTXBAR26_G1 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR26_G1_SEL_MASK                        (0x0000001FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR26_G1_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR26_G1_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR26_G1_SEL_MAX                         (0x0000001FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR26_G1_RESETVAL                        (0x00000000U)

/* INPUTXBAR26_G2 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR26_G2_SEL_MASK                        (0x0000000FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR26_G2_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR26_G2_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR26_G2_SEL_MAX                         (0x0000000FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR26_G2_RESETVAL                        (0x00000000U)

/* INPUTXBAR27_GSEL */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR27_GSEL_GSEL_MASK                     (0x00000003U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR27_GSEL_GSEL_SHIFT                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR27_GSEL_GSEL_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR27_GSEL_GSEL_MAX                      (0x00000003U)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR27_GSEL_RESETVAL                      (0x00000000U)

/* INPUTXBAR27_G0 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR27_G0_SEL_MASK                        (0x000000FFU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR27_G0_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR27_G0_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR27_G0_SEL_MAX                         (0x000000FFU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR27_G0_RESETVAL                        (0x00000000U)

/* INPUTXBAR27_G1 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR27_G1_SEL_MASK                        (0x0000001FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR27_G1_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR27_G1_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR27_G1_SEL_MAX                         (0x0000001FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR27_G1_RESETVAL                        (0x00000000U)

/* INPUTXBAR27_G2 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR27_G2_SEL_MASK                        (0x0000000FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR27_G2_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR27_G2_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR27_G2_SEL_MAX                         (0x0000000FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR27_G2_RESETVAL                        (0x00000000U)

/* INPUTXBAR28_GSEL */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR28_GSEL_GSEL_MASK                     (0x00000003U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR28_GSEL_GSEL_SHIFT                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR28_GSEL_GSEL_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR28_GSEL_GSEL_MAX                      (0x00000003U)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR28_GSEL_RESETVAL                      (0x00000000U)

/* INPUTXBAR28_G0 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR28_G0_SEL_MASK                        (0x000000FFU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR28_G0_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR28_G0_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR28_G0_SEL_MAX                         (0x000000FFU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR28_G0_RESETVAL                        (0x00000000U)

/* INPUTXBAR28_G1 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR28_G1_SEL_MASK                        (0x0000001FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR28_G1_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR28_G1_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR28_G1_SEL_MAX                         (0x0000001FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR28_G1_RESETVAL                        (0x00000000U)

/* INPUTXBAR28_G2 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR28_G2_SEL_MASK                        (0x0000000FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR28_G2_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR28_G2_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR28_G2_SEL_MAX                         (0x0000000FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR28_G2_RESETVAL                        (0x00000000U)

/* INPUTXBAR29_GSEL */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR29_GSEL_GSEL_MASK                     (0x00000003U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR29_GSEL_GSEL_SHIFT                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR29_GSEL_GSEL_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR29_GSEL_GSEL_MAX                      (0x00000003U)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR29_GSEL_RESETVAL                      (0x00000000U)

/* INPUTXBAR29_G0 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR29_G0_SEL_MASK                        (0x000000FFU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR29_G0_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR29_G0_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR29_G0_SEL_MAX                         (0x000000FFU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR29_G0_RESETVAL                        (0x00000000U)

/* INPUTXBAR29_G1 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR29_G1_SEL_MASK                        (0x0000001FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR29_G1_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR29_G1_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR29_G1_SEL_MAX                         (0x0000001FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR29_G1_RESETVAL                        (0x00000000U)

/* INPUTXBAR29_G2 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR29_G2_SEL_MASK                        (0x0000000FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR29_G2_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR29_G2_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR29_G2_SEL_MAX                         (0x0000000FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR29_G2_RESETVAL                        (0x00000000U)

/* INPUTXBAR30_GSEL */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR30_GSEL_GSEL_MASK                     (0x00000003U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR30_GSEL_GSEL_SHIFT                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR30_GSEL_GSEL_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR30_GSEL_GSEL_MAX                      (0x00000003U)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR30_GSEL_RESETVAL                      (0x00000000U)

/* INPUTXBAR30_G0 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR30_G0_SEL_MASK                        (0x000000FFU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR30_G0_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR30_G0_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR30_G0_SEL_MAX                         (0x000000FFU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR30_G0_RESETVAL                        (0x00000000U)

/* INPUTXBAR30_G1 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR30_G1_SEL_MASK                        (0x0000001FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR30_G1_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR30_G1_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR30_G1_SEL_MAX                         (0x0000001FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR30_G1_RESETVAL                        (0x00000000U)

/* INPUTXBAR30_G2 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR30_G2_SEL_MASK                        (0x0000000FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR30_G2_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR30_G2_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR30_G2_SEL_MAX                         (0x0000000FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR30_G2_RESETVAL                        (0x00000000U)

/* INPUTXBAR31_GSEL */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR31_GSEL_GSEL_MASK                     (0x00000003U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR31_GSEL_GSEL_SHIFT                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR31_GSEL_GSEL_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR31_GSEL_GSEL_MAX                      (0x00000003U)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR31_GSEL_RESETVAL                      (0x00000000U)

/* INPUTXBAR31_G0 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR31_G0_SEL_MASK                        (0x000000FFU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR31_G0_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR31_G0_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR31_G0_SEL_MAX                         (0x000000FFU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR31_G0_RESETVAL                        (0x00000000U)

/* INPUTXBAR31_G1 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR31_G1_SEL_MASK                        (0x0000001FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR31_G1_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR31_G1_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR31_G1_SEL_MAX                         (0x0000001FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR31_G1_RESETVAL                        (0x00000000U)

/* INPUTXBAR31_G2 */

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR31_G2_SEL_MASK                        (0x0000000FU)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR31_G2_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR31_G2_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR31_G2_SEL_MAX                         (0x0000000FU)

#define CSL_CONTROLSS_INPUTXBAR_INPUTXBAR31_G2_RESETVAL                        (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
