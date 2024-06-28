/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
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

#ifndef USB_REG_OFFSETS_H_
#define USB_REG_OFFSETS_H_

// MSS CTRL Register offsets
#define MSS_CTRL_CTRL_USB_CTRL                  0x00000874
#define MSS_CTRL_CTRL_USB_STS                   0x00000878
#define MSS_CTRL_USB_SLAVE_CONTROL              0x00000880
#define MSS_CTRL_USB_MASTER_STANDBY             0x00000884
#define MSS_CTRL_USB_UTMI_DRVVBUS_CONTROL       0x0000088C
#define MSS_CTRL_CONTROL_USBOTGHS_CONTROL       0x00000894


// USB2PHY Register offsets
#define USB2_PHY_TERMINATION_CONTROL                    0x00000000
#define USB2_PHY_RX_CALIB                               0x00000004
#define USB2_PHY_DLLHS_2                                0x00000008
#define USB2_PHY_RX_TEST_2                              0x0000000C
#define USB2_PHY_TX_TEST_CHRG_DET                       0x00000010
#define USB2_PHY_CHRG_DET                               0x00000014
#define USB2_PHY_PWR_CNTL                               0x00000018
#define USB2_PHY_UTMI_INTERFACE_CNTL_1                  0x0000001C
#define USB2_PHY_UTMI_INTERFACE_CNTL_2                  0x00000020
#define USB2_PHY_BIST                                   0x00000024
#define USB2_PHY_BIST_CRC                               0x00000028
#define USB2_PHY_CDR_BIST2                              0x0000002C
#define USB2_PHY_GPIO                                   0x00000030
#define USB2_PHY_DLLHS                                  0x00000034
#define USB2_PHY_USB2PHYCM_TRIM                         0x00000038
#define USB2_PHY_USB2PHYCM_CONFIG                       0x0000003C
#define USB2_PHY_AD_INTERFACE_REG1                      0x00000044
#define USB2_PHY_AD_INTERFACE_REG2                      0x00000048
#define USB2_PHY_AD_INTERFACE_REG3                      0x0000004C
#define USB2_PHY_ANA_CONFIG1                            0x00000050
#define USB2_PHY_ANA_CONFIG2                            0x00000054
#define USB2_PHY_RESERVED                               0x00000058

// #define USB_DWC_3 Register offsets
#define USB_DWC_3_CAPLENGTH                             0x00000000
#define USB_DWC_3_HCSPARAMS1                            0x00000004
#define USB_DWC_3_HCSPARAMS2                            0x00000008
#define USB_DWC_3_HCSPARAMS3                            0x0000000C
#define USB_DWC_3_HCCPARAMS                             0x00000010
#define USB_DWC_3_DBOFF                                 0x00000014
#define USB_DWC_3_RTSOFF                                0x00000018
#define USB_DWC_3_USBCMD                                0x00000020
#define USB_DWC_3_USBSTS                                0x00000024
#define USB_DWC_3_PAGESIZE                              0x00000028
#define USB_DWC_3_DNCTRL                                0x00000034
#define USB_DWC_3_CRCR_LO                               0x00000038
#define USB_DWC_3_CRCR_HI                               0x0000003C
#define USB_DWC_3_DCBAAP_LO                             0x00000050
#define USB_DWC_3_DCBAAP_HI                             0x00000054
#define USB_DWC_3_CONFIG                                0x00000058
#define USB_DWC_3_PORTSC1                               0x00000420
#define USB_DWC_3_PORTPMSC1                             0x00000424
#define USB_DWC_3_PORTLI1                               0x00000428
#define USB_DWC_3_PORTHLPMC1                            0x0000042C
#define USB_DWC_3_PORTSC2                               0x00000430
#define USB_DWC_3_PORTPMSC2                             0x00000434
#define USB_DWC_3_PORTLI2                               0x00000438
#define USB_DWC_3_PORTHLPMC2                            0x0000043C
#define USB_DWC_3_MFINDEX                               0x00000440
#define USB_DWC_3_IMAN0                                 0x00000460
#define USB_DWC_3_IMOD0                                 0x00000464
#define USB_DWC_3_ERSTSZ0                               0x00000468
#define USB_DWC_3_ERSTBA_LO0                            0x00000470
#define USB_DWC_3_ERSTBA_HI0                            0x00000474
#define USB_DWC_3_ERDP_LO0                              0x00000478
#define USB_DWC_3_ERDP_HI0                              0x0000047C
#define USB_DWC_3_IMAN1                                 0x00000480
#define USB_DWC_3_IMOD1                                 0x00000484
#define USB_DWC_3_ERSTSZ1                               0x00000488
#define USB_DWC_3_ERSTBA_LO1                            0x00000490
#define USB_DWC_3_ERSTBA_HI1                            0x00000494
#define USB_DWC_3_ERDP_LO1                              0x00000498
#define USB_DWC_3_ERDP_HI1                              0x0000049C
#define USB_DWC_3_IMAN2                                 0x000004A0
#define USB_DWC_3_IMOD2                                 0x000004A4
#define USB_DWC_3_ERSTSZ2                               0x000004A8
#define USB_DWC_3_ERSTBA_LO2                            0x000004B0
#define USB_DWC_3_ERSTBA_HI2                            0x000004B4
#define USB_DWC_3_ERDP_LO2                              0x000004B8
#define USB_DWC_3_ERDP_HI2                              0x000004BC
#define USB_DWC_3_IMAN3                                 0x000004C0
#define USB_DWC_3_IMOD3                                 0x000004C4
#define USB_DWC_3_ERSTSZ3                               0x000004C8
#define USB_DWC_3_ERSTBA_LO3                            0x000004D0
#define USB_DWC_3_ERSTBA_HI3                            0x000004D4
#define USB_DWC_3_ERDP_LO3                              0x000004D8
#define USB_DWC_3_ERDP_HI3                              0x000004DC
#define USB_DWC_3_DB0                                   0x000004E0
#define USB_DWC_3_DB1                                   0x000004E4
#define USB_DWC_3_DB2                                   0x000004E8
#define USB_DWC_3_DB3                                   0x000004EC
#define USB_DWC_3_DB4                                   0x000004F0
#define USB_DWC_3_DB5                                   0x000004F4
#define USB_DWC_3_DB6                                   0x000004F8
#define USB_DWC_3_DB7                                   0x000004FC
#define USB_DWC_3_DB8                                   0x00000500
#define USB_DWC_3_DB9                                   0x00000504
#define USB_DWC_3_DB10                                  0x00000508
#define USB_DWC_3_DB11                                  0x0000050C
#define USB_DWC_3_DB12                                  0x00000510
#define USB_DWC_3_DB13                                  0x00000514
#define USB_DWC_3_DB14                                  0x00000518
#define USB_DWC_3_DB15                                  0x0000051C
#define USB_DWC_3_DB16                                  0x00000520
#define USB_DWC_3_DB17                                  0x00000524
#define USB_DWC_3_DB18                                  0x00000528
#define USB_DWC_3_DB19                                  0x0000052C
#define USB_DWC_3_DB20                                  0x00000530
#define USB_DWC_3_DB21                                  0x00000534
#define USB_DWC_3_DB22                                  0x00000538
#define USB_DWC_3_DB23                                  0x0000053C
#define USB_DWC_3_DB24                                  0x00000540
#define USB_DWC_3_DB25                                  0x00000544
#define USB_DWC_3_DB26                                  0x00000548
#define USB_DWC_3_DB27                                  0x0000054C
#define USB_DWC_3_DB28                                  0x00000550
#define USB_DWC_3_DB29                                  0x00000554
#define USB_DWC_3_DB30                                  0x00000558
#define USB_DWC_3_DB31                                  0x0000055C
#define USB_DWC_3_DB32                                  0x00000560
#define USB_DWC_3_DB33                                  0x00000564
#define USB_DWC_3_DB34                                  0x00000568
#define USB_DWC_3_DB35                                  0x0000056C
#define USB_DWC_3_DB36                                  0x00000570
#define USB_DWC_3_DB37                                  0x00000574
#define USB_DWC_3_DB38                                  0x00000578
#define USB_DWC_3_DB39                                  0x0000057C
#define USB_DWC_3_DB40                                  0x00000580
#define USB_DWC_3_DB41                                  0x00000584
#define USB_DWC_3_DB42                                  0x00000588
#define USB_DWC_3_DB43                                  0x0000058C
#define USB_DWC_3_DB44                                  0x00000590
#define USB_DWC_3_DB45                                  0x00000594
#define USB_DWC_3_DB46                                  0x00000598
#define USB_DWC_3_DB47                                  0x0000059C
#define USB_DWC_3_DB48                                  0x000005A0
#define USB_DWC_3_DB49                                  0x000005A4
#define USB_DWC_3_DB50                                  0x000005A8
#define USB_DWC_3_DB51                                  0x000005AC
#define USB_DWC_3_DB52                                  0x000005B0
#define USB_DWC_3_DB53                                  0x000005B4
#define USB_DWC_3_DB54                                  0x000005B8
#define USB_DWC_3_DB55                                  0x000005BC
#define USB_DWC_3_DB56                                  0x000005C0
#define USB_DWC_3_DB57                                  0x000005C4
#define USB_DWC_3_DB58                                  0x000005C8
#define USB_DWC_3_DB59                                  0x000005CC
#define USB_DWC_3_DB60                                  0x000005D0
#define USB_DWC_3_DB61                                  0x000005D4
#define USB_DWC_3_DB62                                  0x000005D8
#define USB_DWC_3_DB63                                  0x000005DC
#define USB_DWC_3_USBLEGSUP                             0x000008E0
#define USB_DWC_3_USBLEGCTLSTS                          0x000008E4
#define USB_DWC_3_SUPTPRT2_DW0                          0x000008F0
#define USB_DWC_3_SUPTPRT2_DW1                          0x000008F4
#define USB_DWC_3_SUPTPRT2_DW2                          0x000008F8
#define USB_DWC_3_SUPTPRT2_DW3                          0x000008FC
#define USB_DWC_3_SUPTPRT3_DW0                          0x00000900
#define USB_DWC_3_SUPTPRT3_DW1                          0x00000904
#define USB_DWC_3_SUPTPRT3_DW2                          0x00000908
#define USB_DWC_3_SUPTPRT3_DW3                          0x0000090C
#define USB_DWC_3_DCID                                  0x00000910
#define USB_DWC_3_DCDB                                  0x00000914
#define USB_DWC_3_DCERSTSZ                              0x00000918
#define USB_DWC_3_DCERSTBA_LO                           0x00000920
#define USB_DWC_3_DCERSTBA_HI                           0x00000924
#define USB_DWC_3_DCERDP_LO                             0x00000928
#define USB_DWC_3_DCERDP_HI                             0x0000092C
#define USB_DWC_3_DCCTRL                                0x00000930
#define USB_DWC_3_DCST                                  0x00000934
#define USB_DWC_3_DCPORTSC                              0x00000938
#define USB_DWC_3_DCCP_LO                               0x00000940
#define USB_DWC_3_DCCP_HI                               0x00000944
#define USB_DWC_3_DCDDI1                                0x00000948
#define USB_DWC_3_DCDDI2                                0x0000094C
#define USB_DWC_3_GSBUSCFG0                             0x0000C100
#define USB_DWC_3_GSBUSCFG1                             0x0000C104
#define USB_DWC_3_GTXTHRCFG                             0x0000C108
#define USB_DWC_3_GRXTHRCFG                             0x0000C10C
#define USB_DWC_3_GCTL                                  0x0000C110
#define USB_DWC_3_GSTS                                  0x0000C118
#define USB_DWC_3_GSNPSID                               0x0000C120
#define USB_DWC_3_GGPIO                                 0x0000C124
#define USB_DWC_3_GUID                                  0x0000C128
#define USB_DWC_3_GUCTL                                 0x0000C12C
#define USB_DWC_3_GBUSERRADDRLO                         0x0000C130
#define USB_DWC_3_GBUSERRADDRHI                         0x0000C134
#define USB_DWC_3_GPRTBIMAPLO                           0x0000C138
#define USB_DWC_3_GPRTBIMAPHI                           0x0000C13C
#define USB_DWC_3_GHWPARAMS0                            0x0000C140
#define USB_DWC_3_GHWPARAMS1                            0x0000C144
#define USB_DWC_3_GHWPARAMS2                            0x0000C148
#define USB_DWC_3_GHWPARAMS3                            0x0000C14C
#define USB_DWC_3_GHWPARAMS4                            0x0000C150
#define USB_DWC_3_GHWPARAMS5                            0x0000C154
#define USB_DWC_3_GHWPARAMS6                            0x0000C158
#define USB_DWC_3_GHWPARAMS7                            0x0000C15C
#define USB_DWC_3_GDBGFIFOSPACE                         0x0000C160
#define USB_DWC_3_GDBGLTSSM                             0x0000C164
#define USB_DWC_3_GDBGLNMCC                             0x0000C168
#define USB_DWC_3_GDBGBMU                               0x0000C16C
#define USB_DWC_3_GDBGLSPMUX                            0x0000C170
#define USB_DWC_3_GDBGLSP                               0x0000C174
#define USB_DWC_3_GDBGEPINFO0                           0x0000C178
#define USB_DWC_3_GDBGEPINFO1                           0x0000C17C
#define USB_DWC_3_GPRTBIMAP_HSLO                        0x0000C180
#define USB_DWC_3_GPRTBIMAP_HSHI                        0x0000C184
#define USB_DWC_3_GPRTBIMAP_FSLO                        0x0000C188
#define USB_DWC_3_GPRTBIMAP_FSHI                        0x0000C18C
#define USB_DWC_3_GUSB2PHYCFG                           0x0000C200
#define USB_DWC_3_GUSB2PHYACC                           0x0000C280
#define USB_DWC_3_GUSB3PIPECTL                          0x0000C2C0
#define USB_DWC_3_GTXFIFOSIZ0                           0x0000C300
#define USB_DWC_3_GTXFIFOSIZ1                           0x0000C304
#define USB_DWC_3_GTXFIFOSIZ2                           0x0000C308
#define USB_DWC_3_GTXFIFOSIZ3                           0x0000C30C
#define USB_DWC_3_GTXFIFOSIZ4                           0x0000C310
#define USB_DWC_3_GTXFIFOSIZ5                           0x0000C314
#define USB_DWC_3_GTXFIFOSIZ6                           0x0000C318
#define USB_DWC_3_GTXFIFOSIZ7                           0x0000C31C
#define USB_DWC_3_GTXFIFOSIZ8                           0x0000C320
#define USB_DWC_3_GTXFIFOSIZ9                           0x0000C324
#define USB_DWC_3_GTXFIFOSIZ10                          0x0000C328
#define USB_DWC_3_GTXFIFOSIZ11                          0x0000C32C
#define USB_DWC_3_GTXFIFOSIZ12                          0x0000C330
#define USB_DWC_3_GTXFIFOSIZ13                          0x0000C334
#define USB_DWC_3_GTXFIFOSIZ14                          0x0000C338
#define USB_DWC_3_GTXFIFOSIZ15                          0x0000C33C
#define USB_DWC_3_GRXFIFOSIZ0                           0x0000C380
#define USB_DWC_3_GRXFIFOSIZ1                           0x0000C384
#define USB_DWC_3_GRXFIFOSIZ2                           0x0000C388
#define USB_DWC_3_GEVNTADRLO0                           0x0000C400
#define USB_DWC_3_GEVNTADRHI0                           0x0000C404
#define USB_DWC_3_GEVNTSIZ0                             0x0000C408
#define USB_DWC_3_GEVNTCOUNT0                           0x0000C40C
#define USB_DWC_3_GEVNTADRLO1                           0x0000C410
#define USB_DWC_3_GEVNTADRHI1                           0x0000C414
#define USB_DWC_3_GEVNTSIZ1                             0x0000C418
#define USB_DWC_3_GEVNTCOUNT1                           0x0000C41C
#define USB_DWC_3_GEVNTADRLO2                           0x0000C420
#define USB_DWC_3_GEVNTADRHI2                           0x0000C424
#define USB_DWC_3_GEVNTSIZ2                             0x0000C428
#define USB_DWC_3_GEVNTCOUNT2                           0x0000C42C
#define USB_DWC_3_GEVNTADRLO3                           0x0000C430
#define USB_DWC_3_GEVNTADRHI3                           0x0000C434
#define USB_DWC_3_GEVNTSIZ3                             0x0000C438
#define USB_DWC_3_GEVNTCOUNT3                           0x0000C43C
#define USB_DWC_3_GHWPARAMS8                            0x0000C600
#define USB_DWC_3_GHWPARAMS9                            0x0000C604
#define USB_DWC_3_GTXFIFOPRIDEV                         0x0000C610
#define USB_DWC_3_GTXFIFOPRIHST                         0x0000C618
#define USB_DWC_3_GRXFIFOPRIHST                         0x0000C61C
#define USB_DWC_3_GFIFOPRIDBC                           0x0000C620
#define USB_DWC_3_GDMAHLRATIO                           0x0000C624
#define USB_DWC_3_DCFG                                  0x0000C700
#define USB_DWC_3_DCTL                                  0x0000C704
#define USB_DWC_3_DEVTEN                                0x0000C708
#define USB_DWC_3_DSTS                                  0x0000C70C
#define USB_DWC_3_DGCMDPAR                              0x0000C710
#define USB_DWC_3_DGCMD                                 0x0000C714
#define USB_DWC_3_DALEPENA                              0x0000C720
#define USB_DWC_3_DEPCMDPAR20                           0x0000C800
#define USB_DWC_3_DEPCMDPAR10                           0x0000C804
#define USB_DWC_3_DEPCMDPAR00                           0x0000C808
#define USB_DWC_3_DEPCMD0                               0x0000C80C
#define USB_DWC_3_DEPCMDPAR21                           0x0000C810
#define USB_DWC_3_DEPCMDPAR11                           0x0000C814
#define USB_DWC_3_DEPCMDPAR01                           0x0000C818
#define USB_DWC_3_DEPCMD1                               0x0000C81C
#define USB_DWC_3_DEPCMDPAR22                           0x0000C820
#define USB_DWC_3_DEPCMDPAR12                           0x0000C824
#define USB_DWC_3_DEPCMDPAR02                           0x0000C828
#define USB_DWC_3_DEPCMD2                               0x0000C82C
#define USB_DWC_3_DEPCMDPAR23                           0x0000C830
#define USB_DWC_3_DEPCMDPAR13                           0x0000C834
#define USB_DWC_3_DEPCMDPAR03                           0x0000C838
#define USB_DWC_3_DEPCMD3                               0x0000C83C
#define USB_DWC_3_DEPCMDPAR24                           0x0000C840
#define USB_DWC_3_DEPCMDPAR14                           0x0000C844
#define USB_DWC_3_DEPCMDPAR04                           0x0000C848
#define USB_DWC_3_DEPCMD4                               0x0000C84C
#define USB_DWC_3_DEPCMDPAR25                           0x0000C850
#define USB_DWC_3_DEPCMDPAR15                           0x0000C854
#define USB_DWC_3_DEPCMDPAR05                           0x0000C858
#define USB_DWC_3_DEPCMD5                               0x0000C85C
#define USB_DWC_3_DEPCMDPAR26                           0x0000C860
#define USB_DWC_3_DEPCMDPAR16                           0x0000C864
#define USB_DWC_3_DEPCMDPAR06                           0x0000C868
#define USB_DWC_3_DEPCMD6                               0x0000C86C
#define USB_DWC_3_DEPCMDPAR27                           0x0000C870
#define USB_DWC_3_DEPCMDPAR17                           0x0000C874
#define USB_DWC_3_DEPCMDPAR07                           0x0000C878
#define USB_DWC_3_DEPCMD7                               0x0000C87C
#define USB_DWC_3_DEPCMDPAR28                           0x0000C880
#define USB_DWC_3_DEPCMDPAR18                           0x0000C884
#define USB_DWC_3_DEPCMDPAR08                           0x0000C888
#define USB_DWC_3_DEPCMD8                               0x0000C88C
#define USB_DWC_3_DEPCMDPAR29                           0x0000C890
#define USB_DWC_3_DEPCMDPAR19                           0x0000C894
#define USB_DWC_3_DEPCMDPAR09                           0x0000C898
#define USB_DWC_3_DEPCMD9                               0x0000C89C
#define USB_DWC_3_DEPCMDPAR210                          0x0000C8A0
#define USB_DWC_3_DEPCMDPAR110                          0x0000C8A4
#define USB_DWC_3_DEPCMDPAR010                          0x0000C8A8
#define USB_DWC_3_DEPCMD10                              0x0000C8AC
#define USB_DWC_3_DEPCMDPAR211                          0x0000C8B0
#define USB_DWC_3_DEPCMDPAR111                          0x0000C8B4
#define USB_DWC_3_DEPCMDPAR011                          0x0000C8B8
#define USB_DWC_3_DEPCMD11                              0x0000C8BC
#define USB_DWC_3_DEPCMDPAR212                          0x0000C8C0
#define USB_DWC_3_DEPCMDPAR112                          0x0000C8C4
#define USB_DWC_3_DEPCMDPAR012                          0x0000C8C8
#define USB_DWC_3_DEPCMD12                              0x0000C8CC
#define USB_DWC_3_DEPCMDPAR213                          0x0000C8D0
#define USB_DWC_3_DEPCMDPAR113                          0x0000C8D4
#define USB_DWC_3_DEPCMDPAR013                          0x0000C8D8
#define USB_DWC_3_DEPCMD13                              0x0000C8DC
#define USB_DWC_3_DEPCMDPAR214                          0x0000C8E0
#define USB_DWC_3_DEPCMDPAR114                          0x0000C8E4
#define USB_DWC_3_DEPCMDPAR014                          0x0000C8E8
#define USB_DWC_3_DEPCMD14                              0x0000C8EC
#define USB_DWC_3_DEPCMDPAR215                          0x0000C8F0
#define USB_DWC_3_DEPCMDPAR115                          0x0000C8F4
#define USB_DWC_3_DEPCMDPAR015                          0x0000C8F8
#define USB_DWC_3_DEPCMD15                              0x0000C8FC
#define USB_DWC_3_DEPCMDPAR216                          0x0000C900
#define USB_DWC_3_DEPCMDPAR116                          0x0000C904
#define USB_DWC_3_DEPCMDPAR016                          0x0000C908
#define USB_DWC_3_DEPCMD16                              0x0000C90C
#define USB_DWC_3_DEPCMDPAR217                          0x0000C910
#define USB_DWC_3_DEPCMDPAR117                          0x0000C914
#define USB_DWC_3_DEPCMDPAR017                          0x0000C918
#define USB_DWC_3_DEPCMD17                              0x0000C91C
#define USB_DWC_3_DEPCMDPAR218                          0x0000C920
#define USB_DWC_3_DEPCMDPAR118                          0x0000C924
#define USB_DWC_3_DEPCMDPAR018                          0x0000C928
#define USB_DWC_3_DEPCMD18                              0x0000C92C
#define USB_DWC_3_DEPCMDPAR219                          0x0000C930
#define USB_DWC_3_DEPCMDPAR119                          0x0000C934
#define USB_DWC_3_DEPCMDPAR019                          0x0000C938
#define USB_DWC_3_DEPCMD19                              0x0000C93C
#define USB_DWC_3_DEPCMDPAR220                          0x0000C940
#define USB_DWC_3_DEPCMDPAR120                          0x0000C944
#define USB_DWC_3_DEPCMDPAR020                          0x0000C948
#define USB_DWC_3_DEPCMD20                              0x0000C94C
#define USB_DWC_3_DEPCMDPAR221                          0x0000C950
#define USB_DWC_3_DEPCMDPAR121                          0x0000C954
#define USB_DWC_3_DEPCMDPAR021                          0x0000C958
#define USB_DWC_3_DEPCMD21                              0x0000C95C
#define USB_DWC_3_DEPCMDPAR222                          0x0000C960
#define USB_DWC_3_DEPCMDPAR122                          0x0000C964
#define USB_DWC_3_DEPCMDPAR022                          0x0000C968
#define USB_DWC_3_DEPCMD22                              0x0000C96C
#define USB_DWC_3_DEPCMDPAR223                          0x0000C970
#define USB_DWC_3_DEPCMDPAR123                          0x0000C974
#define USB_DWC_3_DEPCMDPAR023                          0x0000C978
#define USB_DWC_3_DEPCMD23                              0x0000C97C
#define USB_DWC_3_DEPCMDPAR224                          0x0000C980
#define USB_DWC_3_DEPCMDPAR124                          0x0000C984
#define USB_DWC_3_DEPCMDPAR024                          0x0000C988
#define USB_DWC_3_DEPCMD24                              0x0000C98C
#define USB_DWC_3_DEPCMDPAR225                          0x0000C990
#define USB_DWC_3_DEPCMDPAR125                          0x0000C994
#define USB_DWC_3_DEPCMDPAR025                          0x0000C998
#define USB_DWC_3_DEPCMD25                              0x0000C99C
#define USB_DWC_3_DEPCMDPAR226                          0x0000C9A0
#define USB_DWC_3_DEPCMDPAR126                          0x0000C9A4
#define USB_DWC_3_DEPCMDPAR026                          0x0000C9A8
#define USB_DWC_3_DEPCMD26                              0x0000C9AC
#define USB_DWC_3_DEPCMDPAR227                          0x0000C9B0
#define USB_DWC_3_DEPCMDPAR127                          0x0000C9B4
#define USB_DWC_3_DEPCMDPAR027                          0x0000C9B8
#define USB_DWC_3_DEPCMD27                              0x0000C9BC
#define USB_DWC_3_DEPCMDPAR228                          0x0000C9C0
#define USB_DWC_3_DEPCMDPAR128                          0x0000C9C4
#define USB_DWC_3_DEPCMDPAR028                          0x0000C9C8
#define USB_DWC_3_DEPCMD28                              0x0000C9CC
#define USB_DWC_3_DEPCMDPAR229                          0x0000C9D0
#define USB_DWC_3_DEPCMDPAR129                          0x0000C9D4
#define USB_DWC_3_DEPCMDPAR029                          0x0000C9D8
#define USB_DWC_3_DEPCMD29                              0x0000C9DC
#define USB_DWC_3_DEPCMDPAR230                          0x0000C9E0
#define USB_DWC_3_DEPCMDPAR130                          0x0000C9E4
#define USB_DWC_3_DEPCMDPAR030                          0x0000C9E8
#define USB_DWC_3_DEPCMD30                              0x0000C9EC
#define USB_DWC_3_DEPCMDPAR231                          0x0000C9F0
#define USB_DWC_3_DEPCMDPAR131                          0x0000C9F4
#define USB_DWC_3_DEPCMDPAR031                          0x0000C9F8
#define USB_DWC_3_DEPCMD31                              0x0000C9FC
#define USB_DWC_3_OCFG                                  0x0000CC00
#define USB_DWC_3_OCTL                                  0x0000CC04
#define USB_DWC_3_OEVT                                  0x0000CC08
#define USB_DWC_3_OEVTEN                                0x0000CC0C
#define USB_DWC_3_OSTS                                  0x0000CC10

// #define USB_OCP2SCP_REG Register offsets
#define USB_OCP2SCP_REG_REVISION                        0x00000000
#define USB_OCP2SCP_REG_SYSCONFIG                       0x00000010
#define USB_OCP2SCP_REG_SYSSTATUS                       0x00000014
#define USB_OCP2SCP_REG_TIMING                          0x00000018

// #define USB_OTGSS_C2 Register offsets
#define USB_OTGSS_C2_REVISION                           0x00000000
#define USB_OTGSS_C2_SYSCONFIG                          0x00000010
#define USB_OTGSS_C2_IRQ_EOI_MAIN                       0x00000018
#define USB_OTGSS_C2_IRQSTATUS_RAW_MAIN_0               0x00000020
#define USB_OTGSS_C2_IRQSTATUS_MAIN_0                   0x00000024
#define USB_OTGSS_C2_IRQENABLE_SET_MAIN_0               0x00000028
#define USB_OTGSS_C2_IRQENABLE_CLR_MAIN_0               0x0000002C
#define USB_OTGSS_C2_IRQSTATUS_RAW_MAIN_1               0x00000030
#define USB_OTGSS_C2_IRQSTATUS_MAIN_1                   0x00000034
#define USB_OTGSS_C2_IRQENABLE_SET_MAIN_1               0x00000038
#define USB_OTGSS_C2_IRQENABLE_CLR_MAIN_1               0x0000003C
#define USB_OTGSS_C2_IRQSTATUS_RAW_MAIN_2               0x00000040
#define USB_OTGSS_C2_IRQSTATUS_MAIN_2                   0x00000044
#define USB_OTGSS_C2_IRQENABLE_SET_MAIN_2               0x00000048
#define USB_OTGSS_C2_IRQENABLE_CLR_MAIN_2               0x0000004C
#define USB_OTGSS_C2_IRQSTATUS_RAW_MAIN_3               0x00000050
#define USB_OTGSS_C2_IRQSTATUS_MAIN_3                   0x00000054
#define USB_OTGSS_C2_IRQENABLE_SET_MAIN_3               0x00000058
#define USB_OTGSS_C2_IRQENABLE_CLR_MAIN_3               0x0000005C
#define USB_OTGSS_C2_IRQ_EOI_MISC                       0x0000042C
#define USB_OTGSS_C2_IRQSTATUS_RAW_MISC                 0x00000430
#define USB_OTGSS_C2_IRQSTATUS_MISC                     0x00000434
#define USB_OTGSS_C2_IRQENABLE_SET_MISC                 0x00000438
#define USB_OTGSS_C2_IRQENABLE_CLR_MISC                 0x0000043C
#define USB_OTGSS_C2_UTMI_OTG_CTRL                      0x00000500
#define USB_OTGSS_C2_UTMI_OTG_STATUS                    0x00000504
#define USB_OTGSS_C2_TXFIFO_DEPTH                       0x00000508
#define USB_OTGSS_C2_RXFIFO_DEPTH                       0x0000050C
#define USB_OTGSS_C2_MMRAM_OFFSET                       0x00000700
#define USB_OTGSS_C2_FLADJ                              0x00000704
#define USB_OTGSS_C2_DEBUG_CFG                          0x00000708
#define USB_OTGSS_C2_DEBUG_DATA                         0x0000070C
#define USB_OTGSS_C2_DEV_EBC_EN                         0x00000710

//#define USB_RAM0 Register offsets
#define USB_RAM0_START                                  0x53908000
#define USB_RAM0_END                                    0x5390FFFC

#endif /* USB_REG_OFFSETS_H_ */
