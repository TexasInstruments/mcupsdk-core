/********************************************************************
 * Copyright (C) 2020 Texas Instruments Incorporated.
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
 *  Name        : cslr_mss_vim.h
*/
#ifndef CSLR_MSS_VIM_H_
#define CSLR_MSS_VIM_H_

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
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t PID;
    volatile uint32_t INFO;
    volatile uint32_t PRIIRQ;
    volatile uint32_t PRIFIQ;
    volatile uint32_t IRQGSTS;
    volatile uint32_t FIQGSTS;
    volatile uint32_t IRQVEC;
    volatile uint32_t FIQVEC;
    volatile uint32_t ACTIRQ;
    volatile uint32_t ACTFIQ;
	volatile uint32_t IRQPRIMSK;		
	volatile uint32_t FIQPRIMSK;		
    volatile uint32_t DEDVEC;
    volatile uint8_t  Resv_1024[972];
    volatile uint32_t RAW;
    volatile uint32_t STS;
    volatile uint32_t INTR_EN_SET;
    volatile uint32_t INTER_EN_CLR;
    volatile uint32_t IRQSTS;
    volatile uint32_t FIQSTS;
    volatile uint32_t INTMAP;
    volatile uint32_t INTTYPE;
    volatile uint32_t RAW_1;
    volatile uint32_t STS_1;
    volatile uint32_t INTR_EN_SET_1;
    volatile uint32_t INTER_EN_CLR_1;
    volatile uint32_t IRQSTS_1;
    volatile uint32_t FIQSTS_1;
    volatile uint32_t INTMAP_1;
    volatile uint32_t INTTYPE_1;
    volatile uint32_t RAW_2;
    volatile uint32_t STS_2;
    volatile uint32_t INTR_EN_SET_2;
    volatile uint32_t INTER_EN_CLR_2;
    volatile uint32_t IRQSTS_2;
    volatile uint32_t FIQSTS_2;
    volatile uint32_t INTMAP_2;
    volatile uint32_t INTTYPE_2;
    volatile uint32_t RAW_3;
    volatile uint32_t STS_3;
    volatile uint32_t INTR_EN_SET_3;
    volatile uint32_t INTER_EN_CLR_3;
    volatile uint32_t IRQSTS_3;
    volatile uint32_t FIQSTS_3;
    volatile uint32_t INTMAP_3;
    volatile uint32_t INTTYPE_3;
    volatile uint32_t RAW_4;
    volatile uint32_t STS_4;
    volatile uint32_t INTR_EN_SET_4;
    volatile uint32_t INTER_EN_CLR_4;
    volatile uint32_t IRQSTS_4;
    volatile uint32_t FIQSTS_4;
    volatile uint32_t INTMAP_4;
    volatile uint32_t INTTYPE_4;
    volatile uint32_t RAW_5;
    volatile uint32_t STS_5;
    volatile uint32_t INTR_EN_SET_5;
    volatile uint32_t INTER_EN_CLR_5;
    volatile uint32_t IRQSTS_5;
    volatile uint32_t FIQSTS_5;
    volatile uint32_t INTMAP_5;
    volatile uint32_t INTTYPE_5;
    volatile uint32_t RAW_6;
    volatile uint32_t STS_6;
    volatile uint32_t INTR_EN_SET_6;
    volatile uint32_t INTER_EN_CLR_6;
    volatile uint32_t IRQSTS_6;
    volatile uint32_t FIQSTS_6;
    volatile uint32_t INTMAP_6;
    volatile uint32_t INTTYPE_6;
    volatile uint32_t RAW_7;
    volatile uint32_t STS_7;
    volatile uint32_t INTR_EN_SET_7;
    volatile uint32_t INTER_EN_CLR_7;
    volatile uint32_t IRQSTS_7;
    volatile uint32_t FIQSTS_7;
    volatile uint32_t INTMAP_7;
    volatile uint32_t INTTYPE_7;
    volatile uint8_t  Resv_4096[2816];
    volatile uint32_t INTPRIORITY;
    volatile uint32_t INTPRIORITY_1;
    volatile uint32_t INTPRIORITY_2;
    volatile uint32_t INTPRIORITY_3;
    volatile uint32_t INTPRIORITY_4;
    volatile uint32_t INTPRIORITY_5;
    volatile uint32_t INTPRIORITY_6;
    volatile uint32_t INTPRIORITY_7;
    volatile uint32_t INTPRIORITY_8;
    volatile uint32_t INTPRIORITY_9;
    volatile uint32_t INTPRIORITY_10;
    volatile uint32_t INTPRIORITY_11;
    volatile uint32_t INTPRIORITY_12;
    volatile uint32_t INTPRIORITY_13;
    volatile uint32_t INTPRIORITY_14;
    volatile uint32_t INTPRIORITY_15;
    volatile uint32_t INTPRIORITY_16;
    volatile uint32_t INTPRIORITY_17;
    volatile uint32_t INTPRIORITY_18;
    volatile uint32_t INTPRIORITY_19;
    volatile uint32_t INTPRIORITY_20;
    volatile uint32_t INTPRIORITY_21;
    volatile uint32_t INTPRIORITY_22;
    volatile uint32_t INTPRIORITY_23;
    volatile uint32_t INTPRIORITY_24;
    volatile uint32_t INTPRIORITY_25;
    volatile uint32_t INTPRIORITY_26;
    volatile uint32_t INTPRIORITY_27;
    volatile uint32_t INTPRIORITY_28;
    volatile uint32_t INTPRIORITY_29;
    volatile uint32_t INTPRIORITY_30;
    volatile uint32_t INTPRIORITY_31;
    volatile uint32_t INTPRIORITY_32;
    volatile uint32_t INTPRIORITY_33;
    volatile uint32_t INTPRIORITY_34;
    volatile uint32_t INTPRIORITY_35;
    volatile uint32_t INTPRIORITY_36;
    volatile uint32_t INTPRIORITY_37;
    volatile uint32_t INTPRIORITY_38;
    volatile uint32_t INTPRIORITY_39;
    volatile uint32_t INTPRIORITY_40;
    volatile uint32_t INTPRIORITY_41;
    volatile uint32_t INTPRIORITY_42;
    volatile uint32_t INTPRIORITY_43;
    volatile uint32_t INTPRIORITY_44;
    volatile uint32_t INTPRIORITY_45;
    volatile uint32_t INTPRIORITY_46;
    volatile uint32_t INTPRIORITY_47;
    volatile uint32_t INTPRIORITY_48;
    volatile uint32_t INTPRIORITY_49;
    volatile uint32_t INTPRIORITY_50;
    volatile uint32_t INTPRIORITY_51;
    volatile uint32_t INTPRIORITY_52;
    volatile uint32_t INTPRIORITY_53;
    volatile uint32_t INTPRIORITY_54;
    volatile uint32_t INTPRIORITY_55;
    volatile uint32_t INTPRIORITY_56;
    volatile uint32_t INTPRIORITY_57;
    volatile uint32_t INTPRIORITY_58;
    volatile uint32_t INTPRIORITY_59;
    volatile uint32_t INTPRIORITY_60;
    volatile uint32_t INTPRIORITY_61;
    volatile uint32_t INTPRIORITY_62;
    volatile uint32_t INTPRIORITY_63;
    volatile uint32_t INTPRIORITY_64;
    volatile uint32_t INTPRIORITY_65;
    volatile uint32_t INTPRIORITY_66;
    volatile uint32_t INTPRIORITY_67;
    volatile uint32_t INTPRIORITY_68;
    volatile uint32_t INTPRIORITY_69;
    volatile uint32_t INTPRIORITY_70;
    volatile uint32_t INTPRIORITY_71;
    volatile uint32_t INTPRIORITY_72;
    volatile uint32_t INTPRIORITY_73;
    volatile uint32_t INTPRIORITY_74;
    volatile uint32_t INTPRIORITY_75;
    volatile uint32_t INTPRIORITY_76;
    volatile uint32_t INTPRIORITY_77;
    volatile uint32_t INTPRIORITY_78;
    volatile uint32_t INTPRIORITY_79;
    volatile uint32_t INTPRIORITY_80;
    volatile uint32_t INTPRIORITY_81;
    volatile uint32_t INTPRIORITY_82;
    volatile uint32_t INTPRIORITY_83;
    volatile uint32_t INTPRIORITY_84;
    volatile uint32_t INTPRIORITY_85;
    volatile uint32_t INTPRIORITY_86;
    volatile uint32_t INTPRIORITY_87;
    volatile uint32_t INTPRIORITY_88;
    volatile uint32_t INTPRIORITY_89;
    volatile uint32_t INTPRIORITY_90;
    volatile uint32_t INTPRIORITY_91;
    volatile uint32_t INTPRIORITY_92;
    volatile uint32_t INTPRIORITY_93;
    volatile uint32_t INTPRIORITY_94;
    volatile uint32_t INTPRIORITY_95;
    volatile uint32_t INTPRIORITY_96;
    volatile uint32_t INTPRIORITY_97;
    volatile uint32_t INTPRIORITY_98;
    volatile uint32_t INTPRIORITY_99;
    volatile uint32_t INTPRIORITY_100;
    volatile uint32_t INTPRIORITY_101;
    volatile uint32_t INTPRIORITY_102;
    volatile uint32_t INTPRIORITY_103;
    volatile uint32_t INTPRIORITY_104;
    volatile uint32_t INTPRIORITY_105;
    volatile uint32_t INTPRIORITY_106;
    volatile uint32_t INTPRIORITY_107;
    volatile uint32_t INTPRIORITY_108;
    volatile uint32_t INTPRIORITY_109;
    volatile uint32_t INTPRIORITY_110;
    volatile uint32_t INTPRIORITY_111;
    volatile uint32_t INTPRIORITY_112;
    volatile uint32_t INTPRIORITY_113;
    volatile uint32_t INTPRIORITY_114;
    volatile uint32_t INTPRIORITY_115;
    volatile uint32_t INTPRIORITY_116;
    volatile uint32_t INTPRIORITY_117;
    volatile uint32_t INTPRIORITY_118;
    volatile uint32_t INTPRIORITY_119;
    volatile uint32_t INTPRIORITY_120;
    volatile uint32_t INTPRIORITY_121;
    volatile uint32_t INTPRIORITY_122;
    volatile uint32_t INTPRIORITY_123;
    volatile uint32_t INTPRIORITY_124;
    volatile uint32_t INTPRIORITY_125;
    volatile uint32_t INTPRIORITY_126;
    volatile uint32_t INTPRIORITY_127;
    volatile uint32_t INTPRIORITY_128;
    volatile uint32_t INTPRIORITY_129;
    volatile uint32_t INTPRIORITY_130;
    volatile uint32_t INTPRIORITY_131;
    volatile uint32_t INTPRIORITY_132;
    volatile uint32_t INTPRIORITY_133;
    volatile uint32_t INTPRIORITY_134;
    volatile uint32_t INTPRIORITY_135;
    volatile uint32_t INTPRIORITY_136;
    volatile uint32_t INTPRIORITY_137;
    volatile uint32_t INTPRIORITY_138;
    volatile uint32_t INTPRIORITY_139;
    volatile uint32_t INTPRIORITY_140;
    volatile uint32_t INTPRIORITY_141;
    volatile uint32_t INTPRIORITY_142;
    volatile uint32_t INTPRIORITY_143;
    volatile uint32_t INTPRIORITY_144;
    volatile uint32_t INTPRIORITY_145;
    volatile uint32_t INTPRIORITY_146;
    volatile uint32_t INTPRIORITY_147;
    volatile uint32_t INTPRIORITY_148;
    volatile uint32_t INTPRIORITY_149;
    volatile uint32_t INTPRIORITY_150;
    volatile uint32_t INTPRIORITY_151;
    volatile uint32_t INTPRIORITY_152;
    volatile uint32_t INTPRIORITY_153;
    volatile uint32_t INTPRIORITY_154;
    volatile uint32_t INTPRIORITY_155;
    volatile uint32_t INTPRIORITY_156;
    volatile uint32_t INTPRIORITY_157;
    volatile uint32_t INTPRIORITY_158;
    volatile uint32_t INTPRIORITY_159;
    volatile uint32_t INTPRIORITY_160;
    volatile uint32_t INTPRIORITY_161;
    volatile uint32_t INTPRIORITY_162;
    volatile uint32_t INTPRIORITY_163;
    volatile uint32_t INTPRIORITY_164;
    volatile uint32_t INTPRIORITY_165;
    volatile uint32_t INTPRIORITY_166;
    volatile uint32_t INTPRIORITY_167;
    volatile uint32_t INTPRIORITY_168;
    volatile uint32_t INTPRIORITY_169;
    volatile uint32_t INTPRIORITY_170;
    volatile uint32_t INTPRIORITY_171;
    volatile uint32_t INTPRIORITY_172;
    volatile uint32_t INTPRIORITY_173;
    volatile uint32_t INTPRIORITY_174;
    volatile uint32_t INTPRIORITY_175;
    volatile uint32_t INTPRIORITY_176;
    volatile uint32_t INTPRIORITY_177;
    volatile uint32_t INTPRIORITY_178;
    volatile uint32_t INTPRIORITY_179;
    volatile uint32_t INTPRIORITY_180;
    volatile uint32_t INTPRIORITY_181;
    volatile uint32_t INTPRIORITY_182;
    volatile uint32_t INTPRIORITY_183;
    volatile uint32_t INTPRIORITY_184;
    volatile uint32_t INTPRIORITY_185;
    volatile uint32_t INTPRIORITY_186;
    volatile uint32_t INTPRIORITY_187;
    volatile uint32_t INTPRIORITY_188;
    volatile uint32_t INTPRIORITY_189;
    volatile uint32_t INTPRIORITY_190;
    volatile uint32_t INTPRIORITY_191;
    volatile uint32_t INTPRIORITY_192;
    volatile uint32_t INTPRIORITY_193;
    volatile uint32_t INTPRIORITY_194;
    volatile uint32_t INTPRIORITY_195;
    volatile uint32_t INTPRIORITY_196;
    volatile uint32_t INTPRIORITY_197;
    volatile uint32_t INTPRIORITY_198;
    volatile uint32_t INTPRIORITY_199;
    volatile uint32_t INTPRIORITY_200;
    volatile uint32_t INTPRIORITY_201;
    volatile uint32_t INTPRIORITY_202;
    volatile uint32_t INTPRIORITY_203;
    volatile uint32_t INTPRIORITY_204;
    volatile uint32_t INTPRIORITY_205;
    volatile uint32_t INTPRIORITY_206;
    volatile uint32_t INTPRIORITY_207;
    volatile uint32_t INTPRIORITY_208;
    volatile uint32_t INTPRIORITY_209;
    volatile uint32_t INTPRIORITY_210;
    volatile uint32_t INTPRIORITY_211;
    volatile uint32_t INTPRIORITY_212;
    volatile uint32_t INTPRIORITY_213;
    volatile uint32_t INTPRIORITY_214;
    volatile uint32_t INTPRIORITY_215;
    volatile uint32_t INTPRIORITY_216;
    volatile uint32_t INTPRIORITY_217;
    volatile uint32_t INTPRIORITY_218;
    volatile uint32_t INTPRIORITY_219;
    volatile uint32_t INTPRIORITY_220;
    volatile uint32_t INTPRIORITY_221;
    volatile uint32_t INTPRIORITY_222;
    volatile uint32_t INTPRIORITY_223;
    volatile uint32_t INTPRIORITY_224;
    volatile uint32_t INTPRIORITY_225;
    volatile uint32_t INTPRIORITY_226;
    volatile uint32_t INTPRIORITY_227;
    volatile uint32_t INTPRIORITY_228;
    volatile uint32_t INTPRIORITY_229;
    volatile uint32_t INTPRIORITY_230;
    volatile uint32_t INTPRIORITY_231;
    volatile uint32_t INTPRIORITY_232;
    volatile uint32_t INTPRIORITY_233;
    volatile uint32_t INTPRIORITY_234;
    volatile uint32_t INTPRIORITY_235;
    volatile uint32_t INTPRIORITY_236;
    volatile uint32_t INTPRIORITY_237;
    volatile uint32_t INTPRIORITY_238;
    volatile uint32_t INTPRIORITY_239;
    volatile uint32_t INTPRIORITY_240;
    volatile uint32_t INTPRIORITY_241;
    volatile uint32_t INTPRIORITY_242;
    volatile uint32_t INTPRIORITY_243;
    volatile uint32_t INTPRIORITY_244;
    volatile uint32_t INTPRIORITY_245;
    volatile uint32_t INTPRIORITY_246;
    volatile uint32_t INTPRIORITY_247;
    volatile uint32_t INTPRIORITY_248;
    volatile uint32_t INTPRIORITY_249;
    volatile uint32_t INTPRIORITY_250;
    volatile uint32_t INTPRIORITY_251;
    volatile uint32_t INTPRIORITY_252;
    volatile uint32_t INTPRIORITY_253;
    volatile uint32_t INTPRIORITY_254;
    volatile uint32_t INTPRIORITY_255;
    volatile uint8_t  Resv_8192[3072];
    volatile uint32_t INTVECTOR;
    volatile uint32_t INTVECTOR_1;
    volatile uint32_t INTVECTOR_2;
    volatile uint32_t INTVECTOR_3;
    volatile uint32_t INTVECTOR_4;
    volatile uint32_t INTVECTOR_5;
    volatile uint32_t INTVECTOR_6;
    volatile uint32_t INTVECTOR_7;
    volatile uint32_t INTVECTOR_8;
    volatile uint32_t INTVECTOR_9;
    volatile uint32_t INTVECTOR_10;
    volatile uint32_t INTVECTOR_11;
    volatile uint32_t INTVECTOR_12;
    volatile uint32_t INTVECTOR_13;
    volatile uint32_t INTVECTOR_14;
    volatile uint32_t INTVECTOR_15;
    volatile uint32_t INTVECTOR_16;
    volatile uint32_t INTVECTOR_17;
    volatile uint32_t INTVECTOR_18;
    volatile uint32_t INTVECTOR_19;
    volatile uint32_t INTVECTOR_20;
    volatile uint32_t INTVECTOR_21;
    volatile uint32_t INTVECTOR_22;
    volatile uint32_t INTVECTOR_23;
    volatile uint32_t INTVECTOR_24;
    volatile uint32_t INTVECTOR_25;
    volatile uint32_t INTVECTOR_26;
    volatile uint32_t INTVECTOR_27;
    volatile uint32_t INTVECTOR_28;
    volatile uint32_t INTVECTOR_29;
    volatile uint32_t INTVECTOR_30;
    volatile uint32_t INTVECTOR_31;
    volatile uint32_t INTVECTOR_32;
    volatile uint32_t INTVECTOR_33;
    volatile uint32_t INTVECTOR_34;
    volatile uint32_t INTVECTOR_35;
    volatile uint32_t INTVECTOR_36;
    volatile uint32_t INTVECTOR_37;
    volatile uint32_t INTVECTOR_38;
    volatile uint32_t INTVECTOR_39;
    volatile uint32_t INTVECTOR_40;
    volatile uint32_t INTVECTOR_41;
    volatile uint32_t INTVECTOR_42;
    volatile uint32_t INTVECTOR_43;
    volatile uint32_t INTVECTOR_44;
    volatile uint32_t INTVECTOR_45;
    volatile uint32_t INTVECTOR_46;
    volatile uint32_t INTVECTOR_47;
    volatile uint32_t INTVECTOR_48;
    volatile uint32_t INTVECTOR_49;
    volatile uint32_t INTVECTOR_50;
    volatile uint32_t INTVECTOR_51;
    volatile uint32_t INTVECTOR_52;
    volatile uint32_t INTVECTOR_53;
    volatile uint32_t INTVECTOR_54;
    volatile uint32_t INTVECTOR_55;
    volatile uint32_t INTVECTOR_56;
    volatile uint32_t INTVECTOR_57;
    volatile uint32_t INTVECTOR_58;
    volatile uint32_t INTVECTOR_59;
    volatile uint32_t INTVECTOR_60;
    volatile uint32_t INTVECTOR_61;
    volatile uint32_t INTVECTOR_62;
    volatile uint32_t INTVECTOR_63;
    volatile uint32_t INTVECTOR_64;
    volatile uint32_t INTVECTOR_65;
    volatile uint32_t INTVECTOR_66;
    volatile uint32_t INTVECTOR_67;
    volatile uint32_t INTVECTOR_68;
    volatile uint32_t INTVECTOR_69;
    volatile uint32_t INTVECTOR_70;
    volatile uint32_t INTVECTOR_71;
    volatile uint32_t INTVECTOR_72;
    volatile uint32_t INTVECTOR_73;
    volatile uint32_t INTVECTOR_74;
    volatile uint32_t INTVECTOR_75;
    volatile uint32_t INTVECTOR_76;
    volatile uint32_t INTVECTOR_77;
    volatile uint32_t INTVECTOR_78;
    volatile uint32_t INTVECTOR_79;
    volatile uint32_t INTVECTOR_80;
    volatile uint32_t INTVECTOR_81;
    volatile uint32_t INTVECTOR_82;
    volatile uint32_t INTVECTOR_83;
    volatile uint32_t INTVECTOR_84;
    volatile uint32_t INTVECTOR_85;
    volatile uint32_t INTVECTOR_86;
    volatile uint32_t INTVECTOR_87;
    volatile uint32_t INTVECTOR_88;
    volatile uint32_t INTVECTOR_89;
    volatile uint32_t INTVECTOR_90;
    volatile uint32_t INTVECTOR_91;
    volatile uint32_t INTVECTOR_92;
    volatile uint32_t INTVECTOR_93;
    volatile uint32_t INTVECTOR_94;
    volatile uint32_t INTVECTOR_95;
    volatile uint32_t INTVECTOR_96;
    volatile uint32_t INTVECTOR_97;
    volatile uint32_t INTVECTOR_98;
    volatile uint32_t INTVECTOR_99;
    volatile uint32_t INTVECTOR_100;
    volatile uint32_t INTVECTOR_101;
    volatile uint32_t INTVECTOR_102;
    volatile uint32_t INTVECTOR_103;
    volatile uint32_t INTVECTOR_104;
    volatile uint32_t INTVECTOR_105;
    volatile uint32_t INTVECTOR_106;
    volatile uint32_t INTVECTOR_107;
    volatile uint32_t INTVECTOR_108;
    volatile uint32_t INTVECTOR_109;
    volatile uint32_t INTVECTOR_110;
    volatile uint32_t INTVECTOR_111;
    volatile uint32_t INTVECTOR_112;
    volatile uint32_t INTVECTOR_113;
    volatile uint32_t INTVECTOR_114;
    volatile uint32_t INTVECTOR_115;
    volatile uint32_t INTVECTOR_116;
    volatile uint32_t INTVECTOR_117;
    volatile uint32_t INTVECTOR_118;
    volatile uint32_t INTVECTOR_119;
    volatile uint32_t INTVECTOR_120;
    volatile uint32_t INTVECTOR_121;
    volatile uint32_t INTVECTOR_122;
    volatile uint32_t INTVECTOR_123;
    volatile uint32_t INTVECTOR_124;
    volatile uint32_t INTVECTOR_125;
    volatile uint32_t INTVECTOR_126;
    volatile uint32_t INTVECTOR_127;
    volatile uint32_t INTVECTOR_128;
    volatile uint32_t INTVECTOR_129;
    volatile uint32_t INTVECTOR_130;
    volatile uint32_t INTVECTOR_131;
    volatile uint32_t INTVECTOR_132;
    volatile uint32_t INTVECTOR_133;
    volatile uint32_t INTVECTOR_134;
    volatile uint32_t INTVECTOR_135;
    volatile uint32_t INTVECTOR_136;
    volatile uint32_t INTVECTOR_137;
    volatile uint32_t INTVECTOR_138;
    volatile uint32_t INTVECTOR_139;
    volatile uint32_t INTVECTOR_140;
    volatile uint32_t INTVECTOR_141;
    volatile uint32_t INTVECTOR_142;
    volatile uint32_t INTVECTOR_143;
    volatile uint32_t INTVECTOR_144;
    volatile uint32_t INTVECTOR_145;
    volatile uint32_t INTVECTOR_146;
    volatile uint32_t INTVECTOR_147;
    volatile uint32_t INTVECTOR_148;
    volatile uint32_t INTVECTOR_149;
    volatile uint32_t INTVECTOR_150;
    volatile uint32_t INTVECTOR_151;
    volatile uint32_t INTVECTOR_152;
    volatile uint32_t INTVECTOR_153;
    volatile uint32_t INTVECTOR_154;
    volatile uint32_t INTVECTOR_155;
    volatile uint32_t INTVECTOR_156;
    volatile uint32_t INTVECTOR_157;
    volatile uint32_t INTVECTOR_158;
    volatile uint32_t INTVECTOR_159;
    volatile uint32_t INTVECTOR_160;
    volatile uint32_t INTVECTOR_161;
    volatile uint32_t INTVECTOR_162;
    volatile uint32_t INTVECTOR_163;
    volatile uint32_t INTVECTOR_164;
    volatile uint32_t INTVECTOR_165;
    volatile uint32_t INTVECTOR_166;
    volatile uint32_t INTVECTOR_167;
    volatile uint32_t INTVECTOR_168;
    volatile uint32_t INTVECTOR_169;
    volatile uint32_t INTVECTOR_170;
    volatile uint32_t INTVECTOR_171;
    volatile uint32_t INTVECTOR_172;
    volatile uint32_t INTVECTOR_173;
    volatile uint32_t INTVECTOR_174;
    volatile uint32_t INTVECTOR_175;
    volatile uint32_t INTVECTOR_176;
    volatile uint32_t INTVECTOR_177;
    volatile uint32_t INTVECTOR_178;
    volatile uint32_t INTVECTOR_179;
    volatile uint32_t INTVECTOR_180;
    volatile uint32_t INTVECTOR_181;
    volatile uint32_t INTVECTOR_182;
    volatile uint32_t INTVECTOR_183;
    volatile uint32_t INTVECTOR_184;
    volatile uint32_t INTVECTOR_185;
    volatile uint32_t INTVECTOR_186;
    volatile uint32_t INTVECTOR_187;
    volatile uint32_t INTVECTOR_188;
    volatile uint32_t INTVECTOR_189;
    volatile uint32_t INTVECTOR_190;
    volatile uint32_t INTVECTOR_191;
    volatile uint32_t INTVECTOR_192;
    volatile uint32_t INTVECTOR_193;
    volatile uint32_t INTVECTOR_194;
    volatile uint32_t INTVECTOR_195;
    volatile uint32_t INTVECTOR_196;
    volatile uint32_t INTVECTOR_197;
    volatile uint32_t INTVECTOR_198;
    volatile uint32_t INTVECTOR_199;
    volatile uint32_t INTVECTOR_200;
    volatile uint32_t INTVECTOR_201;
    volatile uint32_t INTVECTOR_202;
    volatile uint32_t INTVECTOR_203;
    volatile uint32_t INTVECTOR_204;
    volatile uint32_t INTVECTOR_205;
    volatile uint32_t INTVECTOR_206;
    volatile uint32_t INTVECTOR_207;
    volatile uint32_t INTVECTOR_208;
    volatile uint32_t INTVECTOR_209;
    volatile uint32_t INTVECTOR_210;
    volatile uint32_t INTVECTOR_211;
    volatile uint32_t INTVECTOR_212;
    volatile uint32_t INTVECTOR_213;
    volatile uint32_t INTVECTOR_214;
    volatile uint32_t INTVECTOR_215;
    volatile uint32_t INTVECTOR_216;
    volatile uint32_t INTVECTOR_217;
    volatile uint32_t INTVECTOR_218;
    volatile uint32_t INTVECTOR_219;
    volatile uint32_t INTVECTOR_220;
    volatile uint32_t INTVECTOR_221;
    volatile uint32_t INTVECTOR_222;
    volatile uint32_t INTVECTOR_223;
    volatile uint32_t INTVECTOR_224;
    volatile uint32_t INTVECTOR_225;
    volatile uint32_t INTVECTOR_226;
    volatile uint32_t INTVECTOR_227;
    volatile uint32_t INTVECTOR_228;
    volatile uint32_t INTVECTOR_229;
    volatile uint32_t INTVECTOR_230;
    volatile uint32_t INTVECTOR_231;
    volatile uint32_t INTVECTOR_232;
    volatile uint32_t INTVECTOR_233;
    volatile uint32_t INTVECTOR_234;
    volatile uint32_t INTVECTOR_235;
    volatile uint32_t INTVECTOR_236;
    volatile uint32_t INTVECTOR_237;
    volatile uint32_t INTVECTOR_238;
    volatile uint32_t INTVECTOR_239;
    volatile uint32_t INTVECTOR_240;
    volatile uint32_t INTVECTOR_241;
    volatile uint32_t INTVECTOR_242;
    volatile uint32_t INTVECTOR_243;
    volatile uint32_t INTVECTOR_244;
    volatile uint32_t INTVECTOR_245;
    volatile uint32_t INTVECTOR_246;
    volatile uint32_t INTVECTOR_247;
    volatile uint32_t INTVECTOR_248;
    volatile uint32_t INTVECTOR_249;
    volatile uint32_t INTVECTOR_250;
    volatile uint32_t INTVECTOR_251;
    volatile uint32_t INTVECTOR_252;
    volatile uint32_t INTVECTOR_253;
    volatile uint32_t INTVECTOR_254;
    volatile uint32_t INTVECTOR_255;
} CSL_mss_vimRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_MSS_VIM_PID                                                        (0x00000000U)
#define CSL_MSS_VIM_INFO                                                       (0x00000004U)
#define CSL_MSS_VIM_PRIIRQ                                                     (0x00000008U)
#define CSL_MSS_VIM_PRIFIQ                                                     (0x0000000CU)
#define CSL_MSS_VIM_IRQGSTS                                                    (0x00000010U)
#define CSL_MSS_VIM_FIQGSTS                                                    (0x00000014U)
#define CSL_MSS_VIM_IRQVEC                                                     (0x00000018U)
#define CSL_MSS_VIM_FIQVEC                                                     (0x0000001CU)
#define CSL_MSS_VIM_ACTIRQ                                                     (0x00000020U)
#define CSL_MSS_VIM_ACTFIQ                                                     (0x00000024U)
#define CSL_MSS_VIM_IRQPRIMSK                                                  (0x00000028U)
#define CSL_MSS_VIM_FIQPRIMSK                                                  (0x0000002CU)
#define CSL_MSS_VIM_DEDVEC                                                     (0x00000030U)
#define CSL_MSS_VIM_RAW                                                        (0x00000400U)
#define CSL_MSS_VIM_STS                                                        (0x00000404U)
#define CSL_MSS_VIM_INTR_EN_SET                                                (0x00000408U)
#define CSL_MSS_VIM_INTER_EN_CLR                                               (0x0000040CU)
#define CSL_MSS_VIM_IRQSTS                                                     (0x00000410U)
#define CSL_MSS_VIM_FIQSTS                                                     (0x00000414U)
#define CSL_MSS_VIM_INTMAP                                                     (0x00000418U)
#define CSL_MSS_VIM_INTTYPE                                                    (0x0000041CU)
#define CSL_MSS_VIM_RAW_1                                                      (0x00000420U)
#define CSL_MSS_VIM_STS_1                                                      (0x00000424U)
#define CSL_MSS_VIM_INTR_EN_SET_1                                              (0x00000428U)
#define CSL_MSS_VIM_INTER_EN_CLR_1                                             (0x0000042CU)
#define CSL_MSS_VIM_IRQSTS_1                                                   (0x00000430U)
#define CSL_MSS_VIM_FIQSTS_1                                                   (0x00000434U)
#define CSL_MSS_VIM_INTMAP_1                                                   (0x00000438U)
#define CSL_MSS_VIM_INTTYPE_1                                                  (0x0000043CU)
#define CSL_MSS_VIM_RAW_2                                                      (0x00000440U)
#define CSL_MSS_VIM_STS_2                                                      (0x00000444U)
#define CSL_MSS_VIM_INTR_EN_SET_2                                              (0x00000448U)
#define CSL_MSS_VIM_INTER_EN_CLR_2                                             (0x0000044CU)
#define CSL_MSS_VIM_IRQSTS_2                                                   (0x00000450U)
#define CSL_MSS_VIM_FIQSTS_2                                                   (0x00000454U)
#define CSL_MSS_VIM_INTMAP_2                                                   (0x00000458U)
#define CSL_MSS_VIM_INTTYPE_2                                                  (0x0000045CU)
#define CSL_MSS_VIM_RAW_3                                                      (0x00000460U)
#define CSL_MSS_VIM_STS_3                                                      (0x00000464U)
#define CSL_MSS_VIM_INTR_EN_SET_3                                              (0x00000468U)
#define CSL_MSS_VIM_INTER_EN_CLR_3                                             (0x0000046CU)
#define CSL_MSS_VIM_IRQSTS_3                                                   (0x00000470U)
#define CSL_MSS_VIM_FIQSTS_3                                                   (0x00000474U)
#define CSL_MSS_VIM_INTMAP_3                                                   (0x00000478U)
#define CSL_MSS_VIM_INTTYPE_3                                                  (0x0000047CU)
#define CSL_MSS_VIM_RAW_4                                                      (0x00000480U)
#define CSL_MSS_VIM_STS_4                                                      (0x00000484U)
#define CSL_MSS_VIM_INTR_EN_SET_4                                              (0x00000488U)
#define CSL_MSS_VIM_INTER_EN_CLR_4                                             (0x0000048CU)
#define CSL_MSS_VIM_IRQSTS_4                                                   (0x00000490U)
#define CSL_MSS_VIM_FIQSTS_4                                                   (0x00000494U)
#define CSL_MSS_VIM_INTMAP_4                                                   (0x00000498U)
#define CSL_MSS_VIM_INTTYPE_4                                                  (0x0000049CU)
#define CSL_MSS_VIM_RAW_5                                                      (0x000004A0U)
#define CSL_MSS_VIM_STS_5                                                      (0x000004A4U)
#define CSL_MSS_VIM_INTR_EN_SET_5                                              (0x000004A8U)
#define CSL_MSS_VIM_INTER_EN_CLR_5                                             (0x000004ACU)
#define CSL_MSS_VIM_IRQSTS_5                                                   (0x000004B0U)
#define CSL_MSS_VIM_FIQSTS_5                                                   (0x000004B4U)
#define CSL_MSS_VIM_INTMAP_5                                                   (0x000004B8U)
#define CSL_MSS_VIM_INTTYPE_5                                                  (0x000004BCU)
#define CSL_MSS_VIM_RAW_6                                                      (0x000004C0U)
#define CSL_MSS_VIM_STS_6                                                      (0x000004C4U)
#define CSL_MSS_VIM_INTR_EN_SET_6                                              (0x000004C8U)
#define CSL_MSS_VIM_INTER_EN_CLR_6                                             (0x000004CCU)
#define CSL_MSS_VIM_IRQSTS_6                                                   (0x000004D0U)
#define CSL_MSS_VIM_FIQSTS_6                                                   (0x000004D4U)
#define CSL_MSS_VIM_INTMAP_6                                                   (0x000004D8U)
#define CSL_MSS_VIM_INTTYPE_6                                                  (0x000004DCU)
#define CSL_MSS_VIM_RAW_7                                                      (0x000004E0U)
#define CSL_MSS_VIM_STS_7                                                      (0x000004E4U)
#define CSL_MSS_VIM_INTR_EN_SET_7                                              (0x000004E8U)
#define CSL_MSS_VIM_INTER_EN_CLR_7                                             (0x000004ECU)
#define CSL_MSS_VIM_IRQSTS_7                                                   (0x000004F0U)
#define CSL_MSS_VIM_FIQSTS_7                                                   (0x000004F4U)
#define CSL_MSS_VIM_INTMAP_7                                                   (0x000004F8U)
#define CSL_MSS_VIM_INTTYPE_7                                                  (0x000004FCU)
#define CSL_MSS_VIM_INTPRIORITY                                                (0x00001000U)
#define CSL_MSS_VIM_INTPRIORITY_1                                              (0x00001004U)
#define CSL_MSS_VIM_INTPRIORITY_2                                              (0x00001008U)
#define CSL_MSS_VIM_INTPRIORITY_3                                              (0x0000100CU)
#define CSL_MSS_VIM_INTPRIORITY_4                                              (0x00001010U)
#define CSL_MSS_VIM_INTPRIORITY_5                                              (0x00001014U)
#define CSL_MSS_VIM_INTPRIORITY_6                                              (0x00001018U)
#define CSL_MSS_VIM_INTPRIORITY_7                                              (0x0000101CU)
#define CSL_MSS_VIM_INTPRIORITY_8                                              (0x00001020U)
#define CSL_MSS_VIM_INTPRIORITY_9                                              (0x00001024U)
#define CSL_MSS_VIM_INTPRIORITY_10                                             (0x00001028U)
#define CSL_MSS_VIM_INTPRIORITY_11                                             (0x0000102CU)
#define CSL_MSS_VIM_INTPRIORITY_12                                             (0x00001030U)
#define CSL_MSS_VIM_INTPRIORITY_13                                             (0x00001034U)
#define CSL_MSS_VIM_INTPRIORITY_14                                             (0x00001038U)
#define CSL_MSS_VIM_INTPRIORITY_15                                             (0x0000103CU)
#define CSL_MSS_VIM_INTPRIORITY_16                                             (0x00001040U)
#define CSL_MSS_VIM_INTPRIORITY_17                                             (0x00001044U)
#define CSL_MSS_VIM_INTPRIORITY_18                                             (0x00001048U)
#define CSL_MSS_VIM_INTPRIORITY_19                                             (0x0000104CU)
#define CSL_MSS_VIM_INTPRIORITY_20                                             (0x00001050U)
#define CSL_MSS_VIM_INTPRIORITY_21                                             (0x00001054U)
#define CSL_MSS_VIM_INTPRIORITY_22                                             (0x00001058U)
#define CSL_MSS_VIM_INTPRIORITY_23                                             (0x0000105CU)
#define CSL_MSS_VIM_INTPRIORITY_24                                             (0x00001060U)
#define CSL_MSS_VIM_INTPRIORITY_25                                             (0x00001064U)
#define CSL_MSS_VIM_INTPRIORITY_26                                             (0x00001068U)
#define CSL_MSS_VIM_INTPRIORITY_27                                             (0x0000106CU)
#define CSL_MSS_VIM_INTPRIORITY_28                                             (0x00001070U)
#define CSL_MSS_VIM_INTPRIORITY_29                                             (0x00001074U)
#define CSL_MSS_VIM_INTPRIORITY_30                                             (0x00001078U)
#define CSL_MSS_VIM_INTPRIORITY_31                                             (0x0000107CU)
#define CSL_MSS_VIM_INTPRIORITY_32                                             (0x00001080U)
#define CSL_MSS_VIM_INTPRIORITY_33                                             (0x00001084U)
#define CSL_MSS_VIM_INTPRIORITY_34                                             (0x00001088U)
#define CSL_MSS_VIM_INTPRIORITY_35                                             (0x0000108CU)
#define CSL_MSS_VIM_INTPRIORITY_36                                             (0x00001090U)
#define CSL_MSS_VIM_INTPRIORITY_37                                             (0x00001094U)
#define CSL_MSS_VIM_INTPRIORITY_38                                             (0x00001098U)
#define CSL_MSS_VIM_INTPRIORITY_39                                             (0x0000109CU)
#define CSL_MSS_VIM_INTPRIORITY_40                                             (0x000010A0U)
#define CSL_MSS_VIM_INTPRIORITY_41                                             (0x000010A4U)
#define CSL_MSS_VIM_INTPRIORITY_42                                             (0x000010A8U)
#define CSL_MSS_VIM_INTPRIORITY_43                                             (0x000010ACU)
#define CSL_MSS_VIM_INTPRIORITY_44                                             (0x000010B0U)
#define CSL_MSS_VIM_INTPRIORITY_45                                             (0x000010B4U)
#define CSL_MSS_VIM_INTPRIORITY_46                                             (0x000010B8U)
#define CSL_MSS_VIM_INTPRIORITY_47                                             (0x000010BCU)
#define CSL_MSS_VIM_INTPRIORITY_48                                             (0x000010C0U)
#define CSL_MSS_VIM_INTPRIORITY_49                                             (0x000010C4U)
#define CSL_MSS_VIM_INTPRIORITY_50                                             (0x000010C8U)
#define CSL_MSS_VIM_INTPRIORITY_51                                             (0x000010CCU)
#define CSL_MSS_VIM_INTPRIORITY_52                                             (0x000010D0U)
#define CSL_MSS_VIM_INTPRIORITY_53                                             (0x000010D4U)
#define CSL_MSS_VIM_INTPRIORITY_54                                             (0x000010D8U)
#define CSL_MSS_VIM_INTPRIORITY_55                                             (0x000010DCU)
#define CSL_MSS_VIM_INTPRIORITY_56                                             (0x000010E0U)
#define CSL_MSS_VIM_INTPRIORITY_57                                             (0x000010E4U)
#define CSL_MSS_VIM_INTPRIORITY_58                                             (0x000010E8U)
#define CSL_MSS_VIM_INTPRIORITY_59                                             (0x000010ECU)
#define CSL_MSS_VIM_INTPRIORITY_60                                             (0x000010F0U)
#define CSL_MSS_VIM_INTPRIORITY_61                                             (0x000010F4U)
#define CSL_MSS_VIM_INTPRIORITY_62                                             (0x000010F8U)
#define CSL_MSS_VIM_INTPRIORITY_63                                             (0x000010FCU)
#define CSL_MSS_VIM_INTPRIORITY_64                                             (0x00001100U)
#define CSL_MSS_VIM_INTPRIORITY_65                                             (0x00001104U)
#define CSL_MSS_VIM_INTPRIORITY_66                                             (0x00001108U)
#define CSL_MSS_VIM_INTPRIORITY_67                                             (0x0000110CU)
#define CSL_MSS_VIM_INTPRIORITY_68                                             (0x00001110U)
#define CSL_MSS_VIM_INTPRIORITY_69                                             (0x00001114U)
#define CSL_MSS_VIM_INTPRIORITY_70                                             (0x00001118U)
#define CSL_MSS_VIM_INTPRIORITY_71                                             (0x0000111CU)
#define CSL_MSS_VIM_INTPRIORITY_72                                             (0x00001120U)
#define CSL_MSS_VIM_INTPRIORITY_73                                             (0x00001124U)
#define CSL_MSS_VIM_INTPRIORITY_74                                             (0x00001128U)
#define CSL_MSS_VIM_INTPRIORITY_75                                             (0x0000112CU)
#define CSL_MSS_VIM_INTPRIORITY_76                                             (0x00001130U)
#define CSL_MSS_VIM_INTPRIORITY_77                                             (0x00001134U)
#define CSL_MSS_VIM_INTPRIORITY_78                                             (0x00001138U)
#define CSL_MSS_VIM_INTPRIORITY_79                                             (0x0000113CU)
#define CSL_MSS_VIM_INTPRIORITY_80                                             (0x00001140U)
#define CSL_MSS_VIM_INTPRIORITY_81                                             (0x00001144U)
#define CSL_MSS_VIM_INTPRIORITY_82                                             (0x00001148U)
#define CSL_MSS_VIM_INTPRIORITY_83                                             (0x0000114CU)
#define CSL_MSS_VIM_INTPRIORITY_84                                             (0x00001150U)
#define CSL_MSS_VIM_INTPRIORITY_85                                             (0x00001154U)
#define CSL_MSS_VIM_INTPRIORITY_86                                             (0x00001158U)
#define CSL_MSS_VIM_INTPRIORITY_87                                             (0x0000115CU)
#define CSL_MSS_VIM_INTPRIORITY_88                                             (0x00001160U)
#define CSL_MSS_VIM_INTPRIORITY_89                                             (0x00001164U)
#define CSL_MSS_VIM_INTPRIORITY_90                                             (0x00001168U)
#define CSL_MSS_VIM_INTPRIORITY_91                                             (0x0000116CU)
#define CSL_MSS_VIM_INTPRIORITY_92                                             (0x00001170U)
#define CSL_MSS_VIM_INTPRIORITY_93                                             (0x00001174U)
#define CSL_MSS_VIM_INTPRIORITY_94                                             (0x00001178U)
#define CSL_MSS_VIM_INTPRIORITY_95                                             (0x0000117CU)
#define CSL_MSS_VIM_INTPRIORITY_96                                             (0x00001180U)
#define CSL_MSS_VIM_INTPRIORITY_97                                             (0x00001184U)
#define CSL_MSS_VIM_INTPRIORITY_98                                             (0x00001188U)
#define CSL_MSS_VIM_INTPRIORITY_99                                             (0x0000118CU)
#define CSL_MSS_VIM_INTPRIORITY_100                                            (0x00001190U)
#define CSL_MSS_VIM_INTPRIORITY_101                                            (0x00001194U)
#define CSL_MSS_VIM_INTPRIORITY_102                                            (0x00001198U)
#define CSL_MSS_VIM_INTPRIORITY_103                                            (0x0000119CU)
#define CSL_MSS_VIM_INTPRIORITY_104                                            (0x000011A0U)
#define CSL_MSS_VIM_INTPRIORITY_105                                            (0x000011A4U)
#define CSL_MSS_VIM_INTPRIORITY_106                                            (0x000011A8U)
#define CSL_MSS_VIM_INTPRIORITY_107                                            (0x000011ACU)
#define CSL_MSS_VIM_INTPRIORITY_108                                            (0x000011B0U)
#define CSL_MSS_VIM_INTPRIORITY_109                                            (0x000011B4U)
#define CSL_MSS_VIM_INTPRIORITY_110                                            (0x000011B8U)
#define CSL_MSS_VIM_INTPRIORITY_111                                            (0x000011BCU)
#define CSL_MSS_VIM_INTPRIORITY_112                                            (0x000011C0U)
#define CSL_MSS_VIM_INTPRIORITY_113                                            (0x000011C4U)
#define CSL_MSS_VIM_INTPRIORITY_114                                            (0x000011C8U)
#define CSL_MSS_VIM_INTPRIORITY_115                                            (0x000011CCU)
#define CSL_MSS_VIM_INTPRIORITY_116                                            (0x000011D0U)
#define CSL_MSS_VIM_INTPRIORITY_117                                            (0x000011D4U)
#define CSL_MSS_VIM_INTPRIORITY_118                                            (0x000011D8U)
#define CSL_MSS_VIM_INTPRIORITY_119                                            (0x000011DCU)
#define CSL_MSS_VIM_INTPRIORITY_120                                            (0x000011E0U)
#define CSL_MSS_VIM_INTPRIORITY_121                                            (0x000011E4U)
#define CSL_MSS_VIM_INTPRIORITY_122                                            (0x000011E8U)
#define CSL_MSS_VIM_INTPRIORITY_123                                            (0x000011ECU)
#define CSL_MSS_VIM_INTPRIORITY_124                                            (0x000011F0U)
#define CSL_MSS_VIM_INTPRIORITY_125                                            (0x000011F4U)
#define CSL_MSS_VIM_INTPRIORITY_126                                            (0x000011F8U)
#define CSL_MSS_VIM_INTPRIORITY_127                                            (0x000011FCU)
#define CSL_MSS_VIM_INTPRIORITY_128                                            (0x00001200U)
#define CSL_MSS_VIM_INTPRIORITY_129                                            (0x00001204U)
#define CSL_MSS_VIM_INTPRIORITY_130                                            (0x00001208U)
#define CSL_MSS_VIM_INTPRIORITY_131                                            (0x0000120CU)
#define CSL_MSS_VIM_INTPRIORITY_132                                            (0x00001210U)
#define CSL_MSS_VIM_INTPRIORITY_133                                            (0x00001214U)
#define CSL_MSS_VIM_INTPRIORITY_134                                            (0x00001218U)
#define CSL_MSS_VIM_INTPRIORITY_135                                            (0x0000121CU)
#define CSL_MSS_VIM_INTPRIORITY_136                                            (0x00001220U)
#define CSL_MSS_VIM_INTPRIORITY_137                                            (0x00001224U)
#define CSL_MSS_VIM_INTPRIORITY_138                                            (0x00001228U)
#define CSL_MSS_VIM_INTPRIORITY_139                                            (0x0000122CU)
#define CSL_MSS_VIM_INTPRIORITY_140                                            (0x00001230U)
#define CSL_MSS_VIM_INTPRIORITY_141                                            (0x00001234U)
#define CSL_MSS_VIM_INTPRIORITY_142                                            (0x00001238U)
#define CSL_MSS_VIM_INTPRIORITY_143                                            (0x0000123CU)
#define CSL_MSS_VIM_INTPRIORITY_144                                            (0x00001240U)
#define CSL_MSS_VIM_INTPRIORITY_145                                            (0x00001244U)
#define CSL_MSS_VIM_INTPRIORITY_146                                            (0x00001248U)
#define CSL_MSS_VIM_INTPRIORITY_147                                            (0x0000124CU)
#define CSL_MSS_VIM_INTPRIORITY_148                                            (0x00001250U)
#define CSL_MSS_VIM_INTPRIORITY_149                                            (0x00001254U)
#define CSL_MSS_VIM_INTPRIORITY_150                                            (0x00001258U)
#define CSL_MSS_VIM_INTPRIORITY_151                                            (0x0000125CU)
#define CSL_MSS_VIM_INTPRIORITY_152                                            (0x00001260U)
#define CSL_MSS_VIM_INTPRIORITY_153                                            (0x00001264U)
#define CSL_MSS_VIM_INTPRIORITY_154                                            (0x00001268U)
#define CSL_MSS_VIM_INTPRIORITY_155                                            (0x0000126CU)
#define CSL_MSS_VIM_INTPRIORITY_156                                            (0x00001270U)
#define CSL_MSS_VIM_INTPRIORITY_157                                            (0x00001274U)
#define CSL_MSS_VIM_INTPRIORITY_158                                            (0x00001278U)
#define CSL_MSS_VIM_INTPRIORITY_159                                            (0x0000127CU)
#define CSL_MSS_VIM_INTPRIORITY_160                                            (0x00001280U)
#define CSL_MSS_VIM_INTPRIORITY_161                                            (0x00001284U)
#define CSL_MSS_VIM_INTPRIORITY_162                                            (0x00001288U)
#define CSL_MSS_VIM_INTPRIORITY_163                                            (0x0000128CU)
#define CSL_MSS_VIM_INTPRIORITY_164                                            (0x00001290U)
#define CSL_MSS_VIM_INTPRIORITY_165                                            (0x00001294U)
#define CSL_MSS_VIM_INTPRIORITY_166                                            (0x00001298U)
#define CSL_MSS_VIM_INTPRIORITY_167                                            (0x0000129CU)
#define CSL_MSS_VIM_INTPRIORITY_168                                            (0x000012A0U)
#define CSL_MSS_VIM_INTPRIORITY_169                                            (0x000012A4U)
#define CSL_MSS_VIM_INTPRIORITY_170                                            (0x000012A8U)
#define CSL_MSS_VIM_INTPRIORITY_171                                            (0x000012ACU)
#define CSL_MSS_VIM_INTPRIORITY_172                                            (0x000012B0U)
#define CSL_MSS_VIM_INTPRIORITY_173                                            (0x000012B4U)
#define CSL_MSS_VIM_INTPRIORITY_174                                            (0x000012B8U)
#define CSL_MSS_VIM_INTPRIORITY_175                                            (0x000012BCU)
#define CSL_MSS_VIM_INTPRIORITY_176                                            (0x000012C0U)
#define CSL_MSS_VIM_INTPRIORITY_177                                            (0x000012C4U)
#define CSL_MSS_VIM_INTPRIORITY_178                                            (0x000012C8U)
#define CSL_MSS_VIM_INTPRIORITY_179                                            (0x000012CCU)
#define CSL_MSS_VIM_INTPRIORITY_180                                            (0x000012D0U)
#define CSL_MSS_VIM_INTPRIORITY_181                                            (0x000012D4U)
#define CSL_MSS_VIM_INTPRIORITY_182                                            (0x000012D8U)
#define CSL_MSS_VIM_INTPRIORITY_183                                            (0x000012DCU)
#define CSL_MSS_VIM_INTPRIORITY_184                                            (0x000012E0U)
#define CSL_MSS_VIM_INTPRIORITY_185                                            (0x000012E4U)
#define CSL_MSS_VIM_INTPRIORITY_186                                            (0x000012E8U)
#define CSL_MSS_VIM_INTPRIORITY_187                                            (0x000012ECU)
#define CSL_MSS_VIM_INTPRIORITY_188                                            (0x000012F0U)
#define CSL_MSS_VIM_INTPRIORITY_189                                            (0x000012F4U)
#define CSL_MSS_VIM_INTPRIORITY_190                                            (0x000012F8U)
#define CSL_MSS_VIM_INTPRIORITY_191                                            (0x000012FCU)
#define CSL_MSS_VIM_INTPRIORITY_192                                            (0x00001300U)
#define CSL_MSS_VIM_INTPRIORITY_193                                            (0x00001304U)
#define CSL_MSS_VIM_INTPRIORITY_194                                            (0x00001308U)
#define CSL_MSS_VIM_INTPRIORITY_195                                            (0x0000130CU)
#define CSL_MSS_VIM_INTPRIORITY_196                                            (0x00001310U)
#define CSL_MSS_VIM_INTPRIORITY_197                                            (0x00001314U)
#define CSL_MSS_VIM_INTPRIORITY_198                                            (0x00001318U)
#define CSL_MSS_VIM_INTPRIORITY_199                                            (0x0000131CU)
#define CSL_MSS_VIM_INTPRIORITY_200                                            (0x00001320U)
#define CSL_MSS_VIM_INTPRIORITY_201                                            (0x00001324U)
#define CSL_MSS_VIM_INTPRIORITY_202                                            (0x00001328U)
#define CSL_MSS_VIM_INTPRIORITY_203                                            (0x0000132CU)
#define CSL_MSS_VIM_INTPRIORITY_204                                            (0x00001330U)
#define CSL_MSS_VIM_INTPRIORITY_205                                            (0x00001334U)
#define CSL_MSS_VIM_INTPRIORITY_206                                            (0x00001338U)
#define CSL_MSS_VIM_INTPRIORITY_207                                            (0x0000133CU)
#define CSL_MSS_VIM_INTPRIORITY_208                                            (0x00001340U)
#define CSL_MSS_VIM_INTPRIORITY_209                                            (0x00001344U)
#define CSL_MSS_VIM_INTPRIORITY_210                                            (0x00001348U)
#define CSL_MSS_VIM_INTPRIORITY_211                                            (0x0000134CU)
#define CSL_MSS_VIM_INTPRIORITY_212                                            (0x00001350U)
#define CSL_MSS_VIM_INTPRIORITY_213                                            (0x00001354U)
#define CSL_MSS_VIM_INTPRIORITY_214                                            (0x00001358U)
#define CSL_MSS_VIM_INTPRIORITY_215                                            (0x0000135CU)
#define CSL_MSS_VIM_INTPRIORITY_216                                            (0x00001360U)
#define CSL_MSS_VIM_INTPRIORITY_217                                            (0x00001364U)
#define CSL_MSS_VIM_INTPRIORITY_218                                            (0x00001368U)
#define CSL_MSS_VIM_INTPRIORITY_219                                            (0x0000136CU)
#define CSL_MSS_VIM_INTPRIORITY_220                                            (0x00001370U)
#define CSL_MSS_VIM_INTPRIORITY_221                                            (0x00001374U)
#define CSL_MSS_VIM_INTPRIORITY_222                                            (0x00001378U)
#define CSL_MSS_VIM_INTPRIORITY_223                                            (0x0000137CU)
#define CSL_MSS_VIM_INTPRIORITY_224                                            (0x00001380U)
#define CSL_MSS_VIM_INTPRIORITY_225                                            (0x00001384U)
#define CSL_MSS_VIM_INTPRIORITY_226                                            (0x00001388U)
#define CSL_MSS_VIM_INTPRIORITY_227                                            (0x0000138CU)
#define CSL_MSS_VIM_INTPRIORITY_228                                            (0x00001390U)
#define CSL_MSS_VIM_INTPRIORITY_229                                            (0x00001394U)
#define CSL_MSS_VIM_INTPRIORITY_230                                            (0x00001398U)
#define CSL_MSS_VIM_INTPRIORITY_231                                            (0x0000139CU)
#define CSL_MSS_VIM_INTPRIORITY_232                                            (0x000013A0U)
#define CSL_MSS_VIM_INTPRIORITY_233                                            (0x000013A4U)
#define CSL_MSS_VIM_INTPRIORITY_234                                            (0x000013A8U)
#define CSL_MSS_VIM_INTPRIORITY_235                                            (0x000013ACU)
#define CSL_MSS_VIM_INTPRIORITY_236                                            (0x000013B0U)
#define CSL_MSS_VIM_INTPRIORITY_237                                            (0x000013B4U)
#define CSL_MSS_VIM_INTPRIORITY_238                                            (0x000013B8U)
#define CSL_MSS_VIM_INTPRIORITY_239                                            (0x000013BCU)
#define CSL_MSS_VIM_INTPRIORITY_240                                            (0x000013C0U)
#define CSL_MSS_VIM_INTPRIORITY_241                                            (0x000013C4U)
#define CSL_MSS_VIM_INTPRIORITY_242                                            (0x000013C8U)
#define CSL_MSS_VIM_INTPRIORITY_243                                            (0x000013CCU)
#define CSL_MSS_VIM_INTPRIORITY_244                                            (0x000013D0U)
#define CSL_MSS_VIM_INTPRIORITY_245                                            (0x000013D4U)
#define CSL_MSS_VIM_INTPRIORITY_246                                            (0x000013D8U)
#define CSL_MSS_VIM_INTPRIORITY_247                                            (0x000013DCU)
#define CSL_MSS_VIM_INTPRIORITY_248                                            (0x000013E0U)
#define CSL_MSS_VIM_INTPRIORITY_249                                            (0x000013E4U)
#define CSL_MSS_VIM_INTPRIORITY_250                                            (0x000013E8U)
#define CSL_MSS_VIM_INTPRIORITY_251                                            (0x000013ECU)
#define CSL_MSS_VIM_INTPRIORITY_252                                            (0x000013F0U)
#define CSL_MSS_VIM_INTPRIORITY_253                                            (0x000013F4U)
#define CSL_MSS_VIM_INTPRIORITY_254                                            (0x000013F8U)
#define CSL_MSS_VIM_INTPRIORITY_255                                            (0x000013FCU)
#define CSL_MSS_VIM_INTVECTOR                                                  (0x00002000U)
#define CSL_MSS_VIM_INTVECTOR_1                                                (0x00002004U)
#define CSL_MSS_VIM_INTVECTOR_2                                                (0x00002008U)
#define CSL_MSS_VIM_INTVECTOR_3                                                (0x0000200CU)
#define CSL_MSS_VIM_INTVECTOR_4                                                (0x00002010U)
#define CSL_MSS_VIM_INTVECTOR_5                                                (0x00002014U)
#define CSL_MSS_VIM_INTVECTOR_6                                                (0x00002018U)
#define CSL_MSS_VIM_INTVECTOR_7                                                (0x0000201CU)
#define CSL_MSS_VIM_INTVECTOR_8                                                (0x00002020U)
#define CSL_MSS_VIM_INTVECTOR_9                                                (0x00002024U)
#define CSL_MSS_VIM_INTVECTOR_10                                               (0x00002028U)
#define CSL_MSS_VIM_INTVECTOR_11                                               (0x0000202CU)
#define CSL_MSS_VIM_INTVECTOR_12                                               (0x00002030U)
#define CSL_MSS_VIM_INTVECTOR_13                                               (0x00002034U)
#define CSL_MSS_VIM_INTVECTOR_14                                               (0x00002038U)
#define CSL_MSS_VIM_INTVECTOR_15                                               (0x0000203CU)
#define CSL_MSS_VIM_INTVECTOR_16                                               (0x00002040U)
#define CSL_MSS_VIM_INTVECTOR_17                                               (0x00002044U)
#define CSL_MSS_VIM_INTVECTOR_18                                               (0x00002048U)
#define CSL_MSS_VIM_INTVECTOR_19                                               (0x0000204CU)
#define CSL_MSS_VIM_INTVECTOR_20                                               (0x00002050U)
#define CSL_MSS_VIM_INTVECTOR_21                                               (0x00002054U)
#define CSL_MSS_VIM_INTVECTOR_22                                               (0x00002058U)
#define CSL_MSS_VIM_INTVECTOR_23                                               (0x0000205CU)
#define CSL_MSS_VIM_INTVECTOR_24                                               (0x00002060U)
#define CSL_MSS_VIM_INTVECTOR_25                                               (0x00002064U)
#define CSL_MSS_VIM_INTVECTOR_26                                               (0x00002068U)
#define CSL_MSS_VIM_INTVECTOR_27                                               (0x0000206CU)
#define CSL_MSS_VIM_INTVECTOR_28                                               (0x00002070U)
#define CSL_MSS_VIM_INTVECTOR_29                                               (0x00002074U)
#define CSL_MSS_VIM_INTVECTOR_30                                               (0x00002078U)
#define CSL_MSS_VIM_INTVECTOR_31                                               (0x0000207CU)
#define CSL_MSS_VIM_INTVECTOR_32                                               (0x00002080U)
#define CSL_MSS_VIM_INTVECTOR_33                                               (0x00002084U)
#define CSL_MSS_VIM_INTVECTOR_34                                               (0x00002088U)
#define CSL_MSS_VIM_INTVECTOR_35                                               (0x0000208CU)
#define CSL_MSS_VIM_INTVECTOR_36                                               (0x00002090U)
#define CSL_MSS_VIM_INTVECTOR_37                                               (0x00002094U)
#define CSL_MSS_VIM_INTVECTOR_38                                               (0x00002098U)
#define CSL_MSS_VIM_INTVECTOR_39                                               (0x0000209CU)
#define CSL_MSS_VIM_INTVECTOR_40                                               (0x000020A0U)
#define CSL_MSS_VIM_INTVECTOR_41                                               (0x000020A4U)
#define CSL_MSS_VIM_INTVECTOR_42                                               (0x000020A8U)
#define CSL_MSS_VIM_INTVECTOR_43                                               (0x000020ACU)
#define CSL_MSS_VIM_INTVECTOR_44                                               (0x000020B0U)
#define CSL_MSS_VIM_INTVECTOR_45                                               (0x000020B4U)
#define CSL_MSS_VIM_INTVECTOR_46                                               (0x000020B8U)
#define CSL_MSS_VIM_INTVECTOR_47                                               (0x000020BCU)
#define CSL_MSS_VIM_INTVECTOR_48                                               (0x000020C0U)
#define CSL_MSS_VIM_INTVECTOR_49                                               (0x000020C4U)
#define CSL_MSS_VIM_INTVECTOR_50                                               (0x000020C8U)
#define CSL_MSS_VIM_INTVECTOR_51                                               (0x000020CCU)
#define CSL_MSS_VIM_INTVECTOR_52                                               (0x000020D0U)
#define CSL_MSS_VIM_INTVECTOR_53                                               (0x000020D4U)
#define CSL_MSS_VIM_INTVECTOR_54                                               (0x000020D8U)
#define CSL_MSS_VIM_INTVECTOR_55                                               (0x000020DCU)
#define CSL_MSS_VIM_INTVECTOR_56                                               (0x000020E0U)
#define CSL_MSS_VIM_INTVECTOR_57                                               (0x000020E4U)
#define CSL_MSS_VIM_INTVECTOR_58                                               (0x000020E8U)
#define CSL_MSS_VIM_INTVECTOR_59                                               (0x000020ECU)
#define CSL_MSS_VIM_INTVECTOR_60                                               (0x000020F0U)
#define CSL_MSS_VIM_INTVECTOR_61                                               (0x000020F4U)
#define CSL_MSS_VIM_INTVECTOR_62                                               (0x000020F8U)
#define CSL_MSS_VIM_INTVECTOR_63                                               (0x000020FCU)
#define CSL_MSS_VIM_INTVECTOR_64                                               (0x00002100U)
#define CSL_MSS_VIM_INTVECTOR_65                                               (0x00002104U)
#define CSL_MSS_VIM_INTVECTOR_66                                               (0x00002108U)
#define CSL_MSS_VIM_INTVECTOR_67                                               (0x0000210CU)
#define CSL_MSS_VIM_INTVECTOR_68                                               (0x00002110U)
#define CSL_MSS_VIM_INTVECTOR_69                                               (0x00002114U)
#define CSL_MSS_VIM_INTVECTOR_70                                               (0x00002118U)
#define CSL_MSS_VIM_INTVECTOR_71                                               (0x0000211CU)
#define CSL_MSS_VIM_INTVECTOR_72                                               (0x00002120U)
#define CSL_MSS_VIM_INTVECTOR_73                                               (0x00002124U)
#define CSL_MSS_VIM_INTVECTOR_74                                               (0x00002128U)
#define CSL_MSS_VIM_INTVECTOR_75                                               (0x0000212CU)
#define CSL_MSS_VIM_INTVECTOR_76                                               (0x00002130U)
#define CSL_MSS_VIM_INTVECTOR_77                                               (0x00002134U)
#define CSL_MSS_VIM_INTVECTOR_78                                               (0x00002138U)
#define CSL_MSS_VIM_INTVECTOR_79                                               (0x0000213CU)
#define CSL_MSS_VIM_INTVECTOR_80                                               (0x00002140U)
#define CSL_MSS_VIM_INTVECTOR_81                                               (0x00002144U)
#define CSL_MSS_VIM_INTVECTOR_82                                               (0x00002148U)
#define CSL_MSS_VIM_INTVECTOR_83                                               (0x0000214CU)
#define CSL_MSS_VIM_INTVECTOR_84                                               (0x00002150U)
#define CSL_MSS_VIM_INTVECTOR_85                                               (0x00002154U)
#define CSL_MSS_VIM_INTVECTOR_86                                               (0x00002158U)
#define CSL_MSS_VIM_INTVECTOR_87                                               (0x0000215CU)
#define CSL_MSS_VIM_INTVECTOR_88                                               (0x00002160U)
#define CSL_MSS_VIM_INTVECTOR_89                                               (0x00002164U)
#define CSL_MSS_VIM_INTVECTOR_90                                               (0x00002168U)
#define CSL_MSS_VIM_INTVECTOR_91                                               (0x0000216CU)
#define CSL_MSS_VIM_INTVECTOR_92                                               (0x00002170U)
#define CSL_MSS_VIM_INTVECTOR_93                                               (0x00002174U)
#define CSL_MSS_VIM_INTVECTOR_94                                               (0x00002178U)
#define CSL_MSS_VIM_INTVECTOR_95                                               (0x0000217CU)
#define CSL_MSS_VIM_INTVECTOR_96                                               (0x00002180U)
#define CSL_MSS_VIM_INTVECTOR_97                                               (0x00002184U)
#define CSL_MSS_VIM_INTVECTOR_98                                               (0x00002188U)
#define CSL_MSS_VIM_INTVECTOR_99                                               (0x0000218CU)
#define CSL_MSS_VIM_INTVECTOR_100                                              (0x00002190U)
#define CSL_MSS_VIM_INTVECTOR_101                                              (0x00002194U)
#define CSL_MSS_VIM_INTVECTOR_102                                              (0x00002198U)
#define CSL_MSS_VIM_INTVECTOR_103                                              (0x0000219CU)
#define CSL_MSS_VIM_INTVECTOR_104                                              (0x000021A0U)
#define CSL_MSS_VIM_INTVECTOR_105                                              (0x000021A4U)
#define CSL_MSS_VIM_INTVECTOR_106                                              (0x000021A8U)
#define CSL_MSS_VIM_INTVECTOR_107                                              (0x000021ACU)
#define CSL_MSS_VIM_INTVECTOR_108                                              (0x000021B0U)
#define CSL_MSS_VIM_INTVECTOR_109                                              (0x000021B4U)
#define CSL_MSS_VIM_INTVECTOR_110                                              (0x000021B8U)
#define CSL_MSS_VIM_INTVECTOR_111                                              (0x000021BCU)
#define CSL_MSS_VIM_INTVECTOR_112                                              (0x000021C0U)
#define CSL_MSS_VIM_INTVECTOR_113                                              (0x000021C4U)
#define CSL_MSS_VIM_INTVECTOR_114                                              (0x000021C8U)
#define CSL_MSS_VIM_INTVECTOR_115                                              (0x000021CCU)
#define CSL_MSS_VIM_INTVECTOR_116                                              (0x000021D0U)
#define CSL_MSS_VIM_INTVECTOR_117                                              (0x000021D4U)
#define CSL_MSS_VIM_INTVECTOR_118                                              (0x000021D8U)
#define CSL_MSS_VIM_INTVECTOR_119                                              (0x000021DCU)
#define CSL_MSS_VIM_INTVECTOR_120                                              (0x000021E0U)
#define CSL_MSS_VIM_INTVECTOR_121                                              (0x000021E4U)
#define CSL_MSS_VIM_INTVECTOR_122                                              (0x000021E8U)
#define CSL_MSS_VIM_INTVECTOR_123                                              (0x000021ECU)
#define CSL_MSS_VIM_INTVECTOR_124                                              (0x000021F0U)
#define CSL_MSS_VIM_INTVECTOR_125                                              (0x000021F4U)
#define CSL_MSS_VIM_INTVECTOR_126                                              (0x000021F8U)
#define CSL_MSS_VIM_INTVECTOR_127                                              (0x000021FCU)
#define CSL_MSS_VIM_INTVECTOR_128                                              (0x00002200U)
#define CSL_MSS_VIM_INTVECTOR_129                                              (0x00002204U)
#define CSL_MSS_VIM_INTVECTOR_130                                              (0x00002208U)
#define CSL_MSS_VIM_INTVECTOR_131                                              (0x0000220CU)
#define CSL_MSS_VIM_INTVECTOR_132                                              (0x00002210U)
#define CSL_MSS_VIM_INTVECTOR_133                                              (0x00002214U)
#define CSL_MSS_VIM_INTVECTOR_134                                              (0x00002218U)
#define CSL_MSS_VIM_INTVECTOR_135                                              (0x0000221CU)
#define CSL_MSS_VIM_INTVECTOR_136                                              (0x00002220U)
#define CSL_MSS_VIM_INTVECTOR_137                                              (0x00002224U)
#define CSL_MSS_VIM_INTVECTOR_138                                              (0x00002228U)
#define CSL_MSS_VIM_INTVECTOR_139                                              (0x0000222CU)
#define CSL_MSS_VIM_INTVECTOR_140                                              (0x00002230U)
#define CSL_MSS_VIM_INTVECTOR_141                                              (0x00002234U)
#define CSL_MSS_VIM_INTVECTOR_142                                              (0x00002238U)
#define CSL_MSS_VIM_INTVECTOR_143                                              (0x0000223CU)
#define CSL_MSS_VIM_INTVECTOR_144                                              (0x00002240U)
#define CSL_MSS_VIM_INTVECTOR_145                                              (0x00002244U)
#define CSL_MSS_VIM_INTVECTOR_146                                              (0x00002248U)
#define CSL_MSS_VIM_INTVECTOR_147                                              (0x0000224CU)
#define CSL_MSS_VIM_INTVECTOR_148                                              (0x00002250U)
#define CSL_MSS_VIM_INTVECTOR_149                                              (0x00002254U)
#define CSL_MSS_VIM_INTVECTOR_150                                              (0x00002258U)
#define CSL_MSS_VIM_INTVECTOR_151                                              (0x0000225CU)
#define CSL_MSS_VIM_INTVECTOR_152                                              (0x00002260U)
#define CSL_MSS_VIM_INTVECTOR_153                                              (0x00002264U)
#define CSL_MSS_VIM_INTVECTOR_154                                              (0x00002268U)
#define CSL_MSS_VIM_INTVECTOR_155                                              (0x0000226CU)
#define CSL_MSS_VIM_INTVECTOR_156                                              (0x00002270U)
#define CSL_MSS_VIM_INTVECTOR_157                                              (0x00002274U)
#define CSL_MSS_VIM_INTVECTOR_158                                              (0x00002278U)
#define CSL_MSS_VIM_INTVECTOR_159                                              (0x0000227CU)
#define CSL_MSS_VIM_INTVECTOR_160                                              (0x00002280U)
#define CSL_MSS_VIM_INTVECTOR_161                                              (0x00002284U)
#define CSL_MSS_VIM_INTVECTOR_162                                              (0x00002288U)
#define CSL_MSS_VIM_INTVECTOR_163                                              (0x0000228CU)
#define CSL_MSS_VIM_INTVECTOR_164                                              (0x00002290U)
#define CSL_MSS_VIM_INTVECTOR_165                                              (0x00002294U)
#define CSL_MSS_VIM_INTVECTOR_166                                              (0x00002298U)
#define CSL_MSS_VIM_INTVECTOR_167                                              (0x0000229CU)
#define CSL_MSS_VIM_INTVECTOR_168                                              (0x000022A0U)
#define CSL_MSS_VIM_INTVECTOR_169                                              (0x000022A4U)
#define CSL_MSS_VIM_INTVECTOR_170                                              (0x000022A8U)
#define CSL_MSS_VIM_INTVECTOR_171                                              (0x000022ACU)
#define CSL_MSS_VIM_INTVECTOR_172                                              (0x000022B0U)
#define CSL_MSS_VIM_INTVECTOR_173                                              (0x000022B4U)
#define CSL_MSS_VIM_INTVECTOR_174                                              (0x000022B8U)
#define CSL_MSS_VIM_INTVECTOR_175                                              (0x000022BCU)
#define CSL_MSS_VIM_INTVECTOR_176                                              (0x000022C0U)
#define CSL_MSS_VIM_INTVECTOR_177                                              (0x000022C4U)
#define CSL_MSS_VIM_INTVECTOR_178                                              (0x000022C8U)
#define CSL_MSS_VIM_INTVECTOR_179                                              (0x000022CCU)
#define CSL_MSS_VIM_INTVECTOR_180                                              (0x000022D0U)
#define CSL_MSS_VIM_INTVECTOR_181                                              (0x000022D4U)
#define CSL_MSS_VIM_INTVECTOR_182                                              (0x000022D8U)
#define CSL_MSS_VIM_INTVECTOR_183                                              (0x000022DCU)
#define CSL_MSS_VIM_INTVECTOR_184                                              (0x000022E0U)
#define CSL_MSS_VIM_INTVECTOR_185                                              (0x000022E4U)
#define CSL_MSS_VIM_INTVECTOR_186                                              (0x000022E8U)
#define CSL_MSS_VIM_INTVECTOR_187                                              (0x000022ECU)
#define CSL_MSS_VIM_INTVECTOR_188                                              (0x000022F0U)
#define CSL_MSS_VIM_INTVECTOR_189                                              (0x000022F4U)
#define CSL_MSS_VIM_INTVECTOR_190                                              (0x000022F8U)
#define CSL_MSS_VIM_INTVECTOR_191                                              (0x000022FCU)
#define CSL_MSS_VIM_INTVECTOR_192                                              (0x00002300U)
#define CSL_MSS_VIM_INTVECTOR_193                                              (0x00002304U)
#define CSL_MSS_VIM_INTVECTOR_194                                              (0x00002308U)
#define CSL_MSS_VIM_INTVECTOR_195                                              (0x0000230CU)
#define CSL_MSS_VIM_INTVECTOR_196                                              (0x00002310U)
#define CSL_MSS_VIM_INTVECTOR_197                                              (0x00002314U)
#define CSL_MSS_VIM_INTVECTOR_198                                              (0x00002318U)
#define CSL_MSS_VIM_INTVECTOR_199                                              (0x0000231CU)
#define CSL_MSS_VIM_INTVECTOR_200                                              (0x00002320U)
#define CSL_MSS_VIM_INTVECTOR_201                                              (0x00002324U)
#define CSL_MSS_VIM_INTVECTOR_202                                              (0x00002328U)
#define CSL_MSS_VIM_INTVECTOR_203                                              (0x0000232CU)
#define CSL_MSS_VIM_INTVECTOR_204                                              (0x00002330U)
#define CSL_MSS_VIM_INTVECTOR_205                                              (0x00002334U)
#define CSL_MSS_VIM_INTVECTOR_206                                              (0x00002338U)
#define CSL_MSS_VIM_INTVECTOR_207                                              (0x0000233CU)
#define CSL_MSS_VIM_INTVECTOR_208                                              (0x00002340U)
#define CSL_MSS_VIM_INTVECTOR_209                                              (0x00002344U)
#define CSL_MSS_VIM_INTVECTOR_210                                              (0x00002348U)
#define CSL_MSS_VIM_INTVECTOR_211                                              (0x0000234CU)
#define CSL_MSS_VIM_INTVECTOR_212                                              (0x00002350U)
#define CSL_MSS_VIM_INTVECTOR_213                                              (0x00002354U)
#define CSL_MSS_VIM_INTVECTOR_214                                              (0x00002358U)
#define CSL_MSS_VIM_INTVECTOR_215                                              (0x0000235CU)
#define CSL_MSS_VIM_INTVECTOR_216                                              (0x00002360U)
#define CSL_MSS_VIM_INTVECTOR_217                                              (0x00002364U)
#define CSL_MSS_VIM_INTVECTOR_218                                              (0x00002368U)
#define CSL_MSS_VIM_INTVECTOR_219                                              (0x0000236CU)
#define CSL_MSS_VIM_INTVECTOR_220                                              (0x00002370U)
#define CSL_MSS_VIM_INTVECTOR_221                                              (0x00002374U)
#define CSL_MSS_VIM_INTVECTOR_222                                              (0x00002378U)
#define CSL_MSS_VIM_INTVECTOR_223                                              (0x0000237CU)
#define CSL_MSS_VIM_INTVECTOR_224                                              (0x00002380U)
#define CSL_MSS_VIM_INTVECTOR_225                                              (0x00002384U)
#define CSL_MSS_VIM_INTVECTOR_226                                              (0x00002388U)
#define CSL_MSS_VIM_INTVECTOR_227                                              (0x0000238CU)
#define CSL_MSS_VIM_INTVECTOR_228                                              (0x00002390U)
#define CSL_MSS_VIM_INTVECTOR_229                                              (0x00002394U)
#define CSL_MSS_VIM_INTVECTOR_230                                              (0x00002398U)
#define CSL_MSS_VIM_INTVECTOR_231                                              (0x0000239CU)
#define CSL_MSS_VIM_INTVECTOR_232                                              (0x000023A0U)
#define CSL_MSS_VIM_INTVECTOR_233                                              (0x000023A4U)
#define CSL_MSS_VIM_INTVECTOR_234                                              (0x000023A8U)
#define CSL_MSS_VIM_INTVECTOR_235                                              (0x000023ACU)
#define CSL_MSS_VIM_INTVECTOR_236                                              (0x000023B0U)
#define CSL_MSS_VIM_INTVECTOR_237                                              (0x000023B4U)
#define CSL_MSS_VIM_INTVECTOR_238                                              (0x000023B8U)
#define CSL_MSS_VIM_INTVECTOR_239                                              (0x000023BCU)
#define CSL_MSS_VIM_INTVECTOR_240                                              (0x000023C0U)
#define CSL_MSS_VIM_INTVECTOR_241                                              (0x000023C4U)
#define CSL_MSS_VIM_INTVECTOR_242                                              (0x000023C8U)
#define CSL_MSS_VIM_INTVECTOR_243                                              (0x000023CCU)
#define CSL_MSS_VIM_INTVECTOR_244                                              (0x000023D0U)
#define CSL_MSS_VIM_INTVECTOR_245                                              (0x000023D4U)
#define CSL_MSS_VIM_INTVECTOR_246                                              (0x000023D8U)
#define CSL_MSS_VIM_INTVECTOR_247                                              (0x000023DCU)
#define CSL_MSS_VIM_INTVECTOR_248                                              (0x000023E0U)
#define CSL_MSS_VIM_INTVECTOR_249                                              (0x000023E4U)
#define CSL_MSS_VIM_INTVECTOR_250                                              (0x000023E8U)
#define CSL_MSS_VIM_INTVECTOR_251                                              (0x000023ECU)
#define CSL_MSS_VIM_INTVECTOR_252                                              (0x000023F0U)
#define CSL_MSS_VIM_INTVECTOR_253                                              (0x000023F4U)
#define CSL_MSS_VIM_INTVECTOR_254                                              (0x000023F8U)
#define CSL_MSS_VIM_INTVECTOR_255                                              (0x000023FCU)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* PID */

#define CSL_MSS_VIM_PID_SCHEME_MASK                                            (0xC0000000U)
#define CSL_MSS_VIM_PID_SCHEME_SHIFT                                           (0x0000001EU)
#define CSL_MSS_VIM_PID_SCHEME_RESETVAL                                        (0x00000001U)
#define CSL_MSS_VIM_PID_SCHEME_MAX                                             (0x00000003U)

#define CSL_MSS_VIM_PID_BU_MASK                                                (0x30000000U)
#define CSL_MSS_VIM_PID_BU_SHIFT                                               (0x0000001CU)
#define CSL_MSS_VIM_PID_BU_RESETVAL                                            (0x00000002U)
#define CSL_MSS_VIM_PID_BU_MAX                                                 (0x00000003U)

#define CSL_MSS_VIM_PID_FUNC_MASK                                              (0x0FFF0000U)
#define CSL_MSS_VIM_PID_FUNC_SHIFT                                             (0x00000010U)
#define CSL_MSS_VIM_PID_FUNC_RESETVAL                                          (0x00000090U)
#define CSL_MSS_VIM_PID_FUNC_MAX                                               (0x00000FFFU)

#define CSL_MSS_VIM_PID_RTL_MASK                                               (0x0000F800U)
#define CSL_MSS_VIM_PID_RTL_SHIFT                                              (0x0000000BU)
#define CSL_MSS_VIM_PID_RTL_RESETVAL                                           (0x00000000U)
#define CSL_MSS_VIM_PID_RTL_MAX                                                (0x0000001FU)

#define CSL_MSS_VIM_PID_MAJOR_MASK                                             (0x00000700U)
#define CSL_MSS_VIM_PID_MAJOR_SHIFT                                            (0x00000008U)
#define CSL_MSS_VIM_PID_MAJOR_RESETVAL                                         (0x00000000U)
#define CSL_MSS_VIM_PID_MAJOR_MAX                                              (0x00000007U)

#define CSL_MSS_VIM_PID_CUSTOM_MASK                                            (0x000000C0U)
#define CSL_MSS_VIM_PID_CUSTOM_SHIFT                                           (0x00000006U)
#define CSL_MSS_VIM_PID_CUSTOM_RESETVAL                                        (0x00000000U)
#define CSL_MSS_VIM_PID_CUSTOM_MAX                                             (0x00000003U)

#define CSL_MSS_VIM_PID_MINOR_MASK                                             (0x0000003FU)
#define CSL_MSS_VIM_PID_MINOR_SHIFT                                            (0x00000000U)
#define CSL_MSS_VIM_PID_MINOR_RESETVAL                                         (0x00000001U)
#define CSL_MSS_VIM_PID_MINOR_MAX                                              (0x0000003FU)

#define CSL_MSS_VIM_PID_RESETVAL                                               (0x60900001U)

/* INFO */

#define CSL_MSS_VIM_INFO_RES1_MASK                                             (0xFFFFF800U)
#define CSL_MSS_VIM_INFO_RES1_SHIFT                                            (0x0000000BU)
#define CSL_MSS_VIM_INFO_RES1_RESETVAL                                         (0x00000000U)
#define CSL_MSS_VIM_INFO_RES1_MAX                                              (0x001FFFFFU)

#define CSL_MSS_VIM_INFO_INTERRUPTS_MASK                                       (0x000007FFU)
#define CSL_MSS_VIM_INFO_INTERRUPTS_SHIFT                                      (0x00000000U)
#define CSL_MSS_VIM_INFO_INTERRUPTS_RESETVAL                                   (0x00000100U)
#define CSL_MSS_VIM_INFO_INTERRUPTS_MAX                                        (0x000007FFU)

#define CSL_MSS_VIM_INFO_RESETVAL                                              (0x00000100U)

/* PRIIRQ */

#define CSL_MSS_VIM_PRIIRQ_VALID_MASK                                          (0x80000000U)
#define CSL_MSS_VIM_PRIIRQ_VALID_SHIFT                                         (0x0000001FU)
#define CSL_MSS_VIM_PRIIRQ_VALID_RESETVAL                                      (0x00000000U)
#define CSL_MSS_VIM_PRIIRQ_VALID_MAX                                           (0x00000001U)

#define CSL_MSS_VIM_PRIIRQ_RES2_MASK                                           (0x7FF00000U)
#define CSL_MSS_VIM_PRIIRQ_RES2_SHIFT                                          (0x00000014U)
#define CSL_MSS_VIM_PRIIRQ_RES2_RESETVAL                                       (0x00000000U)
#define CSL_MSS_VIM_PRIIRQ_RES2_MAX                                            (0x000007FFU)

#define CSL_MSS_VIM_PRIIRQ_PRI_MASK                                            (0x000F0000U)
#define CSL_MSS_VIM_PRIIRQ_PRI_SHIFT                                           (0x00000010U)
#define CSL_MSS_VIM_PRIIRQ_PRI_RESETVAL                                        (0x00000000U)
#define CSL_MSS_VIM_PRIIRQ_PRI_MAX                                             (0x0000000FU)

#define CSL_MSS_VIM_PRIIRQ_RES3_MASK                                           (0x0000FC00U)
#define CSL_MSS_VIM_PRIIRQ_RES3_SHIFT                                          (0x0000000AU)
#define CSL_MSS_VIM_PRIIRQ_RES3_RESETVAL                                       (0x00000000U)
#define CSL_MSS_VIM_PRIIRQ_RES3_MAX                                            (0x0000003FU)

#define CSL_MSS_VIM_PRIIRQ_NUM_MASK                                            (0x000003FFU)
#define CSL_MSS_VIM_PRIIRQ_NUM_SHIFT                                           (0x00000000U)
#define CSL_MSS_VIM_PRIIRQ_NUM_RESETVAL                                        (0x00000000U)
#define CSL_MSS_VIM_PRIIRQ_NUM_MAX                                             (0x000003FFU)

#define CSL_MSS_VIM_PRIIRQ_RESETVAL                                            (0x00000000U)

/* PRIFIQ */

#define CSL_MSS_VIM_PRIFIQ_VALID_MASK                                          (0x80000000U)
#define CSL_MSS_VIM_PRIFIQ_VALID_SHIFT                                         (0x0000001FU)
#define CSL_MSS_VIM_PRIFIQ_VALID_RESETVAL                                      (0x00000000U)
#define CSL_MSS_VIM_PRIFIQ_VALID_MAX                                           (0x00000001U)

#define CSL_MSS_VIM_PRIFIQ_RES4_MASK                                           (0x7FF00000U)
#define CSL_MSS_VIM_PRIFIQ_RES4_SHIFT                                          (0x00000014U)
#define CSL_MSS_VIM_PRIFIQ_RES4_RESETVAL                                       (0x00000000U)
#define CSL_MSS_VIM_PRIFIQ_RES4_MAX                                            (0x000007FFU)

#define CSL_MSS_VIM_PRIFIQ_PRI_MASK                                            (0x000F0000U)
#define CSL_MSS_VIM_PRIFIQ_PRI_SHIFT                                           (0x00000010U)
#define CSL_MSS_VIM_PRIFIQ_PRI_RESETVAL                                        (0x00000000U)
#define CSL_MSS_VIM_PRIFIQ_PRI_MAX                                             (0x0000000FU)

#define CSL_MSS_VIM_PRIFIQ_RES5_MASK                                           (0x0000FC00U)
#define CSL_MSS_VIM_PRIFIQ_RES5_SHIFT                                          (0x0000000AU)
#define CSL_MSS_VIM_PRIFIQ_RES5_RESETVAL                                       (0x00000000U)
#define CSL_MSS_VIM_PRIFIQ_RES5_MAX                                            (0x0000003FU)

#define CSL_MSS_VIM_PRIFIQ_NUM_MASK                                            (0x000003FFU)
#define CSL_MSS_VIM_PRIFIQ_NUM_SHIFT                                           (0x00000000U)
#define CSL_MSS_VIM_PRIFIQ_NUM_RESETVAL                                        (0x00000000U)
#define CSL_MSS_VIM_PRIFIQ_NUM_MAX                                             (0x000003FFU)

#define CSL_MSS_VIM_PRIFIQ_RESETVAL                                            (0x00000000U)

/* IRQGSTS */

#define CSL_MSS_VIM_IRQGSTS_STS_MASK                                           (0xFFFFFFFFU)
#define CSL_MSS_VIM_IRQGSTS_STS_SHIFT                                          (0x00000000U)
#define CSL_MSS_VIM_IRQGSTS_STS_RESETVAL                                       (0x00000000U)
#define CSL_MSS_VIM_IRQGSTS_STS_MAX                                            (0xFFFFFFFFU)

#define CSL_MSS_VIM_IRQGSTS_RESETVAL                                           (0x00000000U)

/* FIQGSTS */

#define CSL_MSS_VIM_FIQGSTS_STS_MASK                                           (0xFFFFFFFFU)
#define CSL_MSS_VIM_FIQGSTS_STS_SHIFT                                          (0x00000000U)
#define CSL_MSS_VIM_FIQGSTS_STS_RESETVAL                                       (0x00000000U)
#define CSL_MSS_VIM_FIQGSTS_STS_MAX                                            (0xFFFFFFFFU)

#define CSL_MSS_VIM_FIQGSTS_RESETVAL                                           (0x00000000U)

/* IRQVEC */

#define CSL_MSS_VIM_IRQVEC_RES21_MASK                                          (0x00000003U)
#define CSL_MSS_VIM_IRQVEC_RES21_SHIFT                                         (0x00000000U)
#define CSL_MSS_VIM_IRQVEC_RES21_RESETVAL                                      (0x00000000U)
#define CSL_MSS_VIM_IRQVEC_RES21_MAX                                           (0x00000003U)

#define CSL_MSS_VIM_IRQVEC_ADDR_MASK                                           (0xFFFFFFFCU)
#define CSL_MSS_VIM_IRQVEC_ADDR_SHIFT                                          (0x00000002U)
#define CSL_MSS_VIM_IRQVEC_ADDR_RESETVAL                                       (0x00000000U)
#define CSL_MSS_VIM_IRQVEC_ADDR_MAX                                            (0x3FFFFFFFU)

#define CSL_MSS_VIM_IRQVEC_RESETVAL                                            (0x00000000U)

/* FIQVEC */

#define CSL_MSS_VIM_FIQVEC_RES22_MASK                                          (0x00000003U)
#define CSL_MSS_VIM_FIQVEC_RES22_SHIFT                                         (0x00000000U)
#define CSL_MSS_VIM_FIQVEC_RES22_RESETVAL                                      (0x00000000U)
#define CSL_MSS_VIM_FIQVEC_RES22_MAX                                           (0x00000003U)

#define CSL_MSS_VIM_FIQVEC_ADDR_MASK                                           (0xFFFFFFFCU)
#define CSL_MSS_VIM_FIQVEC_ADDR_SHIFT                                          (0x00000002U)
#define CSL_MSS_VIM_FIQVEC_ADDR_RESETVAL                                       (0x00000000U)
#define CSL_MSS_VIM_FIQVEC_ADDR_MAX                                            (0x3FFFFFFFU)

#define CSL_MSS_VIM_FIQVEC_RESETVAL                                            (0x00000000U)

/* ACTIRQ */

#define CSL_MSS_VIM_ACTIRQ_VALID_MASK                                          (0x80000000U)
#define CSL_MSS_VIM_ACTIRQ_VALID_SHIFT                                         (0x0000001FU)
#define CSL_MSS_VIM_ACTIRQ_VALID_RESETVAL                                      (0x00000000U)
#define CSL_MSS_VIM_ACTIRQ_VALID_MAX                                           (0x00000001U)

#define CSL_MSS_VIM_ACTIRQ_RES6_MASK                                           (0x7FF00000U)
#define CSL_MSS_VIM_ACTIRQ_RES6_SHIFT                                          (0x00000014U)
#define CSL_MSS_VIM_ACTIRQ_RES6_RESETVAL                                       (0x00000000U)
#define CSL_MSS_VIM_ACTIRQ_RES6_MAX                                            (0x000007FFU)

#define CSL_MSS_VIM_ACTIRQ_PRI_MASK                                            (0x000F0000U)
#define CSL_MSS_VIM_ACTIRQ_PRI_SHIFT                                           (0x00000010U)
#define CSL_MSS_VIM_ACTIRQ_PRI_RESETVAL                                        (0x00000000U)
#define CSL_MSS_VIM_ACTIRQ_PRI_MAX                                             (0x0000000FU)

#define CSL_MSS_VIM_ACTIRQ_RES7_MASK                                           (0x0000FC00U)
#define CSL_MSS_VIM_ACTIRQ_RES7_SHIFT                                          (0x0000000AU)
#define CSL_MSS_VIM_ACTIRQ_RES7_RESETVAL                                       (0x00000000U)
#define CSL_MSS_VIM_ACTIRQ_RES7_MAX                                            (0x0000003FU)

#define CSL_MSS_VIM_ACTIRQ_NUM_MASK                                            (0x000003FFU)
#define CSL_MSS_VIM_ACTIRQ_NUM_SHIFT                                           (0x00000000U)
#define CSL_MSS_VIM_ACTIRQ_NUM_RESETVAL                                        (0x00000000U)
#define CSL_MSS_VIM_ACTIRQ_NUM_MAX                                             (0x000003FFU)

#define CSL_MSS_VIM_ACTIRQ_RESETVAL                                            (0x00000000U)

/* ACTFIQ */

#define CSL_MSS_VIM_ACTFIQ_VALID_MASK                                          (0x80000000U)
#define CSL_MSS_VIM_ACTFIQ_VALID_SHIFT                                         (0x0000001FU)
#define CSL_MSS_VIM_ACTFIQ_VALID_RESETVAL                                      (0x00000000U)
#define CSL_MSS_VIM_ACTFIQ_VALID_MAX                                           (0x00000001U)

#define CSL_MSS_VIM_ACTFIQ_RES8_MASK                                           (0x7FF00000U)
#define CSL_MSS_VIM_ACTFIQ_RES8_SHIFT                                          (0x00000014U)
#define CSL_MSS_VIM_ACTFIQ_RES8_RESETVAL                                       (0x00000000U)
#define CSL_MSS_VIM_ACTFIQ_RES8_MAX                                            (0x000007FFU)

#define CSL_MSS_VIM_ACTFIQ_PRI_MASK                                            (0x000F0000U)
#define CSL_MSS_VIM_ACTFIQ_PRI_SHIFT                                           (0x00000010U)
#define CSL_MSS_VIM_ACTFIQ_PRI_RESETVAL                                        (0x00000000U)
#define CSL_MSS_VIM_ACTFIQ_PRI_MAX                                             (0x0000000FU)

#define CSL_MSS_VIM_ACTFIQ_RES9_MASK                                           (0x0000FC00U)
#define CSL_MSS_VIM_ACTFIQ_RES9_SHIFT                                          (0x0000000AU)
#define CSL_MSS_VIM_ACTFIQ_RES9_RESETVAL                                       (0x00000000U)
#define CSL_MSS_VIM_ACTFIQ_RES9_MAX                                            (0x0000003FU)

#define CSL_MSS_VIM_ACTFIQ_NUM_MASK                                            (0x000003FFU)
#define CSL_MSS_VIM_ACTFIQ_NUM_SHIFT                                           (0x00000000U)
#define CSL_MSS_VIM_ACTFIQ_NUM_RESETVAL                                        (0x00000000U)
#define CSL_MSS_VIM_ACTFIQ_NUM_MAX                                             (0x000003FFU)

#define CSL_MSS_VIM_ACTFIQ_RESETVAL                                            (0x00000000U)

/* IRQPRIMSK */

#define CSL_MSS_VIM_IRQPRIMSK_MSK_MASK                                         (0x0000FFFFU)
#define CSL_MSS_VIM_IRQPRIMSK_MSK_SHIFT                                        (0x00000000U)
#define CSL_MSS_VIM_IRQPRIMSK_MSK_RESETVAL                                     (0x0000FFFFU)
#define CSL_MSS_VIM_IRQPRIMSK_MSK_MAX                                          (0x0000FFFFU)


#define CSL_MSS_VIM_IRQPRIMSK_RES24_MASK                                       (0xFFFF0000U)
#define CSL_MSS_VIM_IRQPRIMSK_RES24_SHIFT                                      (0x00000010U)
#define CSL_MSS_VIM_IRQPRIMSK_RES24_RESETVAL                                   (0x00000000U)
#define CSL_MSS_VIM_IRQPRIMSK_RES24_MAX                                        (0x0000FFFFU)

/* FIQPRIMSK */ 

#define CSL_MSS_VIM_FIQPRIMSK_MSK_MASK                                         (0x0000FFFFU)
#define CSL_MSS_VIM_FIQPRIMSK_MSK_SHIFT                                        (0x00000000U)
#define CSL_MSS_VIM_FIQPRIMSK_MSK_RESETVAL                                     (0x0000FFFFU)
#define CSL_MSS_VIM_FIQPRIMSK_MSK_MAX                                          (0x0000FFFFU)

#define CSL_MSS_VIM_FIQPRIMSK_RES24_MASK                                       (0xFFFF0000U)
#define CSL_MSS_VIM_FIQPRIMSK_RES24_SHIFT                                      (0x00000010U)
#define CSL_MSS_VIM_FIQPRIMSK_RES24_RESETVAL                                   (0x00000000U)
#define CSL_MSS_VIM_FIQPRIMSK_RES24_MAX                                        (0x0000FFFFU)

/* DEDVEC */

#define CSL_MSS_VIM_DEDVEC_RES23_MASK                                          (0x00000003U)
#define CSL_MSS_VIM_DEDVEC_RES23_SHIFT                                         (0x00000000U)
#define CSL_MSS_VIM_DEDVEC_RES23_RESETVAL                                      (0x00000000U)
#define CSL_MSS_VIM_DEDVEC_RES23_MAX                                           (0x00000003U)

#define CSL_MSS_VIM_DEDVEC_ADDR_MASK                                           (0xFFFFFFFCU)
#define CSL_MSS_VIM_DEDVEC_ADDR_SHIFT                                          (0x00000002U)
#define CSL_MSS_VIM_DEDVEC_ADDR_RESETVAL                                       (0x00000000U)
#define CSL_MSS_VIM_DEDVEC_ADDR_MAX                                            (0x3FFFFFFFU)

#define CSL_MSS_VIM_DEDVEC_RESETVAL                                            (0x00000000U)

/* RAW */

#define CSL_MSS_VIM_RAW_STS_MASK                                               (0xFFFFFFFFU)
#define CSL_MSS_VIM_RAW_STS_SHIFT                                              (0x00000000U)
#define CSL_MSS_VIM_RAW_STS_RESETVAL                                           (0x00000000U)
#define CSL_MSS_VIM_RAW_STS_MAX                                                (0xFFFFFFFFU)

#define CSL_MSS_VIM_RAW_RESETVAL                                               (0x00000000U)

/* STS */

#define CSL_MSS_VIM_STS_MASK_MASK                                              (0xFFFFFFFFU)
#define CSL_MSS_VIM_STS_MASK_SHIFT                                             (0x00000000U)
#define CSL_MSS_VIM_STS_MASK_RESETVAL                                          (0x00000000U)
#define CSL_MSS_VIM_STS_MASK_MAX                                               (0xFFFFFFFFU)

#define CSL_MSS_VIM_STS_RESETVAL                                               (0x00000000U)

/* INTR_EN_SET */

#define CSL_MSS_VIM_INTR_EN_SET_MASK_MASK                                      (0xFFFFFFFFU)
#define CSL_MSS_VIM_INTR_EN_SET_MASK_SHIFT                                     (0x00000000U)
#define CSL_MSS_VIM_INTR_EN_SET_MASK_RESETVAL                                  (0x00000000U)
#define CSL_MSS_VIM_INTR_EN_SET_MASK_MAX                                       (0xFFFFFFFFU)

#define CSL_MSS_VIM_INTR_EN_SET_RESETVAL                                       (0x00000000U)

/* INTER_EN_CLR */

#define CSL_MSS_VIM_INTER_EN_CLR_MASK_MASK                                     (0xFFFFFFFFU)
#define CSL_MSS_VIM_INTER_EN_CLR_MASK_SHIFT                                    (0x00000000U)
#define CSL_MSS_VIM_INTER_EN_CLR_MASK_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTER_EN_CLR_MASK_MAX                                      (0xFFFFFFFFU)

#define CSL_MSS_VIM_INTER_EN_CLR_RESETVAL                                      (0x00000000U)

/* IRQSTS */

#define CSL_MSS_VIM_IRQSTS_MASK_MASK                                           (0xFFFFFFFFU)
#define CSL_MSS_VIM_IRQSTS_MASK_SHIFT                                          (0x00000000U)
#define CSL_MSS_VIM_IRQSTS_MASK_RESETVAL                                       (0x00000000U)
#define CSL_MSS_VIM_IRQSTS_MASK_MAX                                            (0xFFFFFFFFU)

#define CSL_MSS_VIM_IRQSTS_RESETVAL                                            (0x00000000U)

/* FIQSTS */

#define CSL_MSS_VIM_FIQSTS_MASK_MASK                                           (0xFFFFFFFFU)
#define CSL_MSS_VIM_FIQSTS_MASK_SHIFT                                          (0x00000000U)
#define CSL_MSS_VIM_FIQSTS_MASK_RESETVAL                                       (0x00000000U)
#define CSL_MSS_VIM_FIQSTS_MASK_MAX                                            (0xFFFFFFFFU)

#define CSL_MSS_VIM_FIQSTS_RESETVAL                                            (0x00000000U)

/* INTMAP */

#define CSL_MSS_VIM_INTMAP_MASK_MASK                                           (0xFFFFFFFFU)
#define CSL_MSS_VIM_INTMAP_MASK_SHIFT                                          (0x00000000U)
#define CSL_MSS_VIM_INTMAP_MASK_RESETVAL                                       (0x00000000U)
#define CSL_MSS_VIM_INTMAP_MASK_MAX                                            (0xFFFFFFFFU)

#define CSL_MSS_VIM_INTMAP_RESETVAL                                            (0x00000000U)

/* INTTYPE */

#define CSL_MSS_VIM_INTTYPE_VAL_MASK                                           (0xFFFFFFFFU)
#define CSL_MSS_VIM_INTTYPE_VAL_SHIFT                                          (0x00000000U)
#define CSL_MSS_VIM_INTTYPE_VAL_RESETVAL                                       (0x00000000U)
#define CSL_MSS_VIM_INTTYPE_VAL_MAX                                            (0xFFFFFFFFU)

#define CSL_MSS_VIM_INTTYPE_RESETVAL                                           (0x00000000U)

/* RAW_1 */

#define CSL_MSS_VIM_RAW_1_STS_MASK                                             (0xFFFFFFFFU)
#define CSL_MSS_VIM_RAW_1_STS_SHIFT                                            (0x00000000U)
#define CSL_MSS_VIM_RAW_1_STS_RESETVAL                                         (0x00000000U)
#define CSL_MSS_VIM_RAW_1_STS_MAX                                              (0xFFFFFFFFU)

#define CSL_MSS_VIM_RAW_1_RESETVAL                                             (0x00000000U)

/* STS_1 */

#define CSL_MSS_VIM_STS_1_MASK_MASK                                            (0xFFFFFFFFU)
#define CSL_MSS_VIM_STS_1_MASK_SHIFT                                           (0x00000000U)
#define CSL_MSS_VIM_STS_1_MASK_RESETVAL                                        (0x00000000U)
#define CSL_MSS_VIM_STS_1_MASK_MAX                                             (0xFFFFFFFFU)

#define CSL_MSS_VIM_STS_1_RESETVAL                                             (0x00000000U)

/* INTR_EN_SET_1 */

#define CSL_MSS_VIM_INTR_EN_SET_1_MASK_MASK                                    (0xFFFFFFFFU)
#define CSL_MSS_VIM_INTR_EN_SET_1_MASK_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTR_EN_SET_1_MASK_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTR_EN_SET_1_MASK_MAX                                     (0xFFFFFFFFU)

#define CSL_MSS_VIM_INTR_EN_SET_1_RESETVAL                                     (0x00000000U)

/* INTER_EN_CLR_1 */

#define CSL_MSS_VIM_INTER_EN_CLR_1_MASK_MASK                                   (0xFFFFFFFFU)
#define CSL_MSS_VIM_INTER_EN_CLR_1_MASK_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTER_EN_CLR_1_MASK_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTER_EN_CLR_1_MASK_MAX                                    (0xFFFFFFFFU)

#define CSL_MSS_VIM_INTER_EN_CLR_1_RESETVAL                                    (0x00000000U)

/* IRQSTS_1 */

#define CSL_MSS_VIM_IRQSTS_1_MASK_MASK                                         (0xFFFFFFFFU)
#define CSL_MSS_VIM_IRQSTS_1_MASK_SHIFT                                        (0x00000000U)
#define CSL_MSS_VIM_IRQSTS_1_MASK_RESETVAL                                     (0x00000000U)
#define CSL_MSS_VIM_IRQSTS_1_MASK_MAX                                          (0xFFFFFFFFU)

#define CSL_MSS_VIM_IRQSTS_1_RESETVAL                                          (0x00000000U)

/* FIQSTS_1 */

#define CSL_MSS_VIM_FIQSTS_1_MASK_MASK                                         (0xFFFFFFFFU)
#define CSL_MSS_VIM_FIQSTS_1_MASK_SHIFT                                        (0x00000000U)
#define CSL_MSS_VIM_FIQSTS_1_MASK_RESETVAL                                     (0x00000000U)
#define CSL_MSS_VIM_FIQSTS_1_MASK_MAX                                          (0xFFFFFFFFU)

#define CSL_MSS_VIM_FIQSTS_1_RESETVAL                                          (0x00000000U)

/* INTMAP_1 */

#define CSL_MSS_VIM_INTMAP_1_MASK_MASK                                         (0xFFFFFFFFU)
#define CSL_MSS_VIM_INTMAP_1_MASK_SHIFT                                        (0x00000000U)
#define CSL_MSS_VIM_INTMAP_1_MASK_RESETVAL                                     (0x00000000U)
#define CSL_MSS_VIM_INTMAP_1_MASK_MAX                                          (0xFFFFFFFFU)

#define CSL_MSS_VIM_INTMAP_1_RESETVAL                                          (0x00000000U)

/* INTTYPE_1 */

#define CSL_MSS_VIM_INTTYPE_1_VAL_MASK                                         (0xFFFFFFFFU)
#define CSL_MSS_VIM_INTTYPE_1_VAL_SHIFT                                        (0x00000000U)
#define CSL_MSS_VIM_INTTYPE_1_VAL_RESETVAL                                     (0x00000000U)
#define CSL_MSS_VIM_INTTYPE_1_VAL_MAX                                          (0xFFFFFFFFU)

#define CSL_MSS_VIM_INTTYPE_1_RESETVAL                                         (0x00000000U)

/* RAW_2 */

#define CSL_MSS_VIM_RAW_2_STS_MASK                                             (0xFFFFFFFFU)
#define CSL_MSS_VIM_RAW_2_STS_SHIFT                                            (0x00000000U)
#define CSL_MSS_VIM_RAW_2_STS_RESETVAL                                         (0x00000000U)
#define CSL_MSS_VIM_RAW_2_STS_MAX                                              (0xFFFFFFFFU)

#define CSL_MSS_VIM_RAW_2_RESETVAL                                             (0x00000000U)

/* STS_2 */

#define CSL_MSS_VIM_STS_2_MASK_MASK                                            (0xFFFFFFFFU)
#define CSL_MSS_VIM_STS_2_MASK_SHIFT                                           (0x00000000U)
#define CSL_MSS_VIM_STS_2_MASK_RESETVAL                                        (0x00000000U)
#define CSL_MSS_VIM_STS_2_MASK_MAX                                             (0xFFFFFFFFU)

#define CSL_MSS_VIM_STS_2_RESETVAL                                             (0x00000000U)

/* INTR_EN_SET_2 */

#define CSL_MSS_VIM_INTR_EN_SET_2_MASK_MASK                                    (0xFFFFFFFFU)
#define CSL_MSS_VIM_INTR_EN_SET_2_MASK_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTR_EN_SET_2_MASK_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTR_EN_SET_2_MASK_MAX                                     (0xFFFFFFFFU)

#define CSL_MSS_VIM_INTR_EN_SET_2_RESETVAL                                     (0x00000000U)

/* INTER_EN_CLR_2 */

#define CSL_MSS_VIM_INTER_EN_CLR_2_MASK_MASK                                   (0xFFFFFFFFU)
#define CSL_MSS_VIM_INTER_EN_CLR_2_MASK_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTER_EN_CLR_2_MASK_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTER_EN_CLR_2_MASK_MAX                                    (0xFFFFFFFFU)

#define CSL_MSS_VIM_INTER_EN_CLR_2_RESETVAL                                    (0x00000000U)

/* IRQSTS_2 */

#define CSL_MSS_VIM_IRQSTS_2_MASK_MASK                                         (0xFFFFFFFFU)
#define CSL_MSS_VIM_IRQSTS_2_MASK_SHIFT                                        (0x00000000U)
#define CSL_MSS_VIM_IRQSTS_2_MASK_RESETVAL                                     (0x00000000U)
#define CSL_MSS_VIM_IRQSTS_2_MASK_MAX                                          (0xFFFFFFFFU)

#define CSL_MSS_VIM_IRQSTS_2_RESETVAL                                          (0x00000000U)

/* FIQSTS_2 */

#define CSL_MSS_VIM_FIQSTS_2_MASK_MASK                                         (0xFFFFFFFFU)
#define CSL_MSS_VIM_FIQSTS_2_MASK_SHIFT                                        (0x00000000U)
#define CSL_MSS_VIM_FIQSTS_2_MASK_RESETVAL                                     (0x00000000U)
#define CSL_MSS_VIM_FIQSTS_2_MASK_MAX                                          (0xFFFFFFFFU)

#define CSL_MSS_VIM_FIQSTS_2_RESETVAL                                          (0x00000000U)

/* INTMAP_2 */

#define CSL_MSS_VIM_INTMAP_2_MASK_MASK                                         (0xFFFFFFFFU)
#define CSL_MSS_VIM_INTMAP_2_MASK_SHIFT                                        (0x00000000U)
#define CSL_MSS_VIM_INTMAP_2_MASK_RESETVAL                                     (0x00000000U)
#define CSL_MSS_VIM_INTMAP_2_MASK_MAX                                          (0xFFFFFFFFU)

#define CSL_MSS_VIM_INTMAP_2_RESETVAL                                          (0x00000000U)

/* INTTYPE_2 */

#define CSL_MSS_VIM_INTTYPE_2_VAL_MASK                                         (0xFFFFFFFFU)
#define CSL_MSS_VIM_INTTYPE_2_VAL_SHIFT                                        (0x00000000U)
#define CSL_MSS_VIM_INTTYPE_2_VAL_RESETVAL                                     (0x00000000U)
#define CSL_MSS_VIM_INTTYPE_2_VAL_MAX                                          (0xFFFFFFFFU)

#define CSL_MSS_VIM_INTTYPE_2_RESETVAL                                         (0x00000000U)

/* RAW_3 */

#define CSL_MSS_VIM_RAW_3_STS_MASK                                             (0xFFFFFFFFU)
#define CSL_MSS_VIM_RAW_3_STS_SHIFT                                            (0x00000000U)
#define CSL_MSS_VIM_RAW_3_STS_RESETVAL                                         (0x00000000U)
#define CSL_MSS_VIM_RAW_3_STS_MAX                                              (0xFFFFFFFFU)

#define CSL_MSS_VIM_RAW_3_RESETVAL                                             (0x00000000U)

/* STS_3 */

#define CSL_MSS_VIM_STS_3_MASK_MASK                                            (0xFFFFFFFFU)
#define CSL_MSS_VIM_STS_3_MASK_SHIFT                                           (0x00000000U)
#define CSL_MSS_VIM_STS_3_MASK_RESETVAL                                        (0x00000000U)
#define CSL_MSS_VIM_STS_3_MASK_MAX                                             (0xFFFFFFFFU)

#define CSL_MSS_VIM_STS_3_RESETVAL                                             (0x00000000U)

/* INTR_EN_SET_3 */

#define CSL_MSS_VIM_INTR_EN_SET_3_MASK_MASK                                    (0xFFFFFFFFU)
#define CSL_MSS_VIM_INTR_EN_SET_3_MASK_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTR_EN_SET_3_MASK_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTR_EN_SET_3_MASK_MAX                                     (0xFFFFFFFFU)

#define CSL_MSS_VIM_INTR_EN_SET_3_RESETVAL                                     (0x00000000U)

/* INTER_EN_CLR_3 */

#define CSL_MSS_VIM_INTER_EN_CLR_3_MASK_MASK                                   (0xFFFFFFFFU)
#define CSL_MSS_VIM_INTER_EN_CLR_3_MASK_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTER_EN_CLR_3_MASK_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTER_EN_CLR_3_MASK_MAX                                    (0xFFFFFFFFU)

#define CSL_MSS_VIM_INTER_EN_CLR_3_RESETVAL                                    (0x00000000U)

/* IRQSTS_3 */

#define CSL_MSS_VIM_IRQSTS_3_MASK_MASK                                         (0xFFFFFFFFU)
#define CSL_MSS_VIM_IRQSTS_3_MASK_SHIFT                                        (0x00000000U)
#define CSL_MSS_VIM_IRQSTS_3_MASK_RESETVAL                                     (0x00000000U)
#define CSL_MSS_VIM_IRQSTS_3_MASK_MAX                                          (0xFFFFFFFFU)

#define CSL_MSS_VIM_IRQSTS_3_RESETVAL                                          (0x00000000U)

/* FIQSTS_3 */

#define CSL_MSS_VIM_FIQSTS_3_MASK_MASK                                         (0xFFFFFFFFU)
#define CSL_MSS_VIM_FIQSTS_3_MASK_SHIFT                                        (0x00000000U)
#define CSL_MSS_VIM_FIQSTS_3_MASK_RESETVAL                                     (0x00000000U)
#define CSL_MSS_VIM_FIQSTS_3_MASK_MAX                                          (0xFFFFFFFFU)

#define CSL_MSS_VIM_FIQSTS_3_RESETVAL                                          (0x00000000U)

/* INTMAP_3 */

#define CSL_MSS_VIM_INTMAP_3_MASK_MASK                                         (0xFFFFFFFFU)
#define CSL_MSS_VIM_INTMAP_3_MASK_SHIFT                                        (0x00000000U)
#define CSL_MSS_VIM_INTMAP_3_MASK_RESETVAL                                     (0x00000000U)
#define CSL_MSS_VIM_INTMAP_3_MASK_MAX                                          (0xFFFFFFFFU)

#define CSL_MSS_VIM_INTMAP_3_RESETVAL                                          (0x00000000U)

/* INTTYPE_3 */

#define CSL_MSS_VIM_INTTYPE_3_VAL_MASK                                         (0xFFFFFFFFU)
#define CSL_MSS_VIM_INTTYPE_3_VAL_SHIFT                                        (0x00000000U)
#define CSL_MSS_VIM_INTTYPE_3_VAL_RESETVAL                                     (0x00000000U)
#define CSL_MSS_VIM_INTTYPE_3_VAL_MAX                                          (0xFFFFFFFFU)

#define CSL_MSS_VIM_INTTYPE_3_RESETVAL                                         (0x00000000U)

/* RAW_4 */

#define CSL_MSS_VIM_RAW_4_STS_MASK                                             (0xFFFFFFFFU)
#define CSL_MSS_VIM_RAW_4_STS_SHIFT                                            (0x00000000U)
#define CSL_MSS_VIM_RAW_4_STS_RESETVAL                                         (0x00000000U)
#define CSL_MSS_VIM_RAW_4_STS_MAX                                              (0xFFFFFFFFU)

#define CSL_MSS_VIM_RAW_4_RESETVAL                                             (0x00000000U)

/* STS_4 */

#define CSL_MSS_VIM_STS_4_MASK_MASK                                            (0xFFFFFFFFU)
#define CSL_MSS_VIM_STS_4_MASK_SHIFT                                           (0x00000000U)
#define CSL_MSS_VIM_STS_4_MASK_RESETVAL                                        (0x00000000U)
#define CSL_MSS_VIM_STS_4_MASK_MAX                                             (0xFFFFFFFFU)

#define CSL_MSS_VIM_STS_4_RESETVAL                                             (0x00000000U)

/* INTR_EN_SET_4 */

#define CSL_MSS_VIM_INTR_EN_SET_4_MASK_MASK                                    (0xFFFFFFFFU)
#define CSL_MSS_VIM_INTR_EN_SET_4_MASK_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTR_EN_SET_4_MASK_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTR_EN_SET_4_MASK_MAX                                     (0xFFFFFFFFU)

#define CSL_MSS_VIM_INTR_EN_SET_4_RESETVAL                                     (0x00000000U)

/* INTER_EN_CLR_4 */

#define CSL_MSS_VIM_INTER_EN_CLR_4_MASK_MASK                                   (0xFFFFFFFFU)
#define CSL_MSS_VIM_INTER_EN_CLR_4_MASK_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTER_EN_CLR_4_MASK_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTER_EN_CLR_4_MASK_MAX                                    (0xFFFFFFFFU)

#define CSL_MSS_VIM_INTER_EN_CLR_4_RESETVAL                                    (0x00000000U)

/* IRQSTS_4 */

#define CSL_MSS_VIM_IRQSTS_4_MASK_MASK                                         (0xFFFFFFFFU)
#define CSL_MSS_VIM_IRQSTS_4_MASK_SHIFT                                        (0x00000000U)
#define CSL_MSS_VIM_IRQSTS_4_MASK_RESETVAL                                     (0x00000000U)
#define CSL_MSS_VIM_IRQSTS_4_MASK_MAX                                          (0xFFFFFFFFU)

#define CSL_MSS_VIM_IRQSTS_4_RESETVAL                                          (0x00000000U)

/* FIQSTS_4 */

#define CSL_MSS_VIM_FIQSTS_4_MASK_MASK                                         (0xFFFFFFFFU)
#define CSL_MSS_VIM_FIQSTS_4_MASK_SHIFT                                        (0x00000000U)
#define CSL_MSS_VIM_FIQSTS_4_MASK_RESETVAL                                     (0x00000000U)
#define CSL_MSS_VIM_FIQSTS_4_MASK_MAX                                          (0xFFFFFFFFU)

#define CSL_MSS_VIM_FIQSTS_4_RESETVAL                                          (0x00000000U)

/* INTMAP_4 */

#define CSL_MSS_VIM_INTMAP_4_MASK_MASK                                         (0xFFFFFFFFU)
#define CSL_MSS_VIM_INTMAP_4_MASK_SHIFT                                        (0x00000000U)
#define CSL_MSS_VIM_INTMAP_4_MASK_RESETVAL                                     (0x00000000U)
#define CSL_MSS_VIM_INTMAP_4_MASK_MAX                                          (0xFFFFFFFFU)

#define CSL_MSS_VIM_INTMAP_4_RESETVAL                                          (0x00000000U)

/* INTTYPE_4 */

#define CSL_MSS_VIM_INTTYPE_4_VAL_MASK                                         (0xFFFFFFFFU)
#define CSL_MSS_VIM_INTTYPE_4_VAL_SHIFT                                        (0x00000000U)
#define CSL_MSS_VIM_INTTYPE_4_VAL_RESETVAL                                     (0x00000000U)
#define CSL_MSS_VIM_INTTYPE_4_VAL_MAX                                          (0xFFFFFFFFU)

#define CSL_MSS_VIM_INTTYPE_4_RESETVAL                                         (0x00000000U)

/* RAW_5 */

#define CSL_MSS_VIM_RAW_5_STS_MASK                                             (0xFFFFFFFFU)
#define CSL_MSS_VIM_RAW_5_STS_SHIFT                                            (0x00000000U)
#define CSL_MSS_VIM_RAW_5_STS_RESETVAL                                         (0x00000000U)
#define CSL_MSS_VIM_RAW_5_STS_MAX                                              (0xFFFFFFFFU)

#define CSL_MSS_VIM_RAW_5_RESETVAL                                             (0x00000000U)

/* STS_5 */

#define CSL_MSS_VIM_STS_5_MASK_MASK                                            (0xFFFFFFFFU)
#define CSL_MSS_VIM_STS_5_MASK_SHIFT                                           (0x00000000U)
#define CSL_MSS_VIM_STS_5_MASK_RESETVAL                                        (0x00000000U)
#define CSL_MSS_VIM_STS_5_MASK_MAX                                             (0xFFFFFFFFU)

#define CSL_MSS_VIM_STS_5_RESETVAL                                             (0x00000000U)

/* INTR_EN_SET_5 */

#define CSL_MSS_VIM_INTR_EN_SET_5_MASK_MASK                                    (0xFFFFFFFFU)
#define CSL_MSS_VIM_INTR_EN_SET_5_MASK_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTR_EN_SET_5_MASK_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTR_EN_SET_5_MASK_MAX                                     (0xFFFFFFFFU)

#define CSL_MSS_VIM_INTR_EN_SET_5_RESETVAL                                     (0x00000000U)

/* INTER_EN_CLR_5 */

#define CSL_MSS_VIM_INTER_EN_CLR_5_MASK_MASK                                   (0xFFFFFFFFU)
#define CSL_MSS_VIM_INTER_EN_CLR_5_MASK_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTER_EN_CLR_5_MASK_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTER_EN_CLR_5_MASK_MAX                                    (0xFFFFFFFFU)

#define CSL_MSS_VIM_INTER_EN_CLR_5_RESETVAL                                    (0x00000000U)

/* IRQSTS_5 */

#define CSL_MSS_VIM_IRQSTS_5_MASK_MASK                                         (0xFFFFFFFFU)
#define CSL_MSS_VIM_IRQSTS_5_MASK_SHIFT                                        (0x00000000U)
#define CSL_MSS_VIM_IRQSTS_5_MASK_RESETVAL                                     (0x00000000U)
#define CSL_MSS_VIM_IRQSTS_5_MASK_MAX                                          (0xFFFFFFFFU)

#define CSL_MSS_VIM_IRQSTS_5_RESETVAL                                          (0x00000000U)

/* FIQSTS_5 */

#define CSL_MSS_VIM_FIQSTS_5_MASK_MASK                                         (0xFFFFFFFFU)
#define CSL_MSS_VIM_FIQSTS_5_MASK_SHIFT                                        (0x00000000U)
#define CSL_MSS_VIM_FIQSTS_5_MASK_RESETVAL                                     (0x00000000U)
#define CSL_MSS_VIM_FIQSTS_5_MASK_MAX                                          (0xFFFFFFFFU)

#define CSL_MSS_VIM_FIQSTS_5_RESETVAL                                          (0x00000000U)

/* INTMAP_5 */

#define CSL_MSS_VIM_INTMAP_5_MASK_MASK                                         (0xFFFFFFFFU)
#define CSL_MSS_VIM_INTMAP_5_MASK_SHIFT                                        (0x00000000U)
#define CSL_MSS_VIM_INTMAP_5_MASK_RESETVAL                                     (0x00000000U)
#define CSL_MSS_VIM_INTMAP_5_MASK_MAX                                          (0xFFFFFFFFU)

#define CSL_MSS_VIM_INTMAP_5_RESETVAL                                          (0x00000000U)

/* INTTYPE_5 */

#define CSL_MSS_VIM_INTTYPE_5_VAL_MASK                                         (0xFFFFFFFFU)
#define CSL_MSS_VIM_INTTYPE_5_VAL_SHIFT                                        (0x00000000U)
#define CSL_MSS_VIM_INTTYPE_5_VAL_RESETVAL                                     (0x00000000U)
#define CSL_MSS_VIM_INTTYPE_5_VAL_MAX                                          (0xFFFFFFFFU)

#define CSL_MSS_VIM_INTTYPE_5_RESETVAL                                         (0x00000000U)

/* RAW_6 */

#define CSL_MSS_VIM_RAW_6_STS_MASK                                             (0xFFFFFFFFU)
#define CSL_MSS_VIM_RAW_6_STS_SHIFT                                            (0x00000000U)
#define CSL_MSS_VIM_RAW_6_STS_RESETVAL                                         (0x00000000U)
#define CSL_MSS_VIM_RAW_6_STS_MAX                                              (0xFFFFFFFFU)

#define CSL_MSS_VIM_RAW_6_RESETVAL                                             (0x00000000U)

/* STS_6 */

#define CSL_MSS_VIM_STS_6_MASK_MASK                                            (0xFFFFFFFFU)
#define CSL_MSS_VIM_STS_6_MASK_SHIFT                                           (0x00000000U)
#define CSL_MSS_VIM_STS_6_MASK_RESETVAL                                        (0x00000000U)
#define CSL_MSS_VIM_STS_6_MASK_MAX                                             (0xFFFFFFFFU)

#define CSL_MSS_VIM_STS_6_RESETVAL                                             (0x00000000U)

/* INTR_EN_SET_6 */

#define CSL_MSS_VIM_INTR_EN_SET_6_MASK_MASK                                    (0xFFFFFFFFU)
#define CSL_MSS_VIM_INTR_EN_SET_6_MASK_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTR_EN_SET_6_MASK_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTR_EN_SET_6_MASK_MAX                                     (0xFFFFFFFFU)

#define CSL_MSS_VIM_INTR_EN_SET_6_RESETVAL                                     (0x00000000U)

/* INTER_EN_CLR_6 */

#define CSL_MSS_VIM_INTER_EN_CLR_6_MASK_MASK                                   (0xFFFFFFFFU)
#define CSL_MSS_VIM_INTER_EN_CLR_6_MASK_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTER_EN_CLR_6_MASK_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTER_EN_CLR_6_MASK_MAX                                    (0xFFFFFFFFU)

#define CSL_MSS_VIM_INTER_EN_CLR_6_RESETVAL                                    (0x00000000U)

/* IRQSTS_6 */

#define CSL_MSS_VIM_IRQSTS_6_MASK_MASK                                         (0xFFFFFFFFU)
#define CSL_MSS_VIM_IRQSTS_6_MASK_SHIFT                                        (0x00000000U)
#define CSL_MSS_VIM_IRQSTS_6_MASK_RESETVAL                                     (0x00000000U)
#define CSL_MSS_VIM_IRQSTS_6_MASK_MAX                                          (0xFFFFFFFFU)

#define CSL_MSS_VIM_IRQSTS_6_RESETVAL                                          (0x00000000U)

/* FIQSTS_6 */

#define CSL_MSS_VIM_FIQSTS_6_MASK_MASK                                         (0xFFFFFFFFU)
#define CSL_MSS_VIM_FIQSTS_6_MASK_SHIFT                                        (0x00000000U)
#define CSL_MSS_VIM_FIQSTS_6_MASK_RESETVAL                                     (0x00000000U)
#define CSL_MSS_VIM_FIQSTS_6_MASK_MAX                                          (0xFFFFFFFFU)

#define CSL_MSS_VIM_FIQSTS_6_RESETVAL                                          (0x00000000U)

/* INTMAP_6 */

#define CSL_MSS_VIM_INTMAP_6_MASK_MASK                                         (0xFFFFFFFFU)
#define CSL_MSS_VIM_INTMAP_6_MASK_SHIFT                                        (0x00000000U)
#define CSL_MSS_VIM_INTMAP_6_MASK_RESETVAL                                     (0x00000000U)
#define CSL_MSS_VIM_INTMAP_6_MASK_MAX                                          (0xFFFFFFFFU)

#define CSL_MSS_VIM_INTMAP_6_RESETVAL                                          (0x00000000U)

/* INTTYPE_6 */

#define CSL_MSS_VIM_INTTYPE_6_VAL_MASK                                         (0xFFFFFFFFU)
#define CSL_MSS_VIM_INTTYPE_6_VAL_SHIFT                                        (0x00000000U)
#define CSL_MSS_VIM_INTTYPE_6_VAL_RESETVAL                                     (0x00000000U)
#define CSL_MSS_VIM_INTTYPE_6_VAL_MAX                                          (0xFFFFFFFFU)

#define CSL_MSS_VIM_INTTYPE_6_RESETVAL                                         (0x00000000U)

/* RAW_7 */

#define CSL_MSS_VIM_RAW_7_STS_MASK                                             (0xFFFFFFFFU)
#define CSL_MSS_VIM_RAW_7_STS_SHIFT                                            (0x00000000U)
#define CSL_MSS_VIM_RAW_7_STS_RESETVAL                                         (0x00000000U)
#define CSL_MSS_VIM_RAW_7_STS_MAX                                              (0xFFFFFFFFU)

#define CSL_MSS_VIM_RAW_7_RESETVAL                                             (0x00000000U)

/* STS_7 */

#define CSL_MSS_VIM_STS_7_MASK_MASK                                            (0xFFFFFFFFU)
#define CSL_MSS_VIM_STS_7_MASK_SHIFT                                           (0x00000000U)
#define CSL_MSS_VIM_STS_7_MASK_RESETVAL                                        (0x00000000U)
#define CSL_MSS_VIM_STS_7_MASK_MAX                                             (0xFFFFFFFFU)

#define CSL_MSS_VIM_STS_7_RESETVAL                                             (0x00000000U)

/* INTR_EN_SET_7 */

#define CSL_MSS_VIM_INTR_EN_SET_7_MASK_MASK                                    (0xFFFFFFFFU)
#define CSL_MSS_VIM_INTR_EN_SET_7_MASK_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTR_EN_SET_7_MASK_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTR_EN_SET_7_MASK_MAX                                     (0xFFFFFFFFU)

#define CSL_MSS_VIM_INTR_EN_SET_7_RESETVAL                                     (0x00000000U)

/* INTER_EN_CLR_7 */

#define CSL_MSS_VIM_INTER_EN_CLR_7_MASK_MASK                                   (0xFFFFFFFFU)
#define CSL_MSS_VIM_INTER_EN_CLR_7_MASK_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTER_EN_CLR_7_MASK_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTER_EN_CLR_7_MASK_MAX                                    (0xFFFFFFFFU)

#define CSL_MSS_VIM_INTER_EN_CLR_7_RESETVAL                                    (0x00000000U)

/* IRQSTS_7 */

#define CSL_MSS_VIM_IRQSTS_7_MASK_MASK                                         (0xFFFFFFFFU)
#define CSL_MSS_VIM_IRQSTS_7_MASK_SHIFT                                        (0x00000000U)
#define CSL_MSS_VIM_IRQSTS_7_MASK_RESETVAL                                     (0x00000000U)
#define CSL_MSS_VIM_IRQSTS_7_MASK_MAX                                          (0xFFFFFFFFU)

#define CSL_MSS_VIM_IRQSTS_7_RESETVAL                                          (0x00000000U)

/* FIQSTS_7 */

#define CSL_MSS_VIM_FIQSTS_7_MASK_MASK                                         (0xFFFFFFFFU)
#define CSL_MSS_VIM_FIQSTS_7_MASK_SHIFT                                        (0x00000000U)
#define CSL_MSS_VIM_FIQSTS_7_MASK_RESETVAL                                     (0x00000000U)
#define CSL_MSS_VIM_FIQSTS_7_MASK_MAX                                          (0xFFFFFFFFU)

#define CSL_MSS_VIM_FIQSTS_7_RESETVAL                                          (0x00000000U)

/* INTMAP_7 */

#define CSL_MSS_VIM_INTMAP_7_MASK_MASK                                         (0xFFFFFFFFU)
#define CSL_MSS_VIM_INTMAP_7_MASK_SHIFT                                        (0x00000000U)
#define CSL_MSS_VIM_INTMAP_7_MASK_RESETVAL                                     (0x00000000U)
#define CSL_MSS_VIM_INTMAP_7_MASK_MAX                                          (0xFFFFFFFFU)

#define CSL_MSS_VIM_INTMAP_7_RESETVAL                                          (0x00000000U)

/* INTTYPE_7 */

#define CSL_MSS_VIM_INTTYPE_7_VAL_MASK                                         (0xFFFFFFFFU)
#define CSL_MSS_VIM_INTTYPE_7_VAL_SHIFT                                        (0x00000000U)
#define CSL_MSS_VIM_INTTYPE_7_VAL_RESETVAL                                     (0x00000000U)
#define CSL_MSS_VIM_INTTYPE_7_VAL_MAX                                          (0xFFFFFFFFU)

#define CSL_MSS_VIM_INTTYPE_7_RESETVAL                                         (0x00000000U)

/* INTPRIORITY */

#define CSL_MSS_VIM_INTPRIORITY_PRI_MASK                                       (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_PRI_SHIFT                                      (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_PRI_RESETVAL                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_PRI_MAX                                        (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_RES19_MASK                                     (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_RES19_SHIFT                                    (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_RES19_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_RES19_MAX                                      (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_RESETVAL                                       (0x0000000FU)

/* INTPRIORITY_1 */

#define CSL_MSS_VIM_INTPRIORITY_1_PRI_MASK                                     (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_1_PRI_SHIFT                                    (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_1_PRI_RESETVAL                                 (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_1_PRI_MAX                                      (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_1_RES19_MASK                                   (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_1_RES19_SHIFT                                  (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_1_RES19_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_1_RES19_MAX                                    (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_1_RESETVAL                                     (0x0000000FU)

/* INTPRIORITY_2 */

#define CSL_MSS_VIM_INTPRIORITY_2_PRI_MASK                                     (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_2_PRI_SHIFT                                    (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_2_PRI_RESETVAL                                 (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_2_PRI_MAX                                      (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_2_RES19_MASK                                   (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_2_RES19_SHIFT                                  (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_2_RES19_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_2_RES19_MAX                                    (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_2_RESETVAL                                     (0x0000000FU)

/* INTPRIORITY_3 */

#define CSL_MSS_VIM_INTPRIORITY_3_PRI_MASK                                     (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_3_PRI_SHIFT                                    (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_3_PRI_RESETVAL                                 (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_3_PRI_MAX                                      (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_3_RES19_MASK                                   (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_3_RES19_SHIFT                                  (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_3_RES19_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_3_RES19_MAX                                    (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_3_RESETVAL                                     (0x0000000FU)

/* INTPRIORITY_4 */

#define CSL_MSS_VIM_INTPRIORITY_4_PRI_MASK                                     (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_4_PRI_SHIFT                                    (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_4_PRI_RESETVAL                                 (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_4_PRI_MAX                                      (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_4_RES19_MASK                                   (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_4_RES19_SHIFT                                  (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_4_RES19_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_4_RES19_MAX                                    (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_4_RESETVAL                                     (0x0000000FU)

/* INTPRIORITY_5 */

#define CSL_MSS_VIM_INTPRIORITY_5_PRI_MASK                                     (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_5_PRI_SHIFT                                    (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_5_PRI_RESETVAL                                 (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_5_PRI_MAX                                      (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_5_RES19_MASK                                   (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_5_RES19_SHIFT                                  (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_5_RES19_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_5_RES19_MAX                                    (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_5_RESETVAL                                     (0x0000000FU)

/* INTPRIORITY_6 */

#define CSL_MSS_VIM_INTPRIORITY_6_PRI_MASK                                     (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_6_PRI_SHIFT                                    (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_6_PRI_RESETVAL                                 (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_6_PRI_MAX                                      (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_6_RES19_MASK                                   (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_6_RES19_SHIFT                                  (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_6_RES19_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_6_RES19_MAX                                    (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_6_RESETVAL                                     (0x0000000FU)

/* INTPRIORITY_7 */

#define CSL_MSS_VIM_INTPRIORITY_7_PRI_MASK                                     (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_7_PRI_SHIFT                                    (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_7_PRI_RESETVAL                                 (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_7_PRI_MAX                                      (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_7_RES19_MASK                                   (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_7_RES19_SHIFT                                  (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_7_RES19_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_7_RES19_MAX                                    (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_7_RESETVAL                                     (0x0000000FU)

/* INTPRIORITY_8 */

#define CSL_MSS_VIM_INTPRIORITY_8_PRI_MASK                                     (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_8_PRI_SHIFT                                    (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_8_PRI_RESETVAL                                 (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_8_PRI_MAX                                      (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_8_RES19_MASK                                   (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_8_RES19_SHIFT                                  (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_8_RES19_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_8_RES19_MAX                                    (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_8_RESETVAL                                     (0x0000000FU)

/* INTPRIORITY_9 */

#define CSL_MSS_VIM_INTPRIORITY_9_PRI_MASK                                     (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_9_PRI_SHIFT                                    (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_9_PRI_RESETVAL                                 (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_9_PRI_MAX                                      (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_9_RES19_MASK                                   (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_9_RES19_SHIFT                                  (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_9_RES19_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_9_RES19_MAX                                    (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_9_RESETVAL                                     (0x0000000FU)

/* INTPRIORITY_10 */

#define CSL_MSS_VIM_INTPRIORITY_10_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_10_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_10_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_10_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_10_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_10_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_10_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_10_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_10_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_11 */

#define CSL_MSS_VIM_INTPRIORITY_11_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_11_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_11_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_11_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_11_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_11_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_11_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_11_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_11_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_12 */

#define CSL_MSS_VIM_INTPRIORITY_12_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_12_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_12_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_12_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_12_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_12_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_12_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_12_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_12_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_13 */

#define CSL_MSS_VIM_INTPRIORITY_13_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_13_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_13_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_13_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_13_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_13_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_13_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_13_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_13_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_14 */

#define CSL_MSS_VIM_INTPRIORITY_14_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_14_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_14_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_14_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_14_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_14_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_14_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_14_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_14_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_15 */

#define CSL_MSS_VIM_INTPRIORITY_15_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_15_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_15_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_15_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_15_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_15_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_15_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_15_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_15_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_16 */

#define CSL_MSS_VIM_INTPRIORITY_16_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_16_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_16_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_16_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_16_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_16_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_16_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_16_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_16_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_17 */

#define CSL_MSS_VIM_INTPRIORITY_17_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_17_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_17_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_17_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_17_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_17_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_17_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_17_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_17_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_18 */

#define CSL_MSS_VIM_INTPRIORITY_18_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_18_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_18_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_18_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_18_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_18_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_18_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_18_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_18_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_19 */

#define CSL_MSS_VIM_INTPRIORITY_19_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_19_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_19_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_19_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_19_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_19_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_19_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_19_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_19_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_20 */

#define CSL_MSS_VIM_INTPRIORITY_20_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_20_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_20_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_20_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_20_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_20_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_20_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_20_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_20_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_21 */

#define CSL_MSS_VIM_INTPRIORITY_21_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_21_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_21_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_21_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_21_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_21_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_21_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_21_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_21_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_22 */

#define CSL_MSS_VIM_INTPRIORITY_22_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_22_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_22_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_22_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_22_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_22_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_22_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_22_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_22_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_23 */

#define CSL_MSS_VIM_INTPRIORITY_23_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_23_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_23_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_23_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_23_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_23_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_23_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_23_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_23_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_24 */

#define CSL_MSS_VIM_INTPRIORITY_24_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_24_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_24_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_24_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_24_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_24_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_24_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_24_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_24_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_25 */

#define CSL_MSS_VIM_INTPRIORITY_25_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_25_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_25_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_25_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_25_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_25_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_25_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_25_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_25_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_26 */

#define CSL_MSS_VIM_INTPRIORITY_26_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_26_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_26_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_26_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_26_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_26_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_26_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_26_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_26_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_27 */

#define CSL_MSS_VIM_INTPRIORITY_27_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_27_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_27_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_27_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_27_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_27_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_27_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_27_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_27_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_28 */

#define CSL_MSS_VIM_INTPRIORITY_28_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_28_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_28_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_28_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_28_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_28_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_28_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_28_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_28_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_29 */

#define CSL_MSS_VIM_INTPRIORITY_29_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_29_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_29_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_29_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_29_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_29_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_29_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_29_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_29_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_30 */

#define CSL_MSS_VIM_INTPRIORITY_30_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_30_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_30_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_30_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_30_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_30_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_30_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_30_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_30_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_31 */

#define CSL_MSS_VIM_INTPRIORITY_31_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_31_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_31_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_31_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_31_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_31_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_31_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_31_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_31_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_32 */

#define CSL_MSS_VIM_INTPRIORITY_32_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_32_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_32_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_32_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_32_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_32_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_32_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_32_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_32_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_33 */

#define CSL_MSS_VIM_INTPRIORITY_33_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_33_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_33_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_33_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_33_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_33_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_33_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_33_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_33_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_34 */

#define CSL_MSS_VIM_INTPRIORITY_34_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_34_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_34_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_34_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_34_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_34_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_34_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_34_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_34_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_35 */

#define CSL_MSS_VIM_INTPRIORITY_35_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_35_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_35_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_35_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_35_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_35_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_35_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_35_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_35_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_36 */

#define CSL_MSS_VIM_INTPRIORITY_36_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_36_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_36_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_36_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_36_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_36_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_36_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_36_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_36_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_37 */

#define CSL_MSS_VIM_INTPRIORITY_37_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_37_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_37_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_37_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_37_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_37_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_37_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_37_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_37_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_38 */

#define CSL_MSS_VIM_INTPRIORITY_38_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_38_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_38_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_38_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_38_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_38_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_38_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_38_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_38_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_39 */

#define CSL_MSS_VIM_INTPRIORITY_39_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_39_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_39_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_39_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_39_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_39_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_39_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_39_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_39_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_40 */

#define CSL_MSS_VIM_INTPRIORITY_40_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_40_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_40_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_40_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_40_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_40_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_40_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_40_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_40_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_41 */

#define CSL_MSS_VIM_INTPRIORITY_41_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_41_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_41_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_41_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_41_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_41_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_41_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_41_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_41_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_42 */

#define CSL_MSS_VIM_INTPRIORITY_42_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_42_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_42_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_42_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_42_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_42_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_42_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_42_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_42_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_43 */

#define CSL_MSS_VIM_INTPRIORITY_43_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_43_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_43_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_43_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_43_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_43_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_43_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_43_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_43_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_44 */

#define CSL_MSS_VIM_INTPRIORITY_44_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_44_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_44_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_44_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_44_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_44_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_44_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_44_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_44_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_45 */

#define CSL_MSS_VIM_INTPRIORITY_45_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_45_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_45_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_45_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_45_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_45_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_45_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_45_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_45_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_46 */

#define CSL_MSS_VIM_INTPRIORITY_46_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_46_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_46_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_46_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_46_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_46_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_46_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_46_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_46_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_47 */

#define CSL_MSS_VIM_INTPRIORITY_47_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_47_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_47_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_47_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_47_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_47_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_47_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_47_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_47_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_48 */

#define CSL_MSS_VIM_INTPRIORITY_48_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_48_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_48_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_48_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_48_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_48_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_48_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_48_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_48_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_49 */

#define CSL_MSS_VIM_INTPRIORITY_49_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_49_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_49_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_49_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_49_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_49_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_49_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_49_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_49_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_50 */

#define CSL_MSS_VIM_INTPRIORITY_50_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_50_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_50_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_50_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_50_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_50_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_50_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_50_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_50_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_51 */

#define CSL_MSS_VIM_INTPRIORITY_51_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_51_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_51_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_51_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_51_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_51_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_51_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_51_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_51_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_52 */

#define CSL_MSS_VIM_INTPRIORITY_52_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_52_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_52_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_52_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_52_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_52_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_52_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_52_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_52_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_53 */

#define CSL_MSS_VIM_INTPRIORITY_53_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_53_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_53_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_53_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_53_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_53_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_53_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_53_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_53_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_54 */

#define CSL_MSS_VIM_INTPRIORITY_54_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_54_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_54_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_54_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_54_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_54_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_54_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_54_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_54_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_55 */

#define CSL_MSS_VIM_INTPRIORITY_55_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_55_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_55_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_55_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_55_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_55_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_55_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_55_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_55_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_56 */

#define CSL_MSS_VIM_INTPRIORITY_56_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_56_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_56_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_56_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_56_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_56_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_56_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_56_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_56_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_57 */

#define CSL_MSS_VIM_INTPRIORITY_57_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_57_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_57_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_57_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_57_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_57_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_57_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_57_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_57_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_58 */

#define CSL_MSS_VIM_INTPRIORITY_58_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_58_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_58_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_58_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_58_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_58_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_58_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_58_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_58_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_59 */

#define CSL_MSS_VIM_INTPRIORITY_59_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_59_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_59_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_59_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_59_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_59_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_59_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_59_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_59_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_60 */

#define CSL_MSS_VIM_INTPRIORITY_60_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_60_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_60_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_60_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_60_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_60_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_60_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_60_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_60_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_61 */

#define CSL_MSS_VIM_INTPRIORITY_61_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_61_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_61_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_61_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_61_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_61_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_61_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_61_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_61_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_62 */

#define CSL_MSS_VIM_INTPRIORITY_62_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_62_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_62_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_62_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_62_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_62_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_62_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_62_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_62_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_63 */

#define CSL_MSS_VIM_INTPRIORITY_63_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_63_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_63_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_63_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_63_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_63_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_63_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_63_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_63_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_64 */

#define CSL_MSS_VIM_INTPRIORITY_64_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_64_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_64_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_64_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_64_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_64_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_64_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_64_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_64_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_65 */

#define CSL_MSS_VIM_INTPRIORITY_65_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_65_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_65_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_65_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_65_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_65_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_65_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_65_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_65_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_66 */

#define CSL_MSS_VIM_INTPRIORITY_66_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_66_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_66_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_66_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_66_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_66_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_66_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_66_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_66_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_67 */

#define CSL_MSS_VIM_INTPRIORITY_67_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_67_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_67_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_67_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_67_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_67_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_67_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_67_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_67_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_68 */

#define CSL_MSS_VIM_INTPRIORITY_68_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_68_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_68_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_68_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_68_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_68_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_68_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_68_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_68_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_69 */

#define CSL_MSS_VIM_INTPRIORITY_69_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_69_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_69_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_69_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_69_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_69_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_69_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_69_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_69_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_70 */

#define CSL_MSS_VIM_INTPRIORITY_70_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_70_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_70_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_70_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_70_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_70_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_70_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_70_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_70_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_71 */

#define CSL_MSS_VIM_INTPRIORITY_71_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_71_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_71_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_71_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_71_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_71_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_71_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_71_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_71_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_72 */

#define CSL_MSS_VIM_INTPRIORITY_72_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_72_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_72_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_72_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_72_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_72_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_72_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_72_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_72_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_73 */

#define CSL_MSS_VIM_INTPRIORITY_73_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_73_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_73_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_73_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_73_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_73_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_73_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_73_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_73_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_74 */

#define CSL_MSS_VIM_INTPRIORITY_74_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_74_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_74_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_74_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_74_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_74_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_74_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_74_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_74_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_75 */

#define CSL_MSS_VIM_INTPRIORITY_75_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_75_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_75_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_75_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_75_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_75_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_75_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_75_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_75_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_76 */

#define CSL_MSS_VIM_INTPRIORITY_76_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_76_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_76_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_76_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_76_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_76_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_76_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_76_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_76_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_77 */

#define CSL_MSS_VIM_INTPRIORITY_77_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_77_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_77_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_77_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_77_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_77_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_77_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_77_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_77_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_78 */

#define CSL_MSS_VIM_INTPRIORITY_78_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_78_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_78_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_78_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_78_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_78_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_78_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_78_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_78_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_79 */

#define CSL_MSS_VIM_INTPRIORITY_79_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_79_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_79_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_79_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_79_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_79_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_79_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_79_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_79_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_80 */

#define CSL_MSS_VIM_INTPRIORITY_80_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_80_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_80_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_80_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_80_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_80_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_80_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_80_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_80_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_81 */

#define CSL_MSS_VIM_INTPRIORITY_81_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_81_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_81_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_81_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_81_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_81_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_81_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_81_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_81_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_82 */

#define CSL_MSS_VIM_INTPRIORITY_82_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_82_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_82_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_82_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_82_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_82_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_82_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_82_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_82_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_83 */

#define CSL_MSS_VIM_INTPRIORITY_83_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_83_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_83_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_83_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_83_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_83_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_83_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_83_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_83_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_84 */

#define CSL_MSS_VIM_INTPRIORITY_84_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_84_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_84_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_84_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_84_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_84_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_84_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_84_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_84_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_85 */

#define CSL_MSS_VIM_INTPRIORITY_85_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_85_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_85_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_85_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_85_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_85_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_85_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_85_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_85_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_86 */

#define CSL_MSS_VIM_INTPRIORITY_86_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_86_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_86_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_86_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_86_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_86_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_86_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_86_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_86_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_87 */

#define CSL_MSS_VIM_INTPRIORITY_87_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_87_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_87_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_87_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_87_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_87_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_87_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_87_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_87_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_88 */

#define CSL_MSS_VIM_INTPRIORITY_88_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_88_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_88_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_88_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_88_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_88_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_88_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_88_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_88_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_89 */

#define CSL_MSS_VIM_INTPRIORITY_89_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_89_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_89_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_89_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_89_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_89_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_89_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_89_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_89_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_90 */

#define CSL_MSS_VIM_INTPRIORITY_90_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_90_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_90_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_90_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_90_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_90_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_90_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_90_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_90_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_91 */

#define CSL_MSS_VIM_INTPRIORITY_91_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_91_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_91_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_91_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_91_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_91_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_91_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_91_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_91_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_92 */

#define CSL_MSS_VIM_INTPRIORITY_92_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_92_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_92_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_92_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_92_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_92_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_92_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_92_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_92_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_93 */

#define CSL_MSS_VIM_INTPRIORITY_93_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_93_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_93_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_93_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_93_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_93_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_93_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_93_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_93_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_94 */

#define CSL_MSS_VIM_INTPRIORITY_94_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_94_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_94_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_94_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_94_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_94_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_94_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_94_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_94_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_95 */

#define CSL_MSS_VIM_INTPRIORITY_95_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_95_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_95_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_95_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_95_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_95_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_95_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_95_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_95_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_96 */

#define CSL_MSS_VIM_INTPRIORITY_96_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_96_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_96_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_96_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_96_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_96_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_96_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_96_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_96_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_97 */

#define CSL_MSS_VIM_INTPRIORITY_97_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_97_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_97_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_97_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_97_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_97_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_97_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_97_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_97_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_98 */

#define CSL_MSS_VIM_INTPRIORITY_98_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_98_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_98_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_98_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_98_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_98_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_98_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_98_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_98_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_99 */

#define CSL_MSS_VIM_INTPRIORITY_99_PRI_MASK                                    (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_99_PRI_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_99_PRI_RESETVAL                                (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_99_PRI_MAX                                     (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_99_RES19_MASK                                  (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_99_RES19_SHIFT                                 (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_99_RES19_RESETVAL                              (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_99_RES19_MAX                                   (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_99_RESETVAL                                    (0x0000000FU)

/* INTPRIORITY_100 */

#define CSL_MSS_VIM_INTPRIORITY_100_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_100_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_100_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_100_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_100_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_100_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_100_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_100_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_100_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_101 */

#define CSL_MSS_VIM_INTPRIORITY_101_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_101_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_101_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_101_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_101_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_101_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_101_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_101_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_101_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_102 */

#define CSL_MSS_VIM_INTPRIORITY_102_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_102_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_102_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_102_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_102_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_102_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_102_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_102_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_102_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_103 */

#define CSL_MSS_VIM_INTPRIORITY_103_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_103_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_103_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_103_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_103_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_103_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_103_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_103_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_103_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_104 */

#define CSL_MSS_VIM_INTPRIORITY_104_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_104_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_104_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_104_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_104_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_104_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_104_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_104_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_104_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_105 */

#define CSL_MSS_VIM_INTPRIORITY_105_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_105_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_105_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_105_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_105_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_105_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_105_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_105_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_105_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_106 */

#define CSL_MSS_VIM_INTPRIORITY_106_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_106_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_106_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_106_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_106_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_106_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_106_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_106_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_106_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_107 */

#define CSL_MSS_VIM_INTPRIORITY_107_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_107_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_107_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_107_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_107_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_107_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_107_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_107_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_107_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_108 */

#define CSL_MSS_VIM_INTPRIORITY_108_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_108_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_108_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_108_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_108_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_108_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_108_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_108_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_108_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_109 */

#define CSL_MSS_VIM_INTPRIORITY_109_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_109_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_109_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_109_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_109_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_109_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_109_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_109_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_109_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_110 */

#define CSL_MSS_VIM_INTPRIORITY_110_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_110_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_110_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_110_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_110_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_110_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_110_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_110_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_110_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_111 */

#define CSL_MSS_VIM_INTPRIORITY_111_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_111_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_111_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_111_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_111_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_111_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_111_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_111_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_111_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_112 */

#define CSL_MSS_VIM_INTPRIORITY_112_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_112_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_112_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_112_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_112_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_112_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_112_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_112_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_112_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_113 */

#define CSL_MSS_VIM_INTPRIORITY_113_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_113_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_113_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_113_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_113_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_113_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_113_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_113_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_113_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_114 */

#define CSL_MSS_VIM_INTPRIORITY_114_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_114_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_114_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_114_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_114_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_114_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_114_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_114_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_114_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_115 */

#define CSL_MSS_VIM_INTPRIORITY_115_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_115_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_115_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_115_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_115_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_115_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_115_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_115_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_115_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_116 */

#define CSL_MSS_VIM_INTPRIORITY_116_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_116_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_116_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_116_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_116_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_116_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_116_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_116_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_116_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_117 */

#define CSL_MSS_VIM_INTPRIORITY_117_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_117_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_117_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_117_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_117_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_117_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_117_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_117_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_117_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_118 */

#define CSL_MSS_VIM_INTPRIORITY_118_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_118_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_118_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_118_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_118_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_118_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_118_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_118_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_118_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_119 */

#define CSL_MSS_VIM_INTPRIORITY_119_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_119_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_119_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_119_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_119_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_119_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_119_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_119_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_119_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_120 */

#define CSL_MSS_VIM_INTPRIORITY_120_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_120_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_120_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_120_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_120_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_120_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_120_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_120_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_120_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_121 */

#define CSL_MSS_VIM_INTPRIORITY_121_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_121_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_121_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_121_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_121_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_121_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_121_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_121_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_121_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_122 */

#define CSL_MSS_VIM_INTPRIORITY_122_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_122_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_122_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_122_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_122_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_122_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_122_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_122_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_122_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_123 */

#define CSL_MSS_VIM_INTPRIORITY_123_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_123_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_123_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_123_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_123_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_123_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_123_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_123_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_123_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_124 */

#define CSL_MSS_VIM_INTPRIORITY_124_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_124_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_124_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_124_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_124_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_124_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_124_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_124_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_124_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_125 */

#define CSL_MSS_VIM_INTPRIORITY_125_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_125_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_125_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_125_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_125_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_125_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_125_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_125_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_125_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_126 */

#define CSL_MSS_VIM_INTPRIORITY_126_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_126_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_126_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_126_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_126_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_126_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_126_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_126_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_126_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_127 */

#define CSL_MSS_VIM_INTPRIORITY_127_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_127_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_127_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_127_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_127_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_127_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_127_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_127_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_127_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_128 */

#define CSL_MSS_VIM_INTPRIORITY_128_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_128_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_128_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_128_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_128_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_128_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_128_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_128_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_128_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_129 */

#define CSL_MSS_VIM_INTPRIORITY_129_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_129_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_129_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_129_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_129_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_129_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_129_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_129_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_129_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_130 */

#define CSL_MSS_VIM_INTPRIORITY_130_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_130_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_130_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_130_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_130_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_130_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_130_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_130_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_130_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_131 */

#define CSL_MSS_VIM_INTPRIORITY_131_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_131_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_131_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_131_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_131_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_131_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_131_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_131_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_131_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_132 */

#define CSL_MSS_VIM_INTPRIORITY_132_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_132_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_132_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_132_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_132_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_132_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_132_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_132_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_132_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_133 */

#define CSL_MSS_VIM_INTPRIORITY_133_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_133_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_133_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_133_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_133_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_133_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_133_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_133_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_133_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_134 */

#define CSL_MSS_VIM_INTPRIORITY_134_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_134_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_134_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_134_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_134_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_134_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_134_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_134_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_134_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_135 */

#define CSL_MSS_VIM_INTPRIORITY_135_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_135_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_135_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_135_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_135_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_135_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_135_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_135_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_135_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_136 */

#define CSL_MSS_VIM_INTPRIORITY_136_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_136_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_136_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_136_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_136_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_136_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_136_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_136_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_136_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_137 */

#define CSL_MSS_VIM_INTPRIORITY_137_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_137_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_137_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_137_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_137_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_137_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_137_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_137_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_137_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_138 */

#define CSL_MSS_VIM_INTPRIORITY_138_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_138_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_138_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_138_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_138_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_138_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_138_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_138_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_138_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_139 */

#define CSL_MSS_VIM_INTPRIORITY_139_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_139_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_139_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_139_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_139_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_139_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_139_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_139_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_139_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_140 */

#define CSL_MSS_VIM_INTPRIORITY_140_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_140_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_140_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_140_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_140_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_140_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_140_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_140_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_140_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_141 */

#define CSL_MSS_VIM_INTPRIORITY_141_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_141_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_141_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_141_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_141_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_141_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_141_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_141_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_141_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_142 */

#define CSL_MSS_VIM_INTPRIORITY_142_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_142_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_142_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_142_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_142_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_142_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_142_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_142_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_142_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_143 */

#define CSL_MSS_VIM_INTPRIORITY_143_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_143_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_143_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_143_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_143_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_143_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_143_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_143_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_143_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_144 */

#define CSL_MSS_VIM_INTPRIORITY_144_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_144_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_144_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_144_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_144_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_144_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_144_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_144_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_144_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_145 */

#define CSL_MSS_VIM_INTPRIORITY_145_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_145_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_145_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_145_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_145_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_145_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_145_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_145_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_145_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_146 */

#define CSL_MSS_VIM_INTPRIORITY_146_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_146_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_146_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_146_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_146_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_146_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_146_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_146_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_146_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_147 */

#define CSL_MSS_VIM_INTPRIORITY_147_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_147_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_147_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_147_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_147_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_147_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_147_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_147_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_147_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_148 */

#define CSL_MSS_VIM_INTPRIORITY_148_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_148_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_148_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_148_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_148_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_148_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_148_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_148_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_148_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_149 */

#define CSL_MSS_VIM_INTPRIORITY_149_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_149_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_149_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_149_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_149_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_149_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_149_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_149_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_149_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_150 */

#define CSL_MSS_VIM_INTPRIORITY_150_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_150_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_150_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_150_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_150_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_150_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_150_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_150_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_150_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_151 */

#define CSL_MSS_VIM_INTPRIORITY_151_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_151_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_151_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_151_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_151_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_151_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_151_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_151_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_151_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_152 */

#define CSL_MSS_VIM_INTPRIORITY_152_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_152_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_152_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_152_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_152_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_152_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_152_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_152_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_152_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_153 */

#define CSL_MSS_VIM_INTPRIORITY_153_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_153_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_153_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_153_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_153_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_153_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_153_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_153_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_153_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_154 */

#define CSL_MSS_VIM_INTPRIORITY_154_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_154_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_154_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_154_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_154_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_154_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_154_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_154_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_154_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_155 */

#define CSL_MSS_VIM_INTPRIORITY_155_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_155_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_155_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_155_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_155_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_155_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_155_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_155_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_155_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_156 */

#define CSL_MSS_VIM_INTPRIORITY_156_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_156_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_156_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_156_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_156_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_156_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_156_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_156_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_156_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_157 */

#define CSL_MSS_VIM_INTPRIORITY_157_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_157_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_157_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_157_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_157_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_157_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_157_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_157_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_157_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_158 */

#define CSL_MSS_VIM_INTPRIORITY_158_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_158_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_158_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_158_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_158_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_158_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_158_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_158_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_158_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_159 */

#define CSL_MSS_VIM_INTPRIORITY_159_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_159_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_159_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_159_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_159_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_159_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_159_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_159_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_159_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_160 */

#define CSL_MSS_VIM_INTPRIORITY_160_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_160_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_160_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_160_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_160_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_160_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_160_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_160_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_160_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_161 */

#define CSL_MSS_VIM_INTPRIORITY_161_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_161_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_161_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_161_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_161_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_161_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_161_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_161_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_161_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_162 */

#define CSL_MSS_VIM_INTPRIORITY_162_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_162_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_162_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_162_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_162_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_162_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_162_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_162_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_162_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_163 */

#define CSL_MSS_VIM_INTPRIORITY_163_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_163_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_163_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_163_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_163_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_163_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_163_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_163_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_163_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_164 */

#define CSL_MSS_VIM_INTPRIORITY_164_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_164_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_164_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_164_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_164_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_164_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_164_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_164_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_164_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_165 */

#define CSL_MSS_VIM_INTPRIORITY_165_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_165_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_165_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_165_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_165_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_165_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_165_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_165_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_165_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_166 */

#define CSL_MSS_VIM_INTPRIORITY_166_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_166_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_166_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_166_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_166_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_166_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_166_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_166_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_166_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_167 */

#define CSL_MSS_VIM_INTPRIORITY_167_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_167_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_167_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_167_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_167_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_167_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_167_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_167_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_167_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_168 */

#define CSL_MSS_VIM_INTPRIORITY_168_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_168_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_168_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_168_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_168_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_168_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_168_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_168_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_168_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_169 */

#define CSL_MSS_VIM_INTPRIORITY_169_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_169_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_169_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_169_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_169_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_169_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_169_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_169_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_169_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_170 */

#define CSL_MSS_VIM_INTPRIORITY_170_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_170_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_170_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_170_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_170_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_170_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_170_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_170_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_170_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_171 */

#define CSL_MSS_VIM_INTPRIORITY_171_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_171_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_171_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_171_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_171_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_171_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_171_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_171_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_171_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_172 */

#define CSL_MSS_VIM_INTPRIORITY_172_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_172_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_172_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_172_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_172_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_172_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_172_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_172_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_172_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_173 */

#define CSL_MSS_VIM_INTPRIORITY_173_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_173_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_173_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_173_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_173_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_173_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_173_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_173_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_173_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_174 */

#define CSL_MSS_VIM_INTPRIORITY_174_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_174_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_174_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_174_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_174_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_174_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_174_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_174_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_174_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_175 */

#define CSL_MSS_VIM_INTPRIORITY_175_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_175_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_175_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_175_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_175_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_175_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_175_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_175_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_175_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_176 */

#define CSL_MSS_VIM_INTPRIORITY_176_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_176_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_176_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_176_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_176_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_176_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_176_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_176_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_176_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_177 */

#define CSL_MSS_VIM_INTPRIORITY_177_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_177_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_177_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_177_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_177_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_177_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_177_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_177_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_177_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_178 */

#define CSL_MSS_VIM_INTPRIORITY_178_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_178_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_178_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_178_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_178_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_178_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_178_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_178_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_178_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_179 */

#define CSL_MSS_VIM_INTPRIORITY_179_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_179_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_179_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_179_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_179_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_179_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_179_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_179_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_179_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_180 */

#define CSL_MSS_VIM_INTPRIORITY_180_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_180_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_180_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_180_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_180_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_180_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_180_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_180_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_180_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_181 */

#define CSL_MSS_VIM_INTPRIORITY_181_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_181_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_181_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_181_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_181_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_181_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_181_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_181_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_181_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_182 */

#define CSL_MSS_VIM_INTPRIORITY_182_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_182_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_182_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_182_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_182_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_182_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_182_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_182_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_182_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_183 */

#define CSL_MSS_VIM_INTPRIORITY_183_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_183_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_183_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_183_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_183_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_183_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_183_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_183_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_183_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_184 */

#define CSL_MSS_VIM_INTPRIORITY_184_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_184_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_184_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_184_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_184_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_184_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_184_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_184_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_184_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_185 */

#define CSL_MSS_VIM_INTPRIORITY_185_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_185_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_185_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_185_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_185_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_185_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_185_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_185_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_185_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_186 */

#define CSL_MSS_VIM_INTPRIORITY_186_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_186_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_186_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_186_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_186_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_186_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_186_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_186_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_186_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_187 */

#define CSL_MSS_VIM_INTPRIORITY_187_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_187_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_187_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_187_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_187_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_187_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_187_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_187_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_187_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_188 */

#define CSL_MSS_VIM_INTPRIORITY_188_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_188_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_188_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_188_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_188_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_188_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_188_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_188_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_188_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_189 */

#define CSL_MSS_VIM_INTPRIORITY_189_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_189_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_189_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_189_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_189_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_189_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_189_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_189_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_189_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_190 */

#define CSL_MSS_VIM_INTPRIORITY_190_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_190_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_190_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_190_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_190_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_190_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_190_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_190_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_190_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_191 */

#define CSL_MSS_VIM_INTPRIORITY_191_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_191_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_191_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_191_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_191_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_191_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_191_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_191_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_191_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_192 */

#define CSL_MSS_VIM_INTPRIORITY_192_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_192_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_192_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_192_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_192_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_192_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_192_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_192_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_192_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_193 */

#define CSL_MSS_VIM_INTPRIORITY_193_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_193_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_193_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_193_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_193_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_193_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_193_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_193_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_193_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_194 */

#define CSL_MSS_VIM_INTPRIORITY_194_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_194_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_194_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_194_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_194_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_194_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_194_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_194_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_194_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_195 */

#define CSL_MSS_VIM_INTPRIORITY_195_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_195_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_195_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_195_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_195_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_195_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_195_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_195_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_195_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_196 */

#define CSL_MSS_VIM_INTPRIORITY_196_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_196_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_196_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_196_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_196_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_196_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_196_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_196_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_196_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_197 */

#define CSL_MSS_VIM_INTPRIORITY_197_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_197_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_197_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_197_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_197_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_197_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_197_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_197_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_197_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_198 */

#define CSL_MSS_VIM_INTPRIORITY_198_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_198_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_198_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_198_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_198_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_198_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_198_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_198_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_198_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_199 */

#define CSL_MSS_VIM_INTPRIORITY_199_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_199_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_199_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_199_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_199_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_199_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_199_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_199_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_199_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_200 */

#define CSL_MSS_VIM_INTPRIORITY_200_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_200_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_200_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_200_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_200_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_200_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_200_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_200_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_200_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_201 */

#define CSL_MSS_VIM_INTPRIORITY_201_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_201_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_201_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_201_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_201_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_201_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_201_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_201_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_201_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_202 */

#define CSL_MSS_VIM_INTPRIORITY_202_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_202_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_202_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_202_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_202_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_202_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_202_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_202_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_202_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_203 */

#define CSL_MSS_VIM_INTPRIORITY_203_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_203_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_203_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_203_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_203_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_203_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_203_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_203_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_203_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_204 */

#define CSL_MSS_VIM_INTPRIORITY_204_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_204_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_204_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_204_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_204_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_204_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_204_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_204_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_204_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_205 */

#define CSL_MSS_VIM_INTPRIORITY_205_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_205_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_205_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_205_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_205_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_205_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_205_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_205_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_205_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_206 */

#define CSL_MSS_VIM_INTPRIORITY_206_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_206_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_206_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_206_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_206_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_206_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_206_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_206_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_206_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_207 */

#define CSL_MSS_VIM_INTPRIORITY_207_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_207_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_207_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_207_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_207_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_207_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_207_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_207_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_207_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_208 */

#define CSL_MSS_VIM_INTPRIORITY_208_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_208_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_208_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_208_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_208_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_208_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_208_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_208_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_208_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_209 */

#define CSL_MSS_VIM_INTPRIORITY_209_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_209_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_209_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_209_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_209_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_209_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_209_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_209_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_209_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_210 */

#define CSL_MSS_VIM_INTPRIORITY_210_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_210_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_210_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_210_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_210_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_210_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_210_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_210_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_210_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_211 */

#define CSL_MSS_VIM_INTPRIORITY_211_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_211_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_211_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_211_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_211_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_211_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_211_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_211_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_211_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_212 */

#define CSL_MSS_VIM_INTPRIORITY_212_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_212_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_212_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_212_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_212_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_212_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_212_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_212_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_212_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_213 */

#define CSL_MSS_VIM_INTPRIORITY_213_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_213_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_213_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_213_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_213_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_213_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_213_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_213_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_213_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_214 */

#define CSL_MSS_VIM_INTPRIORITY_214_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_214_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_214_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_214_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_214_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_214_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_214_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_214_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_214_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_215 */

#define CSL_MSS_VIM_INTPRIORITY_215_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_215_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_215_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_215_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_215_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_215_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_215_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_215_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_215_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_216 */

#define CSL_MSS_VIM_INTPRIORITY_216_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_216_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_216_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_216_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_216_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_216_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_216_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_216_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_216_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_217 */

#define CSL_MSS_VIM_INTPRIORITY_217_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_217_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_217_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_217_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_217_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_217_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_217_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_217_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_217_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_218 */

#define CSL_MSS_VIM_INTPRIORITY_218_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_218_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_218_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_218_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_218_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_218_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_218_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_218_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_218_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_219 */

#define CSL_MSS_VIM_INTPRIORITY_219_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_219_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_219_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_219_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_219_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_219_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_219_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_219_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_219_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_220 */

#define CSL_MSS_VIM_INTPRIORITY_220_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_220_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_220_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_220_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_220_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_220_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_220_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_220_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_220_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_221 */

#define CSL_MSS_VIM_INTPRIORITY_221_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_221_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_221_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_221_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_221_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_221_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_221_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_221_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_221_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_222 */

#define CSL_MSS_VIM_INTPRIORITY_222_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_222_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_222_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_222_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_222_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_222_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_222_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_222_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_222_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_223 */

#define CSL_MSS_VIM_INTPRIORITY_223_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_223_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_223_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_223_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_223_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_223_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_223_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_223_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_223_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_224 */

#define CSL_MSS_VIM_INTPRIORITY_224_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_224_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_224_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_224_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_224_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_224_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_224_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_224_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_224_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_225 */

#define CSL_MSS_VIM_INTPRIORITY_225_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_225_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_225_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_225_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_225_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_225_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_225_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_225_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_225_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_226 */

#define CSL_MSS_VIM_INTPRIORITY_226_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_226_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_226_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_226_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_226_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_226_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_226_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_226_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_226_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_227 */

#define CSL_MSS_VIM_INTPRIORITY_227_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_227_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_227_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_227_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_227_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_227_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_227_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_227_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_227_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_228 */

#define CSL_MSS_VIM_INTPRIORITY_228_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_228_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_228_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_228_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_228_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_228_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_228_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_228_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_228_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_229 */

#define CSL_MSS_VIM_INTPRIORITY_229_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_229_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_229_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_229_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_229_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_229_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_229_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_229_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_229_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_230 */

#define CSL_MSS_VIM_INTPRIORITY_230_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_230_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_230_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_230_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_230_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_230_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_230_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_230_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_230_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_231 */

#define CSL_MSS_VIM_INTPRIORITY_231_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_231_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_231_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_231_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_231_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_231_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_231_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_231_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_231_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_232 */

#define CSL_MSS_VIM_INTPRIORITY_232_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_232_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_232_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_232_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_232_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_232_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_232_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_232_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_232_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_233 */

#define CSL_MSS_VIM_INTPRIORITY_233_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_233_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_233_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_233_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_233_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_233_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_233_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_233_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_233_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_234 */

#define CSL_MSS_VIM_INTPRIORITY_234_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_234_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_234_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_234_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_234_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_234_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_234_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_234_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_234_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_235 */

#define CSL_MSS_VIM_INTPRIORITY_235_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_235_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_235_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_235_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_235_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_235_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_235_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_235_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_235_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_236 */

#define CSL_MSS_VIM_INTPRIORITY_236_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_236_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_236_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_236_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_236_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_236_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_236_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_236_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_236_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_237 */

#define CSL_MSS_VIM_INTPRIORITY_237_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_237_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_237_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_237_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_237_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_237_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_237_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_237_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_237_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_238 */

#define CSL_MSS_VIM_INTPRIORITY_238_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_238_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_238_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_238_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_238_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_238_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_238_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_238_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_238_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_239 */

#define CSL_MSS_VIM_INTPRIORITY_239_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_239_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_239_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_239_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_239_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_239_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_239_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_239_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_239_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_240 */

#define CSL_MSS_VIM_INTPRIORITY_240_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_240_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_240_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_240_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_240_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_240_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_240_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_240_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_240_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_241 */

#define CSL_MSS_VIM_INTPRIORITY_241_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_241_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_241_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_241_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_241_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_241_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_241_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_241_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_241_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_242 */

#define CSL_MSS_VIM_INTPRIORITY_242_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_242_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_242_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_242_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_242_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_242_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_242_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_242_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_242_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_243 */

#define CSL_MSS_VIM_INTPRIORITY_243_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_243_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_243_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_243_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_243_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_243_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_243_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_243_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_243_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_244 */

#define CSL_MSS_VIM_INTPRIORITY_244_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_244_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_244_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_244_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_244_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_244_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_244_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_244_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_244_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_245 */

#define CSL_MSS_VIM_INTPRIORITY_245_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_245_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_245_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_245_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_245_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_245_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_245_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_245_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_245_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_246 */

#define CSL_MSS_VIM_INTPRIORITY_246_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_246_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_246_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_246_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_246_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_246_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_246_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_246_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_246_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_247 */

#define CSL_MSS_VIM_INTPRIORITY_247_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_247_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_247_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_247_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_247_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_247_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_247_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_247_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_247_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_248 */

#define CSL_MSS_VIM_INTPRIORITY_248_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_248_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_248_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_248_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_248_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_248_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_248_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_248_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_248_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_249 */

#define CSL_MSS_VIM_INTPRIORITY_249_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_249_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_249_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_249_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_249_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_249_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_249_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_249_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_249_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_250 */

#define CSL_MSS_VIM_INTPRIORITY_250_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_250_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_250_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_250_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_250_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_250_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_250_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_250_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_250_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_251 */

#define CSL_MSS_VIM_INTPRIORITY_251_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_251_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_251_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_251_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_251_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_251_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_251_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_251_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_251_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_252 */

#define CSL_MSS_VIM_INTPRIORITY_252_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_252_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_252_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_252_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_252_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_252_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_252_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_252_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_252_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_253 */

#define CSL_MSS_VIM_INTPRIORITY_253_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_253_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_253_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_253_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_253_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_253_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_253_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_253_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_253_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_254 */

#define CSL_MSS_VIM_INTPRIORITY_254_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_254_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_254_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_254_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_254_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_254_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_254_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_254_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_254_RESETVAL                                   (0x0000000FU)

/* INTPRIORITY_255 */

#define CSL_MSS_VIM_INTPRIORITY_255_PRI_MASK                                   (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_255_PRI_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_255_PRI_RESETVAL                               (0x0000000FU)
#define CSL_MSS_VIM_INTPRIORITY_255_PRI_MAX                                    (0x0000000FU)

#define CSL_MSS_VIM_INTPRIORITY_255_RES19_MASK                                 (0xFFFFFFF0U)
#define CSL_MSS_VIM_INTPRIORITY_255_RES19_SHIFT                                (0x00000004U)
#define CSL_MSS_VIM_INTPRIORITY_255_RES19_RESETVAL                             (0x00000000U)
#define CSL_MSS_VIM_INTPRIORITY_255_RES19_MAX                                  (0x0FFFFFFFU)

#define CSL_MSS_VIM_INTPRIORITY_255_RESETVAL                                   (0x0000000FU)

/* INTVECTOR */

#define CSL_MSS_VIM_INTVECTOR_RES20_MASK                                       (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_RES20_SHIFT                                      (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_RES20_RESETVAL                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_RES20_MAX                                        (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_ADDR_MASK                                        (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_ADDR_SHIFT                                       (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_ADDR_RESETVAL                                    (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_ADDR_MAX                                         (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_RESETVAL                                         (0x00000000U)

/* INTVECTOR_1 */

#define CSL_MSS_VIM_INTVECTOR_1_RES20_MASK                                     (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_1_RES20_SHIFT                                    (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_1_RES20_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_1_RES20_MAX                                      (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_1_ADDR_MASK                                      (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_1_ADDR_SHIFT                                     (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_1_ADDR_RESETVAL                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_1_ADDR_MAX                                       (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_1_RESETVAL                                       (0x00000000U)

/* INTVECTOR_2 */

#define CSL_MSS_VIM_INTVECTOR_2_RES20_MASK                                     (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_2_RES20_SHIFT                                    (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_2_RES20_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_2_RES20_MAX                                      (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_2_ADDR_MASK                                      (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_2_ADDR_SHIFT                                     (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_2_ADDR_RESETVAL                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_2_ADDR_MAX                                       (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_2_RESETVAL                                       (0x00000000U)

/* INTVECTOR_3 */

#define CSL_MSS_VIM_INTVECTOR_3_RES20_MASK                                     (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_3_RES20_SHIFT                                    (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_3_RES20_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_3_RES20_MAX                                      (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_3_ADDR_MASK                                      (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_3_ADDR_SHIFT                                     (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_3_ADDR_RESETVAL                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_3_ADDR_MAX                                       (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_3_RESETVAL                                       (0x00000000U)

/* INTVECTOR_4 */

#define CSL_MSS_VIM_INTVECTOR_4_RES20_MASK                                     (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_4_RES20_SHIFT                                    (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_4_RES20_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_4_RES20_MAX                                      (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_4_ADDR_MASK                                      (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_4_ADDR_SHIFT                                     (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_4_ADDR_RESETVAL                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_4_ADDR_MAX                                       (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_4_RESETVAL                                       (0x00000000U)

/* INTVECTOR_5 */

#define CSL_MSS_VIM_INTVECTOR_5_RES20_MASK                                     (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_5_RES20_SHIFT                                    (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_5_RES20_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_5_RES20_MAX                                      (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_5_ADDR_MASK                                      (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_5_ADDR_SHIFT                                     (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_5_ADDR_RESETVAL                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_5_ADDR_MAX                                       (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_5_RESETVAL                                       (0x00000000U)

/* INTVECTOR_6 */

#define CSL_MSS_VIM_INTVECTOR_6_RES20_MASK                                     (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_6_RES20_SHIFT                                    (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_6_RES20_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_6_RES20_MAX                                      (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_6_ADDR_MASK                                      (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_6_ADDR_SHIFT                                     (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_6_ADDR_RESETVAL                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_6_ADDR_MAX                                       (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_6_RESETVAL                                       (0x00000000U)

/* INTVECTOR_7 */

#define CSL_MSS_VIM_INTVECTOR_7_RES20_MASK                                     (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_7_RES20_SHIFT                                    (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_7_RES20_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_7_RES20_MAX                                      (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_7_ADDR_MASK                                      (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_7_ADDR_SHIFT                                     (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_7_ADDR_RESETVAL                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_7_ADDR_MAX                                       (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_7_RESETVAL                                       (0x00000000U)

/* INTVECTOR_8 */

#define CSL_MSS_VIM_INTVECTOR_8_RES20_MASK                                     (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_8_RES20_SHIFT                                    (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_8_RES20_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_8_RES20_MAX                                      (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_8_ADDR_MASK                                      (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_8_ADDR_SHIFT                                     (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_8_ADDR_RESETVAL                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_8_ADDR_MAX                                       (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_8_RESETVAL                                       (0x00000000U)

/* INTVECTOR_9 */

#define CSL_MSS_VIM_INTVECTOR_9_RES20_MASK                                     (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_9_RES20_SHIFT                                    (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_9_RES20_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_9_RES20_MAX                                      (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_9_ADDR_MASK                                      (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_9_ADDR_SHIFT                                     (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_9_ADDR_RESETVAL                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_9_ADDR_MAX                                       (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_9_RESETVAL                                       (0x00000000U)

/* INTVECTOR_10 */

#define CSL_MSS_VIM_INTVECTOR_10_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_10_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_10_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_10_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_10_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_10_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_10_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_10_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_10_RESETVAL                                      (0x00000000U)

/* INTVECTOR_11 */

#define CSL_MSS_VIM_INTVECTOR_11_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_11_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_11_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_11_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_11_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_11_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_11_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_11_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_11_RESETVAL                                      (0x00000000U)

/* INTVECTOR_12 */

#define CSL_MSS_VIM_INTVECTOR_12_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_12_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_12_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_12_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_12_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_12_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_12_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_12_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_12_RESETVAL                                      (0x00000000U)

/* INTVECTOR_13 */

#define CSL_MSS_VIM_INTVECTOR_13_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_13_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_13_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_13_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_13_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_13_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_13_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_13_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_13_RESETVAL                                      (0x00000000U)

/* INTVECTOR_14 */

#define CSL_MSS_VIM_INTVECTOR_14_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_14_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_14_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_14_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_14_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_14_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_14_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_14_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_14_RESETVAL                                      (0x00000000U)

/* INTVECTOR_15 */

#define CSL_MSS_VIM_INTVECTOR_15_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_15_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_15_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_15_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_15_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_15_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_15_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_15_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_15_RESETVAL                                      (0x00000000U)

/* INTVECTOR_16 */

#define CSL_MSS_VIM_INTVECTOR_16_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_16_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_16_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_16_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_16_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_16_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_16_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_16_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_16_RESETVAL                                      (0x00000000U)

/* INTVECTOR_17 */

#define CSL_MSS_VIM_INTVECTOR_17_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_17_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_17_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_17_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_17_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_17_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_17_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_17_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_17_RESETVAL                                      (0x00000000U)

/* INTVECTOR_18 */

#define CSL_MSS_VIM_INTVECTOR_18_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_18_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_18_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_18_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_18_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_18_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_18_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_18_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_18_RESETVAL                                      (0x00000000U)

/* INTVECTOR_19 */

#define CSL_MSS_VIM_INTVECTOR_19_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_19_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_19_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_19_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_19_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_19_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_19_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_19_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_19_RESETVAL                                      (0x00000000U)

/* INTVECTOR_20 */

#define CSL_MSS_VIM_INTVECTOR_20_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_20_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_20_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_20_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_20_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_20_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_20_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_20_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_20_RESETVAL                                      (0x00000000U)

/* INTVECTOR_21 */

#define CSL_MSS_VIM_INTVECTOR_21_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_21_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_21_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_21_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_21_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_21_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_21_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_21_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_21_RESETVAL                                      (0x00000000U)

/* INTVECTOR_22 */

#define CSL_MSS_VIM_INTVECTOR_22_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_22_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_22_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_22_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_22_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_22_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_22_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_22_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_22_RESETVAL                                      (0x00000000U)

/* INTVECTOR_23 */

#define CSL_MSS_VIM_INTVECTOR_23_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_23_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_23_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_23_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_23_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_23_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_23_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_23_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_23_RESETVAL                                      (0x00000000U)

/* INTVECTOR_24 */

#define CSL_MSS_VIM_INTVECTOR_24_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_24_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_24_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_24_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_24_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_24_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_24_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_24_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_24_RESETVAL                                      (0x00000000U)

/* INTVECTOR_25 */

#define CSL_MSS_VIM_INTVECTOR_25_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_25_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_25_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_25_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_25_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_25_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_25_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_25_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_25_RESETVAL                                      (0x00000000U)

/* INTVECTOR_26 */

#define CSL_MSS_VIM_INTVECTOR_26_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_26_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_26_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_26_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_26_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_26_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_26_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_26_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_26_RESETVAL                                      (0x00000000U)

/* INTVECTOR_27 */

#define CSL_MSS_VIM_INTVECTOR_27_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_27_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_27_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_27_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_27_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_27_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_27_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_27_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_27_RESETVAL                                      (0x00000000U)

/* INTVECTOR_28 */

#define CSL_MSS_VIM_INTVECTOR_28_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_28_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_28_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_28_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_28_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_28_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_28_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_28_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_28_RESETVAL                                      (0x00000000U)

/* INTVECTOR_29 */

#define CSL_MSS_VIM_INTVECTOR_29_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_29_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_29_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_29_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_29_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_29_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_29_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_29_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_29_RESETVAL                                      (0x00000000U)

/* INTVECTOR_30 */

#define CSL_MSS_VIM_INTVECTOR_30_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_30_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_30_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_30_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_30_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_30_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_30_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_30_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_30_RESETVAL                                      (0x00000000U)

/* INTVECTOR_31 */

#define CSL_MSS_VIM_INTVECTOR_31_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_31_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_31_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_31_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_31_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_31_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_31_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_31_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_31_RESETVAL                                      (0x00000000U)

/* INTVECTOR_32 */

#define CSL_MSS_VIM_INTVECTOR_32_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_32_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_32_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_32_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_32_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_32_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_32_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_32_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_32_RESETVAL                                      (0x00000000U)

/* INTVECTOR_33 */

#define CSL_MSS_VIM_INTVECTOR_33_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_33_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_33_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_33_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_33_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_33_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_33_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_33_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_33_RESETVAL                                      (0x00000000U)

/* INTVECTOR_34 */

#define CSL_MSS_VIM_INTVECTOR_34_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_34_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_34_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_34_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_34_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_34_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_34_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_34_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_34_RESETVAL                                      (0x00000000U)

/* INTVECTOR_35 */

#define CSL_MSS_VIM_INTVECTOR_35_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_35_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_35_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_35_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_35_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_35_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_35_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_35_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_35_RESETVAL                                      (0x00000000U)

/* INTVECTOR_36 */

#define CSL_MSS_VIM_INTVECTOR_36_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_36_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_36_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_36_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_36_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_36_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_36_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_36_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_36_RESETVAL                                      (0x00000000U)

/* INTVECTOR_37 */

#define CSL_MSS_VIM_INTVECTOR_37_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_37_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_37_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_37_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_37_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_37_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_37_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_37_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_37_RESETVAL                                      (0x00000000U)

/* INTVECTOR_38 */

#define CSL_MSS_VIM_INTVECTOR_38_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_38_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_38_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_38_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_38_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_38_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_38_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_38_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_38_RESETVAL                                      (0x00000000U)

/* INTVECTOR_39 */

#define CSL_MSS_VIM_INTVECTOR_39_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_39_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_39_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_39_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_39_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_39_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_39_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_39_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_39_RESETVAL                                      (0x00000000U)

/* INTVECTOR_40 */

#define CSL_MSS_VIM_INTVECTOR_40_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_40_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_40_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_40_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_40_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_40_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_40_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_40_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_40_RESETVAL                                      (0x00000000U)

/* INTVECTOR_41 */

#define CSL_MSS_VIM_INTVECTOR_41_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_41_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_41_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_41_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_41_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_41_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_41_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_41_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_41_RESETVAL                                      (0x00000000U)

/* INTVECTOR_42 */

#define CSL_MSS_VIM_INTVECTOR_42_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_42_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_42_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_42_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_42_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_42_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_42_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_42_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_42_RESETVAL                                      (0x00000000U)

/* INTVECTOR_43 */

#define CSL_MSS_VIM_INTVECTOR_43_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_43_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_43_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_43_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_43_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_43_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_43_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_43_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_43_RESETVAL                                      (0x00000000U)

/* INTVECTOR_44 */

#define CSL_MSS_VIM_INTVECTOR_44_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_44_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_44_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_44_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_44_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_44_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_44_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_44_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_44_RESETVAL                                      (0x00000000U)

/* INTVECTOR_45 */

#define CSL_MSS_VIM_INTVECTOR_45_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_45_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_45_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_45_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_45_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_45_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_45_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_45_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_45_RESETVAL                                      (0x00000000U)

/* INTVECTOR_46 */

#define CSL_MSS_VIM_INTVECTOR_46_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_46_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_46_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_46_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_46_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_46_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_46_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_46_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_46_RESETVAL                                      (0x00000000U)

/* INTVECTOR_47 */

#define CSL_MSS_VIM_INTVECTOR_47_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_47_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_47_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_47_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_47_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_47_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_47_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_47_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_47_RESETVAL                                      (0x00000000U)

/* INTVECTOR_48 */

#define CSL_MSS_VIM_INTVECTOR_48_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_48_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_48_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_48_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_48_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_48_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_48_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_48_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_48_RESETVAL                                      (0x00000000U)

/* INTVECTOR_49 */

#define CSL_MSS_VIM_INTVECTOR_49_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_49_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_49_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_49_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_49_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_49_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_49_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_49_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_49_RESETVAL                                      (0x00000000U)

/* INTVECTOR_50 */

#define CSL_MSS_VIM_INTVECTOR_50_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_50_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_50_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_50_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_50_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_50_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_50_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_50_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_50_RESETVAL                                      (0x00000000U)

/* INTVECTOR_51 */

#define CSL_MSS_VIM_INTVECTOR_51_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_51_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_51_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_51_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_51_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_51_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_51_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_51_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_51_RESETVAL                                      (0x00000000U)

/* INTVECTOR_52 */

#define CSL_MSS_VIM_INTVECTOR_52_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_52_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_52_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_52_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_52_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_52_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_52_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_52_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_52_RESETVAL                                      (0x00000000U)

/* INTVECTOR_53 */

#define CSL_MSS_VIM_INTVECTOR_53_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_53_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_53_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_53_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_53_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_53_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_53_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_53_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_53_RESETVAL                                      (0x00000000U)

/* INTVECTOR_54 */

#define CSL_MSS_VIM_INTVECTOR_54_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_54_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_54_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_54_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_54_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_54_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_54_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_54_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_54_RESETVAL                                      (0x00000000U)

/* INTVECTOR_55 */

#define CSL_MSS_VIM_INTVECTOR_55_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_55_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_55_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_55_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_55_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_55_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_55_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_55_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_55_RESETVAL                                      (0x00000000U)

/* INTVECTOR_56 */

#define CSL_MSS_VIM_INTVECTOR_56_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_56_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_56_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_56_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_56_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_56_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_56_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_56_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_56_RESETVAL                                      (0x00000000U)

/* INTVECTOR_57 */

#define CSL_MSS_VIM_INTVECTOR_57_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_57_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_57_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_57_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_57_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_57_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_57_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_57_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_57_RESETVAL                                      (0x00000000U)

/* INTVECTOR_58 */

#define CSL_MSS_VIM_INTVECTOR_58_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_58_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_58_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_58_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_58_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_58_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_58_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_58_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_58_RESETVAL                                      (0x00000000U)

/* INTVECTOR_59 */

#define CSL_MSS_VIM_INTVECTOR_59_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_59_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_59_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_59_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_59_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_59_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_59_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_59_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_59_RESETVAL                                      (0x00000000U)

/* INTVECTOR_60 */

#define CSL_MSS_VIM_INTVECTOR_60_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_60_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_60_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_60_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_60_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_60_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_60_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_60_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_60_RESETVAL                                      (0x00000000U)

/* INTVECTOR_61 */

#define CSL_MSS_VIM_INTVECTOR_61_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_61_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_61_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_61_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_61_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_61_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_61_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_61_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_61_RESETVAL                                      (0x00000000U)

/* INTVECTOR_62 */

#define CSL_MSS_VIM_INTVECTOR_62_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_62_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_62_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_62_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_62_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_62_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_62_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_62_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_62_RESETVAL                                      (0x00000000U)

/* INTVECTOR_63 */

#define CSL_MSS_VIM_INTVECTOR_63_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_63_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_63_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_63_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_63_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_63_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_63_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_63_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_63_RESETVAL                                      (0x00000000U)

/* INTVECTOR_64 */

#define CSL_MSS_VIM_INTVECTOR_64_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_64_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_64_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_64_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_64_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_64_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_64_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_64_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_64_RESETVAL                                      (0x00000000U)

/* INTVECTOR_65 */

#define CSL_MSS_VIM_INTVECTOR_65_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_65_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_65_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_65_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_65_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_65_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_65_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_65_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_65_RESETVAL                                      (0x00000000U)

/* INTVECTOR_66 */

#define CSL_MSS_VIM_INTVECTOR_66_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_66_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_66_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_66_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_66_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_66_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_66_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_66_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_66_RESETVAL                                      (0x00000000U)

/* INTVECTOR_67 */

#define CSL_MSS_VIM_INTVECTOR_67_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_67_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_67_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_67_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_67_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_67_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_67_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_67_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_67_RESETVAL                                      (0x00000000U)

/* INTVECTOR_68 */

#define CSL_MSS_VIM_INTVECTOR_68_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_68_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_68_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_68_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_68_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_68_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_68_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_68_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_68_RESETVAL                                      (0x00000000U)

/* INTVECTOR_69 */

#define CSL_MSS_VIM_INTVECTOR_69_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_69_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_69_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_69_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_69_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_69_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_69_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_69_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_69_RESETVAL                                      (0x00000000U)

/* INTVECTOR_70 */

#define CSL_MSS_VIM_INTVECTOR_70_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_70_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_70_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_70_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_70_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_70_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_70_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_70_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_70_RESETVAL                                      (0x00000000U)

/* INTVECTOR_71 */

#define CSL_MSS_VIM_INTVECTOR_71_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_71_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_71_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_71_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_71_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_71_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_71_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_71_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_71_RESETVAL                                      (0x00000000U)

/* INTVECTOR_72 */

#define CSL_MSS_VIM_INTVECTOR_72_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_72_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_72_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_72_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_72_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_72_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_72_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_72_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_72_RESETVAL                                      (0x00000000U)

/* INTVECTOR_73 */

#define CSL_MSS_VIM_INTVECTOR_73_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_73_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_73_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_73_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_73_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_73_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_73_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_73_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_73_RESETVAL                                      (0x00000000U)

/* INTVECTOR_74 */

#define CSL_MSS_VIM_INTVECTOR_74_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_74_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_74_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_74_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_74_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_74_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_74_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_74_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_74_RESETVAL                                      (0x00000000U)

/* INTVECTOR_75 */

#define CSL_MSS_VIM_INTVECTOR_75_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_75_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_75_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_75_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_75_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_75_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_75_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_75_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_75_RESETVAL                                      (0x00000000U)

/* INTVECTOR_76 */

#define CSL_MSS_VIM_INTVECTOR_76_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_76_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_76_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_76_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_76_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_76_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_76_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_76_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_76_RESETVAL                                      (0x00000000U)

/* INTVECTOR_77 */

#define CSL_MSS_VIM_INTVECTOR_77_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_77_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_77_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_77_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_77_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_77_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_77_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_77_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_77_RESETVAL                                      (0x00000000U)

/* INTVECTOR_78 */

#define CSL_MSS_VIM_INTVECTOR_78_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_78_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_78_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_78_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_78_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_78_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_78_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_78_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_78_RESETVAL                                      (0x00000000U)

/* INTVECTOR_79 */

#define CSL_MSS_VIM_INTVECTOR_79_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_79_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_79_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_79_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_79_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_79_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_79_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_79_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_79_RESETVAL                                      (0x00000000U)

/* INTVECTOR_80 */

#define CSL_MSS_VIM_INTVECTOR_80_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_80_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_80_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_80_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_80_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_80_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_80_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_80_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_80_RESETVAL                                      (0x00000000U)

/* INTVECTOR_81 */

#define CSL_MSS_VIM_INTVECTOR_81_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_81_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_81_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_81_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_81_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_81_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_81_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_81_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_81_RESETVAL                                      (0x00000000U)

/* INTVECTOR_82 */

#define CSL_MSS_VIM_INTVECTOR_82_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_82_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_82_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_82_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_82_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_82_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_82_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_82_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_82_RESETVAL                                      (0x00000000U)

/* INTVECTOR_83 */

#define CSL_MSS_VIM_INTVECTOR_83_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_83_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_83_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_83_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_83_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_83_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_83_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_83_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_83_RESETVAL                                      (0x00000000U)

/* INTVECTOR_84 */

#define CSL_MSS_VIM_INTVECTOR_84_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_84_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_84_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_84_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_84_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_84_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_84_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_84_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_84_RESETVAL                                      (0x00000000U)

/* INTVECTOR_85 */

#define CSL_MSS_VIM_INTVECTOR_85_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_85_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_85_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_85_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_85_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_85_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_85_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_85_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_85_RESETVAL                                      (0x00000000U)

/* INTVECTOR_86 */

#define CSL_MSS_VIM_INTVECTOR_86_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_86_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_86_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_86_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_86_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_86_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_86_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_86_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_86_RESETVAL                                      (0x00000000U)

/* INTVECTOR_87 */

#define CSL_MSS_VIM_INTVECTOR_87_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_87_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_87_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_87_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_87_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_87_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_87_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_87_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_87_RESETVAL                                      (0x00000000U)

/* INTVECTOR_88 */

#define CSL_MSS_VIM_INTVECTOR_88_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_88_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_88_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_88_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_88_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_88_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_88_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_88_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_88_RESETVAL                                      (0x00000000U)

/* INTVECTOR_89 */

#define CSL_MSS_VIM_INTVECTOR_89_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_89_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_89_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_89_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_89_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_89_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_89_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_89_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_89_RESETVAL                                      (0x00000000U)

/* INTVECTOR_90 */

#define CSL_MSS_VIM_INTVECTOR_90_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_90_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_90_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_90_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_90_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_90_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_90_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_90_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_90_RESETVAL                                      (0x00000000U)

/* INTVECTOR_91 */

#define CSL_MSS_VIM_INTVECTOR_91_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_91_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_91_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_91_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_91_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_91_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_91_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_91_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_91_RESETVAL                                      (0x00000000U)

/* INTVECTOR_92 */

#define CSL_MSS_VIM_INTVECTOR_92_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_92_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_92_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_92_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_92_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_92_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_92_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_92_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_92_RESETVAL                                      (0x00000000U)

/* INTVECTOR_93 */

#define CSL_MSS_VIM_INTVECTOR_93_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_93_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_93_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_93_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_93_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_93_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_93_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_93_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_93_RESETVAL                                      (0x00000000U)

/* INTVECTOR_94 */

#define CSL_MSS_VIM_INTVECTOR_94_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_94_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_94_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_94_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_94_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_94_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_94_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_94_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_94_RESETVAL                                      (0x00000000U)

/* INTVECTOR_95 */

#define CSL_MSS_VIM_INTVECTOR_95_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_95_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_95_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_95_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_95_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_95_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_95_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_95_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_95_RESETVAL                                      (0x00000000U)

/* INTVECTOR_96 */

#define CSL_MSS_VIM_INTVECTOR_96_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_96_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_96_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_96_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_96_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_96_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_96_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_96_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_96_RESETVAL                                      (0x00000000U)

/* INTVECTOR_97 */

#define CSL_MSS_VIM_INTVECTOR_97_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_97_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_97_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_97_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_97_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_97_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_97_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_97_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_97_RESETVAL                                      (0x00000000U)

/* INTVECTOR_98 */

#define CSL_MSS_VIM_INTVECTOR_98_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_98_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_98_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_98_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_98_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_98_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_98_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_98_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_98_RESETVAL                                      (0x00000000U)

/* INTVECTOR_99 */

#define CSL_MSS_VIM_INTVECTOR_99_RES20_MASK                                    (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_99_RES20_SHIFT                                   (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_99_RES20_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_99_RES20_MAX                                     (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_99_ADDR_MASK                                     (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_99_ADDR_SHIFT                                    (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_99_ADDR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_99_ADDR_MAX                                      (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_99_RESETVAL                                      (0x00000000U)

/* INTVECTOR_100 */

#define CSL_MSS_VIM_INTVECTOR_100_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_100_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_100_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_100_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_100_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_100_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_100_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_100_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_100_RESETVAL                                     (0x00000000U)

/* INTVECTOR_101 */

#define CSL_MSS_VIM_INTVECTOR_101_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_101_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_101_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_101_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_101_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_101_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_101_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_101_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_101_RESETVAL                                     (0x00000000U)

/* INTVECTOR_102 */

#define CSL_MSS_VIM_INTVECTOR_102_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_102_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_102_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_102_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_102_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_102_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_102_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_102_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_102_RESETVAL                                     (0x00000000U)

/* INTVECTOR_103 */

#define CSL_MSS_VIM_INTVECTOR_103_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_103_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_103_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_103_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_103_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_103_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_103_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_103_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_103_RESETVAL                                     (0x00000000U)

/* INTVECTOR_104 */

#define CSL_MSS_VIM_INTVECTOR_104_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_104_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_104_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_104_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_104_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_104_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_104_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_104_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_104_RESETVAL                                     (0x00000000U)

/* INTVECTOR_105 */

#define CSL_MSS_VIM_INTVECTOR_105_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_105_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_105_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_105_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_105_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_105_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_105_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_105_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_105_RESETVAL                                     (0x00000000U)

/* INTVECTOR_106 */

#define CSL_MSS_VIM_INTVECTOR_106_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_106_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_106_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_106_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_106_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_106_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_106_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_106_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_106_RESETVAL                                     (0x00000000U)

/* INTVECTOR_107 */

#define CSL_MSS_VIM_INTVECTOR_107_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_107_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_107_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_107_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_107_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_107_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_107_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_107_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_107_RESETVAL                                     (0x00000000U)

/* INTVECTOR_108 */

#define CSL_MSS_VIM_INTVECTOR_108_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_108_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_108_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_108_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_108_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_108_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_108_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_108_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_108_RESETVAL                                     (0x00000000U)

/* INTVECTOR_109 */

#define CSL_MSS_VIM_INTVECTOR_109_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_109_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_109_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_109_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_109_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_109_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_109_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_109_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_109_RESETVAL                                     (0x00000000U)

/* INTVECTOR_110 */

#define CSL_MSS_VIM_INTVECTOR_110_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_110_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_110_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_110_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_110_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_110_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_110_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_110_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_110_RESETVAL                                     (0x00000000U)

/* INTVECTOR_111 */

#define CSL_MSS_VIM_INTVECTOR_111_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_111_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_111_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_111_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_111_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_111_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_111_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_111_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_111_RESETVAL                                     (0x00000000U)

/* INTVECTOR_112 */

#define CSL_MSS_VIM_INTVECTOR_112_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_112_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_112_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_112_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_112_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_112_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_112_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_112_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_112_RESETVAL                                     (0x00000000U)

/* INTVECTOR_113 */

#define CSL_MSS_VIM_INTVECTOR_113_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_113_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_113_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_113_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_113_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_113_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_113_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_113_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_113_RESETVAL                                     (0x00000000U)

/* INTVECTOR_114 */

#define CSL_MSS_VIM_INTVECTOR_114_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_114_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_114_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_114_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_114_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_114_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_114_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_114_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_114_RESETVAL                                     (0x00000000U)

/* INTVECTOR_115 */

#define CSL_MSS_VIM_INTVECTOR_115_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_115_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_115_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_115_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_115_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_115_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_115_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_115_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_115_RESETVAL                                     (0x00000000U)

/* INTVECTOR_116 */

#define CSL_MSS_VIM_INTVECTOR_116_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_116_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_116_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_116_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_116_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_116_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_116_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_116_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_116_RESETVAL                                     (0x00000000U)

/* INTVECTOR_117 */

#define CSL_MSS_VIM_INTVECTOR_117_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_117_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_117_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_117_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_117_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_117_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_117_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_117_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_117_RESETVAL                                     (0x00000000U)

/* INTVECTOR_118 */

#define CSL_MSS_VIM_INTVECTOR_118_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_118_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_118_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_118_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_118_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_118_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_118_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_118_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_118_RESETVAL                                     (0x00000000U)

/* INTVECTOR_119 */

#define CSL_MSS_VIM_INTVECTOR_119_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_119_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_119_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_119_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_119_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_119_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_119_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_119_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_119_RESETVAL                                     (0x00000000U)

/* INTVECTOR_120 */

#define CSL_MSS_VIM_INTVECTOR_120_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_120_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_120_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_120_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_120_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_120_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_120_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_120_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_120_RESETVAL                                     (0x00000000U)

/* INTVECTOR_121 */

#define CSL_MSS_VIM_INTVECTOR_121_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_121_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_121_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_121_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_121_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_121_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_121_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_121_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_121_RESETVAL                                     (0x00000000U)

/* INTVECTOR_122 */

#define CSL_MSS_VIM_INTVECTOR_122_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_122_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_122_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_122_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_122_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_122_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_122_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_122_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_122_RESETVAL                                     (0x00000000U)

/* INTVECTOR_123 */

#define CSL_MSS_VIM_INTVECTOR_123_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_123_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_123_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_123_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_123_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_123_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_123_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_123_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_123_RESETVAL                                     (0x00000000U)

/* INTVECTOR_124 */

#define CSL_MSS_VIM_INTVECTOR_124_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_124_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_124_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_124_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_124_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_124_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_124_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_124_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_124_RESETVAL                                     (0x00000000U)

/* INTVECTOR_125 */

#define CSL_MSS_VIM_INTVECTOR_125_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_125_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_125_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_125_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_125_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_125_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_125_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_125_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_125_RESETVAL                                     (0x00000000U)

/* INTVECTOR_126 */

#define CSL_MSS_VIM_INTVECTOR_126_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_126_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_126_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_126_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_126_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_126_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_126_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_126_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_126_RESETVAL                                     (0x00000000U)

/* INTVECTOR_127 */

#define CSL_MSS_VIM_INTVECTOR_127_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_127_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_127_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_127_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_127_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_127_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_127_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_127_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_127_RESETVAL                                     (0x00000000U)

/* INTVECTOR_128 */

#define CSL_MSS_VIM_INTVECTOR_128_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_128_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_128_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_128_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_128_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_128_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_128_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_128_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_128_RESETVAL                                     (0x00000000U)

/* INTVECTOR_129 */

#define CSL_MSS_VIM_INTVECTOR_129_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_129_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_129_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_129_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_129_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_129_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_129_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_129_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_129_RESETVAL                                     (0x00000000U)

/* INTVECTOR_130 */

#define CSL_MSS_VIM_INTVECTOR_130_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_130_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_130_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_130_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_130_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_130_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_130_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_130_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_130_RESETVAL                                     (0x00000000U)

/* INTVECTOR_131 */

#define CSL_MSS_VIM_INTVECTOR_131_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_131_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_131_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_131_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_131_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_131_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_131_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_131_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_131_RESETVAL                                     (0x00000000U)

/* INTVECTOR_132 */

#define CSL_MSS_VIM_INTVECTOR_132_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_132_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_132_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_132_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_132_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_132_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_132_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_132_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_132_RESETVAL                                     (0x00000000U)

/* INTVECTOR_133 */

#define CSL_MSS_VIM_INTVECTOR_133_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_133_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_133_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_133_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_133_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_133_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_133_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_133_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_133_RESETVAL                                     (0x00000000U)

/* INTVECTOR_134 */

#define CSL_MSS_VIM_INTVECTOR_134_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_134_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_134_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_134_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_134_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_134_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_134_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_134_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_134_RESETVAL                                     (0x00000000U)

/* INTVECTOR_135 */

#define CSL_MSS_VIM_INTVECTOR_135_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_135_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_135_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_135_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_135_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_135_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_135_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_135_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_135_RESETVAL                                     (0x00000000U)

/* INTVECTOR_136 */

#define CSL_MSS_VIM_INTVECTOR_136_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_136_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_136_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_136_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_136_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_136_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_136_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_136_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_136_RESETVAL                                     (0x00000000U)

/* INTVECTOR_137 */

#define CSL_MSS_VIM_INTVECTOR_137_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_137_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_137_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_137_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_137_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_137_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_137_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_137_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_137_RESETVAL                                     (0x00000000U)

/* INTVECTOR_138 */

#define CSL_MSS_VIM_INTVECTOR_138_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_138_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_138_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_138_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_138_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_138_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_138_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_138_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_138_RESETVAL                                     (0x00000000U)

/* INTVECTOR_139 */

#define CSL_MSS_VIM_INTVECTOR_139_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_139_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_139_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_139_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_139_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_139_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_139_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_139_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_139_RESETVAL                                     (0x00000000U)

/* INTVECTOR_140 */

#define CSL_MSS_VIM_INTVECTOR_140_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_140_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_140_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_140_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_140_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_140_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_140_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_140_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_140_RESETVAL                                     (0x00000000U)

/* INTVECTOR_141 */

#define CSL_MSS_VIM_INTVECTOR_141_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_141_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_141_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_141_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_141_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_141_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_141_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_141_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_141_RESETVAL                                     (0x00000000U)

/* INTVECTOR_142 */

#define CSL_MSS_VIM_INTVECTOR_142_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_142_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_142_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_142_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_142_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_142_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_142_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_142_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_142_RESETVAL                                     (0x00000000U)

/* INTVECTOR_143 */

#define CSL_MSS_VIM_INTVECTOR_143_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_143_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_143_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_143_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_143_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_143_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_143_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_143_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_143_RESETVAL                                     (0x00000000U)

/* INTVECTOR_144 */

#define CSL_MSS_VIM_INTVECTOR_144_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_144_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_144_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_144_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_144_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_144_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_144_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_144_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_144_RESETVAL                                     (0x00000000U)

/* INTVECTOR_145 */

#define CSL_MSS_VIM_INTVECTOR_145_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_145_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_145_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_145_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_145_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_145_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_145_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_145_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_145_RESETVAL                                     (0x00000000U)

/* INTVECTOR_146 */

#define CSL_MSS_VIM_INTVECTOR_146_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_146_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_146_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_146_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_146_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_146_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_146_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_146_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_146_RESETVAL                                     (0x00000000U)

/* INTVECTOR_147 */

#define CSL_MSS_VIM_INTVECTOR_147_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_147_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_147_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_147_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_147_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_147_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_147_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_147_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_147_RESETVAL                                     (0x00000000U)

/* INTVECTOR_148 */

#define CSL_MSS_VIM_INTVECTOR_148_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_148_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_148_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_148_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_148_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_148_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_148_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_148_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_148_RESETVAL                                     (0x00000000U)

/* INTVECTOR_149 */

#define CSL_MSS_VIM_INTVECTOR_149_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_149_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_149_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_149_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_149_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_149_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_149_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_149_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_149_RESETVAL                                     (0x00000000U)

/* INTVECTOR_150 */

#define CSL_MSS_VIM_INTVECTOR_150_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_150_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_150_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_150_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_150_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_150_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_150_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_150_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_150_RESETVAL                                     (0x00000000U)

/* INTVECTOR_151 */

#define CSL_MSS_VIM_INTVECTOR_151_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_151_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_151_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_151_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_151_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_151_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_151_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_151_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_151_RESETVAL                                     (0x00000000U)

/* INTVECTOR_152 */

#define CSL_MSS_VIM_INTVECTOR_152_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_152_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_152_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_152_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_152_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_152_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_152_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_152_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_152_RESETVAL                                     (0x00000000U)

/* INTVECTOR_153 */

#define CSL_MSS_VIM_INTVECTOR_153_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_153_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_153_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_153_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_153_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_153_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_153_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_153_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_153_RESETVAL                                     (0x00000000U)

/* INTVECTOR_154 */

#define CSL_MSS_VIM_INTVECTOR_154_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_154_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_154_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_154_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_154_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_154_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_154_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_154_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_154_RESETVAL                                     (0x00000000U)

/* INTVECTOR_155 */

#define CSL_MSS_VIM_INTVECTOR_155_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_155_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_155_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_155_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_155_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_155_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_155_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_155_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_155_RESETVAL                                     (0x00000000U)

/* INTVECTOR_156 */

#define CSL_MSS_VIM_INTVECTOR_156_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_156_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_156_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_156_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_156_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_156_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_156_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_156_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_156_RESETVAL                                     (0x00000000U)

/* INTVECTOR_157 */

#define CSL_MSS_VIM_INTVECTOR_157_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_157_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_157_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_157_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_157_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_157_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_157_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_157_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_157_RESETVAL                                     (0x00000000U)

/* INTVECTOR_158 */

#define CSL_MSS_VIM_INTVECTOR_158_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_158_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_158_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_158_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_158_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_158_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_158_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_158_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_158_RESETVAL                                     (0x00000000U)

/* INTVECTOR_159 */

#define CSL_MSS_VIM_INTVECTOR_159_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_159_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_159_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_159_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_159_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_159_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_159_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_159_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_159_RESETVAL                                     (0x00000000U)

/* INTVECTOR_160 */

#define CSL_MSS_VIM_INTVECTOR_160_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_160_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_160_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_160_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_160_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_160_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_160_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_160_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_160_RESETVAL                                     (0x00000000U)

/* INTVECTOR_161 */

#define CSL_MSS_VIM_INTVECTOR_161_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_161_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_161_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_161_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_161_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_161_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_161_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_161_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_161_RESETVAL                                     (0x00000000U)

/* INTVECTOR_162 */

#define CSL_MSS_VIM_INTVECTOR_162_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_162_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_162_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_162_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_162_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_162_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_162_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_162_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_162_RESETVAL                                     (0x00000000U)

/* INTVECTOR_163 */

#define CSL_MSS_VIM_INTVECTOR_163_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_163_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_163_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_163_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_163_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_163_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_163_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_163_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_163_RESETVAL                                     (0x00000000U)

/* INTVECTOR_164 */

#define CSL_MSS_VIM_INTVECTOR_164_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_164_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_164_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_164_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_164_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_164_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_164_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_164_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_164_RESETVAL                                     (0x00000000U)

/* INTVECTOR_165 */

#define CSL_MSS_VIM_INTVECTOR_165_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_165_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_165_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_165_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_165_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_165_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_165_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_165_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_165_RESETVAL                                     (0x00000000U)

/* INTVECTOR_166 */

#define CSL_MSS_VIM_INTVECTOR_166_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_166_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_166_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_166_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_166_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_166_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_166_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_166_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_166_RESETVAL                                     (0x00000000U)

/* INTVECTOR_167 */

#define CSL_MSS_VIM_INTVECTOR_167_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_167_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_167_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_167_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_167_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_167_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_167_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_167_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_167_RESETVAL                                     (0x00000000U)

/* INTVECTOR_168 */

#define CSL_MSS_VIM_INTVECTOR_168_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_168_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_168_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_168_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_168_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_168_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_168_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_168_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_168_RESETVAL                                     (0x00000000U)

/* INTVECTOR_169 */

#define CSL_MSS_VIM_INTVECTOR_169_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_169_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_169_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_169_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_169_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_169_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_169_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_169_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_169_RESETVAL                                     (0x00000000U)

/* INTVECTOR_170 */

#define CSL_MSS_VIM_INTVECTOR_170_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_170_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_170_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_170_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_170_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_170_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_170_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_170_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_170_RESETVAL                                     (0x00000000U)

/* INTVECTOR_171 */

#define CSL_MSS_VIM_INTVECTOR_171_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_171_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_171_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_171_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_171_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_171_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_171_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_171_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_171_RESETVAL                                     (0x00000000U)

/* INTVECTOR_172 */

#define CSL_MSS_VIM_INTVECTOR_172_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_172_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_172_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_172_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_172_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_172_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_172_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_172_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_172_RESETVAL                                     (0x00000000U)

/* INTVECTOR_173 */

#define CSL_MSS_VIM_INTVECTOR_173_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_173_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_173_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_173_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_173_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_173_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_173_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_173_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_173_RESETVAL                                     (0x00000000U)

/* INTVECTOR_174 */

#define CSL_MSS_VIM_INTVECTOR_174_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_174_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_174_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_174_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_174_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_174_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_174_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_174_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_174_RESETVAL                                     (0x00000000U)

/* INTVECTOR_175 */

#define CSL_MSS_VIM_INTVECTOR_175_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_175_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_175_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_175_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_175_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_175_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_175_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_175_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_175_RESETVAL                                     (0x00000000U)

/* INTVECTOR_176 */

#define CSL_MSS_VIM_INTVECTOR_176_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_176_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_176_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_176_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_176_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_176_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_176_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_176_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_176_RESETVAL                                     (0x00000000U)

/* INTVECTOR_177 */

#define CSL_MSS_VIM_INTVECTOR_177_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_177_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_177_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_177_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_177_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_177_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_177_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_177_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_177_RESETVAL                                     (0x00000000U)

/* INTVECTOR_178 */

#define CSL_MSS_VIM_INTVECTOR_178_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_178_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_178_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_178_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_178_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_178_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_178_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_178_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_178_RESETVAL                                     (0x00000000U)

/* INTVECTOR_179 */

#define CSL_MSS_VIM_INTVECTOR_179_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_179_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_179_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_179_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_179_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_179_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_179_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_179_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_179_RESETVAL                                     (0x00000000U)

/* INTVECTOR_180 */

#define CSL_MSS_VIM_INTVECTOR_180_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_180_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_180_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_180_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_180_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_180_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_180_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_180_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_180_RESETVAL                                     (0x00000000U)

/* INTVECTOR_181 */

#define CSL_MSS_VIM_INTVECTOR_181_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_181_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_181_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_181_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_181_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_181_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_181_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_181_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_181_RESETVAL                                     (0x00000000U)

/* INTVECTOR_182 */

#define CSL_MSS_VIM_INTVECTOR_182_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_182_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_182_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_182_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_182_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_182_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_182_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_182_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_182_RESETVAL                                     (0x00000000U)

/* INTVECTOR_183 */

#define CSL_MSS_VIM_INTVECTOR_183_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_183_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_183_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_183_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_183_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_183_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_183_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_183_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_183_RESETVAL                                     (0x00000000U)

/* INTVECTOR_184 */

#define CSL_MSS_VIM_INTVECTOR_184_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_184_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_184_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_184_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_184_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_184_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_184_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_184_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_184_RESETVAL                                     (0x00000000U)

/* INTVECTOR_185 */

#define CSL_MSS_VIM_INTVECTOR_185_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_185_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_185_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_185_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_185_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_185_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_185_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_185_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_185_RESETVAL                                     (0x00000000U)

/* INTVECTOR_186 */

#define CSL_MSS_VIM_INTVECTOR_186_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_186_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_186_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_186_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_186_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_186_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_186_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_186_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_186_RESETVAL                                     (0x00000000U)

/* INTVECTOR_187 */

#define CSL_MSS_VIM_INTVECTOR_187_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_187_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_187_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_187_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_187_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_187_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_187_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_187_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_187_RESETVAL                                     (0x00000000U)

/* INTVECTOR_188 */

#define CSL_MSS_VIM_INTVECTOR_188_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_188_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_188_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_188_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_188_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_188_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_188_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_188_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_188_RESETVAL                                     (0x00000000U)

/* INTVECTOR_189 */

#define CSL_MSS_VIM_INTVECTOR_189_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_189_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_189_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_189_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_189_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_189_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_189_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_189_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_189_RESETVAL                                     (0x00000000U)

/* INTVECTOR_190 */

#define CSL_MSS_VIM_INTVECTOR_190_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_190_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_190_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_190_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_190_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_190_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_190_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_190_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_190_RESETVAL                                     (0x00000000U)

/* INTVECTOR_191 */

#define CSL_MSS_VIM_INTVECTOR_191_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_191_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_191_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_191_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_191_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_191_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_191_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_191_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_191_RESETVAL                                     (0x00000000U)

/* INTVECTOR_192 */

#define CSL_MSS_VIM_INTVECTOR_192_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_192_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_192_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_192_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_192_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_192_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_192_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_192_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_192_RESETVAL                                     (0x00000000U)

/* INTVECTOR_193 */

#define CSL_MSS_VIM_INTVECTOR_193_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_193_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_193_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_193_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_193_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_193_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_193_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_193_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_193_RESETVAL                                     (0x00000000U)

/* INTVECTOR_194 */

#define CSL_MSS_VIM_INTVECTOR_194_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_194_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_194_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_194_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_194_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_194_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_194_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_194_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_194_RESETVAL                                     (0x00000000U)

/* INTVECTOR_195 */

#define CSL_MSS_VIM_INTVECTOR_195_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_195_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_195_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_195_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_195_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_195_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_195_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_195_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_195_RESETVAL                                     (0x00000000U)

/* INTVECTOR_196 */

#define CSL_MSS_VIM_INTVECTOR_196_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_196_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_196_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_196_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_196_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_196_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_196_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_196_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_196_RESETVAL                                     (0x00000000U)

/* INTVECTOR_197 */

#define CSL_MSS_VIM_INTVECTOR_197_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_197_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_197_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_197_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_197_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_197_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_197_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_197_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_197_RESETVAL                                     (0x00000000U)

/* INTVECTOR_198 */

#define CSL_MSS_VIM_INTVECTOR_198_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_198_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_198_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_198_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_198_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_198_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_198_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_198_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_198_RESETVAL                                     (0x00000000U)

/* INTVECTOR_199 */

#define CSL_MSS_VIM_INTVECTOR_199_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_199_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_199_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_199_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_199_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_199_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_199_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_199_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_199_RESETVAL                                     (0x00000000U)

/* INTVECTOR_200 */

#define CSL_MSS_VIM_INTVECTOR_200_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_200_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_200_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_200_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_200_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_200_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_200_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_200_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_200_RESETVAL                                     (0x00000000U)

/* INTVECTOR_201 */

#define CSL_MSS_VIM_INTVECTOR_201_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_201_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_201_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_201_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_201_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_201_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_201_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_201_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_201_RESETVAL                                     (0x00000000U)

/* INTVECTOR_202 */

#define CSL_MSS_VIM_INTVECTOR_202_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_202_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_202_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_202_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_202_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_202_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_202_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_202_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_202_RESETVAL                                     (0x00000000U)

/* INTVECTOR_203 */

#define CSL_MSS_VIM_INTVECTOR_203_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_203_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_203_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_203_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_203_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_203_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_203_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_203_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_203_RESETVAL                                     (0x00000000U)

/* INTVECTOR_204 */

#define CSL_MSS_VIM_INTVECTOR_204_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_204_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_204_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_204_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_204_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_204_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_204_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_204_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_204_RESETVAL                                     (0x00000000U)

/* INTVECTOR_205 */

#define CSL_MSS_VIM_INTVECTOR_205_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_205_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_205_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_205_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_205_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_205_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_205_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_205_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_205_RESETVAL                                     (0x00000000U)

/* INTVECTOR_206 */

#define CSL_MSS_VIM_INTVECTOR_206_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_206_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_206_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_206_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_206_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_206_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_206_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_206_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_206_RESETVAL                                     (0x00000000U)

/* INTVECTOR_207 */

#define CSL_MSS_VIM_INTVECTOR_207_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_207_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_207_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_207_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_207_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_207_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_207_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_207_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_207_RESETVAL                                     (0x00000000U)

/* INTVECTOR_208 */

#define CSL_MSS_VIM_INTVECTOR_208_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_208_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_208_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_208_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_208_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_208_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_208_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_208_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_208_RESETVAL                                     (0x00000000U)

/* INTVECTOR_209 */

#define CSL_MSS_VIM_INTVECTOR_209_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_209_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_209_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_209_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_209_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_209_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_209_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_209_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_209_RESETVAL                                     (0x00000000U)

/* INTVECTOR_210 */

#define CSL_MSS_VIM_INTVECTOR_210_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_210_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_210_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_210_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_210_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_210_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_210_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_210_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_210_RESETVAL                                     (0x00000000U)

/* INTVECTOR_211 */

#define CSL_MSS_VIM_INTVECTOR_211_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_211_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_211_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_211_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_211_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_211_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_211_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_211_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_211_RESETVAL                                     (0x00000000U)

/* INTVECTOR_212 */

#define CSL_MSS_VIM_INTVECTOR_212_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_212_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_212_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_212_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_212_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_212_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_212_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_212_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_212_RESETVAL                                     (0x00000000U)

/* INTVECTOR_213 */

#define CSL_MSS_VIM_INTVECTOR_213_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_213_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_213_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_213_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_213_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_213_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_213_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_213_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_213_RESETVAL                                     (0x00000000U)

/* INTVECTOR_214 */

#define CSL_MSS_VIM_INTVECTOR_214_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_214_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_214_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_214_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_214_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_214_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_214_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_214_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_214_RESETVAL                                     (0x00000000U)

/* INTVECTOR_215 */

#define CSL_MSS_VIM_INTVECTOR_215_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_215_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_215_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_215_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_215_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_215_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_215_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_215_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_215_RESETVAL                                     (0x00000000U)

/* INTVECTOR_216 */

#define CSL_MSS_VIM_INTVECTOR_216_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_216_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_216_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_216_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_216_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_216_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_216_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_216_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_216_RESETVAL                                     (0x00000000U)

/* INTVECTOR_217 */

#define CSL_MSS_VIM_INTVECTOR_217_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_217_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_217_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_217_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_217_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_217_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_217_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_217_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_217_RESETVAL                                     (0x00000000U)

/* INTVECTOR_218 */

#define CSL_MSS_VIM_INTVECTOR_218_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_218_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_218_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_218_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_218_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_218_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_218_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_218_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_218_RESETVAL                                     (0x00000000U)

/* INTVECTOR_219 */

#define CSL_MSS_VIM_INTVECTOR_219_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_219_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_219_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_219_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_219_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_219_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_219_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_219_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_219_RESETVAL                                     (0x00000000U)

/* INTVECTOR_220 */

#define CSL_MSS_VIM_INTVECTOR_220_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_220_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_220_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_220_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_220_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_220_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_220_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_220_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_220_RESETVAL                                     (0x00000000U)

/* INTVECTOR_221 */

#define CSL_MSS_VIM_INTVECTOR_221_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_221_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_221_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_221_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_221_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_221_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_221_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_221_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_221_RESETVAL                                     (0x00000000U)

/* INTVECTOR_222 */

#define CSL_MSS_VIM_INTVECTOR_222_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_222_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_222_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_222_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_222_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_222_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_222_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_222_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_222_RESETVAL                                     (0x00000000U)

/* INTVECTOR_223 */

#define CSL_MSS_VIM_INTVECTOR_223_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_223_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_223_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_223_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_223_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_223_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_223_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_223_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_223_RESETVAL                                     (0x00000000U)

/* INTVECTOR_224 */

#define CSL_MSS_VIM_INTVECTOR_224_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_224_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_224_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_224_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_224_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_224_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_224_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_224_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_224_RESETVAL                                     (0x00000000U)

/* INTVECTOR_225 */

#define CSL_MSS_VIM_INTVECTOR_225_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_225_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_225_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_225_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_225_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_225_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_225_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_225_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_225_RESETVAL                                     (0x00000000U)

/* INTVECTOR_226 */

#define CSL_MSS_VIM_INTVECTOR_226_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_226_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_226_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_226_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_226_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_226_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_226_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_226_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_226_RESETVAL                                     (0x00000000U)

/* INTVECTOR_227 */

#define CSL_MSS_VIM_INTVECTOR_227_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_227_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_227_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_227_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_227_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_227_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_227_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_227_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_227_RESETVAL                                     (0x00000000U)

/* INTVECTOR_228 */

#define CSL_MSS_VIM_INTVECTOR_228_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_228_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_228_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_228_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_228_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_228_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_228_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_228_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_228_RESETVAL                                     (0x00000000U)

/* INTVECTOR_229 */

#define CSL_MSS_VIM_INTVECTOR_229_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_229_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_229_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_229_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_229_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_229_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_229_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_229_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_229_RESETVAL                                     (0x00000000U)

/* INTVECTOR_230 */

#define CSL_MSS_VIM_INTVECTOR_230_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_230_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_230_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_230_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_230_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_230_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_230_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_230_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_230_RESETVAL                                     (0x00000000U)

/* INTVECTOR_231 */

#define CSL_MSS_VIM_INTVECTOR_231_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_231_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_231_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_231_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_231_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_231_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_231_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_231_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_231_RESETVAL                                     (0x00000000U)

/* INTVECTOR_232 */

#define CSL_MSS_VIM_INTVECTOR_232_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_232_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_232_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_232_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_232_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_232_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_232_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_232_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_232_RESETVAL                                     (0x00000000U)

/* INTVECTOR_233 */

#define CSL_MSS_VIM_INTVECTOR_233_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_233_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_233_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_233_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_233_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_233_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_233_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_233_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_233_RESETVAL                                     (0x00000000U)

/* INTVECTOR_234 */

#define CSL_MSS_VIM_INTVECTOR_234_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_234_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_234_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_234_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_234_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_234_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_234_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_234_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_234_RESETVAL                                     (0x00000000U)

/* INTVECTOR_235 */

#define CSL_MSS_VIM_INTVECTOR_235_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_235_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_235_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_235_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_235_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_235_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_235_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_235_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_235_RESETVAL                                     (0x00000000U)

/* INTVECTOR_236 */

#define CSL_MSS_VIM_INTVECTOR_236_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_236_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_236_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_236_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_236_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_236_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_236_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_236_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_236_RESETVAL                                     (0x00000000U)

/* INTVECTOR_237 */

#define CSL_MSS_VIM_INTVECTOR_237_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_237_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_237_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_237_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_237_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_237_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_237_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_237_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_237_RESETVAL                                     (0x00000000U)

/* INTVECTOR_238 */

#define CSL_MSS_VIM_INTVECTOR_238_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_238_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_238_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_238_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_238_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_238_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_238_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_238_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_238_RESETVAL                                     (0x00000000U)

/* INTVECTOR_239 */

#define CSL_MSS_VIM_INTVECTOR_239_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_239_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_239_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_239_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_239_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_239_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_239_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_239_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_239_RESETVAL                                     (0x00000000U)

/* INTVECTOR_240 */

#define CSL_MSS_VIM_INTVECTOR_240_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_240_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_240_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_240_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_240_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_240_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_240_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_240_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_240_RESETVAL                                     (0x00000000U)

/* INTVECTOR_241 */

#define CSL_MSS_VIM_INTVECTOR_241_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_241_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_241_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_241_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_241_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_241_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_241_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_241_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_241_RESETVAL                                     (0x00000000U)

/* INTVECTOR_242 */

#define CSL_MSS_VIM_INTVECTOR_242_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_242_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_242_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_242_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_242_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_242_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_242_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_242_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_242_RESETVAL                                     (0x00000000U)

/* INTVECTOR_243 */

#define CSL_MSS_VIM_INTVECTOR_243_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_243_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_243_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_243_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_243_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_243_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_243_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_243_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_243_RESETVAL                                     (0x00000000U)

/* INTVECTOR_244 */

#define CSL_MSS_VIM_INTVECTOR_244_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_244_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_244_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_244_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_244_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_244_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_244_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_244_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_244_RESETVAL                                     (0x00000000U)

/* INTVECTOR_245 */

#define CSL_MSS_VIM_INTVECTOR_245_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_245_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_245_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_245_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_245_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_245_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_245_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_245_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_245_RESETVAL                                     (0x00000000U)

/* INTVECTOR_246 */

#define CSL_MSS_VIM_INTVECTOR_246_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_246_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_246_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_246_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_246_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_246_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_246_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_246_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_246_RESETVAL                                     (0x00000000U)

/* INTVECTOR_247 */

#define CSL_MSS_VIM_INTVECTOR_247_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_247_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_247_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_247_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_247_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_247_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_247_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_247_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_247_RESETVAL                                     (0x00000000U)

/* INTVECTOR_248 */

#define CSL_MSS_VIM_INTVECTOR_248_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_248_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_248_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_248_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_248_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_248_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_248_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_248_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_248_RESETVAL                                     (0x00000000U)

/* INTVECTOR_249 */

#define CSL_MSS_VIM_INTVECTOR_249_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_249_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_249_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_249_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_249_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_249_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_249_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_249_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_249_RESETVAL                                     (0x00000000U)

/* INTVECTOR_250 */

#define CSL_MSS_VIM_INTVECTOR_250_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_250_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_250_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_250_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_250_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_250_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_250_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_250_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_250_RESETVAL                                     (0x00000000U)

/* INTVECTOR_251 */

#define CSL_MSS_VIM_INTVECTOR_251_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_251_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_251_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_251_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_251_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_251_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_251_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_251_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_251_RESETVAL                                     (0x00000000U)

/* INTVECTOR_252 */

#define CSL_MSS_VIM_INTVECTOR_252_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_252_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_252_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_252_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_252_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_252_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_252_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_252_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_252_RESETVAL                                     (0x00000000U)

/* INTVECTOR_253 */

#define CSL_MSS_VIM_INTVECTOR_253_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_253_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_253_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_253_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_253_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_253_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_253_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_253_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_253_RESETVAL                                     (0x00000000U)

/* INTVECTOR_254 */

#define CSL_MSS_VIM_INTVECTOR_254_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_254_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_254_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_254_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_254_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_254_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_254_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_254_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_254_RESETVAL                                     (0x00000000U)

/* INTVECTOR_255 */

#define CSL_MSS_VIM_INTVECTOR_255_RES20_MASK                                   (0x00000003U)
#define CSL_MSS_VIM_INTVECTOR_255_RES20_SHIFT                                  (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_255_RES20_RESETVAL                               (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_255_RES20_MAX                                    (0x00000003U)

#define CSL_MSS_VIM_INTVECTOR_255_ADDR_MASK                                    (0xFFFFFFFCU)
#define CSL_MSS_VIM_INTVECTOR_255_ADDR_SHIFT                                   (0x00000002U)
#define CSL_MSS_VIM_INTVECTOR_255_ADDR_RESETVAL                                (0x00000000U)
#define CSL_MSS_VIM_INTVECTOR_255_ADDR_MAX                                     (0x3FFFFFFFU)

#define CSL_MSS_VIM_INTVECTOR_255_RESETVAL                                     (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
