/*
 * Copyright (C) 2021 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *  * Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

	.section	.textTrig, "axw",  %progbits
	.arm
	.code 32
	.global	ti_r5fmath_atan2
ti_r5fmath_atan2:
        MOVW      A2, #0                // [DPU_V7A8_PIPE0] |476|
        VABS.F32  S6, S0                // y
        MOVT      A2, #0                // A2 = segment, set to 0
        VABS.F32  S11, S1               // x
        VMOV      S5, A2                // [DPU_V7A8_PIPE0] |476|
        VCMPE.F32 S11, S6               // abs(x) > abs(y)
        MOV       r12, #0                // load 0
        VMOV.F32  S10, S5               //S5 = S10 = 0
        VMRS APSR_nzcv, FPSCR 			//
        VLDR.32   S6, [A1, #8]          // PI/6
        VLDR.32   S4, [A1, #12]         // PI/2
        MOV       A2, r12                // load 0
        VLDR.32   S11, [A1, #16]        // PI
        VDIVLE.F32 S14, S1, S0          // inVal = S14 = x / y    (save x and y for comparisons later)
        VDIVGT.F32 S14, S0, S1          // inVal = S14 = y / x

        VLDR.32   S9, [A1, #28]         // c3
        VLDR.32   S8, [A1, #0]          // tan(PI/12)
        VLDR.32   S3, [A1, #20]         // c1
        VLDR.32   S2, [A1, #24]         // c2
        VLDR.32   S7, [A1, #4]          // tan(PI/6)

     	MOVLE      A1, #1               // A1 = complement = 1  (y > x)
        MOVGT	   A1, #0

/* --------------- set k1 constant in y > x case --------------------------- */
        VCMPE.F32 S0, S10 				// set k1 on y +/-, will have to check x later
        VMRS APSR_nzcv, FPSCR
        VNEGLT.F32		S4, S4				// S4 hold PI/2 so reverse sign for later if y<0

/* --------------- set k2 constant in x >= y case ---------------------------*/
        VCMPE.F32 	S0, S10 				// set k2 on y +/-, will have to check x later
        VMRS 		APSR_nzcv, FPSCR
        VNEGLT.F32		S11, S11			// S11 holds PI so reverse sign for later if y<0
        VCMPE.F32 	S1, S10 				// we only adjust by PI is x < 0 so set S11 to 0 if x is positive
        VMRS 		APSR_nzcv, FPSCR
        VMOVGT.F32	S11, S10				// S11 holds PI so reverse sign for later if y<0

/* --------------- set sign if inVal < 0 ---------------------------*/
 		VCMPE.F32 S14, S10 					// set sign on inVal +/-, will have to check x later
        VMRS APSR_nzcv, FPSCR
		MOVGE	   A3, #0					// sign = 0
		MOVLT	   A3, #1					// sign = 1


/* --------------------------------------------------------------------------*/
        VABS.F32  S14, S14                	//
        VMRS APSR_nzcv, FPSCR 			  	//
        VCMPE.F32 S14, S8                	// abs(inVal) > tan(PI/12)
        VMRS APSR_nzcv, FPSCR 				//

/* --------------------------------------------------------------------------*/
        VMOV.F32  S0, #1.00000000000000000000e+00
        MOVGT       r12, #1              	// segment
        VMLAGT.F32  S0, S14, S7            	//
        VSUBGT.F32  S14, S14, S7            //
        VDIVGT.F32  S14, S14, S0            //
/* --------------------------------------------------------------------------*/
        VMUL.F32  S0, S14, S14            	//
        //CMP       r12, #0                	// check segment - segment check still in place
        VMLA.F32  S2, S9, S0            	// c2 + x^2 c3
        VMLA.F32  S3, S0, S2            	// c1 + x^2*(c2 + x^2 c3)
        VMUL.F32  S0, S14, S3            	// x*(c1 + x^2*(c2 + x^2 c3))


        VADDGT.F32 S0, S6, S0           	// add PI/6 if segment

        CMP       A3, #0               		// sign
        VNEGNE.F32 S0, S0               	//

        CMP       A1, #0                	// complement
        VSUBNE.F32 S0, S4, S0           	// complement -> subtract from k1
        VADDEQ.F32 S0, S11, S0           	// complement ->  add k2

        BX        LR                    	//


    .section	.textTrig, "axw",  %progbits
	.arm
	.code 32
	.global	ti_r5fmath_atan
ti_r5fmath_atan:
        VMOV.F32  S1, #1.00000000000000000000e+00
        VABS.F32  S4, S0                	//
        VCMPE.F32 S4, S1                	//

	    VMRS APSR_nzcv, FPSCR 				//
	    VDIVGT.F32 S0, S1, S0           	// S0 = x or 1/x
	    VLDM.32	  r0, {S2-S8}
	    VCMPE.F32 S0, #0.0              	//
	    EOR		  A1, A1, A1				//
	    VMOV	  S14, A1	    			//
	    MOVLE     A1, #0                	//
	    MOVGT     A1, #1                	// A1 = complement

	    VMRS APSR_nzcv, FPSCR 				//
	    VNEGCC.F32 S0, S0               	//
	    MOVCS     r12, #0                	//
	    VABS.F32  S9, S0                	//
	    MOVCC     r12, #1                	//
	    VCMPE.F32 S9, S8                	// check against tan(PI/12)
        VMRS APSR_nzcv, FPSCR 				//
/* --------------------------------------------------------------------------*/
	    VMLAGT.F32  S1, S0, S7            	// 1 + x * tan(PI/6)
        VSUBGT.F32  S0, S0, S7            	// x - tan(PI/6)
        VDIVGT.F32  S0, S0, S1            	// x = (x - tan(PI/6)) / (1 + x* tan(PI/6) )
        VMOVGT.F32	S14, S6				 	// will add either 0 or PI/6 at last step

	    VMUL.F32  S1, S0, S0            	//
	    CMP       A1, #0                	//
	    VMLA.F32  S2, S5, S1            	//
        VMLA.F32  S3, S1, S2            	//
        VMLA.F32  S14, S0, S3            	//
        VMOV.F32  S0, S14                	//

/* --------------------------------------------------------------------------*/

	    VSUBNE.F32 S0, S4, S0           	//
	    CMP       r12, #0                	//
	    VNEGNE.F32 S0, S0               	//

        BX        LR                    	//


	.section	.textTrig, "axw",  %progbits
	.arm
	.code 32
	.global	asm_sincos10


	.section	.textTrig, "axw",  %progbits
	.arm
	.code 32
	.global	ti_r5fmath_atanFast
ti_r5fmath_atanFast:
        VMOV.F32  S1, #1.00000000000000000000e+00
        MOV		  r3, #0						// complement = 0
        VABS.F32  S5, S0                		//
        VCMPE.F32 S5, S1                		//
        VLDR.32   S3, [r0, #4]          		//  PI / 2
        VMRS APSR_nzcv, FPSCR 					//
        VLDR.32   S2, [r0, #8]          		//  PI/ 4
        VLDR.32   S4, [r0, #12]         		// 0.273

        VDIVGT.F32  S0, S1, S0          		// x > 1 so use 1/x
        MOVGT		r3, #1						// complement = 1
        VCMPE.F32 S0, #0.0              		// if x < 0, K = -PI/2 instead of PI/2
        VMRS APSR_nzcv, FPSCR 					//
        VNEGLT.F32  S3, S3						// set K to -PI/2

		VABS.F32  S5, S0        				// if we set x = 1/x we need the abs value again
		VMUL.F32  S4, S0, S4            		// x * 0.273
        VSUB.F32  S7, S1, S5            		// 1 - abs(x)
        VMUL.F32  S7, S7, S4            		// x * 0.273 * (1-abs(x))
        VMLA.F32  S7, S0, S2            		// PI/4 * x + x * 0.273 * (1-abs(x))

		CMP			r3, #1
		VSUBEQ.F32  S0, S3, S7            		//
        VMOVNE.F32  S0, S7

        BX        LR                    		//






	.section	.textTrig, "axw",  %progbits
	.arm
	.code 32
	.global	ti_r5fmath_atan2Fast
ti_r5fmath_atan2Fast:
// add checks for 0
        VABS.F32  S4, S0              		  	// S4 = abs(y)
        VABS.F32  S6, S1                		// S6 = abs(x)
        EOR       r12, r12, r12             		// r12 = 0
        VMOV      S2, r12                		// S2 = 0
        VLDR.32   S5, [A1, #0]          		// S5 = PI
        VCMPE.F32	S1, #0						// check x value in order to set constant to 0 or PI if less than 0
       	VMRS APSR_nzcv, FPSCR
       	VMOVGE.F32	S5, S2						// if x > 0, set S5 to 0 instead of PI

        VLDR.32   S7, [A1, #4]          		// S7 = PI/2

		VCMPE.F32 S1, S2						// check if x = 0
		VMRS APSR_nzcv, FPSCR
		BEQ	 $1

        MOV       A2, #0                		// A2 = 0 = complement
        VCMPE.F32 S6, S4                		// abs(x) > abs(y) ?

        VLDR.32   S4, [A1, #8]          		// S4 = PI/4
        VLDR.32   S6, [A1, #12]         		// S6 = 0.273
        VMRS APSR_nzcv, FPSCR 					//

/* ---------------- ------------------------------*/
        VDIVLT.F32  S12, S1, S0         		// y > x  inVal = x/y
        VMOVLT.F32	S10, S7						// S10 = K2 = S7 = PI/2
        MOVLT       A2, #1 						// A2 = complement = 1

        VDIVGE.F32  S12, S0, S1 				// x >= y  inVal = y/x
        VMOVGE.F32	S10, S5						// S10 = K1 = PI or 0 depending on the earlier comparison

        VCMPE.F32	S0, #0						// now check y value
        VMRS APSR_nzcv, FPSCR
        VNEGLT.F32	S10, S10					// negate the constant if y < 0

        VABS.F32  S0, S12                		// abs(inVal)
        VMOV.F32  S5, #1.00000000000000000000e+00
        VSUB.F32  S0, S5, S0            		// 1 - abs(inVal)
        VMUL.F32  S5, S12, S6            		// inVal * 0.273
        VMUL.F32  S0, S0, S5            		// inVal * 0.273 * ( 1 - abs(inVal) )
        CMP       A2, #1                		//
        VMLA.F32  S0, S12, S4          			// PI/4* inVal + [inVal * 0.273 * ( 1 - abs(inVal) ) ]

        VSUBEQ.F32  S0, S10, S0          		// +/- PI/2 - arctan
        VADDNE.F32  S0, S10, S0          		// +/- PI + arctan
	    BX        LR                    		//

$1:
		VMOV.F32	S0, S7						// if x = 0 return PI/2
		VCMPE.F32	S1, #0						// if y < 0, negate x
		VMRS APSR_nzcv, FPSCR
		VNEGLT.F32	S0, S0						// if y < 0 return - PI/2

        BX        LR                    		//


	.section	.textTrig, "axw",  %progbits
	.arm
	.code 32
	.global ti_r5fmath_sin
ti_r5fmath_sin:
	// Input constants
	// 3PI/2  2PI  PI/2   PI   SIN3   SIN5   SIN1  aligned on 64b
	// S0 is angle in radians input
	// S1 = 3PI/2,  S2 = 2PI, S3 = PI/2  S4 = PI

     	VMOV.F32		S12, S0					// make a copy
     	VLDM.32 		r0, {S1-S4}				// load constants
	    VCMPE.F32 		S12, S3           		// check for >Q1, compare to PI/2
	    VMRS 			APSR_nzcv, FPSCR 		//
        VSUBGT.F32		S0, S4, S12				// Q2 / Q3 value
	   /*  --------------------------------------------------------------------*/
	    VCMPE.F32 		S12, S1          		// compare to 3PI/2
        VMRS 			APSR_nzcv, FPSCR 		//

	 	VSUBGT.F32		S0, S12, S2				// move Q4 value in

		// S5 = SIN1,  S6 = SIN3,  S7 = SIN5,  S8 = SIN7
	    VLDM.32  		r1, {S5-S8}

		VMUL.F32  		S2, S0, S0            	// angle ^ 2 = S2

        VMUL.F32		S10, S0, S8			  	// SIN7 * x
        VMUL.F32  		S3, S0, S6            	// SIN3 * x
	    VMUL.F32  		S9, S0, S7            	// SIN5 * x
	    VMUL.F32		S10, S10, S2		  	// SIN5 * x^3

		VMUL.F32  		S4, S2, S2            	// x ^ 4

        VMUL.F32  		S0, S0, S5           	// SIN1 * x
        VMLA.F32  		S0, S3, S2            	// +=  SIN3 x * x^2
        VMLA.F32  		S0, S9, S4            	// += SIN5x * x^4
        VMLA.F32		S0, S10, S4			  	// += SIN7x^3 * x^4

        BX        LR                    		//



	.section	.textTrig, "axw",  %progbits
	.arm
	.code 32
	.global ti_r5fmath_cos
ti_r5fmath_cos:
	// Input constants
	// 3PI/2  2PI  PI/2   PI   SIN3   SIN5   SIN1  aligned on 64b
	// S0 is angle in radians input
	// S1 = 3PI/2,  S2 = 2PI, S3 = PI/2  S4 = PI

     	VMOV.F32		S12, S0					// make a copy
     	VMOV.F32		S14, #1.0				// hold 1 or -1 depending on quadrant
     	VLDM.32 		r0, {S1-S4}				// load constants
	    VCMPE.F32 		S12, S3           		// check for >Q1, compare to PI/2
	    VMRS 			APSR_nzcv, FPSCR 		//
        VSUBGT.F32		S0, S12, S4				// subtract PI for Q2 / Q3 value
	    VMOVGT.F32		S14, #-1.0

	    /*  --------------------------------------------------------------------*/

	    VCMPE.F32 		S12, S1          		// compare to 3PI/2
        VMRS 			APSR_nzcv, FPSCR 		//

	 	VSUBGT.F32		S0, S0, S4				// subtract PI to get value
	 	VMOVGT.F32		S14, #1.0


		// S5 = COS0,  S6 = COS2,  S7 = COS4,  S8 = COS6, S9 = COS8

	    VLDM.32  		r1, {S5-S9}

		VMUL.F32  		S2, S0, S0        	    // angle ^ 2 = S2
		VMOV.F32		S0, S5				  	// put COS0 into S0
		VMLA.F32		S0, S6, S2				// add COS2 * x^2
		VMUL.F32		S4, S2, S2				// x^4

        VMUL.F32  		S11, S2, S8            	// COS6 * x^2

        VMUL.F32		S10, S4, S9			  	// COS8 * x^4

	    VMLA.F32  		S0, S7, S4            	// + COS4 * x^4
	    VMLA.F32  		S0, S11, S4            	// COS6 x^2 * x^4
		VMLA.F32		S0, S10, S4	    		// COS8 X^4 * x^4
		VMUL.F32		S0, S0, S14


        BX        LR                    		//

	.section	.textTrig, "axw",  %progbits
	.arm
	.code 32
	.global ti_r5fmath_sincosB
ti_r5fmath_sincosB:
	// Input constants
	// 3PI/2  2PI  PI/2   PI   SIN3   SIN5   SIN1  aligned on 64b
	// S0 is angle in radians input
	// S1 = 3PI/2,  S2 = 2PI, S3 = PI/2  S4 = PI
	// Input format is S0, pointer to S1-S4 (r0), pointer to coefficients (r1), pointer to ouput (r2)

		VLDM.32 		r0, {S1-S4}				// load constants
		VMOV.F32		S12, S0					// make a copy of the input value
		VMOV.F32		S15, #1.0				// hold 1 or -1 depending on quadrant

	    VCMPE.F32 		S12, S3           		// check for >Q1, compare to PI/2
	    VMRS 			APSR_nzcv, FPSCR 		//
        VSUBGT.F32		S0, S4, S12				// Q2 / Q3 value
        VMOVGT.F32		S15, #-1.0

	   /* --------------------------------------------------------------------*/

	    VCMPE.F32 		S12, S1          		// compare to 3PI/2
        VMRS 			APSR_nzcv, FPSCR 		//
	 	VSUBGT.F32		S0, S12, S2				// move Q4 value in
	 	VMOVGT.F32		S15, #1.0				// S15 holds multiplier for cosine value at end


		// S2 = SIN1,  S3 = SIN3,  S4 = SIN5,  S5 = SIN7
		// S6 = COS0,  S7 = COS2,  S8 = COS4,  S9 = COS6, S10 = COS8
		// build sin in S0
		// build cos in S6

	    VLDM.32  		r1, {S2-S10}

		VMUL.F32  		S14, S0, S0           	// x ^ 2

	    VMUL.F32		S11, S0, S5			  	// SIN7 * x
        VMUL.F32  		S12, S0, S3           	// SIN3 * x

        VMLA.F32		S6, S7, S14			  	// add COS2 * x^2

		VMUL.F32  		S13, S0, S4           	// SIN5 * x
	    VMUL.F32  		S0, S0, S2            	// SIN1 * x
	    // registers S2-S5 are now clear
	    VMUL.F32  		S4, S14, S14            // x ^ 4



	    VMUL.F32		S2, S11, S14		  	// SIN7 x* x^2
		VMLA.F32  		S6, S4, S8          	// + COS4 * x^4

		VMUL.F32  		S5, S14, S9         	// COS6 * x^2
		VMUL.F32		S10, S4, S10			// COS8 * x^4

        VMLA.F32  		S0, S14, S12        	// +=  SIN3 x * x^2
        VMLA.F32  		S0, S4, S13         	// += SIN5x * x^4
        VMLA.F32		S0, S4, S2		  		// += SIN7x^3 * x^4

		VMLA.F32  		S6, S5, S4         		// +COS6 x^2 * x^4
        VMLA.F32		S6, S10, S4	    		// +COS8 X^4 * x^4
		VMUL.F32		S6, S6, S15

		VSTR.32			S0, [r2]
		VSTR.32			S6, [r2, #4]

        BX        LR                    		//
