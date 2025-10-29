

/* float floatLoad(float f1, float f2)
 * load f1, f2 to s11, s12
 */
    PUBLIC	floatLoadAndMultiply
	SECTION .text:CODE:NOROOT(4)
	ALIGNRAM 4
floatLoadAndMultiply:

	vmov.f32	s11, s0
	vmov.f32	s12, s1
	vmul.f32	s0, s11, s12
	bx	lr

/* float floatMultiply()
 * multiply previously loaded s11, s12 and return it
 */
	PUBLIC	floatMultiply
	SECTION .text:CODE:NOROOT(4)                            
	ALIGNRAM 4								;@ @funWithFloats1
	
floatMultiply:
	sub	sp, sp, #8
	vmul.f32	s0, s11, s12
	add	sp, sp, #8
	bx	lr

	END




