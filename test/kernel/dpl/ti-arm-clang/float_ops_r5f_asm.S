

/* float floatLoad(float f1, float f2)
 * load f1, f2 to s11, s12
 */
    .globl	floatLoadAndMultiply
	.p2align	4
	.type	floatLoadAndMultiply,%function
	.code	32
floatLoadAndMultiply:

	vmov.f32	s11, s0
	vmov.f32	s12, s1
	vmul.f32	s0, s11, s12
	bx	lr

/* float floatMultiply()
 * multiply previously loaded s11, s12 and return it
 */
	.globl	floatMultiply
	.p2align	4
	.type	floatMultiply,%function
	.code	32                              @ @funWithFloats1
floatMultiply:
	.fnstart
	.pad	#8
	sub	sp, sp, #8
	vmul.f32	s0, s11, s12
	.loc	1 12 5                          @ ../FunWithFloats.c:12:5
	add	sp, sp, #8
	bx	lr
	.fnend




