/*
 *  Copyright (C) 2018-2023 Texas Instruments Incorporated
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
#include <stdint.h>
#include <string.h>

extern uint32_t __BSS_START;
extern uint32_t __BSS_END;
int32_t _system_pre_init(void);
void stack_init(register char* stack_ptr);

void stack_init(register char* stack_ptr)
{
   __asm__ __volatile__  ("mrs r1, control"   "\n\t": : : "cc");
   __asm__ __volatile__  ("bic r1, r1, #0x2"  "\n\t": : : "cc");
   __asm__ __volatile__  ("msr control, r1"   "\n\t": : : "cc");
   __asm__ __volatile__  ("isb sy"            "\n\t": : : "memory");
   __asm volatile ("MSR msp, %0" : : "r" (stack_ptr) : );

}

int32_t _system_pre_init(void)
{
    uint32_t bss_size = ((uintptr_t)&__BSS_END - (uintptr_t)&__BSS_START);
    memset((void*)&__BSS_START, 0x00, bss_size);
    return 1;
}

#ifdef __TI_RTS_BUILD
/*---------------------------------------------------------------------------*/
/* __TI_default_c_int00 indicates that the default TI entry routine is being */
/* used.  The linker makes assumptions about what exit does when this symbol */
/* is seen. This symbol should NOT be defined if a customized exit routine   */
/* is used.                                                                  */
/*---------------------------------------------------------------------------*/
__asm__ __volatile__ (".set __TI_default_c_int00, 1": : : "memory");
#endif

/*----------------------------------------------------------------------------*/
/* Define the user mode stack. The size will be determined by the linker.     */
/*----------------------------------------------------------------------------*/
__attribute__((section(".stack")))
int32_t __stack;

/*----------------------------------------------------------------------------*/
/* Linker defined symbol that will point to the end of the user mode stack.   */
/* The linker will enforce 8-byte alignment.                                  */
/*----------------------------------------------------------------------------*/
extern int32_t __STACK_END;

/*----------------------------------------------------------------------------*/
/* Function declarations.                                                     */
/*----------------------------------------------------------------------------*/
__attribute__((weak)) extern void __mpu_init(void);
#ifdef __cplusplus
extern "C" {
#endif
extern void __TI_auto_init(void);
extern void exit(int32_t argc);
void _c_int00(void);
#ifdef __cplusplus
}
#endif
extern int32_t main(int32_t argc, char **argv);

/*----------------------------------------------------------------------------*/
/* boot routine for Cortex-M                                          */
/*----------------------------------------------------------------------------*/
__attribute__((section(".text:_c_int00"), noreturn))
void _c_int00(void)
{
    /* Initialize the stack pointer */
   register char* stack_ptr = (char*)&__STACK_END;
   /*
    * Initialize the CONTROL register to change to Main
    * Stack Pointer (MSP) by setting SPSEL to 0.
    *
    */
   stack_init(stack_ptr);

   /* Initialize the FPU if building for floating point */
   #ifdef __ARM_FP
   volatile uint32_t* cpacr = (volatile uint32_t*)0xE000ED88U;
   *cpacr |= ((uint32_t)0xf0 << 16);
   #endif

   __mpu_init();
   if ((_system_pre_init()) != 0)
   {
       __TI_auto_init();
   }

   main(0, (char**)0);

   exit(1);

   while (1){;}
}