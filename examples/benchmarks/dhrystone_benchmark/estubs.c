/*******************************************************************************
 * Copyright (c) 2004 by Graham Davies, ECROS Technology.  This work is        *
 * released into the Public Domain on condition that the forgoing copyright    *
 * notice is preserved in this and all derived files.                          *
 ******************************************************************************/

/*******************************************************************************
 * File: estubs.c - embedded stubs used to adapt the Dhrystone benchmark for
 * small (4 Kbyte RAM) microcontrollers.
 *
 * Last edited on $Date: Wednesday, June 09, 2004 1:05:22 PM $ by
 * $Author: Graham $ saved as $Revision: 1.5 $ $Version: NONE $
 ******************************************************************************/

/*
 * Copyright (C) 2023 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>
#include <stdarg.h>
#include "dhry.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/*******************************************************************************
 * Function emalloc() - provides a dynamic memory allocation service sufficient
 * for two allocations of 50 bytes each or less.
 */
void * emalloc( size_t nbytes )
{
   static char   space[100];
   static char * ptr = space;
   static char * result;

   result = ptr;
   nbytes = (nbytes & 0xFFFC) + 4;
   ptr += nbytes;

   return ( result );

} /* end of function emalloc() */

/*******************************************************************************
 * Function fopen() - initialize an output port bit.
 */
/*FILE * fopen( const char * filename, const char * mode )
{
#if defined( __AVR_ARCH__ )
   DDRB = 0x01;     bit 0 output
   PORTB = 0x00;    start low (zero)
#elif defined( __ENCORE__ )
   PAADDR = 4;
   PACTL  = 0x01;   bit 0 high drive
   PAADDR = 1;
   PACTL  = 0xFE;   bit 0 output
   PAADDR = 0;
   PAOUT  = 0x00;   start low (zero)
#endif

   return ( (FILE *)1 );

}*/ /* end of function fopen() */

/*******************************************************************************
 * Function fclose() - stub; does nothing.
 */
/*
int fclose( FILE * stream )
{
   return ( 0 );

}  end of function fclose()
*/

/*******************************************************************************
 * Function scanf() - fake a value typed by the user.
 */
/*int scanf( const char * format, ... )
{
   va_list args;
   int *   ptr;

   va_start( args, format );
   ptr = va_arg( args, int * );
   *ptr = 30000;
   va_end( args );

   return ( 0 );

}*/ /* end of function scanf() */

/* end of file estubs.c */