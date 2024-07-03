/*
 *************************************************************************
 *
 *                   "DHRYSTONE" Benchmark Program
 *                   -----------------------------
 *
 *  Version:    C, Version 2.1
 *
 *  File:       dhry_1.c (part 2 of 3)
 *
 *  Date:       May 25, 1988
 *
 *  Author:     Reinhold P. Weicker
 *
 *************************************************************************
 */

/***************************************************************************
 * Adapted for embedded microcontrollers by Graham Davies, ECROS Technology.
 **************************************************************************/

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

#include "dhry.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/_timeval.h>
#include "time.h"
#include <kernel/dpl/CycleCounterP.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include "ti_drivers_config.h"
#if defined SOC_AM263X
#include <drivers/soc/am263x/soc.h>
#endif
#if defined SOC_AM273X
#include <drivers/soc/am273x/soc.h>
#endif
#if defined SOC_AM263PX
#include <drivers/soc/am263px/soc.h>
#endif
#if defined SOC_AM261X
#include <drivers/soc/am261x/soc.h>
#endif
#if defined SOC_AM243X || defined SOC_AM64X
#include <drivers/soc/am64x_am243x/soc.h>
#endif
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <kernel/dpl/TimerP.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#ifndef ROPT
#define REG
        /* REG becomes defined as empty */
        /* i.e. no register variables   */
#else
#define REG register
#endif
#define No_Of_runs 500000
#define clock_freq 200000000.0
#define Too_Small_Time 2
                /* Measurements should last at least 2 seconds */

/* variables for time measurement: */
/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void Proc_1( REG Rec_Pointer Ptr_Val_Par );
void Proc_2( One_Fifty * Int_Par_Ref );
void Proc_3( Rec_Pointer * Ptr_Ref_Par );
void Proc_4( void );
void Proc_5( void );

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

Rec_Pointer Ptr_Glob, Next_Ptr_Glob;

int Status, Int_Glob, Arr_1_Glob [25], Arr_2_Glob [25] [25];

Boolean  Bool_Glob;

char Ch_1_Glob, Ch_2_Glob;

char Reg_Define[] = "Register option selected.";

uint64_t  Begin_Time, End_Time, User_Time;
double Microseconds, User_time, Dhrystones_Per_Second, Vax_Mips;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/*****/

  /* main program, corresponds to procedures        */
  /* Main and Proc_0 in the Ada version             */
void dhrystone( void *args)
{
    DebugP_log("BENCHMARK START - ARM R5F - DHRYSTONE\r\n");
    #if (defined SOC_AM263X) || (defined SOC_AM263PX) || (defined SOC_AM273X)
    Status  = SOC_rcmIsR5FInLockStepMode(CSL_ARM_R5_CLUSTER_GROUP_ID_0);
    DebugP_log("status %d\r\n", Status);
    #endif

    One_Fifty       Int_1_Loc;
    REG One_Fifty   Int_2_Loc;
    One_Fifty       Int_3_Loc;
    REG char        Ch_Index;
    Enumeration     Enum_Loc;
    Str_30          Str_1_Loc;
    Str_30          Str_2_Loc;
    REG int         Run_Index;

    Drivers_open();
    Board_driversOpen();

  /* Initializations */

    Next_Ptr_Glob = (Rec_Pointer)emalloc (sizeof (Rec_Type));
    Ptr_Glob = (Rec_Pointer)emalloc (sizeof (Rec_Type));

    Ptr_Glob->Ptr_Comp                    = Next_Ptr_Glob;
    Ptr_Glob->Discr                       = Ident_1;
    Ptr_Glob->variant.var_1.Enum_Comp     = Ident_3;
    Ptr_Glob->variant.var_1.Int_Comp      = 40;
    strcpy (Ptr_Glob->variant.var_1.Str_Comp, "DHRYSTONE PROGRAM, SOME STRING");
    strcpy (Str_1_Loc, "DHRYSTONE PROGRAM, 1'ST STRING");

    Arr_2_Glob [8][7] = 10;
    /* Was missing in published program. Without this statement,    */
    /* Arr_2_Glob [8][7] would have an undefined value.             */
    /* Warning: With 16-Bit processors and Number_Of_Runs > 32000,  */
    /* overflow may occur for this array element.                   */

    DebugP_log ("\r\n");
    DebugP_log ("Dhrystone Benchmark, Version 2.1 (Language: C)\r\n");
    DebugP_log ("\r\n");

    /***************/
    /* Start timer */
    /***************/
    CycleCounterP_init(SOC_getSelfCpuClk());
    Begin_Time = CycleCounterP_getCount32();

    for (Run_Index = 1; Run_Index <= No_Of_runs; ++Run_Index  )
    {
        Proc_5();
        Proc_4();
        /* Ch_1_Glob == 'A', Ch_2_Glob == 'B', Bool_Glob == true */
        Int_1_Loc = 2;
        Int_2_Loc = 3;
        strcpy (Str_2_Loc, "DHRYSTONE PROGRAM, 2'ND STRING");
        Enum_Loc = Ident_2;
        Bool_Glob = ! Func_2 (Str_1_Loc, Str_2_Loc);
        /* Bool_Glob == 1 */
        while (Int_1_Loc < Int_2_Loc)  /* loop body executed once */
        {
            Int_3_Loc = 5 * Int_1_Loc - Int_2_Loc;
                /* Int_3_Loc == 7 */
            Proc_7 (Int_1_Loc, Int_2_Loc, &Int_3_Loc);
                /* Int_3_Loc == 7 */
            Int_1_Loc += 1;
        } /* while */

        Proc_8 (Arr_1_Glob, Arr_2_Glob, Int_1_Loc, Int_3_Loc);
        Proc_1 (Ptr_Glob);
        for (Ch_Index = 'A'; Ch_Index <= Ch_2_Glob; ++Ch_Index) /* loop body executed twice */
        {
            if (Enum_Loc == Func_1 (Ch_Index, 'C')) /* then, not executed */
            {
                Proc_6 (Ident_1, &Enum_Loc);
                strcpy (Str_2_Loc, "DHRYSTONE PROGRAM, 3'RD STRING");
                Int_2_Loc = Run_Index;
                Int_Glob = Run_Index;
            }
        }
        Int_2_Loc = Int_2_Loc * Int_1_Loc;
        Int_1_Loc = Int_2_Loc / Int_3_Loc;
        Int_2_Loc = 7 * (Int_2_Loc - Int_3_Loc) - Int_1_Loc;
        Proc_2 (&Int_1_Loc);
    } /* loop "for NO of runs" */

    //counter ends
    End_Time = CycleCounterP_getCount32();

    DebugP_log("- BEGIN cycle count:                         %d\r\n",Begin_Time);
    DebugP_log("- END Cycle count:                           %d\r\n",End_Time);
    User_Time = End_Time - Begin_Time;
    DebugP_log("- USER cycle count:                          %d\r\n",User_Time);

    User_time = (float) User_Time/clock_freq;//time
    Microseconds = User_time * Mic_secs_Per_Second/ No_Of_runs;//no of runs is 500000
    Dhrystones_Per_Second = No_Of_runs / User_time;
    Vax_Mips = (Dhrystones_Per_Second / 1757.0);

    DebugP_log("- Usertime in sec:                           %lf\r\n", User_time);
    DebugP_log("- Microseconds for one run through Dhrystone:%6.1f \r\n", Microseconds);
    DebugP_log("- Dhrystones per Second:                     %6.1f \r\n", Dhrystones_Per_Second);
    DebugP_log("\nNormalized MIPS/MHz:                       %8.4f\r\n", Dhrystones_Per_Second/1757.0/200.0);
    #ifdef ROPT
        printf ("Register option selected?  YES\r\n");
    #else
        // printf ("Register option selected?  NO\n");
        // strcpy(Reg_Define, "Register option not selected.");
    #endif
    DebugP_log("BENCHMARK END\r\n");

    DebugP_log("\n All tests have passed. \n");

    Board_driversClose();
    Drivers_close();
}


void Proc_1( REG Rec_Pointer Ptr_Val_Par )
/******************/
    /* executed once */
{
    REG Rec_Pointer Next_Record = Ptr_Val_Par->Ptr_Comp;
                                    /* == Ptr_Glob_Next */
    /* Local variable, initialized with Ptr_Val_Par->Ptr_Comp,    */
    /* corresponds to "rename" in Ada, "with" in Pascal           */

    structassign (*Ptr_Val_Par->Ptr_Comp, *Ptr_Glob);
    Ptr_Val_Par->variant.var_1.Int_Comp = 5;
    Next_Record->variant.var_1.Int_Comp = Ptr_Val_Par->variant.var_1.Int_Comp;
    Next_Record->Ptr_Comp = Ptr_Val_Par->Ptr_Comp;
    Proc_3 (&Next_Record->Ptr_Comp);
    /* Ptr_Val_Par->Ptr_Comp->Ptr_Comp  == Ptr_Glob->Ptr_Comp */
    if(Next_Record->Discr == Ident_1)
    /* then, executed */
    {
        Next_Record->variant.var_1.Int_Comp = 6;
        Proc_6 (Ptr_Val_Par->variant.var_1.Enum_Comp, &Next_Record->variant.var_1.Enum_Comp);
        Next_Record->Ptr_Comp = Ptr_Glob->Ptr_Comp;
        Proc_7 (Next_Record->variant.var_1.Int_Comp, 10, &Next_Record->variant.var_1.Int_Comp);
    }
    else /* not executed */
    {
        structassign (*Ptr_Val_Par, *Ptr_Val_Par->Ptr_Comp);
    }
} /* Proc_1 */

void Proc_2( One_Fifty * Int_Par_Ref )
/******************/
    /* executed once */
    /* *Int_Par_Ref == 1, becomes 4 */
{
    One_Fifty Int_Loc;
    Enumeration Enum_Loc;

    Int_Loc = *Int_Par_Ref + 10;
    do /* executed once */
    if (Ch_1_Glob == 'A')
        /* then, executed */
    {
        Int_Loc -= 1;
        *Int_Par_Ref = Int_Loc - Int_Glob;
        Enum_Loc = Ident_1;
    } /* if */
    while (Enum_Loc != Ident_1); /* true */
} /* Proc_2 */

void Proc_3( Rec_Pointer * Ptr_Ref_Par )
/******************/
    /* executed once */
    /* Ptr_Ref_Par becomes Ptr_Glob */
{
    if (Ptr_Glob != Null) /* then, executed */
    {
        *Ptr_Ref_Par = Ptr_Glob->Ptr_Comp;
    }
    Proc_7 (10, Int_Glob, &Ptr_Glob->variant.var_1.Int_Comp);
} /* Proc_3 */

void Proc_4( void ) /* without parameters */
/*******/
    /* executed once */
{
    Boolean Bool_Loc;

    Bool_Loc = Ch_1_Glob == 'A';
    Bool_Glob = Bool_Loc | Bool_Glob;
    Ch_2_Glob = 'B';
} /* Proc_4 */

void Proc_5( void ) /* without parameters */
/*******/
    /* executed once */
{
    Ch_1_Glob = 'A';
    Bool_Glob = false;
} /* Proc_5 */

/* Procedure for the assignment of structures,          */
/* if the C compiler doesn't support this feature       */
#ifdef  NOSTRUCTASSIGN
memcpy (d, s, l)
register char   *d;
register char   *s;
register int    l;
{
    while (l--) *d++ = *s++;
}
#endif