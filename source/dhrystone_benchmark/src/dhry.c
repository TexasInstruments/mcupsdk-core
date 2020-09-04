/*
 ****************************************************************************
 *
 *                   "DHRYSTONE" Benchmark Program
 *                   -----------------------------
 *
 *  Version:    C, Version 2.1
 *
 *  File:       dhry.c
 *
 *  Date:       May 25, 1988
 *
 *  Author:     Reinhold P. Weicker
 *
 ****************************************************************************
 */

/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
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

/**
 *  \file dhry.c
 *
 *  \brief Dhrystone benchmark source file.
 *
 */

#include "dhry.h"
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/CycleCounterP.h>

#include <stdlib.h>
#include <string.h>
#include <pthread.h>

#ifndef REG
Boolean Reg = false;
#define REG
        /* REG becomes defined as empty */
        /* i.e. no register variables   */
#else
        Boolean Reg = true;
#endif

/* Measurements should last at least about 2 seconds */
#define Too_Small_Time (2 * 1000000)

#define DEFAULT_STACK_SIZE      (0x6000u)
#define DEFAULT_PRIORITY        (13u)

typedef struct {
    int numIterations;
} threadInst_t;

/* Static Function declarations */
static void Proc_1 (dhryInstance_t *inst, REG Rec_Pointer Ptr_Val_Par);
static void Proc_2 (dhryInstance_t *inst, One_Fifty *Int_Par_Ref);
static void Proc_3 (dhryInstance_t *inst, Rec_Pointer *Ptr_Ref_Par);
static void Proc_4 (dhryInstance_t *inst); /* without parameters */
static void Proc_5 (dhryInstance_t *inst); /* without parameters */
static void Proc_6 (dhryInstance_t *inst, Enumeration Enum_Val_Par, Enumeration *Enum_Ref_Par);
static void Proc_7 (One_Fifty Int_1_Par_Val, One_Fifty Int_2_Par_Val, One_Fifty *Int_Par_Ref);
static void Proc_8 (dhryInstance_t *inst, Arr_1_Dim Arr_1_Par_Ref, Arr_2_Dim Arr_2_Par_Ref, int Int_1_Par_Val, int Int_2_Par_Val);
static Enumeration Func_1 (dhryInstance_t *inst, Capital_Letter, Capital_Letter);
static Boolean Func_2 (dhryInstance_t *inst, Str_30 Str_1_Par_Ref, Str_30 Str_2_Par_Ref);
static Boolean Func_3 (Enumeration Enum_Par_Val);

static int nIterations;

void *dhryThread (void* args)
/*****/

  /* main program, corresponds to procedures        */
  /* Main and Proc_0 in the Ada version             */
{
            One_Fifty       Int_1_Loc = 0;
    REG     One_Fifty       Int_2_Loc = 0;
            One_Fifty       Int_3_Loc = 0;
    REG     char            Ch_Index;
            Enumeration     Enum_Loc = Ident_1;
            Str_30          Str_1_Loc;
            Str_30          Str_2_Loc;
    REG     int             Run_Index;
    REG     int             Number_Of_Runs = 0;

    dhryInstance_t inst;

    long        Begin_Time = 0,
                End_Time = 0,
                User_Time = 0;

    uint32_t    Microseconds = 0,
                Dhrystones_Per_Second = -1;

    /* Initializations */

    Number_Of_Runs = nIterations;

    DebugP_assert(Number_Of_Runs == 30000000U);

    inst.Next_Ptr_Glob = (Rec_Pointer) malloc (sizeof (Rec_Type));

    if (inst.Next_Ptr_Glob == NULL)
    {
        DebugP_log ("Memory Allocation Error\r\n");
        DebugP_assertNoLog (0);
    }

    memset (inst.Next_Ptr_Glob, 0, sizeof (Rec_Type));

    inst.Ptr_Glob = (Rec_Pointer) malloc (sizeof (Rec_Type));

    if (inst.Ptr_Glob == NULL)
    {
        DebugP_log ("Memory Allocation Error\r\n");
        DebugP_assertNoLog (0);
    }

    memset (inst.Ptr_Glob, 0, sizeof (Rec_Type));

    inst.Ptr_Glob->Ptr_Comp                    = inst.Next_Ptr_Glob;
    inst.Ptr_Glob->Discr                       = Ident_1;
    inst.Ptr_Glob->variant.var_1.Enum_Comp     = Ident_3;
    inst.Ptr_Glob->variant.var_1.Int_Comp      = 40;
    strcpy (inst.Ptr_Glob->variant.var_1.Str_Comp,
            "DHRYSTONE PROGRAM, SOME STRING");
    strcpy (Str_1_Loc, "DHRYSTONE PROGRAM, 1'ST STRING");
    strcpy (Str_2_Loc, "UNKNOWN");

    inst.Arr_1_Glob[8] = 0;
    inst.Bool_Glob = 0;
    inst.Ch_1_Glob = ' ';
    inst.Ch_2_Glob = ' ';
    inst.Int_Glob = 0;
    inst.Next_Ptr_Glob->Discr = Ident_1;
            /* Was missing in published program. Without this statement,    */
            /* inst.Arr_1_Glob [8] would have an undefined value.           */
            /* inst.Bool_Glob would have an undefined value.                */
            /* inst.Ch_1_Glob would have an undefined value.                */
            /* inst.Ch_2_Glob would have an undefined value.                */
            /* inst.Int_Glob would have an undefined value.                 */
            /* inst.inst.Next_Ptr_Glob->Discr would have an undefined value.*/
    inst.Arr_2_Glob [8][7] = 10;
            /* Was missing in published program. Without this statement,    */
            /* Arr_2_Glob [8][7] would have an undefined value.             */
            /* Warning: With 16-Bit processors and Number_Of_Runs > 32000,  */
            /* overflow may occur for this array element.                   */

    if (dhryLogEnable)
    {
        DebugP_log ("\r\n");
        DebugP_log ("Dhrystone Benchmark, Version 2.1 (Language: C)\r\n");
        DebugP_log ("\r\n");
    }

    if (dhryLogEnable)
    {
        if (Reg)
        {
            DebugP_log ("Program compiled with 'register' attribute\r\n");
            DebugP_log ("\r\n");
        }
        else
        {
            DebugP_log ("Program compiled without 'register' attribute\r\n");
            DebugP_log ("\r\n");
        }
    }

    if(Number_Of_Runs == 0)
    {
        if (dhryLogEnable)
        {
            DebugP_log ("Please give the number of runs through the benchmark: ");
            {
                int n;
                DebugP_scanf ("%d", &n);
                Number_Of_Runs = n;
            }
            DebugP_log ("\r\n");
        }
    }

    if (dhryLogEnable)
    {
        DebugP_log ("Execution starts, %d runs through Dhrystone\r\n", Number_Of_Runs);
    }

    /***************/
    /* Start timer */
    /***************/

    Begin_Time = ClockP_getTimeUsec();

    for (Run_Index = 1; Run_Index <= Number_Of_Runs; ++Run_Index)
    {

        Proc_5(&inst);
        Proc_4(&inst);
        /* Ch_1_Glob == 'A', Ch_2_Glob == 'B', Bool_Glob == true */
        Int_1_Loc = 2;
        Int_2_Loc = 3;
        strcpy (Str_2_Loc, "DHRYSTONE PROGRAM, 2'ND STRING");
        Enum_Loc = Ident_2;
        inst.Bool_Glob = ! Func_2 (&inst, Str_1_Loc, Str_2_Loc);
        /* Bool_Glob == 1 */
        while (Int_1_Loc < Int_2_Loc)  /* loop body executed once */
        {
            Int_3_Loc = 5 * Int_1_Loc - Int_2_Loc;
                /* Int_3_Loc == 7 */
            Proc_7 (Int_1_Loc, Int_2_Loc, &Int_3_Loc);
                /* Int_3_Loc == 7 */
            Int_1_Loc += 1;
        } /* while */
        /* Int_1_Loc == 3, Int_2_Loc == 3, Int_3_Loc == 7 */
        Proc_8 (&inst, inst.Arr_1_Glob, inst.Arr_2_Glob, Int_1_Loc, Int_3_Loc);
        /* Int_Glob == 5 */
        Proc_1 (&inst, inst.Ptr_Glob);
        for (Ch_Index = 'A'; Ch_Index <= inst.Ch_2_Glob; ++Ch_Index)
                                /* loop body executed twice */
        {
            if (Enum_Loc == Func_1 (&inst, Ch_Index, 'C'))
            /* then, not executed */
            {
                Proc_6 (&inst, Ident_1, &Enum_Loc);
                strcpy (Str_2_Loc, "DHRYSTONE PROGRAM, 3'RD STRING");
                Int_2_Loc = Run_Index;
                inst.Int_Glob = Run_Index;
            }
        }
        /* Int_1_Loc == 3, Int_2_Loc == 3, Int_3_Loc == 7 */
        Int_2_Loc = Int_2_Loc * Int_1_Loc;
        Int_1_Loc = Int_2_Loc / Int_3_Loc;
        Int_2_Loc = 7 * (Int_2_Loc - Int_3_Loc) - Int_1_Loc;
        /* Int_1_Loc == 1, Int_2_Loc == 13, Int_3_Loc == 7 */
        Proc_2 (&inst, &Int_1_Loc);
        /* Int_1_Loc == 5 */

    } /* loop "for Run_Index" */

    /**************/
    /* Stop timer */
    /**************/
    End_Time  = ClockP_getTimeUsec();

    if (dhryLogEnable)
    {
        DebugP_log ("Execution ends\r\n");
        DebugP_log ("\r\n");
        DebugP_log ("Final values of the variables used in the benchmark:\r\n");
        DebugP_log ("\r\n");
        DebugP_log ("Int_Glob:            %d\r\n", inst.Int_Glob);
        DebugP_log ("        should be:   %d\r\n", 5);
        DebugP_log ("Bool_Glob:           %d\r\n", inst.Bool_Glob);
        DebugP_log ("        should be:   %d\r\n", 1);
        DebugP_log ("Ch_1_Glob:           %c\r\n", inst.Ch_1_Glob);
        DebugP_log ("        should be:   %c\r\n", 'A');
        DebugP_log ("Ch_2_Glob:           %c\r\n", inst.Ch_2_Glob);
        DebugP_log ("        should be:   %c\r\n", 'B');
        DebugP_log ("Arr_1_Glob[8]:       %d\r\n", inst.Arr_1_Glob[8]);
        DebugP_log ("        should be:   %d\r\n", 7);
        DebugP_log ("Arr_2_Glob[8][7]:    %d\r\n", inst.Arr_2_Glob[8][7]);
        DebugP_log ("        should be:   Number_Of_Runs + 10\r\n");
        DebugP_log ("Ptr_Glob->\r\n");
        DebugP_log ("  Ptr_Comp:          %d\r\n", (int) inst.Ptr_Glob->Ptr_Comp);
        DebugP_log ("        should be:   (implementation-dependent)\r\n");
        DebugP_log ("  Discr:             %d\r\n", inst.Ptr_Glob->Discr);
        DebugP_log ("        should be:   %d\r\n", 0);
        DebugP_log ("  Enum_Comp:         %d\r\n", inst.Ptr_Glob->variant.var_1.Enum_Comp);
        DebugP_log ("        should be:   %d\r\n", 2);
        DebugP_log ("  Int_Comp:          %d\r\n", inst.Ptr_Glob->variant.var_1.Int_Comp);
        DebugP_log ("        should be:   %d\r\n", 17);
        DebugP_log ("  Str_Comp:          %s\r\n", inst.Ptr_Glob->variant.var_1.Str_Comp);
        DebugP_log ("        should be:   DHRYSTONE PROGRAM, SOME STRING\r\n");
        DebugP_log ("Next_Ptr_Glob->\r\n");
        DebugP_log ("  Ptr_Comp:          %d\r\n", (int) inst.Next_Ptr_Glob->Ptr_Comp);
        DebugP_log ("        should be:   (implementation-dependent), same as above\r\n");
        DebugP_log ("  Discr:             %d\r\n", inst.Next_Ptr_Glob->Discr);
        DebugP_log ("        should be:   %d\r\n", 0);
        DebugP_log ("  Enum_Comp:         %d\r\n", inst.Next_Ptr_Glob->variant.var_1.Enum_Comp);
        DebugP_log ("        should be:   %d\r\n", 1);
        DebugP_log ("  Int_Comp:          %d\r\n", inst.Next_Ptr_Glob->variant.var_1.Int_Comp);
        DebugP_log ("        should be:   %d\r\n", 18);
        DebugP_log ("  Str_Comp:          %s\r\n",
                                        inst.Next_Ptr_Glob->variant.var_1.Str_Comp);
        DebugP_log ("        should be:   DHRYSTONE PROGRAM, SOME STRING\r\n");
        DebugP_log ("Int_1_Loc:           %d\r\n", Int_1_Loc);
        DebugP_log ("        should be:   %d\r\n", 5);
        DebugP_log ("Int_2_Loc:           %d\r\n", Int_2_Loc);
        DebugP_log ("        should be:   %d\r\n", 13);
        DebugP_log ("Int_3_Loc:           %d\r\n", Int_3_Loc);
        DebugP_log ("        should be:   %d\r\n", 7);
        DebugP_log ("Enum_Loc:            %d\r\n", Enum_Loc);
        DebugP_log ("        should be:   %d\r\n", 1);
        DebugP_log ("Str_1_Loc:           %s\r\n", Str_1_Loc);
        DebugP_log ("        should be:   DHRYSTONE PROGRAM, 1'ST STRING\r\n");
        DebugP_log ("Str_2_Loc:           %s\r\n", Str_2_Loc);
        DebugP_log ("        should be:   DHRYSTONE PROGRAM, 2'ND STRING\r\n");
        DebugP_log ("\r\n");
    }
    else
    {
        if (inst.Int_Glob != 5)
        {
            DebugP_log ("Int_Glob:            %d\r\n", inst.Int_Glob);
            DebugP_log ("        should be:   %d\r\n", 5);
            DebugP_assertNoLog(0);
        }
        if (inst.Bool_Glob != 1)
        {
            DebugP_log ("Bool_Glob:           %d\r\n", inst.Bool_Glob);
            DebugP_log ("        should be:   %d\r\n", 1);
            DebugP_assertNoLog(0);
        }
        if (inst.Ch_1_Glob != 'A')
        {
            DebugP_log ("Ch_1_Glob:           %c\r\n", inst.Ch_1_Glob);
            DebugP_log ("        should be:   %c\r\n", 'A');
            DebugP_assertNoLog(0);
        }
        if (inst.Ch_2_Glob != 'B')
        {
            DebugP_log ("Ch_2_Glob:           %c\r\n", inst.Ch_2_Glob);
            DebugP_log ("        should be:   %c\r\n", 'B');
            DebugP_assertNoLog(0);
        }
        if (inst.Arr_1_Glob[8] != 7)
        {
            DebugP_log ("Arr_1_Glob[8]:       %d\r\n", inst.Arr_1_Glob[8]);
            DebugP_log ("        should be:   %d\r\n", 7);
            DebugP_assertNoLog(0);
        }
        if (inst.Arr_2_Glob[8][7] != Number_Of_Runs + 10)
        {
            DebugP_log ("Arr_2_Glob[8][7]:    %d\r\n", inst.Arr_2_Glob[8][7]);
            DebugP_log ("        should be:   Number_Of_Runs + 10\r\n");
            DebugP_assertNoLog(0);
        }
        if (inst.Ptr_Glob->Discr != 0)
        {
            DebugP_log ("  Discr:             %d\r\n", inst.Ptr_Glob->Discr);
            DebugP_log ("        should be:   %d\r\n", 0);
            DebugP_assertNoLog(0);
        }
        if (inst.Ptr_Glob->variant.var_1.Enum_Comp != 2)
        {
            DebugP_log ("  Enum_Comp:         %d\r\n", inst.Ptr_Glob->variant.var_1.Enum_Comp);
            DebugP_log ("        should be:   %d\r\n", 2);
            DebugP_assertNoLog(0);
        }
        if (inst.Ptr_Glob->variant.var_1.Int_Comp != 17)
        {
            DebugP_log ("  Int_Comp:          %d\r\n", inst.Ptr_Glob->variant.var_1.Int_Comp);
            DebugP_log ("        should be:   %d\r\n", 17);
            DebugP_assertNoLog(0);
        }
        if (strcmp(inst.Ptr_Glob->variant.var_1.Str_Comp, "DHRYSTONE PROGRAM, SOME STRING") != 0)
        {
            DebugP_log ("  Str_Comp:          %s\r\n", inst.Ptr_Glob->variant.var_1.Str_Comp);
            DebugP_log ("        should be:   DHRYSTONE PROGRAM, SOME STRING\r\n");
            DebugP_assertNoLog(0);
        }
        if (inst.Next_Ptr_Glob->Discr != 0)
        {
            DebugP_log ("  Discr:             %d\r\n", inst.Next_Ptr_Glob->Discr);
            DebugP_log ("        should be:   %d\r\n", 0);
            DebugP_assertNoLog(0);
        }
        if (inst.Next_Ptr_Glob->variant.var_1.Enum_Comp != 1)
        {
            DebugP_log ("  Enum_Comp:         %d\r\n", inst.Next_Ptr_Glob->variant.var_1.Enum_Comp);
            DebugP_log ("        should be:   %d\r\n", 1);
            DebugP_assertNoLog(0);
        }
        if (inst.Next_Ptr_Glob->variant.var_1.Int_Comp != 18)
        {
            DebugP_log ("  Int_Comp:          %d\r\n", inst.Next_Ptr_Glob->variant.var_1.Int_Comp);
            DebugP_log ("        should be:   %d\r\n", 18);
            DebugP_assertNoLog(0);
        }
        if (strcmp (inst.Next_Ptr_Glob->variant.var_1.Str_Comp, "DHRYSTONE PROGRAM, SOME STRING") != 0)
        {
            DebugP_log ("  Str_Comp:          %s\r\n",
                                        inst.Next_Ptr_Glob->variant.var_1.Str_Comp);
            DebugP_log ("        should be:   DHRYSTONE PROGRAM, SOME STRING\r\n");
            DebugP_assertNoLog(0);
        }
        if (Int_1_Loc != 5)
        {
            DebugP_log ("Int_1_Loc:           %d\r\n", Int_1_Loc);
            DebugP_log ("        should be:   %d\r\n", 5);
            DebugP_assertNoLog(0);
        }
        if (Int_2_Loc != 13)
        {
            DebugP_log ("Int_2_Loc:           %d\r\n", Int_2_Loc);
            DebugP_log ("        should be:   %d\r\n", 13);
            DebugP_assertNoLog(0);
        }
        if (Int_3_Loc != 7)
        {
            DebugP_log ("Int_3_Loc:           %d\r\n", Int_3_Loc);
            DebugP_log ("        should be:   %d\r\n", 7);
            DebugP_assertNoLog(0);
        }
        if (Enum_Loc != 1)
        {
            DebugP_log ("Enum_Loc:            %d\r\n", Enum_Loc);
            DebugP_log ("        should be:   %d\r\n", 1);
            DebugP_assertNoLog(0);
        }
        if (strcmp (Str_1_Loc, "DHRYSTONE PROGRAM, 1'ST STRING") != 0)
        {
            DebugP_log ("Str_1_Loc:           %s\r\n", Str_1_Loc);
            DebugP_log ("        should be:   DHRYSTONE PROGRAM, 1'ST STRING\r\n");
            DebugP_assertNoLog(0);
        }
        if (strcmp (Str_2_Loc, "DHRYSTONE PROGRAM, 2'ND STRING") != 0)
        {
            DebugP_log ("Str_2_Loc:           %s\r\n", Str_2_Loc);
            DebugP_log ("        should be:   DHRYSTONE PROGRAM, 2'ND STRING\r\n");
            DebugP_assertNoLog(0);
        }
    }

    free(inst.Next_Ptr_Glob);
    free(inst.Ptr_Glob);

    User_Time = End_Time - Begin_Time;

    if (User_Time < Too_Small_Time)
    {
        DebugP_log ("Measured time too small to obtain meaningful results\r\n");
        DebugP_log ("Please increase number of runs\r\n");
        DebugP_log ("\r\n");

        DebugP_assertNoLog(0);
    }
    else
    {
        Microseconds = User_Time / Number_Of_Runs;
        Dhrystones_Per_Second = ( Number_Of_Runs * Mic_secs_Per_Second) / ( User_Time );

        if (dhryLogEnable)
        {
            DebugP_log ("Microseconds for one run through Dhrystone: ");
            //DebugP_log ("%6.1f \r\n", Microseconds);
            DebugP_log ("%d \r\n", (int)Microseconds);
            DebugP_log ("Dhrystones per Second:                      ");
            //DebugP_log ("%6.1f \r\n", Dhrystones_Per_Second);
            DebugP_log ("%d \r\n", (int)Dhrystones_Per_Second);
            DebugP_log ("\r\n");
        }
    }

    return NULL;
}

void Proc_1 (dhryInstance_t *inst, REG Rec_Pointer Ptr_Val_Par)
{
    REG Rec_Pointer Next_Record = Ptr_Val_Par->Ptr_Comp;
                                            /* == Ptr_Glob_Next */
    /* Local variable, initialized with Ptr_Val_Par->Ptr_Comp,    */
    /* corresponds to "rename" in Ada, "with" in Pascal           */

    structassign (*Ptr_Val_Par->Ptr_Comp, *inst->Ptr_Glob);
    Ptr_Val_Par->variant.var_1.Int_Comp = 5;
    Next_Record->variant.var_1.Int_Comp
            = Ptr_Val_Par->variant.var_1.Int_Comp;
    Next_Record->Ptr_Comp = Ptr_Val_Par->Ptr_Comp;
    Proc_3 (inst, &Next_Record->Ptr_Comp);
        /* Ptr_Val_Par->Ptr_Comp->Ptr_Comp
                            == Ptr_Glob->Ptr_Comp */
    if (Next_Record->Discr == Ident_1)
        /* then, executed */
    {
        Next_Record->variant.var_1.Int_Comp = 6;
        Proc_6 (inst, Ptr_Val_Par->variant.var_1.Enum_Comp,
            &Next_Record->variant.var_1.Enum_Comp);
        Next_Record->Ptr_Comp = inst->Ptr_Glob->Ptr_Comp;
        Proc_7 (Next_Record->variant.var_1.Int_Comp, 10,
            &Next_Record->variant.var_1.Int_Comp);
    }
    else /* not executed */
        structassign (*Ptr_Val_Par, *Ptr_Val_Par->Ptr_Comp);
} /* Proc_1 */

void Proc_2 (dhryInstance_t *inst, One_Fifty *Int_Par_Ref)
/******************/
    /* executed once */
    /* *Int_Par_Ref == 1, becomes 4 */
{
    One_Fifty  Int_Loc;
    Enumeration   Enum_Loc;

    Int_Loc = *Int_Par_Ref + 10;
    do /* executed once */
    {
        if (inst->Ch_1_Glob == 'A')
        /* then, executed */
        {
            Int_Loc -= 1;
            *Int_Par_Ref = Int_Loc - inst->Int_Glob;
            Enum_Loc = Ident_1;
        } /* if */
    } while (Enum_Loc != Ident_1); /* true */
} /* Proc_2 */

void Proc_3 (dhryInstance_t *inst, Rec_Pointer *Ptr_Ref_Par)
/******************/
    /* executed once */
    /* Ptr_Ref_Par becomes Ptr_Glob */
{
    if (inst->Ptr_Glob != Null)
    /* then, executed */
    {
        *Ptr_Ref_Par = inst->Ptr_Glob->Ptr_Comp;
    }
    Proc_7 (10, inst->Int_Glob, &inst->Ptr_Glob->variant.var_1.Int_Comp);
} /* Proc_3 */

void Proc_4 (dhryInstance_t *inst) /* without parameters */
/*******/
    /* executed once */
{
    Boolean Bool_Loc;

    Bool_Loc = inst->Ch_1_Glob == 'A';
    inst->Bool_Glob = Bool_Loc | inst->Bool_Glob;
    inst->Ch_2_Glob = 'B';
} /* Proc_4 */

void Proc_5 (dhryInstance_t *inst) /* without parameters */
/*******/
    /* executed once */
{
    inst->Ch_1_Glob = 'A';
    inst->Bool_Glob = false;
} /* Proc_5 */

void Proc_6 (dhryInstance_t *inst, Enumeration Enum_Val_Par, Enumeration *Enum_Ref_Par)
/*********************************/
    /* executed once */
    /* Enum_Val_Par == Ident_3, Enum_Ref_Par becomes Ident_2 */

{
    *Enum_Ref_Par = Enum_Val_Par;
    if (! Func_3 (Enum_Val_Par))
    /* then, not executed */
    {
        *Enum_Ref_Par = Ident_4;
    }
    switch (Enum_Val_Par)
    {
        case Ident_1:
            *Enum_Ref_Par = Ident_1;
            break;
        case Ident_2:
            if (inst->Int_Glob > 100)
            /* then */
            {
                *Enum_Ref_Par = Ident_1;
            }
            else
            {
                *Enum_Ref_Par = Ident_4;
            }
            break;
        case Ident_3: /* executed */
            *Enum_Ref_Par = Ident_2;
            break;
        case Ident_4:
            break;
        case Ident_5:
            *Enum_Ref_Par = Ident_3;
            break;
    } /* switch */
} /* Proc_6 */

void Proc_7 (One_Fifty Int_1_Par_Val, One_Fifty Int_2_Par_Val, One_Fifty *Int_Par_Ref)
/**********************************************/
    /* executed three times                                      */
    /* first call:      Int_1_Par_Val == 2, Int_2_Par_Val == 3,  */
    /*                  Int_Par_Ref becomes 7                    */
    /* second call:     Int_1_Par_Val == 10, Int_2_Par_Val == 5, */
    /*                  Int_Par_Ref becomes 17                   */
    /* third call:      Int_1_Par_Val == 6, Int_2_Par_Val == 10, */
    /*                  Int_Par_Ref becomes 18                   */
{
    One_Fifty Int_Loc;

    Int_Loc = Int_1_Par_Val + 2;
    *Int_Par_Ref = Int_2_Par_Val + Int_Loc;
} /* Proc_7 */

void Proc_8 (dhryInstance_t *inst, Arr_1_Dim Arr_1_Par_Ref, Arr_2_Dim Arr_2_Par_Ref, int Int_1_Par_Val, int Int_2_Par_Val)
/*********************************************************************/
    /* executed once      */
    /* Int_Par_Val_1 == 3 */
    /* Int_Par_Val_2 == 7 */
{
    REG One_Fifty Int_Index;
    REG One_Fifty Int_Loc;

    Int_Loc = Int_1_Par_Val + 5;
    Arr_1_Par_Ref [Int_Loc] = Int_2_Par_Val;
    Arr_1_Par_Ref [Int_Loc+1] = Arr_1_Par_Ref [Int_Loc];
    Arr_1_Par_Ref [Int_Loc+30] = Int_Loc;
    for (Int_Index = Int_Loc; Int_Index <= Int_Loc+1; ++Int_Index)
    {
        Arr_2_Par_Ref [Int_Loc] [Int_Index] = Int_Loc;
    }
    Arr_2_Par_Ref [Int_Loc] [Int_Loc-1] += 1;
    Arr_2_Par_Ref [Int_Loc+20] [Int_Loc] = Arr_1_Par_Ref [Int_Loc];
    inst->Int_Glob = 5;
} /* Proc_8 */


Enumeration Func_1 (dhryInstance_t *inst, Capital_Letter Ch_1_Par_Val, Capital_Letter Ch_2_Par_Val)
/*************************************************/
    /* executed three times                                         */
    /* first call:      Ch_1_Par_Val == 'H', Ch_2_Par_Val == 'R'    */
    /* second call:     Ch_1_Par_Val == 'A', Ch_2_Par_Val == 'C'    */
    /* third call:      Ch_1_Par_Val == 'B', Ch_2_Par_Val == 'C'    */
{
    Capital_Letter        Ch_1_Loc;
    Capital_Letter        Ch_2_Loc;

    Ch_1_Loc = Ch_1_Par_Val;
    Ch_2_Loc = Ch_1_Loc;
    if (Ch_2_Loc != Ch_2_Par_Val)
    /* then, executed */
    {
        return (Ident_1);
    }
    else  /* not executed */
    {
        inst->Ch_1_Glob = Ch_1_Loc;
        return (Ident_2);
    }
} /* Func_1 */


Boolean Func_2 (dhryInstance_t *inst, Str_30 Str_1_Par_Ref, Str_30 Str_2_Par_Ref)
/*************************************************/
    /* executed once */
    /* Str_1_Par_Ref == "DHRYSTONE PROGRAM, 1'ST STRING" */
    /* Str_2_Par_Ref == "DHRYSTONE PROGRAM, 2'ND STRING" */
{
    REG One_Thirty        Int_Loc;
        Capital_Letter    Ch_Loc;

    Int_Loc = 2;
    while (Int_Loc <= 2) /* loop body executed once */
    {
        if (Func_1 (inst, Str_1_Par_Ref[Int_Loc],
                    Str_2_Par_Ref[Int_Loc+1]) == Ident_1)
        /* then, executed */
        {
            Ch_Loc = 'A';
            Int_Loc += 1;
        } /* if, while */
    }
    if (Ch_Loc >= 'W' && Ch_Loc < 'Z')
    /* then, not executed */
    {
        Int_Loc = 7;
    }
    if (Ch_Loc == 'R')
    /* then, not executed */
    {
        return (true);
    }
    else /* executed */
    {
        if (strcmp (Str_1_Par_Ref, Str_2_Par_Ref) > 0)
        /* then, not executed */
        {
            Int_Loc += 7;
            inst->Int_Glob = Int_Loc;
            return (true);
        }
        else /* executed */
        {
            return (false);
        }
    } /* if Ch_Loc */
} /* Func_2 */


Boolean Func_3 (Enumeration Enum_Par_Val)
/***************************/
    /* executed once        */
    /* Enum_Par_Val == Ident_3 */
{
    Enumeration Enum_Loc;

    Enum_Loc = Enum_Par_Val;
    if (Enum_Loc == Ident_3)
    /* then, executed */
    {
        return (true);
    }
    else /* not executed */
    {
        return (false);
    }
} /* Func_3 */

int dhryCreateThreads(uint32_t numThreads, int numIterations, uint32_t *dhryPerSec)
{
    int i = 0, status = SystemP_SUCCESS;
    uint64_t beginTime, endTime, totalTime;

    pthread_t thread_id[32];
    pthread_attr_t attr;
    struct sched_param param;

    nIterations = numIterations;

    memset(thread_id, 0, sizeof(pthread_t) * 32);

    status = pthread_attr_init(&attr);

    if(status == SystemP_SUCCESS)
    {
        status = pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
        if(status != SystemP_SUCCESS)
        {
            DebugP_log ("Error setting thread detach state\r\n");
        }
    }

    if(status == SystemP_SUCCESS)
    {
        memset(&param, 0, sizeof(param));
        param.sched_priority = DEFAULT_PRIORITY;

        status = pthread_attr_setschedparam(&attr, &param);
        if(status != SystemP_SUCCESS)
        {
            DebugP_log ("Error setting thread priority\r\n");
        }
    }

    if(status == SystemP_SUCCESS)
    {
        status = pthread_attr_setstacksize(&attr, (size_t)DEFAULT_STACK_SIZE);
        if(status != SystemP_SUCCESS)
        {
            DebugP_log ("Error setting thread stack size\r\n");
        }
    }

    if(status == SystemP_SUCCESS)
    {
        beginTime = ClockP_getTimeUsec();

        for (i = 0; i < numThreads; i++)
        {
            status = pthread_create(&thread_id[i], &attr, dhryThread, NULL);

            if(status != SystemP_SUCCESS)
            {
                DebugP_log ("Error starting thread \r\n");
                break;
            }
        }
    }

    if(status == SystemP_SUCCESS)
    {
        for (i = 0; i < numThreads; i++)
        {
            status = pthread_join(thread_id[i], NULL);

            if(status != SystemP_SUCCESS)
            {
                DebugP_log ("Error thread join\r\n");
                break;
            }
        }
    }

    if(status == SystemP_SUCCESS)
    {
        endTime = ClockP_getTimeUsec();

        totalTime = endTime - beginTime;

        *dhryPerSec = ( numThreads * numIterations * Mic_secs_Per_Second) /totalTime;
    }

    return status;
}
