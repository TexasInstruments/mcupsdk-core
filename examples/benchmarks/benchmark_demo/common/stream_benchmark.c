/*-----------------------------------------------------------------------*/
/* Program: STREAM                                                       */
/* Revision: $Id: stream.c,v 5.10 2013/01/17 16:01:06 mccalpin Exp mccalpin $ */
/* Original code developed by John D. McCalpin                           */
/* Programmers: John D. McCalpin                                         */
/*              Joe R. Zagar                                             */
/*                                                                       */
/* This program measures memory transfer rates in MB/s for simple        */
/* computational kernels coded in C.                                     */
/*-----------------------------------------------------------------------*/
/* Copyright 1991-2013: John D. McCalpin                                 */
/*-----------------------------------------------------------------------*/
/* License:                                                              */
/*  1. You are free to use this program and/or to redistribute           */
/*     this program.                                                     */
/*  2. You are free to modify this program for your own use,             */
/*     including commercial use, subject to the publication              */
/*     restrictions in item 3.                                           */
/*  3. You are free to publish results obtained from running this        */
/*     program, or from works that you derive from this program,         */
/*     with the following limitations:                                   */
/*     3a. In order to be referred to as "STREAM benchmark results",     */
/*         published results must be in conformance to the STREAM        */
/*         Run Rules, (briefly reviewed below) published at              */
/*         http://www.cs.virginia.edu/stream/ref.html                    */
/*         and incorporated herein by reference.                         */
/*         As the copyright holder, John McCalpin retains the            */
/*         right to determine conformity with the Run Rules.             */
/*     3b. Results based on modified source code or on runs not in       */
/*         accordance with the STREAM Run Rules must be clearly          */
/*         labelled whenever they are published.  Examples of            */
/*         proper labelling include:                                     */
/*           "tuned STREAM benchmark results"                            */
/*           "based on a variant of the STREAM benchmark code"           */
/*         Other comparable, clear, and reasonable labelling is          */
/*         acceptable.                                                   */
/*     3c. Submission of results to the STREAM benchmark web site        */
/*         is encouraged, but not required.                              */
/*  4. Use of this program or creation of derived works based on this    */
/*     program constitutes acceptance of these licensing restrictions.   */
/*  5. Absolutely no warranty is expressed or implied.                   */
/*-----------------------------------------------------------------------*/
# include <stdio.h>
# include <math.h>
# include <float.h>
# include <limits.h>
#include <kernel/dpl/CycleCounterP.h>
#include "benchmarkdemo.h"

/*-----------------------------------------------------------------------
 * INSTRUCTIONS:
 *
 *  1) STREAM requires different amounts of memory to run on different
 *           systems, depending on both the system cache size(s) and the
 *           granularity of the system timer.
 *     You should adjust the value of 'STREAM_ARRAY_SIZE' (below)
 *           to meet *both* of the following criteria:
 *       (a) Each array must be at least 4 times the size of the
 *           available cache memory. I don't worry about the difference
 *           between 10^6 and 2^20, so in practice the minimum array size
 *           is about 3.8 times the cache size.
 *           Example 1: One Xeon E3 with 8 MB L3 cache
 *               STREAM_ARRAY_SIZE should be >= 4 million, giving
 *               an array size of 30.5 MB and a total memory requirement
 *               of 91.5 MB.
 *           Example 2: Two Xeon E5's with 20 MB L3 cache each (using OpenMP)
 *               STREAM_ARRAY_SIZE should be >= 20 million, giving
 *               an array size of 153 MB and a total memory requirement
 *               of 458 MB.
 *       (b) The size should be large enough so that the 'timing calibration'
 *           output by the program is at least 20 clock-ticks.
 *           Example: most versions of Windows have a 10 millisecond timer
 *               granularity.  20 "ticks" at 10 ms/tic is 200 milliseconds.
 *               If the chip is capable of 10 GB/s, it moves 2 GB in 200 msec.
 *               This means the each array must be at least 1 GB, or 128M elements.
 *
 *      Version 5.10 increases the default array size from 2 million
 *          elements to 10 million elements in response to the increasing
 *          size of L3 caches.  The new default size is large enough for caches
 *          up to 20 MB.
 *      Version 5.10 changes the loop index variables from "register int"
 *          to "ssize_t", which allows array indices >2^32 (4 billion)
 *          on properly configured 64-bit systems.  Additional compiler options
 *          (such as "-mcmodel=medium") may be required for large memory runs.
 *
 *      Array size can be set at compile time without modifying the source
 *          code for the (many) compilers that support preprocessor definitions
 *          on the compile line.  E.g.,
 *                gcc -O -DSTREAM_ARRAY_SIZE=100000000 stream.c -o stream.100M
 *          will override the default size of 10M with a new size of 100M elements
 *          per array.
 */
#ifndef STREAM_ARRAY_SIZE
#   define STREAM_ARRAY_SIZE    16384
#endif

/*  2) STREAM runs each kernel "NTIMES" times and reports the *best* result
 *         for any iteration after the first, therefore the minimum value
 *         for NTIMES is 2.
 *      There are no rules on maximum allowable values for NTIMES, but
 *         values larger than the default are unlikely to noticeably
 *         increase the reported performance.
 *      NTIMES can also be set on the compile line without changing the source
 *         code using, for example, "-DNTIMES=7".
 */
#ifdef NTIMES
#if NTIMES<=1
#   define NTIMES   10
#endif
#endif
#ifndef NTIMES
#   define NTIMES   10
#endif

/*  Users are allowed to modify the "OFFSET" variable, which *may* change the
 *         relative alignment of the arrays (though compilers may change the
 *         effective offset by making the arrays non-contiguous on some systems).
 *      Use of non-zero values for OFFSET can be especially helpful if the
 *         STREAM_ARRAY_SIZE is set to a value close to a large power of 2.
 *      OFFSET can also be set on the compile line without changing the source
 *         code using, for example, "-DOFFSET=56".
 */
#ifndef OFFSET
#   define OFFSET   0
#endif

/*
 *  3) Compile the code with optimization.  Many compilers generate
 *       unreasonably bad code before the optimizer tightens things up.
 *     If the results are unreasonably good, on the other hand, the
 *       optimizer might be too smart for me!
 *
 *     For a simple single-core version, try compiling with:
 *            cc -O stream.c -o stream
 *     This is known to work on many, many systems....
 *
 *     To use multiple cores, you need to tell the compiler to obey the OpenMP
 *       directives in the code.  This varies by compiler, but a common example is
 *            gcc -O -fopenmp stream.c -o stream_omp
 *       The environment variable OMP_NUM_THREADS allows runtime control of the
 *         number of threads/cores used when the resulting "stream_omp" program
 *         is executed.
 *
 *     To run with single-precision variables and arithmetic, simply add
 *         -DSTREAM_TYPE=float
 *     to the compile line.
 *     Note that this changes the minimum array sizes required --- see (1) above.
 *
 *     The preprocessor directive "TUNED" does not do much -- it simply causes the
 *       code to call separate functions to execute each kernel.  Trivial versions
 *       of these functions are provided, but they are *not* tuned -- they just
 *       provide predefined interfaces to be replaced with tuned code.
 *
 *
 *  4) Optional: Mail the results to mccalpin@cs.virginia.edu
 *     Be sure to include info that will help me understand:
 *      a) the computer hardware configuration (e.g., processor model, memory type)
 *      b) the compiler name/version and compilation flags
 *      c) any run-time information (such as OMP_NUM_THREADS)
 *      d) all of the output from the test case.
 *
 * Thanks!
 *
 *-----------------------------------------------------------------------*/

# define HLINE "-------------------------------------------------------------\r\n"

# ifndef MIN
# define MIN(x,y) ((x)<(y)?(x):(y))
# endif
# ifndef MAX
# define MAX(x,y) ((x)>(y)?(x):(y))
# endif

#ifndef STREAM_TYPE
#define STREAM_TYPE double
#endif

static STREAM_TYPE  a[STREAM_ARRAY_SIZE+OFFSET] __attribute__((aligned(128), section(".bss.stream_mem"))),
            b[STREAM_ARRAY_SIZE+OFFSET] __attribute__((aligned(128), section(".bss.stream_mem"))),
            c[STREAM_ARRAY_SIZE+OFFSET] __attribute__((aligned(128), section(".bss.stream_mem")));

double   avgtime[4] = {0}, maxtime[4] = {0},
        mintime[4] = {FLT_MAX,FLT_MAX,FLT_MAX,FLT_MAX};

char *label[4] = {"Copy:      ", "Scale:     ",
    "Add:       ", "Triad:     "};

double   bytes[4] = {
    2 * sizeof(STREAM_TYPE) * STREAM_ARRAY_SIZE,
    2 * sizeof(STREAM_TYPE) * STREAM_ARRAY_SIZE,
    3 * sizeof(STREAM_TYPE) * STREAM_ARRAY_SIZE,
    3 * sizeof(STREAM_TYPE) * STREAM_ARRAY_SIZE
    };

extern double mysecond();
extern void checkSTREAMresults();
#ifdef TUNED
extern void tuned_STREAM_Copy();
extern void tuned_STREAM_Scale(STREAM_TYPE scalar);
extern void tuned_STREAM_Add();
extern void tuned_STREAM_Triad(STREAM_TYPE scalar);
#endif
#ifdef _OPENMP
extern int omp_get_num_threads();
#endif
int stream_run()
{
    int         checktick();
    int         k;
    size_t     j;
    STREAM_TYPE     scalar;
    double      times[4][NTIMES];
    uint32_t start, end;

    /*  --- MAIN LOOP --- repeat test cases NTIMES times --- */

    scalar = 3.0;

    CycleCounterP_reset();
    start = CycleCounterP_getCount32();

    for (k=0; k<NTIMES; k++)
    {
        times[0][k] = mysecond();
#ifdef TUNED
        tuned_STREAM_Copy();
#else
#pragma omp parallel for
        for (j=0; j<STREAM_ARRAY_SIZE; j++)
            c[j] = a[j];
#endif
        times[0][k] = mysecond() - times[0][k];

        times[1][k] = mysecond();
#ifdef TUNED
        tuned_STREAM_Scale(scalar);
#else
#pragma omp parallel for
        for (j=0; j<STREAM_ARRAY_SIZE; j++)
            b[j] = scalar*c[j];
#endif
        times[1][k] = mysecond() - times[1][k];

        times[2][k] = mysecond();
#ifdef TUNED
        tuned_STREAM_Add();
#else
#pragma omp parallel for
        for (j=0; j<STREAM_ARRAY_SIZE; j++)
            c[j] = a[j]+b[j];
#endif
        times[2][k] = mysecond() - times[2][k];

        times[3][k] = mysecond();
#ifdef TUNED
        tuned_STREAM_Triad(scalar);
#else
#pragma omp parallel for
        for (j=0; j<STREAM_ARRAY_SIZE; j++)
            a[j] = b[j]+scalar*c[j];
#endif
        times[3][k] = mysecond() - times[3][k];
    }

    end = CycleCounterP_getCount32();

    /*  --- SUMMARY --- */

    for (k=1; k<NTIMES; k++) /* note -- skip first iteration */
    {
        for (j=0; j<4; j++)
        {
            avgtime[j] = avgtime[j] + times[j][k];
            mintime[j] = MIN(mintime[j], times[j][k]);
            maxtime[j] = MAX(maxtime[j], times[j][k]);
        }
    }

    for (j=0; j<4; j++) {
        avgtime[j] = avgtime[j]/(double)(NTIMES-1);
    }

    App_statsUpdate(start, end);
    return 0;
}

# define    M   20

int checktick()
{
    int     i, minDelta, Delta;
    double  t1, t2, timesfound[M];

/*  Collect a sequence of M unique time values from the system. */

    for (i = 0; i < M; i++) {
        t1 = mysecond();
        while( ((t2=mysecond()) - t1) < 1.0E-6 )
        {
        }
        timesfound[i] = t1 = t2;
    }

/*
 * Determine the minimum difference between these M values.
 * This result will be our estimate (in microseconds) for the
 * clock granularity.
 */

    minDelta = 1000000;
    for (i = 1; i < M; i++) {
        Delta = (int)( 1.0E6 * (timesfound[i]-timesfound[i-1]));
        minDelta = MIN(minDelta, MAX(Delta,0));
    }

   return(minDelta);
}

/* A gettimeofday routine to give access to the wall
   clock timer on most UNIX-like systems.  */

/* Creating custom mysecond() function that uses PMU Cycle Counter */
double mysecond()
{
    double volatile timeval = (double) CycleCounterP_getCount32();
    double volatile retval = timeval / 800000000; //R5F @ 800MHz

    return ( retval );
}

int stream_init()
{
    int         quantum, checktick();
    int         BytesPerWord;
    size_t     j;
    double     t;

    /* --- SETUP --- determine precision and check timing --- */

    CycleCounterP_reset();

    DebugP_log(HLINE);
    DebugP_log("STREAM version $Revision: 5.10 $\r\n");
    DebugP_log(HLINE);
    BytesPerWord = sizeof(STREAM_TYPE);
    DebugP_log("This system uses %d bytes per array element.\r\n",
    BytesPerWord);

    DebugP_log(HLINE);
#ifdef N
    DebugP_log("*****  WARNING: ******\r\n");
    DebugP_log("      It appears that you set the preprocessor variable N when compiling this code.\r\n");
    DebugP_log("      This version of the code uses the preprocesor variable STREAM_ARRAY_SIZE to control the array size\r\n");
    DebugP_log("      Reverting to default value of STREAM_ARRAY_SIZE=%llu\r\n",(unsigned long long) STREAM_ARRAY_SIZE);
    DebugP_log("*****  WARNING: ******\r\n");
#endif

    DebugP_log("Array size = %llu (elements), Offset = %d (elements)\r\n" , (unsigned long long) STREAM_ARRAY_SIZE, OFFSET);
    DebugP_log("Memory per array = %.1f MiB (= %.1f GiB).\r\n",
    BytesPerWord * ( (double) STREAM_ARRAY_SIZE / 1024.0/1024.0),
    BytesPerWord * ( (double) STREAM_ARRAY_SIZE / 1024.0/1024.0/1024.0));
    DebugP_log("Total memory required = %.1f MiB (= %.1f GiB).\r\n",
    (3.0 * BytesPerWord) * ( (double) STREAM_ARRAY_SIZE / 1024.0/1024.),
    (3.0 * BytesPerWord) * ( (double) STREAM_ARRAY_SIZE / 1024.0/1024./1024.));
    DebugP_log("Each kernel will be executed %d times.\r\n", NTIMES);
    DebugP_log(" The *best* time for each kernel (excluding the first iteration)\r\n");
    DebugP_log(" will be used to compute the reported bandwidth.\r\n");

#ifdef _OPENMP
    DebugP_log(HLINE);
#pragma omp parallel
    {
#pragma omp master
        {
            k = omp_get_num_threads();
            DebugP_log ("Number of Threads requested = %i\r\n",k);
        }
    }
#endif

#ifdef _OPENMP
    k = 0;
#pragma omp parallel
#pragma omp atomic
    k++;
    DebugP_log ("Number of Threads counted = %i\r\n",k);
#endif

    /* Get initial value for system clock. */
#pragma omp parallel for
    for (j=0; j<STREAM_ARRAY_SIZE; j++) {
        a[j] = 1.0;
        b[j] = 2.0;
        c[j] = 0.0;
    }

    DebugP_log(HLINE);

    if  ( (quantum = checktick()) >= 1)
        DebugP_log("Your clock granularity/precision appears to be "
            "%d microseconds.\r\n", quantum);
    else {
        DebugP_log("Your clock granularity appears to be "
            "less than one microsecond.\r\n");
        quantum = 1;
    }

    t = mysecond();
#pragma omp parallel for
    for (j = 0; j < STREAM_ARRAY_SIZE; j++)
        a[j] = 2.0E0 * a[j];
    t = 1.0E6 * (mysecond() - t);

    DebugP_log("Each test below will take on the order"
                " of %d microseconds.\r\n", (int) t  );
    DebugP_log("   (= %d clock ticks)\r\n", (int) (t/quantum) );
    DebugP_log("Increase the size of the arrays if this shows that\r\n");
    DebugP_log("you are not getting at least 20 clock ticks per test.\r\n");

    DebugP_log(HLINE);

    DebugP_log("WARNING -- The above is only a rough guideline.\r\n");
    DebugP_log("For best results, please be sure you know the\r\n");
    DebugP_log("precision of your system timer.\r\n");
    DebugP_log(HLINE);

    return 0;
}

#ifdef TUNED
/* stubs for "tuned" versions of the kernels */
void tuned_STREAM_Copy()
{
    ssize_t j;
#pragma omp parallel for
    for (j=0; j<STREAM_ARRAY_SIZE; j++)
        c[j] = a[j];
}

void tuned_STREAM_Scale(STREAM_TYPE scalar)
{
    ssize_t j;
#pragma omp parallel for
    for (j=0; j<STREAM_ARRAY_SIZE; j++)
        b[j] = scalar*c[j];
}

void tuned_STREAM_Add()
{
    ssize_t j;
#pragma omp parallel for
    for (j=0; j<STREAM_ARRAY_SIZE; j++)
        c[j] = a[j]+b[j];
}

void tuned_STREAM_Triad(STREAM_TYPE scalar)
{
    ssize_t j;
#pragma omp parallel for
    for (j=0; j<STREAM_ARRAY_SIZE; j++)
        a[j] = b[j]+scalar*c[j];
}
/* end of stubs for the "tuned" versions of the kernels */
#endif
