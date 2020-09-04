/*
 *  Copyright (c) Texas Instruments Incorporated 2020
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
 *  \file       utils_trace.c
 *
 *  \brief      Trace implementation.
 *
 *              This abstracts and implements the definitions for
 *              user side traces statements and also details
 *              of variable traces supported in existing
 *              implementation.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include "udma_test.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

#if defined (UDMAUT_CFG_TRACE_ENABLE)
/**
 *  \brief      Function to log the trace with zero parameters and just
 *              information string.
 *  \param      mask type of traces.
 *  \param      classType One of three classes where this trace need
 *              to be enabed.
 *  \param      fileName    Where the condition has occured.
 *  \param      lineNum     Line number of the current file where this failure
 *                          has occured.
 *  \param      The debug string.
 */
void
GT_trace0(uint32_t      maskType,
          GT_TraceClass classType,
          const char   *fileName,
          int32_t       lineNum,
          const char   *infoString)
{
    /* Check if trace is enabled. */
    if (((maskType & GT_TRACESTATE_MASK) >> GT_TRACESTATE_SHIFT)
        == GT_TraceState_Enable)
    {
        /* Check if specified class is enabled. */
        if ((maskType & GT_TRACECLASS_MASK) >= classType)
        {
            /* Print if specified class is greater than or equal to class
             * for this specific print.
             */
            if (!((classType == GT_INFO) || (classType == GT_INFO1)))
            {
                DebugP_log("%s @ Line %d: ", fileName, lineNum);
            }
            DebugP_log((const char *) infoString);
        }
    }
}

/**
 *  \brief      Function to log the trace with one additional parameter
 *  \param      mask type of traces
 *  \param      classType One of three classes where this trace
 *              need to be enabed.
 *  \param      fileName    Where the condition has occured.
 *  \param      lineNum     Line number of the current file where this failure
 *                          has occured.
 *  \param      The debug string.
 *  \param      param The additional parameter which needs to be logged.
 */
void
GT_trace1(uint32_t      maskType,
          GT_TraceClass classType,
          const char   *fileName,
          int32_t       lineNum,
          const char   *infoString,
          uintptr_t     param0)
{
    /* Check if trace is enabled. */
    if (((maskType & GT_TRACESTATE_MASK) >> GT_TRACESTATE_SHIFT)
        == GT_TraceState_Enable)
    {
        /* Check if specified class is enabled. */
        if ((maskType & GT_TRACECLASS_MASK) >= classType)
        {
            /* Print if specified class is greater than or equal to class
             * for this specific print.
             */
            if (!((classType == GT_INFO) || (classType == GT_INFO1)))
            {
                DebugP_log("%s @ Line %d: ", fileName, lineNum);
            }
            DebugP_log((const char *) infoString, param0);
        }
    }
}

/**
 *  \brief      Function to log the trace with two additional parameters
 *  \param      mask type of traces
 *  \param      classType One of three classes where this trace
 *              need to be enabed.
 *  \param      fileName    Where the condition has occured.
 *  \param      lineNum     Line number of the current file where this failure
 *                          has occured.
 *  \param      The debug string.
 *  \param      param0 The first parameter which needs to be logged.
 *  \param      param1 The second parameter which needs to be logged.
 */
void
GT_trace2(uint32_t      maskType,
          GT_TraceClass classType,
          const char   *fileName,
          int32_t       lineNum,
          const char   *infoString,
          uintptr_t     param0,
          uintptr_t     param1)
{
    /* Check if trace is enabled. */
    if (((maskType & GT_TRACESTATE_MASK) >> GT_TRACESTATE_SHIFT)
        == GT_TraceState_Enable)
    {
        /* Check if specified class is enabled. */
        if ((maskType & GT_TRACECLASS_MASK) >= classType)
        {
            /* Print if specified class is greater than or equal to class
             * for this specific print.
             */
            if (!((classType == GT_INFO) || (classType == GT_INFO1)))
            {
                DebugP_log("%s @ Line %d: ", fileName, lineNum);
            }
            DebugP_log((const char *) infoString, param0, param1);
        }
    }
}

/**
 *  \brief      Function to log the trace with three parameters.
 *  \param      mask type of traces
 *  \param      classType One of three classes where this trace
 *              need to be enabed.
 *  \param      fileName    Where the condition has occured.
 *  \param      lineNum     Line number of the current file where this failure
 *                          has occured.
 *  \param      The debug string.
 *  \param      param0 The first parameter which needs to be logged.
 *  \param      param1 The second parameter which needs to be logged.
 *  \param      param2 The third parameter which needs to be logged.
 */
void
GT_trace3(uint32_t      maskType,
          GT_TraceClass classType,
          const char   *fileName,
          int32_t       lineNum,
          const char   *infoString,
          uintptr_t     param0,
          uintptr_t     param1,
          uintptr_t     param2)
{
    /* Check if trace is enabled. */
    if (((maskType & GT_TRACESTATE_MASK) >> GT_TRACESTATE_SHIFT)
        == GT_TraceState_Enable)
    {
        /* Check if specified class is enabled. */
        if ((maskType & GT_TRACECLASS_MASK) >= classType)
        {
            /* Print if specified class is greater than or equal to class
             * for this specific print.
             */
            if (!((classType == GT_INFO) || (classType == GT_INFO1)))
            {
                DebugP_log("%s @ Line %d: ", fileName, lineNum);
            }
            DebugP_log((const char *) infoString, param0, param1, param2);
        }
    }
}

/**
 *  \brief      Function to log the trace with four parameters.
 *  \param      mask type of traces
 *  \param      classType One of three classes where this trace
 *              need to be enabed.
 *  \param      fileName    Where the condition has occured.
 *  \param      lineNum     Line number of the current file where this failure
 *                          has occured.
 *  \param      The debug string.
 *  \param      param0 The first parameter which needs to be logged.
 *  \param      param1 The second parameter which needs to be logged.
 *  \param      param2 The third parameter which needs to be logged.
 *  \param      param3 The fourth parameter which needs to be logged.
 */
void
GT_trace4(uint32_t      maskType,
          GT_TraceClass classType,
          const char   *fileName,
          int32_t       lineNum,
          const char   *infoString,
          uintptr_t     param0,
          uintptr_t     param1,
          uintptr_t     param2,
          uintptr_t     param3)
{
    /* Check if trace is enabled. */
    if (((maskType & GT_TRACESTATE_MASK) >> GT_TRACESTATE_SHIFT)
        == GT_TraceState_Enable)
    {
        /* Check if specified class is enabled. */
        if ((maskType & GT_TRACECLASS_MASK) >= classType)
        {
            /* Print if specified class is greater than or equal to class
             * for this specific print.
             */
            if (!((classType == GT_INFO) || (classType == GT_INFO1)))
            {
                DebugP_log("%s @ Line %d: ", fileName, lineNum);
            }
            DebugP_log(
                (const char *) infoString, param0, param1, param2, param3);
        }
    }
}

/**
 *  \brief      Function to log the trace with five parameters.
 *  \param      mask type of traces
 *  \param      classType One of three classes where this trace
 *              need to be enabed.
 *  \param      fileName    Where the condition has occured.
 *  \param      lineNum     Line number of the current file where this failure
 *                          has occured.
 *  \param      The debug string.
 *  \param      param0 The first parameter which needs to be logged.
 *  \param      param1 The second parameter which needs to be logged.
 *  \param      param2 The third parameter which needs to be logged.
 *  \param      param3 The fourth parameter which needs to be logged.
 *  \param      param4 The fifth parameter which needs to be logged.
 */
void
GT_trace5(uint32_t      maskType,
          GT_TraceClass classType,
          const char   *fileName,
          int32_t       lineNum,
          const char   *infoString,
          uintptr_t     param0,
          uintptr_t     param1,
          uintptr_t     param2,
          uintptr_t     param3,
          uintptr_t     param4)
{
    /* Check if trace is enabled. */
    if (((maskType & GT_TRACESTATE_MASK) >> GT_TRACESTATE_SHIFT)
        == GT_TraceState_Enable)
    {
        /* Check if specified class is enabled. */
        if ((maskType & GT_TRACECLASS_MASK) >= classType)
        {
            /* Print if specified class is greater than or equal to class
             * for this specific print.
             */
            if (!((classType == GT_INFO) || (classType == GT_INFO1)))
            {
                DebugP_log("%s @ Line %d: ", fileName, lineNum);
            }
            DebugP_log(
                (const char *) infoString,
                param0, param1, param2, param3, param4);
        }
    }
}

/**
 *  \brief      Function to log the trace with six parameters.
 *  \param      mask type of traces
 *  \param      classType One of three classes where this trace
 *              need to be enabed.
 *  \param      fileName    Where the condition has occured.
 *  \param      lineNum     Line number of the current file where this failure
 *                          has occured.
 *  \param      The debug string.
 *  \param      param0 The first parameter which needs to be logged.
 *  \param      param1 The second parameter which needs to be logged.
 *  \param      param2 The third parameter which needs to be logged.
 *  \param      param3 The fourth parameter which needs to be logged.
 *  \param      param4 The fifth parameter which needs to be logged.
 *  \param      param5 The sixth parameter which needs to be logged.
 */
void
GT_trace6(uint32_t      maskType,
          GT_TraceClass classType,
          const char   *fileName,
          int32_t       lineNum,
          const char   *infoString,
          uintptr_t     param0,
          uintptr_t     param1,
          uintptr_t     param2,
          uintptr_t     param3,
          uintptr_t     param4,
          uintptr_t     param5)
{
    /* Check if trace is enabled. */
    if (((maskType & GT_TRACESTATE_MASK) >> GT_TRACESTATE_SHIFT)
        == GT_TraceState_Enable)
    {
        /* Check if specified class is enabled. */
        if ((maskType & GT_TRACECLASS_MASK) >= classType)
        {
            /* Print if specified class is greater than or equal to class
             * for this specific print.
             */
            if (!((classType == GT_INFO) || (classType == GT_INFO1)))
            {
                DebugP_log("%s @ Line %d: ", fileName, lineNum);
            }
            DebugP_log(
                (const char *) infoString,
                param0, param1, param2, param3, param4, param5);
        }
    }
}

/**
 *  \brief      Function to log the trace with seven parameters.
 *  \param      mask type of traces
 *  \param      classType One of three classes where this trace
 *              need to be enabed.
 *  \param      fileName    Where the condition has occured.
 *  \param      lineNum     Line number of the current file where this failure
 *                          has occured.
 *  \param      The debug string.
 *  \param      param0 The first parameter which needs to be logged.
 *  \param      param1 The second parameter which needs to be logged.
 *  \param      param2 The third parameter which needs to be logged.
 *  \param      param3 The fourth parameter which needs to be logged.
 *  \param      param4 The fifth parameter which needs to be logged.
 *  \param      param5 The sixth parameter which needs to be logged.
 *  \param      param6 The sixth parameter which needs to be logged.
 */
void
GT_trace7(uint32_t      maskType,
          GT_TraceClass classType,
          const char   *fileName,
          int32_t       lineNum,
          const char   *infoString,
          uintptr_t     param0,
          uintptr_t     param1,
          uintptr_t     param2,
          uintptr_t     param3,
          uintptr_t     param4,
          uintptr_t     param5,
          uintptr_t     param6)
{
    /* Check if trace is enabled. */
    if (((maskType & GT_TRACESTATE_MASK) >> GT_TRACESTATE_SHIFT)
        == GT_TraceState_Enable)
    {
        /* Check if specified class is enabled. */
        if ((maskType & GT_TRACECLASS_MASK) >= classType)
        {
            /* Print if specified class is greater than or equal to class
             * for this specific print.
             */
            if (!((classType == GT_INFO) || (classType == GT_INFO1)))
            {
                DebugP_log("%s @ Line %d: ", fileName, lineNum);
            }
            DebugP_log(
                (const char *) infoString,
                param0, param1, param2, param3, param4, param5, param6);
        }
    }
}
#endif

#if defined (UDMAUT_CFG_ASSERT_ENABLE)
void GT_assertLocal(uint32_t    enableMask,
                    uint32_t    condition,
                    const char *str,
                    const char *fileName,
                    int32_t     lineNum)
{
    volatile uint32_t loop = 1U;
    if (condition == 0U)
    {
        GT_3trace(
            GT_DEFAULT_MASK, GT_ERR,
            " Assertion @ Line: %d in %s: %s : failed !!!\n",
            lineNum, fileName, str);
        while (loop == 1U)
        {
            ;
        }
    }

    return;
}
#endif  /* if defined(UDMAUT_CFG_ASSERT_ENABLE) */
