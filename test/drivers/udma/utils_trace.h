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
 *  \file       utils_trace.h
 *
 *  \brief      Kernel Trace enabling/disabling/application interface.
 *
 *              This will have the definitions for kernel side traces
 *              statements and also details of variable traces
 *              supported in existing implementation.
 */

#ifndef UDMAUTTRACE_H_
#define UDMAUTTRACE_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define UDMAUT_CFG_ASSERT_ENABLE
#define UDMAUT_CFG_TRACE_ENABLE

/*!
 *  @def    GT_TRACESTATE_MASK
 *  @brief  Trace state mask
 */
#define GT_TRACESTATE_MASK              (0x0000000FU)
/*!
 *  @def    GT_TRACESTATE_SHIFT
 *  @brief  Bit shift for trace state
 */
#define GT_TRACESTATE_SHIFT             (0U)
/*!
 *  @def    GT_TRACECLASS_MASK
 *  @brief  GT class mask
 */
#define GT_TRACECLASS_MASK              (0x000F0000U)
/*!
 *  @def    GT_TRACECLASS_SHIFT
 *  @brief  Bit shift for GT class mask
 */
#define GT_TRACECLASS_SHIFT             (16U)

/*!
 *  @brief  Default Mask to use with GT_assert
 */
#define GT_DEFAULT_MASK                 (GT_ERR | GT_TraceState_Enable)

/**
 *  \anchor GT_TraceState
 *  \name trace state
 *  @{
 */
/*!
 *  @brief   Enumerates the types of states of trace (enable/disable)
 */
typedef uint32_t GT_TraceState;
#define    GT_TraceState_Disable    (0x00000000U)
/*!< Disable trace */
#define    GT_TraceState_Enable     (0x00000001U)
/*!< Enable trace */
#define    GT_TraceState_EndValue   (0x00000002U)
/*!< End delimiter indicating start of invalid values for this enum */
/* @} */

/**
 *  \anchor GT_TraceClass
 *  \name trace class
 *  @{
 */
/*!
 *  @brief   Enumerates the types of trace classes
 */
typedef uint32_t GT_TraceClass;
#define    GT_ERR                   (0x00010000U)
/*!< Class 1 trace: Used for errors/warnings */
#define    GT_CRIT                  (0x00020000U)
/*!< Class 2 trace: Used for critical information */
#define    GT_INFO                  (0x00030000U)
/*!< Class 3 trace: Used for additional information */
#define    GT_INFO1                 (0x00040000U)
/*!< Class 4 trace: Used for additional information (sub level 1) */
#define    GT_DEBUG                 (0x00050000U)
/*!< Class 5 trace: Used for block level information */
#define    GT_ENTER                 (0x00060000U)
/*!< Indicates a function entry class of trace */
#define    GT_LEAVE                 (0x00070000U)
/*!< Indicates a function leave class of trace */
/* @} */

/**
 *  \brief Prints to Shared memory and CCS console
 *
 *  This function prints the provided formatted string to shared memory and CCS
 *  console
 *
 *  \param format       [IN] Formatted string followed by variable arguments
 *
 */
void UdmaUt_printf(const char *format, ...);

#if defined (UDMAUT_CFG_ASSERT_ENABLE)

/** \brief assert function. */
void GT_assertLocal(uint32_t    enableMask,
                    uint32_t    condition,
                    const char *str,
                    const char *fileName,
                    int32_t     lineNum);

/** \brief GT_assert */
#define GT_assert(x, y)                                           \
    (GT_assertLocal((uint32_t) (x), (uint32_t) (y), (const char *) # y, \
                    (const char *) __FILE__, (int32_t) __LINE__))

#else

#define GT_assert(x, y)

#endif

/**
 *  \brief Log the trace with zero parameters and information string.
 *
 *  \param  maskType   [IN] Trace mask
 *  \param  classType  [IN] Trace class type
 *  \param  fileName   [IN] File Name
 *  \param  lineNum    [IN] Line Number
 *  \param  infoString [IN] Print string
 *
 */
void GT_trace0(uint32_t      maskType,
               GT_TraceClass classType,
               const char   *fileName,
               int32_t       lineNum,
               const char   *infoString);
/**
 *  \brief Function to log the trace with one additional parameter.
 *
 *  \param  maskType   [IN] Trace mask
 *  \param  classType  [IN] Trace class type
 *  \param  fileName   [IN] File Name
 *  \param  lineNum    [IN] Line Number
 *  \param  infoString [IN] Print string
 *  \param  param0     [IN] Parameter 0
 *
 */
void GT_trace1(uint32_t      maskType,
               GT_TraceClass classType,
               const char   *fileName,
               int32_t       lineNum,
               const char   *infoString,
               uintptr_t     param0);

/**
 *  \brief Function to log the trace with two additional parameters.
 *
 *  \param  maskType   [IN] Trace mask
 *  \param  classType  [IN] Trace class type
 *  \param  fileName   [IN] File Name
 *  \param  lineNum    [IN] Line Number
 *  \param  infoString [IN] Print string
 *  \param  param0     [IN] Parameter 0
 *  \param  param1     [IN] Parameter 1
 *
 */
void GT_trace2(uint32_t      maskType,
               GT_TraceClass classType,
               const char   *fileName,
               int32_t       lineNum,
               const char   *infoString,
               uintptr_t     param0,
               uintptr_t     param1);

/**
 *  \brief Function to log the trace with three additional parameters.
 *
 *  \param  maskType   [IN] Trace mask
 *  \param  classType  [IN] Trace class type
 *  \param  fileName   [IN] File Name
 *  \param  lineNum    [IN] Line Number
 *  \param  infoString [IN] Print string
 *  \param  param0     [IN] Parameter 0
 *  \param  param1     [IN] Parameter 1
 *  \param  param2     [IN] Parameter 2
 *
 */
void GT_trace3(uint32_t      maskType,
               GT_TraceClass classType,
               const char   *fileName,
               int32_t       lineNum,
               const char   *infoString,
               uintptr_t     param0,
               uintptr_t     param1,
               uintptr_t     param2);

/**
 *  \brief Function to log the trace with four additional parameters.
 *
 *  \param  maskType   [IN] Trace mask
 *  \param  classType  [IN] Trace class type
 *  \param  fileName   [IN] File Name
 *  \param  lineNum    [IN] Line Number
 *  \param  infoString [IN] Print string
 *  \param  param0     [IN] Parameter 0
 *  \param  param1     [IN] Parameter 1
 *  \param  param2     [IN] Parameter 2
 *  \param  param3     [IN] Parameter 3
 *
 */
void GT_trace4(uint32_t      maskType,
               GT_TraceClass classType,
               const char   *fileName,
               int32_t       lineNum,
               const char   *infoString,
               uintptr_t     param0,
               uintptr_t     param1,
               uintptr_t     param2,
               uintptr_t     param3);

/**
 *  \brief Function to log the trace with five additional parameters.
 *
 *  \param  maskType   [IN] Trace mask
 *  \param  classType  [IN] Trace class type
 *  \param  fileName   [IN] File Name
 *  \param  lineNum    [IN] Line Number
 *  \param  infoString [IN] Print string
 *  \param  param0     [IN] Parameter 0
 *  \param  param1     [IN] Parameter 1
 *  \param  param2     [IN] Parameter 2
 *  \param  param3     [IN] Parameter 3
 *  \param  param4     [IN] Parameter 4
 *
 */
void GT_trace5(uint32_t      maskType,
               GT_TraceClass classType,
               const char   *fileName,
               int32_t       lineNum,
               const char   *infoString,
               uintptr_t     param0,
               uintptr_t     param1,
               uintptr_t     param2,
               uintptr_t     param3,
               uintptr_t     param4);

/**
 *  \brief Function to log the trace with six additional parameters.
 *
 *  \param  maskType   [IN] Trace mask
 *  \param  classType  [IN] Trace class type
 *  \param  fileName   [IN] File Name
 *  \param  lineNum    [IN] Line Number
 *  \param  infoString [IN] Print string
 *  \param  param0     [IN] Parameter 0
 *  \param  param1     [IN] Parameter 1
 *  \param  param2     [IN] Parameter 2
 *  \param  param3     [IN] Parameter 3
 *  \param  param4     [IN] Parameter 4
 *  \param  param5     [IN] Parameter 5
 *
 */
void GT_trace6(uint32_t      maskType,
               GT_TraceClass classType,
               const char   *fileName,
               int32_t       lineNum,
               const char   *infoString,
               uintptr_t     param0,
               uintptr_t     param1,
               uintptr_t     param2,
               uintptr_t     param3,
               uintptr_t     param4,
               uintptr_t     param5);

/**
 *  \brief Function to log the trace with seven additional parameters.
 *
 *  \param  maskType   [IN] Trace mask
 *  \param  classType  [IN] Trace class type
 *  \param  fileName   [IN] File Name
 *  \param  lineNum    [IN] Line Number
 *  \param  infoString [IN] Print string
 *  \param  param0     [IN] Parameter 0
 *  \param  param1     [IN] Parameter 1
 *  \param  param2     [IN] Parameter 2
 *  \param  param3     [IN] Parameter 3
 *  \param  param4     [IN] Parameter 4
 *  \param  param5     [IN] Parameter 5
 *  \param  param6     [IN] Parameter 6
 *
 */
void GT_trace7(uint32_t      maskType,
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
               uintptr_t     param6);

#if defined (UDMAUT_CFG_TRACE_ENABLE)
/** \brief Log the trace with zero parameters and information string. */
#define GT_0trace(maskType, classType, infoString)                          \
    (GT_trace0((maskType), (classType),                                     \
               (const char *) __FILE__, (int32_t) __LINE__,                 \
               (const char *) (infoString)))

/** \brief Function to log the trace with one additional parameters. */
#define GT_1trace(maskType, classType, infoString, param0)                  \
    (GT_trace1((maskType), (classType),                                     \
               (const char *) __FILE__, (int32_t) __LINE__,                 \
               (const char *) (infoString), (uintptr_t) (param0)))

/** \brief Function to log the trace with two additional parameters. */
#define GT_2trace(maskType, classType, infoString, param0, param1)          \
    (GT_trace2((maskType), (classType),                                     \
               (const char *) __FILE__, (int32_t) __LINE__,                 \
               (const char *) (infoString), (uintptr_t) (param0), (uintptr_t) (param1)))

/** \brief Function to log the trace with three additional parameters. */
#define GT_3trace(maskType, classType, infoString, param0, param1, param2)  \
    (GT_trace3((maskType), (classType),                                     \
               (const char *) __FILE__, (int32_t) __LINE__,                 \
               (const char *) (infoString),                                 \
               (uintptr_t) (param0),                                        \
               (uintptr_t) (param1),                                        \
               (uintptr_t) (param2)))

/** \brief Function to log the trace with four additional parameters. */
#define GT_4trace(maskType, classType, infoString,                          \
                  param0, param1, param2, param3)                           \
    (GT_trace4((maskType), (classType),                                     \
               (const char *) __FILE__, (int32_t) __LINE__,                 \
               (const char *) (infoString),                                 \
               (uintptr_t) (param0),                                        \
               (uintptr_t) (param1),                                        \
               (uintptr_t) (param2),                                        \
               (uintptr_t) (param3)))

/** \brief Function to log the trace with five additional parameters. */
#define GT_5trace(maskType, classType, infoString,                          \
                  param0, param1, param2, param3, param4)                   \
    (GT_trace5((maskType), (classType),                                     \
               (const char *) __FILE__, (int32_t) __LINE__,                 \
               (const char *) (infoString),                                 \
               (uintptr_t) (param0),                                        \
               (uintptr_t) (param1),                                        \
               (uintptr_t) (param2),                                        \
               (uintptr_t) (param3),                                        \
               (uintptr_t) (param4)))

/** \brief Function to log the trace with six additional parameters. */
#define GT_6trace(maskType, classType, infoString,                          \
                  param0, param1, param2, param3, param4, param5)           \
    (GT_trace6((maskType), (classType),                                     \
               (const char *) __FILE__, (int32_t) __LINE__,                 \
               (const char *) (infoString),                                 \
               (uintptr_t) (param0),                                        \
               (uintptr_t) (param1),                                        \
               (uintptr_t) (param2),                                        \
               (uintptr_t) (param3),                                        \
               (uintptr_t) (param4),                                        \
               (uintptr_t) (param5)))

/** \brief Function to log the trace with seven additional parameters. */
#define GT_7trace(maskType, classType, infoString,                          \
                  param0, param1, param2, param3, param4, param5, param6)   \
    (GT_trace7((maskType), (classType),                                     \
               (const char *) __FILE__, (int32_t) __LINE__,                 \
               (const char *) (infoString),                                 \
               (uintptr_t) (param0),                                        \
               (uintptr_t) (param1),                                        \
               (uintptr_t) (param2),                                        \
               (uintptr_t) (param3),                                        \
               (uintptr_t) (param4),                                        \
               (uintptr_t) (param5),                                        \
               (uintptr_t) (param6)))

#else   /* if defined (UDMAUT_CFG_TRACE_ENABLE) */

/** \brief Log the trace with zero parameters and information string. */
#define GT_0trace(maskType, classType, infoString)
/** \brief Function to log the trace with one additional parameter. */
#define GT_1trace(maskType, classType, infoString, param0)
/** \brief Function to log the trace with two additional parameters. */
#define GT_2trace(maskType, classType, infoString, param0, param1)
/** \brief Function to log the trace with three additional parameters. */
#define GT_3trace(maskType, classType, infoString, param0, param1, param2)
/** \brief Function to log the trace with four additional parameters. */
#define GT_4trace(maskType, classType, infoString, param0, param1, param2, \
                  param3)
/** \brief Function to log the trace with five additional parameters. */
#define GT_5trace(maskType, classType, infoString, param0, param1, param2, \
                  param3, param4)
/** \brief Function to log the trace with six additional parameters. */
#define GT_6trace(maskType, classType, infoString, param0, param1, param2, \
                  param3, param4, param5)
/** \brief Function to log the trace with seven additional parameters. */
#define GT_7trace(maskType, classType, infoString, param0, param1, param2, \
                  param3, param4, param5, param6)

#endif  /* if defined (UDMAUT_CFG_TRACE_ENABLE) */

#ifdef __cplusplus
}
#endif

#endif /* ifndef UDMAUTTRACE_H_ */
