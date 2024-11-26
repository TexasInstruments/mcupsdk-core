/*
 *  Copyright (c) Texas Instruments Incorporated 2024
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

/*!
 * \file  port.h
 */

#ifndef PORT_H_
#define PORT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef MCU_SDK_BUILD
int printf_(const char* format, ...);

#ifdef printf
#undef printf
#endif
#define printf printf_
#endif

/* Type definitions and helper macros for the PHY Trace interface.*/

/*! \brief All traces disabled at build-time. */
#define PHY_CFG_TRACE_LEVEL_NONE             (0U)
/*! \brief Build-time error level. */
#define PHY_CFG_TRACE_LEVEL_ERROR            (1U)
/*! \brief Build-time warning level. */
#define PHY_CFG_TRACE_LEVEL_WARN             (2U)
/*! \brief Build-time information level. */
#define PHY_CFG_TRACE_LEVEL_INFO             (3U)
/*! \brief Build-time debug level. */
#define PHY_CFG_TRACE_LEVEL_DEBUG            (4U)
/*! \brief Build-time verbose level. */
#define PHY_CFG_TRACE_LEVEL_VERBOSE          (5U)
/*! \brief Default trace level if none is set. */
#ifndef PHY_CFG_TRACE_LEVEL
#define PHY_CFG_TRACE_LEVEL                  (PHY_CFG_TRACE_LEVEL_INFO)
#endif

/*!
 * \brief Variable declaration helper macro to avoid unused variable error
 *        (-Werror=unused-variable) when variable is used in TRACE and when
 *        corresponding trace level is not enabled.
 */
#define PHYTRACE_VAR(var)                    ((var) = (var))

/*!
 * \brief Helper macro to add trace message with #PHY_TRACE_ERROR level.
 */
#if (PHY_CFG_TRACE_LEVEL >= PHY_CFG_TRACE_LEVEL_ERROR)
#define PHYTRACE_ERR(fmt, ...) printf(fmt "\r", __VA_ARGS__)  
/* TODO: Replace the below #ifdef with a method applicable for both C & C++ */
/*!
 * \brief Helper macro to add trace message with #PHY_TRACE_ERROR level if
 *        a condition is met.
 */
#ifdef __cplusplus
#define PHYTRACE_ERR_IF(cond, ...) ((cond) ? PHYTRACE_ERR(__VA_ARGS__) : 0U)
#else
#define PHYTRACE_ERR_IF(cond, ...) ((cond) ? PHYTRACE_ERR(__VA_ARGS__) : 0U)
#endif
#else
#define PHYTRACE_ERR(fmt, ...)
#define PHYTRACE_ERR_IF(cond, ...)
#endif

/*!
 * \brief Helper macro to add trace message with #PHY_TRACE_WARN level
 */
#if (PHY_CFG_TRACE_LEVEL >= PHY_CFG_TRACE_LEVEL_WARN)
#define PHYTRACE_WARN(fmt, ...) printf(fmt "\r", __VA_ARGS__)  
/*!
 * \brief Helper macro to add trace message with #PHY_TRACE_WARN level if
 *        a condition is met.
 */
#ifdef __cplusplus
#define PHYTRACE_WARN_IF(cond, ...) ((cond) ? PHYTRACE_WARN(__VA_ARGS__) : 0U)
#else
#define PHYTRACE_WARN_IF(cond, ...) ((cond) ? PHYTRACE_WARN(__VA_ARGS__) : 0U)
#endif
#else
#define PHYTRACE_WARN(fmt, ...)
#define PHYTRACE_WARN_IF(cond, ...)
#endif

/*!
 * \brief Helper macro to add trace message with #PHY_TRACE_INFO level
 *
 * Traces with this level should give only important informational messages
 * to the user, which typically they don't occur very often (i.e. "NIMU is
 * ready", "PHY n link is up").
 * This trace level may be enabled by default.
 */
#if (PHY_CFG_TRACE_LEVEL >= PHY_CFG_TRACE_LEVEL_INFO)
#define PHYTRACE_INFO(fmt, ...) printf(fmt "\r", __VA_ARGS__)  
/*!
 * \brief Helper macro to add trace message with #PHY_TRACE_INFO level if
 *        a condition is met.
 */
#ifdef __cplusplus
#define PHYTRACE_INFO_IF(cond, ...) ((cond) ? PHYTRACE_INFO(__VA_ARGS__) : 0U)
#else
#define PHYTRACE_INFO_IF(cond, ...) ((cond) ? PHYTRACE_INFO(__VA_ARGS__) : 0U)
#endif
#else
#define PHYTRACE_INFO(fmt, ...) 
#define PHYTRACE_INFO_IF(cond, ...)
#endif

/*!
 * \brief Helper macro to add trace message with #PHY_TRACE_DEBUG level
 *
 * Traces with this level will provide the user further information about
 * operations taking place (i.e. "MDIO module is open", "PHY n has started
 * auto-negotiation, etc).
 * This trace level is likely not enabled by default, so most of driver's
 * traces will follow in this category.
 */
#if (PHY_CFG_TRACE_LEVEL >= PHY_CFG_TRACE_LEVEL_DEBUG)
#define PHYTRACE_DBG(fmt, ...) printf(fmt "\r", __VA_ARGS__)  
/*!
 * \brief Helper macro to add trace message with #PHY_TRACE_DEBUG level if
 *        a condition is met.
 */
#ifdef __cplusplus
#define PHYTRACE_DBG_IF(cond, ...) ((cond) ? PHYTRACE_DBG(__VA_ARGS__) : 0U)
#else
#define PHYTRACE_DBG_IF(cond, ...) ((cond) ? PHYTRACE_DBG(__VA_ARGS__) : 0U)
#endif
#else
#define PHYTRACE_DBG(fmt, ...)
#define PHYTRACE_DBG_IF(cond, ...)
#endif

/*!
 * \brief Helper macro to add trace message with #PHY_TRACE_VERBOSE level
 *
 * Traces with this level will provide even more information and much more
 * often than the DEBUG level (i.e. "PHY n: NWAY_WAIT state", "DMA transfer
 * is complete").
 * Enabling this trace level is likely going to flood with messages, so
 * developers must ensure that their debug messages that occur often enough
 * are set with VERBOSE level.
 */
#if (PHY_CFG_TRACE_LEVEL >= PHY_CFG_TRACE_LEVEL_VERBOSE)
#define PHYTRACE_VERBOSE(fmt, ...) printf(fmt "\r", __VA_ARGS__)  
/*!
 * \brief Helper macro to add trace message with #PHY_TRACE_VERBOSE level if
 *        a condition is met.
 */
#ifdef __cplusplus
#define PHYTRACE_VERBOSE_IF(cond, ...) ((cond) ? PHYTRACE_VERBOSE(__VA_ARGS__) : 0U)
#else
#define PHYTRACE_VERBOSE_IF(cond, ...) ((cond) ? PHYTRACE_VERBOSE(__VA_ARGS__) : 0U)
#endif
#else
#define PHYTRACE_VERBOSE(fmt, ...)
#define PHYTRACE_VERBOSE_IF(cond, ...)
#endif

#ifdef __cplusplus
}
#endif

#endif /* PORT_H_ */
