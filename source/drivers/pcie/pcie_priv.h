/*
 *  Copyright (C) 2022 Texas Instruments Incorporated
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

#ifndef PCIE_PRIV_H_
#define PCIE_PRIV_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Inbound limit */
#define PCIE_INBOUND_MASK 0x00000FFFU

/* Set a bitfield */
#define PCIE_SETBITS(newval,field,val)                            \
    {                                                             \
        /* Eval "val" only once */                                \
        uint32_t working_val = (uint32_t)(val);                   \
        uint32_t working_mask = (field##_MASK >> field##_SHIFT);  \
        working_val &= working_mask;                              \
        working_val <<= field##_SHIFT;                            \
        (newval) &= ~((uint32_t)(field##_MASK));                  \
        (newval) |= working_val;                                  \
    }

/* Extracts a bitfield */
#define PCIE_GETBITS(val,field,final_result)                      \
    ((final_result) = ((val) & (field##_MASK)) >> (field##_SHIFT))

#ifdef __cplusplus
}
#endif

#endif  /* #ifndef PCIE_PRIV_H_ */