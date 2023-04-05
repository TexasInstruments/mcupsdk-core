/*!
 *  \file CMN_mem.h
 *
 *  \brief
 *  Dynamic memory allocation trace.
 *
 *  \author
 *  KUNBUS GmbH
 *
 *  \date
 *  2022-10-27
 *
 *  \copyright
 *  Copyright (c) 2021, KUNBUS GmbH<br /><br />
 *  All rights reserved.<br />
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:<br />
 *  <ol>
 *  <li>Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.</li>
 *  <li>Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.</li>
 *  <li>Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.</li>
 *  </ol>
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#if !(defined PROTECT_CMN_MEM_H)

#include <stdint.h>

#define PROTECT_CMN_MEM_H

#define CMN_MEM_DESCRIPTOR_POSITION       8             // Relative position of memory allocation descriptor against of object pointer in bytes

#define CMN_MEM_DESCRIPTOR_SIZE_MASK      0xFFFFFFFE    // Descriptor mask related to allocated space
#define CMN_MEM_DESCRIPTOR_USED_FLAG_MASK 0x00000001    // Descriptor mask of allocated/deallocated flag

#if (defined __cplusplus)
extern "C" {
#endif

extern void CMN_MEM_traceInit      (void);
extern void CMN_MEM_traceHeapCheck (uint32_t heapStart_p, uint32_t size_p);

#if (defined __cplusplus)
}
#endif

#endif /* PROTECT_CMN_MEM_H */
