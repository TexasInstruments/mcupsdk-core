"""
 *  Copyright (C) 2024 Texas Instruments Incorporated
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
"""
import os
import sys

loopWaitFnx = f"""
/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
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

.global loopwait

"""

# looptime = 2**10
# loopWaitFnx = f"""
#         .arm
#         .section .text.flashloopwait
# loopwait:
#         sub     sp, sp, #4
#         mov     r1, #0
#         str     r1, [sp]
#         b       .LBB0_2
# .LBB0_1:                                @   in Loop: Header=BB0_2 Depth=1
#         ldr     r1, [sp]
#         add     r0, r0, #1
#         add     r1, r1, #1
#         str     r1, [sp]
# .LBB0_2:                                @ =>This Inner Loop Header: Depth=1
#         ldr     r1, [sp]
#         cmp     r1, #{looptime}
#         addge   sp, sp, #4
#         bxge    lr
#         b       .LBB0_1
# """

class hopperFnx:
    def __init__(self, name:str) -> None:
        self.pageSize:int = 8
        self.quantumSize:int = 4
        self.fnxSize:int = 14
        self.name = name

    #
    def generateLabelName(self, pageIndex:int, quantumIndex:int)->str:
        return "__{}_page_{}_quantum_{}".format(self.name, pageIndex, quantumIndex)

    #
    def quantum(self, label:str, goto:str) -> str:
        tpl = "{label}:\npush {lrstr}\nblx loopwait\npop {lrstr}\nb {goto_label}\n" + "nop\n"* ((2**(self.quantumSize)) - 1)
        return tpl.format(label=label, goto_label=goto, lrstr= "{lr}")

    #
    def terminateQuantum(self, label:str) -> str:
        return "nop\n" * (2**(self.quantumSize) - 1) + "{}:\nblx r14\n".format(label)

    #
    def page(self, current_page_index:int, goto_page_index:int, goto_quantum_index_offset:int) -> str:
        retVal = ""
        quantum_numbers = int((2**self.pageSize) / (2**self.quantumSize))
        for quantum_index in range(quantum_numbers):
            if(goto_quantum_index_offset + quantum_index < quantum_numbers):
                goto_quantum_index = quantum_index + goto_quantum_index_offset
                retVal += self.quantum(self.generateLabelName(current_page_index, quantum_index), self.generateLabelName(goto_page_index, goto_quantum_index))
            else:
                retVal += self.terminateQuantum(self.generateLabelName(current_page_index, quantum_index))
        return retVal

    #
    def asmFunction(self)->str:
        retVal = """
        .global {name}
        .section .text.{name}
        {name}:
        {fnx}
        """
        fnxVal = ""

        # fnxVal += self.page(0, 1, 0)
        # fnxVal += self.page(1, 2, 0)
        # fnxVal += self.page(2, 3, 0)
        # fnxVal += self.page(3, 0, 1)

        pageCount = 2**(self.fnxSize - self.pageSize)
        for pageIndex in range(pageCount):
            if(pageIndex < (pageCount - 1)):
                fnxVal += self.page(pageIndex, pageIndex + 1, 0)
            else:
                fnxVal += self.page(pageIndex, 0, 1)

        return retVal.format(name = self.name, fnx = fnxVal)

def main():
    outFilePath = os.path.realpath(os.path.join(os.path.dirname(os.path.realpath(__file__)), "hopperFnx.S"))
    hfFlash = hopperFnx("hopperFnxFlash")
    # hfOcram = hopperFnx("hopperFnxOcram")
    with open(outFilePath, "w+") as file:
        file.write(loopWaitFnx)
        file.write(hfFlash.asmFunction())
        # file.write(hfOcram.asmFunction())


if __name__ == "__main__":
    main()