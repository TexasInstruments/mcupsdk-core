import sys
import os
import socket
from datetime import datetime

if (len(sys.argv) < 4):
    print("USAGE: <path/to/python3>/python bin2c.py <binary file name> <c file name> <array name> <license_type(optional)>")
    exit(0)
if (os.path.isfile(sys.argv[1]) is False):
    print("ERROR: Input file %s not found !!!" % (sys.argv[1]))
    exit(0)

ti_license = '''
/*
 *  Copyright (C) 2023 Texas Instruments Incorporated
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
'''

tspa_license = '''
/*
* TI TSPA License
* TECHNOLOGY AND SOFTWARE PUBLICLY AVAILABLE
* SOFTWARE LICENSE
*
* Copyright (c) 2023, Texas Instruments Incorporated.
*
* All rights reserved not granted herein.
*
* Limited License.
*
* Texas Instruments Incorporated grants a world-wide, royalty-free, non-exclusive
* license under copyrights and patents it now or hereafter owns or controls to
* make, have made, use, import, offer to sell and sell ("Utilize") this software,
* but solely to the extent that any such patent is necessary to Utilize the
* software alone. The patent license shall not apply to any combinations which
* include this software.  No hardware per se is licensed hereunder.
*
* Redistribution and use in binary form, without modification, are permitted
* provided that the following conditions are met:
*
* * Redistributions must preserve existing copyright notices and reproduce this
* license (including the above copyright notice and the disclaimer below) in the
* documentation and/or other materials provided with the distribution.
*
* * Neither the name of Texas Instruments Incorporated nor the names of its
* suppliers may be used to endorse or promote products derived from this software
* without specific prior written permission.
*
* * No reverse engineering, decompilation, or disassembly of this software is
* permitted.
*
* * Nothing shall obligate TI to provide you with source code for the software
* licensed and provided to you in object code.
*
* DISCLAIMER.
*
* THIS SOFTWARE IS PROVIDED BY TI AND TIS LICENSORS "AS IS" AND ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
* EVENT SHALL TI AND TIS LICENSORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
* PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
* LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
* NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
* EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/
'''
licenses = {
    "ti_tspa": tspa_license,
    "ti_lic": ti_license,
}

header = '''
{license}

/* Input file name = {input_file_name} */

/* Image generation timestamp : {date_time} */

#ifndef {file_name}_H_
#define {file_name}_H_

#ifdef __cplusplus
extern "C"
{{
#endif

#define {array_name}_SIZE_IN_BYTES ({size}U)

#define {array_name} {{ \\
'''

footer = '''
}} /* {infilesize} bytes */

#ifdef __cplusplus
}}
#endif

#endif /* {header_name}_H_ */
'''

infile = open(sys.argv[1], "rb")
infilesize = os.path.getsize(sys.argv[1])
outfile = open(sys.argv[2], "w")
header_name = os.path.splitext(os.path.basename(sys.argv[2]))[0].upper()

lic_type = "ti_lic"

if len(sys.argv) > 4:
    lt = sys.argv[4]
    if lt == "ti_tspa":
        lic_type = lt

# write header
outfile.write(header.format(license=licenses[lic_type],
                            input_file_name=sys.argv[1].rsplit('/')[-1],
                            date_time=datetime.now(),
                            file_name=header_name,
                            array_name=sys.argv[3].upper(),
                            size=infilesize))
outfile.write("    ")

count = 0
while True:
    # read upto 4 bytes
    byte = infile.read(1)
    if byte:
        count = count + 1
        # convert 32b word to hex string, then convert to integer and then convert to little endian
        # and then write to file as a C hex string
        outfile.write("0x%sU, " % (byte.hex()))
        # break to new line after 16 bytes
        if (count == 16):
            outfile.write(" \\\n    ")
            count = 0
    else:
        break

outfile.write(" \\")
outfile.write(footer.format(infilesize=infilesize, header_name=header_name))

outfile.close()
infile.close()
