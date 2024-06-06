import sys
import os
from datetime import datetime

# TI License Strings

ti_license = '''
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

ti_commercial_license = '''
/*
 * Copyright (C) 2023 Texas Instruments Incorporated - http://www.ti.com/
 * ALL RIGHTS RESERVED
 * Licensed under the TI Software License Agreement found in [as_installed]/license.txt
 */
'''

licenses = {
    "ti_tspa": tspa_license,
    "ti_lic": ti_license,
    "ti_com": ti_commercial_license
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


def binary_to_header(input_filepath: str, output_filename: str, array_name: str, license_type: str = "ti_lic"):
    input_filesize = os.path.getsize(input_filepath)
    header_name = os.path.splitext(
        os.path.basename(output_filename))[0].upper()

    with open(input_filepath, "rb") as infile, open(output_filename, "w") as outfile:
        # write header
        outfile.write(header.format(license=licenses[license_type],
                                    input_file_name=input_filepath.rsplit(
                                        '/')[-1],
                                    date_time=datetime.now(),
                                    file_name=header_name,
                                    array_name=array_name.upper(),
                                    size=input_filesize))
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
        outfile.write(footer.format(
            infilesize=input_filesize, header_name=header_name))


if __name__ == "__main__":
    if (len(sys.argv) < 4):
        print("USAGE: <path/to/python3>/python bin2c.py <binary file name> <c file name> <array name> <license_type(optional)>")
        exit(-1)

    if (os.path.isfile(sys.argv[1]) is False):
        print("ERROR: Input file %s not found !!!" % (sys.argv[1]))
        exit(-1)

    # Check if license type is passed as argument
    if len(sys.argv) == 4:
        binary_to_header(sys.argv[1], sys.argv[2], sys.argv[3])
    elif len(sys.argv) > 4 and not (sys.argv[4] in licenses):
        print("ERROR: License type %s not found !!!" % (sys.argv[4]))
        exit(-1)
    else:
        binary_to_header(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4])
