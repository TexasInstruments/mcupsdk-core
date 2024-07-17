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

import subprocess
from intelhex import IntelHex
import os
import shutil

outputFolderName = "8051FwOut"

if os.path.isdir(outputFolderName) is False:
	os.mkdir(outputFolderName)

COMPILE_COMMAND = f"sdcc -Iincludes -std-sdcc99 --debug --opt-code-size --stack-size 36 --max-allocs-per-node 100000000 -lmcs51 -llibsdcc fota_function_ospi.c -o{outputFolderName}/"

# compile using 8051 flags
subprocess.run(COMPILE_COMMAND.split(" "))

with open(f"{outputFolderName}/fota_function_ospi.hex", "w") as f:
	subprocess.call(["packihx", f"{outputFolderName}/fota_function_ospi.ihx"], stdout=f)

ih = IntelHex(outputFolderName + "/fota_function_ospi.hex")

fw_sz = len(ih)
fw_list = list(ih.tobinarray(start=0, size=fw_sz))

print(f"FW Size: {fw_sz} B or {fw_sz/(2*1024)} KB")

LICENSE_TEXT="""
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

"""

with open(f'{outputFolderName}/fota_fw_arr.h', 'w+') as the_file:
	the_file.write(LICENSE_TEXT)
	the_file.write("#ifndef __FOTA_FW_ARR_H__\n")
	the_file.write("#define __FOTA_FW_ARR_H__\n")
	the_file.write('\n')
	the_file.write("#define FOTA_FW_SIZE (%dU)\n\n"%fw_sz)
	the_file.write("extern const uint8_t FOTA_FW_ARR[FOTA_FW_SIZE]; ")
	the_file.write('\n')
	the_file.write("#endif //__FOTA_FW_ARR_H__")
	the_file.write('\n')

with open(f'{outputFolderName}/fota_fw_arr.c', 'w+') as the_file:
	the_file.write(LICENSE_TEXT)
	the_file.write("#include <stdint.h>\n")
	the_file.write("#include \"fota_fw_arr.h\"")
	the_file.write('\n\n')
	the_file.write("const uint8_t FOTA_FW_ARR[FOTA_FW_SIZE] __attribute__((aligned(32))) = ")
	the_file.write("\n{")
	line = 0
	for byte in fw_list:
		if ((line % 12) == 0):
			the_file.write('\n\t')
		the_file.write("0x%02x, "%(byte))
		line = line + 1
	the_file.write("\n};")

def replaceFile(src, dst):
    if os.path.exists(dst):
        os.remove(dst)
    shutil.copy2(src, dst)

replaceFile(f'{outputFolderName}/fota_fw_arr.c', "../../source/drivers/flsopskd/v0/fota_fw_arr.c")
replaceFile(f'{outputFolderName}/fota_fw_arr.h', "../../source/drivers/flsopskd/v0/fota_fw_arr.h")