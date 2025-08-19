"""
 *  Copyright (C) 2025 Texas Instruments Incorporated
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

COPYRIGHT_HEADER = """
/*
 *  Copyright (C) 2025 Texas Instruments Incorporated
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

import os

def bytes_to_c_array(data, var_name="bin_data", line_width=12):
    """Converts a bytes object to a C array string."""
    if not data:
        return f"const unsigned char {var_name}[0] = {{}};\nconst unsigned int {var_name}_len = 0;\n"

    hex_bytes = [f"0x{byte:02X}" for byte in data]
    c_decl = f"uint8_t {var_name}[{len(data)}]"
    h_decl = f"extern {c_decl};"
    c_array = f"{c_decl} = {{\n    "

    lines = []
    for i in range(0, len(hex_bytes), line_width):
        line = ", ".join(hex_bytes[i:i + line_width])
        lines.append("    " + line) # Add indentation

    c_array += ",\n".join(lines)
    c_array += "\n};\n"
    c_array += f"const unsigned int {var_name}_len = {len(data)};\n"
    return (c_array, h_decl, len(data)) 

def main():

    this_file_dir = os.path.dirname(__file__)
    output_c_file = os.path.realpath(os.path.join(this_file_dir, "new_application_images.c"))
    output_h_file = os.path.realpath(os.path.join(this_file_dir, "new_application_images.h"))
    basepath = os.path.realpath(os.path.join(this_file_dir, "../../../../../"))
    mcelf_file = os.path.realpath(os.path.join(basepath, "examples/hello_world_xip_fss1/am261x-som/r5fss0-0_nortos/ti-arm-clang/hello_world_xip_fss1.release.mcelf"))
    mcelfxip_file = os.path.realpath(os.path.join(basepath, "examples/hello_world_xip_fss1/am261x-som/r5fss0-0_nortos/ti-arm-clang/hello_world_xip_fss1.release.mcelf_xip"))
    mcelf_varname = "mcelf_file"
    mcelfxip_varname = "mcelfxip_file"

    # Read binary data
    try:
        with open(mcelf_file, 'rb') as f_in:
            mcelf_binary_data = f_in.read()
    except FileNotFoundError:
        print(f"Error: Input file not found: {mcelf_file}")
        return
    except Exception as e:
        print(f"Error reading input file: {e}")
        return
    try:
        with open(mcelfxip_file, 'rb') as f_in:
            mcelfxip_binary_data = f_in.read()
    except FileNotFoundError:
        print(f"Error: Input file not found: {mcelfxip_file}")
        return
    except Exception as e:
        print(f"Error reading input file: {e}")
        return

    # Generate C array string
    mcelf_code, mcelf_h_code, mcelf_size = bytes_to_c_array(mcelf_binary_data, mcelf_varname, 16)
    mcelfxip_code, mcelfxip_h_code, mcelfxip_size = bytes_to_c_array(mcelfxip_binary_data, mcelfxip_varname, 16)

    # Generate header guards
    header_guard = os.path.basename(output_h_file).upper().replace('.', '_') + '_'
    header_guard = ''.join(c if c.isalnum() else '_' for c in header_guard) # Ensure valid identifier
    if not header_guard[0].isalpha():
        header_guard = '_' + header_guard # Ensure it starts with a letter/underscore

    # Write to output file
    try:
        with open(output_h_file, 'w') as f_out:
            f_out.write(COPYRIGHT_HEADER)
            f_out.write(f"#ifndef {header_guard}\n")
            f_out.write(f"#define {header_guard}\n\n")
            f_out.write(f"/* Generated from {os.path.basename(mcelf_file)} by bin_to_c_array.py */\n\n")
            f_out.write(f"#define {mcelf_varname.upper()}_LEN ({mcelf_size}U)\n")
            f_out.write(f"#define {mcelfxip_varname.upper()}_LEN ({mcelfxip_size}U)\n\n\n")
            f_out.write(mcelf_h_code)
            f_out.write(f"\n")
            f_out.write(mcelfxip_h_code)
            f_out.write(f"\n#endif // {header_guard}\n")
            
        with open(output_c_file, 'w') as f_out:
            f_out.write(COPYRIGHT_HEADER)
            f_out.write("#include <stdint.h>\n")
            f_out.write("#include \"./new_application_images.h\"\n")
            f_out.write(mcelf_code)
            f_out.write(mcelfxip_code)
            f_out.write(f"\n")

        print("done")
    except Exception as e:
        print(f"Error writing output file: {e}") 

if __name__ == "__main__":
    main()