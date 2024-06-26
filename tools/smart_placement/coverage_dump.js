/*
 * Copyright (c) 2023, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * [USAGE]
 *
 * This script is meant to be run using DSS tool provided with CCS. DSS is a javascript engine
 * just like node, however, is specific to TI.
 *
 * The aim of this script is to automatically read memory from the connected SOC and write that to a file.
 * Here, this script is reading memory whose start is marked "__start___llvm_prf_cnts" and end with
 * "__stop___llvm_prf_cnts" and writing to a new file which will be created along side the .out file that
 * is running on the target SOC.
 *
 * One easy way to run this script is to open scripting console in CCS while staying connected to the target SOC. In
 * that scripting console, run command loadjsfile("<absolute path to this script>")
 */

importPackage(Packages.com.ti.debug.engine.scripting);
importPackage(Packages.com.ti.debug.engine.scripting.setup);
importPackage(Packages.com.ti.ccstudio.scripting.environment);
importPackage(Packages.java.lang);
importPackage(java.io);
importPackage(java.lang);

var targetCoreProcessorId = [1971336192]
var targetCores = [];
var cpu_list = ds.getListOfCPUs();

for(var cpui = 0; cpui < cpu_list.length; cpui++)
{
    print(cpu_list[cpui]);
}
targetCores = cpu_list;

for(var cpui = 0; cpui < targetCores.length; cpui++)
{
    var sesssionName = targetCores[cpui];
    print("Opening session for CPU: " + sesssionName);
    var session = ds.openSession(sesssionName);
    if(session.target.isConnected() === true)
    {
        var gotSymbols = false;
        session.target.halt();
        try
        {
            var cntStart = session.symbol.getAddress("__start___llvm_prf_cnts");
            var cntStop = session.symbol.getAddress("__stop___llvm_prf_cnts");
            gotSymbols = true;
        }
        catch(err)
        {
            print("\tProvided binary is not instrumented. Cannot find \"__start___llvm_prf_cnts\" and \"__stop___llvm_prf_cnts\" symbols.");
        }

        var executable = session.symbol.getSymbolFileName();
        var outFileName = executable + ".cnt";

        if(gotSymbols == true && executable.length !== 0)
        {
            var cntContent = session.memory.readData(0, cntStart, 8, cntStop - cntStart);

            print("\tWriting to file: " + outFileName);
            var outFile = new Packages.java.io.RandomAccessFile( outFileName, "rw");

            outFile.setLength(0);
            for each (var val in cntContent) {
                outFile.writeByte(Number(val));
            }
            outFile.close();
        }
        session.terminate();
    }
    else
    {
        print("\tSkipping this core, as it is not connected.");
    }
}

