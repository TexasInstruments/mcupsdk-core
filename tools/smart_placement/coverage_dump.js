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

var scriptEnv = Packages.com.ti.ccstudio.scripting.environment.ScriptingEnvironment.instance();
var server = scriptEnv.getServer("DebugServer.1");
var session = server.openSession(".*/Cortex_R5_0");

session.target.halt();

var cntStart = session.symbol.getAddress("__start___llvm_prf_cnts");
var cntStop = session.symbol.getAddress("__stop___llvm_prf_cnts");

var cntContent = session.memory.readData(0, cntStart, 8, cntStop - cntStart);

var executable = session.symbol.getSymbolFileName();
var outFile = new Packages.java.io.RandomAccessFile(executable + ".cnt" , "rw");

outFile.setLength(0);
for each (var val in cntContent) {
    outFile.writeByte(Number(val));
}
outFile.close();