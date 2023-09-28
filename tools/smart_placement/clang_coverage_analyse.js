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

const fs = require('fs');
const yargs = require('yargs');
const _ = require("lodash");

const argv = yargs
    .usage('Usage: node $0 [OPTIONS]')
    .option('input', {
        alias: 'i',
        description: 'Coverage JSON file',
        type: 'string',
        default: "coverage.json",
        array: false
    })
    .option('output-json', {
        alias: 'j',
        description: 'Output analysis JSON file',
        type: 'string',
        default: "coverage-output.json",
        array: false
    })
    .option('output', {
        alias: 'o',
        description: 'Output code that would contains annotations.',
        type: 'string',
        default: "annotations.S",
        array: false
    })
    .option('top-function-count', {
        alias: 't',
        description: 'Number of top functions to use for code placement',
        type: 'number',
        default: 100,
        array: false
    })
    .help()
    .alias('help', 'h')
    .argv;


function getSortedFunctionList(covdataJSON) {

    let functionList = [];

    for( func of covdataJSON.data[0].functions )
    {
        functionList.push( { name: func.name, count: func.count} );
    }
    functionList.sort(
            function(a, b) {

                if(a.count < b.count)
                    return 1;
                if(a.count > b.count)
                    return -1;
                return 0;
            }
        );
    return functionList
}

function findHistogramBin(count)
{
    let i = 1;

    for(; i<10000000000; i = i*10)
    {
        if(count == 0)
        {
            return 0;
        }
        if(count >= i && count < i*10)
        {
            if(count < (i*10)/2)
                return i;
            else
                return i*10/2;
        }
    }
    return i;
}

function findFuncPriority(count)
{
    let i = 1;
    let priority = 1;

    if(count == 0)
    {
        return 0;
    }
    for(; i<10000000000; i = i*10)
    {
        if(count >= i && count < i*10)
        {
            if(count < (i*10)/2)
                return 10*priority-5;
            else
                return 10*priority;
        }
        priority++;
    }
    return 10*priority;
}


function getFunctionCountHistogram(functionList) {

    let functionHistogram = {};

    for( func of functionList )
    {
        let bin = findHistogramBin(func.count);
        if(functionHistogram[bin]===undefined)
        {
            functionHistogram[bin] = 0;
        }
        else
        {
            functionHistogram[bin]++;
        }
    }
    return functionHistogram;
}

function getTopFunctions(functionList, count) {

    let i = 0 ;
    let linkerCmdList = [];
    let maxFuncCount = 0

    for( func of functionList )
    {
        if(maxFuncCount < func.count)
        {
            maxFuncCount = func.count;
        }
    }

    for( func of functionList )
    {
        /* check if function name is of the form file:function, if yes, only store the function */
        let wordList = func.name.split(":");

        functionName = wordList[0];
        if(wordList.length > 1)
            functionName = wordList[1];

        linkerCmdList.push( { name: functionName, priority: maxFuncCount - func.count } );
        i++;
        if(i>=count)
            break;
    }

    return linkerCmdList;
}


let asmAnnotationsFileTemplate =
`
<% for( func of topFunctionsList) { %>
.global <%= func.name %>
.sym_meta_info <%= func.name %> , "of_placement", "local", <%= func.priority %>
<% } %>
`;

function outputASMAnnotationFile(asmFileName, topFunctionsList)
{
    let asmFileContent = _.template( asmAnnotationsFileTemplate )( { topFunctionsList: topFunctionsList})

    fs.writeFileSync(asmFileName, asmFileContent,
        function (err) {
            if (err) throw err;
        }
    );
}

function outputJSONFile(jsonFileName, sortedFunctions)
{
    let functionCountHistogram = getFunctionCountHistogram(sortedFunctions);
    let output = {
        functionCountHistogram: functionCountHistogram,
        sortedFunctions: sortedFunctions,
    };

    fs.writeFileSync(jsonFileName, JSON.stringify(output, null, "    "),
        function (err) {
            if (err) throw err;
        }
    );
}

let covdataJSON = JSON.parse( fs.readFileSync(argv.input) );
let sortedFunctions = getSortedFunctionList(covdataJSON);
let topFunctionsList = getTopFunctions(sortedFunctions, argv["top-function-count"]);

outputJSONFile(argv["output-json"], sortedFunctions);
outputASMAnnotationFile(argv["output"], topFunctionsList);

