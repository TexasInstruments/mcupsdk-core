/*
 * Copyright (c) 2024, Texas Instruments Incorporated
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


/*USAGE: node linearize_cg.js {TI_ARM_CLANG PATH} {LINKINFO.xml FILE} {CG.xml FILE} {PATH TO OUTPUT FILE}*/

const {XMLParser} = require(`${process.argv[2]}\\opti-share\\node_modules\\fast-xml-parser`);
const fs=require('fs');

let input1=process.argv[3];
let input2=process.argv[4]
let outputfile=process.argv[5];

//reading the linkinfo.xml
const xmlDataStr=fs.readFileSync(input1,{ encoding: 'utf8', flag: 'r' });

const options = {
    attributeNamePrefix   : '',
    ignoreAttributes      : false,
    parseAttributeValue   : false,
    allowBooleanAttributes: true,
    parseTagValue         : true,
};

const parser = new XMLParser(options);
const output_link_info = parser.parse(fs.readFileSync(input1,{ encoding: 'utf8', flag: 'r' }));

//finding the object_component name having the of_placement tag
const object_component_list=output_link_info.link_info.object_component_list.object_component;
const flcobj=[];
object_component_list.forEach((object,index) =>{

    if('of_placement' in object){

        const flc=object.name.search('text');
        flcobj.push(object.name.slice(flc+5));
    }
})

const flcarrays=[];
//reading the cg.xml file
const output_cg = parser.parse(fs.readFileSync(input2,{ encoding: 'utf8', flag: 'r' }));
for(var i=0;i<flcobj.length;i++){
    var callname=new Set();
    const func= output_cg.ofd.call_graph.function;
    callname.add(func.find(record => record.name === flcobj[i]));
    for (call of callname){
        if('callee' in call){
            if(typeof(call.callee)==='object'){
                call.callee.forEach((symbol,index) => {

                    callname.add(func.find(record => record.name === symbol));
                });
            }
            else{

                callname.add(func.find(record => record.name === call.callee));
            }
        }

    }
    flcarrays.push(callname); //pushing call-graph of each FLC function
}

//clearing output file
fs.writeFileSync(outputfile,"",);
//Writing to output .S file
for(var i=0;i<flcarrays.length;i++){
    flcarrays[i].forEach((call,index) =>{
        if(call.name!=""){
            const st=`.global ${call.name}
.sym_meta_info ${call.name}, "of_placement", "fast_local_copy", 1\n\n`;
            fs.appendFile(outputfile, st, (err) => {

                // In case of a error throw err.
                if (err) throw err;
            })
     }
})
}