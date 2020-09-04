const fs = require('fs');
const path = require(`path`);
const _ = require('lodash');
const template = require("./templateCompiler");

function concatArrayPropertiesInObject(obj1, obj2, arr_prop, swap) {
    if(obj2.hasOwnProperty(arr_prop)==false)
        return;

    if(obj1.hasOwnProperty(arr_prop)==false)
        obj1[arr_prop] = {};

    for (let prop in obj2[arr_prop]) {
        if (obj2[arr_prop].hasOwnProperty(prop) && Array.isArray(obj2[arr_prop][prop])) {
            if( obj1[arr_prop].hasOwnProperty(prop) == false )
                obj1[arr_prop][prop] = [];

            if( Array.isArray(obj1[arr_prop][prop]))
            {
                if(swap)
                {
                    /* here the CGT options appear after project specific options, for some options
                     * esp libaries we want cgt libs to appear at the end.
                     */
                    obj1[arr_prop][prop] = obj1[arr_prop][prop].concat(obj2[arr_prop][prop]);
                }
                else
                {
                    /* here the CGT options appear before project specific options, most we want
                     * this so that project options can override CGT options
                     */
                    obj1[arr_prop][prop] = obj2[arr_prop][prop].concat(obj1[arr_prop][prop]);
                }
            }
        }
    }
}

function mergeCgtOptions(project, commonCgtOptions) {
    project = _.cloneDeep(project);

    concatArrayPropertiesInObject(project, commonCgtOptions, "cflags", false);
    concatArrayPropertiesInObject(project, commonCgtOptions, "arflags", false);
    concatArrayPropertiesInObject(project, commonCgtOptions, "includes", false);
    concatArrayPropertiesInObject(project, commonCgtOptions, "defines", false);
    concatArrayPropertiesInObject(project, commonCgtOptions, "files", false);
    concatArrayPropertiesInObject(project, commonCgtOptions, "filedirs", false);
    concatArrayPropertiesInObject(project, commonCgtOptions, "asmfiles", false);
    concatArrayPropertiesInObject(project, commonCgtOptions, "lflags", false);
    concatArrayPropertiesInObject(project, commonCgtOptions, "libs", true);
    concatArrayPropertiesInObject(project, commonCgtOptions, "libdirs", true);
    concatArrayPropertiesInObject(project, commonCgtOptions, "lnkfiles", false);

    return project;
}

function relative(pathStr1, pathStr2) {
    let relpath = path.relative(pathStr1, pathStr2)

    relpath = path.normalize(relpath);

    return relpath.replace(/\\/g, "/");
}

function genBuildfiles(device) {

    /* first call genExampleFiles, genComponentFiles since these create files that
     * are then used by genProjectSpec and genMakefiles
     */
    require(`./genExampleFiles`).genExampleFilesDevice(device);
    require(`./genComponentFiles`).genComponentFilesDevice(device);
    require(`./genMakefiles`).genMakefilesDevice(device);
    require(`./genProjectSpec`).genProjectSpecsDevice(device);
    require(`./genTirex`).genTirexDevice(device);
    require(`./genSystemProject`).genSystemProjectDevice(device);
    require(`./genCcsPackageFiles`).genCcsPackageFilesDevice(device);
}

function cleanBuildfiles(device) {
    require(`./genExampleFiles`).cleanExampleFilesDevice(device);
    require(`./genComponentFiles`).cleanComponentFilesDevice(device);
    require(`./genMakefiles`).cleanMakefilesDevice(device);
    require(`./genProjectSpec`).cleanProjectSpecsDevice(device);
    require(`./genSystemProject`).cleanSystemProjectDevice(device);
}

function convertTemplateToFile(templateFilename, outputFilename, args) {

    let inData = fs.readFileSync(templateFilename, 'utf8',
                        function (err, data) {
                            if (err) throw err;
                        }
                    );

    let outData = template.executeTemplate(inData, args);

    fs.writeFileSync(outputFilename, outData,
        function (err) {
            if (err) throw err;
        }
    );
}

function makeExampleOutPath(projectPath, buildOption)
{
    return `${projectPath}/${buildOption.board}/${buildOption.cpu}_${buildOption.os}/${buildOption.cgt}`
}

const fs_promise = require('fs').promises;

function deleteFile(filepath)
{
    (async () => {
      try {
        await fs_promise.unlink(filepath);
      } catch (e) {
        /* file doens't exist. Skip error!! */
      }
    })();
}

function getDefaultProjectDescription(name, cpu, os) {
    let title = name.replace(/_/g, ' ');

    // Title Case : Converts "hello world" to "Hello World" using regex
    title = title.replace(
        /\w\S*/g,
        function (s) {
            return s.charAt(0).toUpperCase() + s.substr(1).toLowerCase();
        }
    );

    title = "A " + title + " Example. CPU is " + cpu.toUpperCase() + " running " + os.toUpperCase() + ".";
    return title;
}

function getDefaultSystemProjectDescription(name) {
    let title = name.replace(/_/g, ' ');

    // Title Case : Converts "hello world" to "Hello World" using regex
    title = title.replace(
        /\w\S*/g,
        function (s) {
            return s.charAt(0).toUpperCase() + s.substr(1).toLowerCase();
        }
    );

    title = "A " + title + " System Example.";
    return title;
}

/* this can be "development" or "production" */
let genBuildFilesMode = "development";

function isDevelopmentMode()
{
    if(genBuildFilesMode=="development")
        return true;

    return false;
}

function setGenBuildFilesMode(mode)
{
    if(mode=="production" || mode == "development")
    {
        genBuildFilesMode = mode;
    }
}

/* this can be "enable" or "disable" */
let genInstrumentationMode = "disable";

function isInstrumentationMode()
{
    if(genInstrumentationMode=="enable")
        return true;

    return false;
}

function setInstrumentationMode(mode)
{
    genInstrumentationMode = mode;
}

module.exports = {
    genBuildfiles,
    isDevelopmentMode,
    setGenBuildFilesMode,
    isInstrumentationMode,
    setInstrumentationMode,
    cleanBuildfiles,
    mergeCgtOptions,
    convertTemplateToFile,
    path: {
        relative,
        makeExampleOutPath,
    },
    deleteFile,
    getDefaultProjectDescription,
    getDefaultSystemProjectDescription,
};
