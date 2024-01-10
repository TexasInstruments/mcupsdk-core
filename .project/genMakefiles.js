const common = require(`./common.js`);
const path = require(`path`);
const fs = require(`fs`);
const _ = require('lodash');

function genMakefileDeviceTop(component_file_list, example_file_list, device, isInternal) {
    let component_make_list = [];
    let example_make_list = [];
    let example_make_projectspec_list = [];
    let system_example_make_list = [];
    let system_example_make_projectspec_list = [];

    let genFolder = ".";
    let relPath = ".";
    if(isInternal == true) {
        genFolder = "./test"
        relPath = "..";
    }

    for(component of component_file_list) {
        let component_make = [];
        let buildTarget = [];
        let buildTarget_gcc = [];
        let buildTargetClean_gcc = [];
        let buildTargetScrub_gcc = [];
        let buildTargetClean = [];
        let buildTargetScrub = [];

        property = require(`../${component}`).getComponentProperty(device);

        let tag="";
        if(property.tag)
        {
            tag = `.${property.tag}`;
        }

        component_make.name = property.name;
        component_make.tag = tag;
        component_make.relpath = common.path.relative(path.normalize(__dirname + `/../${genFolder}`), property.dirPath);
        if(property.isSkipTopLevelBuild === true)
        {
            component_make.isSkipTopLevelBuild = true;
        }
        if(property.isPrebuilt === true)
        {
            component_make.isPrebuilt = true;
        }
        for(buildOption of property.buildOptionCombos) {
            if(buildOption.cgt === "gcc-armv7" && (device === "am64x" ||  device === "am243x")){
                buildTarget_gcc +=` ${property.name}_${buildOption.cpu}.${buildOption.cgt}`;
                buildTargetClean_gcc +=` ${property.name}_${buildOption.cpu}.${buildOption.cgt}_clean`;
                buildTargetScrub_gcc +=` ${property.name}_${buildOption.cpu}.${buildOption.cgt}_scrub`;
            }
            else{
                buildTarget +=` ${property.name}_${buildOption.cpu}.${buildOption.cgt}`;
                buildTargetClean +=` ${property.name}_${buildOption.cpu}.${buildOption.cgt}_clean`;
                buildTargetScrub +=` ${property.name}_${buildOption.cpu}.${buildOption.cgt}_scrub`;
            }

        }
        if((device === "am64x" ||  device === "am243x")){
            component_make.buildTarget_gcc = buildTarget_gcc;
            component_make.buildTargetClean_gcc = buildTargetClean_gcc;
            component_make.buildTargetScrub_gcc = buildTargetScrub_gcc;
        }
        component_make.buildTarget = buildTarget;
        component_make.buildTargetClean = buildTargetClean;
        component_make.buildTargetScrub = buildTargetScrub;
        if(property.isInternal == isInternal) {
            component_make_list.push(component_make);
        }
    }

    for(example of example_file_list) {

        property = require(`../${example}`).getComponentProperty(device);

        for(buildOption of property.buildOptionCombos) {

            let makefileOutPath = common.path.makeExampleOutPath(property.dirPath, buildOption);

            let example_make = {};
            let buildTarget = {};
            let buildTargetClean = {};
            let buildTargetScrub = {};

            example_make.name = `${property.name}_${buildOption.board}_${buildOption.cpu}_${buildOption.os}_${buildOption.cgt}`;
            example_make.relpath = common.path.relative(path.normalize(__dirname + `/../${genFolder}`), makefileOutPath);

            if(property.isSkipTopLevelBuild === true)
            {
                example_make.isSkipTopLevelBuild = true;
            }

            if(property.isBootLoader === true)
            {
                example_make.isBootLoader = true;
            }

            buildTarget =` ${property.name}_${buildOption.board}_${buildOption.cpu}_${buildOption.os}_${buildOption.cgt}`;
            buildTargetClean =` ${property.name}_${buildOption.board}_${buildOption.cpu}_${buildOption.os}_${buildOption.cgt}_clean`;
            buildTargetScrub =` ${property.name}_${buildOption.board}_${buildOption.cpu}_${buildOption.os}_${buildOption.cgt}_scrub`;

            example_make.buildTarget = buildTarget;
            example_make.buildTargetClean = buildTargetClean;
            example_make.buildTargetScrub = buildTargetScrub;
            if(property.isInternal == isInternal) {
                let isPartOfSystemProject = false;
                if(buildOption.isPartOfSystemProject && buildOption.isPartOfSystemProject === true)
                {
                    isPartOfSystemProject = true;
                }
                if(isPartOfSystemProject === false)
                {
                    example_make_list.push(example_make);
                    if(property.skipProjectSpec) {
                        /* no need to output project spec make commands */
                    }
                    else {
                        example_make_projectspec_list.push(example_make);
                    }
                }
            }
        }
    }

    for(example of example_file_list) {

        example = require(`../${example}`);

        if( ! example.getSystemProjects)
            continue;

        let property = example.getComponentProperty(device);
        let systemProjects = example.getSystemProjects(device);

        for ( project of systemProjects)
        {
            let outPath = `${property.dirPath}/${project.board}/system_${project.tag}`

            let system_example_make = {};

            system_example_make.name = `${project.name}_${project.board}_system_${project.tag}`;
            system_example_make.relpath = common.path.relative(path.normalize(__dirname + `/../${genFolder}`), outPath);
            system_example_make.buildTarget = " " + system_example_make.name;
            system_example_make.buildTargetClean = " " + system_example_make.name + "_clean";
            system_example_make.buildTargetScrub = " " + system_example_make.name + "_scrub";

            if(property.isSkipTopLevelBuild === true)
            {
                system_example_make.isSkipTopLevelBuild = true;
            }

            if(property.isInternal == isInternal) {
                system_example_make_list.push(system_example_make);

                if( project.skipProjectSpec && project.skipProjectSpec === true)
                {
                    /* skip project spec */
                }
                else
                {
                    system_example_make_projectspec_list.push(system_example_make);
                }
            }


        }
    }

    let args = {
        component_list: component_make_list,
        example_list: example_make_list,
        system_example_list: system_example_make_list,
        device: device,
    };

    common.convertTemplateToFile(
        `.project/templates/makefile_device_top.xdt`,
        `${genFolder}/makefile.${device}`,
        args);

    args = {
        example_list: example_make_projectspec_list,
        system_example_list: system_example_make_projectspec_list,
        device: device,
        sdkPath: "MCU_PLUS_SDK_PATH",
        relPath: relPath
    };

    common.convertTemplateToFile(
            `.project/templates/makefile_device_projectspec_top.xdt`,
            `${genFolder}/makefile_projectspec.${device}`,
            args);
}

function genMakefileLibrary(component_file_list, device) {

    for(component of component_file_list) {
        property = require(`../${component}`).getComponentProperty(device);
        for(buildOption of property.buildOptionCombos) {
            let commonCgtOptions = require(`./cgt/cgt_${buildOption.cgt}`).getCgtOptions(buildOption.cpu, device);
            let common_build_property = require(`./device/project_${device}`).getProperty();
            build_property = require(`../${component}`).getComponentBuildProperty(buildOption);

            let project = [];
            project = _.merge({}, project, property);
            project.relpath = common.path.relative(path.normalize(__dirname + "/.."), property.dirPath);
            project = _.merge({}, project, buildOption);
            project = _.merge({}, project, build_property);
            project = common.mergeCgtOptions(project, commonCgtOptions);
            project = common.mergeCgtOptions(project, common_build_property);

            let args = {
                sdkPath: "MCU_PLUS_SDK_PATH",
                relPath: common.path.relative(project.dirPath, "."),
                project: project,
                cgtOptions: require(`./cgt/cgt_${project.cgt}`).getCgtOptions(buildOption.cpu, device),
            };

            let tag="";
            if(property.tag)
            {
                tag = `.${property.tag}`;
            }
            common.convertTemplateToFile(
                    `.project/templates/makefile_${project.type}.xdt`,
                    `${project.dirPath}/makefile${tag}.${project.device}.${project.cpu}.${project.cgt}`,
                    args);
        }
    }
}

function cleanMakefileLibrary(component_file_list, device) {

    for(component of component_file_list) {
        property = require(`../${component}`).getComponentProperty(device);
        for(buildOption of property.buildOptionCombos) {
            build_property = require(`../${component}`).getComponentBuildProperty(buildOption);

            let project = [];
            project = _.merge({}, project, property);
            project.relpath = common.path.relative(path.normalize(__dirname + "/.."), property.dirPath);

            common.deleteFile(`${project.dirPath}/makefile.${buildOption.device}.${buildOption.cpu}.${buildOption.cgt}`);
        }
    }
}

function genMakefileExample(example_file_list, device) {

    for(example of example_file_list) {
        property = require(`../${example}`).getComponentProperty(device);
        for(buildOption of property.buildOptionCombos) {
            let commonCgtOptions = require(`./cgt/cgt_${buildOption.cgt}`).getCgtOptions(buildOption.cpu, device);
            let common_build_property = require(`./device/project_${device}`).getProperty();
            let project = [];
            let makefileOutPath = common.path.makeExampleOutPath(property.dirPath, buildOption);

            fs.mkdirSync(makefileOutPath, { recursive: true });

            build_property = require(`../${example}`).getComponentBuildProperty(buildOption);

            project = _.merge({}, project, property);
            project.relpath = common.path.relative(path.normalize(__dirname + "/.."), project.dirPath);
            project = _.merge({}, project, buildOption);
            project = _.merge({}, project, build_property);
            project = common.mergeCgtOptions(project, commonCgtOptions);
            project = common.mergeCgtOptions(project, common_build_property);
            project.dirPath = makefileOutPath;

            let isInstrumentationMode = false;
            if(common.isInstrumentationMode()) {
                isInstrumentationMode = true;
            }
            let args = {
                sdkPath: "MCU_PLUS_SDK_PATH",
                relPath: common.path.relative(project.dirPath, "."),
                project: project,
                cgtOptions: require(`./cgt/cgt_${project.cgt}`).getCgtOptions(buildOption.cpu, device),
                syscfg: {
                    device: require(`./device/project_${device}.js`).getSysCfgDevice(buildOption.board),
                    cpu: require(`./device/project_${device}.js`).getSysCfgCpu(buildOption.cpu),
                    pkg: require(`./device/project_${device}.js`).getSysCfgPkg(buildOption.board),
                    part: require(`./device/project_${device}.js`).getSysCfgPart(buildOption.board),
                },
                linuxFwName: require(`./device/project_${device}`).getLinuxFwName(project.cpu),
                isInstrumentationMode: isInstrumentationMode,
                flashAddr: require(`./device/project_${device}.js`).getFlashAddr(),
            };

            if(project.makefile) {
                common.convertTemplateToFile(
                    `.project/templates/makefile_${project.makefile}.xdt`,
                    `${project.dirPath}/makefile`,
                    args);
            } else
            common.convertTemplateToFile(
                    `.project/templates/makefile_${project.type}.xdt`,
                    `${project.dirPath}/makefile`,
                    args);
        }
    }
}

function cleanMakefileExample(example_file_list, device) {

    for(example of example_file_list) {
        property = require(`../${example}`).getComponentProperty(device);
        for(buildOption of property.buildOptionCombos) {
            let project = [];
            let makefileOutPath = common.path.makeExampleOutPath(property.dirPath, buildOption);
            fs.mkdirSync(makefileOutPath, { recursive: true });
            project = _.merge({}, project, property);
            project.relpath = common.path.relative(path.normalize(__dirname + "/.."), project.dirPath);
            project.dirPath = makefileOutPath;

            common.deleteFile(`${project.dirPath}/makefile`);
        }
    }
}

function genMakefileProjectSpec(example_file_list, device) {

    for(example of example_file_list) {
        property = require(`../${example}`).getComponentProperty(device);

        if(property.skipProjectSpec)
            continue;

        for(buildOption of property.buildOptionCombos) {
            let commonCgtOptions = require(`./cgt/cgt_${buildOption.cgt}`).getCgtOptions(buildOption.cpu, device);
            let common_build_property = require(`./device/project_${device}`).getProperty();
            let project = [];
            let makefileOutPath = common.path.makeExampleOutPath(property.dirPath, buildOption);

            fs.mkdirSync(makefileOutPath, { recursive: true });

            build_property = require(`../${example}`).getComponentBuildProperty(buildOption);

            project = _.merge({}, project, property);
            project.relpath = common.path.relative(path.normalize(__dirname + "/.."), project.dirPath);
            project = _.merge({}, project, buildOption);
            project = common.mergeCgtOptions(project, common_build_property);
            project = _.merge({}, project, build_property);
            project.dirPath = makefileOutPath;

            let args = {
                sdkPath: "MCU_PLUS_SDK_PATH",
                relPath: common.path.relative(project.dirPath, "."),
                project: project
            };

            common.convertTemplateToFile(
                    `.project/templates/makefile_projectspec.xdt`,
                    `${project.dirPath}/makefile_projectspec`,
                    args);
        }
    }
}

function cleanMakefileProjectSpec(example_file_list, device) {

    for(example of example_file_list) {
        property = require(`../${example}`).getComponentProperty(device);

        if(property.skipProjectSpec)
            continue;

        for(buildOption of property.buildOptionCombos) {
            let project = [];
            let makefileOutPath = common.path.makeExampleOutPath(property.dirPath, buildOption);
            fs.mkdirSync(makefileOutPath, { recursive: true });
            project = _.merge({}, project, property);
            project.relpath = common.path.relative(path.normalize(__dirname + "/.."), project.dirPath);
            project.dirPath = makefileOutPath;

            common.deleteFile(`${project.dirPath}/makefile_projectspec`);
        }
    }
}

function genMakeImport(device) {
    let args = {
        device: device,
        utils: require(`./genProjectSpec.js`).utils,
        common: common,
    };

    common.convertTemplateToFile(
        `.project/templates/imports.mak.xdt`,
        `imports.mak`,
        args);
}

function genMakefilesDevice(device) {
    let component_file_list = require(`./device/project_${device}`).getComponentList();
    let example_file_list = require(`./device/project_${device}`).getExampleList();

    genMakefileDeviceTop(component_file_list, example_file_list, device, false);    /* External libs/examples */
    genMakefileDeviceTop(component_file_list, example_file_list, device, true);     /* Internal libs/examples */
    genMakefileLibrary(component_file_list, device);
    genMakefileExample(example_file_list, device);
    genMakefileProjectSpec(example_file_list, device);
    genMakeImport(device);
}

function cleanMakefilesDevice(device) {
    let component_file_list = require(`./device/project_${device}`).getComponentList();
    let example_file_list = require(`./device/project_${device}`).getExampleList();

    /* Remove top level files */
    common.deleteFile(`./makefile.${device}`);
    common.deleteFile(`./makefile_projectspec.${device}`);
    common.deleteFile(`./test/makefile.${device}`);
    common.deleteFile(`./test/makefile_projectspec.${device}`);

    cleanMakefileLibrary(component_file_list, device);
    cleanMakefileExample(example_file_list, device);
    cleanMakefileProjectSpec(example_file_list, device);
}

module.exports = {
    genMakefilesDevice,
    cleanMakefilesDevice,
}