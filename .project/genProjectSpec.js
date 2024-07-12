const common = require(`./common.js`);
const path = require(`path`);
const _ = require('lodash');
const fs = require('fs');

const utils = {

    capitalize: (s) => {
        if (typeof s !== 'string') return ''
        return s.charAt(0).toUpperCase() + s.slice(1)
    },
    replace: (s, a, b) => {
        return s.replace(a, b);
    },
    getDeviceFamilyProjectSpec: (cpu) => {
        let cpuFamily = 'ARM';

        if (cpu.includes('r5f') == true) {
            cpuFamily = 'ARM';
        }
        if (cpu.includes('c66') == true) {
            cpuFamily = 'C6000';
        }
        if (cpu.includes('pru') == true) {
            cpuFamily = 'PRU';
        }
        return cpuFamily;
    },
    getOutputTypeProjectSpec: (type) => {
        let outputType = 'executable';

        if(type == "executable")
            outputType = 'executable';
        if(type == "library")
            outputType = 'staticLibrary';
        return outputType;
    },
    getSysCfgDeviceProjectSpec: (device, board) => {
        return require(`./device/project_${device}`).getSysCfgDevice(board);
    },
    getSysCfgCpuProjectSpec: (cpu) => {
        return require(`./device/project_${device}`).getSysCfgCpu(cpu);
    },
    getProjectSpecDevice: (device) => {
        return require(`./device/project_${device}`).getProjectSpecDevice(device);
    },
    getProjectSpecCpu: (device, cpu) => {
        return require(`./device/project_${device}`).getProjectSpecCpu(cpu);
    },
    getDeviceIdProjectSpec: (device, cpu, board) => {
        let deviceName = '';
        let cpuName = '';

        deviceName = require(`./device/project_${device}`).getProjectSpecDevice(board);

        if (cpu.includes('r5f') == true) {
            cpuName = 'Cortex R';
        }

        if (cpu.includes('a53') == true) {
            cpuName = 'Cortex A';
        }

        if (cpu.includes('m4f') == true) {
            cpuName = 'Cortex M';
        }

        if (cpu.includes('c66') == true) {
            cpuName = 'TMS320C66XX';
        }

        if (cpu.includes('pru') == true) {
            if (board == "am64x-evm") {
                return "AM64x_GP_EVM";      // check for other devices
            } else if (board == "am243x-evm") {
                return "AM243x_GP_EVM";
            } else if (board == "am243x-lp") {
                return "AM243x_LAUNCHPAD";
            } else if (board == "am263x-cc") {
                return "AM263x_CC";
            } else if (board == "am263x-lp") {
                return "AM263x_LAUNCHPAD";
            }else if (board == "am263px-cc") {
                return "AM263px";
            } else if (board == "am263px-lp") {
                return "AM263px";
            }
        }

        return cpuName + '.' + deviceName;
    },
    getFileListProjectSpec: (files, filedirs, projectabspath) => {

        let filelist = [];

        for (prop in files) {
            if (files.hasOwnProperty(prop) && Array.isArray(files[prop])) {
                for (let file of files[prop]) {
                    let foundFile = false;
                    for (filedir of filedirs[prop]) {
                        let checkPath = path.normalize(projectabspath + '/' + filedir + '/' + file);
                        if (fs.existsSync(checkPath) == true) {
                            filelist.push(filedir + '/' + file);
                            foundFile = true;
                        }
                    }
                    if(foundFile == false) {
                        console.log(`ERROR : Couldn't find ${file} in given source directories for ${projectabspath} ...`)
                    }
                }
            }
        }
        return filelist;
    },

    getToolChainProjectSpec: (cgt) => {
        let toolchain = ''

        switch(cgt) {
            case 'ti-arm-clang':
                toolchain = 'TICLANG'
                break;
            case 'ti-arm-cgt':
                toolchain = 'TI'
                break;
            case 'gcc-armv7':
                toolchain = 'GNU'
                break;
            case 'gcc-aarch64':
                toolchain = 'GNU'
                break;
            case 'ti-pru-cgt':
                toolchain = 'TI'
                break;
            default:
                toolchain = 'TI'
        }

        return toolchain;
    },

    getProductNameProjectSpec: (device) => {

        if(common.isDevelopmentMode())
            return "MCU_PLUS_SDK_AMXXX"

        return require(`./device/project_${device}`).getProductNameProjectSpec();
    },
    /* default action for files in project spec, i.e copy or link */
    getDefaultActionProjectSpec: () => {

        if(common.isDevelopmentMode())
            return "copy"; /* use copy for development mode as well */

        return "copy";
    },
    getToolChainVersionProjectSpec: (cgt) => {
        let toolchainVersion = ''

        switch(cgt) {
            case 'ti-arm-clang':
                toolchainVersion = '3.2.2'
                break;
            case 'gcc-aarch64':
                toolchainVersion = '9.2'
                break;
            case 'gcc-armv7':
                toolchainVersion = '7.2'
                break;
            case 'ti-c6000':
                toolchainVersion = '8.3.12'
                break;
            case 'ti-pru-cgt':
                toolchainVersion = '2.3.3'
                break;
        }

        return toolchainVersion;
    },

    getSysCfgVersionProjectSpec: () => {
        return "1.20.0";
    },

    getCCSVersionProjectSpec: () => {
        return "1270";
    },

    getTiClangVersionProjectSpec: () => {
        return "3.2.2";
    },

    getGCCAarch64NoneVersionProjectSpec: () => {
        return "9.2.1";
    },

    getGCCArmv7NoneVersionProjectSpec: () => {
        return "10";
    },

    getTitleProjectSpec: (name) => {
        let title = name.replace(/_/g, ' ');

        // Title Case : Converts "hello world" to "Hello World" using regex
        return title.replace(
            /\w\S*/g,
            function (s) {
                return s.charAt(0).toUpperCase() + s.substr(1).toLowerCase();
            }
        );
    }
}

function genProjectSpecExample(device) {
    let example_file_list = require(`./device/project_${device}`).getExampleList();

    for(example of example_file_list) {
        let property = require(`../${example}`).getComponentProperty(device);

        if(property.skipProjectSpec)
            continue;

        for(buildOption of property.buildOptionCombos) {
            let commonCgtOptions = require(`./cgt/cgt_${buildOption.cgt}`).getCgtOptions(buildOption.cpu, device);
            let common_build_property = require(`./device/project_${device}`).getProperty();
            let project = [];
            let projectSpecOutPath = common.path.makeExampleOutPath(property.dirPath, buildOption);

            fs.mkdirSync(projectSpecOutPath, { recursive: true });

            buildOption.isProjectSpecBuild = true;
            build_property = require(`../${example}`).getComponentBuildProperty(buildOption);

            project = _.merge({}, project, property);
            project.relpath = common.path.relative(path.normalize(__dirname + "/.."), project.dirPath);
            project = _.merge({}, project, buildOption);
            project = _.merge({}, project, build_property);
            project = common.mergeCgtOptions(project, commonCgtOptions);
            project = common.mergeCgtOptions(project, common_build_property);
            project.dirPath = projectSpecOutPath;

            let args = {
                sdkName: "MCU_PLUS_SDK_PATH",
                sdkPath: common.path.relative(projectSpecOutPath, path.normalize(__dirname + "/..")),
                relPath: common.path.relative(project.dirPath, "."),
                project: project,
                utils: utils,
                cgtOptions: require(`./cgt/cgt_${project.cgt}`).getCgtOptions(buildOption.cpu, device),
                linuxFwName: require(`./device/project_${device}`).getLinuxFwName(buildOption.cpu),
                syscfg: {
                    device: require(`./device/project_${device}.js`).getSysCfgDevice(buildOption.board),
                    cpu: require(`./device/project_${device}.js`).getSysCfgCpu(buildOption.cpu),
                    pkg: require(`./device/project_${device}.js`).getSysCfgPkg(buildOption.board),
                    part: require(`./device/project_${device}.js`).getSysCfgPart(buildOption.board),
                },
                flashAddr: require(`./device/project_${device}.js`).getFlashAddr(),
            };

            common.convertTemplateToFile(
                    `.project/templates/projectspec_${project.type}.xdt`,
                    `${project.dirPath}/example.projectspec`,
                    args);
            if("syscfgfile" in args.project) {
                common.convertTemplateToFile(
                        `.project/templates/syscfg_c.rov.xs.xdt`,
                        `${project.dirPath}/syscfg_c.rov.xs`,
                        args);
            }
            if(args.project.skipMakefileCcsBootimageGen) {
                // skip makefile_ccs_bootimage_gen
            }
            else{
                common.convertTemplateToFile(
                    `.project/templates/makefile_ccs_bootimage_gen.xdt`,
                    `${project.dirPath}/makefile_ccs_bootimage_gen`,
                    args);
            }
        }
    }
}

function genProjectSpecsDevice(device) {
    genProjectSpecExample(device);
}

function cleanProjectSpecsDevice(device) {
    let example_file_list = require(`./device/project_${device}`).getExampleList();
    for(example of example_file_list) {
        let property = require(`../${example}`).getComponentProperty(device);

        if(property.skipProjectSpec)
            continue;

        for(buildOption of property.buildOptionCombos) {
            let project = [];
            let projectSpecOutPath = common.path.makeExampleOutPath(property.dirPath, buildOption);

            fs.mkdirSync(projectSpecOutPath, { recursive: true });

            build_property = require(`../${example}`).getComponentBuildProperty(buildOption);

            project = _.merge({}, project, property);
            project.relpath = common.path.relative(path.normalize(__dirname + "/.."), project.dirPath);
            project = _.merge({}, project, buildOption);
            project = _.merge({}, project, build_property);
            project.dirPath = projectSpecOutPath;

            common.deleteFile(`${project.dirPath}/example.projectspec`);
            common.deleteFile(`${project.dirPath}/makefile_ccs_bootimage_gen`);
            common.deleteFile(`${project.dirPath}/syscfg_c.rov.xs`);
        }
    }
}

module.exports = {
    genProjectSpecsDevice,
    cleanProjectSpecsDevice,
    utils,
}
