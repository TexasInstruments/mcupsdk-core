const common = require(`./common.js`);
const path = require(`path`);
const fs = require(`fs`);
const _ = require('lodash');

function genExampleFilesDevice(device) {

    let example_file_list = require(`./device/project_${device}`).getExampleList();

    for(example of example_file_list) {
        property = require(`../${example}`).getComponentProperty(device);
        for(buildOption of property.buildOptionCombos) {
            let commonCgtOptions = require(`./cgt/cgt_${buildOption.cgt}`).getCgtOptions(buildOption.cpu);
            let common_build_property = require(`./device/project_${device}`).getProperty();
            let project = [];
            let outPath = common.path.makeExampleOutPath(property.dirPath, buildOption);

            fs.mkdirSync(outPath, { recursive: true });

            build_property = require(`../${example}`).getComponentBuildProperty(buildOption);

            project = _.merge({}, project, property);
            project.relpath = common.path.relative(path.normalize(__dirname + "/.."), project.dirPath);
            project = _.merge({}, project, buildOption);
            project = common.mergeCgtOptions(project, common_build_property);
            project = _.merge({}, project, build_property);
            project.dirPath = outPath;

            if(!project.templates)
                continue;

            let isInstrumentation = common.isInstrumentationMode();
            for(template of project.templates)
            {
                let args = {
                    sdkPath: "MCU_PLUS_SDK_PATH",
                    relPath: common.path.relative(project.dirPath, "."),
                    project: project,
                    options: template.options,
                    isInstrumentation: isInstrumentation,
                };

                common.convertTemplateToFile(
                        template.input,
                        `${project.dirPath}/${template.output}`,
                        args);
            }
        }
    }
}

function cleanExampleFilesDevice(device) {

    let example_file_list = require(`./device/project_${device}`).getExampleList();

    for(example of example_file_list) {
        property = require(`../${example}`).getComponentProperty(device);
        for(buildOption of property.buildOptionCombos) {
            let project = [];
            let outPath = common.path.makeExampleOutPath(property.dirPath, buildOption);

            fs.mkdirSync(outPath, { recursive: true });
            build_property = require(`../${example}`).getComponentBuildProperty(buildOption);
            project = _.merge({}, project, property);
            project.relpath = common.path.relative(path.normalize(__dirname + "/.."), project.dirPath);
            project = _.merge({}, project, buildOption);
            project = _.merge({}, project, build_property);
            project.dirPath = outPath;

            if(!project.templates)
                continue;

            for(template of project.templates)
            {
                common.deleteFile(`${project.dirPath}/${template.output}`);
            }
        }
    }
}

module.exports = {
    genExampleFilesDevice,
    cleanExampleFilesDevice,
}
