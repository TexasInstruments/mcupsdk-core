const common = require(`./common.js`);

function genTirexSystemProjectContent(example, device) {
    let deviceData = require(`./device/project_${device}.js`);
    let property = require(`../${example}`).getComponentProperty(device);
    let systemProjects = require(`../${example}`).getSystemProjects(device);
    let folder_list = common.path.relative("examples", property.dirPath).split("/");

    let tirex_content = {};

    tirex_content.resourceType = `project.ccs`;
    tirex_content.resourceClass = [ `example` ];
    if(property.tirexResourceSubClass)
    {
        tirex_content.resourceSubClass = property.tirexResourceSubClass;
    }
    else
    {
        tirex_content.resourceSubClass = [ "example.general" ];
    }
    if(property.description)
    {
        tirex_content.description = property.description + " System project.";
    }
    else
    {
        tirex_content.description = common.getDefaultSystemProjectDescription(property.name);
    }

    tirex_content.name = property.name;
    tirex_content.location = common.path.relative(".metadata/.tirex", `${property.dirPath}/${project.board}/system_${project.tag}`) + "/system.projectspec";
    tirex_content.devtools = [ deviceData.getDevToolTirex(project.board) ];
    tirex_content.kernel = [];
    /* Temp fix: TIREX doesn't support multiple compiler for a project. System project doesn't need this. So keep it empty */
    //tirex_content.compiler = [];

    /* Temp fix: Push only one kernel as TIREX doesn't support a list yet. Remove when this is fixed */
    tirex_content.kernel.push(project.projects[0].os);
    for (subproject of project.projects)
    {
        /* Temp fix: see above */
        //tirex_content.kernel.push(subproject.os);
        /* Temp fix: see above */
        //switch(subproject.cgt)
        //{
        //    case "ti-arm-clang":
        //        tirex_content.compiler.push("ticlang");
        //        break;
        //    case "gcc-aarch64":
        //        tirex_content.compiler.push("gcc");
        //        break;
        //    case "gcc-armv7":
        //        tirex_content.compiler.push("gcc");
        //        break;
        //}
    }
    /* Remove duplicates */
    tirex_content.kernel = [...new Set(tirex_content.kernel)];
    /* Temp fix: see above */
    //tirex_content.compiler = [...new Set(tirex_content.compiler)];
    tirex_content.subCategories = [ ...folder_list, "system_" + `${project.tag}`];
    tirex_content.mainCategories = [[ `Examples`, `Development Tools` ]];

    return (tirex_content);
}

function genTirexExampleContentList(example_file_list, device) {

    let tirex_content_list = [];
    let deviceData = require(`./device/project_${device}.js`);
    let devtools_list = []; /* For userguide */

    for(example of example_file_list) {

        let property = require(`../${example}`).getComponentProperty(device);

        if(property.skipUpdatingTirex || property.skipProjectSpec || property.isInternal)
            continue;

        for(buildOption of property.buildOptionCombos) {

            let projectSpecOutPath = common.path.makeExampleOutPath(property.dirPath, buildOption);
            let folder_list = common.path.relative("examples", property.dirPath).split("/");
            let tirex_content = {};

            tirex_content.resourceType = `project.ccs`;
            tirex_content.resourceClass = [ `example` ];
            if(property.tirexResourceSubClass)
            {
                tirex_content.resourceSubClass = property.tirexResourceSubClass;
            }
            else
            {
                tirex_content.resourceSubClass = [ "example.general" ];
            }
            if(property.description)
            {
                tirex_content.description = property.description + " CPU is " + buildOption.cpu.toUpperCase() + " running " + buildOption.os.toUpperCase() + ".";
            }
            else
            {
                tirex_content.description = common.getDefaultProjectDescription(property.name, buildOption.cpu, buildOption.os);
            }

            tirex_content.name = property.name;
            tirex_content.location = common.path.relative(".metadata/.tirex", projectSpecOutPath) + "/example.projectspec";
            let devtools = deviceData.getDevToolTirex(buildOption.board);
            tirex_content.devtools = [ devtools ];
            devtools_list.push(devtools);
            tirex_content.kernel = [];
            tirex_content.compiler = [];

            tirex_content.kernel.push(buildOption.os);
            switch(buildOption.cgt)
            {
                case "ti-arm-clang":
                    tirex_content.compiler.push("ticlang");
                    break;
                case "gcc-aarch64":
                    tirex_content.compiler.push("gcc");
                    break;
                case "gcc-armv7":
                    tirex_content.compiler.push("gcc");
                    break;
                case "ti-c6000":
                    tirex_content.compiler.push("ccs");
                    break;
            }

            tirex_content.subCategories = [ ...folder_list, `${buildOption.cpu}` + "_" +`${buildOption.os}`];
            tirex_content.mainCategories = [[ `Examples`, `Development Tools` ]];

            if (buildOption.os == "freertos-smp")
            {

            }
            else
            {
                tirex_content_list.push(tirex_content);
            }
        }
    }

    for(example of example_file_list) {
        let tirex_content = {};

        example_js = require(`../${example}`);
        if( ! example_js.getSystemProjects)
            continue;

        let property = require(`../${example}`).getComponentProperty(device);
        if(property.skipUpdatingTirex || property.skipProjectSpec || property.isInternal)
            continue;
        let systemProjects = require(`../${example}`).getSystemProjects(device);
        for(project of systemProjects) {
            tirex_content = genTirexSystemProjectContent(example, device);
            tirex_content_list.push(tirex_content);
        }
    }

    let tirex_content = {};

    tirex_content.resourceType = `web.page`
    tirex_content.resourceClass = [ `document` ];
    tirex_content.name = "User Guide";
    tirex_content.location = `../../docs/api_guide_${device}/index.html`;
    tirex_content.devtools = [ ...new Set(devtools_list) ];
    tirex_content.mainCategories = [[ `Documents` ]];

    tirex_content_list.push(tirex_content);

    let args = {
        tirex_content_list
    };

    common.convertTemplateToFile(
        `.project/templates/tirex_content.xdt`,
        `.metadata/.tirex/${device}.content.tirex.json`,
        args);

}

function genTirexDevice(device) {
    let example_file_list = require(`./device/project_${device}`).getExampleList();
    genTirexExampleContentList(example_file_list, device);
}

module.exports = {
    genTirexDevice
}
