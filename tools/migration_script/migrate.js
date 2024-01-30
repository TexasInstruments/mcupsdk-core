const fs = require("fs");
const util = require("util");
const path = require("path");
const yargs = require("yargs");

//calculate the mcu_plus_sdk path using current directory
let mcusdkPath = __dirname.split(path.sep).slice(0,-2).join(path.sep);

//arguments
const argv = yargs
    .option('product', {
        alias: 'p',
        describe: 'Path to product.json file',
        type: 'string',
        demandOption: true,
    })
    .option('sourceDevice', {
        alias: 'srcdev',
        describe: 'Source device',
        type: 'string',
        demandOption: true,
    })
    .option('destinationDevice', {
        alias: 'destdev',
        describe: 'Destination device',
        type: 'string',
        demandOption: true,
    })
    .option('sourcePackage', {
        alias: 'srcpkg',
        describe: 'Source package',
        type: 'string',
        default: null,
    })
    .option('destinationPackage', {
        alias: 'destpkg',
        describe: 'Destination package',
        type: 'string',
        default: null,
    })
    .option('sourcePart', {
        alias: 'srcpart',
        describe: 'Source part',
        type: 'string',
        default: null,
    })
    .option('destinationPart', {
        alias: 'destpart',
        describe: 'Destination part',
        type: 'string',
        default: null,
    })
    .option('migratePath', {
        alias: 'sdkpath',
        describe: 'Path of folder to be migrated. Ideally the mcu_sdk path, but can be path to individual examples also.',
        type: 'string',
        demandOption: true,
    })
    .help('h')
    .alias('h', 'help')
    .parse(process.argv.slice(2), { allowEquals: true });

//command line arguments
let {
    product,
    sourceDevice,
    destinationDevice,
    sourcePackage,
    destinationPackage,
    sourcePart,
    destinationPart,
    migratePath
} = argv;

//convert arguments to sysconfig env supported format eg: AM263Px, ZCZ_F, AM263P4
if (sourceDevice)
    sourceDevice = sourceDevice.slice(0, -1).toUpperCase() + sourceDevice.slice(-1).toLowerCase();
if (destinationDevice)
    destinationDevice = destinationDevice.slice(0, -1).toUpperCase() + destinationDevice.slice(-1).toLowerCase();
if (sourcePackage)
    sourcePackage = sourcePackage.toUpperCase();
if (destinationPackage)
    destinationPackage = destinationPackage.toUpperCase();
if (sourcePart)
    sourcePart = sourcePart.toUpperCase();
if (destinationPart)
    destinationPart = destinationPart.toUpperCase();

//read imports.mak file to get sysconfig version
const importsFile = fs.readFileSync(path.join(mcusdkPath, 'imports.mak'), "utf-8")

let syscfgRegex = /sysconfig.+/gm
let sysconfigVersion = "sysconfig_1.19.0";
const match = importsFile.match(syscfgRegex)
if (null != match) {
    sysconfigVersion = match[0];
}

//calculating the sysconfig path based on the environment
let OS = process.platform;
let syscfgPath = ""
if (OS == "win32") {
    syscfgPath = `C:\\ti\\${sysconfigVersion}\\tests\\sysConfig`;
}
if (OS == "linux") {
    syscfgPath = process.env["HOME"] + `/ti/${sysconfigVersion}/tests/sysConfig`;
}
const sysConfig = require(syscfgPath);

// make list of all the examples paths to be excluded
let excludePaths = read_exclude_list();

//function to validate the inputs
function validate_inputs() {

    console.log("Validating inputs...");

    //Parse sysconfig deviceData
    let deviceJsonPath = path.join(syscfgPath.split(path.sep).slice(0, -2).join(path.sep), 'dist', 'deviceData', 'devices.json')
    let deviceJsonData = fs.readFileSync(deviceJsonPath)
    let devices = JSON.parse(deviceJsonData).devices;

    // Find the source and destination devices in the JSON data
    let srcDevice = devices.find(device => device.name === sourceDevice);
    let destDevice = devices.find(device => device.name === destinationDevice);

    // Validate source device
    if (!srcDevice) {
        console.log(`Source device '${sourceDevice}' not found in deviceData.\nMigration aborted !!`);
        return false;
    }

    // Validate destination device
    if (!destDevice) {
        console.log(`Destination device '${destinationDevice}' not found in deviceData.\nMigration aborted !!`);
        return false;
    }

    // Validate source,destination package
    if (sourcePackage && !srcDevice.package.find(pkg => pkg.name === sourcePackage)) {
        console.log(`Package '${sourcePackage}' not found for source device '${sourceDevice}' in deviceData.\nMigration aborted !!`);
        return false;
    }
    if (destinationPackage && !destDevice.package.find(pkg => pkg.name === destinationPackage)) {
        console.log(`Package '${destinationPackage}' not found for destination device '${destinationDevice}' in deviceData.\nMigration aborted !!`);
        return false;
    }

    // Validate source,destination part
    if (sourcePart && !srcDevice.part.find(part => part.name === sourcePart)) {
        console.log(`Part '${sourcePart}' not found for source device '${sourceDevice}' in deviceData.\nMigration aborted !!`);
        return false;
    }
    if (destinationPart && !destDevice.part.find(part => part.name === destinationPart)) {
        console.log(`Part '${destinationPart}' not found for destination device '${destinationDevice}' in deviceData.\nMigration aborted !!`);
        return false;
    }

    console.log("Inputs validation successful.");
    return true;
}

//function to list all the paths to be excluded from migration -- input from exclude_list.txt
function read_exclude_list() {
    let exclude_list_filename = `exclude_list_${sourceDevice.toLowerCase()}.js`
    const content = require(path.join(mcusdkPath, 'tools', 'migration_script', 'soc', `${exclude_list_filename}`))
    return content.getExcludeList();
}

//function to recursively travel all folders inside given path and perform migration
const get_all_files = async function (dirPath, arrayOfFiles) {

    let files = fs.readdirSync(dirPath)
    arrayOfFiles = arrayOfFiles || []

    for (let file of files) {

        let filePath = path.join(dirPath, file)

        if (fs.statSync(filePath).isDirectory()) {
            if(excludePaths.length === 0 || !excludePaths.some(excpath => filePath.includes(excpath))) {
                arrayOfFiles = await get_all_files(filePath, arrayOfFiles);
            }
        }
        else {
            if (dirPath.includes(sourceDevice.toLowerCase())) {

                if (path.extname(file) == ".syscfg") {

                    //read example.syscfg
                    let data = fs.readFileSync(filePath, "utf-8");

                    //check if the file needs to be migrated using a regex match of given sourcePackage/sourcePart
                    let pkgRegex = null, partRegex = null
                    if (null != sourcePackage) {
                        pkgRegex = new RegExp(sourcePackage, "gmi")
                    }
                    if (null != sourcePart) {
                        partRegex = new RegExp(sourcePart, "gmi")
                    }

                    if ((null != data.match(pkgRegex)) || (null != data.match(partRegex))) {
                        try {
                            let { internals } = await sysConfig.asyncCreateEnv([
                                "--product",
                                product,
                                "--device",
                                sourceDevice,
                                "--package",
                                sourcePackage,
                                "--part",
                                sourcePart,
                                "--script",
                                filePath,
                            ]);

                            internals = (
                                await internals.asyncMigrate({ device: destinationDevice, package: destinationPackage, part: destinationPart }, false, undefined, true)
                            ).internals;

                            const newScript = await internals.asyncSerialize();
                            fs.writeFileSync(filePath, newScript);

                            migrateDone.push(filePath.replace(mcusdkPath, ""));
                        }
                        catch (e) {
                            migrateFail.push(filePath.replace(mcusdkPath, ""));
                            migrateFail.push("\n" + util.format(e));
                        }
                    }
                }

                //update the respective makefile and example.projectspec also only if the example is migrated
                let makefileRegex = /makefile$/g
                let projectspecRegex = /example.projectspec$/g
                let sourcePackageRegex = new RegExp(sourcePackage, "g")
                let sourcePartRegex = new RegExp(sourcePart, "g")
                const pathsep = OS === "win32" ? '\\\\' : '/'
                let checkRegex = new RegExp(`${pathsep}.+` + sourceDevice, "gmi")

                if (migrateDone.some(ele => checkRegex.test(ele))) {
                    if ((null != filePath.match(projectspecRegex)) || (null != filePath.match(makefileRegex))) {
                        let data = fs.readFileSync(filePath, "utf-8");
                        if (null != data.match(sourcePackageRegex)) {
                            data = data.replace(sourcePackageRegex, destinationPackage);
                            data = data.replace(sourcePartRegex, destinationPart);
                            fs.writeFileSync(filePath, data);
                        }
                    }
                }

            }
        }
    }
    return arrayOfFiles
};

//create log file
const logger = fs.createWriteStream(path.join(__dirname,'log_file.txt'));
let migrateDone = [];
let migrateFail = [];

//entry point
(async () => {
    //exit if the inputs are not correct
    if (!validate_inputs()) return;

    console.log("Migration started...")
    await get_all_files(migratePath);
    console.log("Migration completed.")

    //write content to the log file
    logger.write("FILE(S) MIGRATED: \n");
    for (let i = 0; i < migrateDone.length; i++) {
        logger.write(migrateDone[i] + "\n");
    }
    logger.write("\nFILE(S) NOT MIGRATED: \n");
    for (let i = 0; i < migrateFail.length; i++) {
        logger.write(migrateFail[i] + "\n");
    }
})();