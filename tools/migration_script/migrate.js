const fs = require("fs");
const util = require("util");
const path = require("path");

//command line arguments
const product = process.argv[2];
const deviceFrom = process.argv[3];
const deviceTo = process.argv[4];
const packageFrom = process.argv[5];
const packageTo = process.argv[6];
const migratePath = process.argv[7];


//calculate the mcu_plus_sdk path using given product.json path
let mcusdkPath = product.split('/').slice(0,-2).join('/')

//call genProjectSpec.js script to get sysconfig version
const genProjectSpec = require(`${mcusdkPath}/.project/genProjectSpec.js`)


//calculating the sysconfig path based on the environment
let OS = process.platform;
if (OS == "win32") {
    syscfgPath = process.env["USERPROFILE"] + `/ti/sysconfig_${genProjectSpec.utils.getSysCfgVersionProjectSpec()}/tests/sysConfig`;
}
if (OS == "linux") {
    syscfgPath = process.env["HOME"] + `/ti/sysconfig_${genProjectSpec.utils.getSysCfgVersionProjectSpec()}/tests/sysConfig`;
}
const sysConfig = require(syscfgPath);


//create log file
const logger = fs.createWriteStream(`${__dirname}/log_file.txt`);
let migrateDone = [];
let migrateFail = [];


//recursive traversal of all folders inside given path
const getAllFiles = async function (dirPath, arrayOfFiles) {

    let files = fs.readdirSync(dirPath)
    arrayOfFiles = arrayOfFiles || []

    for (let file of files) {

        let filePath = path.join(dirPath, file)

        if (fs.statSync(filePath).isDirectory()) {
            arrayOfFiles = await getAllFiles(filePath, arrayOfFiles);
        }
        else {
            if (dirPath.includes(deviceFrom.toLowerCase())) {

                if (path.extname(file) == ".syscfg") {

                    let data = fs.readFileSync(filePath, "utf-8");

                    //check if the file needs to be migrated
                    if (data.includes(packageFrom)) {
                        try {
                            let { internals } = await sysConfig.asyncCreateEnv([
                                "--product",
                                product,
                                "--device",
                                deviceFrom,
                                "--package",
                                packageFrom,
                                "--script",
                                filePath,
                            ]);

                            internals = (
                                await internals.asyncMigrate({device: deviceTo, package: packageTo}, false, undefined, true)
                            ).internals;

                            const newScript = await internals.asyncSerialize();
                            fs.writeFileSync(filePath, newScript);

                            migrateDone.push(filePath.replace(process.cwd(), ""));
                        }
                        catch (e) {
                            migrateFail.push(filePath.replace(process.cwd(), ""));
                            migrateFail.push("\n" + util.format(e));
                        }
                    }
                }

                //update the respective makefile and example.projectspec also with the new package
                let makefileRegex = /makefile$/g
                let projectspecRegex = /example.projectspec$/g
                let packageFromRegex = new RegExp(packageFrom, "g")

                if ((null != filePath.match(projectspecRegex)) || (null != filePath.match(makefileRegex))) {
                    let data = fs.readFileSync(filePath, "utf-8");
                    if (null != data.match(packageFromRegex)) {
                        data = data.replace(packageFromRegex, packageTo);
                        fs.writeFileSync(filePath, data);
                    }
                }
            }
        }
    }
    return arrayOfFiles
};

//entry point
(async () => {
    await getAllFiles(migratePath);

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

