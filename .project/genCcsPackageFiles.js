const common = require(`./common.js`);

function genCcsPackageFilesDevice(device) {

    let args = {
        device: device,
        utils: require(`./genProjectSpec.js`).utils,
        common: common,
    };

    common.convertTemplateToFile(
        `.project/templates/package.ccs.json.xdt`,
        `.metadata/.tirex/package.ccs.json`,
        args);
    common.convertTemplateToFile(
        `.project/templates/package.tirex.json.xdt`,
        `.metadata/.tirex/package.tirex.json`,
        args);
    common.convertTemplateToFile(
        `.project/templates/product.json.xdt`,
        `.metadata/product.json`,
        args);

}

module.exports = {
    genCcsPackageFilesDevice
}
