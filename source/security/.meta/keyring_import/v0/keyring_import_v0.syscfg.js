let common = system.getScript("/common");

function getInstanceConfig(moduleInstance) {

    return {
        ...moduleInstance,
    };
};

let keyring_import_module = {
    displayName: "Keyring Import",

    templates: {
        "/drivers/system/system_config.c.xdt": {
            driver_config: "/security/keyring_import/templates/keyring_import.c.xdt",
        },
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/security/keyring_import/templates/keyring_import.h.xdt",
        },
    },

    defaultInstanceName: "Keyring",
    maxInstances: 1,

    config: [
        {
            name: "KeyringCertSize",
            displayName: "Keyring Cert Size",
            default: "0",
            hidden: true,
        },
        {
            name: "KeyringCertArr",
            displayName: "Keyring Cert Array",
            default: "",
            hidden: true,
        },
        {
            name: "keyringCertLoad",
            displayName: "Keyring Cert Header File",
            buttonText: "Load Keyring Cert",
            description: "Select Keyring certificate header file",
            fileFilter: ".h",
            pickDirectory: false,
            nonSerializable: true,
            onLaunch: (inst) => {
                let products=system.getProducts()
                let nodeCmd=common.getNodePath()
                let sdkPath = ""
                let copyScriptPath = ""
                if(system.getOS() == "win") {
                    sdkPath = products[0].path.split("\\.metadata\\product.json")[0];
                    copyScriptPath = sdkPath + "//source//board//.meta//flash//copyutil.js";
                } else {
                    sdkPath = products[0].path.split("/.metadata/product.json")[0];
                    copyScriptPath = sdkPath + "/source/board/.meta/flash/copyutil.js";
                }
                return {
                    command: nodeCmd,
                    args: [copyScriptPath, "$browsedFile", "$comFile"],
                    initialData: "initialData",
                    inSystemPath: true,
                };
            },
            onComplete: (inst, _ui, result) => {

                const resultData = result.data;

                // Extract KeyringCertSize
                const keyringCertSizeStart = resultData.indexOf("#define CUST_KEYRINGCERT_SIZE_IN_BYTES");
                const keyringCertSizeEnd = resultData.indexOf(")", keyringCertSizeStart) + 1;
                inst.KeyringCertSize = resultData.substring(keyringCertSizeStart, keyringCertSizeEnd);

                // Extract KeyringCertArr
                const keyringCertArrStart = resultData.indexOf("#define CUST_KEYRINGCERT {");
                const keyringCertArrEnd = resultData.indexOf("}", keyringCertArrStart) + 1;
                inst.KeyringCertArr = resultData.substring(keyringCertArrStart, keyringCertArrEnd);

            },
        },
    ],

    getInstanceConfig,

};

exports = keyring_import_module;