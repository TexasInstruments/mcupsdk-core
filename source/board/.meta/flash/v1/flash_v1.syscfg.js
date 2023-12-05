let common = system.getScript("/common");
let soc = system.getScript(`/board/flash/flash_${common.getSocName()}`);

let addressingModeDescription = `
In flashes with size more than 16 MB, we need 4 address bytes to access memory. But most
flashes in reset configuration start with a 3 byte addressing mode setup. 4 byte addressing
mode is handled differently across flashes. There are mainly three types of addressing
supported:

0: 3-Byte only addressing
1: 3-byte or 4-byte addressing (Enters 4-Byte addressing on command)
2: 4-Byte only addressing

Most cases it is Type 1, so there is a sequence of commands which need to be given
to switch to 4 byte addressing mode. It's given here as a bitmap, according to the
flash, the correct bit sequence has to be given in hex for 4 byte enable sequence.

Here 'x' can be 0 or 1.

- xxxxxxx*1* : Issue instruction 0xB7 without write enable
- xxxxxx*1*x : Issue write enable, then issue 0xB7
- xxxxx*1*xx : 8 bit register used to define the 31:24 bits in the address. Set this
             register to switch to the correct memory bank.
- xxxx*1*xxx : Bit 7 of an 8-bit register is used to enable/disable 4 byte address
             mode. When this is 0, other bits of the register can be used to specify
             full address.
- xxx*1*xxxx : 16 bit register used to control 3/4 byte addressing mode. Writing 1
             makes addressing mode as 4 bytes
- xx*1*xxxxx : Supports dedicated 4 byte instruction set. Check datasheet
- x*1*xxxxxx : Always 4 byte mode
`
let flashResetDescription = `
Flashes support different types of software reset sequences to return the device to its default power-on state.

Here 'x' can be 0 or 1.

- 000000 : Soft Reset Not Supported
- xxxxx*1* : Drive 0x0F on all 4 data wires for 8 clocks
- xxxx*1*x : Drive 0x0F on all 4 data wires for 10 clocks if device is in 4 byte addressing
- xxx*1*xx : Drive 0x0F on all 4 data wires for 16 clocks
- xx*1*xxx : Issue instruction 0xF0
- x*1*xxxx : Issue 0x66 for reset enable, and 0x99 for reset
`
let quadEnableDescription = `
Sequence to enable quad mode **(1s-1s-4s)** changes from flash to flash. According to the JEDEC standards, there are 7 different types of Quad Enable(QE):

 - **0** : Device does not have a QE bit. Detects based on instruction.
 - **1** : QE is bit 1 of status register 2. It is set via Write Status with two data bytes where bit 1 of the second byte is one. It is cleared via Write Status with two data bytes where bit 1 of the second byte is zero. Writing only one byte to the status register has the side-effect of clearing status register 2, including the QE bit.
 - **2** : QE is bit 6 of status register 1. It is set via Write Status with one data byte where bit 6 is one. It is cleared via Write Status with one data byte where bit 6 is zero
 - **3** : QE is bit 7 of status register 2. It is set via Write status register 2 instruction 3Eh with one data byte where bit 7 is one. It is cleared via Write status register 2 instruction "0x3E" with one data byte where bit 7 is zero. The status register 2 is read using instruction "0x3F".
 - **4** : QE is bit 1 of status register 2. It is set via Write Status with two data bytes where bit 1 of the second byte is one. It is cleared via Write Status with two data bytes where bit 1 of the second byte is zero. Writing one byte to the status register does not modify status register 2.
 - **5** : QE is bit 1 of the status register 2. Status register 1 is read using Read Status instruction "0x05". Status register 2 is read using instruction "0x35". QE is set via Write Status instruction 01h with two data bytes where bit 1 of the second byte is one. It is cleared via Write Status with two data bytes where bit 1 of the second byte is zero.
 - **6** : QE is bit 1 of the status register 2. Status register 1 is read using Read Status instruction "0x05". Status register 2 is read using instruction "0x35", and status register 3 is read using instruction "0x15". QE is set via Write Status Register instruction "0x31" with one data byte where bit 1 is one. It is cleared via Write Status Register instruction "0x31" with one data byte where bit 1 is zero.

If your flash is not a quad flash, the value of this field is a don't care.
 `
let eraseConfigDescription = `
Flashes support multiple erase types / sizes. According to JEDS216 D, there are 4 different erase types / sizes. For simplicity we only consider
2 erase sizes in our config. We call the bigger erase size a **"block"** and smaller erase size a **"sector"**.

Say your flash supports 2 or more) erase types, say 3 erase types - 256KB, 64KB and 4KB

You can choose any two out of these and make the larger one **block** and smaller one **sector**. Say we choose 256 KB as block size and 4 KB as
sector size. The following configurables need to be filled based on the flash you're using:

1. Size of the block in bytes (256 x 1024 = 262144 in this case)
2. Size of the sector in bytes (4 x 1024 = 4096 in this case)
3. CMD to erase block in 3 byte addressing mode
4. CMD to erase block in 4 byte addressing mode
5. CMD to erase sector in 3 byte addressing mode
6. CMD to erase sector in 4 byte addressing mode
`
let customProtoDescription = `
If you have selected a custom protocol, different than the ones already provided, a function to set the
protocol also needs to be provided. This function can be defined in the application, and give the function name here.

The function signature would be
~~~
int32_t myProtocolSetFxn(Flash_Config *cfg)
{
    int32_t status = SystemP_SUCCESS;

    /* Your code for setting the protocol goes here */

    return status;
}
~~~

You can define this function in your application's source. It will be invoked in this fashion:
~~~
Board_open()
    -> Flash_open()
        -> Flash_*Open()
            -> myProtocolSetFxn(Flash_Config *cfg)
~~~
`
let quirksDescription = `
Certain flashes would need extra configurations for the flash to work completely. An example is hybrid sector configuration.
So this is a hook to add such quirks, the function mentioned here would be invoked at the end of the **open** function in flash driver.

The function signature would be
~~~
int32_t myQuirksFxn(Flash_Config *cfg)
{
    int32_t status = SystemP_SUCCESS;

    /* Your code for handling quirks goes here */

    return status;
}
~~~

You can define this function in your application's source. It will be invoked in this fashion:
~~~
Board_open()
    -> Flash_open()
        -> Flash_*Open()
            -> myQuirksFxn(Flash_Config *cfg)
~~~
`
function getDriver(drvName) {
    return system.getScript(`/drivers/${drvName}/${drvName}`);
}

function getInstanceConfig(moduleInstance) {

    return {
        ...moduleInstance,
    };
};

let defaultProtocols = getDriver(soc.getDefaultDriver()).getSupportedProtocols();

let flash_module_name = "/board/flash/flash";

let flash_module = {
    displayName: "FLASH",

    templates: {
        "/board/board/board_open_close.c.xdt": {
            board_open_close_config: "/board/flash/templates/v1/flash_open_close_config.c.xdt",
            board_open: "/board/flash/templates/flash_open.c.xdt",
            board_close: "/board/flash/templates/flash_close.c.xdt",
        },
        "/board/board/board_open_close.h.xdt": {
            board_open_close_config: "/board/flash/templates/flash_open_close.h.xdt",
        },
        "/board/board/board_config.h.xdt": {
            board_config: "/board/flash/templates/flash.h.xdt",
        },

    },
    defaultInstanceName: "CONFIG_FLASH",
    config: [
        {
            name: "device",
            displayName: "Flash Device",
            default: "TI_DEFAULT_FLASH",
            options: [
                { name: "TI_DEFAULT_FLASH" , displayName: "TI Board Default Flash" },
                { name: "CUSTOM_FLASH" , displayName: "Custom Flash" },
            ],
            onChange: function(inst, ui) {
                if(inst.device == "TI_DEFAULT_FLASH") {
                    inst.fname = soc.getDefaultFlashName();
                    inst.protocol = soc.getDefaultProtocol().name;
                    fillConfigs(inst, soc.getDefaultFlashConfig());
                } else if(inst.device == "CUSTOM_FLASH") {
                    inst.fname = "";
                }
            }
        },
        {
            name: "fname",
            displayName: "Flash Name",
            default: soc.getDefaultFlashName(),
            placeholder: "Type your flash name here",
        },
        {
            name: "protocol",
            displayName: "Protocol",
            description: "The NOR SPI protocol to be used",
            default: soc.getDefaultProtocol().name,
            options: defaultProtocols,
            onChange: function(inst, ui) {
                let hideLines = true;
                if(inst.protocol == "custom") {
                    hideLines = false;
                }

                let hideCustomProtocolSettings = true;

                if(inst.protocol == "custom") {
                    hideCustomProtocolSettings = false;
                }

                /* Add custom protocol settings */
                ui.customProtoFxn.hidden = hideCustomProtocolSettings;

                /* Add manual config stuff */
                ui.cmdLines.hidden = hideLines;
                ui.addrLines.hidden = hideLines;
                ui.dataLines.hidden = hideLines;
            }
        },
        {
            name: "cmdLines",
            displayName: "CMD Lines",
            description: "Number of transfer lines to be used for sending CMD",
            default: "1",
            options: [
                { name: "1" },
            ],
            hidden: true,
        },
        {
            name: "addrLines",
            displayName: "ADDR Lines",
            description: "Number of transfer lines to be used for sending ADDR",
            default: "1",
            options: [
                { name: "1" },
            ],
            hidden: true,
        },
        {
            name: "dataLines",
            displayName: "DATA Lines",
            description: "Number of transfer lines to be used for sending DATA",
            default: "1",
            options: [
                { name: "1" },
                { name: "2" },
                { name: "4" },
            ],
            hidden: true,
        },
        /* Flash Config */
        {
            name: "basicFlashCfg",
            displayName: "Basic Flash Configuration",
            collapsed: false,
            config: [
                {
                    name: "flashSize",
                    displayName: "Flash Size In Bytes",
                    default: soc.getDefaultFlashConfig().flashSize,
                    displayFormat: "dec",
                },
                {
                    name: "flashPageSize",
                    displayName: "Flash Page Size In Bytes",
                    default: soc.getDefaultFlashConfig().flashPageSize,
                    displayFormat: "dec",
                },
                {
                    name: "flashManfId",
                    displayName: "Flash JEDEC Manufacturer ID",
                    default: soc.getDefaultFlashConfig().flashManfId,
                },
                {
                    name: "flashDeviceId",
                    displayName: "Flash JEDEC Device ID",
                    default: soc.getDefaultFlashConfig().flashDeviceId,
                },
                {
                    name: "eraseConfig",
                    displayName: "Erase Configurations",
                    longDescription: eraseConfigDescription,
                    config : [
                        {
                            name: "flashBlockSize",
                            displayName: "Flash Block Size In Bytes",
                            default: soc.getDefaultFlashConfig().flashBlockSize,
                        },
                        {
                            name: "flashSectorSize",
                            displayName: "Flash Sector Size In Bytes",
                            default: soc.getDefaultFlashConfig().flashSectorSize,
                        },
                        {
                            name: "cmdBlockErase3B",
                            displayName: "Block Erase CMD (3B)",
                            description: "Command to erase a block in 3-Byte addressing mode",
                            default: soc.getDefaultFlashConfig().cmdBlockErase3B,
                        },
                        {
                            name: "cmdBlockErase4B",
                            displayName: "Block Erase CMD (4B)",
                            description: "Command to erase a block in 4-byte addressing mode",
                            default: soc.getDefaultFlashConfig().cmdBlockErase4B,
                        },
                        {
                            name: "cmdSectorErase3B",
                            displayName: "Sector Erase CMD (3B)",
                            description: "Command to erase a block in 3-Byte addressing mode",
                            default: soc.getDefaultFlashConfig().cmdSectorErase3B,
                        },
                        {
                            name: "cmdSectorErase4B",
                            displayName: "Sector Erase CMD (4B)",
                            description: "Command to erase a block in 4-byte addressing mode",
                            default: soc.getDefaultFlashConfig().cmdSectorErase4B,
                        },
                    ]
                },
                {
                    name: "protEnCfg",
                    displayName: "Protocol Enable Configuration",
                    config : [
                        {
                            name: "cmdRd",
                            displayName: "Read Command",
                            default: soc.getDefaultFlashConfig().protos.p114.cmdRd,
                        },
                        {
                            name: "cmdWr",
                            displayName: "Write/Page Program Command",
                            default: soc.getDefaultFlashConfig().protos.p114.cmdWr,
                        },
                        {
                            name: "modeClksCmd",
                            displayName: "Mode Clocks (CMD)",
                            description: "Mode clocks required while sending command",
                            default: soc.getDefaultFlashConfig().protos.p114.modeClksCmd,
                            displayFormat: "dec",
                        },
                        {
                            name: "modeClksRd",
                            displayName: "Mode Clocks (READ)",
                            description: "Mode clocks required while reading data",
                            default: soc.getDefaultFlashConfig().protos.p114.modeClksRd,
                            displayFormat: "dec",
                        },
                        {
                            name: "dummyClksCmd",
                            displayName: "Dummy Clocks (CMD)",
                            description: "Dummy Clocks required while sending command",
                            default: soc.getDefaultFlashConfig().protos.p114.dummyClksCmd,
                            displayFormat: "dec",
                        },
                        {
                            name: "dummyClksRd",
                            displayName: "Dummy Clocks (READ)",
                            description: "Dummy Clocks required while reading data",
                            default: soc.getDefaultFlashConfig().protos.p114.dummyClksRd,
                            displayFormat: "dec",
                        },
                        {
                            name: "flashQeType",
                            displayName: "Quad Enable Type",
                            description: "The type of Quad Enable supported by the flash",
                            longDescription: quadEnableDescription,
                            default: soc.getDefaultFlashConfig().protos.p114.enableType,
                            options: [
                                { name : "0" },
                                { name : "1" },
                                { name : "2" },
                                { name : "3" },
                                { name : "4" },
                                { name : "5" },
                                { name : "6" },
                            ],
                        },
                        {
                            name: "customProtoFxn",
                            displayName: "Custom Protocol Fxn",
                            longDescription: customProtoDescription,
                            default: "NULL",
                            hidden: true,
                        },
                    ]
                },
            ]
        },
        /* Advanced flash configuration options - Mostly wouldn't need changes */
        {
            name: "advancedFlashCfg",
            displayName: "Advanced Flash Configuration",
            collapsed: true,
            config: [
                {
                    name: "resetType",
                    displayName: "Reset Type",
                    description: "Type of reset supported by the flash",
                    longDescription: flashResetDescription,
                    default: soc.getDefaultFlashConfig().resetType,
                },
                {
                    name: "cmdWren",
                    displayName: "Write Enable CMD",
                    description: "Command to enable writes",
                    default: soc.getDefaultFlashConfig().cmdWren,
                },
                {
                    name: "cmdRdsr",
                    displayName: "Read Status Register CMD",
                    description: "Command to read the status register",
                    default: soc.getDefaultFlashConfig().cmdRdsr,
                },
                {
                    name: "srWip",
                    displayName: "WIP Bit",
                    description: "WIP bit position in status register",
                    default: soc.getDefaultFlashConfig().srWip,
                    displayFormat: "dec",
                },
                {
                    name: "srWel",
                    displayName: "WEL Bit",
                    description: "WEL bit position in status register",
                    default: soc.getDefaultFlashConfig().srWel,
                    displayFormat: "dec",
                },
                {
                    name: "cmdChipErase",
                    displayName: "Chip Erase Command",
                    description: "Command to erase the whole flash",
                    default: soc.getDefaultFlashConfig().cmdChipErase,
                },
                {
                    name: "flashDeviceBusyTimeout",
                    displayName: "Flash Busy Timeout",
                    description: "Time to wait for flash to be ready",
                    default: soc.getDefaultFlashConfig().flashDeviceBusyTimeout,
                    displayFormat: "dec",
                },
                {
                    name: "flashPageProgTimeout",
                    displayName: "Page Program Timeout",
                    description: "Time to wait for a page write to complete",
                    default: soc.getDefaultFlashConfig().flashPageProgTimeout,
                    displayFormat: "dec",
                },
                {
                    name: "enable4BAddr",
                    displayName: "Enable 4 Byte Addressing",
                    default: false,
                    onChange: function(inst, ui) {
                        let hide4B = true;
                        if(inst.enable4BAddr == true) {
                            hide4B = false;
                        }
                        ui.addressByteSupport.hidden = hide4B;
                        ui.fourByteEnableSeq.hidden = hide4B;
                    }
                },
                {
                    name: "fourByteAddressSupport",
                    displayName: "4 Byte Addressing",
                    longDescription: addressingModeDescription,
                    config: [
                        {
                            name: "addressByteSupport",
                            displayName: "Supported Addressing Modes",
                            default: "0x00",
                            hidden: true,
                        },
                        {
                            name: "fourByteEnableSeq",
                            displayName: "4 Byte Addressing Enable Sequence",
                            default: "0x00",
                            hidden: true,
                        }
                    ]
                },
            ]
        },
        {
            name: "autoCfgFlashGroup",
            displayName: "Automatically Configure Flash",
            collapsed: false,
            config: [
                {
                    name: "autoCfgFlashFromFile",
                    displayName: "Load Flash Config From JSON",
                    buttonText: "Load From JSON",
                    fileFilter: ".json",
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
                        if(result.data === "error") {
                            inst.fname = "ERROR LOADING CONFIG";
                            return;
                        } else if(result.data === "initialData") {
                            inst.fname = "ERROR RUNNING SCRIPT";
                        } else {
                            let flashConfig = null;

                            try {
                                flashConfig = JSON.parse(result.data);
                            } catch (e) {
                                inst.fname = "ERROR PARSING CONFIG";
                                return;
                            }
                            /* Fill up the configurables with data from JSON file */
                            fillConfigs(inst, flashConfig);
                            return;
                        }
                    },
                },
            ],
        },
        /* Quirks */
        {
            name: "quirks",
            displayName: "Quirks Function",
            description: "Function to handle any vendor specific quirks of the flash",
            longDescription: quirksDescription,
            default: "NULL",
        }
    ],

    validate: validate,
    moduleStatic: {
        modules: function(inst) {
            return [{
                name: "system_common",
                moduleName: "/system_common",
            }]
        },
    },
    moduleInstances: moduleInstances,
    getInstanceConfig,
};

function isValidHexString(s, n) {
    if("0x" == s.slice(0, 2)) {
        let re = new RegExp(`[0-9A-Fa-f]{${n}}`, "g");
        return (s.slice(2).length == n) && (re.test(s.slice(2)));
    } else {
        return false;
    }
}

function validateCmd(inst, cmdName, report) {
    if(!isValidHexString(inst[cmdName], 2)) {
        report.logError(`${cmdName} should be a 2 digit hexadecimal string with leading 0x, for example 0x03 !!!`, inst, cmdName);
    }
}

function validate(inst, report) {
    common.validate.checkSameFieldName(inst, "name", report);

    /* Validate flash name */
    common.validate.checkValidCName(inst, report, "fname");

    /* Validate flash size */
    if(inst.flashSize > 32*1024*1024) {
        report.logError("Maximum flash size supported is 32 MB !!!", inst, "flashSize");
    }
    if(inst.flashSize % 1024*1024) {
        report.logError("Flash size should be multiple of MB (MegaByte) !!!", inst, "flashSize");
    }

    /* Validate flash page size */
    if(inst.flashPageSize % 256) {
        report.logError("Flash page size should be multiple of 256 Bytes !!!", inst, "flashPageSize");
    }

    /* Validate flash manf ID */
    if(!isValidHexString(inst.flashManfId, 2)) {
        report.logError("Manufacturer ID should be a 2 digit hexadecimal string with leading 0x, for example 0x02 !!!", inst, "flashManfId");
    }

    /* Validate flash device ID */
    if(!isValidHexString(inst.flashDeviceId, 4)) {
        report.logError("Device ID should be a 4 digit hexadecimal string with leading 0x, for example 0x0123 !!!", inst, "flashDeviceId");
    }

    /* Validate flash block size */
    if(inst.flashBlockSize % 1024) {
        report.logError("Flash block size should be multiple of KB (KiloByte) !!!", inst, "flashBlockSize");
    }

    /* Validate flash sector size */
    if(inst.flashSectorSize % 1024) {
        report.logError("Flash sector size should be multiple of KB (KiloByte) !!!", inst, "flashSectorSize");
    }

    /* Validate commands */
    validateCmd(inst, "cmdBlockErase3B", report);
    validateCmd(inst, "cmdBlockErase4B", report);
    validateCmd(inst, "cmdSectorErase3B", report);
    validateCmd(inst, "cmdSectorErase4B", report);
    validateCmd(inst, "cmdRd", report);
    validateCmd(inst, "cmdWr", report);
    validateCmd(inst, "cmdWren", report);
    validateCmd(inst, "cmdRdsr", report);
    validateCmd(inst, "cmdChipErase", report);

    validateCmd(inst, "resetType", report);

    common.validate.checkNumberRange(inst, report, "modeClksCmd", 0, 255, "dec");
    common.validate.checkNumberRange(inst, report, "modeClksRd", 0, 255, "dec");
    common.validate.checkNumberRange(inst, report, "dummyClksCmd", 0, 255, "dec");
    common.validate.checkNumberRange(inst, report, "dummyClksRd", 0, 255, "dec");
    common.validate.checkNumberRange(inst, report, "srWip", 0, 7, "dec");
    common.validate.checkNumberRange(inst, report, "srWel", 0, 7, "dec");
    common.validate.checkNumberRange(inst, report, "flashDeviceBusyTimeout", 0, 4294967295, "dec");
    common.validate.checkNumberRange(inst, report, "flashPageProgTimeout", 0, 4294967295, "dec");
}

function moduleInstances(inst) {

    let modInstances = new Array();
    let moduleName = soc.getDefaultDriver()

    if(inst.protocol == "custom") {
        modInstances.push({
            name: "peripheralDriver",
            displayName: "QSPI Driver Configuration",
            moduleName: `/drivers/${moduleName}/${moduleName}`,
            useArray: false,
            requiredArgs: {
                protocol: "custom",
                cmdLines: inst.cmdLines,
                addrLines: inst.addrLines,
                dataLines: inst.dataLines,
            },
        })
    } else {
        modInstances.push({
            name: "peripheralDriver",
            displayName: "QSPI Driver Configuration",
            moduleName: `/drivers/${moduleName}/${moduleName}`,
            useArray: false,
            requiredArgs: {
                protocol: inst.protocol,
            },
        })
    }

    return (modInstances);
}

function fillConfigs(inst, cfg) {
    /* Basic Config */
    inst.flashSize = cfg.flashSize;
    inst.flashPageSize = cfg.flashPageSize;
    inst.flashManfId = cfg.flashManfId;
    inst.flashDeviceId = cfg.flashDeviceId;
    inst.flashBlockSize = cfg.flashBlockSize;
    inst.flashSectorSize = cfg.flashSectorSize;
    inst.cmdBlockErase3B = cfg.cmdBlockErase3B;
    inst.cmdBlockErase4B = cfg.cmdBlockErase4B;
    inst.cmdSectorErase3B = cfg.cmdSectorErase3B;
    inst.cmdSectorErase4B = cfg.cmdSectorErase4B;

    /* Protocol Configs */
    /* 1-1-1 */
    let protoToCfgMap = {
        "1s_1s_1s" : "p111",
        "1s_1s_2s" : "p112",
        "1s_1s_4s" : "p114",
        "custom"   : "pCustom",
    }

    let pCfg = cfg.protos[protoToCfgMap[inst.protocol]];

    if(pCfg != null)
    {
        inst.cmdRd = pCfg.cmdRd;
        inst.cmdWr = pCfg.cmdWr;
        inst.modeClksCmd = pCfg.modeClksCmd;
        inst.modeClksRd = pCfg.modeClksRd;
        inst.dummyClksCmd = pCfg.dummyClksCmd;
        inst.dummyClksRd = pCfg.dummyClksRd;
        inst.flashQeType = pCfg.enableType;
        /* Custom */
        if(inst.protocol == "custom") {
            if(cfg.protos.pCustom.fxn != null) {
                inst.customProtoFxn = cfg.protos.pCustom.fxn;
            }
        }
    }
    if("fourByteAddrEnSeq" in cfg) {
        inst.fourByteEnableSeq = cfg.fourByteAddrEnSeq;
    }
    inst.flashDeviceBusyTimeout = cfg.flashDeviceBusyTimeout;
    inst.flashPageProgTimeout = cfg.flashPageProgTimeout;
}

exports = flash_module;
