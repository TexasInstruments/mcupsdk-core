
let common = system.getScript("/common");

function getInstanceConfig(moduleInstance) {
    return {
        ...moduleInstance,
    };
};

let udma_ch_blkcopy_module = {
    displayName: "UDMA Block Copy Channel Configuration",
    longDescription: `
This adds and configures a block copy DMA channel.
User can then use the created channel handle and submit TRPD to perform DMA operations.

Note: The SysConfig doesn't check the number of channels allocated to a particular core.
It is left to the user to allocate the right number of channels in SysConfig.

The channel creation can fail at runtime in SysConfig generated Drivers_open() under following conditions
1. No Blockcopy DMA channels are allocated to the current core in DMSC board configuration
2. Number of blockcopy channel added in the SysConfig exceeds the allocated channels for the current core in DMSC board configuration
3. Some other SW entity (other than the SysConfig generated Drivers_open() has allocated the channels leading to shortage of channels for Drivers_open()
    `,
    alwaysShowLongDescription: true,
    defaultInstanceName: "CONFIG_UDMA_BLKCOPY_CH",
    config: [
        /* Blockcopy Channel attributes */
        {
            name: "elemCnt",
            displayName: "Ring Element Count",
            default: 1,
            description: "Sets the queue depth of the DMA ring",
        },
        {
            name: "intrEnable",
            displayName: "Interrupt Mode Enable",
            default: false,
            description: "Interrupt Mode. When selected the SysConfig will configure UDMA to generate event on transfer completion",
            onChange: function (inst, ui) {
                let hideConfigs = true;
                if(inst.intrEnable == true) {
                    hideConfigs = false;
                }
                ui.transferCallbackFxn.hidden = hideConfigs;
            },
        },
        {
            name: "transferCallbackFxn",
            displayName: "Transfer Callback",
            default: "NULL",
            hidden: true,
            description: "Transfer callback function when interrupt mode is selected",
        },
    ],
    validate: validate,
    getInstanceConfig,
};

/*
 *  ======== validate ========
 */
function validate(instance, report) {
    common.validate.checkNumberRange(instance, report, "elemCnt", 0x1, 0xFFFFFFFF, "hex");
    common.validate.checkValidCName(instance, report, "transferCallbackFxn");
    if((instance.intrEnable == true) &&
        ((instance.transferCallbackFxn == "NULL") ||
            (instance.transferCallbackFxn == ""))) {
        report.logError("Callback function MUST be provided for interrupt mode", instance, "transferCallbackFxn");
    }
}

exports = udma_ch_blkcopy_module;
