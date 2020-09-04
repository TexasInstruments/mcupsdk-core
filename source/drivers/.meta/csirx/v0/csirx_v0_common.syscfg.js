
let common = system.getScript("/common");


function getIntrCallback(name, displayName)
{
    return {
        name: name,
        displayName: displayName,
        description: "Make sure a function with the name specified here is implemented and linked in to the application",
        default: "NULL",
        skipTests: ["displayNameCheck"],
    };
}

function getIntrCallbackArgs(name, displayName)
{
    return {
        name: `${name}Args`,
        displayName: `${displayName} Arguments`,
        description: "Make sure there is a initialized global variable pointer which points to the desired callback argument",
        default: "NULL",
        skipTests: ["displayNameCheck"],
    };
}

let csirx_common_module = {
    displayName: "CSIRX Common",
    defaultInstanceName: "CONFIG_CSIRX_COMMON",
    config: [
        {
            name: "isSoftStoppingOnInterfaceDisable",
            displayName: "Soft Stopping On Interface Disable",
            description: "When `CSIRX_commonDisable` is issued, the interface stops after full frames are received in all active contexts",
            default: true,
            skipTests: ["displayNameCheck"],
        },
        {
            name: "isHeaderErrorCheckEnabled",
            displayName: "Header Error Check Enabled",
            description: "Enables the Error Correction Code check for the received header (short and long packets for all virtual channel ids)",
            default: false,
            skipTests: ["displayNameCheck"],
        },
        {
            name: "isSignExtensionEnabled",
            displayName: "Sign Extension Enabled",
            description: "Enables sign extension of RAW10/12/14 for all contexts whose context format is among those with EXP16 output",
            default: false,
            skipTests: ["displayNameCheck"],
        },
        {
            name: "burstSize",
            displayName: "Burst Size",
            description: "Sets the DMA burst size on the interconnect",
            default: "CSIRX_BURST_SIZE_16X64",
            skipTests: ["displayNameCheck"],
            options: [
                {
                    name: "CSIRX_BURST_SIZE_1X64",
                },
                {
                    name: "CSIRX_BURST_SIZE_2X64",
                },
                {
                    name: "CSIRX_BURST_SIZE_4X64",
                },
                {
                    name: "CSIRX_BURST_SIZE_8X64",
                },
                {
                    name: "CSIRX_BURST_SIZE_16X64",
                },
            ],
        },
        {
            name: "endianness",
            displayName: "Endianness",
            default: "CSIRX_ENDIANNESS_LITTLE_ENDIAN",
            skipTests: ["displayNameCheck"],
            options: [
                {
                    name: "CSIRX_ENDIANNESS_NATIVE_MIPI_CSI2 ",
                },
                {
                    name: "CSIRX_ENDIANNESS_LITTLE_ENDIAN",
                }
            ]
        },
        {
            name: "startOfFrameIntr0ContextId",
            displayName: "Start Of Frame Interrupt 0 Context ID",
            default: 0,
            skipTests: ["displayNameCheck"],
            options: [
                { name: 0, }, { name: 1, }, { name: 2, }, { name: 3, },
                { name: 4, }, { name: 5, }, { name: 6, }, { name: 7, },
            ]
        },
        {
            name: "startOfFrameIntr1ContextId",
            displayName: "Start Of Frame Interrupt 1 Context ID",
            default: 0,
            skipTests: ["displayNameCheck"],
            options: [
                { name: 0, }, { name: 1, }, { name: 2, }, { name: 3, },
                { name: 4, }, { name: 5, }, { name: 6, }, { name: 7, },
            ]
        },
        {
            name: "enableIntr",
            displayName: "Enable Interrupts",
            collapsed: true,
            config: [
                {
                    name: "isOcpError",
                    displayName: "Ocp Error Interrupt Enable",
                    default: false,
                    skipTests: ["displayNameCheck"],
                },
                {
                    name: "isGenericShortPacketReceive",
                    displayName: "Generic Short Packet Receive Interrupt Enable",
                    default: false,
                    skipTests: ["displayNameCheck"],
                },
                {
                    name: "isOneBitShortPacketErrorCorrect",
                    displayName: "One Bit Short Packet Error Correct Interrupt Enable",
                    default: false,
                    skipTests: ["displayNameCheck"],
                },
                {
                    name: "isMoreThanOneBitShortPacketErrorCannotCorrect",
                    displayName: "More Than One Bit Short Packet Error Cannot Correct Interrupt Enable",
                    default: false,
                    skipTests: ["displayNameCheck"],
                },
                {
                    name: "isComplexioError",
                    displayName: "Complexio Error Interrupt Enable",
                    default: false,
                    skipTests: ["displayNameCheck"],
                },
                {
                    name: "isFifoOverflow",
                    displayName: "Fifo Overflow Interrupt Enable",
                    default: false,
                    skipTests: ["displayNameCheck"],
                },
            ],
        },
        {
            name: "intrCallbaks",
            displayName: "Register Interrupt Callbacks",
            collapsed: true,
            config: [
                getIntrCallback("combinedEndOfLineCallback", "Combined End Of Line Callback"),
                getIntrCallbackArgs("combinedEndOfLineCallback", "Combined End Of Line Callback"),
                getIntrCallback("combinedEndOfFrameCallback", "Combined End Of Frame Callback"),
                getIntrCallbackArgs("combinedEndOfFrameCallback", "Combined End Of Frame Callback"),
                getIntrCallback("commonCallback", "Common Callback"),
                getIntrCallbackArgs("commonCallback", "Common Callback"),
                getIntrCallback("startOfFrameIntr0Callback", "Start of Frame (SOF) Interrupt 0 Callback"),
                getIntrCallbackArgs("startOfFrameIntr0Callback", "Start of Frame (SOF) Interrupt 0 Callback"),
                getIntrCallback("startOfFrameIntr1Callback", "Start of Frame (SOF) Interrupt 1 Callback"),
                getIntrCallbackArgs("startOfFrameIntr1Callback", "Start of Frame (SOF) Interrupt 1 Callback"),
            ]
        },
    ],
    validate: validate,
};

function validate(inst, report) {
    common.validate.checkValidCName(inst, report, "combinedEndOfLineCallback");
    common.validate.checkValidCName(inst, report, "combinedEndOfLineCallbackArgs");
    common.validate.checkValidCName(inst, report, "combinedEndOfFrameCallback");
    common.validate.checkValidCName(inst, report, "combinedEndOfFrameCallbackArgs");
    common.validate.checkValidCName(inst, report, "commonCallback");
    common.validate.checkValidCName(inst, report, "commonCallbackArgs");
    common.validate.checkValidCName(inst, report, "startOfFrameIntr0Callback");
    common.validate.checkValidCName(inst, report, "startOfFrameIntr0CallbackArgs");
    common.validate.checkValidCName(inst, report, "startOfFrameIntr1Callback");
    common.validate.checkValidCName(inst, report, "startOfFrameIntr1CallbackArgs");
}

exports = csirx_common_module;
