
let common = system.getScript("/common");

let csirx_dphy_module = {
    displayName: "CSIRX DPHY",
    defaultInstanceName: "CONFIG_CSIRX_DPHY",
    config: [
        {
            name: "ddrClockInHz",
            displayName: "DPHY Clock in Hz",
            description: "DPHY DDR clock speed in Hz",
            default: 400000000,
            skipTests: ["displayNameCheck"],
        },
        {
            name: "isClockMissingDetectionEnabled",
            displayName: "Clock Missing Detection Enabled",
            description: "Enable clock missing detector",
            default: true,
            skipTests: ["displayNameCheck"],
        },
    ],
    validate: validate,
};

function validate(inst, report) {
    common.validate.checkNumberRange(inst, report, "ddrClockInHz", 100*1000000, 1000*1000000, "dec");
}

exports = csirx_dphy_module;
