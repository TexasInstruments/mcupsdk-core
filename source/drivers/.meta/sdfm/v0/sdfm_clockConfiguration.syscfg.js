let common   = system.getScript("/common");
let device_peripheral = system.getScript(`/drivers/sdfm/soc/sdfm_${common.getSocName()}.syscfg.js`);

let ClockConfigs = [];

ClockConfigs = ClockConfigs.concat([
    {
        name:"Ch1_SDCLKSEL",
        displayName: "Channel 1 SDCLK Source",
        description: "Channel 1 SDCLK source",
        hidden: true,
        default: device_peripheral.SDFM_ClockSource[1].name,
        options: [device_peripheral.SDFM_ClockSource[1]]
    },
    {
        name:"Ch2_SDCLKSEL",
        displayName: "Channel 2 SDCLK Source",
        description: "Channel 2 SDCLK source",
        hidden: true,
        default: device_peripheral.SDFM_ClockSource[0].name,
        options: device_peripheral.SDFM_ClockSource
    },
    {
        name:"Ch3_SDCLKSEL",
        displayName: "Channel 3 SDCLK Source",
        description: "Channel 3 SDCLK source",
        hidden: true,
        default: device_peripheral.SDFM_ClockSource[0].name,
        options: device_peripheral.SDFM_ClockSource
    },
    {
        name:"Ch4_SDCLKSEL",
        displayName: "Channel 4 SDCLK Source",
        description: "Channel 4 SDCLK source",
        hidden: true,
        default: device_peripheral.SDFM_ClockSource[0].name,
        options: device_peripheral.SDFM_ClockSource
    }
])

let sdfmClockConfiguration = {
    displayName: "SDFM Clock Configuration",
    defaultInstanceName: "SDFM_ClockConfig",
    description: "Sigma Delta Filter Modulator Clock Configuration",
    config: ClockConfigs,
    templates: {
        boardc : "",
        boardh : ""
    },
};

exports = sdfmClockConfiguration

