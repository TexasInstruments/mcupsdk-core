"use strict";

let common = system.getScript("/common");
let device = common.getDeviceName();

function enet_icssg_mdio_validate(instance, report) {
    if (instance.mdioMode === "MDIO_MODE_NORMAL")
    {
        if (instance.mdioPollEnMask.length > 2)
        {
            report.logError("MDIO Cfg: Cannot monitor more than TWO PHY adddress in MDIO_NORMAL_MODE", instance);
        }
    }
    if (instance.mdioMode === "MDIO_MODE_MANUAL")
    {
        if (instance.mdioPollEnMask.length > 0)
        {
            report.logError("MDIO Cfg: Cannot monitor any PHY adddress in MDIO_MANUAL_MODE", instance);
        }
    }
}

const enet_icssg_mdio_config = {
    name: "mdioConfig",
    displayName: "MDIO Config",
    longDescription: "Configuration of ICSSG MDIO module",
    config: [
        {
            name: "mdioMode",
            description: "MDIO module operating mode selection",
            displayName: "Operating Mode",
            default: "MDIO_MODE_STATE_CHANGE_MON",
            options: [
                {
                    name: "MDIO_MODE_NORMAL",
                },
                {
                    name: "MDIO_MODE_STATE_CHANGE_MON",
                },
                {
                    name: "MDIO_MODE_MANUAL",
                },
            ],
            onChange: function (inst, ui) {
                if (inst.mdioMode === "MDIO_MODE_MANUAL")
                {
                    ui.mdioPollEnMask.hidden = true;
                    ui.mdioDisableStateMachineOnInit.hidden = true;
                    inst.mdioDisableStateMachineOnInit = true;
                    inst.mdioPollEnMask = [];
                }
                else
                {
                    ui.mdioPollEnMask.hidden = false;
                    ui.mdioDisableStateMachineOnInit.hidden = false;
                    inst.mdioDisableStateMachineOnInit = false;
                }
            },
        },
        {
            name: "mdioBusFreqHz",
            description: "MDIO bus clock (MDCLK) frequency in Hz",
            displayName: "MDCLK Frequency",
            default: 2200000,
            isInteger: true,
            range: [1, 1000000000],
        },
        {
            name: "mdioIPGRatio",
            description: "PHY State Poll interval in number of MDCLK ticks. This is also called as 'Polling inter packet gap'",
            displayName: "PHY State Poll Interval",
            default: 100,
            isInteger: true,
            range: [1, 255],
        },
        {
            name: "mdioPollEnMask",
            longDescription: "Indicates which PHY addresses have polling enabled (bit mask)",
            displayName: "Monitored PHY Addresses",
            default: Array.from(Array(32).keys()).map(String),
            minSelections: 0,
            options: _.keys(Array(32)).map((index)=>({name: index})),
        },
        {
            name: "mdioC45EnMask",
            description: "Indicates which PHY addresses will use Clause-45 frame format",
            displayName: "Supports MDIO Clause-45",
            default: [],
            minSelections: 0,
            options: _.keys(Array(32)).map((index)=>({name: index})),
        },
        {
            name: "mdioIsMaster",
            longDescription: "MDIO module can be shared by multiple peripherals, so only one peripheral(master) must perform the MDIO initial configuration and be skipped by the other peripherals (slaves). There must be only one peripheral marked as master among the those that share the MDIO.  This condition is not enforced in driver, so it's application's responsibility this requirement is met",
            displayName: "Set As Master",
            default: true,
        },
        {
            name: "mdioDisableStateMachineOnInit",
            longDescription: "Config to disable MDIO state machine on MDIO_open. By default MDIO state machine is enabled on MDIO_open. Setting this flag, will cause MDIO to not enable MDIO state machine on open. The MDIO state machine can be enabled at a later point of time via IOCTL. This option allows synchronizing externally managed PHY with the MDIO whereby external PHY initalization is done and then MDIO state machine can be enabled so that any linkup interrupts are not missed",
            displayName: "Disable Statemachine On Open",
            hidden: false,
            default: false,
        },
    ],
    collapsed:true,
};


exports = {
    config: enet_icssg_mdio_config,
    validate: enet_icssg_mdio_validate,
};
