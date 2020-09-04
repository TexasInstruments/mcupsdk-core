"use strict";

let common = system.getScript("/common");
let device = common.getDeviceName();

function enet_iccsg_timesync_validate(instance, report) {

}

const enet_iccsg_timesync_config = {
    name: "timssync_config",
    displayName: "TimeSync Config",
    longDescription: "ICSS Time Sync Configuration",
    config: [
        {
            name: "timesyncEnable",
            description: "Whether TimeSync Will Be Enabled Or Not",
            longDescription: "In case of Dual-MAC, only one port can use TimeSync at a time, so it  should be disabled via this config parameter. This parameter will be overwritten (to false) if it's detected that TimeSync is already is use for another Dual-MAC port. There is no limitation in ICSSG Switch, it can be kept enabled",
            displayName: "Enable",
            default: false,
            onChange: function (inst, ui) {
                if (inst.timesyncEnable == false)
                {
                    ui.timesyncClkType.hidden = true;
                    ui.timesyncSyncOut_start_WC.hidden = true;
                    ui.timesyncSyncOut_pwidth_WC.hidden = true;
                }
                else
                {
                    ui.timesyncClkType.hidden = false;
                    ui.timesyncSyncOut_start_WC.hidden = false;
                    ui.timesyncSyncOut_pwidth_WC.hidden = false;
                }
            },
            hidden: false,
        },
        {
            name: "timesyncClkType",
            description: "ICSSG TimeSync Clock types",
            displayName: "Clock Type",
            default: "ICSSG_TIMESYNC_CLKTYPE_WORKING_CLOCK",
            options: [
                {
                    name: "ICSSG_TIMESYNC_CLKTYPE_WORKING_CLOCK",
                },
                {
                    name: "ICSSG_TIMESYNC_CLKTYPE_SYSTEM_TIME",
                },
                {
                    name: "ICSSG_TIMESYNC_CLKTYPE_GLOBAL_TIME",
                },
            ],
            hidden: true,
        },
        {
            name: "timesyncSyncOut_start_WC",
            description: "Working clock SyncOut start time within the cycle ",
            displayName: "Syncout Start Time",
            default: 10000,
            isInteger: true,
            hidden: true,
        },
        {
            name: "timesyncSyncOut_pwidth_WC",
            description: "Working Clock SyncOut Pulse Width",
            displayName: "Syncout Pulse Width",
            default: 25000,
            isInteger: true,
            hidden: true,
        },
    ],
    collapsed:true,
};


exports = {
    config: enet_iccsg_timesync_config,
    validate: enet_iccsg_timesync_validate,
};
