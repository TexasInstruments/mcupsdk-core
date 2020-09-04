
let common = system.getScript("/common");

function getLaneConfig(name, displayName, defaultPosition)
{
    return {
        name: name,
        displayName: displayName,
        collapsed: false,
        config: [
            {
                name: `${name}_polarity`,
                displayName: "Polarity",
                default: "CSIRX_LANE_POLARITY_PLUS_MINUS",
                skipTests: ["displayNameCheck"],
                options: [
                    {
                        name: "CSIRX_LANE_POLARITY_PLUS_MINUS"
                    },
                    {
                        name: "CSIRX_LANE_POLARITY_MINUS_PLUS"
                    },
                ]
            },
            {
                name: `${name}_position`,
                displayName: "Position",
                default: defaultPosition,
                skipTests: ["displayNameCheck"],
                options: [
                    {
                        name: "CSIRX_LANE_POSITION_LANE_NOT_USED"
                    },
                    {
                        name: "CSIRX_LANE_POSITION_1"
                    },
                    {
                        name: "CSIRX_LANE_POSITION_2"
                    },
                    {
                        name: "CSIRX_LANE_POSITION_3"
                    },
                    {
                        name: "CSIRX_LANE_POSITION_4"
                    },
                    {
                        name: "CSIRX_LANE_POSITION_5"
                    },
                ]
            },
        ],
    };
}


function getLaneIntrConfig()
{
    return {
        name: "lane",
        displayName: "Lane Interrupts",
        description: "Applies to all lanes including clock and data lanes",
        collapsed: false,
        config: [
            {
                name: "isStateTransitionToULPM",
                displayName: "State Transition To ULPM Interrupt Enable",
                default: true,
                skipTests: ["displayNameCheck"],
            },
            {
                name: "isControlError",
                displayName: "Control Error Interrupt Enable",
                default: true,
                skipTests: ["displayNameCheck"],
            },
            {
                name: "isEscapeEntryError",
                displayName: "Escape Entry Error Interrupt Enable",
                default: true,
                skipTests: ["displayNameCheck"],
            },
            {
                name: "isStartOfTransmissionSyncError",
                displayName: "Start Of Transmission Sync Error Interrupt Enable",
                default: true,
                skipTests: ["displayNameCheck"],
            },
            {
                name: "isStartOfTransmissionError",
                displayName: "Start Of Transmission Error Interrupt Enable",
                default: true,
                skipTests: ["displayNameCheck"],
            },
        ],
    };
}

let csirx_complexio_module = {
    displayName: "CSIRX Complex IO",
    defaultInstanceName: "CONFIG_CSIRX_COMPLEXIO",
    config: [
        {
            name: "isPowerAuto",
            displayName: "Auto Power Save",
            description: "Automatically switches between ULP and ON states based on ULPM signals from complex IO",
            default: false,
            skipTests: ["displayNameCheck"],
        },
        {
            name: "enableIntr",
            displayName: "Enable Interrupts",
            collapsed: true,
            config: [
                {
                    name: "isAllLanesEnterULPM",
                    displayName: "All Lanes Enter ULPM Interrupt Enable",
                    default: false,
                    skipTests: ["displayNameCheck"],
                },
                {
                    name: "isAllLanesExitULPM",
                    displayName: "All Lanes Exit ULPM Interrupt Enable",
                    default: false,
                    skipTests: ["displayNameCheck"],
                },
                getLaneIntrConfig(),
            ],
        },
        getLaneConfig("clockLane", "Clock Lane Config", "CSIRX_LANE_POSITION_3"),
        getLaneConfig("dataLane0", "Data Lane 0 Config", "CSIRX_LANE_POSITION_1"),
        getLaneConfig("dataLane1", "Data Lane 1 Config", "CSIRX_LANE_POSITION_2"),
        getLaneConfig("dataLane2", "Data Lane 2 Config", "CSIRX_LANE_POSITION_4"),
        getLaneConfig("dataLane3", "Data Lane 3 Config", "CSIRX_LANE_POSITION_5"),
    ],
    validate: validate,
};

function validate(inst, report) {

    if(inst.clockLane_position != "CSIRX_LANE_POSITION_LANE_NOT_USED"
        &&
        (      inst.dataLane0_position == inst.clockLane_position
            || inst.dataLane1_position == inst.clockLane_position
            || inst.dataLane2_position == inst.clockLane_position
            || inst.dataLane3_position == inst.clockLane_position
        )
    ) {
        report.logError("Conflicting lane position", inst, "clockLane_position");
    }
    if(inst.dataLane0_position != "CSIRX_LANE_POSITION_LANE_NOT_USED"
        &&
        (      inst.clockLane_position == inst.dataLane0_position
            || inst.dataLane1_position == inst.dataLane0_position
            || inst.dataLane2_position == inst.dataLane0_position
            || inst.dataLane3_position == inst.dataLane0_position
        )
    ) {
        report.logError("Conflicting lane position", inst, "dataLane0_position");
    }
    if(inst.dataLane1_position != "CSIRX_LANE_POSITION_LANE_NOT_USED"
        &&
        (      inst.clockLane_position == inst.dataLane1_position
            || inst.dataLane0_position == inst.dataLane1_position
            || inst.dataLane2_position == inst.dataLane1_position
            || inst.dataLane3_position == inst.dataLane1_position
        )
    ) {
        report.logError("Conflicting lane position", inst, "dataLane1_position");
    }
    if(inst.dataLane2_position != "CSIRX_LANE_POSITION_LANE_NOT_USED"
        &&
        (      inst.clockLane_position == inst.dataLane2_position
            || inst.dataLane1_position == inst.dataLane2_position
            || inst.dataLane0_position == inst.dataLane2_position
            || inst.dataLane3_position == inst.dataLane2_position
        )
    ) {
        report.logError("Conflicting lane position", inst, "dataLane2_position");
    }
    if(inst.dataLane3_position != "CSIRX_LANE_POSITION_LANE_NOT_USED"
        &&
        (      inst.clockLane_position == inst.dataLane3_position
            || inst.dataLane1_position == inst.dataLane3_position
            || inst.dataLane2_position == inst.dataLane3_position
            || inst.dataLane0_position == inst.dataLane3_position
        )
    ) {
        report.logError("Conflicting lane position", inst, "dataLane3_position");
    }
    if(inst.clockLane_position == "CSIRX_LANE_POSITION_LANE_NOT_USED")
    {
        report.logError("Clock lane MUST be USED", inst, "clockLane_position");
    }
    if(     inst.dataLane0_position == "CSIRX_LANE_POSITION_LANE_NOT_USED"
        &&  inst.dataLane1_position == "CSIRX_LANE_POSITION_LANE_NOT_USED"
        &&  inst.dataLane2_position == "CSIRX_LANE_POSITION_LANE_NOT_USED"
        &&  inst.dataLane3_position == "CSIRX_LANE_POSITION_LANE_NOT_USED"
    )
    {
        report.logError("Atleast one data lane MUST be USED", inst, "dataLane0_position");
        report.logError("Atleast one data lane MUST be USED", inst, "dataLane1_position");
        report.logError("Atleast one data lane MUST be USED", inst, "dataLane2_position");
        report.logError("Atleast one data lane MUST be USED", inst, "dataLane3_position");
    }
}

exports = csirx_complexio_module;
