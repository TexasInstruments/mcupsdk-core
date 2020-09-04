
let common = system.getScript("/common");


let csirx_ctx_module = {
    displayName: "CSIRX Context",
    defaultInstanceName: "CONFIG_CSIRX_CONTEXT",
    config: [
        {
            name: "virtualChannelId",
            displayName: "Virtual Channel ID",
            description: "Virtual channel Id as per MIPI spec",
            default: 0,
            skipTests: ["displayNameCheck"],
            options : [
                { name: 0, }, { name: 1, }, { name: 2, }, { name: 3, },
            ]
        },
        {
            name: "format",
            displayName: "Data Format",
            default: "CSIRX_FORMAT_RAW8",
            skipTests: ["displayNameCheck"],
            options : [
                { name: "CSIRX_FORMAT_RAW6", },
                { name: "CSIRX_FORMAT_RAW7", },
                { name: "CSIRX_FORMAT_RAW8", },
                { name: "CSIRX_FORMAT_RAW10", },
                { name: "CSIRX_FORMAT_RAW12", },
                { name: "CSIRX_FORMAT_RAW14", },
                { name: "CSIRX_FORMAT_RAW6_EXP8", },
                { name: "CSIRX_FORMAT_RAW7_EXP8", },
                { name: "CSIRX_FORMAT_RAW10_EXP16", },
                { name: "CSIRX_FORMAT_RAW12_EXP16", },
                { name: "CSIRX_FORMAT_RAW14_EXP16", },
                { name: "CSIRX_FORMAT_RGB565", },
                { name: "CSIRX_FORMAT_RGB888", },
                { name: "CSIRX_FORMAT_RGB444_EXP16", },
                { name: "CSIRX_FORMAT_RGB555_EXP16", },
                { name: "CSIRX_FORMAT_RGB666_EXP32", },
                { name: "CSIRX_FORMAT_RGB888_EXP32", },
                { name: "CSIRX_FORMAT_RGB666_EXP32_24", },
                { name: "CSIRX_FORMAT_YUV420_8_BIT", },
                { name: "CSIRX_FORMAT_YUV420_8_BIT_LEGACY", },
                { name: "CSIRX_FORMAT_YUV420_8_BIT_CSPS", },
                { name: "CSIRX_FORMAT_YUV420_10_BIT", },
                { name: "CSIRX_FORMAT_YUV420_10_BIT_CSPS", },
                { name: "CSIRX_FORMAT_YUV422_8_BIT", },
                { name: "CSIRX_FORMAT_YUV422_10_BIT", },
                { name: "CSIRX_FORMAT_USER_DEFINED_8_BIT_DATA_TYPE_1", },
                { name: "CSIRX_FORMAT_USER_DEFINED_8_BIT_DATA_TYPE_2", },
                { name: "CSIRX_FORMAT_USER_DEFINED_8_BIT_DATA_TYPE_3", },
                { name: "CSIRX_FORMAT_USER_DEFINED_8_BIT_DATA_TYPE_4", },
                { name: "CSIRX_FORMAT_USER_DEFINED_8_BIT_DATA_TYPE_5", },
                { name: "CSIRX_FORMAT_USER_DEFINED_8_BIT_DATA_TYPE_6", },
                { name: "CSIRX_FORMAT_USER_DEFINED_8_BIT_DATA_TYPE_7", },
                { name: "CSIRX_FORMAT_USER_DEFINED_8_BIT_DATA_TYPE_8", },
                { name: "CSIRX_FORMAT_USER_DEFINED_8_BIT_DATA_TYPE_1_EXP8", },
                { name: "CSIRX_FORMAT_USER_DEFINED_8_BIT_DATA_TYPE_2_EXP8", },
                { name: "CSIRX_FORMAT_USER_DEFINED_8_BIT_DATA_TYPE_3_EXP8", },
                { name: "CSIRX_FORMAT_USER_DEFINED_8_BIT_DATA_TYPE_4_EXP8", },
                { name: "CSIRX_FORMAT_USER_DEFINED_8_BIT_DATA_TYPE_5_EXP8", },
                { name: "CSIRX_FORMAT_USER_DEFINED_8_BIT_DATA_TYPE_6_EXP8", },
                { name: "CSIRX_FORMAT_USER_DEFINED_8_BIT_DATA_TYPE_7_EXP8", },
                { name: "CSIRX_FORMAT_USER_DEFINED_8_BIT_DATA_TYPE_8_EXP8", },
                { name: "CSIRX_FORMAT_OTHERS_EXCEPT_NULL_AND_BLANKING", },
                { name: "CSIRX_FORMAT_EMBEDDED_8_BIT_NON_IMAGE", },
            ]
        },
        {
            name: "userDefinedMapping",
            displayName: "User Defined Mapping",
            description: "Selects the pixel format of USER_DEFINED in data format CSIRX_DATA_FORMAT`` configuration",
            default: "CSIRX_USER_DEFINED_FORMAT_RAW6",
            skipTests: ["displayNameCheck"],
            options: [
                { name: "CSIRX_USER_DEFINED_FORMAT_RAW6" },
                { name: "CSIRX_USER_DEFINED_FORMAT_RAW7" },
                { name: "CSIRX_USER_DEFINED_FORMAT_RAW8" },
            ]
        },
        {
            name: "numFramesToAcquire",
            displayName: "Number of Frames To Acquire (16-bit)",
            description: "Set to 0, for infinite frame capture",
            default: 0,
            skipTests: ["displayNameCheck"],
        },
        {
            name: "numLinesForIntr",
            displayName: "Number of Lines For Interrupt (16-bit)",
            default: 2,
            skipTests: ["displayNameCheck"],
        },
        {
            name: "isGenerateIntrEveryNumLinesForIntr",
            displayName: "Generate Interrupt Every `Number of Lines For Interrupt`",
            longDescription: "If checked, interrupt is generated every `Number of Lines For Interrupt`, else interrupt is generated once after `Number of Lines For Interrupt`",
            default: true,
            skipTests: ["displayNameCheck"],
        },
        {
            name: "alpha",
            displayName: "Alpha (16-bit)",
            displayFormat: "hex",
            description: "Controls the padding for *_EXP16, *_EXP32 and *_EXP32_24 data formats of the data format field",
            default: 0,
            skipTests: ["displayNameCheck"],
        },
        {
            name: "isByteSwapEnabled",
            displayName: "Byte Swap Enabled",
            default: false,
            skipTests: ["displayNameCheck"],
        },
        {
            name: "isGenericEnabled",
            displayName: "Generic Enabled",
            longDescription: "If checked, data is received as per format and the long packet code transmitted in the MIPI stream is ignored. If unchecked, data is received as per format and the long packet code transmitted in the MIPI stream is used.",
            default: false,
            skipTests: ["displayNameCheck"],
        },
        {
            name: "isEndOfFramePulseEnabled",
            displayName: "End Of Frame Pulse Enabled",
            description: "This controls the combined End of Frame interrupt",
            default: false,
            skipTests: ["displayNameCheck"],
        },
        {
            name: "isEndOfLinePulseEnabled",
            displayName: "End Of Line Pulse Enabled",
            description: "This controls the combined End of Line interrupt",
            default: false,
            skipTests: ["displayNameCheck"],
        },
        {
            name: "isPayloadChecksumEnable",
            displayName: "Payload Checksum Enable",
            description: "Enables checksum checking of long packet payload",
            default: false,
            skipTests: ["displayNameCheck"],
        },
        {
            name: "transcodeConfig",
            displayName: "Transcode Config",
            collapsed: true,
            config: [
                {
                    name: "transcodeFormat",
                    displayName: "Transcode Format",
                    default: "CSIRX_TRANSCODE_FORMAT_NO_TRANSCODE",
                    skipTests: ["displayNameCheck"],
                    options: [
                        { name: "CSIRX_TRANSCODE_FORMAT_NO_TRANSCODE" },
                        { name: "CSIRX_TRANSCODE_FORMAT_IN_RAW10_ALAW_OUT_RAW8" },
                        { name: "CSIRX_TRANSCODE_FORMAT_IN_RAW8_OUT_RAW8" },
                        { name: "CSIRX_TRANSCODE_FORMAT_IN_RAW10_OUT_RAW10_EXP16" },
                        { name: "CSIRX_TRANSCODE_FORMAT_IN_RAW10_OUT_RAW10_PACKED" },
                        { name: "CSIRX_TRANSCODE_FORMAT_IN_RAW12_OUT_RAW12_EXP16" },
                        { name: "CSIRX_TRANSCODE_FORMAT_IN_RAW12_OUT_RAW12_PACKED" },
                        { name: "CSIRX_TRANSCODE_FORMAT_IN_RAW14_OUT_RAW14" },
                    ]
                },
                {
                    name: "isHorizontalDownscalingBy2Enabled",
                    displayName: "Horizontal Downscaling By Two Enabled",
                    default: false,
                    skipTests: ["displayNameCheck"],
                },
                {
                    name: "cropConfig",
                    displayName: "Crop Config",
                    collapsed: false,
                    config: [
                        {
                            name: "horizontalSkip",
                            displayName: "Crop Start X (16-bit)",
                            default: 0,
                            skipTests: ["displayNameCheck"],
                        },
                        {
                            name: "verticalSkip",
                            displayName: "Crop Start Y (16-bit)",
                            default: 0,
                            skipTests: ["displayNameCheck"],
                        },
                        {
                            name: "horizontalCount",
                            displayName: "Crop Width (16-bit)",
                            default: 0,
                            skipTests: ["displayNameCheck"],
                        },
                        {
                            name: "verticalCount",
                            displayName: "Crop Height (16-bit)",
                            default: 0,
                            skipTests: ["displayNameCheck"],
                        },
                    ]
                }
            ]
        },
        {
            name: "pingPongConfig",
            displayName: "Ping Pong Config",
            collapsed: false,
            config: [
                {
                    name: "pingAddress",
                    displayName: "Ping Address",
                    displayFormat: "hex",
                    description: "MUST be 32 byte aligned",
                    default: 0,
                    skipTests: ["displayNameCheck"],
                },
                {
                    name: "pongAddress",
                    displayName: "Pong Address",
                    displayFormat: "hex",
                    description: "MUST be 32 byte aligned",
                    default: 0,
                    skipTests: ["displayNameCheck"],
                },
                {
                    name: "lineOffset",
                    displayName: "Line Offset in bytes (16-bit signed)",
                    description: "Set to 0, for contiguous storage",
                    default: 0,
                    skipTests: ["displayNameCheck"],
                },
                {
                    name: "pingPongSwitchMode",
                    displayName: "Ping Pong Switch Mode",
                    longDescription: "If line based, ping-pong switch happens every `Number of Lines For Line Based Ping Pong Switching` lines. If frame based, ping-pong switch happens every `Number of Frames For Frame Based Ping Pong Switching`",
                    default: "CSIRX_PING_PONG_FRAME_SWITCHING",
                    skipTests: ["displayNameCheck"],
                    options: [
                        { name: "CSIRX_PING_PONG_FRAME_SWITCHING", },
                        { name: "CSIRX_PING_PONG_LINE_SWITCHING", },
                    ]
                },
                {
                    name: "numFramesForFrameBasedPingPongSwitching",
                    displayName: "Number Of Frames For Frame Based Ping Pong Switching (8-bit)",
                    description: "Typically used in interlaced mode (=2). For progressive mode is set to 1.",
                    default: 1,
                    skipTests: ["displayNameCheck"],
                },
                {
                    name: "numLinesForLineBasedPingPongSwitching",
                    displayName: "Number Of Lines For Line Based Ping Pong Switching (16-bit)",
                    description: "Typically used in radar for multi-chirp (when chirp is a line) processing",
                    default: 2,
                    skipTests: ["displayNameCheck"],
                },
            ],
        },
        {
            name: "enableIntr",
            displayName: "Enable Interrupts",
            config: [
                {
                    name: "isNumLines",
                    displayName: "Num Lines Interrupt Enable",
                    default: false,
                    skipTests: ["displayNameCheck"],
                    longDescription: "If checked, a interrupt is generated after number of lines specified in `Number of Lines For Interrupt` were received. The interrupt repeatability within the frame is decided based on `Generate Interrupt Every Number of Lines` configuration.",
                },
                {
                    name: "isFramesToAcquire",
                    displayName: "Frames To Acquire Interrupt Enable",
                    default: false,
                    skipTests: ["displayNameCheck"],
                    longDescription: "If checked, a interrupt is generated after `Number of Frames To Acquire` frames have been acquired.",
                },
                {
                    name: "isPayloadChecksumMismatch",
                    displayName: "Payload Checksum Mismatch Interrupt Enable",
                    default: false,
                    skipTests: ["displayNameCheck"],
                },
                {
                    name: "isLineStartCodeDetect",
                    displayName: "Line Start Code Detect Interrupt Enable",
                    default: false,
                    skipTests: ["displayNameCheck"],
                },
                {
                    name: "isLineEndCodeDetect",
                    displayName: "Line End Code Detect Interrupt Enable",
                    default: false,
                    skipTests: ["displayNameCheck"],
                },
                {
                    name: "isFrameStartCodeDetect",
                    displayName: "Frame Start Code Detect Interrupt Enable",
                    default: false,
                    skipTests: ["displayNameCheck"],
                },
                {
                    name: "isFrameEndCodeDetect",
                    displayName: "Frame End Code Detect Interrupt Enable",
                    default: false,
                    skipTests: ["displayNameCheck"],
                },
                {
                    name: "isLongPacketOneBitErrorCorrect",
                    displayName: "Long Packet One Bit Error Correct Interrupt Enable",
                    default: false,
                    skipTests: ["displayNameCheck"],
                },
            ]
        }
    ],
    validate: validate,
};

function validate(inst, report) {

    common.validate.checkNumberRange(inst, report, "numFramesToAcquire", 0, 64*1024-1, "dec");
    common.validate.checkNumberRange(inst, report, "numLinesForIntr", 0, 64*1024-1, "dec");

    common.validate.checkNumberRange(inst, report, "alpha", 0, 64*1024-1, "dec");

    common.validate.checkNumberRange(inst, report, "horizontalCount", 0, 64*1024-1, "dec");
    common.validate.checkNumberRange(inst, report, "horizontalSkip", 0, 64*1024-1, "dec");
    common.validate.checkNumberRange(inst, report, "verticalCount", 0, 64*1024-1, "dec");
    common.validate.checkNumberRange(inst, report, "verticalSkip", 0, 64*1024-1, "dec");

    common.validate.checkNumberRange(inst, report, "numFramesForFrameBasedPingPongSwitching", 0, 255, "dec");
    common.validate.checkNumberRange(inst, report, "numLinesForLineBasedPingPongSwitching", 0, 0xFFFF, "dec");
    common.validate.checkNumberRange(inst, report, "pingAddress", 0, 0xFFFFFFFF, "hex");
    common.validate.checkNumberRange(inst, report, "pongAddress", 0, 0xFFFFFFFF, "hex");
    common.validate.checkNumberRange(inst, report, "lineOffset", -32*1024, 32*1024-1, "dec");
    if( (inst.pingAddress % 32) != 0 )
    {
        report.logError("MUST be 32 byte aligned", inst, "pingAddress");
    }
    if( (inst.pingAddress % 32) != 0 )
    {
        report.logError("MUST be 32 byte aligned", inst, "pongAddress");
    }
}

exports = csirx_ctx_module;
