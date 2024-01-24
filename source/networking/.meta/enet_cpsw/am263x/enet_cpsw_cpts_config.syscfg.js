"use strict";

let common = system.getScript("/common");
let device = common.getDeviceName();

function enet_cpsw_cpts_validate(instance, report) {

}

const enet_cpsw_cpts_config = {
    name: "cptsConfig",
    displayName: "CPTS Configuration",
    longDescription: "Configuration information for the CPTS module",
    config: [
        {
            name: "cptsHostRxTsEn",
            description: "Host Receive timestamp enable: When set, timestamps are enabled on received packets to host",
            displayName: "Enable Host Receive Timestamp",
            default: true,
        },
        {
            name: "cptsTsCompPolarity",
            description: "Timestamp Assertion Polarity",
            displayName: "Timestamp Assertion Polarity",
            default: "ASSERT_HIGH",
            options: [
                {
                    name: "ASSERT_HIGH",
                },
                {
                    name: "ASSERT_LOW",
                },
            ],
        },
        {
            name: "cptsTsRxEventsDis",
            description: "Disable all timestamp Ethernet receive events",
            displayName: "Disable Timestamp Receive Events",
            default: false,
        },
        {
            name: "cptstsGenfClrEn",
            longDescription: "GENF (and ESTF) clear enable. True:  A TS_GENFn (or TS_ESTFn) output is cleared when the associated ts_genf_length[31:0] (or ts_estf_length[31:0]) is cleared to zero. False: A TS_GENFn (or TS_ESTFn) output is not cleared when the associated ts_genf_length[31:0] (or ts_estf_length[31:0]) is cleared to zero",
            displayName: "Enable GENF (And ESTF) Clear",
            default: true,
        },
        {
            name: "cptsRftClkFreq",
            description: "CPTS RFT clock frequency required to set TS_ADD VAL",
            displayName: "CPTS Clock Frequency Add Value",
            default: "CPSW_CPTS_RFTCLK_FREQ_200MHZ",
            options: [
                {
                    name: "CPSW_CPTS_RFTCLK_FREQ_1000MHZ",
                },
                {
                    name: "CPSW_CPTS_RFTCLK_FREQ_500MHZ",
                },
                {
                    name: "CPSW_CPTS_RFTCLK_FREQ_333_33MHZ",
                },
                {
                    name: "CPSW_CPTS_RFTCLK_FREQ_250MHZ",
                },
                {
                    name: "CPSW_CPTS_RFTCLK_FREQ_200MHZ",
                },
                {
                    name: "CPSW_CPTS_RFTCLK_FREQ_166_66MHZ",
                },
                {
                    name: "CPSW_CPTS_RFTCLK_FREQ_142_85MHZ",
                },
                {
                    name: "CPSW_CPTS_RFTCLK_FREQ_125MHZ",
                },
            ],
        },
    ],
    collapsed:true,
};


exports = {
    config: enet_cpsw_cpts_config,
    validate: enet_cpsw_cpts_validate,
};
