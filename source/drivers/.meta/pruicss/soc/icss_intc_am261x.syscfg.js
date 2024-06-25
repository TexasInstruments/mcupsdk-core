let ICSSM0_MODE0_INTC_INTERNAL_SIGNALS = [
    { eventNumber: "31", interruptSignal: "pr0_pru_mst_intr[15]_intr_req", source: "PRU_ICSSM0 PRU0, PRU_ICSSM0 PRU1" },
    { eventNumber: "30", interruptSignal: "pr0_pru_mst_intr[14]_intr_req", source: "PRU_ICSSM0 PRU0, PRU_ICSSM0 PRU1" },
    { eventNumber: "29", interruptSignal: "pr0_pru_mst_intr[13]_intr_req", source: "PRU_ICSSM0 PRU0, PRU_ICSSM0 PRU1" },
    { eventNumber: "28", interruptSignal: "pr0_pru_mst_intr[12]_intr_req", source: "PRU_ICSSM0 PRU0, PRU_ICSSM0 PRU1" },
    { eventNumber: "27", interruptSignal: "pr0_pru_mst_intr[11]_intr_req", source: "PRU_ICSSM0 PRU0, PRU_ICSSM0 PRU1" },
    { eventNumber: "26", interruptSignal: "pr0_pru_mst_intr[10]_intr_req", source: "PRU_ICSSM0 PRU0, PRU_ICSSM0 PRU1" },
    { eventNumber: "25", interruptSignal: "pr0_pru_mst_intr[9]_intr_req", source: "PRU_ICSSM0 PRU0, PRU_ICSSM0 PRU1" },
    { eventNumber: "24", interruptSignal: "pr0_pru_mst_intr[8]_intr_req", source: "PRU_ICSSM0 PRU0, PRU_ICSSM0 PRU1" },
    { eventNumber: "23", interruptSignal: "pr0_pru_mst_intr[7]_intr_req", source: "PRU_ICSSM0 PRU0, PRU_ICSSM0 PRU1" },
    { eventNumber: "22", interruptSignal: "pr0_pru_mst_intr[6]_intr_req", source: "PRU_ICSSM0 PRU0, PRU_ICSSM0 PRU1" },
    { eventNumber: "21", interruptSignal: "pr0_pru_mst_intr[5]_intr_req", source: "PRU_ICSSM0 PRU0, PRU_ICSSM0 PRU1" },
    { eventNumber: "20", interruptSignal: "pr0_pru_mst_intr[4]_intr_req", source: "PRU_ICSSM0 PRU0, PRU_ICSSM0 PRU1" },
    { eventNumber: "19", interruptSignal: "pr0_pru_mst_intr[3]_intr_req", source: "PRU_ICSSM0 PRU0, PRU_ICSSM0 PRU1" },
    { eventNumber: "18", interruptSignal: "pr0_pru_mst_intr[2]_intr_req", source: "PRU_ICSSM0 PRU0, PRU_ICSSM0 PRU1" },
    { eventNumber: "17", interruptSignal: "pr0_pru_mst_intr[1]_intr_req", source: "PRU_ICSSM0 PRU0, PRU_ICSSM0 PRU1" },
    { eventNumber: "16", interruptSignal: "pr0_pru_mst_intr[0]_intr_req", source: "PRU_ICSSM0 PRU0, PRU_ICSSM0 PRU1" },
    { eventNumber: "15", interruptSignal: "pr0_ecap_intr_req", source: "PRU_ICSSM0 ECAP0" },
    { eventNumber: "14", interruptSignal: "pr0_sync0_out_pend", source: "PRU_ICSSM0 IEP0" },
    { eventNumber: "13", interruptSignal: "pr0_sync1_out_pend", source: "PRU_ICSSM0 IEP0" },
    { eventNumber: "12", interruptSignal: "pr0_latch0_in (input to PRU_ICSSM0)", source: "PRU_ICSSM0 IEP0" },
    { eventNumber: "11", interruptSignal: "pr0_latch1_in (input to PRU_ICSSM0)", source: "PRU_ICSSM0 IEP0" },
    { eventNumber: "10", interruptSignal: "pr0_pdi_wd_exp_pend", source: "PRU_ICSSM0 IEP0" },
    { eventNumber: "9",  interruptSignal: "pr0_pd_wd_exp_pend", source: "PRU_ICSSM0 IEP0" },
    { eventNumber: "8",  interruptSignal: "pr0_digio_event_req", source: "PRU_ICSSM0 IEP0" },
    { eventNumber: "7",  interruptSignal: "pr0_iep_tim_cap_cmp_pend", source: "PRU_ICSSM0 IEP0" },
    { eventNumber: "6",  interruptSignal: "pr0_uart0_uint_intr_req", source: "PRU_ICSSM0 UART0" },
    { eventNumber: "5",  interruptSignal: "pr0_uart0_utxevt_intr_req", source: "PRU_ICSSM0 UART0" },
    { eventNumber: "4",  interruptSignal: "pr0_uart0_urxevt_intr_req", source: "PRU_ICSSM0 UART0" },
    { eventNumber: "3",  interruptSignal: "pr0_rst_reset_iso_req", source: "PRU_ICSSM0 Reset Isolation Requested" },
    { eventNumber: "2",  interruptSignal: "pr0_pru1_r31_status_cnt16", source: "PRU_ICSSM0 PRU1 (Shift Capture)" },
    { eventNumber: "1",  interruptSignal: "pr0_pru0_r31_status_cnt16", source: "PRU_ICSSM0 PRU0 (Shift Capture)" },
    { eventNumber: "0",  interruptSignal: "pr0_ecc_err_intr", source: "PRU_ICSSM0 ECC Logic" },
];

ICSSM0_MODE0_INTC_INTERNAL_SIGNALS.reverse();

let ICSSM0_MODE0_INTC_EXTERNAL_SIGNALS = [
    //Event_no: 32 starts
    { interruptSignal: "PRU_ICSSM0_PR1_SLV_IN_0",  interruptId: "0",  source: "PRU_ICSS_XBAR_INTR_0"},
    { interruptSignal: "PRU_ICSSM0_PR1_SLV_IN_1",  interruptId: "1",  source: "PRU_ICSS_XBAR_INTR_1" },
    { interruptSignal: "PRU_ICSSM0_PR1_SLV_IN_2",  interruptId: "2",  source: "PRU_ICSS_XBAR_INTR_2" },
    { interruptSignal: "PRU_ICSSM0_PR1_SLV_IN_3",  interruptId: "3",  source: "PRU_ICSS_XBAR_INTR_3" },
    { interruptSignal: "PRU_ICSSM0_PR1_SLV_IN_4",  interruptId: "4",  source: "PRU_ICSS_XBAR_INTR_4" },
    { interruptSignal: "PRU_ICSSM0_PR1_SLV_IN_5",  interruptId: "5",  source: "PRU_ICSS_XBAR_INTR_5" },
    { interruptSignal: "PRU_ICSSM0_PR1_SLV_IN_6",  interruptId: "6",  source: "PRU_ICSS_XBAR_INTR_6" },
    { interruptSignal: "PRU_ICSSM0_PR1_SLV_IN_7",  interruptId: "7",  source: "PRU_ICSS_XBAR_INTR_7" },
    { interruptSignal: "PRU_ICSSM0_PR1_SLV_IN_8",  interruptId: "8",  source: "PRU_ICSS_XBAR_INTR_8" },
    { interruptSignal: "PRU_ICSSM0_PR1_SLV_IN_9",  interruptId: "9",  source: "PRU_ICSS_XBAR_INTR_9" },
    { interruptSignal: "PRU_ICSSM0_PR1_SLV_IN_10", interruptId: "10", source: "PRU_ICSS_XBAR_INTR_10" },
    { interruptSignal: "PRU_ICSSM0_PR1_SLV_IN_11", interruptId: "11", source: "PRU_ICSS_XBAR_INTR_11" },
    { interruptSignal: "PRU_ICSSM0_PR1_SLV_IN_12", interruptId: "12", source: "PRU_ICSS_XBAR_INTR_12" },
    { interruptSignal: "PRU_ICSSM0_PR1_SLV_IN_13", interruptId: "13", source: "PRU_ICSS_XBAR_INTR_13" },
    { interruptSignal: "PRU_ICSSM0_PR1_SLV_IN_14", interruptId: "14", source: "PRU_ICSS_XBAR_INTR_14" },
    { interruptSignal: "PRU_ICSSM0_PR1_SLV_IN_15", interruptId: "15", source: "PRU_ICSS_XBAR_INTR_15" },
    { interruptSignal: "PRU_ICSSM0_PR1_SLV_IN_16", interruptId: "16", source: "CONTROL_SS_OUTPUT_XBAR_0" },
    { interruptSignal: "PRU_ICSSM0_PR1_SLV_IN_17", interruptId: "17", source: "CONTROL_SS_OUTPUT_XBAR_1" },
    { interruptSignal: "PRU_ICSSM0_PR1_SLV_IN_18", interruptId: "18", source: "CONTROL_SS_OUTPUT_XBAR_2" },
    { interruptSignal: "PRU_ICSSM0_PR1_SLV_IN_19", interruptId: "19", source: "CONTROL_SS_OUTPUT_XBAR_3" },
    { interruptSignal: "PRU_ICSSM0_PR1_SLV_IN_20", interruptId: "20", source: "CONTROL_SS_OUTPUT_XBAR_4" },
    { interruptSignal: "PRU_ICSSM0_PR1_SLV_IN_21", interruptId: "21", source: "CONTROL_SS_OUTPUT_XBAR_5" },
    { interruptSignal: "PRU_ICSSM0_PR1_SLV_IN_22", interruptId: "22", source: "CONTROL_SS_OUTPUT_XBAR_6" },
    { interruptSignal: "PRU_ICSSM0_PR1_SLV_IN_23", interruptId: "23", source: "CONTROL_SS_OUTPUT_XBAR_7" },
    { interruptSignal: "PRU_ICSSM0_PR1_SLV_IN_24",  interruptId: "24",  source: "CONTROL_SS_OUTPUT_XBAR_8" },
    { interruptSignal: "PRU_ICSSM0_PR1_SLV_IN_25",  interruptId: "25",  source: "CONTROL_SS_OUTPUT_XBAR_9" },
    { interruptSignal: "PRU_ICSSM0_PR1_SLV_IN_26",  interruptId: "26",  source: "CONTROL_SS_OUTPUT_XBAR_10" },
    { interruptSignal: "PRU_ICSSM0_PR1_SLV_IN_27",  interruptId: "27",  source: "CONTROL_SS_OUTPUT_XBAR_11"},
    { interruptSignal: "PRU_ICSSM0_PR1_SLV_IN_28",  interruptId: "28",  source: "CONTROL_SS_OUTPUT_XBAR_12"},
    { interruptSignal: "PRU_ICSSM0_PR1_SLV_IN_29",  interruptId: "29",  source: "CONTROL_SS_OUTPUT_XBAR_13"},
    { interruptSignal: "PRU_ICSSM0_PR1_SLV_IN_30",  interruptId: "30",  source: "CONTROL_SS_OUTPUT_XBAR_14"},
    { interruptSignal: "PRU_ICSSM0_PR1_SLV_IN_31",  interruptId: "31",  source: "CONTROL_SS_OUTPUT_XBAR_15"},
];

let ICSSM0_MODE1_INTC_INTERNAL_SIGNALS = [
    { eventNumber: "55", interruptSignal: "pr0_mii1_col and pr0_mii1_txen (external)", },
    { eventNumber: "54", interruptSignal: "PRU1_RX_EOF", },
    { eventNumber: "53", interruptSignal: "MDIO_MII_LINK[1]", },
    { eventNumber: "52", interruptSignal: "PORT1_TX_OVERFLOW", },
    { eventNumber: "51", interruptSignal: "PORT1_TX_UNDERFLOW", },
    { eventNumber: "50", interruptSignal: "PRU1_RX_OVERFLOW", },
    { eventNumber: "49", interruptSignal: "PRU1_RX_NIBBLE_ODD", },
    { eventNumber: "48", interruptSignal: "PRU1_RX_CRC", },
    { eventNumber: "47", interruptSignal: "PRU1_RX_SOF", },
    { eventNumber: "46", interruptSignal: "PRU1_RX_SFD", },
    { eventNumber: "45", interruptSignal: "PRU1_RX_ERR32", },
    { eventNumber: "44", interruptSignal: "PRU1_RX_ERR", },
    { eventNumber: "43", interruptSignal: "pr0_mii0_col and pr0_mii0_txen (external)", },
    { eventNumber: "42", interruptSignal: "PRU0_RX_EOF", },
    { eventNumber: "41", interruptSignal: "MDIO_MII_LINK[0]", },
    { eventNumber: "40", interruptSignal: "PORT0_TX_OVERFLOW", },
    { eventNumber: "39", interruptSignal: "PORT0_TX_UNDERFLOW", },
    { eventNumber: "38", interruptSignal: "PRU0_RX_OVERFLOW", },
    { eventNumber: "37", interruptSignal: "PRU0_RX_NIBBLE_ODD", },
    { eventNumber: "36", interruptSignal: "PRU0_RX_CRC", },
    { eventNumber: "35", interruptSignal: "PRU0_RX_SOF", },
    { eventNumber: "34", interruptSignal: "PRU0_RX_SFD", },
    { eventNumber: "33", interruptSignal: "PRU0_RX_ERR32", source: "PRU_ICSSM0 PRU0" },
    { eventNumber: "32", interruptSignal: "PRU0_RX_ERR", source: "PRU_ICSSM0 PRU0" },
    { eventNumber: "31", interruptSignal: "pr0_pru_mst_intr[15]_intr_req", source: "PRU_ICSSM0 PRU0, PRU_ICSSM0 PRU1" },
    { eventNumber: "30", interruptSignal: "pr0_pru_mst_intr[14]_intr_req", source: "PRU_ICSSM0 PRU0, PRU_ICSSM0 PRU1" },
    { eventNumber: "29", interruptSignal: "pr0_pru_mst_intr[13]_intr_req", source: "PRU_ICSSM0 PRU0, PRU_ICSSM0 PRU1" },
    { eventNumber: "28", interruptSignal: "pr0_pru_mst_intr[12]_intr_req", source: "PRU_ICSSM0 PRU0, PRU_ICSSM0 PRU1" },
    { eventNumber: "27", interruptSignal: "pr0_pru_mst_intr[11]_intr_req", source: "PRU_ICSSM0 PRU0, PRU_ICSSM0 PRU1" },
    { eventNumber: "26", interruptSignal: "pr0_pru_mst_intr[10]_intr_req", source: "PRU_ICSSM0 PRU0, PRU_ICSSM0 PRU1" },
    { eventNumber: "25", interruptSignal: "pr0_pru_mst_intr[9]_intr_req", source: "PRU_ICSSM0 PRU0, PRU_ICSSM0 PRU1" },
    { eventNumber: "24", interruptSignal: "pr0_pru_mst_intr[8]_intr_req", source: "PRU_ICSSM0 PRU0, PRU_ICSSM0 PRU1" },
    { eventNumber: "23", interruptSignal: "pr0_pru_mst_intr[7]_intr_req", source: "PRU_ICSSM0 PRU0, PRU_ICSSM0 PRU1" },
    { eventNumber: "22", interruptSignal: "pr0_pru_mst_intr[6]_intr_req", source: "PRU_ICSSM0 PRU0, PRU_ICSSM0 PRU1" },
    { eventNumber: "21", interruptSignal: "pr0_pru_mst_intr[5]_intr_req", source: "PRU_ICSSM0 PRU0, PRU_ICSSM0 PRU1" },
    { eventNumber: "20", interruptSignal: "pr0_pru_mst_intr[4]_intr_req", source: "PRU_ICSSM0 PRU0, PRU_ICSSM0 PRU1" },
    { eventNumber: "19", interruptSignal: "pr0_pru_mst_intr[3]_intr_req", source: "PRU_ICSSM0 PRU0, PRU_ICSSM0 PRU1" },
    { eventNumber: "18", interruptSignal: "pr0_pru_mst_intr[2]_intr_req", source: "PRU_ICSSM0 PRU0, PRU_ICSSM0 PRU1" },
    { eventNumber: "17", interruptSignal: "pr0_pru_mst_intr[1]_intr_req", source: "PRU_ICSSM0 PRU0, PRU_ICSSM0 PRU1" },
    { eventNumber: "16", interruptSignal: "pr0_pru_mst_intr[0]_intr_req", source: "PRU_ICSSM0 PRU0, PRU_ICSSM0 PRU1" },
    { eventNumber: "15", interruptSignal: "pr0_ecap_intr_req", source: "PRU_ICSSM0 ECAP0" },
    { eventNumber: "14", interruptSignal: "pr0_sync0_out_pend", source: "PRU_ICSSM0 IEP0" },
    { eventNumber: "13", interruptSignal: "pr0_sync1_out_pend", source: "PRU_ICSSM0 IEP0" },
    { eventNumber: "12", interruptSignal: "pr0_latch0_in (input to PRU_ICSSM0)", source: "PRU_ICSSM0 IEP0" },
    { eventNumber: "11", interruptSignal: "pr0_latch1_in (input to PRU_ICSSM0)", source: "PRU_ICSSM0 IEP0" },
    { eventNumber: "10", interruptSignal: "pr0_pdi_wd_exp_pend", source: "PRU_ICSSM0 IEP0" },
    { eventNumber: "9",  interruptSignal: "pr0_pd_wd_exp_pend", source: "PRU_ICSSM0 IEP0" },
    { eventNumber: "8",  interruptSignal: "pr0_digio_event_req", source: "PRU_ICSSM0 IEP0" },
    { eventNumber: "7",  interruptSignal: "pr0_iep_tim_cap_cmp_pend", source: "PRU_ICSSM0 IEP0" },
    { eventNumber: "6",  interruptSignal: "pr0_uart0_uint_intr_req", source: "PRU_ICSSM0 UART0" },
    { eventNumber: "5",  interruptSignal: "pr0_uart0_utxevt_intr_req", source: "PRU_ICSSM0 UART0" },
    { eventNumber: "4",  interruptSignal: "pr0_uart0_urxevt_intr_req", source: "PRU_ICSSM0 UART0" },
    { eventNumber: "3",  interruptSignal: "pr0_rst_reset_iso_req", source: "PRU_ICSSM0 Reset Isolation Requested" },
    { eventNumber: "2",  interruptSignal: "pr0_pru1_r31_status_cnt16", source: "PRU_ICSSM0 PRU1 (Shift Capture)" },
    { eventNumber: "1",  interruptSignal: "pr0_pru0_r31_status_cnt16", source: "PRU_ICSSM0 PRU0 (Shift Capture)" },
    { eventNumber: "0",  interruptSignal: "pr0_ecc_err_intr", source: "PRU_ICSSM0 ECC Logic" },
];

ICSSM0_MODE1_INTC_INTERNAL_SIGNALS.reverse();

let ICSSM0_MODE1_INTC_EXTERNAL_SIGNALS = [
    //Event_no: 56 starts
    { interruptSignal: "PRU_ICSSM0_PR1_SLV_IN_24",  interruptId: "24",  source: "CONTROL_SS_OUTPUT_XBAR_8" },
    { interruptSignal: "PRU_ICSSM0_PR1_SLV_IN_25",  interruptId: "25",  source: "CONTROL_SS_OUTPUT_XBAR_9" },
    { interruptSignal: "PRU_ICSSM0_PR1_SLV_IN_26",  interruptId: "26",  source: "CONTROL_SS_OUTPUT_XBAR_10" },
    { interruptSignal: "PRU_ICSSM0_PR1_SLV_IN_27",  interruptId: "27",  source: "CONTROL_SS_OUTPUT_XBAR_11"},
    { interruptSignal: "PRU_ICSSM0_PR1_SLV_IN_28",  interruptId: "28",  source: "CONTROL_SS_OUTPUT_XBAR_12"},
    { interruptSignal: "PRU_ICSSM0_PR1_SLV_IN_29",  interruptId: "29",  source: "CONTROL_SS_OUTPUT_XBAR_13"},
    { interruptSignal: "PRU_ICSSM0_PR1_SLV_IN_30",  interruptId: "30",  source: "CONTROL_SS_OUTPUT_XBAR_14"},
    { interruptSignal: "PRU_ICSSM0_PR1_SLV_IN_31",  interruptId: "31",  source: "CONTROL_SS_OUTPUT_XBAR_15"},
];



/* add evenNumber property for external events */
let i = ICSSM0_MODE0_INTC_INTERNAL_SIGNALS.length; // = 32
for (let option of ICSSM0_MODE0_INTC_EXTERNAL_SIGNALS) {
    option.eventNumber = `${i + parseInt(option.interruptId, 10)}`;
    // option.eventNumber = `${i}`;
    // i++;
}


/* add evenNumber property for external events */
i = ICSSM0_MODE1_INTC_INTERNAL_SIGNALS.length; // = 55
for (let option of ICSSM0_MODE1_INTC_EXTERNAL_SIGNALS) {
    option.eventNumber = `${i}`;
    i++;
}


let ICSSM0_MODE0_INTC_SIGNALS = ICSSM0_MODE0_INTC_INTERNAL_SIGNALS.concat(ICSSM0_MODE0_INTC_EXTERNAL_SIGNALS);
let ICSSM0_MODE1_INTC_SIGNALS = ICSSM0_MODE1_INTC_INTERNAL_SIGNALS.concat(ICSSM0_MODE1_INTC_EXTERNAL_SIGNALS);

let ICSSM_INTC_CHANNELS   = [...Array(10).keys()].map(String);     /* 0 to 9 */
let ICSSM_INTC_HOST_INTRS = [...Array(10).keys()].map(String);     /* 0 to 9 */

/**
 *
 * @param {*} instance
 * @param {*} type
 * @returns options array
 */
function getEventConfigOptions (instance, type = "all") {
    let intc_signals;
    if (type === "all")
        intc_signals = instance === "ICSSM0_MODE0" ? ICSSM0_MODE0_INTC_SIGNALS : ICSSM0_MODE1_INTC_SIGNALS;
    if (type === "internal")
        intc_signals = instance === "ICSSM0_M0DE0" ? ICSSM0_MODE0_INTC_INTERNAL_SIGNALS : ICSSM0_MODE1_INTC_INTERNAL_SIGNALS;
    if (type === "external")
        intc_signals = instance === "ICSSM0_MODE0" ? ICSSM0_MODE0_INTC_EXTERNAL_SIGNALS : ICSSM0_MODE1_INTC_EXTERNAL_SIGNALS;
    let options = [];
    for (let signal of intc_signals) {
        let description = `Event no.: ${signal.eventNumber}`;
        if (signal.source)  description += `, Source: ${signal.source}`
        let option = {
            name: signal.eventNumber,
            displayName: signal.eventNumber + ": " + signal.interruptSignal,
            description,
        }
        options.push(option);
    }
    return options;
}

function getChannelConfigOptions () {
    let options = [];
    for (let channel of ICSSM_INTC_CHANNELS) {
        let description = `Channel no.: ${channel}`;
        let option = {
            name: channel,
            displayName: `Channel ${channel}`,
            description,
        }
        options.push(option);
    }
    return options;
}

function getHostConfigOptions () {
    let options = [];
    for (let hostInterrupt of ICSSM_INTC_HOST_INTRS) {
        let description = `Host Interrupt no.: ${hostInterrupt}`;
        let option = {
            name: hostInterrupt,
            displayName: `Host Interrupt ${hostInterrupt}`,
            description,
        }
        options.push(option);
    }
    return options;
}

function getDisabledOptionsSubset (allOptions, filter, reason) {
    let disabledOptions = [];
    allOptions.forEach(option => {
        if (option.displayName.includes(filter))
            disabledOptions.push({
                name: option.name,
                reason,
            });
    });
    return disabledOptions;
}

function getDisabledOptionsSubsetNot (allOptions, filter, reason) {
    let disabledOptions = [];
    allOptions.forEach(option => {
        if (!option.displayName.includes(filter))
            disabledOptions.push({
                name: option.name,
                reason,
            });
    });
    return disabledOptions;
}

function getDisabledOptionsMtoN (allOptions, m, n, reason) {
    let disabledOptions = [];
    if (m > n) return [];
    for (let i = m; i <= n; i++) {
        disabledOptions.push({
            name: allOptions[i].name,
            reason,
        });
    }
    return disabledOptions;
}

exports = {
    getEventConfigOptions,
    getChannelConfigOptions,
    getHostConfigOptions,
    getDisabledOptionsSubset,
    getDisabledOptionsSubsetNot,
    getDisabledOptionsMtoN,
};
