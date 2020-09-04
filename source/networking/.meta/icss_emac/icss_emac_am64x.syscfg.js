
let common = system.getScript("/common");

let icss_instances = [
    {
        name: "ICSSG0",
    },
    {
        name: "ICSSG1",
    }
];

let phyToMacModes_instances = [
    {
        name: "MII",
    },
    {
        name: "RGMII",
    }
];

let INTERRUPT_OFFSET_ICSS = {
    "ICSSG0": 120,
    "ICSSG1": 248,
};

let PORT_INT_OFFSET = {
    "rxIntNum" : [0, 1],
    "txIntNum" : [2, 3],
    "linkIntNum" : [6, 7],
};

const staticConfig =
{
    emacMode: [ "ICSS_EMAC_MODE_SWITCH", "0" ],
    phyAddr: [15, 3],
    phyToMacInterfaceMode: "ICSS_EMAC_MII_MODE",
    halfDuplexEnable: false,
    enableIntrPacing: "ICSS_EMAC_DISABLE_PACING",
    intrPacingMode: "ICSS_EMAC_INTR_PACING_MODE1",
    pacingThreshold: 0,
    ethPrioQueue: "ICSS_EMAC_QUEUE3",
    learningEnable: false,
    portMask: "ICSS_EMAC_MODE_SWITCH",
    txInterruptEnable: false,
    linkIntNum: INTERRUPT_OFFSET_ICSS["ICSSG0"] + PORT_INT_OFFSET["linkIntNum"][0],
    rxIntNum: INTERRUPT_OFFSET_ICSS["ICSSG0"] + PORT_INT_OFFSET["rxIntNum"][0],
    txIntNum: INTERRUPT_OFFSET_ICSS["ICSSG0"] + PORT_INT_OFFSET["txIntNum"][0],
    pktBufSize: 0x10000,
    txTaskPriority: 10,
    rxTaskPriority: 10,
    linkTaskPriority: 10,
    pktBufIndex: 0,
    splitQueue: 0,
};

function getIcssInstancesArr()
{
    return icss_instances;
}

function getPhyToMacInterfacesArr()
{
    return phyToMacModes_instances;
}

function getStaticConfig() {
    return _.cloneDeep(staticConfig);
}

function updateConfig(inst, config)
{
    let port = 0;

    switch(inst.mode)
    {
        case "SWITCH":
            config.emacMode[0] = "ICSS_EMAC_MODE_SWITCH";
            config.emacMode[1] = "0";
            config.portMask = "ICSS_EMAC_MODE_SWITCH";
            port = 0;
            break;

        case "DUAL MAC | MAC1":
            config.emacMode[0] = "ICSS_EMAC_MODE_MAC1";
            config.emacMode[1] = "ICSS_EMAC_MODE_DUALMAC";
            config.portMask = "ICSS_EMAC_MODE_MAC1";
            port = 0;
            break;

        case "MAC1":
            config.emacMode[0] = "ICSS_EMAC_MODE_MAC1";
            config.emacMode[1] = "0";
            config.portMask = "ICSS_EMAC_MODE_MAC1";
            port = 0;
            break;

        case "DUAL MAC | MAC2":
            config.emacMode[0] = "ICSS_EMAC_MODE_MAC2";
            config.emacMode[1] = "ICSS_EMAC_MODE_DUALMAC";
            config.portMask = "ICSS_EMAC_MODE_MAC2";
            port = 1;
            break;

        case "MAC2":
            config.emacMode[0] = "ICSS_EMAC_MODE_MAC2";
            config.emacMode[1] = "0";
            config.portMask = "ICSS_EMAC_MODE_MAC2";
            port = 1;
            break;
    }

    switch(inst.instance)
    {
        case "ICSSG0":
            config.pktBufIndex = 0;
            break;
        case "ICSSG1":
            config.pktBufIndex = 1;
            break;
    }
    config.linkIntNum = INTERRUPT_OFFSET_ICSS[inst.instance] + PORT_INT_OFFSET["linkIntNum"][port];
    config.rxIntNum = INTERRUPT_OFFSET_ICSS[inst.instance] + PORT_INT_OFFSET["rxIntNum"][port];
    config.txIntNum = INTERRUPT_OFFSET_ICSS[inst.instance] + PORT_INT_OFFSET["txIntNum"][port];
    config.phyAddr[0] = inst.phyAddr0;
    config.phyAddr[1] = inst.phyAddr1;
    config.ethPrioQueue = `ICSS_EMAC_${inst.queue}`;
    config.pktBufSize = inst.pktBufSizeKB*1024;
}

function getPktBufSizes(module)
{
    let pktBufSizes= [ 0, 0 ];

    for(let i = 0; i < module.$instances.length; i++)
    {
        let instance = module.$instances[i];
        let config = module.getInstanceConfig(instance);

        switch(config.instance)
        {
            case "ICSSG0":
            if(config.pktBufSize > pktBufSizes[0])
            {
                pktBufSizes[0] = config.pktBufSize;
            }
            break;
            case "ICSSG1":
            if(config.pktBufSize > pktBufSizes[1])
            {
                pktBufSizes[1] = config.pktBufSize;
            }
            break;
        }
    }

    return pktBufSizes;
}

exports = {
    getStaticConfig,
    updateConfig,
    getIcssInstancesArr,
    getPhyToMacInterfacesArr,
    getPktBufSizes,
};
