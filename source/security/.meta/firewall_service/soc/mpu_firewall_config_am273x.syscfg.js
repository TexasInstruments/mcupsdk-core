const mpu_firewall_config = [
    {
        name: "L2_BANKA",
        regionCount: 8,
        memSpace : [
            { startAddr: 0xC0200000, size: 0x80000 },
        ]
    },
    {
        name: "L2_BANKB",
        regionCount: 8,
        memSpace : [
            { startAddr: 0xC0280000, size: 0x70000 },
        ]
    },
    {
        name: "MSS_MBOX",
        regionCount: 8,
        memSpace : [
            { startAddr: 0xC5000000, size: 0x2000 },
        ]
    },
    {
        name: "MSS_PCRA",
        regionCount: 8,
        memSpace : [
            { startAddr: 0x02000000, size: 0x1000000 },
        ]
    },
    {
        name: "QSPI0",
        regionCount: 8,
        memSpace : [
            { startAddr: 0xC8000000, size: 0x40000 },
            { startAddr: 0xC6000000, size: 0x2000000 },
        ]
    },
    {
        name: "R5SS_COREA_AXIS",
        regionCount: 8,
        memSpace : [
            { startAddr: 0xC1000000, size: 0x10000 },
            { startAddr: 0xC1800000, size: 0x10000 },
            { startAddr: 0xC2000000, size: 0x4000 },
            { startAddr: 0xC2800000, size: 0x4000 },
        ]
    },
    {
        name: "R5SS_COREB_AXIS",
        regionCount: 8,
        memSpace : [
            { startAddr: 0xC3000000, size: 0x10000 },
            { startAddr: 0xC3800000, size: 0x10000 },
            { startAddr: 0xC4000000, size: 0x4000 },
            { startAddr: 0xC4800000, size: 0x4000 },
        ]
    },
    {
        name: "L3_BANKA",
        regionCount: 8,
        memSpace : [
            { startAddr: 0x88000000, size: 0x100000 },
        ]
    },
    {
        name: "L3_BANKB",
        regionCount: 8,
        memSpace : [
            { startAddr: 0x88100000, size: 0x100000 },
        ]
    },
    {
        name: "L3_BANKC",
        regionCount: 8,
        memSpace : [
            { startAddr: 0x88200000, size: 0x100000 },
        ]
    },
    {
        name: "L3_BANKD",
        regionCount: 8,
        memSpace : [
            { startAddr: 0x88300000, size: 0x90000  },
        ]
    },
    {
        name: "HWA_DMA0",
        regionCount: 8,
        memSpace : [
            { startAddr: 0x82000000, size: 0x20000 },
        ]
    },
    {
        name: "HWA_DMA1",
        regionCount: 8,
        memSpace : [
            { startAddr: 0x82100000, size: 0x20000 },
        ]
    },
    {
        name: "DSS_HWA_PROC",
        regionCount: 8,
        memSpace : [
            { startAddr: 0x48000000, size: 0x1000 },
            { startAddr: 0x48020000, size: 0x1000 },
        ]
    },
    {
        name: "DSS_MBOX",
        regionCount: 8,
        memSpace : [
            { startAddr: 0x83100000, size: 0x1000 },
        ]
    },
];


const id_list = [
	{ name: "R5FSS", displayName:"R5FSS0_0/R5FSS0_1/CPSW" },
	{ name: "R5FSSI", displayName:"R5FSSI" },
    { name: "DSSTPTC", displayName:"DSS" },
    { name: "AIDX", displayName:"EXTERNAL ID" },
]

const default_id_list = ["R5FSS", "R5FSSI", "DSSTPTC"]

function getConfigArr() {
    return mpu_firewall_config;
}

function getAidList() {
    return id_list;
}

function getdefaultAidList() {
    return default_id_list;
}

exports = {
    getConfigArr,
    getAidList,
    getdefaultAidList,
};
