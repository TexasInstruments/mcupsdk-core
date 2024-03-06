const mpu_firewall_config = [
    {
        name: "L2OCRAM_BANK0_SLV",
        regionCount: 8,
        memSpace : [
            { startAddr: 0x70000000, size: 0x80000 },
        ]
    },
    {
        name: "L2OCRAM_BANK1_SLV",
        regionCount: 8,
        memSpace : [
            { startAddr: 0x70080000, size: 0x80000 },
        ]
    },
    {
        name: "L2OCRAM_BANK2_SLV",
        regionCount: 8,
        memSpace : [
            { startAddr: 0x70100000, size: 0x80000 },
        ]
    },
    {
        name: "L2OCRAM_BANK3_SLV",
        regionCount: 8,
        memSpace : [
            { startAddr: 0x70180000, size: 0x80000 },
        ]
    },
    {
        name: "L2OCRAM_BANK4_SLV",
        regionCount: 8,
        memSpace : [
            { startAddr: 0x70200000, size: 0x80000 },
        ]
    },
    {
        name: "L2OCRAM_BANK5_SLV",
        regionCount: 8,
        memSpace : [
            { startAddr: 0x70280000, size: 0x80000 },
        ]
    },
    {
        name: "R5SS0_CORE0_AXIS_SLV",
        regionCount: 8,
        memSpace : [
            { startAddr: 0x78000000, size: 0x10000 },
            { startAddr: 0x78060000, size: 0x400 },
            { startAddr: 0x78100000, size: 0x10000 },
            { startAddr: 0x74000000, size: 0x800000 },
            { startAddr: 0x74800000, size: 0x800000 },
        ]
    },
    {
        name: "R5SS0_CORE1_AXIS_SLV",
        regionCount: 8,
        memSpace : [
            { startAddr: 0x78200000, size: 0x8000 },
            { startAddr: 0x78260000, size: 0x400 },
            { startAddr: 0x78300000, size: 0x8000 },
            { startAddr: 0x75000000, size: 0x800000 },
            { startAddr: 0x75800000, size: 0x800000 },
        ]
    },
    {
        name: "R5SS1_CORE0_AXIS_SLV",
        regionCount: 8,
        memSpace : [
            { startAddr: 0x78400000, size: 0x10000 },
            { startAddr: 0x78460000, size: 0x400 },
            { startAddr: 0x78500000, size: 0x10000 },
            { startAddr: 0x76000000, size: 0x800000 },
            { startAddr: 0x76800000, size: 0x800000 },
        ]
    },
    {
        name: "R5SS1_CORE1_AXIS_SLV",
        regionCount: 8,
        memSpace : [
            { startAddr: 0x78600000, size: 0x8000 },
            { startAddr: 0x78660000, size: 0x400 },
            { startAddr: 0x78700000, size: 0x8000 },
            { startAddr: 0x77000000, size: 0x800000 },
            { startAddr: 0x77800000, size: 0x800000 },
        ]
    },
    {
        name: "MBOX_RAM_SLV",
        regionCount: 8,
        memSpace : [
            { startAddr: 0x72000000, size: 0x4000 },
        ]
    },
    {
        name: "OSPI0_SLV",
        regionCount: 8,
        memSpace : [
            { startAddr: 0x60000000, size: 0x8000000 },
            { startAddr: 0x80000000, size: 0x8000000 },
            { startAddr: 0x88000000, size: 0x8000000 },
        ]
    },
    {
        name: "OSPI0_CFG_SLV",
        regionCount: 4,
        memSpace : [
            { startAddr: 0x53800000, size: 0x10000 },
        ]
    },
    {
        name: "SCRM2SCRP0_SLV",
        regionCount: 16,
        memSpace : [
            { startAddr: 0x50000000, size: 0x10000000 },
        ]
    },
    {
        name: "SCRM2SCRP1_SLV",
        regionCount: 16,
        memSpace : [
            { startAddr: 0x50000000, size: 0x10000000 },
        ]
    },
    {
        name: "R5SS0_CORE0_AHB_MST",
        regionCount: 16,
        memSpace : [
            { startAddr: 0x50000000, size: 0x10000000 },
        ]
    },
    {
        name: "R5SS0_CORE1_AHB_MST",
        regionCount: 16,
        memSpace : [
            { startAddr: 0x50000000, size: 0x10000000 },
        ]
    },
    {
        name: "R5SS1_CORE0_AHB_MST",
        regionCount: 16,
        memSpace : [
            { startAddr: 0x50000000, size: 0x10000000 },
        ]
    },
    {
        name: "R5SS1_CORE1_AHB_MST",
        regionCount: 16,
        memSpace : [
            { startAddr: 0x50000000, size: 0x10000000 },
        ]
    },
    {
        name: "R5SS0_SLV",
        regionCount: 4,
        memSpace : [
            { startAddr: 0x53000000, size: 0x500400 },
        ]
    },
    {
        name: "R5SS1_SLV",
        regionCount: 4,
        memSpace : [
            { startAddr: 0x53004000, size: 0x50C400 },
        ]
    },
];


const id_list = [
	{ name: "R5FSS0_0", displayName:"R5FSS0_0" },
	{ name: "R5FSS0_1", displayName:"R5FSS0_1" },
    { name: "R5FSS1_0", displayName:"R5FSS1_0" },
	{ name: "R5FSS1_1", displayName:"R5FSS1_1" },
    { name: "ICSSM", displayName:"ICSSM" },
	{ name: "CPSW", displayName:"CPSW" },
    { name: "AIDX", displayName:"EXTERNAL ID" },
]

const default_id_list = ["R5FSS0_0", "R5FSS0_1", "R5FSS1_0", "R5FSS1_1"]

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