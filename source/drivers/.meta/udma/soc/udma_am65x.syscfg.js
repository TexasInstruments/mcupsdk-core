
const udma_config = [
    {
        name: "MCU_0",
        type: "UDMA",
        numBlkCopyCh: 28,
    },
];

function getConfigArr() {
    return udma_config;
}

exports = {
    getConfigArr,
};
