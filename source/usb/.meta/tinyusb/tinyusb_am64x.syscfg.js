
const tinyusb_config = [
    {
        name        : "USB0",
        clockIds    : [ "TISCI_DEV_USB0" ],
    },
];

function getConfigArr() {
    return tinyusb_config;
}

exports = {
    getConfigArr,
};
