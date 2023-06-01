let CMPSS_STS = [
	{ name: "CMPSS_STS_HI_FILTOUT", displayName: "High digital filter output" },
	{ name: "CMPSS_STS_HI_LATCHFILTOUT", displayName: "Latched value of high digital filter output" },
	{ name: "CMPSS_STS_LO_FILTOUT", displayName: "Low digital filter output" },
	{ name: "CMPSS_STS_LO_LATCHFILTOUT", displayName: "Latched value of low digital filter output" },
]
function getInterfaceName(instance) {
    return "CMPSS";
}
exports = {
	CMPSS_STS: CMPSS_STS,
    getInterfaceName,
}
