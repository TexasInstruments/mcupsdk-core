let DAC_ReferenceVoltage = [
	{ name: "DAC_REF_VREF", displayName: "VREF reference voltag (External)" },
	{ name: "DAC_REF_VDDA", displayName: "VDDA reference voltage (Internal)" },
]
let DAC_LoadMode = [
	{ name: "DAC_LOAD_SYSCLK", displayName: "Load on next SYSCLK" },
	{ name: "DAC_LOAD_PWMSYNC", displayName: "Load on next PWMSYNC specified by SYNCSEL" },
]
let DAC_LOCK = [
	{ name: "DAC_LOCK_CONTROL", displayName: "Lock the control register" },
	{ name: "DAC_LOCK_SHADOW", displayName: "Lock the shadow value register" },
	{ name: "DAC_LOCK_OUTPUT", displayName: "Lock the output enable register" },
]
function getInterfaceName(instance) {
    return "DAC";
}
exports = {
	DAC_ReferenceVoltage: DAC_ReferenceVoltage,
	DAC_LoadMode: DAC_LoadMode,
	DAC_LOCK: DAC_LOCK,
    getInterfaceName,
}
