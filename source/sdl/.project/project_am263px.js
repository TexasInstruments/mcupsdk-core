let path = require('path');

let device = "am263px";

const files_r5f = {
    common: [
        "sdl_dpl.c",
		"sdl_ip_esm.c",
        "sdl_esm.c",
        "sdl_esm_core.c",
        "sdl_esm_priv.c",
	    "sdl_dcc.c",
	    "sdl_mcrc.c",
	    "sdl_ip_mcrc.c",
	    "sdl_mcrc_soc.c",
	    "sdl_rti.c",
	    "sdl_ip_rti.c",
	    "sdl_soc_rti.c",
		"sdl_ecc.c",
		"sdl_ip_ecc.c",
		"sdl_ecc_r5.c",
		"sdl_interrupt.c",
		"sdl_interrupt_handlers.c",
		"sdl_interrupt_register.c",
		"sdl_exception.c",
        "sdl_ip_pbist.c",
        "sdl_pbist_soc.c",
        "sdl_pbist.c",
        "sdl_ecc_bus_safety.c",
        "sdl_ccm.c",
        "sdl_mcu_armss_ccmr5.c",
        "sdl_stc_soc.c",

    ],
};

const files_r5fss1 = {
    common: [
      "sdl_dpl.c",
      "sdl_ip_pbist.c",
      "sdl_pbist_soc.c",
      "sdl_pbist.c",
    ],
  };

const asmfiles_r5f = {
    common: [
		"sdl_ecc_utils.S",
		"sdl_r5_utils.S",
	],
};

const filedirs = {
    common: [
        "dpl",
		"esm",
        "esm/v0",
        "esm/v0/v0_0",
        "esm/soc",
        "esm/soc/am263px",
        "dcc/v1",
        "mcrc",
        "mcrc/v0",
        "mcrc/v0/soc/am263px",
        "rti",
        "rti/v0",
        "rti/v0/soc/am263px",
        "ecc",
        "ecc/soc/am263px",
        "ecc/V1",
        "r5",
        "r5/v0",
        "r5/v0/am263px",
        "pbist",
        "pbist/v0",
        "pbist/v0/soc",
        "pbist/v0/soc/am263px",
        "ecc_bus_safety",
        "ecc_bus_safety/v0",
        "ecc_bus_safety/v0/soc",
        "ecc_bus_safety/v0/soc/am263px",
        "stc/v0",
        "stc/v0/soc/am263px",
    ],
};

const defines_r5f = {
    common: [
        "SUBSYS_R5SS0",
        "SUBSYS_MSS",
    ],
};

const defines_r5fss1 = {
    common: [
        "SUBSYS_R5SS1",
    ],
};

const cflags_r5f = {
    common: [
        "-Wno-extra",
    ],
};

const buildOptionCombos = [
    { device: device, cpu: "r5f", cgt: "ti-arm-clang"},
    { device: device, cpu: "r5fss1", cgt: "ti-arm-clang"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "library";
    property.name = "sdl";
    property.isInternal = false;
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.filedirs = filedirs;
    if(buildOption.cpu.match(/r5f*/)) {
        build_property.files = files_r5f;
        build_property.asmfiles = asmfiles_r5f;
        build_property.cflags = cflags_r5f;
      build_property.defines = defines_r5f;
    }

    if(buildOption.cpu.match(/r5fss1*/)){
        build_property.files = files_r5fss1;
        build_property.defines = defines_r5fss1;
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,

};
