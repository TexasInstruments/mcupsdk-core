let path = require('path');

let device = "am64x";

const files_m4f = {
    common: [
        "sdl_dpl.c",
        "sdl_mcrc.c",
        "sdl_ip_mcrc.c",
        "sdl_mcrc_soc.c",
        "sdl_esm.c",
        "sdl_ip_esm.c",
        "sdl_esm_core.c",
        "sdl_esm_priv.c",
        "sdl_dcc.c",
        "sdl_soc_dcc.c",
        "sdl_ip_tog.c",
        "sdl_tog.c",
        "sdl_soc_tog.c",
        "sdl_ip_vtm.c",
		"sdl_vtm_pvt_sensor.c",
		"sdl_vtm.c",
		"sdl_soc_vtm.c",
        "sdl_pok.c",
		"sdl_ip_pok.c",
		"sdl_soc_pok.c",
		"sdl_ip_pok_defs.c",
		"sdl_ip_mtog.c",
		"sdl_mtog.c",
		"sdl_soc_mtog.c",
    "sdl_ecc.c",
    "sdl_ip_ecc.c",
    "sdl_rti.c",
    "sdl_ip_rti.c",
	"sdl_soc_rti.c",

    ],
};

const files_r5f = {
    common: [
      "sdl_dpl.c",
        "sdl_mcrc.c",
        "sdl_ip_mcrc.c",
        "sdl_mcrc_soc.c",
        "sdl_esm.c",
        "sdl_ip_esm.c",
        "sdl_esm_core.c",
        "sdl_esm_priv.c",
        "sdl_dcc.c",
        "sdl_soc_dcc.c",
        "sdl_ip_tog.c",
        "sdl_tog.c",
        "sdl_soc_tog.c",
        "sdl_ip_vtm.c",
    	"sdl_vtm_pvt_sensor.c",
    	"sdl_vtm.c",
    	"sdl_soc_vtm.c",
		"sdl_pok.c",
		"sdl_ip_pok.c",
		"sdl_soc_pok.c",
		"sdl_ip_pok_defs.c",

      "sdl_ecc.c",
      "sdl_ip_ecc.c",
      "sdl_ecc_r5.c",
  		"sdl_interrupt.c",
  		"sdl_interrupt_handlers.c",
  		"sdl_interrupt_register.c",
  		"sdl_exception.c",
    ],
};

const filedirs_m4f = {
    common: [
      "dpl",
      "mcrc",
      "mcrc/v0",
      "mcrc/v0/soc/am64x",
      "esm",
      "esm/soc",
      "esm/soc/am64x",
      "esm/v0",
      "esm/v0/v0_0",
      "dcc",
      "dcc/v0",
      "dcc/v0/soc",
      "dcc/v0/soc/am64x",
      "stog/v0",
      "stog/v0/soc",
      "stog/v0/soc/am64x",
      "vtm",
      "vtm/v0",
      "vtm/v0/soc/am64x",
      "pok",
    "pok/v1",
  "pok/v1/soc",
    "pok/v1/soc/am64x",
  "mtog",
  "mtog/v0",
  "mtog/soc/am64x",
        "ecc",
        "ecc/soc/am64x_am243x",
        "ecc/V0",
        "rti",
        "rti/v0",
		"rti/v0/soc/am64x",

    ],
};

const filedirs_r5f = {
    common: [
      "dpl",
      "mcrc",
      "mcrc/v0",
      "mcrc/v0/soc/am64x",
      "esm",
      "esm/soc",
      "esm/soc/am64x",
      "esm/v0",
      "esm/v0/v0_0",
      "dcc",
      "dcc/v0",
      "dcc/v0/soc",
      "dcc/v0/soc/am64x",
      "stog/v0",
      "stog/v0/soc",
      "stog/v0/soc/am64x",
      "vtm",
      "vtm/v0",
      "vtm/v0/soc/am64x",
      "pok",
    "pok/v1",
  "pok/v1/soc",
    "pok/v1/soc/am64x",
  "mtog",
  "mtog/v0",
  "mtog/soc/am64x",
        "ecc",
        "ecc/soc/am64x_am243x",
        "ecc/V0",
        "r5",
        "r5/v0",
    ],
};

const asmfiles_r5f = {
    common: [
		"sdl_ecc_utils.S",
		"sdl_r5_utils.S",
	],
};

const r5_macro = {
    common: [
        "R5F_CORE",
    ],

};

const m4_macro = {
    common: [
        "M4F_CORE",
    ],

};

const cflags_r5f = {
common: [
"-Wno-extra",
],
};

const buildOptionCombos = [
    { device: device, cpu: "m4f", cgt: "ti-arm-clang"},
    { device: device, cpu: "r5f", cgt: "ti-arm-clang"},
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

    if(buildOption.cpu.match(/m4f*/)) {
        build_property.filedirs = filedirs_m4f;
        build_property.files = files_m4f;
        build_property.defines = m4_macro;
    }

    if(buildOption.cpu.match(/r5f*/)) {
        build_property.files = files_r5f;
        build_property.filedirs = filedirs_r5f;
        build_property.asmfiles = asmfiles_r5f;
        build_property.defines = r5_macro;
        build_property.cflags = cflags_r5f;
    }
    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
