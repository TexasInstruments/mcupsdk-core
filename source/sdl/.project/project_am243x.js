let path = require('path');

let device = "am243x";

const files_r5f = {
    common: [
        "sdl_dpl.c",
        "sdl_esm.c",
        "sdl_ip_esm.c",
        "sdl_esm_core.c",
        "sdl_esm_priv.c",
        "sdl_ip_vtm.c",
    	"sdl_vtm_pvt_sensor.c",
    	"sdl_vtm.c",
    	"sdl_soc_vtm.c",
        "sdl_ip_tog.c",
        "sdl_tog.c",
        "sdl_soc_tog.c",
        "sdl_pok.c",
		"sdl_ip_pok.c",
		"sdl_soc_pok.c",
		"sdl_ip_pok_defs.c",
    ],
};
const filedirs = {
    common: [
        "dpl",
        "esm",
        "esm/soc",
        "esm/soc/am243x",
        "esm/v0",
        "esm/v0/v0_0",
        "vtm",
        "vtm/v0",
        "vtm/v0/soc/am243x",
        "stog/v0",
        "stog/v0/soc",
        "stog/v0/soc/am243x",
        "pok",
	    "pok/v1",
		"pok/v1/soc",
	    "pok/v1/soc/am243x",
    ],
};

const r5_macro = {
    common: [
        "R5F_CORE",
    ],

};

const buildOptionCombos = [
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

    build_property.filedirs = filedirs;
    if(buildOption.cpu.match(/r5f*/)) {
        build_property.files = files_r5f;
        build_property.defines = r5_macro;
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
