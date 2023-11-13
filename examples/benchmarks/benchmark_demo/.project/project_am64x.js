let path = require('path');

let device = "am64x";

const files_r5fss0_0 = {
    common: [
        "main.c",
        "adc_pwm_main.c",
        "benchmarkdemo_stats_with_linux.c",
        "benchmarkdemo_timer.c",
        "benchmarkdemo_ipc_with_linux.c",
    ],
};

const files_r5fss0_1 = {
    common: [
        "main.c",
        "cfft.c",
        "cfft_main.c",
        "math_helper.c",
        "benchmarkdemo_stats_with_linux.c",
        "benchmarkdemo_timer.c",
        "benchmarkdemo_ipc_with_linux.c",
    ],
};

const files_r5fss1_0 = {
    common: [
        "main.c",
        "fir_data.c",
        "fir_main.c",
        "fir.c",
        "math_helper.c",
        "benchmarkdemo_stats_with_linux.c",
        "benchmarkdemo_timer.c",
        "benchmarkdemo_ipc_with_linux.c",
    ],
};

const files_r5fss1_1 = {
    common: [
        "main.c",
        "foc.c",
        "foc_main.c",
        "foc_profile_data.c",
        "foc_test_data.c",
        "ti_r5fmath_trig.c",
        "math_helper.c",
        "benchmarkdemo_stats_with_linux.c",
        "benchmarkdemo_timer.c",
        "benchmarkdemo_ipc_with_linux.c",
    ],
};

/* Relative to where the makefile will be generated
 * Typically at <example_folder>/<BOARD>/<core_os_combo>/<compiler>
 */

const filedirs_r5fss0_0 = {
    common: [
        "..",       /* core_os_combo base */
        "../../../adc_pwm",
        "../../../common",
    ],
};

const filedirs_r5fss0_1 = {
    common: [
        "..",       /* core_os_combo base */
        "../../../cmsis_cfft",
        "../../../common",
    ],
};

const filedirs_r5fss1_0 = {
    common: [
        "..",       /* core_os_combo base */
        "../../../cmsis_fir",
        "../../../common",
    ],
};

const filedirs_r5fss1_1 = {
    common: [
        "..",       /* core_os_combo base */
        "../../../cmsis_foc",
        "../../../common",
    ],
};

const libdirs_nortos = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/nortos/lib",
        "${MCU_PLUS_SDK_PATH}/source/drivers/lib",
        "${MCU_PLUS_SDK_PATH}/source/cmsis/lib",
    ],
};

const libs_nortos_r5f = {
    common: [
        "nortos.am64x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am64x.r5f.ti-arm-clang.${ConfigName}.lib",
        "cmsis.am64x.r5f.ti-arm-clang.release.lib",
    ],
};

const includes_nortos_r5fss0_0 = {
    common: [
        "${MCU_PLUS_SDK_PATH}/examples/benchmarks/benchmark_demo/common",
        "${MCU_PLUS_SDK_PATH}/source/cmsis/DSP/Include",
        "${MCU_PLUS_SDK_PATH}/source/cmsis/Core/Include",
    ],
};

const includes_nortos_r5fss0_1 = {
    common: [
        "${MCU_PLUS_SDK_PATH}/examples/benchmarks/benchmark_demo/common",
        "${MCU_PLUS_SDK_PATH}/source/cmsis/DSP/Include",
        "${MCU_PLUS_SDK_PATH}/source/cmsis/Core/Include",
    ],
};

const includes_nortos_r5fss1_0 = {
    common: [
        "${MCU_PLUS_SDK_PATH}/examples/benchmarks/benchmark_demo/common",
        "${MCU_PLUS_SDK_PATH}/source/cmsis/DSP/Include",
        "${MCU_PLUS_SDK_PATH}/source/cmsis/Core/Include",
    ],
};

const includes_nortos_r5fss1_1 = {
    common: [
        "${MCU_PLUS_SDK_PATH}/examples/benchmarks/benchmark_demo/cmsis_foc/control_suite/math_blocks",
        "${MCU_PLUS_SDK_PATH}/examples/benchmarks/benchmark_demo/common",
        "${MCU_PLUS_SDK_PATH}/source/cmsis/DSP/Include",
        "${MCU_PLUS_SDK_PATH}/source/cmsis/Core/Include",
    ],
};

const lnkfiles = {
    common: [
        "linker.cmd",
    ]
};

const syscfgfile = "../example.syscfg";

const readmeDoxygenPageTag = "EXAMPLE_BENCHMARKDEMO";

const templates_nortos_r5fss0_0 =
[
    {
        input: ".project/templates/am64x/nortos/main_nortos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "benchmarkdemo_adc_pwm_main",
        },
    }
];

const templates_nortos_r5fss0_1 =
[
    {
        input: ".project/templates/am64x/nortos/main_nortos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "benchmarkdemo_cfft_main",
        },
    }
];

const templates_nortos_r5fss1_0 =
[
    {
        input: ".project/templates/am64x/nortos/main_nortos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "benchmarkdemo_fir_main",
        },
    }
];

const templates_nortos_r5fss1_1 =
[
    {
        input: ".project/templates/am64x/nortos/main_nortos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "benchmarkdemo_foc_main",
        },
    }
];

const buildOptionCombos = [
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am64x-evm", os: "nortos", isPartOfSystemProject: true},
    { device: device, cpu: "r5fss0-1", cgt: "ti-arm-clang", board: "am64x-evm", os: "nortos", isPartOfSystemProject: true},
    { device: device, cpu: "r5fss1-0", cgt: "ti-arm-clang", board: "am64x-evm", os: "nortos", isPartOfSystemProject: true},
    { device: device, cpu: "r5fss1-1", cgt: "ti-arm-clang", board: "am64x-evm", os: "nortos", isPartOfSystemProject: true},
];

const systemProject = {
    name: "benchmark_demo",
    tag: "nortos",
    skipProjectSpec: false,
    readmeDoxygenPageTag: readmeDoxygenPageTag,
    board: "am64x-evm",
    projects: [
        { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am64x-evm", os: "nortos"},
        { device: device, cpu: "r5fss0-1", cgt: "ti-arm-clang", board: "am64x-evm", os: "nortos"},
        { device: device, cpu: "r5fss1-0", cgt: "ti-arm-clang", board: "am64x-evm", os: "nortos"},
        { device: device, cpu: "r5fss1-1", cgt: "ti-arm-clang", board: "am64x-evm", os: "nortos"},
    ],
};

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "benchmark_demo";
    property.isInternal = false;
    property.isLinuxInSystem = true;
    property.isLinuxFwGen = true;
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.libdirs = libdirs_nortos;
    build_property.lnkfiles = lnkfiles;
    build_property.syscfgfile = syscfgfile;
    build_property.libs = libs_nortos_r5f;
    build_property.readmeDoxygenPageTag = readmeDoxygenPageTag;
	build_property.projecspecFileAction = "link";
	switch(buildOption.cpu)
	{
		case "r5fss0-1":
			build_property.templates = templates_nortos_r5fss0_1;
			build_property.includes = includes_nortos_r5fss0_1;
			build_property.files = files_r5fss0_1;
			build_property.filedirs = filedirs_r5fss0_1;
			break;
		case "r5fss1-0":
			build_property.templates = templates_nortos_r5fss1_0;
			build_property.includes = includes_nortos_r5fss1_0;
			build_property.files = files_r5fss1_0;
			build_property.filedirs = filedirs_r5fss1_0;
			break;
		case "r5fss1-1":
			build_property.templates = templates_nortos_r5fss1_1;
			build_property.includes = includes_nortos_r5fss1_1;
			build_property.files = files_r5fss1_1;
			build_property.filedirs = filedirs_r5fss1_1;
			break;
		case "r5fss0-0":
			build_property.templates = templates_nortos_r5fss0_0;
			build_property.includes = includes_nortos_r5fss0_0;
			build_property.files = files_r5fss0_0;
			build_property.filedirs = filedirs_r5fss0_0;
			break;
	}
    return build_property;
}

function getSystemProjects(device)
{
    return [systemProject];
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
    getSystemProjects,
};
