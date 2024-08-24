const exclude_list = [
    ".git",
    "docs",
    "docs_src",
    "examples/drivers/adc/adc_burst_mode_epwm",
    "examples/drivers/ospi/ospi_nand_flash_io",
    "test/board/nand_flash",
    "test/drivers/dac",
    "test/drivers/ospi",
    "test/drivers/tester"
];


function getExcludeList() {
    return exclude_list;
}

module.exports = {
    getExcludeList,
};