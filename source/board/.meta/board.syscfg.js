
let common = system.getScript("/common");
let board = system.getScript(`/board/soc/board_${common.getSocName()}`);

exports = {
    displayName: "TI Board Drivers",
    templates: [
        {
            name: "/board/board/board_config.c.xdt",
            outputPath: "ti_board_config.c",
            alwaysRun: true,
        },
        {
            name: "/board/board/board_config.h.xdt",
            outputPath: "ti_board_config.h",
            alwaysRun: true,
        },
        {
            name: "/board/board/board_open_close.c.xdt",
            outputPath: "ti_board_open_close.c",
            alwaysRun: true,
        },
        {
            name: "/board/board/board_open_close.h.xdt",
            outputPath: "ti_board_open_close.h",
            alwaysRun: true,
        },
    ],
    topModules: board.getTopModules(),
};
