#!/bin/bash

# Create all remaining stub files
cat > generated/ti_drivers_open_close.c << 'EOC'
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
void Drivers_open(void) {}
void Drivers_close(void) {}
EOC

cat > generated/ti_dpl_config.h << 'EOC'
#ifndef TI_DPL_CONFIG_H_
#define TI_DPL_CONFIG_H_
#include <kernel/dpl/SystemP.h>
#include <kernel/dpl/DebugP.h>
void System_init(void);
void System_deinit(void);
#endif
EOC

cat > generated/ti_dpl_config.c << 'EOC'
#include "ti_dpl_config.h"
void System_init(void) {}
void System_deinit(void) {}
EOC

cat > generated/ti_board_config.h << 'EOC'
#ifndef TI_BOARD_CONFIG_H_
#define TI_BOARD_CONFIG_H_
#include <drivers/hw_include/hw_types.h>
#define CONFIG_FLASH0 0
void Board_init(void);
void Board_deinit(void);
#endif
EOC

cat > generated/ti_board_config.c << 'EOC'
#include "ti_board_config.h"
void Board_init(void) {}
void Board_deinit(void) {}
EOC

cat > generated/ti_board_open_close.h << 'EOC'
#ifndef TI_BOARD_OPEN_CLOSE_H_
#define TI_BOARD_OPEN_CLOSE_H_
int32_t Board_driversOpen(void);
void Board_driversClose(void);
#endif
EOC

cat > generated/ti_board_open_close.c << 'EOC'
#include "ti_board_config.h"
#include "ti_board_open_close.h"
#include <kernel/dpl/SystemP.h>
int32_t Board_driversOpen(void) { return SystemP_SUCCESS; }
void Board_driversClose(void) {}
EOC

cat > generated/ti_pinmux_config.c << 'EOC'
#include <drivers/hw_include/hw_types.h>
void Pinmux_init(void) {}
EOC

cat > generated/ti_power_clock_config.c << 'EOC'
#include <drivers/hw_include/hw_types.h>
void PowerClock_init(void) {}
void PowerClock_deinit(void) {}
EOC
