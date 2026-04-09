#include <stdlib.h>
#include "ti_drivers_config.h"
#include "ti_board_config.h"

void i2c_dma_target_repeated_start();

int main(void)
{
    System_init();
    Board_init();

    /* Use the new target mode function with repeated start support */
    i2c_dma_target_repeated_start();

    Board_deinit();
    System_deinit();

    return 0;
}
