
#include <stdio.h>
//! [include]
#include <drivers/pinmux.h>
//! [include]

//! [config]
static Pinmux_PerCfg_t gMainPinmuxData[] =
{
    {
        PIN_SPI0_CS1,
        PIN_MODE(7) | ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    {PINMUX_END, PINMUX_END}
};

void Pinmux_init(void)
{
    Pinmux_config(gMainPinmuxData, PINMUX_DOMAIN_ID_MAIN);
}
//! [config]
