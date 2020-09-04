
#include <stdio.h>
//! [include]
#include <drivers/gtc.h>
//! [include]

void enableGTC(void)
{
//! [enableGTC]

    /* Enable GTC */
    GTC_enable();

//! [enableGTC]
}