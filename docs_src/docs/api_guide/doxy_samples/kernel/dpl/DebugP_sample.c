
//! [include]
#include <stdio.h>
#include <kernel/dpl/DebugP.h>
//! [include]



void samples()
{
//! [assert]
    void *addr = NULL;

    /* This will assert when addr is NULL */
    DebugP_assert(addr!=NULL);
//! [assert]

//! [log]
    uint32_t value = 10;
    char *str = "Hello, world !!!";

    /* use snprintf to format the string and then call the logging function */
    DebugP_log("This is %s and value = %d",
        str,
        value);
//! [log]

//! [scanf]
    uint32_t value32;

    DebugP_log("Enter a 32b number\r\n");
    value32 = 0;
    DebugP_scanf("%d", &value32);
    DebugP_log("32b value = %d\r\n", value32);
//! [scanf]

}