#ifndef __QOSAPP_PORTING_H__
#define __QOSAPP_PORTING_H__

#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/DebugP.h>
#include <core/enet_mod_tas.h>
#include <core/enet_types.h>
#include <enet_apputils.h>
#include <enet_ethutils.h>

static inline char EnetQoSApp_getChar(void)
{
    char ch;

    DebugP_scanf("%c", &ch);

    return ch;
}


static inline int32_t EnetQoSApp_getNum(void)
{
    int32_t num;

    DebugP_scanf("%d", &num);

    return num;
}

#endif //__QOSAPP_PORTING_H__
