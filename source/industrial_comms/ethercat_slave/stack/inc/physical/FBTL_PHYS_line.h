/*!
 * \file FBTL_PHYS_line.h
 *
 * \brief
 * FBTL physical layer for Line underlay.
 *
 * \author
 * KUNBUS GmbH
 *
 * \copyright
 * Copyright (c) 2023, KUNBUS GmbH<br /><br />
 * @KUNBUS_LICENSE@
 *
 */

#if !(defined __FBTL_PHYS_LINE_H__)
#define __FBTL_PHYS_LINE_H__		1

#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <physical/FBTL_phys.h>
#include <system/FBTL_sys.h>

#define FBTL_LINE_VERSION       ((uint32_t)(0x02000000u)) /* 2= FBTL v1 */
#define FBTL_READY_FLAG         ((uint32_t)(0xAA5555AAu))

#define FBTL_IRQMANINGUARD      ((uint32_t)(0x55aacc33u))
#define FBTL_IRQMANOUTGUARD     ((uint32_t)(0x33ccaa55u))

typedef enum FBTL_PHYS_LINE_eMemMapIdx
{
    fbtl_phys_eLine_memConfig   = 0x0000,
} FBTL_PHYS_LINE_eMemMapIdx_t;

/*!
    \brief Line config Header structure

    \ingroup FBTL_PHYS
*/
typedef struct FBTL_PHYS_LINE_SHeader
{
    uint32_t    version;                ///!< Version of RAM structure
    uint32_t    appReady;               ///!< Application ready signal
} FBTL_STRUCT_PACKED FBTL_PHYS_LINE_SHeader_t;

/*!
    \brief Control/Status buffer control structure

    \ingroup FBTL_PHYS
*/
typedef struct FBTL_PHYS_LINE_SIrqManager
{
    volatile    uint32_t                    inGuard;            ///!< IRQ guard

    volatile    uint32_t                    latest;             ///!< Simpson latest
    volatile    uint32_t                    reading;            ///!< Simpson reading
    volatile    uint32_t                    aSlot[2];           ///!< Simpson buffer matrix

    volatile    FBTL_PHYS_SCtrlStatus_t     aCtrlStatus[2][2];  ///!< Control status buffers

    volatile    uint32_t                    outGuard;           ///!< IRQ guard
} FBTL_STRUCT_PACKED FBTL_PHYS_LINE_SIrqManager_t;

#if (defined __cplusplus)
extern "C" {
#endif

extern uint32_t FBTL_PHYS_LINE_initCtrl                 (void*                      pFbtlHandle_p
                                                        ,void*                      pFbtlConfig_p);
extern void     FBTL_PHYS_LINE_run                      (void*                      pFbtlHandle_p);

extern void     FBTL_PHYS_LINE_getCurrentStatusControl  (void*                      pFbtlHandle_p
                                                        ,FBTL_PHYS_SCtrlStatus_t*   pIrqStatus_p);

extern void     FBTL_PHYS_LINE_setCurrentStatusControl  (void*                      pFbtlHandle_p
                                                        ,FBTL_PHYS_SCtrlStatus_t*   pIrqStatus_p);
extern uint32_t FBTL_PHYS_LINE_getAcycChannelSize       (void*                      pFbtlHandle_p);

#if (defined __cplusplus)
}
#endif

#endif /* __FBTL_PHYS_RAM_H__ */
