/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <board/eeprom.h>
#include <drivers/hw_include/csl_types.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

extern EEPROM_Config gEepromConfig[];
extern uint32_t gEepromConfigNum;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

EEPROM_Handle EEPROM_open(uint32_t instanceId, const EEPROM_Params *params)
{
    EEPROM_Config *config = NULL;

    if(instanceId < gEepromConfigNum)
    {
        config = &gEepromConfig[instanceId];
        if(config->fxns && config->fxns->openFxn)
        {
            int32_t status;

            status = config->fxns->openFxn(config, params);
            if(status != SystemP_SUCCESS)
            {
                config = NULL;
            }
        }
    }

    return (config);
}

void EEPROM_close(EEPROM_Handle handle)
{
    EEPROM_Config *config = (EEPROM_Config *) handle;

    if(config && config->fxns && config->fxns->closeFxn)
    {
        config->fxns->closeFxn(config);
    }

    return;
}

int32_t EEPROM_read(EEPROM_Handle handle,
                    uint32_t offset,
                    uint8_t *buf,
                    uint32_t len)
{
    int32_t         status = SystemP_FAILURE;
    EEPROM_Config  *config = (EEPROM_Config *) handle;

    if(config && config->fxns && config->fxns->readFxn)
    {
        status = config->fxns->readFxn(config, offset, buf, len);
    }

    return (status);
}

int32_t EEPROM_write(EEPROM_Handle handle,
                     uint32_t offset,
                     const uint8_t *buf,
                     uint32_t len)
{
    int32_t         status = SystemP_FAILURE;
    EEPROM_Config  *config = (EEPROM_Config *) handle;

    if(config && config->fxns && config->fxns->writeFxn)
    {
        status = config->fxns->writeFxn(config, offset, buf, len);
    }

    return (status);
}

const EEPROM_Attrs *EEPROM_getAttrs(uint32_t instanceId)
{
    EEPROM_Attrs   *attrs = NULL;
    EEPROM_Config  *config = NULL;

    if(instanceId < gEepromConfigNum)
    {
        config = &gEepromConfig[instanceId];
        attrs = config->attrs;
    }

    return (attrs);
}

void EEPROM_Params_init(EEPROM_Params *params)
{
    if(NULL != params)
    {
        params->driverInstance  = 0U;
        params->i2cAddress      = 0x50U;
    }

    return;
}
