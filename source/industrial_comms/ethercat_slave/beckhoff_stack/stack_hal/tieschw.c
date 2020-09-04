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

#ifndef DISABLE_UART_PRINT
#include <stdio.h>
#endif

#include "tieschw.h"
#include <tiescsoc.h>

extern uint32_t pd_read_addr_err, pd_write_addr_err;

/*--------------------------------------------------------------------------------------
------
------  local Types and Defines
------
--------------------------------------------------------------------------------------*/
typedef struct
{
    uint16_t PhysicalStartAddress;
    uint16_t Length;
    uint8_t Settings[4];
} __attribute__((packed)) TSYNCMAN;

/*-----------------------------------------------------------------------------------------
------
------  local variables and constants
------
-----------------------------------------------------------------------------------------*/
TSYNCMAN TmpSyncMan;
uint16_t nAlEventMask;

/*-----------------------------------------------------------------------------------------
------
------  local functions
------
-----------------------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------------------------
------
------  functions
------
-----------------------------------------------------------------------------------------*/


/////////////////////////////////////////////////////////////////////////////////////////
/**
\return  0 if initialization was successful

           \brief   This function intializes the EtherCAT Slave Interface.
*////////////////////////////////////////////////////////////////////////////////////////

uint8_t HW_Init(void)
{
    uint16_t u16PdiCtrl;
    int32_t status = SystemP_FAILURE;
    bsp_params bspInitParams;

    /* the memory interface to the ESC, the ESC-interrupt and the ECAT-timer for the
       watchdog monitoring should be initialized here microcontroller specific */

    tiesc_socParamsInit(&bspInitParams);

    status = bsp_init(&bspInitParams);

    DebugP_assert(status == SystemP_SUCCESS);

    /* we have to wait here, until the ESC is started */
    do
    {
        HW_EscReadWord(u16PdiCtrl, ESC_ADDR_PDI_CONTROL);
        u16PdiCtrl = SWAPWORD(u16PdiCtrl) & 0xFF;

    }
    while((u16PdiCtrl != ESC_PDI_INTERFACE_ON_CHIP_BUS)
            && (u16PdiCtrl !=
                ESC_PDI_INTERFACE_SPI_SLAVE));    //Looking for onchip bus or SPI Slave

    /* Reading ESC Revision and Build */
    uint16_t build, revision;
    HW_EscReadWord(revision, ESC_ADDR_REV_TYPE);
    HW_EscReadWord(build, ESC_ADDR_BUILD);
    tiesc_displayEscVersion(revision, build);

    bsp_start_esc_isr(pruIcss1Handle);

    return 0;
}

void HW_Release(void)
{
    bsp_exit(pruIcss1Handle);
}

void HW_EscRead(uint8_t *pData, uint16_t Address, uint16_t Len)
{
    bsp_read(pruIcss1Handle, pData, Address, Len);
    bsp_pdi_post_read_indication(pruIcss1Handle, Address, Len);
}

void HW_EscReadIsr(uint8_t *pData, uint16_t Address, uint16_t Len)
{
    int16_t sm_index;
    uint16_t ActAddress = bsp_get_process_data_address(pruIcss1Handle, Address, Len,
                          &sm_index);

    if(ActAddress < ESC_ADDR_MEMORY)
    {
        pd_read_addr_err++;
        return;
    }

    bsp_read(pruIcss1Handle, pData, ActAddress, Len);

    bsp_process_data_access_complete(pruIcss1Handle, Address, Len, sm_index);
}

uint8_t __HW_EscReadByteIsr(PRUICSS_Handle pruIcssHandle, uint16_t Address)
{
    uint8_t ByteValue;
    int16_t sm_index;
    uint16_t ActAddress = bsp_get_process_data_address(pruIcssHandle, Address, 1,
                          &sm_index);

    if(ActAddress < ESC_ADDR_MEMORY)
    {
        pd_read_addr_err++;
        return 0;
    }

    ByteValue = bsp_read_byte_isr(pruIcssHandle, ActAddress);

    bsp_process_data_access_complete(pruIcssHandle, Address, 1, sm_index);

    return ByteValue;
}
uint16_t __HW_EscReadWordIsr(PRUICSS_Handle pruIcssHandle, uint16_t Address)
{
    uint16_t WordValue;
    int16_t sm_index;
    uint16_t  ActAddress;
    ActAddress = bsp_get_process_data_address(pruIcssHandle,
                 Address, 2, &sm_index);

    if(ActAddress < ESC_ADDR_MEMORY)
    {
        pd_read_addr_err++;
        return 0;
    }

    WordValue = bsp_read_word_isr(pruIcssHandle, ActAddress);

    bsp_process_data_access_complete(pruIcssHandle, Address, 2, sm_index);
    return WordValue;
}

uint32_t __HW_EscReadDWordIsr(PRUICSS_Handle pruIcssHandle, uint16_t Address)
{
    uint32_t DWordValue;
    int16_t sm_index;
    uint16_t ActAddress = bsp_get_process_data_address(pruIcssHandle, Address, 4,
                          &sm_index);


    if(ActAddress < ESC_ADDR_MEMORY)
    {
        pd_read_addr_err++;
        return 0;
    }

    DWordValue = bsp_read_dword_isr(pruIcssHandle, ActAddress);

    bsp_process_data_access_complete(pruIcssHandle, Address, 4, sm_index);

    return DWordValue;
}
void HW_EscReadMbxMem(uint8_t *pData, uint16_t Address, uint16_t Len)
{
    t_sm_properties *p_sm_properties = bsp_get_sm_properties(MAILBOX_WRITE);
    bsp_pdi_mbx_read_start(pruIcss1Handle);
    bsp_read(pruIcss1Handle, pData, Address, Len);

    if(Len >= p_sm_properties->length - 2)
    {
        bsp_pdi_mbx_read_complete(pruIcss1Handle);
    }
}

void HW_EscWrite(uint8_t *pData, uint16_t Address, uint16_t Len)
{
    bsp_write(pruIcss1Handle, pData, Address, Len);
    bsp_pdi_write_indication(pruIcss1Handle, Address, Len, 0);
}

void HW_EscWriteIsr(uint8_t *pData, uint16_t Address, uint16_t Len)
{
    int16_t sm_index;
    uint16_t ActualAddr = bsp_get_process_data_address(pruIcss1Handle, Address, Len,
                          &sm_index);

    if(ActualAddr < ESC_ADDR_MEMORY)
    {
        pd_write_addr_err++;
        return;
    }

    bsp_write(pruIcss1Handle, pData, ActualAddr, Len);

    bsp_process_data_access_complete(pruIcss1Handle, Address, Len, sm_index);

}

void HW_EscWriteDWord(uint32_t DWordValue, uint16_t Address)
{
    t_sm_properties *p_sm_properties = bsp_get_sm_properties(MAILBOX_READ);
    bsp_write_dword(pruIcss1Handle, DWordValue, Address);

    if(Address == (p_sm_properties->physical_start_addr + p_sm_properties->length
                   - 4))
    {
        bsp_pdi_mbx_write_complete(pruIcss1Handle);
    }

    else
    {
        bsp_pdi_write_indication(pruIcss1Handle, Address, sizeof(uint32_t),
                                 (uint16_t)DWordValue);
    }
}

void HW_EscWriteDWordIsr(uint32_t DWordValue, uint16_t Address)
{
    int16_t sm_index;
    uint16_t ActualAddr = bsp_get_process_data_address(pruIcss1Handle, Address, 4,
                          &sm_index);

    if(ActualAddr < ESC_ADDR_MEMORY)
    {
        pd_write_addr_err++;
        return;
    }

    bsp_write_dword(pruIcss1Handle, DWordValue, ActualAddr);
    bsp_process_data_access_complete(pruIcss1Handle, Address, 4, sm_index);

}
void HW_EscWriteWord(uint16_t WordValue, uint16_t Address)
{
    t_sm_properties *p_sm_properties = bsp_get_sm_properties(MAILBOX_READ);
    bsp_write_word(pruIcss1Handle, WordValue, Address);

    if(Address == (p_sm_properties->physical_start_addr + p_sm_properties->length
                   - 2))
    {
        bsp_pdi_mbx_write_complete(pruIcss1Handle);
    }

    else
    {
        bsp_pdi_write_indication(pruIcss1Handle, Address, sizeof(uint16_t), WordValue);
    }
}

void HW_EscWriteWordIsr(uint16_t WordValue, uint16_t Address)
{
    int16_t sm_index;
    uint16_t ActualAddr = bsp_get_process_data_address(pruIcss1Handle, Address, 2,
                          &sm_index);

    if(ActualAddr < ESC_ADDR_MEMORY)
    {
        pd_write_addr_err++;
        return;
    }

    bsp_write_word(pruIcss1Handle, WordValue, ActualAddr);
    bsp_process_data_access_complete(pruIcss1Handle, Address, 2, sm_index);

}

void HW_EscWriteByte(uint8_t ByteValue, uint16_t Address)
{
    t_sm_properties *p_sm_properties = bsp_get_sm_properties(MAILBOX_READ);
    bsp_write_byte(pruIcss1Handle, ByteValue, Address);

    if(Address == (p_sm_properties->physical_start_addr + p_sm_properties->length
                   - 1))
    {
        bsp_pdi_mbx_write_complete(pruIcss1Handle);
    }

    else
    {
        bsp_pdi_write_indication(pruIcss1Handle, Address, sizeof(uint8_t), ByteValue);
    }

}

void HW_EscWriteByteIsr(uint8_t ByteValue, uint16_t Address)
{
    int16_t sm_index;
    uint16_t ActualAddr  = bsp_get_process_data_address(pruIcss1Handle, Address, 1,
                           &sm_index);

    if(ActualAddr < ESC_ADDR_MEMORY)
    {
        pd_write_addr_err++;
        return;
    }

    bsp_write_byte(pruIcss1Handle, ByteValue, ActualAddr);

    bsp_process_data_access_complete(pruIcss1Handle, Address, 1, sm_index);
}

void HW_EscWriteMbxMem(uint8_t *pData, uint16_t Address, uint16_t Len)
{
    t_sm_properties *p_sm_properties = bsp_get_sm_properties(MAILBOX_READ);

    //Do not write to mailbox if already full
    if((bsp_read_byte(pruIcss1Handle,
                      ESC_ADDR_SM1_STATUS) & SM_STATUS_MBX_FULL))
    {
        return;
    }

    bsp_pdi_mbx_write_start(pruIcss1Handle);
    bsp_write(pruIcss1Handle, pData, Address, Len);

    if(Len >= p_sm_properties->length - 2)
    {
        bsp_pdi_mbx_write_complete(pruIcss1Handle);
    }

}

void HW_RestartTarget(void)
{
    tiesc_bspSoftReset();
}
void HW_SetLed(uint8_t RunLed, uint8_t ErrLed)
{
    tiesc_setRunLed(RunLed);
    tiesc_setErrorLed(ErrLed);
}

unsigned int HW_GetTimer()
{
    return bsp_get_timer_register();
}

void HW_ClearTimer()
{
    bsp_clear_timer_register();
}

uint16_t HW_EepromReload()
{
    uint16_t retval;
    retval = bsp_eeprom_emulation_reload(pruIcss1Handle);
    return retval;
}
