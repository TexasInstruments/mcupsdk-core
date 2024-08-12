/*
 *  Copyright (C) 2018-2021 Texas Instruments Incorporated
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


#include <stdio.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <kernel/dpl/HwiP.h>
#define RESTRICTED_ADDRESS 0xE0000000
#define VALUE 0xFFFFFFFF

void HwiP_user_data_abort_handler_c(DFSR dfsr,ADFSR adfsr,volatile uint32_t dfar,volatile uint32_t address,volatile uint32_t spsr){
    /*Provides information about the prefetch abort exception
    Contents of DFSR(Data Fault Status Register) register
    1. status: indicates the type of fault generated
    2. sd: distinguishes between an AXI Decode or Slave error on an external abort.
    This bit is only valid for external aborts. For all other aborts types of abort,
    this bit is set to zero
    3. rw:  Indicates whether a read or write access caused an abort
        (0=read abort; 1=write abort)

    Contents of ADFSR(Auxillary Data Fault Status Register) register
    1. index: returns the index value for the access giving the error
    2. side_ext: value returned in this field indicates the source of the error
    3. recoverable_error:  value returned in this field indicates if the error is recoverable
        (0=Unrecoverable error, 1=Recoverable Error)
    4. cacheway: value returned in this field indicates the cache way or ways in which the error occurred

    dfar(Data Fault Address Register): Address of the data trying to be accessed
    address: Instruction causing the exception
    Please refer to R5F TRM for more information*/
    (void)dfsr;(void)adfsr;(void)dfar;(void)address;(void)spsr;
    volatile uint32_t loop = 1;
    while(loop != 0U){ ; }
}
void data_abort_main(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    /*Create a pointer to a restricted address and try to write*/
    uint32_t *ptr=(uint32_t*)(RESTRICTED_ADDRESS);
    *ptr=VALUE;
    Board_driversClose();
    Drivers_close();
}

