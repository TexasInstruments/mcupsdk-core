/*
 *  Copyright (C) 2025 Texas Instruments Incorporated
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
 #include <kernel/dpl/DebugP.h>
 #include <drivers/ospi/v0/ospi.h>
 #include "ti_drivers_config.h"
 #include "ti_drivers_open_close.h"
 #include "ti_board_open_close.h"
 
 #define FILE_NAME_MAX_LEN (384u)
 
 #define RX_SWEEP_SIZE       (128U)
 #define TX_SWEEP_SIZE       (128U)
 #define RD_DELAY_SWEEP_SIZE (5U)
 
 static char gFilename[FILE_NAME_MAX_LEN] = "graph.bin";
 
 uint8_t gPhySweepData[RD_DELAY_SWEEP_SIZE][TX_SWEEP_SIZE][RX_SWEEP_SIZE];
 
 void loop_forever()
 {
     volatile int loop =1;
 
     while(loop == 1);
 }
 
 void ospi_graph_plotter_main(void *args)
 {
     FILE *fp;
     /* Open drivers to open the UART driver for console */
     Drivers_open();
     Board_driversOpen();
 
     DebugP_log(" Sweeping... !!!\r\n");
 
     OSPI_phyTuneGrapher(gOspiHandle[CONFIG_OSPI0], Flash_getPhyTuningOffset(gFlashHandle[CONFIG_FLASH0]), gPhySweepData);
 
     fp = fopen(gFilename, "wb");
     if (fp == NULL)
     {
         DebugP_log(" [ERROR] Unable to open file %s !!!\r\n", gFilename);
     }
     else
     {
         DebugP_log("Writing to file: %s\r\n", gFilename);

         size_t bytes_written = fwrite((void *)gPhySweepData, 1, RD_DELAY_SWEEP_SIZE*TX_SWEEP_SIZE*RX_SWEEP_SIZE, fp);
         
         if (bytes_written != RD_DELAY_SWEEP_SIZE*TX_SWEEP_SIZE*RX_SWEEP_SIZE) {
            DebugP_log(" [ERROR] Failed to write complete data to file %s !!!\r\n", gFilename);
         }
         else
         {
            DebugP_log("All tests have passed!! \r\n");
         }
         fclose(fp);
     }
 
     
     Board_driversClose();
     Drivers_close();
 }
 
 