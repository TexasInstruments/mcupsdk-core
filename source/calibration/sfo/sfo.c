/*
 *  Copyright (C) 2021-2023 Texas Instruments Incorporated
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

#include <stdbool.h>
#include <stdint.h>
#include <calibration/sfo.h>


#define EPWM_O_HRCNFG      0x40
#define EPWM_O_HRCNFG2     0x4E
#define EPWM_O_HRPCTL      0x5A
#define EPWM_O_HRPWMREV    0x7E
#define EPWM_O_DBREDHR     0xA0
#define EPWM_O_DBRED       0xA2
#define EPWM_O_DBFEDHR     0xA4
#define EPWM_O_DBFED       0xA6
#define EPWM_O_TBPRDHR     0xC4
#define EPWM_O_TBPRD       0xC6
#define EPWM_O_CMPA        0xD4
#define EPWM_O_CMPB        0xD8

#define CSL_OTTOCAL_HRPWR       (0x00000042U)
#define CSL_OTTOCAL_HRCAL       (0x00000044U)
#define CSL_OTTOCAL_HRPRD       (0x00000046U)
#define CSL_OTTOCAL_HRCNT0      (0x00000048U)
#define CSL_OTTOCAL_HRCNT1      (0x0000004AU)
#define CSL_OTTOCAL_HRMSTEP     (0x0000004CU)



/* The following are defines for the bit fields in the HRCNFG register */

#define EPWM_HRCNFG_EDGMODE_S     0U
#define EPWM_HRCNFG_EDGMODE_M     0x3U         // ePWMxA Edge Mode Select Bits
#define EPWM_HRCNFG_CTLMODE       0x4U         // ePWMxA Control Mode Select
                                               // Bits
#define EPWM_HRCNFG_HRLOAD_S      3U
#define EPWM_HRCNFG_HRLOAD_M      0x18U        // ePWMxA Shadow Mode Select
                                               // Bits
#define EPWM_HRCNFG_SELOUTB       0x20U        // EPWMB Output Selection Bit
#define EPWM_HRCNFG_AUTOCONV      0x40U        // Autoconversion Bit
#define EPWM_HRCNFG_SWAPAB        0x80U        // Swap EPWMA and EPWMB Outputs
                                               // Bit
#define EPWM_HRCNFG_EDGMODEB_S    8U
#define EPWM_HRCNFG_EDGMODEB_M    0x300U       // ePWMxB Edge Mode Select Bits
#define EPWM_HRCNFG_CTLMODEB      0x400U       // ePWMxB Control Mode Select
                                               // Bits
#define EPWM_HRCNFG_HRLOADB_S     11U
#define EPWM_HRCNFG_HRLOADB_M     0x1800U      // ePWMxB Shadow Mode Select
                                               // Bits
#define EPWM_HRCNFG_LINESEL_S     14U
#define EPWM_HRCNFG_LINESEL_M     0xC000U      // Delay Line Selection bits

/* The following are defines for the bit fields in the HRPWR register */

#define EPWM_HRPWR_CALSTART       0x4U         // Calibration Start/Stop
#define EPWM_HRPWR_CNTSEL         0x8U         // Counter Select Bit
#define EPWM_HRPWR_CALSTS         0x10U        // Calibration Status Bit
#define EPWM_HRPWR_TESTSEL        0x20U        // Test Mode (Lump Delay) Select
                                               // Bit
#define EPWM_HRPWR_CALSEL_S       6U
#define EPWM_HRPWR_CALSEL_M       0x3C0U       // ePWM Delay Line Selection for
                                               // Calibration
#define EPWM_HRPWR_CALPWRON       0x8000U      // Calibration Power On


/* The following are defines for the bit fields in the HRCAL register */

#define EPWM_HRCAL_HRCAL_S        0U
#define EPWM_HRCAL_HRCAL_M        0xFFU        // Delay Line Calibration Value
                                               // Register.

/* The following are defines for the bit fields in the HRMSTEP register*/

#define EPWM_HRMSTEP_HRMSTEP_S    0U
#define EPWM_HRMSTEP_HRMSTEP_M    0xFFU        // High Resolution Micro Step
                                               // Value

#define HWREG(x)        (*((volatile unsigned long *)(x)))
#define HWREGH(x)       (*((volatile unsigned short *)(x)))
#define HWREGB(x)       __byte((int *)(x),0)


#define MEP1                80
#define MEP2                200
#define Count               50
#define PRDVAL              65535   // for all elements in testmode
#define StopCal             0x0
#define StartCal            0x1
#define Select_HRCNT0       0x0
#define Select_HRCNT1       0x1


/* Prototype declarations */

int SFO(void);


/* Globals */

int MEP_SF[(PWM_CH_MAX + 1)];
int SFO_CAL;        // channel to calibrate
                    // to use, declare "extern int SFO_CAL;" in source file
                    // and call SFO() once before changing to select a channel.

extern int MEP_ScaleFactor;
extern uint32_t gOttoCal_base;

/******************************************************************************

 FUNCTION:    SFO(void)
 DESCRIPTION: MEP calibration module (DCAL) result + channel calibration
              result(channel delay lines) -
              Updates MEP_ScaleFactor value
              for selected ePWM channel calibration if user wants to compare
              with "MEP calibration module" (DCAL) result for proper
              operation.  Only DCAL (n=0) can be run while HRPWM is enabled
              Individual channel calibration can only be run on a
              channel with HRPWM capabilities disabled-one channel at a time.

 PARAMETERS:  SFO_CAL - ePWM channel # on which MEP_ScaleFactor will
                        be calculated (0 = DCAL channel)

             gOttoCal_base - OTTOCAL calibration module instance

 RETURN:      1 - SFO calibration complete on selected channel.
              0 - SFO calibration incomplete on selected channel.

******************************************************************************/


int SFO(void)
{
    static uint16_t   hrc1, hrc2;  // holds HRCNT0 count in 65535 HRCNT1 counts
    static uint16_t   TaskPtr = 0U;

    int16_t status;
    float  Denom;
    uint16_t Numer;


    status = SFO_INCOMPLETE;  /* status remains 0 until SFO completes */

    /* SFO MEP Calibration State Machine */

    switch(TaskPtr)
    {
        case 0:   // General Initialization

            //
            // Clear all bits
            //
            HWREGH(gOttoCal_base + CSL_OTTOCAL_HRPWR) = 0x0U;

            //
            // Set the counter period
            //
            HWREGH(gOttoCal_base + CSL_OTTOCAL_HRPRD) = PRDVAL;

            //
            // Eliminate delay in counter start(CNTSEL = 1), enable lump delay
            // (TESTSEL = 1) & turn on the calibration logic(CALPWRON = 1)
            //
            HWREGH(gOttoCal_base + CSL_OTTOCAL_HRPWR) |= (EPWM_HRPWR_CNTSEL |
                                                  EPWM_HRPWR_TESTSEL);
            HWREGH(gOttoCal_base + CSL_OTTOCAL_HRPWR) |= EPWM_HRPWR_CALPWRON;


            //
            // Initialize variables
            //

            //SFO_CAL = 0;
            hrc1 = 0;
            hrc2 = 0;
            TaskPtr = 1;
            break;

        case 1: // Initialization for 1st run


            //
            // The logic should be reinitialized before every calibration run.
            // Clear all bits in HRPWR
            //
            HWREGH(gOttoCal_base + CSL_OTTOCAL_HRPWR) = 0x0U;

            //AM263x: Fix for missing initialization (other than in task 0) of HRPRD for other OTTOCALs
            HWREGH(gOttoCal_base + CSL_OTTOCAL_HRPRD) = PRDVAL;

            //
            // Configure # MEP steps A (1st point) in DCAL mode. This should be
            // done before powering the CAL logic on
            //
            HWREGH(gOttoCal_base + CSL_OTTOCAL_HRCAL) = MEP1;

            //
            // Eliminate delay in counter start(CNTSEL = 1), enable lump delay
            // (TESTSEL = 1) & turn on the calibration logic(CALPWRON = 1)
            //
            HWREGH(gOttoCal_base + CSL_OTTOCAL_HRPWR) |= (EPWM_HRPWR_CNTSEL |
                                                  EPWM_HRPWR_TESTSEL);
            HWREGH(gOttoCal_base + CSL_OTTOCAL_HRPWR) |= EPWM_HRPWR_CALPWRON;

            //
            // Add some delay(23 cycles) after power off. Unrolled the loop
            // to make it interruptable. Use asm(" RPT #20 || NOP");
            // to make it uninterruptable
            //


            //
            // Manually clear HRCNT0 & HRCNT1 to be safe
            //
            HWREGH(gOttoCal_base + CSL_OTTOCAL_HRCNT0) = 0x0U;
            HWREGH(gOttoCal_base + CSL_OTTOCAL_HRCNT1) = 0x0U;

            //
            // Start calibration
            //
            HWREGH(gOttoCal_base + CSL_OTTOCAL_HRPWR) |= EPWM_HRPWR_CALSTART;


            //
            // Update task pointer to next case for next SFO call.
            //
            TaskPtr = 2;

            break;

        case 2: // Wait for 1st run to complete


            //
            // If calibration is not complete, exit SFO() and check again
            // in next function call. CALSTS becomes zero when HRCNT1 value
            // equals HRPRD value or 0xFFFFU.
            //
            if((HWREGH(gOttoCal_base + CSL_OTTOCAL_HRPWR) & EPWM_HRPWR_CALSTS) == 0x0U)
            {
                //
                // Stop calibration. This bit is NOT automatically cleared.
                //
                HWREGH(gOttoCal_base + CSL_OTTOCAL_HRPWR) &= ~EPWM_HRPWR_CALSTART;

                //
                // Get 1st count in HRCNT0 (# of ring osc oscillations)
                //
                hrc1 = HWREGH(gOttoCal_base + CSL_OTTOCAL_HRCNT0);

                //
                // Update task pointer to next case for next SFO call.
                //
                TaskPtr = 3;

                //
                // Power down the calibration logic
                //
                HWREGH(gOttoCal_base + CSL_OTTOCAL_HRPWR) &= ~EPWM_HRPWR_CALPWRON;

                //
                // Add some delay(23 cycles) after power off. Unrolled the loop
                // to make it interruptable. Use asm(" RPT #20 || NOP");
                // to make it uninterruptable.
                //

            }

            break;

        case 3: // Initialization for 2nd run


            //
            // Clear all bits in HRPWR
            //
            HWREGH(gOttoCal_base + CSL_OTTOCAL_HRPWR) = 0x0U;

            //
            // Configure # MEP steps B (2nd point) in DCAL mode. This
            // should be done before powering the CAL logic on
            //
            HWREGH(gOttoCal_base + CSL_OTTOCAL_HRCAL) = MEP2;

            //
            // Eliminate delay in counter start(CNTSEL = 1), enable lump delay
            // (TESTSEL = 1) & turn on the calibration logic(CALPWRON = 1)
            //
            HWREGH(gOttoCal_base + CSL_OTTOCAL_HRPWR) |= (EPWM_HRPWR_CNTSEL |
                                                  EPWM_HRPWR_TESTSEL);
            HWREGH(gOttoCal_base + CSL_OTTOCAL_HRPWR) |= EPWM_HRPWR_CALPWRON;

            //
            // Add some delay(23 cycles) after power off. Unrolled the loop
            // to make it interruptable. Use asm(" RPT #20 || NOP");
            // to make it uninterruptable.
            //



            //
            // Manually clear HRCNT0 & HRCNT1 to be safe
            //
            HWREGH(gOttoCal_base + CSL_OTTOCAL_HRCNT0) = 0x0U;
            HWREGH(gOttoCal_base + CSL_OTTOCAL_HRCNT1) = 0x0U;

            //
            // Start calibration
            //
            HWREGH(gOttoCal_base + CSL_OTTOCAL_HRPWR) |= EPWM_HRPWR_CALSTART;


            //
            // Move to next case when SFO() is called the next time
            //
            TaskPtr = 4;
            break;

        case 4: // Wait for 2nd run to complete


            //
            // If calibration is not complete, exit SFO() and check again next
            // function call
            //
            if((HWREGH(gOttoCal_base + CSL_OTTOCAL_HRPWR) & EPWM_HRPWR_CALSTS) == 0x0U)
            {
                //
                // Stop calibration. This bit is NOT automatically cleared
                //
                HWREGH(gOttoCal_base + CSL_OTTOCAL_HRPWR) &= ~EPWM_HRPWR_CALSTART;

                //
                // Get the count from HRCNT0 (# of ring osc oscillations)
                //
                hrc2 = HWREGH(gOttoCal_base + CSL_OTTOCAL_HRCNT0);

                //
                // Move to next case when SFO() is called the next time
                //
                TaskPtr = 5;

                //
                // Power down the calibration logic
                //
                HWREGH(gOttoCal_base + CSL_OTTOCAL_HRPWR) &= ~EPWM_HRPWR_CALPWRON;

                //
                // Add some delay(23 cycles) after power-off. Unrolled the loop
                // to make it interruptable. Use asm(" RPT #20 || NOP");
                // to make it uninterruptable.
                //
            }

            break;

        case 5: // Process diagnostics data


            //
            // Calculate MEP delay time in 1 SYSCLK cycle
            //
            Denom = (1/(float)hrc2 - 1/(float)hrc1) * 0xFFFF;

            //
            // Calculate # of MEP steps
            //
            Numer = (MEP2 - MEP1) * 2;

            //
            // Calculate MEP scale factor
            //
            MEP_SF[SFO_CAL] = (int)(Numer / Denom + 0.5);
            MEP_ScaleFactor = MEP_SF[0];

            //
            // Update the task pointer to MEP1 calibration initialization task
            // for next call.
            //
            TaskPtr = 1;

            //
            // Update status & assign scale factor value to HRMSTEP register
            //
            if (MEP_ScaleFactor > 255U)
            {
                status = SFO_ERROR;
            }
            else
            {
                //
                // Update HRMSTEP register only with DCAL result
                //
                HWREGH(gOttoCal_base + CSL_OTTOCAL_HRMSTEP) = MEP_SF[0];
                status = SFO_COMPLETE;
            }


            break;
    }
    return status;
}
