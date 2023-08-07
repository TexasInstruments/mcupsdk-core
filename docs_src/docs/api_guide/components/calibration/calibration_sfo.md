# Scale Factor Optimization (SFO) {#SFO}

[TOC]

## Introduction

The SFO library implements Scale Factor Optimization routine to drive the micro-edge positioner (MEP) calibration
module to run SFO diagnostics and determine the appropriate MEP scale factor (number of MEP steps per coarse EPWMCLK
step) for a device at any given time. 

## Files included
1. SFO lib source code  - sfo.c
2. SFO lib header file  - sfo.h

## Software usage details for SFO library

### 1. Add "Include" Files:   

Include sfo.h header file in application code apart for the other device specific files.   

\code
   E.g. #include "sfo.h"   
\endcode

   
### 2. Variable Declarations:

The application needs to declare below variables to provide OTTOCAL (calibration module) instance addresses used in SFO lib and to capture calibration status & output.       

\code
   E.g. uint16_t status = SFO_INCOMPLETE;
        uint32_t MEP_ScaleFactor = 0; //scale factor value
        uint32_t gOttoCal_base = CSL_CONTROLSS_OTTOCAL0_U_BASE;
\endcode

Use 
- OTTOCAL0 for EPWM0-EPWM7
- OTTOCAL1 for EPWM8-EPWM15
- OTTOCAL2 for EPWM16-EPWM23
- OTTOCAL3 for EPWM24-EPWM31
        
### 3. MEP_ScaleFactor Initialization:

Prior to using the MEP_ScaleFactor variable in application code, SFO() should be called to drive the MEP calibration module to calculate an initial MEP_ScaleFactor value.

\code
   E.g.
   while(status == SFO_INCOMPLETE)
    {
        status = SFO();
        if(status == SFO_ERROR)
        {
            //
            // SFO function returns 2 if an error occurs & # of MEP
            // steps/coarse step exceeds maximum of 255.
            //
            error();
        }
    }
\endcode
    
### 4. Application Code

While the application is running, fluctuations in both device temperature and supply voltage may be expected. To be sure that optimal Scale Factors are used for each ePWM module, the SFO function should be re-run periodically as part of a slower back-ground loop. Some examples of this are shown here. 

\code
E.g.
   void main ()
   {
    int status;
    // User code
    // The status variable returns 1 once a new MEP_ScaleFactor has been
    // calculated by the MEP Calibration Module running SFO
    // diagnostics.
    status = SFO();
    if(status == SFO_ERROR)
    {
     //
     // SFO function returns 2 if an error occurs & # of
     // MEP steps/coarse step exceeds maximum of 255.
     //
     error();
    }
   }
\endcode
   
## Examples

- Refer \ref EXAMPLES_DRIVERS_HRPWM_DEADBAND_SFO
- Refer \ref EXAMPLES_DRIVERS_HRPWM_DUTY_CYCLE_SFO
- Refer \ref EXAMPLES_DRIVERS_HRPWM_PHASE_SHIFT_SFO

   
   
