# EPWM protection solution using PRU {#EXAMPLES_DRIVERS_EPWM_PROTECTION_PRU}

[TOC]

# Introduction

This example shows:
- Support for PWM protection solution using PRU ICSS
 - ICSS PRU to latch, CPU (R5F) to clear the PWM protection status
- Waveforms for PWM protection status, transition from normal to protection status
- Benchmark for ASC shutdown delay time (time from trip signal to PWM output entering protection status).
- Configurable deadband time in normal PWM output.
 - No deadband time needed during changing from freewheeling to H-ASC or L-ASC
- Configurable Freewheeling time.


\imageStyle{am263_epwm_protection_pru_fig1a.png,width:40%}
 \image html am263_epwm_protection_pru_fig1a.png "Figure 1a. PWM protection example - Block Diagram"


## Inputs
- IGBT Fault signal
 - Digital Signal , e.g.1-- Low side IGBT has issue, 0-- no issue.
- OCP Signal
 - Over Current protection ,Digital Signal e.g. 1-- Over current , 0 -- current normal.
- OVP Signal
 - Over Voltage protection, Digital Signal e.g. 1-- Over voltage , 0 --voltage normal.
- For the speed threshold, 2 cases
 - Case 1 (not implemented by this example): Speed Signal: is PWM signal, PWM frequency corresponding to motor Speed. Calculate the motor speed based on this PWM frequency and judge the speed > Threshold or not.
 - Case 2: Speed Level : IO signal ,e.g. 1--Speed > high Threshold, 0-- Speed < high Threshold.

## Outputs
Flowchart for expected output/PWM protection behavior


\imageStyle{am263_epwm_protection_pru_fig2a.png,width:40%}
 \image html am263_epwm_protection_pru_fig2a.png "Figure 2a. Protection flow chart"

Truth table equivalent of protection flowchart
 Speed level/Speed Signal   | IGBT fault signal | OVP signal            | OCP signal            | PWM behavior/status
 ---------------------------|-------------------|-----------------------|-----------------------|-------------
 0 (Speed < high Threshold) |X                  |0                      |0                      | Normal PWM output
 0 (Speed < high Threshold) |X                  |0                      |1                      | Shut down all PWM
 0 (Speed < high Threshold) |X                  |1                      |0                      | Shut down all PWM
 0 (Speed < high Threshold) |X                  |1                      |1                      | Shut down all PWM
 1 (Speed > high Threshold) |0                  |0                      |0                      | Normal PWM output
 1 (Speed > high Threshold) |0                  |0                      |1                      | Shut down all PWM
 1 (Speed > high Threshold) |0                  |1                      |0                      | Low side ASC
 1 (Speed > high Threshold) |0                  |1                      |1                      | Low side ASC
 1 (Speed > high Threshold) |1                  |0                      |0                      | Normal PWM output
 1 (Speed > high Threshold) |1                  |0                      |1                      | Shut down all PWM
 1 (Speed > high Threshold) |1                  |1                      |0                      | High side ASC
 1 (Speed > high Threshold) |1                  |1                      |1                      | High side ASC

- To ensure transition from normal to shut down: Speed level must be updated first followed by OCP, OVP and then IGBT 
- To ensure transition from normal to low/high ASC: Speed level must be updated first followed by IGBT fault signal, OVP and then OCP

Details of PWM behavior/status

 PWM behavior/status                                                                    | Details
 ---------------------------------------------------------------------------------------|-----------------------
 Normal PWM output                                                                      | Normal operation of \n PWM0/1/2_A,\n PWM0/1/2_B
 Shut down all PWM                                                                      | PWM0/1/2_A trips to low (0% duty), \n PWM0/1/2_B trips to low (0% duty)
 Low side ASC, \n Three low side PWM 100% duty ON, \n three high side PWM shunt down          | PWM0/1/2_A trips to low (0% duty), \n PWM0/1/2_B trips to high (100% duty)
 High side ASC, \n Three high side PWM 100% duty ON, \n three Low side PWM shunt down         | PWM0/1/2_A trips to high (100% duty), \n PWM0/1/2_B trips to low (0% duty)


\imageStyle{am263_epwm_protection_pru_fig2b.png,width:100%}
 \image html am263_epwm_protection_pru_fig2b.png "Figure 2b. Normal PWM output to Low side ASC, Normal PWM output to High side ASC"
 
 \imageStyle{am263_epwm_protection_pru_fig2c.png,width:100%}
 \image html am263_epwm_protection_pru_fig2c.png "Figure 2c. Normal PWM output to shut down"


## Implementation

\imageStyle{am263_epwm_protection_pru_fig3a.png,width:100%}
 \image html am263_epwm_protection_pru_fig3a.png "Figure 3a. Implementation block diagram"



Signal path:
GPIO51/52/53/54 -> INPUTXBAR0/1/2/3 -> PWMXBAR0/1/2/3 -> GPI0/1/2/3 ICSS (PRU firmware reads GPI reg, process, writes to GPO reg) GPO0/1/2/3 -> INPUTXBAR4/5/6/7 -> PWMXBAR4/5/6/7 -> EPWM0+1+2 Trip5/6/7/8

- No setup for GPIO51/52/53/54 required to feed input to InputXbar
- INPUTXBAR0/1/2/3 configured to select its inputs as GPIO51/52/53/54 respectively
- PWMXBAR0/1/2/3 configured to select its inputs as INPUTXBAR0/1/2/3 respectively
- ICSS GPI MUX configured to select PWMXBAR0/1/2/3 as input for ICSS GPI0/1/2/3 respectively
- Details of PRU firmware below
- INPUTXBAR4/5/6/7 configured to select its inputs as ICSS GPO0/1/2/3 respectively
- PWMXBAR4/5/6/7 configured to select its inputs as INPUTXBAR4/5/6/7 respectively
- PWMXBAR4/5/6/7 feeds to EPWM trip inputs
- EPWM setup:
 - EPWM0 (used for PWM0_A and PWM0_B)
  - Frequency: 20KHz
  - Left aligned
  - PWM_A duty: 70%
  - Deadband settings: Active High Complementary. RED, FED: 100 (500ns)
  - PWM_A and PWM_B complementary
 - EPWM1 (used for PWM1_A and PWM1_B)
  - Frequency: 20KHz
  - Left aligned
  - PWM_A duty: 70%
  - Deadband settings: Active High Complementary. RED, FED: 100 (500ns)
  - PWM_A and PWM_B complementary
 - EPWM2 (used for PWM2_A and PWM2_B)
  - Frequency: 20KHz
  - Left aligned
  - PWM_A duty: 70%
  - Deadband settings: Active High Complementary. RED, FED: 100 (500ns)
  - PWM_A and PWM_B complementary
- EPWM trip setup:
 - TripInput5 -> DCAH, DCAH high -> DCAEVT1, DCAEVT1 -> Trip EPWM0_A high
 - TripInput6 -> DCAL, DCAL high -> DCAEVT2, DCAEVT2 -> Trip EPWM0_A low
 - TripInput7 -> DCBH, DCBH high -> DCBEVT1, DCBEVT1 -> Trip EPWM0_B high
 - TripInput8 -> DCBL, DCBL high -> DCBEVT2, DCBEVT2 -> Trip EPWM0_B low


ICSS PRU firmware:


\imageStyle{am263_epwm_protection_pru_fig3b.png,width:100%}
 \image html am263_epwm_protection_pru_fig3b.png "Figure 3b. Implementation flow chart"

\code
    .retain     ; Required for building .out with assembly file
    .retainrefs ; Required for building .out with assembly file

    .global     main
    .sect       ".text"

    .asg    r31,    gpi_reg
    .asg    r30,    gpo_reg
    .asg    r1.b0, 	offset_reg8
    .asg    r20,    temp_reg32
    .asg    r21.b0, input_bits_reg8
    .asg    r22.b0, trip_bits_reg8
    .asg    r23.b0, clr_reg8
    .asg    r24.b0, zero_reg8
    .asg    r25,    freewheeling_countdown_reg32
    .asg    r26,    freewheeling_time_reg32
    .asg    0,      LUT_OFFSET                                      ; 16 byte.  Offset of Look up table for PRU FW use. Placed at base of PRU DMEM0 - as optimization
                                                                    ;           Initialize before PRU FW load and run
    .asg    16,     PROTECTION_STATUS_OFFSET                        ; 1 byte.   Offset of Protection status from PRU
    .asg    20,     PROTECTION_STATUS_CLR_OFFSET                    ; 1 byte.   Offset of Clear request from R5
    .asg    24,     PRU_FW_VERSION_OFFSET                           ; 4 byte.   Offset of PRU firmware version
    .asg    28,     FREEWHEELING_TIME_OFFSET                        ; 4 byte.   Offset of Freewheeling time
                                                                    ;           Initialize before PRU FW load and run
main:
    ;Initializations
    ldi     gpo_reg.b0, 0x00                                        ;Clear GPO and GPI registers
    ldi     gpi_reg.b0, 0x00
    ldi     zero_reg8, 0                                            ;Initialize constant value registers
    ldi     offset_reg8, PRU_FW_VERSION_OFFSET
    ldi     temp_reg32.w0, 0x0002                                   ;Write firmware version. Version 2
    ldi     temp_reg32.w2, 0x0000
    sbco    &temp_reg32, c24, offset_reg8, 4
    ldi     offset_reg8, FREEWHEELING_TIME_OFFSET                   ;Initialization of freewheeling time
    lbco    &freewheeling_countdown_reg32, c24, offset_reg8, 4

poll_gpi:
    ;Wait for fault
    mov     input_bits_reg8, gpi_reg.b0                             ;1 cycle. Latch input bits
    qbeq    poll_gpi, input_bits_reg8, 0                            ;1 cycle. Poll inputs until non zero. Optimization: Remove this and directly look up. This will reduce latency to enter protection status, but reduces frequency of polling
    lbco    &trip_bits_reg8, c24, input_bits_reg8, 1                ;3 cycle. Look up in LUT at DMEM0. Optimization: move to PRU core registers
    qbeq    poll_gpi, trip_bits_reg8, 0                             ;1 cycle. Continue to poll if no protection required

    ;Freewheeling
    qbeq    end_freewheeling, freewheeling_countdown_reg32, 0       ;1 cycle. Skip freewheeling if no or 0 freewheeling time
    ldi     gpo_reg.b0, 0x0a                                        ;1 cycle. Turn off all PWMs
freewheeling:                                                       ; Freewheeling start. 
    sub     freewheeling_countdown_reg32, freewheeling_countdown_reg32, 1   ; 1 cycle * freewheeling time programmed
    qbne    freewheeling, freewheeling_countdown_reg32, 0                   ; 1 cycle * freewheeling time programmed
end_freewheeling:

    ;Apply protection and notify CPU
    mov     gpo_reg.b0, trip_bits_reg8                              ;1 cycle. Apply trip output signals to GPO
    sbco    &trip_bits_reg8, c24, PROTECTION_STATUS_OFFSET, 1       ;2 cycle. Write trip bits as protection status

    ;Wait for clear command from CPU and clear
poll_clr:
    lbco    &clr_reg8, c24, PROTECTION_STATUS_CLR_OFFSET, 1         ;3 cycles
    qbeq    poll_clr, clr_reg8, 0                                   ;1 cycle. Poll for clear/ack request from R5F
    ldi     gpo_reg.b0, 0x00                                        ;1 cycle. Clear trip output signals to GPO
    sbco    &zero_reg8, c24, PROTECTION_STATUS_OFFSET, 1            ;2 cycle. Clear protection status
    sbco    &zero_reg8, c24, PROTECTION_STATUS_CLR_OFFSET, 1        ;2 cycle. Clear clear/ack request
    
    ldi     offset_reg8, FREEWHEELING_TIME_OFFSET                   ;1 cycle. Re-Initialization of freewheeling time
    lbco    &freewheeling_countdown_reg32, c24, offset_reg8, 4      ;3 cycle.
    
    qba poll_gpi                                                    ;1 cycle

\endcode

ICSS PRU firmware memory map:

- LUT_OFFSET
 - 16 byte Look up table for PRU FW use. 
 - Input bits to trip output signal mapping
- PROTECTION_STATUS_OFFSET                        
 - 1 byte Protection status from PRU. 
 - bit 0 PWM_A 100%, 
 - bit 1 PWM_A 0%, 
 - bit 2 PWM_B 100%, 
 - bit 3 PWM_B 0%
- PROTECTION_STATUS_CLR_OFFSET                    
 - 1 byte Clear request from R5F. 
 - 0 - no clear request. 
 - Non-zero - clear request
- PRU_FW_VERSION_OFFSET                           
 - 4 byte PRU firmware version
- FREEWHEELING_TIME_OFFSET                        
 - 4 byte Freewheeling time. 
 - Absolute time in 10ns unit.

ICSS PRU Firmware latency:

Execution time - from detection of input fault signal to generation of trip output signal : 8 cycles * 5ns. 40ns


# External Connections


## AM263X-CC
Connect AM263x ControlCard to TMDSHSECDOCK (HSEC180 controlCARD Baseboard Docking Station)


\imageStyle{am263_epwm_protection_pru_fig4a.png,width:91%}
 \image html am263_epwm_protection_pru_fig4a.png "Figure 4a. External connections required"


- Provide 'IGBT Fault signal' input to GPIO51

 - 3.3v -> 1 (Low side IGBT has issue)
 - 0v -> 0 (no issue)

- Provide 'OCP Signal' input to GPIO52
 - 3.3v -> 1 (Over current)
 - 0v -> 0 (current normal)


- Provide 'OVP Signal' input to GPIO53
 - 3.3v -> 1 (Over voltage)
 - 0v -> 0 (voltage normal)


- (not supported) Provide 'Speed Signal' input to pin (TBD)

 - PWM signal, PWM frequency corresponding to motor Speed. CPLD need to calculate the motor speed based on this PWM frequency and judge the speed > Threshold or not.


- Provide 'Speed Level' input to GPIO54

 - 3.3v -> 1 (Speed > high Threshold)
 - 0v -> 0 (Speed < high Threshold)
 
 
- Capture EPWM0_A waveform at HSEC Pin 49
- Capture EPWM0_B waveform at HSEC Pin 51
- Capture EPWM1_A waveform at HSEC Pin 53
- Capture EPWM1_B waveform at HSEC Pin 55
- Capture EPWM2_A waveform at HSEC Pin 50
- Capture EPWM2_B waveform at HSEC Pin 52


## AM263X-LP
This example is not supported on AM263X-LP


# Supported Combinations {#EXAMPLES_DRIVERS_EPWM_PROTECTION_PRU_COMBOS}

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/epwm/epwm_protection_pru

\endcond



# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Establish connections as mentioned in External Connections section
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref DRIVERS_EPWM_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
[Cortex_R5_0] EPWM Protection PRU Test Started ...
PRU0 Processor disabled ...
PRU0 IRAM written ...
PRU LUT written ...
Freewheeling time written ...
PRU0 Processor enabled ...
Normal PWM output (check PWM0_A, PWM0_B, PWM1_A, PWM1_B, PWM2_A, PWM2_B)...
Generating test signals (IGBT fault, OCP, OVP, Speed level signals)...
OCP set. 
Done...
Waiting to enter protection status...
Waiting to enter protection status...
Entered Protection status 0x0a...
Clearing protection status...
Waiting to clear protection status...
Cleared protection status...
 
Normal PWM output (check PWM0_A, PWM0_B, PWM1_A, PWM1_B, PWM2_A, PWM2_B)...
Generating test signals (IGBT fault, OCP, OVP, Speed level signals)...
OCP set. IGBT fault set. 
Done...
Waiting to enter protection status...
Waiting to enter protection status...
Entered Protection status 0x0a...
Clearing protection status...
Waiting to clear protection status...
Cleared protection status...
 
Normal PWM output (check PWM0_A, PWM0_B, PWM1_A, PWM1_B, PWM2_A, PWM2_B)...
Generating test signals (IGBT fault, OCP, OVP, Speed level signals)...
OVP set. 
Done...
Waiting to enter protection status...
Waiting to enter protection status...
Entered Protection status 0x0a...
Clearing protection status...
Waiting to clear protection status...
Cleared protection status...
 
Normal PWM output (check PWM0_A, PWM0_B, PWM1_A, PWM1_B, PWM2_A, PWM2_B)...
Generating test signals (IGBT fault, OCP, OVP, Speed level signals)...
OVP set. IGBT fault set. 
Done...
Waiting to enter protection status...
Waiting to enter protection status...
Entered Protection status 0x0a...
Clearing protection status...
Waiting to clear protection status...
Cleared protection status...
 
Normal PWM output (check PWM0_A, PWM0_B, PWM1_A, PWM1_B, PWM2_A, PWM2_B)...
Generating test signals (IGBT fault, OCP, OVP, Speed level signals)...
OCP set. OVP set. 
Done...
Waiting to enter protection status...
Waiting to enter protection status...
Entered Protection status 0x0a...
Clearing protection status...
Waiting to clear protection status...
Cleared protection status...
 
Normal PWM output (check PWM0_A, PWM0_B, PWM1_A, PWM1_B, PWM2_A, PWM2_B)...
Generating test signals (IGBT fault, OCP, OVP, Speed level signals)...
OCP set. OVP set. IGBT fault set. 
Done...
Waiting to enter protection status...
Waiting to enter protection status...
Entered Protection status 0x0a...
Clearing protection status...
Waiting to clear protection status...
Cleared protection status...
 
Normal PWM output (check PWM0_A, PWM0_B, PWM1_A, PWM1_B, PWM2_A, PWM2_B)...
Generating test signals (IGBT fault, OCP, OVP, Speed level signals)...
Speed level set. OCP set. 
Done...
Waiting to enter protection status...
Waiting to enter protection status...
Entered Protection status 0x0a...
Clearing protection status...
Waiting to clear protection status...
Cleared protection status...
 
Normal PWM output (check PWM0_A, PWM0_B, PWM1_A, PWM1_B, PWM2_A, PWM2_B)...
Generating test signals (IGBT fault, OCP, OVP, Speed level signals)...
Speed level set. OCP set. IGBT fault set. 
Done...
Waiting to enter protection status...
Waiting to enter protection status...
Entered Protection status 0x0a...
Clearing protection status...
Waiting to clear protection status...
Cleared protection status...
 
Normal PWM output (check PWM0_A, PWM0_B, PWM1_A, PWM1_B, PWM2_A, PWM2_B)...
Generating test signals (IGBT fault, OCP, OVP, Speed level signals)...
Speed level set. OVP set. 
Done...
Waiting to enter protection status...
Waiting to enter protection status...
Entered Protection status 0x06...
Clearing protection status...
Waiting to clear protection status...
Cleared protection status...
 
Normal PWM output (check PWM0_A, PWM0_B, PWM1_A, PWM1_B, PWM2_A, PWM2_B)...
Generating test signals (IGBT fault, OCP, OVP, Speed level signals)...
Speed level set. IGBT fault set. OVP set. 
Done...
Waiting to enter protection status...
Waiting to enter protection status...
Entered Protection status 0x09...
Clearing protection status...
Waiting to clear protection status...
Cleared protection status...
 
Normal PWM output (check PWM0_A, PWM0_B, PWM1_A, PWM1_B, PWM2_A, PWM2_B)...
Generating test signals (IGBT fault, OCP, OVP, Speed level signals)...
Speed level set. OVP set. OCP set. 
Done...
Waiting to enter protection status...
Waiting to enter protection status...
Entered Protection status 0x06...
Clearing protection status...
Waiting to clear protection status...
Cleared protection status...
 
Normal PWM output (check PWM0_A, PWM0_B, PWM1_A, PWM1_B, PWM2_A, PWM2_B)...
Generating test signals (IGBT fault, OCP, OVP, Speed level signals)...
Speed level set. IGBT fault set. OVP set. OCP set. 
Done...
Waiting to enter protection status...
Waiting to enter protection status...
Entered Protection status 0x09...
Clearing protection status...
Waiting to clear protection status...
Cleared protection status...


\endcode


\imageStyle{am263_epwm_protection_pru_fig4h.png,width:100%}
 \image html am263_epwm_protection_pru_fig4h.png "Figure 4h. Waveform: Expected waveform from example"


## Waveforms


\imageStyle{am263_epwm_protection_pru_fig4b.png,width:100%}
 \image html am263_epwm_protection_pru_fig4b.png "Figure 4b. Waveform: Normal to Low ASC"


\imageStyle{am263_epwm_protection_pru_fig4c.png,width:100%}
 \image html am263_epwm_protection_pru_fig4c.png "Figure 4c. Waveform: Normal to High ASC"


\imageStyle{am263_epwm_protection_pru_fig4d.png,width:100%}
 \image html am263_epwm_protection_pru_fig4d.png "Figure 4d. Waveform: Normal to Shut down"

\imageStyle{am263_epwm_protection_pru_fig4e.png,width:100%}
 \image html am263_epwm_protection_pru_fig4e.png "Figure 4e. Waveform: Deadband"

\imageStyle{am263_epwm_protection_pru_fig4f.png,width:100%}
 \image html am263_epwm_protection_pru_fig4f.png "Figure 4f. Waveform: freewheeling"



## Benchmarks

OCP high to PWM shut down: Approx 105ns

This is the latency of following path:
GPIO52 (receives OCP rising edge) -> INPUTXBAR1 -> PWMXBAR1 -> GPI1 ICSS (PRU firmware reads GPI reg, process, writes to GPO reg) GPO0/1/2/3 (0101) -> INPUTXBAR4/5/6/7 -> PWMXBAR4/5/6/7 -> EPWM0+1+2 Trip5/6/7/8 -> PWM0/1/2 A/B(Falling edge)

\imageStyle{am263_epwm_protection_pru_fig4g.png,width:100%}
 \image html am263_epwm_protection_pru_fig4g.png "Figure 4g. Benchmark: time from trip signal to PWM output entering protection status"




# References

## ICSS connectivity


\imageStyle{am263_epwm_protection_pru_fig5a.png,width:60%}
 \image html am263_epwm_protection_pru_fig5a.png "Figure 5a. ICSS connectivity"

## AM263x XBARs

InputXbar, PWMXbar


\imageStyle{am263_epwm_protection_pru_fig6a.png,width:60%}
 \image html am263_epwm_protection_pru_fig6a.png "Figure 6a. Input XBAR"
 
\imageStyle{am263_epwm_protection_pru_fig6b.png,width:60%}
 \image html am263_epwm_protection_pru_fig6b.png "Figure 6b. PWM XBAR"
 
 ## AM263x EPWM Digital Compare and Trip Zone module


\imageStyle{am263_epwm_protection_pru_fig6c.png,width:60%}
 \image html am263_epwm_protection_pru_fig6c.png "Figure 6c. EPWM Digital Compare and Trip Zone module"
 
 