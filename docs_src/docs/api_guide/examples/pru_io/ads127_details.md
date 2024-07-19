# ADS127 Implementation Details {#EXAMPLES_PRU_ADC_ADS127_DETAILS}

# Implementation Details

The control of ADC pins is divided between PRU and R5F cores.

In example, the interfacing with ADS127L11 operates in following configuration:
-  1.0677MSPS sampling rate
-  Continous sampling of data
-  Signal trace for the current implementation is shown in figure below (can check timing details from this)

  \imageStyle{ads127_timing_capture.png,width:80%}
  \image html ads127_timing_capture.png " "

<!-- ### R5F Role/Job?: -->
### R5F Function:
- Write PRU firmware instructions into IRAM of PRU 
- Send ADC register configuration information to the PRU core
- Send instructions to PRU core
- Wait for interrupt from PRU indicating 1 block of adc samples has been received
- On interrupt fetch the block id to read and read the samples
- Process them as needed

\code
    /*  Total number of Commands to send to ADC */
    uint16_t noOfCommands = 4;
    /*  Commands array
     *  Add your commands to this (maximum = noOfCommands)
     */
    uint16_t adcConfigCommands[4] = {0x8708, 0x8708, 0x8708, 0x8708};

    ADC_init();
    ADC_powerUp();
    ADC_configure(noOfCommands, adcConfigCommands);
    ADC_startConversion();

    /* Get ADC data from PRU */
    PRU_IPC_getData(gPruIpc0Handle, samples);
\endcode

### PRU Function:
- Wait for commands from R5F and do accordingly
  While executing ADC interface section:
- Wait for ADC Busy to go low indicating conversion has completed
- Start reading samples from each ADC channel
- Preprocess samples as required
- Send samples to shared memory using ipc
- Repeat above steps again

\code
START:
    ; Wait for DRDY to go low indicating conversion has completed
    ; DRDY: _᠆᠆᠆᠆_____________________________________________________
    ; SDI:  _____|  Status  |  < Channel 0 - 4 Data >  |  CRC  |____
    m_wait_high_pulse       2, DRDY_PIN
    ; Fetch samples using serial interface
	m_wait_nano_sec     10
    m_pru_clr_pin       ADC_CS_PIN      ; set CS low
    m_wait_nano_sec     10

	m_read_packet_spi_mode1_msb_iep_sclk       ADC_DATA_REG_1, 24, R5.b0, TEMP_REG_1, 0, SDI_PIN, SYNC0, 15, 3

    m_wait_nano_sec     10
    m_pru_set_pin       ADC_CS_PIN      ; set CS high

    m_sign_ext          24, 32, ADC_DATA_REG_1, EXTEND_0

    ldi                 R1.b0, &ADC_DATA_REG_1
    m_pru_ipc_send      R1.b0, ADC_DATA_REG_1, R0.b0, R29, CONFIG_PRU_IPC0_RX_INTR_ENABLE, CONFIG_PRU_IPC0_RX_EVENT

    m_prgm_flow_jump_on_intr    PRGM_FLOW_EVENT, PRGM_FLOW_EVENT_BIT, 6, TEMP_REG_1, IDLE, IEP, SAMPLE_TRANSFER, ADC_CONFIG, READ_SAMPLES, RESET

    qba    START    ; loop to continue collecting samples
    halt
\endcode

\par
Tips to Modify PRU Firmware:
Refer to \htmllink{../pru_macros_docs/index.html, PRU Source Macros Documentation} for making changes to PRU firmware.