# ADS131 Implementation Details {#EXAMPLES_PRU_ADC_ADS131_DETAILS}

# Implementation Details

The control of ADC pins is divided between PRU and R5F cores.

In example, the interfacing with ADS131M08 operates in following configuration:
-  8 simultaneously sampling differential inputs
-  Programmable data rate up to 32 kSPS
-  Continous sampling of data
-  Signal trace for the current implementation is shown in figure below (can check timing details from this)

  \imageStyle{ads131_timing_capture.png,width:50% height:120%}
  \image html ads131_timing_capture.png " "
  \imageStyle{ads131_timing_capture_1.png,width:50% height:120%}
  \image html ads131_timing_capture_1.png " "

<!-- ### R5F Role/Job?: -->
### R5F Function:
- Write PRU firmware instructions into IRAM of PRU 
- Configure the IO Expander to connect the PRU IOs to HSE
- Send ADC register configuration information to the PRU core
- Send instructions to PRU core
- Wait for interrupt from PRU indicating 1 block of adc samples has been received
- On interrupt fetch the block id to read and read the samples
- Process them as needed

\code
    /*  Total number of Commands to send to ADC */
    uint32_t noOfCommands = 2;
    /*  Commands array
     *  Add your commands to this (maximum = noOfCommands)
     */
    uint32_t adcConfigCommands[2] = {0x00618400,0x00FF0200};

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
    ; DRDY: _á †á †á †á †_____________________________________________________
    ; SDI:  _____|  Status  |  < Channel 0 - 7 Data >  |  CRC  |____
    m_wait_high_pulse       2, DRDY_PIN

	m_wait_nano_sec     10
    m_pru_clr_pin       ADC_CS_PIN      ; set CS low
    m_wait_nano_sec     10

	;Fetch status register value using serial interface
	m_read_packet_spi_mode1_msb_gpo_sclk       TEMP_REG_1, 24, R11.b0, SCLK_PIN, SDI_PIN, 8, 4

	; Fetch samples using serial interface from each 8 channels
	m_read_packet_spi_mode1_msb_gpo_sclk       ADC_DATA_REG_1, 24, R11.b0, SCLK_PIN, SDI_PIN, 8, 4
	m_read_packet_spi_mode1_msb_gpo_sclk       ADC_DATA_REG_2, 24, R11.b0, SCLK_PIN, SDI_PIN, 8, 4
	m_read_packet_spi_mode1_msb_gpo_sclk       ADC_DATA_REG_3, 24, R11.b0, SCLK_PIN, SDI_PIN, 8, 4
	m_read_packet_spi_mode1_msb_gpo_sclk       ADC_DATA_REG_4, 24, R11.b0, SCLK_PIN, SDI_PIN, 8, 4
	m_read_packet_spi_mode1_msb_gpo_sclk       ADC_DATA_REG_5, 24, R11.b0, SCLK_PIN, SDI_PIN, 8, 4
	m_read_packet_spi_mode1_msb_gpo_sclk       ADC_DATA_REG_6, 24, R11.b0, SCLK_PIN, SDI_PIN, 8, 4
	m_read_packet_spi_mode1_msb_gpo_sclk       ADC_DATA_REG_7, 24, R11.b0, SCLK_PIN, SDI_PIN, 8, 4
	m_read_packet_spi_mode1_msb_gpo_sclk       ADC_DATA_REG_8, 24, R11.b0, SCLK_PIN, SDI_PIN, 8, 4

	;Fetch crc register value using serial interface
	m_read_packet_spi_mode1_msb_gpo_sclk       TEMP_REG_1, 24, R11.b0, SCLK_PIN, SDI_PIN, 8, 4

    m_wait_nano_sec     10
    m_pru_set_pin       ADC_CS_PIN      ; set CS high

	;Convert each sample to 32-bit
   	m_sign_ext          24, 32, ADC_DATA_REG_1, EXTEND_0
    m_sign_ext          24, 32, ADC_DATA_REG_2, EXTEND_0
    m_sign_ext          24, 32, ADC_DATA_REG_3, EXTEND_0
    m_sign_ext          24, 32, ADC_DATA_REG_4, EXTEND_0
    m_sign_ext          24, 32, ADC_DATA_REG_5, EXTEND_0
    m_sign_ext          24, 32, ADC_DATA_REG_6, EXTEND_0
    m_sign_ext          24, 32, ADC_DATA_REG_7, EXTEND_0
    m_sign_ext          24, 32, ADC_DATA_REG_8, EXTEND_0

	;Send samples to shared memory using ipc
    ldi                 R1.b0, &ADC_DATA_REG_1
    m_pru_ipc_send      R1.b0, ADC_DATA_REG_1, R0.b0, R29, CONFIG_PRU_IPC0_RX_INTR_ENABLE, CONFIG_PRU_IPC0_RX_EVENT

    m_prgm_flow_jump_on_intr    PRGM_FLOW_EVENT, PRGM_FLOW_EVENT_BIT, 5, TEMP_REG_1, IDLE, SAMPLE_TRANSFER, ADC_CONFIG, READ_SAMPLES, RESET

    qba    START    ; loop to continue collecting samples
    halt
\endcode

\par
Tips to Modify PRU Firmware:
Refer to \htmllink{../pru_macros_docs/index.html, PRU Source Macros Documentation} for making changes to PRU firmware.