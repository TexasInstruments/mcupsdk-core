# ADS85x8 Implementation Details {#EXAMPLES_PRU_ADC_ADS85X8_DETAILS}

# Implementation Details

The control of ADC pins is divided between PRU and R5F cores as shown:

  \imageStyle{AM64x-ads85x8_interface.png,width:40%}
  \image html AM64x-ads85x8_interface.png "PRU ADS85x8 Setup"

The current interfacing with ADS 85xx operates in following configuration:
-  40kSPS sampling rate
-  Common ConvstA and ConvstB signals (Both pins connected together) (Convst = Conversion Start pin)
-  The CS line is kept low while reading data from all channels
-  The ADC data is read after the Convst signal goes low
-  First Data signal from ADC is not being used
-  Signal trace for the current implementation is shown in figure below (can check timing details from this)

  \imageStyle{ads8598_timing_capture.png,width:80%}
  \image html ads8598_timing_capture.png " "

<!-- ### R5F Role/Job?: -->
### R5F Function:
- Write PRU firmware instructions into IRAM of PRU 
- Initialize ADC and PRU
- Send instructions to PRU core
- Wait for interrupt from PRU indicating 1 block of adc samples has been received
- On interrupt fetch the block id to read and read the samples
- Process them as needed

\code
    ADC_init();
    ADC_powerUp();
    ADC_reset();
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
    ; Wait for Busy to go low indicating conversion has completed
    m_wait_high_pulse       4, 16
    ; Fetch and store channel 1-8 Sample
    m_parallel_read_packet   ADC_COMM_MODE, ADC_COMM_INTERFACE, ADC_COMM_DATAWIDTH, ADC_COMM_TRIG_PRD, ADC_DATA_REG_1
    m_parallel_read_packet   ADC_COMM_MODE, ADC_COMM_INTERFACE, ADC_COMM_DATAWIDTH, ADC_COMM_TRIG_PRD, ADC_DATA_REG_2
    m_parallel_read_packet   ADC_COMM_MODE, ADC_COMM_INTERFACE, ADC_COMM_DATAWIDTH, ADC_COMM_TRIG_PRD, ADC_DATA_REG_3
    m_parallel_read_packet   ADC_COMM_MODE, ADC_COMM_INTERFACE, ADC_COMM_DATAWIDTH, ADC_COMM_TRIG_PRD, ADC_DATA_REG_4
    m_parallel_read_packet   ADC_COMM_MODE, ADC_COMM_INTERFACE, ADC_COMM_DATAWIDTH, ADC_COMM_TRIG_PRD, ADC_DATA_REG_5
    m_parallel_read_packet   ADC_COMM_MODE, ADC_COMM_INTERFACE, ADC_COMM_DATAWIDTH, ADC_COMM_TRIG_PRD, ADC_DATA_REG_6
    m_parallel_read_packet   ADC_COMM_MODE, ADC_COMM_INTERFACE, ADC_COMM_DATAWIDTH, ADC_COMM_TRIG_PRD, ADC_DATA_REG_7
    m_parallel_read_packet   ADC_COMM_MODE, ADC_COMM_INTERFACE, ADC_COMM_DATAWIDTH, ADC_COMM_TRIG_PRD, ADC_DATA_REG_8

    ; Sign extend data to 32 bits
    m_sign_ext_32bit    ADC_DATA_REG_1
    m_sign_ext_32bit    ADC_DATA_REG_2
    m_sign_ext_32bit    ADC_DATA_REG_3
    m_sign_ext_32bit    ADC_DATA_REG_4
    m_sign_ext_32bit    ADC_DATA_REG_5
    m_sign_ext_32bit    ADC_DATA_REG_6
    m_sign_ext_32bit    ADC_DATA_REG_7
    m_sign_ext_32bit    ADC_DATA_REG_8

    ; Send samples to R5F
    ldi        R1.b0, &R2
    m_pru_ipc_send  R1.b0, R2, R0.b0, PRU_IPC_RX_INTR_ENABLE, PRU_IPC_RX_EVENT

    ; Check if there is some instruction from R5F
    m_prgm_flow_jump_on_intr    0x16, 31, 5, TEMP_REG_1, IDLE, task_manager, sample_transfer, iep, main_loop

    qba    START ; loop
    halt
\endcode

\par
Tips to Modify PRU Firmware:
 * For driving the conversion start pins separately, first select separate conversion start from SyConfig to do automatic pinmux settings for that. Then you can check out the IEP code section in PRU to drive that pin by enabling that iep sync out signal.
 * To drive CS pin along with RD pin, check the `m_parallel_read_packet` macro implementation and add code for driving CS pin in that macro.
 * To use input from first data pin, check the `m_parallel_read_packet` macro implementation and add code for checking first data pin in that macro
 * The current firmware does sign extension of 18bits data to 32 bits data. For ADCs with resolution 16 bits, replace the macro `m_sign_ext_32bit` in `examples/pru_io/adc/ads85x8/firmware/main.asm` with `m_sign_ext_16to32_bits`.

Refer to \htmllink{../pru_macros_docs/index.html, PRU Source Macros Documentation} for making changes to PRU firmware.