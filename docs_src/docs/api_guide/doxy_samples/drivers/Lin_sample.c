
#include <stdio.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include <drivers/hw_include/cslr_soc.h>
//! [include]
#include <drivers/lin.h>
//! [include]

#define FRAME_LENGTH        (0x8)

#define APP_LIN_BASE_ADDR   (CONFIG_LIN1_BASE_ADDR)
#define APP_LIN_INTR_NUM_0  (CONFIG_LIN1_INTR_NUM_0)
#define APP_LIN_INTR_NUM_1  (CONFIG_LIN1_INTR_NUM_1)

/* Semaphore to indicate transfer completion */
static SemaphoreP_Object gLinTxDoneSem, gLinRxDoneSem;
static HwiP_Object       gLinHwiObject;
static uint32_t          gLinBaseAddr = APP_LIN_BASE_ADDR;

/* Static Function Declarations */
static void    App_linIntrISR(void *arg);

static int32_t App_linIntrReg()
{
    //! [App_linIntrReg]
    int32_t                 status = SystemP_SUCCESS;
    HwiP_Params             hwiPrms;

    /* Register interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = APP_LIN_INTR_NUM_0;
    hwiPrms.callback    = &App_linIntrISR;
    status              = HwiP_construct(&gLinHwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);
    //! [App_linIntrReg]

    return status;
}

/*
 * Common Functions
 */
void App_linSysInit(void)
{
    /* DPL init sets up address transalation unit, on some CPUs this is needed
     * to access SCICLIENT services, hence this needs to happen first
     */
    Dpl_init();
    PowerClock_init();
    /* Now we can do pinmux */
    Pinmux_init();
    /* finally we initialize all peripheral drivers */

//! [App_linSysInit]
/******************************************************************************
 * LIN Configurations
 *****************************************************************************/

    LIN_initModule(CONFIG_LIN1_BASE_ADDR);
    /*
     * Enable LIN test mode
     */
    /*
     * Enable Internal Loopback mode
     */
    LIN_enableIntLoopback(CONFIG_LIN1_BASE_ADDR);
    /*
     * Enable LIN Interrupts
     */
    LIN_enableInterrupt(CONFIG_LIN1_BASE_ADDR,  LIN_INT_ID);
    /*
     * Set the interrupt priority line
     */
    LIN_setInterruptLevel0(CONFIG_LIN1_BASE_ADDR,  LIN_INT_ALL);
    LIN_enableGlobalInterrupt(CONFIG_LIN1_BASE_ADDR, LIN_INTERRUPT_LINE0);
    LIN_clearGlobalInterruptStatus(CONFIG_LIN1_BASE_ADDR, LIN_INTERRUPT_LINE0);
    /*
     * Set the interrupt priority line
     */
    LIN_setInterruptLevel1(CONFIG_LIN1_BASE_ADDR,  LIN_INT_ALL);
    LIN_enableGlobalInterrupt(CONFIG_LIN1_BASE_ADDR, LIN_INTERRUPT_LINE1);
    LIN_clearGlobalInterruptStatus(CONFIG_LIN1_BASE_ADDR, LIN_INTERRUPT_LINE1);
//! [App_linSysInit]
    Drivers_uartInit();
}

void App_SendReceive()
{
    /*
     * Perform 8 data transmissions with different transmit IDs and varying
     * number of bytes transmitted. Received data is checked for correctness.
     */
    for(i = 1 ; i <= FRAME_LENGTH; i++)
    {
        /* Create a new transmit ID and update with parity bits */
        txID = (0x10 + i);
        txID = LIN_generateParityID(txID);

        /*
         * Increment the value of the first 8-bits of the transmitted
         * message data
         */
        txData[0]++;

        /* Reset values in receive buffer array */
        for(dataIndex=0; dataIndex < 8; dataIndex++)
        {
            rxData[dataIndex] = 0xFF;
        }

        /*
         * Set the frame length (number of bytes to be transmitted)
         */
        LIN_setFrameLength(APP_LIN_BASE_ADDR, i);

        //! [App_SendReceive_1]
        /*
         * This places data into the transmit buffer.
         * No ID or data is placed on the bus and transmitted yet.
         */
        LIN_sendData(APP_LIN_BASE_ADDR, txData);

        /*
         * Set the message ID to initiate a header transmission.
         * This causes the ID to be written to the bus followed by the
         * data in the transmit buffers.
         */
        LIN_setIDByte(APP_LIN_BASE_ADDR, txID);

        /* Wait until Transmit buffer is empty and has completed transmission */
        while(!LIN_isTxBufferEmpty(APP_LIN_BASE_ADDR));
        //! [App_SendReceive_1]

        //! [App_SendReceive_2]
        /* Read the received data in the receive buffers */
        LIN_getData(APP_LIN_BASE_ADDR, rxData);
        //! [App_SendReceive_2]

        /* Verify the transmitted data matches the received data */
        for (dataIndex=0; dataIndex < i; dataIndex++)
        {
            if (rxData[dataIndex] != txData[dataIndex])
            {
                error++;
            }
        }
    }
}

/*
 * Configure SCI Mode - This function configures the LIN module to operate as
 * an SCI with the specified settings.
*/
void
App_configureSCIMode(void)
{
    //! [App_configureSCIMode]
    /* Enter LIN reset state to perform configurations */
    LIN_enterSoftwareReset(APP_LIN_BASE_ADDR);

    /* Switch LIN into SCI mode */
    LIN_enableSCIMode(APP_LIN_BASE_ADDR);

    /* Set the SCI communication mode to idle line */
    LIN_setSCICommMode(APP_LIN_BASE_ADDR, LIN_COMM_SCI_IDLELINE);

    /* Set SCI to transmit one stop bit */
    LIN_setSCIStopBits(APP_LIN_BASE_ADDR, LIN_SCI_STOP_ONE);

    /* Disable parity check */
    LIN_disableSCIParity(APP_LIN_BASE_ADDR);

    /* Disable multi-buffer mode */
    LIN_disableMultibufferMode(APP_LIN_BASE_ADDR);

    /* Module set to complete operations when halted by debugger */
    LIN_setDebugSuspendMode(APP_LIN_BASE_ADDR, LIN_DEBUG_COMPLETE);

    /* Set character length as 8-bits */
    LIN_setSCICharLength(APP_LIN_BASE_ADDR, CHAR_LENGTH);

    /* Set to 1 character in response field */
    LIN_setSCIFrameLength(APP_LIN_BASE_ADDR, FRAME_LENGTH);

    /* Enable Internal Loopback mode */
    LIN_enableIntLoopback(APP_LIN_BASE_ADDR);

    /* Enable interrupt for when a frame has been completely received */
    LIN_enableSCIInterrupt(APP_LIN_BASE_ADDR, LIN_SCI_INT_RX);

    /* Set the interrupt priority to line 0 (high) */
    LIN_setSCIInterruptLevel0(APP_LIN_BASE_ADDR, LIN_SCI_INT_RX);

    /* Exit LIN reset state */
    LIN_exitSoftwareReset(APP_LIN_BASE_ADDR);
    //! [App_configureSCIMode]
}
