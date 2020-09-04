
//! [include]
#include <stdio.h>
#include <drivers/sciclient.h>
//! [include]

void sciclient_module_power_on_main(void *args)
{
//! [sciclient_module_power_on]
    int32_t  status = SystemP_SUCCESS;
    uint32_t moduleId = TISCI_DEV_TIMER0;
    uint32_t moduleState, resetState, contextLossState;

    /* Check the module status. Need not do power on if it's already ON */
    status = Sciclient_pmGetModuleState(moduleId,
                                        &moduleState,
                                        &resetState,
                                        &contextLossState,
                                        SystemP_WAIT_FOREVER);
    DebugP_assert(SystemP_SUCCESS == status);                                
    if(moduleState == TISCI_MSG_VALUE_DEVICE_HW_STATE_OFF)
    {
        status = Sciclient_pmSetModuleState(moduleId,
                                            TISCI_MSG_VALUE_DEVICE_SW_STATE_ON,
                                            (TISCI_MSG_FLAG_AOP |
                                             TISCI_MSG_FLAG_DEVICE_EXCLUSIVE |
                                             TISCI_MSG_FLAG_DEVICE_RESET_ISO),
                                             SystemP_WAIT_FOREVER);
        DebugP_assert(status == SystemP_SUCCESS);
        
        status = Sciclient_pmSetModuleRst (moduleId,
                                               0x0U,
                                               SystemP_WAIT_FOREVER);
        DebugP_assert(status == SystemP_SUCCESS);
    }
//! [sciclient_module_power_on]
}

#if defined(SOC_AM64X) || defined(SOC_AM243X)
#include <drivers/hw_include/cslr_soc.h> /* For interrupt router output number macro */
#include <drivers/gpio.h> /* For GPIO specific */
void sciclient_rm_irq_main(void *args)
{
//! [sciclient_rm_irq_gpio]
    struct tisci_msg_rm_irq_set_req     rmIrqReq;
    struct tisci_msg_rm_irq_set_resp    rmIrqResp;

    /* Here we specify the core specific interrupt router number which we want to tie to the GPIO peripheral interrupt.
     * In this example we are considering the MAIN GPIO instance, which 54 interrupt router outputs, out of which first 16
     * are routed to all the R5 cores. Since these are shared resources, we will need to decide before hand which outputs will
     * be used by which core, and specify this in the sciclient_defaultBoardCfg_rm.c file. In the current configuration,
     * outputs 8 and 9 are allocated to R50-0 core, we can choose either of these to configure the interrupt configuration
     */
    uint32_t gpioIntrNumber = CSLR_R5FSS0_CORE0_INTR_MAIN_GPIOMUX_INTROUTER0_OUTP_8;

    /* Among the GPIOMUX_INTRRTR0 input pins, 0:86 comes from GPIO0 and 90:177 from GPIO1. We are configuring GPIO1 pin in this example, so
     * we define a base interrupt number to be used later.
     */
    uint32_t gpioIntrRtrInputGpio1_Base = 90;

    /* The user interrupt button SW5 is tied to interrupt num 54 among the 88 GPIO interrupt lines of GPIO1 instance */
    uint32_t gpioPushButtonPinNum = 54;

    /* For setting the IRQ for GPIO using sciclient APIs, we need to populate
     * a structure, tisci_msg_rm_irq_set_req instantiated above. The definition
     * of this struct and details regarding the struct members can be found in 
     * the tisci_rm_irq.h.
     */
    /* Initialize all flags to zero since we'll be setting only a few */
    rmIrqReq.valid_params           = 0U; 
    /* Our request has a destination id, so enable the flag for DST ID */
    rmIrqReq.valid_params          |= TISCI_MSG_VALUE_RM_DST_ID_VALID;
    /* DST HOST IRQ is the output index of the interrupt router. We need to make sure this is also enabled as a valid param */
    rmIrqReq.valid_params          |= TISCI_MSG_VALUE_RM_DST_HOST_IRQ_VALID;
    /* This is not a global event */
    rmIrqReq.global_event           = 0U;
    /* Our interrupt source would be the GPIO peripheral. The source id has to be a device id recognizable by the SYSFW.
     * The list of device IDs can be found in tisci_devices.h file under source/drivers/sciclient/include/tisci/am64x_am243x/.
     * In GPIO case there are 3 possible options - TISCI_DEV_GPIO0, TISCI_DEV_GPIO1, TISCI_DEV_MCU_GPIO0. For input interrupt,
     * we need to choose the TISCI_DEV_GPIO1
     */
    rmIrqReq.src_id                 = TISCI_DEV_GPIO1;
    /* This is the interrupt source index within the GPIO peripheral */
    rmIrqReq.src_index              = gpioIntrRtrInputGpio1_Base + GPIO_GET_BANK_INDEX(gpioPushButtonPinNum);
    /* This is the destination of the interrupt, usually a CPU core. Here we choose the TISCI device ID for R5F0-0 core.
     * For a different core, the corresponding TISCI device id has to be provided */
    rmIrqReq.dst_id                 = TISCI_DEV_R5FSS0_CORE0;
    /* This is the output index of the interrupt router. This depends on the core and board configuration */
    rmIrqReq.dst_host_irq           = gpioIntrNumber;
    /* Rest of the struct members are unused for GPIO interrupt */
    rmIrqReq.ia_id                  = 0U;
    rmIrqReq.vint                   = 0U;
    rmIrqReq.vint_status_bit_index  = 0U;
    rmIrqReq.secondary_host         = TISCI_MSG_VALUE_RM_UNUSED_SECONDARY_HOST;

    /* To set the interrupt we now invoke the Sciclient_rmIrqSet function which
     * will find out the route to configure the interrupt and request DMSC to
     * grant the resource
     */
    if(0 != Sciclient_rmIrqSet(&rmIrqReq, &rmIrqResp, SystemP_WAIT_FOREVER))
    {
        DebugP_log("[Error] Sciclient event config failed!!!\r\n");
        DebugP_assert(FALSE);
    }
//! [sciclient_rm_irq_gpio]
}
#endif