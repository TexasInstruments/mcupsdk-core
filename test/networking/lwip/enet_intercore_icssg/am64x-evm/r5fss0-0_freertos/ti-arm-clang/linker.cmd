#include "ti_enet_config.h"

/* This is the stack that is used by code running within main()
 * In case of NORTOS,
 * - This means all the code outside of ISR uses this stack
 * In case of FreeRTOS
 * - This means all the code until vTaskStartScheduler() is called in main()
 *   uses this stack.
 * - After vTaskStartScheduler() each task created in FreeRTOS has its own stack
 */
--stack_size=8192
/* This is the heap size for malloc() API in NORTOS and FreeRTOS
 * This is also the heap used by pvPortMalloc in FreeRTOS
 */
--heap_size=34816
-e_vectors  /* This is the entry of the application, _vector MUST be plabed starting address 0x0 */

/* This is the size of stack when R5 is in IRQ mode
 * In NORTOS,
 * - Here interrupt nesting is disabled as of now
 * - This is the stack used by ISRs registered as type IRQ
 * In FreeRTOS,
 * - Here interrupt nesting is enabled
 * - This is stack that is used initally when a IRQ is received
 * - But then the mode is switched to SVC mode and SVC stack is used for all user ISR callbacks
 * - Hence in FreeRTOS, IRQ stack size is less and SVC stack size is more
 */
__IRQ_STACK_SIZE = 256;
/* This is the size of stack when R5 is in IRQ mode
 * - In both NORTOS and FreeRTOS nesting is disabled for FIQ
 */
__FIQ_STACK_SIZE = 256;
__SVC_STACK_SIZE = 4096; /* This is the size of stack when R5 is in SVC mode */
__ABORT_STACK_SIZE = 256;  /* This is the size of stack when R5 is in ABORT mode */
__UNDEFINED_STACK_SIZE = 256;  /* This is the size of stack when R5 is in UNDEF mode */

SECTIONS
{
    /* This has the R5F entry point and vector table, this MUST be at 0x0 */
    .vectors:{} palign(8) > R5F_VECS

    /* This has the R5F boot code until MPU is enabled,  this MUST be at a address < 0x80000000
     * i.e this cannot be placed in DDR
     */
    GROUP {
        .text.hwi: palign(8)
        .text.cache: palign(8)
        .text.mpu: palign(8)
        .text.boot: palign(8)
        .text:abort: palign(8) /* this helps in loading symbols when using XIP mode */
    } > R5F_TCMA

    UNION:
    {
        .icss_mem: type = NOLOAD {
#if (ENET_SYSCFG_ICSSG0_ENABLED == 1)
    #if(ENET_SYSCFG_DUAL_MAC == 1)
        #if(ENET_SYSCFG_DUALMAC_PORT1_ENABLED == 1)
            *(*gEnetSoc_icssg0HostPoolMem_0)
            *(*gEnetSoc_icssg0HostQueueMem_0)
            *(*gEnetSoc_icssg0ScratchMem_0)
            #if (ENET_SYSCFG_PREMPTION_ENABLE == 1)
                *(*gEnetSoc_icssg0HostPreQueueMem_0)
            #endif
        #else
            *(*gEnetSoc_icssg0HostPoolMem_1)
            *(*gEnetSoc_icssg0HostQueueMem_1)
            *(*gEnetSoc_icssg0ScratchMem_1)
            #if (ENET_SYSCFG_PREMPTION_ENABLE == 1)
                    *(*gEnetSoc_icssg0HostPreQueueMem_1)
            #endif
        #endif
    #endif
#endif
#if (ENET_SYSCFG_ICSSG1_ENABLED == 1)
    #if(ENET_SYSCFG_DUAL_MAC == 1)
        #if(ENET_SYSCFG_DUALMAC_PORT1_ENABLED == 1)
                *(*gEnetSoc_icssg1HostPoolMem_0)
                *(*gEnetSoc_icssg1HostQueueMem_0)
                *(*gEnetSoc_icssg1ScratchMem_0)
            #if (ENET_SYSCFG_PREMPTION_ENABLE == 1)
                *(*gEnetSoc_icssg1HostPreQueueMem_0)
            #endif
        #else
                *(*gEnetSoc_icssg1HostPoolMem_1)
                *(*gEnetSoc_icssg1HostQueueMem_1)
                *(*gEnetSoc_icssg1ScratchMem_1)
            #if (ENET_SYSCFG_PREMPTION_ENABLE == 1)
                *(*gEnetSoc_icssg1HostPreQueueMem_1)
            #endif
        #endif
    #endif
#endif
#if (ENET_SYSCFG_ICSSG0_ENABLED == 1)
    #if(ENET_SYSCFG_DUAL_MAC == 0)
        *(*gEnetSoc_icssg0PortPoolMem_0)
        *(*gEnetSoc_icssg0PortPoolMem_1)
        *(*gEnetSoc_icssg0HostPoolMem_0)
        *(*gEnetSoc_icssg0HostPoolMem_1)
        *(*gEnetSoc_icssg0HostQueueMem_0)
        *(*gEnetSoc_icssg0HostQueueMem_1)
        *(*gEnetSoc_icssg0ScratchMem_0)
        *(*gEnetSoc_icssg0ScratchMem_1)
        #if (ENET_SYSCFG_PREMPTION_ENABLE == 1)
            *(*gEnetSoc_icssg0HostPreQueueMem_0)
            *(*gEnetSoc_icssg0HostPreQueueMem_1)
        #endif
    #endif
#endif
#if (ENET_SYSCFG_ICSSG1_ENABLED == 1)
    #if(ENET_SYSCFG_DUAL_MAC == 0)
        *(*gEnetSoc_icssg1PortPoolMem_0)
        *(*gEnetSoc_icssg1PortPoolMem_1)
        *(*gEnetSoc_icssg1HostPoolMem_0)
        *(*gEnetSoc_icssg1HostPoolMem_1)
        *(*gEnetSoc_icssg1HostQueueMem_0)
        *(*gEnetSoc_icssg1HostQueueMem_1)
        *(*gEnetSoc_icssg1ScratchMem_0)
        *(*gEnetSoc_icssg1ScratchMem_1)
        #if (ENET_SYSCFG_PREMPTION_ENABLE == 1)
            *(*gEnetSoc_icssg1HostPreQueueMem_0)
            *(*gEnetSoc_icssg1HostPreQueueMem_1)
        #endif
    #endif
#endif
        }
    } > MSRAM

    /* This is rest of code. This can be placed in DDR if DDR is available and needed */
    GROUP {
        .text:   {} palign(8)   /* This is where code resides */
        .rodata: {} palign(8)   /* This is where const's go */
    } > DDR

    /* This is rest of initialized data. This can be placed in DDR if DDR is available and needed */
    GROUP {
        .data:   {} palign(8)   /* This is where initialized globals and static go */
    } > DDR

    /* This is rest of uninitialized data. This can be placed in DDR if DDR is available and needed */
    GROUP {
        .sysmem: {} palign(8)   /* This is where the malloc heap goes */
        .stack:  {} palign(8)   /* This is where the main() stack goes */
    } > DDR

    .enet_dma_mem {
        *(*ENET_DMA_DESC_MEMPOOL)
        *(*ENET_DMA_RING_MEMPOOL)
#if (ENET_SYSCFG_PKT_POOL_ENABLE == 1)
        *(*ENET_DMA_PKT_MEMPOOL)
#endif
    } (NOLOAD) > DDR
    GROUP {
        .bss:    {} palign(8)   /* This is where uninitialized globals go */
        RUN_START(__BSS_START)
        RUN_END(__BSS_END)
    } > DDR

    /* This is where the stacks for different R5F modes go */
    GROUP {
        .irqstack: {. = . + __IRQ_STACK_SIZE;} align(8)
        RUN_START(__IRQ_STACK_START)
        RUN_END(__IRQ_STACK_END)
        .fiqstack: {. = . + __FIQ_STACK_SIZE;} align(8)
        RUN_START(__FIQ_STACK_START)
        RUN_END(__FIQ_STACK_END)
        .svcstack: {. = . + __SVC_STACK_SIZE;} align(8)
        RUN_START(__SVC_STACK_START)
        RUN_END(__SVC_STACK_END)
        .abortstack: {. = . + __ABORT_STACK_SIZE;} align(8)
        RUN_START(__ABORT_STACK_START)
        RUN_END(__ABORT_STACK_END)
        .undefinedstack: {. = . + __UNDEFINED_STACK_SIZE;} align(8)
        RUN_START(__UNDEFINED_STACK_START)
        RUN_END(__UNDEFINED_STACK_END)
    } > DDR
    
    GROUP  :   {
    .resource_table : {
    } palign(4096)
    } > DDR_0  

    /* General purpose user shared memory, used in some examples */
    .bss.user_shared_mem (NOLOAD) : {} > USER_SHM_MEM
    /* this is used when Debug log's to shared memory are enabled, else this is not used */
    .bss.log_shared_mem  (NOLOAD) : {} > LOG_SHM_MEM
    /* this is used only when IPC RPMessage is enabled, else this is not used */
    .bss.ipc_vring_mem   (NOLOAD) : {} > RTOS_NORTOS_IPC_SHM_MEM
    /* General purpose non cacheable memory, used in some examples */
    
    .bss:ddr_shared_mem     (NOLOAD) : {} > DDR
    
    .icssfw: palign(128)    : {} > DDR
    /* InterCore Packet queue non cacheable memory, used in some examples */
    intercore_eth_desc_mem (NOLOAD) : {} palign(128) > ETHFW_SHR_MEM
    intercore_eth_data_mem (NOLOAD) : {} palign(128) > ETHFW_SHR_MEM
}

/*
NOTE: Below memory is reserved for DMSC usage
 - During Boot till security handoff is complete
   0x701E0000 - 0x701FFFFF (128KB)
 - After "Security Handoff" is complete (i.e at run time)
   0x701FC000 - 0x701FFFFF (16KB)

 Security handoff is complete when this message is sent to the DMSC,
   TISCI_MSG_SEC_HANDOVER

 This should be sent once all cores are loaded and all application
 specific firewall calls are setup.
*/

MEMORY
{
    R5F_VECS                : ORIGIN = 0x00000000 , LENGTH = 0x00000040
    R5F_TCMA                : ORIGIN = 0x00000040 , LENGTH = 0x00007FC0
    R5F_TCMB0               : ORIGIN = 0x41010000 , LENGTH = 0x00008000
    MSRAM                   : ORIGIN = 0x70080000 , LENGTH = 0x00100000
    FLASH                   : ORIGIN = 0x60200000 , LENGTH = 0x00100000
    DDR_0                   : ORIGIN = 0xA0100000 , LENGTH = 0x00001000 
    DDR                     : ORIGIN = 0xA0101000 , LENGTH = 0x002FF000
    LINUX_IPC_SHM_MEM       : ORIGIN = 0xA0000000 , LENGTH = 0x00100000
    USER_SHM_MEM            : ORIGIN = 0xA5000000 , LENGTH = 0x80 
    LOG_SHM_MEM             : ORIGIN = 0xA5000080 , LENGTH = 0x00003F80 
    RTOS_NORTOS_IPC_SHM_MEM : ORIGIN = 0xA5004000 , LENGTH = 0x0000C000 
    ETHFW_SHR_MEM           : ORIGIN = 0xA0800000 , LENGTH = 0x00800000

}
