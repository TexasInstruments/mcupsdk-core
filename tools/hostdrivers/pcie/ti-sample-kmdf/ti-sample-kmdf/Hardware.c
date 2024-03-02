/*
 *  Copyright (C) 2023-2024 Texas Instruments Incorporated
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

/*++

Module Name:

    Hardware.c - Hardware specific functions

Abstract:

   This file contains the functions to initialize and use the hardware.

Environment:

    Kernel-mode Driver Framework

--*/

#include "Driver.h"
#include "Hardware.tmh"

#include <wdmguid.h>

/* Sample device BAR0 "data" area (bar0.data) */
#define SMPL_BAR0_DATA_SIZE                     (16 * 1024)
/* Sample device BAR1 data buffer size */
#define SMPL_BAR1_DATA_SIZE                     (1 * 1024 * 1024)
/* Sample device BAR2 data buffer size */
#define SMPL_BAR2_DATA_SIZE                     (1 * 1024 * 1024)

struct bar0
{
    struct config {
        ULONG ctrl;
        ULONG stat;
        ULONGLONG dma_addr;
        ULONG dma_len;
    } config;
    UCHAR pad[128 - sizeof (struct config)];
    ULONG data[SMPL_BAR0_DATA_SIZE / sizeof(ULONG)];
};

#define SMPL_CTRL_INIT_DONE         0x1
#define SMPL_CTRL_RESET             0x2
#define SMPL_CTRL_COPY              0x4
#define SMPL_CTRL_TEST_MSI          0x8
#define SMPL_CTRL_TEST_BARS         0x10

NTSTATUS triggerDwnStrIRQ(IN PDEVICE_CONTEXT DevExt)
/*++
Routine Description:

    Trigger an IRQ in the EP ("downstream") by setting the HTI bit
    in the vendor specific control register in the EP's config space

Arguments:

    DevExt - Pointer to our PDEVICE_CONTEXT

Return Value:

     STATUS_SUCCESS if write succeeded, STATUS_IO_DEVICE_ERROR in case writing fails,
     or STATUS_NO_SUCH_DEVICE if we don't have an interface for writing to config space

--*/

{
    if (DevExt->bus_interface_init)
    {
        ULONG vendor_specific_ctrl = (1 << 8); // HTI
        ULONG retval;
        retval = DevExt->bus_interface.SetBusData(
            DevExt->bus_interface.Context,
            PCI_WHICHSPACE_CONFIG,
            (PUCHAR)&vendor_specific_ctrl,
            0x408,
            sizeof(vendor_specific_ctrl));
        if (retval != sizeof(vendor_specific_ctrl))
            return STATUS_IO_DEVICE_ERROR;
        else
            return STATUS_SUCCESS;
    }

    return STATUS_NO_SUCH_DEVICE;
}

BOOLEAN
deviceIsr(
    IN WDFINTERRUPT Interrupt,
    IN ULONG        MessageID
)
/*++
Routine Description:

    Interrupt handler for the device.

Arguments:

    Interupt - Address of the framework interrupt object
    MessageID - Hardware interrupt message ID (MSI)

Return Value:

     TRUE if our device is interrupting, FALSE otherwise.

--*/
{
    PDEVICE_CONTEXT DevExt = NULL;

    UNREFERENCED_PARAMETER(MessageID);

    TraceEvents(TRACE_LEVEL_VERBOSE, TRACE_HARDWARE, "--> deviceIsr %lu\n", MessageID);

    DevExt = GetDeviceContext(WdfInterruptGetDevice(Interrupt));

    if (MessageID < 32)
        DevExt->msi_test |= (1 << MessageID);

    WdfInterruptQueueDpcForIsr(Interrupt);

    TraceEvents(TRACE_LEVEL_VERBOSE, TRACE_HARDWARE, "<-- deviceIsr\n");

    return TRUE;
}

VOID
deviceDpc(
    IN WDFINTERRUPT WdfInterrupt,
    IN WDFOBJECT    WdfDevice
)

/*++

Routine Description:

    DPC callback for ISR.

Arguments:

    WdfInterrupt - Handle to the framework interrupt object

    WdfDevice - Associated device object.

Return Value:

--*/
{
    PDEVICE_CONTEXT DevExt = NULL;
    WDFREQUEST Request;
    WDF_REQUEST_PARAMETERS  Params;
    NTSTATUS status = STATUS_SUCCESS;
    PVOID                   DataBuffer;
    size_t                  BufferLength;
    PTISAMPLEKMDF_TEST_DMA  copy_request;
    PTISAMPLEKMDF_TEST_MSI  msi_request;
    PTISAMPLEKMDF_TEST_BARS bar_request;
    volatile struct bar0* pbar0;
    UNREFERENCED_PARAMETER(WdfInterrupt);

    TraceEvents(TRACE_LEVEL_VERBOSE, TRACE_HARDWARE, "--> deviceDpc\n");

    DevExt = GetDeviceContext(WdfDevice);

    status = WdfIoQueueRetrieveNextRequest(DevExt->pending_queue, &Request);
    if (!NT_SUCCESS(status)) {
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_HARDWARE,
            "WdfIoQueueRetrieveNextRequest failed 0x%x\n", status);
        return;
    }

    WDF_REQUEST_PARAMETERS_INIT(&Params);

    WdfRequestGetParameters(
        Request,
        &Params
    );

    switch (Params.Parameters.DeviceIoControl.IoControlCode)
    {
    case IOCTL_TISAMPLEKMDF_TEST_DMA:
        status = WdfRequestRetrieveOutputBuffer(Request,
            sizeof(TISAMPLEKMDF_TEST_DMA),
            &DataBuffer,
            &BufferLength);
        if (!NT_SUCCESS(status)) {
            TraceEvents(TRACE_LEVEL_ERROR, TRACE_HARDWARE,
                "WdfRequestRetrieveOutputBuffer failed 0x%x\n", status);
            WdfRequestComplete(Request, status);
            return;
        }

        /* clear copy request in the hardware */
        pbar0 = (volatile struct bar0*)DevExt->bar_mem[DEVICE_BAR_CTRL];
        pbar0->config.ctrl &= ~SMPL_CTRL_COPY;

        copy_request = (PTISAMPLEKMDF_TEST_DMA)DataBuffer;

        copyLoop(DevExt, copy_request->result, sizeof(copy_request->result));

        WdfRequestCompleteWithInformation(Request, STATUS_SUCCESS, sizeof(TISAMPLEKMDF_TEST_DMA));

        break;
    case IOCTL_TISAMPLEKMDF_TEST_MSI:

        /* wait until every interrupt was triggered */
        if (DevExt->msi_test == ((1U << DevExt->num_interrupts) - 1U))
        {
            status = WdfRequestRetrieveOutputBuffer(Request,
                sizeof(TISAMPLEKMDF_TEST_MSI),
                &DataBuffer,
                &BufferLength);
            if (!NT_SUCCESS(status)) {
                TraceEvents(TRACE_LEVEL_ERROR, TRACE_HARDWARE,
                    "WdfRequestRetrieveOutputBuffer failed 0x%x\n", status);
                WdfRequestComplete(Request, status);
                return;
            }

            /* clear MSI request in the hardware */
            pbar0 = (volatile struct bar0*)DevExt->bar_mem[DEVICE_BAR_CTRL];
            pbar0->config.ctrl &= ~SMPL_CTRL_TEST_MSI;

            msi_request = (PTISAMPLEKMDF_TEST_MSI)DataBuffer;

            msi_request->result = DevExt->msi_test;

            WdfRequestCompleteWithInformation(Request, STATUS_SUCCESS, sizeof(TISAMPLEKMDF_TEST_MSI));
        }
        else
        {
            // not done yet, put request back on our pending queue
            status = WdfRequestForwardToIoQueue(Request,
                DevExt->pending_queue);
            if (!NT_SUCCESS(status)) {
                TraceEvents(TRACE_LEVEL_ERROR, TRACE_QUEUE,
                    "WdfRequestForwardToIoQueue failed 0x%x\n", status);
                WdfRequestComplete(Request, status);
                break;
            }
        }

        break;

    case IOCTL_TISAMPLEKMDF_TEST_BARS:

        status = WdfRequestRetrieveOutputBuffer(Request,
            sizeof(TISAMPLEKMDF_TEST_BARS),
            &DataBuffer,
            &BufferLength);
        if (!NT_SUCCESS(status)) {
            TraceEvents(TRACE_LEVEL_ERROR, TRACE_HARDWARE,
                "WdfRequestRetrieveOutputBuffer failed 0x%x\n", status);
            WdfRequestComplete(Request, status);
            return;
        }

        /* clear Bar test request in the hardware */
        pbar0 = (volatile struct bar0*)DevExt->bar_mem[DEVICE_BAR_CTRL];
        pbar0->config.ctrl &= ~SMPL_CTRL_TEST_BARS;

        bar_request = (PTISAMPLEKMDF_TEST_BARS)DataBuffer;

        bar_request->result = 1;

        WdfRequestCompleteWithInformation(Request, STATUS_SUCCESS, sizeof(TISAMPLEKMDF_TEST_BARS));

        break;

    default:
        TraceEvents(TRACE_LEVEL_VERBOSE, TRACE_HARDWARE, "--> deviceDpc\n");
        WdfRequestComplete(Request, STATUS_INVALID_DEVICE_REQUEST);
        return;
    }


    TraceEvents(TRACE_LEVEL_VERBOSE, TRACE_HARDWARE, "<-- deviceDpc\n");
}

NTSTATUS tisamplekmdfPrepareHardware(IN PDEVICE_CONTEXT DevExt, IN WDFCMRESLIST Resources, IN WDFCMRESLIST ResourcesTranslated)
/*++
Routine Description:

    Gets the HW resources assigned by the bus driver from the start-irp
    and maps it to system address space.

Arguments:

    DevExt                  Pointer to our PDEVICE_CONTEXT
    ResourcesTranslated     Wdf provided list of resources

Return Value:

     None

--*/
{
    ULONG i;
    PCM_PARTIAL_RESOURCE_DESCRIPTOR  desc;
    PCM_PARTIAL_RESOURCE_DESCRIPTOR  desc_raw;
    NTSTATUS status = STATUS_SUCCESS;
    ULONG bar = 0;
    ULONG bar_cnt = 0;
    PHYSICAL_ADDRESS bar_pa[3] = { { 0 }, { 0 }, { 0 } };
    ULONG bar_len[3] = { 0, 0, 0 };

    UNREFERENCED_PARAMETER(DevExt);
    UNREFERENCED_PARAMETER(Resources);

    //
    // Parse the resource list and save the resource information.
    //
    for (i = 0; i < WdfCmResourceListGetCount(ResourcesTranslated); i++) {

        desc = WdfCmResourceListGetDescriptor(ResourcesTranslated, i);

        if (!desc) {
            TraceEvents(TRACE_LEVEL_ERROR, TRACE_HARDWARE,
                "WdfResourceCmGetDescriptor failed");
            return STATUS_DEVICE_CONFIGURATION_ERROR;
        }

        switch (desc->Type) {

        case CmResourceTypeMemory:

            TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_HARDWARE,
                " - Memory   Resource [%I64X-%I64X]",
                desc->u.Memory.Start.QuadPart,
                desc->u.Memory.Start.QuadPart +
                desc->u.Memory.Length);

            if (bar <= 2) {
                bar_pa[bar] = desc->u.Memory.Start;
                bar_len[bar] = desc->u.Memory.Length;
            }

            bar++;
            break;

        case CmResourceTypePort:

            TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_HARDWARE,
                " - Port     Resource [%08I64X-%08I64X]",
                desc->u.Port.Start.QuadPart,
                desc->u.Port.Start.QuadPart +
                desc->u.Port.Length);
            break;

        case CmResourceTypeInterrupt:

            desc_raw = WdfCmResourceListGetDescriptor(Resources, i);

            if (desc->Flags & CM_RESOURCE_INTERRUPT_MESSAGE) {
                TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_HARDWARE,
                    " - Message Interrupt Resource [%lu, %lu, %hu]",
                    desc->u.MessageInterrupt.Translated.Vector,
                    desc->u.MessageInterrupt.Translated.Level,
                    desc_raw->u.MessageInterrupt.Raw.MessageCount);
            } else {
                TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_HARDWARE,
                    " - Line Interrupt Resource [%lu, %lu]",
                    desc->u.Interrupt.Vector,
                    desc->u.Interrupt.Level);
            }
            DevExt->irq_vector = desc->u.Interrupt.Vector;
            DevExt->irq_level = desc->u.Interrupt.Level;

            {
                WDF_INTERRUPT_CONFIG interruptConfig;
                int msi_cnt;

                //
                // Create WDFINTERRUPT object.
                //
                WDF_INTERRUPT_CONFIG_INIT(&interruptConfig,
                    deviceIsr,
                    deviceDpc);

                //
                // These first two callbacks will be called at DIRQL.  Their job is to
                // enable and disable interrupts.
                //
                interruptConfig.EvtInterruptEnable = NULL;
                interruptConfig.EvtInterruptDisable = NULL;
                interruptConfig.InterruptTranslated = desc;
                interruptConfig.InterruptRaw = desc_raw;

                for (msi_cnt = 0; msi_cnt < desc_raw->u.MessageInterrupt.Raw.MessageCount; msi_cnt++) {
                    status = WdfInterruptCreate(
                        DevExt->device,
                        &interruptConfig,
                        WDF_NO_OBJECT_ATTRIBUTES,
                        &DevExt->interrupt[msi_cnt]);

                    if (!NT_SUCCESS(status)) {
                        TraceEvents(TRACE_LEVEL_ERROR, TRACE_HARDWARE,
                            "WdfInterruptCreate failed: %!STATUS!\n", status);

                        return status;
                    }
                }

                DevExt->num_interrupts = msi_cnt;

                TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_HARDWARE,
                    "Interrupt registered!\n");
            }

            break;

        default:
            TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_HARDWARE,
                " - %u Resource",
                desc->Type);
            break;

        }
    }

    bar_cnt = bar;

    //
    // if we have a non-zero length for the BAR0 window that means we've found the resource descriptor
    //
    for (bar = 0; bar < bar_cnt; bar++)
    {
        if (bar_len[bar])
        {
            DevExt->bar_mem[bar] = MmMapIoSpaceEx(bar_pa[bar], bar_len[bar], PAGE_READWRITE | PAGE_NOCACHE);

            if (!DevExt->bar_mem[bar]) {
                TraceEvents(TRACE_LEVEL_ERROR, TRACE_HARDWARE,
                    " - Unable to map Bar%u memory %08I64X, length %d",
                    bar, bar_pa[bar].QuadPart, bar_len[bar]);
                return STATUS_INSUFFICIENT_RESOURCES;
            }

            DevExt->bar_pa[bar] = bar_pa[bar];
            DevExt->bar_len[bar] = bar_len[bar];

            TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_HARDWARE,
                "Bar%d at PA %08I64X, length %d mapped at VA %08I64X!\n",
                bar, bar_pa[bar].QuadPart, bar_len[bar],
                (ULONGLONG)DevExt->bar_mem[bar]);
        }
    }

    WDF_DMA_ENABLER_CONFIG   dmaConfig;

    WdfDeviceSetAlignmentRequirement(DevExt->device, FILE_32_BYTE_ALIGNMENT);
    WDF_DMA_ENABLER_CONFIG_INIT(&dmaConfig,
        WdfDmaProfilePacket64,
        64 * 1024);
    status = WdfDmaEnablerCreate(DevExt->device,
        &dmaConfig,
        WDF_NO_OBJECT_ATTRIBUTES,
        &DevExt->dma_enabler);

    if (!NT_SUCCESS(status)) {
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_HARDWARE, "WdfDmaEnablerCreate "
            "failed %08X\n", status);
        return status;
    }
    status = WdfCommonBufferCreate(DevExt->dma_enabler,
        64 * 1024,
        WDF_NO_OBJECT_ATTRIBUTES,
        &DevExt->dma_cmnbuf);

    if (status != STATUS_SUCCESS)
    {
        DevExt->dma_len = 0;
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_HARDWARE, "WdfCommonBufferCreate(Send) "
            "failed %08X\n", status);
        return status;
    }

    DevExt->dma_mem = WdfCommonBufferGetAlignedVirtualAddress(DevExt->dma_cmnbuf);
    DevExt->dma_len = 64 * 1024;
    DevExt->dma_la = WdfCommonBufferGetAlignedLogicalAddress(DevExt->dma_cmnbuf);

    RtlZeroMemory(DevExt->dma_mem, DevExt->dma_len);

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_HARDWARE,
        "DMA buffer at PA %08I64X, length %d mapped at VA %08I64X!\n",
        DevExt->dma_la.QuadPart, DevExt->dma_len, (ULONGLONG)DevExt->dma_mem);

    //
    // Get the BUS_INTERFACE_STANDARD for our device so that we can
    // read & write to PCI config space.
    //
    status = WdfFdoQueryForInterface(DevExt->device,
        &GUID_BUS_INTERFACE_STANDARD,
        (PINTERFACE)&DevExt->bus_interface,
        sizeof(BUS_INTERFACE_STANDARD),
        1, // Version
        NULL); //InterfaceSpecificData
    if (!NT_SUCCESS(status)) {
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_HARDWARE, "Failed to get BUS_INTERFACE_STANDARD for PCIe config space accesses"
            "failed %08X\n", status);
        return status;
    }

    DevExt->bus_interface_init = TRUE;

    volatile struct bar0* pbar0 = (volatile struct bar0*)DevExt->bar_mem[DEVICE_BAR_CTRL];

    pbar0->config.dma_addr = DevExt->dma_la.QuadPart;
    pbar0->config.dma_len = DevExt->dma_len;

    pbar0->config.ctrl = SMPL_CTRL_INIT_DONE;

    triggerDwnStrIRQ(DevExt);

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_HARDWARE,
        "TI sample device sucessfully configured\n");

    return status;
}

NTSTATUS tisamplekmdfUnmapHardware(IN PDEVICE_CONTEXT DevExt)
{
    ULONG bar;

    /* if we've mapped Bar0, tell the device to reset */
    if (DevExt->bar_mem[DEVICE_BAR_CTRL])
    {
        volatile struct bar0* pbar0 = (volatile struct bar0*)DevExt->bar_mem[DEVICE_BAR_CTRL];

        pbar0->config.ctrl = SMPL_CTRL_RESET;

        triggerDwnStrIRQ(DevExt);

        pbar0->config.dma_len = 0;
        pbar0->config.dma_addr = 0;
    }

    /* if we have a DMA buffer delete it */
    if (DevExt->dma_len) {
        WdfObjectDelete(DevExt->dma_cmnbuf);
        DevExt->dma_mem = NULL;
        DevExt->dma_len = 0;
        DevExt->dma_la.QuadPart = 0;
    }

    /* unmap all BARs */
    for (bar = 0; bar < DEVICE_NUM_BARS; bar++) {
        if (DevExt->bar_mem[bar]) {
            MmUnmapIoSpace(DevExt->bar_mem[bar], DevExt->bar_len[bar]);
            DevExt->bar_pa[bar].QuadPart = 0;
            DevExt->bar_mem[bar] = NULL;
            DevExt->bar_len[bar] = 0;
        }
    }

    return STATUS_SUCCESS;
}

NTSTATUS fillBar0(IN PDEVICE_CONTEXT DevExt, ULONG pattern)
{
    volatile struct bar0* pbar0 = (volatile struct bar0*)DevExt->bar_mem[DEVICE_BAR_CTRL];
    int i;

    for (i = 0; i < (sizeof(pbar0->data) / sizeof(ULONG)); i++)
        pbar0->data[i] = pattern;

    return STATUS_SUCCESS;
}

NTSTATUS copyLoop(IN PDEVICE_CONTEXT DevExt, ULONG* buffer, size_t bufSize)
{
    int i;
    PULONG dma_buf = (PULONG)DevExt->dma_mem;

    for (i = 0; i < (DevExt->dma_len / sizeof(ULONG)) && i < (bufSize / sizeof (ULONG)); i++)
        buffer[i] = dma_buf[i];

    return STATUS_SUCCESS;
}

NTSTATUS triggerCopy(IN PDEVICE_CONTEXT DevExt)
{
    volatile struct bar0* pbar0 = (volatile struct bar0*)DevExt->bar_mem[DEVICE_BAR_CTRL];

    pbar0->config.ctrl |= SMPL_CTRL_COPY;

    return triggerDwnStrIRQ(DevExt);
}

NTSTATUS triggerMsis(IN PDEVICE_CONTEXT DevExt)
{
    volatile struct bar0* pbar0 = (volatile struct bar0*)DevExt->bar_mem[DEVICE_BAR_CTRL];

    /* reset bitfield of received MSI vectors */
    DevExt->msi_test = 0x0;

    pbar0->config.ctrl |= SMPL_CTRL_TEST_MSI;

    return triggerDwnStrIRQ(DevExt);
}

NTSTATUS fillBars12(IN PDEVICE_CONTEXT DevExt)
{
    volatile struct bar0* pbar0 = (volatile struct bar0*)DevExt->bar_mem[DEVICE_BAR_CTRL];
    volatile PULONG pbar1 = (PULONG)DevExt->bar_mem[1];
    volatile PULONG pbar2 = (PULONG)DevExt->bar_mem[2];
    int i;

    /* copy data to bar1 */
    for (i = 0; i < DevExt->bar_len[1] / sizeof(ULONG); i++)
    {
        pbar1[i] = i;
    }

    /* copy data to bar2 */
    for (i = 0; i < DevExt->bar_len[2] / sizeof(ULONG); i++)
    {
        pbar2[i] = ~i;
    }

    pbar0->config.ctrl |= SMPL_CTRL_TEST_BARS;

    return triggerDwnStrIRQ(DevExt);
}
