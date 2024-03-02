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

    device.c - Device handling events for example driver.

Abstract:

   This file contains the device entry points and callbacks.
    
Environment:

    Kernel-mode Driver Framework

--*/

#include "driver.h"
#include "hardware.h"
#include "device.tmh"

#ifdef ALLOC_PRAGMA
#pragma alloc_text (PAGE, tisamplekmdfCreateDevice)
#endif

NTSTATUS
tisamplekmdfEvtDevicePrepareHardware(
    WDFDEVICE      Device,
    WDFCMRESLIST   Resources,
    WDFCMRESLIST   ResourcesTranslated
)
/*++

Routine Description:

    EvtDeviceStart event callback performs operations that are necessary
    to make the driver's device operational. The framework calls the driver's
    EvtDeviceStart callback when the PnP manager sends an IRP_MN_START_DEVICE
    request to the driver stack.

Arguments:

    Device - Handle to a framework device object.

    Resources - Handle to a collection of framework resource objects.
                This collection identifies the raw (bus-relative) hardware
                resources that have been assigned to the device.

    ResourcesTranslated - Handle to a collection of framework resource objects.
                This collection identifies the translated (system-physical)
                hardware resources that have been assigned to the device.
                The resources appear from the CPU's point of view.
                Use this list of resources to map I/O space and
                device-accessible memory into virtual address space

Return Value:

    WDF status code

--*/
{
    NTSTATUS        status = STATUS_SUCCESS;
    PDEVICE_CONTEXT devExt = NULL;

    UNREFERENCED_PARAMETER(Resources);
    UNREFERENCED_PARAMETER(ResourcesTranslated);

    PAGED_CODE();

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE,
        "--> tisamplekmdfEvtDevicePrepareHardware\n");

    devExt = GetDeviceContext(Device);

    status = tisamplekmdfPrepareHardware(devExt, Resources, ResourcesTranslated);
    if (!NT_SUCCESS(status)) {
        return status;
    }

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE,
        "<-- tisamplekmdfEvtDevicePrepareHardware\n");

    return status;

}

NTSTATUS
tisamplekmdfEvtDeviceReleaseHardware(
    IN  WDFDEVICE    Device,
    IN  WDFCMRESLIST ResourcesTranslated
)
/*++

Routine Description:

    EvtDeviceReleaseHardware is called by the framework whenever the PnP manager
    is revoking ownership of our resources.  This may be in response to either
    IRP_MN_STOP_DEVICE or IRP_MN_REMOVE_DEVICE.  The callback is made before
    passing down the IRP to the lower driver.

    In this callback, do anything necessary to free those resources.

Arguments:

    Device - Handle to a framework device object.

    ResourcesTranslated - Handle to a collection of framework resource objects.
                This collection identifies the translated (system-physical)
                hardware resources that have been assigned to the device.
                The resources appear from the CPU's point of view.
                Use this list of resources to map I/O space and
                device-accessible memory into virtual address space

Return Value:

    NTSTATUS - Failures will be logged, but not acted on.

--*/
{
    PDEVICE_CONTEXT devExt = NULL;

    UNREFERENCED_PARAMETER(ResourcesTranslated);

    PAGED_CODE();

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE,
        "--> tisamplekmdfEvtDeviceReleaseHardware\n");

    devExt = GetDeviceContext(Device);

    //
    // Unmap any I/O ports. Disconnecting from the interrupt will be done
    // automatically by the framework.
    //
    tisamplekmdfUnmapHardware(devExt);

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE,
        "<-- tisamplekmdfEvtDeviceReleaseHardware\n");

    return STATUS_SUCCESS;
}

NTSTATUS
tisamplekmdfCreateDevice(
    _Inout_ PWDFDEVICE_INIT DeviceInit
    )
/*++

Routine Description:

    Worker routine called to create a device and its software resources.

Arguments:

    DeviceInit - Pointer to an opaque init structure. Memory for this
                    structure will be freed by the framework when the WdfDeviceCreate
                    succeeds. So don't access the structure after that point.

Return Value:

    NTSTATUS

--*/
{
    WDF_PNPPOWER_EVENT_CALLBACKS pnpPowerCallbacks;
    WDF_OBJECT_ATTRIBUTES deviceAttributes;
    PDEVICE_CONTEXT deviceContext;
    WDFDEVICE device;
    NTSTATUS status;

    PAGED_CODE();

    //
    // Zero out the PnpPowerCallbacks structure.
    //
    WDF_PNPPOWER_EVENT_CALLBACKS_INIT(&pnpPowerCallbacks);

    //
    // Set Callbacks for any of the functions we are interested in.
    // If no callback is set, Framework will take the default action
    // by itself.
    //
    pnpPowerCallbacks.EvtDevicePrepareHardware = tisamplekmdfEvtDevicePrepareHardware;
    pnpPowerCallbacks.EvtDeviceReleaseHardware = tisamplekmdfEvtDeviceReleaseHardware;

    //
    // These two callbacks set up and tear down hardware state that must be
    // done every time the device moves in and out of the D0-working state.
    //
    //pnpPowerCallbacks.EvtDeviceD0Entry = PLxEvtDeviceD0Entry;
    //pnpPowerCallbacks.EvtDeviceD0Exit = PLxEvtDeviceD0Exit;

    //
    // Register the PnP Callbacks..
    //
    WdfDeviceInitSetPnpPowerEventCallbacks(DeviceInit, &pnpPowerCallbacks);

    WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&deviceAttributes, DEVICE_CONTEXT);

    status = WdfDeviceCreate(&DeviceInit, &deviceAttributes, &device);

    if (NT_SUCCESS(status)) {
        //
        // Get a pointer to the device context structure that we just associated
        // with the device object. We define this structure in the device.h
        // header file. DeviceGetContext is an inline function generated by
        // using the WDF_DECLARE_CONTEXT_TYPE_WITH_NAME macro in device.h.
        // This function will do the type checking and return the device context.
        // If you pass a wrong object handle it will return NULL and assert if
        // run under framework verifier mode.
        //
        deviceContext = GetDeviceContext(device);

        //
        // Initialize the context.
        //
        deviceContext->device = device;

        //
        // Create a device interface so that applications can find and talk
        // to us.
        //
        status = WdfDeviceCreateDeviceInterface(
            device,
            &GUID_DEVINTERFACE_tisamplekmdf,
            NULL // ReferenceString
            );

        if (NT_SUCCESS(status)) {
            //
            // Initialize the I/O Package and any Queues
            //
            status = tisamplekmdfQueueInitialize(device);
        }

        DECLARE_CONST_UNICODE_STRING(userDeviceName, L"\\DosDevices\\SampleTI");

        status = WdfDeviceCreateSymbolicLink(device, &userDeviceName);

        if (!NT_SUCCESS(status)) {
            DbgPrint("Failed to create symbolic link");
        }
    }

    return status;
}
