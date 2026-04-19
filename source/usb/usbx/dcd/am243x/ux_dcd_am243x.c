/*
 *  Copyright (C) 2018-2026 Texas Instruments Incorporated
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

#include <ux_api.h>
#include <ux_device_stack.h>
#include <ux_utility.h>

#include <usb_wrapper.h>

#include "ux_dcd_am243x.h"

static UINT _ux_dcd_am243x_frame_number_get(UX_SLAVE_DCD *dcd, ULONG *frame_number);
static UINT _ux_dcd_am243x_transfer_request(UX_SLAVE_DCD *dcd, UX_SLAVE_TRANSFER *transfer_request);
static UINT _ux_dcd_am243x_transfer_abort(UX_SLAVE_DCD *dcd, UX_SLAVE_TRANSFER *transfer_request);
static UINT _ux_dcd_am243x_endpoint_create(UX_SLAVE_DCD *dcd, UX_SLAVE_ENDPOINT *endpoint);
static UINT _ux_dcd_am243x_endpoint_destroy(UX_SLAVE_DCD *dcd, UX_SLAVE_ENDPOINT *endpoint);
static UINT _ux_dcd_am243x_endpoint_stall_clear(UX_SLAVE_DCD *dcd, UX_SLAVE_ENDPOINT *endpoint);
static UINT _ux_dcd_am243x_endpoint_stall(UX_SLAVE_DCD *dcd, UX_SLAVE_ENDPOINT *endpoint);
static UINT _ux_dcd_am243x_address_set(UX_SLAVE_DCD *dcd, ULONG address);
static UINT _ux_dcd_am243x_state_change(UX_SLAVE_DCD *dcd, ULONG state);
static UINT _ux_dcd_am243x_endpoint_status(UX_SLAVE_DCD *dcd, ULONG endpoint_index);

static void _ux_am234x_ep0_in_xfer_cb(CUSBD_Ep *ep, CUSBD_Req *req);
static void _ux_am234x_ep0_out_xfer_cb(CUSBD_Ep* ep, CUSBD_Req *req);
static void _ux_am234x_ep_in_xfer_cb(CUSBD_Ep *ep, CUSBD_Req *req);
static void _ux_am234x_ep_out_xfer_cb(CUSBD_Ep *ep, CUSBD_Req *req);

static CUSBD_Ep* getEpFromAddr(CUSBD_Dev *dev, uint8_t epaddr);

#define UX_DCD_AM243X_EP_STATUS_DISABLED 0u
#define UX_DCD_AM243X_EP_STATUS_ENABLED 1u
#define UX_DCD_AM243X_EP_STATUS_STALLED 2u

/* Local list of created endpoints. */
#define UX_SLAVE_EP_COUNT (8U)
static UX_SLAVE_ENDPOINT *g_ep0;
static UINT ep0_status;
static UX_SLAVE_ENDPOINT *g_ep_in[UX_SLAVE_EP_COUNT];
static UX_SLAVE_ENDPOINT *g_ep_out[UX_SLAVE_EP_COUNT];
static UINT ep_in_status[UX_SLAVE_EP_COUNT];
static UINT ep_out_status[UX_SLAVE_EP_COUNT];

/* Event queue memory. */
#define DCD_QUEUE_MEM_SIZE (1024U)
uint8_t dcd_queue_mem[DCD_QUEUE_MEM_SIZE];
TX_QUEUE _ux_dcd_am243x_queue;

static CUSBD_Ep* getEpFromAddr(CUSBD_Dev *dev, uint8_t epaddr)
{
    LIST_ListHead *list;
    CUSBD_Ep *ep;

    if ((dev->ep0->address & 0x0Fu) == (epaddr & 0x0Fu)) {
        return dev->ep0;
    }

    list = dev->epList.next;
    do {
        ep = (CUSBD_Ep *)list;

        if (ep->address == epaddr) {
            return ep;
        }

        list = list->next;
    } while(list != &dev->epList);

    /* Endpoint not found, return NULL */
    return NULL;
}


/*
 * Initializes the device controller driver. Must be called after the USBX
 *  device subsystems are initialized but before the hardware is initialized.
 */
UINT _ux_dcd_am243x_initialize(ULONG dcd_io)
{
    UX_SLAVE_DCD *dcd;
    UINT tx_status;

    (void)dcd_io; /* Parameter unused, the cadence reference driver performs all the necessary hardware accesses. */

    /* Create the driver event queue. */
    tx_status = tx_queue_create(&_ux_dcd_am243x_queue, "dcd_queue", sizeof(_ux_am243x_dcd_msg_t)/sizeof(ULONG), dcd_queue_mem, sizeof(dcd_queue_mem));
    if(tx_status != TX_SUCCESS) {
        return UX_ERROR;
    }

    dcd = &_ux_system_slave -> ux_system_slave_dcd;
    dcd->ux_slave_dcd_controller_type = UX_DCD_AM243X_SLAVE_CONTROLLER;
    dcd->ux_slave_dcd_controller_hardware = NULL;
    dcd->ux_slave_dcd_function = _ux_dcd_am243x_function;
    dcd->ux_slave_dcd_status = UX_DCD_STATUS_OPERATIONAL;

    dcd->ux_slave_dcd_io = 0u; /* Parameter is unused by the driver. */

    return UX_SUCCESS;
}


UINT _ux_dcd_am243x_initialize_complete(VOID)
{
    UX_SLAVE_DCD *dcd;
    UX_SLAVE_DEVICE *device;
    UCHAR *device_framework;
    UX_SLAVE_TRANSFER *transfer_request;


    dcd = &_ux_system_slave->ux_system_slave_dcd;
    device = &_ux_system_slave->ux_system_slave_device;

    /* Set the descriptors according to the speed. */
    if (_ux_system_slave->ux_system_slave_speed == UX_FULL_SPEED_DEVICE)
    {
        _ux_system_slave->ux_system_slave_device_framework = _ux_system_slave->ux_system_slave_device_framework_full_speed;
        _ux_system_slave->ux_system_slave_device_framework_length = _ux_system_slave->ux_system_slave_device_framework_length_full_speed;
    }
    else
    {
        _ux_system_slave->ux_system_slave_device_framework = _ux_system_slave->ux_system_slave_device_framework_high_speed;
        _ux_system_slave->ux_system_slave_device_framework_length = _ux_system_slave->ux_system_slave_device_framework_length_high_speed;
    }

    device_framework = _ux_system_slave -> ux_system_slave_device_framework;

    /* Parse descriptor. */
    _ux_utility_descriptor_parse(device_framework, _ux_system_device_descriptor_structure,
                                UX_DEVICE_DESCRIPTOR_ENTRIES, (UCHAR *) &device -> ux_slave_device_descriptor);

    /* Setup the endpoint 0 transfer request structure. */
    transfer_request = &device->ux_slave_device_control_endpoint.ux_slave_endpoint_transfer_request;
    transfer_request->ux_slave_transfer_request_timeout = UX_CONTROL_TRANSFER_TIMEOUT;
    transfer_request->ux_slave_transfer_request_current_data_pointer = transfer_request->ux_slave_transfer_request_data_pointer;
    transfer_request->ux_slave_transfer_request_endpoint =  &device->ux_slave_device_control_endpoint;
    transfer_request->ux_slave_transfer_request_endpoint->ux_slave_endpoint_descriptor.wMaxPacketSize =
            device->ux_slave_device_descriptor.bMaxPacketSize0;
    transfer_request->ux_slave_transfer_request_requested_length = device->ux_slave_device_descriptor.bMaxPacketSize0;
    transfer_request->ux_slave_transfer_request_endpoint = &device->ux_slave_device_control_endpoint;

    /* Create and initialize the default endpoint. */
    dcd->ux_slave_dcd_function(dcd, UX_DCD_CREATE_ENDPOINT, (VOID *)&device->ux_slave_device_control_endpoint);

    device->ux_slave_device_control_endpoint.ux_slave_endpoint_state = UX_ENDPOINT_RESET;
    transfer_request->ux_slave_transfer_request_phase = UX_TRANSFER_PHASE_DATA_IN;

    /* Check if we must call the slave change callback. */
    if(_ux_system_slave->ux_system_slave_change_function != UX_NULL)
    {
        _ux_system_slave->ux_system_slave_change_function(UX_DEVICE_ATTACHED);
    }

    return(UX_SUCCESS);
}


VOID _ux_dcd_am243x_interrupt_thread(ULONG dcd_pointer)
{
    UINT res;
    UX_SLAVE_ENDPOINT *endpoint;
    UX_SLAVE_TRANSFER *transfer_request;
    ULONG remaining_length;
    UX_SLAVE_DCD *dcd;
    _ux_am243x_dcd_msg_t msg;

    while(1) {

        res = tx_queue_receive(&_ux_dcd_am243x_queue, &msg, TX_WAIT_FOREVER);
        if(res != TX_SUCCESS) {
            return;
        }

        if (msg.type == UX_DCD_AM243X_MSG_SETUP) {
            UX_SLAVE_DEVICE *device;
            UX_SLAVE_ENDPOINT *ep;
            UX_SLAVE_TRANSFER *transfer_request;
            UCHAR *data_pointer;
            UINT res;

            if(msg.status != 0u) {
                UX_DCD_LOG("[interrupt_thread] Transfer Error %d\r\n", msg.status);
            }

            device = &_ux_system_slave->ux_system_slave_device;

            ep = &device->ux_slave_device_control_endpoint;

            transfer_request = &ep->ux_slave_endpoint_transfer_request;

            data_pointer = transfer_request->ux_slave_transfer_request_setup;

            memcpy(data_pointer, &msg.setup, sizeof(CH9_UsbSetup));

            transfer_request->ux_slave_transfer_request_actual_length = 8u;

            transfer_request->ux_slave_transfer_request_type = UX_TRANSFER_PHASE_SETUP;

            transfer_request->ux_slave_transfer_request_completion_code = UX_SUCCESS;

            _ux_device_stack_control_request_process(transfer_request);

            if (((*transfer_request -> ux_slave_transfer_request_setup & UX_REQUEST_IN) == 0) &&
                    *(transfer_request -> ux_slave_transfer_request_setup + 6) == 0 &&
                    *(transfer_request -> ux_slave_transfer_request_setup + 7) == 0) {

                transfer_request->ux_slave_transfer_request_phase = UX_TRANSFER_PHASE_DATA_OUT;
                transfer_request->ux_slave_transfer_request_requested_length = 0u;
                transfer_request->ux_slave_transfer_request_actual_length = 0u;

                res = _ux_dcd_am243x_transfer_request(dcd, transfer_request);
                if(res != UX_SUCCESS) {
                    UX_DCD_LOG("[interrupt_thread] _ux_dcd_am243x_transfer_request returned %u.\r\n", res);
                    return;
                }
            } else {
                if ((*transfer_request -> ux_slave_transfer_request_setup & UX_REQUEST_IN) == 0) {
                    transfer_request->ux_slave_transfer_request_phase = UX_TRANSFER_PHASE_DATA_IN;
                    transfer_request->ux_slave_transfer_request_requested_length = 64u;
                    transfer_request->ux_slave_transfer_request_actual_length = 0u;

                    res = _ux_dcd_am243x_transfer_request(dcd, transfer_request);
                    if(res != UX_SUCCESS) {
                        UX_DCD_LOG("[interrupt_thread] _ux_dcd_am243x_transfer_request returned %u.\r\n", res);
                        return;
                    }
                } else {

                }

            }

        } else if (msg.type == UX_DCD_AM243X_MSG_EP0IN) {

            if(msg.status != 0u) {
                UX_DCD_LOG("[interrupt_thread] Transfer Error %d\r\n", msg.status);
            }

            dcd = &_ux_system_slave->ux_system_slave_dcd;

            if((msg.ep_addr & 0x0Fu) == 0u) {
                endpoint = g_ep0;
            } else {
                UX_DCD_LOG("[interrupt_thread] Invalid endpoint.\r\n");
                return;
            }

            if(endpoint == NULL) {
                UX_DCD_LOG("[interrupt_thread] Invalid endpoint.\r\n");
                return;
            }

            if((endpoint->ux_slave_endpoint_descriptor.bmAttributes & UX_MASK_ENDPOINT_TYPE) != UX_CONTROL_ENDPOINT) {
                UX_DCD_LOG("[interrupt_thread] Invalid endpoint.\r\n");
                return;
            }

            transfer_request = &endpoint->ux_slave_endpoint_transfer_request;

            if(msg.status != 0u) {
                transfer_request->ux_slave_transfer_request_completion_code = UX_TRANSFER_STALLED;
            } else {
                transfer_request->ux_slave_transfer_request_completion_code = UX_SUCCESS;
            }

            /*
             * The Cadence driver doesn't handle endpoint 0 control messages greater than 64 bytes.
             * Here we check if the last transfer was a short packet or if there is an error.
             * Otherwise if there is more data to transfer we queue in the remaining data.
             */
            if(((msg.actual < endpoint->ux_slave_endpoint_descriptor.wMaxPacketSize) || (msg.status != 0u)) ||
                    ((transfer_request->ux_slave_transfer_request_actual_length + msg.actual) >= transfer_request->ux_slave_transfer_request_requested_length)) {
                transfer_request->ux_slave_transfer_request_actual_length = transfer_request->ux_slave_transfer_request_requested_length;
            } else {
                transfer_request->ux_slave_transfer_request_actual_length += msg.actual;
                remaining_length = transfer_request->ux_slave_transfer_request_requested_length - transfer_request->ux_slave_transfer_request_actual_length;

                if(remaining_length > 0u) {
                    transfer_request->ux_slave_transfer_request_current_data_pointer += msg.actual;
                    _ux_dcd_am243x_transfer_request(dcd, transfer_request);
                } else {
                    /* This should not happen. */
                    UX_DCD_LOG("[interrupt_thread] Unfinished transfer with no data left\r\n");
                    return;
                }
            }
        } else if (msg.type == UX_DCD_AM243X_MSG_EPIN) {
            UX_SLAVE_ENDPOINT *ep;
            UX_SLAVE_TRANSFER *transfer_request;
            UINT ix;

            if(msg.status != 0u) {
                UX_DCD_LOG("[interrupt_thread] Transfer Error %d\r\n", msg.status);
            }

            ix = msg.ep_addr & 0xFu;

            if(ix >= 8u) {
                UX_DCD_LOG("[interrupt_thread] Invalid index.\r\n");
                return;
            }

            if(ix == 0u) {
                UX_DCD_LOG("[interrupt_thread] Invalid index.\r\n");
                return;
            }

            extern UX_SLAVE_ENDPOINT *g_ep_in[8u];
            ep = g_ep_in[ix - 1u];
            if(ep == NULL) {
                UX_DCD_LOG("[interrupt_thread] Invalid endpoint.\r\n");
                return;
            }

            transfer_request = &ep->ux_slave_endpoint_transfer_request;

            if(msg.status != 0u) {
                transfer_request->ux_slave_transfer_request_completion_code = UX_TRANSFER_STALLED;
            } else {
                transfer_request->ux_slave_transfer_request_completion_code = UX_SUCCESS;
            }

            transfer_request->ux_slave_transfer_request_actual_length += msg.actual;
            transfer_request->ux_slave_transfer_request_status = UX_TRANSFER_STATUS_COMPLETED;

            _ux_utility_semaphore_put(&transfer_request -> ux_slave_transfer_request_semaphore);

        } else if (msg.type == UX_DCD_AM243X_MSG_EPOUT) {
            UX_SLAVE_ENDPOINT *ep;
            UX_SLAVE_TRANSFER *transfer_request;
            UINT ix;

            if(msg.status != 0u) {
                UX_DCD_LOG("[interrupt_thread] Transfer Error %d\r\n", msg.status);
            }

            ix = msg.ep_addr & 0xFu;

            if(ix >= 8u) {
                UX_DCD_LOG("[interrupt_thread] Invalid index.\r\n");
                return;
            }

            if(ix == 0u) {
                UX_DCD_LOG("[interrupt_thread] Invalid index.\r\n");
                return;
            }

            extern UX_SLAVE_ENDPOINT *g_ep_out[8u];
            ep = g_ep_out[ix - 1u];
            if(ep == NULL) {
                UX_DCD_LOG("[interrupt_thread] Invalid endpoint.\r\n");
                return;
            }

            UX_DCD_LOG("[interrupt_thread] EP %d OUT len %u status %u\r\n", msg.ep_addr, msg.actual, msg.status);

            transfer_request = &ep->ux_slave_endpoint_transfer_request;

            if(msg.status != 0u) {
                transfer_request->ux_slave_transfer_request_completion_code = UX_TRANSFER_STALLED;
            } else {
                transfer_request->ux_slave_transfer_request_completion_code = UX_SUCCESS;
            }

            transfer_request->ux_slave_transfer_request_actual_length += msg.actual;
            transfer_request->ux_slave_transfer_request_status = UX_TRANSFER_STATUS_COMPLETED;

            _ux_utility_semaphore_put(&transfer_request -> ux_slave_transfer_request_semaphore);

        } else {
            UX_DCD_LOG("[interrupt_thread] Invalid message type.\r\n");
        }

    }

}




/* USBX device driver entry function. */
UINT _ux_dcd_am243x_function(UX_SLAVE_DCD *dcd, UINT function, VOID *parameter)
{
    UINT status;


    if (dcd->ux_slave_dcd_status == UX_UNUSED)
    {
        return(UX_CONTROLLER_UNKNOWN);
    }


    /* Look at the function and route it.  */
    switch(function)
    {

    case UX_DCD_GET_FRAME_NUMBER:
        status = _ux_dcd_am243x_frame_number_get(dcd, (ULONG *)parameter);
        break;

    case UX_DCD_TRANSFER_REQUEST:

        status = _ux_dcd_am243x_transfer_request(dcd, (UX_SLAVE_TRANSFER *)parameter);
        break;

    case UX_DCD_TRANSFER_ABORT:

        status = _ux_dcd_am243x_transfer_abort(dcd, (UX_SLAVE_TRANSFER *) parameter);
        break;

    case UX_DCD_CREATE_ENDPOINT:

        status = _ux_dcd_am243x_endpoint_create(dcd, (UX_SLAVE_ENDPOINT *)parameter);
        break;

    case UX_DCD_DESTROY_ENDPOINT:

        status =  _ux_dcd_am243x_endpoint_destroy(dcd, (UX_SLAVE_ENDPOINT *)parameter);
        break;

    case UX_DCD_RESET_ENDPOINT:

        status = _ux_dcd_am243x_endpoint_stall_clear(dcd, (UX_SLAVE_ENDPOINT *)parameter);
        break;

    case UX_DCD_STALL_ENDPOINT:
        status = _ux_dcd_am243x_endpoint_stall(dcd, (UX_SLAVE_ENDPOINT *)parameter);

        break;

    case UX_DCD_SET_DEVICE_ADDRESS:

        status = _ux_dcd_am243x_address_set(dcd, (ULONG)parameter);

        break;

    case UX_DCD_CHANGE_STATE:

        status = _ux_dcd_am243x_state_change(dcd, (ULONG)parameter);

        break;

    case UX_DCD_ENDPOINT_STATUS:

        status = _ux_dcd_am243x_endpoint_status(dcd, (ULONG)parameter);
        break;

    default:
        status =  UX_FUNCTION_NOT_SUPPORTED;
        break;

    }

    /* Return completion status.  */
    return(status);
}


static UINT _ux_dcd_am243x_frame_number_get(UX_SLAVE_DCD *dcd, ULONG *frame_number)
{
    CUSBD_Dev *dev;
    uint32_t res;
    uint32_t frame_num;

    (void)dcd;

    UX_DCD_LOG("[frame_number_get] Frame number get.\r\n");

    /* Validate the address. */
    if (frame_number == NULL) {
        return UX_ERROR;
    }

    /* Fetch the controller instance. */
    usb_handle.drv->getDevInstance(usb_handle.pD, &dev);
    if(dev == NULL) {
        UX_DCD_LOG("Controller not initialized.\r\n");
        return UX_ERROR;
    }

    res = CUSBD_DGetFrame(usb_handle.pD, &frame_num);
    if(res != 0u) {
        return UX_ERROR;
    }

    *frame_number = frame_num;

    return UX_SUCCESS;
}


static UINT _ux_dcd_am243x_transfer_request(UX_SLAVE_DCD *dcd, UX_SLAVE_TRANSFER *transfer_request)
{
    ULONG transfer_length;
    ULONG endpoint_address;
    UCHAR *transfer_buffer;
    UX_SLAVE_ENDPOINT *endpoint;
    ULONG transfer_phase;
    CUSBD_Dev *dev;
    CUSBD_Ep *ep;
    int32_t reqIdx = 0;
    int32_t res = 0;


    endpoint =  transfer_request->ux_slave_transfer_request_endpoint;
    endpoint_address = endpoint->ux_slave_endpoint_descriptor.bEndpointAddress & ~UX_ENDPOINT_DIRECTION;
    transfer_phase = transfer_request->ux_slave_transfer_request_phase;

    transfer_length = transfer_request->ux_slave_transfer_request_requested_length - transfer_request->ux_slave_transfer_request_actual_length;

    transfer_buffer = transfer_request->ux_slave_transfer_request_current_data_pointer;


    /* Get the driver's endpoint. */
    usb_handle.drv->getDevInstance(usb_handle.pD, &dev);

    ep = (CUSBD_Ep *) getEpFromAddr(dev, endpoint->ux_slave_endpoint_descriptor.bEndpointAddress);
    if(ep == NULL) {
        UX_DCD_LOG("[transfer_request] Invalid endpoint\r\n");
        return UX_ERROR;
    }

    if((endpoint_address == 0u) && (transfer_phase == UX_TRANSFER_PHASE_DATA_IN)) {
        if(transfer_length == 0u) {

            UX_DCD_LOG("[transfer_request] EP0 IN 0 length transfer request\r\n", res);

            /* Everything related to a zero length transfer on EP0 OUT is handled by the low level cadence driver. Return success. */
            return UX_SUCCESS;
        } else {
            memset(&ep0DataXferRequest, 0, sizeof(CUSBD_Req));
            ep0DataXferRequest.length = transfer_length;

            ep0DataXferRequest.buf = transfer_buffer;
            ep0DataXferRequest.dma =  (uintptr_t)buffEp0;
            ep0DataXferRequest.complete = _ux_am234x_ep0_out_xfer_cb;
            ep0DataXferRequest.deferStatusStage = 1;
            usb_handle.pD->ep0NextState = CH9_EP0_DATA_PHASE;
            usb_handle.pD->ep0DataDirFlag = 0;

            UX_DCD_LOG("DATA IN EP %u - len %u\r\n", endpoint->ux_slave_endpoint_descriptor.bEndpointAddress, transfer_length);

            res = ep->ops->reqQueue(usb_handle.pD, ep, &ep0DataXferRequest);
            if(res != 0u)
            {
                UX_DCD_LOG("Error queuing request with error %d\r\n", res);
                return UX_ERROR;
            }
        }
    } else if ((endpoint_address == 0u) && (transfer_phase == UX_TRANSFER_PHASE_DATA_OUT)) {
        ep0Req.length = transfer_length;
        ep0Req.buf = ep0Buff;
        ep0Req.dma = (uintptr_t)(ep0Req.buf);
        ep0Req.complete = _ux_am234x_ep0_in_xfer_cb;

        usb_handle.pD->ep0DataDirFlag = 1u;

        if(transfer_length > 0u) {
            memcpy(ep0Buff, transfer_buffer, transfer_length);
        }

        if (ep0Req.length > 0u) {
          usb_handle.pD->ep0NextState = CH9_EP0_DATA_PHASE;
        } else {
          usb_handle.pD->ep0NextState = CH9_EP0_STATUS_PHASE;
        }

        UX_DCD_LOG("DATA OUT EP %u - len %u\r\n", endpoint->ux_slave_endpoint_descriptor.bEndpointAddress, transfer_length);
        res = ep->ops->reqQueue(usb_handle.pD, ep, &ep0Req);
        if(res != 0u)
        {
            UX_DCD_LOG("Error queuing request with error %d\r\n", res);
            return UX_ERROR;
        }
    } else if (transfer_phase == UX_TRANSFER_PHASE_DATA_OUT) {

        reqIdx = endpoint_address - 1u;
        if ((reqIdx < 0 ) || (reqIdx >= DATA_XFER_BUFFER_COUNT))
        {
            UX_DCD_LOG("[transfer_request] Invalid request index.\r\n", res);
            return UX_ERROR;
        }

        /* use DataXferRequestIn[reqIdx] for EP IN */
        memset(&DataXferRequestsIn[reqIdx], 0, sizeof(CUSBD_Req));
        DataXferRequestsIn[reqIdx].length = transfer_length;
        DataXferRequestsIn[reqIdx].buf = dataXferBufferIn[reqIdx];
        DataXferRequestsIn[reqIdx].dma = (uintptr_t)(dataXferBufferIn[reqIdx]);
        DataXferRequestsIn[reqIdx].complete = _ux_am234x_ep_in_xfer_cb;

        if(transfer_length > 0u) {
            memcpy(dataXferBufferIn[reqIdx], transfer_buffer, transfer_length);
        }

        UX_DCD_LOG("DATA OUT EP %u - len %u\r\n", endpoint->ux_slave_endpoint_descriptor.bEndpointAddress, transfer_length);

        res = ep->ops->reqQueue(usb_handle.pD, ep, &DataXferRequestsIn[reqIdx]);
        if (res != 0)
        {
            UX_DCD_LOG("Error queuing request with error %d\r\n", res);
            return UX_ERROR;
        }

        /* Wait for the transfer to be complete. */
        res =  _ux_utility_semaphore_get(&transfer_request -> ux_slave_transfer_request_semaphore, UX_WAIT_FOREVER);
        if (res != UX_SUCCESS) {
            return UX_ERROR;
        }

        /* Check transfer status and return an error if it failed. */
        if (transfer_request -> ux_slave_transfer_request_completion_code != UX_SUCCESS) {
                return (transfer_request -> ux_slave_transfer_request_completion_code);
        }

    } else if (transfer_phase == UX_TRANSFER_PHASE_DATA_IN) {

        reqIdx = (ep->address & 0x0F) - 1;

        if ((reqIdx < 0)||(reqIdx >= DATA_XFER_BUFFER_COUNT))
        {
            UX_DCD_LOG("[transfer_request] Invalid request index.\r\n", res);
            return UX_ERROR;
        }

        UX_DCD_LOG("DATA IN EP %u - len %u\r\n", endpoint->ux_slave_endpoint_descriptor.bEndpointAddress, transfer_length);

        memset(&DataXferRequestsOut[reqIdx], 0, sizeof(CUSBD_Req));

        DataXferRequestsOut[reqIdx].buf = transfer_buffer;
        DataXferRequestsOut[reqIdx].dma = (uintptr_t)(dataXferBufferOut[reqIdx]);
        DataXferRequestsOut[reqIdx].complete = _ux_am234x_ep_out_xfer_cb;
        DataXferRequestsOut[reqIdx].length = transfer_length;

        res = ep->ops->reqQueue(usb_handle.pD, ep, &DataXferRequestsOut[reqIdx]);
        if (res != 0)
        {
            UX_DCD_LOG("Error queuing request with error %d\r\n", res);
            return UX_ERROR;
        }

        /* Wait for the transfer to be complete. */
        res = _ux_utility_semaphore_get(&transfer_request->ux_slave_transfer_request_semaphore, UX_WAIT_FOREVER);
        if (res != UX_SUCCESS) {
            return UX_ERROR;
        }

        /* Check transfer status and return an error if it failed. */
        if (transfer_request->ux_slave_transfer_request_completion_code != UX_SUCCESS) {
                return (transfer_request->ux_slave_transfer_request_completion_code);
        }
    } else {
        UX_DCD_LOG("[transfer_request] Invalid transfer request.\r\n", res);
        return UX_ERROR;
    }

    return UX_SUCCESS;
}


static UINT _ux_dcd_am243x_transfer_abort(UX_SLAVE_DCD *dcd, UX_SLAVE_TRANSFER *transfer_request)
{
    UX_SLAVE_ENDPOINT *endpoint;
    CUSBD_Dev *dev;
    CUSBD_Ep *ep;
    ULONG endpoint_address;
    ULONG transfer_phase;
    CUSBD_Req *req;
    int32_t reqIdx = 0;
    int32_t res = 0;

    (void)dcd;

    if(transfer_request == NULL) {
        return UX_ERROR;
    }

    endpoint =  transfer_request->ux_slave_transfer_request_endpoint;
    if(endpoint == NULL) {
        UX_DCD_LOG("[transfer_abort] Invalid endpoint\r\n");
        return UX_ERROR;
    }

    endpoint_address = endpoint->ux_slave_endpoint_descriptor.bEndpointAddress & ~UX_ENDPOINT_DIRECTION;
    transfer_phase = transfer_request->ux_slave_transfer_request_phase;

    /* Get the driver's endpoint. */
    usb_handle.drv->getDevInstance(usb_handle.pD, &dev);

    ep = (CUSBD_Ep *)getEpFromAddr(dev, endpoint->ux_slave_endpoint_descriptor.bEndpointAddress);
    if(ep == NULL) {
        UX_DCD_LOG("[transfer_abort] Invalid endpoint\r\n");
        return UX_ERROR;
    }

    if((endpoint_address == 0u) && (transfer_phase == UX_TRANSFER_PHASE_DATA_IN)) {
        if(transfer_request->ux_slave_transfer_request_requested_length == 0u) {
            /* Nothing to do here. */
            return UX_SUCCESS;
        }

        req = &ep0DataXferRequest;

    } else if ((endpoint_address == 0u) && (transfer_phase == UX_TRANSFER_PHASE_DATA_OUT)) {

        req = &ep0Req;

    } else if (transfer_phase == UX_TRANSFER_PHASE_DATA_OUT) {

        reqIdx = endpoint_address - 1u;
        if ((reqIdx < 0 ) || (reqIdx >= DATA_XFER_BUFFER_COUNT))
        {
            UX_DCD_LOG("[transfer_request] Invalid request index.\r\n", res);
            return UX_ERROR;
        }

        req = &DataXferRequestsIn[reqIdx];

    } else if (transfer_phase == UX_TRANSFER_PHASE_DATA_IN) {

        reqIdx = (ep->address & 0x0F) - 1;

        if ((reqIdx < 0)||(reqIdx >= DATA_XFER_BUFFER_COUNT))
        {
            UX_DCD_LOG("[transfer_request] Invalid request index.\r\n", res);
            return UX_ERROR;
        }

        req = &DataXferRequestsOut[reqIdx];

    } else {
        UX_DCD_LOG("[transfer_abort] Invalid transfer request.\r\n", res);
        return UX_ERROR;
    }

    res = ep->ops->reqDequeue(usb_handle.pD, ep, req);
    if (res != 0)
    {
        UX_DCD_LOG("Error dequeuing request with error %d\r\n", res);
        return UX_ERROR;
    }

    return UX_SUCCESS;
}



UINT _ux_dcd_am243x_endpoint_create(UX_SLAVE_DCD *dcd, UX_SLAVE_ENDPOINT *endpoint)
{
    UCHAR endpoint_address;
    UCHAR endpoint_direction;
    CUSBD_Dev * dev;
    CUSBD_Ep * ep;
    int32_t cusbd_res;


    UX_DCD_LOG("[endpoint create] Endpoint create %u\r\n", endpoint->ux_slave_endpoint_descriptor.bEndpointAddress);

    endpoint_address = endpoint->ux_slave_endpoint_descriptor.bEndpointAddress & ~UX_ENDPOINT_DIRECTION;
    endpoint_direction = endpoint->ux_slave_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION;

    /*
     * Endpoint 0 is handled differently and is bi-directional. Just save the endpoint structure
     * for future reference. The Cadence driver always initialized Endpoint 0 automatically.
     */
    if(endpoint_address == 0u) {
        if(g_ep0 != NULL) {
            UX_DCD_LOG("Initialization error.\r\n");
            return UX_ERROR;
        }

        g_ep0 = endpoint;
        ep0_status = UX_DCD_AM243X_EP_STATUS_ENABLED;

        return UX_SUCCESS;
    }

    if(endpoint_address >= UX_SLAVE_EP_COUNT) {
        UX_DCD_LOG("Invalid endpoint address.\r\n");
        return UX_ERROR;
    }

    /* Save the endpoint structure reference. */
    if(endpoint_direction != 0u) {
        if(g_ep_in[endpoint_address - 1u] != NULL) {
            UX_DCD_LOG("Endpoint already created.\r\n");
            return UX_ERROR;
        }

        g_ep_in[endpoint_address - 1u] = endpoint;
        ep_in_status[endpoint_address - 1u] = UX_DCD_AM243X_EP_STATUS_ENABLED;
    } else {
        if(g_ep_out[endpoint_address - 1u] != NULL) {
            UX_DCD_LOG("Endpoint already created.\r\n");
            return UX_ERROR;
        }

        g_ep_out[endpoint_address - 1u] = endpoint;
        ep_out_status[endpoint_address - 1u] = UX_DCD_AM243X_EP_STATUS_ENABLED;
    }

    /* Enable the endpoint at the driver and hardware level. */
    usb_handle.drv->getDevInstance(usb_handle.pD, &dev);

    ep = (CUSBD_Ep *)getEpFromAddr(dev, endpoint->ux_slave_endpoint_descriptor.bEndpointAddress);
    if(ep == NULL) {
        UX_DCD_LOG("Endpoint not found.\r\n");
        return UX_ERROR;
    }

    cusbd_res = ep->ops->epEnable(usb_handle.pD, ep, (uint8_t *) &endpoint->ux_slave_endpoint_descriptor);
    if(cusbd_res != 0) {
        UX_DCD_LOG("epEnable returned error %d.\r\n", cusbd_res);
        return UX_ERROR;
    }

    return UX_SUCCESS;

}


static UINT _ux_dcd_am243x_endpoint_destroy(UX_SLAVE_DCD *dcd, UX_SLAVE_ENDPOINT *endpoint)
{
    UCHAR endpoint_address;
    UCHAR endpoint_direction;
    CUSBD_Dev * dev;
    CUSBD_Ep * ep;
    int32_t cusbd_res;

    (void)dcd;

    UX_DCD_LOG("[endpoint destroy] Endpoint destroy %u\r\n", endpoint->ux_slave_endpoint_descriptor.bEndpointAddress);

    endpoint_address = endpoint->ux_slave_endpoint_descriptor.bEndpointAddress & ~UX_ENDPOINT_DIRECTION;
    endpoint_direction = endpoint->ux_slave_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION;

    /* Endpoint 0 is always active. */
    if(endpoint_address == 0u) {
        return UX_SUCCESS;
    }

    if(endpoint_address >= UX_SLAVE_EP_COUNT) {
        UX_DCD_LOG("Invalid endpoint address.\r\n");
        return UX_ERROR;
    }

    if(endpoint_direction != 0u) {
        if(g_ep_in[endpoint_address - 1u] == NULL) {
            UX_DCD_LOG("Endpoint not created.\r\n");
        }

        g_ep_in[endpoint_address - 1u] = NULL;
        ep_in_status[endpoint_address - 1u] = UX_DCD_AM243X_EP_STATUS_DISABLED;
    } else {
        if(g_ep_out[endpoint_address - 1u] == NULL) {
            UX_DCD_LOG("Endpoint not created.\r\n");
        }

        g_ep_out[endpoint_address - 1u] = NULL;
        ep_out_status[endpoint_address - 1u] = UX_DCD_AM243X_EP_STATUS_DISABLED;
    }

    /* Disable the endpoint at the driver and hardware level. */
    usb_handle.drv->getDevInstance(usb_handle.pD, &dev);

    ep = (CUSBD_Ep *)getEpFromAddr(dev, endpoint->ux_slave_endpoint_descriptor.bEndpointAddress);
    if(ep == NULL) {
        UX_DCD_LOG("Endpoint not found.\r\n");
        return UX_ERROR;
    }

    cusbd_res = ep->ops->epDisable(usb_handle.pD, ep);
    if(cusbd_res != 0) {
        UX_DCD_LOG("epEnable returned error %d.\r\n", cusbd_res);
        return UX_ERROR;
    }

    return UX_SUCCESS;
}


static UINT _ux_dcd_am243x_endpoint_stall_clear(UX_SLAVE_DCD *dcd, UX_SLAVE_ENDPOINT *endpoint)
{
    ULONG endpoint_address;
    ULONG endpoint_direction;
    CUSBD_Dev *dev;
    CUSBD_Ep *ep;
    int32_t res;

    (void)dcd;

    if(endpoint == NULL) {
        return UX_ERROR;
    }

    UX_DCD_LOG("[endpoint stall clear] Endpoint stall clear %u\r\n", endpoint->ux_slave_endpoint_descriptor.bEndpointAddress);

    endpoint_address = endpoint->ux_slave_endpoint_descriptor.bEndpointAddress & ~UX_ENDPOINT_DIRECTION;
    endpoint_direction = endpoint->ux_slave_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION;

    if(endpoint_address >= UX_SLAVE_EP_COUNT) {
        UX_DCD_LOG("Invalid endpoint address.\r\n");
        return UX_ERROR;
    }

    if(endpoint_address == 0u) {
        ep0_status = UX_DCD_AM243X_EP_STATUS_ENABLED;
    } else {
        if(endpoint_direction != 0u) {
            if(g_ep_in[endpoint_address - 1u] == NULL) {
                UX_DCD_LOG("Endpoint not created.\r\n");
                return UX_ERROR;
            }

            ep_in_status[endpoint_address - 1u] = UX_DCD_AM243X_EP_STATUS_ENABLED;
        } else {
            if(g_ep_out[endpoint_address - 1u] == NULL) {
                UX_DCD_LOG("Endpoint not created.\r\n");
                return UX_ERROR;
            }

            ep_out_status[endpoint_address - 1u] = UX_DCD_AM243X_EP_STATUS_ENABLED;
        }
    }

    usb_handle.drv->getDevInstance(usb_handle.pD, &dev);

    ep = (CUSBD_Ep *)getEpFromAddr(dev, endpoint->ux_slave_endpoint_descriptor.bEndpointAddress);
    if(ep == NULL) {
        UX_DCD_LOG("Endpoint not found.\r\n");
        return UX_ERROR;
    }

    res = usb_handle.drv->epSetHalt(usb_handle.pD, ep, 0);
    if(res != 0) {
        return UX_ERROR;
    }

    return UX_SUCCESS;
}


static UINT _ux_dcd_am243x_endpoint_stall(UX_SLAVE_DCD *dcd, UX_SLAVE_ENDPOINT *endpoint)
{
    ULONG endpoint_address;
    ULONG endpoint_direction;
    CUSBD_Dev *dev;
    CUSBD_Ep *ep;
    int32_t res;

    (void)dcd;

    if(endpoint == NULL) {
        return UX_ERROR;
    }

    UX_DCD_LOG("[endpoint stall] Endpoint stall %u\r\n", endpoint->ux_slave_endpoint_descriptor.bEndpointAddress);

    endpoint_address = endpoint->ux_slave_endpoint_descriptor.bEndpointAddress & ~UX_ENDPOINT_DIRECTION;
    endpoint_direction = endpoint->ux_slave_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION;

    if(endpoint_address >= UX_SLAVE_EP_COUNT) {
        UX_DCD_LOG("Invalid endpoint address.\r\n");
        return UX_ERROR;
    }

    if(endpoint_address == 0u) {
        ep0_status = UX_DCD_AM243X_EP_STATUS_STALLED;
    } else {
        if(endpoint_direction != 0u) {
            if(g_ep_in[endpoint_address - 1u] == NULL) {
                UX_DCD_LOG("Endpoint not created.\r\n");
                return UX_ERROR;
            }

            ep_in_status[endpoint_address - 1u] = UX_DCD_AM243X_EP_STATUS_STALLED;
        } else {
            if(g_ep_out[endpoint_address - 1u] == NULL) {
                UX_DCD_LOG("Endpoint not created.\r\n");
                return UX_ERROR;
            }

            ep_out_status[endpoint_address - 1u] = UX_DCD_AM243X_EP_STATUS_STALLED;
        }
    }

    usb_handle.drv->getDevInstance(usb_handle.pD, &dev);

    ep = (CUSBD_Ep *)getEpFromAddr(dev, endpoint->ux_slave_endpoint_descriptor.bEndpointAddress);
    if(ep == NULL) {
        UX_DCD_LOG("Endpoint not found.\r\n");
        return UX_ERROR;
    }

    res = usb_handle.drv->epSetHalt(usb_handle.pD, ep, 1);
    if(res != 0) {
        return UX_ERROR;
    }

    return UX_SUCCESS;
}


static UINT _ux_dcd_am243x_address_set(UX_SLAVE_DCD *dcd, ULONG address)
{
    CUSBD_PrivateData * dev;
    CH9_UsbState state;

    (void)dcd;

    UX_DCD_LOG("[address set] Address Set %u.\r\n", address);

    /* Validate the address. */
    if (address > 0x007FU) {
        UX_DCD_LOG("Invalid address.\r\n");
        return UX_ERROR;
    }

    /* Fetch the controller instance. */
    dev = usb_handle.pD;

    if(dev == NULL) {
        UX_DCD_LOG("Controller not initialized.\r\n");
        return UX_ERROR;
    }

    state = dev->device.state;

    /* Check if device is in the correct state */
    if (state == CH9_USB_STATE_CONFIGURED) {
        UX_DCD_LOG("Invalid controller state.\r\n");
        return UX_ERROR;
    }

    /* Store the device address in the controller. */
    CPS_UncachedWrite32(&dev->reg->USBR_CMD, ((address << 1) | 0x00000001U));

    if (address > 0U) {
        dev->device.state = CH9_USB_STATE_ADDRESS;
    } else {
        dev->device.state = CH9_USB_STATE_DEFAULT;
    }

    return UX_SUCCESS;
}


static UINT _ux_dcd_am243x_state_change(UX_SLAVE_DCD *dcd, ULONG state)
{

    (void)dcd;
    (void)state;

    UX_DCD_LOG("[state change] state %u\r\n", state);

    /* Nothing to do here, return success. */

    return UX_SUCCESS;
}


static UINT _ux_dcd_am243x_endpoint_status(UX_SLAVE_DCD *dcd, ULONG endpoint_index)
{
    UINT endpoint_status = (unsigned)-1;

    (void)dcd;

    UX_DCD_LOG("[endpoint_status] Endpoint status %u\r\n", endpoint_index);

    if(endpoint_index >= UX_SLAVE_EP_COUNT) {
        UX_DCD_LOG("Invalid endpoint address.\r\n");
        return UX_ERROR;
    }

    /* Can't query the status of endpoint 0� */
    if(endpoint_index == 0u) {
        endpoint_status = ep0_status;
    } else {
        if (g_ep_in[endpoint_index - 1u] != NULL) {
            endpoint_status = ep_in_status[endpoint_index - 1u];
        } else if (g_ep_out[endpoint_index - 1u] == NULL) {
                endpoint_status = ep_out_status[endpoint_index - 1u];
        } else {
            endpoint_status = (unsigned)-1;
        }
    }

    if(endpoint_status == (unsigned)-1) {
        return UX_ERROR;
    } else if(endpoint_status == UX_DCD_AM243X_EP_STATUS_STALLED) {
        return UX_TRUE;
    } else {
        return UX_FALSE;
    }

}


static void _ux_am234x_ep0_out_xfer_cb(CUSBD_Ep *ep, CUSBD_Req *req) {
    UX_SLAVE_DEVICE *device;
    UX_SLAVE_ENDPOINT *endpoint;
    UX_SLAVE_TRANSFER *transfer_request;
    UX_SLAVE_DCD *dcd;

    req->complete = 0;

    device = &_ux_system_slave->ux_system_slave_device;
    endpoint = &device->ux_slave_device_control_endpoint;
    transfer_request = &endpoint->ux_slave_endpoint_transfer_request;

    dcd = &_ux_system_slave->ux_system_slave_dcd;

    transfer_request->ux_slave_transfer_request_phase = UX_TRANSFER_PHASE_DATA_OUT;
    transfer_request->ux_slave_transfer_request_requested_length = 0u;
    _ux_dcd_am243x_transfer_request(dcd, transfer_request);

}


static void _ux_am234x_ep0_in_xfer_cb(CUSBD_Ep *ep, CUSBD_Req *req)
{
    _ux_am243x_dcd_msg_t msg;

    memset(&msg, 0u, sizeof(msg));

    msg.type = UX_DCD_AM243X_MSG_EP0IN;
    msg.ep_addr = ep->address;
    msg.actual = req->actual;
    msg.req = req->length;
    msg.status = req->status;

    (void)tx_queue_send(&_ux_dcd_am243x_queue, &msg, TX_NO_WAIT);
}


static void _ux_am234x_ep_in_xfer_cb(CUSBD_Ep *ep, CUSBD_Req *req) {
    _ux_am243x_dcd_msg_t msg;

    memset(&msg, 0u, sizeof(msg));

    msg.type = UX_DCD_AM243X_MSG_EPIN;
    msg.ep_addr = ep->address;
    msg.actual = req->actual;
    msg.req = req->length;
    msg.status = req->status;

    (void)tx_queue_send(&_ux_dcd_am243x_queue, &msg, TX_NO_WAIT);
}


static void _ux_am234x_ep_out_xfer_cb(CUSBD_Ep *ep, CUSBD_Req *req)
{
    _ux_am243x_dcd_msg_t msg;
    int reqIdx;

    reqIdx = usbGetDataXferRequestIndexOut(req);
    if (reqIdx == -1) {
        UX_DCD_LOG("[ep_out_xfer_cb] invalid reqIdx\r\n");
        return;
    } else {
        memcpy(req->buf, (uint8_t *)req->dma, req->actual);
    }

    memset(&msg, 0u, sizeof(msg));

    msg.type = UX_DCD_AM243X_MSG_EPOUT;
    msg.ep_addr = ep->address;
    msg.actual = req->actual;
    msg.req = req->length;
    msg.status = req->status;

    (void)tx_queue_send(&_ux_dcd_am243x_queue, &msg, TX_NO_WAIT);
}
