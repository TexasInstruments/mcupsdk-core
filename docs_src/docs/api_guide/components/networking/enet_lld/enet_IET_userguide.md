ENET IET Frame Pre-emption userguide {#enet_iet_userguide}
=====================================

[TOC]

# Introduction

## IEEE 802.1Qbu IET

CPSW support Intersperse Express Traffic (IET, defined in P802.3br/D2.0 spec which later is included in IEEE 802.3 2018) Frame preemption (inArgs) feature and also to allow MAC layer to verify if the peer supports IET MAC merge layer or not. IET (one of the TSN standards), which defines a mechanism to transmit an express frame with a minimum delay at the expense of delaying completion of normal priority frames.

Ethernet allows any device to send messages at any moment which might cause congestion in network. Messages are queued in a queue to be sent out based on packet priority.  A ethernet frame with high priority can be delayed by a low priority frame in front it, which just happened to arrive a little bit earlier.

IET allows premptable traffic on selected transmit priorities to be prempted by express traffic on selected express priorities. Outgoing Ethernet packets are segregated into Express or Preempt Tx FIFOs according to their priority levels.  Application should enable IET frame preemption on a specific MAC port at init time. When IET is disabled, all Ethernet traffic is express traffic and switch operation is as if IET does not exist.
Traffic is only intended to be moved to the prempt queue after premption is verified and enabled.

The diagram below shows how an express frame with a minimum delay is transmitted at the expense of delaying (preempting) of normal priority frames.

\imageStyle{IET_Schedule_Diagram.png,width:60%}
\image html IET_Schedule_Diagram.png

\note Express traffic is expected to be made on high priority traffic.

# IET Architecture

MAC merge layer is responsible for preempting the transmission of frame from a preemptible queue if there is frame waiting for transmission at a higher priority Express queue. The h/w sends an initial segment of the frame satisfying min fragment size requirement and then schedule frame from the Express queue for transmission. Finally when no more frames available at the Express queue, it will resume transmission of remaining segments of the frame of the preemptible queue which was preempted. At the peer end, the segments are re-assembled and delivered to the MAC interface.

The diagram below shows transmission of express and preemptable traffic at MAC merge layer.

\imageStyle{IET_Architecture.png,width:30%}
\image html IET_Architecture.png

# Enet LLD API

Enet LLD provides support for IET through the IOCTLs described in the API Guide. The most important IOCTLs for any application using IET will require are listed below (in order of usage):
IET configuration can be set through syscfg-gui in Enet (CPSW) section under category `MAC config` with sub-category as `IET Configuration`.

Syscfg GUI for enabling IET with configuration parameters:

  \imageStyle{IET_syscfg.png,width:80%}
  \image html IET_syscfg.png

\note Enable Hostport configuration **hostPortCfg->rxVlanRemapEn** to map different priority TX packets to differnt TX queues.

\imageStyle{IET_rx_Vlan_remap.png,width:60%}
\image html IET_rx_Vlan_remap.png


- **EnableIET** - Checkbox to enable IET frame preemption on a specific MAC port by calling IOCTL **ENET_MACPORT_IOCTL_ENABLE_PREEMPTION**:
    \code
    /* Enable preemption */
    EnetMacPort_GenericInArgs inArgs;
    Enet_IoctlPrms prms;
    inArgs.macPort = ENET_MAC_PORT_1;
    ENET_IOCTL_SET_IN_ARGS(&prms, &inArgs);
    Enet_ioctl(hEnet, coreId, ENET_MACPORT_IOCTL_ENABLE_PREEMPTION, &prms);
    \endcode

- **Enable IET Macport Verification** - Checkbox for application to enable preemption verification using IOCTL **ENET_MACPORT_IOCTL_ENABLE_PREEMPT_VERIFICATION**. It should verify IET status **ENET_MACPORT_IOCTL_GET_PREEMPT_VERIFY_STATUS** upon link-up event. Application should take into account that link speed can change on every link-up on a specific MAC port.

  This is a demo function to poll the verify status and restart verification. Application needs to call this after link-up.
    \code
    void EnetApp_doIetVerification(Enet_Handle hEnet,
                                    uint32_t coreId,
                                    Enet_MacPort macPort)
    {
        uint32_t try = ENETAPP_NUM_IET_VERIFY_ATTEMPTS;
        int32_t status = ENET_SOK;
        EnetMacPort_GenericInArgs fpe;
        Enet_IoctlPrms prms;
        EnetMacPort_PreemptVerifyStatus verifyStatus;

        fpe.macPort = macPort;
        do{
            ENET_IOCTL_SET_IN_ARGS(&prms, &fpe);
            status = Enet_ioctl(hEnet, coreId, ENET_MACPORT_IOCTL_ENABLE_PREEMPT_VERIFICATION , &prms);
            if (status != ENET_SOK)
            {
                EnetAppUtils_print("Failed to start IET verification: %d\r\n", status);
                break;
            }
            /*
            * Takes 10 msec to complete this in h/w assuming other
            * side is already ready. However since both side might
            * take variable setup/config time, need to Wait for
            * additional time. Chose 50 msec through trials
            */
            TaskP_sleep(50U);
            ENET_IOCTL_SET_INOUT_ARGS(&prms, &fpe, &verifyStatus);
            status = Enet_ioctl(hEnet, coreId, ENET_MACPORT_IOCTL_GET_PREEMPT_VERIFY_STATUS , &prms);
            if (status != ENET_SOK)
            {
                EnetAppUtils_print("Failed to read IET verify status : %d\r\n", status);
                break;
            }
            if(verifyStatus == ENET_MAC_VERIFYSTATUS_SUCCEEDED)
            {
                EnetAppUtils_print("IET verify Success \r\n");
                break;
            }
            else if(verifyStatus == ENET_MAC_VERIFYSTATUS_FAILED )
            {
                EnetAppUtils_print("IET verify failed, trying again \r\n");
            }
            else
            {
                EnetAppUtils_print("Unexpected IET Failure : %d \r\n", verifyStatus);
                break;
            }
            try--;
        } while(try > 0);

        if(try == 0){
            EnetAppUtils_print("IET verify timeout \r\n");
        }
    }
    \endcode

Link partner preemption capability is discovered through LLDP. It verifies that link between 2 ports is able to carry preemptable packets. Networks that are fixed by design can disable IET verification (i.e. forced at HW).

- **IET Minimum Fragments** - Integer value to set minimum fragment size using the IOCTL **ENET_MACPORT_IOCTL_SET_PREEMPT_MIN_FRAG_SIZE**. As preemption isn't instantaneous, packets smaller than this size will NOT be preempted.
    \code
    EnetMacPort_SetPreemptMinFragSizeInArgs fragSizeInArgs;
    Enet_IoctlPrms prms;
    fragSizeInArgs.macPort = ENET_MAC_PORT_1;
    /*! preemptMinFragSize is addFragSize as defined in IEEE 802.3br
     * 64*(1  addFragSize) becomes the minimum fragment size */
    fragSizeInArgs.preemptMinFragSize = 1U;
    ENET_IOCTL_SET_IN_ARGS(&prms, &fragSizeInArgs);
    status = Enet_ioctl(hEnet, coreId, ENET_MACPORT_IOCTL_SET_PREEMPT_MIN_FRAG_SIZE, &prms);
    \endcode

- **IET Queue Fragments** - **ENET_MACPORT_IOCTL_SET_PREEMPT_QUEUE** is used to set macport queue preemption mode (i.e. to identify/separate preemptable and express traffic). In the given example code below we are setting traffic with priority > 4 as express and less as preemptable traffic.
    \code
    EnetMacPort_SetPreemptQueueInArgs queuePreemptInArgs;
    queuePreemptInArgs.macPort = ENET_MAC_PORT_1;
    for(uint32_t i = 0U; i < 8U; i++)
    {
        if (i > 4)
        {
            queuePreemptInArgs.queuePreemptCfg.preemptMode[i] = ENET_MAC_QUEUE_PREEMPT_MODE_EXPRESS;
        }
        else
        {
            queuePreemptInArgs.queuePreemptCfg.preemptMode[i] = ENET_MAC_QUEUE_PREEMPT_MODE_PREEMPT;
        }
    }
    ENET_IOCTL_SET_IN_ARGS(&prms, &queuePreemptInArgs);
    status = Enet_ioctl(hEnet, coreId, ENET_MACPORT_IOCTL_SET_PREEMPT_QUEUE, &prms);
    \endcode

- **ENET_MACPORT_IOCTL_IET_HOLD_PREEMPT_TRAFFIC** is used to hold preemptable traffic on a specific MAC port and only allow express traffic to flow.
    \code
    EnetMacPort_GenericInArgs inArgs;
    Enet_IoctlPrms prms;
    inArgs.macPort = ENET_MAC_PORT_1;
    ENET_IOCTL_SET_IN_ARGS(&prms, &inArgs);
    status = Enet_ioctl(hEnet, coreId, ENET_MACPORT_IOCTL_IET_HOLD_PREEMPT_TRAFFIC, &prms);
    \endcode

- **ENET_MACPORT_IOCTL_IET_RELEASE_PREEMPT_TRAFFIC** is used to release preemptable traffic.
    \code
    EnetMacPort_GenericInArgs inArgs;
    Enet_IoctlPrms prms;
    inArgs.macPort = ENET_MAC_PORT_1;
    ENET_IOCTL_SET_IN_ARGS(&prms, &inArgs);
    status = Enet_ioctl(hEnet, coreId, ENET_MACPORT_IOCTL_IET_RELEASE_PREEMPT_TRAFFIC, &prms);
    \endcode

# Limitations

- IET verification will fail for RMII (not supported).
