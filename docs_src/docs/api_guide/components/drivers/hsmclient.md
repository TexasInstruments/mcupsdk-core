# HSM client {#DRIVERS_HSMCLIENT_PAGE}

[TOC]

\note In the following document and in **TIFS-MCU** document the term used to refer to the Firmware which runs on HSM core is **TIFS-MCU Firmware**. Please
note that the terms 1.**HSMRt Firmware** and 2.**HSMRt** are synonymous to **TIFS-MCU Firmware**.

## Introduction
- The objective of HSM client is to provide APIs for accessing HSM services. Below given model explains the use of HSM client graphically. It uses @ref DRIVERS_SIPC_NOTIFY_PAGE driver as a low level message passing mechanism to talk to HSM M4 core. HSM client APIs can be used with either **FreeRTOS** or **noRTOS** application.

- The HSM Server provides APIs that will read the incoming request in the form of SIPC message and provide appropriate service based on it. The application which uses HSM server is also known as TIFS-MCU (HSM runtime firmware). 


\imageStyle{sipc_communication_model.png,width:30%}
\image html sipc_communication_model.png "HSM CLIENT communication model"


- Along side providing services at runtime **TIFS-MCU** also enables **secure boot flow**. It is responsible for decrypting and authenticating R5Fs application images and sends a notification to SBL if it is a valid app image , later SBL will boot the R5 applications to respective cores.
- The trusted R5Fs cores which can communicate with HSM are known as **Secure hosts**.

- **Following services are supported in MCU+SDK v8.4.0**
	- **BootNotify** -: A special message sent by TIFS-MCU Firmware to indicate **SBL** that **TIFS-MCU boot** was successful.
	- **GetVersion** -: Get The current version of TIFS-MCU Firmware.
	- **Debug Services** -: For (HS-SE) device only. By default debug port for all R5F and HSM is closed. Secure R5Fs can request debug port access via runtime **Debug services**.


## HSM Client message format.

- Which R5 cores are going to be **secure hosts** will be decided by sysconfig configurations of **TIFS-MCU** at compile time and user has to make sure that similar configurations is done on the R5F side as well.

- Below mentioned is the Message structure defined by @ref DRIVERS_SIPC_NOTIFY_PAGE driver. HSM client populates a SIPC message with relevant parameters and sends this message to HSM Server.


\imageStyle{sipc_message_ex.png,width:40%}
\image html sipc_message_ex.png "SIPC message structure"

#### Message structure description.

- **DestClientID** -: Client Id to which a message is going to be sent. In case of R5F -> HSM ( R5 send message to HSM ) this field will always be **equal to 1** because TIFS-MCU is a NORTOS application and only one client is required.

- **SrcClientID** -: Client Id from which a message is going to be sent. In case of HSM -> R5F( HSM send message to R5 ) this field will always be equal to 1.

- **flags** -: Used for ACK and NACK response signals based on weather a request has been processed correctly or not.
	- Client side signals 1. **AOP** -> ACK on process 2. **NOP** -> No ACK on process
	- Server side signals 1. **ACK** -> request processed successfully 2. **NACK** -> request process failure.
- **Service_type** -:  Type of a requested service.

- **Pointer_to_args** -: Pointer to the arguments required by a particular service.

- **CRC16(args)** -: CRC16 to check the integrity of arguments.

- **CRC16(msg)** -: CRC16 to check the integrity of message.

- The argument will reside in **OCRAM** and only its pointer is going to be passed in the message itself. When HSM receive the message it reads the argument content from OCRAM and process it. Thus other information needed by HSM server can be passed along with the message through HSM MBOX.
\note It is strongly suggested that user application must cleary define a boundary between secure R5FSS and non secure R5FSS core's OCRAM memory so that non secure core cannot access the parameters of HSM message.
One way to define such boundary is to protect some predefined memory region in OCRAM with firewalls so that only secure cores can access this particular memory region.

## HSM client Initialization.
- User needs to add an instance of HSM client in **Secure R5F's** sysconfig configuration.
- Select the **Secure_host_id** for the current **secure R5F** core.
@note Make sure that **SIPC Message Queue Depth, Number of secure Hosts** fields are configured same as in **HSM Server's** sysconfig.

\imageStyle{hsmclient_syscfg.png,width:30%}
\image html hsmclient_syscfg.png "HSM Client sysconfig"

- Based on the selected configuration @ref HsmClient_init() will be called during **system_init()**. HsmClient_init() will set the necessary meta data and calls **SIPC_init()** function.
- Refer to sysconfig generated file **ti_drivers_config.c** for more details.
- After initialization user needs to instantiate HsmClient_t  and use  HsmClient_register()  to register a client with a unique client Id.
- There can be at max **32** distinct registered clients at a time. If user tries to register any more clients HsmClient_register() api will return Failure.
- The idea here is that every **RTOS** task has to register a **unique** **client** **Id** to request services from HSM Server.

-
@note If user is not using sysconfig then it is important to call **HsmClient_init()** with appropriate SIPC_Params before any service function calls.

## HSM Client generic service flow.

- Below mentioned are the generic steps that every service request made by HSM client follows except @ref HSM_MSG_BOOT_NOTIFY


\imageStyle{ hsmclient_generic_msg.png,width:35% }
\image html hsmclient_generic_msg.png "HSM Client generic service flow"


- First instantiate HsmClient_t and register this client object using HsmClient_register().
- populate **ReqMsg**  field of HsmClient_t with appropriate paramters.
	- ReqMsg → destClientId = 0
	- ReqMsg → srcClientId = current client ID.
	- ReqMsg → serType = supported service MACRO.
	- ReqMsg → flags = AOP  if ack is expected otherwise NAOP
	- ReqMsg → args = pointer to message structure based on service type.
	- ReqMsg → crcArgs = crc of args which is in OCRAM
	- ReqMsg → crcMsg = crc of the ReqMsg without crcMsg field.
- Use SIPC_sendMsg() Api to send the 13 byte message to HSM.
- If @ref HSM_FLAG_AOP flag is selected then pend on HsmClient_t::Semaphore till timeout exception occurs or a Response message is received. If @ref HSM_FLAG_NAOP flag is passed then server will not respond with message.
- HsmClient_isr() will receive the response message and copy the same in to HsmClient_t::RespMsg and post the semaphore. As this ISR is blocking we want to quickly read the message and exit it.
- check RespMsg integrity i.e CRC16 for HSM message and CRC16 for args. if integrity check fails then return SystemP_FAILURE.
- If HSM_FLAG_ACK is received then returns SystemP_SUCCESS else SystemP_FAILURE.

## HSM Client Load TIFS-MCU API and BootNotify message.

- HSM client Module is used by **SBL** to load **TIFS-MCU Fiwmware** on HSM core.
- SBL includes **hsmRtImg.h** file which contains **TIFS-MCU Firmware** in byte format, next the TIFS-MCU will be built along with SBL application.
- **SBL** instantiates HSM client Module via sysconfig and calls HsmClient_loadHSMRtFirmware() which will load the TIFS-MCU Firmware.
- After loading TIFS-MCU Firmware **SBL** application will wait for a @ref HSM_MSG_BOOT_NOTIFY message from HSM server. This messages indicates that TIFS-MCU load is successfull and HSM Server is now ready to take requests from **Secure R5F** cores.
- SBL will call HsmClient_waitForBootNotify() API which waits till it receives **BootNotify** message.
- Similarly on the HSM side once the **TIFS-MCU** initialization sequence completes it calls **HsmServer_sendBootNotify()** API to send a **BootNotify** message.
- Boot Notify sequence Flow chart.

\imageStyle{BootNotifyFlow.png,width:50%}
\image html BootNotifyFlow.png "Boot Notify sequence Flow chart"

- HsmClient_waitForBootNotify() API takes two parameters one of them is **timeToWaitInTicks**, This parameter defines how long SBL will wait for HSM_MSG_BOOT_NOTIFY message .User can change this parameter in SBL source code as per need.
\note Currently the HsmClient_waitForBootNotify()'s **timeToWaitInTicks** paramter is set to SystemP_WAIT_FOREVER i.e SBL will keep waiting for HSM_MSG_BOOT_NOTIFY message indefinitely.

- refer @ref BOOTFLOW_GUIDE for more information on SBL(Secondary Boot loader)

## HSM Client GetVersion Service.
- This service is used to get the current version of **TIFS-MCU** **Firmware** running on HSM core.
- User needs to instantiate HsmVer_t object and call HsmClient_getVersion() API to get the current TIFS-MCU's version.
- Refer to HsmVer_t_ for the description of different fields that defines a unique TIFS-MCU Firmware version. If User needs to know just the unique 64 bit version ID then, user should read HsmVer_t_::HsmrtVer a 64 bit field which combines all the different fields of HsmVer_t_.

- HsmClient_getVersion() API takes **timeToWaitInTicks** parameter as input which dictates how long application will wait for the response from HSM core. If the timeout exception occurs HsmClient_getVersion() API return SystemP_TIMEOUT.
#### Example Usage
- Demo code for HsmClient_init(). This will be autogenerated by sysconfig.

\snippet hsmclient.c hsm_syscfg

- Registering HsmClient_t with client ID and requesting current TIFS-MCU Version.

\snippet hsmclient.c hsm_ver
\cond (!SOC_AWR294X)
- Example UART getVersion output.
\imageStyle{hsmrt_ver.png,width:40%}
\image html hsmrt_ver.png "Demo Get Version UART log"
\endcond

## HSM Client GetUID Service.

- This service is only available for **HS-SE** devices.
- This service is used to get UID or Unique ID of the device running TIFS_MCU Firmware.
- UID is a 64 byte unique ID for a device which user needs to instantiate as an uint8_t * object and call HsmClient_getUID() API to get the device UID.

#### Example Usage

- Registering HsmClient_t with client ID and requesting device UID.

\snippet hsmclient.c hsm_uid

## HSM Client OpenDbgFirewall Service.

- This service is only available for **HS-SE** devices.
- As debug is closed for both HSM (M4) core and R5F cores in HS-SE device, this service is used to open debug firewall on device running TIFS-MCU Firmware.
- User needs to create a signed certificate with correct cert type as a part of boot extensions, correct sw revision and correct UID in debug extensions.
- User needs to instantiate a uint8_t * pointing to the certificate with certificate length and call HsmClient_openDbgFirewall() API to send the certificate to TIFS-MCU Firmware.

#### Example Usage

- Registering HsmClient_t with client ID and requesting TIFS-MCU Firmware to open Debug Firewalls.

\snippet hsmclient.c hsm_open_dbg_firewall

## APIs
@ref DRV_HSMCLIENT_MODULE
