
//![hsm_syscfg]
/*
 * HSM Client
 */

/* memory assigned for each R5x <-> HSM channel */
uint8_t gQueue_R5ToHsm[SIPC_NUM_R5_CORES][SIPC_QUEUE_LENGTH*SIPC_MSG_SIZE] __attribute__((aligned(8),section(".bss.sipc_hsm_queue_mem")));
uint8_t gQueue_HsmToR5[SIPC_NUM_R5_CORES][SIPC_QUEUE_LENGTH*SIPC_MSG_SIZE] __attribute__((aligned(8),section(".bss.sipc_r5f_queue_mem")));

void HsmClient_config(void)
{
    SIPC_Params sipcParams;
    int32_t status;

    /* initialize parameters to default */
    SIPC_Params_init(&sipcParams);

    sipcParams.ipcQueue_eleSize_inBytes = SIPC_MSG_SIZE;
    sipcParams.ipcQueue_length = SIPC_QUEUE_LENGTH ;
    /* list the cores that will do SIPC communication with this core
    * Make sure to NOT list 'self' core in the list below
    */
    sipcParams.numCores = 1;
    sipcParams.coreIdList[0] = CORE_INDEX_HSM;

    /* This is HSM -> R5F queue */
    sipcParams.tx_SipcQueues[CORE_INDEX_HSM] = (uintptr_t)gQueue_HsmToR5[0] ;
    sipcParams.rx_SipcQueues[CORE_INDEX_HSM] = (uintptr_t)gQueue_R5ToHsm[0] ;
    sipcParams.secHostCoreId[CORE_INDEX_SEC_MASTER_0] = CORE_ID_R5FSS0_0;

    /* initialize the HsmClient module */
    status = HsmClient_init(&sipcParams);
    DebugP_assert(status==SystemP_SUCCESS);
//![hsm_syscfg]

//![hsm_ver]

#define APP_CLIENT_ID                  (0x02)

/* Demo Application code on R5 */
void HsmClientApp_start(void)
{
    int32_t status ;
    HsmClient_t client ;
    HsmVer_t *hsmVer = malloc(sizeof(HsmVer_t)) ;
    uint32_t CycleCounterA = 0, CycleCounterB = 0, TotalTimeInNsec;

    status = HsmClient_register(&client,APP_CLIENT_ID);
    DebugP_assert(status == SystemP_SUCCESS);

    status = HsmClient_getVersion(&client,hsmVer,SystemP_WAIT_FOREVER);
    DebugP_assert(status == SystemP_SUCCESS);

    /* print version */
    DebugP_log("[HSM CLIENT] HSMRT 64bit version string = 0x00%llx",hsmVer->HsmrtVer);
    DebugP_log("\r\n[HSM_CLIENT] HSMRt Version \r\n\
                    [Device Type]   = 0x%x\r\n\
                    [Bin Type]      = 0x%x\r\n\
                    [Soc Type]      = 0x%x\r\n\
                    [Arch Num]      = 0x%x\r\n\
                    [Api Version]   = 0x%x\r\n\
                    [Major Version] = 0x%x\r\n\
                    [Minor Version] = 0x%x\r\n\
                    [Patch Version] = 0x%x\r\n",\
                    hsmVer->VerStruct.DevType,hsmVer->VerStruct.BinType,hsmVer->VerStruct.SocType,hsmVer->VerStruct.ApiVer,hsmVer->VerStruct.MajorVer,hsmVer->VerStruct.MinorVer,hsmVer->VerStruct.PatchVer);
}

//![hsm_ver]

//![hsm_uid]

#define APP_CLIENT_ID                 (0x02)
#define HSM_UID_SIZE                  (64U)

/* Demo Application code on R5 */
void HsmClientApp_start(void)
{
    int32_t status ;
    HsmClient_t client ;
    uint8_t *uid = malloc(HSM_UID_SIZE) ;

    status = HsmClient_register(&client,APP_CLIENT_ID);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Send Request for UID to HSM Server */
    status = HsmClient_getUID(&client, (uint8_t *)uid, SystemP_WAIT_FOREVER);

    /* print UID */
    DebugP_log("\r\n [HSM CLIENT] Device UID is : ");
    for(uint8_t i = 0; i<HSM_UID_SIZE; i++)
    {
        DebugP_log("%02X", uid[i]);
    }
}

//![hsm_uid]

//![hsm_set_firewall]

/* Demo Application code on R5 */
void HsmClientApp_start(void)
{
    int32_t status ;
    HsmClient_t client ;
    /* struct instance used for sending set firewall request */
    FirewallReq_t FirewallReqObj;

    status = HsmClient_register(&client,APP_CLIENT_ID);
    DebugP_assert(status == SystemP_SUCCESS);

    /* region count and region configuration array is generated via sysconfig and is used
        for populating  FirewallReqObj*/
    FirewallReqObj.regionCount = FIREWALL_ARRAY0_NUM_REGIONS;
    FirewallReqObj.FirewallRegionArr = gMpuFirewallRegionConfig_0;

    DebugP_log("\r\n [HSM CLIENT] Sending SET FIREWALL Request to HSM Server..");
    /* This requests configures the region and allow all permissions to only R5FSS0_0 */
    status = HsmClient_setFirewall(&client,&FirewallReqObj,SystemP_WAIT_FOREVER);

    /* print statusFirewallRegionArr */
    DebugP_log("\r\n [HSM CLIENT] statusFirewallRegionArr : 0x%x\r\n ", FirewallReqObj.statusFirewallRegionArr);
}

//![hsm_set_firewall]

//![hsm_services_example_qspi]

 [HSM_CLIENT] New Client Registered with Client Id = 0
 Starting QSPI Bootloader ...
[BOOTLOADER_PROFILE] Boot Media       : NOR SPI FLASH
[BOOTLOADER_PROFILE] Boot Media Clock : 80.000 MHz
[BOOTLOADER_PROFILE] Boot Image Size  : 0 KB
[BOOTLOADER_PROFILE] Cores present    :
r5f0-0
[BOOTLOADER PROFILE] System_init                      :        290us
[BOOTLOADER PROFILE] Drivers_open                     :         75us
[BOOTLOADER PROFILE] LoadHsmRtFw                      :      14518us
[BOOTLOADER PROFILE] Board_driversOpen                :       2971us
[BOOTLOADER PROFILE] CPU load                         :        130us
[BOOTLOADER_PROFILE] SBL Total Time Taken             :      17987us

Image loading done, switching to application ...

 [HSM_CLIENT] New Client Registered with Client Id = 2

 [HSM CLIENT] Sending UID Request to HSM Server..
 [HSM CLIENT] Device UID Suceessfully retrived from the HSM Server.

 [HSM CLIENT_PROFILE] Time taken by GetUID Request : 84us

 [HSM CLIENT] Device UID is : 06D3BF416160CDD6DDF8CB8D7DEE63DFB42EDFE805726C0DE705D76CC586BD03E18B601A6D5854C810CBD00

 [HSM CLIENT] TIFS-MCU Information
[Soc Type]          = AM263x
[Device Type]       = HS-FS
[HSM Type]          = HSM_V1
[Bin Type]          = STANDARD
[TIFS-MCU Version]  = 9.0.0

Firewall request status = 11111111 11111111
Firewall Id = 12
Firewall region number = 2
Start Address = 0x502f0000
End Address = 0x502f0fff
Aid Config = 0x10
Supervisor Read = 1
Supervisor Write = 1
Supervisor Execute = 1
User Read = 1
User Write = 1
User Execute = 1
Non Secure Access = 1
Emulation = 1

Firewall Id = 13
Firewall region number = 2
Start Address = 0x502f0000
End Address = 0x502f0fff
Aid Config = 0x10
Supervisor Read = 1
Supervisor Write = 1
Supervisor Execute = 1
User Read = 1
User Write = 1
User Execute = 1
Non Secure Access = 1
Emulation = 1

Firewall Id = 14
Firewall region number = 2
Start Address = 0x502f0000
End Address = 0x502f0fff
Aid Config = 0x10
Supervisor Read = 1
Supervisor Write = 1
Supervisor Execute = 1
User Read = 1
User Write = 1
User Execute = 1
Non Secure Access = 1
Emulation = 1

Firewall Id = 15
Firewall region number = 2
Start Address = 0x502f0000
End Address = 0x502f0fff
Aid Config = 0x10
Supervisor Read = 1
Supervisor Write = 1
Supervisor Execute = 1
User Read = 1
User Write = 1
User Execute = 1
Non Secure Access = 1
Emulation = 1

Firewall Id = 16
Firewall region number = 2
Start Address = 0x502f0000
End Address = 0x502f0fff
Aid Config = 0x10
Supervisor Read = 1
Supervisor Write = 1
Supervisor Execute = 1
User Read = 1
User Write = 1
User Execute = 1
Non Secure Access = 1
Emulation = 1

Firewall Id = 17
Firewall region number = 2
Start Address = 0x502f0000
End Address = 0x502f0fff
Aid Config = 0x10
Supervisor Read = 1
Supervisor Write = 1
Supervisor Execute = 1
User Read = 1
User Write = 1
User Execute = 1
Non Secure Access = 1
Emulation = 1

Firewall request status = 11111111 11111111
Firewall Id = 8
Firewall region number = 0
Start Address = 0x72000400
End Address = 0x720007ff
Aid Config = 0xe0
Supervisor Read = 0
Supervisor Write = 0
Supervisor Execute = 0
User Read = 0
User Write = 0
User Execute = 0
Non Secure Access = 1
Emulation = 1

//![hsm_services_example_qspi]

//![hsm_services_example_uart]

 [HSM_CLIENT] New Client Registered with Client Id = 2

 [HSM CLIENT] Sending UID Request to HSM Server..
 [HSM CLIENT] Device UID Suceessfully retrived from the HSM Server.

 [HSM CLIENT_PROFILE] Time taken by GetUID Request : 84us

 [HSM CLIENT] Device UID is : 06D3BF416160CDD6DDF8CB8D7DEE63DFB42EDFE805726C0DE705D76CC586BD03E18B601A6D5854C810CBD00

 [HSM CLIENT] TIFS-MCU Information
[Soc Type]          = AM263x
[Device Type]       = HS-FS
[HSM Type]          = HSM_V1
[Bin Type]          = STANDARD
[TIFS-MCU Version]  = 9.0.0

Firewall request status = 11111111 11111111
Firewall Id = 12
Firewall region number = 2
Start Address = 0x502f0000
End Address = 0x502f0fff
Aid Config = 0x10
Supervisor Read = 1
Supervisor Write = 1
Supervisor Execute = 1
User Read = 1
User Write = 1
User Execute = 1
Non Secure Access = 1
Emulation = 1

Firewall Id = 13
Firewall region number = 2
Start Address = 0x502f0000
End Address = 0x502f0fff
Aid Config = 0x10
Supervisor Read = 1
Supervisor Write = 1
Supervisor Execute = 1
User Read = 1
User Write = 1
User Execute = 1
Non Secure Access = 1
Emulation = 1

Firewall Id = 14
Firewall region number = 2
Start Address = 0x502f0000
End Address = 0x502f0fff
Aid Config = 0x10
Supervisor Read = 1
Supervisor Write = 1
Supervisor Execute = 1
User Read = 1
User Write = 1
User Execute = 1
Non Secure Access = 1
Emulation = 1

Firewall Id = 15
Firewall region number = 2
Start Address = 0x502f0000
End Address = 0x502f0fff
Aid Config = 0x10
Supervisor Read = 1
Supervisor Write = 1
Supervisor Execute = 1
User Read = 1
User Write = 1
User Execute = 1
Non Secure Access = 1
Emulation = 1

Firewall Id = 16
Firewall region number = 2
Start Address = 0x502f0000
End Address = 0x502f0fff
Aid Config = 0x10
Supervisor Read = 1
Supervisor Write = 1
Supervisor Execute = 1
User Read = 1
User Write = 1
User Execute = 1
Non Secure Access = 1
Emulation = 1

Firewall Id = 17
Firewall region number = 2
Start Address = 0x502f0000
End Address = 0x502f0fff
Aid Config = 0x10
Supervisor Read = 1
Supervisor Write = 1
Supervisor Execute = 1
User Read = 1
User Write = 1
User Execute = 1
Non Secure Access = 1
Emulation = 1

Firewall request status = 11111111 11111111
Firewall Id = 8
Firewall region number = 0
Start Address = 0x72000400
End Address = 0x720007ff
Aid Config = 0xe0
Supervisor Read = 0
Supervisor Write = 0
Supervisor Execute = 0
User Read = 0
User Write = 0
User Execute = 0
Non Secure Access = 1
Emulation = 1

//![hsm_services_example_uart]