
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

//![hsm_services_example_ospi]

[HSM_CLIENT] New Client Registered with Client Id = 0
 Starting OSPI Bootloader ...
[BOOTLOADER_PROFILE] Boot Media       : NOR SPI FLASH
[BOOTLOADER_PROFILE] Boot Media Clock : 133.333 MHz
[BOOTLOADER_PROFILE] Boot Image Size  : 0 KB
[BOOTLOADER_PROFILE] Cores present    :
r5f0-0
[BOOTLOADER PROFILE] System_init                      :        259us
[BOOTLOADER PROFILE] Drivers_open                     :         91us
[BOOTLOADER PROFILE] LoadHsmRtFw                      :      13409us
[BOOTLOADER PROFILE] Board_driversOpen                :      62004us
[BOOTLOADER PROFILE] CPU load                         :         79us
[BOOTLOADER_PROFILE] SBL Total Time Taken             :      75848us

Image loading done, switching to application ...

 [HSM_CLIENT] New Client Registered with Client Id = 2

 [HSM CLIENT] Sending UID Request to HSM Server..
 [HSM CLIENT] Device UID Successfully retrived from the HSM Server.

 [HSM CLIENT_PROFILE] Time taken by GetUID Request : 81us

 [HSM CLIENT] Device UID is : B045B77BCB5C45087FCF982AD2A67D926775E4FDB2C585CD4307C9F9CEE8C4EFB9EE9949520CD947883A12F28AB8FD8929CC0C5CA66B37A452C6D9BB20E1F9F5[HSM CLIENT] TIFS-MCU 64bit version string = 0x001550200090100

 [HSM CLIENT] TIFS-MCU Information
[Soc Type]          = AM263p
[Device Type]       = HS-FS
[HSM Type]          = HSM_V1
[Bin Type]          = STANDARD
[TIFS-MCU Version]  = 9.1.0

RNG output word -- 0xA647974C
RNG output word -- 0xC36E64BF
RNG output word -- 0x84836E95
RNG output word -- 0x640F84EA


 [HSM CLIENT_PROFILE] Time taken by get RNG request : 473us
Firewall request #1 status = 11111111 11111111
Firewall Id = 12
Firewall region number = 3
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
Firewall region number = 3
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

Firewall request #2 status = 11111111 11111111
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

//![hsm_services_example_ospi]

//![mpu_firewall_services_demo_qspi]
 [HSM_CLIENT] New Client Registered with Client Id = 0
 Starting QSPI Bootloader ...
[BOOTLOADER_PROFILE] Boot Media       : NOR SPI FLASH
[BOOTLOADER_PROFILE] Boot Media Clock : 80.000 MHz
[BOOTLOADER_PROFILE] Boot Image Size  : 28 KB
[BOOTLOADER_PROFILE] Cores present    :
r5f0-1
r5f0-0
[BOOTLOADER PROFILE] System_init                      :        286us
[BOOTLOADER PROFILE] Drivers_open                     :         75us
[BOOTLOADER PROFILE] LoadHsmRtFw                      :      14503us
[BOOTLOADER PROFILE] Board_driversOpen                :       2975us
[BOOTLOADER PROFILE] CPU load                         :       2292us
[BOOTLOADER_PROFILE] SBL Total Time Taken             :      20134us

Image loading done, switching to application ...

 [HSM_CLIENT] New Client Registered with Client Id = 2

 Enabling the interrupt for both address violation and protection violation for MPU_R5SS0_CORE1_AHB
Firewall request #1 status = 11111111 11111111

 Permissions for peripheral region(0x5230 0000 - 0x5230 03FF) are revoked for R5FSS0_1

 [HSM_CLIENT] New Client Registered with Client Id = 2

 Interrupt registeration successful, notifying remote core to read CSL_UART0_U_BASE register
Success
//![mpu_firewall_services_demo_qspi]

//![hsm_services_example_qspi_am273x]
 [HSM_CLIENT] New Client Registered with Client Id = 0
 Starting QSPI Bootloader ...
[BOOTLOADER_PROFILE] Boot Media       : NOR SPI FLASH
[BOOTLOADER_PROFILE] Boot Media Clock : 80.000 MHz
[BOOTLOADER_PROFILE] Boot Image Size  : 0 KB
[BOOTLOADER_PROFILE] Cores present    :
r5f0-0
[BOOTLOADER PROFILE] System_init                      :         72us
[BOOTLOADER PROFILE] Drivers_open                     :         16us
[BOOTLOADER PROFILE] LoadHsmRtFw                      :      12977us
[BOOTLOADER PROFILE] Board_driversOpen                :       2531us
[BOOTLOADER PROFILE] CPU load                         :         43us
[BOOTLOADER_PROFILE] SBL Total Time Taken             :      15641us

Image loading done, switching to application ...

 [HSM_CLIENT] New Client Registered with Client Id = 2

 [HSM CLIENT] Sending UID Request to HSM Server..
 [HSM CLIENT] Device UID Successfully retrived from the HSM Server.

 [HSM CLIENT_PROFILE] Time taken by GetUID Request : 82us

 [HSM CLIENT] Device UID is : AA510E0F5C88A00A8E5918A13C3091D9D102851928513961B6AF3E5BA323FA1DB72EC0FA26B13677DD6211CA04E8FCB12646599773416047C0DE49502E747683[HSM CLIENT] TIFS-MCU 64bit version string = 0
x001550300090100

 [HSM CLIENT] TIFS-MCU Information
[Soc Type]          = AM273x
[Device Type]       = HS-FS
[HSM Type]          = HSM_V1
[Bin Type]          = STANDARD
[TIFS-MCU Version]  = 9.1.0

RNG output word -- 0x6F88FFD6
RNG output word -- 0x9A7DF418
RNG output word -- 0x592BE4B6
RNG output word -- 0x2D3F1E32


 [HSM CLIENT_PROFILE] Time taken by get RNG request : 476us
Firewall request #1 status = 11111111 11111111
Firewall Id = 4
Firewall region number = 7
Start Address = 0x2f7e800
End Address = 0x2f7ebff
Aid Config = 0x4
Supervisor Read = 1
Supervisor Write = 1
Supervisor Execute = 1
User Read = 1
User Write = 1
User Execute = 1
Non Secure Access = 1
Emulation = 1

Firewall request #2 status = 11111111 11111111
Firewall Id = 4
Firewall region number = 6
Start Address = 0x2f7f800
End Address = 0x2f7fbff
Aid Config = 0x4
Supervisor Read = 0
Supervisor Write = 0
Supervisor Execute = 0
User Read = 0
User Write = 0
User Execute = 0
Non Secure Access = 0
Emulation = 0
//![hsm_services_example_qspi_am273x]

//![mpu_firewall_services_demo_ospi]

 [HSM_CLIENT] New Client Registered with Client Id = 0
 Starting OSPI Bootloader ...
[BOOTLOADER_PROFILE] Boot Media       : NOR SPI FLASH
[BOOTLOADER_PROFILE] Boot Media Clock : 133.333 MHz
[BOOTLOADER_PROFILE] Boot Image Size  : 28 KB
[BOOTLOADER_PROFILE] Cores present    :
r5f0-1
r5f0-0
[BOOTLOADER PROFILE] System_init                      :        259us
[BOOTLOADER PROFILE] Drivers_open                     :         91us
[BOOTLOADER PROFILE] LoadHsmRtFw                      :      13410us
[BOOTLOADER PROFILE] Board_driversOpen                :      62013us
[BOOTLOADER PROFILE] CPU load                         :        818us
[BOOTLOADER_PROFILE] SBL Total Time Taken             :      76597us

Image loading done, switching to application ...

 [HSM_CLIENT] New Client Registered with Client Id = 2

 Enabling the interrupt for both address violation and protection violation for MPU_R5SS0_CORE1_AHB
Firewall request #1 status = 11111111 11111111

 Permissions for peripheral region(0x5230 0000 - 0x5230 03FF) are revoked for R5FSS0_1

 [HSM_CLIENT] New Client Registered with Client Id = 2

 Interrupt registeration successful, notifying remote core to read CSL_UART0_U_BASE register
Success

//![mpu_firewall_services_demo_ospi]