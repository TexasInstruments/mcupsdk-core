
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

//![hsm_open_dbg_firewall]

#define APP_CLIENT_ID                      (0x02)
#define HSM_DBG_CERT_SIZE                  (4096U)

/* Demo Application code on R5 */
void HsmClientApp_start(void)
{
    int32_t status ;
    HsmClient_t client ;
    uint8_t cert[] = {....};
    uint32_t cert_len = sizeof(cert);

    status = HsmClient_register(&client, APP_CLIENT_ID);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Send Certificate to HSM Server */
    status = HsmClient_openDbgFirewall(&client, (uint8_t *)cert, cert_len, SystemP_WAIT_FOREVER);
    DebugP_assert(status == SystemP_SUCCESS);

    DebugP_log("\r\n [HSM CLIENT] Debug is Opened.");
}

//![hsm_open_dbg_firewall]

