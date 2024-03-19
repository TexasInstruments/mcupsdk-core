//app_netif.h

void EthApp_initNetif(void);

void App_shutdownNetworkStack();

void EnetMp_togglePromisc(Enet_Type enetType, uint32_t instId);

void App_waitForBridgeUp();

int32_t AddNetif_addBridgeMcastEntry(Icssg_MacAddr mac);

int32_t AddNetif_delBridgeMcastEntry(Icssg_MacAddr mac);

int32_t App_addMacFdbEntry(Enet_Type enetType, uint32_t instId, Icssg_MacAddr mac);

int32_t App_delMacFdbEntry(Enet_Type enetType, uint32_t instId, Icssg_MacAddr mac);

