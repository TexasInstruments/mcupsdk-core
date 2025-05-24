

#ifndef NETXDUO_ENET_H
#define NETXDUO_ENET_H

#include <enet.h>
#include <nx_api.h>


#define NX_DRIVER_ERROR      (0x80)

typedef struct nx_enet_drv_rx_ch *nx_enet_drv_rx_ch_hndl_t;
typedef struct nx_enet_drv_tx_ch *nx_enet_drv_tx_ch_hndl_t;


void _nx_enet_driver(NX_IP_DRIVER *driver_req_ptr);

void NetxEnetDriver_allocRxCh(EnetDma_RxChHandle hRxCh, uint32_t numPkts, NX_PACKET_POOL *p_rx_packet_pool, nx_enet_drv_rx_ch_hndl_t *pRxChHandle);

void NetxEnetDriver_allocTxCh(EnetDma_TxChHandle hTxCh, uint32_t numPkts, nx_enet_drv_tx_ch_hndl_t *pTxChHandle);

void NetxEnetDriver_allocIf(const char *p_name, Enet_MacPort macport, uint8_t macAddr[ENET_MAC_ADDR_LEN],
                            nx_enet_drv_rx_ch_hndl_t hRxChs[], uint32_t numRxCh, nx_enet_drv_tx_ch_hndl_t hTxChs[], uint32_t numTxCh);

#endif
