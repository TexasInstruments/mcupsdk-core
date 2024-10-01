

#ifndef NETXDUO_ENET_H
#define NETXDUO_ENET_H

#include <enet.h>
#include <nx_api.h>


#define NX_DRIVER_ERROR      (0x80)

typedef struct nx_enet_drv_rx_ch *nx_enet_drv_rx_ch_hndl_t;
typedef struct nx_enet_drv_tx_ch *nx_enet_drv_tx_ch_hndl_t;


void _nx_enet_driver(NX_IP_DRIVER *driver_req_ptr);

void nx_enet_driver_port_to_if_map(const char *p_if_name, Enet_MacPort macport);

void nx_enet_driver_port_to_rx_channels_map(Enet_MacPort macport, uint32_t const t_rx_ch_ids[], size_t ch_id_cnt);

void nx_enet_driver_port_to_tx_channel_map(Enet_MacPort macport, uint32_t tx_ch_id);

void NetxEnetDriver_allocRxCh(EnetDma_RxChHandle hRxCh, uint32_t numPkts, nx_enet_drv_rx_ch_hndl_t *pRxChHandle);

void NetxEnetDriver_allocTxCh(EnetDma_TxChHandle hTxCh, uint32_t numPkts, nx_enet_drv_tx_ch_hndl_t *pTxChHandle);

void NetxEnetDriver_allocIf(const char *p_name, Enet_MacPort macport, uint8_t macAddr[ENET_MAC_ADDR_LEN],
                            nx_enet_drv_rx_ch_hndl_t hRxChs[], uint32_t numRxCh, nx_enet_drv_tx_ch_hndl_t hTxChs[], uint32_t numTxCh);

#endif
