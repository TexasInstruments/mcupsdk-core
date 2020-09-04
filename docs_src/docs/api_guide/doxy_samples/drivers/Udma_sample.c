
//! [include]
#include <stdio.h>
#include <drivers/udma.h>
//! [include]

Udma_DrvObject      gUdmaDrvObj;
Udma_ChObject       gUdmaChObj;
Udma_DrvHandle      drvHandle = (Udma_DrvHandle) &gUdmaDrvObj;
static uint8_t      gTxRingMem[UDMA_CACHELINE_ALIGNMENT] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));

void ch_open(void)
{
//! [ch_open]
    int32_t         retVal;
    Udma_ChHandle   chHandle = (Udma_ChHandle) &gUdmaChObj;
    uint32_t        chType;
    Udma_ChPrms     chPrms;
    Udma_ChTxPrms   txPrms;
    Udma_ChRxPrms   rxPrms;

    chType = UDMA_CH_TYPE_TR_BLK_COPY;
    UdmaChPrms_init(&chPrms, chType);
    chPrms.fqRingPrms.ringMem       = &gTxRingMem[0U];
    chPrms.fqRingPrms.elemCnt       = 1;
    chPrms.fqRingPrms.ringMemSize   = chPrms.fqRingPrms.elemCnt * sizeof(uint64_t);
    retVal = Udma_chOpen(drvHandle, chHandle, chType, &chPrms);
    if(UDMA_SOK != retVal)
    {
        printf("[Error] UDMA channel open failed!!\r\n");
    }

    /* Config TX channel */
    UdmaChTxPrms_init(&txPrms, chType);
    retVal = Udma_chConfigTx(chHandle, &txPrms);
    if(UDMA_SOK != retVal)
    {
        printf("[Error] UDMA TX channel config failed!!\r\n");
    }

    /* Config RX channel - which is implicitly paired to TX channel in
     * block copy mode */
    UdmaChRxPrms_init(&rxPrms, chType);
    retVal = Udma_chConfigRx(chHandle, &rxPrms);
    if(UDMA_SOK != retVal)
    {
        printf("[Error] UDMA RX channel config failed!!\r\n");
    }

    /* Channel enable */
    retVal = Udma_chEnable(chHandle);
    if(UDMA_SOK != retVal)
    {
        printf("[Error] UDMA channel enable failed!!\r\n");
    }
//! [ch_open]
}

void ch_close(void)
{
//! [ch_close]
    int32_t         retVal;
    Udma_ChHandle   chHandle = (Udma_ChHandle) &gUdmaChObj;

    retVal = Udma_chDisable(chHandle, UDMA_DEFAULT_CH_DISABLE_TIMEOUT);
    if(UDMA_SOK != retVal)
    {
        printf("[Error] UDMA channel disable failed!!\r\n");
    }

    retVal = Udma_chClose(chHandle);
    if(UDMA_SOK != retVal)
    {
        printf("[Error] UDMA channel close failed!!\r\n");
    }

//! [ch_close]
}
