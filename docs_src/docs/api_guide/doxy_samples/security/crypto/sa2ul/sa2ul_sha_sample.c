//! [include]

#include<stdio.h>
#include <security/security_common/drivers/crypto/crypto.h>
#include <security/security_common/drivers/crypto/sa2ul/sa2ul.h>

Crypto_Context gCryptoSha512Context;
static uint8_t gCryptoSha512TestBuf[9] = {"abcdefpra"};
SA2UL_ContextObject  gCtxObj;
//! [include]

void sa2ul_Sha(void)
{
//! [sa2ulsha]

    int32_t             status;
    uint8_t             sha512sum[64];
    Crypto_Handle       shaHandle;
    SA2UL_ContextParams ctxParams;

    /* Init SA */
    SA2UL_init();

    /* Open SHA instance */
    shaHandle = Crypto_open(&gCryptoSha512Context);

    /* Configure secure context */
    ctxParams.opType    = SA2UL_OP_AUTH;
    ctxParams.hashAlg   = SA2UL_HASH_ALG_SHA2_512;
    ctxParams.inputLen  = sizeof(gCryptoSha512TestBuf);
    gCtxObj.totalLengthInBytes = sizeof(gCryptoSha512TestBuf);
    status = SA2UL_contextAlloc(gCryptoSha512Context.drvHandle,&gCtxObj,&ctxParams);

    /* Perform SHA operation */
    status = SA2UL_contextProcess(&gCtxObj,&gCryptoSha512TestBuf[0], sizeof(gCryptoSha512TestBuf), sha512sum);

    /* Free the secure context configuration */
    SA2UL_contextFree(&gCtxObj);

    /* Close SHA instance */
    status = Crypto_close(shaHandle);

    /*deinit SA */
    SA2UL_deinit();

    /* Kill warning of variable set but not used */
    (void) status;

//! [sa2ulsha]
}