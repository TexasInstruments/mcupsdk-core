//! [include1]

#include<stdio.h>
#include <security/security_common/drivers/crypto/crypto.h>
#include <security/security_common/drivers//crypto/sa2ul/sa2ul.h>

/* input or output length*/
#define APP_CRYPTO_AES_CBC_256_INOUT_LENGTH           (16U)
/* Aes max key length*/
#define APP_CRYPTO_AES_CBC_256_MAX_KEY_LENGTH         (32U)
/* Aes max IV length*/
#define APP_CRYPTO_AES_CBC_256_MAX_IV_LENGTH          (16U)
/* Aes key length in bites*/
#define APP_CRYPTO_AES_CBC_256_KEY_LENGTH_IN_BITS     (256U)
/* Alignment */
#define APP_CRYPTO_AES_CBC_ALIGNMENT                  (128U)

/* input buffer for encryption or decryption */
static uint8_t gCryptoAesCbc256InputBuf[APP_CRYPTO_AES_CBC_256_INOUT_LENGTH] =
{
    0x81, 0xEA, 0x5B, 0xA4, 0x69, 0x45, 0xC1, 0x70,
    0x5F, 0x6F, 0x89, 0x77, 0x88, 0x68, 0xCC, 0x67
};

/* The AES encryption algorithm encrypts and decrypts data in blocks of 128 bits. It can do this using 128-bit, 192-bit, or 256-bit keys */
static uint8_t gCryptoAesCbc256Key[APP_CRYPTO_AES_CBC_256_MAX_KEY_LENGTH] =
{
    0x33, 0xA3, 0x66, 0x46, 0xFE, 0x56, 0xF7, 0x0D,
    0xC0, 0xC5, 0x1A, 0x31, 0x17, 0xE6, 0x39, 0xF1,
    0x82, 0xDE, 0xF8, 0xCA, 0xB5, 0xC0, 0x66, 0x71,
    0xEE, 0xA0, 0x40, 0x7C, 0x48, 0xA9, 0xC7, 0x57
};

/* initialization vector (IV) is an arbitrary number that can be used along with a secret key for data encryption/decryption. */
static uint8_t gCryptoAesCbc256Iv[APP_CRYPTO_AES_CBC_256_MAX_IV_LENGTH] =
{
    0x7C, 0xE2, 0xAB, 0xAF, 0x8B, 0xEF, 0x23, 0xC4,
    0x81, 0x6D, 0xC8, 0xCE, 0x84, 0x20, 0x48, 0xA7
};

/* Encryption output buf */
uint8_t     gCryptoAesCbc256EncResultBuf[APP_CRYPTO_AES_CBC_256_INOUT_LENGTH] __attribute__ ((aligned (APP_CRYPTO_AES_CBC_ALIGNMENT)));
/* Decryption output buf */
uint8_t     gCryptoAesCbc256DecResultBuf[APP_CRYPTO_AES_CBC_256_INOUT_LENGTH] __attribute__ ((aligned (APP_CRYPTO_AES_CBC_ALIGNMENT)));

/* Context memory */
static Crypto_Context gCryptoAesCbcContext __attribute__ ((aligned (SA2UL_CACHELINE_ALIGNMENT)));

/* Context Object */
SA2UL_ContextObject  gSa2ulCtxObj __attribute__ ((aligned (SA2UL_CACHELINE_ALIGNMENT)));

//! [include1]

void sa2ul_Aes_cbc(void)
{
//! [sa2ulAesCbc]
    int32_t             status;
    Crypto_Handle       aesHandle;
    SA2UL_ContextParams ctxParams;

    aesHandle = Crypto_open(&gCryptoAesCbcContext);

    /* Configure secure context */
    ctxParams.opType       = SA2UL_OP_ENC;
    ctxParams.encAlg       = SA2UL_ENC_ALG_AES;
    ctxParams.encMode      = SA2UL_ENC_MODE_CBC;
    ctxParams.encKeySize   = SA2UL_ENC_KEYSIZE_256;
    ctxParams.encDirection = SA2UL_ENC_DIR_ENCRYPT;
    memcpy( &ctxParams.key[0], &gCryptoAesCbc256Key[0], SA2UL_MAX_KEY_SIZE_BYTES);
    memcpy( &ctxParams.iv[0], &gCryptoAesCbc256Iv[0], SA2UL_MAX_IV_SIZE_BYTES);
    ctxParams.inputLen = sizeof(gCryptoAesCbc256InputBuf);
    gSa2ulCtxObj.totalLengthInBytes = sizeof(gCryptoAesCbc256InputBuf);

    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoAesCbcContext.drvHandle, &gSa2ulCtxObj, &ctxParams);

    /* Encryption */
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj,&gCryptoAesCbc256InputBuf[0], sizeof(gCryptoAesCbc256InputBuf), gCryptoAesCbc256EncResultBuf);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);

    /* Close AES instance */
    status = Crypto_close(aesHandle);

    /* Kill warning of variable set but not used */
    (void) status;

    return;
//! [sa2ulAesCbc]
}