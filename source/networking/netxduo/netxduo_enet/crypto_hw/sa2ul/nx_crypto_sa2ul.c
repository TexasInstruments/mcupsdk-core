/*
 *  Copyright (c) Texas Instruments Incorporated 2025
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "nx_crypto_sa2ul.h"



#include <tx_port.h>
#include <nx_crypto.h>
#include <nx_secure_tls.h>

#include <security_common/drivers/crypto/crypto.h>
#include <security_common/drivers/crypto/asym_crypt.h>
#include <security_common/drivers/crypto/sa2ul/sa2ul.h>

#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/SystemP.h>


Crypto_Handle gCryptoHandle = NULL;
AsymCrypt_Handle gAsymCryptHandle = NULL;
Crypto_Context gCryptoContext __attribute__ ((aligned (SA2UL_CACHELINE_ALIGNMENT)));
SA2UL_ContextObject gSa2ulCtxObj __attribute__ ((aligned (SA2UL_CACHELINE_ALIGNMENT)));





extern NX_CRYPTO_METHOD crypto_method_aes_cbc_128;
extern NX_CRYPTO_METHOD crypto_method_aes_cbc_256;

#ifdef NX_SECURE_TLS_TLS_1_3_ENABLED
extern NX_CRYPTO_METHOD crypto_method_aes_128_gcm_16;
extern NX_CRYPTO_METHOD crypto_method_aes_ccm_16;
extern NX_CRYPTO_METHOD crypto_method_aes_ccm_8;
extern NX_CRYPTO_METHOD crypto_method_hkdf;
#endif

#ifdef NX_SECURE_ENABLE_ECC_CIPHERSUITE
extern NX_CRYPTO_METHOD crypto_method_ec_secp256;
extern NX_CRYPTO_METHOD crypto_method_ec_secp384;
extern NX_CRYPTO_METHOD crypto_method_ec_secp521;

extern NX_CRYPTO_METHOD crypto_method_ecdsa;
extern NX_CRYPTO_METHOD crypto_method_ecdhe;
extern NX_CRYPTO_METHOD crypto_method_hmac;
#endif

extern NX_CRYPTO_METHOD crypto_method_null;


extern NX_CRYPTO_METHOD crypto_method_rsa;
extern NX_CRYPTO_METHOD crypto_method_hmac_sha256;
extern NX_CRYPTO_METHOD crypto_method_tls_prf_sha256;
extern NX_CRYPTO_METHOD crypto_method_sha224;
extern NX_CRYPTO_METHOD crypto_method_sha256;
extern NX_CRYPTO_METHOD crypto_method_sha384;
extern NX_CRYPTO_METHOD crypto_method_sha512;
extern NX_CRYPTO_METHOD crypto_method_sha1;
extern NX_CRYPTO_METHOD crypto_method_md5;



/* Ciphersuite table without ECC. */
/* Lookup table used to map ciphersuites to cryptographic routines. */
/* For TLS Web servers, define NX_SECURE_ENABLE_AEAD_CIPHER to allow web browsers to connect using AES_128_GCM cipher suites. */
const NX_SECURE_TLS_CIPHERSUITE_INFO sa2ul_crypto_ciphersuite_lookup_table[] =
{
    /* Ciphersuite,                           public cipher,                  public_auth,                    session cipher & cipher mode,         iv size, key size,  hash method,                    hash size, TLS PRF */
#ifndef NX_SECURE_DISABLE_X509
#ifdef NX_SECURE_ENABLE_AEAD_CIPHER
    {TLS_RSA_WITH_AES_128_GCM_SHA256,         &sa2ul_crypto_method_rsa,       &sa2ul_crypto_method_rsa,       &crypto_method_aes_128_gcm_16,        16,      16,        &crypto_method_null,            0,         &crypto_method_tls_prf_sha256},
#endif /* NX_SECURE_ENABLE_AEAD_CIPHER */
    {TLS_RSA_WITH_AES_256_CBC_SHA256,         &sa2ul_crypto_method_rsa,       &sa2ul_crypto_method_rsa,       &sa2ul_crypto_method_aes_cbc_256,     16,      32,        &crypto_method_hmac_sha256,     32,        &crypto_method_tls_prf_sha256},
    {TLS_RSA_WITH_AES_128_CBC_SHA256,         &sa2ul_crypto_method_rsa,       &sa2ul_crypto_method_rsa,       &sa2ul_crypto_method_aes_cbc_128,     16,      16,        &crypto_method_hmac_sha256,     32,        &crypto_method_tls_prf_sha256},
#endif /* NX_SECURE_DISABLE_X509 */

#ifdef NX_SECURE_ENABLE_PSK_CIPHERSUITES
    {TLS_PSK_WITH_AES_128_CBC_SHA256,         &crypto_method_null,            &crypto_method_auth_psk,        &aes_crypto_method_aes_cbc_128,       16,      16,        &crypto_method_hmac_sha256,     32,        &crypto_method_tls_prf_sha256},
#ifdef NX_SECURE_ENABLE_AEAD_CIPHER
    {TLS_PSK_WITH_AES_128_CCM_8,              &crypto_method_null,            &crypto_method_auth_psk,        &crypto_method_aes_ccm_8,             16,      16,        &crypto_method_null,            0,         &crypto_method_tls_prf_sha256},
#endif
#endif /* NX_SECURE_ENABLE_PSK_CIPHERSUITES */
};

const UINT sa2ul_crypto_ciphersuite_lookup_table_size = sizeof(sa2ul_crypto_ciphersuite_lookup_table) / sizeof(NX_SECURE_TLS_CIPHERSUITE_INFO);

#ifndef NX_SECURE_DISABLE_X509
/* Lookup table for X.509 digital certificates - they need a public-key algorithm and a hash routine for verification. */
const NX_SECURE_X509_CRYPTO sa2ul_crypto_x509_cipher_lookup_table[] =
{
    /* OID identifier,                        public cipher,            hash method */
    {NX_SECURE_TLS_X509_TYPE_RSA_SHA_256,    &sa2ul_crypto_method_rsa,       &crypto_method_sha256},
    {NX_SECURE_TLS_X509_TYPE_RSA_SHA_384,    &sa2ul_crypto_method_rsa,       &crypto_method_sha384},
    {NX_SECURE_TLS_X509_TYPE_RSA_SHA_512,    &sa2ul_crypto_method_rsa,       &crypto_method_sha512},
    {NX_SECURE_TLS_X509_TYPE_RSA_SHA_1,      &sa2ul_crypto_method_rsa,       &crypto_method_sha1},
    {NX_SECURE_TLS_X509_TYPE_RSA_MD5,        &sa2ul_crypto_method_rsa,       &crypto_method_md5},
};

const UINT sa2ul_crypto_x509_cipher_lookup_table_size = sizeof(sa2ul_crypto_x509_cipher_lookup_table) / sizeof(NX_SECURE_X509_CRYPTO);
#endif /* NX_SECURE_DISABLE_X509 */

/* Define the object we can pass into TLS. */
const NX_SECURE_TLS_CRYPTO sa2ul_crypto_tls_ciphers =
{
    /* Ciphersuite lookup table and size. */
    (NX_SECURE_TLS_CIPHERSUITE_INFO *)sa2ul_crypto_ciphersuite_lookup_table,
    sizeof(sa2ul_crypto_ciphersuite_lookup_table) / sizeof(NX_SECURE_TLS_CIPHERSUITE_INFO),

#ifndef NX_SECURE_DISABLE_X509
    /* X.509 certificate cipher table and size. */
    (NX_SECURE_X509_CRYPTO *)sa2ul_crypto_x509_cipher_lookup_table,
    sizeof(sa2ul_crypto_x509_cipher_lookup_table) / sizeof(NX_SECURE_X509_CRYPTO),
#endif

    /* TLS version-specific methods. */
#if (NX_SECURE_TLS_TLS_1_0_ENABLED || NX_SECURE_TLS_TLS_1_1_ENABLED)
    &crypto_method_md5,
    &crypto_method_sha1,
    &crypto_method_tls_prf_1,
#endif

#if (NX_SECURE_TLS_TLS_1_2_ENABLED)
    &crypto_method_sha256,
    &crypto_method_tls_prf_sha256,
#endif

#if (NX_SECURE_TLS_TLS_1_3_ENABLED)
    &crypto_method_hkdf,
    &crypto_method_hmac,
    &crypto_method_ecdhe,
#endif
};


#ifdef NX_SECURE_ENABLE_ECC_CIPHERSUITE

#ifndef NX_SECURE_DISABLE_X509

/* Lookup table for X.509 digital certificates - they need a public-key algorithm and a hash routine for verification. */
const NX_SECURE_X509_CRYPTO sa2ul_crypto_x509_cipher_lookup_table_ecc[] =
{
    /* OID identifier,                        public cipher,            hash method */
    {NX_SECURE_TLS_X509_TYPE_ECDSA_SHA_256,  &sa2ul_crypto_method_ecdsa,     &crypto_method_sha256},
    {NX_SECURE_TLS_X509_TYPE_ECDSA_SHA_384,  &sa2ul_crypto_method_ecdsa,     &crypto_method_sha384},
    {NX_SECURE_TLS_X509_TYPE_ECDSA_SHA_512,  &sa2ul_crypto_method_ecdsa,     &crypto_method_sha512},
    {NX_SECURE_TLS_X509_TYPE_RSA_SHA_256,    &sa2ul_crypto_method_rsa,       &crypto_method_sha256},
    {NX_SECURE_TLS_X509_TYPE_RSA_SHA_384,    &sa2ul_crypto_method_rsa,       &crypto_method_sha384},
    {NX_SECURE_TLS_X509_TYPE_RSA_SHA_512,    &sa2ul_crypto_method_rsa,       &crypto_method_sha512},
    {NX_SECURE_TLS_X509_TYPE_ECDSA_SHA_224,  &sa2ul_crypto_method_ecdsa,     &crypto_method_sha224},
    {NX_SECURE_TLS_X509_TYPE_ECDSA_SHA_1,    &sa2ul_crypto_method_ecdsa,     &crypto_method_sha1},
    {NX_SECURE_TLS_X509_TYPE_RSA_SHA_1,      &sa2ul_crypto_method_rsa,       &crypto_method_sha1},
    {NX_SECURE_TLS_X509_TYPE_RSA_MD5,        &sa2ul_crypto_method_rsa,       &crypto_method_md5},
};

const UINT sa2ul_crypto_x509_cipher_lookup_table_ecc_size = sizeof(sa2ul_crypto_x509_cipher_lookup_table_ecc) / sizeof(NX_SECURE_X509_CRYPTO);

#endif


/* Ciphersuite table with ECC. */
/* Lookup table used to map ciphersuites to cryptographic routines. */
/* Ciphersuites are negotiated IN ORDER - top priority first. Ciphersuites lower in the list are considered less secure. */
/* For TLS Web servers, define NX_SECURE_ENABLE_AEAD_CIPHER to allow web browsers to connect using AES_128_GCM cipher suites. */
const NX_SECURE_TLS_CIPHERSUITE_INFO sa2ul_crypto_ciphersuite_lookup_table_ecc[] =
{
    /* Ciphersuite,                           public cipher,            public_auth,              session cipher & cipher mode,   iv size, key size,  hash method,                    hash size, TLS PRF */
#ifndef NX_SECURE_DISABLE_X509
#if (NX_SECURE_TLS_TLS_1_3_ENABLED)
    {TLS_AES_128_GCM_SHA256,                  &crypto_method_ecdhe,           &sa2ul_crypto_method_ecdsa,     &crypto_method_aes_128_gcm_16,  96,      16,        &crypto_method_sha256,         32,         &crypto_method_hkdf},
    {TLS_AES_128_CCM_SHA256,                  &crypto_method_ecdhe,           &sa2ul_crypto_method_ecdsa,     &crypto_method_aes_ccm_16,      96,      16,        &crypto_method_sha256,         32,         &crypto_method_hkdf},
    {TLS_AES_128_CCM_8_SHA256,                &crypto_method_ecdhe,           &sa2ul_crypto_method_ecdsa,     &crypto_method_aes_ccm_8,       96,      16,        &crypto_method_sha256,         32,         &crypto_method_hkdf},
#endif

#ifdef NX_SECURE_ENABLE_AEAD_CIPHER
    {TLS_ECDHE_ECDSA_WITH_AES_128_GCM_SHA256, &crypto_method_ecdhe,           &sa2ul_crypto_method_ecdsa,     &crypto_method_aes_128_gcm_16,  16,      16,        &crypto_method_null,            0,         &crypto_method_tls_prf_sha256},
    {TLS_ECDHE_RSA_WITH_AES_128_GCM_SHA256,   &crypto_method_ecdhe,           &sa2ul_crypto_method_rsa,       &crypto_method_aes_128_gcm_16,  16,      16,        &crypto_method_null,            0,         &crypto_method_tls_prf_sha256},
#endif /* NX_SECURE_ENABLE_AEAD_CIPHER */

    {TLS_ECDHE_ECDSA_WITH_AES_128_CBC_SHA256, &crypto_method_ecdhe,           &sa2ul_crypto_method_ecdsa,     &sa2ul_crypto_method_aes_cbc_128,     16,      16,        &crypto_method_hmac_sha256,     32,        &crypto_method_tls_prf_sha256},
    {TLS_ECDHE_RSA_WITH_AES_128_CBC_SHA256,   &crypto_method_ecdhe,           &sa2ul_crypto_method_rsa,       &sa2ul_crypto_method_aes_cbc_128,     16,      16,        &crypto_method_hmac_sha256,     32,        &crypto_method_tls_prf_sha256},

#ifdef NX_SECURE_ENABLE_AEAD_CIPHER
    {TLS_RSA_WITH_AES_128_GCM_SHA256,         &sa2ul_crypto_method_rsa,       &sa2ul_crypto_method_rsa,       &crypto_method_aes_128_gcm_16,  16,      16,        &crypto_method_null,            0,         &crypto_method_tls_prf_sha256},
#endif /* NX_SECURE_ENABLE_AEAD_CIPHER */

    {TLS_RSA_WITH_AES_256_CBC_SHA256,         &sa2ul_crypto_method_rsa,       &sa2ul_crypto_method_rsa,       &sa2ul_crypto_method_aes_cbc_256,     16,      32,        &crypto_method_hmac_sha256,     32,        &crypto_method_tls_prf_sha256},
    {TLS_RSA_WITH_AES_128_CBC_SHA256,         &sa2ul_crypto_method_rsa,       &sa2ul_crypto_method_rsa,       &sa2ul_crypto_method_aes_cbc_128,     16,      16,        &crypto_method_hmac_sha256,     32,        &crypto_method_tls_prf_sha256},
#endif

#ifdef NX_SECURE_ENABLE_PSK_CIPHERSUITES
    {TLS_PSK_WITH_AES_128_CBC_SHA256,         &crypto_method_null,      &crypto_method_auth_psk,              &sa2ul_crypto_method_aes_cbc_128,     16,      16,        &crypto_method_hmac_sha256,     32,        &crypto_method_tls_prf_sha256},
#ifdef NX_SECURE_ENABLE_AEAD_CIPHER
    {TLS_PSK_WITH_AES_128_CCM_8,              &crypto_method_null,      &crypto_method_auth_psk,              &crypto_method_aes_ccm_8,             16,      16,        &crypto_method_null,            0,         &crypto_method_tls_prf_sha256},
#endif
#endif /* NX_SECURE_ENABLE_PSK_CIPHERSUITES */


};

const UINT sa2ul_crypto_ciphersuite_lookup_table_ecc_size = sizeof(sa2ul_crypto_ciphersuite_lookup_table_ecc) / sizeof(NX_SECURE_TLS_CIPHERSUITE_INFO);


/* Define the object we can pass into TLS. */
const NX_SECURE_TLS_CRYPTO sa2ul_crypto_tls_ciphers_ecc =
{
    /* Ciphersuite lookup table and size. */
    (NX_SECURE_TLS_CIPHERSUITE_INFO *)sa2ul_crypto_ciphersuite_lookup_table_ecc,
    sizeof(sa2ul_crypto_ciphersuite_lookup_table_ecc) / sizeof(NX_SECURE_TLS_CIPHERSUITE_INFO),

#ifndef NX_SECURE_DISABLE_X509
    /* X.509 certificate cipher table and size. */
    (NX_SECURE_X509_CRYPTO *)sa2ul_crypto_x509_cipher_lookup_table_ecc,
    sizeof(sa2ul_crypto_x509_cipher_lookup_table_ecc) / sizeof(NX_SECURE_X509_CRYPTO),
#endif

    /* TLS version-specific methods. */
#if (NX_SECURE_TLS_TLS_1_0_ENABLED || NX_SECURE_TLS_TLS_1_1_ENABLED)
    &crypto_method_md5,
    &crypto_method_sha1,
    &crypto_method_tls_prf_1,
#endif

#if (NX_SECURE_TLS_TLS_1_2_ENABLED)
    &crypto_method_sha256,
    &crypto_method_tls_prf_sha256,
#endif

#if (NX_SECURE_TLS_TLS_1_3_ENABLED)
    &crypto_method_hkdf,
    &crypto_method_hmac,
    &crypto_method_ecdhe,
#endif


};

#endif // #ifdef NX_SECURE_ENABLE_ECC_CIPHERSUITE



UINT nx_crypto_sa2ul_open(void)
{
    gCryptoHandle = Crypto_open(&gCryptoContext);
    if (gCryptoHandle == NULL) {
        return (NX_CRYPTO_NOT_SUCCESSFUL);
    }

    return (NX_CRYPTO_SUCCESS);
}


UINT nx_crypto_sa2ul_close(void)
{
    int res;

    res = Crypto_close(gCryptoHandle);
    if (res != SystemP_SUCCESS) {
        return (NX_CRYPTO_NOT_SUCCESSFUL);
    }

    return (NX_CRYPTO_SUCCESS);
}


UINT nx_crypto_sa2ul_asym_open(void)
{
    // TODO: instance number not always 0.
    gAsymCryptHandle = AsymCrypt_open(0u);
    if (gAsymCryptHandle == NULL) {
        return (NX_CRYPTO_NOT_SUCCESSFUL);
    }

    return (NX_CRYPTO_SUCCESS);
}


UINT nx_crypto_sa2ul_asym_close(void)
{
    int res;

    res = AsymCrypt_close(gAsymCryptHandle);
    if (res != SystemP_SUCCESS) {
        return (NX_CRYPTO_NOT_SUCCESSFUL);
    }

    return (NX_CRYPTO_SUCCESS);
}


