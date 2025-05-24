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


#include <nx_crypto.h>
#include <nx_crypto_ec.h>

#include <security_common/drivers/crypto/sa2ul/sa2ul.h>
#include <security_common/drivers/crypto/crypto.h>
#include <security_common/drivers/crypto/asym_crypt.h>

#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/CacheP.h>

#include "nx_crypto_sa2ul.h"
#include "nx_crypto_sa2ul_util.h"


#define SA2UL_CRYPTO_RSA_MAX_DATA_SIZE_IN_BYTES    (256u)

typedef struct sa2ul_crypto_rsa {
    struct AsymCrypt_RSAPubkey pub_key;
    uint32_t *t_data;
} sa2ul_crypto_rsa_t;


static UINT sa2ul_crypto_method_rsa_public(sa2ul_crypto_rsa_t *p_rsa, UCHAR *key, NX_CRYPTO_KEY_SIZE key_size_in_bits, UCHAR *input, UCHAR *output, size_t length);

static UINT sa2ul_crypto_method_rsa_private(struct AsymCrypt_RSAPrivkey *p_priv_key, UCHAR *input, UCHAR *output, size_t length, VOID *crypto_metadata, ULONG crypto_metadata_size);


static UINT sa2ul_crypto_method_rsa_init(struct NX_CRYPTO_METHOD_STRUCT *method,
                                         UCHAR *key,
                                         NX_CRYPTO_KEY_SIZE key_size_in_bits,
                                         VOID **handle,
                                         VOID *crypto_metadata,
                                         ULONG crypto_metadata_size);

static UINT sa2ul_crypto_method_rsa_operation(UINT op,
                                              VOID *handle,
                                              struct NX_CRYPTO_METHOD_STRUCT *method,
                                              UCHAR *key,
                                              NX_CRYPTO_KEY_SIZE key_size_in_bits,
                                              UCHAR *input,
                                              ULONG input_length_in_byte,
                                              UCHAR *iv_ptr,
                                              UCHAR *output,
                                              ULONG output_length_in_byte,
                                              VOID *crypto_metadata,
                                              ULONG crypto_metadata_size,
                                              VOID *packet_ptr,
                                              VOID (*nx_crypto_hw_process_callback)(VOID *packet_ptr, UINT status));

static UINT  sa2ul_crypto_method_rsa_cleanup(VOID *crypto_metadata);


const NX_CRYPTO_METHOD sa2ul_crypto_method_rsa = {

        NX_CRYPTO_KEY_EXCHANGE_RSA,
        0,
        0,
        0,
        0,
        sizeof(sa2ul_crypto_rsa_t) + (SA2UL_CRYPTO_RSA_MAX_DATA_SIZE_IN_BYTES / 4u) + 1u * 4u + 2u * SA2UL_CACHELINE_ALIGNMENT,
        sa2ul_crypto_method_rsa_init,
        sa2ul_crypto_method_rsa_cleanup,
        sa2ul_crypto_method_rsa_operation
};


static UINT sa2ul_crypto_method_rsa_init(struct NX_CRYPTO_METHOD_STRUCT *method,
                                         UCHAR *key, // public modulus
                                         NX_CRYPTO_KEY_SIZE key_size_in_bits, // public modulus size
                                         VOID **handle,
                                         VOID *crypto_metadata,
                                         ULONG crypto_metadata_size)
{
    sa2ul_crypto_rsa_t *p_rsa;
    uint8_t *p_cur_byte;

    NX_CRYPTO_PARAMETER_NOT_USED(handle);

    if (method->nx_crypto_algorithm != NX_CRYPTO_KEY_EXCHANGE_RSA) {
        return (NX_CRYPTO_INVALID_ALGORITHM);
    }

    // Make sure there is *at least* enough space for the crypto instance.
    // Further checks are performed hereafter as more memory is needed.
    if (crypto_metadata_size < sizeof(sa2ul_crypto_rsa_t)) {
        return (NX_CRYPTO_PTR_ERROR);
    }
    if (((((ULONG)crypto_metadata) & 0x3) != 0)) {
        return (NX_CRYPTO_PTR_ERROR);
    }
    p_cur_byte = (uint8_t *)crypto_metadata;

    p_rsa = (sa2ul_crypto_rsa_t *)p_cur_byte;
    p_cur_byte += sizeof(sa2ul_crypto_rsa_t);
    p_cur_byte += (SA2UL_CACHELINE_ALIGNMENT - (uintptr_t)p_cur_byte % SA2UL_CACHELINE_ALIGNMENT);

    p_rsa->t_data = (uint32_t *)p_cur_byte;
    p_cur_byte += (SA2UL_CRYPTO_RSA_MAX_DATA_SIZE_IN_BYTES / 4u) + 1u * 4u;
    p_cur_byte += (SA2UL_CACHELINE_ALIGNMENT - (uintptr_t)p_cur_byte % SA2UL_CACHELINE_ALIGNMENT);

    if ((uintptr_t)p_cur_byte - (uintptr_t)crypto_metadata > crypto_metadata_size) {
        return (NX_CRYPTO_PTR_ERROR);
    }

    /* Convert the modulus to big number format. */
    DebugP_assert(key_size_in_bits % 32u == 0u);
    stream_to_big_number(key, key_size_in_bits / 8u, &p_rsa->pub_key.n[0]);

    return (NX_CRYPTO_SUCCESS);
}


static UINT sa2ul_crypto_method_rsa_operation(UINT op,
                                              VOID *handle,
                                              struct NX_CRYPTO_METHOD_STRUCT *method,
                                              UCHAR *key,
                                              NX_CRYPTO_KEY_SIZE key_size_in_bits,
                                              UCHAR *input,
                                              ULONG input_length_in_byte,
                                              UCHAR *iv_ptr,
                                              UCHAR *output,
                                              ULONG output_length_in_byte,
                                              VOID *crypto_metadata,
                                              ULONG crypto_metadata_size,
                                              VOID *packet_ptr,
                                              VOID (*nx_crypto_hw_process_callback)(VOID *packet_ptr, UINT status))
{

    UINT res;

    NX_CRYPTO_PARAMETER_NOT_USED(handle);
    NX_CRYPTO_PARAMETER_NOT_USED(key);
    NX_CRYPTO_PARAMETER_NOT_USED(key_size_in_bits);
    NX_CRYPTO_PARAMETER_NOT_USED(output_length_in_byte);
    NX_CRYPTO_PARAMETER_NOT_USED(packet_ptr);
    NX_CRYPTO_PARAMETER_NOT_USED(nx_crypto_hw_process_callback);

    NX_CRYPTO_STATE_CHECK

    /* Verify the metadata address is 4-byte aligned. */
    if((method == NX_CRYPTO_NULL) || (crypto_metadata == NX_CRYPTO_NULL) || ((((ULONG)crypto_metadata) & 0x3) != 0))
    {
        return(NX_CRYPTO_PTR_ERROR);
    }

    if(crypto_metadata_size < sizeof(sa2ul_crypto_rsa_t)) {
        return(NX_CRYPTO_PTR_ERROR);
    }


    switch (op)
    {
        case NX_CRYPTO_SET_PRIME_P:
        {
            DebugP_assert(false);

        } break;

        case NX_CRYPTO_SET_PRIME_Q:
        {
            DebugP_assert(false);

        } break;

        case NX_SECURE_X509_KEY_TYPE_USER_DEFINED_MASK:
        {
            res = sa2ul_crypto_method_rsa_private((struct AsymCrypt_RSAPrivkey *)key, input, output, input_length_in_byte, crypto_metadata, crypto_metadata_size);
            if (res != NX_CRYPTO_SUCCESS) return (res);

        } break;

        /* Public encryption/decryption. */
        case NX_CRYPTO_ENCRYPT:
        case NX_CRYPTO_DECRYPT:
        {
            sa2ul_crypto_rsa_t *p_rsa;

            p_rsa = (sa2ul_crypto_rsa_t *)crypto_metadata;

            res = sa2ul_crypto_method_rsa_public(p_rsa, key, key_size_in_bits, input, output, input_length_in_byte);
            if (res != NX_CRYPTO_SUCCESS) return (res);

        } break;

        default:
            DebugP_assert(false);
            return (NX_CRYPTO_INVALID_ALGORITHM);
    }

    return (NX_CRYPTO_SUCCESS);
}

static UINT  sa2ul_crypto_method_rsa_cleanup(VOID *crypto_metadata)
{

    NX_CRYPTO_STATE_CHECK

#ifdef NX_SECURE_KEY_CLEAR
    if (!crypto_metadata)
        return (NX_CRYPTO_SUCCESS);

    /* Clean up the crypto metadata.  */
    NX_CRYPTO_MEMSET(crypto_metadata, 0, sizeof(sa2ul_crypto_rsa_t));
#else
    NX_CRYPTO_PARAMETER_NOT_USED(crypto_metadata);
#endif/* NX_SECURE_KEY_CLEAR  */

    return(NX_CRYPTO_SUCCESS);
}


static UINT sa2ul_crypto_method_rsa_public(sa2ul_crypto_rsa_t *p_rsa, UCHAR *key, NX_CRYPTO_KEY_SIZE key_size_in_bits, UCHAR *input, UCHAR *output, size_t length)
{
    AsymCrypt_Return_t res;

    /* The message length must match the modulus length. */
    DebugP_assert(length == 4u * p_rsa->pub_key.n[0]);

    /* Convert the exponent to big number format. */
    stream_to_big_number(key, key_size_in_bits / 8u, &p_rsa->pub_key.e[0]);

    /* Convert the input to big number format. */
    stream_to_big_number(input, length, &p_rsa->t_data[0]);

    CacheP_wb((void *)&p_rsa->t_data[0], length, CacheP_TYPE_ALLD);
    CacheP_inv((void *)output, length, CacheP_TYPE_ALLD);

    res = AsymCrypt_RSAPublic(gAsymCryptHandle, (uint32_t *)&p_rsa->t_data[0], &p_rsa->pub_key, (uint32_t *)output);
    if (res != ASYM_CRYPT_RETURN_SUCCESS) return (NX_CRYPTO_NOT_SUCCESSFUL);

    big_number_to_stream((uint32_t *)&output[0], (UCHAR *)&p_rsa->t_data[0]);
    memcpy((void *)&output[0], (const void *)&p_rsa->t_data[0], length);

    return (NX_CRYPTO_SUCCESS);
}


static UINT sa2ul_crypto_method_rsa_private(struct AsymCrypt_RSAPrivkey *p_priv_key, UCHAR *input, UCHAR *output, size_t length, VOID *crypto_metadata, ULONG crypto_metadata_size)
{
    AsymCrypt_Return_t res;

    /* The message length must match the modulus length. */
    DebugP_assert(length == 4u * p_priv_key->n[0]);

    /* Convert the input to big number format. */
    stream_to_big_number(input, length, crypto_metadata);

    CacheP_wb(crypto_metadata, length, CacheP_TYPE_ALLD);
    CacheP_inv((void *)output, length, CacheP_TYPE_ALLD);

    res = AsymCrypt_RSAPrivate(gAsymCryptHandle, (const uint32_t *)crypto_metadata, p_priv_key, (uint32_t *)output);
    if (res != ASYM_CRYPT_RETURN_SUCCESS) return (NX_CRYPTO_NOT_SUCCESSFUL);

    big_number_to_stream((uint32_t *)&output[0], (UCHAR *)crypto_metadata);
    memcpy((void *)&output[0], crypto_metadata, length);

    return (NX_CRYPTO_SUCCESS);
}


