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
#include <nx_secure_x509.h>

#include <security_common/drivers/crypto/sa2ul/sa2ul.h>
#include <security_common/drivers/crypto/crypto.h>
#include <security_common/drivers/crypto/asym_crypt.h>

#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/CacheP.h>

#include "nx_crypto_sa2ul.h"
#include "nx_crypto_sa2ul_util.h"


typedef struct sa2ul_crypto_ecdsa {
    struct AsymCrypt_ECPrimeCurveP curve_params;
} sa2ul_crypto_ecdsa_t;



static UINT sa2ul_crypto_method_ecdsa_curve_set(sa2ul_crypto_ecdsa_t *p_ecdsa, const NX_CRYPTO_EC *p_ec);

static UINT sa2ul_crypto_method_sig_data_parse(UCHAR t_sig_data[], size_t sig_data_size, struct AsymCrypt_ECDSASig *p_sig);

static UINT sa2ul_crypto_method_ecdsa_pub_key_parse(const UCHAR *t_key_data, size_t key_data_size, struct AsymCrypt_ECPoint *p_key);

static UINT sa2ul_crypto_method_ecdsa_verify(sa2ul_crypto_ecdsa_t *p_ecdsa, UCHAR *t_key_data, size_t key_data_size,
                                             UCHAR t_hash[], size_t hash_size, UCHAR t_sig_data[], size_t sig_data_size);

static UINT sa2ul_crypto_method_ecdsa_sign(sa2ul_crypto_ecdsa_t *p_ecdsa, const UCHAR t_priv_key_data[], size_t key_data_size, const UCHAR t_hash_data[],
                                           ULONG hash_data_size, UCHAR t_sig_data[], ULONG max_sig_data_len, ULONG *p_actual_sig_data_len);


static UINT sa2ul_crypto_method_ecdsa_init(struct NX_CRYPTO_METHOD_STRUCT *method,
                                           UCHAR *key,
                                           NX_CRYPTO_KEY_SIZE key_size_in_bits,
                                           VOID **handle,
                                           VOID *crypto_metadata,
                                           ULONG crypto_metadata_size);


static UINT sa2ul_crypto_method_ecdsa_operation(UINT op,
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


static UINT  sa2ul_crypto_method_ecdsa_cleanup(VOID *crypto_metadata);


const NX_CRYPTO_METHOD sa2ul_crypto_method_ecdsa = {

        NX_CRYPTO_DIGITAL_SIGNATURE_ECDSA,
        0,
        0,
        0,
        0,
        sizeof(sa2ul_crypto_ecdsa_t),
        sa2ul_crypto_method_ecdsa_init,
        sa2ul_crypto_method_ecdsa_cleanup,
        sa2ul_crypto_method_ecdsa_operation
};



static UINT sa2ul_crypto_method_ecdsa_init(struct NX_CRYPTO_METHOD_STRUCT *method,
                                           UCHAR *key,
                                           NX_CRYPTO_KEY_SIZE key_size_in_bits,
                                           VOID **handle,
                                           VOID *crypto_metadata,
                                           ULONG crypto_metadata_size)
{
    NX_CRYPTO_PARAMETER_NOT_USED(key);
    NX_CRYPTO_PARAMETER_NOT_USED(key_size_in_bits);
    NX_CRYPTO_PARAMETER_NOT_USED(handle);

    NX_CRYPTO_STATE_CHECK

    if ((method == NX_CRYPTO_NULL) || (crypto_metadata == NX_CRYPTO_NULL)) {
        return(NX_CRYPTO_PTR_ERROR);
    }

    /* Verify the metadata address is 4-byte aligned. */
    if((((ULONG)crypto_metadata) & 0x3) != 0) {
        return(NX_CRYPTO_PTR_ERROR);
    }

    if(crypto_metadata_size < sizeof(sa2ul_crypto_ecdsa_t)) {
        return(NX_CRYPTO_PTR_ERROR);
    }

    return(NX_CRYPTO_SUCCESS);
}


static UINT sa2ul_crypto_method_ecdsa_operation(UINT op,
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
    sa2ul_crypto_ecdsa_t *p_ecdsa;
    UINT status;

    NX_CRYPTO_PARAMETER_NOT_USED(handle);
    NX_CRYPTO_PARAMETER_NOT_USED(packet_ptr);
    NX_CRYPTO_PARAMETER_NOT_USED(nx_crypto_hw_process_callback);

    NX_CRYPTO_STATE_CHECK

    /* Verify the metadata address is 4-byte aligned. */
    if((method == NX_CRYPTO_NULL) || (crypto_metadata == NX_CRYPTO_NULL) || ((((ULONG)crypto_metadata) & 0x3) != 0)) {
        return(NX_CRYPTO_PTR_ERROR);
    }

    if(crypto_metadata_size < sizeof(sa2ul_crypto_ecdsa_t)) {
        return(NX_CRYPTO_PTR_ERROR);
    }

    p_ecdsa = (sa2ul_crypto_ecdsa_t *)crypto_metadata;

    switch (op) {

        case NX_CRYPTO_EC_CURVE_SET:
        {
            NX_CRYPTO_EC *p_ec_curve;

            status = ((NX_CRYPTO_METHOD *)input)->nx_crypto_operation(NX_CRYPTO_EC_CURVE_GET,
                                                                      NX_CRYPTO_NULL,
                                                                      (NX_CRYPTO_METHOD *)input,
                                                                      NX_CRYPTO_NULL, 0,
                                                                      NX_CRYPTO_NULL, 0,
                                                                      NX_CRYPTO_NULL,
                                                                      (UCHAR *)&p_ec_curve,
                                                                      sizeof(UCHAR *),
                                                                      NX_CRYPTO_NULL, 0,
                                                                      NX_CRYPTO_NULL, NX_CRYPTO_NULL);
            if (status != NX_CRYPTO_SUCCESS) {
                return (status);
            }

            status = sa2ul_crypto_method_ecdsa_curve_set(p_ecdsa, p_ec_curve);
            if (status != NX_CRYPTO_SUCCESS) {
                return (status);
            }

        } break;

        case NX_CRYPTO_AUTHENTICATE:
        {
            NX_CRYPTO_EXTENDED_OUTPUT *extended_output;

            extended_output = (NX_CRYPTO_EXTENDED_OUTPUT *)output;

            DebugP_assert(key_size_in_bits % 8 == 0);
            status = sa2ul_crypto_method_ecdsa_sign(p_ecdsa, &key[0], key_size_in_bits / 8u, &input[0], input_length_in_byte, &extended_output->nx_crypto_extended_output_data[0],
                                                    extended_output->nx_crypto_extended_output_length_in_byte, &extended_output->nx_crypto_extended_output_actual_size);
            if (status != NX_CRYPTO_SUCCESS) {
                return (status);
            }

        } break;

        case NX_CRYPTO_VERIFY:
        {
            DebugP_assert(key_size_in_bits % 8 == 0);
            status = sa2ul_crypto_method_ecdsa_verify(p_ecdsa, key, key_size_in_bits / 8u, input, input_length_in_byte, &output[0], output_length_in_byte);
            if (status != NX_CRYPTO_SUCCESS) {
                return (status);
            }

        } break;

        default:
            DebugP_assert(false);
            return (NX_CRYPTO_INVALID_ALGORITHM);
    }

    return (NX_CRYPTO_SUCCESS);
}


static UINT sa2ul_crypto_method_ecdsa_cleanup(VOID *crypto_metadata)
{
    NX_CRYPTO_STATE_CHECK

#ifdef NX_SECURE_KEY_CLEAR
    if (!crypto_metadata)
        return (NX_CRYPTO_SUCCESS);

    /* Clean up the crypto metadata.  */
    NX_CRYPTO_MEMSET(crypto_metadata, 0, sizeof(sa2ul_crypto_ecdsa_t));
#else
    NX_CRYPTO_PARAMETER_NOT_USED(crypto_metadata);
#endif/* NX_SECURE_KEY_CLEAR  */

    return(NX_CRYPTO_SUCCESS);
}



static UINT sa2ul_crypto_method_ecdsa_curve_set(sa2ul_crypto_ecdsa_t *p_ecdsa, const NX_CRYPTO_EC *p_ec)
{
    /* Only p256r1 curve for now. */
    DebugP_assert(p_ec->nx_crypto_ec_id == NX_CRYPTO_EC_SECP256R1);
    DebugP_assert(p_ec->nx_crypto_ec_field.fp.nx_crypto_huge_buffer_size <= EC_PARAM_MAXLEN - 1u); /* -1 to account for the word count. */

    /* P */
    memset(&p_ecdsa->curve_params.prime[0], 0, EC_PARAM_MAXLEN * sizeof(p_ecdsa->curve_params.prime[0]));
    p_ecdsa->curve_params.prime[0] = p_ec->nx_crypto_ec_field.fp.nx_crypto_huge_number_size;
    memcpy(&p_ecdsa->curve_params.prime[1],
           &p_ec->nx_crypto_ec_field.fp.nx_crypto_huge_number_data[0],
           p_ec->nx_crypto_ec_field.fp.nx_crypto_huge_buffer_size);

    /* N */
    memset(&p_ecdsa->curve_params.order[0], 0, EC_PARAM_MAXLEN * sizeof(p_ecdsa->curve_params.order[0]));
    p_ecdsa->curve_params.order[0] = p_ec->nx_crypto_ec_n.nx_crypto_huge_number_size;
    memcpy(&p_ecdsa->curve_params.order[1],
           &p_ec->nx_crypto_ec_n.nx_crypto_huge_number_data[0],
           p_ec->nx_crypto_ec_n.nx_crypto_huge_buffer_size);

    /* A */
    memset(&p_ecdsa->curve_params.a[0], 0, EC_PARAM_MAXLEN * sizeof(p_ecdsa->curve_params.a[0]));
    p_ecdsa->curve_params.a[0] = p_ec->nx_crypto_ec_a.nx_crypto_huge_number_size;
    memcpy(&p_ecdsa->curve_params.a[1],
           &p_ec->nx_crypto_ec_a.nx_crypto_huge_number_data[0],
           p_ec->nx_crypto_ec_a.nx_crypto_huge_buffer_size);

    /* B */
    memset(&p_ecdsa->curve_params.b[0], 0, EC_PARAM_MAXLEN * sizeof(p_ecdsa->curve_params.b[0]));
    p_ecdsa->curve_params.b[0] = p_ec->nx_crypto_ec_b.nx_crypto_huge_number_size;
    memcpy(&p_ecdsa->curve_params.b[1],
           &p_ec->nx_crypto_ec_b.nx_crypto_huge_number_data[0],
           p_ec->nx_crypto_ec_b.nx_crypto_huge_buffer_size);

    /* X */
    memset(&p_ecdsa->curve_params.g.x[0], 0, EC_PARAM_MAXLEN * sizeof(p_ecdsa->curve_params.g.x[0]));
    p_ecdsa->curve_params.g.x[0] = p_ec->nx_crypto_ec_g.nx_crypto_ec_point_x.nx_crypto_huge_number_size;
    memcpy(&p_ecdsa->curve_params.g.x[1],
           &p_ec->nx_crypto_ec_g.nx_crypto_ec_point_x.nx_crypto_huge_number_data[0],
           p_ec->nx_crypto_ec_g.nx_crypto_ec_point_x.nx_crypto_huge_buffer_size);

    /* Y */
    memset(&p_ecdsa->curve_params.g.y[0], 0, EC_PARAM_MAXLEN * sizeof(p_ecdsa->curve_params.g.y[0]));
    p_ecdsa->curve_params.g.y[0] = p_ec->nx_crypto_ec_g.nx_crypto_ec_point_y.nx_crypto_huge_number_size;
    memcpy(&p_ecdsa->curve_params.g.y[1],
           &p_ec->nx_crypto_ec_g.nx_crypto_ec_point_y.nx_crypto_huge_number_data[0],
           p_ec->nx_crypto_ec_g.nx_crypto_ec_point_y.nx_crypto_huge_buffer_size);


    return (NX_CRYPTO_SUCCESS);
}


static UINT sa2ul_crypto_method_sig_data_parse(UCHAR t_sig_data[], size_t sig_data_size, struct AsymCrypt_ECDSASig *p_sig)
{
    UCHAR *p_cur_sig_byte;
    size_t r_length;
    size_t s_length;

    /* The byte sequence must start with 0x30. */
    p_cur_sig_byte = &t_sig_data[0];
    if (*p_cur_sig_byte != 0x30) {
        return(NX_CRYPTO_AUTHENTICATION_FAILED);
    }
    p_cur_sig_byte++;

    /* The length should fit in 7 bits (single-byte value). */
    if (*p_cur_sig_byte & 0x80) { /* Multi-byte length. */
        return(NX_CRYPTO_AUTHENTICATION_FAILED);
    }

    /* Make sure the length of the signature is consistent with the total length. */
    if (sig_data_size != (*p_cur_sig_byte + 2u)) {
        return(NX_CRYPTO_SIZE_ERROR);
    }
    p_cur_sig_byte++;

    /* Make sure the type of the 'r' TLV is INTEGER (0x2). */
    if (*p_cur_sig_byte != 0x2) {
        return(NX_CRYPTO_SIZE_ERROR);
    }
    p_cur_sig_byte++;

    r_length = *p_cur_sig_byte;
    p_cur_sig_byte++;

    /* Check that the length makes some sense. */
    if (r_length >= sig_data_size - (p_cur_sig_byte - t_sig_data)) {
        return(NX_CRYPTO_SIZE_ERROR);
    }

    // Convert r to big number.
    stream_to_big_number(p_cur_sig_byte, r_length, &p_sig->r[0]);
    p_cur_sig_byte += r_length;

    /* Make sure the type of the 's' TLV is INTEGER (0x2). */
    if (*p_cur_sig_byte != 0x2) {
        return(NX_CRYPTO_SIZE_ERROR);
    }
    p_cur_sig_byte++;

    s_length = *p_cur_sig_byte;
    p_cur_sig_byte++;

    /* Check that the length of s fits with the remaining size. */
    if (s_length != sig_data_size - (p_cur_sig_byte - t_sig_data)) {
        return(NX_CRYPTO_SIZE_ERROR);
    }

    // Convert s to big number.
    stream_to_big_number(p_cur_sig_byte, s_length, &p_sig->s[0]);
    p_cur_sig_byte += s_length;

    return (NX_CRYPTO_SUCCESS);
}


static UINT sa2ul_crypto_method_ecdsa_pub_key_parse(const UCHAR *t_key_data, size_t key_data_size, struct AsymCrypt_ECPoint *p_key)
{
    /* Only uncompressed format is supported. */
    if (t_key_data[0] != 0x04) {
        return(NX_CRYPTO_FORMAT_NOT_SUPPORTED);
    }

    stream_to_big_number(&t_key_data[1], key_data_size / 2u, &p_key->x[0]);
    stream_to_big_number(&t_key_data[1 + key_data_size / 2], key_data_size / 2u, &p_key->y[0]);

    return (NX_CRYPTO_SUCCESS);
}


static UINT sa2ul_crypto_method_ecdsa_verify(sa2ul_crypto_ecdsa_t *p_ecdsa, UCHAR *t_key_data, size_t key_data_size,
                                             UCHAR t_hash[], size_t hash_size, UCHAR t_sig_data[], size_t sig_data_size)
{
    struct AsymCrypt_ECPoint pub_key;
    struct AsymCrypt_ECDSASig sig;
    uint32_t t_hash_big_num[ECDSA_MAX_LENGTH];
    int32_t res;
    UINT status;

    status = sa2ul_crypto_method_sig_data_parse(t_sig_data, sig_data_size, &sig);
    if (status != NX_CRYPTO_SUCCESS) return (status);

    status = sa2ul_crypto_method_ecdsa_pub_key_parse(t_key_data, key_data_size, &pub_key);
    if (status != NX_CRYPTO_SUCCESS) return (status);

    DebugP_assert(sig.s[0] <= sizeof(t_hash_big_num));
    DebugP_assert(sig.s[0] <= hash_size);

    memset(&t_hash_big_num[0], 0, sizeof(t_hash_big_num));
    stream_to_big_number(&t_hash[0], hash_size, &t_hash_big_num[0]);

    /* Truncate/zero extend hash and cure parameters. */
    t_hash_big_num[0] = sig.s[0];
    sig.r[0] = sig.s[0];
    p_ecdsa->curve_params.prime[0] = sig.s[0];
    p_ecdsa->curve_params.order[0] = sig.s[0];
    p_ecdsa->curve_params.a[0] = sig.s[0];
    p_ecdsa->curve_params.b[0] = sig.s[0];
    p_ecdsa->curve_params.g.x[0] = sig.s[0];
    p_ecdsa->curve_params.g.y[0] = sig.s[0];
    res = AsymCrypt_ECDSAVerify(gAsymCryptHandle, &p_ecdsa->curve_params, &pub_key, &sig, &t_hash_big_num[0]);
    DebugP_assert(ASYM_CRYPT_RETURN_SUCCESS == res);

    return (NX_CRYPTO_SUCCESS);
}


static UINT sa2ul_crypto_method_ecdsa_sign(sa2ul_crypto_ecdsa_t *p_ecdsa, const UCHAR t_priv_key_data[], size_t key_data_size, const UCHAR t_hash_data[],
                                           ULONG hash_data_size, UCHAR t_sig_data[], ULONG max_sig_data_len, ULONG *p_actual_sig_data_len)
{
    uint32_t t_priv[ECDSA_MAX_LENGTH];
    uint32_t t_rand[ECDSA_MAX_LENGTH];
    uint32_t t_hash[ECDSA_MAX_LENGTH];
    size_t r_size;
    size_t s_size;
    UCHAR *p_sig_r;
    UCHAR *p_sig_s;
    UCHAR *p_sig;
    UINT pad_zero_r;
    UINT pad_zero_s;
    UINT sequence_size;
    struct AsymCrypt_ECDSASig sig;
    int32_t res;
    UINT status;


    /* Make sure the given key is not too long. */
    DebugP_assert((key_data_size + 3u) / 4u + 1u <= ECDSA_MAX_LENGTH);


    /* Get random number with specified length. */
    t_rand[0] = (key_data_size + 3u) / 4u;
    status = NX_CRYPTO_RBG(8u * key_data_size, (UCHAR *)&t_rand[1]);
    if (status != NX_CRYPTO_SUCCESS) return (status);

    if (hash_data_size > key_data_size) {
        hash_data_size = key_data_size;
    }

    stream_to_big_number(&t_priv_key_data[0], key_data_size, &t_priv[0]);
    stream_to_big_number(&t_hash_data[0], hash_data_size, &t_hash[0]);

    res = AsymCrypt_ECDSASign(gAsymCryptHandle, &p_ecdsa->curve_params, &t_priv[0], &t_rand[0], &t_hash[0], &sig);
    if (res != ASYM_CRYPT_RETURN_SUCCESS) return (NX_CRYPTO_NOT_SUCCESSFUL);


    r_size = 4u * sig.r[0];
    s_size = 4u * sig.s[0];

    DebugP_assert(r_size <= key_data_size);
    DebugP_assert(s_size <= key_data_size);

    p_sig_r = &t_sig_data[3];
    big_number_to_stream(&sig.r[0], p_sig_r);

#if 0
    /* Output r and s as two INTEGER in ASN.1 encoding */
    signature_r = signature + 3;
    status = _nx_crypto_huge_number_extract(&pt.nx_crypto_ec_point_x, signature_r, (curve_size + 3), &r_size);
    if (status != NX_CRYPTO_SUCCESS)
    {
        return(status);
    }
#endif

    p_sig_s = &t_sig_data[key_data_size + 6u];
    big_number_to_stream(&sig.s[0], p_sig_s);

#if 0
    signature_s = signature + (key_data_size + 6);
    status = _nx_crypto_huge_number_extract(&temp, signature_s, (curve_size + 3), &s_size);
    if (status != NX_CRYPTO_SUCCESS)
    {
        return(status);
    }
#endif

    /* Trim prefix zeros. */
    while (r_size > 0u) {
        if (*p_sig_r) {
            break;
        }
        p_sig_r++;
        r_size--;
    }

    /* The most significant bit must be zero to indicate positive integer. */
    /* Pad zero at the front if necessary. */
    pad_zero_r = (*p_sig_r & 0x80) ? 1 : 0;


    while (s_size > 0u) {
        if (*p_sig_s) {
            break;
        }
        p_sig_s++;
        s_size--;
    }

    /* The most significant bit must be zero to indicate positive integer. */
    /* Pad zero at the front if necessary. */
    pad_zero_s = (*p_sig_s & 0x80) ? 1 : 0;

    /* Size of sequence. */
    sequence_size = r_size + pad_zero_r + s_size + pad_zero_s + 4;
    DebugP_assert(sequence_size < 0x80);

   *p_actual_sig_data_len = sequence_size + 2;

    p_sig = &t_sig_data[0];

    /* Sequence start. */
   *p_sig = 0x30;
    p_sig++;

    /* Sequence size. */
   *p_sig = (UCHAR)sequence_size;
    p_sig++;


    /* Remove trailing zeros and make room for type and size. */
    NX_CRYPTO_MEMMOVE(p_sig + 2u + pad_zero_r, p_sig_r, r_size); /* Use case of memmove is verified. */

    /* Integer. */
   *p_sig = 0x02;
    p_sig++;

    /* Size. */
   *p_sig = (UCHAR)(r_size + pad_zero_r);
    p_sig++;
    if (pad_zero_r) {
       *p_sig = 0;
        p_sig++;
    }

    /* Increment past r. */
    p_sig += r_size;

    /* Setup s. */
    NX_CRYPTO_MEMMOVE(p_sig + 2u + pad_zero_s, p_sig_s, s_size); /* Use case of memmove is verified. */

    /* Integer. */
   *p_sig = 0x02;
    p_sig++;

    /* Size. */
   *p_sig = (UCHAR)(s_size + pad_zero_s);
    p_sig++;
    if (pad_zero_s) {
       *p_sig = 0;
        p_sig++;
    }

    return (NX_CRYPTO_SUCCESS);
}






