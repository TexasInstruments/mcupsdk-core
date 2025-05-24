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



#define SA2UL_CRYPTO_AES_IS_ALIGNED_PTR(ptr, align)         ((((uint32_t)ptr) & ((align)-1)) == 0)

#define SA2UL_CRYPTO_AES_IV_LEN_IN_BITS                (128u)
#define SA2UL_CRYPTO_AES_128_KEY_LEN_IN_BITS           (128u)
#define SA2UL_CRYPTO_AES_256_KEY_LEN_IN_BITS           (256u)
#define SA2UL_CRYPTO_AES_MAX_KEY_SIZE_IN_BYTES          (32u)
#define SA2UL_CRYPTO_AES_CBC_BLOCK_SIZE_IN_BYTES        (16u)


typedef struct sa2ul_crypto_aes_cbc {
    uint8_t t_key[SA2UL_CRYPTO_AES_MAX_KEY_SIZE_IN_BYTES];
    uint8_t *last_block;
} sa2ul_crypto_aes_cbc_t;


typedef enum sa2ul_crypto_aes_op {
    SA2UL_CRYPTO_AES_OP_ENCRYPT,
    SA2UL_CRYPTO_AES_OP_DECRYPT
} sa2ul_crypto_aes_op_t;


int sa2ul_crypto_method_aes_cbc_encrypt(sa2ul_crypto_aes_cbc_t *p_aes_cbc, UCHAR *input, UCHAR *output, UINT length, UCHAR block_size);
int sa2ul_crypto_method_aes_cbc_decrypt(sa2ul_crypto_aes_cbc_t *p_aes_cbc, UCHAR *input, UCHAR *output, UINT length, UCHAR block_size);
int sa2ul_crypto_method_aes_cbc_encrypt_decrypt_init(sa2ul_crypto_aes_cbc_t *p_aes_cbc, UCHAR *iv, UINT iv_len);


static UINT sa2ul_crypto_method_aes_init(struct NX_CRYPTO_METHOD_STRUCT *method,
                                         UCHAR *key,
                                         NX_CRYPTO_KEY_SIZE key_size_in_bits,
                                         VOID **handler,
                                         VOID *crypto_metadata,
                                         ULONG crypto_metadata_size);

static UINT sa2ul_crypto_method_aes_cleanup(VOID *crypto_metadata);

static UINT sa2ul_crypto_method_aes_cbc_operation(UINT op,
                                                  VOID *handler,
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



/* Declare the AES-CBC 128 encrytion method. */
const NX_CRYPTO_METHOD sa2ul_crypto_method_aes_cbc_128 = {

        NX_CRYPTO_ENCRYPTION_AES_CBC,
        SA2UL_CRYPTO_AES_128_KEY_LEN_IN_BITS,
        SA2UL_CRYPTO_AES_IV_LEN_IN_BITS,
        0,
        SA2UL_CRYPTO_AES_CBC_BLOCK_SIZE_IN_BYTES,
        sizeof(sa2ul_crypto_aes_cbc_t) + SA2UL_CRYPTO_AES_CBC_BLOCK_SIZE_IN_BYTES + SA2UL_CACHELINE_ALIGNMENT,
        sa2ul_crypto_method_aes_init,
        sa2ul_crypto_method_aes_cleanup,
        sa2ul_crypto_method_aes_cbc_operation
};


/* Declare the AES-CBC 256 encryption method */
const NX_CRYPTO_METHOD sa2ul_crypto_method_aes_cbc_256 = {

        NX_CRYPTO_ENCRYPTION_AES_CBC,
        SA2UL_CRYPTO_AES_256_KEY_LEN_IN_BITS,
        SA2UL_CRYPTO_AES_IV_LEN_IN_BITS,
        0,
        SA2UL_CRYPTO_AES_CBC_BLOCK_SIZE_IN_BYTES,
        sizeof(sa2ul_crypto_aes_cbc_t) + SA2UL_CRYPTO_AES_CBC_BLOCK_SIZE_IN_BYTES + SA2UL_CACHELINE_ALIGNMENT,
        sa2ul_crypto_method_aes_init,
        sa2ul_crypto_method_aes_cleanup,
        sa2ul_crypto_method_aes_cbc_operation
};




static UINT sa2ul_crypto_method_aes_init(struct NX_CRYPTO_METHOD_STRUCT *method,
                                         UCHAR *key,
                                         NX_CRYPTO_KEY_SIZE key_size_in_bits,
                                         VOID **handle,
                                         VOID *crypto_metadata,
                                         ULONG crypto_metadata_size)
{
    sa2ul_crypto_aes_cbc_t *p_aes_cbc;
    uint8_t *p_cur_byte;

    NX_CRYPTO_PARAMETER_NOT_USED(handle);

    // Make sure there is *at least* enough space for the crypto instance.
    // Further checks are performed hereafter as more memory is needed.
    if (crypto_metadata_size < sizeof(sa2ul_crypto_aes_cbc_t)) {
        return (NX_CRYPTO_PTR_ERROR);
    }
    if (((((ULONG)crypto_metadata) & 0x3) != 0)) {
        return (NX_CRYPTO_PTR_ERROR);
    }
    p_cur_byte = (uint8_t *)crypto_metadata;

    p_aes_cbc = (sa2ul_crypto_aes_cbc_t *)p_cur_byte;
    p_cur_byte += sizeof(sa2ul_crypto_aes_cbc_t);
    p_cur_byte += (SA2UL_CACHELINE_ALIGNMENT - (uintptr_t)p_cur_byte % SA2UL_CACHELINE_ALIGNMENT);

    p_aes_cbc->last_block = p_cur_byte;
    p_cur_byte += SA2UL_CRYPTO_AES_CBC_BLOCK_SIZE_IN_BYTES;

    if ((uintptr_t)p_cur_byte - (uintptr_t)crypto_metadata > crypto_metadata_size) {
        return (NX_CRYPTO_PTR_ERROR);
    }

    if ((key_size_in_bits != SA2UL_CRYPTO_AES_128_KEY_LEN_IN_BITS) && (key_size_in_bits != SA2UL_CRYPTO_AES_256_KEY_LEN_IN_BITS)) {
        return(NX_CRYPTO_UNSUPPORTED_KEY_SIZE);
    }

    /* Copy the key. */
    NX_CRYPTO_MEMCPY(&p_aes_cbc->t_key[0], key, (key_size_in_bits >> 3));

    return (NX_CRYPTO_SUCCESS);
}


static UINT sa2ul_crypto_method_aes_cleanup(VOID *crypto_metadata)
{
#ifdef NX_SECURE_KEY_CLEAR
    /* Clean up the crypto metadata.  */
    NX_CRYPTO_MEMSET(crypto_metadata, 0, sizeof(sa2ul_crypto_aes_cbc_t));
#else
    NX_CRYPTO_PARAMETER_NOT_USED(crypto_metadata);
#endif /* NX_SECURE_KEY_CLEAR  */

    return(NX_CRYPTO_SUCCESS);
}


static UINT sa2ul_crypto_method_aes_cbc_operation(UINT op,
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
    sa2ul_crypto_aes_cbc_t *p_aes_cbc;
    int res;

    NX_CRYPTO_PARAMETER_NOT_USED(handle);
    NX_CRYPTO_PARAMETER_NOT_USED(key);
    NX_CRYPTO_PARAMETER_NOT_USED(key_size_in_bits);
    NX_CRYPTO_PARAMETER_NOT_USED(output_length_in_byte);
    NX_CRYPTO_PARAMETER_NOT_USED(packet_ptr);
    NX_CRYPTO_PARAMETER_NOT_USED(nx_crypto_hw_process_callback);
    
    NX_CRYPTO_STATE_CHECK

    /* Verify the metadata address is 4-byte aligned. */
    if((method == NX_CRYPTO_NULL) || (crypto_metadata == NX_CRYPTO_NULL) || ((((ULONG)crypto_metadata) & 0x3) != 0)) {
        return(NX_CRYPTO_PTR_ERROR);
    }

    if(crypto_metadata_size < sizeof(sa2ul_crypto_aes_cbc_t)) {
        return(NX_CRYPTO_PTR_ERROR);
    }

    p_aes_cbc = (sa2ul_crypto_aes_cbc_t *)crypto_metadata;

    switch (op)
    {
        case NX_CRYPTO_DECRYPT:
        {
            res = sa2ul_crypto_method_aes_cbc_encrypt_decrypt_init(p_aes_cbc, iv_ptr, method->nx_crypto_IV_size_in_bits >> 3);
            if (res != NX_CRYPTO_SUCCESS) return (res);

            res = sa2ul_crypto_method_aes_cbc_decrypt(p_aes_cbc, &input[0], &output[0], input_length_in_byte, SA2UL_CRYPTO_AES_CBC_BLOCK_SIZE_IN_BYTES);
            if (res != NX_CRYPTO_SUCCESS) return (res);

        } break;

        case NX_CRYPTO_ENCRYPT:
        {
            res = sa2ul_crypto_method_aes_cbc_encrypt_decrypt_init(p_aes_cbc, iv_ptr, method->nx_crypto_IV_size_in_bits >> 3);
            if (res != NX_CRYPTO_SUCCESS) return (res);

            res = sa2ul_crypto_method_aes_cbc_encrypt(p_aes_cbc, &input[0], &output[0], input_length_in_byte, SA2UL_CRYPTO_AES_CBC_BLOCK_SIZE_IN_BYTES);
            if (res != NX_CRYPTO_SUCCESS) return (res);
            
        } break;

        case NX_CRYPTO_DECRYPT_INITIALIZE:
        {
            res = sa2ul_crypto_method_aes_cbc_encrypt_decrypt_init(p_aes_cbc, iv_ptr, method->nx_crypto_IV_size_in_bits >> 3);
            if (res != NX_CRYPTO_SUCCESS) return (res);

        } break;

        case NX_CRYPTO_ENCRYPT_INITIALIZE:
        {
            res = sa2ul_crypto_method_aes_cbc_encrypt_decrypt_init(p_aes_cbc, iv_ptr, method->nx_crypto_IV_size_in_bits >> 3);
            if (res != NX_CRYPTO_SUCCESS) return (res);

        } break;

        case NX_CRYPTO_DECRYPT_UPDATE:
        {
            res = sa2ul_crypto_method_aes_cbc_decrypt(p_aes_cbc, &input[0], &output[0], input_length_in_byte, SA2UL_CRYPTO_AES_CBC_BLOCK_SIZE_IN_BYTES);
            if (res != NX_CRYPTO_SUCCESS) return (res);
            
        } break;

        case NX_CRYPTO_ENCRYPT_UPDATE:
        {
            res = sa2ul_crypto_method_aes_cbc_encrypt(p_aes_cbc, &input[0], &output[0], input_length_in_byte, SA2UL_CRYPTO_AES_CBC_BLOCK_SIZE_IN_BYTES);
            if (res != NX_CRYPTO_SUCCESS) return (res);
            
        } break;

        case NX_CRYPTO_ENCRYPT_CALCULATE:
        case NX_CRYPTO_DECRYPT_CALCULATE:

            break;

        default:
            return (NX_CRYPTO_INVALID_ALGORITHM);
    }

    return (NX_CRYPTO_SUCCESS);
}


static int sa2ul_crypto_method_aes_cbc_op(sa2ul_crypto_aes_cbc_t *p_aes_cbc, uint8_t op, UCHAR *input, UCHAR *output, UINT length, UCHAR block_size)
{
    SA2UL_ContextParams ctxParams;
    uint32_t status;
    uintptr_t first_input_line_addr;
    uintptr_t next_input_line_addr;
    uintptr_t first_output_line_addr;
    size_t cache_len;

    if (block_size > SA2UL_CRYPTO_AES_CBC_BLOCK_SIZE_IN_BYTES) {
        return(NX_CRYPTO_PTR_ERROR);
    }

    /* Configure secure context */
    ctxParams.opType       = SA2UL_OP_ENC;
    ctxParams.encAlg       = SA2UL_ENC_ALG_AES;
    ctxParams.encMode      = SA2UL_ENC_MODE_CBC;
    ctxParams.encKeySize   = SA2UL_ENC_KEYSIZE_256;
    ctxParams.encDirection = (op == SA2UL_CRYPTO_AES_OP_ENCRYPT ? SA2UL_ENC_DIR_ENCRYPT : SA2UL_ENC_DIR_DECRYPT);
    NX_CRYPTO_MEMCPY(&ctxParams.key[0], &p_aes_cbc->t_key[0], SA2UL_CRYPTO_AES_MAX_KEY_SIZE_IN_BYTES);
    NX_CRYPTO_MEMCPY(&ctxParams.iv[0], &p_aes_cbc->last_block[0], block_size);
    ctxParams.inputLen = length;

    gSa2ulCtxObj.totalLengthInBytes = length;

    // Save the last encrypted block which will become the next decryption IV
    if (op == SA2UL_CRYPTO_AES_OP_DECRYPT) {
        NX_CRYPTO_MEMCPY(&p_aes_cbc->last_block[0], &input[length - block_size], block_size);
    }

    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Flush input array and invalidate output array. The DMA is expected to handle arbitrary alignments but the input/output
     * buffers should be cache-aligned nonetheless. Therefore, we round the input/output addresses down to the nearest cache line.
     * The rounded down address is expected to be within the containing packet as the cache line is 16 bytes and the start of
     * the packet is assuredly behind the payload address by more than 16 bytes.
     */
    first_input_line_addr = (((uintptr_t)input / SA2UL_CACHELINE_ALIGNMENT) * SA2UL_CACHELINE_ALIGNMENT);
    first_output_line_addr = (((uintptr_t)output / SA2UL_CACHELINE_ALIGNMENT) * SA2UL_CACHELINE_ALIGNMENT);
    next_input_line_addr = ((((uintptr_t)input + length - 1u) / SA2UL_CACHELINE_ALIGNMENT) + 1u) * SA2UL_CACHELINE_ALIGNMENT;
    cache_len = next_input_line_addr - first_input_line_addr;

    CacheP_wb((void *)first_input_line_addr, cache_len, CacheP_TYPE_ALLD);
    CacheP_inv((void *)first_output_line_addr, cache_len, CacheP_TYPE_ALLD);

    /* Encryption */
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj, &input[0], length, &output[0]);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Store the last cipher block for next round. */
    if (op == SA2UL_CRYPTO_AES_OP_ENCRYPT) {
        NX_CRYPTO_MEMCPY(&p_aes_cbc->last_block[0], &output[length - block_size], block_size);
    }

    return (NX_CRYPTO_SUCCESS);
}


int sa2ul_crypto_method_aes_cbc_encrypt(sa2ul_crypto_aes_cbc_t *p_aes_cbc, UCHAR *input, UCHAR *output, UINT length, UCHAR block_size)
{
    return (sa2ul_crypto_method_aes_cbc_op(p_aes_cbc, SA2UL_CRYPTO_AES_OP_ENCRYPT, input, output, length, block_size));
}


int sa2ul_crypto_method_aes_cbc_decrypt(sa2ul_crypto_aes_cbc_t *p_aes_cbc, UCHAR *input, UCHAR *output, UINT length, UCHAR block_size)
{
    return (sa2ul_crypto_method_aes_cbc_op(p_aes_cbc, SA2UL_CRYPTO_AES_OP_DECRYPT, input, output, length, block_size));
}


int sa2ul_crypto_method_aes_cbc_encrypt_decrypt_init(sa2ul_crypto_aes_cbc_t *p_aes_cbc, UCHAR *iv, UINT iv_len)
{
    /* Determine if IV size is larger than the size of save_input. */
    if (iv_len > SA2UL_CRYPTO_AES_CBC_BLOCK_SIZE_IN_BYTES) {
        return(NX_CRYPTO_PTR_ERROR);
    }

    /* Copy IV to last cipher. */
    NX_CRYPTO_MEMCPY(&p_aes_cbc->last_block[0], iv, iv_len);

    return (NX_CRYPTO_SUCCESS);
}

