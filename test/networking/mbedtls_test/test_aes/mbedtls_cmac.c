/*
 * Copyright (C) 2023 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "utils/mbedtls_test_utils.h"

/* ========================================================================== */
/*                           TEST-DATA DECLARATION                            */
/* ========================================================================== */

/*
    data_cmac is an array of string literals of test data for mbedTLS sanity tests for AES-CMAC,
    these data vectors will be used in case of both software and hardware cryptography offloaded mbedTLS (TO DO)
*/

const char* data_cmac[] = {
    "0",
    "1",
    "depends_on:0",
    "2:exp:0:int:128:int:0",
    "depends_on:0",
    "2:exp:1:int:192:int:0",
    "depends_on:0",
    "2:exp:2:int:256:int:0",
    "depends_on:1",
    "2:exp:3:int:192:int:0",
    "depends_on:0",
    "2:exp:4:int:224:exp:5",
    "depends_on:0",
    "2:exp:4:int:0:exp:5",
    "depends_on:2",
    "2:exp:6:int:128:exp:5",
    "3:exp:0:hex:\"2b7e151628aed2a6abf7158809cf4f3c\":int:128:int:16:hex:\"\":exp:7:hex:\"\":exp:7:hex:\"\":exp:7:hex:\"\":exp:7:hex:\"bb1d6929e95937287fa37d129b756746\"",
    "3:exp:0:hex:\"2b7e151628aed2a6abf7158809cf4f3c\":int:128:int:16:hex:\"6bc1bee22e409f96e93d7e117393172a\":int:16:hex:\"\":exp:7:hex:\"\":exp:7:hex:\"\":exp:7:hex:\"070a16b46b4d4144f79bdd9dd04a287c\"",
    "3:exp:0:hex:\"2b7e151628aed2a6abf7158809cf4f3c\":int:128:int:16:hex:\"6bc1bee22e409f96e93d7e117393172aae2d8a571e03ac9c9eb76fac45af8e5130c81c46a35ce411e5fbc1191a0a52eff69f2445df4f9b17ad2b417be66c3710\":int:64:hex:\"\":exp:7:hex:\"\":exp:7:hex:\"\":exp:7:hex:\"51f0bebf7e3b9d92fc49741779363cfe\"",
    "3:exp:0:hex:\"2b7e151628aed2a6abf7158809cf4f3c\":int:128:int:16:hex:\"6bc1bee22e409f96\":int:8:hex:\"e93d7e117393172a\":int:8:hex:\"\":exp:7:hex:\"\":exp:7:hex:\"070a16b46b4d4144f79bdd9dd04a287c\"",
    "3:exp:0:hex:\"2b7e151628aed2a6abf7158809cf4f3c\":int:128:int:16:hex:\"6bc1bee22e409f96e93d7e117393172a\":int:16:hex:\"ae2d8a571e03ac9c9eb76fac45af8e51\":int:16:hex:\"30c81c46a35ce411e5fbc1191a0a52ef\":int:16:hex:\"f69f2445df4f9b17ad2b417be66c3710\":int:16:hex:\"51f0bebf7e3b9d92fc49741779363cfe\"",
    "3:exp:0:hex:\"2b7e151628aed2a6abf7158809cf4f3c\":int:128:int:16:hex:\"6bc1bee22e409f96\":int:8:hex:\"e93d7e117393172aae2d8a571e03ac9c\":int:16:hex:\"9eb76fac45af8e5130c81c46a35ce411e5fbc1191a0a52ef\":int:24:hex:\"f69f2445df4f9b17ad2b417be66c3710\":int:16:hex:\"51f0bebf7e3b9d92fc49741779363cfe\"",
    "3:exp:0:hex:\"2b7e151628aed2a6abf7158809cf4f3c\":int:128:int:16:hex:\"\":int:0:hex:\"6bc1bee22e409f96\":int:8:hex:\"\":int:0:hex:\"e93d7e117393172a\":int:8:hex:\"070a16b46b4d4144f79bdd9dd04a287c\"",
    "4:exp:1:hex:\"8e73b0f7da0e6452c810f32b809079e562f8ead2522c6b7b\":int:192:int:16:hex:\"\":exp:7:hex:\"\":exp:7:hex:\"\":exp:7:hex:\"d17ddf46adaacde531cac483de7a9367\":hex:\"\":exp:7:hex:\"\":exp:7:hex:\"\":exp:7:hex:\"d17ddf46adaacde531cac483de7a9367\"",
    "4:exp:1:hex:\"8e73b0f7da0e6452c810f32b809079e562f8ead2522c6b7b\":int:192:int:16:hex:\"\":exp:7:hex:\"\":exp:7:hex:\"\":exp:7:hex:\"d17ddf46adaacde531cac483de7a9367\":hex:\"6bc1bee22e409f96e93d7e117393172aae2d8a571e03ac9c9eb76fac45af8e5130c81c46a35ce411e5fbc1191a0a52eff69f2445df4f9b17ad2b417be66c3710\":int:64:hex:\"\":exp:7:hex:\"\":exp:7:hex:\"a1d5df0eed790f794d77589659f39a11\"",
    "4:exp:1:hex:\"8e73b0f7da0e6452c810f32b809079e562f8ead2522c6b7b\":int:192:int:16:hex:\"6bc1bee22e409f96e93d7e117393172aae2d8a571e03ac9c9eb76fac45af8e51\":int:32:hex:\"30c81c46a35ce411e5fbc1191a0a52eff69f2445df4f9b17ad2b417be66c3710\":int:32:hex:\"\":exp:7:hex:\"a1d5df0eed790f794d77589659f39a11\":hex:\"6bc1bee22e409f96e93d7e117393172aae2d8a571e03ac9c9eb76fac45af8e51\":int:32:hex:\"30c81c46a35ce411e5fbc1191a0a52eff69f2445df4f9b17ad2b417be66c3710\":int:32:hex:\"\":exp:7:hex:\"a1d5df0eed790f794d77589659f39a11\"",
};

/* ========================================================================== */
/*                                LOCAL TYPEDEFS                              */
/* ========================================================================== */

static struct
{
    int failed;
    const char *test;
    const char *filename;
    int line_no;
}
test_info;

/* ========================================================================== */
/*                                    MACROS                                  */
/* ========================================================================== */

#define TEST_ASSERT_MBEDTLS( TEST )                 \
    do {                                            \
        if( ! (TEST) )                              \
        {                                           \
            test_fail( #TEST, __LINE__, __FILE__ ); \
            goto exit;                              \
        }                                           \
    } while( 0 )

/* ========================================================================== */
/*                            FUNCTION DECLARATIONS                           */
/* ========================================================================== */

void test_mbedtls_cmac_self_test( );
void test_mbedtls_cmac_self_test_wrapper( void ** params );
void test_mbedtls_cmac_null_args(  );
void test_mbedtls_cmac_null_args_wrapper( void ** params );
void test_mbedtls_cmac_setkey( int cipher_type, int key_size, int result );
void test_mbedtls_cmac_setkey_wrapper( void ** params );
void test_mbedtls_cmac_multiple_blocks( int cipher_type, data_t * key,
                                   int keybits, int block_size,
                                   data_t * block1, int block1_len,
                                   data_t * block2, int block2_len,
                                   data_t * block3, int block3_len,
                                   data_t * block4, int block4_len,
                                   data_t * expected_result );
void test_mbedtls_cmac_multiple_blocks_wrapper( void ** params );
void test_mbedtls_cmac_multiple_operations_same_key( int cipher_type,
                                                data_t * key, int keybits,
                                                int block_size,
                                                data_t * block_a1,
                                                int block_a1_len,
                                                data_t * block_a2,
                                                int block_a2_len,
                                                data_t * block_a3,
                                                int block_a3_len,
                                                data_t * expected_result_a,
                                                data_t * block_b1,
                                                int block_b1_len,
                                                data_t * block_b2,
                                                int block_b2_len,
                                                data_t * block_b3,
                                                int block_b3_len,
                                                data_t * expected_result_b
                                                );
                                                int check_test_cmac( int func_idx );
void test_mbedtls_cmac_multiple_operations_same_key_wrapper( void ** params );

/* TEST-FUNCTIONS */
int dispatch_test_cmac( int func_idx, void ** params );
static int dep_check( int dep_id );
static int get_expression( int32_t exp_id, int32_t * out_value );
static int convert_params( size_t cnt , char ** params , int * int_params_store );

/* ========================================================================== */
/*                             FUNCTION DEFINITIONS                           */
/* ========================================================================== */

static void test_fail( const char *test, int line_no, const char* filename )
{
    test_info.failed = 1;
    test_info.test = test;
    test_info.line_no = line_no;
    test_info.filename = filename;
}

/**
 * \brief       Table of test function wrappers. Used by dispatch_test_cmac().
 *              This table is populated by script:
 *              generate_test_code.py
 *
 */
TestWrapper_t test_funcs_cmac[] =
{
    /* Function Id: 0 */

    #if defined(MBEDTLS_CMAC_C) && defined(MBEDTLS_SELF_TEST)
        test_mbedtls_cmac_self_test_wrapper,
    #else
        NULL,
    #endif
    /* Function Id: 1 */

    #if defined(MBEDTLS_CMAC_C)
        test_mbedtls_cmac_null_args_wrapper,
    #else
        NULL,
    #endif
    /* Function Id: 2 */

    #if defined(MBEDTLS_CMAC_C)
        test_mbedtls_cmac_setkey_wrapper,
    #else
        NULL,
    #endif
    /* Function Id: 3 */

    #if defined(MBEDTLS_CMAC_C)
        test_mbedtls_cmac_multiple_blocks_wrapper,
    #else
        NULL,
    #endif
    /* Function Id: 4 */

    #if defined(MBEDTLS_CMAC_C)
        test_mbedtls_cmac_multiple_operations_same_key_wrapper,
    #else
        NULL,
    #endif
};

/* ========================================================================== */
/*                           Functions Defination                             */
/* ========================================================================== */

#if defined(MBEDTLS_CMAC_C)
#include "mbedtls/cipher.h"
#include "mbedtls/cmac.h"
#if defined(MBEDTLS_SELF_TEST)

void test_mbedtls_cmac_self_test(  )
{
    TEST_ASSERT_MBEDTLS( mbedtls_cmac_self_test( 1 ) == 0 );
exit:
    ;
}

void test_mbedtls_cmac_self_test_wrapper( void ** params )
{
    (void)params;

    test_mbedtls_cmac_self_test(  );
}
#endif /* MBEDTLS_SELF_TEST */

void test_mbedtls_cmac_null_args(  )
{
    mbedtls_cipher_context_t ctx;
    const mbedtls_cipher_info_t *cipher_info;
    unsigned char test_key[MBEDTLS_CIPHER_BLKSIZE_MAX];
    unsigned char test_data[MBEDTLS_CIPHER_BLKSIZE_MAX] __attribute__ ((aligned (128U)));
    unsigned char test_output[MBEDTLS_CIPHER_BLKSIZE_MAX] __attribute__ ((aligned (128U)));

    mbedtls_cipher_init( &ctx );

    /* Test NULL cipher info */
    TEST_ASSERT_MBEDTLS( mbedtls_cipher_cmac_update( &ctx, test_data, 16 ) ==
                                         MBEDTLS_ERR_CIPHER_BAD_INPUT_DATA );

    cipher_info = mbedtls_cipher_info_from_type( MBEDTLS_CIPHER_AES_128_ECB );
    TEST_ASSERT_MBEDTLS( mbedtls_cipher_setup( &ctx, cipher_info ) == 0 );

    TEST_ASSERT_MBEDTLS( mbedtls_cipher_cmac_starts( NULL, test_key, 128 ) ==
                                         MBEDTLS_ERR_CIPHER_BAD_INPUT_DATA );

    TEST_ASSERT_MBEDTLS( mbedtls_cipher_cmac_starts( &ctx, NULL, 128 ) ==
                                         MBEDTLS_ERR_CIPHER_BAD_INPUT_DATA );

    TEST_ASSERT_MBEDTLS( mbedtls_cipher_cmac_update( NULL, test_data, 16 ) ==
                                         MBEDTLS_ERR_CIPHER_BAD_INPUT_DATA );

    TEST_ASSERT_MBEDTLS( mbedtls_cipher_cmac_update( &ctx, NULL, 16 ) ==
                                         MBEDTLS_ERR_CIPHER_BAD_INPUT_DATA );

    TEST_ASSERT_MBEDTLS( mbedtls_cipher_cmac_finish( NULL, test_output ) ==
                                         MBEDTLS_ERR_CIPHER_BAD_INPUT_DATA );

    TEST_ASSERT_MBEDTLS( mbedtls_cipher_cmac_finish( &ctx, NULL ) ==
                                         MBEDTLS_ERR_CIPHER_BAD_INPUT_DATA );

    TEST_ASSERT_MBEDTLS( mbedtls_cipher_cmac_reset( NULL ) ==
                                         MBEDTLS_ERR_CIPHER_BAD_INPUT_DATA );

    TEST_ASSERT_MBEDTLS( mbedtls_cipher_cmac( NULL,
                                      test_key, 128,
                                      test_data, 16,
                                      test_output ) ==
                                            MBEDTLS_ERR_CIPHER_BAD_INPUT_DATA );

    TEST_ASSERT_MBEDTLS( mbedtls_cipher_cmac( cipher_info,
                                      NULL, 128,
                                      test_data, 16,
                                      test_output ) ==
                                            MBEDTLS_ERR_CIPHER_BAD_INPUT_DATA );

    TEST_ASSERT_MBEDTLS( mbedtls_cipher_cmac( cipher_info,
                                      test_key, 128,
                                      NULL, 16,
                                      test_output ) ==
                                            MBEDTLS_ERR_CIPHER_BAD_INPUT_DATA );

    TEST_ASSERT_MBEDTLS( mbedtls_cipher_cmac( cipher_info,
                                      test_key, 128,
                                      test_data, 16,
                                      NULL ) ==
                                            MBEDTLS_ERR_CIPHER_BAD_INPUT_DATA );

    TEST_ASSERT_MBEDTLS( mbedtls_aes_cmac_prf_128( NULL, 16,
                                           test_data, 16,
                                           test_output ) ==
                                           MBEDTLS_ERR_CIPHER_BAD_INPUT_DATA );

    TEST_ASSERT_MBEDTLS( mbedtls_aes_cmac_prf_128( test_key, 16,
                                           NULL, 16,
                                           test_output ) ==
                                              MBEDTLS_ERR_CIPHER_BAD_INPUT_DATA );

    TEST_ASSERT_MBEDTLS( mbedtls_aes_cmac_prf_128( test_key, 16,
                                           test_data, 16,
                                           NULL ) ==
                                              MBEDTLS_ERR_CIPHER_BAD_INPUT_DATA );

exit:
    mbedtls_cipher_free( &ctx );
}

void test_mbedtls_cmac_null_args_wrapper( void ** params )
{
    (void)params;

    test_mbedtls_cmac_null_args(  );
}

void test_mbedtls_cmac_setkey( int cipher_type, int key_size, int result )
{
    const mbedtls_cipher_info_t *cipher_info;
    unsigned char key[32];
    unsigned char buf[16] __attribute__ ((aligned (128U)));
    unsigned char tmp[16] __attribute__ ((aligned (128U)));

    memset( key, 0x2A, sizeof( key ) );
    TEST_ASSERT_MBEDTLS( (unsigned) key_size <= 8 * sizeof( key ) );

    TEST_ASSERT_MBEDTLS( ( cipher_info = mbedtls_cipher_info_from_type( cipher_type ) )
                    != NULL );

    memset( buf, 0x2A, sizeof( buf ) );

    TEST_ASSERT_MBEDTLS( ( result == mbedtls_cipher_cmac( cipher_info, key, key_size,
                                                buf, 16, tmp ) ) != 0 );
exit:
    ;
}

void test_mbedtls_cmac_setkey_wrapper( void ** params )
{
    test_mbedtls_cmac_setkey( *( (int *) params[0] ), *( (int *) params[1] ), *( (int *) params[2] ) );
}

void test_mbedtls_cmac_multiple_blocks( int cipher_type, data_t * key,
                                   int keybits, int block_size,
                                   data_t * block1, int block1_len,
                                   data_t * block2, int block2_len,
                                   data_t * block3, int block3_len,
                                   data_t * block4, int block4_len,
                                   data_t * expected_result )
{
    const mbedtls_cipher_info_t *cipher_info;
    mbedtls_cipher_context_t ctx;
    unsigned char output[MBEDTLS_CIPHER_BLKSIZE_MAX];
    uint8_t inputBuf1[block1->len] __attribute__ ((aligned (128U)));
    uint8_t inputBuf2[block2->len] __attribute__ ((aligned (128U)));
    uint8_t inputBuf3[block3->len] __attribute__ ((aligned (128U)));
    uint8_t inputBuf4[block4->len] __attribute__ ((aligned (128U)));

    memcpy(&inputBuf1[0], block1->x, block1->len);

    memcpy(&inputBuf2[0], block2->x, block2->len);

    memcpy(&inputBuf3[0], block3->x, block3->len);

    memcpy(&inputBuf4[0], block4->x, block4->len);

    /* Convert the test parameters to binary data */

    mbedtls_cipher_init( &ctx );

    /* Validate the test inputs */
    TEST_ASSERT_MBEDTLS( block1_len <= 100 );
    TEST_ASSERT_MBEDTLS( block2_len <= 100 );
    TEST_ASSERT_MBEDTLS( block3_len <= 100 );
    TEST_ASSERT_MBEDTLS( block4_len <= 100 );

    /* Set up */
    TEST_ASSERT_MBEDTLS( ( cipher_info = mbedtls_cipher_info_from_type( cipher_type ) )
                    != NULL );

    TEST_ASSERT_MBEDTLS( mbedtls_cipher_setup( &ctx, cipher_info ) == 0 );

    TEST_ASSERT_MBEDTLS( mbedtls_cipher_cmac_starts( &ctx,
                                             (const unsigned char*)key->x,
                                             keybits ) == 0 );

    /* Multiple partial and complete blocks. A negative length means skip the
     * update operation */
    if( block1_len >= 0)
        TEST_ASSERT_MBEDTLS( mbedtls_cipher_cmac_update( &ctx,
                                                 inputBuf1,
                                                 block1_len ) == 0);

    if( block2_len >= 0 )
        TEST_ASSERT_MBEDTLS( mbedtls_cipher_cmac_update( &ctx,
                                                 inputBuf2,
                                                 block2_len ) == 0);

    if( block3_len >= 0 )
        TEST_ASSERT_MBEDTLS( mbedtls_cipher_cmac_update( &ctx,
                                                 inputBuf3,
                                                 block3_len ) == 0);

    if( block4_len >= 0 )
        TEST_ASSERT_MBEDTLS( mbedtls_cipher_cmac_update( &ctx,
                                                 inputBuf4,
                                                 block4_len ) == 0);

    TEST_ASSERT_MBEDTLS( mbedtls_cipher_cmac_finish( &ctx, output ) == 0 );

    TEST_ASSERT_MBEDTLS( memcmp( output, expected_result->x, block_size )  == 0 );

exit:
    mbedtls_cipher_free( &ctx );
}

void test_mbedtls_cmac_multiple_blocks_wrapper( void ** params )
{
    data_t data1 = {(uint8_t *) params[1], *( (uint32_t *) params[2] )};
    data_t data5 = {(uint8_t *) params[5], *( (uint32_t *) params[6] )};
    data_t data8 = {(uint8_t *) params[8], *( (uint32_t *) params[9] )};
    data_t data11 = {(uint8_t *) params[11], *( (uint32_t *) params[12] )};
    data_t data14 = {(uint8_t *) params[14], *( (uint32_t *) params[15] )};
    data_t data17 = {(uint8_t *) params[17], *( (uint32_t *) params[18] )};

    test_mbedtls_cmac_multiple_blocks( *( (int *) params[0] ), &data1, *( (int *) params[3] ), *( (int *) params[4] ), &data5, *( (int *) params[7] ), &data8, *( (int *) params[10] ), &data11, *( (int *) params[13] ), &data14, *( (int *) params[16] ), &data17 );
}

void test_mbedtls_cmac_multiple_operations_same_key( int cipher_type,
                                                data_t * key, int keybits,
                                                int block_size,
                                                data_t * block_a1,
                                                int block_a1_len,
                                                data_t * block_a2,
                                                int block_a2_len,
                                                data_t * block_a3,
                                                int block_a3_len,
                                                data_t * expected_result_a,
                                                data_t * block_b1,
                                                int block_b1_len,
                                                data_t * block_b2,
                                                int block_b2_len,
                                                data_t * block_b3,
                                                int block_b3_len,
                                                data_t * expected_result_b
                                                )
{
    const mbedtls_cipher_info_t *cipher_info;
    mbedtls_cipher_context_t ctx;
    unsigned char output[MBEDTLS_CIPHER_BLKSIZE_MAX] __attribute__ ((aligned (128U)));
    uint8_t inputBufA1[block_a1->len] __attribute__ ((aligned (128U)));
    memcpy(inputBufA1, block_a1->x, block_a1->len);

    uint8_t inputBufA2[block_a2->len] __attribute__ ((aligned (128U)));
    memcpy(inputBufA2, block_a2->x, block_a2->len);

    uint8_t inputBufA3[block_a3->len] __attribute__ ((aligned (128U)));
    memcpy(inputBufA3, block_a3->x, block_a3->len);

    uint8_t inputBufB1[block_b1->len] __attribute__ ((aligned (128U)));
    memcpy(inputBufB1, block_b1->x, block_b1->len);

    uint8_t inputBufB2[block_b2->len] __attribute__ ((aligned (128U)));
    memcpy(inputBufB2, block_b2->x, block_b2->len);

    uint8_t inputBufB3[block_b3->len] __attribute__ ((aligned (128U)));
    memcpy(inputBufB3, block_b3->x, block_b3->len);

    uint8_t expectedA[expected_result_a->len] __attribute__ ((aligned (128U)));
    memcpy(expectedA,expected_result_a->x, expected_result_a->len);

    uint8_t expectedB[expected_result_b->len] __attribute__ ((aligned (128U)));
    memcpy(expectedB,expected_result_b->x, expected_result_b->len);

    /* Convert the test parameters to binary data */

    mbedtls_cipher_init( &ctx );

    /* Validate the test inputs */
    TEST_ASSERT_MBEDTLS( block_a1_len <= 100 );
    TEST_ASSERT_MBEDTLS( block_a2_len <= 100 );
    TEST_ASSERT_MBEDTLS( block_a3_len <= 100 );

    TEST_ASSERT_MBEDTLS( block_b1_len <= 100 );
    TEST_ASSERT_MBEDTLS( block_b2_len <= 100 );
    TEST_ASSERT_MBEDTLS( block_b3_len <= 100 );

    /* Set up */
    TEST_ASSERT_MBEDTLS( ( cipher_info = mbedtls_cipher_info_from_type( cipher_type ) )
                    != NULL );

    TEST_ASSERT_MBEDTLS( mbedtls_cipher_setup( &ctx, cipher_info ) == 0 );

    TEST_ASSERT_MBEDTLS( mbedtls_cipher_cmac_starts( &ctx,
                                             (const unsigned char*)key->x,
                                             keybits ) == 0 );

    /* Sequence A */

    /* Multiple partial and complete blocks. A negative length means skip the
     * update operation */
    if( block_a1_len >= 0 )
        TEST_ASSERT_MBEDTLS( mbedtls_cipher_cmac_update( &ctx,
                                                 inputBufA1,
                                                 block_a1_len ) == 0);

    if( block_a2_len >= 0 )
        TEST_ASSERT_MBEDTLS( mbedtls_cipher_cmac_update( &ctx,
                                                 inputBufA2,
                                                 block_a2_len ) == 0);

    if( block_a3_len >= 0 )
        TEST_ASSERT_MBEDTLS( mbedtls_cipher_cmac_update( &ctx,
                                                inputBufA3,
                                                  block_a3_len ) == 0);

    TEST_ASSERT_MBEDTLS( mbedtls_cipher_cmac_finish( &ctx, output ) == 0 );

    TEST_ASSERT_MBEDTLS( memcmp( output, expectedA, block_size )  == 0 );

    TEST_ASSERT_MBEDTLS( mbedtls_cipher_cmac_reset( &ctx ) == 0 );

    /* Sequence B */

    /* Multiple partial and complete blocks. A negative length means skip the
     * update operation */
    if( block_b1_len >= 0)
        TEST_ASSERT_MBEDTLS( mbedtls_cipher_cmac_update( &ctx,
                                                 inputBufB1,
                                                 block_b1_len ) == 0);

    if( block_b2_len >= 0 )
        TEST_ASSERT_MBEDTLS( mbedtls_cipher_cmac_update( &ctx,
                                                 inputBufB2,
                                                 block_b2_len ) == 0);

    if( block_b3_len >= 0 )
        TEST_ASSERT_MBEDTLS( mbedtls_cipher_cmac_update( &ctx,
                                                 inputBufB3,
                                                 block_b3_len ) == 0);

    TEST_ASSERT_MBEDTLS( mbedtls_cipher_cmac_finish( &ctx, output ) == 0 );

    TEST_ASSERT_MBEDTLS( memcmp( output, expectedB, block_size )  == 0 );

exit:
    mbedtls_cipher_free( &ctx );
}

void test_mbedtls_cmac_multiple_operations_same_key_wrapper( void ** params )
{
    data_t data1 = {(uint8_t *) params[1], *( (uint32_t *) params[2] )};
    data_t data5 = {(uint8_t *) params[5], *( (uint32_t *) params[6] )};
    data_t data8 = {(uint8_t *) params[8], *( (uint32_t *) params[9] )};
    data_t data11 = {(uint8_t *) params[11], *( (uint32_t *) params[12] )};
    data_t data14 = {(uint8_t *) params[14], *( (uint32_t *) params[15] )};
    data_t data16 = {(uint8_t *) params[16], *( (uint32_t *) params[17] )};
    data_t data19 = {(uint8_t *) params[19], *( (uint32_t *) params[20] )};
    data_t data22 = {(uint8_t *) params[22], *( (uint32_t *) params[23] )};
    data_t data25 = {(uint8_t *) params[25], *( (uint32_t *) params[26] )};

    test_mbedtls_cmac_multiple_operations_same_key( *( (int *) params[0] ), &data1, *( (int *) params[3] ), *( (int *) params[4] ), &data5, *( (int *) params[7] ), &data8, *( (int *) params[10] ), &data11, *( (int *) params[13] ), &data14, &data16, *( (int *) params[18] ), &data19, *( (int *) params[21] ), &data22, *( (int *) params[24] ), &data25 );
}
#endif /* MBEDTLS_CMAC_C */

/**
 * \brief       Checks if the dependency i.e. the compile flag is set.
 *              For optimizing space for embedded targets each dependency
 *              is identified by a unique identifier instead of string literals.
 *              Identifiers and check code is generated by script:
 *              generate_test_code.py
 *
 * \param exp_id    Dependency identifier.
 *
 * \return       DEPENDENCY_SUPPORTED if set else DEPENDENCY_NOT_SUPPORTED
 */
static int dep_check( int dep_id )
{
    int ret = DEPENDENCY_NOT_SUPPORTED;

    (void) dep_id;

    switch( dep_id )
    {

    #if defined(MBEDTLS_CMAC_C)

        case 0:
            {
            #if defined(MBEDTLS_AES_C)
                ret = DEPENDENCY_SUPPORTED;
            #else
                ret = DEPENDENCY_NOT_SUPPORTED;
            #endif
            }
            break;
        case 1:
            {
            #if defined(MBEDTLS_DES_C)
                ret = DEPENDENCY_SUPPORTED;
            #else
                ret = DEPENDENCY_NOT_SUPPORTED;
            #endif
            }
            break;
        case 2:
            {
            #if defined(MBEDTLS_CAMELLIA_C)
                ret = DEPENDENCY_SUPPORTED;
            #else
                ret = DEPENDENCY_NOT_SUPPORTED;
            #endif
            }
            break;
#endif
        default:
            break;
    }
    return ret;
}

/**
 * \brief       Checks if test function is supported
 *
 * \param exp_id    Test function index.
 *
 * \return       DISPATCH_TEST_SUCCESS if found
 *               DISPATCH_TEST_FN_NOT_FOUND if not found
 *               DISPATCH_UNSUPPORTED_SUITE if not compile time enabled.
 */
int check_test_cmac( int func_idx )
{
    int ret = DISPATCH_TEST_SUCCESS;
    TestWrapper_t fp = NULL;

    if ( func_idx < (int)( sizeof(test_funcs_cmac)/sizeof( TestWrapper_t ) ) )
    {
        fp = test_funcs_cmac[func_idx];
        if ( fp == NULL )
            ret = DISPATCH_UNSUPPORTED_SUITE;
    }
    else
    {
        ret = DISPATCH_TEST_FN_NOT_FOUND;
    }

    return ret;
}

/**
 * \brief       Dispatches test functions based on function index.
 *
 * \param exp_id    Test function index.
 *
 * \return       DISPATCH_TEST_SUCCESS if found
 *               DISPATCH_TEST_FN_NOT_FOUND if not found
 *               DISPATCH_UNSUPPORTED_SUITE if not compile time enabled.
 */
int dispatch_test_cmac( int func_idx, void ** params )
{
    int ret = DISPATCH_TEST_SUCCESS;
    TestWrapper_t fp = NULL;

    if ( func_idx < (int)( sizeof( test_funcs_cmac ) / sizeof( TestWrapper_t ) ) )
    {
        fp = test_funcs_cmac[func_idx];
        if ( fp )
            fp( params );
        else
            ret = DISPATCH_UNSUPPORTED_SUITE;
    }
    else
    {
        ret = DISPATCH_TEST_FN_NOT_FOUND;
    }

    return ret;
}

static /**
 * \brief       Evaluates an expression/macro into its literal integer value.
 *              For optimizing space for embedded targets each expression/macro
 *              is identified by a unique identifier instead of string literals.
 *              Identifiers and evaluation code is generated by script:
 *              generate_test_code.py
 *
 * \param exp_id    Expression identifier.
 * \param out_value Pointer to int to hold the integer.
 *
 * \return       0 if exp_id is found. 1 otherwise.
 */
int get_expression( int32_t exp_id, int32_t * out_value )
{
    int ret = KEY_VALUE_MAPPING_FOUND;

    (void) exp_id;
    (void) out_value;

    switch( exp_id )
    {

#if defined(MBEDTLS_CMAC_C)

        case 0:
            {
                *out_value = MBEDTLS_CIPHER_AES_128_ECB;
            }
            break;
        case 1:
            {
                *out_value = MBEDTLS_CIPHER_AES_192_ECB;
            }
            break;
        case 2:
            {
                *out_value = MBEDTLS_CIPHER_AES_256_ECB;
            }
            break;
        case 3:
            {
                *out_value = MBEDTLS_CIPHER_DES_EDE3_ECB;
            }
            break;
        case 4:
            {
                *out_value = MBEDTLS_CIPHER_ID_AES;
            }
            break;
        case 5:
            {
                *out_value = MBEDTLS_ERR_CIPHER_BAD_INPUT_DATA;
            }
            break;
        case 6:
            {
                *out_value = MBEDTLS_CIPHER_ID_CAMELLIA;
            }
            break;
        case 7:
            {
                *out_value = -1;
            }
            break;
#endif

        default:
           {
                ret = KEY_VALUE_MAPPING_NOT_FOUND;
           }
           break;
    }
    return ret;
}

/**
 * \brief       Converts parameters into test function consumable parameters.
 *              Example: Input:  {"int", "0", "char*", "Hello",
 *                                "hex", "abef", "exp", "1"}
 *                      Output:  {
 *                                0,                // Verified int
 *                                "Hello",          // Verified string
 *                                2, { 0xab, 0xef },// Converted len,hex pair
 *                                9600              // Evaluated expression
 *                               }
 *
 *
 * \param cnt               Parameter array count.
 * \param params            Out array of found parameters.
 * \param int_params_store  Memory for storing processed integer parameters.
 *
 * \return      0 for success else 1
 */
static int convert_params( size_t cnt , char ** params , int * int_params_store )
{
    char ** cur = params;
    char ** out = params;
    int ret = DISPATCH_TEST_SUCCESS;

    while ( cur < params + cnt )
    {
        char * type = *cur++;
        char * val = *cur++;

        if ( strcmp( type, "char*" ) == 0 )
        {
            if ( verify_string( &val ) == 0 )
            {
              *out++ = val;
            }
            else
            {
                ret = ( DISPATCH_INVALID_TEST_DATA );
                break;
            }
        }
        else if ( strcmp( type, "int" ) == 0 )
        {
            if ( verify_int( val, int_params_store ) == 0 )
            {
              *out++ = (char *) int_params_store++;
            }
            else
            {
                ret = ( DISPATCH_INVALID_TEST_DATA );
                break;
            }
        }
        else if ( strcmp( type, "hex" ) == 0 )
        {
            if ( verify_string( &val ) == 0 )
            {
                *int_params_store = unhexify( (unsigned char *) val, val );
                *out++ = val;
                *out++ = (char *)(int_params_store++);
            }
            else
            {
                ret = ( DISPATCH_INVALID_TEST_DATA );
                break;
            }
        }
        else if ( strcmp( type, "exp" ) == 0 )
        {
            int exp_id = strtol( val, NULL, 10 );
            if ( get_expression ( exp_id, int_params_store ) == 0 )
            {
              *out++ = (char *)int_params_store++;
            }
            else
            {
              ret = ( DISPATCH_INVALID_TEST_DATA );
              break;
            }
        }
        else
        {
          ret = ( DISPATCH_INVALID_TEST_DATA );
          break;
        }
    }
    return ret;
}

int* mbedtls_test_cmac()
{
    TEST_MESSAGE("----------------------------TEST FOR CMAC----------------------------");
    uint8_t i=0 ,j=0;
    int ret = 0;
    uint8_t function_id = 0;
    char *params[50];
    int int_params[50];
    char unity_buffer[100];
    char* buffer;
    /* Other Local variables */
    int cnt;
    int total_errors = 0, total_tests = 0, total_skipped = 0;
    while ( i < sizeof(data_cmac)/sizeof(data_cmac[0]) )
    {
        buffer = malloc(sizeof(char)*strlen(data_cmac[i]));
        strcpy(buffer, data_cmac[i]);
        total_tests++;

        int unmet_dep_count = 0;
        char *unmet_dependencies[20];
        ret = 0;
        test_info.failed = 0;
        if ( (uint8_t *)buffer == NULL )
            continue;

        cnt = parse_arguments( buffer, strlen( buffer ), params,
                                    sizeof( params ) / sizeof( params[0] ) );

        if( strcmp( params[0], "depends_on" ) == 0 )
        {
            for( j = 1; j < cnt; j++ )
            {
                int dep_id = strtol( params[j], NULL, 10 );
                if( dep_check( dep_id ) != DEPENDENCY_SUPPORTED )
                {
                    unmet_dependencies[ unmet_dep_count ] = strdup( params[j] );
                    if(  unmet_dependencies[ unmet_dep_count ] == NULL )
                    {
                        TEST_MESSAGE("FATAL: Out of memory" );
                        mbedtls_exit( MBEDTLS_EXIT_FAILURE );
                    }
                    unmet_dep_count++;
                }
            }
            ++i;
            free(buffer);
            buffer = malloc(sizeof(char)*strlen(data_cmac[i]));
            strcpy(buffer, data_cmac[i]);
            cnt = parse_arguments( buffer, strlen( buffer ), params,
                                    sizeof( params ) / sizeof( params[0] ) );
        }

        // If there are no unmet dependencies execute the test
        if( unmet_dep_count == 0 )
        {
            test_info.failed = 0;
            function_id = strtol( params[0], NULL, 10 );
            if ( (ret = check_test_cmac( function_id )) == DISPATCH_TEST_SUCCESS )
            {
                ret = convert_params( cnt - 1, params + 1, int_params );
                if ( DISPATCH_TEST_SUCCESS == ret )
                {
                    ret = dispatch_test_cmac( function_id, (void **)( params + 1 ) );
                }
            }
        }
        if( unmet_dep_count > 0 || ret == DISPATCH_UNSUPPORTED_SUITE )
        {
            total_skipped++;
            sprintf(unity_buffer, "AES CMAC-Test %d : Skipped", total_tests);
            TEST_MESSAGE(unity_buffer);

            if(ret == DISPATCH_UNSUPPORTED_SUITE )
            {
                TEST_MESSAGE("Test Suite not enabled" );
            }

            if(unmet_dep_count > 0 )
            {
                TEST_MESSAGE("Unmet dependencies: " );
            }
            unmet_dep_count = 0;
        }

        else if( ret == DISPATCH_TEST_SUCCESS )
        {
            if( test_info.failed == 0 )
            {
                sprintf(unity_buffer, "AES CMAC-Test %d : PASSED", total_tests);
                TEST_MESSAGE(unity_buffer);
            }
            else
            {
                total_errors++;
                sprintf(unity_buffer, "Test %d : FAILED", total_tests);
                TEST_MESSAGE(unity_buffer);
                sprintf(unity_buffer, "  %s  at line %d, %s",
                                    test_info.test, test_info.line_no,
                                    test_info.filename );
                TEST_MESSAGE(unity_buffer);
            }
        }
        else if( ret == DISPATCH_INVALID_TEST_DATA )
        {
            TEST_MESSAGE("FAILED: FATAL PARSE ERROR" );
            mbedtls_exit( 2 );
        }
        else if( ret == DISPATCH_TEST_FN_NOT_FOUND )
        {
            TEST_MESSAGE("FAILED: FATAL TEST FUNCTION NOT FOUND" );
            mbedtls_exit( 2 );
        }
        else
        {
            total_errors++;
        }
        for( int k = 0; k < unmet_dep_count; k++ )
            free( unmet_dependencies[k] );
        ++i;
        free(buffer);
    }

    if(total_errors == 0 && total_skipped==0)
        TEST_MESSAGE("ALL CMAC TESTS PASSED" );
    else if(total_errors>0)
        TEST_MESSAGE("CMAC TESTS FAILED" );
    else
        TEST_MESSAGE("Some tests skipped" );

    int* test_results = malloc(3*sizeof(int));
    test_results[0] = total_tests;
    test_results[1] = total_errors;
    test_results[2] = total_skipped;
    return test_results;
}
