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
    data_entropy is an array of string literals of test data for mbedTLS sanity tests for Entropy
    these data vectors will be used in case of both software and hardware cryptography offloaded mbedTLS (TO DO)
*/

const char* data_entropy[] = {
    "5",
    "1",
    "2:int:0:int:0",
    "2:int:1:int:0",
    "2:int:2:int:0",
    "2:int:31:int:0",
    "2:int:65:exp:1",
    "4:int:16:int:2:int:8",
    "4:int:32:int:1:int:32",
    "4:int:16:int:0:exp:1",
    "4:int:1024:int:1:exp:1",
    "6",
    "7:hex:\"00112233445566778899AABBCCDDEEFF00112233445566778899AABBCCDDEEFF00112233445566778899AABBCCDDEEFF00112233445566778899AABBCCDDEEFF\"",
    "7:hex:\"00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000\"",
    "7:hex:\"ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff\"",
    "depends_on:0",
    "8:int:0",
    "depends_on:1",
    "8:int:1",
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

static int dep_check( int dep_id );
static int get_expression( int32_t exp_id, int32_t * out_value );
static int check_test_entropy( int func_idx );
static int dispatch_test_entropy( int func_idx, void ** params );
static int buffer_nv_seed_read( unsigned char *buf, size_t buf_len );
static int buffer_nv_seed_write( unsigned char *buf, size_t buf_len );
static int write_nv_seed( unsigned char *buf, size_t buf_len );
static int read_nv_seed( unsigned char *buf, size_t buf_len );
void test_entropy_seed_file( char * path, int ret );
void test_entropy_seed_file_wrapper( void ** params );
void test_entropy_too_many_sources(  );
void test_entropy_too_many_sources_wrapper( void ** params );
void test_entropy_func_len( int len, int ret );
void test_entropy_func_len_wrapper( void ** params );
void test_entropy_source_fail( char * path );
void test_entropy_threshold( int threshold, int chunk_size, int result );
void test_entropy_threshold_wrapper( void ** params );
void test_nv_seed_file_create(  );
void test_nv_seed_file_create_wrapper( void ** params );
void test_entropy_nv_seed_std_io(  );
void test_entropy_source_fail_wrapper( void ** params );
static int entropy_dummy_source( void *data, unsigned char *output,
                                 size_t len, size_t *olen );
static int convert_params( size_t cnt , char ** params , int * int_params_store );

TestWrapper_t test_funcs[] =
{
    /* Function Id: 0 */

    #if defined(MBEDTLS_ENTROPY_C) && defined(MBEDTLS_ENTROPY_NV_SEED) && defined(MBEDTLS_FS_IO)
        test_entropy_seed_file_wrapper,
    #else
        NULL,
    #endif
    /* Function Id: 1 */

    #if defined(MBEDTLS_ENTROPY_C)
        test_entropy_too_many_sources_wrapper,
    #else
        NULL,
    #endif
    /* Function Id: 2 */

    #if defined(MBEDTLS_ENTROPY_C) && defined(ENTROPY_HAVE_STRONG)
        test_entropy_func_len_wrapper,
    #else
        NULL,
    #endif
    /* Function Id: 3 */

    #if defined(MBEDTLS_ENTROPY_C)
        test_entropy_source_fail_wrapper,
    #else
        NULL,
    #endif
    /* Function Id: 4 */

    #if defined(MBEDTLS_ENTROPY_C) && defined(ENTROPY_HAVE_STRONG)
        test_entropy_threshold_wrapper,
    #else
        NULL,
    #endif
    /* Function Id: 5 */

    #if defined(MBEDTLS_ENTROPY_C) && defined(MBEDTLS_ENTROPY_NV_SEED) && defined(MBEDTLS_FS_IO)
        test_nv_seed_file_create_wrapper,
    #else
        NULL,
    #endif
    /* Function Id: 6 */

    #if defined(MBEDTLS_ENTROPY_C) && defined(MBEDTLS_ENTROPY_NV_SEED) && defined(MBEDTLS_FS_IO) && defined(MBEDTLS_PLATFORM_NV_SEED_ALT)
        test_entropy_nv_seed_std_io_wrapper,
    #else
        NULL,
    #endif
    /* Function Id: 7 */

    #if defined(MBEDTLS_ENTROPY_C) && defined(MBEDTLS_ENTROPY_NV_SEED) && defined(MBEDTLS_PLATFORM_NV_SEED_ALT) && defined(MBEDTLS_ENTROPY_SHA512_ACCUMULATOR)
        test_entropy_nv_seed_wrapper,
    #else
        NULL,
    #endif
    /* Function Id: 8 */

    #if defined(MBEDTLS_ENTROPY_C) && defined(ENTROPY_HAVE_STRONG) && defined(MBEDTLS_SELF_TEST)
        test_entropy_selftest_wrapper,
    #else
        NULL,
    #endif

};

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

    #if defined(MBEDTLS_ENTROPY_C)
        case 0:
        {
        #if !defined(MBEDTLS_TEST_NULL_ENTROPY)
            ret = DEPENDENCY_SUPPORTED;
        #else
            ret = DEPENDENCY_NOT_SUPPORTED;
        #endif
        }
        break;
        case 1:
        {
        #if defined(MBEDTLS_TEST_NULL_ENTROPY)
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

static size_t entropy_dummy_calls;

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

/**
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
static int get_expression( int32_t exp_id, int32_t * out_value )
{
    int ret = KEY_VALUE_MAPPING_FOUND;

    (void) exp_id;
    (void) out_value;

    switch( exp_id )
    {

    #if defined(MBEDTLS_ENTROPY_C)

        case 0:
        {
            *out_value = MBEDTLS_ERR_ENTROPY_FILE_IO_ERROR;
        }
        break;
        case 1:
        {
            *out_value = MBEDTLS_ERR_ENTROPY_SOURCE_FAILED;
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
 * \brief       Checks if test function is supported
 *
 * \param exp_id    Test function index.
 *
 * \return       DISPATCH_TEST_SUCCESS if found
 *               DISPATCH_TEST_FN_NOT_FOUND if not found
 *               DISPATCH_UNSUPPORTED_SUITE if not compile time enabled.
 */
static int check_test_entropy( int func_idx )
{
    int ret = DISPATCH_TEST_SUCCESS;
    TestWrapper_t fp = NULL;

    if ( func_idx < (int)( sizeof(test_funcs)/sizeof( TestWrapper_t ) ) )
    {
        fp = test_funcs[func_idx];
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
static int dispatch_test_entropy( int func_idx, void ** params )
{
    int ret = DISPATCH_TEST_SUCCESS;
    TestWrapper_t fp = NULL;

    if ( func_idx < (int)( sizeof( test_funcs ) / sizeof( TestWrapper_t ) ) )
    {
        fp = test_funcs[func_idx];
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

#if defined(MBEDTLS_ENTROPY_NV_SEED)
/*
 * NV seed read/write functions that use a buffer instead of a file
 */
static unsigned char buffer_seed[MBEDTLS_ENTROPY_BLOCK_SIZE];

static int buffer_nv_seed_read( unsigned char *buf, size_t buf_len )
{
    if( buf_len != MBEDTLS_ENTROPY_BLOCK_SIZE )
        return -1;

    memcpy( buf, buffer_seed, MBEDTLS_ENTROPY_BLOCK_SIZE );
    return 0;
}

static int buffer_nv_seed_write( unsigned char *buf, size_t buf_len )
{
    if( buf_len != MBEDTLS_ENTROPY_BLOCK_SIZE )
        return -1;

    memcpy( buffer_seed, buf, MBEDTLS_ENTROPY_BLOCK_SIZE );
    return 0;
}

/*
 * NV seed read/write helpers that fill the base seedfile
 */
static int write_nv_seed( unsigned char *buf, size_t buf_len )
{
    FILE *f;

    if( buf_len != MBEDTLS_ENTROPY_BLOCK_SIZE )
        return -1;

    if( ( f = fopen( MBEDTLS_PLATFORM_STD_NV_SEED_FILE, "w" ) ) == NULL )
        return -1;

    if( fwrite( buf, 1, MBEDTLS_ENTROPY_BLOCK_SIZE, f ) !=
                    MBEDTLS_ENTROPY_BLOCK_SIZE )
        return -1;

    fclose( f );

    return 0;
}

static int read_nv_seed( unsigned char *buf, size_t buf_len )
{
    FILE *f;

    if( buf_len != MBEDTLS_ENTROPY_BLOCK_SIZE )
        return -1;

    if( ( f = fopen( MBEDTLS_PLATFORM_STD_NV_SEED_FILE, "rb" ) ) == NULL )
        return -1;

    if( fread( buf, 1, MBEDTLS_ENTROPY_BLOCK_SIZE, f ) !=
                    MBEDTLS_ENTROPY_BLOCK_SIZE )
        return -1;

    fclose( f );

    return 0;
}
#endif /* MBEDTLS_ENTROPY_NV_SEED */
#if defined(MBEDTLS_ENTROPY_NV_SEED)
#if defined(MBEDTLS_FS_IO)

void test_entropy_seed_file( char * path, int ret )
{
    mbedtls_entropy_context ctx;

    mbedtls_entropy_init( &ctx );

    TEST_ASSERT_MBEDTLS( mbedtls_entropy_write_seed_file( &ctx, path ) == ret );
    TEST_ASSERT_MBEDTLS( mbedtls_entropy_update_seed_file( &ctx, path ) == ret );

exit:
    mbedtls_entropy_free( &ctx );
}

void test_entropy_seed_file_wrapper( void ** params )
{

    test_entropy_seed_file( (char *) params[0], *( (int *) params[1] ) );
}
#endif /* MBEDTLS_FS_IO */
#endif /* MBEDTLS_ENTROPY_NV_SEED */

void test_entropy_too_many_sources(  )
{
    mbedtls_entropy_context ctx;
    size_t i;

    mbedtls_entropy_init( &ctx );

    /*
     * It's hard to tell precisely when the error will occur,
     * since we don't know how many sources were automatically added.
     */
    for( i = 0; i < MBEDTLS_ENTROPY_MAX_SOURCES; i++ )
        (void) mbedtls_entropy_add_source( &ctx, entropy_dummy_source, NULL,
                                           16, MBEDTLS_ENTROPY_SOURCE_WEAK );

    TEST_ASSERT_MBEDTLS( mbedtls_entropy_add_source( &ctx, entropy_dummy_source, NULL,
                                             16, MBEDTLS_ENTROPY_SOURCE_WEAK )
                 == MBEDTLS_ERR_ENTROPY_MAX_SOURCES );

exit:
    mbedtls_entropy_free( &ctx );
}

void test_entropy_too_many_sources_wrapper( void ** params )
{
    (void)params;

    test_entropy_too_many_sources(  );
}
#if defined(ENTROPY_HAVE_STRONG)

void test_entropy_func_len( int len, int ret )
{
    mbedtls_entropy_context ctx;
    unsigned char buf[MBEDTLS_ENTROPY_BLOCK_SIZE + 10] = { 0 };
    unsigned char acc[MBEDTLS_ENTROPY_BLOCK_SIZE + 10] = { 0 };
    size_t i, j;

    mbedtls_entropy_init( &ctx );

    /*
     * See comments in mbedtls_entropy_self_test()
     */
    for( i = 0; i < 8; i++ )
    {
        TEST_ASSERT_MBEDTLS( mbedtls_entropy_func( &ctx, buf, len ) == ret );
        for( j = 0; j < sizeof( buf ); j++ )
            acc[j] |= buf[j];
    }

    if( ret == 0 )
        for( j = 0; j < (size_t) len; j++ )
            TEST_ASSERT_MBEDTLS( acc[j] != 0 );

    for( j = len; j < sizeof( buf ); j++ )
        TEST_ASSERT_MBEDTLS( acc[j] == 0 );
exit:
    ;
}

void test_entropy_func_len_wrapper( void ** params )
{

    test_entropy_func_len( *( (int *) params[0] ), *( (int *) params[1] ) );
}
#endif /* ENTROPY_HAVE_STRONG */

void test_entropy_source_fail( char * path )
{
    mbedtls_entropy_context ctx;
    int fail = -1;
    unsigned char buf[16];

    mbedtls_entropy_init( &ctx );

    TEST_ASSERT_MBEDTLS( mbedtls_entropy_add_source( &ctx, entropy_dummy_source,
                                             &fail, 16,
                                             MBEDTLS_ENTROPY_SOURCE_WEAK )
                 == 0 );

    TEST_ASSERT_MBEDTLS( mbedtls_entropy_func( &ctx, buf, sizeof( buf ) )
                 == MBEDTLS_ERR_ENTROPY_SOURCE_FAILED );
    TEST_ASSERT_MBEDTLS( mbedtls_entropy_gather( &ctx )
                 == MBEDTLS_ERR_ENTROPY_SOURCE_FAILED );
#if defined(MBEDTLS_FS_IO) && defined(MBEDTLS_ENTROPY_NV_SEED)
    TEST_ASSERT_MBEDTLS( mbedtls_entropy_write_seed_file( &ctx, path )
                 == MBEDTLS_ERR_ENTROPY_SOURCE_FAILED );
    TEST_ASSERT_MBEDTLS( mbedtls_entropy_update_seed_file( &ctx, path )
                 == MBEDTLS_ERR_ENTROPY_SOURCE_FAILED );
#else
    ((void) path);
#endif

exit:
    mbedtls_entropy_free( &ctx );
}

void test_entropy_source_fail_wrapper( void ** params )
{

    test_entropy_source_fail( (char *) params[0] );
}
#if defined(ENTROPY_HAVE_STRONG)

void test_entropy_threshold( int threshold, int chunk_size, int result )
{
    mbedtls_entropy_context ctx;
    unsigned char buf[MBEDTLS_ENTROPY_BLOCK_SIZE] = { 0 };
    int ret;

    mbedtls_entropy_init( &ctx );

    TEST_ASSERT_MBEDTLS( mbedtls_entropy_add_source( &ctx, entropy_dummy_source,
                                     &chunk_size, threshold,
                                     MBEDTLS_ENTROPY_SOURCE_WEAK ) == 0 );

    entropy_dummy_calls = 0;
    ret = mbedtls_entropy_func( &ctx, buf, sizeof( buf ) );

    if( result >= 0 )
    {
        TEST_ASSERT_MBEDTLS( ret == 0 );
#if defined(MBEDTLS_ENTROPY_NV_SEED)
        // Two times as much calls due to the NV seed update
        result *= 2;
#endif
        TEST_ASSERT_MBEDTLS( entropy_dummy_calls == (size_t) result );
    }
    else
    {
        TEST_ASSERT_MBEDTLS( ret == result );
    }

exit:
    mbedtls_entropy_free( &ctx );
}

void test_entropy_threshold_wrapper( void ** params )
{

    test_entropy_threshold( *( (int *) params[0] ), *( (int *) params[1] ), *( (int *) params[2] ) );
}
#endif /* ENTROPY_HAVE_STRONG */
#if defined(MBEDTLS_ENTROPY_NV_SEED)
#if defined(MBEDTLS_FS_IO)

void test_nv_seed_file_create(  )
{
    unsigned char buf[MBEDTLS_ENTROPY_BLOCK_SIZE];

    memset( buf, 0, MBEDTLS_ENTROPY_BLOCK_SIZE );

    TEST_ASSERT_MBEDTLS( write_nv_seed( buf, MBEDTLS_ENTROPY_BLOCK_SIZE ) == 0 );
exit:
    ;
}

void test_nv_seed_file_create_wrapper( void ** params )
{
    (void)params;

    test_nv_seed_file_create(  );
}
#endif /* MBEDTLS_FS_IO */
#endif /* MBEDTLS_ENTROPY_NV_SEED */
#if defined(MBEDTLS_ENTROPY_NV_SEED)
#if defined(MBEDTLS_FS_IO)
#if defined(MBEDTLS_PLATFORM_NV_SEED_ALT)

void test_entropy_nv_seed_std_io(  )
{
    unsigned char io_seed[MBEDTLS_ENTROPY_BLOCK_SIZE];
    unsigned char check_seed[MBEDTLS_ENTROPY_BLOCK_SIZE];

    memset( io_seed, 1, MBEDTLS_ENTROPY_BLOCK_SIZE );
    memset( check_seed, 0, MBEDTLS_ENTROPY_BLOCK_SIZE );

    mbedtls_platform_set_nv_seed( mbedtls_platform_std_nv_seed_read,
                                  mbedtls_platform_std_nv_seed_write );

    /* Check if platform NV read and write manipulate the same data */
    TEST_ASSERT_MBEDTLS( write_nv_seed( io_seed, MBEDTLS_ENTROPY_BLOCK_SIZE ) == 0 );
    TEST_ASSERT_MBEDTLS( mbedtls_nv_seed_read( check_seed, MBEDTLS_ENTROPY_BLOCK_SIZE ) ==
                    MBEDTLS_ENTROPY_BLOCK_SIZE );

    TEST_ASSERT_MBEDTLS( memcmp( io_seed, check_seed, MBEDTLS_ENTROPY_BLOCK_SIZE ) == 0 );

    memset( check_seed, 0, MBEDTLS_ENTROPY_BLOCK_SIZE );

    /* Check if platform NV write and raw read manipulate the same data */
    TEST_ASSERT_MBEDTLS( mbedtls_nv_seed_write( io_seed, MBEDTLS_ENTROPY_BLOCK_SIZE ) ==
                    MBEDTLS_ENTROPY_BLOCK_SIZE );
    TEST_ASSERT_MBEDTLS( read_nv_seed( check_seed, MBEDTLS_ENTROPY_BLOCK_SIZE ) == 0 );

    TEST_ASSERT_MBEDTLS( memcmp( io_seed, check_seed, MBEDTLS_ENTROPY_BLOCK_SIZE ) == 0 );
exit:
    ;
}

void test_entropy_nv_seed_std_io_wrapper( void ** params )
{
    (void)params;

    test_entropy_nv_seed_std_io(  );
}
#endif /* MBEDTLS_PLATFORM_NV_SEED_ALT */
#endif /* MBEDTLS_FS_IO */
#endif /* MBEDTLS_ENTROPY_NV_SEED */

/*
 * Dummy entropy source
 *
 * If data is NULL, write exactly the requested length.
 * Otherwise, write the length indicated by data or error if negative
 */
static int entropy_dummy_source( void *data, unsigned char *output,
                                 size_t len, size_t *olen )
{
    entropy_dummy_calls++;

    if( data == NULL )
        *olen = len;
    else
    {
        int *d = (int *) data;

        if( *d < 0 )
            return MBEDTLS_ERR_ENTROPY_SOURCE_FAILED;
        else
            *olen = *d;
    }

    memset( output, 0x2a, *olen );

    return 0;
}

int* mbedtls_test_entropy()
{
    TEST_MESSAGE("----------------------------TEST FOR ENTROPY----------------------------");
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
    while ( i < sizeof(data_entropy)/sizeof(data_entropy[0]) )
    {
        total_tests++;
        buffer = (char*)malloc(sizeof(char)*strlen(data_entropy[i]));
        strcpy(buffer, data_entropy[i]);
        int unmet_dep_count = 0;
        char *unmet_dependencies[20];
        ret = 0;
        test_info.failed = 0;

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
                        TEST_MESSAGE("FATAL: Out of memory");
                        mbedtls_exit( MBEDTLS_EXIT_FAILURE );
                    }
                    unmet_dep_count++;
                }
            }
            ++i;
            free(buffer);
            buffer = (char*)malloc(sizeof(char)*strlen(data_entropy[i]));
            strcpy(buffer, data_entropy[i]);
            cnt = parse_arguments( buffer, strlen( buffer ), params,
                                    sizeof( params ) / sizeof( params[0] ) );
        }

        // If there are no unmet dependencies execute the test
        if( unmet_dep_count == 0 )
        {
            test_info.failed = 0;
            function_id = strtol( params[0], NULL, 10 );
            if ( (ret = check_test_entropy( function_id )) == DISPATCH_TEST_SUCCESS )
            {
                ret = convert_params( cnt - 1, params + 1, int_params );
                if ( DISPATCH_TEST_SUCCESS == ret )
                {
                    ret = dispatch_test_entropy( function_id, (void **)( params + 1 ) );
                }
            }
        }
        if( unmet_dep_count > 0 || ret == DISPATCH_UNSUPPORTED_SUITE )
        {
            total_skipped++;
            sprintf(unity_buffer, "Entropy-Test %d : Skipped", total_tests);
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
                sprintf(unity_buffer, "Entropy-Test %d : PASSED", total_tests);
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
        }
        else if( ret == DISPATCH_TEST_FN_NOT_FOUND )
        {
            TEST_MESSAGE("FAILED: FATAL TEST FUNCTION NOT FOUND" );
        }
        else
        {
            total_errors++;
        }
        for( int k = 0; k < unmet_dep_count; k++ )
        {
            free( unmet_dependencies[k] );
        }
        ++i;
        free(buffer);
    }

    if(total_errors == 0 && total_skipped==0)
        TEST_MESSAGE("ALL ENTROPY tests PASSED" );
    else if(total_errors>0)
        TEST_MESSAGE("ENTROPY TESTS FAILED" );
    else
        TEST_MESSAGE("Some tests skipped" );

    int* test_results = (int*)malloc(3*sizeof(int));
    test_results[0] = total_tests;
    test_results[1] = total_errors;
    test_results[2] = total_skipped;
    return test_results;
}