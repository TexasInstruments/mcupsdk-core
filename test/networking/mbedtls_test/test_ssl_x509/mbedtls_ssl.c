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
    data_ssl is an array of string literals of test data for mbedTLS sanity tests for SSL
    these data vectors will be used in case of both software and hardware cryptography offloaded mbedTLS (TO DO)
*/

const char* data_ssl[] = {
    "0:hex:\"\":hex:\"000000000000\":int:0",
    "0:hex:\"000000000000\":hex:\"000000000001\":int:0",
    "0:hex:\"000000000000\":hex:\"000000000000\":exp:0",
    "0:hex:\"000000000000000000000001\":hex:\"000000000002\":int:0",
    "0:hex:\"000000000000000000000001\":hex:\"000000000001\":exp:0",
    "0:hex:\"000000000000000000000001\":hex:\"000000000000\":exp:0",
    "0:hex:\"abcd12340000abcd12340001abcd12340003\":hex:\"abcd12340004\":int:0",
    "0:hex:\"abcd12340000abcd12340001abcd12340003\":hex:\"abcd12350000\":int:0",
    "0:hex:\"abcd12340000abcd12340001abcd12340003\":hex:\"abcd12340002\":int:0",
    "0:hex:\"abcd12340000abcd12340001abcd12340003\":hex:\"abcd12340003\":exp:0",
    "0:hex:\"abcd12340000abcd12340001abcd12340003\":hex:\"abcd12340001\":exp:0",
    "0:hex:\"abcd12340000abcd12340002abcd12340003\":hex:\"abcd12340002\":exp:0",
    "0:hex:\"abcd12340000abcd12340001abcd1234003f\":hex:\"abcd12340000\":exp:0",
    "0:hex:\"abcd12340001abcd12340002abcd1234003f\":hex:\"abcd12340000\":int:0",
    "0:hex:\"abcd12340001abcd12340002abcd1234003f\":hex:\"abcd1233ffff\":exp:0",
    "0:hex:\"abcd12340001abcd12340002abcd1234003f\":hex:\"abcd12330000\":exp:0",
    "0:hex:\"abcd12340000abcd12340100\":hex:\"abcd12340100\":exp:0",
    "0:hex:\"abcd12340000abcd12340100\":hex:\"abcd12340101\":int:0",
    "0:hex:\"abcd12340000abcd12340100\":hex:\"abcd123400ff\":int:0",
    "1:char*:\"server0\":char*:\"server1\"",
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

static void test_fail( const char *test, int line_no, const char* filename )
{
    test_info.failed = 1;
    test_info.test = test;
    test_info.line_no = line_no;
    test_info.filename = filename;
}

/* TEST-FUNCTIONS */
static int dep_check( int dep_id );
static int get_expression( int32_t exp_id, int32_t * out_value );
static int check_test_ssl( int func_idx );
static int dispatch_test_ssl( int func_idx, void ** params );
static int convert_params( size_t cnt , char ** params , int * int_params_store );

void test_ssl_set_hostname_twice_wrapper( void ** params );
void test_ssl_set_hostname_twice( char *hostname0, char *hostname1 );
void test_ssl_dtls_replay_wrapper( void ** params );
void test_ssl_dtls_replay( data_t * prevs, data_t * new, int ret );

/* ========================================================================== */
/*                             FUNCTION DEFINITIONS                           */
/* ========================================================================== */

TestWrapper_t test_funcs_ssl[] =
{

    /* Function Id: 0 */
    #if defined(MBEDTLS_SSL_TLS_C) && defined(MBEDTLS_SSL_DTLS_ANTI_REPLAY)
        test_ssl_dtls_replay_wrapper,
    #else
        NULL,
    #endif
    /* Function Id: 1 */

    #if defined(MBEDTLS_SSL_TLS_C) && defined(MBEDTLS_X509_CRT_PARSE_C)
        test_ssl_set_hostname_twice_wrapper,
    #else
        NULL,
    #endif

};

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
int dep_check( int dep_id )
{
    int ret = DEPENDENCY_NOT_SUPPORTED;

    (void) dep_id;

    switch( dep_id )
    {

    #if defined(MBEDTLS_SSL_TLS_C)

    #endif

        default:
            break;
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
int get_expression( int32_t exp_id, int32_t * out_value )
{
    int ret = KEY_VALUE_MAPPING_FOUND;

    (void) exp_id;
    (void) out_value;

    switch( exp_id )
    {
        #if defined(MBEDTLS_SSL_TLS_C)
        case 0:
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
 * \brief       Checks if test function is supported
 *
 * \param exp_id    Test function index.
 *
 * \return       DISPATCH_TEST_SUCCESS if found
 *               DISPATCH_TEST_FN_NOT_FOUND if not found
 *               DISPATCH_UNSUPPORTED_SUITE if not compile time enabled.
 */
int check_test_ssl( int func_idx )
{
    int ret = DISPATCH_TEST_SUCCESS;
    TestWrapper_t fp = NULL;

    if ( func_idx < (int)( sizeof(test_funcs_ssl)/sizeof( TestWrapper_t ) ) )
    {
        fp = test_funcs_ssl[func_idx];
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
int dispatch_test_ssl( int func_idx, void ** params )
{
    int ret = DISPATCH_TEST_SUCCESS;
    TestWrapper_t fp = NULL;

    if ( func_idx < (int)( sizeof( test_funcs_ssl ) / sizeof( TestWrapper_t ) ) )
    {
        fp = test_funcs_ssl[func_idx];
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

#if defined(MBEDTLS_SSL_TLS_C)
#if defined(MBEDTLS_SSL_DTLS_ANTI_REPLAY)
void test_ssl_dtls_replay( data_t * prevs, data_t * new, int ret )
{
    uint32_t len = 0;
    mbedtls_ssl_context ssl;
    mbedtls_ssl_config conf;

    mbedtls_ssl_init( &ssl );
    mbedtls_ssl_config_init( &conf );

    TEST_ASSERT_MBEDTLS( mbedtls_ssl_config_defaults( &conf,
                 MBEDTLS_SSL_IS_CLIENT,
                 MBEDTLS_SSL_TRANSPORT_DATAGRAM,
                 MBEDTLS_SSL_PRESET_DEFAULT ) == 0 );
    TEST_ASSERT_MBEDTLS( mbedtls_ssl_setup( &ssl, &conf ) == 0 );

    /* Read previous record numbers */
    for( len = 0; len < prevs->len; len += 6 )
    {
        memcpy( ssl.in_ctr + 2, prevs->x + len, 6 );
        mbedtls_ssl_dtls_replay_update( &ssl );
    }

    /* Check new number */
    memcpy( ssl.in_ctr + 2, new->x, 6 );
    TEST_ASSERT_MBEDTLS( mbedtls_ssl_dtls_replay_check( &ssl ) == ret );

    mbedtls_ssl_free( &ssl );
    mbedtls_ssl_config_free( &conf );
exit:
    ;
}

void test_ssl_dtls_replay_wrapper( void ** params )
{
    data_t data0 = {(uint8_t *) params[0], *( (uint32_t *) params[1] )};
    data_t data2 = {(uint8_t *) params[2], *( (uint32_t *) params[3] )};

    test_ssl_dtls_replay( &data0, &data2, *( (int *) params[4] ) );
}
#endif /* MBEDTLS_SSL_DTLS_ANTI_REPLAY */
#if defined(MBEDTLS_X509_CRT_PARSE_C)
void test_ssl_set_hostname_twice( char *hostname0, char *hostname1 )
{
    mbedtls_ssl_context ssl;
    mbedtls_ssl_init( &ssl );

    TEST_ASSERT_MBEDTLS( mbedtls_ssl_set_hostname( &ssl, hostname0 ) == 0 );
    TEST_ASSERT_MBEDTLS( mbedtls_ssl_set_hostname( &ssl, hostname1 ) == 0 );

    mbedtls_ssl_free( &ssl );
exit:
    ;
}

void test_ssl_set_hostname_twice_wrapper( void ** params )
{

    test_ssl_set_hostname_twice( (char *) params[0], (char *) params[1] );
}
#endif /* MBEDTLS_X509_CRT_PARSE_C */
#endif /* MBEDTLS_SSL_TLS_C */

int* mbedtls_test_ssl()
{
    TEST_MESSAGE("----------------------------TEST FOR SSL----------------------------");
    uint8_t i=0 ,j=0;
    int ret = 0;
    uint8_t function_id = 0;
    char *params[50];
    char unity_buffer[100];
    int int_params[50];
    char* buffer;
    /* Other Local variables */
    int cnt;
    int total_errors = 0, total_tests = 0, total_skipped = 0;
    while ( i < sizeof(data_ssl)/sizeof(data_ssl[0]) )
    {
        total_tests++;
        buffer = malloc(sizeof(char)*strlen(data_ssl[i]));
        strcpy(buffer, data_ssl[i]);
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
                        TEST_MESSAGE("FATAL: Out of memory" );
                        mbedtls_exit( MBEDTLS_EXIT_FAILURE );
                    }
                    unmet_dep_count++;
                }
            }
            ++i;
            free(buffer);
            buffer = malloc(sizeof(char)*strlen(data_ssl[i]));
            strcpy(buffer, data_ssl[i]);
            cnt = parse_arguments( buffer, strlen( buffer ), params,
                                    sizeof( params ) / sizeof( params[0] ) );
        }

        // If there are no unmet dependencies execute the test
        if( unmet_dep_count == 0 )
        {
            test_info.failed = 0;
            function_id = strtol( params[0], NULL, 10 );
            if ( (ret = check_test_ssl( function_id )) == DISPATCH_TEST_SUCCESS )
            {
                ret = convert_params( cnt - 1, params + 1, int_params );
                if ( DISPATCH_TEST_SUCCESS == ret )
                {
                    ret = dispatch_test_ssl( function_id, (void **)( params + 1 ) );
                }
            }
        }
        if( unmet_dep_count > 0 || ret == DISPATCH_UNSUPPORTED_SUITE )
        {
            total_skipped++;
            sprintf(unity_buffer, "SSL-Test %d : Skipped", total_tests);
            TEST_MESSAGE(unity_buffer);

            if(ret == DISPATCH_UNSUPPORTED_SUITE )
            {
                TEST_MESSAGE("Test Suite not enabled" );
            }

            if(unmet_dep_count > 0 )
            {
                TEST_MESSAGE("Unmet dependencies: " );
            }
            TEST_MESSAGE("" );
            unmet_dep_count = 0;
        }
        else if( ret == DISPATCH_TEST_SUCCESS )
        {
            if( test_info.failed == 0 )
            {
                sprintf(unity_buffer, "SSL-Test %d : PASSED", total_tests);
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
        TEST_MESSAGE("ALL SSL PASSED" );
    else if(total_errors>0)
        TEST_MESSAGE("SSL TESTS FAILED" );
    else
        TEST_MESSAGE("Some tests skipped" );

    int* test_results = malloc(3*sizeof(int));
    test_results[0] = total_tests;
    test_results[1] = total_errors;
    test_results[2] = total_skipped;
    return test_results;
}