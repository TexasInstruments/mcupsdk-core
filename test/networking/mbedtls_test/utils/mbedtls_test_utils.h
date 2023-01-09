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

/* ========================================================================== */
/*                             INCLUDE FILES                                  */
/* ========================================================================== */

#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "string.h"
#include <unity.h>
#include <kernel/dpl/DebugP.h>
#include "alt_config.h"
#include "mbedtls/padlock.h"
#include "mbedtls/aes.h"
#include "mbedtls/gcm.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/ccm.h"
#include "mbedtls/ecp.h"
#include "mbedtls/ecdsa.h"
#include "mbedtls/rsa.h"
#include "mbedtls/rsa_internal.h"
#include "mbedtls/platform_util.h"
#include "mbedtls/aesni.h"
#include "mbedtls/platform.h"
#include "mbedtls/hmac_drbg.h"
#include "kernel/dpl/CacheP.h"
#include "mbedtls/sha1.h"
#include "mbedtls/sha256.h"
#include "mbedtls/sha512.h"
#include "mbedtls/entropy.h"
#include "mbedtls/md2.h"
#include "mbedtls/md4.h"
#include "mbedtls/md5.h"
#include <mbedtls/ssl.h>
#include "mbedtls/bignum.h"
#include "mbedtls/x509.h"
#include "mbedtls/x509_crt.h"
#include "mbedtls/x509_crl.h"
#include "mbedtls/x509_csr.h"
#include "mbedtls/compat-1.3.h"
#include "mbedtls/pem.h"
#include "mbedtls/oid.h"
#include "mbedtls/base64.h"
#include <mbedtls/ssl_internal.h>
#if defined(MBEDTLS_MEMORY_BUFFER_ALLOC_C)
#include "mbedtls/memory_buffer_alloc.h"
#endif

/* ========================================================================== */
/*                           MACROS & TYPEDEFS                                */
/* ========================================================================== */

#ifndef PUT_UINT32_BE
#define PUT_UINT32_BE(n,b,i)                            \
{                                                       \
    (b)[(i)    ] = (unsigned char) ( (n) >> 24 );       \
    (b)[(i) + 1] = (unsigned char) ( (n) >> 16 );       \
    (b)[(i) + 2] = (unsigned char) ( (n) >>  8 );       \
    (b)[(i) + 3] = (unsigned char) ( (n)       );       \
}
#endif

typedef struct
{
    unsigned char *p;
    size_t len;
} entropy_ctx;
typedef struct data_tag
{
    uint8_t*    x;
    uint32_t    len;
} data_t;

/**
 * Info structure for the pseudo random function
 *
 * Key should be set at the start to a test-unique value.
 * Do not forget endianness!
 * State( v0, v1 ) should be set to zero.
 */
typedef struct
{
    uint32_t key[16];
    uint32_t v0, v1;
} rnd_pseudo_info;

typedef struct
{
    unsigned char *buf;
    size_t length;
} rnd_buf_info;

/**
 * \brief       Function pointer type for test function wrappers.
 *
 *
 * \param void **   Pointer to void pointers. Represents an array of test
 *                  function parameters.
 *
 * \return       void
 */
typedef void (*TestWrapper_t)( void ** );

#define assert(a) if( !( a ) )                                      \
{                                                                   \
    mbedtls_fprintf( stderr, "Assertion Failed at %s:%d - %s\n",   \
                             __FILE__, __LINE__, #a );              \
    mbedtls_exit( 1 );                                             \
}

#define INCR_ASSERT(p, start, len, step) do                     \
{                                                               \
    assert( ( p ) >= ( start ) );                               \
    assert( sizeof( *( p ) ) == sizeof( *( start ) ) );         \
    /* <= is checked to support use inside a loop where         \
       pointer is incremented after reading data.       */      \
    assert( (uint32_t)( ( ( p ) - ( start ) ) + ( step ) ) <= ( len ) );\
    ( p ) += ( step );                                          \
}                                                               \
while( 0 )

/**
 * \brief       4 byte align unsigned char pointer
 *
 * \param p     Pointer to byte array
 * \param start Pointer to start of byte array
 * \param len   Length of byte array
 *
 */
#define ALIGN_32BIT(p, start, len) do               \
{                                                   \
    uint32_t align = ( - (uintptr_t)( p ) ) % 4;    \
    INCR_ASSERT( ( p ), ( start ), ( len ), align );\
}                                                   \
while( 0 )

/*----------------------------------------------------------------------------*/
/* Status and error constants */

#define DEPENDENCY_SUPPORTED            0   /* Dependency supported by build */
#define KEY_VALUE_MAPPING_FOUND         0   /* Integer expression found */
#define DISPATCH_TEST_SUCCESS           0   /* Test dispatch successful */

#define KEY_VALUE_MAPPING_NOT_FOUND     -1  /* Integer expression not found */
#define DEPENDENCY_NOT_SUPPORTED        -2  /* Dependency not supported */
#define DISPATCH_TEST_FN_NOT_FOUND      -3  /* Test function not found */
#define DISPATCH_INVALID_TEST_DATA      -4  /* Invalid test parameter type. Only int, string, binary data and integer expressions are allowed */
#define DISPATCH_UNSUPPORTED_SUITE      -5  /* Test suite not supported by the build  */

/* ========================================================================== */
/*                          FUNCTION DECLARATIONS                             */
/* ========================================================================== */

static int get_expression( int32_t exp_id, int32_t * out_value );
void hexify( unsigned char *obuf, const unsigned char *ibuf, int len );
int hexcmp( uint8_t * a, uint8_t * b, uint32_t a_len, uint32_t b_len );
int verify_int( char *str, int *value );
int verify_string( char **str );
int unhexify( unsigned char *obuf, const char *ibuf );
int parse_arguments( char *buf, size_t len, char **params,
                            size_t params_len );
unsigned char *unhexify_alloc( const char *ibuf, size_t *olen );
unsigned char *zero_alloc( size_t len );