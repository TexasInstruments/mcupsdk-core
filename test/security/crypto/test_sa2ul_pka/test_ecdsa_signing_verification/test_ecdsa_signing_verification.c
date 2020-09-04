/*
 * Copyright (C) 2021 Texas Instruments Incorporated
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

/* This test demonstrates the PKA ECDSA Signing and verification Operations */

#include <string.h>
#include <stdlib.h>
#include <unity.h>
#include <drivers/soc.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

#define TEST_PKA_ECDSA_P_256_SIZE_IN_BYTES      (32U)
#define TEST_PKA_ECDSA_P_384_SIZE_IN_BYTES      (48U)
#define SA2UL_PKA_INSTANCE                      (0U)

/* Openssl command To generate private key: openssl ecparam -name prime256v1 -genkey -noout -out ecdsa_prime256v1_private.pem
Openssl cmd To see content of key in text form: Openssl pkey -in ecdsa_prime256v1_private.pem -text -noout
The below key is in Bigint format please check in Api guide to know about Bigint format*/
static const uint32_t gPkaEcdsaPrivateP256Key[] =
{
    8UL,
	0xD30943FBUL, 0x68DD0ADFUL, 0x6EBB144EUL, 0xD04712A3UL,
    0x97B3CAAEUL, 0x56021A66UL, 0x7E55D38EUL, 0x97356E6DUL,
};

/* Openssl command To generate EC curve params: openssl ecparam -name prime256v1 -out prime256v1.pem
Openssl cmd To see content of key in text form: openssl ecparam -in prime256v1.pem -text -param_enc explicit -noout
The below key is in Bigint format please check in Api guide to know about Bigint format*/
static const struct PKA_ECPrimeCurveP gPkaEcPrimeP256CurveParams =
{
	{
		8UL,
		0xFFFFFFFFUL, 0xFFFFFFFFUL, 0xFFFFFFFFUL, 0x00000000UL,
		0x00000000UL, 0x00000000UL, 0x00000001UL, 0xFFFFFFFFUL,
	},
	{
		8UL,
		0xFC632551UL, 0xF3B9CAC2UL, 0xA7179E84UL, 0xBCE6FAADUL,
		0xFFFFFFFFUL, 0xFFFFFFFFUL, 0x00000000UL, 0xFFFFFFFFUL,
	},
	{
		8UL,
		0xFFFFFFFCUL, 0xFFFFFFFFUL, 0xFFFFFFFFUL, 0x00000000UL,
		0x00000000UL, 0x00000000UL, 0x00000001UL, 0xFFFFFFFFUL,
	},
	{
		8UL,
		0x27d2604bUL, 0x3bce3c3eUL, 0xcc53b0f6UL, 0x651d06b0UL,
		0x769886bcUL, 0xb3ebbd55UL, 0xaa3a93e7UL, 0x5ac635d8UL
	},
	{
		{
			8UL,
			0xd898c296UL, 0xf4a13945UL, 0x2deb33a0UL, 0x77037d81UL,
			0x63a440f2UL, 0xf8bce6e5UL, 0xe12c4247UL, 0x6b17d1f2UL
		},
		{
			8UL,
			0x37bf51f5UL, 0xcbb64068UL, 0x6b315eceUL, 0x2bce3357UL,
			0x7c0f9e16UL, 0x8ee7eb4aUL, 0xfe1a7f9bUL, 0x4fe342e2UL
		}
	}
};

/* Openssl command To generate public key: openssl ec -in ecdsa_prime256v1_private.pem -pubout -out ecdsa_prime256v1_public.pem
The below key is in Bigint format please check in Api guide to know about Bigint format */
static const struct PKA_ECPoint gPkaEcdsaPublicP256Key =
{
	{
		8UL,
		0xB850404CUL, 0x33B1608DUL, 0xBD45DE96UL, 0x42ED579DUL,
        0x27611160UL, 0x6EB15919UL, 0xF5573CE1UL, 0x7F7412E2UL,
	},
	{
		8UL,
	    0x3B441835UL, 0x644FF7F6UL, 0x9F55463CUL, 0xC54E2ACBUL,
        0x89C1C2ABUL, 0x1E807955UL, 0x1B9968DEUL, 0x0714BC31UL,
	}
};

/* Random key with Bigint format */
static const uint32_t gPkaEcdsaRandamP256Key[] =
{
	8UL,
	0x3D8AAD60UL, 0x4D612949UL, 0x3382B0F2UL, 0x3B17AA87UL,
	0x8355DD4CUL, 0x08653839UL, 0xD01ABE90UL, 0xA6E3C57DUL,
};

/* Sha256 hash of message "sample" with out null at end and Bigint format */
static const uint32_t gPkaEcdsaHashP256 [] =
{
	8UL,
	0x62ADD1BFUL, 0x62113D8AUL, 0x68E98915UL, 0x1A831D02UL,
	0x94F41FC7UL, 0xE2ADE1D6UL, 0xAA9B6EC1UL, 0xAF2BDBE1UL,
};

/* Openssl command To generate private key: openssl ecparam -name secp384r1 -genkey -noout -out ecdsa_secp384r1_private.pem
Openssl cmd To see content of key in text form: Openssl pkey -in ecdsa_secp384r1_private.pem -text -noout
The below key is in Bigint format please check in Api guide to know about Bigint format*/
static const uint32_t gPkaEcdsaPrivateP384Key[] =
{
    12UL,
	0xCE28B184UL, 0xC7D49676UL, 0x5460DC6EUL, 0xD98FE0AAUL,
    0x0943D701UL, 0x024B14F8UL, 0xB1555484UL, 0x9A290CE1UL,
    0x7FDC6B64UL, 0x5DA03FBEUL, 0x3F737D05UL, 0x5C843E98UL,
};

/* Openssl command To generate EC curve params: openssl ecparam -name secp384r1 -out secp384r1.pem
Openssl cmd To see content of key in text form: openssl ecparam -in secp384r1.pem -text -param_enc explicit -noout
The below key is in Bigint format please check in Api guide to know about Bigint format*/
static const struct PKA_ECPrimeCurveP gPkaEcPrimeP384CurveParams =
{
	{
		12UL,
		0xFFFFFFFFUL, 0x00000000UL, 0x00000000UL, 0xFFFFFFFFUL,
        0xFFFFFFFEUL, 0xFFFFFFFFUL, 0xFFFFFFFFUL, 0xFFFFFFFFUL,
        0xFFFFFFFFUL, 0xFFFFFFFFUL, 0xFFFFFFFFUL, 0xFFFFFFFFUL,
	},
	{
		12UL,
		0xCCC52973UL, 0xECEC196AUL, 0x48B0A77AUL, 0x581A0DB2UL,
        0xF4372DDFUL, 0xC7634D81UL, 0xFFFFFFFFUL, 0xFFFFFFFFUL,
        0xFFFFFFFFUL, 0xFFFFFFFFUL, 0xFFFFFFFFUL, 0xFFFFFFFFUL,
	},
	{
		12UL,
		0xFFFFFFFCUL, 0x00000000UL, 0x00000000UL, 0xFFFFFFFFUL,
        0xFFFFFFFEUL, 0xFFFFFFFFUL, 0xFFFFFFFFUL, 0xFFFFFFFFUL,
        0xFFFFFFFFUL, 0xFFFFFFFFUL, 0xFFFFFFFFUL, 0xFFFFFFFFUL,
	},
	{
		12UL,
		0xD3EC2AEFUL, 0x2A85C8EDUL, 0x8A2ED19DUL, 0xC656398DUL,
        0x5013875AUL, 0x314088FUL, 0xFE814112UL, 0x181D9C6EUL,
        0xE3F82D19UL, 0x988E056BUL, 0xE23EE7E4UL, 0xB3312FA7UL,
	},
	{
		{
			12UL,
			0x72760AB7UL, 0x3A545E38UL, 0xBF55296CUL, 0x5502F25DUL,
            0x82542A38UL, 0x59F741E0UL, 0x8BA79B98UL, 0x6E1D3B62UL,
            0xF320AD74UL, 0x8EB1C71EUL, 0xBE8B0537UL, 0xAA87CA22UL,
		},
		{
			12UL,
			0x90EA0E5FUL, 0x7A431D7CUL, 0x1D7E819DUL, 0xA60B1CEUL,
            0xB5F0B8C0UL, 0xE9DA3113UL, 0x289A147CUL, 0xF8F41DBDUL,
            0x9292DC29UL, 0x5D9E98BFUL, 0x96262C6FUL, 0x3617DE4AUL,
		}
	}
};

/* Openssl command To generate public key: openssl ec -in ecdsa_secp384r1_private.pem -pubout -out ecdsa_secp384r1_public.pem
The below key is in Bigint format please check in Api guide to know about Bigint format */
static const struct PKA_ECPoint gPkaEcdsaPublicP384Key =
{
	{
		12UL,
        0x4024E398UL, 0xE1123361UL, 0xA2C9529FUL, 0x1500BA75UL,
        0xCBC0419AUL, 0xA9B2776CUL, 0x48896AF0UL, 0x41AEA6A8UL,
        0x8AE66008UL, 0x49931CD5UL, 0xA92EDF38UL, 0x1E9DBD2CUL,
	},
	{
		12UL,
        0x8A04C848UL, 0x6C5B6621UL, 0xF883A81BUL, 0xF4A22B39UL,
        0xF0340253UL, 0xA377C9A2UL, 0x7B7DA66CUL, 0xFE38EBF8UL,
        0x72EAD75EUL, 0xF4A833D0UL, 0xB2E0225DUL, 0x7AAA0D3BUL,
	}
};

/* Random key with Bigint format */
static const uint32_t gPkaEcdsaRandamP384Key[] =
{
	12UL,
	0x86915CF9UL, 0x623B8C46UL, 0x3BA95368UL, 0x2907E3E8UL,
    0x6D10FBCAUL, 0x7A555FD5UL, 0x8EAF0CA8UL, 0xE4BA1516UL,
    0x2AE85ABDUL, 0x3254E924UL, 0x1A099DADUL, 0x94ED910DUL,
};

/* Sha384 hash of message "sample" with out null at end and Bigint format */
static const uint32_t gPkaEcdsaHashP384 [] =
{
    12UL,
	0xB1EE25FEUL, 0xFEE42C77UL, 0x9B5B890EUL, 0x313BCA4AUL,
    0x40585312UL, 0x96A029F3UL, 0x4BBD3811UL, 0xF3BF603FUL,
    0x2696EF7BUL, 0xAEC4BE31UL, 0x5BC92276UL, 0x9A908350UL,
};

/* Local test functions */
static void test_pka_ecdsa_sign_verify_p_256(void *args);
static void test_pka_ecdsa_sign_verify_p_384(void *args);

void App_printPerformanceResults(uint64_t t1, uint64_t t2);
void App_printTotalPerformanceResults(uint64_t tTotal);

/* PKA handle for processing every api's */
PKA_Handle			gPkaHandle = NULL;

void test_main(void *args)
{
    Drivers_open();
    Board_driversOpen();

	PKA_Return_t             status = PKA_RETURN_SUCCESS;

    DebugP_log("[PKA] ECDSA Signing and verification example started ...\r\n");

  	/* Open PKA instance, enable PKA engine, Initialize clocks and Load PKA Fw */
    gPkaHandle = PKA_open(SA2UL_PKA_INSTANCE);
    TEST_ASSERT_EQUAL_UINT32(PKA_RETURN_SUCCESS, status);

    RUN_TEST(test_pka_ecdsa_sign_verify_p_256,  2714, NULL);
	RUN_TEST(test_pka_ecdsa_sign_verify_p_384,  2674, NULL);

    /* Close PKA instance, disable PKA engine, deinitialize clocks*/
	status = PKA_close(gPkaHandle);
	TEST_ASSERT_EQUAL_UINT32(PKA_RETURN_SUCCESS, status);

    UNITY_END();
    Board_driversClose();
    Drivers_close();
}

/** Unity framework required functions */
void setUp(void)
{
}

void tearDown(void)
{
}

void test_pka_ecdsa_sign_verify_p_256(void *args)
{
	PKA_Return_t 		status = PKA_RETURN_SUCCESS;
    static struct 		PKA_ECDSASig sig;
    uint64_t            t1, t2, tTotal;

	DebugP_log("[PKA] ECDSA signing and verification with P-256 started ...\r\n");

    t1 = ClockP_getTimeUsec();

    /* Openssl Command for Sign: openssl dgst -sha256 -sign ecdsa_prime256v1_private.pem -rand rand_key.bin -out ecdsa_sign.bin msg.bin */
    status = PKA_ECDSASign(gPkaHandle, &gPkaEcPrimeP256CurveParams, gPkaEcdsaPrivateP256Key, gPkaEcdsaRandamP256Key, gPkaEcdsaHashP256, &sig);
    TEST_ASSERT_EQUAL_UINT32(PKA_RETURN_SUCCESS, status);

    t2 = ClockP_getTimeUsec();
    DebugP_log("ECDSA Signing Performance :\r\n");
    App_printPerformanceResults(t1, t2);

    tTotal = t2 - t1;

    t1 = ClockP_getTimeUsec();

    /* Openssl Command for Verify: openssl dgst -sha256 -verify ecdsa_prime256v1_public.pem -signature ecdsa_sign.bin msg.bin*/
    status = PKA_ECDSAVerify(gPkaHandle, &gPkaEcPrimeP256CurveParams, &gPkaEcdsaPublicP256Key, &sig, gPkaEcdsaHashP256);
    DebugP_assert(PKA_RETURN_SUCCESS == status);

    t2 = ClockP_getTimeUsec();
    DebugP_log("ECDSA Verification Performance :\r\n");
    App_printPerformanceResults(t1, t2);

    tTotal = tTotal +(t2 - t1);

    App_printTotalPerformanceResults(tTotal);
    return;
}

void test_pka_ecdsa_sign_verify_p_384(void *args)
{
	PKA_Return_t 		status = PKA_RETURN_SUCCESS;
    static struct 		PKA_ECDSASig sig;
    static uint64_t     t1, t2, tTotal;

	DebugP_log("[PKA] ECDSA signing and verification with P-384 started ...\r\n");

    t1 = ClockP_getTimeUsec();

    /* Openssl Command for Sign: openssl dgst -sha256 -sign ecdsa_secp384r1_private.pem -rand rand_key.bin -out ecdsa_sign.bin msg.bin */
    status = PKA_ECDSASign(gPkaHandle, &gPkaEcPrimeP384CurveParams, gPkaEcdsaPrivateP384Key, gPkaEcdsaRandamP384Key, gPkaEcdsaHashP384, &sig);
    TEST_ASSERT_EQUAL_UINT32(PKA_RETURN_SUCCESS, status);

    t2 = ClockP_getTimeUsec();
    DebugP_log("ECDSA Signing Performance :\r\n");
    App_printPerformanceResults(t1, t2);

    tTotal = t2 - t1;

    t1 =ClockP_getTimeUsec();

    /* Openssl Command for Verify: openssl dgst -sha256 -verify ecdsa_secp384r1_public.pem -signature ecdsa_sign.bin msg.bin*/
    status = PKA_ECDSAVerify(gPkaHandle, &gPkaEcPrimeP384CurveParams, &gPkaEcdsaPublicP384Key, &sig, gPkaEcdsaHashP384);
    TEST_ASSERT_EQUAL_UINT32(PKA_RETURN_SUCCESS, status);

    t2 = ClockP_getTimeUsec();
    DebugP_log("ECDSA Verification Performance :\r\n");
    App_printPerformanceResults(t1, t2);

    tTotal = tTotal +(t2 - t1);
    App_printTotalPerformanceResults(tTotal);

    return;
}

void App_printPerformanceResults(uint64_t t1, uint64_t t2)
{
    uint64_t totalTimeInMicroSec = t2 - t1;
    uint64_t throughputInOps = 1000000/totalTimeInMicroSec;

    DebugP_log("[CRYPTO] Tstart(us) : %ld \r\n", t1);
	DebugP_log("[CRYPTO] Tend(us)   : %ld \r\n", t2);
	DebugP_log("[CRYPTO] Tdiff(us)   : %ld \r\n", totalTimeInMicroSec);
    DebugP_log("[CRYPTO] Operations/seconds  : %ld \r\n", throughputInOps);
}

void App_printTotalPerformanceResults(uint64_t tTotal)
{
    uint64_t throughputInOps = 1000000/tTotal;

    DebugP_log("[CRYPTO] Ttotal(us) : %ld \r\n", tTotal);
    DebugP_log("[CRYPTO] Sign and Verify Operations/seconds  : %ld \r\n", throughputInOps);
}