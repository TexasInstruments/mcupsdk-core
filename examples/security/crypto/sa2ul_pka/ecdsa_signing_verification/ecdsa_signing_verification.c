/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
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

/* This example demonstrates the ECDSA signing and verification */

#include <string.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* Sa2ul Pka Instance */
#define SA2UL_PKA_INSTANCE						(0U)

/* Openssl command To generate private key: openssl ecparam -name prime256v1 -genkey -noout -out ecdsa_prime256v1_private.pem
Openssl cmd To see content of key in text form: Openssl pkey -in ecdsa_prime256v1_private.pem -text -noout
The below key is in Bigint format please check in Api guide to know about Bigint format*/
static const uint32_t gPkaEcdsaPrivateKey[] =
{
    8UL,
	0x1079C448UL, 0xFF423CA0UL, 0xC456B088UL, 0x08394B9CUL,
	0xD2907079UL, 0xFFB35CDCUL, 0x12057B2DUL, 0xEE5ECE29UL,
};

/* Openssl command To generate EC curve params: openssl ecparam -name prime256v1 -out prime256v1.pem
Openssl cmd To see content of key in text form: openssl ecparam -in prime256v1.pem -text -param_enc explicit -noout
The below key is in Bigint format please check in Api guide to know about Bigint format*/
static const struct PKA_ECPrimeCurveP gPkaEcPrimeCurveParams =
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
static const struct PKA_ECPoint gPkaEcdsaPublicKey =
{
	{
		8UL,
		0x2135FC71UL, 0x4C40215EUL, 0x7A9C6B02UL, 0xDC4E6FD0UL,
		0x41B6D54DUL, 0x92C53C93UL, 0x6B5408A7UL, 0x59F8B5DFUL,
	},
	{
		8UL,
		0xB05C9054UL, 0x23295516UL, 0x2BDAFEA9UL, 0x31DFBBF1UL,
		0xCC6FD393UL, 0x9E5A2852UL, 0x92A14DDDUL, 0xB5E3B4A9UL,
	}
};

/* Random key with Bigint format */
static const uint32_t gPkaEcdsaRandamKey[] =
{
	8UL,
	0x3D8AAD60UL, 0x4D612949UL, 0x3382B0F2UL, 0x3B17AA87UL,
	0x8355DD4CUL, 0x08653839UL, 0xD01ABE90UL, 0xA6E3C57DUL,
};

/* Sha256 hash of message "sample" with out null at end and Bigint format */
static const uint32_t gPkaEcdsaHash [] =
{
	8UL,
	0x62ADD1BFUL, 0x62113D8AUL, 0x68E98915UL, 0x1A831D02UL,
	0x94F41FC7UL, 0xE2ADE1D6UL, 0xAA9B6EC1UL, 0xAF2BDBE1UL,
};

void ecdsa_signing_verification(void *args)
{
    Drivers_open();
    Board_driversOpen();

	/* PKA handle for processing every api's */
	PKA_Handle			pkaHandle = NULL;
	PKA_Return_t		status = PKA_RETURN_SUCCESS;
    static struct 		PKA_ECDSASig sig;

	DebugP_log("[PKA] ECDSA Signing and Verification example started ...\r\n");

	/* Open PKA instance, enable PKA engine, Initialize clocks and Load PKA Fw */
    pkaHandle = PKA_open(SA2UL_PKA_INSTANCE);
    DebugP_assert(pkaHandle != NULL);

	/* Openssl Command for Sign: openssl dgst -sha256 -sign ecdsa_prime256v1_private.pem -rand rand_key.bin -out ecdsa_sign.bin msg.bin */
    status = PKA_ECDSASign(pkaHandle, &gPkaEcPrimeCurveParams, gPkaEcdsaPrivateKey, gPkaEcdsaRandamKey, gPkaEcdsaHash, &sig);
    DebugP_assert(PKA_RETURN_SUCCESS == status);

	/* Openssl Command for Verify: openssl dgst -sha256 -verify ecdsa_prime256v1_public.pem -signature ecdsa_sign.bin msg.bin*/
    status = PKA_ECDSAVerify(pkaHandle, &gPkaEcPrimeCurveParams, &gPkaEcdsaPublicKey, &sig, gPkaEcdsaHash);
    DebugP_assert(PKA_RETURN_SUCCESS == status);

	/* Close PKA instance, disable PKA engine, deinitialize clocks*/
	status = PKA_close(pkaHandle);
	DebugP_assert(PKA_RETURN_SUCCESS == status);

	DebugP_log("[PKA] ECDSA Signing and Verification example completed!!\r\n");
    DebugP_log("All tests have passed!!\r\n");

	Board_driversClose();
    Drivers_close();
}