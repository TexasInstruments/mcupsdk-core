/*
 * Copyright (C) 2022 Texas Instruments Incorporated
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

/* This test demonstrates the PKA RSA Signing and verification Operations */

#include <string.h>
#include <stdlib.h>
#include <unity.h>
#include <drivers/soc.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <drivers/hw_include/am263x/cslr_soc_baseaddress.h>
#include <security/crypto/pka/pka.h>
#include <security/crypto/dthe/dthe.h>
#include <security/crypto/dthe/dthe_sha.h>

static uint32_t gPkaRsaSignOutputResult[PKA_BIGINT_MAX];
static uint32_t gPkaRsaVerifyOutputResult[PKA_BIGINT_MAX];
static uint32_t gPkaRsaShaHashWith32BitFormate[PKA_BIGINT_MAX];
static uint32_t gPkaRsaShaHashWithBigIntFormate[PKA_BIGINT_MAX];
static uint8_t  gPkaRsaShaHashWithPadding[(PKA_BIGINT_MAX * 4)];

/* SHA512 length */
#define APP_SHA512_LENGTH                           (64U)

/* Message length */
#define TEST_PKA_RSA_MSG_SIZE_IN_BYTES	            (512U)

/* For 2048 bit key*/
#define TEST_PKA_RSA_2048_BIT_KEY_SIZE_IN_BYTES	    (256U)
#define TEST_PKA_RSA_2048_BIT_KEY_SIZE_IN_WORDS	    (64U)

/* For 4096 bit key*/
#define TEST_PKA_RSA_4096_BIT_KEY_SIZE_IN_BYTES	    (512U)
#define TEST_PKA_RSA_4096_BIT_KEY_SIZE_IN_WORDS	    (128U)
#define DTHE_PKA_INSTANCE                           (0U)
/* DTHE Public address */
#define CSL_DTHE_PUBLIC_U_BASE                      (0xCE000810U)
/* DTHE Aes Public address */
#define CSL_DTHE_PUBLIC_AES_U_BASE                  (0xCE007000U)
/* DTHE Aes Public address */
#define CSL_DTHE_PUBLIC_SHA_U_BASE                  (0xCE005000U)

/* Openssl command To generate public key : Openssl rsa -pubout -in private.pem -out public.pem*/
static const struct PKA_RSAPubkey gPkaRsa2kPublicKey = 
{
	{
		64UL,
		0x8C6C85B9UL, 0x056953ACUL, 0x9F24B6E7UL, 0x91D77638UL,
		0x3E28B3C7UL, 0x3DDCCC06UL, 0xBFD47EB2UL, 0x6B606CC6UL,
		0x5175DF58UL, 0x7D2A303DUL, 0xCB5A09CEUL, 0xD9430637UL,
		0x6CCB3436UL, 0x6B12FE2FUL, 0xAC169F63UL, 0xEE112EA4UL,
		0x3FA18B8DUL, 0xD35A6216UL, 0x9A95897DUL, 0x10C853C2UL,
		0x371C8456UL, 0xE767B532UL, 0x4834F9C3UL, 0x8B00B9E8UL,
		0x47DC231DUL, 0xC979A0E1UL, 0x7FB0CC0DUL, 0x2EA266CDUL,
		0x3EDE8C2DUL, 0xC78D2E6FUL, 0xB27C8E0CUL, 0x5C4DC759UL,
		0x72D66874UL, 0x589ED84BUL, 0x1A23F0B0UL, 0xEE420C45UL,
		0x0930D8F4UL, 0x9793E7F4UL, 0x19586A35UL, 0xED664D03UL,
		0xA55EF9D6UL, 0x3CA68A5DUL, 0x93275527UL, 0x210F0365UL,
		0x4336EB94UL, 0x3BE61CD6UL, 0x330F4E75UL, 0x79816748UL,
		0xD1906A36UL, 0x9D311DB6UL, 0x0BF47E6FUL, 0xBE7D5D82UL,
		0xB3DC1598UL, 0x6E9F6DB7UL, 0xCFCC4A8AUL, 0xDFBA2F3EUL,
		0x0CEA1413UL, 0xF6E7AE2FUL, 0x0429A8F5UL, 0xFF21AFADUL,
		0x9E529C69UL, 0x75245098UL, 0x958033AFUL, 0xC56969C2UL,
	},
	{   1UL,
		0x00010001UL, 
	}
};

/* Openssl command To generate private key : Openssl genrsa -out private.pem*/
static const struct PKA_RSAPrivkey gPkaRsa2kPrivateKey = 
{
	{
		64UL,
		0x8C6C85B9UL, 0x056953ACUL, 0x9F24B6E7UL, 0x91D77638UL,
		0x3E28B3C7UL, 0x3DDCCC06UL, 0xBFD47EB2UL, 0x6B606CC6UL,
		0x5175DF58UL, 0x7D2A303DUL, 0xCB5A09CEUL, 0xD9430637UL,
		0x6CCB3436UL, 0x6B12FE2FUL, 0xAC169F63UL, 0xEE112EA4UL,
		0x3FA18B8DUL, 0xD35A6216UL, 0x9A95897DUL, 0x10C853C2UL,
		0x371C8456UL, 0xE767B532UL, 0x4834F9C3UL, 0x8B00B9E8UL,
		0x47DC231DUL, 0xC979A0E1UL, 0x7FB0CC0DUL, 0x2EA266CDUL,
		0x3EDE8C2DUL, 0xC78D2E6FUL, 0xB27C8E0CUL, 0x5C4DC759UL,
		0x72D66874UL, 0x589ED84BUL, 0x1A23F0B0UL, 0xEE420C45UL,
		0x0930D8F4UL, 0x9793E7F4UL, 0x19586A35UL, 0xED664D03UL,
		0xA55EF9D6UL, 0x3CA68A5DUL, 0x93275527UL, 0x210F0365UL,
		0x4336EB94UL, 0x3BE61CD6UL, 0x330F4E75UL, 0x79816748UL,
		0xD1906A36UL, 0x9D311DB6UL, 0x0BF47E6FUL, 0xBE7D5D82UL,
		0xB3DC1598UL, 0x6E9F6DB7UL, 0xCFCC4A8AUL, 0xDFBA2F3EUL,
		0x0CEA1413UL, 0xF6E7AE2FUL, 0x0429A8F5UL, 0xFF21AFADUL,
		0x9E529C69UL, 0x75245098UL, 0x958033AFUL, 0xC56969C2UL,
	},
	{
		1UL,
		0x00010001UL,
	},
	{
		64UL,
		0x76B24971UL, 0xABAE2708UL, 0x7090F0B9UL, 0xF2B4C776UL,
		0xD30CD9E2UL, 0x77A17624UL, 0xCC8C2F9CUL, 0x205ADBD6UL,
		0x709F9778UL, 0xBF700835UL, 0x39415524UL, 0x2603467DUL,
		0xD19476C9UL, 0x57176E0AUL, 0xE6EB00C1UL, 0x9F2C8188UL,
		0xAEB5AB6CUL, 0x3CC340A0UL, 0x86C8D4B8UL, 0x9ED2DE1AUL,
		0x1DE5126CUL, 0x91C518A6UL, 0xB2F933DEUL, 0x588146D2UL,
		0xCB2D4E92UL, 0xD4420D3AUL, 0xE9C29055UL, 0xAA0E226EUL,
		0x86E83ED8UL, 0xBFCB8270UL, 0x0BEECA18UL, 0x22FEE79EUL,
		0xFB39B96DUL, 0x3A461E0AUL, 0x5F9CB1A5UL, 0x7A3CB7C8UL,
		0x8AD36D97UL, 0xDED1073BUL, 0x95A4B28CUL, 0x0D8F7612UL,
		0xECDBA3A1UL, 0xD7B6B2D6UL, 0xC493C52CUL, 0x30D4DB55UL,
		0x1435D7E4UL, 0x6369AA17UL, 0xB0F2E9FDUL, 0xE6604C1CUL,
		0xF634C564UL, 0xD619065AUL, 0xCC97DF2AUL, 0x4B88C523UL,
		0x3437D542UL, 0x48FD52CAUL, 0x125BD29EUL, 0x0D6C69D7UL,
		0xBDFCA12AUL, 0x69BD7BAAUL, 0xFD5D8D04UL, 0x24E4DFEDUL,
		0x3F3E724FUL, 0x9EAD06CAUL, 0x0EE1B3A8UL, 0x171211ADUL,
	},
	{
		32UL,
		0x1615910DUL, 0x896F86ADUL, 0x3FA600B5UL, 0xBE5CAE1FUL,
		0x431ACC84UL, 0xBE0B507FUL, 0xE19E3780UL, 0xE4B1701BUL,
		0x9C03114EUL, 0x5A2C16D7UL, 0xFDE1682FUL, 0xDBAEE508UL,
		0xD9A2A8A2UL, 0x03EFB7A2UL, 0xDEAA84EFUL, 0x8AEE33C9UL,
		0x0D95FED7UL, 0xA3ED1F55UL, 0xB0EA72C7UL, 0x773291B0UL,
		0x055BBBE7UL, 0x50ABE824UL, 0x936448DFUL, 0x6D96E050UL,
		0x1AE0060CUL, 0xF513D05DUL, 0x53C29C4CUL, 0x5BE4EDE3UL,
		0x53B357C3UL, 0xC2624CCFUL, 0x508FA4EAUL, 0xFA5D6875UL,
	},
	{
		32UL,
		0xB0B5245DUL, 0x6B6FBB98UL, 0xC9D6FA06UL, 0xB35E18FAUL,
		0x180B5D7DUL, 0xFF895C08UL, 0xF311CE70UL, 0x03B9D7F1UL,
		0x17FBC6BAUL, 0x5D06779BUL, 0x3F7F6FC8UL, 0xA84C4879UL,
		0x26516954UL, 0xC6A3D297UL, 0x70566BEEUL, 0x161F2DE1UL,
		0xC1ED432BUL, 0xF38BF70CUL, 0x2A3BA241UL, 0x85256B85UL,
		0x738F5EDAUL, 0x54076D5AUL, 0x0FC4655CUL, 0x44440091UL,
		0xD8D167B7UL, 0x2A45134DUL, 0xE2D0891EUL, 0x7427A577UL,
		0xBC7B661EUL, 0x33AFA2F3UL, 0x98E19426UL, 0xC9DAE439UL,
	},
	{
		32UL,
		0x37383D85UL, 0xF81ABE55UL, 0xED5CFFEAUL, 0x769715C7UL,
		0xEAFF10E5UL, 0x64D022FEUL, 0xDCF6656FUL, 0x20AAC7E2UL,
		0xBDBB9A84UL, 0xCA084CA2UL, 0xEE9C9558UL, 0x338C49D4UL,
		0xC3DF145CUL, 0x1B91A89BUL, 0xA6158E06UL, 0x7C6F439FUL,
		0xEBD7825CUL, 0x6615B326UL, 0x57CF4140UL, 0xD8DD57C1UL,
		0xCD38197DUL, 0xB0F3B372UL, 0xAE891C00UL, 0xF1A1CBBAUL,
		0x0F15E9CDUL, 0xD1DD5A08UL, 0xCA5DFAA8UL, 0xDA257067UL,
		0x54983EF3UL, 0x98F84C4EUL, 0xEE658CBDUL, 0x345D12FAUL,
	},
	{
		32UL,
		0x1ABB7FA9UL, 0xA5EF70EAUL, 0x8416D267UL, 0xC5B28801UL,
		0xBEB01413UL, 0x84B2EE52UL, 0x3174051DUL, 0x2243F2A8UL,
		0xD09F9009UL, 0xB1C004A5UL, 0x4FFFCE1BUL, 0x02B68260UL,
		0x5960F482UL, 0x1FD99580UL, 0x6969D74BUL, 0x05C522CAUL,
		0x1E331D08UL, 0x902FAFBAUL, 0x7DC0D445UL, 0x7C30D555UL,
		0xA49DFB17UL, 0x2BA5A83DUL, 0x8441CD44UL, 0x42443663UL,
		0x6E401D09UL, 0xA75D0581UL, 0x18C311F8UL, 0xD21AAE22UL,
		0xAD807E6FUL, 0x3A1CDDD1UL, 0x79F297BDUL, 0xC7FF6F7DUL,
	},
	{
		32UL,
		0x05F2328EUL, 0xBCD27D4CUL, 0x77F5E127UL, 0xE7FADF6DUL,
		0xDD8E2EB2UL, 0x8AC22E8AUL, 0x95B55248UL, 0xD55D3842UL,
		0x446C93E0UL, 0x2F127699UL, 0x4A991495UL, 0x5A4CE6BAUL,
		0x020CDB4EUL, 0xE52F73A1UL, 0xBB225AE5UL, 0xDB31939EUL,
		0x53F34268UL, 0x440EA84CUL, 0x07FB118EUL, 0x30584688UL,
		0x0D7F92A1UL, 0xB4C09F8EUL, 0xB503B548UL, 0x1F32A473UL,
		0x656A316EUL, 0xB4A06EE4UL, 0x5BADB7F0UL, 0x4064A01AUL,
		0x3D3F86F5UL, 0x9B2482F4UL, 0x686A6368UL, 0x569021AFUL
	}
};

/* Openssl command To generate public key : Openssl rsa -pubout -in private.pem -out public.pem*/
static const struct PKA_RSAPubkey gPkaRsa4kPublicKey = 
{
	{
		128UL,
		0x7C36D59FUL,0x58B7CD79UL,0xAEBEFD75UL,0xD86B1E68UL,
        0x9471136DUL,0xD3508900UL,0x2FF5339CUL,0x1B782710UL,
        0x07A50A33UL,0x4BA75804UL,0x944865FEUL,0x7E8D027CUL,
        0xB756575EUL,0xAE1F963DUL,0x9BE6D757UL,0xC7B0B09BUL,
        0x21FC16ECUL,0xADDF752DUL,0xA104D242UL,0x22F2C012UL,
        0xAA3C1B52UL,0x2FE8F06CUL,0x6DEE4C12UL,0x0478E621UL,
        0x7F348F04UL,0x3EE4FF19UL,0x185B73A2UL,0x35A564A1UL,
        0xAE1AF4ABUL,0x3BB5C8D4UL,0xB58E29BBUL,0x0ED9044BUL,
        0x1EAAD92DUL,0x4DACCD6FUL,0xDD2ADF84UL,0x8D643F24UL,
        0x32A41D77UL,0x16218997UL,0x4D6148F2UL,0xD785AC13UL,
        0x2A489E84UL,0x36E0E640UL,0x22FA3E5CUL,0xD854EE09UL,
        0x70D5A98FUL,0x2601CA07UL,0x844C53ABUL,0xAF79710CUL,
        0x7629FC2EUL,0xE74E929AUL,0x3322517BUL,0x11B960D6UL,
        0x2145ABABUL,0x0D8E217EUL,0x4A37D6A9UL,0x4BF55C8DUL,
        0x0E983532UL,0xF255E00AUL,0x0B2AE1A3UL,0xFA23ECB8UL,
        0x206746A6UL,0x3A078F9FUL,0x3DD3C59EUL,0x7D3281CCUL,
        0xAAB8955CUL,0x3D8E7F84UL,0x1B63F5F9UL,0x8B7FC89CUL,
        0x048B1E87UL,0xE8045893UL,0x794C4F32UL,0x2A931250UL,
        0xD9FF391BUL,0x8A32ACD2UL,0xB2102085UL,0x0FDE26FDUL,
        0xEE8061CCUL,0x07B0DDDCUL,0x7DDE7C03UL,0xD498FDF4UL,
        0xED6DB0D2UL,0x6A4F3556UL,0xBA5B4A3AUL,0x605E0B06UL,
        0xEFC6C52EUL,0x848CE241UL,0xFB5417CDUL,0x31381D66UL,
        0x953B637EUL,0x0B88C818UL,0x6899BC03UL,0x9AE3B1BBUL,
        0x954093E6UL,0xF9F5865DUL,0x83EFD948UL,0x5DD4CD37UL,
        0x97BA66F6UL,0x291F25E9UL,0x605D5463UL,0x45AAE3B1UL,
        0x548CCE71UL,0x6272EC0CUL,0xA93C5C65UL,0x0E9CCDD8UL,
        0xA146D31CUL,0x71EE205DUL,0xEBFF5CCAUL,0xBA39C3B5UL,
        0x92B7D6F5UL,0xE778694AUL,0xC4A8EF03UL,0xA57AA580UL,
        0x1AE79731UL,0x6ECB0EE0UL,0x22A9D9B7UL,0x6031645BUL,
        0xC8D5FCC6UL,0x25B1C4DBUL,0x17C01149UL,0x5AEC0064UL,
        0x18F20835UL,0x5AF61238UL,0x2A0E3475UL,0x3494519BUL,
        0xB4F3BD20UL,0x67DBE96EUL,0x3F37F0E0UL,0xC2226AFCUL,
	},
	{   1UL,
		0x00010001UL, 
	}
};

/* Openssl command To generate private key : Openssl genrsa -out private.pem*/
static const struct PKA_RSAPrivkey gPkaRsa4kPrivateKey = 
{
	{
		128UL,
		0x7C36D59FUL,0x58B7CD79UL,0xAEBEFD75UL,0xD86B1E68UL,
        0x9471136DUL,0xD3508900UL,0x2FF5339CUL,0x1B782710UL,
        0x07A50A33UL,0x4BA75804UL,0x944865FEUL,0x7E8D027CUL,
        0xB756575EUL,0xAE1F963DUL,0x9BE6D757UL,0xC7B0B09BUL,
        0x21FC16ECUL,0xADDF752DUL,0xA104D242UL,0x22F2C012UL,
        0xAA3C1B52UL,0x2FE8F06CUL,0x6DEE4C12UL,0x0478E621UL,
        0x7F348F04UL,0x3EE4FF19UL,0x185B73A2UL,0x35A564A1UL,
        0xAE1AF4ABUL,0x3BB5C8D4UL,0xB58E29BBUL,0x0ED9044BUL,
        0x1EAAD92DUL,0x4DACCD6FUL,0xDD2ADF84UL,0x8D643F24UL,
        0x32A41D77UL,0x16218997UL,0x4D6148F2UL,0xD785AC13UL,
        0x2A489E84UL,0x36E0E640UL,0x22FA3E5CUL,0xD854EE09UL,
        0x70D5A98FUL,0x2601CA07UL,0x844C53ABUL,0xAF79710CUL,
        0x7629FC2EUL,0xE74E929AUL,0x3322517BUL,0x11B960D6UL,
        0x2145ABABUL,0x0D8E217EUL,0x4A37D6A9UL,0x4BF55C8DUL,
        0x0E983532UL,0xF255E00AUL,0x0B2AE1A3UL,0xFA23ECB8UL,
        0x206746A6UL,0x3A078F9FUL,0x3DD3C59EUL,0x7D3281CCUL,
        0xAAB8955CUL,0x3D8E7F84UL,0x1B63F5F9UL,0x8B7FC89CUL,
        0x048B1E87UL,0xE8045893UL,0x794C4F32UL,0x2A931250UL,
        0xD9FF391BUL,0x8A32ACD2UL,0xB2102085UL,0x0FDE26FDUL,
        0xEE8061CCUL,0x07B0DDDCUL,0x7DDE7C03UL,0xD498FDF4UL,
        0xED6DB0D2UL,0x6A4F3556UL,0xBA5B4A3AUL,0x605E0B06UL,
        0xEFC6C52EUL,0x848CE241UL,0xFB5417CDUL,0x31381D66UL,
        0x953B637EUL,0x0B88C818UL,0x6899BC03UL,0x9AE3B1BBUL,
        0x954093E6UL,0xF9F5865DUL,0x83EFD948UL,0x5DD4CD37UL,
        0x97BA66F6UL,0x291F25E9UL,0x605D5463UL,0x45AAE3B1UL,
        0x548CCE71UL,0x6272EC0CUL,0xA93C5C65UL,0x0E9CCDD8UL,
        0xA146D31CUL,0x71EE205DUL,0xEBFF5CCAUL,0xBA39C3B5UL,
        0x92B7D6F5UL,0xE778694AUL,0xC4A8EF03UL,0xA57AA580UL,
        0x1AE79731UL,0x6ECB0EE0UL,0x22A9D9B7UL,0x6031645BUL,
        0xC8D5FCC6UL,0x25B1C4DBUL,0x17C01149UL,0x5AEC0064UL,
        0x18F20835UL,0x5AF61238UL,0x2A0E3475UL,0x3494519BUL,
        0xB4F3BD20UL,0x67DBE96EUL,0x3F37F0E0UL,0xC2226AFCUL,
	},
	{
		1UL,
		0x00010001UL,
	},
	{
		128UL,
		0x2A5DD271UL,0xF707CCAEUL,0x467FB019UL,0xF173DEF4UL,
        0xE64B66D2UL,0xC009119BUL,0x9AE8D4B5UL,0x9A89482CUL,
        0x3343DA32UL,0x746B0599UL,0x9C5855CEUL,0xEE83F862UL,
        0x958B3D5AUL,0x7F2B38E8UL,0x471E4FFAUL,0x1BB5E027UL,
        0x22EE20DDUL,0x4BB68E49UL,0xC4564836UL,0xA920B6A7UL,
        0x31A8C2F4UL,0x537838E5UL,0xC98B2DCCUL,0xADE355E3UL,
        0x1EF147EBUL,0xFF6AFF67UL,0x77556C46UL,0xBAE5A141UL,
        0x46FC9415UL,0x4E3ABBD9UL,0xB727D9F4UL,0x83BB1FB3UL,
        0x84A8178EUL,0x7700EE39UL,0x322A4772UL,0x607F3561UL,
        0xDD7B4EAAUL,0x4F02E571UL,0xB0B5D61DUL,0x22719E37UL,
        0xD029B717UL,0x3E4DE029UL,0x0217765DUL,0x47A95898UL,
        0x1CE13B18UL,0x66F65E71UL,0x8F400BD4UL,0xEF6946DDUL,
        0x642F1B97UL,0x5389ECDAUL,0x78249749UL,0x08950C16UL,
        0x83C0AB60UL,0x6C5F24D6UL,0x5975477AUL,0x1E7911F5UL,
        0x907297EFUL,0x5E1F334EUL,0x680D944DUL,0xDF662FA5UL,
        0x8599F9AFUL,0xA18ECFD8UL,0xFFEC0D45UL,0x8A862CC8UL,
        0x2CDF3D8FUL,0xF2DB6BCAUL,0x66733AD6UL,0xEE712BD9UL,
        0x705CE93BUL,0xC549D8F8UL,0x1D8D8773UL,0x80F40466UL,
        0x0D79E998UL,0xE5DB9E03UL,0x0AECD779UL,0xDCA7883DUL,
        0xD012562BUL,0xCBC8B63EUL,0x98D07B3CUL,0x4037998EUL,
        0x2305F4E9UL,0xC25BDA01UL,0x70996388UL,0xF900412BUL,
        0xD1D7FAE7UL,0x39706596UL,0x539DFE45UL,0x93D565ACUL,
        0xB7655C67UL,0x29F05F1BUL,0x8A13DEC3UL,0xC0825174UL,
        0xF6426221UL,0x5788C26FUL,0x0455F88EUL,0x72940619UL,
        0xB05DAC90UL,0xC181368DUL,0xD9181140UL,0xE948F5F8UL,
        0xC401425CUL,0x3CA9E396UL,0x95A9A6ECUL,0x0F75E2D5UL,
        0x6A954CC1UL,0x64D0E15EUL,0x5292AA18UL,0x14031911UL,
        0xBB9813A0UL,0x7E46F725UL,0x7DE2238CUL,0xD0F21891UL,
        0x7F15C6D3UL,0x6C34DD4CUL,0x37862A7CUL,0x6CFE4951UL,
        0x23BABA9EUL,0xC9BFB73DUL,0x15594174UL,0x49BBA9B4UL,
        0xF02C2000UL,0xCFB58A62UL,0xDC8C96CAUL,0x27A0C745UL,
        0xA06782BAUL,0x14FE5D38UL,0x8105CF06UL,0x54D8BC0FUL,
	},
	{
		64UL,
		0xBAC22139UL,0x1BA831F4UL,0xB781B015UL,0x082FA82BUL,
        0x0BD0FB16UL,0x75D226A7UL,0xC579CDA7UL,0x43F9E342UL,
        0x21436B6BUL,0xB6D19A74UL,0xEA34FB7EUL,0xC0661497UL,
        0x720FAB03UL,0x229E28E3UL,0x23B58134UL,0x579BD7B9UL,
        0x5E08909EUL,0x219E87ECUL,0x978463FDUL,0x161EA424UL,
        0x0831178DUL,0xFDDBAE42UL,0xE7DA572EUL,0xE56864C8UL,
        0xA989EE95UL,0xD759343BUL,0x4609D675UL,0x929ACFF7UL,
        0x8B194350UL,0x882ADBE9UL,0x7FF1B23BUL,0x9CB48267UL,
        0x280CD09FUL,0xBF60593EUL,0x5CFDE269UL,0x5C4AC054UL,
        0xB514AF1EUL,0xC1AA2394UL,0x3277036AUL,0xDACEEF9AUL,
        0xFF518382UL,0x50A3880FUL,0xD63E1011UL,0xA829F880UL,
        0x65A20598UL,0x3F259CE2UL,0x04C7CB5DUL,0xB0670D96UL,
        0x8C5FA643UL,0x09B4E4E3UL,0x7BD86DBEUL,0xBA69BDA1UL,
        0xA705CB9EUL,0x0B1D6D29UL,0x8463F17CUL,0x530DFE36UL,
        0xFA08C636UL,0x4D23C4F5UL,0x4305BABAUL,0xAEC06517UL,
        0x4B90AF92UL,0xDA2A78D2UL,0x71CB4A29UL,0xEC93D4CCUL,
	},
	{
		64UL,
		0xB4282597UL,0xFA3A44F0UL,0x0500F33BUL,0x888DE615UL,
        0xCC87D8A6UL,0xA6187432UL,0x67599737UL,0x36603CB0UL,
        0x83BC878CUL,0x1B7C164FUL,0x2A9BAD6FUL,0x2980A9EDUL,
        0xDB4DEC2DUL,0x4EC6EA98UL,0x2B47803AUL,0xAD1DD4AAUL,
        0x5877168EUL,0x72784DE1UL,0x40F5890DUL,0x767E9D25UL,
        0x99AB3BD9UL,0xDA09C993UL,0x1B7F4B53UL,0xC5DDF27AUL,
        0x9C5D281CUL,0x29696DCEUL,0xED243DBEUL,0xAA14B939UL,
        0x52CC1C75UL,0x70797838UL,0xFF1AC5AAUL,0x2FDD50C0UL,
        0x7F693387UL,0xF43A8398UL,0xDE0FA41AUL,0x048A5574UL,
        0x4BE909C9UL,0x5FD79704UL,0xB3CE5EB4UL,0xB5696939UL,
        0xBC94BEACUL,0xF9F47C88UL,0xD13B2067UL,0x72786900UL,
        0xDB366DE3UL,0xCA153F91UL,0x3D98EFB0UL,0xA340FC9CUL,
        0x04D70E4AUL,0x0615DF43UL,0x341EA547UL,0x67F3C307UL,
        0xF4FAA943UL,0x5AE1BC8BUL,0xF8E8D3E2UL,0xE84056ACUL,
        0x636CA2D0UL,0x0ACD66E1UL,0xF19DA303UL,0x9D0F664FUL,
        0xD422FC7CUL,0xF650BF40UL,0x74119682UL,0xD2128EDEUL
	},
	{
		64UL,
		0xBF38F8F9UL,0xC558BFF5UL,0x8959F3AFUL,0x9FEE7946UL,
        0xEC3C1EFAUL,0x21F9F185UL,0x6957A3C7UL,0xE2094C76UL,
        0x57CA7E48UL,0xB1C38A31UL,0x12F4230EUL,0xCDE23D62UL,
        0xC1948A75UL,0x558445B1UL,0x9D6F5A26UL,0xF0D72EE8UL,
        0x8B53288AUL,0xDBEDB7B9UL,0x8B20AE5DUL,0xF41C6B79UL,
        0x53DCC253UL,0xA8D8C299UL,0xB71E8214UL,0x0A7D2591UL,
        0x20630BF9UL,0xF329C92FUL,0x83C1387DUL,0x7336C9E8UL,
        0xE4EE238DUL,0xEB1D604BUL,0xD0FAD579UL,0x42A1867EUL,
        0x4476F69EUL,0xE608E9CBUL,0xB25D5DB0UL,0x8FEB259FUL,
        0x9C1A3F30UL,0x6C2EC5CBUL,0x43E1F267UL,0x97F411C8UL,
        0xF9DDDECAUL,0x99F2D239UL,0xE050E26DUL,0x3F198FB6UL,
        0x12014D0BUL,0x8D28506DUL,0x2CF08BD8UL,0x5099D620UL,
        0x279AA1FAUL,0x94278DCAUL,0xBE745DB5UL,0x80C959BCUL,
        0xED01AFA2UL,0x8FBDD6B2UL,0xE23F9445UL,0x37721650UL,
        0xADA0A725UL,0xFE329869UL,0xB70C55F5UL,0xFFB31AA9UL,
        0x1BB1E944UL,0x7CF3EC39UL,0xC6FC6EF8UL,0xA2E8758BUL,
	},
	{
		64UL,
		0xD5AF6161UL,0x65761BE1UL,0x39F29B21UL,0x7C97A61AUL,
        0xF27A207EUL,0xA886188CUL,0x3A8239C9UL,0xFE2B0909UL,
        0x2D942854UL,0xE085B36FUL,0x131FB973UL,0xB5F83F00UL,
        0x927ABF8CUL,0x05B8AF01UL,0xED96E101UL,0xB5736CB6UL,
        0xEDA3BA23UL,0xC5492CD2UL,0x30242627UL,0xA42401B6UL,
        0x392E0640UL,0xE2C41815UL,0xA76FADB6UL,0x773E473BUL,
        0xF0FEC6C3UL,0x5535E96DUL,0x1EB58552UL,0xD3FE2299UL,
        0x7FF24C5FUL,0xA47BD0DAUL,0xAA17EBADUL,0x1883ED7AUL,
        0x70A50E1CUL,0x150F2E90UL,0x8488EA30UL,0x332BBC35UL,
        0xD80A57E0UL,0xF251F1BDUL,0x7B529D5DUL,0x5584A69DUL,
        0x1916087CUL,0xDCCB7697UL,0x63050665UL,0x617E9F74UL,
        0x982CDF52UL,0x1B4524D2UL,0xEE588E4BUL,0x711A4D1AUL,
        0xA46A44D7UL,0x610AF66BUL,0x3993CAB9UL,0xE683E760UL,
        0xD9AF8A8BUL,0xA82F7908UL,0x82C35FA9UL,0x3340802BUL,
        0x670338D1UL,0xB7A74972UL,0x2C8E2271UL,0x4C56F093UL,
        0xC2ECA157UL,0xE87FA523UL,0xC1051E63UL,0x72EED45BUL,
	},
	{
		64UL,
		0x8622EF34UL,0xEBD71DABUL,0x9E875EB1UL,0xBAAD0DC4UL,
        0x715A6494UL,0x445B6E4AUL,0x747FC62FUL,0x258BBD95UL,
        0x738CE271UL,0xDFE5734EUL,0xEC71E5A5UL,0xAB1B2073UL,
        0x82DC8C79UL,0xAD3E85FDUL,0x50DEFF2AUL,0xDCB276AFUL,
        0xE6D1A122UL,0x734DA5F0UL,0xBAB5CC18UL,0xFC665F4AUL,
        0xF3B8BA58UL,0x5C49D09FUL,0x2EFCCE64UL,0xF06552D1UL,
        0xDAC0E077UL,0xE7C5B1FFUL,0x11BCCA75UL,0xC280AB97UL,
        0x92D6AB0AUL,0xC22794ECUL,0xD1085349UL,0x8A44FA3FUL,
        0xFEB02025UL,0x126BE823UL,0x61BDF126UL,0x33AC9DF9UL,
        0xABC1A238UL,0xB267E83DUL,0x67AC4D65UL,0xC1F82A09UL,
        0x3A251E36UL,0x99710BF4UL,0x7EDE75D1UL,0xBB44FA4BUL,
        0x217E369FUL,0x71879993UL,0x707D772AUL,0x8A34A8A2UL,
        0x0D48C484UL,0x1771B6BEUL,0x966C81E6UL,0x50564301UL,
        0xE735B9D8UL,0x2A6915F0UL,0xE89F9795UL,0x222709E8UL,
        0x3E4424EFUL,0x17651CC3UL,0x8BEC519AUL,0x6ED64EC4UL,
        0xB251DD5EUL,0x3B4A252FUL,0x31E91B43UL,0x638739EDUL,
	}
};

static uint8_t gPkaRsaMessage[TEST_PKA_RSA_MSG_SIZE_IN_BYTES] = 
{
	0x33, 0xcd, 0x12, 0x5a, 0x18, 0x6d, 0xc4, 0x49, 0x89, 0xeb, 0x04, 0xc7,
  	0x66, 0x17, 0x05, 0x3f, 0xd7, 0x70, 0x8f, 0x1f, 0x27, 0x51, 0x7c, 0x5c,
  	0xb6, 0x0c, 0xd7, 0xa4, 0xa0, 0xb6, 0xf5, 0xd3, 0xd1, 0x55, 0x0d, 0x71,
  	0x35, 0x0b, 0x51, 0xf8, 0x9f, 0x00, 0x9d, 0xa9, 0x75, 0x92, 0x3c, 0x34,
  	0xc6, 0xba, 0x6d, 0x8d, 0xc6, 0xe5, 0xfe, 0x6d, 0x37, 0x8e, 0xab, 0x5a,
  	0x06, 0x2d, 0xd8, 0x84, 0xb2, 0x70, 0x93, 0xd9, 0xf9, 0xae, 0x5c, 0x18,
  	0x7b, 0x67, 0xd4, 0x36, 0x6f, 0xe3, 0x0f, 0xbc, 0x96, 0xcb, 0x3e, 0x21,
  	0x64, 0xe3, 0x20, 0xaa, 0x0b, 0xcb, 0x11, 0x40, 0xc1, 0xc3, 0x38, 0xf5,
  	0x08, 0x3c, 0x6b, 0xda, 0xb3, 0x9f, 0xb1, 0x42, 0x7d, 0xbb, 0x45, 0xb9,
  	0x4b, 0xe2, 0x03, 0xfd, 0x68, 0xdd, 0xd4, 0xc0, 0xdc, 0x06, 0xb1, 0x28,
  	0xa0, 0x4b, 0x28, 0x98, 0x24, 0x46, 0x49, 0xaa, 0xb5, 0xfe, 0x0f, 0x7a,
  	0x7e, 0x62, 0xe4, 0xac, 0xff, 0xa0, 0xcd, 0xbd, 0xd6, 0x4f, 0xab, 0x29,
  	0x74, 0xb8, 0xac, 0x7c, 0x0b, 0x0f, 0xdf, 0x47, 0xb2, 0x6c, 0xeb, 0x01,
  	0x6b, 0x0f, 0x56, 0xa8, 0x5d, 0xee, 0xbe, 0x8b, 0x00, 0xd4, 0x06, 0xdd,
  	0x3b, 0x62, 0x74, 0xbc, 0x61, 0x12, 0x2f, 0xd1, 0x98, 0xd4, 0xf3, 0x8d,
  	0x84, 0x19, 0x8e, 0xa5, 0xc3, 0x91, 0xab, 0xd6, 0xf1, 0xa4, 0xe4, 0xb9,
  	0x79, 0xf8, 0x96, 0x7a, 0xc0, 0xbc, 0x27, 0x46, 0x73, 0x1a, 0x85, 0xa2,
  	0x6b, 0xe1, 0xb3, 0x5b, 0x7a, 0x4a, 0xc1, 0x24, 0xdd, 0x2b, 0xcb, 0xba,
  	0xed, 0x7e, 0x23, 0x7a, 0x46, 0xef, 0xb1, 0x73, 0x20, 0xbe, 0x38, 0x5b,
  	0x4f, 0x25, 0xcc, 0x21, 0xc0, 0x42, 0x1b, 0x16, 0x11, 0xe3, 0xd2, 0x35,
  	0x89, 0x22, 0x1a, 0xc2, 0x83, 0x9e, 0x9e, 0x41, 0x05, 0x0a, 0xc1, 0xf8,
  	0xb9, 0xf6, 0x64, 0x88, 0xf8, 0x83, 0x30, 0xd0, 0x97, 0xe1, 0x7b, 0xe7,
  	0xe4, 0x76, 0x40, 0x8e, 0x6d, 0x1c, 0xb2, 0x04, 0x8a, 0x72, 0x69, 0x77,
  	0x19, 0x76, 0x77, 0xf7, 0x5b, 0x63, 0x66, 0x90, 0x05, 0x5f, 0x61, 0xc1,
  	0x21, 0xd0, 0xf4, 0xdb, 0x92, 0x45, 0xc5, 0x8e, 0x25, 0xf9, 0x1f, 0x32,
  	0x5e, 0x57, 0x7c, 0xf0, 0x9b, 0xc2, 0x38, 0x7a, 0xaa, 0x81, 0x3a, 0x09,
  	0x87, 0x77, 0x34, 0x14, 0x82, 0xf0, 0x4c, 0x40, 0xa7, 0xea, 0x49, 0x04,
  	0x07, 0x0c, 0xae, 0xf1, 0x1f, 0x9e, 0x38, 0xf9, 0x6e, 0x57, 0x39, 0x39,
  	0x53, 0xee, 0x37, 0x2e, 0xff, 0xd2, 0x27, 0x24, 0x25, 0xd4, 0xa1, 0xcf,
  	0x3a, 0x3c, 0x75, 0x20, 0x1f, 0xe0, 0xa1, 0x0b, 0xf5, 0x58, 0x10, 0xe7,
  	0xb6, 0xb9, 0x44, 0xdc, 0x17, 0xb2, 0xee, 0x74, 0x09, 0xbe, 0x9d, 0xc0,
  	0x50, 0x7c, 0x35, 0x1a, 0xaf, 0xb7, 0x9d, 0xc1, 0xf8, 0x52, 0x2e, 0x1b,
  	0x10, 0x65, 0x1b, 0x26, 0x08, 0x18, 0xcc, 0xc8, 0x07, 0x9a, 0x22, 0x13,
  	0xf6, 0xe1, 0x3c, 0x99, 0x11, 0xf4, 0x9f, 0x48, 0x83, 0x5b, 0x66, 0xfc,
  	0x96, 0x87, 0xd7, 0x90, 0xe6, 0x3f, 0x53, 0x69, 0xb5, 0xb2, 0xdc, 0xc4,
  	0xb3, 0x69, 0xdc, 0x21, 0xe4, 0x68, 0xd0, 0xd5, 0x7a, 0xa1, 0x77, 0xd5,
  	0x0e, 0x53, 0x5b, 0x2d, 0x2b, 0x71, 0x1d, 0x6c, 0xa2, 0x67, 0x07, 0x46,
  	0x55, 0xf9, 0x00, 0xf4, 0xfd, 0x3f, 0xcc, 0x81, 0x69, 0xda, 0x9c, 0x0a,
  	0x21, 0xa5, 0x59, 0xbe, 0x6d, 0x0b, 0x04, 0xd4, 0x9b, 0xfb, 0x3b, 0x24,
  	0x70, 0xe2, 0x9f, 0xc8, 0x7f, 0xc3, 0xc9, 0xc2, 0x44, 0x5a, 0x88, 0x0e,
  	0x80, 0x7e, 0x43, 0xec, 0xdb, 0xb5, 0x2d, 0x99, 0x23, 0xc1, 0xb0, 0x5f,
  	0x3d, 0xa7, 0x47, 0xba, 0x19, 0xbd, 0xfa, 0x4e, 0x2b, 0xbb, 0xf7, 0x05,
  	0x3f, 0xbe, 0x92, 0x08, 0xef, 0xf1, 0x2e, 0xa5
};

/* PKA handle for processing every api */
PKA_Handle			gPkaHandle = NULL;

/*Output test buffer for sha computation */
uint8_t gCryptoShaOutputBuf[TEST_PKA_RSA_MSG_SIZE_IN_BYTES] __attribute__ ((aligned (128)));
uint8_t gCryptoShaHashBufForCompare[APP_SHA512_LENGTH];

int32_t sha512(uint8_t *inputBuf, uint32_t inputLength, uint8_t *output);
void app_getHashFormVerifyOutput(uint32_t *verifyOutput, uint8_t *hash, uint8_t typeOfAlgo);

/* Local test functions */
static void test_pka_rsa_sign_verify_2kBit_key(void *args);
static void test_pka_rsa_sign_verify_4kBit_key(void *args);

void App_printPerformanceResults(uint64_t t1, uint64_t t2);
void App_printTotalPerformanceResults(uint64_t tTotal);
void test_main(void *args)
{
    Drivers_open();
    Board_driversOpen();
    
	PKA_Return_t             status = PKA_RETURN_SUCCESS;

    DebugP_log("[PKA] RSA Signing and verification example started ...\r\n");

	/* Open PKA instance, enable PKA engine, Initialize clocks and Load PKA Fw */
    gPkaHandle = PKA_open(DTHE_PKA_INSTANCE);
    TEST_ASSERT_EQUAL_UINT32(PKA_RETURN_SUCCESS, status);

    RUN_TEST(test_pka_rsa_sign_verify_2kBit_key,  8480, NULL);
	RUN_TEST(test_pka_rsa_sign_verify_4kBit_key,  8481, NULL);
    
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

void test_pka_rsa_sign_verify_2kBit_key(void *args)
{
    PKA_Return_t        status = PKA_RETURN_SUCCESS;
    uint64_t            t1, t2, tTotal;

	DebugP_log("[PKA] RSA Signing and verification with 2k bit started ...\r\n");

    status = sha512(gPkaRsaMessage, sizeof(gPkaRsaMessage), gCryptoShaOutputBuf);
	TEST_ASSERT_EQUAL_UINT32(DTHE_SHA_RETURN_SUCCESS, status);
	
	/* Padding operation */
	Crypto_PKCSPaddingForSign(gCryptoShaOutputBuf, TEST_PKA_RSA_2048_BIT_KEY_SIZE_IN_BYTES, HASH_ALG_SHA2_512, gPkaRsaShaHashWithPadding);
	/* Uint8_t to Uint32_t conversion */
	Crypto_Uint8ToUint32(gPkaRsaShaHashWithPadding, TEST_PKA_RSA_2048_BIT_KEY_SIZE_IN_BYTES, gPkaRsaShaHashWith32BitFormate);
	/* Uint32_t to BigInt conversion */
	Crypto_Uint32ToBigInt(gPkaRsaShaHashWith32BitFormate, TEST_PKA_RSA_2048_BIT_KEY_SIZE_IN_WORDS, gPkaRsaShaHashWithBigIntFormate);

    t1 = ClockP_getTimeUsec();

    /* Openssl Command for Sign: openssl rsautl -sign -inkey privkey.pem -in sha512.dgt -out sha512_signed.dgt */
	status = PKA_RSAPrivate(gPkaHandle, gPkaRsaShaHashWithBigIntFormate, &gPkaRsa2kPrivateKey, gPkaRsaSignOutputResult);
	TEST_ASSERT_EQUAL_UINT32(PKA_RETURN_SUCCESS, status);

    t2 = ClockP_getTimeUsec();
    DebugP_log("RSA Signing Performance :\r\n");
    App_printPerformanceResults(t1, t2);

	tTotal = t2 - t1;

    t1 = ClockP_getTimeUsec();
    /* Openssl Command for Verify: openssl rsautl -verify -pubin -inkey pubkey.pem -in sha512_signed.dgt -out sha512_decrypted.dgt */
    status = PKA_RSAPublic(gPkaHandle, gPkaRsaSignOutputResult, &gPkaRsa2kPublicKey, gPkaRsaVerifyOutputResult);
	TEST_ASSERT_EQUAL_UINT32(PKA_RETURN_SUCCESS, status);

    t2 = ClockP_getTimeUsec();
    DebugP_log("RSA Verification Performance :\r\n");
    App_printPerformanceResults(t1, t2);

    /* Extracting Message hash from verify output */
	app_getHashFormVerifyOutput(gPkaRsaVerifyOutputResult, gCryptoShaHashBufForCompare, HASH_ALG_SHA2_512);

	tTotal = tTotal +(t2 - t1);
    App_printTotalPerformanceResults(tTotal);

    if (0 != memcmp(gCryptoShaOutputBuf, gCryptoShaHashBufForCompare, sizeof(gCryptoShaHashBufForCompare)))
	{
		DebugP_log("[PKA] Verification output did not match expected output\n");
		TEST_ASSERT_EQUAL_UINT32(SystemP_FAILURE, 0);
	}
    return;
}

void test_pka_rsa_sign_verify_4kBit_key(void *args)
{
    PKA_Return_t        status = PKA_RETURN_SUCCESS;
    uint64_t            t1, t2, tTotal;

	DebugP_log("[PKA] RSA Signing and verification with 4k bit started ...\r\n");

    status = sha512(gPkaRsaMessage, sizeof(gPkaRsaMessage), gCryptoShaOutputBuf);
	TEST_ASSERT_EQUAL_UINT32(DTHE_SHA_RETURN_SUCCESS, status);
	
	/* Padding operation */
	Crypto_PKCSPaddingForSign(gCryptoShaOutputBuf, TEST_PKA_RSA_4096_BIT_KEY_SIZE_IN_BYTES, HASH_ALG_SHA2_512, gPkaRsaShaHashWithPadding);
	/* Uint8_t to Uint32_t conversion */
	Crypto_Uint8ToUint32(gPkaRsaShaHashWithPadding, TEST_PKA_RSA_4096_BIT_KEY_SIZE_IN_BYTES, gPkaRsaShaHashWith32BitFormate);
	/* Uint32_t to BigInt conversion */
	Crypto_Uint32ToBigInt(gPkaRsaShaHashWith32BitFormate, TEST_PKA_RSA_4096_BIT_KEY_SIZE_IN_WORDS, gPkaRsaShaHashWithBigIntFormate);

    t1 = ClockP_getTimeUsec();

    /* Openssl Command for Sign: openssl rsautl -sign -inkey privkey.pem -in sha512.dgt -out sha512_signed.dgt */
	status = PKA_RSAPrivate(gPkaHandle, gPkaRsaShaHashWithBigIntFormate, &gPkaRsa4kPrivateKey, gPkaRsaSignOutputResult);
	TEST_ASSERT_EQUAL_UINT32(PKA_RETURN_SUCCESS, status);

    t2 = ClockP_getTimeUsec();
    DebugP_log("RSA Signing Performance :\r\n");
    App_printPerformanceResults(t1, t2);

	tTotal = t2 - t1;

    t1 = ClockP_getTimeUsec();

    /* Openssl Command for Verify: openssl rsautl -verify -pubin -inkey pubkey.pem -in sha512_signed.dgt -out sha512_decrypted.dgt */
    status = PKA_RSAPublic(gPkaHandle, gPkaRsaSignOutputResult, &gPkaRsa4kPublicKey, gPkaRsaVerifyOutputResult);
	TEST_ASSERT_EQUAL_UINT32(PKA_RETURN_SUCCESS, status);

    t2 = ClockP_getTimeUsec();
    DebugP_log("RSA Verification Performance :\r\n");
    App_printPerformanceResults(t1, t2);

    /* Extracting Message hash from verify output */
	app_getHashFormVerifyOutput(gPkaRsaVerifyOutputResult, gCryptoShaHashBufForCompare, HASH_ALG_SHA2_512);

	tTotal = tTotal +(t2 - t1);
    App_printTotalPerformanceResults(tTotal);

    if (0 != memcmp(gCryptoShaOutputBuf, gCryptoShaHashBufForCompare, sizeof(gCryptoShaHashBufForCompare)))
	{
		DebugP_log("[PKA] Verification output did not match expected output\n");
		TEST_ASSERT_EQUAL_UINT32(SystemP_FAILURE, 0);
	}
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

int32_t sha512(uint8_t *inputBuf, uint32_t inputLength, uint8_t *output )
{
    DTHE_SHA_Return_t   status;
    DTHE_Handle         shaHandle;
	DTHE_SHA_Params     shaParams;
    
    /* Opening crypto driver */
    shaHandle = DTHE_open(0);
    DebugP_assert(shaHandle != NULL);

	/* Opening sha driver */
    status = DTHE_SHA_open(shaHandle);
    DebugP_assert(DTHE_SHA_RETURN_SUCCESS == status);

    /* Initialize the SHA Parameters */
    shaParams.algoType          = DTHE_SHA_ALGO_SHA512;
    shaParams.ptrDataBuffer     = (uint32_t*)&inputBuf[0];
    shaParams.dataLenBytes      = inputLength;

    /* Performing DTHE SHA operation */
    status = DTHE_SHA_compute(shaHandle, &shaParams, TRUE);
    DebugP_assert(DTHE_SHA_RETURN_SUCCESS == status);

    /* Closing sha driver */
    status = DTHE_SHA_close(shaHandle);
    DebugP_assert(DTHE_SHA_RETURN_SUCCESS == status);
	memcpy((void *)output, shaParams.digest, APP_SHA512_LENGTH);

    /* Closing DTHE driver */
    if (DTHE_RETURN_SUCCESS == DTHE_close(shaHandle))
    {
        status = DTHE_SHA_RETURN_SUCCESS;
    }
    else
    {
        status = DTHE_SHA_RETURN_FAILURE;
    }
    DebugP_assert(DTHE_SHA_RETURN_SUCCESS == status);

	return (status);
}

void app_getHashFormVerifyOutput(uint32_t *verifyOutput, uint8_t *hash, uint8_t typeOfAlgo)
{
	static uint32_t shaLen;
	uint32_t temp[16];

	switch(typeOfAlgo)
	{
		case 0:
			shaLen = 20/4;
			break;
		case 1:
			shaLen = 32/4;
			break;
		case 2:
			shaLen = 64/4;
			break;
	}

	Crypto_bigIntToUint32(verifyOutput, shaLen, temp);

	Crypto_Uint32ToUint8(temp, (shaLen*4), hash);
	
	return;
}

void App_printTotalPerformanceResults(uint64_t tTotal)
{
    uint64_t throughputInOps = 1000000/tTotal;
    
    DebugP_log("[CRYPTO] Ttotal(us) : %ld \r\n", tTotal);
    DebugP_log("[CRYPTO] Sign and Verify Operations/seconds  : %ld \r\n", throughputInOps);
}

/* Public context crypto dthe and pka accelerators base address */
PKA_Attrs gPKA_Attrs[1] =
{
    {
        /* crypto accelerator base address */
        .caBaseAddr         = CSL_DTHE_PUBLIC_U_BASE,
        /* PKA base address */
        .pkaBaseAddr        = CSL_HSM_PKA_U_BASE,
        /* For checking dthe driver open or close */
        .isOpen             = FALSE,
    },
};

PKA_Config gPkaConfig[1] =
{
    {
        &gPKA_Attrs[0],
    },
};

uint32_t gPkaConfigNum = 1;

/* Public context crypto dthe, aes and sha accelerators base address */
DTHE_Attrs gDTHE_Attrs[1] =
{
    {
        /* crypto accelerator base address */
        .caBaseAddr         = CSL_HSM_DTHE_U_BASE,
        /* AES base address */
        .aesBaseAddr        = CSL_DTHE_PUBLIC_AES_U_BASE,
        /* SHA base address */
        .shaBaseAddr        = CSL_DTHE_PUBLIC_SHA_U_BASE,
        /* For checking dthe driver open or close */
        .isOpen             = FALSE,
    },
};

DTHE_Config gDtheConfig[1]=
{
    {
        &gDTHE_Attrs[0],
    },
};

uint32_t gDtheConfigNum = 1;