/*
 *  Copyright (C) 2022 Texas Instruments Incorporated
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

/**
 *  \defgroup SECURITY_PKA_MODULE APIs for PKA
 *  \ingroup  SECURITY_MODULE
 *
 *  This module contains APIs to program and use the PKA.
 *
 *  @{
 */

/**
 *  \file pka.h
 *
 *  \brief This file contains the prototype of PKA driver APIs
 */

#ifndef PKA_H_
#define PKA_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <security/crypto/pka/eip29t2_firmware.h>
#include <security/crypto/pka/hw_include/cslr_cp_ace.h>
#include <kernel/dpl/SystemP.h>
#include <security/crypto/crypto_util.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
 * \brief
 *  PKA Driver Error code
 *
 * \details
 *  The enumeration describes all the possible return and error codes which
 *  the PKA Driver can return
 */
typedef enum PKA_Return_e
{
    PKA_RETURN_SUCCESS                  = 0xCEF6A572U, /*!< Success/pass return code */
    PKA_RETURN_FAILURE                  = 0xD20341DDU, /*!< General or unspecified failure/error */
}PKA_Return_t;

/** \brief Handle to the PKA driver */
typedef void *PKA_Handle;

/** Max size of bigint in words - for RSA */
#define PKA_BIGINT_MAX                          (130U)

/** Max size of bigint in words - for ECDSA */
#define PKA_EC_BIGINT_MAX                       (18U)

/**
 * Maximum length of a big integer used in EC crypto in bytes, enough to
 * accommodate 521-bit prime curves
 */
#define PKA_EC_PARAM_MAXLEN						(68U)

/**
 * Length of a biginteger array in words, the +1 is for the size
 */
#define PKA_BIGINT_LEN(bytelen)                 (((bytelen) / 4U) + 1U)

/** RSA KEY E maximun length */
#define PKA_RSA_KEY_E_MAXLEN                    (8U)
/** RSA KEY N maximun length */
#define PKA_RSA_KEY_N_MAXLEN                    (520U)
/** RSA KEY PQ maximun length */
#define PKA_RSA_KEY_PQ_MAXLEN                   ((PKA_RSA_KEY_N_MAXLEN / 2U) + 4U)
/** RSA KEY SIG maximun length */
#define PKA_RSA_SIG_MAXLEN                      PKA_RSA_KEY_N_MAXLEN

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 * \brief RSA public key. All values are in biginteger format (size followed
 *        by word value array, least significant word first)
 *
 * \param n RSA modulus (n)
 * \param e Public exponent (e)
 */
struct PKA_RSAPubkey {
	/** RSA modulus (n) */
	uint32_t	n[PKA_BIGINT_LEN(PKA_RSA_KEY_N_MAXLEN)];
	/** Public exponent (e) */
	uint32_t	e[PKA_BIGINT_LEN(PKA_RSA_KEY_E_MAXLEN)];
};

/**
 * \brief RSA private key. All values are in biginteger format (size followed
 *        by word value array, least significant word first)
 *
 * \param n RSA modulus (n)
 * \param e Public exponent (e)
 * \param d Private exponent (d)
 * \param p Prime 1 (p)
 * \param q Prime 2 (q)
 * \param dp d mod (p-1)
 * \param dq d mod (q-1)
 * \param coefficient crt coefficient q^(-1) mod p
 */
struct PKA_RSAPrivkey {
	/** RSA modulus (n) */
	uint32_t	n[PKA_BIGINT_LEN(PKA_RSA_KEY_N_MAXLEN)];
	/** Public exponent (e) */
	uint32_t	e[PKA_BIGINT_LEN(PKA_RSA_KEY_E_MAXLEN)];
	/** Private exponent (d) */
	uint32_t	d[PKA_BIGINT_LEN(PKA_RSA_KEY_N_MAXLEN)];
	/** Prime 1 (p) */
	uint32_t	p[PKA_BIGINT_LEN(PKA_RSA_KEY_PQ_MAXLEN)];
	/** Prime 2 (q) */
	uint32_t	q[PKA_BIGINT_LEN(PKA_RSA_KEY_PQ_MAXLEN)];
	/** d mod (p-1) */
	uint32_t	dp[PKA_BIGINT_LEN(PKA_RSA_KEY_PQ_MAXLEN)];
	/** d mod (q-1) */
	uint32_t	dq[PKA_BIGINT_LEN(PKA_RSA_KEY_PQ_MAXLEN)];
	/** crt coefficient q^(-1) mod p */
	uint32_t	coefficient[PKA_BIGINT_LEN(PKA_RSA_KEY_PQ_MAXLEN)];
};

/**
 * \brief EC Point, also the public key
 *
 * \param x x-coordinate
 * \param y y-coordinate
 */
struct PKA_ECPoint {
	/** x-coordinate */
	uint32_t	x[PKA_BIGINT_LEN(PKA_EC_PARAM_MAXLEN)];
	/** y-coordinate */
	uint32_t	y[PKA_BIGINT_LEN(PKA_EC_PARAM_MAXLEN)];
};

/**
 * \brief EC prime curve parameters
 *
 * \param prime Prime number for the group
 * \param order Order of the group
 * \param a "a" parameter in the equation x^3 + ax + b = y
 * \param b "b" parameter in the equation x^3 + ax + b = y
 * \param g Generator point on the Elliptic curve
 */
struct PKA_ECPrimeCurveP {
	/** Prime number for the group */
	uint32_t		prime[PKA_BIGINT_LEN(PKA_EC_PARAM_MAXLEN)];
	/** Order of the group */
	uint32_t		order[PKA_BIGINT_LEN(PKA_EC_PARAM_MAXLEN)];
	/** "a" parameter in the equation x^3 + ax + b = y */
	uint32_t		a[PKA_BIGINT_LEN(PKA_EC_PARAM_MAXLEN)];
	/** "b" parameter in the equation x^3 + ax + b = y */
	uint32_t		b[PKA_BIGINT_LEN(PKA_EC_PARAM_MAXLEN)];
	/** Generator point on the Elliptic curve */
	struct PKA_ECPoint g;
};

/**
 * \brief ECDSA signature
 *
 * \param r "r" value in ECDSA signature
 * \param s "s" value in ECDSA signature
 */
struct PKA_ECDSASig {
	/** "r" value in ECDSA signature */
	uint32_t	r[PKA_BIGINT_LEN(PKA_EC_PARAM_MAXLEN)];
	/** "s" value in ECDSA signature */
	uint32_t	s[PKA_BIGINT_LEN(PKA_EC_PARAM_MAXLEN)];
};

/** \brief PKA attributes */
typedef struct
{
    /*
     * SOC configuration
     */
	uint32_t				caBaseAddr;
	/**< Crypto Accelerator Base Adders*/
    uint32_t                pkaBaseAddr;
    /**< PKA Base address */
	uint32_t                isOpen;
    /**< Flag to indicate whether the instance is opened already */
} PKA_Attrs;

/** \brief PKA driver context */
typedef struct
{
    PKA_Attrs             *attrs;
    /**< Driver params passed during open */
} PKA_Config;

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/** \brief Externally defined driver configuration array */
extern PKA_Config            gPkaConfig[];
/** \brief Externally defined driver configuration Num */
extern uint32_t             gPkaConfigNum;

/* ========================================================================== */
/*                              Function Definitions                          */
/* ========================================================================== */

/**
 * \brief Function to Open PKA instance, enable PKA engine, Initialize clocks and Load PKA Fw
 *
 * \return        A #PKA_Handle on success or a NULL on an error or if it has been
 *				  opened already
 */
PKA_Handle PKA_open(uint32_t index);

/**
 *  \brief  Function to close a PKA module specified by the PKA handle
 *
 *  \param  handle  #PKA_Handle returned from #PKA_open()
 */
PKA_Return_t PKA_close(PKA_Handle handle);

/**
 * \brief This Function performs Decryption or Signing operations
 *
 * \param  handle  #PKA_Handle returned from #PKA_open()
 * 
 * \param m       m value in bigint format.
 * \param k       RSA private key
 *
 * \param result  Result of the operation in bigint format. caller must allocate
 *                memory size of (2 * sizeof(p)) for the result.
 *
 * \return        #PKA_RETURN_SUCCESS if requested operation completed.
 *                #PKA_RETURN_FAILURE if requested operation not completed.
 */
PKA_Return_t PKA_RSAPrivate(PKA_Handle handle, const uint32_t m[PKA_BIGINT_MAX], const struct PKA_RSAPrivkey *k, uint32_t result[PKA_BIGINT_MAX]);

/**
 * \brief This Function performs Encryption or Verification operations
 *
 * \param  handle  #PKA_Handle returned from #PKA_open()
 * 
 * \param m       m value in bigint format.
 * \param k       RSA public key
 * \param result  Result of the operation in bigint format. caller must allocate
 *                the same memory as s and n for this array.
 *
 * \return        #PKA_RETURN_SUCCESS if requested operation completed.
 *                #PKA_RETURN_FAILURE if requested operation not completed.
 */
PKA_Return_t PKA_RSAPublic(PKA_Handle handle, const uint32_t m[PKA_BIGINT_MAX], const struct PKA_RSAPubkey *k, uint32_t result[PKA_BIGINT_MAX]);

/**
 * \brief ECDSA sign primitive function
 *
 * \param  handle  #PKA_Handle returned from #PKA_open()
 * 
 * \param cp      EC curve parameters
 * \param priv    EC private key
 * \param k       Random number for each signing
 * \param h       Hash value of message to sign in bigint format
 * \param sig     ECDSA Signature - 'r' and 's' values
 * \return        #PKA_RETURN_SUCCESS if requested operation completed.
 *                #PKA_RETURN_FAILURE if requested operation not completed.
 */
PKA_Return_t PKA_ECDSASign(PKA_Handle handle, 
                    const struct PKA_ECPrimeCurveP *cp, 
                    const uint32_t priv[PKA_EC_BIGINT_MAX], 
                    const uint32_t k[PKA_EC_BIGINT_MAX], 
                    const uint32_t h[PKA_EC_BIGINT_MAX], 
                    struct PKA_ECDSASig *sig);

/**
 * \brief ECDSA verify primitive function
 *
 * \param  handle  #PKA_Handle returned from #PKA_open()
 * 
 * \param cp      EC curve parameters
 * \param pub     EC Public key
 * \param sig     ECDSA Signature - 'r' & 's' value in bigint format
 * \param h       Hash value of message to verify in bigint format
 *
 * \return        #PKA_RETURN_SUCCESS if requested operation completed.
 *                #PKA_RETURN_FAILURE if requested operation not completed.
 */
PKA_Return_t PKA_ECDSAVerify(PKA_Handle handle,
                        const struct PKA_ECPrimeCurveP *cp, 
                        const struct PKA_ECPoint *pub, 
                        const struct PKA_ECDSASig *sig, 
                        const uint32_t h[PKA_EC_BIGINT_MAX]);

#ifdef __cplusplus
}
#endif

#endif /* PKA_H_ */

/** @} */
