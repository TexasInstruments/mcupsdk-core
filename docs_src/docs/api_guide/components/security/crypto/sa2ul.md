# SA2UL {#SECURITY_SA2UL_MODULE_PAGE}
[TOC]

The SA2_UL subsystem is designed to provide a generic cryptographic acceleration for different use cases such as secure boot, secure content, key exchange etc.

## Features Supported In Hardware

- Compatible with ARM Trustzone operations
- Support non-secure authentication
- Crypto function library for software acceleration
    - <b>AES</b> operation
    - <b>SHA2-224, 256, 512 operation</b>
- Block data encryption supported via hardware cores
    - <b>AES</b> with 128, 192 and 256 bits key support
- Data encryption modes support
    - <b>ECB</b> (Electronic code book)
    - <b>CBC</b> (Cipher block chaining)
    - <b>CFB</b> (Cipher feedback)
    - <b>OFB</b> (Output feedback)
    - <b>F8</b> (3GPP confidentiality and integrity algorithms)
    - <b>CTR</b> (Counter)
    - <b>CBC-MAC</b> (Cipher block chaining - Message authentication code)
    - <b>CCM</b> (Counter with CBC-MAC)
    - <b>GCM</b> (Galois counter mode)
    - <b>GMAC</b> (Galois Message Authentication Code)
- <b>Public Key accelerator</b> with Elliptic Curve Cryptography (ECC)
    - High performance Public Key Accelerator (PKA) for large vector math operation
- Supports modulus size up to 4096-bits
    - A dual <b>LNME</b> module for Montgomery multiplication and exponentiation
    - Prime field GF(p) <b>ECC</b> point addition, doubling and multiplication for all NIST (FIPS 186-3) recommended prime curves
    - Binary field GF(2m) ECC point addition, doubling and multiplication.
    - Support public key computations such as <b>RSA</b> (4k).
    - Single call operations for RSA up to 4k and ECC (Prime NIST curves) signing and verification
    - Supports Brainpool, NIST and Curve25519 ECC curves
    - Authentication supported via following hardware cores
    - <b>SHA2-224</b>
    - <b>SHA2-256</b>
    - <b>SHA2-512</b>
- Keyed <b>HMAC</b> operation via hardware core
    - <b>HMAC</b> using <b>SHA2-224</b>, <b>SHA2-256</b> and <b>SHA2-512</b>
    - Support for truncated authentication tag
- True Random number generator (<b>TRNG</b>)
    - True (not pseudo) random number generator
    - FIPS 140-1 compliant
    - Non-deterministic noise source for generating keys, IV etc
- Context cache module to auto fetch security context
    - Cache limited number (up to 4) contexts for low cost
    - Auto-fetch security context based on current state of engine
    - Option to allow storage of security context within engine for high performance connections
- Auto evict security context based on unavailability of space within context cache
- Fully pipelined engines for parallel processing
- <b>DMA</b> support for Cryptographic acceleration

## Features Supported In Driver

- Encryption and authentication
- Crypto function library for Security acceleration
    - <b>S</b>ecure <b>h</b>ash <b>a</b>lgorithms
        - <b>SHA256</b>, <b>SHA512</b>
    - <b>H</b>ash-based <b>m</b>essage <b>a</b>uthentication <b>c</b>ode
        - <b>HMAC SHA-256</b>, <b>HMAC SHA-512</b>, <b>HMAC SHA1</b>
    - <b>A</b>dvanced <b>e</b>ncryption <b>s</b>tandard
        - <b>AES-CBC</b>(128/256)(<b>C</b>ipher <b>B</b>lock <b>C</b>haining)
        - <b>AES-ECB</b>(128/256)(<b>E</b>lectronic <b>C</b>ode <b>B</b>ook)
        - <b>AES-CMAC</b>(128/256)(<b>C</b>ipher-based <b>M</b>essage <b>A</b>uthentication <b>C</b>ode)
- Supports random number generator(<b>RNG</b>)
    - Keys and initialization values (<b>IV</b>s) for encryption
    - Keys for <b>keyed MAC</b> algorithms
    - Private keys for <b>digital signature algorithms</b>
    - Values to be used in <b>entity authentication mechanisms</b>
    - <b>PIN</b> and <b>password</b> generation
- Supports public-key accelerator(<b>PKA</b>)
    - Supports up to 4K bit key
    - Supports Raw operations
    - Supports PKA <b>RSA encryption and decryption</b>
    - Supports <b>RSA signing and verification operations</b>
    - Supports <b>ECDSA signing and verification operations</b>
        - <b>P-256 and P-384 curves</b>

## Block diagram

\note This block diagram referred from Sa2ul functional specification.

\imageStyle{security_crypto_block_diagram.png,width:60%}
\image html security_crypto_block_diagram.png "Sa2ul block diagram"

### PSI-L
- The PSI-L(Packet Streaming Interface Link) is used to transfer words of packet data and control information between two peer entities via direct connection. There are credits used for flow control to guarantee that blocking does not occur between threads. Multiple packet transfers can be ongoing simultaneously across a PSI-L interface each on a separate logical thread but all sharing a single data path through time division multiplexing. Each packet which is transferred is accompanied by a destination thread ID which indicates the logical destination thread to which the packet is being sent.

- SA2_UL accepts packets from <b>Streaming interface (PSI-L)</b> port with 8 ingress threads where each thread is a <b>UDMA Tx channel</b>. Each packet destined to SA2_UL must be prefixed with three extended packet info words (software words) that hold information about security context that is required to uniquely identify security connection and associated security parameters. The coherency is expected to be maintained by the <b>UDMA-P</b>.

### Context Cache
- Context cache fetch process involves two primary levels of security checks for the fetch to be successful. The first level of check is for SA2_UL to translate the security attributes from the incoming packets to the attributes on the <b>VBUSP DMA</b> host interface. The firewall IP near the external memory will then check these attributes before allowing access. The second level check is done after SA2_UL fetches and caches the context. SA2_UL compares the security attributes from the packets and the attributes stored inside the security context. If the attributes satisfy the check requirements, the context is used by the encryption and/or authentication engine, otherwise SA2_UL does not process the packet (packet will pass through) and a security exception will be generated.

### Encryption Engine
- It carries out the task of encrypting/decrypting payload from desired offset using hardware encryption cryptographic cores. Encryption engine has AES core, 3DES core and Galois multiplier core which is operated in conjunction with MCE (mode control engine). Mode control engine implements various encryption modes like ECB, CBC, CTR, OFB, GCM etc.

### Authentication Engine
- It caters the requirement of providing integrity protection. Authentication engine is equipped with SHA1 core, MD5 core, SHA2 core (up to 512 bits) to support keyed (HMAC) and non-keyed hash calculation.

### ECC Aggregator
- To increase functional and system reliability the memories (for example, FIFOs, queues, SRAMs and others) in many device modules and subsystems are protected by error correcting code (ECC). This is accomplished through an ECC aggregator.

### CBASS
- CBASS is the interconnection technique used to communicate between modules and subsystems in the device for any memory map accesses.

### Context RAM
- Each engine/subsystem has a <b>Context RAM</b> to store the control information pertaining to the logical connection. The <b>context RAM</b> holds the information like Keys, IV, partial data etc. for each active context. The SA2_UL subsystem provides the option to store 4 contexts on-chip. <b>Context RAM</b> is coupled with <b>Context Cache</b> module to fetch the context information from external memory to populate the active context on real-time demand basis.

### Packet RAM
- Packet RAM used to store Chunks of packets for processing sa2ul operations normally size of packet RAM is 2 banks (1KB or 2 chunks).

### TRNG
- TRNG (True Random number generator) used to create initialization vector required for certain encryption modes. The random numbers are accessible to the host in four 32-bit registers allowing 128-bit random number to be read with a single burst read.

### PKA
- The PKA(Public Key Accelerator) module provides a high-performance public key engine to accelerate the large vector math processing that is required for Public Key computations. It also includes hw acceleration for Elliptic Curve Cryptography such as binary field ECC point addition, inversion, multiplication and ECC prime field point addition, inversion and multiplication.

\section Dma_flow DMA Flow
- Sa2ul Uses PKTDMA, the PKTDMA module supports the transmission and reception of various packet types. The PKTDMA is architected to facilitate the segmentation and reassembly of KSLC DMA data structure compliant packets to/from smaller data blocks that are natively compatible with the specific requirements of each connected peripheral.

\imageStyle{sa2ul_dma_flow.png,width:60%}
\image html sa2ul_dma_flow.png "sa2ul dma flow"

## SysConfig Features
- Select Crypto instance, it automatically create sa2ul and Dma with PKTDMA instances.
- We can not create more then one instances due to Dma limitations.

## Features NOT Supported
- Context fetch using the MMR is no longer supported to ensure data security.

## Usage Overview
### Dependencies
- #SA2UL_ContextParams should be populated for intended cryptographic operation. Please see this \ref Example_usage for AES and SHA.
- DMA initialization is required.
- DMA Tx and Rx Channels need to be initialized. Please refer \ref Dma_flow.

## SA2UL API's Supported

- #SA2UL_init()                 : Initialize the SA2UL module.
- #SA2UL_Params_init()          : Initialize a #SA2UL_Params structure with default values. Then change the parameters from non-default values as needed.
- #SA2UL_open()                 : Open an instance of the SA2UL module, enable Sa2ul hw engines and initialize dma.
- #SA2UL_contextAlloc()         : Function to configure secure context.
- #SA2UL_contextFree()          : Function to free secure context configuration.
- #SA2UL_contextProcess()       : Function to transfer and receive data buffer.
- #SA2UL_rngSetup()             : setup the SA2UL RNG module.
- #SA2UL_rngRead()              : Read random numbers into the output buffer.
- #SA2UL_close()                : De-initialize the SA2UL instance, disable Sa2ul hw engines and de-initialize dma.
- #SA2UL_deinit()               : De-Initialize the SA2UL module.

### API Sequence for CBC abd ECB Algorithms

This sequence performs Encryption and decryption operations for AES-CBC/ECB algorithms.
Supported key lengths are 128 and 256 bit.

- #Crypto_open()                : Initializes a Crypto context and Open an instance of the SA2UL module, enable Sa2ul hw engines and initialize dma.
- #SA2UL_contextAlloc()         : Function to configure secure context.
- #SA2UL_contextProcess()       : Function to transfer and receive data buffer.
- #SA2UL_contextFree()          : Function to free secure context configuration.
- #Crypto_close()               : De-initialize the SA2UL instance, disable Sa2ul hw engines and de-initialize dma and clears context.

### API Sequence for SHA Algorithms

This sequence performs SHA-512 and SHA-256.

- #Crypto_open()                : Initializes a Crypto context and Open an instance of the SA2UL module, enable Sa2ul hw engines and initialize dma.
- #SA2UL_contextAlloc()         : Function to configure secure context.
- #SA2UL_contextProcess()       : Function to transfer and receive data buffer.
- #SA2UL_contextFree()          : Function to free secure context configuration.
- #Crypto_close()               : De-initialize the SA2UL instance, disable Sa2ul hw engines and de-initialize dma and clears context.

### API Sequence for HMAC-SHA Algorithms

This sequence performs HMAC SHA-512, SHA-256 and SHA1.

- #Crypto_open()                : Initializes a Crypto context and Open an instance of the SA2UL module, enable Sa2ul hw engines and initialize dma.
- #Crypto_hmacSha()             : This function updates oPad & iPad for HMAC calculation.
- #SA2UL_contextAlloc()         : Function to configure secure context.
- #SA2UL_contextProcess()       : Function to transfer and receive data buffer.
- #SA2UL_contextFree()          : Function to free secure context configuration.
- #Crypto_close()               : De-initialize the SA2UL instance, disable Sa2ul hw engines and de-initialize dma and clears context.

### API Sequence for AES-CMAC Algorithms

This sequence performs AES CMAC-128 and CMAC-256.

- #Crypto_open()                : Initializes a Crypto context and Open an instance of the SA2UL module, enable Sa2ul hw engines and initialize dma.
- #Crypto_cmacGenSubKeys()      : This function Generate Sub keys for CMAC calculation.
- #SA2UL_contextAlloc()         : Function to configure secure context.
- #SA2UL_contextProcess()       : Function to transfer and receive data buffer.
- #SA2UL_contextFree()          : Function to free secure context configuration.
- #Crypto_close()               : De-initialize the SA2UL instance, disable Sa2ul hw engines and de-initialize dma and clears context.

### API Sequence for RNG (Random number generation)

This sequence to get RNG (Random number generation).

- #Crypto_open()                : Initializes a Crypto context and Open an instance of the SA2UL module, enable Sa2ul hw engines and initialize dma.
- #SA2UL_rngSetup()             : setup the SA2UL RNG module.
- #SA2UL_rngRead()              : Read random numbers into the output buffer.
- #Crypto_close()               : De-initialize the SA2UL instance, disable Sa2ul hw engines and de-initialize dma and clears context.

### Opening the SA2UL Driver

-   The application can open a SA2UL instance by calling #Crypto_open(). Please note that opening SA2UL driver is taken care by the SysConfig generated code.
This function takes an index into the gSa2ulConfig[] array, and the SA2UL parameters data
structure. The SA2UL instance is specified by the index of the SA2UL in gSa2ulConfig[].
Calling #Crypto_open() second time with the same index previously passed to #Crypto_open() will result in an error.
Re-use the index if the instance is closed via #Crypto_close().

### Ultra lite Security Accelerator
-   Set the ctx parameters using #SA2UL_ContextParams structure, all necessary parameters set to #SA2UL_ContextParams variables and allocate sa2ul context by calling #SA2UL_contextAlloc(), to get expected result by calling #SA2UL_contextProcess() with input buffer and output buffer as parameters, final output will stored in output buffer. Close sa2ul context by calling #SA2UL_contextFree().

### AES (Advanced Encryption Standard)
-   For enabling Sa2ul Aes engine use #Crypto_open() and it also do initialize Crypto context, set the ctx parameters using #SA2UL_ContextParams structure, all necessary parameters set to #SA2UL_ContextParams variables and allocate sa2ul context by calling #SA2UL_contextAlloc(), to get expected result by calling #SA2UL_contextProcess() with input buffer and output buffer as parameters, final output will stored in output buffer. Close sa2ul context by calling #SA2UL_contextFree(). For closing sa2ul Aes engine use #Crypto_close().

### SHA (Secure Hash Algorithm)
-   For enabling Sa2ul Sha engine use #Crypto_open() and it also do initialize Crypto context, set the ctx parameters using #SA2UL_ContextParams structure, all necessary parameters set to #SA2UL_ContextParams variables and allocate sa2ul context by calling #SA2UL_contextAlloc(), to get expected result by calling #SA2UL_contextProcess() with input buffer and output buffer as parameters, final output will stored in gSa2ulCtxObj(computedHash) buffer. Close sa2ul context by calling #SA2UL_contextFree(). For closing sa2ul Sha engine use #Crypto_close().

### HMAC-SHA (Keyed-Hash Message Authentication Code Secure Hash Algorithm)
-   For enabling Sa2ul Sha engine use #Crypto_open() and it also do initialize Crypto context, set the ctx parameters using #SA2UL_ContextParams structure, all necessary parameters set to #SA2UL_ContextParams variables and allocate sa2ul context by calling #SA2UL_contextAlloc(), for ipad and opad call #Crypto_hmacSha(), to get expected result by calling #SA2UL_contextProcess() with input buffer and output buffer as parameters, final output will stored in computedHash buffer (present in Sa2ul Ctx Obj). Close sa2ul context by calling #SA2UL_contextFree(). For closing sa2ul Sha engine use #Crypto_close().

### AES-CMAC (Cipher-based Message Authentication Code)
-   For enabling Sa2ul Aes engine use #Crypto_open() and it also do initialize Crypto context, set the ctx parameters using #SA2UL_ContextParams structure, all necessary parameters set to #SA2UL_ContextParams variables and allocate sa2ul context by calling #SA2UL_contextAlloc(), for key1 and key2 call #Crypto_cmacGenSubKeys(), to get expected result by calling #SA2UL_contextProcess() with input buffer and output buffer as parameters, final output will stored in output buffer. Close sa2ul context by calling #SA2UL_contextFree(). For closing sa2ul Sha engine use #Crypto_close().

### RNG (Random number generation)
-   For enabling SA2UL RNG use #Crypto_open(), Setup rng module by calling #SA2UL_rngSetup(), to get 128 bit random number by calling #SA2UL_rngRead() with output buffer as parameters, final 128 bit random number will stored in output buffer. For disabling SA2UL RNG use #Crypto_close().

### Closing the SA2UL Driver

- The application can Close a SA2UL instance by calling #Crypto_close(), it close the sa2ul instance passed by the user.

\section Example_usage Example Usage

Include the below file to access the SA2UL SHA APIs
\snippet sa2ul_sha_sample.c include

sa2ul_sha Example
\snippet sa2ul_sha_sample.c sa2ulsha

Include the below file to access the SA2UL AES CBC APIs
\snippet sa2ul_aes_cbc_sample.c include1

sa2ul_aes_cbc Example
\snippet sa2ul_aes_cbc_sample.c sa2ulAesCbc

## Directory Structure
Given below is a overview of the directory structure to help you navigate the Crypto structure.
Folder/Files                                |   Description
--------------------------------------------|------------------
source/security/crypto                      | Contains Crypto driver files
source/security/crypto/sa2ul                | Contains Sa2ul driver files
source/security/crypto/pka                  | Contains Pka driver files
examples/security                           | Contains all Security examples

## API

- \ref SECURITY_SA2UL_MODULE