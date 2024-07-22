# DTHE {#DRIVERS_DTHE_PAGE}
[TOC]

DTHE stands for Data Transform and Hashing Engine. This module is a wrapper
on top of the Crypto IP with some additional capability, including CRC and Checksum

### Features Supported In Hardware
DTHE provides the following features:

* Symmetric encryption and decryption
	* AES: 128, 192, and 256 bit keys
	* Cipher modes ECB, CTR, CBC, GCM, CCM, F9, F8, XTS, CFB, ICM, CTR,
      CBC-MAC, CMAC based on AES
* Asymmetric cryptography
	* High performance PKA (public key engine) for large vector math/modulus operation
	* RSA2048, RSA3092, RSA4096
	* ECC (accelerated using PKA module)
* Hashing
	* MD5, SHA-1, SHA-224, SHA-256, SHA-384 and SHA-512
	* HMAC-SHA256, HMAC-SHA512  keyed hashing
* Random number generator
	* 128 bit True random number generator
    * 128 bit Deterministic random bit generator

- This module wraps following IP inside
    - <b>EIP29T</b> : PKA accelerator
    - <b>EIP57T</b> : SHA/MD5 accelerator
    - <b>EIP36T</b> : AES accelerator
    - <b>EIP76T</b> : True Random Number Generation

Apart from this the module holds HW accelerator for <b>CRC and Checksum</b>.

The IP supports the following features:

*  Supports these CRC functions:
	- Bisync, Modbus, USB, ANSI X3.28, many others; also known as CRC-16 and CRC-16-ANSI :
(x^16+x^15+x^2+1)
	- CRC16- /X.25 with Polynomial 0x1021 : (x^16+x^12+x^5+1)
	- CRC32-IEEE/MPEG2/Hamming with Polynomial 0x4C11DB7 :
(x^32+x^26+x^23+x^22+x^16+x^12+x^11+x^10+x^8+x^7+x^5+x^4+x^2 + x+1)
 	- CRC32-G.Hn/CRC32C with Polynomial 0x1EDC6F41:
(x^32+x^28+x^27+x^26+x^25+x^23+x^22+x^20+x^19+x^18+x^14+x^13+ x^11+x^10+x^9+x^8+x^6+1)
*  Supports TCP CheckSum (CSUM)

### Features Supported In Driver
\note True Random Number Generation (TRNG) and PKA (Public Key Accelerator) are not supported from R5F on HS-SE/HS-FS devices as these are secure assets and are locked for HSM.

\note TRNG and PKA are single context engines and hence are secure assets on HS-SE/HS-FS devices.

\cond SOC_AM273X || SOC_AWR294X

\note On GP devices, AES, PKA and TRNG are not supported. SHA examples will work as is.
\endcond

\cond SOC_AM273X || SOC_AWR294X || SOC_AM263X || SOC_AM263PX || SOC_AM261X
- \subpage DRIVERS_DTHE_SHA_PAGE
	- <b>S</b>ecure <b>h</b>ash <b>a</b>lgorithms)
        - <b>SHA256</b>, <b>SHA512</b>
	- <b>H</b>ash-based <b>m</b>essage <b>a</b>uthentication <b>c</b>ode
        - <b>HMAC SHA-256</b>, <b>HMAC SHA-512</b>
- \subpage DRIVERS_DTHE_AES_PAGE
	- <b>A</b>dvanced <b>e</b>ncryption <b>s</b>tandard
        - <b>AES-CBC</b>(128/192/256)(<b>C</b>ipher <b>B</b>lock <b>C</b>haining)
        - <b>AES-ECB</b>(128/192/256)(<b>E</b>lectronic <b>C</b>ode <b>B</b>ook)
        - <b>AES-CTR</b>(128/192/256)(<b>C</b>oun<b>T</b>e<b>R</b>)
        - <b>AES-CFB</b>(128/192/256)(<b>C</b>ipher <b>F</b>eed<b>B</b>ack)
        - <b>AES-CMAC</b>(128/256)(<b>C</b>ipher-based <b>M</b>essage <b>A</b>uthentication <b>C</b>ode)
\endcond
### Block diagram

\imageStyle{dthe_block_diagram.png,width:60%}
\image html dthe_block_diagram.png "DTHE block diagram"

### Functional description
- #DTHE_open() checks if the #DTHE_Handle is already open and if it is not open it continues to initialize the #DTHE_Handle with the desired SOC configuration.
- The #DTHE_Handle is further used by AES and SHA drivers.
- #DTHE_close() takes #DTHE_Handle as parameter and proceeds to check if handle is not NULL and whether handle is open, if both conditions are satisfied then after changing #DTHE_Handle to NULL, #SystemP_SUCCESS is returned.



### DTHE API's Supported
- #DTHE_init(): Initializes the DTHE module.
- #DTHE_deinit(): This function de-initializes the DTHE.
- #DTHE_open(): Function to open DTHE instance, enable DTHE engine.
- #DTHE_close(): Function to close a DTHE module specified by the DTHE handle.

### API
- \ref SECURITY_DTHE_MODULE
