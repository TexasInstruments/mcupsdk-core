# PKA {#SECURITY_PKA_MODULE_PAGE}
[TOC]
The PKA module provides a high-performance public key engine to accelerate the large vector math processing that is required for Public Key computations. It also includes hw acceleration for Elliptic Curve Cryptography such as binary field ECC point addition, inversion, multiplication and ECC prime field point addition, inversion and multiplication.The ECC prime field engine GF(p) supports all NIST (FIPS 186-3) recommended prime curves up to 521 bit key length.

## Features Supported In Hardware
- Basic Public key crypto operations that use the 32-bit PKCP engine
    - Large vector addition, subtraction and combined addition/subtraction
    - Large vector bit shift right or left
    - Large vector multiplication, modulo and division
    - Large vector compare and copy

- Dual LNME engine for Montgomery multiplication and exponentiation
    - <b>Y = X * Y * R-1 mod N</b>
    - <b>Y[1] = X * Y[0] * R-1 mod N</b>
    - <b>B = X * Y * R-1 mod N</b>
    - <b>Y = XB * R-1 mod N</b>
- Binary field GF2m engine to accelerate ECC binary field GF(2m) operations such as add, multiply and modular inversion
- ECC prime field GF(p) operations such as point addition, multiplication, doubling over all NIST recommended prime curves

## Features Supported In Driver
- RSA Module
    - Supports up to 4096 bit key.
    - Supports Raw operations.
    - Supports RSA encryption and decryption operations.
    - Supports RSA signing and verification operations.
- ECDSA Module
    - ECDSA signing and verification operations
        - P-256 and P-384 curves
## Key Types
- RSA Keys
    - RSA private key
    - RSA public key
- ECDSA Keys
    - ECDSA private key
    - ECDSA public key

### RSA Private Key
- RSA private key consists of below components:
    - <b>n</b>             : RSA modulus (n)
    - <b>d</b>             : Private exponent (d)
    - <b>p</b>             : Prime 1 (p)
    - <b>q</b>             : Prime 2 (q)
    - <b>dp</b>            : d mod (p-1)
    - <b>dq</b>            : d mod (q-1)
    - <b>coefficient</b>   : crt coefficient q^(-1) mod p

- To generate private key use this openssl command <b>openssl genrsa -out rsa_priv.pem</b>, it default create 2048 bit key.
- To generate specific size of private key use this openssl command <b>openssl genrsa -out rsa_priv.pem 4096</b>, it creates 4096 bit key.

### RSA public key
- RSA public key consists of two components:
    -   <b>n</b>        : The RSA modulus (a positive integer)
    -   <b>e</b>        : The RSA public exponent (a positive integer)
- To Extract public key from private key use this openssl command <b> openssl rsa -pubout -in rsa_pub.pem -out public.pem </b> 

### ECDSA public key
- ECDSA public key consists of below components:
    - <b>Ux</b>          : X-coordinate
    - <b>Uy</b>          : Y-coordinate
- To Extract public key from private key use this openssl command <b>openssl ec -in private.pem -pubout -out public.pem</b>

### ECDSA private key
- ECDSA private key consists of below components:
    - <b>x</b>           : private component
- To generate private key use this reference openssl command <b>openssl ecparam -name prime256v1 -genkey -noout -out private.pem</b>
## RSA Encryption and Decryption
### RSA Encryption
- PKA follows Bigint format, before passing data to driver please convert to \ref Bigintformat
- Rsa encryption operation produce a cipher text from a message using a public key.
- <b> c = m^e mod n </b>
    -    Input:
        - <b>k</b>          :RSA public key with Big int format, K has the following form:
            - <b>(n, e)</b> 
        - <b>m</b>          :Message with Big int format, message length and key length should be same

    -    Output:
        - <b>c</b>          :Cipher text

\imageStyle{pka_rsa_encrypt.png,width:80%}
\image html pka_rsa_encrypt.png "RSA Encryption operation"

### RSA Decryption
- PKA follows Bigint format, before passing data to driver please convert to \ref Bigintformat
- Rsa decryption operation recovers the message from the cipher text using private key.
- <b>m = c^d mod n</b>
    -   Input:
        - <b>k</b>        :RSA private key with Big int format, K has the following form:
            - <b>(n, d, p, q, dP, dQ, qInv)</b>
        - <b>c</b>        :cipher text

    -    Output:
        - <b>m</b>        :Message
\imageStyle{pka_rsa_decrypt.png,width:100%}
\image html pka_rsa_decrypt.png "RSA Decryption operation"
## RSA Signing and Verification
### RSA Signing
- PKA follows Bigint format, before passing data to driver please convert to \ref Bigintformat
- RSA signing operation produces a signature from a message using private key
- <b>s = m^d mod n</b>
    - Input:
        - <b>k</b>        :RSA private key with Big int format, K has the following form:
            - <b>(n, d, p, q, dP, dQ, qInv)</b>        

        - <b>m</b>        :Padded Message hash, message length and key length should be same

    - Output:
        - <b>s</b>        :Signature
\imageStyle{pka_rsa_sign.png,width:80%}
\image html pka_rsa_sign.png "RSA Signing operation"

### RSA Verification
- PKA follows Bigint format, before passing data to driver please convert to \ref Bigintformat
- RSA verification operation recovers the message from the signature using public key.
- <b>m = s^e mod n</b>
    - Input:
        - <b>k</b>   :RSA public key with Big int format, K has the following form:
            - <b>(n, e)</b>
        - <b>s</b>        :Signature

    - Output:
        - <b>m</b>        :Padded Message hash

\imageStyle{pka_rsa_verify.png,width:80%}
\image html pka_rsa_verify.png "RSA Verification operation"

## ECDSA Signing and Verification
### ECDSA Signing
- PKA follows Bigint format, before passing data to driver please convert to \ref Bigintformat
- signing operation produces a signature from a message using private key
- Input :
    - <b>cp</b>     : EC prime curve parameters
    - <b>priv</b>   : Private key
    - <b>k</b>      : Random key
    - <b>h</b>      : Message Hash

- Output :
    - <b>sig</b>    : Signature
\imageStyle{pka_ecdsa_sign.png,width:80%}
\image html pka_ecdsa_sign.png "ECDSA Signing operation"
### ECDSA Verification
- PKA follows Bigint format, before passing data to driver please convert to \ref Bigintformat
- ECDSA verification operation check using public key
- Input :
    - <b>cp</b>     : EC prime curve parameters
    - <b>pub</b>    : Public key
    - <b>sig</b>    : Signature
    - <b>h</b>      : Message Hash

- Output :
    - Return TRUE if Verified or FALSE if verify Failed
\imageStyle{pka_ecdsa_verify.png,width:80%}
\image html pka_ecdsa_verify.png "ECDSA Verification operation"
## SysConfig Features
- SysConfig contains PkA Instances, base address of Crypto accelerator and PKA module base address for pka handle to manage PKA api's.

## Features NOT Supported
- ECDH not supported

\note Please use Linux terminal or git bash in windows to execute below openssl commands

## Openssl Commands For RSA
- Used openssl version is <b>OpenSSL 1.1.1k  25 Mar 2021</b>
### To Create Public and private keys
- <b> openssl genrsa -out rsa_priv.pem </b>, default it creates 2048 bit Private key.
- <b> openssl genrsa -out rsa_priv.pem 4096 </b>, it creates 4096 bit Private key.
- <b> openssl rsa -pubout -in rsa_priv.pem -out rsa_pub.pem </b>, it extract public key from private key.
### Create message
- Write a message in message.txt file : <b>vi message.txt</b>,
- Convert txt file to bin file for better result : <b>xxd -r -p message.txt >message.bin</b>
### RSA Encryption operations

\note Input message/sign and key length should be same. Non equivalent lengths not Supported, required padding in this case.

- PKA follows Bigint format, before passing data to driver please convert to \ref Bigintformat
- To perform encryption operation : <b> openssl rsautl -encrypt -raw -pubin -inkey rsa_pub.pem -in message.bin -out cipher.bin </b>
- To see the result : <b>xxd cipher.bin</b>
### RSA Decryption operations
- PKA follows Bigint format, before passing data to driver please convert to \ref Bigintformat
- To perform decryption operation : <b> openssl rsautl -decrypt -raw -inkey rsa_priv.pem -in cipher.bin -out decipher.bin </b>
- To see the result : <b>xxd decipher.bin</b>
### To Create Sha Hash
- <b> openssl dgst -sha512 -binary -out msg_sha512.dgt message.bin </b>
### RSA Signing operations
- PKA follows Bigint format, before passing data to driver please convert to \ref Bigintformat
- To perform signing operation : <b> openssl rsautl -sign -inkey rsa_priv.key -in msg_sha512.dgt -out msg_sha512_signed.dgt </b>
### RSA Verification operations
- PKA follows Bigint format, before passing data to driver please convert to \ref Bigintformat
- To perform verification operation : <b> openssl rsautl -verify -pubin -inkey rsa_pub.key -in msg_sha512_signed.dgt -out msg_sha512_verify.dgt </b>
- To see the result : <b>xxd msg_sha512_verify.dgt</b>
- To see the raw result use below command
    -   <b>openssl rsautl -verify -pubin -inkey rsa_pub.key -in msg_sha512_signed.dgt -raw >msg_sha512_verify.dgt</b>
    -   <b>xxd msg_sha512_verify.dgt</b>

## Openssl Commands For ECDSA
### To Create Public and private keys
- To create prime256v1 Private key : <b> openssl ecparam -name prime256v1 -genkey -noout -out private.pem </b>
- To extract public key from private key : <b> openssl ec -in private.pem -pubout -out public.pem </b>
- To see the key in text format : <b> Openssl pkey -in private.pem -text -noout </b>
### Create message
- Write a message in message.txt file : <b>vi message.txt</b>,
- Convert txt file to bin file for better result : <b>xxd -r -p message.txt >message.bin</b>
### Create Random key
- Write a random number in rand_key.txt file : <b>vi rand_key.txt</b>,
- Convert txt file to bin file for better result : <b>xxd -r -p rand_key.txt >rand_key.bin</b>
### ECDSA Signing operations

\note Openssl ECDSA Signature and our PKA ECDSA Signature will not match due to randomness of openssl

- PKA follows Bigint format, before passing data to driver please convert to \ref Bigintformat
- To perform signing operation : <b> openssl dgst -sha256 -sign private.pem -rand rand_key.bin -out ecdsa_sign.bin message.bin </b>
- To see the signature in text format : <b>openssl asn1parse -in ecdsa_sign.bin -inform der</b>

### ECDSA Verification operations
- PKA follows Bigint format, before passing data to driver please convert to \ref Bigintformat
- To perform verification operation : <b> openssl dgst -sha256 -verify public.pem -signature ecdsa_sign.bin message.bin </b>

\note PKA RSA driver follows Big int Format for inputs, please refer below section for Big int format
\section Bigintformat Bigint format

\code 

bigIntArr[] =
{
    0x000000xx -->Number of words in array

    0xXXXXXXXX  |
    0xXXXXXXXX  |
    0xXXXXXXXX  |
    0xXXXXXXXX  |
    0xXXXXXXXX  |----------- Key values in word format with least significant word first
    0xXXXXXXXX  |
    0xXXXXXXXX  |
    0xXXXXXXXX  |
    0xXXXXXXXX  |
}

\endcode

### Bigint Format Conversion for PKA RSA Driver
#### Steps for Bigint conversion
- For converting input to bigint please follow below steps
    - <b>step 1</b> : Convert uint8_t to uint32_t format using #Crypto_Uint8ToUint32()
    - <b>step 2</b> : Convert uint32_t to Bigint format using #Crypto_Uint32ToBigInt()
- Sample input key in below, to see content of key use this command <b> Openssl rsa -text -in rsa_priv.pem </b>
\code

RSA Private-Key: (2048 bit, 2 primes)
modulus:
    00:c5:69:69:c2:95:80:33:af:75:24:50:98:9e:52:
    9c:69:ff:21:af:ad:04:29:a8:f5:f6:e7:ae:2f:0c:
    ea:14:13:df:ba:2f:3e:cf:cc:4a:8a:6e:9f:6d:b7:
    b3:dc:15:98:be:7d:5d:82:0b:f4:7e:6f:9d:31:1d:
    b6:d1:90:6a:36:79:81:67:48:33:0f:4e:75:3b:e6:
    1c:d6:43:36:eb:94:21:0f:03:65:93:27:55:27:3c:
    a6:8a:5d:a5:5e:f9:d6:ed:66:4d:03:19:58:6a:35:
    97:93:e7:f4:09:30:d8:f4:ee:42:0c:45:1a:23:f0:
    b0:58:9e:d8:4b:72:d6:68:74:5c:4d:c7:59:b2:7c:
    8e:0c:c7:8d:2e:6f:3e:de:8c:2d:2e:a2:66:cd:7f:
    b0:cc:0d:c9:79:a0:e1:47:dc:23:1d:8b:00:b9:e8:
    48:34:f9:c3:e7:67:b5:32:37:1c:84:56:10:c8:53:
    c2:9a:95:89:7d:d3:5a:62:16:3f:a1:8b:8d:ee:11:
    2e:a4:ac:16:9f:63:6b:12:fe:2f:6c:cb:34:36:d9:
    43:06:37:cb:5a:09:ce:7d:2a:30:3d:51:75:df:58:
    6b:60:6c:c6:bf:d4:7e:b2:3d:dc:cc:06:3e:28:b3:
    c7:91:d7:76:38:9f:24:b6:e7:05:69:53:ac:8c:6c:
    85:b9
publicExponent: 65537 (0x10001)
privateExponent:
    17:12:11:ad:0e:e1:b3:a8:9e:ad:06:ca:3f:3e:72:
    4f:24:e4:df:ed:fd:5d:8d:04:69:bd:7b:aa:bd:fc:
    a1:2a:0d:6c:69:d7:12:5b:d2:9e:48:fd:52:ca:34:
    37:d5:42:4b:88:c5:23:cc:97:df:2a:d6:19:06:5a:
    f6:34:c5:64:e6:60:4c:1c:b0:f2:e9:fd:63:69:aa:
    17:14:35:d7:e4:30:d4:db:55:c4:93:c5:2c:d7:b6:
    b2:d6:ec:db:a3:a1:0d:8f:76:12:95:a4:b2:8c:de:
    d1:07:3b:8a:d3:6d:97:7a:3c:b7:c8:5f:9c:b1:a5:
    3a:46:1e:0a:fb:39:b9:6d:22:fe:e7:9e:0b:ee:ca:
    18:bf:cb:82:70:86:e8:3e:d8:aa:0e:22:6e:e9:c2:
    90:55:d4:42:0d:3a:cb:2d:4e:92:58:81:46:d2:b2:
    f9:33:de:91:c5:18:a6:1d:e5:12:6c:9e:d2:de:1a:
    86:c8:d4:b8:3c:c3:40:a0:ae:b5:ab:6c:9f:2c:81:
    88:e6:eb:00:c1:57:17:6e:0a:d1:94:76:c9:26:03:
    46:7d:39:41:55:24:bf:70:08:35:70:9f:97:78:20:
    5a:db:d6:cc:8c:2f:9c:77:a1:76:24:d3:0c:d9:e2:
    f2:b4:c7:76:70:90:f0:b9:ab:ae:27:08:76:b2:49:
    71
prime1:
    00:fa:5d:68:75:50:8f:a4:ea:c2:62:4c:cf:53:b3:
    57:c3:5b:e4:ed:e3:53:c2:9c:4c:f5:13:d0:5d:1a:
    e0:06:0c:6d:96:e0:50:93:64:48:df:50:ab:e8:24:
    05:5b:bb:e7:77:32:91:b0:b0:ea:72:c7:a3:ed:1f:
    55:0d:95:fe:d7:8a:ee:33:c9:de:aa:84:ef:03:ef:
    b7:a2:d9:a2:a8:a2:db:ae:e5:08:fd:e1:68:2f:5a:
    2c:16:d7:9c:03:11:4e:e4:b1:70:1b:e1:9e:37:80:
    be:0b:50:7f:43:1a:cc:84:be:5c:ae:1f:3f:a6:00:
    b5:89:6f:86:ad:16:15:91:0d
prime2:
    00:c9:da:e4:39:98:e1:94:26:33:af:a2:f3:bc:7b:
    66:1e:74:27:a5:77:e2:d0:89:1e:2a:45:13:4d:d8:
    d1:67:b7:44:44:00:91:0f:c4:65:5c:54:07:6d:5a:
    73:8f:5e:da:85:25:6b:85:2a:3b:a2:41:f3:8b:f7:
    0c:c1:ed:43:2b:16:1f:2d:e1:70:56:6b:ee:c6:a3:
    d2:97:26:51:69:54:a8:4c:48:79:3f:7f:6f:c8:5d:
    06:77:9b:17:fb:c6:ba:03:b9:d7:f1:f3:11:ce:70:
    ff:89:5c:08:18:0b:5d:7d:b3:5e:18:fa:c9:d6:fa:
    06:6b:6f:bb:98:b0:b5:24:5d
exponent1:
    34:5d:12:fa:ee:65:8c:bd:98:f8:4c:4e:54:98:3e:
    f3:da:25:70:67:ca:5d:fa:a8:d1:dd:5a:08:0f:15:
    e9:cd:f1:a1:cb:ba:ae:89:1c:00:b0:f3:b3:72:cd:
    38:19:7d:d8:dd:57:c1:57:cf:41:40:66:15:b3:26:
    eb:d7:82:5c:7c:6f:43:9f:a6:15:8e:06:1b:91:a8:
    9b:c3:df:14:5c:33:8c:49:d4:ee:9c:95:58:ca:08:
    4c:a2:bd:bb:9a:84:20:aa:c7:e2:dc:f6:65:6f:64:
    d0:22:fe:ea:ff:10:e5:76:97:15:c7:ed:5c:ff:ea:
    f8:1a:be:55:37:38:3d:85
exponent2:
    00:c7:ff:6f:7d:79:f2:97:bd:3a:1c:dd:d1:ad:80:
    7e:6f:d2:1a:ae:22:18:c3:11:f8:a7:5d:05:81:6e:
    40:1d:09:42:44:36:63:84:41:cd:44:2b:a5:a8:3d:
    a4:9d:fb:17:7c:30:d5:55:7d:c0:d4:45:90:2f:af:
    ba:1e:33:1d:08:05:c5:22:ca:69:69:d7:4b:1f:d9:
    95:80:59:60:f4:82:02:b6:82:60:4f:ff:ce:1b:b1:
    c0:04:a5:d0:9f:90:09:22:43:f2:a8:31:74:05:1d:
    84:b2:ee:52:be:b0:14:13:c5:b2:88:01:84:16:d2:
    67:a5:ef:70:ea:1a:bb:7f:a9
coefficient:
    56:90:21:af:68:6a:63:68:9b:24:82:f4:3d:3f:86:
    f5:40:64:a0:1a:5b:ad:b7:f0:b4:a0:6e:e4:65:6a:
    31:6e:1f:32:a4:73:b5:03:b5:48:b4:c0:9f:8e:0d:
    7f:92:a1:30:58:46:88:07:fb:11:8e:44:0e:a8:4c:
    53:f3:42:68:db:31:93:9e:bb:22:5a:e5:e5:2f:73:
    a1:02:0c:db:4e:5a:4c:e6:ba:4a:99:14:95:2f:12:
    76:99:44:6c:93:e0:d5:5d:38:42:95:b5:52:48:8a:
    c2:2e:8a:dd:8e:2e:b2:e7:fa:df:6d:77:f5:e1:27:
    bc:d2:7d:4c:05:f2:32:8e

\endcode 

<b>Step 1</b>:
As part of first step we need to convert uint8_t to uint32_t format, for this conversion we are using #Crypto_Uint8ToUint32().
Here we are using modulus value for converting to bigint format.  

Input of #Crypto_Uint8ToUint32()

\code
0xC5,0x69,0x69,0xC2,0x95,0x80,0x33,0xAF,0x75,0x24,
0x50,0x98,0x9E,0x52,0x9C,0x69,0xFF,0x21,0xAF,0xAD,
0x04,0x29,0xA8,0xF5,0xF6,0xE7,0xAE,0x2F,0x0C,0xEA,
0x14,0x13,0xDF,0xBA,0x2F,0x3E,0xCF,0xCC,0x4A,0x8A,
0x6E,0x9F,0x6D,0xB7,0xB3,0xDC,0x15,0x98,0xBE,0x7D,
0x5D,0x82,0x0B,0xF4,0x7E,0x6F,0x9D,0x31,0x1D,0xB6,
0xD1,0x90,0x6A,0x36,0x79,0x81,0x67,0x48,0x33,0x0F,
0x4E,0x75,0x3B,0xE6,0x1C,0xD6,0x43,0x36,0xEB,0x94,
0x21,0x0F,0x03,0x65,0x93,0x27,0x55,0x27,0x3C,0xA6,
0x8A,0x5D,0xA5,0x5E,0xF9,0xD6,0xED,0x66,0x4D,0x03,
0x19,0x58,0x6A,0x35,0x97,0x93,0xE7,0xF4,0x09,0x30,
0xD8,0xF4,0xEE,0x42,0x0C,0x45,0x1A,0x23,0xF0,0xB0,
0x58,0x9E,0xD8,0x4B,0x72,0xD6,0x68,0x74,0x5C,0x4D,
0xC7,0x59,0xB2,0x7C,0x8E,0x0C,0xC7,0x8D,0x2E,0x6F,
0x3E,0xDE,0x8C,0x2D,0x2E,0xA2,0x66,0xCD,0x7F,0xB0,
0xCC,0x0D,0xC9,0x79,0xA0,0xE1,0x47,0xDC,0x23,0x1D,
0x8B,0x00,0xB9,0xE8,0x48,0x34,0xF9,0xC3,0xE7,0x67,
0xB5,0x32,0x37,0x1C,0x84,0x56,0x10,0xC8,0x53,0xC2,
0x9A,0x95,0x89,0x7D,0xD3,0x5A,0x62,0x16,0x3F,0xA1,
0x8B,0x8D,0xEE,0x11,0x2E,0xA4,0xAC,0x16,0x9F,0x63,
0x6B,0x12,0xFE,0x2F,0x6C,0xCB,0x34,0x36,0xD9,0x43,
0x06,0x37,0xCB,0x5A,0x09,0xCE,0x7D,0x2A,0x30,0x3D,
0x51,0x75,0xDF,0x58,0x6B,0x60,0x6C,0xC6,0xBF,0xD4,
0x7E,0xB2,0x3D,0xDC,0xCC,0x06,0x3E,0x28,0xB3,0xC7,
0x91,0xD7,0x76,0x38,0x9F,0x24,0xB6,0xE7,0x05,0x69,
0x53,0xAC,0x8C,0x6C,0x85,0xB9

\endcode

output of #Crypto_Uint8ToUint32()
\code

0xC56969C2UL,0x958033AFUL,0x75245098UL,0x9E529C69UL,
0xFF21AFADUL,0x0429A8F5UL,0xF6E7AE2FUL,0x0CEA1413UL,
0xDFBA2F3EUL,0xCFCC4A8AUL,0x6E9F6DB7UL,0xB3DC1598UL,
0xBE7D5D82UL,0x0BF47E6FUL,0x9D311DB6UL,0xD1906A36UL,
0x79816748UL,0x330F4E75UL,0x3BE61CD6UL,0x4336EB94UL,
0x210F0365UL,0x93275527UL,0x3CA68A5DUL,0xA55EF9D6UL,
0xED664D03UL,0x19586A35UL,0x9793E7F4UL,0x0930D8F4UL,
0xEE420C45UL,0x1A23F0B0UL,0x589ED84BUL,0x72D66874UL,
0x5C4DC759UL,0xB27C8E0CUL,0xC78D2E6FUL,0x3EDE8C2DUL,
0x2EA266CDUL,0x7FB0CC0DUL,0xC979A0E1UL,0x47DC231DUL,
0x8B00B9E8UL,0x4834F9C3UL,0xE767B532UL,0x371C8456UL,
0x10C853C2UL,0x9A95897DUL,0xD35A6216UL,0x3FA18B8DUL,
0xEE112EA4UL,0xAC169F63UL,0x6B12FE2FUL,0x6CCB3436UL,
0xD9430637UL,0xCB5A09CEUL,0x7D2A303DUL,0x5175DF58UL,
0x6B606CC6UL,0xBFD47EB2UL,0x3DDCCC06UL,0x3E28B3C7UL,
0x91D77638UL,0x9F24B6E7UL,0x056953ACUL,0x8C6C85B9UL,

\endcode

<b>Step 2</b>:

As part of second step we need to convert uint32_t to bigint format, for this conversion we are using #Crypto_bigIntToUint32().

Input of #Crypto_bigIntToUint32()
\code

0xC56969C2UL,0x958033AFUL,0x75245098UL,0x9E529C69UL,
0xFF21AFADUL,0x0429A8F5UL,0xF6E7AE2FUL,0x0CEA1413UL,
0xDFBA2F3EUL,0xCFCC4A8AUL,0x6E9F6DB7UL,0xB3DC1598UL,
0xBE7D5D82UL,0x0BF47E6FUL,0x9D311DB6UL,0xD1906A36UL,
0x79816748UL,0x330F4E75UL,0x3BE61CD6UL,0x4336EB94UL,
0x210F0365UL,0x93275527UL,0x3CA68A5DUL,0xA55EF9D6UL,
0xED664D03UL,0x19586A35UL,0x9793E7F4UL,0x0930D8F4UL,
0xEE420C45UL,0x1A23F0B0UL,0x589ED84BUL,0x72D66874UL,
0x5C4DC759UL,0xB27C8E0CUL,0xC78D2E6FUL,0x3EDE8C2DUL,
0x2EA266CDUL,0x7FB0CC0DUL,0xC979A0E1UL,0x47DC231DUL,
0x8B00B9E8UL,0x4834F9C3UL,0xE767B532UL,0x371C8456UL,
0x10C853C2UL,0x9A95897DUL,0xD35A6216UL,0x3FA18B8DUL,
0xEE112EA4UL,0xAC169F63UL,0x6B12FE2FUL,0x6CCB3436UL,
0xD9430637UL,0xCB5A09CEUL,0x7D2A303DUL,0x5175DF58UL,
0x6B606CC6UL,0xBFD47EB2UL,0x3DDCCC06UL,0x3E28B3C7UL,
0x91D77638UL,0x9F24B6E7UL,0x056953ACUL,0x8C6C85B9UL,

\endcode

Output of #Crypto_bigIntToUint32()

\code

0x00000040UL, --> Number of words in array

0x8C6C85B9UL,0x056953ACUL,0x9F24B6E7UL,0x91D77638UL,   --> Modulus value in word format, with least significant word 
0x3E28B3C7UL,0x3DDCCC06UL,0xBFD47EB2UL,0x6B606CC6UL,                                                           first
0x5175DF58UL,0x7D2A303DUL,0xCB5A09CEUL,0xD9430637UL,
0x6CCB3436UL,0x6B12FE2FUL,0xAC169F63UL,0xEE112EA4UL,
0x3FA18B8DUL,0xD35A6216UL,0x9A95897DUL,0x10C853C2UL,
0x371C8456UL,0xE767B532UL,0x4834F9C3UL,0x8B00B9E8UL,
0x47DC231DUL,0xC979A0E1UL,0x7FB0CC0DUL,0x2EA266CDUL,
0x3EDE8C2DUL,0xC78D2E6FUL,0xB27C8E0CUL,0x5C4DC759UL,
0x72D66874UL,0x589ED84BUL,0x1A23F0B0UL,0xEE420C45UL,
0x0930D8F4UL,0x9793E7F4UL,0x19586A35UL,0xED664D03UL,
0xA55EF9D6UL,0x3CA68A5DUL,0x93275527UL,0x210F0365UL,
0x4336EB94UL,0x3BE61CD6UL,0x330F4E75UL,0x79816748UL,
0xD1906A36UL,0x9D311DB6UL,0x0BF47E6FUL,0xBE7D5D82UL,
0xB3DC1598UL,0x6E9F6DB7UL,0xCFCC4A8AUL,0xDFBA2F3EUL,
0x0CEA1413UL,0xF6E7AE2FUL,0x0429A8F5UL,0xFF21AFADUL,
0x9E529C69UL,0x75245098UL,0x958033AFUL,0xC56969C2UL,

\endcode

## PKA API's Supported

- #PKA_open()           : Open PKA instance, enable PKA engine, initialize clocks and load PKA fw.
- #PKA_RSAPrivate()     : This function performs decryption or signing operations
- #PKA_RSAPublic()      : This function performs encryption or verification operations
- #PKA_ECDSASign()      : This function performs ECDSA signing operations.
- #PKA_ECDSAVerify()    : This function performs ECDSA verification operations.
- #PKA_close()          : Close PKA instance, disable PKA engine, un-initialize clocks and unload PKA fw.

## API Sequence for PKA (Public-key accelerator)

#### RSA
This sequence performs PKA RSA operations.

- #PKA_open()           : Open PKA instance, enable PKA engine, initialize clocks and load PKA fw.
- #PKA_RSAPrivate()     : This function performs decryption or signing operations.
- #PKA_RSAPublic()      : This function performs encryption or verification operations.
- #PKA_close()          : Close PKA instance, disable PKA engine, un-initialize clocks and unload PKA fw.
- Open the pka module with instance by using #PKA_open(), this call used for loading pka fw, clock enable and pka engine enable, for encryption or verification use #PKA_RSAPublic(), for decryption or signing use #PKA_RSAPrivate(), to close pka instance, disable engine, disable clock and fw unload use #PKA_close().

#### ECDSA
This sequence performs PKA ECDSA operations.

- #PKA_open()           : Open PKA instance, enable PKA engine, initialize clocks and load PKA fw.
- #PKA_ECDSASign()      : This function performs ECDSA signing operations.
- #PKA_ECDSAVerify()    : This function performs ECDSA verification operations.
- #PKA_close()          : Close PKA instance, disable PKA engine, un-initialize clocks and unload PKA fw.
- Open the pka module with instance by using #PKA_open(), this call used for loading pka fw, clock enable and pka engine enable, for signing use #PKA_ECDSASign(), for verification use #PKA_ECDSAVerify(), to close pka instance, disable engine, disable clock and fw unload use #PKA_close().

## Example Usage
\cond SOC_AM64X || SOC_AM243X
Include the below file to access the SA2UL PKA APIs
\snippet Sa2ul_pka_rsa_encrypt_decrypt_sample.c include

sa2ul pka rsa Encryption and Decryption Example
\snippet Sa2ul_pka_rsa_encrypt_decrypt_sample.c sa2ulpkarsa

\endcond
## API

\ref SECURITY_PKA_MODULE