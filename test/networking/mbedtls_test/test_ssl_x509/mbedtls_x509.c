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
    data_x509 is an array of string literals of test data for mbedTLS sanity tests for X509
    these data vectors will be used in case of both software and hardware cryptography offloaded mbedTLS (TO DO)
*/

const char* data_x509[] = {
    "depends_on:0:1:2",
    "0:char*:\"data_files/server1.crt\":char*:\"cert. version     \\: 3\nserial number     \\: 01\nissuer name       \\: C=NL, O=PolarSSL, CN=PolarSSL Test CA\nsubject name      \\: C=NL, O=PolarSSL, CN=PolarSSL Server 1\nissued  on        \\: 2011-02-12 14\\:44\\:06\nexpires on        \\: 2021-02-12 14\\:44\\:06\nsigned using      \\: RSA with SHA1\nRSA key size      \\: 2048 bits\nbasic constraints \\: CA=false\n\"",
    "depends_on:0:1:2",
    "0:char*:\"data_files/server2.crt\":char*:\"cert. version     \\: 3\nserial number     \\: 02\nissuer name       \\: C=NL, O=PolarSSL, CN=PolarSSL Test CA\nsubject name      \\: C=NL, O=PolarSSL, CN=localhost\nissued  on        \\: 2011-02-12 14\\:44\\:06\nexpires on        \\: 2021-02-12 14\\:44\\:06\nsigned using      \\: RSA with SHA1\nRSA key size      \\: 2048 bits\nbasic constraints \\: CA=false\n\"",
    "depends_on:0:1:2",
    "0:char*:\"data_files/test-ca.crt\":char*:\"cert. version     \\: 3\nserial number     \\: 00\nissuer name       \\: C=NL, O=PolarSSL, CN=PolarSSL Test CA\nsubject name      \\: C=NL, O=PolarSSL, CN=PolarSSL Test CA\nissued  on        \\: 2011-02-12 14\\:44\\:00\nexpires on        \\: 2021-02-12 14\\:44\\:00\nsigned using      \\: RSA with SHA1\nRSA key size      \\: 2048 bits\nbasic constraints \\: CA=true\n\"",
    "depends_on:0:1:3",
    "0:char*:\"data_files/cert_md2.crt\":char*:\"cert. version     \\: 3\nserial number     \\: 09\nissuer name       \\: C=NL, O=PolarSSL, CN=PolarSSL Test CA\nsubject name      \\: C=NL, O=PolarSSL, CN=PolarSSL Cert MD2\nissued  on        \\: 2009-07-12 10\\:56\\:59\nexpires on        \\: 2011-07-12 10\\:56\\:59\nsigned using      \\: RSA with MD2\nRSA key size      \\: 2048 bits\nbasic constraints \\: CA=false\n\"",
    "depends_on:0:1:4",
    "0:char*:\"data_files/cert_md4.crt\":char*:\"cert. version     \\: 3\nserial number     \\: 05\nissuer name       \\: C=NL, O=PolarSSL, CN=PolarSSL Test CA\nsubject name      \\: C=NL, O=PolarSSL, CN=PolarSSL Cert MD4\nissued  on        \\: 2011-02-12 14\\:44\\:07\nexpires on        \\: 2021-02-12 14\\:44\\:07\nsigned using      \\: RSA with MD4\nRSA key size      \\: 2048 bits\nbasic constraints \\: CA=false\n\"",
    "depends_on:0:1:5",
    "0:char*:\"data_files/cert_md5.crt\":char*:\"cert. version     \\: 3\nserial number     \\: 06\nissuer name       \\: C=NL, O=PolarSSL, CN=PolarSSL Test CA\nsubject name      \\: C=NL, O=PolarSSL, CN=PolarSSL Cert MD5\nissued  on        \\: 2011-02-12 14\\:44\\:07\nexpires on        \\: 2021-02-12 14\\:44\\:07\nsigned using      \\: RSA with MD5\nRSA key size      \\: 2048 bits\nbasic constraints \\: CA=false\n\"",
    "depends_on:0:1:2",
    "0:char*:\"data_files/cert_sha1.crt\":char*:\"cert. version     \\: 3\nserial number     \\: 07\nissuer name       \\: C=NL, O=PolarSSL, CN=PolarSSL Test CA\nsubject name      \\: C=NL, O=PolarSSL, CN=PolarSSL Cert SHA1\nissued  on        \\: 2011-02-12 14\\:44\\:07\nexpires on        \\: 2021-02-12 14\\:44\\:07\nsigned using      \\: RSA with SHA1\nRSA key size      \\: 2048 bits\nbasic constraints \\: CA=false\n\"",
    "depends_on:0:1:6",
    "0:char*:\"data_files/cert_sha224.crt\":char*:\"cert. version     \\: 3\nserial number     \\: 08\nissuer name       \\: C=NL, O=PolarSSL, CN=PolarSSL Test CA\nsubject name      \\: C=NL, O=PolarSSL, CN=PolarSSL Cert SHA224\nissued  on        \\: 2011-02-12 14\\:44\\:07\nexpires on        \\: 2021-02-12 14\\:44\\:07\nsigned using      \\: RSA with SHA-224\nRSA key size      \\: 2048 bits\nbasic constraints \\: CA=false\n\"",
    "depends_on:0:1:6",
    "0:char*:\"data_files/cert_sha256.crt\":char*:\"cert. version     \\: 3\nserial number     \\: 09\nissuer name       \\: C=NL, O=PolarSSL, CN=PolarSSL Test CA\nsubject name      \\: C=NL, O=PolarSSL, CN=PolarSSL Cert SHA256\nissued  on        \\: 2011-02-12 14\\:44\\:07\nexpires on        \\: 2021-02-12 14\\:44\\:07\nsigned using      \\: RSA with SHA-256\nRSA key size      \\: 2048 bits\nbasic constraints \\: CA=false\n\"",
    "depends_on:0:1:7",
    "0:char*:\"data_files/cert_sha384.crt\":char*:\"cert. version     \\: 3\nserial number     \\: 0A\nissuer name       \\: C=NL, O=PolarSSL, CN=PolarSSL Test CA\nsubject name      \\: C=NL, O=PolarSSL, CN=PolarSSL Cert SHA384\nissued  on        \\: 2011-02-12 14\\:44\\:07\nexpires on        \\: 2021-02-12 14\\:44\\:07\nsigned using      \\: RSA with SHA-384\nRSA key size      \\: 2048 bits\nbasic constraints \\: CA=false\n\"",
    "depends_on:0:1:7",
    "0:char*:\"data_files/cert_sha512.crt\":char*:\"cert. version     \\: 3\nserial number     \\: 0B\nissuer name       \\: C=NL, O=PolarSSL, CN=PolarSSL Test CA\nsubject name      \\: C=NL, O=PolarSSL, CN=PolarSSL Cert SHA512\nissued  on        \\: 2011-02-12 14\\:44\\:07\nexpires on        \\: 2021-02-12 14\\:44\\:07\nsigned using      \\: RSA with SHA-512\nRSA key size      \\: 2048 bits\nbasic constraints \\: CA=false\n\"",
    "depends_on:0:8:2",
    "0:char*:\"data_files/server9.crt\":char*:\"cert. version     \\: 3\nserial number     \\: 16\nissuer name       \\: C=NL, O=PolarSSL, CN=PolarSSL Test CA\nsubject name      \\: C=NL, O=PolarSSL, CN=localhost\nissued  on        \\: 2014-01-20 13\\:38\\:16\nexpires on        \\: 2024-01-18 13\\:38\\:16\nsigned using      \\: RSASSA-PSS (SHA1, MGF1-SHA1, 0xEA)\nRSA key size      \\: 1024 bits\nbasic constraints \\: CA=false\n\"",
    "depends_on:0:8:6",
    "0:char*:\"data_files/server9-sha224.crt\":char*:\"cert. version     \\: 3\nserial number     \\: 17\nissuer name       \\: C=NL, O=PolarSSL, CN=PolarSSL Test CA\nsubject name      \\: C=NL, O=PolarSSL, CN=localhost\nissued  on        \\: 2014-01-20 13\\:57\\:36\nexpires on        \\: 2024-01-18 13\\:57\\:36\nsigned using      \\: RSASSA-PSS (SHA224, MGF1-SHA224, 0xE2)\nRSA key size      \\: 1024 bits\nbasic constraints \\: CA=false\n\"",
    "depends_on:0:8:6",
    "0:char*:\"data_files/server9-sha256.crt\":char*:\"cert. version     \\: 3\nserial number     \\: 18\nissuer name       \\: C=NL, O=PolarSSL, CN=PolarSSL Test CA\nsubject name      \\: C=NL, O=PolarSSL, CN=localhost\nissued  on        \\: 2014-01-20 13\\:57\\:45\nexpires on        \\: 2024-01-18 13\\:57\\:45\nsigned using      \\: RSASSA-PSS (SHA256, MGF1-SHA256, 0xDE)\nRSA key size      \\: 1024 bits\nbasic constraints \\: CA=false\n\"",
    "depends_on:0:8:7",
    "0:char*:\"data_files/server9-sha384.crt\":char*:\"cert. version     \\: 3\nserial number     \\: 19\nissuer name       \\: C=NL, O=PolarSSL, CN=PolarSSL Test CA\nsubject name      \\: C=NL, O=PolarSSL, CN=localhost\nissued  on        \\: 2014-01-20 13\\:57\\:58\nexpires on        \\: 2024-01-18 13\\:57\\:58\nsigned using      \\: RSASSA-PSS (SHA384, MGF1-SHA384, 0xCE)\nRSA key size      \\: 1024 bits\nbasic constraints \\: CA=false\n\"",
    "depends_on:0:8:7",
    "0:char*:\"data_files/server9-sha512.crt\":char*:\"cert. version     \\: 3\nserial number     \\: 1A\nissuer name       \\: C=NL, O=PolarSSL, CN=PolarSSL Test CA\nsubject name      \\: C=NL, O=PolarSSL, CN=localhost\nissued  on        \\: 2014-01-20 13\\:58\\:12\nexpires on        \\: 2024-01-18 13\\:58\\:12\nsigned using      \\: RSASSA-PSS (SHA512, MGF1-SHA512, 0xBE)\nRSA key size      \\: 1024 bits\nbasic constraints \\: CA=false\n\"",
    "depends_on:0:9:10:2",
    "0:char*:\"data_files/server5-sha1.crt\":char*:\"cert. version     \\: 3\nserial number     \\: 12\nissuer name       \\: C=NL, O=PolarSSL, CN=Polarssl Test EC CA\nsubject name      \\: C=NL, O=PolarSSL, CN=localhost\nissued  on        \\: 2013-09-24 16\\:21\\:27\nexpires on        \\: 2023-09-22 16\\:21\\:27\nsigned using      \\: ECDSA with SHA1\nEC key size       \\: 256 bits\nbasic constraints \\: CA=false\n\"",
    "depends_on:0:9:10:6",
    "0:char*:\"data_files/server5-sha224.crt\":char*:\"cert. version     \\: 3\nserial number     \\: 13\nissuer name       \\: C=NL, O=PolarSSL, CN=Polarssl Test EC CA\nsubject name      \\: C=NL, O=PolarSSL, CN=localhost\nissued  on        \\: 2013-09-24 16\\:21\\:27\nexpires on        \\: 2023-09-22 16\\:21\\:27\nsigned using      \\: ECDSA with SHA224\nEC key size       \\: 256 bits\nbasic constraints \\: CA=false\n\"",
    "depends_on:0:9:10:6",
    "0:char*:\"data_files/server5.crt\":char*:\"cert. version     \\: 3\nserial number     \\: 09\nissuer name       \\: C=NL, O=PolarSSL, CN=Polarssl Test EC CA\nsubject name      \\: C=NL, O=PolarSSL, CN=localhost\nissued  on        \\: 2013-09-24 15\\:52\\:04\nexpires on        \\: 2023-09-22 15\\:52\\:04\nsigned using      \\: ECDSA with SHA256\nEC key size       \\: 256 bits\nbasic constraints \\: CA=false\n\"",
    "depends_on:0:9:10:7",
    "0:char*:\"data_files/server5-sha384.crt\":char*:\"cert. version     \\: 3\nserial number     \\: 14\nissuer name       \\: C=NL, O=PolarSSL, CN=Polarssl Test EC CA\nsubject name      \\: C=NL, O=PolarSSL, CN=localhost\nissued  on        \\: 2013-09-24 16\\:21\\:27\nexpires on        \\: 2023-09-22 16\\:21\\:27\nsigned using      \\: ECDSA with SHA384\nEC key size       \\: 256 bits\nbasic constraints \\: CA=false\n\"",
    "depends_on:0:9:10:7",
    "0:char*:\"data_files/server5-sha512.crt\":char*:\"cert. version     \\: 3\nserial number     \\: 15\nissuer name       \\: C=NL, O=PolarSSL, CN=Polarssl Test EC CA\nsubject name      \\: C=NL, O=PolarSSL, CN=localhost\nissued  on        \\: 2013-09-24 16\\:21\\:27\nexpires on        \\: 2023-09-22 16\\:21\\:27\nsigned using      \\: ECDSA with SHA512\nEC key size       \\: 256 bits\nbasic constraints \\: CA=false\n\"",
    "depends_on:0:1:2",
    "0:char*:\"data_files/server1.cert_type.crt\":char*:\"cert. version     \\: 3\nserial number     \\: 01\nissuer name       \\: C=NL, O=PolarSSL, CN=PolarSSL Test CA\nsubject name      \\: C=NL, O=PolarSSL, CN=PolarSSL Server 1\nissued  on        \\: 2011-02-12 14\\:44\\:06\nexpires on        \\: 2021-02-12 14\\:44\\:06\nsigned using      \\: RSA with SHA1\nRSA key size      \\: 2048 bits\nbasic constraints \\: CA=false\ncert. type        \\: SSL Server\n\"",
    "depends_on:0:1:2",
    "0:char*:\"data_files/server1.key_usage.crt\":char*:\"cert. version     \\: 3\nserial number     \\: 01\nissuer name       \\: C=NL, O=PolarSSL, CN=PolarSSL Test CA\nsubject name      \\: C=NL, O=PolarSSL, CN=PolarSSL Server 1\nissued  on        \\: 2011-02-12 14\\:44\\:06\nexpires on        \\: 2021-02-12 14\\:44\\:06\nsigned using      \\: RSA with SHA1\nRSA key size      \\: 2048 bits\nbasic constraints \\: CA=false\nkey usage         \\: Digital Signature, Non Repudiation, Key Encipherment\n\"",
    "depends_on:0:1:2",
    "0:char*:\"data_files/keyUsage.decipherOnly.crt\":char*:\"cert. version     \\: 3\nserial number     \\: 9B\\:13\\:CE\\:4C\\:A5\\:6F\\:DE\\:52\nissuer name       \\: C=GB, L=Cambridge, O=Default Company Ltd\nsubject name      \\: C=GB, L=Cambridge, O=Default Company Ltd\nissued  on        \\: 2015-05-12 10\\:36\\:55\nexpires on        \\: 2018-05-11 10\\:36\\:55\nsigned using      \\: RSA with SHA1\nRSA key size      \\: 1024 bits\nbasic constraints \\: CA=false\nkey usage         \\: Digital Signature, Non Repudiation, Key Encipherment, Decipher Only\n\"",
    "depends_on:0:1:2",
    "0:char*:\"data_files/cert_example_multi.crt\":char*:\"cert. version     \\: 3\nserial number     \\: 11\nissuer name       \\: C=NL, O=PolarSSL, CN=PolarSSL Test CA\nsubject name      \\: C=NL, O=PolarSSL, CN=www.example.com\nissued  on        \\: 2012-05-10 13\\:23\\:41\nexpires on        \\: 2022-05-11 13\\:23\\:41\nsigned using      \\: RSA with SHA1\nRSA key size      \\: 2048 bits\nbasic constraints \\: CA=false\nsubject alt name  \\: example.com, example.net, *.example.org\n\"",
    "depends_on:0:1:2",
    "0:char*:\"data_files/cert_example_multi_nocn.crt\":char*:\"cert. version     \\: 3\nserial number     \\: F7\\:C6\\:7F\\:F8\\:E9\\:A9\\:63\\:F9\nissuer name       \\: C=NL\nsubject name      \\: C=NL\nissued  on        \\: 2014-01-22 10\\:04\\:33\nexpires on        \\: 2024-01-22 10\\:04\\:33\nsigned using      \\: RSA with SHA1\nRSA key size      \\: 1024 bits\nbasic constraints \\: CA=false\nsubject alt name  \\: www.shotokan-braunschweig.de, www.massimo-abate.eu\nkey usage         \\: Digital Signature, Non Repudiation, Key Encipherment\n\"",
    "depends_on:0:1:6",
    "0:char*:\"data_files/server1.ext_ku.crt\":char*:\"cert. version     \\: 3\nserial number     \\: 21\nissuer name       \\: C=NL, O=PolarSSL, CN=PolarSSL Test CA\nsubject name      \\: C=NL, O=PolarSSL, CN=PolarSSL Server 1\nissued  on        \\: 2014-04-01 14\\:44\\:43\nexpires on        \\: 2024-03-29 14\\:44\\:43\nsigned using      \\: RSA with SHA-256\nRSA key size      \\: 2048 bits\nbasic constraints \\: CA=false\nkey usage         \\: Digital Signature, Non Repudiation, Key Encipherment\next key usage     \\: TLS Web Server Authentication\n\"",
    "depends_on:0:1:6:9",
    "0:char*:\"data_files/server4.crt\":char*:\"cert. version     \\: 3\nserial number     \\: 08\nissuer name       \\: C=NL, O=PolarSSL, CN=Polarssl Test EC CA\nsubject name      \\: C=NL, O=PolarSSL, CN=localhost\nissued  on        \\: 2013-09-24 15\\:52\\:04\nexpires on        \\: 2023-09-22 15\\:52\\:04\nsigned using      \\: ECDSA with SHA256\nRSA key size      \\: 2048 bits\nbasic constraints \\: CA=false\n\"",
    "depends_on:0:9:11:2:1",
    "0:char*:\"data_files/server3.crt\":char*:\"cert. version     \\: 3\nserial number     \\: 0D\nissuer name       \\: C=NL, O=PolarSSL, CN=PolarSSL Test CA\nsubject name      \\: C=NL, O=PolarSSL, CN=localhost\nissued  on        \\: 2013-08-09 09\\:17\\:03\nexpires on        \\: 2023-08-07 09\\:17\\:03\nsigned using      \\: RSA with SHA1\nEC key size       \\: 192 bits\nbasic constraints \\: CA=false\n\"",
    "depends_on:0:1:2",
    "0:char*:\"data_files/bitstring-in-dn.pem\":char*:\"cert. version     \\: 3\nserial number     \\: 02\nissuer name       \\: CN=Test CA 01, ST=Ecnivorp, C=XX, emailAddress=tca@example.com, O=Test CA Authority\nsubject name      \\: C=XX, O=tca, ST=Ecnivorp, OU=TCA, CN=Client, emailAddress=client@example.com, serialNumber=7101012255, uniqueIdentifier=?7101012255\nissued  on        \\: 2015-03-11 12\\:06\\:51\nexpires on        \\: 2025-03-08 12\\:06\\:51\nsigned using      \\: RSA with SHA1\nRSA key size      \\: 2048 bits\nbasic constraints \\: CA=false\nsubject alt name  \\: \next key usage     \\: TLS Web Client Authentication\n\"",
    "depends_on:0:1:12:2",
    "0:char*:\"data_files/cert_v1_with_ext.crt\":char*:\"cert. version     \\: 1\nserial number     \\: BD\\:ED\\:44\\:C7\\:D2\\:3E\\:C2\\:A4\nissuer name       \\: C=XX, ST=XX, L=XX, O=XX, OU=XX, emailAddress=admin@identity-check.org, CN=identity-check.org\nsubject name      \\: C=XX, ST=XX, L=XX, O=XX, OU=XX, emailAddress=admin@identity-check.org, CN=identity-check.org\nissued  on        \\: 2013-07-04 16\\:17\\:02\nexpires on        \\: 2014-07-04 16\\:17\\:02\nsigned using      \\: RSA with SHA1\nRSA key size      \\: 2048 bits\nsubject alt name  \\: identity-check.org, www.identity-check.org\n\"",
    "depends_on:0:2:1",
    "1:char*:\"data_files/crl_expired.pem\":char*:\"CRL version   \\: 1\nissuer name   \\: C=NL, O=PolarSSL, CN=PolarSSL Test CA\nthis update   \\: 2011-02-20 10\\:24\\:19\nnext update   \\: 2011-02-20 11\\:24\\:19\nRevoked certificates\\:\nserial number\\: 01 revocation date\\: 2011-02-12 14\\:44\\:07\nserial number\\: 03 revocation date\\: 2011-02-12 14\\:44\\:07\nsigned using  \\: RSA with SHA1\n\"",
    "depends_on:0:3:1",
    "1:char*:\"data_files/crl_md2.pem\":char*:\"CRL version   \\: 1\nissuer name   \\: C=NL, O=PolarSSL, CN=PolarSSL Test CA\nthis update   \\: 2009-07-19 19\\:56\\:37\nnext update   \\: 2009-09-17 19\\:56\\:37\nRevoked certificates\\:\nserial number\\: 01 revocation date\\: 2009-02-09 21\\:12\\:36\nserial number\\: 03 revocation date\\: 2009-02-09 21\\:12\\:36\nsigned using  \\: RSA with MD2\n\"",
    "depends_on:0:4",
    "1:char*:\"data_files/crl_md4.pem\":char*:\"CRL version   \\: 1\nissuer name   \\: C=NL, O=PolarSSL, CN=PolarSSL Test CA\nthis update   \\: 2011-02-12 14\\:44\\:07\nnext update   \\: 2011-04-13 14\\:44\\:07\nRevoked certificates\\:\nserial number\\: 01 revocation date\\: 2011-02-12 14\\:44\\:07\nserial number\\: 03 revocation date\\: 2011-02-12 14\\:44\\:07\nsigned using  \\: RSA with MD4\n\"",
    "depends_on:0:5:1",
    "1:char*:\"data_files/crl_md5.pem\":char*:\"CRL version   \\: 1\nissuer name   \\: C=NL, O=PolarSSL, CN=PolarSSL Test CA\nthis update   \\: 2011-02-12 14\\:44\\:07\nnext update   \\: 2011-04-13 14\\:44\\:07\nRevoked certificates\\:\nserial number\\: 01 revocation date\\: 2011-02-12 14\\:44\\:07\nserial number\\: 03 revocation date\\: 2011-02-12 14\\:44\\:07\nsigned using  \\: RSA with MD5\n\"",
    "depends_on:0:2:1",
    "1:char*:\"data_files/crl_sha1.pem\":char*:\"CRL version   \\: 1\nissuer name   \\: C=NL, O=PolarSSL, CN=PolarSSL Test CA\nthis update   \\: 2011-02-12 14\\:44\\:07\nnext update   \\: 2011-04-13 14\\:44\\:07\nRevoked certificates\\:\nserial number\\: 01 revocation date\\: 2011-02-12 14\\:44\\:07\nserial number\\: 03 revocation date\\: 2011-02-12 14\\:44\\:07\nsigned using  \\: RSA with SHA1\n\"",
    "depends_on:0:6:1",
    "1:char*:\"data_files/crl_sha224.pem\":char*:\"CRL version   \\: 1\nissuer name   \\: C=NL, O=PolarSSL, CN=PolarSSL Test CA\nthis update   \\: 2011-02-12 14\\:44\\:07\nnext update   \\: 2011-04-13 14\\:44\\:07\nRevoked certificates\\:\nserial number\\: 01 revocation date\\: 2011-02-12 14\\:44\\:07\nserial number\\: 03 revocation date\\: 2011-02-12 14\\:44\\:07\nsigned using  \\: RSA with SHA-224\n\"",
    "depends_on:0:6:1",
    "1:char*:\"data_files/crl_sha256.pem\":char*:\"CRL version   \\: 1\nissuer name   \\: C=NL, O=PolarSSL, CN=PolarSSL Test CA\nthis update   \\: 2011-02-12 14\\:44\\:07\nnext update   \\: 2011-04-13 14\\:44\\:07\nRevoked certificates\\:\nserial number\\: 01 revocation date\\: 2011-02-12 14\\:44\\:07\nserial number\\: 03 revocation date\\: 2011-02-12 14\\:44\\:07\nsigned using  \\: RSA with SHA-256\n\"",
    "depends_on:0:7:1",
    "1:char*:\"data_files/crl_sha384.pem\":char*:\"CRL version   \\: 1\nissuer name   \\: C=NL, O=PolarSSL, CN=PolarSSL Test CA\nthis update   \\: 2011-02-12 14\\:44\\:07\nnext update   \\: 2011-04-13 14\\:44\\:07\nRevoked certificates\\:\nserial number\\: 01 revocation date\\: 2011-02-12 14\\:44\\:07\nserial number\\: 03 revocation date\\: 2011-02-12 14\\:44\\:07\nsigned using  \\: RSA with SHA-384\n\"",
    "depends_on:0:7:1",
    "1:char*:\"data_files/crl_sha512.pem\":char*:\"CRL version   \\: 1\nissuer name   \\: C=NL, O=PolarSSL, CN=PolarSSL Test CA\nthis update   \\: 2011-02-12 14\\:44\\:07\nnext update   \\: 2011-04-13 14\\:44\\:07\nRevoked certificates\\:\nserial number\\: 01 revocation date\\: 2011-02-12 14\\:44\\:07\nserial number\\: 03 revocation date\\: 2011-02-12 14\\:44\\:07\nsigned using  \\: RSA with SHA-512\n\"",
    "depends_on:0:8:2",
    "1:char*:\"data_files/crl-rsa-pss-sha1.pem\":char*:\"CRL version   \\: 2\nissuer name   \\: C=NL, O=PolarSSL, CN=PolarSSL Test CA\nthis update   \\: 2014-01-20 13\\:46\\:35\nnext update   \\: 2024-01-18 13\\:46\\:35\nRevoked certificates\\:\nserial number\\: 0A revocation date\\: 2013-09-24 16\\:28\\:38\nserial number\\: 16 revocation date\\: 2014-01-20 13\\:43\\:05\nsigned using  \\: RSASSA-PSS (SHA1, MGF1-SHA1, 0xEA)\n\"",
    "depends_on:0:8:6",
    "1:char*:\"data_files/crl-rsa-pss-sha224.pem\":char*:\"CRL version   \\: 2\nissuer name   \\: C=NL, O=PolarSSL, CN=PolarSSL Test CA\nthis update   \\: 2014-01-20 13\\:56\\:06\nnext update   \\: 2024-01-18 13\\:56\\:06\nRevoked certificates\\:\nserial number\\: 0A revocation date\\: 2013-09-24 16\\:28\\:38\nserial number\\: 16 revocation date\\: 2014-01-20 13\\:43\\:05\nsigned using  \\: RSASSA-PSS (SHA224, MGF1-SHA224, 0xE2)\n\"",
    "depends_on:0:8:6",
    "1:char*:\"data_files/crl-rsa-pss-sha256.pem\":char*:\"CRL version   \\: 2\nissuer name   \\: C=NL, O=PolarSSL, CN=PolarSSL Test CA\nthis update   \\: 2014-01-20 13\\:56\\:16\nnext update   \\: 2024-01-18 13\\:56\\:16\nRevoked certificates\\:\nserial number\\: 0A revocation date\\: 2013-09-24 16\\:28\\:38\nserial number\\: 16 revocation date\\: 2014-01-20 13\\:43\\:05\nsigned using  \\: RSASSA-PSS (SHA256, MGF1-SHA256, 0xDE)\n\"",
    "depends_on:0:8:7",
    "1:char*:\"data_files/crl-rsa-pss-sha384.pem\":char*:\"CRL version   \\: 2\nissuer name   \\: C=NL, O=PolarSSL, CN=PolarSSL Test CA\nthis update   \\: 2014-01-20 13\\:56\\:28\nnext update   \\: 2024-01-18 13\\:56\\:28\nRevoked certificates\\:\nserial number\\: 0A revocation date\\: 2013-09-24 16\\:28\\:38\nserial number\\: 16 revocation date\\: 2014-01-20 13\\:43\\:05\nsigned using  \\: RSASSA-PSS (SHA384, MGF1-SHA384, 0xCE)\n\"",
    "depends_on:0:8:7",
    "1:char*:\"data_files/crl-rsa-pss-sha512.pem\":char*:\"CRL version   \\: 2\nissuer name   \\: C=NL, O=PolarSSL, CN=PolarSSL Test CA\nthis update   \\: 2014-01-20 13\\:56\\:38\nnext update   \\: 2024-01-18 13\\:56\\:38\nRevoked certificates\\:\nserial number\\: 0A revocation date\\: 2013-09-24 16\\:28\\:38\nserial number\\: 16 revocation date\\: 2014-01-20 13\\:43\\:05\nsigned using  \\: RSASSA-PSS (SHA512, MGF1-SHA512, 0xBE)\n\"",
    "depends_on:0:2:9",
    "1:char*:\"data_files/crl-ec-sha1.pem\":char*:\"CRL version   \\: 2\nissuer name   \\: C=NL, O=PolarSSL, CN=Polarssl Test EC CA\nthis update   \\: 2013-09-24 16\\:31\\:08\nnext update   \\: 2023-09-22 16\\:31\\:08\nRevoked certificates\\:\nserial number\\: 0A revocation date\\: 2013-09-24 16\\:28\\:38\nsigned using  \\: ECDSA with SHA1\n\"",
    "depends_on:0:6:9",
    "1:char*:\"data_files/crl-ec-sha224.pem\":char*:\"CRL version   \\: 2\nissuer name   \\: C=NL, O=PolarSSL, CN=Polarssl Test EC CA\nthis update   \\: 2013-09-24 16\\:31\\:08\nnext update   \\: 2023-09-22 16\\:31\\:08\nRevoked certificates\\:\nserial number\\: 0A revocation date\\: 2013-09-24 16\\:28\\:38\nsigned using  \\: ECDSA with SHA224\n\"",
    "depends_on:0:6:9",
    "1:char*:\"data_files/crl-ec-sha256.pem\":char*:\"CRL version   \\: 2\nissuer name   \\: C=NL, O=PolarSSL, CN=Polarssl Test EC CA\nthis update   \\: 2013-09-24 16\\:31\\:08\nnext update   \\: 2023-09-22 16\\:31\\:08\nRevoked certificates\\:\nserial number\\: 0A revocation date\\: 2013-09-24 16\\:28\\:38\nsigned using  \\: ECDSA with SHA256\n\"",
    "depends_on:0:7:9",
    "1:char*:\"data_files/crl-ec-sha384.pem\":char*:\"CRL version   \\: 2\nissuer name   \\: C=NL, O=PolarSSL, CN=Polarssl Test EC CA\nthis update   \\: 2013-09-24 16\\:31\\:08\nnext update   \\: 2023-09-22 16\\:31\\:08\nRevoked certificates\\:\nserial number\\: 0A revocation date\\: 2013-09-24 16\\:28\\:38\nsigned using  \\: ECDSA with SHA384\n\"",
    "depends_on:0:7:9",
    "1:char*:\"data_files/crl-ec-sha512.pem\":char*:\"CRL version   \\: 2\nissuer name   \\: C=NL, O=PolarSSL, CN=Polarssl Test EC CA\nthis update   \\: 2013-09-24 16\\:31\\:08\nnext update   \\: 2023-09-22 16\\:31\\:08\nRevoked certificates\\:\nserial number\\: 0A revocation date\\: 2013-09-24 16\\:28\\:38\nsigned using  \\: ECDSA with SHA512\n\"",
    "depends_on:0:2:7:9",
    "2:char*:\"data_files/crl-malformed-trailing-spaces.pem\":exp:0",
    "depends_on:0:1:6",
    "2:char*:\"data_files/crl-idp.pem\":exp:1",
    "depends_on:0:1:6",
    "2:char*:\"data_files/crl-idpnc.pem\":int:0",
    "depends_on:0:4:1",
    "3:char*:\"data_files/server1.req.md4\":char*:\"CSR version   \\: 1\nsubject name  \\: C=NL, O=PolarSSL, CN=PolarSSL Server 1\nsigned using  \\: RSA with MD4\nRSA key size  \\: 2048 bits\n\"",
    "depends_on:0:5:1",
    "3:char*:\"data_files/server1.req.md5\":char*:\"CSR version   \\: 1\nsubject name  \\: C=NL, O=PolarSSL, CN=PolarSSL Server 1\nsigned using  \\: RSA with MD5\nRSA key size  \\: 2048 bits\n\"",
    "depends_on:0:2:1",
    "3:char*:\"data_files/server1.req.sha1\":char*:\"CSR version   \\: 1\nsubject name  \\: C=NL, O=PolarSSL, CN=PolarSSL Server 1\nsigned using  \\: RSA with SHA1\nRSA key size  \\: 2048 bits\n\"",
    "depends_on:0:6:1",
    "3:char*:\"data_files/server1.req.sha224\":char*:\"CSR version   \\: 1\nsubject name  \\: C=NL, O=PolarSSL, CN=PolarSSL Server 1\nsigned using  \\: RSA with SHA-224\nRSA key size  \\: 2048 bits\n\"",
    "depends_on:0:6:1",
    "3:char*:\"data_files/server1.req.sha256\":char*:\"CSR version   \\: 1\nsubject name  \\: C=NL, O=PolarSSL, CN=PolarSSL Server 1\nsigned using  \\: RSA with SHA-256\nRSA key size  \\: 2048 bits\n\"",
    "depends_on:0:7:1",
    "3:char*:\"data_files/server1.req.sha384\":char*:\"CSR version   \\: 1\nsubject name  \\: C=NL, O=PolarSSL, CN=PolarSSL Server 1\nsigned using  \\: RSA with SHA-384\nRSA key size  \\: 2048 bits\n\"",
    "depends_on:0:7:1",
    "3:char*:\"data_files/server1.req.sha512\":char*:\"CSR version   \\: 1\nsubject name  \\: C=NL, O=PolarSSL, CN=PolarSSL Server 1\nsigned using  \\: RSA with SHA-512\nRSA key size  \\: 2048 bits\n\"",
    "depends_on:9:0:10:2",
    "3:char*:\"data_files/server5.req.sha1\":char*:\"CSR version   \\: 1\nsubject name  \\: C=NL, O=PolarSSL, CN=localhost\nsigned using  \\: ECDSA with SHA1\nEC key size   \\: 256 bits\n\"",
    "depends_on:9:0:10:6",
    "3:char*:\"data_files/server5.req.sha224\":char*:\"CSR version   \\: 1\nsubject name  \\: C=NL, O=PolarSSL, CN=localhost\nsigned using  \\: ECDSA with SHA224\nEC key size   \\: 256 bits\n\"",
    "depends_on:9:0:10:6",
    "3:char*:\"data_files/server5.req.sha256\":char*:\"CSR version   \\: 1\nsubject name  \\: C=NL, O=PolarSSL, CN=localhost\nsigned using  \\: ECDSA with SHA256\nEC key size   \\: 256 bits\n\"",
    "depends_on:9:0:10:7",
    "3:char*:\"data_files/server5.req.sha384\":char*:\"CSR version   \\: 1\nsubject name  \\: C=NL, O=PolarSSL, CN=localhost\nsigned using  \\: ECDSA with SHA384\nEC key size   \\: 256 bits\n\"",
    "depends_on:9:0:10:7",
    "3:char*:\"data_files/server5.req.sha512\":char*:\"CSR version   \\: 1\nsubject name  \\: C=NL, O=PolarSSL, CN=localhost\nsigned using  \\: ECDSA with SHA512\nEC key size   \\: 256 bits\n\"",
    "depends_on:0:8:2",
    "3:char*:\"data_files/server9.req.sha1\":char*:\"CSR version   \\: 1\nsubject name  \\: C=NL, O=PolarSSL, CN=localhost\nsigned using  \\: RSASSA-PSS (SHA1, MGF1-SHA1, 0x6A)\nRSA key size  \\: 1024 bits\n\"",
    "depends_on:0:8:6",
    "3:char*:\"data_files/server9.req.sha224\":char*:\"CSR version   \\: 1\nsubject name  \\: C=NL, O=PolarSSL, CN=localhost\nsigned using  \\: RSASSA-PSS (SHA224, MGF1-SHA224, 0x62)\nRSA key size  \\: 1024 bits\n\"",
    "depends_on:0:8:6",
    "3:char*:\"data_files/server9.req.sha256\":char*:\"CSR version   \\: 1\nsubject name  \\: C=NL, O=PolarSSL, CN=localhost\nsigned using  \\: RSASSA-PSS (SHA256, MGF1-SHA256, 0x5E)\nRSA key size  \\: 1024 bits\n\"",
    "depends_on:0:8:7",
    "3:char*:\"data_files/server9.req.sha384\":char*:\"CSR version   \\: 1\nsubject name  \\: C=NL, O=PolarSSL, CN=localhost\nsigned using  \\: RSASSA-PSS (SHA384, MGF1-SHA384, 0x4E)\nRSA key size  \\: 1024 bits\n\"",
    "depends_on:0:8:7",
    "3:char*:\"data_files/server9.req.sha512\":char*:\"CSR version   \\: 1\nsubject name  \\: C=NL, O=PolarSSL, CN=localhost\nsigned using  \\: RSASSA-PSS (SHA512, MGF1-SHA512, 0x3E)\nRSA key size  \\: 1024 bits\n\"",
    "4:int:0:char*:\"\":char*:\"\"",
    "4:exp:2:char*:\"\":char*:\"Certificate was missing\n\"",
    "4:exp:3:char*:\"\":char*:\"The certificate validity has expired\nThe CRL is expired\n\"",
    "4:exp:4:char*:\"\":char*:\"Other reason (can be used by verify callback)\nUnknown reason (this should not happen)\n\"",
    "4:int:0:char*:\"  ! \":char*:\"\"",
    "4:exp:2:char*:\"  ! \":char*:\"  ! Certificate was missing\n\"",
    "4:exp:3:char*:\"  ! \":char*:\"  ! The certificate validity has expired\n  ! The CRL is expired\n\"",
    "depends_on:0:1:2",
    "7:char*:\"data_files/server1.crt\":char*:\"subject\":char*:\"C=NL, O=PolarSSL, CN=PolarSSL Server 1\"",
    "depends_on:0:1:2",
    "7:char*:\"data_files/server1.crt\":char*:\"issuer\":char*:\"C=NL, O=PolarSSL, CN=PolarSSL Test CA\"",
    "depends_on:0:1:2",
    "7:char*:\"data_files/server2.crt\":char*:\"subject\":char*:\"C=NL, O=PolarSSL, CN=localhost\"",
    "depends_on:0:1:2",
    "7:char*:\"data_files/server2.crt\":char*:\"issuer\":char*:\"C=NL, O=PolarSSL, CN=PolarSSL Test CA\"",
    "depends_on:0:1:13:2",
    "8:char*:\"data_files/server1.crt\":char*:\"valid_from\":int:1",
    "depends_on:0:1:13:2",
    "8:char*:\"data_files/server1.crt\":char*:\"valid_to\":int:0",
    "depends_on:0:1:13:2",
    "8:char*:\"data_files/server2.crt\":char*:\"valid_from\":int:1",
    "depends_on:0:1:13:2",
    "8:char*:\"data_files/server2.crt\":char*:\"valid_to\":int:0",
    "depends_on:0:1:13:2",
    "8:char*:\"data_files/test-ca.crt\":char*:\"valid_from\":int:1",
    "depends_on:0:1:13:2",
    "8:char*:\"data_files/test-ca.crt\":char*:\"valid_to\":int:0",
    "depends_on:0:9:10:13:6",
    "9:char*:\"data_files/server5.crt\":char*:\"valid_from\":int:0",
    "depends_on:0:9:10:13:6",
    "9:char*:\"data_files/server5.crt\":char*:\"valid_to\":int:1",
    "depends_on:0:9:10:13:6",
    "9:char*:\"data_files/server5-future.crt\":char*:\"valid_from\":int:1",
    "depends_on:0:9:10:13:6",
    "9:char*:\"data_files/server5-future.crt\":char*:\"valid_to\":int:1",
    "depends_on:0:9:14:13:6",
    "9:char*:\"data_files/test-ca2.crt\":char*:\"valid_from\":int:0",
    "depends_on:0:9:14:13:6",
    "9:char*:\"data_files/test-ca2.crt\":char*:\"valid_to\":int:1",
    "depends_on:0:2:1:15:13",
    "5:char*:\"data_files/server1.crt\":char*:\"data_files/test-ca.crt\":char*:\"data_files/crl_expired.pem\":char*:\"NULL\":exp:5:exp:6:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:6:9:10:14:2:13",
    "5:char*:\"data_files/server6.crt\":char*:\"data_files/test-ca2.crt\":char*:\"data_files/crl-future.pem\":char*:\"NULL\":exp:5:exp:7:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:2:1:15:13",
    "5:char*:\"data_files/server1.crt\":char*:\"data_files/test-ca.crt\":char*:\"data_files/crl_expired.pem\":char*:\"PolarSSL Server 1\":exp:5:exp:6:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:6:9:10:14:2:13",
    "5:char*:\"data_files/server6.crt\":char*:\"data_files/test-ca2.crt\":char*:\"data_files/crl-future.pem\":char*:\"localhost\":exp:5:exp:7:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:2:1:15:13",
    "5:char*:\"data_files/server1.crt\":char*:\"data_files/test-ca.crt\":char*:\"data_files/crl_expired.pem\":char*:\"PolarSSL Wrong CN\":exp:5:exp:8:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:6:9:10:14:2:13",
    "5:char*:\"data_files/server6.crt\":char*:\"data_files/test-ca2.crt\":char*:\"data_files/crl-future.pem\":char*:\"Wrong CN\":exp:5:exp:9:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:2:1:15:13",
    "5:char*:\"data_files/server2.crt\":char*:\"data_files/test-ca.crt\":char*:\"data_files/crl_expired.pem\":char*:\"NULL\":exp:5:exp:10:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:6:9:10:14:2:13",
    "5:char*:\"data_files/server5.crt\":char*:\"data_files/test-ca2.crt\":char*:\"data_files/crl-future.pem\":char*:\"NULL\":exp:5:exp:11:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:2:1:15:13",
    "5:char*:\"data_files/server1.crt\":char*:\"data_files/test-ca.crt\":char*:\"data_files/crl.pem\":char*:\"NULL\":exp:5:exp:12:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:2:1:15:13",
    "5:char*:\"data_files/server1.crt\":char*:\"data_files/test-ca.crt\":char*:\"data_files/crl.pem\":char*:\"PolarSSL Server 1\":exp:5:exp:12:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:2:1:15:13",
    "5:char*:\"data_files/server1.crt\":char*:\"data_files/test-ca.crt\":char*:\"data_files/crl.pem\":char*:\"PolarSSL Wrong CN\":exp:5:exp:13:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:6:9:10:14:2",
    "5:char*:\"data_files/server5.crt\":char*:\"data_files/test-ca2.crt\":char*:\"data_files/crl-ec-sha1.pem\":char*:\"NULL\":int:0:int:0:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:6:9:10:14:2:13",
    "5:char*:\"data_files/server5-expired.crt\":char*:\"data_files/test-ca2.crt\":char*:\"data_files/crl-ec-sha1.pem\":char*:\"NULL\":exp:5:exp:14:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:6:9:10:14:2:13",
    "5:char*:\"data_files/server5-future.crt\":char*:\"data_files/test-ca2.crt\":char*:\"data_files/crl-ec-sha1.pem\":char*:\"NULL\":exp:5:exp:15:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:6:9:10:14:2:1:15:13",
    "5:char*:\"data_files/server7-expired.crt\":char*:\"data_files/test-ca2.crt\":char*:\"data_files/crl-ec-sha1.pem\":char*:\"NULL\":exp:5:exp:14:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:6:9:10:14:2:1:15:13",
    "5:char*:\"data_files/server7-future.crt\":char*:\"data_files/test-ca2.crt\":char*:\"data_files/crl-ec-sha1.pem\":char*:\"NULL\":exp:5:exp:15:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:2:1:15",
    "5:char*:\"data_files/server2.crt\":char*:\"data_files/server1.crt\":char*:\"data_files/crl.pem\":char*:\"NULL\":exp:5:exp:16:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:1:15:2",
    "5:char*:\"data_files/server2.crt\":char*:\"data_files/server1.crt\":char*:\"data_files/crl_expired.pem\":char*:\"NULL\":exp:5:exp:16:char*:\"compat\":char*:\"NULL\"",
    "depends_on:4:0:2:1:15",
    "5:char*:\"data_files/cert_md4.crt\":char*:\"data_files/test-ca.crt\":char*:\"data_files/crl.pem\":char*:\"NULL\":exp:5:exp:17:char*:\"compat\":char*:\"NULL\"",
    "depends_on:5:0:2:1:15",
    "5:char*:\"data_files/cert_md5.crt\":char*:\"data_files/test-ca.crt\":char*:\"data_files/crl.pem\":char*:\"NULL\":exp:5:exp:17:char*:\"compat\":char*:\"NULL\"",
    "depends_on:2:0:2:1:15",
    "5:char*:\"data_files/cert_sha1.crt\":char*:\"data_files/test-ca.crt\":char*:\"data_files/crl.pem\":char*:\"NULL\":int:0:int:0:char*:\"compat\":char*:\"NULL\"",
    "depends_on:2:0:2:1:15:16",
    "5:char*:\"data_files/cert_sha1.crt\":char*:\"data_files/test-ca.crt\":char*:\"data_files/crl.pem\":char*:\"NULL\":int:0:int:0:char*:\"\":char*:\"NULL\"",
    "depends_on:2:0:2:1:15:17",
    "5:char*:\"data_files/cert_sha1.crt\":char*:\"data_files/test-ca.crt\":char*:\"data_files/crl.pem\":char*:\"NULL\":exp:5:exp:18:char*:\"\":char*:\"NULL\"",
    "depends_on:6:0:2:1:15",
    "5:char*:\"data_files/cert_sha224.crt\":char*:\"data_files/test-ca.crt\":char*:\"data_files/crl.pem\":char*:\"NULL\":int:0:int:0:char*:\"compat\":char*:\"NULL\"",
    "depends_on:6:0:2:1:15",
    "5:char*:\"data_files/cert_sha256.crt\":char*:\"data_files/test-ca.crt\":char*:\"data_files/crl.pem\":char*:\"NULL\":int:0:int:0:char*:\"compat\":char*:\"NULL\"",
    "depends_on:7:0:2:1:15",
    "5:char*:\"data_files/cert_sha384.crt\":char*:\"data_files/test-ca.crt\":char*:\"data_files/crl.pem\":char*:\"NULL\":int:0:int:0:char*:\"compat\":char*:\"NULL\"",
    "depends_on:7:0:2:1:15",
    "5:char*:\"data_files/cert_sha512.crt\":char*:\"data_files/test-ca.crt\":char*:\"data_files/crl.pem\":char*:\"NULL\":int:0:int:0:char*:\"compat\":char*:\"NULL\"",
    "depends_on:7:0:2:1:15",
    "5:char*:\"data_files/cert_sha512.crt\":char*:\"data_files/test-ca.crt\":char*:\"data_files/crl.pem\":char*:\"NULL\":exp:5:exp:19:char*:\"compat\":char*:\"verify_none\"",
    "depends_on:0:1:15:2",
    "5:char*:\"data_files/server2.crt\":char*:\"data_files/server1.crt\":char*:\"data_files/crl_expired.pem\":char*:\"NULL\":int:0:int:0:char*:\"compat\":char*:\"verify_all\"",
    "depends_on:0:2:1:15",
    "5:char*:\"data_files/cert_example_wildcard.crt\":char*:\"data_files/test-ca.crt\":char*:\"data_files/crl.pem\":char*:\"mail.ExAmPlE.com\":int:0:int:0:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:2:1:15",
    "5:char*:\"data_files/cert_example_wildcard.crt\":char*:\"data_files/test-ca.crt\":char*:\"data_files/crl.pem\":char*:\"mail.example.net\":exp:5:exp:20:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:2:1:15",
    "5:char*:\"data_files/cert_example_wildcard.crt\":char*:\"data_files/test-ca.crt\":char*:\"data_files/crl.pem\":char*:\"example.com\":exp:5:exp:20:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:2:1:15",
    "5:char*:\"data_files/cert_example_multi.crt\":char*:\"data_files/test-ca.crt\":char*:\"data_files/crl.pem\":char*:\"www.example.com\":exp:5:exp:20:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:2:1:15",
    "5:char*:\"data_files/cert_example_multi.crt\":char*:\"data_files/test-ca.crt\":char*:\"data_files/crl.pem\":char*:\"example.net\":int:0:int:0:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:2:1:15",
    "5:char*:\"data_files/cert_example_multi.crt\":char*:\"data_files/test-ca.crt\":char*:\"data_files/crl.pem\":char*:\"www.example.net\":exp:5:exp:20:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:2:1:15",
    "5:char*:\"data_files/cert_example_multi.crt\":char*:\"data_files/test-ca.crt\":char*:\"data_files/crl.pem\":char*:\"xample.net\":exp:5:exp:20:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:2:1:15",
    "5:char*:\"data_files/cert_example_multi.crt\":char*:\"data_files/test-ca.crt\":char*:\"data_files/crl.pem\":char*:\"bexample.net\":exp:5:exp:20:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:2:1:15",
    "5:char*:\"data_files/cert_example_multi.crt\":char*:\"data_files/test-ca.crt\":char*:\"data_files/crl.pem\":char*:\"example.org\":exp:5:exp:20:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:2:1:15",
    "5:char*:\"data_files/cert_example_multi.crt\":char*:\"data_files/test-ca.crt\":char*:\"data_files/crl.pem\":char*:\"mail.example.org\":int:0:int:0:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:2:1:15",
    "5:char*:\"data_files/cert_example_multi_nocn.crt\":char*:\"data_files/test-ca.crt\":char*:\"data_files/crl.pem\":char*:\"www.shotokan-braunschweig.de\":exp:5:exp:16:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:2:1:15",
    "5:char*:\"data_files/cert_example_multi_nocn.crt\":char*:\"data_files/test-ca.crt\":char*:\"data_files/crl.pem\":char*:\"www.example.net\":exp:5:exp:21:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:1:9:11:1:15:2",
    "5:char*:\"data_files/server3.crt\":char*:\"data_files/test-ca.crt\":char*:\"data_files/crl.pem\":char*:\"NULL\":int:0:int:0:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:1:9:6:10:1:15:14",
    "5:char*:\"data_files/server4.crt\":char*:\"data_files/test-ca2.crt\":char*:\"data_files/crl-ec-sha256.pem\":char*:\"NULL\":int:0:int:0:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:9:6:10:14",
    "5:char*:\"data_files/server5.crt\":char*:\"data_files/test-ca2.crt\":char*:\"data_files/crl-ec-sha256.pem\":char*:\"NULL\":int:0:int:0:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:9:6:10:14:13",
    "5:char*:\"data_files/server6.crt\":char*:\"data_files/test-ca2.crt\":char*:\"data_files/crl-ec-sha256.pem\":char*:\"NULL\":exp:5:exp:12:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:9:6:10:14:2",
    "5:char*:\"data_files/server5-sha1.crt\":char*:\"data_files/test-ca2.crt\":char*:\"data_files/crl-ec-sha256.pem\":char*:\"NULL\":int:0:int:0:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:9:6:10:14",
    "5:char*:\"data_files/server5-sha224.crt\":char*:\"data_files/test-ca2.crt\":char*:\"data_files/crl-ec-sha256.pem\":char*:\"NULL\":int:0:int:0:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:9:6:7:10:14",
    "5:char*:\"data_files/server5-sha384.crt\":char*:\"data_files/test-ca2.crt\":char*:\"data_files/crl-ec-sha256.pem\":char*:\"NULL\":int:0:int:0:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:9:6:7:10:14",
    "5:char*:\"data_files/server5-sha512.crt\":char*:\"data_files/test-ca2.crt\":char*:\"data_files/crl-ec-sha256.pem\":char*:\"NULL\":int:0:int:0:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:1:15:6:2",
    "5:char*:\"data_files/test-ca.crt\":char*:\"data_files/test-ca.crt\":char*:\"data_files/crl.pem\":char*:\"NULL\":int:0:int:0:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:9:10:14:6",
    "5:char*:\"data_files/test-ca2.crt\":char*:\"data_files/test-ca2.crt\":char*:\"data_files/crl-ec-sha256.pem\":char*:\"NULL\":int:0:int:0:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:1:15:2",
    "5:char*:\"data_files/server2.crt\":char*:\"data_files/server2.crt\":char*:\"data_files/crl.pem\":char*:\"NULL\":exp:5:exp:16:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:9:10:6",
    "5:char*:\"data_files/server5.crt\":char*:\"data_files/server5.crt\":char*:\"data_files/crl-ec-sha256.pem\":char*:\"NULL\":exp:5:exp:16:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:9:10:14:6",
    "5:char*:\"data_files/server5-badsign.crt\":char*:\"data_files/test-ca2.crt\":char*:\"data_files/crl-ec-sha256.pem\":char*:\"NULL\":exp:5:exp:16:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:1:15:2",
    "5:char*:\"data_files/server2-badsign.crt\":char*:\"data_files/test-ca.crt\":char*:\"data_files/crl.pem\":char*:\"NULL\":exp:5:exp:16:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:1:15:2:9:10:14:6",
    "5:char*:\"data_files/server7-badsign.crt\":char*:\"data_files/test-ca2.crt\":char*:\"data_files/crl.pem\":char*:\"NULL\":exp:5:exp:16:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:9:1:10:14:1:15:6",
    "5:char*:\"data_files/server7_int-ca.crt\":char*:\"data_files/test-ca2.crt\":char*:\"data_files/crl-ec-sha256.pem\":char*:\"NULL\":int:0:int:0:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:9:1:10:1:15:2:6",
    "5:char*:\"data_files/server7_int-ca.crt\":char*:\"data_files/test-ca.crt\":char*:\"data_files/crl-ec-sha256.pem\":char*:\"NULL\":exp:5:exp:16:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:9:1:10:1:15:2:6",
    "5:char*:\"data_files/server7.crt\":char*:\"data_files/test-ca.crt\":char*:\"data_files/crl-ec-sha256.pem\":char*:\"NULL\":exp:5:exp:16:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:9:1:14:1:15:6:2",
    "5:char*:\"data_files/server8_int-ca2.crt\":char*:\"data_files/test-ca.crt\":char*:\"data_files/crl-ec-sha256.pem\":char*:\"NULL\":int:0:int:0:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:1:15:9:14:2:6",
    "5:char*:\"data_files/server2.crt\":char*:\"data_files/test-ca_cat12.crt\":char*:\"data_files/crl.pem\":char*:\"NULL\":int:0:int:0:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:1:15:9:14:2:6",
    "5:char*:\"data_files/server2.crt\":char*:\"data_files/test-ca_cat21.crt\":char*:\"data_files/crl.pem\":char*:\"NULL\":int:0:int:0:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:9:6:10:14",
    "5:char*:\"data_files/server5.crt\":char*:\"data_files/test-ca2.ku-crt_crl.crt\":char*:\"data_files/crl-ec-sha256.pem\":char*:\"NULL\":int:0:int:0:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:9:6:18:10:14",
    "5:char*:\"data_files/server5.crt\":char*:\"data_files/test-ca2.ku-crt.crt\":char*:\"data_files/crl-ec-sha256.pem\":char*:\"NULL\":exp:5:exp:22:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:9:6:10:14:2:1",
    "5:char*:\"data_files/server5.crt\":char*:\"data_files/test-ca2.ku-crt.crt\":char*:\"data_files/crl.pem\":char*:\"NULL\":int:0:int:0:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:9:6:18:10:14",
    "5:char*:\"data_files/server5.crt\":char*:\"data_files/test-ca2.ku-crl.crt\":char*:\"data_files/crl-ec-sha256.pem\":char*:\"NULL\":exp:5:exp:16:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:9:6:18:10:14",
    "5:char*:\"data_files/server5.crt\":char*:\"data_files/test-ca2.ku-ds.crt\":char*:\"data_files/crl-ec-sha256.pem\":char*:\"NULL\":exp:5:exp:16:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:8:2:15",
    "5:char*:\"data_files/server9.crt\":char*:\"data_files/test-ca.crt\":char*:\"data_files/crl.pem\":char*:\"NULL\":int:0:int:0:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:8:6:2",
    "5:char*:\"data_files/server9-sha224.crt\":char*:\"data_files/test-ca.crt\":char*:\"data_files/crl-rsa-pss-sha224.pem\":char*:\"NULL\":int:0:int:0:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:8:6:2",
    "5:char*:\"data_files/server9-sha256.crt\":char*:\"data_files/test-ca.crt\":char*:\"data_files/crl-rsa-pss-sha256.pem\":char*:\"NULL\":int:0:int:0:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:8:7:2",
    "5:char*:\"data_files/server9-sha384.crt\":char*:\"data_files/test-ca.crt\":char*:\"data_files/crl-rsa-pss-sha384.pem\":char*:\"NULL\":int:0:int:0:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:8:7:2",
    "5:char*:\"data_files/server9-sha512.crt\":char*:\"data_files/test-ca.crt\":char*:\"data_files/crl-rsa-pss-sha512.pem\":char*:\"NULL\":int:0:int:0:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:8:2:13",
    "5:char*:\"data_files/server9.crt\":char*:\"data_files/test-ca.crt\":char*:\"data_files/crl-rsa-pss-sha1.pem\":char*:\"NULL\":exp:5:exp:12:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:8:2",
    "5:char*:\"data_files/server9.crt\":char*:\"data_files/test-ca.crt\":char*:\"data_files/crl-rsa-pss-sha1-badsign.pem\":char*:\"NULL\":exp:5:exp:22:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:8:2:1:15",
    "5:char*:\"data_files/server9-with-ca.crt\":char*:\"data_files/test-ca.crt\":char*:\"data_files/crl.pem\":char*:\"NULL\":int:0:int:0:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:8:2",
    "5:char*:\"data_files/server9-badsign.crt\":char*:\"data_files/test-ca.crt\":char*:\"data_files/crl.pem\":char*:\"NULL\":exp:5:exp:16:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:8:2:9:14:6",
    "5:char*:\"data_files/server9.crt\":char*:\"data_files/test-ca2.crt\":char*:\"data_files/crl.pem\":char*:\"NULL\":exp:5:exp:16:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:8:2",
    "5:char*:\"data_files/server9-defaults.crt\":char*:\"data_files/test-ca.crt\":char*:\"data_files/crl-rsa-pss-sha1.pem\":char*:\"NULL\":int:0:int:0:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:8:6:2",
    "5:char*:\"data_files/server9-bad-saltlen.crt\":char*:\"data_files/test-ca.crt\":char*:\"data_files/crl.pem\":char*:\"NULL\":exp:5:exp:16:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:8:6:2",
    "5:char*:\"data_files/server9-bad-mgfhash.crt\":char*:\"data_files/test-ca.crt\":char*:\"data_files/crl.pem\":char*:\"NULL\":exp:5:exp:16:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:1:15:6:2",
    "5:char*:\"data_files/server1-v1.crt\":char*:\"data_files/test-ca-v1.crt\":char*:\"data_files/crl.pem\":char*:\"NULL\":int:0:int:0:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:1:15:6:2",
    "5:char*:\"data_files/server2-v1.crt\":char*:\"data_files/server1-v1.crt\":char*:\"data_files/crl.pem\":char*:\"NULL\":int:0:int:0:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:1:15:6:2",
    "5:char*:\"data_files/server2-v1-chain.crt\":char*:\"data_files/test-ca-v1.crt\":char*:\"data_files/crl.pem\":char*:\"NULL\":exp:5:exp:16:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:9:6:10:2:1",
    "5:char*:\"data_files/server5-selfsigned.crt\":char*:\"data_files/server5-selfsigned.crt\":char*:\"data_files/crl.pem\":char*:\"NULL\":int:0:int:0:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:9:6:10:2:1",
    "5:char*:\"data_files/server6-ss-child.crt\":char*:\"data_files/server5-selfsigned.crt\":char*:\"data_files/crl.pem\":char*:\"NULL\":exp:5:exp:16:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:2:1:15",
    "5:char*:\"data_files/enco-cert-utf8str.pem\":char*:\"data_files/enco-ca-prstr.pem\":char*:\"data_files/crl.pem\":char*:\"NULL\":int:0:int:0:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:9:14:10:6:1:2",
    "5:char*:\"data_files/server5.crt\":char*:\"data_files/test-ca_cat12.crt\":char*:\"data_files/crl_cat_ec-rsa.pem\":char*:\"NULL\":int:0:int:0:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:9:14:10:6:1:2:13",
    "5:char*:\"data_files/server6.crt\":char*:\"data_files/test-ca_cat12.crt\":char*:\"data_files/crl_cat_ec-rsa.pem\":char*:\"NULL\":exp:5:exp:12:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:9:14:10:6:1:2:13",
    "5:char*:\"data_files/server6.crt\":char*:\"data_files/test-ca_cat12.crt\":char*:\"data_files/crl_cat_rsa-ec.pem\":char*:\"NULL\":exp:5:exp:12:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:9:14:10:6:1:2:13",
    "5:char*:\"data_files/server6.crt\":char*:\"data_files/test-ca_cat12.crt\":char*:\"data_files/crl_cat_ecfut-rsa.pem\":char*:\"NULL\":exp:5:exp:23:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:9:14:10:6:1:15:2:13",
    "5:char*:\"data_files/server1.crt\":char*:\"data_files/test-ca_cat12.crt\":char*:\"data_files/crl_cat_ecfut-rsa.pem\":char*:\"NULL\":exp:5:exp:12:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:9:14:10:6:1:15:2",
    "5:char*:\"data_files/enco-cert-utf8str.pem\":char*:\"data_files/enco-ca-prstr.pem\":char*:\"data_files/crl_cat_rsa-ec.pem\":char*:\"NULL\":int:0:int:0:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:9:10:14:2:6",
    "5:char*:\"data_files/server5.crt\":char*:\"data_files/test-ca2_cat-future-present.crt\":char*:\"data_files/crl-ec-sha1.pem\":char*:\"NULL\":int:0:int:0:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:9:10:14:2:6",
    "5:char*:\"data_files/server5.crt\":char*:\"data_files/test-ca2_cat-present-future.crt\":char*:\"data_files/crl-ec-sha1.pem\":char*:\"NULL\":int:0:int:0:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:9:10:14:2:6",
    "5:char*:\"data_files/server5.crt\":char*:\"data_files/test-ca2_cat-present-past.crt\":char*:\"data_files/crl-ec-sha1.pem\":char*:\"NULL\":int:0:int:0:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:9:10:14:2:6",
    "5:char*:\"data_files/server5.crt\":char*:\"data_files/test-ca2_cat-past-present.crt\":char*:\"data_files/crl-ec-sha1.pem\":char*:\"NULL\":int:0:int:0:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:9:10:14:2:6:13",
    "5:char*:\"data_files/server5.crt\":char*:\"data_files/test-ca2_cat-future-invalid.crt\":char*:\"data_files/crl-ec-sha1.pem\":char*:\"NULL\":exp:5:exp:15:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:9:10:14:2:6:13",
    "5:char*:\"data_files/server5.crt\":char*:\"data_files/test-ca2_cat-past-invalid.crt\":char*:\"data_files/crl-ec-sha1.pem\":char*:\"NULL\":exp:5:exp:14:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:9:10:14:2:6:1:15",
    "5:char*:\"data_files/server7_spurious_int-ca.crt\":char*:\"data_files/test-ca2.crt\":char*:\"data_files/crl-ec-sha1.pem\":char*:\"NULL\":int:0:int:0:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:9:10:14:2:6:1:15",
    "5:char*:\"data_files/server10_int3_spurious_int-ca2.crt\":char*:\"data_files/test-ca.crt\":char*:\"data_files/crl-ec-sha1.pem\":char*:\"NULL\":int:0:int:0:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:9:10:6:2",
    "5:char*:\"data_files/server5-ss-forgeca.crt\":char*:\"data_files/test-int-ca3.crt\":char*:\"data_files/crl-ec-sha1.pem\":char*:\"NULL\":exp:5:exp:16:char*:\"\":char*:\"NULL\"",
    "depends_on:0:2:1:15:6:9",
    "5:char*:\"data_files/server1.crt\":char*:\"data_files/test-ca-good-alt.crt\":char*:\"data_files/crl-ec-sha256.pem\":char*:\"NULL\":int:0:int:0:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:2:1:15:6:9",
    "5:char*:\"data_files/server1.crt\":char*:\"data_files/test-ca-alt-good.crt\":char*:\"data_files/crl-ec-sha256.pem\":char*:\"NULL\":int:0:int:0:char*:\"compat\":char*:\"NULL\"",
    "depends_on:0:9:6:10:14",
    "5:char*:\"data_files/server5.crt\":char*:\"data_files/test-ca2.crt\":char*:\"data_files/crl-ec-sha256.pem\":char*:\"globalhost\":int:0:int:0:char*:\"\":char*:\"verify_all\"",
    "depends_on:0:1:19:11:15:2",
    "5:char*:\"data_files/server3.crt\":char*:\"data_files/test-ca.crt\":char*:\"data_files/crl.pem\":char*:\"NULL\":exp:5:exp:24:char*:\"suite_b\":char*:\"NULL\"",
    "depends_on:0:1:9:6:10:15:14",
    "5:char*:\"data_files/server4.crt\":char*:\"data_files/test-ca2.crt\":char*:\"data_files/crl-ec-sha256.pem\":char*:\"NULL\":exp:5:exp:25:char*:\"suite_b\":char*:\"NULL\"",
    "depends_on:0:9:6:10:14",
    "5:char*:\"data_files/server5.crt\":char*:\"data_files/test-ca2.crt\":char*:\"data_files/crl-ec-sha256.pem\":char*:\"NULL\":int:0:int:0:char*:\"suite_b\":char*:\"NULL\"",
    "depends_on:6:0:1:15:2",
    "5:char*:\"data_files/cert_sha224.crt\":char*:\"data_files/test-ca.crt\":char*:\"data_files/crl.pem\":char*:\"NULL\":exp:5:exp:26:char*:\"next\":char*:\"NULL\"",
    "depends_on:6:0:1:15:9:2",
    "5:char*:\"data_files/cert_sha256.crt\":char*:\"data_files/test-ca.crt\":char*:\"data_files/crl-ec-sha256.pem\":char*:\"NULL\":int:0:int:0:char*:\"next\":char*:\"NULL\"",
    "depends_on:0:9:6:10:14",
    "6:char*:\"data_files/server5.crt\":char*:\"data_files/test-ca2.crt\":char*:\"globalhost\":exp:5:char*:\"depth 1 - serial C1\\:43\\:E2\\:7E\\:62\\:43\\:CC\\:E8 - subject C=NL, O=PolarSSL, CN=Polarssl Test EC CA - flags 0x00000000\ndepth 0 - serial 09 - subject C=NL, O=PolarSSL, CN=localhost - flags 0x00000004\n\"",
    "depends_on:0:9:6:10",
    "6:char*:\"data_files/server5-selfsigned.crt\":char*:\"data_files/server5-selfsigned.crt\":char*:\"NULL\":int:0:char*:\"depth 0 - serial 53\\:A2\\:CB\\:4B\\:12\\:4E\\:AD\\:83\\:7D\\:A8\\:94\\:B2 - subject CN=selfsigned, OU=testing, O=PolarSSL, C=NL - flags 0x00000000\n\"",
    "depends_on:0:9:6:10:13",
    "6:char*:\"data_files/server5-ss-expired.crt\":char*:\"data_files/server5-ss-expired.crt\":char*:\"NULL\":exp:5:char*:\"depth 0 - serial D8\\:64\\:61\\:05\\:E3\\:A3\\:CD\\:78 - subject C=UK, O=mbed TLS, OU=testsuite, CN=localhost - flags 0x00000001\n\"",
    "depends_on:0:2:1:15",
    "6:char*:\"data_files/server1.crt\":char*:\"data_files/test-ca.crt\":char*:\"NULL\":int:0:char*:\"depth 1 - serial 00 - subject C=NL, O=PolarSSL, CN=PolarSSL Test CA - flags 0x00000000\ndepth 0 - serial 01 - subject C=NL, O=PolarSSL, CN=PolarSSL Server 1 - flags 0x00000000\n\"",
    "depends_on:0:6:9:10:14:2:13",
    "6:char*:\"data_files/server5-expired.crt\":char*:\"data_files/test-ca2.crt\":char*:\"NULL\":exp:5:char*:\"depth 1 - serial C1\\:43\\:E2\\:7E\\:62\\:43\\:CC\\:E8 - subject C=NL, O=PolarSSL, CN=Polarssl Test EC CA - flags 0x00000000\ndepth 0 - serial 1E - subject C=NL, O=PolarSSL, CN=localhost - flags 0x00000001\n\"",
    "depends_on:0:6:9:10:14:2:13",
    "6:char*:\"data_files/server5.crt\":char*:\"data_files/test-ca2-expired.crt\":char*:\"NULL\":exp:5:char*:\"depth 1 - serial 01 - subject C=NL, O=PolarSSL, CN=Polarssl Test EC CA - flags 0x00000001\ndepth 0 - serial 09 - subject C=NL, O=PolarSSL, CN=localhost - flags 0x00000000\n\"",
    "depends_on:0:2:1:15:9:14:6",
    "6:char*:\"data_files/server1.crt\":char*:\"data_files/test-ca_cat12.crt\":char*:\"NULL\":int:0:char*:\"depth 1 - serial 00 - subject C=NL, O=PolarSSL, CN=PolarSSL Test CA - flags 0x00000000\ndepth 0 - serial 01 - subject C=NL, O=PolarSSL, CN=PolarSSL Server 1 - flags 0x00000000\n\"",
    "depends_on:0:2:1:15:9:14:6",
    "6:char*:\"data_files/server1.crt\":char*:\"data_files/test-ca_cat21.crt\":char*:\"NULL\":int:0:char*:\"depth 1 - serial 00 - subject C=NL, O=PolarSSL, CN=PolarSSL Test CA - flags 0x00000000\ndepth 0 - serial 01 - subject C=NL, O=PolarSSL, CN=PolarSSL Server 1 - flags 0x00000000\n\"",
    "depends_on:0:2:1:15:9:14:6",
    "6:char*:\"data_files/server1_ca.crt\":char*:\"data_files/test-ca_cat21.crt\":char*:\"NULL\":int:0:char*:\"depth 1 - serial 00 - subject C=NL, O=PolarSSL, CN=PolarSSL Test CA - flags 0x00000000\ndepth 0 - serial 01 - subject C=NL, O=PolarSSL, CN=PolarSSL Server 1 - flags 0x00000000\n\"",
    "depends_on:0:9:1:10:14:1:15:6:2",
    "6:char*:\"data_files/server7_int-ca.crt\":char*:\"data_files/test-ca_cat12.crt\":char*:\"NULL\":int:0:char*:\"depth 2 - serial C1\\:43\\:E2\\:7E\\:62\\:43\\:CC\\:E8 - subject C=NL, O=PolarSSL, CN=Polarssl Test EC CA - flags 0x00000000\ndepth 1 - serial 0E - subject C=NL, O=PolarSSL, CN=PolarSSL Test Intermediate CA - flags 0x00000000\ndepth 0 - serial 10 - subject C=NL, O=PolarSSL, CN=localhost - flags 0x00000000\n\"",
    "depends_on:0:9:1:10:14:1:15:6:2",
    "6:char*:\"data_files/server7_int-ca_ca2.crt\":char*:\"data_files/test-ca_cat12.crt\":char*:\"NULL\":int:0:char*:\"depth 2 - serial C1\\:43\\:E2\\:7E\\:62\\:43\\:CC\\:E8 - subject C=NL, O=PolarSSL, CN=Polarssl Test EC CA - flags 0x00000000\ndepth 1 - serial 0E - subject C=NL, O=PolarSSL, CN=PolarSSL Test Intermediate CA - flags 0x00000000\ndepth 0 - serial 10 - subject C=NL, O=PolarSSL, CN=localhost - flags 0x00000000\n\"",
    "depends_on:0:9:1:10:14:1:15:6",
    "6:char*:\"data_files/server7_int-ca_ca2.crt\":char*:\"data_files/test-int-ca.crt\":char*:\"NULL\":int:0:char*:\"depth 1 - serial 0E - subject C=NL, O=PolarSSL, CN=PolarSSL Test Intermediate CA - flags 0x00000000\ndepth 0 - serial 10 - subject C=NL, O=PolarSSL, CN=localhost - flags 0x00000000\n\"",
    "depends_on:0:9:1:10:14:1:15:6:2:13",
    "6:char*:\"data_files/server7-expired.crt\":char*:\"data_files/test-ca2.crt\":char*:\"NULL\":exp:5:char*:\"depth 2 - serial C1\\:43\\:E2\\:7E\\:62\\:43\\:CC\\:E8 - subject C=NL, O=PolarSSL, CN=Polarssl Test EC CA - flags 0x00000000\ndepth 1 - serial 0E - subject C=NL, O=PolarSSL, CN=PolarSSL Test Intermediate CA - flags 0x00000000\ndepth 0 - serial 10 - subject C=NL, O=PolarSSL, CN=localhost - flags 0x00000001\n\"",
    "depends_on:0:9:1:10:14:1:15:6:2:13",
    "6:char*:\"data_files/server7_int-ca-exp.crt\":char*:\"data_files/test-ca2.crt\":char*:\"NULL\":exp:5:char*:\"depth 2 - serial C1\\:43\\:E2\\:7E\\:62\\:43\\:CC\\:E8 - subject C=NL, O=PolarSSL, CN=Polarssl Test EC CA - flags 0x00000000\ndepth 1 - serial 0E - subject C=NL, O=PolarSSL, CN=PolarSSL Test Intermediate CA - flags 0x00000001\ndepth 0 - serial 10 - subject C=NL, O=PolarSSL, CN=localhost - flags 0x00000000\n\"",
    "depends_on:0:9:1:10:14:1:15:6:2:13",
    "6:char*:\"data_files/server7_int-ca.crt\":char*:\"data_files/test-ca2-expired.crt\":char*:\"NULL\":exp:5:char*:\"depth 2 - serial 01 - subject C=NL, O=PolarSSL, CN=Polarssl Test EC CA - flags 0x00000001\ndepth 1 - serial 0E - subject C=NL, O=PolarSSL, CN=PolarSSL Test Intermediate CA - flags 0x00000000\ndepth 0 - serial 10 - subject C=NL, O=PolarSSL, CN=localhost - flags 0x00000000\n\"",
    "depends_on:0:9:1:10:14:1:15:6:2",
    "6:char*:\"data_files/server10_int3_int-ca2.crt\":char*:\"data_files/test-ca_cat21.crt\":char*:\"NULL\":int:0:char*:\"depth 3 - serial 00 - subject C=NL, O=PolarSSL, CN=PolarSSL Test CA - flags 0x00000000\ndepth 2 - serial 0F - subject C=NL, O=PolarSSL, CN=PolarSSL Test Intermediate EC CA - flags 0x00000000\ndepth 1 - serial 4D - subject C=UK, O=mbed TLS, CN=mbed TLS Test intermediate CA 3 - flags 0x00000000\ndepth 0 - serial 4B - subject CN=localhost - flags 0x00000000\n\"",
    "depends_on:0:9:1:10:14:1:15:6:2",
    "6:char*:\"data_files/server10_int3_int-ca2_ca.crt\":char*:\"data_files/test-ca_cat21.crt\":char*:\"NULL\":int:0:char*:\"depth 3 - serial 00 - subject C=NL, O=PolarSSL, CN=PolarSSL Test CA - flags 0x00000000\ndepth 2 - serial 0F - subject C=NL, O=PolarSSL, CN=PolarSSL Test Intermediate EC CA - flags 0x00000000\ndepth 1 - serial 4D - subject C=UK, O=mbed TLS, CN=mbed TLS Test intermediate CA 3 - flags 0x00000000\ndepth 0 - serial 4B - subject CN=localhost - flags 0x00000000\n\"",
    "depends_on:0:9:1:10:14:1:15:6",
    "6:char*:\"data_files/server10_int3_int-ca2.crt\":char*:\"data_files/test-int-ca2.crt\":char*:\"NULL\":int:0:char*:\"depth 2 - serial 0F - subject C=NL, O=PolarSSL, CN=PolarSSL Test Intermediate EC CA - flags 0x00000000\ndepth 1 - serial 4D - subject C=UK, O=mbed TLS, CN=mbed TLS Test intermediate CA 3 - flags 0x00000000\ndepth 0 - serial 4B - subject CN=localhost - flags 0x00000000\n\"",
    "depends_on:0:9:1:10:14:1:15:6:2",
    "6:char*:\"data_files/server10_int3_int-ca2_ca.crt\":char*:\"data_files/test-int-ca3.crt\":char*:\"NULL\":int:0:char*:\"depth 1 - serial 4D - subject C=UK, O=mbed TLS, CN=mbed TLS Test intermediate CA 3 - flags 0x00000000\ndepth 0 - serial 4B - subject CN=localhost - flags 0x00000000\n\"",
    "depends_on:0:9:6:10:14",
    "6:char*:\"data_files/server5-badsign.crt\":char*:\"data_files/test-ca2.crt\":char*:\"NULL\":exp:5:char*:\"depth 0 - serial 09 - subject C=NL, O=PolarSSL, CN=localhost - flags 0x00000008\n\"",
    "depends_on:0:1:15:2:9:10:14:6",
    "6:char*:\"data_files/server7-badsign.crt\":char*:\"data_files/test-ca2.crt\":char*:\"NULL\":exp:5:char*:\"depth 2 - serial C1\\:43\\:E2\\:7E\\:62\\:43\\:CC\\:E8 - subject C=NL, O=PolarSSL, CN=Polarssl Test EC CA - flags 0x00000000\ndepth 1 - serial 0E - subject C=NL, O=PolarSSL, CN=PolarSSL Test Intermediate CA - flags 0x00000000\ndepth 0 - serial 10 - subject C=NL, O=PolarSSL, CN=localhost - flags 0x00000008\n\"",
    "depends_on:2:0:20:1:15",
    "23",
    "11:hex:\"\":char*:\"\":exp:27",
    "11:hex:\"300000\":char*:\"\":exp:28",
    "11:hex:\"3000\":char*:\"\":exp:28",
    "11:hex:\"30023085\":char*:\"\":exp:29",
    "11:hex:\"30023083\":char*:\"\":exp:28",
    "11:hex:\"30023081\":char*:\"\":exp:28",
    "11:hex:\"3003308200\":char*:\"\":exp:28",
    "11:hex:\"300100\":char*:\"\":exp:30",
    "11:hex:\"3003300100\":char*:\"\":exp:31",
    "11:hex:\"30053003a00101\":char*:\"\":exp:32",
    "11:hex:\"30053003a00102\":char*:\"\":exp:33",
    "11:hex:\"30163014a012021000000000000000000000000000000000\":char*:\"\":exp:34",
    "11:hex:\"30073005a003020104\":char*:\"\":exp:35",
    "11:hex:\"30083006a00402010400\":char*:\"\":exp:36",
    "11:hex:\"30083006a00302010400\":char*:\"\":exp:31",
    "11:hex:\"30083006a00302010482\":char*:\"\":exp:35",
    "11:hex:\"300d300ba0030201048204deadbeef\":char*:\"\":exp:37",
    "11:hex:\"300e300ca0030201048204deadbeef00\":char*:\"\":exp:38",
    "11:hex:\"300f300da0030201048204deadbeef3000\":char*:\"\":exp:37",
    "11:hex:\"30163014a0030201048204deadbeef30070604cafed00d01\":char*:\"\":exp:37",
    "11:hex:\"30153013a0030201048204deadbeef30060604cafed00d\":char*:\"\":exp:39",
    "11:hex:\"30173015a0030201048204deadbeef30080604cafed00d0500\":char*:\"\":exp:39",
    "11:hex:\"30183016a0030201048204deadbeef30090604cafed00d050000\":char*:\"\":exp:40",
    "11:hex:\"30173015a0030201028204deadbeef30080604cafed00d0500\":char*:\"\":exp:41",
    "depends_on:1:3",
    "11:hex:\"301c301aa0030201028204deadbeef300d06092a864886f70d0101020500\":char*:\"\":exp:28",
    "11:hex:\"301c301aa0030201028204deadbeef300d06092a864886f70d0101010500\":char*:\"\":exp:41",
    "depends_on:8",
    "11:hex:\"30193017A003020102020118300D06092A864886F70D01010A3100\":char*:\"\":exp:38",
    "depends_on:1:3",
    "11:hex:\"301e301ca0030201028204deadbeef300d06092a864886f70d01010205003000\":char*:\"\":exp:42",
    "depends_on:1:3",
    "11:hex:\"3020301ea0030201028204deadbeef300d06092a864886f70d010102050030023100\":char*:\"\":exp:42",
    "depends_on:1:3",
    "11:hex:\"30223020a0030201028204deadbeef300d06092a864886f70d0101020500300431023000\":char*:\"\":exp:42",
    "depends_on:1:3",
    "11:hex:\"30243022a0030201028204deadbeef300d06092a864886f70d01010205003006310430003000\":char*:\"\":exp:43",
    "depends_on:1:3",
    "11:hex:\"30243022a0030201028204deadbeef300d06092a864886f70d01010205003006310430020600\":char*:\"\":exp:42",
    "depends_on:1:3",
    "11:hex:\"302a3028a0030201028204deadbeef300d06092a864886f70d0101020500300c310a30080600060454657374\":char*:\"\":exp:43",
    "depends_on:1:3",
    "11:hex:\"30253023a0030201028204deadbeef300d06092a864886f70d0101020500300731053003060013\":char*:\"\":exp:42",
    "depends_on:1:3",
    "11:hex:\"302b3029a0030201028204deadbeef300d06092a864886f70d0101020500300d310b3009060013045465737400\":char*:\"\":exp:44",
    "depends_on:1:3",
    "11:hex:\"302a3028a0030201028204deadbeef300d06092a864886f70d0101020500300c310a30080600130454657374\":char*:\"\":exp:45",
    "depends_on:1:3",
    "11:hex:\"30493047a0030201028204deadbeef300d06092a864886f70d0101020500300c310a30080600130454657374301d170c303930313031303030303030170c30393132333132333539353900\":char*:\"\":exp:46",
    "depends_on:1:3",
    "11:hex:\"30483046a0030201028204deadbeef300d06092a864886f70d0101020500300c310a30080600130454657374301c170c303930313031303000000000170c303931323331323300000000\":char*:\"\":exp:47",
    "depends_on:1:3",
    "11:hex:\"30483046a0030201028204deadbeef300d06092a864886f70d0101020500300c310a30080600130454657374301c170c303930313031303030303030170c303931323331323300000000\":char*:\"\":exp:47",
    "depends_on:1:3",
    "11:hex:\"30493047a0030201028204deadbeef300d06092a864886f70d0101020500300c310a30080600130454657374301c170c303930313031303030303030170c30393132333132333539353930\":char*:\"\":exp:28",
    "depends_on:1:3",
    "11:hex:\"30563054a0030201028204deadbeef300d06092a864886f70d0101020500300c310a30080600130454657374301c170c303930313031303030303030170c303931323331323335393539300c310a30080600130454657374\":char*:\"\":exp:48",
    "depends_on:1:3",
    "11:hex:\"30583056a0030201028204deadbeef300d06092a864886f70d0101020500300c310a30080600130454657374301c170c303930313031303030303030170c303931323331323335393539300c310a300806001304546573743000\":char*:\"\":exp:49",
    "depends_on:1:3",
    "11:hex:\"30673065a0030201028204deadbeef300d06092a864886f70d0101020500300c310a30080600130454657374301c170c303930313031303030303030170c303931323331323335393539300c310a30080600130454657374300f300d06092A864886F70D0101000500\":char*:\"\":exp:50",
    "depends_on:1:3",
    "11:hex:\"30673065a0030201028204deadbeef300d06092a864886f70d0101020500300c310a30080600130454657374301c170c303930313031303030303030170c303931323331323335393539300c310a30080600130454657374300f300d06092A864886F70D0101010500\":char*:\"\":exp:51",
    "depends_on:1:3",
    "11:hex:\"30693067a0030201028204deadbeef300d06092a864886f70d0101020500300c310a30080600130454657374301c170c303930313031303030303030170c303931323331323335393539300c310a300806001304546573743011300d06092A864886F70D01010105000300\":char*:\"\":exp:52",
    "depends_on:1:3",
    "11:hex:\"306a3068a0030201028204deadbeef300d06092a864886f70d0101020500300c310a30080600130454657374301c170c303930313031303030303030170c303931323331323335393539300c310a300806001304546573743012300d06092A864886F70D0101010500030101\":char*:\"\":exp:52",
    "depends_on:1:3",
    "11:hex:\"306d306ba0030201028204deadbeef300d06092a864886f70d0101020500300c310a30080600130454657374301c170c303930313031303030303030170c303931323331323335393539300c310a300806001304546573743015300d06092A864886F70D0101010500030400300000\":char*:\"\":exp:53",
    "depends_on:1:3",
    "11:hex:\"306d306ba0030201028204deadbeef300d06092a864886f70d0101020500300c310a30080600130454657374301c170c303930313031303030303030170c303931323331323335393539300c310a300806001304546573743015300d06092A864886F70D0101010500030400310000\":char*:\"\":exp:54",
    "depends_on:1:3",
    "11:hex:\"30743072a0030201028204deadbeef300d06092a864886f70d0101020500300c310a30080600130454657374301c170c303930313031303030303030170c303931323331323335393539300c310a30080600130454657374301c300d06092A864886F70D0101010500030b0030080202ffff0302ffff\":char*:\"\":exp:54",
    "depends_on:1:3",
    "11:hex:\"30753073a0030201028204deadbeef300d06092a864886f70d0101020500300c310a30080600130454657374301c170c303930313031303030303030170c303931323331323335393539300c310a30080600130454657374301d300d06092A864886F70D0101010500030b0030080202ffff0202ffff00\":char*:\"\":exp:53",
    "depends_on:1:3",
    "11:hex:\"30743072a0030201028204deadbeef300d06092a864886f70d0101020500300c310a30080600130454657374301c170c303930313031303030303030170c303931323331323335393539300c310a30080600130454657374301c300d06092A864886F70D0101010500030b0030080202ffff0202ffff\":char*:\"\":exp:55",
    "depends_on:1:3",
    "11:hex:\"308183308180a0030201028204deadbeef300d06092a864886f70d0101020500300c310a30080600130454657374301c170c303930313031303030303030170c303931323331323335393539300c310a30080600130454657374302a300d06092A864886F70D010101050003190030160210fffffffffffffffffffffffffffffffe0202ffff\":char*:\"\":exp:55",
    "depends_on:1:3",
    "11:hex:\"308183308180a0030201028204deadbeef300d06092a864886f70d0101020500300c310a30080600130454657374301c170c303930313031303030303030170c303931323331323335393539300c310a30080600130454657374302a300d06092A864886F70D010101050003190030160210ffffffffffffffffffffffffffffffff0202ffff\":char*:\"\":exp:37",
    "depends_on:1:3",
    "11:hex:\"308184308181a0030201028204deadbeef300d06092a864886f70d0101020500300c310a30080600130454657374301c170c303930313031303030303030170c303931323331323335393539300c310a30080600130454657374302a300d06092A864886F70D010101050003190030160210ffffffffffffffffffffffffffffffff0202ffff00\":char*:\"\":exp:56",
    "depends_on:1:3",
    "11:hex:\"308189308186a0030201028204deadbeef300d06092a864886f70d0101020500300c310a30080600130454657374301c170c303930313031303030303030170c303931323331323335393539300c310a30080600130454657374302a300d06092A864886F70D010101050003190030160210ffffffffffffffffffffffffffffffff0202ffffa101aaa201bb\":char*:\"\":exp:37",
    "depends_on:1:3",
    "11:hex:\"308189308186a0030201028204deadbeef300d06092a864886f70d0101020500300c310a30080600130454657374301c170c303930313031303030303030170c303931323331323335393539300c310a30080600130454657374302a300d06092A864886F70D010101050003190030160210ffffffffffffffffffffffffffffffff0202ffffa185aaa201bb\":char*:\"\":exp:57",
    "depends_on:1:3",
    "11:hex:\"30818b308188a0030201028204deadbeef300d06092a864886f70d0101020500300c310a30080600130454657374301c170c303930313031303030303030170c303931323331323335393539300c310a30080600130454657374302a300d06092A864886F70D010101050003190030160210ffffffffffffffffffffffffffffffff0202ffffa101aaa201bba300\":char*:\"\":exp:58",
    "depends_on:1:3",
    "11:hex:\"30818e30818ba0030201028204deadbeef300d06092a864886f70d0101020500300c310a30080600130454657374301c170c303930313031303030303030170c303931323331323335393539300c310a30080600130454657374302a300d06092A864886F70D010101050003190030160210ffffffffffffffffffffffffffffffff0202ffffa101aaa201bba303300000\":char*:\"\":exp:59",
    "depends_on:1:3",
    "11:hex:\"30818f30818ca0030201028204deadbeef300d06092a864886f70d0101020500300c310a30080600130454657374301c170c303930313031303030303030170c303931323331323335393539300c310a30080600130454657374302a300d06092A864886F70D010101050003190030160210ffffffffffffffffffffffffffffffff0202ffffa101aaa201bba30330023000\":char*:\"\":exp:58",
    "depends_on:1:3",
    "11:hex:\"30819030818da0030201028204deadbeef300d06092a864886f70d0101020500300c310a30080600130454657374301c170c303930313031303030303030170c303931323331323335393539300c310a30080600130454657374302a300d06092A864886F70D010101050003190030160210ffffffffffffffffffffffffffffffff0202ffffa101aaa201bba3043002310000\":char*:\"\":exp:1",
    "depends_on:1:3",
    "11:hex:\"308198308195a0030201028204deadbeef300d06092a864886f70d0101020500300c310a30080600130454657374301c170c303930313031303030303030170c303931323331323335393539300c310a30080600130454657374302a300d06092A864886F70D010101050003190030160210ffffffffffffffffffffffffffffffff0202ffffa101aaa201bba30c300a30060603551d1301010100\":char*:\"\":exp:58",
    "depends_on:1:3",
    "11:hex:\"308198308195a0030201028204deadbeef300d06092a864886f70d0101020500300c310a30080600130454657374301c170c303930313031303030303030170c303931323331323335393539300c310a30080600130454657374302a300d06092A864886F70D010101050003190030160210ffffffffffffffffffffffffffffffff0202ffffa101aaa201bba30c300a30080603551d1301010100\":char*:\"\":exp:58",
    "depends_on:1:3",
    "11:hex:\"308198308195a0030201028204deadbeef300d06092a864886f70d0101020500300c310a30080600130454657374301c170c303930313031303030303030170c303931323331323335393539300c310a30080600130454657374302a300d06092A864886F70D010101050003190030160210ffffffffffffffffffffffffffffffff0202ffffa101aaa201bba30d300b30090603551d1301010100\":char*:\"\":exp:1",
    "depends_on:1:3",
    "11:hex:\"30819c308199a0030201028204deadbeef300d06092a864886f70d0101020500300c310a30080600130454657374301c170c303930313031303030303030170c303931323331323335393539300c310a30080600130454657374302a300d06092A864886F70D010101050003190030160210ffffffffffffffffffffffffffffffff0202ffffa101aaa201bba311300f300d0603551d130101010403300100\":char*:\"\":exp:1",
    "depends_on:1:3",
    "11:hex:\"30819f30819ca0030201028204deadbeef300d06092a864886f70d0101020500300c310a30080600130454657374301c170c303930313031303030303030170c303931323331323335393539300c310a30080600130454657374302a300d06092A864886F70D010101050003190030160210ffffffffffffffffffffffffffffffff0202ffffa101aaa201bba314301230100603551d130101010406300402010102\":char*:\"\":exp:58",
    "depends_on:1:3",
    "11:hex:\"3081a230819fa0030201028204deadbeef300d06092a864886f70d0101020500300c310a30080600130454657374301c170c303930313031303030303030170c303931323331323335393539300c310a30080600130454657374302a300d06092A864886F70D010101050003190030160210ffffffffffffffffffffffffffffffff0202ffffa101aaa201bba317301530130603551d130101010409300702010102010100\":char*:\"\":exp:59",
    "depends_on:9:10:2",
    "11:hex:\"3081de3081dba003020102020900ebdbcd14105e1839300906072a8648ce3d0401300f310d300b0603550403130454657374301e170d3134313131313230353935345a170d3234313130383230353935345a300f310d300b06035504031304546573743059301306072a8648ce3d020106082a8648ce3d0301070342000437cc56d976091e5a723ec7592dff206eee7cf9069174d0ad14b5f768225962924ee500d82311ffea2fd2345d5d16bd8a88c26b770d55cd8a2a0efa01c8b4edffa321301f301d0603551d250416301406082b0601050507030107082b06010505070302\":char*:\"\":exp:1",
    "depends_on:9:10:2",
    "11:hex:\"3081fd3081faa003020102020900a8b31ff37d09a37f300906072a8648ce3d0401300f310d300b0603550403130454657374301e170d3134313131313231333731365a170d3234313130383231333731365a300f310d300b06035504031304546573743059301306072a8648ce3d020106082a8648ce3d0301070342000437cc56d976091e5a723ec7592dff206eee7cf9069174d0ad14b5f768225962924ee500d82311ffea2fd2345d5d16bd8a88c26b770d55cd8a2a0efa01c8b4edffa321301f301d0603551d11041630148208666f6f2e7465737482086261722e74657374301d0603551d11041630148208666f6f2e7465737482086261722e74657374\":char*:\"\":exp:60",
    "depends_on:9:10:2",
    "11:hex:\"3081fd3081faa003020102020900ebdbcd14105e1839300906072a8648ce3d0401300f310d300b0603550403130454657374301e170d3134313131313230353935345a170d3234313130383230353935345a300f310d300b06035504031304546573743059301306072a8648ce3d020106082a8648ce3d0301070342000437cc56d976091e5a723ec7592dff206eee7cf9069174d0ad14b5f768225962924ee500d82311ffea2fd2345d5d16bd8a88c26b770d55cd8a2a0efa01c8b4edffa340303e301d0603551d250416301406082b0601050507030106082b06010505070302301d0603551d250416301406082b0601050507030106082b06010505070302\":char*:\"\":exp:60",
    "depends_on:1:3",
    "11:hex:\"308183308180a0030201008204deadbeef300d06092a864886f70d0101020500300c310a30080600130454657374301c170c303930313031303030303030170c303931323331323335393539300c310a30080600130454657374302a300d06092A864886F70D010101050003190030160210ffffffffffffffffffffffffffffffff0202ffff\":char*:\"\":exp:37",
    "depends_on:1:3",
    "11:hex:\"308192308180a0030201008204deadbeef300d06092a864886f70d0101020500300c310a30080600130454657374301c170c303930313031303030303030170c303931323331323335393539300c310a30080600130454657374302a300d06092A864886F70D010101050003190030160210ffffffffffffffffffffffffffffffff0202ffff300d06092a864886f70d0102020500\":char*:\"\":exp:61",
    "depends_on:1:3",
    "11:hex:\"308192308180a0030201008204deadbeef300d06092a864886f70d0101020500300c310a30080600130454657374301c170c303930313031303030303030170c303931323331323335393539300c310a30080600130454657374302a300d06092A864886F70D010101050003190030160210ffffffffffffffffffffffffffffffff0202ffff300d06092a864886f70d0101020500\":char*:\"\":exp:62",
    "depends_on:1:3",
    "11:hex:\"308195308180a0030201008204deadbeef300d06092a864886f70d0101020500300c310a30080600130454657374301c170c303930313031303030303030170c303931323331323335393539300c310a30080600130454657374302a300d06092A864886F70D010101050003190030160210ffffffffffffffffffffffffffffffff0202ffff300d06092a864886f70d0101020500030100\":char*:\"\":exp:63",
    "depends_on:1:3",
    "11:hex:\"308197308180a0030201008204deadbeef300d06092a864886f70d0101020500300c310a30080600130454657374301c170c303930313031303030303030170c303931323331323335393539300c310a30080600130454657374302a300d06092A864886F70D010101050003190030160210ffffffffffffffffffffffffffffffff0202ffff300d06092a864886f70d0101020500030200ff00\":char*:\"\":exp:56",
    "depends_on:1:3",
    "11:hex:\"308196308180a0030201008204deadbeef300d06092a864886f70d0101020500300c310a30080600130454657374301c170c303930313031303030303030170c303931323331323335393539300c310a30080600130454657374302a300d06092A864886F70D010101050003190030160210ffffffffffffffffffffffffffffffff0202ffff300d06092a864886f70d0101020500030200ff\":char*:\"cert. version     \\: 1\nserial number     \\: DE\\:AD\\:BE\\:EF\nissuer name       \\: \?\?=Test\nsubject name      \\: \?\?=Test\nissued  on        \\: 2009-01-01 00\\:00\\:00\nexpires on        \\: 2009-12-31 23\\:59\\:59\nsigned using      \\: RSA with MD2\nRSA key size      \\: 128 bits\n\":int:0",
    "depends_on:1:3",
    "11:hex:\"308198308182a0030201008204deadbeef300d06092a864886f70d0101020500300c310a30080600130454657374301e180e3230313030313031303030303030170c303931323331323335393539300c310a30080600130454657374302a300d06092A864886F70D010101050003190030160210ffffffffffffffffffffffffffffffff0202ffff300d06092a864886f70d0101020500030200ff\":char*:\"cert. version     \\: 1\nserial number     \\: DE\\:AD\\:BE\\:EF\nissuer name       \\: \?\?=Test\nsubject name      \\: \?\?=Test\nissued  on        \\: 2010-01-01 00\\:00\\:00\nexpires on        \\: 2009-12-31 23\\:59\\:59\nsigned using      \\: RSA with MD2\nRSA key size      \\: 128 bits\n\":int:0",
    "depends_on:1:3",
    "11:hex:\"308199308183a0030201008204deadbeef300d06092a864886f70d0101020500300f310d300b0603550403130454657374301c170c303930313031303030303030170c303931323331323335393539300c310a30080600130454657374302a300d06092A864886F70D010101050003190030160210ffffffffffffffffffffffffffffffff0202ffff300d06092a864886f70d0101020500030200ff\":char*:\"cert. version     \\: 1\nserial number     \\: DE\\:AD\\:BE\\:EF\nissuer name       \\: CN=Test\nsubject name      \\: \?\?=Test\nissued  on        \\: 2009-01-01 00\\:00\\:00\nexpires on        \\: 2009-12-31 23\\:59\\:59\nsigned using      \\: RSA with MD2\nRSA key size      \\: 128 bits\n\":int:0",
    "depends_on:1:3",
    "11:hex:\"308199308183a0030201008204deadbeef300d06092a864886f70d0101020500300f310d300b0603550406130454657374301c170c303930313031303030303030170c303931323331323335393539300c310a30080600130454657374302a300d06092A864886F70D010101050003190030160210ffffffffffffffffffffffffffffffff0202ffff300d06092a864886f70d0101020500030200ff\":char*:\"cert. version     \\: 1\nserial number     \\: DE\\:AD\\:BE\\:EF\nissuer name       \\: C=Test\nsubject name      \\: \?\?=Test\nissued  on        \\: 2009-01-01 00\\:00\\:00\nexpires on        \\: 2009-12-31 23\\:59\\:59\nsigned using      \\: RSA with MD2\nRSA key size      \\: 128 bits\n\":int:0",
    "depends_on:1:3",
    "11:hex:\"308199308183a0030201008204deadbeef300d06092a864886f70d0101020500300f310d300b0603550407130454657374301c170c303930313031303030303030170c303931323331323335393539300c310a30080600130454657374302a300d06092A864886F70D010101050003190030160210ffffffffffffffffffffffffffffffff0202ffff300d06092a864886f70d0101020500030200ff\":char*:\"cert. version     \\: 1\nserial number     \\: DE\\:AD\\:BE\\:EF\nissuer name       \\: L=Test\nsubject name      \\: \?\?=Test\nissued  on        \\: 2009-01-01 00\\:00\\:00\nexpires on        \\: 2009-12-31 23\\:59\\:59\nsigned using      \\: RSA with MD2\nRSA key size      \\: 128 bits\n\":int:0",
    "depends_on:1:3",
    "11:hex:\"308199308183a0030201008204deadbeef300d06092a864886f70d0101020500300f310d300b0603550408130454657374301c170c303930313031303030303030170c303931323331323335393539300c310a30080600130454657374302a300d06092A864886F70D010101050003190030160210ffffffffffffffffffffffffffffffff0202ffff300d06092a864886f70d0101020500030200ff\":char*:\"cert. version     \\: 1\nserial number     \\: DE\\:AD\\:BE\\:EF\nissuer name       \\: ST=Test\nsubject name      \\: \?\?=Test\nissued  on        \\: 2009-01-01 00\\:00\\:00\nexpires on        \\: 2009-12-31 23\\:59\\:59\nsigned using      \\: RSA with MD2\nRSA key size      \\: 128 bits\n\":int:0",
    "depends_on:1:3",
    "11:hex:\"308199308183a0030201008204deadbeef300d06092a864886f70d0101020500300f310d300b060355040a130454657374301c170c303930313031303030303030170c303931323331323335393539300c310a30080600130454657374302a300d06092A864886F70D010101050003190030160210ffffffffffffffffffffffffffffffff0202ffff300d06092a864886f70d0101020500030200ff\":char*:\"cert. version     \\: 1\nserial number     \\: DE\\:AD\\:BE\\:EF\nissuer name       \\: O=Test\nsubject name      \\: \?\?=Test\nissued  on        \\: 2009-01-01 00\\:00\\:00\nexpires on        \\: 2009-12-31 23\\:59\\:59\nsigned using      \\: RSA with MD2\nRSA key size      \\: 128 bits\n\":int:0",
    "depends_on:1:3",
    "11:hex:\"308199308183a0030201008204deadbeef300d06092a864886f70d0101020500300f310d300b060355040b130454657374301c170c303930313031303030303030170c303931323331323335393539300c310a30080600130454657374302a300d06092A864886F70D010101050003190030160210ffffffffffffffffffffffffffffffff0202ffff300d06092a864886f70d0101020500030200ff\":char*:\"cert. version     \\: 1\nserial number     \\: DE\\:AD\\:BE\\:EF\nissuer name       \\: OU=Test\nsubject name      \\: \?\?=Test\nissued  on        \\: 2009-01-01 00\\:00\\:00\nexpires on        \\: 2009-12-31 23\\:59\\:59\nsigned using      \\: RSA with MD2\nRSA key size      \\: 128 bits\n\":int:0",
    "depends_on:1:3",
    "11:hex:\"308199308183a0030201008204deadbeef300d06092a864886f70d0101020500300f310d300b06035504de130454657374301c170c303930313031303030303030170c303931323331323335393539300c310a30080600130454657374302a300d06092A864886F70D010101050003190030160210ffffffffffffffffffffffffffffffff0202ffff300d06092a864886f70d0101020500030200ff\":char*:\"cert. version     \\: 1\nserial number     \\: DE\\:AD\\:BE\\:EF\nissuer name       \\: \?\?=Test\nsubject name      \\: \?\?=Test\nissued  on        \\: 2009-01-01 00\\:00\\:00\nexpires on        \\: 2009-12-31 23\\:59\\:59\nsigned using      \\: RSA with MD2\nRSA key size      \\: 128 bits\n\":int:0",
    "depends_on:1:2",
    "11:hex:\"3082029f30820208a00302010202044c20e3bd300d06092a864886f70d01010505003056310b3009060355040613025553310b300906035504080c0243413121301f060355040a0c18496e7465726e6574205769646769747320507479204c74643117301506035504030c0e4672616e6b656e63657274204341301e170d3133303830323135313433375a170d3135303831373035353433315a3081d1310b3009060355040613025553311330110603550408130a57617368696e67746f6e31133011060b2b0601040182373c0201031302555331193017060b2b0601040182373c020102130844656c6177617265311a3018060355040a1311417574686f72697a652e4e6574204c4c43311d301b060355040f131450726976617465204f7267616e697a6174696f6e312a300e06035504051307343336393139313018060355040313117777772e617574686f72697a652e6e6574311630140603550407130d53616e204672616e636973636f30819f300d06092a864886f70d010101050003818d0030818902818100d885c62e209b6ac005c64f0bcfdaac1f2b67a18802f75b08851ff933deed888b7b68a62fcabdb21d4a8914becfeaaa1b7e08a09ffaf9916563586dc95e2877262b0b5f5ec27eb4d754aa6facd1d39d25b38a2372891bacdd3e919f791ed25704e8920e380e5623a38e6a23935978a3aec7a8e761e211d42effa2713e44e7de0b0203010001300d06092a864886f70d010105050003818100092f7424d3f6da4b8553829d958ed1980b9270b42c0d3d5833509a28c66bb207df9f3c51d122065e00b87c08c2730d2745fe1c279d16fae4d53b4bf5bdfa3631fceeb2e772b6b08a3eca5a2e2c687aefd23b4b73bf77ac6099711342cf070b35c6f61333a7cbf613d8dd4bd73e9df34bcd4284b0b4df57c36c450613f11e5dac\":char*:\"cert. version     \\: 3\nserial number     \\: 4C\\:20\\:E3\\:BD\nissuer name       \\: C=US, ST=CA, O=Internet Widgits Pty Ltd, CN=Frankencert CA\nsubject name      \\: C=US, ST=Washington, \?\?=US, \?\?=Delaware, O=Authorize.Net LLC, \?\?=Private Organization, serialNumber=4369191 + CN=www.authorize.net, L=San Francisco\nissued  on        \\: 2013-08-02 15\\:14\\:37\nexpires on        \\: 2015-08-17 05\\:54\\:31\nsigned using      \\: RSA with SHA1\nRSA key size      \\: 1024 bits\n\":int:0",
    "depends_on:1:3",
    "11:hex:\"30819f308189a0030201008204deadbeef300d06092a864886f70d010102050030153113301106092a864886f70d010901130454657374301c170c303930313031303030303030170c303931323331323335393539300c310a30080600130454657374302a300d06092A864886F70D010101050003190030160210ffffffffffffffffffffffffffffffff0202ffff300d06092a864886f70d0101020500030200ff\":char*:\"cert. version     \\: 1\nserial number     \\: DE\\:AD\\:BE\\:EF\nissuer name       \\: emailAddress=Test\nsubject name      \\: \?\?=Test\nissued  on        \\: 2009-01-01 00\\:00\\:00\nexpires on        \\: 2009-12-31 23\\:59\\:59\nsigned using      \\: RSA with MD2\nRSA key size      \\: 128 bits\n\":int:0",
    "depends_on:1:3",
    "11:hex:\"30819f308189a0030201008204deadbeef300d06092a864886f70d010102050030153113301106092a864886f70d0109ab130454657374301c170c303930313031303030303030170c303931323331323335393539300c310a30080600130454657374302a300d06092A864886F70D010101050003190030160210ffffffffffffffffffffffffffffffff0202ffff300d06092a864886f70d0101020500030200ff\":char*:\"cert. version     \\: 1\nserial number     \\: DE\\:AD\\:BE\\:EF\nissuer name       \\: \?\?=Test\nsubject name      \\: \?\?=Test\nissued  on        \\: 2009-01-01 00\\:00\\:00\nexpires on        \\: 2009-12-31 23\\:59\\:59\nsigned using      \\: RSA with MD2\nRSA key size      \\: 128 bits\n\":int:0",
    "depends_on:1:2:9",
    "11:hex:\"3081E630819E020103300906072A8648CE3D0401300F310D300B0603550403130454657374301E170D3133303731303039343631385A170D3233303730383039343631385A300F310D300B0603550403130454657374304C300D06092A864886F70D0101010500033B003038023100E8F546061D3B49BC2F6B7524B7EA4D73A8D5293EE8C64D9407B70B5D16BAEBC32B8205591EAB4E1EB57E9241883701250203010001300906072A8648CE3D0401033800303502186E18209AFBED14A0D9A796EFCAD68891E3CCD5F75815C833021900E92B4FD460B1994693243B9FFAD54729DE865381BDA41D25\":char*:\"cert. version     \\: 1\nserial number     \\: 03\nissuer name       \\: CN=Test\nsubject name      \\: CN=Test\nissued  on        \\: 2013-07-10 09\\:46\\:18\nexpires on        \\: 2023-07-08 09\\:46\\:18\nsigned using      \\: ECDSA with SHA1\nRSA key size      \\: 384 bits\n\":int:0",
    "depends_on:9:11:2",
    "11:hex:\"3081EB3081A3020900F41534662EC7E912300906072A8648CE3D0401300F310D300B0603550403130454657374301E170D3133303731303039343031395A170D3233303730383039343031395A300F310D300B06035504031304546573743049301306072A8648CE3D020106082A8648CE3D030101033200042137969FABD4E370624A0E1A33E379CAB950CCE00EF8C3C3E2ADAEB7271C8F07659D65D3D777DCF21614363AE4B6E617300906072A8648CE3D04010338003035021858CC0F957946FE6A303D92885A456AA74C743C7B708CBD37021900FE293CAC21AF352D16B82EB8EA54E9410B3ABAADD9F05DD6\":char*:\"cert. version     \\: 1\nserial number     \\: F4\\:15\\:34\\:66\\:2E\\:C7\\:E9\\:12\nissuer name       \\: CN=Test\nsubject name      \\: CN=Test\nissued  on        \\: 2013-07-10 09\\:40\\:19\nexpires on        \\: 2023-07-08 09\\:40\\:19\nsigned using      \\: ECDSA with SHA1\nEC key size       \\: 192 bits\n\":int:0",
    "depends_on:9:11:2:1",
    "11:hex:\"3081E430819F020104300D06092A864886F70D0101050500300F310D300B0603550403130454657374301E170D3133303731303135303233375A170D3233303730383135303233375A300F310D300B06035504031304546573743049301306072A8648CE3D020106082A8648CE3D03010103320004E962551A325B21B50CF6B990E33D4318FD16677130726357A196E3EFE7107BCB6BDC6D9DB2A4DF7C964ACFE81798433D300D06092A864886F70D01010505000331001A6C18CD1E457474B2D3912743F44B571341A7859A0122774A8E19A671680878936949F904C9255BDD6FFFDB33A7E6D8\":char*:\"cert. version     \\: 1\nserial number     \\: 04\nissuer name       \\: CN=Test\nsubject name      \\: CN=Test\nissued  on        \\: 2013-07-10 15\\:02\\:37\nexpires on        \\: 2023-07-08 15\\:02\\:37\nsigned using      \\: RSA with SHA1\nEC key size       \\: 192 bits\n\":int:0",
    "11:hex:\"30173015a0030201038204deadbeef30080604cafed00d0500\":char*:\"\":exp:39",
    "11:hex:\"301A3018a00602047FFFFFFF8204deadbeef30080604cafed00d0500\":char*:\"\":exp:39",
    "depends_on:1:2",
    "11:hex:\"308203723082025AA003020102020111300D06092A864886F70D0101050500303B310B3009060355040613024E4C3111300F060355040A1308506F6C617253534C3119301706035504031310506F6C617253534C2054657374204341301E170D3132303531303133323334315A170D3232303531313133323334315A303A310B3009060355040613024E4C3111300F060355040A1308506F6C617253534C311830160603550403130F7777772E6578616D706C652E636F6D30820122300D06092A864886F70D01010105000382010F003082010A0282010100B93C4AC5C8A38E9017A49E52AA7175266180E7C7B56D8CFFAAB64126B7BE11AD5C73160C64114804FFD6E13B05DB89BBB39709D51C14DD688739B03D71CBE276D01AD8182D801B54F6E5449AF1CBAF612EDF490D9D09B7EDB1FD3CFD3CFA24CF5DBF7CE453E725B5EA4422E926D3EA20949EE66167BA2E07670B032FA209EDF0338F0BCE10EF67A4C608DAC1EDC23FD74ADD153DF95E1C8160463EB5B33D2FA6DE471CBC92AEEBDF276B1656B7DCECD15557A56EEC7525F5B77BDFABD23A5A91987D97170B130AA76B4A8BC14730FB3AF84104D5C1DFB81DBF7B01A565A2E01E36B7A65CCC305AF8CD6FCDF1196225CA01E3357FFA20F5DCFD69B26A007D17F70203010001A38181307F30090603551D1304023000301D0603551D0E041604147DE49C6BE6F9717D46D2123DAD6B1DFDC2AA784C301F0603551D23041830168014B45AE4A5B3DED252F6B9D5A6950FEB3EBCC7FDFF30320603551D11042B3029C20B6578616D706C652E636F6D820B6578616D706C652E6E6574820D2A2E6578616D706C652E6F7267300D06092A864886F70D010105050003820101004F09CB7AD5EEF5EF620DDC7BA285D68CCA95B46BDA115B92007513B9CA0BCEEAFBC31FE23F7F217479E2E6BCDA06E52F6FF655C67339CF48BC0D2F0CD27A06C34A4CD9485DA0D07389E4D4851D969A0E5799C66F1D21271F8D0529E840AE823968C39707CF3C934C1ADF2FA6A455487F7C8C1AC922DA24CD9239C68AECB08DF5698267CB04EEDE534196C127DC2FFE33FAD30EB8D432A9842853A5F0D189D5A298E71691BB9CC0418E8C58ACFFE3DD2E7AABB0B97176AD0F2733F7A929D3C076C0BF06407C0ED5A47C8AE2326E16AEDA641FB0557CDBDDF1A4BA447CB39958D2346E00EA976C143AF2101E0AA249107601F4F2C818FDCC6346128B091BF194E6\":char*:\"\":exp:1",
    "12:hex:\"\":char*:\"\":exp:27",
    "12:hex:\"300000\":char*:\"\":exp:56",
    "12:hex:\"3000\":char*:\"\":exp:28",
    "12:hex:\"3003300102\":char*:\"\":exp:33",
    "12:hex:\"30053003020100\":char*:\"\":exp:37",
    "12:hex:\"300b3009020102300406000500\":char*:\"\":exp:39",
    "12:hex:\"300b3009020100300406000500\":char*:\"\":exp:64",
    "12:hex:\"30143012020100300d06092a864886f70d01010f0500\":char*:\"\":exp:64",
    "depends_on:1:6",
    "12:hex:\"30143012020100300d06092a864886f70d01010e0500\":char*:\"\":exp:28",
    "depends_on:1:6",
    "12:hex:\"30163014020100300d06092a864886f70d01010e05003000\":char*:\"\":exp:42",
    "depends_on:1:6",
    "12:hex:\"30253023020100300d06092a864886f70d01010e0500300f310d300b0603550403130441424344\":char*:\"\":exp:45",
    "depends_on:1:6",
    "12:hex:\"30343032020100300d06092a864886f70d01010e0500300f310d300b0603550403130441424344170c30393031303130303030303030\":char*:\"\":exp:65",
    "depends_on:1:6",
    "12:hex:\"304a3047020100300d06092a864886f70d01010e0500300f310d300b0603550403130441424344170c303930313031303030303030301430128202abcd170c30383132333132333539353900\":char*:\"\":exp:38",
    "depends_on:1:6",
    "12:hex:\"304a3047020100300d06092a864886f70d01010e0500300f310d300b0603550403130441424344170c303930313031303030303030301430128202abcd190c30383132333132333539353900\":char*:\"\":exp:66",
    "depends_on:1:6",
    "12:hex:\"30583047020100300d06092a864886f70d01010e0500300f310d300b0603550403130441424344170c303930313031303030303030301430128202abcd170c303831323331323335393539300d06092a864886f70d01010d0500\":char*:\"\":exp:61",
    "depends_on:1:6",
    "12:hex:\"305d3047020100300d06092a864886f70d01010e0500300f310d300b0603550403130441424344170c303930313031303030303030301430128202abcd170c303831323331323335393539300d06092a864886f70d01010e05000302000100\":char*:\"\":exp:56",
    "depends_on:1:6",
    "12:hex:\"305c3047020100300d06092a864886f70d01010e0500300f310d300b0603550403130441424344170c303930313031303030303030301430128202abcd170c303831323331323335393539300d06092a864886f70d01010e050003020001\":char*:\"CRL version   \\: 1\nissuer name   \\: CN=ABCD\nthis update   \\: 2009-01-01 00\\:00\\:00\nnext update   \\: 0000-00-00 00\\:00\\:00\nRevoked certificates\\:\nserial number\\: AB\\:CD revocation date\\: 2008-12-31 23\\:59\\:59\nsigned using  \\: RSA with SHA-224\n\":int:0",
    "depends_on:1:6",
    "12:hex:\"30463031020100300d06092a864886f70d01010e0500300f310d300b0603550403130441424344170c303930313031303030303030300d06092a864886f70d01010e050003020001\":char*:\"CRL version   \\: 1\nissuer name   \\: CN=ABCD\nthis update   \\: 2009-01-01 00\\:00\\:00\nnext update   \\: 0000-00-00 00\\:00\\:00\nRevoked certificates\\:\nsigned using  \\: RSA with SHA-224\n\":int:0",
    "12:hex:\"30463031020102300d06092a864886f70d01010e0500300f310d300b0603550403130441424344170c303930313031303030303030300d06092a864886f70d01010e050003020001\":char*:\"\":exp:39",
    "12:hex:\"3049303102047FFFFFFF300d06092a864886f70d01010e0500300f310d300b0603550403130441424344170c303930313031303030303030300d06092a864886f70d01010e050003020001\":char*:\"\":exp:39",
    "depends_on:0:1:6",
    "12:hex:\"308201b330819c020101300d06092a864886f70d01010b0500303b310b3009060355040613024e4c3111300f060355040a1308506f6c617253534c3119301706035504031310506f6c617253534c2054657374204341170d3138303331343037333134385a170d3238303331343037333134385aa02d302b30300603551d1c0101ff041f301da01ba0198617687474703a2f2f706b692e6578616d706c652e636f6d2f300d06092a864886f70d01010b05000382010100b3fbe9d586eaf4b8ff60cf8edae06a85135db78f78198498719725b5b403c0b803c2c150f52faae7306d6a7871885dc2e9dc83a164bac7263776474ef642b660040b35a1410ac291ac8f6f18ab85e7fd6e22bd1af1c41ca95cf2448f6e2b42a018493dfc03c6b6aa1b9e3fe7b76af2182fb2121db4166bf0167d6f379c5a58adee5082423434d97be2909f5e7488053f996646db10dd49782626da53ad8eada01813c031b2bacdb0203bc017aac1735951a11d013ee4d1d5f7143ccbebf2371e66a1bec6e1febe69148f50784eef8adbb66664c96196d7e0c0bcdc807f447b54e058f37642a3337995bfbcd332208bd6016936705c82263eabd7affdba92fae3\":char*:\"\":exp:58",
    "depends_on:0:1:6",
    "12:hex:\"308201b330819c020101300d06092a864886f70d01010b0500303b310b3009060355040613024e4c3111300f060355040a1308506f6c617253534c3119301706035504031310506f6c617253534c2054657374204341170d3138303331343037333134385a170d3238303331343037333134385aa02d302b30290628551d1c0101ff041f301da01ba0198617687474703a2f2f706b692e6578616d706c652e636f6d2f300d06092a864886f70d01010b05000382010100b3fbe9d586eaf4b8ff60cf8edae06a85135db78f78198498719725b5b403c0b803c2c150f52faae7306d6a7871885dc2e9dc83a164bac7263776474ef642b660040b35a1410ac291ac8f6f18ab85e7fd6e22bd1af1c41ca95cf2448f6e2b42a018493dfc03c6b6aa1b9e3fe7b76af2182fb2121db4166bf0167d6f379c5a58adee5082423434d97be2909f5e7488053f996646db10dd49782626da53ad8eada01813c031b2bacdb0203bc017aac1735951a11d013ee4d1d5f7143ccbebf2371e66a1bec6e1febe69148f50784eef8adbb66664c96196d7e0c0bcdc807f447b54e058f37642a3337995bfbcd332208bd6016936705c82263eabd7affdba92fae3\":char*:\"\":exp:58",
    "depends_on:0:1:6",
    "12:hex:\"308201b330819c020101300d06092a864886f70d01010b0500303b310b3009060355040613024e4c3111300f060355040a1308506f6c617253534c3119301706035504031310506f6c617253534c2054657374204341170d3138303331343037333134385a170d3238303331343037333134385aa02d302b30290603551d1c0102ff041f301da01ba0198617687474703a2f2f706b692e6578616d706c652e636f6d2f300d06092a864886f70d01010b05000382010100b3fbe9d586eaf4b8ff60cf8edae06a85135db78f78198498719725b5b403c0b803c2c150f52faae7306d6a7871885dc2e9dc83a164bac7263776474ef642b660040b35a1410ac291ac8f6f18ab85e7fd6e22bd1af1c41ca95cf2448f6e2b42a018493dfc03c6b6aa1b9e3fe7b76af2182fb2121db4166bf0167d6f379c5a58adee5082423434d97be2909f5e7488053f996646db10dd49782626da53ad8eada01813c031b2bacdb0203bc017aac1735951a11d013ee4d1d5f7143ccbebf2371e66a1bec6e1febe69148f50784eef8adbb66664c96196d7e0c0bcdc807f447b54e058f37642a3337995bfbcd332208bd6016936705c82263eabd7affdba92fae3\":char*:\"\":exp:67",
    "depends_on:0:1:6",
    "12:hex:\"308201b330819c020101300d06092a864886f70d01010b0500303b310b3009060355040613024e4c3111300f060355040a1308506f6c617253534c3119301706035504031310506f6c617253534c2054657374204341170d3138303331343037333134385a170d3238303331343037333134385aa02d302b30290603551d1c0101ff0420301da01ba0198617687474703a2f2f706b692e6578616d706c652e636f6d2f300d06092a864886f70d01010b05000382010100b3fbe9d586eaf4b8ff60cf8edae06a85135db78f78198498719725b5b403c0b803c2c150f52faae7306d6a7871885dc2e9dc83a164bac7263776474ef642b660040b35a1410ac291ac8f6f18ab85e7fd6e22bd1af1c41ca95cf2448f6e2b42a018493dfc03c6b6aa1b9e3fe7b76af2182fb2121db4166bf0167d6f379c5a58adee5082423434d97be2909f5e7488053f996646db10dd49782626da53ad8eada01813c031b2bacdb0203bc017aac1735951a11d013ee4d1d5f7143ccbebf2371e66a1bec6e1febe69148f50784eef8adbb66664c96196d7e0c0bcdc807f447b54e058f37642a3337995bfbcd332208bd6016936705c82263eabd7affdba92fae3\":char*:\"\":exp:58",
    "depends_on:0:1:6",
    "12:hex:\"308201b330819c020101300d06092a864886f70d01010b0500303b310b3009060355040613024e4c3111300f060355040a1308506f6c617253534c3119301706035504031310506f6c617253534c2054657374204341170d3138303331343037333134385a170d3238303331343037333134385aa02d302b30290603551d1c0101ff041e301da01ba0198617687474703a2f2f706b692e6578616d706c652e636f6d2f300d06092a864886f70d01010b05000382010100b3fbe9d586eaf4b8ff60cf8edae06a85135db78f78198498719725b5b403c0b803c2c150f52faae7306d6a7871885dc2e9dc83a164bac7263776474ef642b660040b35a1410ac291ac8f6f18ab85e7fd6e22bd1af1c41ca95cf2448f6e2b42a018493dfc03c6b6aa1b9e3fe7b76af2182fb2121db4166bf0167d6f379c5a58adee5082423434d97be2909f5e7488053f996646db10dd49782626da53ad8eada01813c031b2bacdb0203bc017aac1735951a11d013ee4d1d5f7143ccbebf2371e66a1bec6e1febe69148f50784eef8adbb66664c96196d7e0c0bcdc807f447b54e058f37642a3337995bfbcd332208bd6016936705c82263eabd7affdba92fae3\":char*:\"\":exp:59",
    "depends_on:0:1:6",
    "12:hex:\"308201b330819c020101300d06092a864886f70d01010b0500303b310b3009060355040613024e4c3111300f060355040a1308506f6c617253534c3119301706035504031310506f6c617253534c2054657374204341170d3138303331343037333134385a170d3238303331343037333134385aa02d302b30290603551d1c010100041f301da01ba0198617687474703a2f2f706b692e6578616d706c652e636f6d2f300d06092a864886f70d01010b05000382010100b3fbe9d586eaf4b8ff60cf8edae06a85135db78f78198498719725b5b403c0b803c2c150f52faae7306d6a7871885dc2e9dc83a164bac7263776474ef642b660040b35a1410ac291ac8f6f18ab85e7fd6e22bd1af1c41ca95cf2448f6e2b42a018493dfc03c6b6aa1b9e3fe7b76af2182fb2121db4166bf0167d6f379c5a58adee5082423434d97be2909f5e7488053f996646db10dd49782626da53ad8eada01813c031b2bacdb0203bc017aac1735951a11d013ee4d1d5f7143ccbebf2371e66a1bec6e1febe69148f50784eef8adbb66664c96196d7e0c0bcdc807f447b54e058f37642a3337995bfbcd332208bd6016936705c82263eabd7affdba92fae3\":char*:\"CRL version   \\: 2\nissuer name   \\: C=NL, O=PolarSSL, CN=PolarSSL Test CA\nthis update   \\: 2018-03-14 07\\:31\\:48\nnext update   \\: 2028-03-14 07\\:31\\:48\nRevoked certificates\\:\nsigned using  \\: RSA with SHA-256\n\":int:0",
    "depends_on:2:1",
    "14:char*:\"data_files/dir1\":int:0:int:1",
    "depends_on:2:1:6:9:14",
    "14:char*:\"data_files/dir2\":int:0:int:2",
    "depends_on:2:1:6:9:14",
    "14:char*:\"data_files/dir3\":int:1:int:2",
    "depends_on:6:9:10",
    "15:char*:\"data_files/dir-maxpath/00.crt\":char*:\"data_files/dir-maxpath\":exp:68:int:0:int:0",
    "depends_on:6:9:10:14",
    "15:char*:\"data_files/test-ca2.crt\":char*:\"data_files/dir-maxpath\":exp:69:exp:5:exp:16",
    "depends_on:6:9:10",
    "15:char*:\"data_files/dir-maxpath/00.crt\":char*:\"data_files/dir-maxpath\":exp:70:exp:71:exp:72",
    "depends_on:6:1",
    "16:char*:\"data_files/dir4/cert14.crt data_files/dir4/cert13.crt data_files/dir4/cert12.crt\":char*:\"data_files/dir4/cert11.crt\":exp:16:exp:5:char*:\"\":int:0",
    "depends_on:6:1",
    "16:char*:\"data_files/dir4/cert23.crt data_files/dir4/cert22.crt\":char*:\"data_files/dir4/cert21.crt\":exp:16:exp:5:char*:\"\":int:0",
    "depends_on:6:1",
    "16:char*:\"data_files/dir4/cert34.crt data_files/dir4/cert33.crt data_files/dir4/cert32.crt\":char*:\"data_files/dir4/cert31.crt\":exp:16:exp:5:char*:\"\":int:0",
    "depends_on:6:1",
    "16:char*:\"data_files/dir4/cert45.crt data_files/dir4/cert44.crt data_files/dir4/cert43.crt data_files/dir4/cert42.crt\":char*:\"data_files/dir4/cert41.crt\":exp:16:exp:5:char*:\"\":int:0",
    "depends_on:6:1:15",
    "16:char*:\"data_files/dir4/cert54.crt data_files/dir4/cert53.crt data_files/dir4/cert52.crt\":char*:\"data_files/dir4/cert51.crt\":int:0:int:0:char*:\"\":int:0",
    "depends_on:6:1:15",
    "16:char*:\"data_files/dir4/cert63.crt data_files/dir4/cert62.crt\":char*:\"data_files/dir4/cert61.crt\":int:0:int:0:char*:\"\":int:0",
    "depends_on:6:1:15",
    "16:char*:\"data_files/dir4/cert74.crt data_files/dir4/cert73.crt data_files/dir4/cert72.crt\":char*:\"data_files/dir4/cert71.crt\":int:0:int:0:char*:\"\":int:0",
    "depends_on:6:1:15",
    "16:char*:\"data_files/dir4/cert61.crt data_files/dir4/cert63.crt data_files/dir4/cert62.crt\":char*:\"data_files/dir4/cert61.crt\":int:0:int:0:char*:\"\":int:0",
    "depends_on:6:9:10",
    "16:char*:\"data_files/dir4/cert83.crt data_files/dir4/cert82.crt\":char*:\"data_files/dir4/cert81.crt\":int:0:int:0:char*:\"\":int:0",
    "depends_on:6:9:10",
    "16:char*:\"data_files/dir4/cert92.crt\":char*:\"data_files/dir4/cert91.crt\":int:0:int:0:char*:\"\":int:0",
    "depends_on:6:9:10",
    "16:char*:\"data_files/dir4/cert92.crt\":char*:\"data_files/dir4/cert91.crt\":exp:72:exp:73:char*:\"nonesuch\":int:0",
    "depends_on:6:1:15:9:11:2",
    "16:char*:\"data_files/server3.crt\":char*:\"data_files/test-ca.crt\":exp:74:exp:5:char*:\"suiteb\":int:0",
    "depends_on:6:1:9:14",
    "16:char*:\"data_files/server4.crt\":char*:\"data_files/test-ca2.crt\":exp:75:exp:5:char*:\"rsa3072\":int:0",
    "depends_on:6:1:9:10",
    "16:char*:\"data_files/server5-selfsigned.crt\":char*:\"data_files/server5-selfsigned.crt\":exp:75:exp:5:char*:\"rsa3072\":int:0",
    "depends_on:6:1:15:2",
    "16:char*:\"data_files/server1.crt\":char*:\"data_files/test-ca.crt\":exp:76:exp:5:char*:\"rsa3072\":int:0",
    "depends_on:6:1:15:9:14:10",
    "16:char*:\"data_files/server7.crt data_files/test-int-ca.crt\":char*:\"data_files/test-ca2.crt\":exp:25:exp:5:char*:\"suiteb\":int:0",
    "depends_on:6:1:15:9:14:10:2",
    "16:char*:\"data_files/server8.crt data_files/test-int-ca2.crt\":char*:\"data_files/test-ca.crt\":exp:75:exp:5:char*:\"rsa3072\":int:0",
    "depends_on:6:1:15:9:10:14",
    "16:char*:\"data_files/server7.crt data_files/test-int-ca.crt\":char*:\"data_files/test-ca2.crt\":exp:17:exp:5:char*:\"sha512\":int:0",
    "depends_on:6:9:10:14:7",
    "16:char*:\"data_files/server5.crt\":char*:\"data_files/test-ca2.crt\":exp:72:exp:77:char*:\"\":int:2",
    "depends_on:6:9:10:14:7",
    "16:char*:\"data_files/server5.crt\":char*:\"data_files/test-ca2.crt\":exp:72:exp:72:char*:\"\":int:1",
    "depends_on:6:9:10:14:7:2:1",
    "16:char*:\"data_files/server5.crt\":char*:\"data_files/test-ca.crt\":exp:72:exp:72:char*:\"\":int:1",
    "depends_on:6:9:10:1:15:2:14",
    "16:char*:\"data_files/server10_int3_int-ca2_ca.crt\":char*:\"data_files/test-ca.crt\":exp:72:exp:78:char*:\"\":int:8",
    "depends_on:6:9:10:1:2:14",
    "16:char*:\"data_files/server10_int3_int-ca2_ca.crt\":char*:\"data_files/test-ca.crt\":exp:72:exp:79:char*:\"\":int:4",
    "depends_on:6:9:10:1:2:14",
    "16:char*:\"data_files/server10_int3_int-ca2_ca.crt\":char*:\"data_files/test-ca.crt\":exp:72:exp:77:char*:\"\":int:2",
    "depends_on:6:9:10:1:2:14",
    "16:char*:\"data_files/server10_int3_int-ca2_ca.crt\":char*:\"data_files/test-ca.crt\":exp:72:exp:72:char*:\"\":int:1",
    "depends_on:6:9:10:1:2:14",
    "16:char*:\"data_files/server10_int3_int-ca2_ca.crt\":char*:\"data_files/test-ca2.crt\":exp:72:exp:78:char*:\"\":int:8",
    "17:hex:\"2B06010505070301\":char*:\"TLS Web Server Authentication\"",
    "17:hex:\"2B0601050507030f\":char*:\"notfound\"",
    "17:hex:\"2B0601050507030100\":char*:\"notfound\"",
    "18:hex:\"2B06010505070301\":char*:\"1.3.6.1.5.5.7.3.1\":int:20:int:17",
    "18:hex:\"2B06010505070301\":char*:\"1.3.6.1.5.5.7.3.1\":int:18:int:17",
    "18:hex:\"2B06010505070301\":char*:\"1.3.6.1.5.5.7.3.1\":int:17:exp:80",
    "18:hex:\"2A864886F70D\":char*:\"1.2.840.113549\":int:15:int:14",
    "18:hex:\"2A8648F9F8F7F6F5F4F3F2F1F001\":char*:\"\":int:100:exp:80",
    "depends_on:1:2",
    "19:char*:\"data_files/server1.crt\":exp:81:int:0",
    "depends_on:1:2",
    "19:char*:\"data_files/server1.crt\":exp:82:int:0",
    "depends_on:1:2",
    "19:char*:\"data_files/server1.key_usage.crt\":int:0:int:0",
    "depends_on:1:2",
    "19:char*:\"data_files/server1.key_usage.crt\":exp:83:int:0",
    "depends_on:1:2",
    "19:char*:\"data_files/server1.key_usage.crt\":exp:82:exp:73",
    "depends_on:1:2",
    "19:char*:\"data_files/server1.key_usage.crt\":exp:81:int:0",
    "depends_on:1:2",
    "19:char*:\"data_files/server1.key_usage.crt\":exp:84:exp:73",
    "depends_on:1:2",
    "19:char*:\"data_files/server1.key_usage.crt\":exp:85:exp:73",
    "depends_on:1:2",
    "19:char*:\"data_files/server1.key_usage.crt\":exp:86:int:0",
    "depends_on:1:2",
    "19:char*:\"data_files/keyUsage.decipherOnly.crt\":exp:81:exp:73",
    "depends_on:1:2",
    "19:char*:\"data_files/keyUsage.decipherOnly.crt\":exp:86:int:0",
    "depends_on:9:10:6",
    "20:char*:\"data_files/server5.crt\":hex:\"2B06010505070301\":int:0",
    "depends_on:9:10:6",
    "20:char*:\"data_files/server5.eku-srv.crt\":hex:\"2B06010505070301\":int:0",
    "depends_on:9:10:6",
    "20:char*:\"data_files/server5.eku-cli.crt\":hex:\"2B06010505070301\":exp:73",
    "depends_on:9:10:6",
    "20:char*:\"data_files/server5.eku-srv_cli.crt\":hex:\"2B06010505070301\":int:0",
    "depends_on:9:10:6",
    "20:char*:\"data_files/server5.eku-srv_cli.crt\":hex:\"2B06010505070302\":int:0",
    "depends_on:9:10:6",
    "20:char*:\"data_files/server5.eku-srv_cli.crt\":hex:\"2B06010505070303\":exp:73",
    "depends_on:9:10:6",
    "20:char*:\"data_files/server5.eku-cs_any.crt\":hex:\"2B060105050703FF\":int:0",
    "22:hex:\"\":exp:87:exp:88:exp:88:int:20:int:0",
    "22:hex:\"\":exp:89:exp:88:exp:88:int:20:exp:38",
    "22:hex:\"A400\":exp:87:exp:88:exp:88:int:20:exp:40",
    "depends_on:1:6",
    "22:hex:\"A00D300B0609608648016503040201\":exp:87:exp:90:exp:88:int:20:int:0",
    "depends_on:1:2",
    "22:hex:\"A009300706052B0E03021A\":exp:87:exp:88:exp:88:int:20:int:0",
    "22:hex:\"A00A300706052B0E03021A\":exp:87:exp:88:exp:88:int:20:exp:37",
    "depends_on:1:2",
    "22:hex:\"A00A300706052B0E03021A00\":exp:87:exp:88:exp:88:int:20:exp:40",
    "22:hex:\"A00F300D06096086480165030402013000\":exp:87:exp:90:exp:88:int:20:exp:91",
    "22:hex:\"A00D300B06096086480165030402FF\":exp:87:exp:90:exp:88:int:20:exp:92",
    "depends_on:1:6",
    "22:hex:\"A11A301806092A864886F70D010108300B0609608648016503040201\":exp:87:exp:88:exp:90:int:20:int:0",
    "depends_on:1:2",
    "22:hex:\"A116301406092A864886F70D010108300706052B0E03021A\":exp:87:exp:88:exp:88:int:20:int:0",
    "22:hex:\"A11B301806092A864886F70D010108300B0609608648016503040201\":exp:87:exp:88:exp:90:int:20:exp:37",
    "depends_on:1:6",
    "22:hex:\"A11B301806092A864886F70D010108300B060960864801650304020100\":exp:87:exp:88:exp:90:int:20:exp:40",
    "22:hex:\"A11A301906092A864886F70D010108300B0609608648016503040201\":exp:87:exp:88:exp:90:int:20:exp:37",
    "22:hex:\"A11A301806092A864886F70D010109300B0609608648016503040201\":exp:87:exp:88:exp:90:int:20:exp:93",
    "22:hex:\"A11A301806092A864886F70D010108310B0609608648016503040201\":exp:87:exp:88:exp:90:int:20:exp:38",
    "22:hex:\"A10F300D06092A864886F70D0101083000\":exp:87:exp:88:exp:90:int:20:exp:37",
    "22:hex:\"A11B301906092A864886F70D010108300C0609608648016503040201\":exp:87:exp:88:exp:90:int:20:exp:37",
    "22:hex:\"A11A301806092A864886F70D010108300B0709608648016503040201\":exp:87:exp:88:exp:90:int:20:exp:38",
    "22:hex:\"A11A301806092A864886F70D010108300B06096086480165030402FF\":exp:87:exp:88:exp:90:int:20:exp:92",
    "depends_on:1:6",
    "22:hex:\"A11C301A06092A864886F70D010108300D06096086480165030402010500\":exp:87:exp:88:exp:90:int:20:int:0",
    "depends_on:1:6",
    "22:hex:\"A11C301A06092A864886F70D010108300D06096086480165030402013000\":exp:87:exp:88:exp:90:int:20:exp:38",
    "22:hex:\"A11D301B06092A864886F70D010108300E06096086480165030402010500\":exp:87:exp:88:exp:90:int:20:exp:37",
    "depends_on:1:6",
    "22:hex:\"A11D301B06092A864886F70D010108300E0609608648016503040201050000\":exp:87:exp:88:exp:90:int:20:exp:40",
    "22:hex:\"A20302015E\":exp:87:exp:88:exp:88:int:94:int:0",
    "22:hex:\"A203020114\":exp:87:exp:88:exp:88:int:20:int:0",
    "22:hex:\"A20402015E\":exp:87:exp:88:exp:88:int:94:exp:37",
    "22:hex:\"A20402015E00\":exp:87:exp:88:exp:88:int:94:exp:40",
    "22:hex:\"A2023000\":exp:87:exp:88:exp:88:int:94:exp:38",
    "22:hex:\"A303020101\":exp:87:exp:88:exp:88:int:20:int:0",
    "22:hex:\"A304020101\":exp:87:exp:88:exp:88:int:20:exp:37",
    "22:hex:\"A30402010100\":exp:87:exp:88:exp:88:int:20:exp:40",
    "22:hex:\"A3023000\":exp:87:exp:88:exp:88:int:20:exp:38",
    "22:hex:\"A303020102\":exp:87:exp:88:exp:88:int:20:exp:94",
    "depends_on:9:10:2",
    "13:hex:\"308201183081BF0201003034310B3009060355040613024E4C3111300F060355040A1308506F6C617253534C31123010060355040313096C6F63616C686F73743059301306072A8648CE3D020106082A8648CE3D0301070342000437CC56D976091E5A723EC7592DFF206EEE7CF9069174D0AD14B5F768225962924EE500D82311FFEA2FD2345D5D16BD8A88C26B770D55CD8A2A0EFA01C8B4EDFFA029302706092A864886F70D01090E311A301830090603551D1304023000300B0603551D0F0404030205E0300906072A8648CE3D04010349003046022100B49FD8C8F77ABFA871908DFBE684A08A793D0F490A43D86FCF2086E4F24BB0C2022100F829D5CCD3742369299E6294394717C4B723A0F68B44E831B6E6C3BCABF97243\":char*:\"CSR version   \\: 1\nsubject name  \\: C=NL, O=PolarSSL, CN=localhost\nsigned using  \\: ECDSA with SHA1\nEC key size   \\: 256 bits\n\":int:0",
    "13:hex:\"3100\":char*:\"\":exp:27",
    "13:hex:\"3001\":char*:\"\":exp:27",
    "13:hex:\"30010000\":char*:\"\":exp:56",
    "13:hex:\"30023100\":char*:\"\":exp:30",
    "13:hex:\"30023001\":char*:\"\":exp:28",
    "13:hex:\"30053002020100\":char*:\"\":exp:33",
    "13:hex:\"30053003020101\":char*:\"\":exp:39",
    "13:hex:\"300730050201003100\":char*:\"\":exp:30",
    "13:hex:\"30083005020100300100\":char*:\"\":exp:28",
    "13:hex:\"3009300702010030023000\":char*:\"\":exp:43",
    "13:hex:\"300A30080201003002310100\":char*:\"\":exp:42",
    "13:hex:\"30143012020100300D310B3009060355040613024E4C\":char*:\"\":exp:48",
    "13:hex:\"30163014020100300D310B3009060355040613024E4C3100\":char*:\"\":exp:95",
    "13:hex:\"30173014020100300D310B3009060355040613024E4C300100\":char*:\"\":exp:48",
    "depends_on:9:10",
    "13:hex:\"3081973081940201003034310B3009060355040613024E4C3111300F060355040A1308506F6C617253534C31123010060355040313096C6F63616C686F73743059301306072A8648CE3D020106082A8648CE3D0301070342000437CC56D976091E5A723EC7592DFF206EEE7CF9069174D0AD14B5F768225962924EE500D82311FFEA2FD2345D5D16BD8A88C26B770D55CD8A2A0EFA01C8B4EDFF\":char*:\"\":exp:28",
    "depends_on:9:10",
    "13:hex:\"3081993081960201003034310B3009060355040613024E4C3111300F060355040A1308506F6C617253534C31123010060355040313096C6F63616C686F73743059301306072A8648CE3D020106082A8648CE3D0301070342000437CC56D976091E5A723EC7592DFF206EEE7CF9069174D0AD14B5F768225962924EE500D82311FFEA2FD2345D5D16BD8A88C26B770D55CD8A2A0EFA01C8B4EDFF0500\":char*:\"\":exp:30",
    "depends_on:9:10",
    "13:hex:\"30819A3081960201003034310B3009060355040613024E4C3111300F060355040A1308506F6C617253534C31123010060355040313096C6F63616C686F73743059301306072A8648CE3D020106082A8648CE3D0301070342000437CC56D976091E5A723EC7592DFF206EEE7CF9069174D0AD14B5F768225962924EE500D82311FFEA2FD2345D5D16BD8A88C26B770D55CD8A2A0EFA01C8B4EDFFA00100\":char*:\"\":exp:28",
    "depends_on:9:10",
    "13:hex:\"3081C23081BF0201003034310B3009060355040613024E4C3111300F060355040A1308506F6C617253534C31123010060355040313096C6F63616C686F73743059301306072A8648CE3D020106082A8648CE3D0301070342000437CC56D976091E5A723EC7592DFF206EEE7CF9069174D0AD14B5F768225962924EE500D82311FFEA2FD2345D5D16BD8A88C26B770D55CD8A2A0EFA01C8B4EDFFA029302706092A864886F70D01090E311A301830090603551D1304023000300B0603551D0F0404030205E0\":char*:\"\":exp:37",
    "depends_on:9:10",
    "13:hex:\"3081C43081BF0201003034310B3009060355040613024E4C3111300F060355040A1308506F6C617253534C31123010060355040313096C6F63616C686F73743059301306072A8648CE3D020106082A8648CE3D0301070342000437CC56D976091E5A723EC7592DFF206EEE7CF9069174D0AD14B5F768225962924EE500D82311FFEA2FD2345D5D16BD8A88C26B770D55CD8A2A0EFA01C8B4EDFFA029302706092A864886F70D01090E311A301830090603551D1304023000300B0603551D0F0404030205E03100\":char*:\"\":exp:38",
    "depends_on:9:10",
    "13:hex:\"3081C43081BF0201003034310B3009060355040613024E4C3111300F060355040A1308506F6C617253534C31123010060355040313096C6F63616C686F73743059301306072A8648CE3D020106082A8648CE3D0301070342000437CC56D976091E5A723EC7592DFF206EEE7CF9069174D0AD14B5F768225962924EE500D82311FFEA2FD2345D5D16BD8A88C26B770D55CD8A2A0EFA01C8B4EDFFA029302706092A864886F70D01090E311A301830090603551D1304023000300B0603551D0F0404030205E03001\":char*:\"\":exp:37",
    "depends_on:9:10",
    "13:hex:\"3081CD3081BF0201003034310B3009060355040613024E4C3111300F060355040A1308506F6C617253534C31123010060355040313096C6F63616C686F73743059301306072A8648CE3D020106082A8648CE3D0301070342000437CC56D976091E5A723EC7592DFF206EEE7CF9069174D0AD14B5F768225962924EE500D82311FFEA2FD2345D5D16BD8A88C26B770D55CD8A2A0EFA01C8B4EDFFA029302706092A864886F70D01090E311A301830090603551D1304023000300B0603551D0F0404030205E0300906072A8648CE3D04FF\":char*:\"\":exp:64",
    "depends_on:9:10:2",
    "13:hex:\"3081CD3081BF0201003034310B3009060355040613024E4C3111300F060355040A1308506F6C617253534C31123010060355040313096C6F63616C686F73743059301306072A8648CE3D020106082A8648CE3D0301070342000437CC56D976091E5A723EC7592DFF206EEE7CF9069174D0AD14B5F768225962924EE500D82311FFEA2FD2345D5D16BD8A88C26B770D55CD8A2A0EFA01C8B4EDFFA029302706092A864886F70D01090E311A301830090603551D1304023000300B0603551D0F0404030205E0300906072A8648CE3D0401\":char*:\"\":exp:62",
    "depends_on:9:10:2",
    "13:hex:\"3081CF3081BF0201003034310B3009060355040613024E4C3111300F060355040A1308506F6C617253534C31123010060355040313096C6F63616C686F73743059301306072A8648CE3D020106082A8648CE3D0301070342000437CC56D976091E5A723EC7592DFF206EEE7CF9069174D0AD14B5F768225962924EE500D82311FFEA2FD2345D5D16BD8A88C26B770D55CD8A2A0EFA01C8B4EDFFA029302706092A864886F70D01090E311A301830090603551D1304023000300B0603551D0F0404030205E0300906072A8648CE3D04010400\":char*:\"\":exp:96",
    "depends_on:9:10:2",
    "13:hex:\"3081CF3081BF0201003034310B3009060355040613024E4C3111300F060355040A1308506F6C617253534C31123010060355040313096C6F63616C686F73743059301306072A8648CE3D020106082A8648CE3D0301070342000437CC56D976091E5A723EC7592DFF206EEE7CF9069174D0AD14B5F768225962924EE500D82311FFEA2FD2345D5D16BD8A88C26B770D55CD8A2A0EFA01C8B4EDFFA029302706092A864886F70D01090E311A301830090603551D1304023000300B0603551D0F0404030205E0300906072A8648CE3D04010301\":char*:\"\":exp:62",
    "depends_on:9:10:2",
    "13:hex:\"308201193081BF0201003034310B3009060355040613024E4C3111300F060355040A1308506F6C617253534C31123010060355040313096C6F63616C686F73743059301306072A8648CE3D020106082A8648CE3D0301070342000437CC56D976091E5A723EC7592DFF206EEE7CF9069174D0AD14B5F768225962924EE500D82311FFEA2FD2345D5D16BD8A88C26B770D55CD8A2A0EFA01C8B4EDFFA029302706092A864886F70D01090E311A301830090603551D1304023000300B0603551D0F0404030205E0300906072A8648CE3D04010349003046022100B49FD8C8F77ABFA871908DFBE684A08A793D0F490A43D86FCF2086E4F24BB0C2022100F829D5CCD3742369299E6294394717C4B723A0F68B44E831B6E6C3BCABF9724300\":char*:\"\":exp:56",
    "13:hex:\"3008300602047FFFFFFF\":char*:\"\":exp:39",
    "depends_on:9:10:6:1",
    "10:char*:\"data_files/server7_int-ca.crt\":int:0",
    "depends_on:9:6:1",
    "10:char*:\"data_files/server7_pem_space.crt\":int:1",
    "depends_on:9:1",
    "10:char*:\"data_files/server7_all_space.crt\":exp:97",
    "depends_on:9:10:6:1",
    "10:char*:\"data_files/server7_trailing_space.crt\":int:0",
    "depends_on:21",
    "21:exp:98:char*:\"500101000000Z\":int:0:int:1950:int:1:int:1:int:0:int:0:int:0",
    "depends_on:21",
    "21:exp:99:char*:\"99991231235959Z\":int:0:int:9999:int:12:int:31:int:23:int:59:int:59",
    "depends_on:21",
    "21:exp:98:char*:\"490229121212Z\":exp:47:int:0:int:0:int:0:int:0:int:0:int:0",
    "depends_on:21",
    "21:exp:98:char*:\"000229121212Z\":int:0:int:2000:int:2:int:29:int:12:int:12:int:12",
    "depends_on:21",
    "21:exp:98:char*:\"000132121212Z\":exp:47:int:0:int:0:int:0:int:0:int:0:int:0",
    "depends_on:21",
    "21:exp:98:char*:\"001131121212Z\":exp:47:int:0:int:0:int:0:int:0:int:0:int:0",
    "depends_on:21",
    "21:exp:98:char*:\"001130241212Z\":exp:47:int:0:int:0:int:0:int:0:int:0:int:0",
    "depends_on:21",
    "21:exp:98:char*:\"001130236012Z\":exp:47:int:0:int:0:int:0:int:0:int:0:int:0",
    "depends_on:21",
    "21:exp:98:char*:\"001130235960Z\":exp:47:int:0:int:0:int:0:int:0:int:0:int:0",
    "depends_on:21",
    "21:exp:98:char*:\"000229121212\":int:0:int:2000:int:2:int:29:int:12:int:12:int:12",
    "depends_on:21",
    "21:exp:98:char*:\"000229121212J\":exp:47:int:0:int:0:int:0:int:0:int:0:int:0",
    "depends_on:21",
    "21:exp:98:char*:\"000229121212+0300\":exp:47:int:0:int:0:int:0:int:0:int:0:int:0",
    "depends_on:21",
    "21:exp:100:char*:\"000229121212\":exp:101:int:0:int:0:int:0:int:0:int:0:int:0",
    "depends_on:21",
    "21:exp:98:char*:\"000229121\":exp:47:int:0:int:0:int:0:int:0:int:0:int:0",
    "depends_on:21",
    "21:exp:99:char*:\"20000229121\":exp:47:int:0:int:0:int:0:int:0:int:0:int:0",
    "depends_on:21",
    "21:exp:98:char*:\"0002291212\":exp:47:int:2000:int:2:int:29:int:12:int:12:int:0",
    "depends_on:21",
    "21:exp:98:char*:\"0002291212J\":exp:47:int:0:int:0:int:0:int:0:int:0:int:0",
    "depends_on:21",
    "21:exp:98:char*:\"0002291212+0300\":exp:47:int:0:int:0:int:0:int:0:int:0:int:0",
    "depends_on:21",
    "21:exp:98:char*:\"0\1130231212Z\":exp:47:int:0:int:0:int:0:int:0:int:0:int:0",
    "depends_on:21",
    "21:exp:98:char*:\"001%30231212Z\":exp:47:int:0:int:0:int:0:int:0:int:0:int:0",
    "depends_on:21",
    "21:exp:98:char*:\"0011`0231212Z\":exp:47:int:0:int:0:int:0:int:0:int:0:int:0",
    "depends_on:21",
    "21:exp:98:char*:\"0011302h1212Z\":exp:47:int:0:int:0:int:0:int:0:int:0:int:0",
    "depends_on:21",
    "21:exp:98:char*:\"00113023u012Z\":exp:47:int:0:int:0:int:0:int:0:int:0:int:0",
    "depends_on:21",
    "21:exp:98:char*:\"0011302359n0Z\":exp:47:int:0:int:0:int:0:int:0:int:0:int:0",
    "depends_on:21",
    "21:exp:99:char*:\"19000229000000Z\":exp:47:int:0:int:0:int:0:int:0:int:0:int:0",
    "depends_on:21",
    "21:exp:99:char*:\"19920229000000Z\":int:0:int:1992:int:2:int:29:int:0:int:0:int:0",
    "depends_on:21",
    "21:exp:99:char*:\"20000229000000Z\":int:0:int:2000:int:2:int:29:int:0:int:0:int:0",
    "depends_on:21",
    "21:exp:99:char*:\"19910229000000Z\":exp:47:int:0:int:0:int:0:int:0:int:0:int:0",
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

/* TEST-FUNCTIONS */
static void test_fail( const char *test, int line_no, const char* filename );
int verify_all( void *data, mbedtls_x509_crt *crt, int certificate_depth, uint32_t *flags );
int verify_fatal( void *data, mbedtls_x509_crt *crt, int certificate_depth, uint32_t *flags );
int verify_none( void *data, mbedtls_x509_crt *crt, int certificate_depth, uint32_t *flags );
static int dep_check( int dep_id );
static int get_expression( int32_t exp_id, int32_t * out_value );
static int check_test_x509( int func_idx );
static int dispatch_test_x509( int func_idx, void ** params );
static int convert_params( size_t cnt , char ** params , int * int_params_store );

void test_x509_cert_info_wrapper( void ** params );
void test_mbedtls_x509_crl_info_wrapper( void ** params );
void test_mbedtls_x509_crl_parse_wrapper( void ** params );
void test_mbedtls_x509_csr_info_wrapper( void ** params );
void test_x509_verify_info_wrapper( void ** params );
void test_x509_verify_wrapper( void ** params );
void test_x509_verify_callback_wrapper( void ** params );
void test_mbedtls_x509_dn_gets_wrapper( void ** params );
void test_mbedtls_x509_time_is_past_wrapper( void ** params );
void test_mbedtls_x509_time_is_future_wrapper( void ** params );
void test_x509parse_crt_file_wrapper( void ** params );
void test_x509parse_crt_wrapper( void ** params );
void test_x509parse_crl_wrapper( void ** params );
void test_mbedtls_x509_csr_parse_wrapper( void ** params );
void test_mbedtls_x509_crt_parse_path_wrapper( void ** params );
void test_mbedtls_x509_crt_verify_max_wrapper( void ** params );
void test_mbedtls_x509_crt_verify_chain_wrapper( void ** params );
void test_x509_oid_desc_wrapper( void ** params );
void test_x509_oid_numstr_wrapper( void ** params );
void test_x509_check_key_usage_wrapper( void ** params );
void test_x509_check_extended_key_usage_wrapper( void ** params );
void test_x509_get_time_wrapper( void ** params );
void test_x509_parse_rsassa_pss_params_wrapper( void ** params );
void test_x509_selftest_wrapper( void ** params );
char *mystrsep(char **stringp, const char *delim);

/* ========================================================================== */
/*                             FUNCTION DEFINITIONS                           */
/* ========================================================================== */

/* Profile for backward compatibility. Allows SHA-1, unlike the default
   profile. */
const mbedtls_x509_crt_profile compat_profile =
{
    MBEDTLS_X509_ID_FLAG( MBEDTLS_MD_SHA1 ) |
    MBEDTLS_X509_ID_FLAG( MBEDTLS_MD_RIPEMD160 ) |
    MBEDTLS_X509_ID_FLAG( MBEDTLS_MD_SHA224 ) |
    MBEDTLS_X509_ID_FLAG( MBEDTLS_MD_SHA256 ) |
    MBEDTLS_X509_ID_FLAG( MBEDTLS_MD_SHA384 ) |
    MBEDTLS_X509_ID_FLAG( MBEDTLS_MD_SHA512 ),
    0xFFFFFFF, /* Any PK alg    */
    0xFFFFFFF, /* Any curve     */
    1024,
};

const mbedtls_x509_crt_profile profile_rsa3072 =
{
    MBEDTLS_X509_ID_FLAG( MBEDTLS_MD_SHA256 ) |
    MBEDTLS_X509_ID_FLAG( MBEDTLS_MD_SHA384 ) |
    MBEDTLS_X509_ID_FLAG( MBEDTLS_MD_SHA512 ),
    MBEDTLS_X509_ID_FLAG( MBEDTLS_PK_RSA ),
    0,
    3072,
};

const mbedtls_x509_crt_profile profile_sha512 =
{
    MBEDTLS_X509_ID_FLAG( MBEDTLS_MD_SHA512 ),
    0xFFFFFFF, /* Any PK alg    */
    0xFFFFFFF, /* Any curve     */
    1024,
};

static void test_fail( const char *test, int line_no, const char* filename )
{
    test_info.failed = 1;
    test_info.test = test;
    test_info.line_no = line_no;
    test_info.filename = filename;
}

TestWrapper_t test_funcs_x509[] =
{
    /* Function Id: 0 */

    #if defined(MBEDTLS_BIGNUM_C) && defined(MBEDTLS_FS_IO) && defined(MBEDTLS_X509_CRT_PARSE_C)
        test_x509_cert_info_wrapper,
    #else
        NULL,
    #endif
    /* Function Id: 1 */

    #if defined(MBEDTLS_BIGNUM_C) && defined(MBEDTLS_FS_IO) && defined(MBEDTLS_X509_CRL_PARSE_C)
        test_mbedtls_x509_crl_info_wrapper,
    #else
        NULL,
    #endif
    /* Function Id: 2 */

    #if defined(MBEDTLS_BIGNUM_C) && defined(MBEDTLS_FS_IO) && defined(MBEDTLS_X509_CRL_PARSE_C)
        test_mbedtls_x509_crl_parse_wrapper,
    #else
        NULL,
    #endif
    /* Function Id: 3 */

    #if defined(MBEDTLS_BIGNUM_C) && defined(MBEDTLS_FS_IO) && defined(MBEDTLS_X509_CSR_PARSE_C)
        test_mbedtls_x509_csr_info_wrapper,
    #else
        NULL,
    #endif
    /* Function Id: 4 */

    #if defined(MBEDTLS_BIGNUM_C) && defined(MBEDTLS_X509_CRT_PARSE_C)
        test_x509_verify_info_wrapper,
    #else
        NULL,
    #endif
    /* Function Id: 5 */

    #if defined(MBEDTLS_BIGNUM_C) && defined(MBEDTLS_FS_IO) && defined(MBEDTLS_X509_CRT_PARSE_C) && defined(MBEDTLS_X509_CRL_PARSE_C)
        test_x509_verify_wrapper,
    #else
        NULL,
    #endif
    /* Function Id: 6 */

    #if defined(MBEDTLS_BIGNUM_C) && defined(MBEDTLS_FS_IO) && defined(MBEDTLS_X509_CRT_PARSE_C)
        test_x509_verify_callback_wrapper,
    #else
        NULL,
    #endif
    /* Function Id: 7 */

    #if defined(MBEDTLS_BIGNUM_C) && defined(MBEDTLS_FS_IO) && defined(MBEDTLS_X509_CRT_PARSE_C)
        test_mbedtls_x509_dn_gets_wrapper,
    #else
        NULL,
    #endif
    /* Function Id: 8 */

    #if defined(MBEDTLS_BIGNUM_C) && defined(MBEDTLS_FS_IO) && defined(MBEDTLS_X509_CRT_PARSE_C)
        test_mbedtls_x509_time_is_past_wrapper,
    #else
        NULL,
    #endif
    /* Function Id: 9 */

    #if defined(MBEDTLS_BIGNUM_C) && defined(MBEDTLS_FS_IO) && defined(MBEDTLS_X509_CRT_PARSE_C)
        test_mbedtls_x509_time_is_future_wrapper,
    #else
        NULL,
    #endif
    /* Function Id: 10 */

    #if defined(MBEDTLS_BIGNUM_C) && defined(MBEDTLS_X509_CRT_PARSE_C) && defined(MBEDTLS_FS_IO)
        test_x509parse_crt_file_wrapper,
    #else
        NULL,
    #endif
    /* Function Id: 11 */

    #if defined(MBEDTLS_BIGNUM_C) && defined(MBEDTLS_X509_CRT_PARSE_C)
        test_x509parse_crt_wrapper,
    #else
        NULL,
    #endif
    /* Function Id: 12 */

    #if defined(MBEDTLS_BIGNUM_C) && defined(MBEDTLS_X509_CRL_PARSE_C)
        test_x509parse_crl_wrapper,
    #else
        NULL,
    #endif
    /* Function Id: 13 */

    #if defined(MBEDTLS_BIGNUM_C) && defined(MBEDTLS_X509_CSR_PARSE_C)
        test_mbedtls_x509_csr_parse_wrapper,
    #else
        NULL,
    #endif
    /* Function Id: 14 */

    #if defined(MBEDTLS_BIGNUM_C) && defined(MBEDTLS_FS_IO) && defined(MBEDTLS_X509_CRT_PARSE_C)
        test_mbedtls_x509_crt_parse_path_wrapper,
    #else
        NULL,
    #endif
    /* Function Id: 15 */

    #if defined(MBEDTLS_BIGNUM_C) && defined(MBEDTLS_FS_IO) && defined(MBEDTLS_X509_CRT_PARSE_C)
        test_mbedtls_x509_crt_verify_max_wrapper,
    #else
        NULL,
    #endif
    /* Function Id: 16 */

    #if defined(MBEDTLS_BIGNUM_C) && defined(MBEDTLS_FS_IO) && defined(MBEDTLS_X509_CRT_PARSE_C)
        test_mbedtls_x509_crt_verify_chain_wrapper,
    #else
        NULL,
    #endif
    /* Function Id: 17 */

    #if defined(MBEDTLS_BIGNUM_C) && defined(MBEDTLS_X509_USE_C)
        test_x509_oid_desc_wrapper,
    #else
        NULL,
    #endif
    /* Function Id: 18 */

    #if defined(MBEDTLS_BIGNUM_C) && defined(MBEDTLS_X509_USE_C)
        test_x509_oid_numstr_wrapper,
    #else
        NULL,
    #endif
    /* Function Id: 19 */

    #if defined(MBEDTLS_BIGNUM_C) && defined(MBEDTLS_FS_IO) && defined(MBEDTLS_X509_CRT_PARSE_C) && defined(MBEDTLS_X509_CHECK_KEY_USAGE)
        test_x509_check_key_usage_wrapper,
    #else
        NULL,
    #endif
    /* Function Id: 20 */

    #if defined(MBEDTLS_BIGNUM_C) && defined(MBEDTLS_FS_IO) && defined(MBEDTLS_X509_CRT_PARSE_C) && defined(MBEDTLS_X509_CHECK_EXTENDED_KEY_USAGE)
        test_x509_check_extended_key_usage_wrapper,
    #else
        NULL,
    #endif
    /* Function Id: 21 */

    #if defined(MBEDTLS_BIGNUM_C) && defined(MBEDTLS_X509_USE_C)
        test_x509_get_time_wrapper,
    #else
        NULL,
    #endif
    /* Function Id: 22 */

    #if defined(MBEDTLS_BIGNUM_C) && defined(MBEDTLS_X509_CRT_PARSE_C) && defined(MBEDTLS_X509_RSASSA_PSS_SUPPORT)
        test_x509_parse_rsassa_pss_params_wrapper,
    #else
        NULL,
    #endif
    /* Function Id: 23 */

    #if defined(MBEDTLS_BIGNUM_C) && defined(MBEDTLS_X509_CRT_PARSE_C) && defined(MBEDTLS_SELF_TEST)
        test_x509_selftest_wrapper,
    #else
        NULL,
    #endif
};

#if defined(MBEDTLS_X509_CRT_PARSE_C)
typedef struct {
    char buf[512];
    char *p;
} verify_print_context;

void verify_print_init( verify_print_context *ctx )
{
    memset( ctx, 0, sizeof( verify_print_context ) );
    ctx->p = ctx->buf;
}

int verify_print( void *data, mbedtls_x509_crt *crt, int certificate_depth, uint32_t *flags )
{
    int ret;
    verify_print_context *ctx = (verify_print_context *) data;
    char *p = ctx->p;
    size_t n = ctx->buf + sizeof( ctx->buf ) - ctx->p;
    ((void) flags);

    ret = mbedtls_snprintf( p, n, "depth %d - serial ", certificate_depth );
    MBEDTLS_X509_SAFE_SNPRINTF;

    ret = mbedtls_x509_serial_gets( p, n, &crt->serial );
    MBEDTLS_X509_SAFE_SNPRINTF;

    ret = mbedtls_snprintf( p, n, " - subject " );
    MBEDTLS_X509_SAFE_SNPRINTF;

    ret = mbedtls_x509_dn_gets( p, n, &crt->subject );
    MBEDTLS_X509_SAFE_SNPRINTF;

    ret = mbedtls_snprintf( p, n, " - flags 0x%08x\n", *flags );
    MBEDTLS_X509_SAFE_SNPRINTF;

    ctx->p = p;

    return 0;
}
#endif /* MBEDTLS_X509_CRT_PARSE_C */

#if defined(MBEDTLS_FS_IO)
#if defined(MBEDTLS_X509_CRT_PARSE_C)

void test_x509_cert_info( char * crt_file, char * result_str )
{
    mbedtls_x509_crt   crt;
    char buf[2000];
    int res;

    mbedtls_x509_crt_init( &crt );
    memset( buf, 0, 2000 );

    TEST_ASSERT_MBEDTLS( mbedtls_x509_crt_parse_file( &crt, crt_file ) == 0 );
    res = mbedtls_x509_crt_info( buf, 2000, "", &crt );

    TEST_ASSERT_MBEDTLS( res != -1 );
    TEST_ASSERT_MBEDTLS( res != -2 );

    TEST_ASSERT_MBEDTLS( strcmp( buf, result_str ) == 0 );

exit:
    mbedtls_x509_crt_free( &crt );
}

void test_x509_cert_info_wrapper( void ** params )
{

    test_x509_cert_info( (char *) params[0], (char *) params[1] );
}
#endif /* MBEDTLS_X509_CRT_PARSE_C */
#endif /* MBEDTLS_FS_IO */
#if defined(MBEDTLS_FS_IO)
#if defined(MBEDTLS_X509_CRL_PARSE_C)

void test_mbedtls_x509_crl_info( char * crl_file, char * result_str )
{
    mbedtls_x509_crl   crl;
    char buf[2000];
    int res;

    mbedtls_x509_crl_init( &crl );
    memset( buf, 0, 2000 );

    TEST_ASSERT_MBEDTLS( mbedtls_x509_crl_parse_file( &crl, crl_file ) == 0 );
    res = mbedtls_x509_crl_info( buf, 2000, "", &crl );

    TEST_ASSERT_MBEDTLS( res != -1 );
    TEST_ASSERT_MBEDTLS( res != -2 );

    TEST_ASSERT_MBEDTLS( strcmp( buf, result_str ) == 0 );

exit:
    mbedtls_x509_crl_free( &crl );
}

void test_mbedtls_x509_crl_info_wrapper( void ** params )
{

    test_mbedtls_x509_crl_info( (char *) params[0], (char *) params[1] );
}
#endif /* MBEDTLS_X509_CRL_PARSE_C */
#endif /* MBEDTLS_FS_IO */
#if defined(MBEDTLS_FS_IO)
#if defined(MBEDTLS_X509_CRL_PARSE_C)

void test_mbedtls_x509_crl_parse( char * crl_file, int result )
{
    mbedtls_x509_crl   crl;
    char buf[2000];

    mbedtls_x509_crl_init( &crl );
    memset( buf, 0, 2000 );

    TEST_ASSERT_MBEDTLS( mbedtls_x509_crl_parse_file( &crl, crl_file ) == result );

exit:
    mbedtls_x509_crl_free( &crl );
}

void test_mbedtls_x509_crl_parse_wrapper( void ** params )
{

    test_mbedtls_x509_crl_parse( (char *) params[0], *( (int *) params[1] ) );
}
#endif /* MBEDTLS_X509_CRL_PARSE_C */
#endif /* MBEDTLS_FS_IO */
#if defined(MBEDTLS_FS_IO)
#if defined(MBEDTLS_X509_CSR_PARSE_C)
void test_mbedtls_x509_csr_info( char * csr_file, char * result_str )
{
    mbedtls_x509_csr   csr;
    char buf[2000];
    int res;

    mbedtls_x509_csr_init( &csr );
    memset( buf, 0, 2000 );

    TEST_ASSERT_MBEDTLS( mbedtls_x509_csr_parse_file( &csr, csr_file ) == 0 );
    res = mbedtls_x509_csr_info( buf, 2000, "", &csr );

    TEST_ASSERT_MBEDTLS( res != -1 );
    TEST_ASSERT_MBEDTLS( res != -2 );

    TEST_ASSERT_MBEDTLS( strcmp( buf, result_str ) == 0 );

exit:
    mbedtls_x509_csr_free( &csr );
}

void test_mbedtls_x509_csr_info_wrapper( void ** params )
{

    test_mbedtls_x509_csr_info( (char *) params[0], (char *) params[1] );
}
#endif /* MBEDTLS_X509_CSR_PARSE_C */
#endif /* MBEDTLS_FS_IO */
#if defined(MBEDTLS_X509_CRT_PARSE_C)
void test_x509_verify_info( int flags, char * prefix, char * result_str )
{
    char buf[2000];
    int res;

    memset( buf, 0, sizeof( buf ) );

    res = mbedtls_x509_crt_verify_info( buf, sizeof( buf ), prefix, flags );

    TEST_ASSERT_MBEDTLS( res >= 0 );

    TEST_ASSERT_MBEDTLS( strcmp( buf, result_str ) == 0 );
exit:
    ;
}

void test_x509_verify_info_wrapper( void ** params )
{

    test_x509_verify_info( *( (int *) params[0] ), (char *) params[1], (char *) params[2] );
}
#endif /* MBEDTLS_X509_CRT_PARSE_C */
#if defined(MBEDTLS_FS_IO)
#if defined(MBEDTLS_X509_CRT_PARSE_C)
#if defined(MBEDTLS_X509_CRL_PARSE_C)
void test_x509_verify( char *crt_file, char *ca_file, char *crl_file,
                  char *cn_name_str, int result, int flags_result,
                  char *profile_str,
                  char *verify_callback )
{
    mbedtls_x509_crt   crt;
    mbedtls_x509_crt   ca;
    mbedtls_x509_crl    crl;
    uint32_t         flags = 0;
    int         res;
    int (*f_vrfy)(void *, mbedtls_x509_crt *, int, uint32_t *) = NULL;
    char *      cn_name = NULL;
    const mbedtls_x509_crt_profile *profile;

    mbedtls_x509_crt_init( &crt );
    mbedtls_x509_crt_init( &ca );
    mbedtls_x509_crl_init( &crl );

    if( strcmp( cn_name_str, "NULL" ) != 0 )
        cn_name = cn_name_str;

    if( strcmp( profile_str, "" ) == 0 )
        profile = &mbedtls_x509_crt_profile_default;
    else if( strcmp( profile_str, "next" ) == 0 )
        profile = &mbedtls_x509_crt_profile_next;
    else if( strcmp( profile_str, "suite_b" ) == 0 )
        profile = &mbedtls_x509_crt_profile_suiteb;
    else if( strcmp( profile_str, "compat" ) == 0 )
        profile = &compat_profile;
    else
        TEST_ASSERT_MBEDTLS( "Unknown algorithm profile" == 0 );

    if( strcmp( verify_callback, "NULL" ) == 0 )
        f_vrfy = NULL;
    else if( strcmp( verify_callback, "verify_none" ) == 0 )
        f_vrfy = verify_none;
    else if( strcmp( verify_callback, "verify_all" ) == 0 )
        f_vrfy = verify_all;
    else
        TEST_ASSERT_MBEDTLS( "No known verify callback selected" == 0 );

    TEST_ASSERT_MBEDTLS( mbedtls_x509_crt_parse_file( &crt, crt_file ) == 0 );
    TEST_ASSERT_MBEDTLS( mbedtls_x509_crt_parse_file( &ca, ca_file ) == 0 );
    TEST_ASSERT_MBEDTLS( mbedtls_x509_crl_parse_file( &crl, crl_file ) == 0 );

    res = mbedtls_x509_crt_verify_with_profile( &crt, &ca, &crl, profile, cn_name, &flags, f_vrfy, NULL );

    TEST_ASSERT_MBEDTLS( res == ( result ) );
    TEST_ASSERT_MBEDTLS( flags == (uint32_t)( flags_result ) );

exit:
    mbedtls_x509_crt_free( &crt );
    mbedtls_x509_crt_free( &ca );
    mbedtls_x509_crl_free( &crl );
}

void test_x509_verify_wrapper( void ** params )
{

    test_x509_verify( (char *) params[0], (char *) params[1], (char *) params[2], (char *) params[3], *( (int *) params[4] ), *( (int *) params[5] ), (char *) params[6], (char *) params[7] );
}
#endif /* MBEDTLS_X509_CRL_PARSE_C */
#endif /* MBEDTLS_X509_CRT_PARSE_C */
#endif /* MBEDTLS_FS_IO */

#if defined(MBEDTLS_FS_IO)
#if defined(MBEDTLS_X509_CRT_PARSE_C)
void test_x509_verify_callback( char *crt_file, char *ca_file, char *name,
                           int exp_ret, char *exp_vrfy_out )
{
    int ret;
    mbedtls_x509_crt crt;
    mbedtls_x509_crt ca;
    uint32_t flags = 0;
    verify_print_context vrfy_ctx;

    mbedtls_x509_crt_init( &crt );
    mbedtls_x509_crt_init( &ca );
    verify_print_init( &vrfy_ctx );

    TEST_ASSERT_MBEDTLS( mbedtls_x509_crt_parse_file( &crt, crt_file ) == 0 );
    TEST_ASSERT_MBEDTLS( mbedtls_x509_crt_parse_file( &ca, ca_file ) == 0 );

    if( strcmp( name, "NULL" ) == 0 )
        name = NULL;

    ret = mbedtls_x509_crt_verify_with_profile( &crt, &ca, NULL,
                                                &compat_profile,
                                                name, &flags,
                                                verify_print, &vrfy_ctx );

    TEST_ASSERT_MBEDTLS( ret == exp_ret );
    TEST_ASSERT_MBEDTLS( strcmp( vrfy_ctx.buf, exp_vrfy_out ) == 0 );

exit:
    mbedtls_x509_crt_free( &crt );
    mbedtls_x509_crt_free( &ca );
}

void test_x509_verify_callback_wrapper( void ** params )
{

    test_x509_verify_callback( (char *) params[0], (char *) params[1], (char *) params[2], *( (int *) params[3] ), (char *) params[4] );
}
#endif /* MBEDTLS_X509_CRT_PARSE_C */
#endif /* MBEDTLS_FS_IO */

#if defined(MBEDTLS_FS_IO)
#if defined(MBEDTLS_X509_CRT_PARSE_C)
void test_mbedtls_x509_dn_gets( char * crt_file, char * entity, char * result_str )
{
    mbedtls_x509_crt   crt;
    char buf[2000];
    int res = 0;

    mbedtls_x509_crt_init( &crt );
    memset( buf, 0, 2000 );

    TEST_ASSERT_MBEDTLS( mbedtls_x509_crt_parse_file( &crt, crt_file ) == 0 );
    if( strcmp( entity, "subject" ) == 0 )
        res =  mbedtls_x509_dn_gets( buf, 2000, &crt.subject );
    else if( strcmp( entity, "issuer" ) == 0 )
        res =  mbedtls_x509_dn_gets( buf, 2000, &crt.issuer );
    else
        TEST_ASSERT_MBEDTLS( "Unknown entity" == 0 );

    TEST_ASSERT_MBEDTLS( res != -1 );
    TEST_ASSERT_MBEDTLS( res != -2 );

    TEST_ASSERT_MBEDTLS( strcmp( buf, result_str ) == 0 );

exit:
    mbedtls_x509_crt_free( &crt );
}

void test_mbedtls_x509_dn_gets_wrapper( void ** params )
{

    test_mbedtls_x509_dn_gets( (char *) params[0], (char *) params[1], (char *) params[2] );
}
#endif /* MBEDTLS_X509_CRT_PARSE_C */
#endif /* MBEDTLS_FS_IO */

#if defined(MBEDTLS_FS_IO)
#if defined(MBEDTLS_X509_CRT_PARSE_C)
void test_mbedtls_x509_time_is_past( char * crt_file, char * entity, int result )
{
    mbedtls_x509_crt   crt;

    mbedtls_x509_crt_init( &crt );

    TEST_ASSERT_MBEDTLS( mbedtls_x509_crt_parse_file( &crt, crt_file ) == 0 );

    if( strcmp( entity, "valid_from" ) == 0 )
        TEST_ASSERT_MBEDTLS( mbedtls_x509_time_is_past( &crt.valid_from ) == result );
    else if( strcmp( entity, "valid_to" ) == 0 )
        TEST_ASSERT_MBEDTLS( mbedtls_x509_time_is_past( &crt.valid_to ) == result );
    else
        TEST_ASSERT_MBEDTLS( "Unknown entity" == 0 );

exit:
    mbedtls_x509_crt_free( &crt );
}

void test_mbedtls_x509_time_is_past_wrapper( void ** params )
{

    test_mbedtls_x509_time_is_past( (char *) params[0], (char *) params[1], *( (int *) params[2] ) );
}
#endif /* MBEDTLS_X509_CRT_PARSE_C */
#endif /* MBEDTLS_FS_IO */

#if defined(MBEDTLS_FS_IO)
#if defined(MBEDTLS_X509_CRT_PARSE_C)
void test_mbedtls_x509_time_is_future( char * crt_file, char * entity, int result )
{
    mbedtls_x509_crt   crt;

    mbedtls_x509_crt_init( &crt );

    TEST_ASSERT_MBEDTLS( mbedtls_x509_crt_parse_file( &crt, crt_file ) == 0 );

    if( strcmp( entity, "valid_from" ) == 0 )
        TEST_ASSERT_MBEDTLS( mbedtls_x509_time_is_future( &crt.valid_from ) == result );
    else if( strcmp( entity, "valid_to" ) == 0 )
        TEST_ASSERT_MBEDTLS( mbedtls_x509_time_is_future( &crt.valid_to ) == result );
    else
        TEST_ASSERT_MBEDTLS( "Unknown entity" == 0 );

exit:
    mbedtls_x509_crt_free( &crt );
}

void test_mbedtls_x509_time_is_future_wrapper( void ** params )
{

    test_mbedtls_x509_time_is_future( (char *) params[0], (char *) params[1], *( (int *) params[2] ) );
}
#endif /* MBEDTLS_X509_CRT_PARSE_C */
#endif /* MBEDTLS_FS_IO */

#if defined(MBEDTLS_X509_CRT_PARSE_C)
#if defined(MBEDTLS_FS_IO)
void test_x509parse_crt_file( char * crt_file, int result )
{
    mbedtls_x509_crt crt;

    mbedtls_x509_crt_init( &crt );

    TEST_ASSERT_MBEDTLS( mbedtls_x509_crt_parse_file( &crt, crt_file ) == result );

exit:
    mbedtls_x509_crt_free( &crt );
}

void test_x509parse_crt_file_wrapper( void ** params )
{

    test_x509parse_crt_file( (char *) params[0], *( (int *) params[1] ) );
}
#endif /* MBEDTLS_FS_IO */
#endif /* MBEDTLS_X509_CRT_PARSE_C */

#if defined(MBEDTLS_X509_CRT_PARSE_C)
void test_x509parse_crt( data_t * buf, char * result_str, int result )
{
    mbedtls_x509_crt   crt;
    unsigned char output[2000];
    int res;

    mbedtls_x509_crt_init( &crt );
    memset( output, 0, 2000 );

    TEST_ASSERT_MBEDTLS( mbedtls_x509_crt_parse( &crt, buf->x, buf->len ) == ( result ) );
    if( ( result ) == 0 )
    {
        res = mbedtls_x509_crt_info( (char *) output, 2000, "", &crt );

        TEST_ASSERT_MBEDTLS( res != -1 );
        TEST_ASSERT_MBEDTLS( res != -2 );

        TEST_ASSERT_MBEDTLS( strcmp( (char *) output, result_str ) == 0 );
    }

exit:
    mbedtls_x509_crt_free( &crt );
}

void test_x509parse_crt_wrapper( void ** params )
{
    data_t data0 = {(uint8_t *) params[0], *( (uint32_t *) params[1] )};

    test_x509parse_crt( &data0, (char *) params[2], *( (int *) params[3] ) );
}
#endif /* MBEDTLS_X509_CRT_PARSE_C */

#if defined(MBEDTLS_X509_CRL_PARSE_C)
void test_x509parse_crl( data_t * buf, char * result_str, int result )
{
    mbedtls_x509_crl   crl;
    unsigned char output[2000];
    int res;

    mbedtls_x509_crl_init( &crl );
    memset( output, 0, 2000 );

    TEST_ASSERT_MBEDTLS( mbedtls_x509_crl_parse( &crl, buf->x, buf->len ) == ( result ) );
    if( ( result ) == 0 )
    {
        res = mbedtls_x509_crl_info( (char *) output, 2000, "", &crl );

        TEST_ASSERT_MBEDTLS( res != -1 );
        TEST_ASSERT_MBEDTLS( res != -2 );

        TEST_ASSERT_MBEDTLS( strcmp( (char *) output, result_str ) == 0 );
    }

exit:
    mbedtls_x509_crl_free( &crl );
}

void test_x509parse_crl_wrapper( void ** params )
{
    data_t data0 = {(uint8_t *) params[0], *( (uint32_t *) params[1] )};

    test_x509parse_crl( &data0, (char *) params[2], *( (int *) params[3] ) );
}
#endif /* MBEDTLS_X509_CRL_PARSE_C */

#if defined(MBEDTLS_X509_CSR_PARSE_C)
void test_mbedtls_x509_csr_parse( data_t * csr_der, char * ref_out, int ref_ret )
{
    mbedtls_x509_csr csr;
    char my_out[1000];
    int my_ret;

    mbedtls_x509_csr_init( &csr );
    memset( my_out, 0, sizeof( my_out ) );

    my_ret = mbedtls_x509_csr_parse_der( &csr, csr_der->x, csr_der->len );
    TEST_ASSERT_MBEDTLS( my_ret == ref_ret );

    if( ref_ret == 0 )
    {
        size_t my_out_len = mbedtls_x509_csr_info( my_out, sizeof( my_out ), "", &csr );
        TEST_ASSERT_MBEDTLS( my_out_len == strlen( ref_out ) );
        TEST_ASSERT_MBEDTLS( strcmp( my_out, ref_out ) == 0 );
    }

exit:
    mbedtls_x509_csr_free( &csr );
}

void test_mbedtls_x509_csr_parse_wrapper( void ** params )
{
    data_t data0 = {(uint8_t *) params[0], *( (uint32_t *) params[1] )};

    test_mbedtls_x509_csr_parse( &data0, (char *) params[2], *( (int *) params[3] ) );
}
#endif /* MBEDTLS_X509_CSR_PARSE_C */

#if defined(MBEDTLS_FS_IO)
#if defined(MBEDTLS_X509_CRT_PARSE_C)
void test_mbedtls_x509_crt_parse_path( char * crt_path, int ret, int nb_crt )
{
    mbedtls_x509_crt chain, *cur;
    int i;

    mbedtls_x509_crt_init( &chain );

    TEST_ASSERT_MBEDTLS( mbedtls_x509_crt_parse_path( &chain, crt_path ) == ret );

    /* Check how many certs we got */
    for( i = 0, cur = &chain; cur != NULL; cur = cur->next )
        if( cur->raw.p != NULL )
            i++;

    TEST_ASSERT_MBEDTLS( i == nb_crt );

exit:
    mbedtls_x509_crt_free( &chain );
}

void test_mbedtls_x509_crt_parse_path_wrapper( void ** params )
{

    test_mbedtls_x509_crt_parse_path( (char *) params[0], *( (int *) params[1] ), *( (int *) params[2] ) );
}
#endif /* MBEDTLS_X509_CRT_PARSE_C */
#endif /* MBEDTLS_FS_IO */

#if defined(MBEDTLS_FS_IO)
#if defined(MBEDTLS_X509_CRT_PARSE_C)

void test_mbedtls_x509_crt_verify_max( char *ca_file, char *chain_dir, int nb_int,
                                  int ret_chk, int flags_chk )
{
    char file_buf[128];
    int ret;
    uint32_t flags;
    mbedtls_x509_crt trusted, chain;

    /*
     * We expect chain_dir to contain certificates 00.crt, 01.crt, etc.
     * with NN.crt signed by NN-1.crt
     */

    mbedtls_x509_crt_init( &trusted );
    mbedtls_x509_crt_init( &chain );

    /* Load trusted root */
    TEST_ASSERT_MBEDTLS( mbedtls_x509_crt_parse_file( &trusted, ca_file ) == 0 );

    /* Load a chain with nb_int intermediates (from 01 to nb_int),
     * plus one "end-entity" cert (nb_int + 1) */
    ret = mbedtls_snprintf( file_buf, sizeof file_buf, "%s/c%02d.pem", chain_dir,
                                                            nb_int + 1 );
    TEST_ASSERT_MBEDTLS( ret > 0 && (size_t) ret < sizeof file_buf );
    TEST_ASSERT_MBEDTLS( mbedtls_x509_crt_parse_file( &chain, file_buf ) == 0 );

    /* Try to verify that chain */
    ret = mbedtls_x509_crt_verify( &chain, &trusted, NULL, NULL, &flags,
                                   NULL, NULL );
    TEST_ASSERT_MBEDTLS( ret == ret_chk );
    TEST_ASSERT_MBEDTLS( flags == (uint32_t) flags_chk );

exit:
    mbedtls_x509_crt_free( &chain );
    mbedtls_x509_crt_free( &trusted );
}

void test_mbedtls_x509_crt_verify_max_wrapper( void ** params )
{

    test_mbedtls_x509_crt_verify_max( (char *) params[0], (char *) params[1], *( (int *) params[2] ), *( (int *) params[3] ), *( (int *) params[4] ) );
}
#endif /* MBEDTLS_X509_CRT_PARSE_C */
#endif /* MBEDTLS_FS_IO */

#if defined(MBEDTLS_FS_IO)
#if defined(MBEDTLS_X509_CRT_PARSE_C)

void test_mbedtls_x509_crt_verify_chain(  char *chain_paths, char *trusted_ca,
                                     int flags_result, int result,
                                     char *profile_name, int vrfy_fatal_lvls )
{
    char* act;
    uint32_t flags;
    int res;
    mbedtls_x509_crt trusted, chain;
    const mbedtls_x509_crt_profile *profile = NULL;

    mbedtls_x509_crt_init( &chain );
    mbedtls_x509_crt_init( &trusted );

    while( ( act = mystrsep( &chain_paths, " " ) ) != NULL )
        TEST_ASSERT_MBEDTLS( mbedtls_x509_crt_parse_file( &chain, act ) == 0 );
    TEST_ASSERT_MBEDTLS( mbedtls_x509_crt_parse_file( &trusted, trusted_ca ) == 0 );

    if( strcmp( profile_name, "" ) == 0 )
        profile = &mbedtls_x509_crt_profile_default;
    else if( strcmp( profile_name, "next" ) == 0 )
        profile = &mbedtls_x509_crt_profile_next;
    else if( strcmp( profile_name, "suiteb" ) == 0 )
        profile = &mbedtls_x509_crt_profile_suiteb;
    else if( strcmp( profile_name, "rsa3072" ) == 0 )
        profile = &profile_rsa3072;
    else if( strcmp( profile_name, "sha512" ) == 0 )
        profile = &profile_sha512;

    res = mbedtls_x509_crt_verify_with_profile( &chain, &trusted, NULL, profile,
            NULL, &flags, verify_fatal, &vrfy_fatal_lvls );

    TEST_ASSERT_MBEDTLS( res == ( result ) );
    TEST_ASSERT_MBEDTLS( flags == (uint32_t)( flags_result ) );

exit:
    mbedtls_x509_crt_free( &trusted );
    mbedtls_x509_crt_free( &chain );
}

void test_mbedtls_x509_crt_verify_chain_wrapper( void ** params )
{

    test_mbedtls_x509_crt_verify_chain( (char *) params[0], (char *) params[1], *( (int *) params[2] ), *( (int *) params[3] ), (char *) params[4], *( (int *) params[5] ) );
}
#endif /* MBEDTLS_X509_CRT_PARSE_C */
#endif /* MBEDTLS_FS_IO */

#if defined(MBEDTLS_X509_USE_C)

void test_x509_oid_desc( data_t * buf, char * ref_desc )
{
    mbedtls_x509_buf oid;
    const char *desc = NULL;
    int ret;

    oid.tag = MBEDTLS_ASN1_OID;
    oid.p   = buf->x;
    oid.len   = buf->len;

    ret = mbedtls_oid_get_extended_key_usage( &oid, &desc );

    if( strcmp( ref_desc, "notfound" ) == 0 )
    {
        TEST_ASSERT_MBEDTLS( ret != 0 );
        TEST_ASSERT_MBEDTLS( desc == NULL );
    }
    else
    {
        TEST_ASSERT_MBEDTLS( ret == 0 );
        TEST_ASSERT_MBEDTLS( desc != NULL );
        TEST_ASSERT_MBEDTLS( strcmp( desc, ref_desc ) == 0 );
    }
exit:
    ;
}

void test_x509_oid_desc_wrapper( void ** params )
{
    data_t data0 = {(uint8_t *) params[0], *( (uint32_t *) params[1] )};

    test_x509_oid_desc( &data0, (char *) params[2] );
}
#endif /* MBEDTLS_X509_USE_C */

#if defined(MBEDTLS_X509_USE_C)

void test_x509_oid_numstr( data_t * oid_buf, char * numstr, int blen, int ret )
{
    mbedtls_x509_buf oid;
    char num_buf[100];

    memset( num_buf, 0x2a, sizeof num_buf );

    oid.tag = MBEDTLS_ASN1_OID;
    oid.p   = oid_buf->x;
    oid.len   = oid_buf->len;

    TEST_ASSERT_MBEDTLS( (size_t) blen <= sizeof num_buf );

    TEST_ASSERT_MBEDTLS( mbedtls_oid_get_numeric_string( num_buf, blen, &oid ) == ret );

    if( ret >= 0 )
    {
        TEST_ASSERT_MBEDTLS( num_buf[ret] == 0 );
        TEST_ASSERT_MBEDTLS( strcmp( num_buf, numstr ) == 0 );
    }
exit:
    ;
}

void test_x509_oid_numstr_wrapper( void ** params )
{
    data_t data0 = {(uint8_t *) params[0], *( (uint32_t *) params[1] )};

    test_x509_oid_numstr( &data0, (char *) params[2], *( (int *) params[3] ), *( (int *) params[4] ) );
}
#endif /* MBEDTLS_X509_USE_C */

#if defined(MBEDTLS_FS_IO)
#if defined(MBEDTLS_X509_CRT_PARSE_C)
#if defined(MBEDTLS_X509_CHECK_KEY_USAGE)

void test_x509_check_key_usage( char * crt_file, int usage, int ret )
{
    mbedtls_x509_crt crt;

    mbedtls_x509_crt_init( &crt );

    TEST_ASSERT_MBEDTLS( mbedtls_x509_crt_parse_file( &crt, crt_file ) == 0 );

    TEST_ASSERT_MBEDTLS( mbedtls_x509_crt_check_key_usage( &crt, usage ) == ret );

exit:
    mbedtls_x509_crt_free( &crt );
}

void test_x509_check_key_usage_wrapper( void ** params )
{

    test_x509_check_key_usage( (char *) params[0], *( (int *) params[1] ), *( (int *) params[2] ) );
}
#endif /* MBEDTLS_X509_CHECK_KEY_USAGE */
#endif /* MBEDTLS_X509_CRT_PARSE_C */
#endif /* MBEDTLS_FS_IO */

#if defined(MBEDTLS_FS_IO)
#if defined(MBEDTLS_X509_CRT_PARSE_C)
#if defined(MBEDTLS_X509_CHECK_EXTENDED_KEY_USAGE)

void test_x509_check_extended_key_usage( char * crt_file, data_t * oid, int ret
                                    )
{
    mbedtls_x509_crt crt;

    mbedtls_x509_crt_init( &crt );

    TEST_ASSERT_MBEDTLS( mbedtls_x509_crt_parse_file( &crt, crt_file ) == 0 );

    TEST_ASSERT_MBEDTLS( mbedtls_x509_crt_check_extended_key_usage( &crt, (const char *)oid->x, oid->len ) == ret );

exit:
    mbedtls_x509_crt_free( &crt );
}

void test_x509_check_extended_key_usage_wrapper( void ** params )
{
    data_t data1 = {(uint8_t *) params[1], *( (uint32_t *) params[2] )};

    test_x509_check_extended_key_usage( (char *) params[0], &data1, *( (int *) params[3] ) );
}
#endif /* MBEDTLS_X509_CHECK_EXTENDED_KEY_USAGE */
#endif /* MBEDTLS_X509_CRT_PARSE_C */
#endif /* MBEDTLS_FS_IO */

#if defined(MBEDTLS_X509_USE_C)

void test_x509_get_time( int tag, char * time_str, int ret, int year, int mon,
                    int day, int hour, int min, int sec )
{
    mbedtls_x509_time time;
    unsigned char buf[21];
    unsigned char* start = buf;
    unsigned char* end = buf;

    memset( &time, 0x00, sizeof( time ) );
    *end = (unsigned char)tag; end++;
    *end = strlen( time_str );
    TEST_ASSERT_MBEDTLS( *end < 20 );
    end++;
    memcpy( end, time_str, (size_t)*(end - 1) );
    end += *(end - 1);

    TEST_ASSERT_MBEDTLS( mbedtls_x509_get_time( &start, end, &time ) == ret );
    if( ret == 0 )
    {
        TEST_ASSERT_MBEDTLS( year == time.year );
        TEST_ASSERT_MBEDTLS( mon  == time.mon  );
        TEST_ASSERT_MBEDTLS( day  == time.day  );
        TEST_ASSERT_MBEDTLS( hour == time.hour );
        TEST_ASSERT_MBEDTLS( min  == time.min  );
        TEST_ASSERT_MBEDTLS( sec  == time.sec  );
    }
exit:
    ;
}

void test_x509_get_time_wrapper( void ** params )
{

    test_x509_get_time( *( (int *) params[0] ), (char *) params[1], *( (int *) params[2] ), *( (int *) params[3] ), *( (int *) params[4] ), *( (int *) params[5] ), *( (int *) params[6] ), *( (int *) params[7] ), *( (int *) params[8] ) );
}
#endif /* MBEDTLS_X509_USE_C */

#if defined(MBEDTLS_X509_CRT_PARSE_C)
#if defined(MBEDTLS_X509_RSASSA_PSS_SUPPORT)

void test_x509_parse_rsassa_pss_params( data_t * hex_params, int params_tag,
                                   int ref_msg_md, int ref_mgf_md,
                                   int ref_salt_len, int ref_ret )
{
    int my_ret;
    mbedtls_x509_buf params;
    mbedtls_md_type_t my_msg_md, my_mgf_md;
    int my_salt_len;

    params.p = hex_params->x;
    params.len = hex_params->len;
    params.tag = params_tag;

    my_ret = mbedtls_x509_get_rsassa_pss_params( &params, &my_msg_md, &my_mgf_md,
                                         &my_salt_len );

    TEST_ASSERT_MBEDTLS( my_ret == ref_ret );

    if( ref_ret == 0 )
    {
        TEST_ASSERT_MBEDTLS( my_msg_md == (mbedtls_md_type_t) ref_msg_md );
        TEST_ASSERT_MBEDTLS( my_mgf_md == (mbedtls_md_type_t) ref_mgf_md );
        TEST_ASSERT_MBEDTLS( my_salt_len == ref_salt_len );
    }

exit:
    ;;
}

void test_x509_parse_rsassa_pss_params_wrapper( void ** params )
{
    data_t data0 = {(uint8_t *) params[0], *( (uint32_t *) params[1] )};

    test_x509_parse_rsassa_pss_params( &data0, *( (int *) params[2] ), *( (int *) params[3] ), *( (int *) params[4] ), *( (int *) params[5] ), *( (int *) params[6] ) );
}
#endif /* MBEDTLS_X509_RSASSA_PSS_SUPPORT */
#endif /* MBEDTLS_X509_CRT_PARSE_C */

#if defined(MBEDTLS_X509_CRT_PARSE_C)
#if defined(MBEDTLS_SELF_TEST)

void test_x509_selftest(  )
{
    TEST_ASSERT_MBEDTLS( mbedtls_x509_self_test( 1 ) == 0 );
exit:
    ;
}

void test_x509_selftest_wrapper( void ** params )
{
    (void)params;

    test_x509_selftest(  );
}
#endif /* MBEDTLS_SELF_TEST */
#endif /* MBEDTLS_X509_CRT_PARSE_C */

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

        #if defined(MBEDTLS_BIGNUM_C)

        case 0:
            {
            #if defined(MBEDTLS_PEM_PARSE_C)
                ret = DEPENDENCY_SUPPORTED;
            #else
                ret = DEPENDENCY_NOT_SUPPORTED;
            #endif
            }
            break;
        case 1:
            {
            #if defined(MBEDTLS_RSA_C)
                ret = DEPENDENCY_SUPPORTED;
            #else
                ret = DEPENDENCY_NOT_SUPPORTED;
            #endif
            }
            break;
        case 2:
            {
            #if defined(MBEDTLS_SHA1_C)
                ret = DEPENDENCY_SUPPORTED;
            #else
                ret = DEPENDENCY_NOT_SUPPORTED;
            #endif
            }
            break;
        case 3:
            {
            #if defined(MBEDTLS_MD2_C)
                ret = DEPENDENCY_SUPPORTED;
            #else
                ret = DEPENDENCY_NOT_SUPPORTED;
            #endif
            }
            break;
        case 4:
            {
            #if defined(MBEDTLS_MD4_C)
                ret = DEPENDENCY_SUPPORTED;
            #else
                ret = DEPENDENCY_NOT_SUPPORTED;
            #endif
            }
            break;
        case 5:
            {
            #if defined(MBEDTLS_MD5_C)
                ret = DEPENDENCY_SUPPORTED;
            #else
                ret = DEPENDENCY_NOT_SUPPORTED;
            #endif
            }
            break;
        case 6:
            {
            #if defined(MBEDTLS_SHA256_C)
                ret = DEPENDENCY_SUPPORTED;
            #else
                ret = DEPENDENCY_NOT_SUPPORTED;
            #endif
            }
            break;
        case 7:
            {
            #if defined(MBEDTLS_SHA512_C)
                ret = DEPENDENCY_SUPPORTED;
            #else
                ret = DEPENDENCY_NOT_SUPPORTED;
            #endif
            }
            break;
        case 8:
            {
            #if defined(MBEDTLS_X509_RSASSA_PSS_SUPPORT)
                ret = DEPENDENCY_SUPPORTED;
            #else
                ret = DEPENDENCY_NOT_SUPPORTED;
            #endif
            }
            break;
        case 9:
            {
            #if defined(MBEDTLS_ECDSA_C)
                ret = DEPENDENCY_SUPPORTED;
            #else
                ret = DEPENDENCY_NOT_SUPPORTED;
            #endif
            }
            break;
        case 10:
            {
            #if defined(MBEDTLS_ECP_DP_SECP256R1_ENABLED)
                ret = DEPENDENCY_SUPPORTED;
            #else
                ret = DEPENDENCY_NOT_SUPPORTED;
            #endif
            }
            break;
        case 11:
            {
            #if defined(MBEDTLS_ECP_DP_SECP192R1_ENABLED)
                ret = DEPENDENCY_SUPPORTED;
            #else
                ret = DEPENDENCY_NOT_SUPPORTED;
            #endif
            }
            break;
        case 12:
            {
            #if defined(MBEDTLS_X509_ALLOW_EXTENSIONS_NON_V3)
                ret = DEPENDENCY_SUPPORTED;
            #else
                ret = DEPENDENCY_NOT_SUPPORTED;
            #endif
            }
            break;
        case 13:
            {
            #if defined(MBEDTLS_HAVE_TIME_DATE)
                ret = DEPENDENCY_SUPPORTED;
            #else
                ret = DEPENDENCY_NOT_SUPPORTED;
            #endif
            }
            break;
        case 14:
            {
            #if defined(MBEDTLS_ECP_DP_SECP384R1_ENABLED)
                ret = DEPENDENCY_SUPPORTED;
            #else
                ret = DEPENDENCY_NOT_SUPPORTED;
            #endif
            }
            break;
        case 15:
            {
            #if defined(MBEDTLS_PKCS1_V15)
                ret = DEPENDENCY_SUPPORTED;
            #else
                ret = DEPENDENCY_NOT_SUPPORTED;
            #endif
            }
            break;
        case 16:
            {
            #if defined(MBEDTLS_TLS_DEFAULT_ALLOW_SHA1_IN_CERTIFICATES)
                ret = DEPENDENCY_SUPPORTED;
            #else
                ret = DEPENDENCY_NOT_SUPPORTED;
            #endif
            }
            break;
        case 17:
            {
            #if !defined(MBEDTLS_TLS_DEFAULT_ALLOW_SHA1_IN_CERTIFICATES)
                ret = DEPENDENCY_SUPPORTED;
            #else
                ret = DEPENDENCY_NOT_SUPPORTED;
            #endif
            }
            break;
        case 18:
            {
            #if defined(MBEDTLS_X509_CHECK_KEY_USAGE)
                ret = DEPENDENCY_SUPPORTED;
            #else
                ret = DEPENDENCY_NOT_SUPPORTED;
            #endif
            }
            break;
        case 19:
            {
            #if defined(MBEDTLS_ECP_C)
                ret = DEPENDENCY_SUPPORTED;
            #else
                ret = DEPENDENCY_NOT_SUPPORTED;
            #endif
            }
            break;
        case 20:
            {
            #if defined(MBEDTLS_CERTS_C)
                ret = DEPENDENCY_SUPPORTED;
            #else
                ret = DEPENDENCY_NOT_SUPPORTED;
            #endif
            }
            break;
        case 21:
            {
            #if defined(MBEDTLS_X509_USE_C)
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

static int get_expression( int32_t exp_id, int32_t * out_value )
{
    int ret = KEY_VALUE_MAPPING_FOUND;

    (void) exp_id;
    (void) out_value;

    switch( exp_id )
    {

#if defined(MBEDTLS_BIGNUM_C)

    case 0:
        {
            *out_value = MBEDTLS_ERR_PEM_NO_HEADER_FOOTER_PRESENT;
        }
        break;
    case 1:
        {
            *out_value = MBEDTLS_ERR_X509_INVALID_EXTENSIONS + MBEDTLS_ERR_ASN1_UNEXPECTED_TAG;
        }
        break;
    case 2:
        {
            *out_value = MBEDTLS_X509_BADCERT_MISSING;
        }
        break;
    case 3:
        {
            *out_value = MBEDTLS_X509_BADCERT_EXPIRED | MBEDTLS_X509_BADCRL_EXPIRED;
        }
        break;
    case 4:
        {
            *out_value = MBEDTLS_X509_BADCERT_OTHER | 0x80000000;
        }
        break;
    case 5:
        {
            *out_value = MBEDTLS_ERR_X509_CERT_VERIFY_FAILED;
        }
        break;
    case 6:
        {
            *out_value = MBEDTLS_X509_BADCERT_REVOKED | MBEDTLS_X509_BADCRL_EXPIRED;
        }
        break;
    case 7:
        {
            *out_value = MBEDTLS_X509_BADCERT_REVOKED | MBEDTLS_X509_BADCRL_FUTURE;
        }
        break;
    case 8:
        {
            *out_value = MBEDTLS_X509_BADCERT_REVOKED | MBEDTLS_X509_BADCRL_EXPIRED | MBEDTLS_X509_BADCERT_CN_MISMATCH;
        }
        break;
    case 9:
        {
            *out_value = MBEDTLS_X509_BADCERT_REVOKED | MBEDTLS_X509_BADCRL_FUTURE | MBEDTLS_X509_BADCERT_CN_MISMATCH;
        }
        break;
    case 10:
        {
            *out_value = MBEDTLS_X509_BADCRL_EXPIRED;
        }
        break;
    case 11:
        {
            *out_value = MBEDTLS_X509_BADCRL_FUTURE;
        }
        break;
    case 12:
        {
            *out_value = MBEDTLS_X509_BADCERT_REVOKED;
        }
        break;
    case 13:
        {
            *out_value = MBEDTLS_X509_BADCERT_REVOKED | MBEDTLS_X509_BADCERT_CN_MISMATCH;
        }
        break;
    case 14:
        {
            *out_value = MBEDTLS_X509_BADCERT_EXPIRED;
        }
        break;
    case 15:
        {
            *out_value = MBEDTLS_X509_BADCERT_FUTURE;
        }
        break;
    case 16:
        {
            *out_value = MBEDTLS_X509_BADCERT_NOT_TRUSTED;
        }
        break;
    case 17:
        {
            *out_value = MBEDTLS_X509_BADCERT_BAD_MD;
        }
        break;
    case 18:
        {
            *out_value = MBEDTLS_X509_BADCRL_BAD_MD | MBEDTLS_X509_BADCERT_BAD_MD;
        }
        break;
    case 19:
        {
            *out_value = MBEDTLS_X509_BADCERT_OTHER;
        }
        break;
    case 20:
        {
            *out_value = MBEDTLS_X509_BADCERT_CN_MISMATCH;
        }
        break;
    case 21:
        {
            *out_value = MBEDTLS_X509_BADCERT_CN_MISMATCH + MBEDTLS_X509_BADCERT_NOT_TRUSTED;
        }
        break;
    case 22:
        {
            *out_value = MBEDTLS_X509_BADCRL_NOT_TRUSTED;
        }
        break;
    case 23:
        {
            *out_value = MBEDTLS_X509_BADCERT_REVOKED|MBEDTLS_X509_BADCRL_FUTURE;
        }
        break;
    case 24:
        {
            *out_value = MBEDTLS_X509_BADCERT_BAD_MD|MBEDTLS_X509_BADCERT_BAD_PK|MBEDTLS_X509_BADCERT_BAD_KEY|MBEDTLS_X509_BADCRL_BAD_MD|MBEDTLS_X509_BADCRL_BAD_PK;
        }
        break;
    case 25:
        {
            *out_value = MBEDTLS_X509_BADCERT_BAD_PK;
        }
        break;
    case 26:
        {
            *out_value = MBEDTLS_X509_BADCERT_BAD_MD|MBEDTLS_X509_BADCRL_BAD_MD;
        }
        break;
    case 27:
        {
            *out_value = MBEDTLS_ERR_X509_INVALID_FORMAT;
        }
        break;
    case 28:
        {
            *out_value = MBEDTLS_ERR_X509_INVALID_FORMAT + MBEDTLS_ERR_ASN1_OUT_OF_DATA;
        }
        break;
    case 29:
        {
            *out_value = MBEDTLS_ERR_X509_INVALID_FORMAT + MBEDTLS_ERR_ASN1_INVALID_LENGTH;
        }
        break;
    case 30:
        {
            *out_value = MBEDTLS_ERR_X509_INVALID_FORMAT + MBEDTLS_ERR_ASN1_UNEXPECTED_TAG;
        }
        break;
    case 31:
        {
            *out_value = MBEDTLS_ERR_X509_INVALID_SERIAL + MBEDTLS_ERR_ASN1_UNEXPECTED_TAG;
        }
        break;
    case 32:
        {
            *out_value = MBEDTLS_ERR_X509_INVALID_VERSION + MBEDTLS_ERR_ASN1_UNEXPECTED_TAG;
        }
        break;
    case 33:
        {
            *out_value = MBEDTLS_ERR_X509_INVALID_VERSION + MBEDTLS_ERR_ASN1_OUT_OF_DATA;
        }
        break;
    case 34:
        {
            *out_value = MBEDTLS_ERR_X509_INVALID_VERSION + MBEDTLS_ERR_ASN1_INVALID_LENGTH;
        }
        break;
    case 35:
        {
            *out_value = MBEDTLS_ERR_X509_INVALID_SERIAL + MBEDTLS_ERR_ASN1_OUT_OF_DATA;
        }
        break;
    case 36:
        {
            *out_value = MBEDTLS_ERR_X509_INVALID_VERSION + MBEDTLS_ERR_ASN1_LENGTH_MISMATCH;
        }
        break;
    case 37:
        {
            *out_value = MBEDTLS_ERR_X509_INVALID_ALG + MBEDTLS_ERR_ASN1_OUT_OF_DATA;
        }
        break;
    case 38:
        {
            *out_value = MBEDTLS_ERR_X509_INVALID_ALG + MBEDTLS_ERR_ASN1_UNEXPECTED_TAG;
        }
        break;
    case 39:
        {
            *out_value = MBEDTLS_ERR_X509_UNKNOWN_VERSION;
        }
        break;
    case 40:
        {
            *out_value = MBEDTLS_ERR_X509_INVALID_ALG + MBEDTLS_ERR_ASN1_LENGTH_MISMATCH;
        }
        break;
    case 41:
        {
            *out_value = MBEDTLS_ERR_X509_UNKNOWN_SIG_ALG + MBEDTLS_ERR_OID_NOT_FOUND;
        }
        break;
    case 42:
        {
            *out_value = MBEDTLS_ERR_X509_INVALID_NAME + MBEDTLS_ERR_ASN1_OUT_OF_DATA;
        }
        break;
    case 43:
        {
            *out_value = MBEDTLS_ERR_X509_INVALID_NAME + MBEDTLS_ERR_ASN1_UNEXPECTED_TAG;
        }
        break;
    case 44:
        {
            *out_value = MBEDTLS_ERR_X509_INVALID_NAME+MBEDTLS_ERR_ASN1_UNEXPECTED_TAG;
        }
        break;
    case 45:
        {
            *out_value = MBEDTLS_ERR_X509_INVALID_DATE + MBEDTLS_ERR_ASN1_OUT_OF_DATA;
        }
        break;
    case 46:
        {
            *out_value = MBEDTLS_ERR_X509_INVALID_DATE + MBEDTLS_ERR_ASN1_LENGTH_MISMATCH;
        }
        break;
    case 47:
        {
            *out_value = MBEDTLS_ERR_X509_INVALID_DATE;
        }
        break;
    case 48:
        {
            *out_value = MBEDTLS_ERR_PK_KEY_INVALID_FORMAT + MBEDTLS_ERR_ASN1_OUT_OF_DATA;
        }
        break;
    case 49:
        {
            *out_value = MBEDTLS_ERR_PK_INVALID_ALG + MBEDTLS_ERR_ASN1_OUT_OF_DATA;
        }
        break;
    case 50:
        {
            *out_value = MBEDTLS_ERR_PK_UNKNOWN_PK_ALG;
        }
        break;
    case 51:
        {
            *out_value = MBEDTLS_ERR_PK_INVALID_PUBKEY + MBEDTLS_ERR_ASN1_OUT_OF_DATA;
        }
        break;
    case 52:
        {
            *out_value = MBEDTLS_ERR_PK_INVALID_PUBKEY + MBEDTLS_ERR_ASN1_INVALID_DATA;
        }
        break;
    case 53:
        {
            *out_value = MBEDTLS_ERR_PK_INVALID_PUBKEY + MBEDTLS_ERR_ASN1_LENGTH_MISMATCH;
        }
        break;
    case 54:
        {
            *out_value = MBEDTLS_ERR_PK_INVALID_PUBKEY + MBEDTLS_ERR_ASN1_UNEXPECTED_TAG;
        }
        break;
    case 55:
        {
            *out_value = MBEDTLS_ERR_PK_INVALID_PUBKEY;
        }
        break;
    case 56:
        {
            *out_value = MBEDTLS_ERR_X509_INVALID_FORMAT + MBEDTLS_ERR_ASN1_LENGTH_MISMATCH;
        }
        break;
    case 57:
        {
            *out_value = MBEDTLS_ERR_ASN1_INVALID_LENGTH;
        }
        break;
    case 58:
        {
            *out_value = MBEDTLS_ERR_X509_INVALID_EXTENSIONS + MBEDTLS_ERR_ASN1_OUT_OF_DATA;
        }
        break;
    case 59:
        {
            *out_value = MBEDTLS_ERR_X509_INVALID_EXTENSIONS + MBEDTLS_ERR_ASN1_LENGTH_MISMATCH;
        }
        break;
    case 60:
        {
            *out_value = MBEDTLS_ERR_X509_INVALID_EXTENSIONS;
        }
        break;
    case 61:
        {
            *out_value = MBEDTLS_ERR_X509_SIG_MISMATCH;
        }
        break;
    case 62:
        {
            *out_value = MBEDTLS_ERR_X509_INVALID_SIGNATURE + MBEDTLS_ERR_ASN1_OUT_OF_DATA;
        }
        break;
    case 63:
        {
            *out_value = MBEDTLS_ERR_X509_INVALID_SIGNATURE + MBEDTLS_ERR_ASN1_INVALID_DATA;
        }
        break;
    case 64:
        {
            *out_value = MBEDTLS_ERR_X509_UNKNOWN_SIG_ALG;
        }
        break;
    case 65:
        {
            *out_value = MBEDTLS_ERR_ASN1_OUT_OF_DATA;
        }
        break;
    case 66:
        {
            *out_value = MBEDTLS_ERR_X509_INVALID_DATE + MBEDTLS_ERR_ASN1_UNEXPECTED_TAG;
        }
        break;
    case 67:
        {
            *out_value = MBEDTLS_ERR_X509_INVALID_EXTENSIONS + MBEDTLS_ERR_ASN1_INVALID_LENGTH;
        }
        break;
    case 68:
        {
            *out_value = MBEDTLS_X509_MAX_INTERMEDIATE_CA;
        }
        break;
    case 69:
        {
            *out_value = MBEDTLS_X509_MAX_INTERMEDIATE_CA-1;
        }
        break;
    case 70:
        {
            *out_value = MBEDTLS_X509_MAX_INTERMEDIATE_CA+1;
        }
        break;
    case 71:
        {
            *out_value = MBEDTLS_ERR_X509_FATAL_ERROR;
        }
        break;
    case 72:
        {
            *out_value = -1;
        }
        break;
    case 73:
        {
            *out_value = MBEDTLS_ERR_X509_BAD_INPUT_DATA;
        }
        break;
    case 74:
        {
            *out_value = MBEDTLS_X509_BADCERT_BAD_MD|MBEDTLS_X509_BADCERT_BAD_PK|MBEDTLS_X509_BADCERT_BAD_KEY;
        }
        break;
    case 75:
        {
            *out_value = MBEDTLS_X509_BADCERT_BAD_PK|MBEDTLS_X509_BADCERT_BAD_KEY;
        }
        break;
    case 76:
        {
            *out_value = MBEDTLS_X509_BADCERT_BAD_MD|MBEDTLS_X509_BADCERT_BAD_KEY;
        }
        break;
    case 77:
        {
            *out_value = -2;
        }
        break;
    case 78:
        {
            *out_value = -4;
        }
        break;
    case 79:
        {
            *out_value = -3;
        }
        break;
    case 80:
        {
            *out_value = MBEDTLS_ERR_OID_BUF_TOO_SMALL;
        }
        break;
    case 81:
        {
            *out_value = MBEDTLS_X509_KU_DIGITAL_SIGNATURE|MBEDTLS_X509_KU_KEY_ENCIPHERMENT;
        }
        break;
    case 82:
        {
            *out_value = MBEDTLS_X509_KU_KEY_CERT_SIGN;
        }
        break;
    case 83:
        {
            *out_value = MBEDTLS_X509_KU_DIGITAL_SIGNATURE;
        }
        break;
    case 84:
        {
            *out_value = MBEDTLS_X509_KU_KEY_CERT_SIGN|MBEDTLS_X509_KU_CRL_SIGN;
        }
        break;
    case 85:
        {
            *out_value = MBEDTLS_X509_KU_KEY_ENCIPHERMENT|MBEDTLS_X509_KU_KEY_AGREEMENT;
        }
        break;
    case 86:
        {
            *out_value = MBEDTLS_X509_KU_DIGITAL_SIGNATURE|MBEDTLS_X509_KU_KEY_ENCIPHERMENT|MBEDTLS_X509_KU_DECIPHER_ONLY;
        }
        break;
    case 87:
        {
            *out_value = MBEDTLS_ASN1_CONSTRUCTED | MBEDTLS_ASN1_SEQUENCE;
        }
        break;
    case 88:
        {
            *out_value = MBEDTLS_MD_SHA1;
        }
        break;
    case 89:
        {
            *out_value = MBEDTLS_ASN1_SEQUENCE;
        }
        break;
    case 90:
        {
            *out_value = MBEDTLS_MD_SHA256;
        }
        break;
    case 91:
        {
            *out_value = MBEDTLS_ERR_X509_INVALID_ALG + MBEDTLS_ERR_ASN1_INVALID_DATA;
        }
        break;
    case 92:
        {
            *out_value = MBEDTLS_ERR_X509_INVALID_ALG + MBEDTLS_ERR_OID_NOT_FOUND;
        }
        break;
    case 93:
        {
            *out_value = MBEDTLS_ERR_X509_FEATURE_UNAVAILABLE + MBEDTLS_ERR_OID_NOT_FOUND;
        }
        break;
    case 94:
        {
            *out_value = MBEDTLS_ERR_X509_INVALID_ALG;
        }
        break;
    case 95:
        {
            *out_value = MBEDTLS_ERR_PK_KEY_INVALID_FORMAT + MBEDTLS_ERR_ASN1_UNEXPECTED_TAG;
        }
        break;
    case 96:
        {
            *out_value = MBEDTLS_ERR_X509_INVALID_SIGNATURE + MBEDTLS_ERR_ASN1_UNEXPECTED_TAG;
        }
        break;
    case 97:
        {
            *out_value = MBEDTLS_ERR_PEM_INVALID_DATA + MBEDTLS_ERR_BASE64_INVALID_CHARACTER;
        }
        break;
    case 98:
        {
            *out_value = MBEDTLS_ASN1_UTC_TIME;
        }
        break;
    case 99:
        {
            *out_value = MBEDTLS_ASN1_GENERALIZED_TIME;
        }
        break;
    case 100:
        {
            *out_value = MBEDTLS_ASN1_CONTEXT_SPECIFIC;
        }
        break;
    case 101:
        {
            *out_value = MBEDTLS_ERR_X509_INVALID_DATE+MBEDTLS_ERR_ASN1_UNEXPECTED_TAG;
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

static int check_test_x509( int func_idx )
{
    int ret = DISPATCH_TEST_SUCCESS;
    TestWrapper_t fp = NULL;

    if ( func_idx < (int)( sizeof(test_funcs_x509)/sizeof( TestWrapper_t ) ) )
    {
        fp = test_funcs_x509[func_idx];
        if ( fp == NULL )
            ret = DISPATCH_UNSUPPORTED_SUITE;
    }
    else
    {
        ret = DISPATCH_TEST_FN_NOT_FOUND;
    }

    return ret;
}

static int dispatch_test_x509( int func_idx, void ** params )
{
    int ret = DISPATCH_TEST_SUCCESS;
    TestWrapper_t fp = NULL;

    if ( func_idx < (int)( sizeof( test_funcs_x509 ) / sizeof( TestWrapper_t ) ) )
    {
        fp = test_funcs_x509[func_idx];
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

int verify_none( void *data, mbedtls_x509_crt *crt, int certificate_depth, uint32_t *flags )
{
    ((void) data);
    ((void) crt);
    ((void) certificate_depth);
    *flags |= MBEDTLS_X509_BADCERT_OTHER;

    return 0;
}

int verify_all( void *data, mbedtls_x509_crt *crt, int certificate_depth, uint32_t *flags )
{
    ((void) data);
    ((void) crt);
    ((void) certificate_depth);
    *flags = 0;

    return 0;
}

int verify_fatal( void *data, mbedtls_x509_crt *crt, int certificate_depth, uint32_t *flags )
{
    int *levels = (int *) data;

    ((void) crt);
    ((void) certificate_depth);

    /* Simulate a fatal error in the callback */
    if( *levels & ( 1 << certificate_depth ) )
    {
        *flags |= ( 1 << certificate_depth );
        return ( -1 - certificate_depth );
    }

    return 0;
}

/* strsep() not available on Windows */
char *mystrsep(char **stringp, const char *delim)
{
    const char *p;
    char *ret = *stringp;

    if( *stringp == NULL )
        return NULL;

    for( ; ; (*stringp)++ )
    {
        if( **stringp == '\0' )
        {
            *stringp = NULL;
            goto done;
        }

        for( p = delim; *p != '\0'; p++ )
            if( **stringp == *p )
            {
                **stringp = '\0';
                (*stringp)++;
                goto done;
            }
    }

done:
    return ret;
}

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

int* mbedtls_test_x509()
{
    TEST_MESSAGE("----------------------------TEST FOR x509----------------------------");
    uint16_t i = 0, j = 0;
    int ret = 0;
    uint8_t function_id = 0;
    char *params[50];
    int int_params[50];
    char* buffer;
    char unity_buffer[100];
    /* Other Local variables */
    int cnt;
    int total_errors = 0, total_tests = 0, total_skipped = 0;
    memset( &test_info, 0, sizeof( test_info ) );
    while ( i < sizeof(data_x509)/sizeof(data_x509[0]) )
    {
        buffer = (char*)malloc(sizeof(char)*strlen(data_x509[i]));
        strcpy(buffer, data_x509[i]);
        total_tests++;
        int unmet_dep_count = 0;
        char* unmet_dependencies[20];
        memset(unmet_dependencies, 0x0, sizeof(unmet_dependencies));
        ret = 0;
        test_info.failed = 0;

        cnt = parse_arguments( (char *)buffer, strlen( ((char *)buffer )), params,
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
            i++;
            free(buffer);
            buffer = (char*)malloc(sizeof(char)*strlen(data_x509[i]));
            strcpy(buffer, data_x509[i]);
            cnt = parse_arguments( (char *)buffer, strlen( ((char *)buffer )), params,
                                    sizeof( params ) / sizeof( params[0] ) );
        }

        // If there are no unmet dependencies execute the test
        if( unmet_dep_count == 0 )
        {
            test_info.failed = 0;
            function_id = strtol( params[0], NULL, 10 );
            if ( (ret = check_test_x509( function_id )) == DISPATCH_TEST_SUCCESS )
            {
                ret = convert_params( cnt - 1, params + 1, int_params );
                if ( DISPATCH_TEST_SUCCESS == ret )
                {
                    ret = dispatch_test_x509( function_id, (void **)( params + 1 ) );
                }
            }
        }
        if( unmet_dep_count > 0 || ret == DISPATCH_UNSUPPORTED_SUITE )
        {
            total_skipped++;
            sprintf(unity_buffer, "X509-Test %d : Skipped", total_tests);
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
                sprintf(unity_buffer, "x509-Test %d : PASSED", total_tests);
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
        TEST_MESSAGE("ALL X509 PASSED" );
    else if(total_errors>0)
        TEST_MESSAGE("X509 TESTS FAILED" );
    else
        TEST_MESSAGE("Some tests skipped" );

    int* test_results = (int *)malloc(3*sizeof(int));
    test_results[0] = total_tests;
    test_results[1] = total_errors;
    test_results[2] = total_skipped;
    return test_results;
}