

files = [
    "nx_crypto_gcm.c",
    "nx_crypto_method_self_test_hmac_sha.c",
    "nx_crypto_tls_prf_sha256.c",
    "nx_crypto_hmac_sha5.c",
    "nx_crypto_method_self_test_rsa.c",
    "nx_crypto_ec.c",
    "nx_crypto_method_self_test_prf.c",
    "nx_crypto_cbc.c",
    "nx_crypto_drbg.c",
    "nx_crypto_ecdh.c",
    "nx_crypto_aes.c",
    "nx_crypto_huge_number_extended.c",
    "nx_crypto_method_self_test_drbg.c",
    "nx_crypto_hmac.c",
    "nx_crypto_dh.c",
    "nx_crypto_method_self_test_ecdh.c",
    "nx_crypto_ec_secp224r1_fixed_points.c",
    "nx_crypto_huge_number.c",
    "nx_crypto_xcbc_mac.c",
    "nx_crypto_ecjpake.c",
    "nx_crypto_method_self_test_des.c",
    "nx_crypto_ec_secp521r1_fixed_points.c",
    "nx_crypto_method_self_test_ecdsa.c",
    "nx_crypto_method_self_test_hmac_md5.c",
    "nx_crypto_ec_secp256r1_fixed_points.c",
    "nx_crypto_null_cipher.c",
    "nx_crypto_initialize.c",
    "nx_crypto_methods.c",
    "nx_crypto_md5.c",
    "nx_crypto_sha5.c",
    "nx_crypto_hmac_md5.c",
    "nx_crypto_sha1.c",
    "nx_crypto_method_self_test_3des.c",
    "nx_crypto_tls_prf_sha512.c",
    "nx_crypto_ec_secp384r1_fixed_points.c",
    "nx_crypto_des.c",
    "nx_crypto_3des.c",
    "nx_crypto_tls_prf_sha384.c",
    "nx_crypto_method_self_test_md5.c",
    "nx_crypto_phash.c",
    "nx_crypto_hmac_sha2.c",
    "nx_crypto_sha2.c",
    "nx_crypto_rsa.c",
    "nx_crypto_hkdf.c",
    "nx_crypto_method_self_test_sha.c",
    "nx_crypto_generic_ciphersuites.c",
    "nx_crypto_method_self_test.c",
    "nx_crypto_method_self_test_pkcs1.c",
    "nx_crypto_ccm.c",
    "nx_crypto_pkcs1_v1.5.c",
    "nx_crypto_ec_secp192r1_fixed_points.c",
    "nx_crypto_ecdsa.c",
    "nx_crypto_method_self_test_aes.c",
    "nx_crypto_tls_prf_1.c",
    "nx_crypto_ctr.c",
    "nx_crypto_hmac_sha1.c",
];

file_dirs = [
    "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/crypto_libraries/src",
];

includes = [
    "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/crypto_libraries/inc",
];

module.exports = {
    files,
    file_dirs,
    includes
};