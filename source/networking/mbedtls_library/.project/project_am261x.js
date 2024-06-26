let path = require('path');

let device = "am261x";

const files = {
    common: [
       "aesni.c",
       "aes.c ",
       "arc4.c",
       "aria.c ",
       "asn1parse.c",
       "asn1write.c",
       "base64.c",
       "bignum.c",
       "blowfish.c",
       "camellia.c",
       "ccm.c",
       "certs.c",
       "chacha20.c",
       "chachapoly.c",
       "cipher.c",
       "cipher_wrap.c",
       "cmac.c",
       "ctr_drbg.c",
       "debug.c",
       "des.c",
       "dhm.c",
       "ecdh.c",
       "ecdsa.c",
       "ecjpake.c",
       "ecp.c",
       "ecp_curves.c",
       "entropy.c",
       "entropy_poll.c",
       "error.c",
       "gcm.c",
       "havege.c",
       "hkdf.c",
       "hmac_drbg.c",
       "md.c",
       "md2.c",
       "md4.c",
       "md5.c",
       "md_wrap.c",
       "memory_buffer_alloc.c",
       "net_sockets.c",
       "nist_kw.c",
       "oid.c",
       "padlock.c",
       "pem.c",
       "pk.c",
       "pkcs11.c",
       "pkcs12.c",
       "pkcs5.c",
       "pkparse.c",
       "pkwrite.c",
       "pk_wrap.c",
       "platform.c",
       "platform_util.c",
       "poly1305.c",
       "ripemd160.c",
       "rsa.c",
       "rsa_internal.c",
       "sha1.c",
       "sha256.c",
       "sha512.c",
       "ssl_cache.c",
       "ssl_ciphersuites.c",
       "ssl_cli.c",
       "ssl_cookie.c",
       "ssl_srv.c",
       "ssl_ticket.c",
       "ssl_tls.c",
       "threading.c",
       "timing.c",
       "timing_alt.c",
       "version.c",
       "version_features.c",
       "x509.c",
       "x509write_crt.c",
       "x509write_csr.c",
       "x509_create.c",
       "x509_crl.c",
       "x509_crt.c",
       "x509_csr.c",
       "xtea.c",
    ],
};
const includes = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/networking/mbedtls_library/mbedtls/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/mbedtls_library/mbedtls_ti",
        "${MCU_PLUS_SDK_PATH}/source/kernel/dpl",
    ],
};
const filedirs = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/networking/mbedtls_library/mbedtls/library",
        "${MCU_PLUS_SDK_PATH}/source/networking/mbedtls_library/mbedtls_ti",
        "${MCU_PLUS_SDK_PATH}/source/kernel/dpl",
    ],
};
const cflags = {
    common: [
        "-mno-unaligned-access",
        "-Wno-extra",
        "-Wvisibility",
        "-Wno-error=unused-but-set-variable",
        "-Wno-unused-but-set-variable",
    ],
    release: [
        "-Oz",
        "-flto",
    ],
    cpp_common: [
        "-E",
    ]
};

const defines_r5f = {
    common: [
        "MBEDTLS_CONFIG_FILE=\\\"alt_config.h\\\"",
    ],
};

const buildOptionCombos = [
    { device: device, cpu: "r5f", cgt: "ti-arm-clang"},
];
function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "library";
    property.name = "mbedtls";
    property.isInternal = false;
    property.buildOptionCombos = buildOptionCombos;

    return property;
}
function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.files = files;
    build_property.filedirs = filedirs;
    build_property.includes = includes;
    build_property.cflags = cflags;
    build_property.defines = defines_r5f;
    return build_property;
}
module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};