

files = [
    "nx_crypto_sa2ul.c",
    "nx_crypto_sa2ul_aes.c",
    "nx_crypto_sa2ul_ecdsa.c",
    "nx_crypto_sa2ul_rsa.c",
    "nx_crypto_sa2ul_util.c"
];

file_dirs = [
    "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_enet/crypto_hw/sa2ul",
];

includes = [ 
    "${MCU_PLUS_SDK_PATH}/source/security",
];

module.exports = {
    files,
    file_dirs,
    includes
};