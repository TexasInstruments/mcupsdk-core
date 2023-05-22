import hashlib
import hmac

hash_len = 64


def hmac_sha512(key, data):
    return bytearray(hmac.new(key, data, hashlib.sha512).digest())


def hkdf(length, ikm, salt):
    prk = hmac_sha512(salt, ikm)
    t = bytearray()
    okm = bytearray()

    for i in range(int((length + hash_len - 1) / hash_len)):
        t = bytearray(hmac_sha512(prk, t + bytearray([1+i])))
        okm += t

    return okm[:length]


