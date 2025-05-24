
import struct
import sys
from cryptography.hazmat.primitives import serialization

def swap32(x):
    return (((x << 24) & 0xFF000000) |
            ((x <<  8) & 0x00FF0000) |
            ((x >>  8) & 0x0000FF00) |
            ((x >> 24) & 0x000000FF))

def int_to_bigint(i):
    word_array = []
    while i > 0:
        (i, r) = divmod(i, (2**32))
        # r = swap32(r)
        word_array.append(r)

    # word_array = word_array[::-1];
    return (word_array);

def bigint_to_str(bi):
    items_per_line = 6
    s = ''
    s += '{' + str(len(bi))
    while(len(bi)):
        s += ',\n'
        s += ', '.join("0x{:08x}".format(x) for x in bi[:items_per_line])
        bi = bi[items_per_line:]
    s += '}'

    return (s)


if __name__ == "__main__":

    # Open the key file.
    with open(sys.argv[1], "rb") as key_file:
        private_key = serialization.load_pem_private_key(key_file.read(), password=None);

    # Extract the key.
    privn = private_key.private_numbers();

    # Print key parameters (public, private and optimization coefficients) as a C structure init.
    print("{")
    print(bigint_to_str(int_to_bigint(privn.public_numbers.n))) # n (modulus)
    print(",")
    print(bigint_to_str(int_to_bigint(privn.public_numbers.e))) # e (public exponent)
    print(",")
    print(bigint_to_str(int_to_bigint(privn.d))) # d (private exponent)
    print(",")
    print(bigint_to_str(int_to_bigint(privn.p))) # p prime
    print(",")
    print(bigint_to_str(int_to_bigint(privn.q))) # q prime
    print(",")
    print(bigint_to_str(int_to_bigint(privn.dmp1))) # dp
    print(",")
    print(bigint_to_str(int_to_bigint(privn.dmq1))) # dq
    print(",")
    print(bigint_to_str(int_to_bigint(privn.iqmp))) # iq
    print("}")
