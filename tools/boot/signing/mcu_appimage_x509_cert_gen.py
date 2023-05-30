# Script to add x509 certificate to binary/ELF
# for booting application images
# The certificate format used here is different from
# the format used by ROM. Supposed to be read by HSM
#
# Python 3 script

import argparse
import os
import subprocess
import binascii
from re import sub
from random import randint
import shutil
from textwrap import dedent
from hkdf import hkdf

# Some globals
g_sha_to_use = "sha512"

g_valid_cores = [
    "r5_cl0_c0",
    "r5_cl0_c1",
    "r5_cl1_c0",
    "r5_cl1_c1",
]

g_core_ids = {
    "r5_cl0_c0" : 0x01,
    "r5_cl0_c1" : 0x02,
    "r5_cl1_c0" : 0x06,
    "r5_cl1_c1" : 0x07,
}

g_sha_oids = {
    "sha256" : "2.16.840.1.101.3.4.2.1",
    "sha384" : "2.16.840.1.101.3.4.2.2",
    "sha512" : "2.16.840.1.101.3.4.2.3",
    "sha224" : "2.16.840.1.101.3.4.2.4",
}

g_x509_template = '''
[ req ]
distinguished_name     = req_distinguished_name
x509_extensions        = v3_ca
prompt                 = no

dirstring_type = nobmp

[ req_distinguished_name ]
C                      = US
ST                     = SC
L                      = New York
O                      = Texas Instruments., Inc.
OU                     = DSP
CN                     = Albert
emailAddress           = Albert@gt.ti.com

[ v3_ca ]
basicConstraints = CA:true
1.3.6.1.4.1.294.1.1=ASN1:SEQUENCE:boot_seq
1.3.6.1.4.1.294.1.2=ASN1:SEQUENCE:image_integrity
1.3.6.1.4.1.294.1.3=ASN1:SEQUENCE:swrv
{ENCRYPTION_SEQUENCE}

[ boot_seq ]
certType     =  INTEGER:{CERT_TYPE}
bootCore     =  INTEGER:0
bootCoreOpts =  INTEGER:0
destAddr     =  FORMAT:HEX,OCT:00000000
imageSize    =  INTEGER:{IMAGE_LENGTH}

[ image_integrity ]
shaType = OID:{TEST_IMAGE_SHA_OID}
shaValue = FORMAT:HEX,OCT:{TEST_IMAGE_SHA_VAL}

[ swrv ]
swrv = INTEGER:{SWRV}

'''
g_enc_boot_seq = '''
[ encryption ]
initalVector =  FORMAT:HEX,OCT:{TEST_IMAGE_ENC_IV}
randomString =  FORMAT:HEX,OCT:{TEST_IMAGE_ENC_RS}
iterationCnt =  INTEGER:{TEST_IMAGE_KEY_DERIVE_INDEX}
salt         =  FORMAT:HEX,OCT:{TEST_IMAGE_KEY_DERIVE_SALT}
'''

def get_cert(args):
    '''Generate the x509 certificate config'''
    print("Generating certificate for {} ...".format(args.bin))

    # Default values of template replacements
    v_TEST_IMAGE_SHA_OID          = g_sha_oids[g_sha_to_use]
    v_TEST_IMAGE_SHA_VAL          = None
    v_TEST_IMAGE_LENGTH           = None
    v_TEST_BOOT_CORE              = None
    v_RESET_VECTOR                = ""
    v_TEST_IMAGE_ENC_IV           = "0000"
    v_TEST_IMAGE_ENC_RS           = "0000"
    v_TEST_IMAGE_KEY_DERIVE_INDEX = 0
    v_TEST_IMAGE_KEY_DERIVE_SALT  = "0000"
    device_cert_type              = 0xA5A50000

    # Validation of args.

    # Bin file has to be present
    bin_file = args.bin
    swrev = args.swrv

    if((args.bin is None) or (not os.path.exists(args.bin))):
        # No file, exit
        print("Binary file not found!")
        exit(2)
    else:
        # Check if encryption is enabled
        if(args.enc == 'y'):
            enc_app_name, v_TEST_IMAGE_ENC_IV, v_TEST_IMAGE_ENC_RS = get_encrypted_file_iv_rs(args.bin, args.enckey)
            if args.kd_salt:
                v_TEST_IMAGE_KEY_DERIVE_INDEX = 1
                v_TEST_IMAGE_KEY_DERIVE_SALT = get_key_derivation_salt(args.kd_salt)
            bin_file = enc_app_name
        else:
            pass

        # Get file size and SHA value
        v_TEST_IMAGE_LENGTH = os.path.getsize(bin_file)
        sha_val = subprocess.check_output('openssl dgst -{} -hex {}'.format(g_sha_to_use, bin_file), shell=True).decode()
        v_TEST_IMAGE_SHA_VAL = sub("^.*= ", r'', sha_val).strip('\n')

    # Load address has to be valid hex
    # TODO

    # Has to provide key for authentication
    if((args.key is None) or(not os.path.exists(args.key))):
        # No file, exit
        print("Authentication key file not found!")
        exit(2)
    else:
        pass

    if(swrev is None):
            # Default to 1
            swrev = 1
        # Replace the variables in the main template now.
    enc_seq=''
    if args.enc:
        enc_seq = "1.3.6.1.4.1.294.1.4 = ASN1:SEQUENCE:encryption"
    else:
        enc_seq = ''

    ret_cert = g_x509_template.format(TEST_IMAGE_SHA_OID=v_TEST_IMAGE_SHA_OID,
                TEST_IMAGE_SHA_VAL=v_TEST_IMAGE_SHA_VAL,
                TEST_IMAGE_LENGTH=v_TEST_IMAGE_LENGTH,
                CERT_TYPE=device_cert_type,
                SWRV=swrev,
                IMAGE_LENGTH = os.path.getsize(bin_file),
                ENCRYPTION_SEQUENCE=enc_seq)

    # If encryption is enabled, append that sequence to the current certificate
    if args.enc:
        ret_cert += g_enc_boot_seq.format(TEST_IMAGE_ENC_IV=v_TEST_IMAGE_ENC_IV,
                                            TEST_IMAGE_ENC_RS=v_TEST_IMAGE_ENC_RS,
                                            TEST_IMAGE_KEY_DERIVE_INDEX=v_TEST_IMAGE_KEY_DERIVE_INDEX,
                                            TEST_IMAGE_KEY_DERIVE_SALT=v_TEST_IMAGE_KEY_DERIVE_SALT)

    # NOTE: Boot sequence is not used. We assume that SBL always sets the reset vectors and does image load

    return dedent(ret_cert)

def get_enc_filename(fname):
    return fname+"-enc"

def get_encrypted_file_iv_rs(bin_file_name, enc_key):
    if((enc_key is None) or (not os.path.exists(enc_key))):
        # Error, enc key has to be given
        print("Please give the key to be used for encryption. It's either missing or file not found!")
        exit(1)
    else:
        enckey = None
        with open(enc_key, "rb") as f:
            enckey = f.read()
            if(args.kd_salt is not None):
                isalt = get_key_derivation_salt(args.kd_salt)
                isalt = bytearray(binascii.unhexlify(isalt))
                d_key = hkdf(32, enckey, isalt)
                enckey = binascii.hexlify(d_key).decode('utf-8')
            else:
                enckey = binascii.hexlify(enckey).decode('ascii')

        # we need the value of enc_iv as hex, so convert the bytes output to hex
        enc_iv = subprocess.check_output('openssl rand 16', shell=True)
        enc_iv = binascii.hexlify(enc_iv).decode('ascii')
        v_TEST_IMAGE_ENC_IV = enc_iv

        # we don't need the value of enc_rs as hex for encryption, so keep the bytes object
        enc_rs = subprocess.check_output('openssl rand 32', shell=True)
        v_TEST_IMAGE_ENC_RS = binascii.hexlify(enc_rs).decode('ascii')

        # Pad zeros to a temporary binary to make the size multiple of 16
        zeros_pad = bytearray( 16 - (os.path.getsize(bin_file_name) % 16))
        tempfile_name = "tmpfile" + str(randint(1111, 9999))
        encbin_name = get_enc_filename(bin_file_name)

        shutil.copy(bin_file_name, tempfile_name)

        # append zeros to tempfile
        with open(tempfile_name, "ab") as f:
            f.write(zeros_pad)
            # append the enc_rs value to the padded tempfile
            f.write(enc_rs)

        # Finally generate the encrypted image
        subprocess.check_output('openssl aes-256-cbc -e -K {} -iv {} -in {} -out {} -nopad'.format(enckey, enc_iv, tempfile_name, encbin_name), shell=True)

        # Delete the tempfile
        os.remove(tempfile_name)

        return encbin_name, v_TEST_IMAGE_ENC_IV, v_TEST_IMAGE_ENC_RS

def get_key_derivation_salt(kd_salt_file_name):
    if(not os.path.exists(kd_salt_file_name)):
        # Error, key derivation salt has to be given
        print("Please give the key derivation salt file name. It's either missing or file not found!")
        exit(1)
    else:
        kd_salt = None
        with open(kd_salt_file_name, "r") as f:
            kd_salt = f.read()
            kd_salt = kd_salt.strip('\n')

    return kd_salt


# arguments definition
my_parser = argparse.ArgumentParser(description="Generates a x509 certificate for an application binary to boot it in HS device")

my_parser.add_argument('--bin',        type=str, help='Bin file that needs to be signed')
my_parser.add_argument('--key',        type=str, help='File with signing key inside it')
my_parser.add_argument('--swrv',        type=str, help='Sw Revision of the application')
my_parser.add_argument('--enckey',     type=str, help='File with encryption key inside it')
my_parser.add_argument('--cert',       type=str, help='Certificate file name (optional, will use a default name otherwise)')
my_parser.add_argument('--output',     type=str, help='Output file name (concatenated cert+bin)')
my_parser.add_argument('--enc',        type=str, help='If the binary need to be encrypted or not [y/n]')
my_parser.add_argument('--kd-salt' ,   type=str, help='Path to the salt required to calculate derived key from manufacturers encryption key')
my_parser.add_argument('--loadaddr',   type=str, help='Target load address of the binary in hex. Default to 0x70000000')

args = my_parser.parse_args()
cert_str = get_cert(args)
cert_file_name = "temp_cert"+str(randint(111, 999))

with open(cert_file_name, "w+") as f:
    f.write(cert_str)

cert_name = args.cert
out_name = args.output

if(cert_name is None):
    cert_name = "cert"+str(randint(111, 999))

if(args.output is None):
    out_name = args.bin + "signed"

# Generate the certificate
subprocess.check_output('openssl req -new -x509 -key {} -nodes -outform DER -out {} -config {} -{}'.format(args.key, cert_name, cert_file_name, g_sha_to_use), shell=True)

# Concatenate the certificate with the binary. If binary was encrypted, concatenate with the encrypted image

# copy the certificate to output file
shutil.copy(cert_name, out_name)

unsigned_bin = args.bin

if(args.enc == 'y'):
    # choose the encrypted binary
    unsigned_bin = args.bin + "-enc"

bin_data = None

with open(unsigned_bin, "rb") as f:
    bin_data = f.read()

with open(out_name, "ab") as f_out:
    f_out.write(bin_data)

# remove the temporary files
os.remove(cert_file_name)
os.remove(cert_name)
