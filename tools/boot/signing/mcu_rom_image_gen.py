# Script to add x509 certificate to input binary (SBL or hsmRt)
# The certificate used will be ROM format
#
# Python 3 script

import argparse
import os
import sys
import subprocess
import binascii
from re import sub
from random import randint
import shutil
from textwrap import dedent
from hkdf import hkdf

g_sbl_hsm_max_size = 983000

g_sha_to_use = "sha512"

g_sha_oids = {
    "sha256" : "2.16.840.1.101.3.4.2.1",
    "sha384" : "2.16.840.1.101.3.4.2.2",
    "sha512" : "2.16.840.1.101.3.4.2.3",
    "sha224" : "2.16.840.1.101.3.4.2.4",
}


g_dbg_types = {
    "DBG_PERM_DISABLE"   : '0',
    "DBG_SOC_DEFAULT"    : '1',
    "DBG_PUBLIC_ENABLE"  : '2',
    "DBG_FULL_ENABLE"    : '4',
}

g_core_types = {
    "R5"          : '0',
    "HSM"         : '1',
}

g_openssl111_x509_template = '''
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
OU                     = SITARA MCU
CN                     = Albert
emailAddress           = Albert@gt.ti.com

[ v3_ca ]
basicConstraints = CA:true
1.3.6.1.4.1.294.1.1=ASN1:SEQUENCE:boot_seq
1.3.6.1.4.1.294.1.2=ASN1:SEQUENCE:image_integrity
1.3.6.1.4.1.294.1.3=ASN1:SEQUENCE:swrv
{EXT_ENC_SEQ}
{DBG_EXT}
{KD_EXT}

[ boot_seq ]
certType     =  INTEGER:{CERT_TYPE}
bootCore     =  INTEGER:{BOOT_CORE_ID}
bootCoreOpts =  INTEGER:{BOOT_CORE_OPTS}
destAddr     =  FORMAT:HEX,OCT:{BOOT_ADDR}
imageSize    =  INTEGER:{IMAGE_LENGTH}

[ image_integrity ]
shaType = OID:{SHA_OID}
shaValue = FORMAT:HEX,OCT:{SHA_VAL}

[ swrv ]
swrv = INTEGER:{SWRV}
'''

g_ext_enc_seq = '''
[ encryption ]
Iv =FORMAT:HEX,OCT:{ENC_IV}
Rstring = FORMAT:HEX,OCT:{ENC_RS}
Icount = INTEGER:{ENC_ITER_CNT}
Salt = FORMAT:HEX,OCT:{ENC_SALT}
'''

g_dbg_seq = '''
[ debug ]
debugUID     =  FORMAT:HEX,OCT:{DBG_DEVICE}
debugType    =  INTEGER:{DBG_TYPE}
coreDbgEn    =  INTEGER:0
coreDbgSecEn =  INTEGER:0
'''

g_kd_seq = '''
[ key_derivation ]
kd_salt = FORMAT:HEX,OCT:{KDSALT_VAL}
'''

g_openssl3_x509_template = '''
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
OU                     = SITARA MCU
CN                     = Albert
emailAddress           = Albert@gt.ti.com

[ v3_ca ]
basicConstraints = CA:true
subjectKeyIdentifier = none
1.3.6.1.4.1.294.1.1=ASN1:SEQUENCE:boot_seq
1.3.6.1.4.1.294.1.2=ASN1:SEQUENCE:image_integrity
1.3.6.1.4.1.294.1.3=ASN1:SEQUENCE:swrv
{EXT_ENC_SEQ}
{DBG_EXT}
{KD_EXT}

[ boot_seq ]
certType     =  INTEGER:{CERT_TYPE}
bootCore     =  INTEGER:{BOOT_CORE_ID}
bootCoreOpts =  INTEGER:{BOOT_CORE_OPTS}
destAddr     =  FORMAT:HEX,OCT:{BOOT_ADDR}
imageSize    =  INTEGER:{IMAGE_LENGTH}

[ image_integrity ]
shaType = OID:{SHA_OID}
shaValue = FORMAT:HEX,OCT:{SHA_VAL}

[ swrv ]
swrv = INTEGER:{SWRV}
'''

g_ext_enc_seq = '''
[ encryption ]
Iv =FORMAT:HEX,OCT:{ENC_IV}
Rstring = FORMAT:HEX,OCT:{ENC_RS}
Icount = INTEGER:{ENC_ITER_CNT}
Salt = FORMAT:HEX,OCT:{ENC_SALT}
'''

g_dbg_seq = '''
[ debug ]
debugUID     =  FORMAT:HEX,OCT:{DBG_DEVICE}
debugType    =  INTEGER:{DBG_TYPE}
coreDbgEn    =  INTEGER:0
coreDbgSecEn =  INTEGER:0
'''

g_kd_seq = '''
[ key_derivation ]
kd_salt = FORMAT:HEX,OCT:{KDSALT_VAL}
'''

def get_sha_val(f_name, sha_type):
    sha_val = subprocess.check_output('openssl dgst -{} -hex {}'.format(sha_type, f_name), shell=True).decode()
    return sub("^.*= ", r'', sha_val).strip('\n')

def get_cert(args):
    swrev = args.swrv
    if(swrev is None):
        swrev = 1

    if(args.core is not None):
        if(args.core == 'R5'):
            bootAddress = 0
            bootCore_id = 16
            certType = 1
            bootCoreOptions = 0
        else:
            bootAddress = 0
            bootCore_id = 0
            certType = 2
            bootCoreOptions = 0

    dbg_seq = ''
    ext_enc_seq = ''
    sbl_enc_seq = ''
    ext_kd_seq = ''
    kd_seq = ''

    if(args.debug is not None):
        if(args.debug in g_dbg_types):
            # all good
            dbg_seq = "1.3.6.1.4.1.294.1.8=ASN1:SEQUENCE:debug"
        else:
            dbg_seq = ""
            # Invalid debug extension, exit fail
            print("Invalid debug extension, exiting ...")
            exit(2)

    image_bin_name = args.image_bin

    if (args.sbl_enc or args.tifs_enc):
        enc_iter_count = ''
        enc_salt = ''
        if args.kd_salt:
            enc_iter_count = 1
            enc_salt = get_key_derivation_salt(args.kd_salt)
        else:
            enc_iter_count = 0
            enc_salt = '0000'

    if args.sbl_enc:
        # SBL encryption is enabled
        encsbl_name, enc_iv, enc_rs = get_encrypted_file_iv_rs(args.image_bin, args.enc_key)
        ext_enc_seq = "1.3.6.1.4.1.294.1.4=ASN1:SEQUENCE:encryption"
        sbl_enc_seq = g_ext_enc_seq.format(
                        NUM_COMP = 1,
                        ENC_IV = enc_iv,
                        ENC_RS = enc_rs,
                        ENC_ITER_CNT = enc_iter_count,
                        ENC_SALT = enc_salt,
                        extra_enc_comp='',
                        extra_enc_comp_seq='',
                        )
        image_bin_name = encsbl_name

    if args.tifs_enc:
        enctifs_name, enc_iv, enc_rs = get_encrypted_file_iv_rs(args.image_bin, args.enc_key)
        ext_enc_seq = "1.3.6.1.4.1.294.1.4=ASN1:SEQUENCE:encryption"
        tifs_enc_seq = g_ext_enc_seq.format(
                        NUM_COMP = 1,
                        ENC_IV = enc_iv,
                        ENC_RS = enc_rs,
                        ENC_ITER_CNT = enc_iter_count,
                        ENC_SALT = enc_salt,
                        extra_enc_comp='',
                        extra_enc_comp_seq='',
                        )
        image_bin_name = enctifs_name

    if args.kd_salt and args.sbl_enc:
        ext_kd_seq = "1.3.6.1.4.1.294.1.5=ASN1:SEQUENCE:key_derivation"
        kd_salt = get_key_derivation_salt(args.kd_salt)
        kd_seq = g_kd_seq.format(
                    KDSALT_VAL = kd_salt
                    )
    ret_cert = ""

    openssl_version: str = str(subprocess.check_output(f"openssl version", shell=True))

    if "1.1.1" in openssl_version:
        print(f"WARNING: OpenSSL version {openssl_version.split()[1]} found is not recommended due to EOL. Please install version 3.x .")
        ret_cert = g_openssl111_x509_template.format(
                    DBG_EXT = dbg_seq,
                    SHA_OID = g_sha_oids[g_sha_to_use],
                    SWRV = swrev,
                    EXT_ENC_SEQ = ext_enc_seq,
                    KD_EXT = ext_kd_seq,
                    BOOT_CORE_ID = bootCore_id,
                    CERT_TYPE = certType,
                    BOOT_CORE_OPTS = bootCoreOptions,
                    BOOT_ADDR = '{:08X}'.format(int(args.loadaddr, 16)),
                    IMAGE_LENGTH = os.path.getsize(image_bin_name),
                    SHA_VAL = get_sha_val(image_bin_name, g_sha_to_use),
                    )

    elif "3." in openssl_version:
        print(f"INFO: OpenSSL version {openssl_version.split()[1]} found.")
        ret_cert = g_openssl3_x509_template.format(
                    DBG_EXT = dbg_seq,
                    SHA_OID = g_sha_oids[g_sha_to_use],
                    SWRV = swrev,
                    EXT_ENC_SEQ = ext_enc_seq,
                    KD_EXT = ext_kd_seq,
                    BOOT_CORE_ID = bootCore_id,
                    CERT_TYPE = certType,
                    BOOT_CORE_OPTS = bootCoreOptions,
                    BOOT_ADDR = '{:08X}'.format(int(args.loadaddr, 16)),
                    IMAGE_LENGTH = os.path.getsize(image_bin_name),
                    SHA_VAL = get_sha_val(image_bin_name, g_sha_to_use),
                    )
    else:
        print(f"ERROR: OpenSSL version {openssl_version.split()[1]} found is not compatible. Please install version 1.1.1 or 3.x to continue.")
        sys.exit()

    if args.sbl_enc:
        ret_cert += sbl_enc_seq
    elif args.tifs_enc:
        ret_cert += tifs_enc_seq

    if(args.kd_salt and args.sbl_enc):
        ret_cert += kd_seq

    if(dbg_seq != ''):
        ret_cert += g_dbg_seq.format(
                DBG_DEVICE='00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000',
                DBG_TYPE=g_dbg_types[args.debug],
                 )
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

# MAIN
my_parser = argparse.ArgumentParser(description="Creates a ROM-boot-able images for sbl and hsmRt")

my_parser.add_argument('--image-bin',   type=str, required=True, help='Path to the SBL/hsmRT binary')
my_parser.add_argument('--core',        type=str, help='R5/HSM are the options to build for specific core')
my_parser.add_argument('--sbl-enc',     action='store_true', required=False, help='Encrypt SBL or not')
my_parser.add_argument('--tifs-enc',    action='store_true', required=False, help='Encrypt TIFS-MCU or not')
my_parser.add_argument('--enc-key',     type=str, required=False, help='Path to the SBL Encryption Key')
my_parser.add_argument('--swrv',        type=str, help='Software revision number')
my_parser.add_argument('--loadaddr',    type=str, required=True, help='Load address at which SBL/hsmRT needs to be loaded')
my_parser.add_argument('--sign-key',    type=str, required=True, help='Path to the signing key to be used while creating the certificate')
my_parser.add_argument('--kd-salt' ,    type=str, required=False, help='Path to the salt required to calculate derived key from manufacturers encryption key')
my_parser.add_argument('--out-image',   type=str, required=True, help='Output file of SBL/hsmRT images')
my_parser.add_argument('--debug',       type=str, help='Debug options for the image')

args = my_parser.parse_args()

cert_str = get_cert(args)

cert_file_name = "temp_cert"+str(randint(111, 999))

with open(cert_file_name, "w+") as f:
	f.write(cert_str)

cert_name = "cert"+str(randint(111, 999))

out_name = args.out_image

# Generate the certificate
subprocess.check_output('openssl req -new -x509 -key {} -nodes -outform DER -out {} -config {} -{}'.format(args.sign_key, cert_name, cert_file_name, g_sha_to_use), shell=True)

# Concatenate the certificate and  input binary
final_fh = open(args.out_image, 'wb+')
cert_fh = open(cert_name, 'rb')

if os.path.getsize(args.image_bin) >= g_sbl_hsm_max_size:
    except_msg = f'SBL/HSM size should be less than {g_sbl_hsm_max_size}'
    raise Exception(except_msg)

if args.sbl_enc or args.tifs_enc:
    bin_fh = open(get_enc_filename(args.image_bin), 'rb')
else:
    bin_fh = open(args.image_bin, 'rb')

shutil.copyfileobj(cert_fh, final_fh)
shutil.copyfileobj(bin_fh, final_fh)


final_fh.close()
cert_fh.close()
bin_fh.close()


# Delete the temporary files
os.remove(cert_file_name)
os.remove(cert_name)

if args.sbl_enc or args.tifs_enc:
    os.remove(get_enc_filename(args.image_bin))