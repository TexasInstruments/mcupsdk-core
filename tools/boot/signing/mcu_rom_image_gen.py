# Script to add x509 certificate to input binary (SBL or hsmRt)
# The certificate used will be ROM format
#
# Python 3 script

import argparse
import os
import subprocess
from re import sub
from random import randint
from shutil import copyfileobj
from textwrap import dedent

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
OU                     = SITARA MCU
CN                     = Albert
emailAddress           = Albert@gt.ti.com

[ v3_ca ]
basicConstraints = CA:true
1.3.6.1.4.1.294.1.1=ASN1:SEQUENCE:boot_seq
1.3.6.1.4.1.294.1.2=ASN1:SEQUENCE:image_integrity
1.3.6.1.4.1.294.1.3=ASN1:SEQUENCE:swrv
{DBG_EXT}

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

g_dbg_seq = '''
[ debug ]
debugUID     =  FORMAT:HEX,OCT:{DBG_DEVICE}
debugType    =  INTEGER:{DBG_TYPE}
coreDbgEn    =  INTEGER:0
coreDbgSecEn =  INTEGER:0
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

    if(args.debug is not None):
        if(args.debug in g_dbg_types):
            # all good
            dbg_seq = "1.3.6.1.4.1.294.1.8 = ASN1:SEQUENCE:debug"
        else:
            dbg_seq = ""
            # Invalid debug extension, exit fail
            print("Invalid debug extension, exiting ...")
            exit(2)

    ret_cert = g_x509_template.format(
				DBG_EXT = dbg_seq,
				SHA_OID = g_sha_oids[g_sha_to_use],
				SWRV = swrev,
                BOOT_CORE_ID = bootCore_id,
                CERT_TYPE = certType,
                BOOT_CORE_OPTS = bootCoreOptions,
				BOOT_ADDR = '{:08X}'.format(int(args.loadaddr, 16)),
				IMAGE_LENGTH = os.path.getsize(args.image_bin),
				SHA_VAL = get_sha_val(args.image_bin, g_sha_to_use),
				)

    if(dbg_seq != ''):
	    ret_cert += g_dbg_seq.format(
				DBG_DEVICE='00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000',
				DBG_TYPE=g_dbg_types[args.debug],
				 )
    return dedent(ret_cert)

# MAIN
my_parser = argparse.ArgumentParser(description="Creates a ROM-boot-able images for sbl and hsmRt")

my_parser.add_argument('--image-bin',   type=str, required=True, help='Path to the SBL/hsmRT binary')
my_parser.add_argument('--core',        type=str, help='R5/HSM are the options to build for specific core')
my_parser.add_argument('--swrv',        type=str, help='Software revision number')
my_parser.add_argument('--loadaddr',    type=str, required=True, help='Load address at which SBL/hsmRT needs to be loaded')
my_parser.add_argument('--sign-key',    type=str, required=True, help='Path to the signing key to be used while creating the certificate')
my_parser.add_argument('--out-image',   type=str, required=True, help='Output file of SBL/hsmRT images')
my_parser.add_argument('--debug',       type=str, help='Debug options for the image')

args = my_parser.parse_args()

cert_str = get_cert(args)
# print(cert_str)

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
sbl_fh = open(args.image_bin, 'rb')


copyfileobj(cert_fh, final_fh)
copyfileobj(sbl_fh, final_fh)


final_fh.close()
cert_fh.close()
sbl_fh.close()


# Delete the temporary files
os.remove(cert_file_name)
os.remove(cert_name)
