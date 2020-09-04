# Script to add x509 certificate to binary/ELF
# for booting application images
# The certificate format used here is different from
# the format used by ROM. Supposed to be read by SYSFW/HSM
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

# Some globals
g_sha_to_use = "sha512"

g_valid_cores = [
	"m4f_0",
	"r5_cl0_c0",
	"r5_cl0_c1",
	"r5_cl1_c0",
	"r5_cl1_c1",
	"a53_cl0_c0",
	"a53_cl0_c1",
]

g_valid_auth_types = ["0", "1", "2"]

g_core_ids = {
	"m4f_0"     : 0x18,
	"r5_cl0_c0" : 0x01,
	"r5_cl0_c1" : 0x02,
	"r5_cl1_c0" : 0x06,
	"r5_cl1_c1" : 0x07,
	"a53_cl0_c0": 0x20,
	"a53_cl0_c1": 0x21,
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
1.3.6.1.4.1.294.1.34 = ASN1:SEQUENCE:sysfw_image_integrity
1.3.6.1.4.1.294.1.35 = ASN1:SEQUENCE:sysfw_image_load
1.3.6.1.4.1.294.1.3 = ASN1:SEQUENCE:swrv
{ENCRYPTION_SEQUENCE}

[ sysfw_image_integrity ]
shaType = OID:{TEST_IMAGE_SHA_OID}
shaValue = FORMAT:HEX,OCT:{TEST_IMAGE_SHA_VAL}
imageSize = INTEGER:{TEST_IMAGE_LENGTH}

[ sysfw_image_load ]
destAddr = FORMAT:HEX,OCT:{TEST_BOOT_ADDR}
authInPlace = INTEGER:{AUTH_TYPE}

[ swrv ]
swrv = INTEGER:0
'''

g_sysfw_boot_seq = '''
[ sysfw_boot_seq ]
bootCore = INTEGER:{TEST_BOOT_CORE}
configFlagsSet = INTEGER:0
configFlagsClear = INTEGER:0
resetVec = FORMAT:HEX,OCT:{RESET_VECTOR}
rsvdFldValid = FORMAT:HEX,OCT:0000
rsvd1 = INTEGER:0
rsvd2 = INTEGER:0
rsvd3 = INTEGER:0
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
	v_TEST_BOOT_ADDR              = "00000000"
	v_AUTH_TYPE                   = '1'
	v_TEST_BOOT_CORE              = None
	v_RESET_VECTOR                = ""
	v_TEST_IMAGE_ENC_IV           = "0000"
	v_TEST_IMAGE_ENC_RS           = "0000"
	v_TEST_IMAGE_KEY_DERIVE_INDEX = 0
	v_TEST_IMAGE_KEY_DERIVE_SALT  = "0000"

	# Validation of args.

	# Bin file has to be present
	bin_file = args.bin

	if((args.bin is None) or (not os.path.exists(args.bin))):
		# No file, exit
		print("Binary file not found!")
		exit(2)
	else:
		# Check if encryption is enabled
		if(args.enc == 'y'):
			if((args.enckey is None) or (not os.path.exists(args.enckey))):
				# Error, enc key has to be given
				print("Please give the key to be used for encryption. It's either missing or file not found!")
			else:
				# Encryption key given, generate the IV and RS and add the encryption boot seq to the certificate
				enc_key = None
				with open(args.enckey, "r") as f:
					enc_key = f.read()

				# we need the value of enc_iv as hex, so convert the bytes output to hex
				enc_iv = subprocess.check_output('openssl rand 16', shell=True)
				enc_iv = binascii.hexlify(enc_iv).decode('ascii')
				v_TEST_IMAGE_ENC_IV = enc_iv

				# we don't need the value of enc_rs as hex for encryption, so keep the bytes object
				enc_rs = subprocess.check_output('openssl rand 32', shell=True)
				v_TEST_IMAGE_ENC_RS = binascii.hexlify(enc_rs).decode('ascii')

				# Pad zeros to a temporary binary to make the size multiple of 16
				zeros_pad = bytearray(16 - (os.path.getsize(bin_file) % 16))
				tempfile_name = "tmpfile" + str(randint(1111, 9999))
				encbin_name = args.bin + "-enc"
				
				shutil.copy(args.bin, tempfile_name)

				# append zeros to tempfile
				with open(tempfile_name, "ab") as f:
					f.write(zeros_pad)
					# append the enc_rs value to the padded tempfile
					f.write(enc_rs)

				# Finally generate the encrypted image
				subprocess.check_output('openssl aes-256-cbc -e -K {} -iv {} -in {} -out {} -nopad'.format(enc_key, enc_iv, tempfile_name, encbin_name), shell=True)

				# If encryption was successful, point the bin file to the encrypted image
				bin_file = encbin_name

				# Delete the temp file
				os.remove(tempfile_name)			
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

	# Auth type has to be one of 0,1,2
	if(args.authtype not in g_valid_auth_types):
		# Not a valid auth type. But don't exit, go with default
		print("[WARNING]{} is not a valid authentication type. Valid types are : {}. Using 1 by default".format(args.authtype, ','.join(g_valid_auth_types)))
		v_AUTH_TYPE = '1'
	else:
		v_AUTH_TYPE = args.authtype

	# Core has to belong to valid cores
	if(args.core is not None):
		if(args.core not in g_valid_cores):
			# Not a valid core, exit
			print("{} is not a valid core! Valid cores are : {}.".format(args.core, ','.join(g_valid_cores)))
			exit(2)
		else:
			v_TEST_BOOT_CORE = g_core_ids[args.core]
	else:
		pass

	# Replace the variables in the main template now.
	enc_seq=''
	if(args.enc == 'y'):
		enc_seq = "1.3.6.1.4.1.294.1.4 = ASN1:SEQUENCE:encryption"
	else:
		enc_seq = ''

	ret_cert = g_x509_template.format(TEST_IMAGE_SHA_OID=v_TEST_IMAGE_SHA_OID, TEST_IMAGE_SHA_VAL=v_TEST_IMAGE_SHA_VAL, TEST_IMAGE_LENGTH=v_TEST_IMAGE_LENGTH, TEST_BOOT_ADDR=v_TEST_BOOT_ADDR, AUTH_TYPE=v_AUTH_TYPE, ENCRYPTION_SEQUENCE=enc_seq)

	# If encryption is enabled, append that sequence to the current certificate
	if(args.enc == 'y'):
		ret_cert += g_enc_boot_seq.format(TEST_IMAGE_ENC_IV=v_TEST_IMAGE_ENC_IV, TEST_IMAGE_ENC_RS=v_TEST_IMAGE_ENC_RS, TEST_IMAGE_KEY_DERIVE_INDEX=v_TEST_IMAGE_KEY_DERIVE_INDEX, TEST_IMAGE_KEY_DERIVE_SALT=v_TEST_IMAGE_KEY_DERIVE_SALT)

	# NOTE: Boot sequence is not used. We assume that SBL always sets the reset vectors and does image load

	return dedent(ret_cert)


# arguments definition
my_parser = argparse.ArgumentParser(description="Generates a x509 certificate for an application binary to boot it in HS device")

my_parser.add_argument('--bin',        type=str, help='Bin file that needs to be signed')
my_parser.add_argument('--key',        type=str, help='File with signing key inside it')
my_parser.add_argument('--enckey',     type=str, help='File with encryption key inside it')
my_parser.add_argument('--cert',       type=str, help='Certificate file name (optional, will use a default name otherwise)')
my_parser.add_argument('--output',     type=str, help='Output file name (concatenated cert+bin)')
my_parser.add_argument('--core',       type=str, help='Core on which the binary is meant to be loaded. Optional')
my_parser.add_argument('--enc',        type=str, help='If the binary need to be encrypted or not [y/n]')
my_parser.add_argument('--loadaddr',   type=str, help='Target load address of the binary in hex. Default to 0x70000000')
my_parser.add_argument('--authtype',   type=str, help='Authentication type. [0/1/2]. 0 - Move to destination address specified after authentication, 1 - In place authentication, 2 - Move to the certificate start after authentication. Default is 1')

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
