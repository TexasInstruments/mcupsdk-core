# Script to add x509 certificate for public key certificate creation
# The certificate used will be ROM format
#
# Python 3.10 script

import argparse
import os
import subprocess
import binascii
from re import sub
from random import randint
import shutil
import json
import sys
from textwrap import dedent
sys.path.insert(0, '../bin2c/')
import bin2c

g_sha_to_use = "sha512"
max_keyring_size = 8192

# Dictionaries used to store information about various extensions

output_info: dict[str, str] = {}
hash_algo: dict[str, int] = {
    "SHA384": 2,
    "SHA512": 4,
    "SHA256": 6
}

###########################################################################################################

output_info['hash_csv'] = "verify_hash.csv"

g_sha_oids = {
    "sha256": "2.16.840.1.101.3.4.2.1",
    "sha384": "2.16.840.1.101.3.4.2.2",
    "sha512": "2.16.840.1.101.3.4.2.3",
    "sha224": "2.16.840.1.101.3.4.2.4",
}

g_x509_template = '''
[ req ]
distinguished_name         = req_distinguished_name
x509_extensions            = v3_ca
prompt                             = no

dirstring_type = nobmp

[ req_distinguished_name ]
C                                          = US
ST                                         = SC
L                                          = Dallas
O                                          = Texas Instruments., Inc.
OU                                         = PBU
CN                                         = Albert
emailAddress               = Albert@ti.com

[ v3_ca ]
basicConstraints = CA:true
1.3.6.1.4.1.294.1.1=ASN1:SEQUENCE:boot_seq
1.3.6.1.4.1.294.1.9=ASN1:SEQUENCE:keyring_ext
{KEYRING_ASYMM}
{KEYRING_SYMM}

[ boot_seq ]
certType         =      INTEGER:{CERT_TYPE}
bootCore         =      INTEGER:0
bootCoreOpts     =      INTEGER:0
destAddr         =      FORMAT:HEX,OCT:00000000
imageSize        =      INTEGER:0

[ keyring_ext ]
keyring_sw_rev=INTEGER:{KEYRING_SW_REV}
keyring_ver=INTEGER:{KEYRING_VER}
num_of_asymm_keys=INTEGER:{NUM_ASYMM}
num_of_symm_keys=INTEGER:{NUM_SYMM}
'''

keyring_ext_asymm_seq = '''
[ keyring_asymm ]
'''

keyring_ext_symm_seq = '''
[ keyring_symm ]
'''


def pub_pem_to_pub_der(input: str, output: str) -> None:
    """Converts a public key from PEM format to DER format.

    Args:
        input (str): The path to the public key file in PEM format.
        output (str): The path to the output file in DER format.
    """

    os.system(
        f"openssl pkey -in \"{input}\" -pubin -outform der -out \"{output}\"")


def calc_hash(input: str, output: str, hash_algo: str) -> None:
    """Calculates hash of public key file in DER format.

    Args:
        input (str) : The path to the public key file in DER format.
        output (str): The path to the output file containing public key hash.
        hash_algo (str): Algorithm used for hash calculation.
    """
    if(hash_algo == "SHA256"):
        os.system(f"openssl dgst -sha256 -binary \"{input}\" > \"{output}\"")
    elif(hash_algo == "SHA512"):
        os.system(f"openssl dgst -sha512 -binary \"{input}\" > \"{output}\"")
    elif(hash_algo == "SHA384"):
        os.system(f"openssl dgst -sha384 -binary \"{input}\" > \"{output}\"")
    else:
        print("Invalid hash algorithm")


def utils_hex_from_file(filename: str, print_to_stdout=False, out_file=None, width=32) -> str:
    """Converts the binary data in a file to hexadecimal format.

Args:
    filename (str): The name of the file to be read.
    print_to_stdout (bool): If True, prints the hex data to stdout.
    out_file (str): The name of the file to write the hex data to.
    width (int): The number of bytes to be printed per line.

Returns:
        str: The hex data as a string.
    """
    if not (os.path.exists(filename) and os.path.isfile(filename)):
        print(f"{filename} does not exist")
        return "ERROR READING FILE"

    hexdata = ""

    with open(filename, 'rb') as f:
        hexdata = f.read().hex()

    line_list: list[str] = []
    if out_file or print_to_stdout:
        for i in range(0, len(hexdata), width):
            line_list.append(hexdata[i: i+width])

    if out_file:
        with open(str(out_file), 'w') as f:
            f.write('\n'.join(line_list))
            f.write('\n')

    if print_to_stdout:
        print('\n'.join(line_list))

    return hexdata


def populate_keyring_ext_symm(keys_data: dict) -> None:
    """Populates symmetric keyring extension.

Args:
    keys_data (dict): Dict with keyring data

Returns:
    None
"""
    global keyring_ext_symm_seq

    temp_string = "asymm_key1=SEQUENCE:comp1"
    symm_keys = ''''''

    temp_comp = '''
[ comp1 ]
keyId=INTEGER:key_id_val
key_rights=FORMAT:HEX,OCT:key_rights_val
public_key=FORMAT:HEX,OCT:public_key_val
'''

    for iter in range(keys_data["num_of_symm_keys"]):
        keyring_ext_symm_seq += (temp_string.replace("1", str(iter)) + "\n")

        symm_keys += temp_comp.replace('1', str(iter))\
            .replace('key_id_val', str(iter))\
            .replace('key_rights_val', keys_data['keyring_symm'][iter]['key_rights'])\
            .replace('public_key_val', keys_data['keyring_symm'][iter]['pub_key'])\

    keyring_ext_symm_seq += symm_keys
    print(keyring_ext_symm_seq)


def populate_keyring_ext_asymm(keys_data: dict) -> None:
    """Populates Asymmetric keyring extension.

    Args:
    keys_data (dict): Dict with keyring data

Returns:
    None
"""

    global keyring_ext_asymm_seq

    temp_string = "asymm_key1=SEQUENCE:comp1"
    asymm_keys = ''''''
    index_start_asymm = 32

    temp_comp = '''
[ comp1 ]
keyId=INTEGER:key_id_val
key_rights=FORMAT:HEX,OCT:key_rights_val
hash_algo=INTEGER:hash_algo_val
public_key=FORMAT:HEX,OCT:public_key_val
'''

    for iter in range(keys_data["num_of_asymm_keys"]):
        keyring_ext_asymm_seq += (temp_string.replace("1", str(iter)) + "\n")
        pub_pem_to_pub_der(keys_data['keyring_asymm'][iter]['pub_key'], os.path.join(
            'tmpdir', 'pub_key.der'))

        # calc_hash <INPUT> <OUTPUT>
        calc_hash(os.path.join('tmpdir', 'pub_key.der'),
                  os.path.join('tmpdir', 'pub_key_hash'),
                  keys_data['keyring_asymm'][iter]['hash_algo'])

        asymm_keys += temp_comp.replace('1', str(iter))\
            .replace('key_id_val', str(iter + index_start_asymm))\
            .replace('key_rights_val', keys_data['keyring_asymm'][iter]['key_rights'])\
            .replace('public_key_val', utils_hex_from_file(os.path.join('tmpdir', 'pub_key_hash')))\
            .replace('hash_algo_val', str(hash_algo[keys_data['keyring_asymm'][iter]['hash_algo']]))

    keyring_ext_asymm_seq += asymm_keys
    print(keyring_ext_asymm_seq)


def get_cert(args) -> None:
    """
    This function reads keys from a JSON file and generates a certificate
    based on the keyring information.

    Args:
    args (argparse.Namespace)
    """
    # read keys from keys.json file
    with open(args.keys_info, 'r') as keys:
        keys_data = json.load(keys, strict=False)

    # subtract keyring info structure size  from the total available memory
    available_keyring_size = max_keyring_size - 16

    # this is cert type unique to keyring certificate, this is required for TIFS to verify this certificate against the active root keys
    device_cert_type = 0xA5000002

    # populate keyring_version from the dict
    if(keys_data["keyring_ver"] is None):
        # Default to 1
        keyring_ver = 1
    else:
        keyring_ver = keys_data["keyring_ver"]

    print('keyring version = ' + str(keyring_ver))
    print('keyring software revision = ' + str(keys_data["keyring_sw_rev"]))
    print('Number of asymmetric keys = ' + str(keys_data["num_of_asymm_keys"]))
    print('Number of symmetric keys = ' + str(keys_data["num_of_symm_keys"]))

    # populate number of asymmetric keys from the dict
    if(keys_data["num_of_asymm_keys"] == 0):
        # Default to 0
        num_of_asymm_keys = 0
        keyring_asymm = ""
    else:
        num_of_asymm_keys = keys_data["num_of_asymm_keys"]
        keyring_asymm = "1.3.6.1.4.1.294.1.10=ASN1:SEQUENCE:keyring_asymm"
        available_keyring_size -= (num_of_asymm_keys*(76))
        populate_keyring_ext_asymm(keys_data)
    if(keys_data["num_of_symm_keys"] == 0):
        # Default to 0
        num_of_symm_keys = 0
        keyring_symm = ""
    else:
        num_of_symm_keys = keys_data["num_of_symm_keys"]
        keyring_symm = "1.3.6.1.4.1.294.1.11=ASN1:SEQUENCE:keyring_symm"
        populate_keyring_ext_symm(keys_data)

    if(available_keyring_size < 0):
        print("The size of the keyring exceeds the memory availability of HSM secure ram.")
        exit(1)

    ret_cert = g_x509_template.format(
        CERT_TYPE=device_cert_type,
        KEYRING_SW_REV=keys_data["keyring_sw_rev"],
        KEYRING_VER=keyring_ver,
        KEYRING_ASYMM=keyring_asymm,
        NUM_ASYMM=num_of_asymm_keys,
        KEYRING_SYMM=keyring_symm,
        NUM_SYMM=num_of_symm_keys
    )
    if(keys_data["num_of_asymm_keys"] > 0):
        ret_cert += keyring_ext_asymm_seq
    if(keys_data["num_of_symm_keys"] > 0):
        ret_cert += keyring_ext_symm_seq
    return dedent(ret_cert)


if __name__ == "__main__":
    python_exe = 'python3'

    if os.name == 'nt':
        python_exe = 'python'

    BIN2C = f"{python_exe} {os.path.join('..', '..', '..', '..', '..', '..', '..', 'tools', 'bin2c', 'bin2c.py')}"
    my_parser = argparse.ArgumentParser(
        description="Creates a Public Key Certificate for (non-K3) HS-SE devices")

    my_parser.add_argument('--root_key',                type=str,
                           required=True, help='Customer MPK key')
    my_parser.add_argument('--keys_info',               type=str,
                           required=True, help='Keys info json file')
    my_parser.add_argument('--rsassa_pss',
                           help='If binary needs to be signed RSASSA PSS scheme or not',  action="store_true")
    my_parser.add_argument('--pss_saltlen', type=int,   default=0,
                           help='Salt length for RSASSA PSS scheme',)

    args = my_parser.parse_args()

    # create temp directory for temp files
    try:
        os.mkdir('tmpdir')
    except:
        None

    cert_str = get_cert(args)
    # print(cert_str)

    cert_file_name = "temp_cert"+str(randint(111, 999))

    with open(cert_file_name, "w+") as f:
        f.write(cert_str)

    cert_name = "x509_keyringcert_"+str(randint(111, 999))+".cert"

    # Generate the certificate
    if args.rsassa_pss:
        subprocess.check_output('openssl req -new -x509 -key {} -nodes -outform DER -out {} -config {} -{} -sigopt rsa_padding_mode:pss -sigopt rsa_pss_saltlen:{} '.format(
            args.root_key, cert_name, cert_file_name, g_sha_to_use, args.pss_saltlen), shell=True)
    else:
        subprocess.check_output('openssl req -new -x509 -key {} -nodes -outform DER -out {} -config {} -{}'.format(
            args.root_key, cert_name, cert_file_name, g_sha_to_use), shell=True)

    bin2c.binary_to_header(cert_name, 'keyringCert.h', 'CUST_KEYRINGCERT')

    # Delete the temporary files
    # remove temp directory
    try:
        shutil.rmtree('tmpdir')
    except:
        None
    os.remove(cert_file_name)
    os.remove(cert_name)
