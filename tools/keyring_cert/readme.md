# Keyring Cert-Gen tool {#TOOLS_KEYRING_CERT_GEN}

# Description:
   This script is to generate Keyring Certificates.

# Usage:

## Argument Options -
    - 'root_key'     : This is a mandatory parameter which is required to give
                       the key value required to sign the certificate. Key
                       file name with path required.
    - 'keys_info'    : This is a mandatory parameter which is required to give
                       the json file which contains the keyring data for certificate creation.

## Example Use -
\code
python3.10 keyring_cert_gen.py  --root_key ../boot/signing/mcu_custMpk.pem --keys_info keys.json
\endcode
