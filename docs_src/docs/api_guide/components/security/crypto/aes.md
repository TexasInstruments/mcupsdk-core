# AES {#DRIVERS_DTHE_AES_PAGE}

### Access Protection Scheme
AES and SHA IP in DTHE support two contexts, namely Public and Secure. 

### AES
- The AES IP is an efficient implementation of the Rijndael cipher (the AES 
algorithm) and a 128-bit polynomial multiplication. Rijndael is a block cipher 
with each data block consisting of 128-bits.
- AES encryption requires a specific number of rounds depending on the key length.
Supported key lengths are 128-bit, 192-bit, and 256 bit, requiring 10, 12, and 14 
rounds respectively; or 32, 38, and 44 clock cycles respectively because (# of 
clock cycles) = 2 + 3 x (# of rounds). The larger key lengths provide greater 
encryption strength at the expense of additional rounds and therefore reduced 
throughput.
\imageStyle{aes_functional_diagram.png,width:50%}
\image html aes_functional_diagram.png "AES functional diagram"
#### AES-ECB
\imageStyle{ecb_feedback_mode.png,width:80%}
\image html ecb_feedback_mode.png "AES ECB feedback mode"
- Above figure illustrates the basic Electronic Code Book (ECB) feedback mode 
of operation, where the input data is passed directly to the basic crypto core 
and the output of the crypto core is passed directly to the output buffer. For 
decryption the crypto core operates in reverse, thus, the decrypt datapath is 
used for the data processing, where encryption uses the encrypt datapath.
#### AES-CBC
\imageStyle{cbc_feedback_mode.png,width:80%}
\image html cbc_feedback_mode.png "AES CBC feedback mode"
- Above figure illustrates the Cipher Block Chaining (CBC) feedback mode of 
operation, where the input data is XOR-ed with the initialization vector (IV) 
before it is passed to the basic crypto core. The output of the crypto core 
passes directly to the output buffer and becomes the next IV. For decryption, 
the operation is reversed, resulting in an XOR at the output of the crypto core. 
The input cipher text of the current operation is the IV for the next operation.

### API Sequence for CBC and ECB Algorithms

This sequence performs Encryption and decryption operations for AES-CBC/ECB algorithms. Supported key lengths are 128 and 256 bit.

- #DTHE_open(): Function to open DTHE instance, enable DTHE engine.
- #DTHE_AES_open(): Function to open DTHE AES instance using the handle returned from DTHE_open().
- #DTHE_AES_execute(): Function to execute the AES driver with specified parameters.
- #DTHE_AES_close(): Function to close DTHE AES driver.
- #DTHE_close(): Function to De-initialize the DTHE instance.

### API Sequence for AES-CMAC Algorithms

This sequence performs AES CMAC-128 and CMAC-256.

- #DTHE_open(): Function to open DTHE instance, enable DTHE engine.
- app_aes_cmac128(): Function (defined in example) to calculate CMAC for input 
buffer. This is not supported directly through Hardware and hence a software 
implementation is available in the examples. However, for AES computation we 
use the DTHE
- #DTHE_close(): Function to De-initialize the DTHE instance.

## API
- \ref SECURITY_DTHE_AES_MODULE

