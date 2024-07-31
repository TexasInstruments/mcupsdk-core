# AES {#DRIVERS_DTHE_AES_PAGE}

### Access Protection Scheme
AES and SHA IP in DTHE support two contexts, namely Public and Secure.

\note Dma functionality is not yet added in dthe R5f, it will be added in the future.

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
#### AES-CTR
\imageStyle{aes_ctr_mode.png,width:80%}
\image html aes_ctr_mode.png "AES CTR mode"
- Above figure illustrates the Counter(CTR) mode of operation, where the operation
encrypts the IV. The output of the crypto core (encrypted IV) is XOR-ed with the data,
creating the output result. The IV is built out of two components, one fixed part and
a counter part. The counter part is incremented with each block. The counter width is
variable per context and can be 16, 32, 64, 96 or 128-bit wide. Note that in this mode,
encryption and decryption use the same operation.
#### AES-CFB
\imageStyle{aes_cfb_mode.png,width:80%}
\image html aes_cfb_mode.png "AES CFB mode"
- Above figure illustrates the full block (128-bit) Cipher Feedback (CFB) mode of operation
for both encryption and decryption. The input for the crypto core is the IV; the result is
XOR-ed with the data. The result is fed back via the IV register, as the next input for the
crypto core. The decrypt operation is reversed, but the crypto core is still performing an encryption.

#### AES-CMAC
\imageStyle{aes_cmac_mode.png,width:80%}
\image html aes_cmac_mode.png "AES CMAC mode"
- Above figure illustrates the One Key Mac mode of operation, where the input to the
crypto core is XOR-ed with the IV. The crypto core output is then fed back as IV for
the next block. The last data input block is XOR-ed with an additional input value stored
in the Temporary Buffer; this can be any precalculated value and is dependent on the
alignment of the last input block. For CMAC mode, the tweak value (in temporary buffer)
is calculated via K1 and K2 which is dervied from Key value from software.


### API Sequence for ECB, CBC, CTR, CMAC Algorithms (Single Shot Mechanism)

This sequence performs Encryption and decryption operations for AES-ECB/CBC/CTR algorithms. Supported key lengths are 128, 192, 256 bit.

- #DTHE_open(): Function to open DTHE instance, enable DTHE engine.
- #DTHE_AES_open(): Function to open DTHE AES instance using the handle returned from DTHE_open().
- #DTHE_AES_execute(): Function to execute the AES driver with specified parameters.
- #DTHE_AES_close(): Function to close DTHE AES driver.
- #DTHE_close(): Function to De-initialize the DTHE instance.

### API Sequence for ECB, CBC, CTR, CMAC Algorithms (Multi Shot Mechanism)

This sequence performs Encryption and decryption operations for AES-ECB/CBC/CTR algorithms. Supported key lengths are 128, 192, 256 bit.

- #DTHE_open(): Function to open DTHE instance, enable DTHE engine.
- #DTHE_AES_open(): Function to open DTHE AES instance using the handle returned from DTHE_open().
- #DTHE_AES_execute(): Function to execute the AES driver with specified parameters and streamstate as <b>init</b>.
- #DTHE_AES_execute(): Function to execute the AES driver with specified parameters and streamstate as <b>update</b>.
- #DTHE_AES_execute(): Function to execute the AES driver with specified parameters and streamstate as <b>finish</b>.
- #DTHE_AES_close(): Function to close DTHE AES driver.
- #DTHE_close(): Function to De-initialize the DTHE instance.


In order to cancel an existing stream, <i>DTHE_AES_close</i> should be called followed by <i>DTHE_AES_open</i> like this. This will discard the
current stream data -
- #DTHE_open(): Function to open DTHE instance, enable DTHE engine.
- #DTHE_AES_open(): Function to open DTHE AES instance using the handle returned from DTHE_open().
- #DTHE_AES_execute(): Function to execute the AES driver with specified parameters and streamstate as <b>init</b>.
- #DTHE_AES_execute(): Function to execute the AES driver with specified parameters and streamstate as <b>update</b>.
- #DTHE_AES_close(): Function to close DTHE AES driver.
- #DTHE_AES_open(): Function to open DTHE AES instance using the handle returned from DTHE_open().
- #DTHE_AES_execute(): Function to execute the AES driver with specified parameters and streamstate as <b>init</b>.
- #DTHE_AES_execute(): Function to execute the AES driver with specified parameters and streamstate as <b>update</b>.
- #DTHE_AES_execute(): Function to execute the AES driver with specified parameters and streamstate as <b>finish</b>.
- #DTHE_AES_close(): Function to close DTHE AES driver.
- #DTHE_close(): Function to De-initialize the DTHE instance.

## API
- \ref SECURITY_DTHE_AES_MODULE

