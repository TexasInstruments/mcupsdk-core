# SHA {#DRIVERS_DTHE_SHA_PAGE}

### Access Protection Scheme
AES and SHA IP in DTHE support two contexts, namely Public and Secure. 

\note Dma functionality is not yet added in dthe R5f, it will be added in the future.

### SHA
- The Hash/HMAC engine performs the SHA-1, SHA-2, and MD5 hash computation. 
When loaded with a data block, and optionally an intermediate digest, it 
independently performs the hash computation (64 or 80 rounds, depending on the 
algorithm) on that data block.

- The engine can also start from the specified initial digest values instead of
 a loaded intermediate. Furthermore, it can perform the IPAD and OPAD XORs for 
MAC operations. The hash core does not perform any hash padding; this is 
performed in the host interface block, where the data input registers are 
located. A loaded data block must always be a full 64 bytes (512 bits) long.

\imageStyle{sha_module_block_diagram.png,width:50%}
\image html sha_module_block_diagram.png "sha module block diagram"

### API Sequence for SHA Algorithms

This sequence performs SHA-512 and SHA-256.

- #DTHE_open(): Function to open DTHE instance, enable DTHE engine.
- #DTHE_SHA_open(): Function to open DTHE SHA instance using the handle returned from DTHE_open().
- #DTHE_SHA_compute(): Function to execute the SHA driver with specified parameters.
- #DTHE_SHA_close(): Function to close DTHE SHA driver.
- #DTHE_close(): Function to De-initialize the DTHE instance.

### API Sequence for HMAC-SHA Algorithms

This sequence performs HMAC SHA-512 and SHA-256.

- #DTHE_open(): Function to open DTHE instance, enable DTHE engine.
- #DTHE_SHA_open(): Function to open DTHE SHA instance using the handle returned from DTHE_open().
- #DTHE_HMACSHA_compute(): The function is used to execute the HMAC SHA Operations with the specified parameters.
- #DTHE_SHA_close(): Function to close DTHE SHA driver.
- #DTHE_close(): Function to De-initialize the DTHE instance.

## API
- \ref SECURITY_DTHE_SHA_MODULE

