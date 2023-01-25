# TRNG {#DRIVER_DTHE_TRNG}
[TOC]

## TRNG
The TRNG (True Random Number Generator) can be used for the following 
cryptographic purposes:

* Generation of cryptographic key material.
* Generation of Initialization Vectors.
* Generation of Cookies, Nonces and Seeds.

The EIP-76 TRNG provides a true, non-deterministic Noise Source coupled to an 
(optional) Deterministic Random Bit Generator for the purpose of generating keys,
Initialization Vectors (IVs), and other random number requirements.

The random numbers are accessible to the host in four 32-bit registers allowing
a 128-bit random number to be read with a single burst read. Acknowledging the
‘data ready’ (interrupt) state causes the EIP-76 to move a new value, if 
available in the data buffer, to the TRNG output register.
The EIP-76 always tries to keep the data buffer filled completely, so pulling 
out data starts the regeneration of a new number by either the DRBG or (if that
one is not available) the Conditioning Function to replenish the buffer.
The major functional blocks of the EIP-76 are:

* The actual EIP-76 core with control and test circuits, optional
Conditioning Function, optional DRBG (with or without BC_DF functionality) and 
optional data buffer control logic.

* Free Running Oscillators (FROs) instantiated outside the EIP-76 core.

\imageStyle{eip_76_trng_top_level.png,width:50%}
\image html eip_76_trng_top_level.png "Top level diagram of TRNG engine"

## TRNG core engine

The true entropy source uses Free Running Oscillators (FROs) as basic building 
block. The accumulation of timing jitter, caused (for the largest part) by shot
noise, creates uncertainty intervals for the output transitions of each FRO. 
Sampling within an uncertainty interval generates a single bit of entropy, 
which is ‘accumulated’ in a ‘toggle’ flip-flop. As the uncertainty interval is
very narrow compared to the cycle time of a FRO, the mean amount of entropy
generated per sample is very small (less than 1/100 bit per sample). To
increase the entropy generation rate, multiple FROs are used in parallel. 

\imageStyle{eip_76_trng_single_fro.png,width:50%}
\image html eip_76_trng_single_fro.png "Single Free Running Oscillator"

## Modes of operation

### Without DRBG
* start the engine and obtain random data from it without using a DRBG.

### With AES-256 DRBG
* start (‘Initialize’) the engine
* obtain data (using the ‘Generate’ operation) and
* perform a ‘Reseed’ of the engine.

\note TRNG with AES-256 DRBG re-seeding will be supported in future releases.

## API Sequence for TRNG without DRBG

This sequence starts the TRNG engine and read a 128 bit random number without using a 
DRBG to reseed the engine.

- #RNG_open(): Function to open RNG instance.
- #RNG_setup(): Function to setup the TRNG engine.
- #RNG_read(): Function to read the 128 bit random number.
- #RNG_close(): Function todDe-initialize the RNG instance.

