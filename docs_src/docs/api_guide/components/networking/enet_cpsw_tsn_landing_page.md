# Time-Sensitive Networking (TSN) {#ENET_CPSW_TSN}

[TOC]

# Overview

TI's TSN stack provides a comprehensive IEEE 802.1 Time-Sensitive Networking implementation for Sitara MCU+ SoC families.
The stack is co-developed with Excelfore Corporation and runs on FreeRTOS, integrated with the Enet LLD over the CPSW peripheral.

The TSN Stack provides hardware configuration abstractions for some features 802.1 TSN features like 802.1av, 802.3br, 802.1Qbv etc. 
The stack is organized into four open-source core libraries plus optional evaluation libraries for AVB (AVTP):

\cond SOC_AM62LX || SOC_AM62PX || SOC_AM62X || SOC_AM243X || SOC_AM64X
| Library        | Role                                                                 | Source Path                                                 |
|----------------|----------------------------------------------------------------------|-------------------------------------------------------------|
| tsn_unibase    | Platform-independent utility functions (logging, memory, lists)      | `source/networking/tsn/tsn-stack/tsn_unibase/`             |
| tsn_combase    | Communication primitives (sockets, threads, timers, IPC)             | `source/networking/tsn/tsn-stack/tsn_combase/`             |
| tsn_gptp       | IEEE 802.1AS-2020 gPTP protocol state machines and clock servo        | `source/networking/tsn/tsn-stack/tsn_gptp/`                |
| tsn_uniconf    | Universal YANG configuration daemon with SimpleDB                    | `source/networking/tsn/tsn-stack/tsn_uniconf/`             |
| yangemb        | YANG database licensing library (required for all TSN applications)  | `source/networking/tsn/tsn-stack/license_lib/`             |
\endcond

\cond SOC_AM62DX || SOC_AM275X
An additional evaluation library provides AVB (AVTP) support:

| Library        | Role                                                                 | Source Path                                                 |
|----------------|----------------------------------------------------------------------|-------------------------------------------------------------|
| tsn_unibase    | Platform-independent utility functions (logging, memory, lists)      | `source/networking/tsn/tsn-stack/tsn_unibase/`             |
| tsn_combase    | Communication primitives (sockets, threads, timers, IPC)             | `source/networking/tsn/tsn-stack/tsn_combase/`             |
| tsn_gptp       | IEEE 802.1AS-2020 gPTP protocol state machines and clock servo        | `source/networking/tsn/tsn-stack/tsn_gptp/`                |
| tsn_uniconf    | Universal YANG configuration daemon with SimpleDB                    | `source/networking/tsn/tsn-stack/tsn_uniconf/`             |
| tsn_l2 / tsn_conl2 *(eval)* | IEEE 1722-2016 AVTP (AAF PCM, AES3, CRF)            | `source/networking/tsn/tsn-stack/eval_lib/`                |
| yangemb        | YANG database licensing library (required for all TSN applications)  | `source/networking/tsn/tsn-stack/license_lib/`             |
\endcond

---

# Features and IEEE Standard Coverage {#ENET_CPSW_TSN_FEATURES}

## Feature/Standard Supported

The following TSN Features of CPSW are supported to be configurable by the TSN Stack. 

\cond SOC_AM62LX || SOC_AM62PX || SOC_AM62X || SOC_AM243X || SOC_AM64X

| Feature                                    | IEEE Standard     | Notes                                                           |
|--------------------------------------------|-------------------|-----------------------------------------------------------------|
| Credit-Based Shaper (CBS / FQTSS)          | 802.1Qav          | Hardware per-class CBS shaper in CPSW; reserves per-stream bandwidth using idle slope; YANG model configuration available |
| Interspersing Express Traffic (IET)        | IEEE 802.3br      | CPSW frame preemption separating express and preemptable traffic queues (802.3br / 802.1Qbu); YANG model configuration available |
| Enhancements for Scheduled Traffic (EST)   | 802.1Qbv          | Time-aware gate control list (GCL) via CPSW TAS hardware; per-traffic-class gate scheduling with nanosecond precision; YANG model configuration available |
| Time Synchronization (gPTP)                | 802.1AS-2020      | Full end-to-end IEEE 802.1AS-2020 stack; grandmaster, slave, and Bridge/Relay Instances; tested against Intel and TI endpoints; YANG data model configuration available |
\endcond

\cond SOC_AM62DX || SOC_AM275X
| Feature                                    | IEEE Standard     | Notes                                                           |
|--------------------------------------------|-------------------|-----------------------------------------------------------------|
| Credit-Based Shaper (CBS / FQTSS)          | 802.1Qav          | Hardware per-class CBS shaper in CPSW; reserves per-stream bandwidth using idle slope; YANG model configuration available |
| Interspersing Express Traffic (IET)        | IEEE 802.3br      | CPSW frame preemption separating express and preemptable traffic queues (802.3br / 802.1Qbu); YANG model configuration available |
| Enhancements for Scheduled Traffic (EST)   | 802.1Qbv          | Time-aware gate control list (GCL) via CPSW TAS hardware; per-traffic-class gate scheduling with nanosecond precision; YANG model configuration available |
| Time Synchronization (gPTP)                | 802.1AS-2020      | Full end-to-end IEEE 802.1AS-2020 stack; grandmaster, slave, and Bridge/Relay Instances; tested against Intel and TI endpoints; YANG data model configuration available |
| Audio Video Bridging (AVB)                 | IEEE 1722-2016    | IEEE 1722-2016 AVTP transport layer abstraction; AAF PCM, AES3, and CRF subtypes; talker and listener roles; evaluation library (Excelfore) |
\endcond

### Supported Features - gPTP

- IEEE 802.1AS-2020 grandmaster (Time Transmitter), slave (Time Receiver), and bridge/Relay Instance (Boundary clock mode)
- Peer-to-peer (P2P) path delay measurement via PDelay_Req / PDelay_Resp / PDelay_Resp_Follow_Up exchange
- Best Master Clock Algorithm (BMCA) with full priority vector comparison
- Announce, Sync, Follow_Up, and Signaling message exchange
- gPTP Capable advertisement and negotiation via organization-specific signaling TLVs
- Multiple clock domain support (CMLDS) — up to 2 simultaneous domains (`GPTP_MAX_DOMAINS`)
- Hardware timestamping integration for Sync and PDelay messages
- IIR clock servo with configurable phase and frequency filter coefficients (`PHASE_OFFSET_IIR_ALPHA`, `FREQ_OFFSET_IIR_ALPHA`)
- Quick sync mode: reduced time-to-lock via shortened compute interval, increased frequency update rate, and hardware phase adjustment
- AVNU compliance mode (configurable)
- Static port state configuration (bypass BMCA for fixed topologies)

### Not Supported Features - gPTP

- End-to-end (E2E) delay mechanism — P2P only
- No Transparent Clock support
- More than 2 simultaneous gPTP clock domains (`GPTP_MAX_DOMAINS` ceiling = 2)
- UDP/IPv4 transport — Ethernet L2 only

\cond SOC_AM62DX || SOC_AM275X
### Supported Features - AVB

- IEEE 1722-2016 AVTP subtypes: AAF (PCM and AES3 audio), CRF (Clock Reference Format), CVF (Compressed Video), ACF, TSCF, NTSCF
- AAF audio sample rates: 8 kHz – 192 kHz; formats: Int16, Int24, Int32, Float32, AES3
- Talker and Listener roles; configurable stream counts (`AVB_TALKER_STREAMS_NUMBER`, `AVB_LISTENER_STREAMS_NUMBER`)
- CBS (Credit-Based Shaper) integration for per-stream bandwidth reservation
- CRF media clock recovery with pull/multiplier support; DP83TG721 and CDCE1214 PHY support
- VLAN ID and PCP (Priority Code Point) per-stream configuration
- ACF subtypes: CAN, FlexRay, LIN, Sensor, Serial, Parallel, and user-defined

### Not Supported Features - AVB

- AVDECC (IEEE 1722.1) — device discovery, enumeration, and stream connection management
- SVF (SDI Video Format), RVF (Raw Video Format), MMA (MIDI), AEF (AES-encrypted), and VSF (Vendor Specific) subtypes
- MSRP / IEEE 802.1Qat stream reservation protocol (manual CBS configuration required)
- AVTP over UDP/IP transport — Ethernet L2 only
- Receiver timeout callback in direct RX mode
\endcond

---

# Architecture

## Software Block Diagram

\imageStyle{tsn_architecture.png,width:40%}
\image html tsn_architecture.png "TSN Stack Software Block Diagram"


# Resource Utilization

The table below depicts the typical resource utilization of TSN Features in a typical TSN included application.
\cond SOC_AM62DX || SOC_AM275X
| Resource   | Count | Usage                                                                                 |
|------------|-------|---------------------------------------------------------------------------------------|
| TX channel | 2     | gPTP (1), AVTP Tx (1)                                                                 |
| RX flow    | 2     | gPTP (1), AVTP Rx (1)                                                                 |
| Tasks      | 3     | uniconf_task (1), gptp2d_task (1), avtp_rx_task (1) |
\endcond


\cond SOC_AM62LX || SOC_AM62PX || SOC_AM62X || SOC_AM243X || SOC_AM64X
| Resource   | Count | Usage                                                                                 |
|------------|-------|---------------------------------------------------------------------------------------|
| TX channel | 1     | gPTP (1)                                                                 |
| RX flow    | 1     | gPTP (1)                                                                 |
| Tasks      | 2     | uniconf_task (1), gptp2d_task (1) |
\endcond

\note Default configuration: each use case uses one dedicated CPSW Tx DMA channel and one dedicated Rx Flow per port. Debug builds use 16 KB stack per task (`TSN_TSK_STACK_SIZE = 16 KB`).

# Examples and Feature Coverage {#ENET_CPSW_TSN_EXAMPLES}

\cond SOC_AM62LX || SOC_AM62PX 

| Example | Feature Demonstrated | Standards Exercised |
|---------|----------------------|---------------------|
| \ref EXAMPLES_ENET_CPSW_TSN_GPTP_TR "gPTP Time Receiver" | gPTP slave mode, clock lock to grandmaster | 802.1AS-2020 |
| \ref EXAMPLES_ENET_CPSW_TSN_GPTP_TT "gPTP Time Transmitter" | gPTP grandmaster mode | 802.1AS-2020 |
| \ref EXAMPLES_ENET_CPSW_TSN_GPTP_BRIDGE "gPTP Bridge" | gPTP Relay Instance (Boundary clock / bridge) | 802.1AS-2020 |

\endcond

\cond SOC_AM62X || SOC_AM243X || SOC_AM64X

| Example | Feature Demonstrated | Standards Exercised |
|---------|----------------------|---------------------|
| \ref EXAMPLES_ENET_CPSW_TSN_GPTP_TR "gPTP Time Receiver" | gPTP slave mode, clock lock to grandmaster | 802.1AS-2020 |
| \ref EXAMPLES_ENET_CPSW_TSN_GPTP_TT "gPTP Time Transmitter" | gPTP grandmaster mode | 802.1AS-2020 |
| \ref EXAMPLES_ENET_CPSW_TSN_GPTP_BRIDGE "gPTP Bridge" | gPTP Relay Instance (Boundary clock / bridge) | 802.1AS-2020 |
| \ref EXAMPLES_ENET_CPSW_EST "TSN EST" | Scheduled Traffic gate control list (GCL) | 802.1Qbv |

\endcond

\cond SOC_AM275X

| Example | Feature Demonstrated | Standards Exercised |
|---------|----------------------|---------------------|
| \ref EXAMPLES_ENET_CPSW_TSN_GPTP_TR "gPTP Time Receiver" | gPTP slave mode, clock lock to grandmaster | 802.1AS-2020 |
| \ref EXAMPLES_ENET_CPSW_TSN_GPTP_TT "gPTP Time Transmitter" | gPTP grandmaster mode | 802.1AS-2020 |
| \ref EXAMPLES_ENET_CPSW_TSN_GPTP_BRIDGE "gPTP Bridge" | gPTP Relay Instance (Boundary clock / bridge) | 802.1AS-2020 |
| \ref EXAMPLES_ENET_CPSW_EST "TSN EST" | Scheduled Traffic gate control list (GCL) | 802.1Qbv | 
| \ref EXAMPLES_ENET_CPSW_AVTP "AVTP AAF PCM" | AVTP AAF PCM Talker + Listener, CBS shaper | IEEE 1722-2016, 802.1Qav |
| \ref EXAMPLES_ENET_CPSW_TSN_AES3_AAF_APP "AVTP AES3 AAF" | AVTP AES3 (Dolby EC3) format listener | IEEE 1722-2016 |
| \ref EXAMPLES_ENET_AVB_MULTISTREAM_MCR "Multistream MCR" | Multistream AAF PCM + CRF media clock recovery | IEEE 1722-2016 |
| \ref EXAMPLES_ENET_CPSW_TSN_CRF_MASTER "CRF Master" | CRF master clock reference stream | IEEE 1722-2016 |
| \ref EXAMPLES_ENET_CPSW_TSN_CRF_AVTP "CRF + AVTP" | CRF combined with AVTP | IEEE 1722-2016 |
| \ref EXAMPLES_ENET_CPSW_TSN_CRF_AUTOAMP_DEMO "AutoAmp Demo" | Multistream AAF PCM + CRF + TCP control server | IEEE 1722-2016, 802.1Qav |
| \ref EXAMPLES_ENET_CPSW_AVB_AUDIO_DEMO "AVB Audio Playback" | End-to-end AVB audio playback | 802.1AS-2020, 802.1Qav, 1722 |
| \ref EXAMPLES_ENET_AVB_AUDIO_ETHERRING_DEMO "AVB Ethernet Ring" | Zonal audio playback with Ethernet ring topology | 802.1AS-2020, 802.1Qav, 1722 |

\endcond

\cond SOC_AM62DX

| Example | Feature Demonstrated | Standards Exercised |
|---------|----------------------|---------------------|
| \ref EXAMPLES_ENET_CPSW_TSN_GPTP_TR "gPTP Time Receiver" | gPTP slave mode, clock lock to grandmaster | 802.1AS-2020 |
| \ref EXAMPLES_ENET_CPSW_TSN_GPTP_TT "gPTP Time Transmitter" | gPTP grandmaster mode | 802.1AS-2020 |
| \ref EXAMPLES_ENET_CPSW_TSN_GPTP_BRIDGE "gPTP Bridge" | gPTP Relay Instance (Boundary clock / bridge) | 802.1AS-2020 |
| \ref EXAMPLES_ENET_CPSW_EST "TSN EST" | Scheduled Traffic gate control list (GCL) | 802.1Qbv |
| \ref EXAMPLES_ENET_CPSW_AVTP "AVTP AAF PCM" | AVTP AAF PCM Talker + Listener, CBS shaper | IEEE 1722-2016, 802.1Qav |
| \ref EXAMPLES_ENET_CPSW_TSN_AES3_AAF_APP "AVTP AES3 AAF" | AVTP AES3 (Dolby EC3) format listener | IEEE 1722-2016 |
| \ref EXAMPLES_ENET_CPSW_TSN_CRF_MASTER "CRF Master" | CRF master clock reference stream | IEEE 1722-2016 |
| \ref EXAMPLES_ENET_CPSW_TSN_CRF_AVTP "CRF + AVTP" | CRF combined with AVTP | IEEE 1722-2016 |
| \ref EXAMPLES_ENET_CPSW_TSN_CRF_AUTOAMP_DEMO "AutoAmp Demo" | Multistream AAF PCM + CRF + TCP control server | IEEE 1722-2016, 802.1Qav |

\endcond

---

# Interoperability Tests {#ENET_CPSW_TSN_INTEROP}

## gPTP Interoperability

The following third-party devices/Stacks have been used for interoperability verification of the IEEE 802.1AS-2020 gPTP implementation:

| Peer Hardware                    | Peer Software                    | Roles Tested                                              |
|----------------------------------|----------------------------------|-----------------------------------------------------------|
| Intel I210 NIC                   | Linux `ptp4l` (linuxptp)         | Grandmaster, slave, Bridge/Relay                          |
| Intel I350 NIC                   | Linux `ptp4l` (linuxptp)         | Grandmaster, slave, Bridge/Relay                          |
| TI EVM                           | TI gPTP stack (TSN SDK)          | Peer-to-peer slave / master; daisy-chain with multiple nodes     |
| TI EVM with DP83TG721 PHY        | TI gPTP stack (TSN SDK)          | Grandmaster, slave                                        |

---

# Memory Footprint {#ENET_CPSW_TSN_MEMORY}

All measurements are on Sitara R5F, release build. Columns: Code = `.text`, Read Only = `.rodata` / `.const`, Read Write = `.bss` + `.data` (RAM).

## Configuration 1: gPTP Only

\note Measured using \ref EXAMPLES_ENET_CPSW_TSN_GPTP_TR "gptp_cpsw_app" (AM275x R5F, release, `-flto` disabled), It is expected to reduce further by 10-20% by enabling -`-flto` flag.

| Library      | Code (KB) | Read Only (KB) | Read Write (KB) | Total (KB) |
|--------------|-----------|----------------|-----------------|------------|
| tsn_unibase  | 4         | 1              | 7               | 12         |
| tsn_combase  | 14        | 4              | 18              | 36         |
| tsn_uniconf  | 34        | 12             | 67              | 113        |
| tsn_gptp     | 61        | 12             | 25              | 97         |
| yangemb      | 3         | 1              | 0               | 3          |
| **Total**    | **117**   | **29**         | **116**         | **262**    |

\cond SOC_AM62DX || SOC_AM275X
## Configuration 2: gPTP + AVB

\note Measured using \ref EXAMPLES_ENET_CPSW_AVTP "aafpcmtalker_app" (AM275x R5F, release, `-flto` disabled), It is expected to reduce further by 10-20% by enabling -`-flto` flag.
\note The number of talker and listener streams is application-level configuration (`AVB_TALKER_STREAMS_NUMBER`,
`AVB_LISTENER_STREAMS_NUMBER` in `avtp_buildconf.h`) and does not affect the library footprint.

| Library           | Code (KB) | Read Only (KB) | Read Write (KB) | Total (KB) |
|-------------------|-----------|----------------|-----------------|------------|
| tsn_unibase       | 5         | 1              | 7               | 13         |
| tsn_combase       | 15        | 4              | 18              | 37         |
| tsn_uniconf       | 35        | 13             | 67              | 114        |
| tsn_gptp          | 63        | 13             | 25              | 100        |
| tsn_l2 / tsn_conl2| 27        | 6              | 9               | 42         |
| yangemb           | 3         | 1              | 0               | 3          |
| **Total**         | **147**   | **38**         | **124**         | **309**    |
\endcond

---

# Performance Metrics {#ENET_CPSW_TSN_PERFORMANCE}

\cond SOC_AM275X
## AVB End-to-End Audio Latency (AM275x EVM)

Measured using \ref EXAMPLES_ENET_CPSW_AVB_AUDIO_DEMO.

**Test Setup:**
- SoC: AM275x, Cortex-R5F0 Core 0 @ 1000 MHz, MSRAM cached
- Link: RGMII @ 1 Gbps
- AVTP subtype: AAF PCM, 8 channels, 48 kHz, 32-bit, 6 samples/AVTPDU, 125 µs packet interval

| Measurement              | Min      | Max      |
|--------------------------|----------|----------|
| AVB network latency (A)  | 16 µs    | 37 µs    |
| McASP–McASP latency (B)  | —        | 332 µs * |
| Jack–Jack latency (C)    | —        | 1270 µs *|

\* Measured with McASP transaction size = 6 samples.

## Media Clock Recovery Accuracy (AM275x EVM)

CRF slave via DP83TG721 PHY:

| Parameter                        | Value   |
|----------------------------------|---------|
| Max absolute phase difference    | 200 ns* |


CRF slave via CDCE1214 PHY:

| Parameter                        | Value   |
|----------------------------------|---------|
| Max absolute phase difference    | 40 us* |

\* Further improvement is possible by tuning CRF control-loop parameters.
\endcond

## gPTP performance

| gPTP Parameter    | Value    | Notes                                                                                              |
|-------------------|----------|----------------------------------------------------------------------------------------------------|
| Initial time to sync | ~500 ms | Measured from link up with Quick sync enabled. See \ref ENET_CPSW_TSN_GPTP_SYNC_OPT "gPTP Time-to-Sync Optimization" for tuning details. |
| Sync accuracy     | < 100 ns | Tested with Intel I210, I229, and I350 NICs and TI EVM-to-EVM setups.                            |
---

For memory optimization, time-to-sync tuning, troubleshooting, and known limitations see:

- \ref ENET_CPSW_TSN_DEV_GUIDE — TSN Developer Guidelines (memory footprint, sync optimization, troubleshooting, limitations)

---

# See Also

- \ref ENET_CPSW_TSN_ARCH_GUIDE — gPTP/AVB design, hardware interaction, and EST/CBS/IET configuration via uniconf
- \ref ENET_CPSW_TSN_GPTP — gPTP stack API and integration guide
- \ref ENET_CPSW_TSN_DEV_GUIDE — TSN Developer Guidelines (memory, sync tuning, troubleshooting, limitations)
- \ref EXAMPLES_ENET_CPSW_TSN_GPTP — gPTP example index
- \ref NETWORKING — Ethernet and Networking overview

\cond SOC_AM275X
- \ref EAVB_PERFORMANCE — eAVB latency performance data
- \ref ENET_CPSW_AVTP — AVB stack API and integration guide
\endcond

\cond SOC_AM62DX
- \ref ENET_CPSW_AVTP — AVB stack API and integration guide
\endcond
