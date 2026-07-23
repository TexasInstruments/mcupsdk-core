# TSN Userguide {#ENET_CPSW_TSN_ARCH_GUIDE}

[TOC]

# Introduction

This guide explains how the two core protocols of TSN stack — gPTP (IEEE 802.1AS-2020) and AVB (IEEE 1722-2016 AVTP) — are designed and how they drive CPSW hardware.
It also introduces the three helper libraries (`tsn_unibase`, `tsn_combase`, `tsn_uniconf`) that every TSN application is built on, and shows how the CPSW hardware-offloaded traffic shaping features — EST, CBS, and IET — are configured through `tsn_uniconf`.

For feature/device coverage, memory footprint, and interoperability data, see \ref ENET_CPSW_TSN. For step-by-step integration into an application, see \ref ENET_CPSW_TSN_GPTP.

---

# IEEE 802.1AS-2020 (gPTP)

gPTP distributes a single, common notion of time across every device on the network so that senders and receivers can act on the same clock. It does this by exchanging four message types — **Announce**, **Sync**, **Follow_Up**, and **PDelay** — and running a set of standardized state machines on every port. Every device plays one of three roles: **Master** (Time-Transmitter), **Slave** (Time-Receiver), or **Bridge / Relay Instance** .

\imageStyle{gptp_roles.png,width:35%}
\image html gptp_roles.png 

## How gPTP Interacts With Hardware

Timing accuracy comes from hardware feature of the CPSW. It timestamps every PTP event frame in hardware at the exact instant it is transmitted or received, rather than when application code runs.

\imageStyle{gptp_hw.png,width:25%}
\image html gptp_hw.png "gPTP synchronization mechanism"

- **Hardware timestamp capture** — The gPTP task stores metadata(message type and equence number) of every outgoing PTP message in a queue. The CPTS, timestamps the PTP message events when the frame leaves the MAC and it stores the packet metadata along with the event timestamp. The gPTP task then can match the metadata to retrieve the required event timestamp and propagates to its inner port state machines.This queue is what makes **two-step** operation possible: the Sync frame goes out first, and once its true hardware TX timestamp is known, a Follow_Up carries that precise value.
- **Clock hardware** — The CPTS present within CPSW is used as PTP clock. gPTP's clock servo (`gptpclock.c`) adjusts this counter in two ways: a phase step to correct an instantaneous offset, this happens during initial correction or when grandmaster changes in a network, and a frequency trim to correct for oscillator drift. Whichever adjustment is available in hardware is preferred.
- **Why this matters** — because timestamps are captured in the MAC hardware path, they are largely immune to OS scheduling jitter, RTOS task latency, or interrupt load — the property that makes sub-microsecond synchronization achievable on an embedded RTOS.

## Master Role

A Master port is the source of time for everything downstream of it. It runs two independent send state machines:

- **Announce** periodically advertises the device's grandmaster credentials — priority1/priority2, clock class, clock accuracy, and the number of hops (steps removed) from the grandmaster. This is the message every port listens to when deciding who the best master is.
- **Sync + Follow_Up** periodically transmits the time itself. In two-step mode, Sync carries a placeholder and Follow_Up carries the precise origin timestamp, the accumulated correction field, and the rate ratio (how fast the master's clock is running relative to nominal) — this rate information is what lets a slave track frequency drift, not just a one-time offset.

## Slave Role

A Slave port receives Sync/Follow_Up from its master and continuously re-aligns the local clock to it:

- **Reception** pairs each Sync with its Follow_Up, applies the link's measured propagation delay, and produces an offset-from-master and a rate ratio.
- **Clock servo** feeds that offset and rate ratio into an IIR filter (independently tunable for phase and frequency) and applies the result to the hardware clock. The filter's job is to converge smoothly — reacting to a genuine change in master frequency, while ignoring per-packet network jitter.

### Clock Servo Design {#ENET_CPSW_TSN_ARCH_GUIDE_SERVO}

The servo lives in `computeGmRateRatio()` in `tsn_gptp/clock_master_sync_receive_sm.c`, and runs once per received Sync/Follow_Up pair. It maintains **two independent control loops** — one for phase, one for frequency — that both act on the same pair of timestamps: `lts` (local receipt time) and `mts` (master's origin time, corrected for path delay and residence time).

\imageStyle{clock_servo_architecture.png,width:48%}
\image html clock_servo_architecture.png "Clock servo: shared Sync/Follow_Up measurements feed two independent loops"

Both loops share the same measurement, but they correct different things: the frequency loop tracks how fast the master's oscillator is running relative to the local one (a ratio, corrected continuously in small increments), while the phase loop tracks the absolute gap between the two clocks at this instant (corrected either by a direct step or by temporarily biasing the frequency).

#### Frequency Offset Loop

Each Sync interval, the raw rate is computed from the elapsed master/local time deltas and filtered with a single-pole IIR:

\code
// IIR filter, M(n) = a*R(n) + (1-a)*M(n-1)
nrate = alpha * (dmts / dlts) + (1 - alpha) * mrate;
ppb   = (int)((nrate - 1.0) * 1.0E9);
\endcode

`alpha` itself switches between two configured values depending on a **stability state**:

\imageStyle{freq_stability_state_machine.png,width:40%}
\image html freq_stability_state_machine.png "Frequency loop stability state machine"

While unstable, the filter reacts quickly (default `alpha = 1/2`) to pull the rate estimate in fast; once three consecutive samples land inside `FREQ_OFFSET_STABLE_PPB` (default 100 ppb) of each other, the servo switches to the heavily-filtered `alpha = 1/10` used for steady-state tracking, which rejects per-packet jitter. If the rate estimate ever jumps by more than `FREQ_OFFSET_TIMELEAP_MAX_JUMP_PPB` (default 800 ppb) — e.g. a grandmaster change, or the phase loop just stepped the clock — the loop drops back to `RATE_UNSTABLE` and re-converges from scratch.

The filtered `ppb` is only pushed to hardware (`gptpclock_setadj()`) when it differs from the last applied value by more than `FREQ_OFFSET_UPDATE_MRATE_PPB` (default 10 ppb), and the cumulative adjustment is clamped to `MAX_ADJUST_RATE_ON_CLOCK` (default 1,000,000 ppb) in either direction.

#### Phase Offset Loop

The phase loop tracks the instantaneous gap `dts = mts - lts` through its own three-state machine, implemented in `set_phase_offsetGM()`:

\imageStyle{phase_offset_state_machine.png,width:55%}
\image html phase_offset_state_machine.png "Phase loop offset state machine"

Once a new `offsetGM` is computed, the servo decides *how* to apply it, not just what value to apply:

- If the residual gap is smaller than `PHASE_OFFSET_ADJUST_BY_FREQ` (default 100us) and `PHASE_ADJUSTMENT_BY_FREQ` is enabled (it is, by default), the gap is folded into the frequency loop instead of the hardware clock — the servo briefly biases the rate so the phase error is walked down smoothly, rather than stepped. Gaps smaller than `PHASE_OFFSET_ADJUST_TARGET` (10ns) are treated as already converged and ignored.
- Otherwise, `gptpclock_setoffset64()` performs a direct phase step (jump) on the hardware/software clock. Immediately after a step, the servo skips frequency adjustment for `SKIP_FREQADJ_COUNT_MAX` (default 2) Sync intervals — a step changes the clock's absolute time, which would otherwise look like a huge, spurious rate change to the frequency loop on the very next sample.

#### Tunable Constants

All of the thresholds above are configured via the same non-YANG `gptpgcfg_set_item()` mechanism described in \ref ENET_CPSW_TSN_YANG_CONFIG_PARAMS (defaults from `gptp_nonyangconfig.xml`):

 Parameter                                    | Default   | Meaning
 ----------------------------------------------|-----------|--------
 `FREQ_OFFSET_IIR_ALPHA_START_VALUE`            | 2         | Reciprocal alpha while frequency is unstable (fast)
 `FREQ_OFFSET_IIR_ALPHA_STABLE_VALUE`           | 10        | Reciprocal alpha once frequency is stable (slow, jitter-rejecting)
 `FREQ_OFFSET_STABLE_PPB`                       | 100       | ppb delta below which 3 consecutive samples mark rate stable
 `FREQ_OFFSET_TIMELEAP_MAX_JUMP_PPB`             | 800       | ppb delta above which a stable rate is declared unstable again
 `FREQ_OFFSET_UPDATE_MRATE_PPB`                  | 10        | Minimum ppb change before writing a new rate to hardware
 `MAX_ADJUST_RATE_ON_CLOCK`                      | 1,000,000 | Clamp on the cumulative ppb adjustment
 `PHASE_OFFSET_IIR_ALPHA_START_VALUE`            | 2         | Reciprocal alpha while phase is unstable
 `PHASE_OFFSET_IIR_ALPHA_STABLE_VALUE`           | 10        | Reciprocal alpha once phase is stable
 `PHASE_OFFSET_ADJUST_BY_FREQ`                   | 100000 (100us) | Above this gap, always step the clock instead of biasing frequency
 `SKIP_FREQADJ_COUNT_MAX`                        | 2         | Sync intervals to skip frequency adjustment right after a phase step
 `CLOCK_COMPUTE_INTERVAL_MSEC`                   | 1000      | Minimum spacing enforced between servo computations once adjusting

#### QUICK_SYNC_ALGO — Fast-Sync Path

By default (`QUICK_SYNC_ALGO = 0`), the phase and frequency loops run **simultaneously** from the very first Sync — every received Sync/Follow_Up feeds both loops, and both may push corrections to hardware in the same interval. This is standards-compliant and works, but the two loops can fight each other: a frequency correction shifts future timestamps, which the phase loop sees as more offset to correct, which triggers a step, which the frequency loop then misreads as a rate change (this is exactly why `SKIP_FREQADJ_COUNT_MAX` exists at all).

Setting `QUICK_SYNC_ALGO = 1` removes that interaction by strictly sequencing the two loops: **frequency first, phase only after frequency is stable.**

\code
/* QUICK_SYNC_ALGO: Apply the phase offset by freq. only after the freq. has
 * stabilized to prevent interference between phase and freq. adjustments. */
if (!gptpgcfg_get_intitem(GPTPINSTNUM, XL4_EXTMOD_XL4GPTP_QUICK_SYNC_ALGO, YDBI_CONFIG)
    || sm->rate_is_stable) {
    offset_comp = set_phase_offsetGM(sm, dts, dlts);
}
\endcode

\imageStyle{quick_sync_timeline.png,width:50%}
\image html quick_sync_timeline.png "Default vs Quicksync Algorithm timelines"

Because the frequency loop no longer has to fight a moving phase target, and the phase loop no longer starts from a still-drifting rate estimate, both loops individually converge faster — at the cost of the phase gap staying uncorrected for the first handful of Syncs after startup or a grandmaster change. `QUICK_SYNC_ALGO` defaults to off for backward compatibility; enable it when fast time-to-sync after a grandmaster change matters more than the phase gap during that initial window.

Whichever mode is active, once both loops settle — `rate_is_stable == true` and the phase state machine reaches `OFFSET_STABLE_ADJ` — the port's status is promoted from `GMSYNC_SYNC` to `GMSYNC_SYNC_STABLE`, which is the state application code should treat as "locked."

## Bridge (Boundary Clock)

A bridge (Relay Instance) has one Slave port receiving time from upstream and one or more Master ports distributing that same time downstream — it does not have its own independent time source. The relay is handled by `port_sync_sync_send_sm.c` / `port_sync_sync_receive_sm.c`: the timestamp and rate ratio recovered on the Slave port are forwarded internally and re-transmitted as a new Sync/Follow_Up on every Master port, with each port's own link delay folded into the correction field. From the network's point of view, a chain of bridges behaves as a single distributed clock, with the `steps removed` count in Announce messages growing by one at each hop so downstream devices can gauge how far they are from the grandmaster.

**BMCA (Best Master Clock Algorithm)** is what decides which port is Master, Slave, or Passive. `port_state_selection_sm.c` compares the priority vector carried in every Announce received on every port — grandmaster identity, clock class, clock accuracy, steps removed, and port identity, in that order — and gives the Slave role to whichever port received the numerically best vector. Every other port becomes Master (or Passive, if it hears an equally good vector from a peer bridge). The stack also supports pinning port state statically, bypassing BMCA entirely for topologies that never change.

**Path delay** is measured independently of port role, on every link, via a three-message peer-to-peer exchange (`md_pdelay_req_sm.c` / `md_pdelay_resp_sm.c`):

[Back To Top](\ref ENET_CPSW_TSN_ARCH_GUIDE)

---

\cond SOC_AM62DX || SOC_AM275X
# IEEE 1722 (AVB)

AVB (via the AVTP transport defined in IEEE 1722-2016) carries synchronized audio/video streams over the same Ethernet network, where the whole network is timed by gPTP  and the traffic is shaped by CBS/EST. Every AVTPDU (AVTP Data Unit) shares a common stream header — an 8-bit `subtype` field selects the payload format, followed by a `stream_id` (unique per talker/stream) and a 32-bit `avtp_timestamp` that is expressed in gPTP time, which is what lets a listener play out media in sync with other listeners on the network without any additional handshaking.

## AAF PCM, AES3, CRF Packet Types

|            Subtype                            | Value  |                        Purpose                                                                   |
|-----------------------------------------------|--------|--------------------------------------------------------------------------------------------------|
| **AAF** (Audio Format)                        | `0x02` | Carries audio payload — either raw PCM samples or an encapsulated AES3 bitstream                 |
| **CRF** (Clock Reference Format)              | `0x04` | Carries *only* timestamps, no media — used to distribute a shared media clock                    |
| **TSCF** (Time-Sensitive Control Format)      | `0x05` | Carries time sensitive and high priority control data that require strict timing constraints     |
| **NTSCF** (Non-Time-Sensitive Control Format) | `0x82` | Used for control information or configuration changes that do not rely on hard time constraints  |

**AAF PCM** packs raw audio samples directly into the AVTPDU payload. The format field selects `Int16` / `Int24` / `Int32` / `Float32`; sample rate (8 kHz–192 kHz) and channel count are set per stream, and the number of samples per AVTPDU is chosen so that packet interval lines up with the network's Class A/B intervals (typically 6 samples/packet at 48 kHz, 125 µs cadence).

**AAF AES3** reuses the same AAF container to carry a compressed AES3/SMPTE 337 bitstream (e.g., Dolby EAC3) instead of raw PCM. Because a Dolby sync frame (up to 4096 bytes plus a 12-byte SMPTE 337 preamble) is larger than one AVTPDU payload, the talker fragments it across several AAF packets and the listener reassembles it before handing it to the decoder — AES3 sub-frame validity bits (V/U/C/P) are only meaningful once a full 192-frame block has been reassembled.

**CRF** carries a small array of network timestamps per packet (typically 6, at roughly 50 packets/second). The packet headers carries information about the nominal Media clock frequency and `pull` field lets the stream represent a scaled frequency rather than only the nominal rate.

## When To Use Which Packet Type

| Use case | Packet type |
|----------|-------------|
| Uncompressed audio streaming (the common case) | **AAF PCM** |
| Compressed/bitstream audio (Dolby, DTS, etc.) | **AAF AES3** |
| Distributing one shared clock reference to many talkers/listeners without sending media on that stream | **CRF** |
| Time sensitive data like motor control messages etc| **TSCF** |
| Non Time sensitive data | **NTSCF** |

## Talker, Listener, and CBS

A **Talker** timestamps every outgoing AVTPDU with the current gPTP time and transmits it on a configured stream ID, VLAN, and multicast address. A **Listener** joins that stream, unpacks the AVTPDU, and delivers samples (or, for AES3, reassembled frames) to the application. Bandwidth for each stream is reserved in hardware by the **Credit-Based Shaper (CBS, IEEE 802.1Qav)** — every stream's traffic class gets an idle-slope (bytes/sec) budget, so AVB traffic is smoothed to its reserved rate instead of bursting and starving other classes. See [CBS Configuration](#ENET_CPSW_TSN_ARCH_GUIDE_CBS) below for how this idle-slope is set.

## Media Clock Recovery

Audio hardware (DACs, codecs) needs a stable *sample clock*, not just correctly-timed packets. A CRF stream solves this by carrying nothing but timestamps: the CRF listener compares each incoming network timestamp against its own local clock and feeds the error into a PLL ("nudge") that steers a recovered clock output — on TI platforms this drives a DP83TG721 PHY's recovered-clock pin or a CDCE1214 clock synthesizer, which in turn clocks the audio codec via McASP. Because one CRF stream can feed many AAF talkers/listeners, clock distribution and media transport stay decoupled — a codec doesn't need to derive its clock from the AAF stream it happens to be playing.

[Back To Top](\ref ENET_CPSW_TSN_ARCH_GUIDE)
\endcond

---

# Unibase, Uniconf, Combase Libraries

Both gPTP and AVB are built as clients of three shared, protocol-agnostic libraries. They abstract the underlying hardware and to avoid reimplementation of logging, sockets, or configuration management in gPTP and AVB libraries.

\imageStyle{tsn_helper_libs_stack.png,width:50%}
\image html tsn_helper_libs_stack.png "TSN software stack: gPTP and AVTP both run on uniconf, combase, and unibase"

- **tsn_unibase** is the lowest layer: portable logging (category- and level-based, e.g. `"gptp:55"`), memory allocation, timestamp/time-conversion macros, and linked-list/array utilities. Every other TSN library links against it.
- **tsn_combase** sits above unibase and provides the OS/network binding: sockets, threads, timers, mutexes, and — critically for hardware configuration — `combase_link`, a hardware-agnostic parameter layer (e.g. `cbl_cbs_params_t`, `cbl_tas_sched_params_t`) that uniconf's hardware abstraction uses to reach the Enet LLD without knowing about CPSW registers directly.
- **tsn_uniconf** is the configuration and state spine: a YANG-modeled key/value database (SimpleDB) plus a notification mechanism. gPTP reads its Yang and non-Yang parameters from this database at startup and writes runtime state back into it (AS_CAPABLE, clock state, performance counters); AVTP's CBS bandwidth reservation and CPSW's EST/IET hardware configuration are all driven through the same database. Uniconf runs as its own FreeRTOS task alongside the gPTP task.

In short: gPTP and AVTP are the protocol logic, uniconf is where their configuration and live state lives; combase is how uniconf (and the protocols) talk to sockets, timers, and hardware; unibase is the common utility floor underneath all of it.

[Back To Top](\ref ENET_CPSW_TSN_ARCH_GUIDE)

---

# Configuring TSN Hardware Features via Uniconf

EST, CBS, and IET are all CPSW hardware offloads — the actual gate scheduling, credit shaping, and frame preemption happen in silicon. Uniconf's job is to take a YANG-modeled configuration write from the application and push it down to the right hardware registers through a common pipeline:

\imageStyle{uniconf_hw_config_flow.png,width:50%}
\image html uniconf_hw_config_flow.png "Application write reaches CPSW hardware through uniconf's HAL and combase_link"

Every feature below is a leaf (or sub-tree) of the same `ietf-interfaces` YANG model, and every write follows the same two-call pattern:

1. **Write the value into the database** with `YDBI_SET_ITEM(<accessor>, <ifname>, <key1>, <key2>, ..., YDBI_CONFIG, &value, sizeof(value), YDBI_NO_NOTICE, ...)`. The `<accessor>` (e.g. `ifknvk0`, `ifk4vk1`) selects how many YANG key levels and array-index parameters the call takes — you don't need to memorize these, just copy the pattern used by the leaf you're setting in the examples below.
2. **Ask uniconf to push it to hardware** with `uc_nc_askaction_push(ucntd, dbald, aps, kvs, kss)`, where `aps[]` is the same key path as an array of numeric YANG-node IDs. `uc_hwal.c` reacts to this call, builds a hardware-agnostic `combase_link` struct (e.g. `cbl_tas_sched_params_t`, `cbl_preempt_params_t`), and the Enet LLD turns that into the actual CPSW IOCTL.

All code below is taken from (or, where noted, assembled from confirmed building blocks in) the SDK's own TSN example applications under `source/networking/enet/core/examples/tsn/`.

## Configuring EST (802.1Qbv Scheduled Traffic) {#ENET_CPSW_TSN_ARCH_GUIDE_EST}

EST (Enhancements for Scheduled Traffic) opens and closes each traffic class's gate on a repeating, time-anchored schedule, so time-critical traffic gets a guaranteed transmission window every cycle.

Configuration is a **gate control list**: a cycle time, a base time (the PTP-domain instant the schedule starts), and an ordered list of `(gate-state-mask, duration)` entries, where each bit in the mask opens (`1`) or closes (`0`) one of the eight traffic classes for that interval. Because `admin-base-time` is expressed in gPTP time, **gPTP must be synchronized before EST is armed** — otherwise every device's schedule starts at a different wall-clock instant and the whole point of scheduled traffic is lost.

The gate list itself is built as a plain C struct — no uniconf calls yet, just the schedule you want applied:

\code
#define EST_INTERVAL_NS  (62000U)   /* 62 us per gate interval          */

EnetTas_ControlList list =
{
    .baseTime  = 0ULL,                 /* filled in from gPTP time below */
    .cycleTime = 4 * EST_INTERVAL_NS,  /* 248 us, 4 intervals per cycle   */
    .gateCmdList =
    {
        /* tc7 tc6 tc5 tc4 tc3 tc2 tc1 tc0 */
        { .gateStateMask = ENET_TAS_GATE_MASK(1,  0,  0,  0,  0,  0,  0,  1), .timeInterval = EST_INTERVAL_NS },
        { .gateStateMask = ENET_TAS_GATE_MASK(1,  0,  0,  0,  0,  1,  0,  0), .timeInterval = EST_INTERVAL_NS },
        { .gateStateMask = ENET_TAS_GATE_MASK(1,  0,  0,  0,  0,  0,  0,  1), .timeInterval = EST_INTERVAL_NS },
        { .gateStateMask = ENET_TAS_GATE_MASK(1,  0,  0,  0,  0,  1,  0,  0), .timeInterval = EST_INTERVAL_NS },
    },
    .listLength = 4U,
};
\endcode

Each `ENET_TAS_GATE_MASK(tc7..tc0)` entry opens traffic class 7 (highest priority — always kept open here) together with exactly one other class for that interval, so tc0 and tc2 alternate getting an exclusive window every 62 µs. `admin-base-time` should be a few cycles into the future, rounded up to a cycle boundary, which is exactly what `EnetEstApp_runSchedule()` does using the current gPTP time (`gptpmasterclock_getts64()`).

Pushing that list to uniconf is a sequence of `YDBI_SET_ITEM()` writes followed by one trigger:

\code
    uint8_t kn[5] = {
        [0] = IETF_INTERFACES_BRIDGE_PORT, 
        [1] = IETF_INTERFACES_GATE_PARAMETER_TABLE
    };

    /* admin-cycle-time = numerator/denominator, in microseconds */
    uint32_t cycleNum = list.cycleTime / 1000U; /* 248 us */
    uint32_t cycleDen = 1000000UL;
    kn[2] = IETF_INTERFACES_ADMIN_CYCLE_TIME;
    kn[3] = IETF_INTERFACES_NUMERATOR;
    YDBI_SET_ITEM(ifknvk0, ifname, kn, 4, YDBI_CONFIG, &cycleNum, sizeof(cycleNum), YDBI_NO_NOTICE);
    kn[3] = IETF_INTERFACES_DENOMINATOR;
    YDBI_SET_ITEM(ifknvk0, ifname, kn, 4, YDBI_CONFIG, &cycleDen, sizeof(cycleDen), YDBI_NO_NOTICE);

    /* admin-base-time = seconds + nanoseconds, in gPTP time */
    uint32_t sec = list.baseTime / 1000000000ULL;
    uint32_t nsec = list.baseTime % 1000000000ULL;
    kn[2] = IETF_INTERFACES_ADMIN_BASE_TIME;
    kn[3] = IETF_INTERFACES_SECONDS;
    YDBI_SET_ITEM(ifknvk0, ifname, kn, 4, YDBI_CONFIG, &sec, sizeof(sec), YDBI_NO_NOTICE);
    kn[3] = IETF_INTERFACES_NANOSECONDS;
    YDBI_SET_ITEM(ifknvk0, ifname, kn, 4, YDBI_CONFIG, &nsec, sizeof(nsec), YDBI_NO_NOTICE);

    /* admin-control-list: one gate-control-entry per gateCmdList[] slot */
    for (int i = 0; i < list.listLength; i++)
    {
        kn[2] = IETF_INTERFACES_ADMIN_CONTROL_LIST;
        kn[3] = IETF_INTERFACES_GATE_CONTROL_ENTRY;
        kn[4] = IETF_INTERFACES_GATE_STATES_VALUE;
        YDBI_SET_ITEM(ifknvk1, ifname, i, 4u, kn, 5u, YDBI_CONFIG,
                      &list.gateCmdList[i].gateStateMask, sizeof(uint8_t), YDBI_NO_NOTICE);
        kn[4] = IETF_INTERFACES_TIME_INTERVAL_VALUE;
        YDBI_SET_ITEM(ifknvk1, ifname, i, 4u, kn, 5u, YDBI_CONFIG,
                      &list.gateCmdList[i].timeInterval, sizeof(uint32_t), YDBI_NO_NOTICE);
    }

    /* gate-enabled = true arms the schedule at admin-base-time */
    bool enable = true;
    YDBI_SET_ITEM(ifk3vk0, ifname, IETF_INTERFACES_BRIDGE_PORT, IETF_INTERFACES_GATE_PARAMETER_TABLE,
                  IETF_INTERFACES_GATE_ENABLED, YDBI_CONFIG, &enable, sizeof(enable), YDBI_NO_NOTICE);

    /* Ask uniconf to push gate-enabled (and everything staged above it) to CPSW TAS registers */
    uint8_t aps[] = {IETF_INTERFACES_RW, IETF_INTERFACES_INTERFACES, IETF_INTERFACES_INTERFACE,
                     IETF_INTERFACES_BRIDGE_PORT, IETF_INTERFACES_GATE_PARAMETER_TABLE,
                     IETF_INTERFACES_GATE_ENABLED, 255u};
    void *kvs[] = {(void *)ifname, NULL, NULL};
    uint8_t kss[] = {strlen(ifname) + 1, 0};
    uc_nc_askaction_push(ucntd, dbald, aps, kvs, kss);
\endcode

`ifname` is the same `combase` device name used everywhere else in TSN init (e.g. `"tilld0"`). See the full, working version — including the wait for gPTP sync before arming the schedule — in `est_init.c` under \ref ENET_CPSW_TSN_EXAMPLES (listed on devices where the EST example is supported).

## Configuring IET (802.3br / 802.1Qbu Frame Preemption)

IET (Interspersing Express Traffic) lets latency-critical "express" frames interrupt a lower-priority "preemptable" frame mid-transmission, then resume the preempted frame afterward — bounding the worst-case latency of express traffic regardless of what else is queued in a lower class.

Configuration is a static, one-time classification of each of the eight traffic classes as `express` (value `1` — default; never preempted) or `preemptable` (value `2`; may be paused by any express frame) — there is no schedule or timing involved. The SDK's shipped IET example (`enet_cpsw_iet/iet_loopback_test.c`) applies this classification with a direct Enet LLD call, without going through uniconf:

\code
/* From EnetApp_IET_setup() in test/enet_cpsw_iet/iet_loopback_test.c */
EnetMacPort_SetPreemptQueueInArgs queuePreemptInArgs;
queuePreemptInArgs.macPort = gEnetLpbk.macPort;
for (i = 0U; i < CPSW_MACPORT_FIFO; i++)
{
    queuePreemptInArgs.queuePreemptCfg.preemptMode[i] =
        (i >= 4U) ? ENET_MAC_QUEUE_PREEMPT_MODE_EXPRESS   /* tc4..tc7: never preempted */
                  : ENET_MAC_QUEUE_PREEMPT_MODE_PREEMPT;  /* tc0..tc3: preemptable     */
}
ENET_IOCTL_SET_IN_ARGS(&prms, &queuePreemptInArgs);
ENET_IOCTL(gEnetLpbk.hEnet, gEnetLpbk.coreId, ENET_MACPORT_IOCTL_SET_PREEMPT_QUEUE, &prms, status);
\endcode

A uniconf-based path for the same setting also exists in the stack — `tsn_uniconf`'s hardware abstraction (`uc_hwal.c`, `preempt_hw_action()`) already reads a per-priority `frame-preemption-status-table` leaf from the `ietf-interfaces` YANG model and calls the same `combase_link` primitive (`cbl_preempt_setup()`, `cbl_preempt_params_t.prioiry_preempt[8]`, `1 = express` / `2 = preemptable`) that the IOCTL path above drives directly. No shipped example application exercises this leaf yet, but the write follows the identical pattern used for EST and CBS above:

\code
/* Assembled from the confirmed ietf-interfaces YANG leaves consumed by
 * uc_hwal.c's preempt_hw_action() — no shipped example calls this today,
 * but the leaves and the HAL that reads them are real. */
uint8_t kn[4] = { IETF_INTERFACES_BRIDGE_PORT, IETF_INTERFACES_FRAME_PREEMPTION_PARAMETERS,
                  IETF_INTERFACES_FRAME_PREEMPTION_STATUS_TABLE, 0 };
for (uint8_t tc = 0; tc < 8; tc++)
{
    uint8_t mode = (tc >= 4) ? 1 /* express */ : 2 /* preemptable */;
    kn[3] = IETF_INTERFACES_PRIORITY0 + tc;
    YDBI_SET_ITEM(ifknvk0, ifname, kn, 4, YDBI_CONFIG, &mode, sizeof(mode), YDBI_NO_NOTICE);
}

uint8_t aps[] = { IETF_INTERFACES_RW, IETF_INTERFACES_INTERFACES, IETF_INTERFACES_INTERFACE,
                  IETF_INTERFACES_BRIDGE_PORT, IETF_INTERFACES_FRAME_PREEMPTION_PARAMETERS,
                  IETF_INTERFACES_FRAME_PREEMPTION_STATUS_TABLE, 255u };
void  *kvs[] = { (void *)ifname, NULL, NULL };
uint8_t kss[] = { strlen(ifname) + 1, 0 };
uc_nc_askaction_push(ucntd, dbald, aps, kvs, kss);  /* preempt_hw_action() reads all 8 priorities and applies them */
\endcode

Preemption only becomes active (`preemption-active` reads true) once the link partner also negotiates 802.3br support — check that before assuming express traffic is actually being fast-pathed.

## Configuring CBS (802.1Qav Credit-Based Shaper) {#ENET_CPSW_TSN_ARCH_GUIDE_CBS}

CBS smooths a stream's transmissions to a reserved bandwidth using a credit counter: credit accumulates at the configured **idle-slope** while the queue is empty or waiting, and drains while frames are sent, so a stream can never burst far above its reservation even if the application bursts data into the queue.

Configuration is one `admin-idleslope` value (bits/sec) per traffic class. **Order matters**: idle-slope must be written starting from the highest-priority traffic class downward, since CPSW's shaper hardware computes each lower class's available bandwidth relative to what's already reserved above it — the AAF PCM Ethernet-ring demo's own comment says this explicitly:

\code
/* From gCbsDefaultCfg in aafpcm_audio_etherring_demo/common_files/cbs_config.c */
EnetCbsParam_t gCbsDefaultCfg = {
    .cbsparams =
    {
        /* The order for setting idleSlope must be started from highest
         * priority queue 7 and next lower priority queue in descending order. */
        { .tc = 7, .priority = 7, .idleSlope = 10 * MBPS },
        { .tc = 6, .priority = 6, .idleSlope = 10 * MBPS },
        { .tc = 5, .priority = 5, .idleSlope = 10 * MBPS },
        { .tc = 4, .priority = 4, .idleSlope = 10 * MBPS },
        /* ... tc 3..0 ... */
    },
    .length = 8U,
};
\endcode

Enabling CBS and writing each class's idle-slope both go through the same YDBI write + trigger pattern as EST:

\code
    /* cbs-enabled = true, once per interface */
    uint8_t kn[3] = {
        IETF_INTERFACES_BRIDGE_PORT, 
        IETF_INTERFACES_TRAFFIC_CLASS, 
        IETF_INTERFACES_CBS_ENABLED
    };
    bool cbsEnabled = true;
    YDBI_SET_ITEM(ifknvk0, ifname, kn, 3, YDBI_CONFIG, &cbsEnabled, sizeof(cbsEnabled), YDBI_NO_NOTICE);

    uint8_t enAps[] = {IETF_INTERFACES_RW, IETF_INTERFACES_INTERFACES, IETF_INTERFACES_INTERFACE,
                       IETF_INTERFACES_BRIDGE_PORT, IETF_INTERFACES_TRAFFIC_CLASS,
                       IETF_INTERFACES_CBS_ENABLED, 255u};
    void *enKvs[] = {(void *)ifname, NULL, NULL};
    uint8_t enKss[] = {strlen(ifname) + 1, 0};
    uc_nc_askaction_push(ucntd, dbald, enAps, enKvs, enKss);

    /* admin-idleslope, per traffic class, highest priority first */
    for (int i = 0; i < cbsPrm->length; i++) /* cbsPrm->cbsparams[] pre-sorted tc7 -> tc0 */
    {
        uint8_t tc = cbsPrm->cbsparams[i].tc;
        YDBI_SET_ITEM(ifk4vk1, ifname,
                      IETF_INTERFACES_TRAFFIC_CLASS, IETF_INTERFACES_TC_DATA, IETF_INTERFACES_ADMIN_IDLESLOPE,
                      255, &tc, sizeof(tc),
                      YDBI_CONFIG, &cbsPrm->cbsparams[i].idleSlope, sizeof(uint64_t),
                      YDBI_NO_NOTICE, YANG_DB_ONHW_NOACTION);

        uint8_t aps[] = {IETF_INTERFACES_RW, IETF_INTERFACES_INTERFACES, IETF_INTERFACES_INTERFACE,
                         IETF_INTERFACES_BRIDGE_PORT, IETF_INTERFACES_TRAFFIC_CLASS,
                         IETF_INTERFACES_TC_DATA, IETF_INTERFACES_ADMIN_IDLESLOPE, 255u};
        void *kvs[] = {(void *)ifname, (void *)&tc, NULL};
        uint8_t kss[] = {strlen(ifname) + 1, sizeof(tc), 0};
         /* uniconf computes hi/lo-credit + send-slope and programs the shaper */
        uc_nc_askaction_push(ucntd, dbald, aps, kvs, kss);
    }
\endcode

If the application needs to know when the hardware update has actually landed before moving to the next (lower-priority) class, register a notice semaphore on the same key path before writing and wait on it (`uc_nc_notice_register()` / `uc_notice_sig_check()`) — `cbs_config.c` does this at startup so the highest-priority class is guaranteed configured in hardware before the next one is written. See `cbs_config.c` (used by the AAF PCM Ethernet-ring demo, `aafpcm_audio_etherring_demo`) for the complete, working multi-class sequence.

---

# See Also

- \ref ENET_CPSW_TSN — TSN feature/standard coverage, resource utilization, memory footprint
- \ref ENET_CPSW_TSN_GPTP — gPTP stack API and integration guide
- \ref ENET_CPSW_TSN_DEV_GUIDE — TSN developer guidelines (sync tuning, troubleshooting, limitations)
- \ref NETWORKING — Ethernet and Networking overview
\cond SOC_AM275X
- \ref ENET_CPSW_AVTP — AVB stack API and integration guide
- \ref EAVB_PERFORMANCE — eAVB latency performance data
\endcond
\cond SOC_AM62DX
- \ref ENET_CPSW_AVTP — AVB stack API and integration guide
\endcond

[Back To Top](\ref ENET_CPSW_TSN_ARCH_GUIDE)
