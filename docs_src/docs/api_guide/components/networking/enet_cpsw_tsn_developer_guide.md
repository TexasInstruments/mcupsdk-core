# TSN Developer Guidelines {#ENET_CPSW_TSN_DEV_GUIDE}

[TOC]

This page covers memory optimization, time-to-sync tuning, troubleshooting, and known limitations for the TI TSN stack.

---

# Memory Footprint Optimization {#ENET_CPSW_TSN_MEMORY_OPT}

The primary knobs for reducing RAM are the SimpleDB entry counts and the number of enabled modules.
All knobs are in `source/networking/tsn/tsn-stack/tsn_buildconf/sitara_buildconf.h`.

## Disable unused modules

```c
/* sitara_buildconf.h */
#define GPTP_LIB_ENABLE   1  /* Set to 0 to exclude gPTP */
```
\cond SOC_AM62DX || SOC_AM275X
```c
/* sitara_buildconf.h */
#define AVTP_LIB_ENABLE   1  /* Set to 0 to exclude AVTP */
```
\endcond

## Reduce SimpleDB entry counts

Each SimpleDB entry is a fixed-size slot. Reduce to the minimum needed for your configuration:

```c
/* sitara_buildconf.h — overrides gptp_buildconf.h defaults for Sitara targets */
#define GPTP_SIMPLEDB_DBDATANUM   0     /* Sitara uses SIMPLEDB_DBDATANUM total; reduce via other knobs */
#define GPTP_MAX_DOMAINS          1     /* Keep at 1 if CMLDS/dual-domain not needed */
#define GPTP_MAX_PORTS            2     /* Match to actual MAC port count */
```

\cond SOC_AM62DX || SOC_AM275X
```c
/* avtp_buildconf.h */
#define AVB_TALKER_STREAMS_NUMBER   2   /* Reduce to actual number of talker streams */
#define AVB_LISTENER_STREAMS_NUMBER 1   /* Reduce to actual number of listener streams */
#define AVTP_SIMPLEDB_DBDATANUM    50   /* Reduce for fewer stream entries */
```
\endcond

## Reduce UB_ESARRAY pool

```c
/* sitara_buildconf.h */
#define UB_ESARRAY_DFNUM    256  /* Number of entries per elastic array instance */
#define UB_ESARRAY_INSTNUM   40  /* Number of elastic array instances */
```

Reduce `UB_ESARRAY_DFNUM` and `UB_ESARRAY_INSTNUM` if profiling shows they are over-provisioned
for your application's data model size.

## gPTP shared memory (clock domain IPC)

The `CB_NOIPCSHMEM_DFNUM` and `CB_NOIPCSHMEM_DFSIZE` control the shared clock memory
used to pass gPTP timestamps to application tasks without OS IPC overhead.

```c
/* gptp_buildconf.h — values for 1 domain (R5F 32-bit arch) */
#define CB_NOIPCSHMEM_DFNUM    4   /* 5 if GPTP_MAX_DOMAINS == 2 */
#define CB_NOIPCSHMEM_DFSIZE  64   /* bytes per entry: 64 on 32-bit (R5F), 128 on aarch64 */
```

For a single domain (`GPTP_MAX_DOMAINS=1`), 4 entries of 64 bytes suffice on Sitara R5F.
Adding a second domain requires 5 entries.

---

# gPTP Time-to-Sync Optimization {#ENET_CPSW_TSN_GPTP_SYNC_OPT}

These knobs reduce the time from link-up to steady-state synchronization (`gmsync=true`).
All are set via `gptpgcfg_set_item()` before `gptpman_run()` in `tsninit.c`.

## Shorten the clock computation interval

Reduces how long the servo waits before updating the phase and frequency estimate.
Default is 1000 ms (1 second). Setting to 100 ms gives 10× faster initial convergence:

```c
uint8_t compute_interval = 100;  /* ms; default = 1000 */
gptpgcfg_set_item(instnum, XL4_EXTMOD_XL4GPTP_CLOCK_COMPUTE_INTERVAL_MSEC,
                  YDBI_CONFIG, &compute_interval, sizeof(compute_interval));
```

## Increase frequency offset update rate

Higher update rate allows the servo to track frequency deviation faster:

```c
uint32_t freq_mrate = 5;  /* ppb/step; default = 10 */
gptpgcfg_set_item(instnum, XL4_EXTMOD_XL4GPTP_FREQ_OFFSET_UPDATE_MRATE_PPB,
                  YDBI_CONFIG, &freq_mrate, sizeof(freq_mrate));
```

## Reduce IIR filter alpha (faster response)

Lower alpha = faster servo response = quicker lock, but less noise filtering.
Use value 1–2 for fast initial lock with compatible hardware:

```c
uint32_t phase_alpha = 1;   /* 1/alpha = filter response; default = 10 */
uint32_t freq_alpha  = 2;   /* default = 10 */
gptpgcfg_set_item(instnum, XL4_EXTMOD_XL4GPTP_PHASE_OFFSET_IIR_ALPHA_STABLE_VALUE,
                  YDBI_CONFIG, &phase_alpha, sizeof(phase_alpha));
gptpgcfg_set_item(instnum, XL4_EXTMOD_XL4GPTP_FREQ_OFFSET_IIR_ALPHA_STABLE_VALUE,
                  YDBI_CONFIG, &freq_alpha, sizeof(freq_alpha));
```

**Caution:** These aggressive settings are tested with Intel I210 and EVM-to-EVM setups.
Tune `phase_alpha` and `freq_alpha` upward if oscillation is observed after lock with other endpoints.

## Shorten the sync interval

Faster sync messages reach the slave more frequently, accelerating initial phase acquisition:

```c
/* gptp_init.c */
#define SYNC_LOG  -4   /* 62.5 ms; default = -3 (125 ms) */
#define PDELAY_LOG 0   /* peer delay interval: keep at 0 (1 s) unless network is noisy */
```

Also update the base timer interval in `sitara_buildconf.h`:

```c
#define GPTPNET_INTERVAL_TIMEOUT_NSEC  62500000u  /* Must be <= sync interval */
```

Minimum achievable sync interval is **7.8125 ms** (limited by the 7.8125 ms base timer tick).

## Enable hardware phase adjustment

If the SoC and PHY combination supports nanosecond-resolution hardware phase correction,
enabling it reduces the number of software-only phase-jump steps needed during initial lock:

```c
bool use_hwphase = true;
gptpgcfg_set_item(instnum, XL4_EXTMOD_XL4GPTP_USE_HW_PHASE_ADJUSTMENT,
                  YDBI_CONFIG, &use_hwphase, sizeof(use_hwphase));
```

---

# Troubleshooting Guidelines {#ENET_CPSW_TSN_TROUBLESHOOT}

## How to read gPTP log output

Enable INFO-level logging for all modules:

```c
unibase_init_para_t initPara;
ubb_default_initpara(&initPara);
initPara.ub_log_initstr = "4,ubase:45,cbase:45,uconf:45,gptp:55m,avtp:45";
unibase_init(&initPara);
```

Format: `<console_level>,<module>:<console_level><internal_level>[m]`.
The `m` suffix on gptp enables memory-buffered output for that module.
Level 4 = INFO, 5 = INFOV, 6 = DEBUG.

## gPTP is running but clock is not locked (gmsync=false)

1. Check that at least one port reports `asCapable=true`. If no port is `asCapable`,
   the peer-delay measurement has not completed — verify physical link is up and the
   peer device is sending pdelay-request frames.
2. Check that `portState` is either `SLAVE` or `MASTER` (not `PASSIVE` or `DISABLED`).
3. Check that `syncReceiptTimeout` is not firing. A timeout of three consecutive sync
   intervals (3 × syncInterval) is the IEEE-defined limit. If sync messages are missing,
   verify the grandmaster is running and the network path is intact.

## Sync gaps appear in the log (INFO "sync gap" or "fup gap")

These INFO messages appear when the gap between consecutive Sync or FollowUp messages
exceeds 1.4× the configured sync interval (the TI early-warning threshold).
They do **not** indicate a failure if `gmsync=true` and `offset ≈ 0 ns`.

Assessment:
- Gap < 375 ms (3 × 125 ms default sync interval): system is healthy.
- Gap consistently > 250 ms: monitor for network jitter or CPU load issues.
- Gap approaching 375 ms: investigate network quality or task preemption.
- `gmsync=false` with repeated gaps: critical — check for missing sync messages.

## `timeleap_past` errors in gPTP log

Cause: the computed clock rate was rejected as physically impossible (typically −750 Mppb
to −1 Bppb). This is almost always a symptom of CPU starvation of the gPTP task, not a
hardware oscillator fault.

Diagnosis: check whether other tasks are monopolizing the CPU and starving gPTP for extended
periods (~10 ms or more). Each such delay accumulates in the servo; after prolonged starvation
the servo tries to compensate with a rate that exceeds the rejection threshold.

Fix: ensure the gPTP task receives enough CPU time. Since gPTP requires less than 1% CPU
bandwidth, even a low-priority configuration is sufficient as long as no other task holds the
CPU continuously for more than a few milliseconds.

\cond SOC_AM62DX || SOC_AM275X
## AVB stream drops or audio glitches

1. Verify CBS shaper is configured for the correct stream bandwidth.
2. Verify presentation time offset is large enough to absorb network jitter.
3. Check that `gmsync=true` before AVB traffic starts — AVTP relies on a synchronized clock.
4. Confirm the listener is not being starved by a higher-priority task during DMA callbacks.
\endcond

## uniconf fails to start

1. Ensure the `yangemb` licensing library is linked in the application Makefile.
2. Check that `ucman_data_t.ucmode` is set to `UC_CALLMODE_THREAD | UC_CALLMODE_UNICONF`.
3. Check that enough FreeRTOS heap is available for the uniconf task stack and SimpleDB.

## How to enable logging to a buffer (avoid gPTP jitter from slow UART)

Log buffering is enabled by default (`TSN_USE_LOG_BUFFER=1` in `debug_log.h`). A dedicated
`log_task` flushes the ring buffer to UART at low priority, decoupling log I/O from the
gPTP task. To redirect log output, set the `consoleOutCb` field in `AppTsnCfg_t` before
calling `EnetApp_initTsnByCfg()`:

```c
AppTsnCfg_t appCfg = {
    .consoleOutCb = myLogOutputFn,  /* custom output; NULL = use default ConsolePrint */
};
EnetApp_initTsnByCfg(&appCfg);
```

---

# Known Limitations {#ENET_CPSW_TSN_LIMITATIONS}

| Limitation                                              | Detail                                                                                 |
|---------------------------------------------------------|----------------------------------------------------------------------------------------|
| Minimum gPTP sync interval: 7.8125 ms                   | Base timer tick is 7.8125 ms; sync intervals below this are not supported             |
| All interval parameters must be multiples of 7.8125 ms  | Non-multiple intervals produce inaccurate timing                                       |
| Maximum gPTP clock domains: 2                           | `GPTP_MAX_DOMAINS` supports 1 or 2; CMLDS mode required for 2-domain operation        |
| No persistent configuration (no filesystem)             | `ucman_data_t.dbname = NULL`; all YANG config reverts to defaults on reboot           |
| Single gPTP instance per SoC                            | Only one gPTP instance (`gptpInstanceIndex = 0`) supported in FreeRTOS builds         |

\cond SOC_AM62DX || SOC_AM275X
| Limitation                                              | Detail                                                                                 |
|----------------------------------------------------------|----------------------------------------------------------------------------------------|
| yangemb license limits AVB applications to 1 hour       | AVTP talker/listener stops after 1 hour; gPTP-only applications are unaffected        |
\endcond
