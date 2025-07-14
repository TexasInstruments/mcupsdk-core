# Ethernet AVTP Example {#EXAMPLES_ENET_CPSW_AVTP}

[TOC]

# Introduction
This ethernet AVTP example illustrates the usage of IEEE Std 1722™-2016 stack with CPSW peripheral.

In this example, the DUT MAC port is connected to a PC which can act as either AVTP Talker or AVTP Listener. In the talker example, the DUT sends
audio stream in AVTP PCM format to PC, where PC logs the received PCM data in a output file. In the listener example, the PC streams the Audio from an input file and DUT receives the Audio data and prints the info to the console. Yang based configuration is also supported in the stack, However file System is not supported yet, hence full yang based support will be added in the future releases.

See also :\ref ENET_CPSW_AVTP

# Supported Combinations

\cond SOC_AM64X

Not support in this device

\endcond

\cond SOC_AM243X

Not support in this device

\endcond

\cond SOC_AM263X

Parameter      | Value
 ---------------|-----------
 CPU + OS       | mcu-r5fss0-0_freertos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | source/networking/enet/core/examples/tsn/aafpcmlistener_app, aafpcmtalker_app

\endcond

\cond SOC_AM273X

Not support in this device

\endcond

\cond SOC_AM275X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | source/networking/enet/core/examples/tsn/aafpcmlistener_app, aafpcmtalker_app

\endcond

\cond SOC_AM62DX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | mcu-r5fss0-0_freertos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | source/networking/enet/core/examples/tsn/aafpcmlistener_app, aafpcmtalker_app, dolbyec3talker_app, dolbyec3listener_app

\endcond
# Prerequisites
- EVM Board
- Cat6 ethernet cable
- PC with Linux Ubuntu OS with wireshark, xl4 deb build installed.
- ffplay on PC (Optional, can be used to listen to recorded audio)
<!-- - The test setup will be:
  \imageStyle{avtp_test_setup.png,width:80%}
  \image html avtp_test_setup.png AVTP test setup -->

# Steps to Run the Example
## To start AVB app on PC, follow below steps:
- Create a workspace to run avtp app.
```
$ mkdir avbrun_demo && cd avbrun_demo
$ avbrun -w rnn-cl -d enp3s0 -o setcaps=1
```
- This creates a file named <b>```avbrun.cfg```</b>. open the file and modify [DEFAULT] field, `uniconf_conf = uniconf.conf,streaminfo.conf`

- To use Excelfore avbrun as talker, modify ```avbrun.cfg``` to read/send AVTP data from wav file
\code
[Talker] `conl2t_extops: conl2t_extops = --src afile --mfile file_example_WAV_10MG.wav
\endcode

- To read/send AVTP data from wav file, modify this line in ```avbrun.cfg```
\code
conl2t_extops = --src afile --mfile BeepBeep.pcm --conf conl2_talker.conf -C 2 -P 180 -F s16be
\endcode
 Argument       | Description
 ---------------|-----------
 -C             | Channel number
 -F             | Format audio 16be
 -P             | Number of sample per frame

- To read/send AVTP data from ec3 file, modify this line in ```avbrun.cfg```
\code
conl2t_extops = --src afile --format aes32 --mfile in.ec3
\endcode

- To use Excelfore avbrun as listener and dump the output in PCM format, modify this line in ```avbrun.cfg```
\code
[Listener] conl2t_extops: conl2l_extops = --sink afile --mfile output.pcm -C 2 -F s16be
\endcode

- To dump the output in EC3 format, modify this line in ```avbrun.cfg```
\code
`conl2l_extops = --sink afile --mfile out.ec3 --sync-audio-file-type 2`: Dump to ec3 file
\endcode

- To play output.pcm `ffplay -f s16be -ar 48k -ac 2 output.pcm`.

- Create a file with name `streaminfo.conf` and paste the contents mentioned below.
In xl4 deb pakages from 7.0.7 version, there is new configuration file added to conl2 test app to check for stream information. Prepare streaminfo.conf with below content, replace <b>eno1</b> with the name of your network device and replace all instances of `stream-id:00-01-02-03-04-05:00-02` with your stream ID, This stream-id can be kept unchanged for the existing examples.
\code
/ieee802-dot1q-cnc-config/cnc-config/domain|domain-id:domain00|/cuc|cuc-id:br0|/stream|stream-id:00-01-02-03-04-05:00-02|/talker/end-station-interfaces|mac-address:00-01-02-03-04-05|interface-name:eno1|/accept 1
/ieee802-dot1q-cnc-config/cnc-config/domain|domain-id:domain01|/cuc|cuc-id:br0|/stream|stream-id:00-01-02-03-04-05:00-02|/listener|index:0|/end-station-interfaces|mac-address:00-01-02-03-04-05|interface-name:eno1|/accept 1
/ieee802-dot1q-cnc-config/cnc-config/domain|domain-id:domain00|/cuc|cuc-id:br0|/stream|stream-id:00-01-02-03-04-05:00-02|/talker/stream-rank/rank 1
../data-frame-specification|index:0|/ieee802-mac-addresses/destination-mac-address 91-E0-F0-00-FE-00
source-mac-address 00-01-02-03-04-05
../ieee802-vlan-tag/priority-code-point 3
vlan-id 110
/ieee802-dot1q-cnc-config/cnc-config/domain|domain-id:domain00|/cuc|cuc-id:br0|/stream|stream-id:00-01-02-03-04-05:00-02|/talker/traffic-specification/
max-frames-per-interval 1
max-frame-size 1400
interval/numerator 1
denominator 8000
/ieee802-dot1q-cnc-config/cnc-config/domain|domain-id:domain01|/cuc|cuc-id:br0|/stream|stream-id:00-01-02-03-04-05:00-02|/listener|index:0|/interface-configuration/interface-list|mac-address:00-01-02-03-04-05|interface-name:eno1|/config-list|index:0|/ieee802-mac-addresses/destination-mac-address 91-E0-F0-00-FE-00
source-mac-address 00-01-02-03-04-05
../ieee802-vlan-tag/priority-code-point 3
vlan-id 110
\endcode

- Create `uniconf.conf` file in the same folder and paste the following contents. Replace `eno1` with the name of your network device.
\code
/ietf-interfaces/interfaces/interface|name:eno1|/enabled true
\endcode

- For Talker, Create `conl2_talker.conf` and paste the following contents.
\code
/xl4-extmod/xl4conl2/CONL2_INSTANCE|INSTANCE_INDEX:0|/
CONF_NOSYNC false
CONF_AVTPD_BUFFTIME_MSEC 1000
CONF_SEND_AHEADTS_USEC 20000
CONF_AUDIO_PAYLOAD_TYPE "aaf"
CONF_ECHOBACK false
CONF_DROPLATE false
CONF_SINK_DEVICE_NAME  ""
CONF_SOURCE_AUDIO_CHANNELS  2
CONF_SOURCE_AUDIO_RATE  48000
CONF_SINK_FILE_NAME ""
CONF_SOURCE_FILE_NAME ""
CONF_SOURCE_WAVE_FREQ 500
CONF_SOURCE_AUDIO_SAMPLES 180
CONF_TSOFFSET_USEC 200000
CONF_AUDIO_LITTLE false
CONF_GPTP_SHMEM "/gptp_mc_shmeno1"
CONF_SHMEM_SUFFIX "avmt"
CONF_MEDIA_SOURCE "sinwave"
CONF_SOURCE_AUDIO_FORMAT "s24be"
\endcode

- For Listener, Create `conl2_listener.conf` and paste the following contents.
\code
/xl4-extmod/xl4conl2/CONL2_INSTANCE|INSTANCE_INDEX:1|/
CONF_NOSYNC false
CONF_AVTPD_BUFFTIME_MSEC 1000
CONF_SEND_AHEADTS_USEC 20000
CONF_AUDIO_PAYLOAD_TYPE "aaf"
CONF_ECHOBACK false
CONF_DROPLATE false
CONF_SINK_DEVICE_NAME  ""
CONF_SOURCE_AUDIO_CHANNELS  2
CONF_SOURCE_AUDIO_RATE  48000
CONF_SINK_FILE_NAME ""
CONF_SOURCE_FILE_NAME ""
CONF_SOURCE_WAVE_FREQ 500
CONF_SOURCE_AUDIO_SAMPLES 180
CONF_TSOFFSET_USEC 200000
CONF_AUDIO_LITTLE false
CONF_GPTP_SHMEM "/gptp_mc_shmeno1"
CONF_SHMEM_SUFFIX "avml"
CONF_SINK_AUDIO_FILE_TYPE "1"
CONF_MEDIA_SINK "aplay"
CONF_SOURCE_AUDIO_FORMAT "s24be"
\endcode

- To start the AVB Talker
```
avbrun -r talker
```
- To start the AVB Listener
```
avbrun -r listener
```

## Build the example

Refer \ref EXAMPLES_ENET_CPSW_TSN_GPTP to build the avtp examples.

## HW Setup

Refer \ref EXAMPLES_ENET_CPSW_TSN_GPTP for HW Setup.

## Create a network between EVM and host PC
EVM and PC has to be connected directly as shown below using CAT6 or CAT5 cable
  \imageStyle{gptp_topology_evm_pc.png,width:30%}
  \image html gptp_topology_evm_pc.png Local network between PC and EVM

PORT1 instead of PORT0 on EVM can be used as well.

## Run the example

\attention If you need to reload and run again, a CPU power-cycle is MUST

Refer :\ref ENET_CPSW_AVTP for how to configure AVTP parameters.

## AVTP verification

### DUT as pcmaaf_talker
Flash binary in @VAR_BOARD_NAME_LOWER and start it.

On the PC side, configure avbrun as conl2 listener(pcm) and start `avbrun -r listener`.

The log on DUT side should be printed:
```
INF:avtp:audio talker sent: 200.000000(packets/sec), 1.536000(mbps)
INF:avtp:audio talker sent: 200.000595(packets/sec), 1.536005(mbps)
INF:avtp:audio talker sent: 199.999786(packets/sec), 1.535998(mbps)
INF:avtp:audio talker sent: 200.000000(packets/sec), 1.536000(mbps)
```

On PC side, after running for a while, stop avbrun and confirm output.pcm is created.
Play the file with `ffplay -f s16be -ar 48k -ac 2 output.pcm`, the audio should be heard from speaker.

### Verification DUT as pcmaaf_listener
Flash binary in aafpcm_listener and start it.

On the PC side, configure avbrun as conl2 talker(pcm) and start `avbrun -r talker`.

On the DUT side, correct pcm info including bitdepth, channels, format and data rate should be printed.

\code
[RX 00:01:02:03:04:05:00:02] Rp#8 Mbps(1.536034, 1.536804) Delay(-23487, -11303894, -17810) Pkt(2413, 0, 0)
IFV:gptp:000369-441329:domainIndex=0, clock_master_sync_receive:the master clock rate to 97579ppb, GMdiff=-6nsec
IFV:gptp:000369-565787:domainIndex=0, clock_master_sync_receive:the master clock rate to 97429ppb, GMdiff=-72nsec
IFV:gptp:000369-690810:domainIndex=0, clock_master_sync_receive:the master clock rate to 97675ppb, GMdiff=46nsec
IFV:gptp:000369-817439:domainIndex=0, clock_master_sync_receive:the master clock rate to 97486ppb, GMdiff=-44nsec
IFV:gptp:000369-940807:domainIndex=0, clock_master_sync_receive:the master clock rate to 97839ppb, GMdiff=117nsec
IFV:gptp:000370-065716:domainIndex=0, clock_master_sync_receive:the master clock rate to 97623ppb, GMdiff=8nsec
IFV:gptp:000370-191183:domainIndex=0, clock_master_sync_receive:the master clock rate to 97642ppb, GMdiff=16nsec
IFV:gptp:000370-315754:domainIndex=0, clock_master_sync_receive:the master clock rate to 97846ppb, GMdiff=105nsec
[RX 00:01:02:03:04:05:00:02] Rp#9 Mbps(1.536211, 1.536745) Delay(-23490, -11303892, -17810) Pkt(2681, 0, 0)
IFV:gptp:000370-440775:domainIndex=0, clock_master_sync_receive:the master clock rate to 97900ppb, GMdiff=118nsec
    370.471s : CPU load =   8.20 %
IFV:gptp:000370-566110:domainIndex=0, clock_master_sync_receive:the master clock rate to 97677ppb, GMdiff=6nsec
IFV:gptp:000370-690721:domainIndex=0, clock_master_sync_receive:the master clock rate to 97796ppb, GMdiff=59nsec
domain=0, offset=0nsec, hw-adjrate=97796ppb
        gmsync=true, last_setts64=0nsec
IFV:gptp:000370-816837:domainIndex=0, clock_master_sync_receive:the master clock rate to 97877ppb, GMdiff=89nsec
IFV:gptp:000370-941093:domainIndex=0, clock_master_sync_receive:the master clock rate to 97951ppb, GMdiff=112nsec
IFV:gptp:000371-065656:domainIndex=0, clock_master_sync_receive:the master clock rate to 97736ppb, GMdiff=4nsec
IFV:gptp:000371-190631:domainIndex=0, clock_master_sync_receive:the master clock rate to 97719ppb, GMdiff=-4nsec
IFV:gptp:000371-316052:domainIndex=0, clock_master_sync_receive:the master clock rate to 97921ppb, GMdiff=87nsec
[RX 00:01:02:03:04:05:00:02] Rp#10 Mbps(1.536044, 1.536681) Delay(-23490, -11303894, -17810) Pkt(2949, 0, 0)
IFV:gptp:000371-440734:domainIndex=0, clock_master_sync_receive:the master clock rate to 97887ppb, GMdiff=62nsec
IFV:gptp:000371-566091:domainIndex=0, clock_master_sync_receive:the master clock rate to 97812ppb, GMdiff=22nsec
IFV:gptp:000371-691026:domainIndex=0, clock_master_sync_receive:the master clock rate to 97839ppb, GMdiff=32nsec
IFV:gptp:000371-816876:domainIndex=0, clock_master_sync_receive:the master clock rate to 97812ppb, GMdiff=16nsec
IFV:gptp:000371-940623:domainIndex=0, clock_master_sync_receive:the master clock rate to 97836ppb, GMdiff=25nsec
IFV:gptp:000372-066382:domainIndex=0, clock_master_sync_receive:the master clock rate to 97957ppb, GMdiff=76nsec
IFV:gptp:000372-190569:domainIndex=0, clock_master_sync_receive:the master clock rate to 98008ppb, GMdiff=91nsec
IFV:gptp:000372-315571:domainIndex=0, clock_master_sync_receive:the master clock rate to 98019ppb, GMdiff=86nsec
[RX 00:01:02:03:04:05:00:02] Rp#11 Mbps(1.536200, 1.536641) Delay(-23490, -11303881, -17810) Pkt(3217, 0, 0)
\endcode
# See Also

\ref NETWORKING |
\ref ENET_CPSW_AVTP
