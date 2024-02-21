# PCIE EP Enumeration {#EXAMPLES_DRIVERS_PCIE_ENUMERATE_EP}

[TOC]

# Introduction

The PCIe Enumeration (EP) example demonstrates an EP that supports enumeration through an RC that is running Windows or Linux.

The EP waits for enumeration and configuration through the RC. It offers a BAR0 memory region with a control and status register interface through which three tests can be executed:
- RC fills a data buffer in the EP's BAR0 memory region and configures the address of a DMA buffer. The EP copies the data from the BAR0 data buffer back to the RC's DMA buffer.
- EP triggers each configured message signaled interrupt in the RC in ascending order.
- RC fills data buffers in the EP's BAR1 and BAR2 memory regions with a known pattern. The EP verifies the content of its BAR1 and BAR2 memory regions and in case of successful verification it sends an MSI back to the RC.


# Supported Combinations

\cond SOC_AM64X || SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/pcie/pcie_enumerate/pcie_enumerate_ep

\endcond

# Steps to Run the Example

## Build the example

- When using CCS projects to build, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- When using makefiles to build, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)

## HW Setup

\note Make sure you have setup the EVM with cable connections as shown here, \ref EVM_SETUP_PAGE.
      In addition do below steps.

\cond SOC_AM64X || SOC_AM243X

\cond SOC_AM243X
### AM243-EVM
\endcond
\cond SOC_AM64X
### AM64X-EVM
\endcond

- For connecting a board in EP mode and a Windows or Linux PC a specialized
  cable as below is required
    \imageStyle{pcie_cable.png, width:50%}
    \image html pcie_cable.png
- This cable can be obtained from Adex Electronics (https://www.adexelec.com).
- PCs (more specifically the PCIe electromechanical spefication) mandates
  an add-in card (the board in EP mode in this case) to use the reference
  clock on the connector. This requires modifications to the EVM:
  - use a TMDS243EVM or TMDS64EVM Rev. C
  - remove Resistors R661, R662, R667 & R668
  - populate Resistors R665, R666, R679 & R680 (all 0 ohm)
- \note Contrary to other examples, this example requires the CK+ and CK- lines of the PCIe cable to be present.
- The jumper J34 on the EVM needs to be disconnected, as we want neither
  the AM24x to driver the PERST signal (we're an EP, this is an input)
  nor do we want the x86's PERST signal to reset our processor, because
  we want to boot the AM24x BEFORE the x86 to make sure the startup and
  reset timing requirements are met.
\endcond

## RC Setup

This example assumes use of Windows or Linux on an x86 PC as the RC.
Only specific versions of Windows have been Linux have been tested, though
others should works as well, but might require minor changes.

While the EP application should work with every x86 based system, there
could be differences especially in how the BIOS handles PCIe cards. The
driver has been tested on the following hardware:

- Intel 10th Generation Core i3-10100 on MSI MAG Z490 mainboard
- "UP Squared Pro 7000" with an Intel Atom x7425E

### Windows

On the Windows target system, the following software needs to be installed:
- Windows 10 22H2
- WDK for windows 10, version 2004
  https://learn.microsoft.com/en-us/windows-hardware/drivers/other-wdk-downloads
- Windows SDK 10.0.19041.685
  https://developer.microsoft.com/en-us/windows/downloads/sdk-archive/
- Microsoft Visual C++ Redistributable from
  https://learn.microsoft.com/en-us/cpp/windows/latest-supported-vc-redist?view=msvc-170#visual-studio-2015-2017-2019-and-2022

In order to build the Windows driver that runs on the RC to talk to the
EP, the following software needs to be installed:

- WDK for windows 10, version 2004
  https://learn.microsoft.com/en-us/windows-hardware/drivers/other-wdk-downloads
- Windows SDK 10.0.19041.685
  https://developer.microsoft.com/en-us/windows/downloads/sdk-archive/
- Visual Studio 2019 (Professional or Community edition)
  - spectre mitigation libraries need to be added as an individual
    component using Visual Studio's installer
- ti-sample-kmdf and ti-sample-console source code


### Linux

On the Linux target system, the following distributions and versions have
been tested:
- Ubuntu 20.04 with Kernel 5.15.0-91-generic
- Debian 12 with Kernel 6.1.0-10-amd64

The Linux kernel needs to include the following configuration options:

The kernel version currently running can be verified with the uname utility:

\code

uname -a
Linux debian 6.1.0-17-rt-amd64 #1 SMP PREEMPT_RT Debian 6.1.69-1 (2023-12-30) x86_64 GNU/Linux

\endcode

Kernel options used to compile the running kernel are usually available
in a file under /boot/*kernel_version* or at /proc/config.gz.
The following kernel options are required:
- CONFIG_VFIO, CONFIG_VFIO_PCI
- CONFIG_IOMMU, CONFIG_INTEL_IOMMU
On AMD based systems, CONFIG_AMD_IOMMU is used instead of CONFIG_INTEL_IOMMU.

On Intel based systems, the IOMMU usually needs to be enabled by adding
the parameter intel_iommu=on to the kernel command line. This can be done
by modifying the file /etc/default/grub:

\code

GRUB_CMDLINE_LINUX="intel_iommu=on"

\endcode

On AMD based systems, the IOMMU is enabled by default.

## Run the example
- Make sure the Windows or Linux PC is powered off
- Connect the Windows or Linux PC and the EVM with the PCIe cable
- Launch a CCS debug session and run the example executable, see CCS Launch, Load and Run

  After starting the example application, only the following status is printed
  on the UART terminal:

  \code
  PCIe: EP initialized and waiting for link
  \endcode

- Power on the Windows or Linux PC

  After powering up the RC system, the following output should be visible
  on the UART terminal:

  \code
  PCIe: link detected
  PCIe Link Parameter: PCIe Gen1 with 2.5 GT/s speed, Number of Lanes: 1
  EP is in D0 state
  PCIe: signaling APPL ready
  APPL: pcie ready
  \endcode

  The initial link parameters depend on the behavior of the RC system. With
  some systems, the initial link is already established at 5 GT/s, while other
  systems establish a link at 2.5 GT/s and upgrade the link to 5 GT/s upon
  the call to #Pcie_cfgEP().

At this point, different instructions apply to Windows and Linux based RC systems.

### Windows

#### Building for Windows

Before trying to build the ti-sample-kmdf driver, make sure that your
Visual Studio installation and Microsoft's SDK are all correctly installed
by trying to build a template KMDF driver according to Microsoft's
instructions:

https://learn.microsoft.com/en-us/windows-hardware/drivers/gettingstarted/writing-a-kmdf-driver-based-on-a-template

The ti-sample-kmdf solution contains two projects, the kernel mode driver
ti-sample-kmdf and a console application ti-sample-console.
Both projects can be built by opening the ti-sample-kmdf solution in
Visual Studio 2019 and building the entire solution.

- The driver currently only support 64-bit mode on x86 machines, so the
  platform "x64" should be selected, along with the configuration "Release".

- In the solution explorer select the solution "ti-sample-kmdf" and build
  the solution.

The build output will be located in a new folder "x64\\Release" below the
solution directory (ti-sample-kmdf\\x64\\Release).

#### Deploying on Windows

The following files from the build output folder need to be copied to
the target machine:

- ti-sample-kmdf.inf
- ti-sample-kmdf.sys
- ti-sample-kmdf.cat
- ti-sample-kmdf.cer
- ti-sample-console.exe

Windows by default only accepts signed drivers. An installation can be
modified to accept socalled test signed drivers. The Windows KMDF sample
driver ti-sample-kmdf uses this approach and is built as a test signed
driver.

In order to allow Windows to use test signed drivers, open an
administrator prompt (cmd, Run as administrator) and enter the following
command:

\code
Bcdedit.exe -set TESTSIGNING ON
\endcode

After enabling test signing the system needs to be rebooted. At this point
the AM24x EVM with the pcie_enumerate_ep application should be started
as described above.

You then need to install the certificate used to test sign the driver on
the target computer. This certificate is placed in the solution output
folder along with the driver and is named ti-sample-kmdf.cer.

It can be installed using the CertMgr.exe tool that comes with the WDK
from an administrator prompt:

\code
cd C:\\Program Files (x86)\\Windows Kits\\10\\bin\\10.0.19041.0\\x64\\

CertMgr.exe /add ti-sample-kmdf.cer /s /r localMachine root /all
CertMgr.exe /add ti-sample-kmdf.cer /s /r localMachine trustedpublisher
\endcode

(use full path to ti-sample-kmdf.cer)

**This CertMgr.exe is different from the certmgr that opens when you try to just run CertMgr on an administrator command line.**

With the certificate installed you can install the driver for the device
using the Windows device manager. Look for an unknown PCI device and verify
the hardware ID is PCI\\VEN_17cd&DEV_0100. Right-click the device, select
"Browse my computer for drivers" and select the folder containing the
solution build output.

Windows should install the driver and inform you that it finished
installing the driver for the ti-sample-kmdf device.

Alternatively, the driver can also be deployed from within Visual Studio.
This latter approach is more convenient for development as it works from
within the development environment and includes additional test steps.
Follow the Microsoft documentation at
https://learn.microsoft.com/en-us/windows-hardware/drivers/gettingstarted/writing-a-kmdf-driver-based-on-a-template
and
https://learn.microsoft.com/en-us/windows-hardware/drivers/develop/deploying-a-driver-to-a-test-computer
for further instructions.

#### Usage on Windows

In order to make use of the driver the ti-sample-console application can
be run. This application opens the driver, sends IOCTLs to the driver,
and waits for the IOCTLs to return.

The KMDF driver uses the "pattern" sent with the IOCTL to fill the Bar0
data area of the EP and then triggers a "downstream" interrupt in the EP.
The EP copies the data area from its Bar0 to the RC driver's DMA buffer
and triggers an MSI in the RC. The RC handles this interrupt and replies
to the IOCTL with the data sent back via DMA.

### Linux

#### Building for Linux

- The ti-sample-vfio example driver consists of a single C file that can
  be compiled on-target using a simple GCC command:

  gcc ti-sample-vfio.c -o ti-sample-vfio -g -O2

#### Deploying on Linux

- Since the driver can be compiled self-hosted on Linux there's no separate
  deployment step.

#### Usage on Linux

- Get root privileges:

  sudo su -

- Make sure the AM24x PCIe EP is correctly enumerated by the Linux OS
  and note the bus, device and function assigned to the EP:

  \code
  lspci -vt

  -[0000:00]-+-00.0  Intel Corporation Device 4679
             ...
             +-1d.0-[03]----00.0  Cadence Design Systems, Inc. Device 0100
  \endcode

  **In this example, our device is connected to the RC at 0000:00:1d.0 and was assigned address 0000:03:00.0 (bus 3, device 0, function 0).**

- Load the VFIO-PCI driver:

  \code
  modprobe vfio-pci
  \endcode


- Assign the pcie_enumerate_ep sample's vendor and device ID to the VFIO
  driver:

  \code
  echo "17cd 0100" > /sys/bus/pci/drivers/vfio-pci/new_id
  \endcode

  At this point the EP is put into D3hot state by the Linux VFIO driver:

  \code
  PCIe: power state entry
  EP is in D3hot state
  PCIe: signaling APPL halt
  APPL: pcie not ready
  \endcode


- Check which IOMMU group the EP was assigned to:

  \code
  readlink /sys/bus/pci/devices/0000:03:00.0/iommu_group

  ../../../../kernel/iommu_groups/16
  \endcode

  **our device is in IOMMU group 16**

- Make sure the EP is the only device in this IOMMU group:

  \code
  ls -l /sys/bus/pci/devices/0000:03:00.0/iommu_group/devices

  lrwxrwxrwx 1 root root 0 17. Nov 10:57 0000:03:00.0 -> ../../../../devices/pci0000:00/0000:00:1d.0/0000:03:00.0
  \endcode

  **our device is the sole device in this IOMMU group**

- **if there are other devices listed as part of this IOMMU group, you
  need to try a different PCIe slot on your x86 target.**

  **If that doesn't help, you could try loading the vfio_pci driver for
  all devices in this IOMMU group, provided of course that these devices
  aren't needed for normal operation.**

  **See IOMMU background at the end of this page for further information.**

- Start the ti-sample-vfio application with bus, device, function and
  the IOMMU group as parameters:

  \code
  ./ti-sample-vfio 3 0 0 16
  \endcode


- Optionally specify the number of MSI vectors to allocate (multiple message
  enable):

  \code
  ./ti-sample-vfio 3 0 0 16 8
  \endcode


- Additionally you can specify the number of loops the DMA copy test should
  execute:

  \code
  ./ti-sample-vfio 3 0 0 16 8 1000
  \endcode


- In order to debug the EP application, the RC application optionally waits
  for user input at certain locations, e.g. allowing the user to observe
  register content:

  \code
  ./ti-sample-vfio 3 0 0 16 8 1000 wait
  \endcode

  Running the sample application puts the device from D3hot into D0 state.
  The application outputs further state changes while the sample executes
  until finally the EP is put back into D3hot state.

  See below for what the output should look like when executing the Linux
  example.

# Important concepts in the example

The PCIe Enumeration (EP) example demonstrates some important concepts for
EP applications when used in conjunction with "typical" host systems based
on e.g. x86 Linux and Windows.

## x86 RC requirements

x86 systems typically assume that PCIe devices are built-in and thus available
at power-up of the x86 system. Some x86 implementations also support hot-plug
PCIe devices that can be connected at runtime.

Connecting the AM24x as an EP to an x86 as a RC needs to follow PCIe
specification requirements in order to be properly detected by the x86
system's BIOS.

### Startup requirements, reset behavior

The PCI express specification requires an endpoint device to enter link
training within 20 ms of deassertion of the PERST# signal.

The PERST# signal is guaranteed to be active for at least 100 ms after
power supplies are stable for “standard” PCI express cards. In effect
this means that a device needs to enter link training within 120 ms from
power supplies becoming stable. The time it takes for the AM24x to boot
up to a point where it can enter PCIe link training depends on many
variables.

- A) In order to ensure startup timing requirements are met, an embedded
system consisting of an x86 and an AM24x could release the AM24x's reset
before releasing the x86's reset. This allows the AM24x to complete
startup before the x86 releases its PERST signal.

- B) If that's not an option a system might optimize bootup of the AM24x
application to ensure that the PCIe EP driver is ready to enable link
training within the allotted time frame. Such optimization is outside of
the scope of this example

- C) If the x86 system supports hot-plugging of PCIe devices, the AM24x EP
can boot up in parallel with the x86, but the BIOS and the operating
system need to support this.

This example ensures the timing conditions are met by powering up the EP
before powering up the RC, thus resembling the first approach, where the
AM24x boots before the x86.


### Reference clock

There are different schemes for distributing the PCIe reference clock
in a system.

x86 usually systems assume a common reference clock topology where the
PCIe RefClk is provided by the x86 on the PCI express card connector.
The EP is expected to use that reference clock. If the x86 uses spread
spectrum clocking (usually it does) the same frequency spread is applied
to the EP and the RC, ensuring stable communication.

With separate reference clocks there exist two options:

Separate reference clock with no spread (SRNS): Both RC and EP use a
separate reference clock, and there's no spread spectrum clocking
applied.

Separate reference clock, independent spread (SRIS): Both RC and EP use
a separate reference clock both with individual spread spectrum clocking.
This mode of operation is supported by the AM24x but support within x86
is usually not "user" configurable.

On the AM24x EVM starting with revision A there is a clock generator IC
that supplies the reference clock to both the AM24x (where RefClk thus
acts as an input) and to the PCIe connector. A x86 also supplies its
reference clock on its PCIe connector.

There are two options to handle this situation:

- A) Rework the AM24x EVM to directly connect the AM24x RefClk pins to the
  PCIe connector and use a PCIe male-to-male cable that connects the
  RefClk pins. This is the recommended approach.

- B) Leave the AM24x EVM as-is and use a PCIe male-to-male cable where the
  RefClk pins are /not/ connected. This results in a SRNS/SRIS setup
  which may or may not work.


#### External reference clock implications

The reference clock is used as the PCIe core clock on the AM24x, and
without that clock running certain registers of the AM24x's PCIe
peripheral are inaccessible (PCIE0_CORE_DBN_CFG_PCIE_CORE). For the
generic use case we need an external reference clock, but that in turn
means the EP needs to support working even if the reference clock is
not (yet) available, if only to allow powering the EP before powering
the RC.

### EP->RC and RC->EP interrupt signaling

Several mechanisms exist for the EP to signal events back to the RC.
Traditionally PCI devices used to be able to trigger one of four interrupt
lines (INTA-D) that allowed interrupts to be raised in the RC's CPU.

Later versions of PCI and all PCIe devices added support for message
signaled interrupts (MSI), where the RC tells the EP an address and a
value that the EP should send to the RC in order to raise an interrupt.

PCIe doesn't specify a means for an RC to raise an interrupt in an EP,
as EPs are usually "hardware", where every incoming access can trigger
any kind of response.

When using a CPU to implement the EP functionality on the other hand,
it is desirable to allow the RC to send interrupts to the EP in order to
inform the EP's CPU that some action is desired. On the AM24x/AM64x this
can be achieved by tryingger a downstream interrupt in the EP, which
causes an interrupt to be delivered to the EP's CPU (e.g. R5f).

An RC can trigger a downstream interrupt by setting bit 8 (0x100) in the
EP's vendor specific control register in the EP's configuration space.


## Software concepts

### Driver startup

In order to be able to support scenarios where the PCIe reference clock
isn't immediately available upon startup of the AM24x, the PCIe EP driver
initializes only the bare minimum in order to be able to start link
training during Pcie_open (called from Drivers_open).

The application then needs to wait (poll) for a PCIe link via #Pcie_isLinkUp().
Once a link is established, we can assume that the reference
clock is available and configure the rest of the PCIe EP. In order to
avoid a race condition with the RC which might be trying to enumerate the
EP as soon as a link is established, the EP driver configures the device to respond
with CRS completions (configuration request retry status) to config
requests until the EP finished initialization, at which point #Pcie_cfgEP()
is called to enable the EP to properly respond to configuration requests.

The PCIe specification calls for system software (x86 BIOS) to wait at
least one second while the device responds with CRS completions. Care
must be taken if the EP keeps responding with CRS completions for too
long, as this might not be handled gracefully in x86 BIOS implementations.


### Memory map

#### Local memories

If an endpoint wants to expose part of its local memories via a PCIe BAR
it needs to add "Inbound Address Translation" entries. The target address
for an inbound ATU entry is the address in the SoC address space. This
address needs to be aligned to the size of the memory region.

This example allocates a dedicated memory section in the linker file to
place the bar0 memory. The bar0 memory includes a configruation area
(struct config) and a memory buffer (uin32_t data[]). The configuration
area is used as a control and status register space for the RC to configure
the EP and for the EP to respond with status information.

Since accesses to that memory region from the RC are not coherent with
the R5f caches, a MPU region (CONFIG_MPU_REGION7) is configured to map a
part of the bar0 memory as strongly ordered memory, thus aleviating the
need to manually maintain cache coherency.

In order to improve performance for the data buffers these are still
accessed as normal, cacheable memory, and thus require the use of #CacheP_inv()
to manually invalidate cache content before and after accessing the data
buffer.


#### DMA memory / outbound mappings

If an endpoint wants to access part of the RC's memory via DMA it needs
to create an outbound mapping. Since the PCIe address of that memory is
at the discretion of the RC this mapping can't be created at
compilation or initialization time but rather needs to be created at
runtime after the RC informed the EP about the location of the RC's DMA
buffer.

This example implements two "registers" to allow the RC
to specify the address and length of a DMA buffer and informs the EP
via downstream IRQ (RC->EP) that this configuration has been performed.
The EP then dynamically maps the RC's DMA buffer via an outbound mapping.

Outbound mappings are currently only supported via the PCIE0_DAT0 window.
Where outbound mappings are located within this 128 MB area is up to the
application. Outbound mappings need to align both the base address (local
SoC address) and the target address (PCIe address) to the size of the
region. Since the PCIe bus address of the RC's DMA buffer is at the RC's
discretion there can't be any assumptions about its alignment.

The example uses one 64 MB area starting at 0x68000000
(start of PCIE0_DAT0) to map the RC's DMA buffer. Due to the unknown
alignment the EP can therefore map a maximum of 32 MB in that area.

The example uses message signaled interrupts which also
requires an outbound mapping. The example uses a 256 byte region at the
end of the PCIE0_DAT0 address space (0x6fffff00) to map the RC's MSI
target address.


### Driver runtime operation

Since the driver initializes only the bare minimum during Pcie_open,
the remaining configuration needs to be performed at runtime. In order
to allow an application maximum flexibility in how it uses the AM24x
PCIe EP's feature, the driver provides only passive functions to read
status and RC configured settings and to modify the EP's configuration.

The application needs to poll for hardware status changes or handle
interrupts in order to track the state of the PCIe interface, and then
call the appropriate configuration functions.

The pcie_enumerate_ep sample application acts on the following hardware
events:

- Link up detection (polled, since there is no IRQ generated for this
  event
- Link down detection (IRQ triggered)
- Power management state entry (IRQ triggered)
- Power management return to active state "D0" (polled, since there is no
  IRQ generated for this event)
- PCIe hot-reset (IRQ triggered)
- "Downstream" interrupts (IRQ triggered)

The pcie_enumerate_ep sample application acts on the following software
events (all triggered via a downstream IRQ and action flags in the ctrl
"register"):

- Configuration Done
- Reset
- DMA Copy Request (copies from Bar0 to RC's DMA buffer)
- MSI Request (sends every enabled MSI interrupt once)
- BAR1/2 Request (verifies Bar1/Bar2 content and sends a MSI)

#### Pcie_onLinkDetect

Once the EP application detected a link it can rely on the PCIe RefClk
being available. The example application then calls Pcie_onLinkDetect().
That function can perform any additional application specific configuration
(in this example we set the "slot clock configuration" to indicate we're
using the RefClk from the connector) and then calls #Pcie_cfgEP to finalize
the EP configuration.

# Background information

## PCIe reference clock

PCIe systems usually use a 100 MHz reference clock. On the AM24x, this
reference clock can be generated internally or it can be sourced externally.
If the reference clock is generated internally, it can optionally be
output on the RefClk pins.

In the scope of the PCIe standard, use of a common reference clock is
optional, provided that both systems use a reference clock that ensures
certain jitter thresholds. The PCI express card electromechanical
specification that describes the type of PCIe connectors typically found
on PC mainboards mandates the use of a common reference clock.

The PCIe reference clock is often modulated to reduce electromagnetic
interference, so called "spread spectrum clocking". Use of spread spectrum
clocking requires both link partners to be able to tolerate much higher
timing differences than what the PCIe specification requires for reference
clock stability.

Setups with a common reference clock usually have no problems when spread
spectrum clocking is enabled. Setups without a common reference clock
and with spread spectrum clocking enabled are called "SRIS" (separate
reference, independent spread). Setups without a common reference clock
and no spread spectrum clocking are called "SRNS" (separate reference,
no spread).

The PCIe driver allows the AM24x to be configured for every possible
reference clock configuration, but x86 systems often rely on a common
reference clock (in accordance with the PCI express card specification).
On some systems, spread spectrum clocking can be disabled, allowing a
SRNS type setup. Other systems might support SRIS, but this usually needs
to be configured in the x86 BIOS, and is often not configurable at all.


## IOMMU background

An IOMMU sits as an MMU between memory accesses coming from DMA capable
devices and the system's main memory. With the IOMMU active, the PCIe
addresses seen by the EP are no longer actual physical addresses but rather
logical addresses that are valid only for this device (or rather this IOMMU
group).

This has several benefits, for example it allows the host's operating system
to map physically not contiguous memory buffers as a logically contiguous
buffer for the device. Another benefit is that a device can only access
the system memory that is explicitly intended for the device to use, providing
extra security to a system.
The Linux VFIO framework requires an IOMMU to be present in order to prevent
a user space process from accessing arbitrary parts of system memory by
means of a DMA capable device. There is a special No-IOMMU mode in VFIO
but use of this mode is not supported by this sample.

The granularity with which an IOMMU can differentiate between devices depends
on the particular system. Linux uses a concept of IOMMU groups where a single
group holds all the devices that need to use the same IOMMU mapping.
For VFIO there must not be any other devices in the same IOMMU group as
the AM24x EP running the sample application.

Devices directly connected to root ports of an Intel system can be expected
to be assigned to an individual IOMMU group.

See https://docs.kernel.org/driver-api/vfio.html for more details.


# Example Output

## Linux

When running the Linux RC sample driver ti-sample-vfio, the output on the
Linux terminal should be similar to this:

\code
sudo ./ti-sample-vfio 3 0 0 16 8 5
--------------------------------------------------------------------------------
Starting PCIe RC VFIO test application with parameters:
PCIe Bus Number: 3
PCIe Device Number: 0
PCIe Function Number: 0
PCIe IOMMU Number: 16
Test mode: default test
MSI IRQ number: 8
Iteration number: 5
--------------------------------------------------------------------------------
Using PCI device 0000:03:00.0 in IOMMU group 16
VFIO_CHECK_EXTENSION VFIO_TYPE1_IOMMU: Present
VFIO_CHECK_EXTENSION VFIO_NOIOMMU_IOMMU: Not Present
Config region info: region index 0x7, size 0x1000, offset 0x70000000000, cap_offset 0x0, flags 0x3
BAR0 Info: size 0x8000, offset 0x0, flags 0x7
MSI IRQ Info: index: 1, count: 16, flags: 9
RC completed EP initialization
--------------------------------------------------------------------------------
Start COPY test
COPY test passed with 5 loops
--------------------------------------------------------------------------------
Initialize MSI test. Expect 8 distinct MSI IRQs
Expect MSI IRQ nr. 0
Expect MSI IRQ nr. 1
Expect MSI IRQ nr. 2
Expect MSI IRQ nr. 3
Expect MSI IRQ nr. 4
Expect MSI IRQ nr. 5
Expect MSI IRQ nr. 6
Expect MSI IRQ nr. 7
MSI test passed
--------------------------------------------------------------------------------
Initialize BARs test
BAR1 Info: size 0x100000, offset 0x10000000000, flags 0x7
BAR2 Info: size 0x100000, offset 0x20000000000, flags 0x7
BAR test passed
--------------------------------------------------------------------------------
RC resets EP
\endcode

During the execution of the ti-sample-vfio test program, the following
output should be visible on the EVM's UART:

\code
EP is in D0 state
PCIe: signaling APPL ready
APPL: pcie ready
PCIe: lost PCIe link
PCIe: hot reset detected
PCIe: signaling APPL halt
APPL: pcie not ready
PCIe: link detected
PCIe link parameter: PCIe Gen2 with 5.0 GT/s speed, Number of Lanes: 1
PCIe: signaling APPL ready
APPL: pcie ready
PCIe: MSI enabled with 8 vector(s) using address fee00618 and data 0
Mapping MSI target at 0xfee00600 - size 0xff...
Mapping DMA buffer at 0x0 - size 0xffff...
APPL: EP configured
DMA test done
DMA test done
DMA test done
DMA test done
DMA test done
Send MSI Irq Nr. 0
Send MSI Irq Nr. 1
Send MSI Irq Nr. 2
Send MSI Irq Nr. 3
Send MSI Irq Nr. 4
Send MSI Irq Nr. 5
Send MSI Irq Nr. 6
Send MSI Irq Nr. 7
MSI test done
BAR test done
Disabling MSI mapping...
Disabling DMA buffer mapping...
APPL: EP unconfigured
PCIe: lost PCIe link
PCIe: hot reset detected
PCIe: signaling APPL halt
APPL: pcie not ready
PCIe: link detected
PCIe Link Parameter: PCIe Gen2 with 5.0 GT/s speed, Number of Lanes: 1
PCIe: signaling APPL ready
APPL: pcie ready
PCIe: power state entry
EP is in D3hot state
PCIe: signaling APPL halt
APPL: pcie not ready
\endcode

## Windows

When running the Windows RC sample driver ti-sample-kmdf along with the
Windows console application ti-sample-console, the output on the Windows
terminal should be similar to this:

\code
--------------------------------------------------------------------------------
Starting PCIe RC KMDF test application
--------------------------------------------------------------------------------
Opening windows kernel mode driver \\.\SampleTI
--------------------------------------------------------------------------------
Start COPY test
IOCTL_TISAMPLEKMDF_TEST_DMA returned data, verifying...
COPY test passed
--------------------------------------------------------------------------------
Start MSI test
IOCTL_TISAMPLEKMDF_TEST_MSI returned, result: 0000ffff
MSI test passed
--------------------------------------------------------------------------------
Start Bar1/2 test
IOCTL_TISAMPLEKMDF_TEST_BARS returned, result: 00000001
BAR test passed
--------------------------------------------------------------------------------
Closing windows kernel mode driver
KMDF test application done
--------------------------------------------------------------------------------
\endcode

During the execution of the ti-sample-console test program, the following
output should be visible on the EVM's UART:

\code
DMA test done
Send MSI IRQ nr. 0
Send MSI IRQ nr. 1
Send MSI IRQ nr. 2
Send MSI IRQ nr. 3
Send MSI IRQ nr. 4
Send MSI IRQ nr. 5
Send MSI IRQ nr. 6
Send MSI IRQ nr. 7
Send MSI IRQ nr. 8
Send MSI IRQ nr. 9
Send MSI IRQ nr. 10
Send MSI IRQ nr. 11
Send MSI IRQ nr. 12
Send MSI IRQ nr. 13
Send MSI IRQ nr. 14
Send MSI IRQ nr. 15
MSI test done
BAR test done
\endcode

# See Also

\ref DRIVERS_PCIE_PAGE
