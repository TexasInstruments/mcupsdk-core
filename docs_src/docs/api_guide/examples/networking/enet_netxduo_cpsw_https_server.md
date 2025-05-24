# Enet NetxDuo CPSW HTTPS Server Example{#EXAMPLES_ENET_NETXDUO_CPSW_HTTPS_SERVER}

[TOC]

# Introduction

\note NetxDuo features are made available as is from public NetxDuo project. SDK configuration may only enable and exercise a subset of these features.

This example illustrates the configuration and usage of the NetX HTTPS web server addon, using TLSv1.3 with ECDSA signed certificates. 
The CPSW controller is used in MAC mode (single port).

This example does the following
- Initializes the ethernet driver for the underlying HW.
- Initializes the NetxDuo stack for TCP/UDP IP.
- Initializes NetX secure with a ECDSA (TLSv1.3) self-signed server certificate.
- Accepts a request from a web browser and sends a sample HTML page.

# Supported Combinations

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_threadx
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | source/networking/enet/core/examples/netxduo/enet_netxduo_cpsw_sa2ul_https_benchmark

Note: To run the example on any core other than r5fss0-0, user needs to change the DMA channel resource ownership accordingly using the resource partioning tool in \ref RESOURCE_ALLOCATION_GUIDE and build the new SBL.


# Configuring Syscfg

- Following Syscfg option allows flexibility to configure memory foot print based on required use case like: Number of DMA descriptors and buffering.

- Supported Options with default configuration

<table>
<tr>
    <th>Feature
    <th>Section
    <th>Description
    <th>Remarks/Default Setting
</tr>

<tr>
    <td>Mdio Manual Mode Enable
    <td>TI Networking / Enet (CPSW)
    <td>Flag to enable MDIO manual mode in example. Driver support for Manual mode is enabled, so this parameter configures manual mode in the example.
    <td>Default is true. If your silicon is affected with errata <a href="https://www.ti.com/lit/er/sprz457e/sprz457e.pdf" target="_blank">i2329— MDIO interface corruption</a>, then TI suggests to use MDIO_MANUAL_MODE as software workaround.
</tr>

<tr>
    <td>Disable Mac Port1, Disable Mac Port2
    <td>TI Networking / Enet (CPSW)
    <td>Select which port to disable.
    <td>Default is Port1 enabled. If both Port1 and Port 2 are enabled, any port can be used and  if operating in switch mode, it enables traffic switching between the two ports.
</tr>

<tr>
    <td>Enable Packet Pool Allocation
    <td>TI Networking / Enet (CPSW)
    <td>Flag to enable packet buffer memory allocation from enet utils library. It should be disabled to avoid utils memory wastage, in case application allots packet via other mechanism.
    <td>Default is true. If enabled size of pkt pool size depends on 'Large Pool Packet Size', 'Large Pool Packet Count', 'Medium Pool Packet Size', 'Medium Pool Packet Count', 'Small Pool Packet Size' and 'Small Pool Packet Count'. EnetMem_allocEthPkt API uses this memory to allocate the DMA Ethernet packet.
</tr>

<tr>
    <td>Only Enable Packet Info Allocation
    <td>TI Networking / Enet (CPSW)
    <td>Flag to allocate only the DMA Packet Info structures, this does not include the buffer memory. This is useful when the buffer memory is internally allocated by the application.
    <td>Default is true. If enabled "PktInfoMem Only Count" determines the number of additional DMA Packet Info structures allocated. EnetMem_allocEthPktInfoMem uses this memory to allocate empty DMA Packet Info structures.
</tr>

<tr>
    <td>Number of Tx Packet
    <td>TI Networking / Enet (CPSW) / DMA channel config
    <td>No of Tx packets required for DMA channel
    <td>Default is 16. The size of the NetxDuo packet pool will be automatically set at runtime to match the number of packets configured here.
</tr>

<tr>
    <td>Number of Rx Packet
    <td>TI Networking / Enet (CPSW) / DMA channel config
    <td>No of Rx packets required for DMA channel
    <td>Default is 32. It contributes to the size of Pkt Mem Pool, DMA ring buffer and accessories size. The size of the NetxDuo packet pool will be automatically set at runtime to match the number of packets configured here.
</tr>

<tr>
    <td>NetxDuo instance
    <td>TI Networking / NetxDuo
    <td>No of interfaces allocated by the example
    <td>Only one interface should be set to default when more than one interface is allocated.
</tr>
</table>



# TLS certificates used

For this example, we use self-signed ECDSA certificate generated using openSSL. The steps for generation of certificate are shown below.
First we generate the certificates and then convert it to DER format, which is further converted to hex.

## Generating the certificate and the private key

Both the self-signed certificate and private key are generated using

\code
$ openssl req -x509 -nodes -days 365 -new -newkey ec -pkeyopt ec_paramgen_curve:prime256v1 -keyout private_key.key -out self_signed_cert.crt
\endcode


## Converting to DER

The certificate is converted to DER using

\code
$ openssl x509 -outform der -in self_signed_cert.crt -out self_signed_cert.der
\endcode

The private key is converted to DER using

\code
$ openssl pkey -outform der -in private_key.key -out private_key.der
\endcode

## Output as a C array

The certificate and the private key are written out as a C array using

\code
$ xxd -i private_key.der
$ xxd -i self_signed_cert.der
\endcode

# Steps to Run the Example

## Build the example

- When using CCS projects to build, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- When using makefiles to build, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)

## HW Setup

\note Make sure you have setup the EVM with cable connections as shown here, \ref EVM_SETUP_PAGE.
      In addition do below steps.


### AM243X-EVM

#### For CPSW based example

- Connect a ethernet cable to the EVM from host PC as shown below

  \imageStyle{am64x_evm_lwip_example_00.png,width:30%}
  \image html am64x_evm_lwip_example_00.png Ethernet cable for CPSW based ethernet

### AM243X-LP

\note AM243X-LP has two ethernet Ports which can be configured as both CPSW ports.

#### For CPSW based examples

- Connect a ethernet cable to the AM243X-LP from host PC as shown below

  \imageStyle{am243x_lp_lwip_example_00.png,width:30%}
  \image html am243x_lp_lwip_example_00.png Ethernet cable for CPSW based ethernet


## Create a network between EVM and host PC

- The EVM will get an IP address using DHCP, so make sure to connect the other end of the cable
to a network which has a DHCP server running.

- To get started one can create a simple local network
  between the EVM and the host PC by using a home broadband/wifi router as shown below.
  Most such routers run a DHCP server

  \imageStyle{lwip_example_01.png,width:30%}
  \image html lwip_example_01.png Local network between PC and EVM

- To check the router connection with host PC, recommend to disconnect all other networking conenctions
  on the PC, sometimes you may need to disable firewall SW, and make sure the router is able
  to assign a IP address to your host PC

- After we run the example on the EVM (next step), the EVM will similarly be assigned a IP address, and then host
  can communicate with the EVM using the assigned IP address.

## Run the example

\attention If you need to reload and run again, a CPU power-cycle is MUST

- Launch a CCS debug session and run the example executable, see \ref CCS_LAUNCH_PAGE
- You will see logs in the UART terminal as shown in the next section.
- Note the IP address seen in the log, this is what we will use to communicate with the EVM.

## Steps to execute

1. Run example on EVM

2. Try to reach the EVM using ping as shown below, using a command shell on the host PC
    \code
    $ping 192.168.1.100
    \endcode
    "192.168.1.100" should be replaced with IP of EVM.

3. To start the client on host PC, open a web browser of your choice and enter url https://192.168.1.100/index.html.
   Replace 192.168.1.100 with the EVM IP assigned during run-time.
   The above has been tried with google chrome, microsoft edge, mozilla firefox.

4. Your browser will likely emit a security warning indicating that your are trying to connect to a server with an invalid certificate authority. This is normal as we use a self-signed certificate for testing purposes. You can safely ignore this warning.

5. The NetxDuo home page will load with the title "NetxDuo HTTPS web server"

6. To close the connection, simply close the tab in the web broswer.

## Sample output for HTTPs server

- Sample output of the target platform (NetxDuo HTTPS server)

\code

===============================
   CPSW NETXDUO HTTPS SERVER   
===============================
Enabling clocks!
EnetAppUtils_reduceCoreMacAllocation: Reduced Mac Address Allocation for CoreId:1 From 4 To 2 
Open MAC port 1
EnetPhy_bindDriver: PHY 0: OUI:080028 Model:23 Ver:01 <-> 'DP83867' : OK

Open MAC port 2
EnetPhy_bindDriver: PHY 3: OUI:080028 Model:0f Ver:01 <-> 'DP83869' : OK

PHY 0 is alive
PHY 3 is alive
Waiting for link up...
Waiting for link up...
Cpsw_handleLinkUp: Port 1: Link up: 1-Gbps Full-Duplex

MAC Port 1: link up
Waiting for link up...
Waiting for address from DHCP server on primary interface...
Local Interface IP is: 192.168.50.117
Socket created
Listening on port 443

\endcode

- Security warning sample from web browser

  \imageStyle{netxduo_https_browser_warning.png,width:80%}
  \image html netxduo_https_browser_warning.png Warning from the web browser about the self-signed certificate

- Sample output of web browser

  \imageStyle{netxduo_https_web_server_page.png,width:50%}
  \image html netxduo_https_web_server_page.png Client web browser connected to HTTPS server


## Troubleshooting issues


- If you see MAC address as `00:00:00:00:00:00`, likely you are using a very early Si sample which does not
  have MAC address "fused" in, in this case do below steps

   - Open file `source\networking\enet\soc\j7x\am64x_am243x\enet_soc.c`
   - Uncomment below line
        \code
        #define ENET_MAC_ADDR_HACK (TRUE)
        \endcode
   - Rebuild the libraries and examples (\ref MAKEFILE_BUILD_PAGE)

- If the execution hangs at `Waiting for address from DHCP server on primary interface...`
   - Make sure you see `MAC Port X: link up` message, if not check the ethernet cable
   - Check the local network and check if the DHCP server is indeed running as expected
   - When using a home broadband/wifi router, its possible to check the clients connected to the DHCP server via a web
     browser. Check your router user manual for more details.

- If you see "address cannot be reached" message in the client browser,
   - Make sure you see the `Enet IF UP Event` message, if not check the connection of router, host PC and EVM
   - Make sure a valid Local Interface IP is assigned.


# See Also
\ref NETWORKING
