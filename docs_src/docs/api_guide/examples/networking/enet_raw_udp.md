# Enet Static UDP Example {#EXAMPLE_ENET_CPSW_RAW_UDP}

[TOC]

# Introduction

This example demonstartes the UDP-Lite (Limited Checksum coverage) sent API from the DUT. The source and destinetion IP addresss and ports are fixed. No DHCP is used. This Example just fills ethernet packet with data (random data for testing) and sends it to specific IP & port.<br>

As the IP addresses and ports  are fixed the header of the every packet is identical thus the combined Ethernet , IPv4 and UDPLite header are created just once before sending any frame.
After that using this header and actual payload an entire Ethernet Frame is constructed and send over the link to the Linux machine.


# Supported Combinations

\cond SOC_AM273X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER
 Example folder | source/networking/enet/core/examples/enet_rawudp

\endcond

# Testing
Since Netcat does not support UDP-lite, use the following C code snippet to verify the Packet reception. Alternatively a Packet capture application like wireshark can be used to observe the packet reception

-------------------------------------------------------------------------------------------

```c
#include <stdio.h> 
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>

int main(int argc, char **argv) 
{
	int port;
	if (argc >= 2) 
	    port = atoi(argv[1]);
    else
    {
        perror("pass PORT NO on Command line");
        exit(1);
    }

	int s = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDPLITE);
	if (s < 0)
	{ 
	    perror("socket"); 
	    return 1; 
	}

	int cscov = 8; 

	struct sockaddr_in sa;
	memset(&sa,0,sizeof(sa));
	sa.sin_family = AF_INET;
	inet_pton(AF_INET, "10.5.5.26", &sa.sin_addr);
	sa.sin_port = htons(port);

	if (bind(s, (struct sockaddr*)&sa, sizeof(sa)) < 0)
	{ 
	    perror("bind"); 
	    return 1; 
	}

	printf("UDPLite server listening on port %d (fd=%d). CSCOV=%d\n", port, s, cscov);

	while (1) {
	    char buf[2048];
	    struct sockaddr_in peer;
	    socklen_t plen = sizeof(peer);
	    ssize_t n = recvfrom(s, buf, sizeof(buf)-1, 0, (struct sockaddr*)&peer, &plen);
	    if (n < 0) 
	    { 
	        perror("recvfrom");
	        continue;
	    }
	    buf[n] = 0;
	    char addrbuf[INET_ADDRSTRLEN];
	    inet_ntop(AF_INET, &peer.sin_addr, addrbuf, sizeof(addrbuf));
	    printf("from %s:%d len=%zd\n", addrbuf, ntohs(peer.sin_port), n);
	    fwrite(buf, 1, n, stdout);
	    fflush(stdout);
	}

	close(s);
	return 0;
}
```

-------------------------------------------------------------------------------------------------

Note: IP address (in the code 10.5.5.26 is used ) has to be assigned to the ethernet interface connected to DUT 

## Sample output for CPSW example

\code

=============================
 Enet Raw UDP : Iteration 9 
=============================
CPSW_2G Test
Open MAC port 1
EnetPhy_bindDriver: PHY 0: OUI:080028 Model:23 Ver:01 <-> 'DP83867' : OK
PHY 0 is alive
initQs() txFreePktInfoQ initialized with 16 pkts
Waiting for link up...
Cpsw_handleLinkUp: Port 1: Link up: 100-Mbps Full-Duplex
Link is up
Packet 1 sent successfully
Packet 2 sent successfully
Packet 3 sent successfully
Packet 4 sent successfully
Packet 5 sent successfully

.
.
.

Packet 97 sent successfully
Packet 98 sent successfully
Packet 99 sent successfully
Packet 100 sent successfully
All packets sent successfully

 Port 0 Statistics
-----------------------------------------
  rxGoodFrames            = 100
  rxOctets                = 11000
  octetsFrames65to127     = 100
  netOctets               = 11000


 Port 1 Statistics
-----------------------------------------
  txGoodFrames            = 100
  txOctets                = 11000
  octetsFrames65to127     = 100
  netOctets               = 11000
  txPri[0]                = 100
  txPriBcnt[0]            = 11000

 TX Channel Statistics
-----------------------------------------
Data Notify Count          = 100
Total Packets Count        = 100
Packets per Notify Max     = 1
Packets per Notify[0] = 1
Packets per Notify[1] = 1
Data Notify Count          = 100
Total Packets Count        = 100
Packets per Notify Max     = 1
Packets per Notify[0] = 1
Packets per Notify[1] = 1
TX Submit Packet EnQ count           = 100
TX Submit Packet DeQ count           = 100
Cpsw_handleLinkDown: Port 1: Link down
Cpsw_handleLinkDown: Port 1: Failed to disable EST: -13
De-init complete

\endcode

# See Also
\ref NETWORKING