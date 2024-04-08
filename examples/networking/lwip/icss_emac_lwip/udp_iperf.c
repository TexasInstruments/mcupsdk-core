/*
 * Copyright (c) 2001-2003 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
   *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 */

/** Connection handle for a UDP Server session */

#include "udp_iperf.h"

static void udp_send_perf_traffic(int sock, u64_t test_duration_ms, char *snd_buf, u32_t snd_size, const struct sockaddr_in *from);

extern struct netif server_netif;
struct perf_stats udp_server_stats;

/* labels for formats [KMG] */
const char udperf_kLabel[] =
{
	' ',
	'K',
	'M',
	'G'
};

/* Report interval in ms */
#define REPORT_INTERVAL_TIME (INTERIM_REPORT_INTERVAL * 1000)

/*
 * Enable printing of connection stats and intermediate reports
 * Note: This degrades performance and causes packet loss errors
 * due to the CPU heavy string formatting (printf, snprintf) done
 * in the reporting functions.
 */
#define DISPLAY_INTERIM_STATS 0


#define UDP_IPERF_ENABLE_UDPLITE            (0)

void print_app_header(void)
{
	DebugP_log("UDP server listening on port %d\r\n", UDP_CONN_PORT);
}

static void print_udp_conn_stats(struct sockaddr_in from)
{
	DebugP_log("Connected to %s port %d\r\n", inet_ntoa(from.sin_addr),
				ntohs(from.sin_port));

	DebugP_log("[ ID] Interval\t     Transfer     Bandwidth\t");
	DebugP_log("    Lost/Total Datagrams\r\n");
}

static void stats_buffer(char* outString,
		u32_t outStringLen,
		double data, enum measure_t type)
{
	int conv = KCONV_UNIT;
	const char *format;
	double unit = 1024.0;

	if (type == SPEED)
		unit = 1000.0;

	while (data >= unit && conv <= KCONV_GIGA) {
		data /= unit;
		conv++;
	}

	/* Fit data in 4 places */
	if (data < 9.995) { /* 9.995 rounded to 10.0 */
		format = "%4.2f %c"; /* #.## */
	} else if (data < 99.95) { /* 99.95 rounded to 100 */
		format = "%4.1f %c"; /* ##.# */
	} else {
		format = "%4.0f %c"; /* #### */
	}
	snprintf(outString, outStringLen, format, data, udperf_kLabel[conv]);
}

/** The report function of a TCP server session */
static void udp_conn_report(u64_t diff,
		enum report_type report_type)
{
	u64_t total_len, cnt_datagrams, cnt_dropped_datagrams, total_packets;
	u32_t cnt_out_of_order_datagrams;
	double duration, bandwidth = 0;
	char data[16], perf[16], time[64], drop[64];

	if (report_type == INTER_REPORT) {
		total_len = udp_server_stats.i_report.total_bytes;
		cnt_datagrams = udp_server_stats.i_report.cnt_datagrams;
		cnt_dropped_datagrams = udp_server_stats.i_report.cnt_dropped_datagrams;
	} else {
	    udp_server_stats.i_report.last_report_time = 0;
		total_len = udp_server_stats.total_bytes;
		cnt_datagrams = udp_server_stats.cnt_datagrams;
		cnt_dropped_datagrams = udp_server_stats.cnt_dropped_datagrams;
		cnt_out_of_order_datagrams = udp_server_stats.cnt_out_of_order_datagrams;
	}

	total_packets = cnt_datagrams + cnt_dropped_datagrams;
	/* Converting duration from milliseconds to secs,
	 * and bandwidth to bits/sec .
	 */
	duration = diff / 1000.0; /* secs */
	if (duration)
		bandwidth = (total_len / duration) * 8.0;

	stats_buffer(data, sizeof(data), total_len, BYTES);
	stats_buffer(perf, sizeof(perf), bandwidth, SPEED);
	/* On 32-bit platforms, xil_printf is not able to print
	 * u64_t values, so converting these values in strings and
	 * displaying results
	 */
	snprintf(time, sizeof(time), "%4.1f-%4.1f sec",
			(double)udp_server_stats.i_report.last_report_time,
			(double)(udp_server_stats.i_report.last_report_time + duration));
	snprintf(drop, sizeof(drop), "%4llu/%5llu (%.2g%%)", cnt_dropped_datagrams,
			total_packets,
			(100.0 * cnt_dropped_datagrams)/total_packets);
	DebugP_log("[%3d] %s  %sBytes  %sbits/sec  %s\r\n\r", udp_server_stats.client_id,
			time, data, perf, drop);

	if (report_type == INTER_REPORT) {
	    udp_server_stats.i_report.last_report_time += duration;
	} else if ((report_type != INTER_REPORT) && cnt_out_of_order_datagrams) {
		DebugP_log("[%3d] %s  %u datagrams received out-of-order\r\n\r",
		       udp_server_stats.client_id, time,
				cnt_out_of_order_datagrams);
	}
}

static void reset_stats(void)
{
    udp_server_stats.client_id++;
	/* Save start time */
    udp_server_stats.start_time = sys_now();
    udp_server_stats.end_time = 0; /* ms */
    udp_server_stats.total_bytes = 0;
    udp_server_stats.cnt_datagrams = 0;
    udp_server_stats.cnt_dropped_datagrams = 0;
    udp_server_stats.cnt_out_of_order_datagrams = 0;
    udp_server_stats.expected_datagram_id = 0;

	/* Initialize Interim report parameters */
    udp_server_stats.i_report.start_time = 0;
    udp_server_stats.i_report.total_bytes = 0;
    udp_server_stats.i_report.cnt_datagrams = 0;
    udp_server_stats.i_report.cnt_dropped_datagrams = 0;
    udp_server_stats.i_report.last_report_time = 0;
}

#define UTILS_ALIGN(x,align)  ((((x) + ((align) - 1))/(align)) * (align))
/** Receive data on a udp session */
static void udp_recv_perf_traffic(int sock)
{
	u8_t first = 1;
	u32_t drop_datagrams = 0;
	s32_t recv_id;
	int count;
	char recv_buf[UTILS_ALIGN(UDP_RECV_BUFSIZE,32)] __attribute__ ((aligned(32)));
	struct sockaddr_in from;
	socklen_t fromlen = sizeof(from);

	volatile int flag = 1;
	while (flag) {
		if((count = lwip_recvfrom(sock, recv_buf, UDP_RECV_BUFSIZE, 0,
				(struct sockaddr *)&from, &fromlen)) <= 0) {
			continue;
		}
		/* first, check if the datagram is received in order */
		recv_id = ntohl(*((int *)recv_buf));

		if (first && (recv_id == 0)) {
			/* First packet should always start with recv id 0.
			 * However, If Iperf client is running with parallel
			 * thread, then this condition will also avoid
			 * multiple print of connection header
			 */
			reset_stats();
#if DISPLAY_INTERIM_STATS
			/* Print connection statistics */
			print_udp_conn_stats(from);
#endif
			first = 0;
		} else if (first) {
			/* Avoid rest of the packets if client
			 * connection is already terminated.
			 */
			continue;
		}

		if (recv_id < 0) {
			u64_t now = sys_now();
			u64_t diff_ms = now - udp_server_stats.start_time;
			/* Send Ack */
			if (sendto(sock, recv_buf, count, 0,
				(struct sockaddr *)&from, fromlen) < 0) {
			    DebugP_log("Error in write\r\n\r");
			}

			udp_conn_report(diff_ms, UDP_DONE_SERVER);
            udp_send_perf_traffic(sock, diff_ms, recv_buf, count, &from);
            lwip_recvfrom(sock, recv_buf, count, 0,
				                  (struct sockaddr *)&from, &fromlen);
			DebugP_log("UDP test passed Successfully\r\n\r");
			first = 1;
			continue;
		}

		/* Update dropped datagrams statistics */
		if (udp_server_stats.expected_datagram_id != recv_id) {
			if (udp_server_stats.expected_datagram_id < recv_id) {
				drop_datagrams =
					recv_id - udp_server_stats.expected_datagram_id;
				udp_server_stats.cnt_dropped_datagrams += drop_datagrams;
				udp_server_stats.i_report.cnt_dropped_datagrams += drop_datagrams;
				udp_server_stats.expected_datagram_id = recv_id + 1;
			} else if (udp_server_stats.expected_datagram_id > recv_id) {
			    udp_server_stats.cnt_out_of_order_datagrams++;
			}
		} else {
		    udp_server_stats.expected_datagram_id++;
		}

		udp_server_stats.cnt_datagrams++;

		/* Record total bytes for final report */
		udp_server_stats.total_bytes += count;
		if (REPORT_INTERVAL_TIME) {
			u64_t now = sys_now();

			udp_server_stats.i_report.cnt_datagrams++;

			/* Record total bytes for interim report */
			udp_server_stats.i_report.total_bytes += count;
			if (udp_server_stats.i_report.start_time) {
				u64_t diff_ms = now - udp_server_stats.i_report.start_time;

				if (diff_ms >= REPORT_INTERVAL_TIME) {
					udp_conn_report(diff_ms, INTER_REPORT);
					/* Reset Interim report counters */
					udp_server_stats.i_report.start_time = 0;
					udp_server_stats.i_report.total_bytes = 0;
					udp_server_stats.i_report.cnt_datagrams = 0;
					udp_server_stats.i_report.cnt_dropped_datagrams = 0;
				}
			} else {
				/* Save start time for interim report */
			    udp_server_stats.i_report.start_time = now;
			}
		}
	}
}

#define UDP_TX_TIME_CHECK_PACKET_COUNT (50 * 1000)
#define UDP_TX_NUM_BUFS  (192)
#define UDP_IPERF_R5F_CACHE_LINE_SIZE  (32)

char snd_buf_array[UDP_TX_NUM_BUFS][UTILS_ALIGN(UDP_RECV_BUFSIZE,UDP_IPERF_R5F_CACHE_LINE_SIZE)] 
__attribute__ ((aligned(UDP_IPERF_R5F_CACHE_LINE_SIZE),
                section(".bss:UDP_IPERF_SND_BUF")));

/** Receive data on a udp session */
static void udp_send_perf_traffic(int sock, u64_t test_duration_ms, char *snd_buf, u32_t snd_size, const struct sockaddr_in *from)
{
	s32_t snd_id;
	int count;
	struct sockaddr_in to;
	socklen_t tolen = sizeof(to);
	volatile int flag = 1;
    u64_t start_time = sys_now();
    u64_t diff_ms;
    u32_t i;

    to = *from;
    to.sin_port = htons(UDP_CONN_PORT);
    snd_id = 0;

    for (i = 0; i <  UDP_TX_NUM_BUFS; i++)
    {
        memcpy(&snd_buf_array[i][0], snd_buf, snd_size);
    }
	while (flag) {
        snd_id++;
        snd_buf = &snd_buf_array[snd_id % UDP_TX_NUM_BUFS][0];
        if ((snd_id % UDP_TX_TIME_CHECK_PACKET_COUNT) == 0)
        {
            u64_t now  = sys_now();
            
            diff_ms = now - start_time;
            if (diff_ms >= test_duration_ms)
            {
                snd_id = -1;
                flag = 0;
            }
        }
        *((int *)snd_buf) = htonl(snd_id);
        do {
            count = lwip_sendto(sock, snd_buf, snd_size, 0,
                   (struct sockaddr *)&to, tolen);
            if (count != snd_size)
            {
                sys_arch_msleep(1U);
            }
        } while (count != snd_size);
	}
}



#if (UDP_IPERF_ENABLE_UDPLITE == 1)
#define UDP_IPERF_SOCKET_PROTOCOL  (IPPROTO_UDPLITE)
#else
#define UDP_IPERF_SOCKET_PROTOCOL  (IPPROTO_IP)
#endif



void start_application(void *arg)
{
	err_t err;
	int sock;
	struct sockaddr_in addr;
#if (UDP_IPERF_ENABLE_UDPLITE == 1)
    const int cssend  = 8;
    const int csrecv  = 8;
#endif

	if ((sock = socket(AF_INET, SOCK_DGRAM, UDP_IPERF_SOCKET_PROTOCOL)) < 0) {
		DebugP_log("UDP server: Error creating Socket\r\r\n");
		return;
	}

	memset(&addr, 0, sizeof(struct sockaddr_in));
	addr.sin_family = AF_INET;
	addr.sin_port = htons(UDP_CONN_PORT);
	addr.sin_addr.s_addr = htonl(INADDR_ANY);

	err = bind(sock, (struct sockaddr *)&addr, sizeof(addr));
	if (err != ERR_OK) {
		DebugP_log("UDP server: Error on bind: %d\r\r\n", err);
		close(sock);
		return;
	}

#if (UDP_IPERF_ENABLE_UDPLITE == 1)
	err = setsockopt(sock, IPPROTO_UDPLITE, UDPLITE_SEND_CSCOV, &cssend, sizeof(cssend));
	if (err != ERR_OK) {
		DebugP_log("UDP server: Error on setsockopt: %d\r\n", err);
		close(sock);
		return;
	}

	err = setsockopt(sock, IPPROTO_UDPLITE, UDPLITE_RECV_CSCOV, &csrecv, sizeof(csrecv));
	if (err != ERR_OK) {
		DebugP_log("UDP server: Error on setsockopt: %d\r\n", err);
		close(sock);
		return;
	}
#endif

	udp_recv_perf_traffic(sock);
}
