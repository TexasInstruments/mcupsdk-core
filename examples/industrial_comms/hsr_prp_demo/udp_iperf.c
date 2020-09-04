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

extern struct netif server_netif;
static struct perf_stats server;

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
	sprintf(outString, format, data, udperf_kLabel[conv]);
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
		total_len = server.i_report.total_bytes;
		cnt_datagrams = server.i_report.cnt_datagrams;
		cnt_dropped_datagrams = server.i_report.cnt_dropped_datagrams;
	} else {
		server.i_report.last_report_time = 0;
		total_len = server.total_bytes;
		cnt_datagrams = server.cnt_datagrams;
		cnt_dropped_datagrams = server.cnt_dropped_datagrams;
		cnt_out_of_order_datagrams = server.cnt_out_of_order_datagrams;
	}

	total_packets = cnt_datagrams + cnt_dropped_datagrams;
	/* Converting duration from milliseconds to secs,
	 * and bandwidth to bits/sec .
	 */
	duration = diff / 1000.0; /* secs */
	if (duration)
		bandwidth = (total_len / duration) * 8.0;

	stats_buffer(data, total_len, BYTES);
	stats_buffer(perf, bandwidth, SPEED);
	/* On 32-bit platforms, xil_printf is not able to print
	 * u64_t values, so converting these values in strings and
	 * displaying results
	 */
	sprintf(time, "%4.1f-%4.1f sec",
			(double)server.i_report.last_report_time,
			(double)(server.i_report.last_report_time + duration));
	sprintf(drop, "%4llu/%5llu (%.2g%%)", cnt_dropped_datagrams,
			total_packets,
			(100.0 * cnt_dropped_datagrams)/total_packets);
	DebugP_log("[%3d] %s  %sBytes  %sbits/sec  %s\r\n\r", server.client_id,
			time, data, perf, drop);

	if (report_type == INTER_REPORT) {
		server.i_report.last_report_time += duration;
	} else if ((report_type != INTER_REPORT) && cnt_out_of_order_datagrams) {
		DebugP_log("[%3d] %s  %u datagrams received out-of-order\r\n\r",
				server.client_id, time,
				cnt_out_of_order_datagrams);
	}
}

static void reset_stats(void)
{
	server.client_id++;
	/* Save start time */
	server.start_time = sys_now();
	server.end_time = 0; /* ms */
	server.total_bytes = 0;
	server.cnt_datagrams = 0;
	server.cnt_dropped_datagrams = 0;
	server.cnt_out_of_order_datagrams = 0;
	server.expected_datagram_id = 0;

	/* Initialize Interim report parameters */
	server.i_report.start_time = 0;
	server.i_report.total_bytes = 0;
	server.i_report.cnt_datagrams = 0;
	server.i_report.cnt_dropped_datagrams = 0;
	server.i_report.last_report_time = 0;
}

/** Receive data on a udp session */
static void udp_recv_perf_traffic(int sock)
{
	u8_t first = 1;
	u32_t drop_datagrams = 0;
	s32_t recv_id;
	int count;
	char recv_buf[UDP_RECV_BUFSIZE];
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
			/* Print connection statistics */
			print_udp_conn_stats(from);
			first = 0;
		} else if (first) {
			/* Avoid rest of the packets if client
			 * connection is already terminated.
			 */
			continue;
		}

		if (recv_id < 0) {
			u64_t now = sys_now();
			u64_t diff_ms = now - server.start_time;
			/* Send Ack */
			if (sendto(sock, recv_buf, count, 0,
				(struct sockaddr *)&from, fromlen) < 0) {
			    DebugP_log("Error in write\r\n\r");
			}

			udp_conn_report(diff_ms, UDP_DONE_SERVER);
			DebugP_log("UDP test passed Successfully\r\n\r");
			first = 1;
			continue;
		}

		/* Update dropped datagrams statistics */
		if (server.expected_datagram_id != recv_id) {
			if (server.expected_datagram_id < recv_id) {
				drop_datagrams =
					recv_id - server.expected_datagram_id;
				server.cnt_dropped_datagrams += drop_datagrams;
				server.i_report.cnt_dropped_datagrams += drop_datagrams;
				server.expected_datagram_id = recv_id + 1;
			} else if (server.expected_datagram_id > recv_id) {
				server.cnt_out_of_order_datagrams++;
			}
		} else {
			server.expected_datagram_id++;
		}

		server.cnt_datagrams++;

		/* Record total bytes for final report */
		server.total_bytes += count;

		if (REPORT_INTERVAL_TIME) {
			u64_t now = sys_now();

			server.i_report.cnt_datagrams++;

			/* Record total bytes for interim report */
			server.i_report.total_bytes += count;
			if (server.i_report.start_time) {
				u64_t diff_ms = now - server.i_report.start_time;

				if (diff_ms >= REPORT_INTERVAL_TIME) {
					udp_conn_report(diff_ms, INTER_REPORT);
					/* Reset Interim report counters */
					server.i_report.start_time = 0;
					server.i_report.total_bytes = 0;
					server.i_report.cnt_datagrams = 0;
					server.i_report.cnt_dropped_datagrams = 0;
				}
			} else {
				/* Save start time for interim report */
				server.i_report.start_time = now;
			}
		}
	}
}

void start_application(void *arg)
{
	err_t err;
	int sock;
	struct sockaddr_in addr;

	if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
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

	udp_recv_perf_traffic(sock);
}
