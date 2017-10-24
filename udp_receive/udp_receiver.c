/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2010-2015 Intel Corporation. All rights reserved.
 *   All rights reserved.
 *
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *     * Neither the name of Intel Corporation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdint.h>
#include <inttypes.h>
#include <rte_eal.h>
#include <rte_ethdev.h>
#include <rte_cycles.h>
#include <rte_lcore.h>
#include <rte_mbuf.h>
#include <rte_ether.h>
#include <rte_ip.h>
#include <rte_ether.h>

#define RX_RING_SIZE 128
#define TX_RING_SIZE 512

#define NUM_MBUFS 8191
#define MBUF_CACHE_SIZE 250
#define BURST_SIZE 32

static const struct rte_eth_conf port_conf_default = {
	.rxmode = { .max_rx_pkt_len = ETHER_MAX_LEN }
};

/* basicfwd.c: Basic DPDK skeleton forwarding example. */

/*
 * Initializes a given port using global settings and with the RX buffers
 * coming from the mbuf_pool passed as a parameter.
 */
static inline int
port_init(uint8_t port, struct rte_mempool *mbuf_pool)
{
	struct rte_eth_conf port_conf = port_conf_default;
	const uint16_t rx_rings = 1, tx_rings = 1;
	int retval;
	uint16_t q;

	if (port >= rte_eth_dev_count())
		return -1;

	/* Configure the Ethernet device. */
	retval = rte_eth_dev_configure(port, rx_rings, tx_rings, &port_conf);
	if (retval != 0)
		return retval;

	/* Allocate and set up 1 RX queue per Ethernet port. */
	for (q = 0; q < rx_rings; q++) {
		retval = rte_eth_rx_queue_setup(port, q, RX_RING_SIZE,
				rte_eth_dev_socket_id(port), NULL, mbuf_pool);
		if (retval < 0)
			return retval;
	}

	/* Allocate and set up 1 TX queue per Ethernet port. */
	for (q = 0; q < tx_rings; q++) {
		retval = rte_eth_tx_queue_setup(port, q, TX_RING_SIZE,
				rte_eth_dev_socket_id(port), NULL);
		if (retval < 0)
			return retval;
	}

	/* Start the Ethernet port. */
	retval = rte_eth_dev_start(port);
	if (retval < 0)
		return retval;

	/* Display the port MAC address. */
	struct ether_addr addr;
	rte_eth_macaddr_get(port, &addr);
	printf("Port %u MAC: %02" PRIx8 " %02" PRIx8 " %02" PRIx8
			   " %02" PRIx8 " %02" PRIx8 " %02" PRIx8 "\n",
			(unsigned)port,
			addr.addr_bytes[0], addr.addr_bytes[1],
			addr.addr_bytes[2], addr.addr_bytes[3],
			addr.addr_bytes[4], addr.addr_bytes[5]);

	/* Enable RX in promiscuous mode for the Ethernet device. */
	rte_eth_promiscuous_enable(port);

	return 0;
}

/*
 * The lcore main. This is the main thread that does the work, reading from
 * an input port and writing to an output port.
 */
static __attribute__((noreturn)) void
lcore_main(void)
{
	const uint8_t nb_ports = rte_eth_dev_count();
	uint8_t port_senter = 0;
	uint8_t port_receiver = 1;
	uint8_t port = port_receiver;

	struct ether_addr mac_senter;
	struct ether_addr mac_receiver;

	rte_eth_macaddr_get(port_senter, &mac_senter);
	rte_eth_macaddr_get(port_receiver, &mac_receiver);

	/*
	 * Check that the port is on the same NUMA node as the polling thread
	 * for best performance.
	 */
	for (port = 0; port < nb_ports; port++)
		if (rte_eth_dev_socket_id(port) > 0 &&
				rte_eth_dev_socket_id(port) !=
						(int)rte_socket_id())
			printf("WARNING, port %u is on remote NUMA node to "
					"polling thread.\n\tPerformance will "
					"not be optimal.\n", port);

	printf("\nCore %u forwarding packets. [Ctrl+C to quit]\n",
			rte_lcore_id());

	/* Run until the application is quit or killed. */
	for (;;) {
		/*
		 * Receive packets on a port and forward them on the paired
		 * port. The mapping is 0 -> 1, 1 -> 0, 2 -> 3, 3 -> 2, etc.
		 */
		port  = 1;
		

		/* Get burst of RX packets, from first port of pair. */
		struct rte_mbuf *bufs[BURST_SIZE];
		const uint16_t nb_rx = rte_eth_rx_burst(port, 0,
				bufs, BURST_SIZE);
		if (unlikely(nb_rx == 0))
			continue;
		else {
			struct ether_hdr *ether_header;
			//printf("port %d received %d packets.\n", port, nb_rx);
			for(int i=0; i< nb_rx; i++){
				//printf("	packets : %d\n", i);
				
				ether_header = rte_pktmbuf_mtod(bufs[i], struct ether_hdr *);
				
				//printf("data len: %d\n", rte_pktmbuf_data_len(bufs[i]));
				// printf("Port %u MAC: %02" PRIx8 " %02" PRIx8 " %02" PRIx8
				// " %02" PRIx8 " %02" PRIx8 " %02" PRIx8 "\n",
			 	// (unsigned)port,
			 	// ether_header->s_addr.addr_bytes[0], ether_header->s_addr.addr_bytes[1],
			 	// ether_header->s_addr.addr_bytes[2], ether_header->s_addr.addr_bytes[3],
				// ether_header->s_addr.addr_bytes[4], ether_header->s_addr.addr_bytes[5]);
				
				if(is_same_ether_addr(&mac_senter, &ether_header->s_addr));
					printf("this is a packet sent by udp_senter\n");
				// if(
				// 	ether_header->s_addr.addr_bytes[0]==mac_senter.addr_bytes[0]&&
				// 	ether_header->s_addr.addr_bytes[1]==mac_senter.addr_bytes[1]&&
				// 	ether_header->s_addr.addr_bytes[2]==mac_senter.addr_bytes[2]&&
				// 	ether_header->s_addr.addr_bytes[3]==mac_senter.addr_bytes[3]&&
				// 	ether_header->s_addr.addr_bytes[4]==mac_senter.addr_bytes[4]&&
				// 	ether_header->s_addr.addr_bytes[5]==mac_senter.addr_bytes[5])
				// 	printf("received packet udp_senter transmited.\n");
				// else
				// 	continue;
				printf("ether_frame_type: 0x%04x\n", ntohs(ether_header->ether_type));
				if(ntohs(ether_header->ether_type) == 0x0800){  //if this is a ipv4 packet.
					printf("this is a ipv4 packet.\n");
					struct ipv4_hdr *ipv4_header;
					ipv4_header = (struct ipv4_hdr *)( (char *)(ether_header+1) );
					uint16_t ipv4_len = ipv4_header->version_ihl&0x0f;
					// printf("ipv4_len : %d\n", ipv4_len);
					// printf("type of service: %u\n", ipv4_header->type_of_service);
					// printf("total_length: %d\n", ipv4_header->total_length);
					// printf("packet_id: %d\n", ipv4_header->packet_id);
					printf("next_proto_id: 0x%02x\n", ipv4_header->next_proto_id);
					printf("src_addr: %lu.%lu.%lu.%lu\n", 
															(ipv4_header->src_addr&0x000000ff),
													  	  	(ipv4_header->src_addr&0x0000ff00)>>8,
													  	  	(ipv4_header->src_addr&0x00ff0000)>>16,
													      	(ipv4_header->src_addr&0xff000000)>>24);
					printf("dst_addr: %lu.%lu.%lu.%lu\n", 
															(ipv4_header->dst_addr&0x000000ff),
													      	(ipv4_header->dst_addr&0x0000ff00)>>8,
													      	(ipv4_header->dst_addr&0x00ff0000)>>16,
															(ipv4_header->dst_addr&0xff000000)>>24);
					if(ipv4_header->next_proto_id == 0x11){
						printf("this is a udp packet.\n");
					}
				}
			}
		}
	}
}

/*
 * The main function, which does initialization and calls the per-lcore
 * functions.
 */
int
main(int argc, char *argv[])
{
	struct rte_mempool *mbuf_pool;
	unsigned nb_ports;
	uint8_t portid;

	/* Initialize the Environment Abstraction Layer (EAL). */
	int ret = rte_eal_init(argc, argv);
	if (ret < 0)
		rte_exit(EXIT_FAILURE, "Error with EAL initialization\n");

	argc -= ret;
	argv += ret;

	/* Check that there is an even number of ports to send/receive on. */
	nb_ports = rte_eth_dev_count();
	printf("nb_port %d \n", nb_ports);
	if (nb_ports < 2 || (nb_ports & 1))
		rte_exit(EXIT_FAILURE, "Error: number of ports must be even\n");

	/* Creates a new mempool in memory to hold the mbufs. */
	mbuf_pool = rte_pktmbuf_pool_create("MBUF_POOL", NUM_MBUFS * nb_ports,
		MBUF_CACHE_SIZE, 0, RTE_MBUF_DEFAULT_BUF_SIZE, rte_socket_id());

	if (mbuf_pool == NULL)
		rte_exit(EXIT_FAILURE, "Cannot create mbuf pool\n");

	/* Initialize all ports. */
	//for (portid = 0; portid < nb_ports; portid++)
		portid = 1;
		if (port_init(portid, mbuf_pool) != 0)
			rte_exit(EXIT_FAILURE, "Cannot init port %"PRIu8 "\n",
					portid);
  
	if (rte_lcore_count() > 1)
		printf("\nWARNING: Too many lcores enabled. Only 1 used.\n");

	/* Call lcore_main on the master core only. */
	lcore_main();

	return 0;
}