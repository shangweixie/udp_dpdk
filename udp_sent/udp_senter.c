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
#include <rte_udp.h>
#include <unistd.h>

#define RX_RING_SIZE 128
#define TX_RING_SIZE 512

#define DST_MAC 
#define SRC_MAC
#define SRC_IP
#define DST_IP
#define DST_PORT 6666
#define SRC_PORT 6666

#define NUM_MBUFS 8191
#define MBUF_CACHE_SIZE 250
#define BURST_SIZE 32
#define DATA_LEN 64

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
	uint16_t nb_rxd = RX_RING_SIZE;
	uint16_t nb_txd = TX_RING_SIZE;
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
		retval = rte_eth_rx_queue_setup(port, q, nb_rxd,
				rte_eth_dev_socket_id(port), NULL, mbuf_pool);
		if (retval < 0)
			return retval;
	}

	/* Allocate and set up 1 TX queue per Ethernet port. */
	for (q = 0; q < tx_rings; q++) {
		retval = rte_eth_tx_queue_setup(port, q, nb_txd,
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
lcore_main(struct rte_mempool *mbuf_pool)
{
	const uint8_t nb_ports = rte_eth_dev_count();
	uint8_t port;

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
	for (int j = 0 ; j< 64; j) {
		/*
		 *sent through port 0 
		 */
			port = 0;
			
			/* Get burst of RX packets, from first port of pair. */
			struct rte_mbuf *bufs[BURST_SIZE];

			struct ether_hdr *eth_head;
			struct ipv4_hdr *ip_head;
			struct udp_hdr *udp_head;

			char udp_content[64] = {'i', ' ', 'a', 'm', ' ', 'x', 's', 'w'};
			printf("%s\n", udp_content);
			uint16_t src_port = SRC_PORT;
			uint16_t dst_port = DST_PORT;
			uint16_t udp_len = 64+8;
			uint16_t udp_cksum = 0;
			
			struct ether_addr dst_mac;
			dst_mac.addr_bytes[0] = 0x08;
			dst_mac.addr_bytes[1] = 0x00;
			dst_mac.addr_bytes[2] = 0x27;
			dst_mac.addr_bytes[3] = 0x43;
			dst_mac.addr_bytes[4] = 0x45;
			dst_mac.addr_bytes[5] = 0x75;
			// dst_mac.addr_bytes[0] = 0xff;
			// dst_mac.addr_bytes[1] = 0xff;
			// dst_mac.addr_bytes[2] = 0xff;
			// dst_mac.addr_bytes[3] = 0xff;
			// dst_mac.addr_bytes[4] = 0xff;
			// dst_mac.addr_bytes[5] = 0xff;


			struct ether_addr src_mac;
			rte_eth_macaddr_get(port, &src_mac);

			int retval = rte_pktmbuf_alloc_bulk(mbuf_pool, bufs, BURST_SIZE);
			if(retval != 0)
				printf("pktmbufs alloc failed\n");

			for(int i=0; i< BURST_SIZE; i++)
			{

				/* fill the data */
				char *tmp_addr = rte_pktmbuf_append(bufs[i], DATA_LEN);
				if(tmp_addr == NULL)
					return -1;
				rte_memcpy(tmp_addr, udp_content, DATA_LEN);

				/*  add udp head */
				udp_head = (struct ether_hdr *)rte_pktmbuf_prepend(bufs[i], sizeof(struct udp_hdr));
				udp_head->src_port = rte_cpu_to_be_16(src_port);
				udp_head->dst_port =  rte_cpu_to_be_16(dst_port);
				udp_head->dgram_len = rte_cpu_to_be_16(DATA_LEN + sizeof(struct udp_hdr));
				udp_head->dgram_cksum = udp_cksum;
				//bufs[i]->ol_flags |= PKT_TX_UDP_CKSUM;

				/* add the ip head */
				ip_head = (struct ipv4_hdr *)rte_pktmbuf_prepend(bufs[i], sizeof(struct ipv4_hdr));
				ip_head->version_ihl = 0x45;	
				ip_head->total_length = rte_cpu_to_be_16(udp_len + sizeof(struct ipv4_hdr));
				ip_head->next_proto_id = 0x11;
				ip_head->src_addr = rte_cpu_to_be_32(IPv4(192,168,1,64));
				ip_head->dst_addr = rte_cpu_to_be_32(IPv4(192,168,1,121));
				ip_head->hdr_checksum = rte_ipv4_cksum(ip_head);
				//udp_head->dgram_cksum = rte_ipv4_phdr_cksum(ip_head, bufs[i]->ol_flags);
				//udp_head->dgram_cksum = htobe16(udp_cksum);

				/* add the ether head , the ether frame has no CRC*/
				eth_head = (struct ether_hdr *)rte_pktmbuf_prepend(bufs[i], sizeof(struct ether_hdr));
				eth_head->ether_type = rte_cpu_to_be_16(ETHER_TYPE_IPv4);
				ether_addr_copy(&eth_head->d_addr, &dst_mac);
				ether_addr_copy(&eth_head->s_addr, &src_mac);
				//rte_eth_macaddr_get(port, &(eth_head->s_addr));


				//ip_head->hdr_checksum = rte_ipv4_cksum(ip_head);
				//udp_head->dgram_cksum = 
				// printf("src mac: %02x %02x %02x %02x %02x %02x\n", 
				// 			eth_head->s_addr.addr_bytes[0],
				// 			eth_head->s_addr.addr_bytes[1],
				// 			eth_head->s_addr.addr_bytes[2],
				// 			eth_head->s_addr.addr_bytes[3],
				// 			eth_head->s_addr.addr_bytes[4],
				// 			eth_head->s_addr.addr_bytes[5]
				// );
				//printf("data len: %d , pkt len: %ld \n", bufs[i]->data_len, bufs[i]->pkt_len);
			}
			const uint16_t nb_tx = rte_eth_tx_burst(port, 0, bufs, 1);
			printf("<%d> pktmbufs has been sent.\n", nb_tx);
			if(unlikely(nb_tx < BURST_SIZE)){
				uint16_t buf;
				for(buf = nb_tx; buf < BURST_SIZE ; buf++)
					rte_pktmbuf_free(bufs[buf]);
			}
			// if (nb_tx == 0)
			// 	return 0;
			usleep(5000);
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

	nb_ports = rte_eth_dev_count();

	/* Creates a new mempool in memory to hold the mbufs. */
	mbuf_pool = rte_pktmbuf_pool_create("MBUF_POOL", NUM_MBUFS * nb_ports,
		MBUF_CACHE_SIZE, 0, RTE_MBUF_DEFAULT_BUF_SIZE, rte_socket_id());

	if (mbuf_pool == NULL)
		rte_exit(EXIT_FAILURE, "Cannot create mbuf pool\n");

	/* Initialize all ports. */
	//for (portid = 0; portid < nb_ports; portid++)
	portid = 0;
		if (port_init(portid, mbuf_pool) != 0)
			rte_exit(EXIT_FAILURE, "Cannot init port %"PRIu8 "\n",
					portid);
  
	if (rte_lcore_count() > 1)
		printf("\nWARNING: Too many lcores enabled. Only 1 used.\n");

	/* Call lcore_main on the master core only. */
	lcore_main(mbuf_pool);

	return 0;
}
