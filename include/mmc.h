/*
 * Copyright (c) 2018-2020, NVIDIA CORPORATION. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#ifndef MMC_H
#define MMC_H
/**
 * @brief osi_mmc_counters - The structure to hold RMON counter values
 */
struct osi_mmc_counters {
	/** This counter provides the number of bytes transmitted, exclusive of
	 * preamble and retried bytes, in good and bad packets */
	nveu64_t mmc_tx_octetcount_gb;
	/** This counter provides upper 32 bits of transmitted octet count */
	nveu64_t mmc_tx_octetcount_gb_h;
	/** This counter provides the number of good and
	 * bad packets transmitted, exclusive of retried packets */
	nveu64_t mmc_tx_framecount_gb;
	/** This counter provides upper 32 bits of transmitted good and bad
	 * packets count */
	nveu64_t mmc_tx_framecount_gb_h;
	/** This counter provides number of good broadcast
	 * packets transmitted */
	nveu64_t mmc_tx_broadcastframe_g;
	/** This counter provides upper 32 bits of transmitted good broadcast
	 * packets count */
	nveu64_t mmc_tx_broadcastframe_g_h;
	/** This counter provides number of good multicast
	 * packets transmitted */
	nveu64_t mmc_tx_multicastframe_g;
	/** This counter provides upper 32 bits of transmitted good multicast
	 * packet count */
	nveu64_t mmc_tx_multicastframe_g_h;
	/** This counter provides the number of good and bad packets
	 * transmitted with length 64 bytes, exclusive of preamble and
	 * retried packets */
	nveu64_t mmc_tx_64_octets_gb;
	/** This counter provides upper 32 bits of transmitted 64 octet size
	 * good and bad packets count */
	nveu64_t mmc_tx_64_octets_gb_h;
	/** This counter provides the number of good and bad packets
	 * transmitted with length 65-127 bytes, exclusive of preamble and
	 * retried packets */
	nveu64_t mmc_tx_65_to_127_octets_gb;
	/** Provides upper 32 bits of transmitted 65-to-127 octet size good and
	 * bad packets count */
	nveu64_t mmc_tx_65_to_127_octets_gb_h;
	/** This counter provides the number of good and bad packets
	 * transmitted with length 128-255 bytes, exclusive of preamble and
	 * retried packets */
	nveu64_t mmc_tx_128_to_255_octets_gb;
	/** This counter provides upper 32 bits of transmitted 128-to-255
	 * octet size good and bad packets count */
	nveu64_t mmc_tx_128_to_255_octets_gb_h;
	/** This counter provides the number of good and bad packets
	 * transmitted with length 256-511 bytes, exclusive of preamble and
	 * retried packets */
	nveu64_t mmc_tx_256_to_511_octets_gb;
	/** This counter provides upper 32 bits of transmitted 256-to-511
	 * octet size good and bad packets count. */
	nveu64_t mmc_tx_256_to_511_octets_gb_h;
	/** This counter provides the number of good and bad packets
	 * transmitted with length 512-1023 bytes, exclusive of preamble and
	 * retried packets */
	nveu64_t mmc_tx_512_to_1023_octets_gb;
	/** This counter provides upper 32 bits of transmitted 512-to-1023
	 * octet size good and bad packets count.*/
	nveu64_t mmc_tx_512_to_1023_octets_gb_h;
	/** This counter provides the number of good and bad packets
	 * transmitted with length 1024-max bytes, exclusive of preamble and
	 * retried packets */
	nveu64_t mmc_tx_1024_to_max_octets_gb;
	/** This counter provides upper 32 bits of transmitted 1024-tomaxsize
	 * octet size good and bad packets count. */
	nveu64_t mmc_tx_1024_to_max_octets_gb_h;
	/** This counter provides the number of good and bad unicast packets */
	nveu64_t mmc_tx_unicast_gb;
	/** This counter provides upper 32 bits of transmitted good bad
	 * unicast packets count */
	nveu64_t mmc_tx_unicast_gb_h;
	/** This counter provides the number of good and bad
	 * multicast packets */
	nveu64_t mmc_tx_multicast_gb;
	/** This counter provides upper 32 bits of transmitted good bad
	 * multicast packets count */
	nveu64_t mmc_tx_multicast_gb_h;
	/** This counter provides the number of good and bad
	 * broadcast packets */
	nveu64_t mmc_tx_broadcast_gb;
	/** This counter provides upper 32 bits of transmitted good bad
	 * broadcast packets count */
	nveu64_t mmc_tx_broadcast_gb_h;
	/** This counter provides the number of abort packets due to
	 * underflow error */
	nveu64_t mmc_tx_underflow_error;
	/** This counter provides upper 32 bits of abort packets due to
	 * underflow error */
	nveu64_t mmc_tx_underflow_error_h;
	/** This counter provides the number of successfully transmitted
	 * packets after a single collision in the half-duplex mode */
	nveu64_t mmc_tx_singlecol_g;
	/** This counter provides the number of successfully transmitted
	 * packets after a multi collision in the half-duplex mode */
	nveu64_t mmc_tx_multicol_g;
	/** This counter provides the number of successfully transmitted
	 * after a deferral in the half-duplex mode */
	nveu64_t mmc_tx_deferred;
	/** This counter provides the number of packets aborted because of
	 * late collision error */
	nveu64_t mmc_tx_latecol;
	/** This counter provides the number of packets aborted because of
	 * excessive (16) collision errors */
	nveu64_t mmc_tx_exesscol;
	/** This counter provides the number of packets aborted because of
	 * carrier sense error (no carrier or loss of carrier) */
	nveu64_t mmc_tx_carrier_error;
	/** This counter provides the number of bytes transmitted,
	 * exclusive of preamble, only in good packets */
	nveu64_t mmc_tx_octetcount_g;
	/** This counter provides upper 32 bytes of bytes transmitted,
	 * exclusive of preamble, only in good packets */
	nveu64_t mmc_tx_octetcount_g_h;
	/** This counter provides the number of good packets transmitted */
	nveu64_t mmc_tx_framecount_g;
	/** This counter provides upper 32 bytes of good packets transmitted */
	nveu64_t mmc_tx_framecount_g_h;
	/** This counter provides the number of packets aborted because of
	 * excessive deferral error
	 * (deferred for more than two max-sized packet times) */
	nveu64_t mmc_tx_excessdef;
	/** This counter provides the number of good Pause
	 * packets transmitted */
	nveu64_t mmc_tx_pause_frame;
	/** This counter provides upper 32 bytes of good Pause
	 * packets transmitted */
	nveu64_t mmc_tx_pause_frame_h;
	/** This counter provides the number of good VLAN packets transmitted */
	nveu64_t mmc_tx_vlan_frame_g;
	/** This counter provides upper 32 bytes of good VLAN packets
	 * transmitted */
	nveu64_t mmc_tx_vlan_frame_g_h;
	/** This counter provides the number of packets transmitted without
	 * errors and with length greater than the maxsize (1,518 or 1,522 bytes
	 * for VLAN tagged packets; 2000 bytes */
	nveu64_t mmc_tx_osize_frame_g;
	/** This counter provides the number of good and bad packets received */
	nveu64_t mmc_rx_framecount_gb;
	/** This counter provides upper 32 bytes of good and bad packets
	 * received */
	nveu64_t mmc_rx_framecount_gb_h;
	/** This counter provides the number of bytes received by DWC_ther_qos,
	 * exclusive of preamble, in good and bad packets */
	nveu64_t mmc_rx_octetcount_gb;
	/** This counter provides upper 32 bytes of bytes received by
	 * DWC_ether_qos, exclusive of preamble, in good and bad packets */
	nveu64_t mmc_rx_octetcount_gb_h;
	/** This counter provides the number of bytes received by DWC_ether_qos,
	 * exclusive of preamble, in good and bad packets */
	nveu64_t mmc_rx_octetcount_g;
	/** This counter provides upper 32 bytes of bytes received by
	 * DWC_ether_qos, exclusive of preamble, in good and bad packets */
	nveu64_t mmc_rx_octetcount_g_h;
	/** This counter provides the number of good
	 * broadcast packets received */
	nveu64_t mmc_rx_broadcastframe_g;
	/** This counter provides upper 32 bytes of good
	 * broadcast packets received */
	nveu64_t mmc_rx_broadcastframe_g_h;
	/** This counter provides the number of good
	 * multicast packets received */
	nveu64_t mmc_rx_multicastframe_g;
	/** This counter provides upper 32 bytes of good
	 * multicast packets received */
	nveu64_t mmc_rx_multicastframe_g_h;
	/** This counter provides the number of packets
	 * received with CRC error */
	nveu64_t mmc_rx_crc_error;
	/** This counter provides upper 32 bytes of packets
	 * received with CRC error */
	nveu64_t mmc_rx_crc_error_h;
	/** This counter provides the number of packets received with
	 * alignment (dribble) error. It is valid only in 10/100 mode */
	nveu64_t mmc_rx_align_error;
	/** This counter provides the number of packets received  with
	 * runt (length less than 64 bytes and CRC error) error */
	nveu64_t mmc_rx_runt_error;
	/** This counter provides the number of giant packets received  with
	 * length (including CRC) greater than 1,518 bytes (1,522 bytes for
	 * VLAN tagged) and with CRC error */
	nveu64_t mmc_rx_jabber_error;
	/** This counter provides the number of packets received  with length
	 * less than 64 bytes, without any errors */
	nveu64_t mmc_rx_undersize_g;
	/** This counter provides the number of packets received  without error,
	 * with length greater than the maxsize */
	nveu64_t mmc_rx_oversize_g;
	/** This counter provides the number of good and bad packets received
	 * with length 64 bytes, exclusive of the preamble */
	nveu64_t mmc_rx_64_octets_gb;
	/** This counter provides upper 32 bytes of good and bad packets
	 * received with length 64 bytes, exclusive of the preamble */
	nveu64_t mmc_rx_64_octets_gb_h;
	/** This counter provides the number of good and bad packets received
	 * with length 65-127 bytes, exclusive of the preamble */
	nveu64_t mmc_rx_65_to_127_octets_gb;
	/** This counter provides upper 32 bytes of good and bad packets
	 * received with length 65-127 bytes, exclusive of the preamble */
	nveu64_t mmc_rx_65_to_127_octets_gb_h;
	/** This counter provides the number of good and bad packets received
	 * with length 128-255 bytes, exclusive of the preamble */
	nveu64_t mmc_rx_128_to_255_octets_gb;
	/** This counter provides upper 32 bytes of good and bad packets
	 * received with length 128-255 bytes, exclusive of the preamble */
	nveu64_t mmc_rx_128_to_255_octets_gb_h;
	/** This counter provides the number of good and bad packets received
	 * with length 256-511 bytes, exclusive of the preamble */
	nveu64_t mmc_rx_256_to_511_octets_gb;
	/** This counter provides upper 32 bytes of good and bad packets
	 * received with length 256-511 bytes, exclusive of the preamble */
	nveu64_t mmc_rx_256_to_511_octets_gb_h;
	/** This counter provides the number of good and bad packets received
	 * with length 512-1023 bytes, exclusive of the preamble */
	nveu64_t mmc_rx_512_to_1023_octets_gb;
	/** This counter provides upper 32 bytes of good and bad packets
	 * received with length 512-1023 bytes, exclusive of the preamble */
	nveu64_t mmc_rx_512_to_1023_octets_gb_h;
	/** This counter provides the number of good and bad packets received
	 * with length 1024-maxbytes, exclusive of the preamble */
	nveu64_t mmc_rx_1024_to_max_octets_gb;
	/** This counter provides upper 32 bytes of good and bad packets
	 * received with length 1024-maxbytes, exclusive of the preamble */
	nveu64_t mmc_rx_1024_to_max_octets_gb_h;
	/** This counter provides the number of good unicast packets received */
	nveu64_t mmc_rx_unicast_g;
	/** This counter provides upper 32 bytes of good unicast packets
	 * received */
	nveu64_t mmc_rx_unicast_g_h;
	/** This counter provides the number of packets received  with length
	 * error (Length Type field not equal to packet size), for all packets
	 * with valid length field */
	nveu64_t mmc_rx_length_error;
	/** This counter provides upper 32 bytes of packets received  with
	 * length error (Length Type field not equal to packet size), for all
	 * packets with valid length field */
	nveu64_t mmc_rx_length_error_h;
	/** This counter provides the number of packets received  with length
	 * field not equal to the valid packet size (greater than 1,500 but
	 * less than 1,536) */
	nveu64_t mmc_rx_outofrangetype;
	/** This counter provides upper 32 bytes of packets received  with
	 * length field not equal to the valid packet size (greater than 1,500
	 * but less than 1,536) */
	nveu64_t mmc_rx_outofrangetype_h;
	/** This counter provides the number of good and valid Pause packets
	 * received */
	nveu64_t mmc_rx_pause_frames;
	/** This counter provides upper 32 bytes of good and valid Pause packets
	 * received */
	nveu64_t mmc_rx_pause_frames_h;
	/** This counter provides the number of missed received packets
	 * because of FIFO overflow in DWC_ether_qos */
	nveu64_t mmc_rx_fifo_overflow;
	/** This counter provides upper 32 bytes of missed received packets
	 * because of FIFO overflow in DWC_ether_qos */
	nveu64_t mmc_rx_fifo_overflow_h;
	/** This counter provides the number of good and bad VLAN packets
	 * received */
	nveu64_t mmc_rx_vlan_frames_gb;
	/** This counter provides upper 32 bytes of good and bad VLAN packets
	 * received */
	nveu64_t mmc_rx_vlan_frames_gb_h;
	/** This counter provides the number of packets received with error
	 * because of watchdog timeout error */
	nveu64_t mmc_rx_watchdog_error;
	/** This counter provides the number of packets received with Receive
	 * error or Packet Extension error on the GMII or MII interface */
	nveu64_t mmc_rx_receive_error;
	/** This counter provides the number of packets received with Receive
	 * error or Packet Extension error on the GMII or MII interface */
	nveu64_t mmc_rx_ctrl_frames_g;
	/** This counter provides the number of microseconds Tx LPI is asserted
	 * in the MAC controller */
	nveu64_t mmc_tx_lpi_usec_cntr;
	/** This counter provides the number of times MAC controller has
	 * entered Tx LPI. */
	nveu64_t mmc_tx_lpi_tran_cntr;
	/** This counter provides the number of microseconds Rx LPI is asserted
	 * in the MAC controller */
	nveu64_t mmc_rx_lpi_usec_cntr;
	/** This counter provides the number of times MAC controller has
	 * entered Rx LPI.*/
	nveu64_t mmc_rx_lpi_tran_cntr;
	/** This counter provides the number of good IPv4 datagrams received
	 * with the TCP, UDP, or ICMP payload */
	nveu64_t mmc_rx_ipv4_gd;
	/** This counter provides upper 32 bytes of good IPv4 datagrams received
	 * with the TCP, UDP, or ICMP payload */
	nveu64_t mmc_rx_ipv4_gd_h;
	/** RxIPv4 Header Error Packets */
	nveu64_t mmc_rx_ipv4_hderr;
	/** RxIPv4 of upper 32 bytes of Header Error Packets */
	nveu64_t mmc_rx_ipv4_hderr_h;
	/** This counter provides the number of IPv4 datagram packets received
	 * that did not have a TCP, UDP, or ICMP payload */
	nveu64_t mmc_rx_ipv4_nopay;
	/** This counter provides upper 32 bytes of IPv4 datagram packets
	 * received that did not have a TCP, UDP, or ICMP payload */
	nveu64_t mmc_rx_ipv4_nopay_h;
	/** This counter provides the number of good IPv4 datagrams received
	 * with fragmentation */
	nveu64_t mmc_rx_ipv4_frag;
	/** This counter provides upper 32 bytes of good IPv4 datagrams received
	 * with fragmentation */
	nveu64_t mmc_rx_ipv4_frag_h;
	/** This counter provides the number of good IPv4 datagrams received
	 * that had a UDP payload with checksum disabled */
	nveu64_t mmc_rx_ipv4_udsbl;
	/** This counter provides upper 32 bytes of good IPv4 datagrams received
	 * that had a UDP payload with checksum disabled */
	nveu64_t mmc_rx_ipv4_udsbl_h;
	/** This counter provides the number of good IPv6 datagrams received
	 * with the TCP, UDP, or ICMP payload */
	nveu64_t mmc_rx_ipv6_gd_octets;
	/** This counter provides upper 32 bytes of good IPv6 datagrams received
	 * with the TCP, UDP, or ICMP payload */
	nveu64_t mmc_rx_ipv6_gd_octets_h;
	/** This counter provides the number of IPv6 datagrams received
	 * with header (length or version mismatch) errors */
	nveu64_t mmc_rx_ipv6_hderr_octets;
	/** This counter provides the number of IPv6 datagrams received
	 * with header (length or version mismatch) errors */
	nveu64_t mmc_rx_ipv6_hderr_octets_h;
	/** This counter provides the number of IPv6 datagram packets received
	 * that did not have a TCP, UDP, or ICMP payload */
	nveu64_t mmc_rx_ipv6_nopay_octets;
	/** This counter provides upper 32 bytes of IPv6 datagram packets
	 * received that did not have a TCP, UDP, or ICMP payload */
	nveu64_t mmc_rx_ipv6_nopay_octets_h;
	/* Protocols */
	/** This counter provides the number of good IP datagrams received by
	 * DWC_ether_qos with a good UDP payload */
	nveu64_t mmc_rx_udp_gd;
	/** This counter provides upper 32 bytes of good IP datagrams received
	 * by DWC_ether_qos with a good UDP payload */
	nveu64_t mmc_rx_udp_gd_h;
	/** This counter provides the number of good IP datagrams received by
	 * DWC_ether_qos with a good UDP payload. This counter is not updated
	 * when the RxIPv4_UDP_Checksum_Disabled_Packets counter is
	 * incremented */
	nveu64_t mmc_rx_udp_err;
	/** This counter provides upper 32 bytes of good IP datagrams received
	 * by DWC_ether_qos with a good UDP payload. This counter is not updated
	 * when the RxIPv4_UDP_Checksum_Disabled_Packets counter is
	 * incremented */
	nveu64_t mmc_rx_udp_err_h;
	/** This counter provides the number of good IP datagrams received
	 * with a good TCP payload */
	nveu64_t mmc_rx_tcp_gd;
	/** This counter provides the number of good IP datagrams received
	 * with a good TCP payload */
	nveu64_t mmc_rx_tcp_gd_h;
	/** This counter provides upper 32 bytes of good IP datagrams received
	 * with a good TCP payload */
	nveu64_t mmc_rx_tcp_err;
	/** This counter provides upper 32 bytes of good IP datagrams received
	 * with a good TCP payload */
	nveu64_t mmc_rx_tcp_err_h;
	/** This counter provides the number of good IP datagrams received
	 * with a good ICMP payload */
	nveu64_t mmc_rx_icmp_gd;
	/** This counter provides upper 32 bytes of good IP datagrams received
	 * with a good ICMP payload */
	nveu64_t mmc_rx_icmp_gd_h;
	/** This counter provides the number of good IP datagrams received
	 * whose ICMP payload has a checksum error */
	nveu64_t mmc_rx_icmp_err;
	/** This counter provides upper 32 bytes of good IP datagrams received
	 * whose ICMP payload has a checksum error */
	nveu64_t mmc_rx_icmp_err_h;
	/** This counter provides the number of bytes received by DWC_ether_qos
	 * in good IPv4 datagrams encapsulating TCP, UDP, or ICMP data.
	 * (Ethernet header, FCS, pad, or IP pad bytes are not included
	 * in this counter */
	nveu64_t mmc_rx_ipv4_gd_octets;
	/** This counter provides upper 32 bytes received by DWC_ether_qos
	 * in good IPv4 datagrams encapsulating TCP, UDP, or ICMP data.
	 * (Ethernet header, FCS, pad, or IP pad bytes are not included
	 * in this counter */
	nveu64_t mmc_rx_ipv4_gd_octets_h;
	/** This counter provides the number of bytes received in IPv4 datagram
	 * with header errors (checksum, length, version mismatch). The value
	 * in the Length field of IPv4 header is used to update this counter.
	 * (Ethernet header, FCS, pad, or IP pad bytes are not included
	 * in this counter */
	nveu64_t mmc_rx_ipv4_hderr_octets;
	/** This counter provides upper 32 bytes received in IPv4 datagram
	 * with header errors (checksum, length, version mismatch). The value
	 * in the Length field of IPv4 header is used to update this counter.
	 * (Ethernet header, FCS, pad, or IP pad bytes are not included
	 * in this counter */
	nveu64_t mmc_rx_ipv4_hderr_octets_h;
	/** This counter provides the number of bytes received in IPv4 datagram
	 * that did not have a TCP, UDP, or ICMP payload. The value in the
	 * Length field of IPv4 header is used to update this counter.
	 * (Ethernet header, FCS, pad, or IP pad bytes are not included
	 * in this counter */
	nveu64_t mmc_rx_ipv4_nopay_octets;
	/** This counter provides upper 32 bytes received in IPv4 datagram
	 * that did not have a TCP, UDP, or ICMP payload. The value in the
	 * Length field of IPv4 header is used to update this counter.
	 * (Ethernet header, FCS, pad, or IP pad bytes are not included
	 * in this counter */
	nveu64_t mmc_rx_ipv4_nopay_octets_h;
	/** This counter provides the number of bytes received in fragmented
	 * IPv4 datagrams. The value in the Length field of IPv4 header is
	 * used to update this counter. (Ethernet header, FCS, pad, or IP pad
	 * bytes are not included in this counter */
	nveu64_t mmc_rx_ipv4_frag_octets;
	/** This counter provides upper 32 bytes received in fragmented
	 * IPv4 datagrams. The value in the Length field of IPv4 header is
	 * used to update this counter. (Ethernet header, FCS, pad, or IP pad
	 * bytes are not included in this counter */
	nveu64_t mmc_rx_ipv4_frag_octets_h;
	/** This counter provides the number of bytes received in a UDP segment
	 * that had the UDP checksum disabled. This counter does not count IP
	 * Header bytes. (Ethernet header, FCS, pad, or IP pad bytes are not
	 * included in this counter */
	nveu64_t mmc_rx_ipv4_udsbl_octets;
	/** This counter provides upper 32 bytes received in a UDP segment
	 * that had the UDP checksum disabled. This counter does not count IP
	 * Header bytes. (Ethernet header, FCS, pad, or IP pad bytes are not
	 * included in this counter */
	nveu64_t mmc_rx_ipv4_udsbl_octets_h;
	/** This counter provides the number of bytes received in good IPv6
	 * datagrams encapsulating TCP, UDP, or ICMP data. (Ethernet header,
	 * FCS, pad, or IP pad bytes are not included in this counter */
	nveu64_t mmc_rx_ipv6_gd;
	/** This counter provides upper 32 bytes received in good IPv6
	 * datagrams encapsulating TCP, UDP, or ICMP data. (Ethernet header,
	 * FCS, pad, or IP pad bytes are not included in this counter */
	nveu64_t mmc_rx_ipv6_gd_h;
	/** This counter provides the number of bytes received in IPv6 datagrams
	 * with header errors (length, version mismatch). The value in the
	 * Length field of IPv6 header is used to update this counter.
	 * (Ethernet header, FCS, pad, or IP pad bytes are not included in
	 * this counter */
	nveu64_t mmc_rx_ipv6_hderr;
	/** This counter provides upper 32 bytes received in IPv6 datagrams
	 * with header errors (length, version mismatch). The value in the
	 * Length field of IPv6 header is used to update this counter.
	 * (Ethernet header, FCS, pad, or IP pad bytes are not included in
	 * this counter */
	nveu64_t mmc_rx_ipv6_hderr_h;
	/** This counter provides the number of bytes received in IPv6
	 * datagrams that did not have a TCP, UDP, or ICMP payload. The value
	 * in the Length field of IPv6 header is used to update this counter.
	 * (Ethernet header, FCS, pad, or IP pad bytes are not included
	 * in this counter */
	nveu64_t mmc_rx_ipv6_nopay;
	/** This counter provides upper 32 bytes received in IPv6
	 * datagrams that did not have a TCP, UDP, or ICMP payload. The value
	 * in the Length field of IPv6 header is used to update this counter.
	 * (Ethernet header, FCS, pad, or IP pad bytes are not included
	 * in this counter */
	nveu64_t mmc_rx_ipv6_nopay_h;
	/* Protocols */
	/** This counter provides the number of bytes received in a good UDP
	 * segment. This counter does not count IP header bytes */
	nveu64_t mmc_rx_udp_gd_octets;
	/** This counter provides upper 32 bytes received in a good UDP
	 * segment. This counter does not count IP header bytes */
	nveu64_t mmc_rx_udp_gd_octets_h;
	/** This counter provides the number of bytes received in a UDP
	 * segment that had checksum errors. This counter does not count
	 * IP header bytes */
	nveu64_t mmc_rx_udp_err_octets;
	/** This counter provides upper 32 bytes received in a UDP
	 * segment that had checksum errors. This counter does not count
	 * IP header bytes */
	nveu64_t mmc_rx_udp_err_octets_h;
	/** This counter provides the number of bytes received in a good
	 * TCP segment. This counter does not count IP header bytes */
	nveu64_t mmc_rx_tcp_gd_octets;
	/** This counter provides upper 32 bytes received in a good
	 * TCP segment. This counter does not count IP header bytes */
	nveu64_t mmc_rx_tcp_gd_octets_h;
	/** This counter provides the number of bytes received in a TCP
	 * segment that had checksum errors. This counter does not count
	 * IP header bytes */
	nveu64_t mmc_rx_tcp_err_octets;
	/** This counter provides upper 32 bytes received in a TCP
	 * segment that had checksum errors. This counter does not count
	 * IP header bytes */
	nveu64_t mmc_rx_tcp_err_octets_h;
	/** This counter provides the number of bytes received in a good
	 * ICMP segment. This counter does not count IP header bytes */
	nveu64_t mmc_rx_icmp_gd_octets;
	/** This counter provides upper 32 bytes received in a good
	 * ICMP segment. This counter does not count IP header bytes */
	nveu64_t mmc_rx_icmp_gd_octets_h;
	/** This counter provides the number of bytes received in a ICMP
	 * segment that had checksum errors. This counter does not count
	 * IP header bytes */
	nveu64_t mmc_rx_icmp_err_octets;
	/** This counter provides upper 32 bytes received in a ICMP
	 * segment that had checksum errors. This counter does not count
	 * IP header bytes */
	nveu64_t mmc_rx_icmp_err_octets_h;
};

/**
 * @brief osi_xtra_stat_counters - OSI core extra stat counters
 */
struct osi_xtra_stat_counters {
	/** RX buffer unavailable irq count */
	nveu64_t rx_buf_unavail_irq_n[OSI_EQOS_MAX_NUM_QUEUES];
	/** Transmit Process Stopped irq count */
	nveu64_t tx_proc_stopped_irq_n[OSI_EQOS_MAX_NUM_QUEUES];
	/** Transmit Buffer Unavailable irq count */
	nveu64_t tx_buf_unavail_irq_n[OSI_EQOS_MAX_NUM_QUEUES];
	/** Receive Process Stopped irq count */
	nveu64_t rx_proc_stopped_irq_n[OSI_EQOS_MAX_NUM_QUEUES];
	/** Receive Watchdog Timeout irq count */
	nveu64_t rx_watchdog_irq_n;
	/** Fatal Bus Error irq count */
	nveu64_t fatal_bus_error_irq_n;
	/** rx skb allocation failure count */
	nveu64_t re_alloc_rxbuf_failed[OSI_EQOS_MAX_NUM_QUEUES];
	/** TX per channel interrupt count */
	nveu64_t tx_normal_irq_n[OSI_EQOS_MAX_NUM_QUEUES];
	/** TX per channel SW timer callback count */
	nveu64_t tx_usecs_swtimer_n[OSI_EQOS_MAX_NUM_QUEUES];
	/** RX per channel interrupt count */
	nveu64_t rx_normal_irq_n[OSI_EQOS_MAX_NUM_QUEUES];
	/** link connect count */
	nveu64_t link_connect_count;
	/** link disconnect count */
	nveu64_t link_disconnect_count;
};

/**
 * @brief osi_xtra_dma_stat_counters -  OSI DMA extra stats counters
 */
struct osi_xtra_dma_stat_counters {
	/** Per Q TX packet count */
	nveu64_t q_tx_pkt_n[OSI_EQOS_MAX_NUM_QUEUES];
	/** Per Q RX packet count */
	nveu64_t q_rx_pkt_n[OSI_EQOS_MAX_NUM_QUEUES];
	/** Per Q TX complete call count */
	nveu64_t tx_clean_n[OSI_EQOS_MAX_NUM_QUEUES];
	/** Total number of tx packets count */
	nveu64_t tx_pkt_n;
	/** Total number of rx packet count */
	nveu64_t rx_pkt_n;
	/** Total number of VLAN RX packet count */
	nveu64_t rx_vlan_pkt_n;
	/** Total number of VLAN TX packet count */
	nveu64_t tx_vlan_pkt_n;
	/** Total number of TSO packet count */
	nveu64_t tx_tso_pkt_n;
};

#endif
