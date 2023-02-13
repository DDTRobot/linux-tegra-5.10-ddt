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

#ifndef OSI_DMA_H
#define OSI_DMA_H

#include "osi_common.h"
#include "osi_dma_txrx.h"
#include "mmc.h"

/**
 * @addtogroup EQOS-PKT Packet context fields
 *
 * @brief These flags are used to convey context information about a packet
 * between HW and SW. The context information includes
 * whether a VLAN tag is to be inserted for a packet,
 * whether a received packet is valid,
 * whether checksum offload is to be enabled for the packet upon transmit,
 * whether IP checksum offload is to be enabled for the packet upon transmit,
 * whether TCP segmentation offload is to be enabled for the packet,
 * whether the HW should timestamp transmit/arrival of a packet respectively,
 * whether tx payload length to be updated
 * @{
 */
/** VLAN packet */
#define OSI_PKT_CX_VLAN			OSI_BIT(0)
/** CSUM packet */
#define OSI_PKT_CX_CSUM			OSI_BIT(1)
/** TSO packet */
#define OSI_PKT_CX_TSO			OSI_BIT(2)
/** PTP packet */
#define OSI_PKT_CX_PTP			OSI_BIT(3)
/** Valid packet */
#define OSI_PKT_CX_VALID		OSI_BIT(10)
/** Update Packet Length in Tx Desc3 */
#define OSI_PKT_CX_LEN			OSI_BIT(11)
/** IP CSUM packet */
#define OSI_PKT_CX_IP_CSUM		OSI_BIT(12)

/** @} */

/**
 * @addtogroup SLOT function context fields
 *
 * @brief These flags are used for DMA channel Slot context configuration
 * @{
 */
#define OSI_SLOT_INTVL_DEFAULT		125U
#define OSI_SLOT_INTVL_MAX		4095U
#define OSI_SLOT_NUM_MAX		16U
/** @} */

/**
 * @addtogroup EQOS-TX Tx done packet context fields
 *
 * @brief These flags used to convey transmit done packet context information,
 * whether transmitted packet used a paged buffer, whether transmitted packet
 * has an tx error, whether transmitted packet has an TS
 * 
 * @{
 */
/** Flag to indicate if buffer programmed in desc. is DMA map'd from
 * linear/Paged buffer from OS layer */
#define OSI_TXDONE_CX_PAGED_BUF		OSI_BIT(0)
/** Flag to indicate if there was any tx error */
#define OSI_TXDONE_CX_ERROR		OSI_BIT(1)
/** Flag to indicate the availability of time stamp */
#define OSI_TXDONE_CX_TS		OSI_BIT(2)
/** @} */

/**
 * @addtogroup EQOS-CHK Checksum offload results
 *
 * @brief Flag to indicate the result from checksum offload engine
 * to SW network stack in receive path.
 * OSI_CHECKSUM_NONE indicates that HW checksum offload
 * engine did not verify the checksum, SW network stack has to do it.
 * OSI_CHECKSUM_UNNECESSARY indicates that HW validated the
 * checksum already, network stack can skip validation.
 * @{
 */
/* Checksum offload result flags */
#define OSI_CHECKSUM_NONE		0x0U
/* TCP header/payload */
#define OSI_CHECKSUM_TCPv4		OSI_BIT(0)
/* UDP header/payload */
#define OSI_CHECKSUM_UDPv4		OSI_BIT(1)
/* TCP/UDP checksum bad */
#define OSI_CHECKSUM_TCP_UDP_BAD	OSI_BIT(2)
/* IPv6 TCP header/payload */
#define OSI_CHECKSUM_TCPv6		OSI_BIT(4)
/* IPv6 UDP header/payload */
#define OSI_CHECKSUM_UDPv6		OSI_BIT(5)
/* IPv4 header */
#define OSI_CHECKSUM_IPv4		OSI_BIT(6)
/* IPv4 header checksum bad */
#define OSI_CHECKSUM_IPv4_BAD		OSI_BIT(7)
/* Checksum check not required */
#define OSI_CHECKSUM_UNNECESSARY	OSI_BIT(8)
/** @} */

/**
 * @addtogroup EQOS-RX Rx SW context flags
 *
 * @brief Flags to share info about the Rx SW context structure per descriptor
 * between OSI and OSD.
 * @{
 */
/* Rx swcx flags */
#define OSI_RX_SWCX_REUSE	OSI_BIT(0)
#define OSI_RX_SWCX_BUF_VALID	OSI_BIT(1)
/** Packet is processed by driver */
#define OSI_RX_SWCX_PROCESSED	OSI_BIT(3)

/** @} */

/**
 * @brief OSI packet error stats
 */
struct osi_pkt_err_stats {
	/** IP Header Error */
	unsigned long ip_header_error;
	/** Jabber time out Error */
	unsigned long jabber_timeout_error;
	/** Packet Flush Error */
	unsigned long pkt_flush_error;
	/** Payload Checksum Error */
	unsigned long payload_cs_error;
	/** Loss of Carrier Error */
	unsigned long loss_of_carrier_error;
	/** No Carrier Error */
	unsigned long no_carrier_error;
	/** Late Collision Error */
	unsigned long late_collision_error;
	/** Excessive Collision Error */
	unsigned long excessive_collision_error;
	/** Excessive Deferal Error */
	unsigned long excessive_deferal_error;
	/** Under Flow Error */
	unsigned long underflow_error;
	/** Rx CRC Error */
	unsigned long rx_crc_error;
	/** Rx Frame Error */
	unsigned long rx_frame_error;
	/** clear_tx_pkt_err_stats() API invoked */
	unsigned long clear_tx_err;
	/** clear_rx_pkt_err_stats() API invoked */
	unsigned long clear_rx_err;
};

/**
 * @brief Receive Descriptor
 */
struct osi_rx_desc {
	/** Receive Descriptor 0 */
	unsigned int rdes0;
	/** Receive Descriptor 1 */
	unsigned int rdes1;
	/** Receive Descriptor 2 */
	unsigned int rdes2;
	/** Receive Descriptor 3 */
	unsigned int rdes3;
};

/**
 * @brief Receive descriptor software context
 */
struct osi_rx_swcx {
	/** DMA buffer physical address */
	unsigned long buf_phy_addr;
	/** DMA buffer virtual address */
	void *buf_virt_addr;
	/** Length of buffer */
	unsigned int len;
	/** Flags to share info about Rx swcx between OSD and OSI */
	unsigned int flags;
};

/**
 * @brief - Received packet context. This is a single instance
 * and it is reused for all rx packets.
 */
struct osi_rx_pkt_cx {
	/** Bit map which holds the features that rx packets supports */
	unsigned int flags;
	/** Stores the Rx csum */
	unsigned int rxcsum;
	/** Stores the VLAN tag ID in received packet */
	unsigned int vlan_tag;
	/** Length of received packet */
	unsigned int pkt_len;
	/** TS in nsec for the received packet */
	unsigned long long ns;
};

/**
 * @brief DMA channel Rx ring. The number of instances depends on the
 * number of DMA channels configured
 */
struct osi_rx_ring {
	/** Pointer to Rx DMA descriptor */
	struct osi_rx_desc *rx_desc;
	/** Pointer to Rx DMA descriptor software context information */
	struct osi_rx_swcx *rx_swcx;
	/** Physical address of Rx DMA descriptor */
	unsigned long rx_desc_phy_addr;
	/** Descriptor index current reception */
	unsigned int cur_rx_idx;
	/** Descriptor index for descriptor re-allocation */
	unsigned int refill_idx;
	/** Receive packet context */
	struct osi_rx_pkt_cx rx_pkt_cx;
};

/**
 *@brief Transmit descriptor software context
 */
struct osi_tx_swcx {
	/** Physical address of DMA mapped buffer */
	unsigned long buf_phy_addr;
	/** Virtual address of DMA buffer */
	void *buf_virt_addr;
	/** Length of buffer */
	unsigned int len;
	/** Flag to keep track of whether buffer pointed by buf_phy_addr
	 * is a paged buffer/linear buffer */
	unsigned int is_paged_buf;
};

/**
 * @brief Transmit descriptor
 */
struct osi_tx_desc {
	/** Transmit descriptor 0 */
	unsigned int tdes0;
	/** Transmit descriptor 1 */
	unsigned int tdes1;
	/** Transmit descriptor 2 */
	unsigned int tdes2;
	/** Transmit descriptor 3 */
	unsigned int tdes3;
};

/**
 * @brief Transmit packet context for a packet. This is a single instance
 * and it is reused for all tx packets.
 */
struct osi_tx_pkt_cx {
	/** Holds the features which a Tx packets supports */
	unsigned int flags;
	/** Stores the VLAN tag ID */
	unsigned int vtag_id;
	/** Descriptor count */
	unsigned int desc_cnt;
	/** Max. segment size for TSO/USO/GSO/LSO packet */
	unsigned int mss;
	/** Length of application payload */
	unsigned int payload_len;
	/** Length of transport layer tcp/udp header */
	unsigned int tcp_udp_hdrlen;
	/** Length of all headers (ethernet/ip/tcp/udp) */
	unsigned int total_hdrlen;
};

/**
 * @brief Transmit done packet context for a packet
 */
struct osi_txdone_pkt_cx {
	/** Indicates status flags for Tx complete (tx error occurred, or
	 * indicate whether desc had buf mapped from paged/linear memory etc) */
	unsigned int flags;
	/** TS captured for the tx packet and this is valid only when the PTP
	 * bit is set in fields */
	unsigned long long ns;
};

/**
 * @brief DMA channel Tx ring. The number of instances depends on the
 * number of DMA channels configured
 */
struct osi_tx_ring {
	/** Pointer to tx dma descriptor */
	struct osi_tx_desc *tx_desc;
	/** Pointer to tx dma descriptor software context information */
	struct osi_tx_swcx *tx_swcx;
	/** Physical address of Tx descriptor */
	unsigned long tx_desc_phy_addr;
	/** Descriptor index current transmission */
	unsigned int cur_tx_idx;
	/** Descriptor index for descriptor cleanup */
	unsigned int clean_idx;
	/** Slot function check */
	unsigned int slot_check;
	/** Slot number */
	unsigned int slot_number;
	/** Transmit packet context */
	struct osi_tx_pkt_cx tx_pkt_cx;
	/** Transmit complete packet context information */
	struct osi_txdone_pkt_cx txdone_pkt_cx;
	/** Number of packets or frames transmitted */
	unsigned int frame_cnt;
};

struct osi_dma_priv_data;

/**
 * @brief MAC DMA Channel operations
 */
struct osi_dma_chan_ops {
	/** Called to set Transmit Ring length */
	void (*set_tx_ring_len)(void *addr, unsigned int chan,
				unsigned int len);
	/** Called to set Transmit Ring Base address */
	void (*set_tx_ring_start_addr)(void *addr, unsigned int chan,
				       unsigned long base_addr);
	/** Called to update Tx Ring tail pointer */
	void (*update_tx_tailptr)(void *addr, unsigned int chan,
				  unsigned long tailptr);
	/** Called to set Receive channel ring length */
	void (*set_rx_ring_len)(void *addr, unsigned int chan,
				unsigned int len);
	/** Called to set receive channel ring base address */
	void (*set_rx_ring_start_addr)(void *addr, unsigned int chan,
				       unsigned long base_addr);
	/** Called to update Rx ring tail pointer */
	void (*update_rx_tailptr)(void *addr, unsigned int chan,
				  unsigned long tailptr);
	/** Called to disable DMA Tx channel interrupts at wrapper level */
	void (*disable_chan_tx_intr)(void *addr, unsigned int chan);
	/** Called to enable DMA Tx channel interrupts at wrapper level */
	void (*enable_chan_tx_intr)(void *addr, unsigned int chan);
	/** Called to disable DMA Rx channel interrupts at wrapper level */
	void (*disable_chan_rx_intr)(void *addr, unsigned int chan);
	/** Called to enable DMA Rx channel interrupts at wrapper level */
	void (*enable_chan_rx_intr)(void *addr, unsigned int chan);
	/** Called to start the Tx/Rx DMA */
	void (*start_dma)(void *addr, unsigned int chan);
	/** Called to stop the Tx/Rx DMA */
	void (*stop_dma)(void *addr, unsigned int chan);
	/** Called to initialize the DMA channel */
	void (*init_dma_channel) (struct osi_dma_priv_data *osi_dma);
	/** Called to set Rx buffer length */
	void (*set_rx_buf_len)(struct osi_dma_priv_data *osi_dma);
#ifndef OSI_STRIPPED_LIB
	/** Called periodically to read and validate safety critical
	 * registers against last written value */
	int (*validate_regs)(struct osi_dma_priv_data *osi_dma);
	/** Called to configure the DMA channel slot function */
	void (*config_slot)(struct osi_dma_priv_data *osi_dma,
			    unsigned int chan,
			    unsigned int set,
			    unsigned int interval);
#endif /* !OSI_STRIPPED_LIB */
	/** Called to get Global DMA status */
	unsigned int (*get_global_dma_status)(void *addr);
	/** Called to clear VM Tx interrupt */
	void (*clear_vm_tx_intr)(void *addr, unsigned int chan);
	/** Called to clear VM Rx interrupt */
	void (*clear_vm_rx_intr)(void *addr, unsigned int chan);
};

/**
 * @brief OSI VM IRQ data
 */
struct osi_vm_irq_data {
	/** Number of VM channels per VM IRQ */
	unsigned int num_vm_chans;
	/** Array of VM channel list */
	unsigned int vm_chans[OSI_EQOS_MAX_NUM_CHANS];
};

/**
 *@brief OSD DMA callbacks
 */
struct osd_dma_ops {
	/** DMA transmit complete callback */
	void (*transmit_complete)(void *priv, void *buffer,
				  unsigned long dmaaddr, unsigned int len,
				  void *txdone_pkt_cx);
	/** DMA receive packet callback */
	void (*receive_packet)(void *priv, void *rxring,
			       unsigned int chan, unsigned int dma_buf_len,
			       void *rxpkt_cx, void *rx_pkt_swcx);
	/** RX buffer reallocation callback */
	void (*realloc_buf)(void *priv, void *rxring, unsigned int chan);
};

/**
 * @brief The OSI DMA private data structure.
 */
struct osi_dma_priv_data {
	/** Array of pointers to DMA Tx channel Ring */
	struct osi_tx_ring *tx_ring[OSI_EQOS_MAX_NUM_CHANS];
	/** Array of pointers to DMA Rx channel Ring */
	struct osi_rx_ring *rx_ring[OSI_EQOS_MAX_NUM_CHANS];
	/** Memory mapped base address of MAC IP */
	void *base;
	/** Pointer to OSD private data structure */
	void *osd;
	/** Address of HW operations structure */
	struct osi_dma_chan_ops *ops;
	/** MAC HW type (EQOS) */
	unsigned int mac;
	/** Number of channels enabled in MAC */
	unsigned int num_dma_chans;
	/** Array of supported DMA channels */
	unsigned int dma_chans[OSI_EQOS_MAX_NUM_CHANS];
	/** DMA Rx channel buffer length at HW level */
	unsigned int rx_buf_len;
	/** MTU size */
	unsigned int mtu;
	/** Packet error stats */
	struct osi_pkt_err_stats pkt_err_stats;
	/** Extra DMA stats */
	struct osi_xtra_dma_stat_counters dstats;
	/** Receive Interrupt Watchdog Timer Count Units */
	unsigned int rx_riwt;
	/** Flag which decides riwt is enabled(1) or disabled(0) */
	unsigned int use_riwt;
	/** Max no of pkts to be received before triggering Rx interrupt */
	unsigned int rx_frames;
	/** Flag which decides rx_frames is enabled(1) or disabled(0) */
	unsigned int use_rx_frames;
	/** Transmit Interrupt Software Timer Count Units */
	unsigned int tx_usecs;
	/** Flag which decides Tx timer is enabled(1) or disabled(0) */
	unsigned int use_tx_usecs;
	/** Max no of pkts to transfer before triggering Tx interrupt */
	unsigned int tx_frames;
	/** Flag which decides tx_frames is enabled(1) or disabled(0) */
	unsigned int use_tx_frames;
	/** Flag which decides virtualization is enabled(1) or disabled(0) */
	unsigned int use_virtualization;
	/** Functional safety config to do periodic read-verify of
	 * certain safety critical dma registers */
	void *safety_config;
	/** Array of DMA channel slot snterval value from DT */
	unsigned int slot_interval[OSI_EQOS_MAX_NUM_CHANS];
	/** Array of DMA channel slot enabled status from DT*/
	unsigned int slot_enabled[OSI_EQOS_MAX_NUM_CHANS];
	/** number of VM IRQ's */
	unsigned int num_vm_irqs;
	/** Array of VM IRQ's */
	struct osi_vm_irq_data irq_data[OSI_MAX_VM_IRQS];
	/** DMA callback ops structure */
	struct osd_dma_ops osd_ops;
	/** Virtual address of reserved DMA buffer */
	void *resv_buf_virt_addr;
	/** Physical address of reserved DMA buffer */
	unsigned long resv_buf_phy_addr;
};


/**
 * @brief osi_disable_chan_tx_intr - Disables DMA Tx channel interrupts.
 *
 * @note
 * Algorithm:
 *  - Disables Tx interrupts at wrapper level.
 *
 * @param[in] osi_dma: DMA private data.
 * @param[in] chan: DMA Tx channel number.
 *
 * @pre
 *  - MAC needs to be out of reset and proper clocks need to be configured.
 *  - DMA HW init need to be completed successfully, see osi_hw_dma_init
 *  - Mapping of physical IRQ line to DMA channel need to be maintained at
 *    OS Dependent layer and pass corresponding channel number.
 *
 * @note
 * Traceability Details:
 * - SWUD_ID: ETHERNET_NVETHERNETCL_001
 *
 * @note
 * Classification:
 * - Interrupt: No
 * - Signal handler: No
 * - Thread safe: No
 * - Required Privileges: None
 *
 * @note
 * API Group:
 * - Initialization: Yes
 * - Run time: Yes
 * - De-initialization: Yes
 *
 * @retval 0 on success
 * @retval -1 on failure.
 */
int osi_disable_chan_tx_intr(struct osi_dma_priv_data *osi_dma,
			     unsigned int chan);

/**
 * @brief osi_enable_chan_tx_intr - Enable DMA Tx channel interrupts.
 *
 * @note
 * Algorithm:
 *  - Enables Tx interrupts at wrapper level.
 *
 * @param[in] osi_dma: DMA private data.
 * @param[in] chan: DMA Tx channel number.
 *
 * @pre
 *  - MAC needs to be out of reset and proper clocks need to be configured.
 *  - DMA HW init need to be completed successfully, see osi_hw_dma_init
 *  - Mapping of physical IRQ line to DMA channel need to be maintained at
 *    OS Dependent layer and pass corresponding channel number.
 *
 * @note
 * Traceability Details:
 * - SWUD_ID: ETHERNET_NVETHERNETCL_002
 *
 * @note
 * Classification:
 * - Interrupt: No
 * - Signal handler: No
 * - Thread safe: No
 * - Required Privileges: None
 *
 * @note
 * API Group:
 * - Initialization: Yes
 * - Run time: Yes
 * - De-initialization: Yes
 *
 * @retval 0 on success
 * @retval -1 on failure.
 */
int osi_enable_chan_tx_intr(struct osi_dma_priv_data *osi_dma,
			    unsigned int chan);

/**
 * @brief osi_disable_chan_rx_intr - Disable DMA Rx channel interrupts.
 *
 * @note
 * Algorithm:
 *  - Disables Rx interrupts at wrapper level.
 *
 * @param[in] osi_dma: DMA private data.
 * @param[in] chan: DMA rx channel number.
 *
 * @pre
 *  - MAC needs to be out of reset and proper clocks need to be configured.
 *  - DMA HW init need to be completed successfully, see osi_hw_dma_init
 *  - Mapping of physical IRQ line to DMA channel need to be maintained at
 *    OS Dependent layer and pass corresponding channel number.
 *
 * @note
 * Traceability Details:
 * - SWUD_ID: ETHERNET_NVETHERNETCL_003
 *
 * @note
 * Classification:
 * - Interrupt: No
 * - Signal handler: No
 * - Thread safe: No
 * - Required Privileges: None
 *
 * @note
 * API Group:
 * - Initialization: Yes
 * - Run time: Yes
 * - De-initialization: Yes
 *
 * @retval 0 on success
 * @retval -1 on failure.
 */
int osi_disable_chan_rx_intr(struct osi_dma_priv_data *osi_dma,
			     unsigned int chan);

/**
 * @brief osi_enable_chan_rx_intr - Enable DMA Rx channel interrupts.
 *
 * @note
 * Algorithm:
 *  - Enables Rx interrupts at wrapper level.
 *
 * @param[in] osi_dma: DMA private data.
 * @param[in] chan: DMA rx channel number.
 *
 * @pre
 *  - MAC needs to be out of reset and proper clocks need to be configured.
 *  - DMA HW init need to be completed successfully, see osi_hw_dma_init
 *  - Mapping of physical IRQ line to DMA channel need to be maintained at
 *    OS Dependent layer and pass corresponding channel number.
 *
 * @note
 * Traceability Details:
 * - SWUD_ID: ETHERNET_NVETHERNETCL_004
 *
 * @note
 * Classification:
 * - Interrupt: No
 * - Signal handler: No
 * - Thread safe: No
 * - Required Privileges: None
 *
 * @note
 * API Group:
 * - Initialization: Yes
 * - Run time: Yes
 * - De-initialization: Yes
 *
 * @retval 0 on success
 * @retval -1 on failure.
 */
int osi_enable_chan_rx_intr(struct osi_dma_priv_data *osi_dma,
			    unsigned int chan);

/**
 * @brief osi_get_global_dma_status - Gets DMA status.
 *
 * Algorithm: Returns global DMA Tx/Rx interrupt status
 *
 * @param[in] osi_dma: DMA private data.
 * @param[in] chan: DMA tx channel number.
 *
 * @note
 *	Dependencies: None.
 *	Protection: None.
 *
 * @retval status
 */
unsigned int osi_get_global_dma_status(struct osi_dma_priv_data *osi_dma);

/**
 * @brief osi_clear_vm_tx_intr - Handles VM Tx interrupt source.
 *
 * Algorithm: Clear Tx interrupt source at wrapper level and DMA level.
 *
 * @param[in] osi_dma: DMA private data.
 * @param[in] chan: DMA tx channel number.
 *
 * @note
 *	1) MAC needs to be out of reset and proper clocks need to be configured.
 *	2) DMA HW init need to be completed successfully, see osi_hw_dma_init
 *
 * @retval 0 on success
 * @retval -1 on failure.
 */
int osi_clear_vm_tx_intr(struct osi_dma_priv_data *osi_dma,
			 unsigned int chan);

/**
 * @brief osi_clear_vm_rx_intr - Handles VM Rx interrupt source.
 *
 * Algorithm: Clear Rx interrupt source at wrapper level and DMA level.
 *
 * @param[in] osi_dma: DMA private data.
 * @param[in] chan: DMA rx channel number.
 *
 * @note
 *	1) MAC needs to be out of reset and proper clocks need to be configured.
 *	2) DMA HW init need to be completed successfully, see osi_hw_dma_init
 *	3) Mapping of physical IRQ line to DMA channel need to be maintained at
 *	OS Dependent layer and pass corresponding channel number.
 *
 * @retval 0 on success
 * @retval -1 on failure.
 */
int osi_clear_vm_rx_intr(struct osi_dma_priv_data *osi_dma,
			 unsigned int chan);

/**
 * @brief Start DMA
 *
 * @note
 * Algorithm:
 *  - Start the DMA for specific MAC
 *
 * @param[in] osi_dma: DMA private data.
 * @param[in] chan: DMA Tx/Rx channel number
 *
 * @pre
 *  - MAC needs to be out of reset and proper clocks need to be configured.
 *  - DMA HW init need to be completed successfully, see osi_hw_dma_init
 *
 * @note
 * Traceability Details:
 * - SWUD_ID: ETHERNET_NVETHERNETCL_005
 *
 * @note
 * Classification:
 * - Interrupt: No
 * - Signal handler: No
 * - Thread safe: No
 * - Required Privileges: None
 *
 * @note
 * API Group:
 * - Initialization: Yes
 * - Run time: No
 * - De-initialization: No
 *
 * @retval 0 on success
 * @retval -1 on failure.
 */
int osi_start_dma(struct osi_dma_priv_data *osi_dma,
		  unsigned int chan);

/**
 * @brief osi_stop_dma - Stop DMA
 *
 * @note
 * Algorithm:
 *  - Stop the DMA for specific MAC
 *
 * @param[in] osi_dma: DMA private data.
 * @param[in] chan: DMA Tx/Rx channel number
 *
 * @pre
 *  - MAC needs to be out of reset and proper clocks need to be configured.
 *  - DMA HW init need to be completed successfully, see osi_hw_dma_init
 *
 * @note
 * Traceability Details:
 * - SWUD_ID: ETHERNET_NVETHERNETCL_006
 *
 * @note
 * Classification:
 * - Interrupt: No
 * - Signal handler: No
 * - Thread safe: No
 * - Required Privileges: None
 *
 * @note
 * API Group:
 * - Initialization: No
 * - Run time: No
 * - De-initialization: Yes
 *
 * @retval 0 on success
 * @retval -1 on failure.
 */
int osi_stop_dma(struct osi_dma_priv_data *osi_dma,
		 unsigned int chan);

/**
 * @brief osi_get_refill_rx_desc_cnt - Rx descriptors count that needs to refill
 *
 * @note
 * Algorithm:
 *  - subtract current index with fill (need to cleanup)
 *    to get Rx descriptors count that needs to refill.
 *
 * @param[in] rx_ring: DMA channel Rx ring.
 *
 * @note
 * Traceability Details:
 * - SWUD_ID: ETHERNET_NVETHERNETCL_007
 *
 * @note
 * Classification:
 * - Interrupt: No
 * - Signal handler: No
 * - Thread safe: No
 * - Required Privileges: None
 *
 * @note
 * API Group:
 * - Initialization: No
 * - Run time: Yes
 * - De-initialization: No
 *
 * @retval "Number of available free descriptors."
 */
unsigned int osi_get_refill_rx_desc_cnt(struct osi_rx_ring *rx_ring);

/**
 * @brief osi_rx_dma_desc_init - DMA Rx descriptor init
 *
 * @note
 * Algorithm:
 *  - Initialize a Rx DMA descriptor.
 *
 * @param[in] osi_dma: OSI DMA private data structure.
 * @param[in] rx_ring: HW ring corresponding to Rx DMA channel.
 * @param[in] chan: Rx DMA channel number
 *
 * @pre
 *  - MAC needs to be out of reset and proper clocks need to be configured.
 *  - rx_swcx->buf_phy_addr need to be filled with DMA mapped address
 *  - DMA HW init need to be completed successfully, see osi_hw_dma_init
 *
 * @note
 * Traceability Details:
 * - SWUD_ID: ETHERNET_NVETHERNETCL_008
 *
 * @note
 * Classification:
 * - Interrupt: No
 * - Signal handler: No
 * - Thread safe: No
 * - Required Privileges: None
 *
 * @note
 * API Group:
 * - Initialization: Yes
 * - Run time: Yes
 * - De-initialization: No
 *
 * @retval 0 on success
 * @retval -1 on failure.
 */
int osi_rx_dma_desc_init(struct osi_dma_priv_data *osi_dma,
			 struct osi_rx_ring *rx_ring, unsigned int chan);

/**
 * @brief Updates rx buffer length.
 *
 * @param[in] osi_dma: OSI DMA private data structure.
 *
 * @pre
 *  - MAC needs to be out of reset and proper clocks need to be configured.
 *  - DMA HW init need to be completed successfully, see osi_hw_dma_init
 *  - osi_dma->mtu need to be filled with current MTU size <= 9K
 *
 * @note
 * Traceability Details:
 * - SWUD_ID: ETHERNET_NVETHERNETCL_009
 *
 * @note
 * Classification:
 * - Interrupt: No
 * - Signal handler: No
 * - Thread safe: No
 * - Required Privileges: None
 *
 * @note
 * API Group:
 * - Initialization: Yes
 * - Run time: No
 * - De-initialization: No
 *
 * @retval 0 on success
 * @retval -1 on failure.
 */
int osi_set_rx_buf_len(struct osi_dma_priv_data *osi_dma);

/**
 * @brief osi_hw_transmit - Initialize Tx DMA descriptors for a channel
 *
 * @note
 * Algorithm:
 *  - Initialize Transmit descriptors with DMA mappable buffers,
 *    set OWN bit, Tx ring length and set starting address of Tx DMA channel
 *    Tx ring base address in Tx DMA registers.
 *
 * @param[in] osi: DMA private data.
 * @param[in] chan: DMA Tx channel number.
 *
 * @pre
 *  - MAC needs to be out of reset and proper clocks need to be configured.
 *  - DMA HW init need to be completed successfully, see osi_hw_dma_init
 *  - DMA channel need to be started, see osi_start_dma
 *  - Need to set update tx_pkt_cx->flags accordingly as per the
 *    requirements
 *    OSI_PKT_CX_VLAN                 OSI_BIT(0)
 *    OSI_PKT_CX_CSUM                 OSI_BIT(1)
 *    OSI_PKT_CX_TSO                  OSI_BIT(2)
 *    OSI_PKT_CX_PTP                  OSI_BIT(3)
 *  - tx_pkt_cx->desc_cnt need to be populated which holds the number
 *    of swcx descriptors allocated for that packet
 *  - tx_swcx structure need to be filled for per packet with the
 *    buffer len, DMA mapped address of buffer for each descriptor
 *    consumed by the packet
 *
 * @note
 * Traceability Details:
 * - SWUD_ID: ETHERNET_NVETHERNETCL_010
 *
 * @note
 * Classification:
 * - Interrupt: No
 * - Signal handler: No
 * - Thread safe: No
 * - Required Privileges: None
 *
 * @note
 * API Group:
 * - Initialization: No
 * - Run time: Yes
 * - De-initialization: No
 *
 */
void osi_hw_transmit(struct osi_dma_priv_data *osi, unsigned int chan);

/**
 * @brief osi_process_tx_completions - Process Tx complete on DMA channel ring.
 *
 * @note
 * Algorithm:
 *  - This function will be invoked by OSD layer to process Tx
 *    complete interrupt.
 *    - First checks whether descriptor owned by DMA or not.
 *    - Invokes OSD layer to release DMA address and Tx buffer which are
 *      updated as part of transmit routine.
 *
 * @param[in] osi: OSI private data structure.
 * @param[in] chan: Channel number on which Tx complete need to be done.
 * @param[in] budget: Threshold for reading the packets at a time.
 *
 * @pre
 *  - MAC needs to be out of reset and proper clocks need to be configured.
 *  - DMA HW init need to be completed successfully, see osi_hw_dma_init
 *  - DMA need to be started, see osi_start_dma
 *
 * @note
 * Traceability Details:
 * - SWUD_ID: ETHERNET_NVETHERNETCL_011
 *
 * @note
 * Classification:
 * - Interrupt: Yes
 * - Signal handler: Yes
 * - Thread safe: No
 * - Required Privileges: None
 *
 * @note
 * API Group:
 * - Initialization: No
 * - Run time: Yes
 * - De-initialization: No
 *
 * @returns Number of descriptors (buffers) processed.
 */
int osi_process_tx_completions(struct osi_dma_priv_data *osi,
			       unsigned int chan, int budget);

/**
 * @brief osi_process_rx_completions - Read data from rx channel descriptors
 *
 * @note
 * Algorithm:
 *  - This routine will be invoked by OSD layer to get the
 *    data from Rx descriptors and deliver the packet to the stack.
 *    - Checks descriptor owned by DMA or not.
 *    - If rx buffer is reserve buffer, reallocate receive buffer and
 *      read next descriptor.
 *    - Get the length from Rx descriptor
 *    - Invokes OSD layer to deliver the packet to network stack.
 *    - Re-allocate the receive buffers, populate Rx descriptor and
 *      handover to DMA.
 *
 * @param[in] osi: OSI private data structure.
 * @param[in] chan: Rx DMA channel number
 * @param[in] budget: Threshold for reading the packets at a time.
 * @param[in] more_data_avail: Pointer to more data available flag. OSI fills
 *         this flag if more rx packets available to read(1) or not(0).
 *
 * @pre
 *  - MAC needs to be out of reset and proper clocks need to be configured.
 *  - DMA HW init need to be completed successfully, see osi_hw_dma_init
 *  - DMA need to be started, see osi_start_dma
 *
 * @note
 * Traceability Details:
 * - SWUD_ID: ETHERNET_NVETHERNETCL_012
 *
 * @note
 * Classification:
 * - Interrupt: Yes
 * - Signal handler: Yes
 * - Thread safe: No
 * - Required Privileges: None
 *
 * @note
 * API Group:
 * - Initialization: No
 * - Run time: Yes
 * - De-initialization: No
 *
 * @returns Number of descriptors (buffers) processed.
 */
int osi_process_rx_completions(struct osi_dma_priv_data *osi,
			       unsigned int chan, int budget,
			       unsigned int *more_data_avail);

/**
 * @brief osi_hw_dma_init - Initialize DMA
 *
 * @note
 * Algorithm:
 *  - Takes care of initializing the tx, rx ring and descriptors
 *    based on the number of channels selected.
 *
 * @param[in] osi_dma: DMA private data.
 *
 *
 * @pre
 *  - Allocate memory for osi_dma
 *  - MAC needs to be out of reset and proper clocks need to be configured.
 *  - Number of dma channels osi_dma->num_dma_chans
 *  - channel list osi_dma->dma_chan
 *  - base address osi_dma->base
 *  - allocate tx ring osi_dma->tx_ring[chan] for each channel
 *    based on TX_DESC_CNT (256)
 *  - allocate tx descriptors osi_dma->tx_ring[chan]->tx_desc for all
 *    channels and dma map it.
 *  - allocate tx sw descriptors osi_dma->tx_ring[chan]->tx_swcx for all
 *    channels
 *  - allocate rx ring osi_dma->rx_ring[chan] for each channel
 *    based on RX_DESC_CNT (256)
 *  - allocate rx descriptors osi_dma->rx_ring[chan]->rx_desc for all
 *    channels and dma map it.
 *  - allocate rx sw descriptors osi_dma->rx_ring[chan]->rx_swcx for all
 *    channels
 *  - osi_dma->use_riwt  ==> OSI_DISABLE/OSI_ENABLE
 *  - osi_dma->rx_riwt  ===> Actual value read from DT
 *  - osi_dma->use_rx_frames  ==> OSI_DISABLE/OSI_ENABLE
 *  - osi_dma->rx_frames ===> Actual value read from DT
 *
 * @note
 * Traceability Details:
 * - SWUD_ID: ETHERNET_NVETHERNETCL_013
 *
 * @note
 * Classification:
 * - Interrupt: No
 * - Signal handler: No
 * - Thread safe: No
 * - Required Privileges: None
 *
 * @note
 * API Group:
 * - Initialization: Yes
 * - Run time: No
 * - De-initialization: No
 *
 * @retval 0 on success
 * @retval -1 on failure.
 */
int osi_hw_dma_init(struct osi_dma_priv_data *osi_dma);

/**
 * @brief osi_hw_dma_deinit - De initialize DMA
 *
 * @note
 * Algorithm:
 *  - Takes care of stopping the MAC
 *
 * @param[in] osi_dma: DMA private data.
 *
 * @pre
 *  - MAC needs to be out of reset and proper clocks need to be configured.
 *  - DMA HW init need to be completed successfully, see osi_hw_dma_init
 *
 * @note
 * Traceability Details:
 * - SWUD_ID: ETHERNET_NVETHERNETCL_014
 *
 * @note
 * Classification:
 * - Interrupt: No
 * - Signal handler: No
 * - Thread safe: No
 * - Required Privileges: None
 *
 * @note
 * API Group:
 * - Initialization: No
 * - Run time: No
 * - De-initialization: Yes
 *
 * @retval 0 on success
 * @retval -1 on failure.
 */
int osi_hw_dma_deinit(struct osi_dma_priv_data *osi_dma);

/**
 * @brief osi_init_dma_ops - Initialize DMA operations
 *
 * @param[in] osi_dma: DMA private data.
 *
 * @note
 * Traceability Details:
 * - SWUD_ID: ETHERNET_NVETHERNETCL_015
 *
 * @note
 * Classification:
 * - Interrupt: No
 * - Signal handler: No
 * - Thread safe: No
 * - Required Privileges: None
 *
 * @note
 * API Group:
 * - Initialization: Yes
 * - Run time: No
 * - De-initialization: No
 *
 *
 * @retval 0 on success
 * @retval -1 on failure.
 */
int osi_init_dma_ops(struct osi_dma_priv_data *osi_dma);

/**
 * @brief osi_dma_get_systime_from_mac - Get system time
 *
 * @note
 * Algorithm:
 *  - Gets the current system time
 *
 * @param[in] osi_dma: OSI DMA private data structure.
 * @param[out] sec: Value read in Seconds
 * @param[out] nsec: Value read in Nano seconds
 *
 * @pre MAC should be init and started. see osi_start_mac()
 *
 * @note
 * Traceability Details:
 * - SWUD_ID: ETHERNET_NVETHERNETCL_016
 *
 * @note
 * Classification:
 * - Interrupt: No
 * - Signal handler: No
 * - Thread safe: No
 * - Required Privileges: None
 *
 * @note
 * API Group:
 * - Initialization: No
 * - Run time: Yes
 * - De-initialization: No
 *
 * @retval 0 on success
 * @retval -1 on failure.
 */
int osi_dma_get_systime_from_mac(
			     struct osi_dma_priv_data *const osi_dma,
			     unsigned int *sec,
			     unsigned int *nsec);

/**
 * @brief osi_is_mac_enabled - Checks if MAC is enabled.
 *
 * @note
 * Algorithm:
 *  - Reads MAC MCR register for Tx and Rx enabled bits.
 *
 * @param[in] osi_dma: OSI DMA private data structure.
 *
 * @pre MAC should be init and started. see osi_start_mac()
 *
 * @note
 * Traceability Details:
 * - SWUD_ID: ETHERNET_NVETHERNETCL_017
 *
 * @note
 * Classification:
 * - Interrupt: No
 * - Signal handler: No
 * - Thread safe: No
 * - Required Privileges: None
 *
 * @note
 * API Group:
 * - Initialization: Yes
 * - Run time: No
 * - De-initialization: No
 *
 * @retval OSI_ENABLE if MAC enabled.
 * @retval OSI_DISABLE otherwise.
 */
unsigned int osi_is_mac_enabled(struct osi_dma_priv_data *const osi_dma);

#ifndef OSI_STRIPPED_LIB
/**
 * @brief - Read-validate HW registers for func safety.
 *
 * @note
 * Algorithm:
 *  - Reads pre-configured list of DMA configuration registers
 *    and compares with last written value for any modifications.
 *
 * @param[in] osi_dma: OSI DMA private data structure.
 *
 * @pre
 *  - MAC has to be out of reset.
 *  - osi_hw_dma_init has to be called. Internally this would initialize
 *    the safety_config (see osi_dma_priv_data) based on MAC version and
 *    which specific registers needs to be validated periodically.
 *  - Invoke this call if (osi_dma_priv_data->safety_config != OSI_NULL)
 *
 * @note
 * Traceability Details:
 *
 * @note
 * Classification:
 * - Interrupt: No
 * - Signal handler: No
 * - Thread safe: No
 * - Required Privileges: None
 *
 * @note
 * API Group:
 * - Initialization: No
 * - Run time: Yes
 * - De-initialization: No
 *
 * @retval 0 on success
 * @retval -1 on failure.
 */
int osi_validate_dma_regs(struct osi_dma_priv_data *osi_dma);

/**
 * @brief osi_clear_tx_pkt_err_stats - Clear tx packet error stats.
 *
 * @note
 * Algorithm:
 *  - This function will be invoked by OSD layer to clear the
 *    tx stats mentioned in osi_dma->pkt_err_stats structure
 *
 * @param[in] osi_dma: OSI DMA private data structure.
 *
 * @pre
 *  - MAC needs to be out of reset and proper clocks need to be configured.
 *  - DMA HW init need to be completed successfully, see osi_hw_dma_init
 *
 * @note
 * Traceability Details:
 *
 * @note
 * Classification:
 * - Interrupt: No
 * - Signal handler: No
 * - Thread safe: No
 * - Required Privileges: None
 *
 * @note
 * API Group:
 * - Initialization: No
 * - Run time: Yes
 * - De-initialization: No
 *
 * @retval 0 on success
 * @retval -1 on failure.
 */
int osi_clear_tx_pkt_err_stats(struct osi_dma_priv_data *osi_dma);

/**
 * @brief osi_config_slot_function - Configure slot function
 *
 * @note
 * Algorithm:
 *  - Set or reset the slot function based on set input
 *
 * @param[in] osi_dma: OSI DMA private data structure.
 * @param[in] set: Flag to set with OSI_ENABLE and reset with OSI_DISABLE
 *
 * @pre MAC should be init and started. see osi_start_mac()
 *
 * @note
 * Traceability Details:
 *
 * @note
 * Classification:
 * - Interrupt: No
 * - Signal handler: No
 * - Thread safe: No
 * - Required Privileges: None
 *
 * @note
 * API Group:
 * - Initialization: Yes
 * - Run time: No
 * - De-initialization: No
 *
 * @retval 0 on success
 * @retval -1 on failure.
 */
int osi_config_slot_function(struct osi_dma_priv_data *osi_dma,
			     unsigned int set);
/**
 * @brief osi_clear_rx_pkt_err_stats - Clear rx packet error stats.
 *
 * @note
 * Algorithm:
 *  - This function will be invoked by OSD layer to clear the
 *    rx_crc_error mentioned in osi_dma->pkt_err_stats structure.
 *
 * @param[in] osi_dma: OSI DMA private data structure.
 *
 *
 * @pre
 *  - MAC needs to be out of reset and proper clocks need to be configured.
 *  - DMA HW init need to be completed successfully, see osi_hw_dma_init
 *
 * @retval 0 on success
 * @retval -1 on failure.
 */
int osi_clear_rx_pkt_err_stats(struct osi_dma_priv_data *osi_dma);

/**
 * @brief osi_txring_empty - Check if Txring is empty.
 *
 * @note
 * Algorithm:
 *  - This function will be invoked by OSD layer to check if the Tx ring
 *    is empty or still has outstanding packets to be processed for Tx done.
 *
 * @param[in] osi_dma: OSI DMA private data structure.
 * @param[in] chan: Channel number whose ring is to be checked.
 *
 * @pre
 *  - MAC needs to be out of reset and proper clocks need to be configured.
 *  - DMA HW init need to be completed successfully, see osi_hw_dma_init
 *
 * @retval 1 if ring is empty.
 * @retval 0 if ring has outstanding packets.
 */
int osi_txring_empty(struct osi_dma_priv_data *osi_dma, unsigned int chan);
#endif /* !OSI_STRIPPED_LIB */
#endif /* OSI_DMA_H */
