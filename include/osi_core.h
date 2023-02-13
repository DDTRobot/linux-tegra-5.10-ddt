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

#ifndef OSI_CORE_H
#define OSI_CORE_H

#include "osi_common.h"
#include "mmc.h"

/**
 * @addtogroup PTP related information
 *
 * @brief PTP SSINC values
 * @{
 */
#define OSI_PTP_SSINC_16	16U
#define OSI_PTP_SSINC_4		4U
/** @} */

struct osi_core_priv_data;

/**
 * @brief OSI core structure for filters
 */
struct osi_filter {
	/** indicates operation needs to perform. refer to OSI_OPER_* */
	unsigned int oper_mode;
	/** Indicates the index of the filter to be modified.
	 * Filter index must be between 0 - 127 */
	unsigned int index;
	/** Ethernet MAC address to be added */
	const unsigned char *mac_address;
	/** Indicates dma channel routing enable(1) disable (0) */
	unsigned int dma_routing;
	/**  indicates dma channel number to program */
	unsigned int dma_chan;
	/** filter will not consider byte in comparison
	 *	Bit 5: MAC_Address${i}_High[15:8]
	 *	Bit 4: MAC_Address${i}_High[7:0]
	 *	Bit 3: MAC_Address${i}_Low[31:24]
	 *	..
	 *	Bit 0: MAC_Address${i}_Low[7:0] */
	unsigned int addr_mask;
	/** src_dest: SA(1) or DA(0) */
	unsigned int src_dest;
};

/**
 * @brief L3/L4 filter function dependent parameter
 */
struct osi_l3_l4_filter {
	/** Indicates the index of the filter to be modified.
	 * Filter index must be between 0 - 7 */
	unsigned int filter_no;
	/** filter enable(1) or disable(0) */
	unsigned int filter_enb_dis;
	/** source(0) or destination(1) */
	unsigned int src_dst_addr_match;
	/** perfect(0) or inverse(1) */
	unsigned int perfect_inverse_match;
	/** ipv4 address */
	unsigned char ip4_addr[4];
	/** ipv6 address */
	unsigned short ip6_addr[8];
	/** Port number */
	unsigned short port_no;
};

/**
 * @brief Vlan filter Function dependent parameter
 */
struct osi_vlan_filter {
	/** vlan filter enable(1) or disable(0) */
	unsigned int filter_enb_dis;
	/** perfect(0) or hash(1) */
	unsigned int perfect_hash;
	/** perfect(0) or inverse(1) */
	unsigned int perfect_inverse_match;
};

/**
 * @brief L2 filter function dependent parameter
 */
struct osi_l2_da_filter {
	/** perfect(0) or hash(1) */
	unsigned int perfect_hash;
	/** perfect(0) or inverse(1) */
	unsigned int perfect_inverse_match;
};

/**
 * @brief OSI Core avb data structure per queue.
 */
struct  osi_core_avb_algorithm {
	/** TX Queue/TC index */
	unsigned int qindex;
	/** CBS Algorithm enable(1) or disable(0) */
	unsigned int algo;
	/** When this bit is set, the accumulated credit parameter in the
	 * credit-based shaper algorithm logic is not reset to zero when
	 * there is positive credit and no packet to transmit in Channel.
	 * 
	 * Expected values are enable(1) or disable(0) */
	unsigned int credit_control;
	/** idleSlopeCredit value required for CBS */
	  unsigned int idle_slope;
	/** sendSlopeCredit value required for CBS */
	unsigned int send_slope;
	/** hiCredit value required for CBS */
	unsigned int hi_credit;
	/** lowCredit value required for CBS */
	unsigned int low_credit;
	/** Transmit queue operating mode
	 *
	 * 00: disable
	 * 
	 * 01: avb 
	 * 
	 * 10: enable */
	unsigned int oper_mode;
};

/**
 * @brief Initialize MAC & MTL core operations.
 */
struct osi_core_ops {
	/** Called to poll for software reset bit */
	int (*poll_for_swr)(void *ioaddr, unsigned int pre_si);
	/** Called to initialize MAC and MTL registers */
	int (*core_init)(struct osi_core_priv_data *const osi_core,
			 const unsigned int tx_fifo_size,
			 const unsigned int rx_fifo_size);
	/** Called to deinitialize MAC and MTL registers */
	void (*core_deinit)(struct osi_core_priv_data *const osi_core);
	/** Called periodically to read and validate safety critical
	 * registers against last written value */
	int (*validate_regs)(struct osi_core_priv_data *const osi_core);
	/**  Called to start MAC Tx and Rx engine */
	void (*start_mac)(void *addr);
	/** Called to stop MAC Tx and Rx engine */
	void (*stop_mac)(void *addr);
	/** Called to handle common interrupt */
	void (*handle_common_intr)(struct osi_core_priv_data *const osi_core);
	/** Called to set the mode at MAC (full/duplex) */
	void (*set_mode)(void *ioaddr, const int mode);
	/** Called to set the speed (10/100/1000) at MAC */
	void (*set_speed)(void *ioaddr, const int speed);
	/** Called to do pad caliberation */
	int (*pad_calibrate)(void *ioaddr);
	/** Called to set MDC clock rate for MDIO operation */
	void (*set_mdc_clk_rate)(struct osi_core_priv_data *const osi_core,
				 const unsigned long csr_clk_rate);
	/** Called to flush MTL Tx queue */
	int (*flush_mtl_tx_queue)(void *ioaddr, const unsigned int qinx);
	/** Called to configure MAC in loopback mode */
	int (*config_mac_loopback)(void *addr, const unsigned int lb_mode);
	/** Called to set av parameter */
	int (*set_avb_algorithm)(struct osi_core_priv_data *const osi_core,
			   const struct osi_core_avb_algorithm *const avb);
	/** Called to get av parameter */
	int (*get_avb_algorithm)(struct osi_core_priv_data *const osi_core,
				 struct osi_core_avb_algorithm *const avb);
	/** Called to configure MTL RxQ to forward the err pkt */
	int (*config_fw_err_pkts)(void *addr, const unsigned int qinx,
				 const unsigned int fw_err);
	/** Called to configure the MTL to forward/drop tx status */
	int (*config_tx_status)(void *addr, const unsigned int tx_status);
	/** Called to configure the MAC rx crc */
	int (*config_rx_crc_check)(void *addr, const unsigned int crc_chk);
	/** Called to configure the MAC flow control */
	int (*config_flow_control)(void *addr, const unsigned int flw_ctrl);
	/** Called to enable/disable HW ARP offload feature */
	int (*config_arp_offload)(const unsigned int mac_ver, void *addr,
				  const unsigned int enable,
				  const unsigned char *ip_addr);
	/** Called to configure Rx Checksum offload engine */
	int (*config_rxcsum_offload)(void *addr, const unsigned int enabled);
	/** Called to config mac packet filter */
	int (*config_mac_pkt_filter_reg)(
				struct osi_core_priv_data *const osi_core,
				const struct osi_filter *filter);
	/** Called to update MAC address 1-127 */
	int (*update_mac_addr_low_high_reg)(
				struct osi_core_priv_data *const osi_core,
				const struct osi_filter *filter);
	/** Called to configure l3/L4 filter */
	int (*config_l3_l4_filter_enable)(void *base,
					  const unsigned int enable);
	/** Called to configure L3 filter */
	int (*config_l3_filters)(struct osi_core_priv_data *const osi_core,
				 const unsigned int filter_no,
				 const unsigned int enb_dis,
				 const unsigned int ipv4_ipv6_match,
				 const unsigned int src_dst_addr_match,
				 const unsigned int perfect_inverse_match,
				 const unsigned int dma_routing_enable,
				 const unsigned int dma_chan);
	/** Called to update ip4 src or desc address */
	int (*update_ip4_addr)(struct osi_core_priv_data *const osi_core,
			       const unsigned int filter_no,
			       const unsigned char addr[],
			       const unsigned int src_dst_addr_match);
	/** Called to update ip6 address */
	int (*update_ip6_addr)(struct osi_core_priv_data *const osi_core,
			       const unsigned int filter_no,
			       const unsigned short addr[]);
	/** Called to configure L4 filter */
	int (*config_l4_filters)(struct osi_core_priv_data *const osi_core,
				 const unsigned int filter_no,
				 const unsigned int enb_dis,
				 const unsigned int tcp_udp_match,
				 const unsigned int src_dst_port_match,
				 const unsigned int perfect_inverse_match,
				 const unsigned int dma_routing_enable,
				 const unsigned int dma_chan);
	/** Called to update L4 Port for filter packet */
	int (*update_l4_port_no)(struct osi_core_priv_data *const osi_core,
				 const unsigned int filter_no,
				 const unsigned short port_no,
				 const unsigned int src_dst_port_match);

	/** Called to configure VLAN filtering */
	int (*config_vlan_filtering)(struct osi_core_priv_data *const osi_core,
				     const unsigned int filter_enb_dis,
				     const unsigned int perfect_hash_filtering,
				     const unsigned int perfect_inverse_match);
	/** called to update VLAN id */
	int (*update_vlan_id)(void *base, const unsigned int vid);
	/** Called to set current system time to MAC */
	int (*set_systime_to_mac)(void *addr, const unsigned int sec,
				  const unsigned int nsec);
	/** Called to set the addend value to adjust the time */
	int (*config_addend)(void *addr, const unsigned int addend);
	/** Called to adjust the mac time */
	int (*adjust_mactime)(void *addr, const unsigned int sec,
			      const unsigned int nsec,
			      const unsigned int neg_adj,
			      const unsigned int one_nsec_accuracy);
	/** Called to get the current time from MAC */
	unsigned long long (*get_systime_from_mac)(void *addr);
	/** Called to configure the TimeStampControl register */
	void (*config_tscr)(void *addr, const unsigned int ptp_filter);
	/** Called to configure the sub second increment register */
	void (*config_ssir)(struct osi_core_priv_data *const osi_core);
	/** Called to update MMC counter from HW register */
	void (*read_mmc)(struct osi_core_priv_data *const osi_core);
	/** Called to reset MMC HW counter structure */
	void (*reset_mmc)(struct osi_core_priv_data *const osi_core);
	/** Called to configure EEE Tx LPI */
	void (*configure_eee)(struct osi_core_priv_data *const osi_core,
			      const unsigned int tx_lpi_enabled,
			      const unsigned int tx_lpi_timer);
	/** Called to save MAC register space during SoC suspend */
	int (*save_registers)(struct osi_core_priv_data *const osi_core);
	/** Called to restore MAC control registers during SoC resume */
	int (*restore_registers)(struct osi_core_priv_data *const osi_core);
	/** Called to write into a PHY reg over MDIO bus */
	int (*write_phy_reg)(struct osi_core_priv_data *const osi_core,
			     const unsigned int phyaddr,
			     const unsigned int phyreg,
			     const unsigned short phydata);
	/** Called to read from a PHY reg over MDIO bus */
	int (*read_phy_reg)(struct osi_core_priv_data *const osi_core,
			    const unsigned int phyaddr,
			    const unsigned int phyreg);
};

/**
 * @brief PTP configuration structure
 */
struct osi_ptp_config {
	/** PTP filter parameters bit fields.
	 * 
	 * Enable Time stamp,Fine Timestamp,1 nanosecond accuracy
	 * are enabled by default.
	 * 
	 * Need to set below bit fields accordingly as per the requirements.
	 * 
	 * Enable Timestamp for All Packets			OSI_BIT(8)
	 * 
	 * Enable PTP Packet Processing for Version 2 Format	OSI_BIT(10)
	 * 
	 * Enable Processing of PTP over Ethernet Packets	OSI_BIT(11)
	 * 
	 * Enable Processing of PTP Packets Sent over IPv6-UDP	OSI_BIT(12)
	 * 
	 * Enable Processing of PTP Packets Sent over IPv4-UDP	OSI_BIT(13)
	 * 
	 * Enable Timestamp Snapshot for Event Messages		OSI_BIT(14)
	 * 
	 * Enable Snapshot for Messages Relevant to Master	OSI_BIT(15)
	 * 
	 * Select PTP packets for Taking Snapshots		OSI_BIT(16)
	 * 
	 * Select PTP packets for Taking Snapshots		OSI_BIT(17)
	 * 
	 * Select PTP packets for Taking Snapshots (OSI_BIT(16) + OSI_BIT(17))
	 * 
	 * AV 802.1AS Mode Enable				OSI_BIT(28)
	 * 
	 * if ptp_fitler is set to Zero then Time stamping is disabled */
	unsigned int ptp_filter;
	/** seconds to be updated to MAC */
	unsigned int sec;
	/** nano seconds to be updated to MAC */
	unsigned int nsec;
	/** PTP reference clock read from DT */
	unsigned int ptp_ref_clk_rate;
	/** Use one nsec accuracy (need to set 1) */
	unsigned int one_nsec_accuracy;
	/** PTP system clock which is 62500000Hz */
	unsigned int ptp_clock;
};

/**
 * @brief Max num of MAC core registers to backup. It should be max of or >=
 * (EQOS_MAX_BAK_IDX=380, coreX,...etc) backup registers.
 */
#define CORE_MAX_BAK_IDX	700U

/**
 * @brief core_backup - Struct used to store backup of core HW registers.
 */
struct core_backup {
	/** Array of reg MMIO addresses (base of MAC + offset of reg) */
	void *reg_addr[CORE_MAX_BAK_IDX];
	/** Array of value stored in each corresponding register */
	unsigned int reg_val[CORE_MAX_BAK_IDX];
};

/**
 * @brief The OSI Core (MAC & MTL) private data structure.
 */
struct osi_core_priv_data {
	/** Memory mapped base address of MAC IP */
	void *base;
	/** Memory mapped base address of DMA window of MAC IP */
	void *dma_base;
	/** Pointer to OSD private data structure */
	void *osd;
	/** Address of HW Core operations structure */
	struct osi_core_ops *ops;
	/** Number of MTL queues enabled in MAC */
	unsigned int num_mtl_queues;
	/** Array of MTL queues */
	unsigned int mtl_queues[OSI_EQOS_MAX_NUM_CHANS];
	/** List of MTL Rx queue mode that need to be enabled */
	unsigned int rxq_ctrl[OSI_EQOS_MAX_NUM_CHANS];
	/** Rx MTl Queue mapping based on User Priority field */
	unsigned int rxq_prio[OSI_EQOS_MAX_NUM_CHANS];
	/** MAC HW type EQOS based on DT compatible */
	unsigned int mac;
	/** MAC version */
	unsigned int mac_ver;
	/** MDC clock rate */
	unsigned int mdc_cr;
	/** MTU size */
	unsigned int mtu;
	/** Ethernet MAC address */
	unsigned char mac_addr[OSI_ETH_ALEN];
	/** DT entry to enable(0) or disable(1) pause frame support */
	unsigned int pause_frames;
	/** Current flow control settings */
	unsigned int flow_ctrl;
	/** PTP configuration settings */
	struct osi_ptp_config ptp_config;
	/** Default addend value */
	unsigned int default_addend;
	/** mmc counter structure */
	struct osi_mmc_counters mmc;
	/** xtra sw error counters */
	struct osi_xtra_stat_counters xstats;
	/** DMA channel selection enable (1) */
	unsigned int dcs_en;
	/** Functional safety config to do periodic read-verify of
	 * certain safety critical registers */
	void *safety_config;
	/** Backup config to save/restore registers during suspend/resume */
	struct core_backup backup_config;
	/** VLAN tag stripping enable(1) or disable(0) */
	unsigned int strip_vlan_tag;
	/** L3L4 filter bit bask, set index corresponding bit for
	 * filter if filter enabled */
	unsigned int l3l4_filter_bitmask;
	/** csr clock is to program LPI 1 us tick timer register.
	 * Value stored in MHz
	 */
	unsigned int csr_clk_speed;
	/** Tegra Pre-si platform info */
	unsigned int pre_si;
};

/**
 * @brief osi_poll_for_mac_reset_complete - Poll Software reset bit in MAC HW
 *
 * Algorithm: Invokes EQOS routine to check for SWR (software reset)
 * bit in DMA Basic mode register to make sure IP reset was successful.
 *
 * @param[in] osi_core: OSI Core private data structure.
 *
 * @note MAC needs to be out of reset and proper clock configured.
 *
 * @retval 0 on success
 * @retval -1 on failure.
 */

int osi_poll_for_mac_reset_complete(
			struct osi_core_priv_data *const osi_core);

/**
 * @brief osi_set_mdc_clk_rate - Derive MDC clock based on provided AXI_CBB clk.
 *
 * Algorithm: MDC clock rate will be populated in OSI core private data
 * structure based on AXI_CBB clock rate.
 *
 * @param[in] osi_core: OSI core private data structure.
 * @param[in] csr_clk_rate: CSR (AXI CBB) clock rate.
 *
 * @note OSD layer needs get the AXI CBB clock rate with OSD clock API
 *	(ex - clk_get_rate())
 *
 * @retval 0 on success
 * @retval -1 on failure.
 */
int osi_set_mdc_clk_rate(struct osi_core_priv_data *const osi_core,
			 const unsigned long csr_clk_rate);

/**
 * @brief osi_hw_core_init - EQOS MAC, MTL and common DMA initialization.
 * 
 * Algorithm: Invokes EQOS MAC, MTL and common DMA register init code.
 *
 * @param[in] osi_core: OSI core private data structure.
 * @param[in] tx_fifo_size: OSI core private data structure.
 * @param[in] rx_fifo_size: OSI core private data structure.
 *
 * @note
 * 1) MAC should be out of reset. See osi_poll_for_mac_reset_complete()
 *    for details.
 * 2) osi_core->base needs to be filled based on ioremap.
 * 3) osi_core->num_mtl_queues needs to be filled.
 * 4)osi_core->mtl_queues[qinx] need to be filled.
 *
 * @retval 0 on success
 * @retval -1 on failure.
 */
int osi_hw_core_init(struct osi_core_priv_data *const osi_core,
		     unsigned int tx_fifo_size,
		     unsigned int rx_fifo_size);

/**
 * @brief osi_hw_core_deinit - EQOS MAC deinitialization.
 * 
 * Algorithm: Stops MAC transmisson and reception.
 *
 * @param[in] osi_core: OSI core private data structure.
 *
 * @note MAC has to be out of reset.
 *
 * @retval 0 on success
 * @retval -1 on failure.
 */
int osi_hw_core_deinit(struct osi_core_priv_data *const osi_core);

/**
 * @brief osi_validate_core_regs - Read-validate HW registers for func safety.
 *
 * Algorithm: Reads pre-configured list of MAC/MTL configuration registers
 *	and compares with last written value for any modifications.
 *
 * @param[in] osi_core: OSI core private data structure.
 *
 * @note
 *	1) MAC has to be out of reset.
 *	2) osi_hw_core_init has to be called. Internally this would initialize
 *	the safety_config (see osi_core_priv_data) based on MAC version and
 *	which specific registers needs to be validated periodically.
 *	3) Invoke this call iff (osi_core_priv_data->safety_config != OSI_NULL)
 *
 * @retval 0 on success
 * @retval -1 on failure.
 */
int osi_validate_core_regs(struct osi_core_priv_data *const osi_core);

/**
 * @brief osi_start_mac - Start MAC Tx/Rx engine
 * 
 * Algorithm: Enable MAC Tx and Rx engine.
 *
 * @param[in] osi_core: OSI core private data structure.
 *
 * @note MAC init should be complete. See osi_hw_core_init() and
 * 	 osi_hw_dma_init()
 *
 * @retval 0 on success
 * @retval -1 on failure.
 */
int osi_start_mac(struct osi_core_priv_data *const osi_core);

/**
 * @brief osi_stop_mac - Stop MAC Tx/Rx engine
 * 
 * Algorithm: Stop MAC Tx and Rx engine
 *
 * @param[in] osi_core: OSI core private data structure.
 *
 * @note MAC DMA deinit should be complete. See osi_hw_dma_deinit()
 *
 * @retval 0 on success
 * @retval -1 on failure.
 */
int osi_stop_mac(struct osi_core_priv_data *const osi_core);

/**
 * @brief osi_common_isr - Common ISR.
 * 
 * Algorithm: Takes care of handling the common interrupts accordingly as per
 * the MAC IP
 *
 * @param[in] osi_core: OSI core private data structure.
 *
 * @note MAC should be init and started. see osi_start_mac()
 *
 * @retval 0 on success
 * @retval -1 on failure.
 */
int osi_common_isr(struct osi_core_priv_data *const osi_core);

/**
 * @brief osi_set_mode - Set FD/HD mode.
 *
 * Algorithm: Takes care of  setting HD or FD mode accordingly as per the MAC IP
 *
 * @param[in] osi_core: OSI core private data structure.
 * @param[in] mode: Operating mode.
 *
 * @note MAC should be init and started. see osi_start_mac()
 *
 * @retval 0 on success
 * @retval -1 on failure.
 */
int osi_set_mode(struct osi_core_priv_data *const osi_core,
		 const int mode);

/**
 * @brief osi_set_speed - Set operating speed.
 * 
 * Algorithm: Takes care of  setting the operating speed accordingly as per
 * the MAC IP.
 *
 * @param[in] osi_core: OSI core private data structure.
 * @param[in] speed: Operating speed.
 *
 * @note MAC should be init and started. see osi_start_mac()
 *
 * @retval 0 on success
 * @retval -1 on failure.
 */
int osi_set_speed(struct osi_core_priv_data *const osi_core,
		  const int speed);

/**
 * @brief osi_pad_calibrate - PAD calibration
 *
 * Algorithm: Takes care of  doing the pad calibration
 * accordingly as per the MAC IP.
 *
 * @param[in] osi_core: OSI core private data structure.
 *
 * @note
 *	1) MAC should out of reset and clocks enabled.
 *	2) RGMII and MDIO interface needs to be IDLE before performing PAD
 *	calibration.
 *
 * @retval 0 on success
 * @retval -1 on failure.
 */
int osi_pad_calibrate(struct osi_core_priv_data *const osi_core);

/**
 * @brief osi_flush_mtl_tx_queue - Flushing a MTL Tx Queue.
 *
 * Algorithm: Invokes EQOS flush Tx queue routine.
 *
 * @param[in] osi_core: OSI core private data structure.
 * @param[in] qinx: MTL queue index.
 *
 * @note
 *	1) MAC should out of reset and clocks enabled.
 *	2) hw core initialized. see osi_hw_core_init().
 *
 * @retval 0 on success
 * @retval -1 on failure.
 */
int osi_flush_mtl_tx_queue(struct osi_core_priv_data *const osi_core,
			   const unsigned int qinx);

/**
 * @brief osi_config_mac_loopback - Configure MAC loopback
 *
 * Algorithm: Configure the MAC to support the loopback.
 *
 * @param[in] osi_core: OSI core private data structure.
 * @param[in] lb_mode: Enable or disable MAC loopback
 *
 * @note MAC should be init and started. see osi_start_mac()
 *
 * @retval 0 on success
 * @retval -1 on failure.
 */
int osi_config_mac_loopback(struct osi_core_priv_data *const osi_core,
			    const unsigned int lb_mode);

/**
 * @brief osi_set_avb - Set CBS algo and parameters
 *
 * Algorithm: Set AVB algo and  populated parameter from osi_core_avb
 * structure for TC/TQ
 *
 * @param[in] osi_core: OSI core private data structure.
 * @param[in] avb: osi core avb data structure.
 *
 * @note
 *	1) MAC should be init and started. see osi_start_mac()
 *	2) osi_core->osd should be populated.
 *
 * @retval 0 on success
 * @retval -1 on failure.
 */
int osi_set_avb(struct osi_core_priv_data *const osi_core,
		const struct osi_core_avb_algorithm *avb);

/**
 * @brief osi_get_avb - Get CBS algo and parameters
 *
 * Algorithm: get AVB algo and  populated parameter from osi_core_avb
 * structure for TC/TQ
 *
 * @param[in] osi_core: OSI core private data structure.
 * @param[out] avb: osi core avb data structure.
 *
 * @note
 *	1) MAC should be init and started. see osi_start_mac()
 *	2) osi_core->osd should be populated.
 *
 * @retval 0 on success
 * @retval -1 on failure.
 */
int osi_get_avb(struct osi_core_priv_data *const osi_core,
		struct osi_core_avb_algorithm *avb);

/**
 * @brief osi_configure_txstatus - Configure Tx packet status reporting
 *
 * Algorithm: Configure MAC to enable/disable Tx status error
 * reporting.
 *
 * @param[in] osi_core: OSI core private data structure.
 * @param[in] tx_status: Enable or disable tx packet status reporting
 *
 * @note MAC should be init and started. see osi_start_mac()
 *
 * @retval 0 on success
 * @retval -1 on failure.
 */
int osi_configure_txstatus(struct osi_core_priv_data *const osi_core,
			   const unsigned int tx_status);

/**
 * @brief osi_config_fw_err_pkts - Configure forwarding of error packets
 *
 * Algorithm: Configure MAC to enable/disable forwarding of error packets.
 *
 * @param[in] osi_core: OSI core private data structure.
 * @param[in] qinx: Q index
 * @param[in] fw_err: Enable or disable forwarding of error packets
 *
 * @note MAC should be init and started. see osi_start_mac()
 *
 * @retval 0 on success
 * @retval -1 on failure.
 */
int osi_config_fw_err_pkts(struct osi_core_priv_data *const osi_core,
			   const unsigned int qinx, const unsigned int fw_err);

/**
 * @brief osi_config_rx_crc_check - Configure CRC Checking for Received Packets
 *
 * Algorithm: When this bit is set, the MAC receiver does not check the CRC
 * field in the received packets. When this bit is reset, the MAC receiver
 * always checks the CRC field in the received packets.
 *
 * @param[in] osi_core: OSI core private data structure.
 * @param[in] crc_chk: Enable or disable checking of CRC field in received pkts
 *
 * @note MAC should be init and started. see osi_start_mac()
 *
 * @retval 0 on success
 * @retval -1 on failure.
 */
int osi_config_rx_crc_check(struct osi_core_priv_data *const osi_core,
			    const unsigned int crc_chk);

/**
 * @brief osi_configure_flow_ctrl - Configure flow control settings
 *
 * Algorithm: This will enable or disable the flow control.
 * flw_ctrl BIT0 is for tx flow ctrl enable/disable
 * flw_ctrl BIT1 is for rx flow ctrl enable/disable
 *
 * @param[in] osi_core: OSI core private data structure.
 * @param[in] flw_ctrl: Enable or disable flow control settings
 *
 * @note MAC should be init and started. see osi_start_mac()
 *
 * @retval 0 on success
 * @retval -1 on failure.
 */
int osi_configure_flow_control(
			       struct osi_core_priv_data *const osi_core,
			       const unsigned int flw_ctrl);

/**
 * @brief osi_config_arp_offload - Configure ARP offload in MAC.
 *
 * Algorithm: Invokes EQOS config ARP offload routine.
 *
 * @param[in] osi_core: OSI core private data structure.
 * @param[in] flags: Enable/disable flag.
 * @param[in] ip_addr: Char array representation of IP address
 *
 * @note
 *	1) MAC should be init and started. see osi_start_mac()
 *	2) Valid 4 byte IP address as argument ip_addr
 *
 * @retval 0 on success
 * @retval -1 on failure.
 */
int osi_config_arp_offload(struct osi_core_priv_data *const osi_core,
			   const unsigned int flags,
			   const unsigned char *ip_addr);

/**
 * @brief osi_config_rxcsum_offload - Configure RX checksum offload in MAC.
 *
 * Algorithm: Invokes EQOS config RX checksum offload routine.
 *
 * @param[in] osi_core: OSI core private data structure.
 * @param[in] enable: Enable/disable flag.
 *
 * @note MAC should be init and started. see osi_start_mac()
 *
 * @retval 0 on success
 * @retval -1 on failure.
 */
int osi_config_rxcsum_offload(
			      struct osi_core_priv_data *const osi_core,
			      const unsigned int enable);

/**
 * @brief osi_l2_filter - configure L2 mac filter.
 *
 * Algorithm: This sequence is used to configure MAC in different packet
 * processing modes like promiscuous, multicast, unicast,
 * hash unicast/multicast and perfect/inverse matching for L2 DA
 *
 * @param[in] osi_core: OSI core private data structure.
 * @param[in] filter: OSI filter structure.
 *
 * @note
 *	1) MAC should be initialized and started. see osi_start_mac()
 *
 * @retval 0 on success
 * @retval -1 on failure.
 */
int osi_l2_filter(struct osi_core_priv_data *const osi_core,
		  const struct osi_filter *filter);

/**
 * @brief osi_config_vlan_filtering - OSI call for configuring VLAN filter
 *
 * Algorithm: This sequence is used to enable/disable VLAN filtering and
 * also selects VLAN filtering mode- perfect/hash
 *
 * @param[in] osi_core: OSI core private data structure.
 * @param[in] filter_enb_dis: vlan filter enable(1) disable(0)
 * @param[in] perfect_hash_filtering: perfect(0) or hash filter(1)
 * @param[in] perfect_inverse_match: normal(0) or inverse filter(1)
 *
 * @note
 *	1) MAC should be init and started. see osi_start_mac()
 *	2) osi_core->osd should be populated
 *
 * @retval 0 on success
 * @retval -1 on failure.
 */
int osi_config_vlan_filtering(
			      struct osi_core_priv_data *const osi_core,
			      const unsigned int filter_enb_dis,
			      const unsigned int perfect_hash_filtering,
			      const unsigned int perfect_inverse_match);

/**
 * @brief osi_update_vlan_id - invoke osi call to update VLAN ID
 *
 * Algorithm: return 16 bit VLAN ID
 *
 * @param[in] osi_core: OSI core private data structure.
 * @param[in] vid: VLAN ID
 *
 * @note MAC should be init and started. see osi_start_mac()
 *
 * @retval 0 on success
 * @retval -1 on failure.
 */
int  osi_update_vlan_id(struct osi_core_priv_data *const osi_core,
			const unsigned int vid);

/**
 * @brief osi_write_phy_reg - Write to a PHY register through MAC over MDIO bus.
 *
 * Algorithm:
 * 1) Before proceeding for reading for PHY register check whether any MII
 *    operation going on MDIO bus by polling MAC_GMII_BUSY bit.
 * 2) Program data into MAC MDIO data register.
 * 3) Populate required parameters like phy address, phy register etc,,
 *	in MAC MDIO Address register. write and GMII busy bits needs to be set
 *	in this operation.
 * 4) Write into MAC MDIO address register poll for GMII busy for MDIO
 *	operation to complete.
 *
 * @param[in] osi_core: OSI core private data structure.
 * @param[in] phyaddr: PHY address (PHY ID) associated with PHY
 * @param[in] phyreg: Register which needs to be write to PHY.
 * @param[in] phydata: Data to write to a PHY register.
 *
 * @note MAC should be init and started. see osi_start_mac()
 *
 * @retval 0 on success
 * @retval -1 on failure.
 */
int osi_write_phy_reg(struct osi_core_priv_data *const osi_core,
		      const unsigned int phyaddr, const unsigned int phyreg,
		      const unsigned short phydata);

/**
 * @brief osi_read_mmc - invoke function to read actual registers and update
 *	  structure variable mmc
 * 
 * Algorithm: Read the registers, mask reserve bits if required, update
 *	  structure.
 *
 * @param[in] osi_core: OSI core private data structure.
 *
 * @note
 *	1) MAC should be init and started. see osi_start_mac()
 *	2) osi_core->osd should be populated
 *
 * @retval 0 on success
 * @retval -1 on failure.
 */
int osi_read_mmc(struct osi_core_priv_data *const osi_core);

/**
 * @brief osi_reset_mmc - invoke function to reset MMC counter and data
 *	  structure
 *
 * Algorithm: Read the registers, mask reserve bits if required, update
 *	  structure.
 *
 * @param[in] osi_core: OSI core private data structure.
 *
 * @note
 *	1) MAC should be init and started. see osi_start_mac()
 *	2) osi_core->osd should be populated
 *
 * @retval 0 on success
 * @retval -1 on failure.
 */
int osi_reset_mmc(struct osi_core_priv_data *const osi_core);

/**
 * @brief osi_read_phy_reg - Read from a PHY register through MAC over MDIO bus.
 *
 * Algorithm:
 *	1) Before proceeding for reading for PHY register check whether any MII
 *	operation going on MDIO bus by polling MAC_GMII_BUSY bit.
 *	2) Populate required parameters like phy address, phy register etc,,
 *	in program it in MAC MDIO Address register. Read and GMII busy bits
 *	needs to be set in this operation.
 *	3) Write into MAC MDIO address register poll for GMII busy for MDIO
 *	operation to complete. After this data will be available at MAC MDIO
 *	data register.
 *
 * @param[in] osi_core: OSI core private data structure.
 * @param[in] phyaddr: PHY address (PHY ID) associated with PHY
 * @param[in] phyreg: Register which needs to be read from PHY.
 *
 * @note MAC should be init and started. see osi_start_mac()
 *
 * @retval data from PHY register on success
 * @retval -1 on failure
 */
int osi_read_phy_reg(struct osi_core_priv_data *const osi_core,
		     const unsigned int phyaddr,
		     const unsigned int phyreg);

/**
 * @brief initializing the core operations
 *
 * @param[in] osi_core: OSI core private data structure.
 *
 * @retval data from PHY register on success
 * @retval -1 on failure
 */
int osi_init_core_ops(struct osi_core_priv_data *const osi_core);

/**
 * @brief osi_set_systime_to_mac - Handles setting of system time.
 *
 * Algorithm: Set current system time to MAC.
 *
 * @param[in] osi_core: OSI core private data structure.
 * @param[in] sec: Seconds to be configured.
 * @param[in] nsec: Nano seconds to be configured.
 *
 * @note MAC should be init and started. see osi_start_mac()
 *
 * @retval 0 on success
 * @retval -1 on failure.
 */
int osi_set_systime_to_mac(struct osi_core_priv_data *const osi_core,
			   const unsigned int sec, const unsigned int nsec);

/**
 * @brief osi_adjust_freq - Adjust frequency
 *
 * Algorithm: Adjust a drift of +/- comp nanoseconds per second.
 *	  "Compensation" is the difference in frequency between
 *	  the master and slave clocks in Parts Per Billion.
 *
 * @param[in] osi_core: OSI core private data structure.
 * @param[in] ppb: Parts per Billion
 *
 * @note MAC should be init and started. see osi_start_mac()
 *
 * @retval 0 on success
 * @retval -1 on failure.
 */
int osi_adjust_freq(struct osi_core_priv_data *const osi_core, int ppb);

/**
 * @brief osi_adjust_time - Adjust MAC time with system time
 *
 * Algorithm: Adjust/update the MAC time (delta time from MAC to system time
 * passed in nanoseconds, can be + or -).
 *
 * @param[in] osi_core: OSI core private data structure.
 * @param[in] nsec_delta: Delta time in nano seconds
 *
 * @note
 *	1) MAC should be init and started. see osi_start_mac()
 *	2) osi_core->ptp_config.one_nsec_accuracy need to be set to 1
 *
 * @retval 0 on success
 * @retval -1 on failure.
 */
int osi_adjust_time(struct osi_core_priv_data *const osi_core,
		    long long nsec_delta);

/**
 * @brief osi_get_systime_from_mac - Get system time
 *
 * Algorithm: Gets the current system time
 *
 * @param[in] osi_core: OSI core private data structure.
 * @param[out] sec: Value read in Seconds
 * @param[out] nsec: Value read in Nano seconds
 *
 * @note MAC should be init and started. see osi_start_mac()
 *
 * @retval 0 on success
 * @retval -1 on failure.
 */
int osi_get_systime_from_mac(
			     struct osi_core_priv_data *const osi_core,
			     unsigned int *sec,
			     unsigned int *nsec);
/**
 * @brief osi_ptp_configuration - Configure PTP
 *
 * Algorithm: Configure the PTP registers that are required for PTP.
 *
 * @param[in] osi_core: OSI core private data structure.
 * @param[in] enable: Enable or disable Time Stamping. 0: Disable 1: Enable
 *
 * @note
 *	1) MAC should be init and started. see osi_start_mac()
 *	2) osi->ptp_config.ptp_filter need to be filled accordingly to the
 *	filter that need to be set for PTP packets. Please check osi_ptp_config
 *	structure declaration on the bit fields that need to be filled.
 *	3) osi->ptp_config.ptp_clock need to be filled with the ptp system clk.
 *	Currently it is set to 62500000Hz.
 *	4) osi->ptp_config.ptp_ref_clk_rate need to be filled with the ptp
 *	reference clock that platform supports.
 *	5) osi->ptp_config.sec need to be filled with current time of seconds
 *	6) osi->ptp_config.nsec need to be filled with current time of nseconds
 *	7) osi->base need to be filled with the ioremapped base address
 *
 * @retval 0 on success
 * @retval -1 on failure.
 */
int osi_ptp_configuration(struct osi_core_priv_data *const osi_core,
			  const unsigned int enable);

/* MAC version specific implementation function prototypes added here
 * for misra compliance to have
 * 1. Visible prototype for all functions.
 * 2. Only one prototype for all function.
 */
struct osi_core_ops *eqos_get_hw_core_ops(void);
void *eqos_get_core_safety_config(void);

/**
 * @brief osi_l3l4_filter -  invoke OSI call to add L3/L4
 * filters.
 *
 * Algorithm: This routine is to enable/disable L3/l4 filter.
 * Check for DCS enable as well as validate channel
 * number if dcs_enable is set. After validation, configure L3(IPv4/IPv6)
 * filters register for given address. Based on input arguments update
 * IPv4/IPv6 source/destination address for L3 layer filtering or source and
 * destination Port Number for L4(TCP/UDP) layer
 * filtering.
 *
 * @param[in] osi_core: OSI core private data structure.
 * @param[in] l_filter: L3L4 filter data structure.
 * @param[in] type: L3 filter (ipv4(0) or ipv6(1))
 *	      or L4 filter (tcp(0) or udp(1))
 * @param[in] dma_routing_enable: filter based dma routing enable(1)
 * @param[in] dma_chan: dma channel for routing based on filter
 * @param[in] is_l4_filter: API call for L3 filter(0) or L4 filter(1)
 *
 * @note
 *	1) MAC should be init and started. see osi_start_mac()
 *	2) Concurrent invocations to configure filters is not supported.
 *	   OSD driver shall serialize calls.
 *
 * @retval 0 on success
 * @retval -1 on failure.
 */
int osi_l3l4_filter(struct osi_core_priv_data *const osi_core,
		    const struct osi_l3_l4_filter l_filter,
		    const unsigned int type,
		    const unsigned int dma_routing_enable,
		    const unsigned int dma_chan,
		    const unsigned int is_l4_filter);

/**
 * @brief osi_configure_eee - Configure EEE LPI in MAC.
 *
 * Algorithm: This routine invokes configuration of EEE LPI in the MAC.
 *
 * @param[in] osi_core: OSI core private data structure.
 * @param[in] tx_lpi_enabled: Enable (1)/disable (0) tx lpi
 * @param[in] tx_lpi_timer: Tx LPI entry timer in usecs upto
 *	      OSI_MAX_TX_LPI_TIMER (in steps of 8usec)
 *
 * @note
 *	1) MAC and PHY should be init and started. see osi_start_mac()
 *
 * @retval 0 on success
 * @retval -1 on failure.
 */
int osi_configure_eee(struct osi_core_priv_data *const osi_core,
		      unsigned int tx_lpi_enabled,
		      unsigned int tx_lpi_timer);

/**
 * @brief osi_save_registers - Take backup of MAC MMIO address space
 *
 * @param[in] osi_core: OSI core private data structure.
 *
 * @note
 *	1) MAC and PHY should be init and started. see osi_start_mac()
 *	2) No further configuration change in MAC shall be done after invoking
 *	this API
 *
 * @retval 0 on success
 * @retval -1 on failure.
 */
int osi_save_registers(struct osi_core_priv_data *const osi_core);

/**
 * @brief osi_restore_registers - Restore backup of MAC MMIO address space
 *
 * @param[in] osi_core: OSI core private data structure.
 *
 * @note
 *	1) MAC and PHY should be init and started. see osi_start_mac()
 *
 * @retval 0 on success
 * @retval -1 on failure.
 */
int osi_restore_registers(struct osi_core_priv_data *const osi_core);
#endif /* OSI_CORE_H */
