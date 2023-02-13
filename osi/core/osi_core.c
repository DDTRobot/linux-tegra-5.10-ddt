/*
 * Copyright (c) 2018-2021, NVIDIA CORPORATION. All rights reserved.
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

#include <local_common.h>
#include <ivc_core.h>
#include "core_local.h"
#include "../osi/common/common.h"
#include "vlan_filter.h"
#include "frp.h"

/**
 * @brief g_core - Static core local data array.
 */
static struct core_local g_core[MAX_CORE_INSTANCES];

/**
 * @brief g_ops - Static core operations array.
 */
static struct core_ops g_ops[MAX_MAC_IP_TYPES];

struct osi_core_priv_data *osi_get_core(void)
{
	nveu32_t i;

	for (i = 0U; i < MAX_CORE_INSTANCES; i++) {
		if (g_core[i].init_done == OSI_ENABLE) {
			continue;
		}

		break;
	}

	if (i == MAX_CORE_INSTANCES) {
		return OSI_NULL;
	}

	g_core[i].magic_num = (nveu64_t)&g_core[i].osi_core;

	return &g_core[i].osi_core;
}

/**
 * @brief Function to validate input arguments of API.
 *
 * @param[in] osi_core: OSI Core private data structure.
 * @param[in] l_core: OSI local core data structure.
 *
 * @note
 * API Group:
 * - Initialization: Yes
 * - Run time: Yes
 * - De-initialization: Yes
 *
 * @retval 0 on Success
 * @retval -1 on Failure
 */
static inline nve32_t validate_args(struct osi_core_priv_data *const osi_core,
				    struct core_local *l_core)
{
	if ((osi_core == OSI_NULL) ||
	    (osi_core->base == OSI_NULL) ||
	    (l_core->init_done == OSI_DISABLE) ||
	    (l_core->magic_num != (nveu64_t)osi_core)) {
		return -1;
	}

	return 0;
}

/**
 * @brief Function to validate function pointers.
 *
 * @param[in] osi_core: OSI Core private data structure.
 * @param[in] ops_p: OSI Core operations structure.
 *
 * @note
 * API Group:
 * - Initialization: Yes
 * - Run time: No
 * - De-initialization: No
 *
 * @retval 0 on Success
 * @retval -1 on Failure
 */
static nve32_t validate_func_ptrs(struct osi_core_priv_data *const osi_core,
				  struct core_ops *ops_p)
{
	nveu32_t i = 0;
	void *temp_ops = (void *)ops_p;
#if __SIZEOF_POINTER__ == 8
	nveu64_t *l_ops = (nveu64_t *)temp_ops;
#elif __SIZEOF_POINTER__ == 4
	nveu32_t *l_ops = (nveu32_t *)temp_ops;
#else
	OSI_CORE_ERR(OSI_NULL, OSI_LOG_ARG_INVALID,
		     "Undefined architecture\n", 0ULL);
	return -1;
#endif

	for (i = 0; i < (sizeof(*ops_p) / (nveu64_t)__SIZEOF_POINTER__); i++) {
		if (*l_ops == 0U) {
			return -1;
		}

		l_ops++;
	}

	return 0;
}

nve32_t osi_write_phy_reg(struct osi_core_priv_data *const osi_core,
			  const nveu32_t phyaddr, const nveu32_t phyreg,
			  const nveu16_t phydata)
{
	struct core_local *l_core = (struct core_local *)osi_core;

	if (validate_args(osi_core, l_core) < 0) {
		return -1;
	}

	return l_core->ops_p->write_phy_reg(osi_core, phyaddr, phyreg, phydata);
}

nve32_t osi_read_phy_reg(struct osi_core_priv_data *const osi_core,
			 const nveu32_t phyaddr, const nveu32_t phyreg)
{
	struct core_local *l_core = (struct core_local *)osi_core;

	if (validate_args(osi_core, l_core) < 0) {
		return -1;
	}

	return l_core->ops_p->read_phy_reg(osi_core, phyaddr, phyreg);
}

nve32_t osi_init_core_ops(struct osi_core_priv_data *const osi_core)
{
	struct core_local *l_core = (struct core_local *)osi_core;
	typedef void (*init_ops_arr)(struct core_ops *);
	typedef void *(*safety_init)(void);

	init_ops_arr i_ops[MAX_MAC_IP_TYPES][MAX_MAC_IP_TYPES] = {
		{ eqos_init_core_ops, ivc_init_core_ops },
		{ mgbe_init_core_ops, OSI_NULL }
	};

	safety_init s_init[MAX_MAC_IP_TYPES][MAX_MAC_IP_TYPES] = {
		{ eqos_get_core_safety_config, ivc_get_core_safety_config },
		{ OSI_NULL, OSI_NULL }
	};

	if (osi_core == OSI_NULL) {
		return -1;
	}

	if ((l_core->magic_num != (nveu64_t)osi_core) ||
	    (l_core->init_done == OSI_ENABLE)) {
		return -1;
	}

	if ((osi_core->osd_ops.ops_log == OSI_NULL) ||
	    (osi_core->osd_ops.udelay == OSI_NULL) ||
	    (osi_core->osd_ops.msleep == OSI_NULL) ||
	    (osi_core->osd_ops.usleep_range == OSI_NULL)) {
		return -1;
	}

	if (osi_core->mac > OSI_MAC_HW_MGBE) {
		OSI_CORE_ERR(OSI_NULL, OSI_LOG_ARG_INVALID,
			     "Invalid MAC HW type\n", 0ULL);
		return -1;
	}

	if (osi_core->use_virtualization > OSI_ENABLE) {
		OSI_CORE_ERR(OSI_NULL, OSI_LOG_ARG_INVALID,
			     "Invalid use_virtualization value\n", 0ULL);
		return -1;
	}

	if (i_ops[osi_core->mac][osi_core->use_virtualization] != OSI_NULL) {
		i_ops[osi_core->mac][osi_core->use_virtualization](&g_ops[osi_core->mac]);
	}

	if (s_init[osi_core->mac][osi_core->use_virtualization] != OSI_NULL) {
		osi_core->safety_config =
			s_init[osi_core->mac][osi_core->use_virtualization]();
	}

	if (validate_func_ptrs(osi_core, &g_ops[osi_core->mac]) < 0) {
		OSI_CORE_ERR(OSI_NULL, OSI_LOG_ARG_INVALID,
				"core: function ptrs validation failed\n", 0ULL);
		return -1;
	}

	l_core->ops_p = &g_ops[osi_core->mac];
	l_core->init_done = OSI_ENABLE;

	return 0;
}

nve32_t osi_poll_for_mac_reset_complete(
			struct osi_core_priv_data *const osi_core)
{
	struct core_local *l_core = (struct core_local *)osi_core;

	if (validate_args(osi_core, l_core) < 0) {
		return -1;
	}

	return l_core->ops_p->poll_for_swr(osi_core);
}

/**
 * @brief init_vlan_filters - Helper function to init all VLAN SW information.
 *
 * Algorithm: Initilize VLAN filtering information.
 *
 * @param[in] osi_core: OSI Core private data structure.
 */
static inline void init_vlan_filters(struct osi_core_priv_data *const osi_core)
{
	unsigned int i = 0U;

	for (i = 0; i < VLAN_NUM_VID; i++) {
		osi_core->vid[i] = VLAN_ID_INVALID;
	}

	osi_core->vf_bitmap = 0U;
	osi_core->vlan_filter_cnt = 0U;
}

nve32_t osi_hw_core_init(struct osi_core_priv_data *const osi_core,
			 nveu32_t tx_fifo_size, nveu32_t rx_fifo_size)
{
	struct core_local *l_core = (struct core_local *)osi_core;

	if (validate_args(osi_core, l_core) < 0) {
		return -1;
	}

	init_vlan_filters(osi_core);

	/* Init FRP */
	init_frp(osi_core);

	return l_core->ops_p->core_init(osi_core, tx_fifo_size, rx_fifo_size);
}

nve32_t osi_hw_core_deinit(struct osi_core_priv_data *const osi_core)
{
	struct core_local *l_core = (struct core_local *)osi_core;

	if (validate_args(osi_core, l_core) < 0) {
		return -1;
	}

	l_core->ops_p->core_deinit(osi_core);

	l_core->init_done = OSI_DISABLE;
	l_core->magic_num = 0;

	return 0;
}

nve32_t osi_start_mac(struct osi_core_priv_data *const osi_core)
{
	struct core_local *l_core = (struct core_local *)osi_core;

	if (validate_args(osi_core, l_core) < 0) {
		return -1;
	}

	l_core->ops_p->start_mac(osi_core);

	return 0;
}

nve32_t osi_stop_mac(struct osi_core_priv_data *const osi_core)
{
	struct core_local *l_core = (struct core_local *)osi_core;

	if (validate_args(osi_core, l_core) < 0) {
		return -1;
	}

	l_core->ops_p->stop_mac(osi_core);

	return 0;
}

nve32_t osi_common_isr(struct osi_core_priv_data *const osi_core)
{
	struct core_local *l_core = (struct core_local *)osi_core;

	if (validate_args(osi_core, l_core) < 0) {
		return -1;
	}

	l_core->ops_p->handle_common_intr(osi_core);

	return 0;
}

nve32_t osi_set_mode(struct osi_core_priv_data *const osi_core,
		     const nve32_t mode)
{
	struct core_local *l_core = (struct core_local *)osi_core;

	if (validate_args(osi_core, l_core) < 0) {
		return -1;
	}

	return l_core->ops_p->set_mode(osi_core, mode);
}

nve32_t osi_set_speed(struct osi_core_priv_data *const osi_core,
		      const nve32_t speed)
{
	struct core_local *l_core = (struct core_local *)osi_core;

	if (validate_args(osi_core, l_core) < 0) {
		return -1;
	}

	return l_core->ops_p->set_speed(osi_core, speed);
}

nve32_t osi_pad_calibrate(struct osi_core_priv_data *const osi_core)
{
	struct core_local *l_core = (struct core_local *)osi_core;

	if (validate_args(osi_core, l_core) < 0) {
		return -1;
	}

	return l_core->ops_p->pad_calibrate(osi_core);
}

nve32_t osi_config_fw_err_pkts(struct osi_core_priv_data *const osi_core,
			       const nveu32_t qinx, const nveu32_t fw_err)
{
	struct core_local *l_core = (struct core_local *)osi_core;

	if (validate_args(osi_core, l_core) < 0) {
		return -1;
	}

	/* Configure Forwarding of Error packets */
	return l_core->ops_p->config_fw_err_pkts(osi_core, qinx, fw_err);
}

int osi_config_ptp_offload(struct osi_core_priv_data *const osi_core,
			   struct osi_pto_config *const pto_config)
{
	struct core_local *l_core = (struct core_local *)osi_core;
	int ret = -1;

	if (validate_args(osi_core, l_core) < 0) {
		return -1;
	}

	/* Validate input arguments */
	if (pto_config == OSI_NULL) {
		OSI_CORE_ERR(OSI_NULL, OSI_LOG_ARG_INVALID,
			"pto_config is NULL\n", 0ULL);
		return ret;
	}

	if (pto_config->mc_uc != OSI_ENABLE &&
	    pto_config->mc_uc != OSI_DISABLE) {
		OSI_CORE_ERR(osi_core->osd, OSI_LOG_ARG_INVALID,
			"invalid mc_uc flag value\n",
			pto_config->mc_uc);
		return ret;
	}

	if (pto_config->en_dis != OSI_ENABLE &&
	    pto_config->en_dis != OSI_DISABLE) {
		OSI_CORE_ERR(osi_core->osd, OSI_LOG_ARG_INVALID,
			"invalid enable flag value\n",
			pto_config->en_dis);
		return ret;
	}

	if (pto_config->snap_type != OSI_PTP_SNAP_ORDINARY &&
	    pto_config->snap_type != OSI_PTP_SNAP_TRANSPORT &&
	    pto_config->snap_type != OSI_PTP_SNAP_P2P) {
		OSI_CORE_ERR(osi_core->osd, OSI_LOG_ARG_INVALID,
			"invalid SNAP type value\n",
			pto_config->snap_type);
		return ret;
	}

	if (pto_config->master != OSI_ENABLE &&
	    pto_config->master != OSI_DISABLE) {
		OSI_CORE_ERR(osi_core->osd, OSI_LOG_ARG_INVALID,
			"invalid master flag value\n",
			pto_config->master);
		return ret;
	}

	if (pto_config->domain_num >= OSI_PTP_MAX_DOMAIN) {
		OSI_CORE_ERR(osi_core->osd, OSI_LOG_ARG_INVALID,
			"invalid ptp domain\n",
			pto_config->domain_num);
		return ret;
	}

	if (pto_config->portid >= OSI_PTP_MAX_PORTID) {
		OSI_CORE_ERR(osi_core->osd, OSI_LOG_ARG_INVALID,
			"invalid ptp port ID\n",
			pto_config->portid);
		return ret;
	}

	ret = l_core->ops_p->config_ptp_offload(osi_core, pto_config);
	if (ret < 0) {
		OSI_CORE_ERR(osi_core->osd, OSI_LOG_ARG_INVALID,
			"Fail to configure PTO\n",
			pto_config->en_dis);
		return ret;
	}

	/* Configure PTP */
	ret = osi_ptp_configuration(osi_core, pto_config->en_dis);
	if (ret < 0) {
		OSI_CORE_ERR(osi_core->osd, OSI_LOG_ARG_INVALID,
			"Fail to configure PTP\n",
			pto_config->en_dis);
		return ret;
	}

	return ret;
}

nve32_t osi_l2_filter(struct osi_core_priv_data *const osi_core,
		      const struct osi_filter *filter)
{
	struct core_local *l_core = (struct core_local *)osi_core;
	nve32_t ret;

	if ((validate_args(osi_core, l_core) < 0) || (filter == OSI_NULL)) {
		return -1;
	}

	if (filter == OSI_NULL) {
		OSI_CORE_ERR(OSI_NULL, OSI_LOG_ARG_INVALID,
			     "CORE: filter is NULL\n", 0ULL);
		return -1;
	}

	ret = l_core->ops_p->config_mac_pkt_filter_reg(osi_core, filter);
	if (ret < 0) {
		OSI_CORE_ERR(OSI_NULL, OSI_LOG_ARG_HW_FAIL,
			     "failed to configure MAC packet filter register\n",
			     0ULL);
		return ret;
	}

	if (((filter->oper_mode & OSI_OPER_ADDR_UPDATE) != OSI_NONE) ||
	    ((filter->oper_mode & OSI_OPER_ADDR_DEL) != OSI_NONE)) {
		ret = -1;

		if ((filter->dma_routing == OSI_ENABLE) &&
		    (osi_core->dcs_en != OSI_ENABLE)) {
			OSI_CORE_ERR(osi_core->osd, OSI_LOG_ARG_INVALID,
				     "DCS requested. Conflicts with DT config\n",
				     0ULL);
			return ret;
		}

		ret = l_core->ops_p->update_mac_addr_low_high_reg(osi_core, filter);
	}

	return ret;
}

/**
 * @brief helper_l4_filter helper function for l4 filtering
 *
 * @param[in] osi_core: OSI Core private data structure.
 * @param[in] l_filter: filter structure
 * @param[in] type: filter type l3 or l4
 * @param[in] dma_routing_enable: dma routing enable (1) or disable (0)
 * @param[in] dma_chan: dma channel
 *
 * @pre MAC needs to be out of reset and proper clock configured.
 *
 * @note
 * API Group:
 * - Initialization: Yes
 * - Run time: Yes
 * - De-initialization: No
 *
 * @retval 0 on Success
 * @retval -1 on Failure
 */
static inline nve32_t helper_l4_filter(
				   struct osi_core_priv_data *const osi_core,
				   struct core_ops *ops_p,
				   struct osi_l3_l4_filter l_filter,
				   nveu32_t type,
				   nveu32_t dma_routing_enable,
				   nveu32_t dma_chan)
{
	nve32_t ret = 0;

	ret = ops_p->config_l4_filters(osi_core,
				    l_filter.filter_no,
				    l_filter.filter_enb_dis,
				    type,
				    l_filter.src_dst_addr_match,
				    l_filter.perfect_inverse_match,
				    dma_routing_enable,
				    dma_chan);
	if (ret < 0) {
		OSI_CORE_ERR(OSI_NULL, OSI_LOG_ARG_HW_FAIL,
			     "failed to configure L4 filters\n", 0ULL);
		return ret;
	}

	return ops_p->update_l4_port_no(osi_core,
				     l_filter.filter_no,
				     l_filter.port_no,
				     l_filter.src_dst_addr_match);
}

/**
 * @brief helper_l3_filter helper function for l3 filtering
 *
 * @param[in] osi_core: OSI Core private data structure.
 * @param[in] l_filter: filter structure
 * @param[in] type: filter type l3 or l4
 * @param[in] dma_routing_enable: dma routing enable (1) or disable (0)
 * @param[in] dma_chan: dma channel
 *
 * @pre MAC needs to be out of reset and proper clock configured.
 *
 * @note
 * API Group:
 * - Initialization: No
 * - Run time: Yes
 * - De-initialization: No
 *
 * @retval 0 on Success
 * @retval -1 on Failure
 */
static inline nve32_t helper_l3_filter(
				   struct osi_core_priv_data *const osi_core,
				   struct core_ops *ops_p,
				   struct osi_l3_l4_filter l_filter,
				   nveu32_t type,
				   nveu32_t dma_routing_enable,
				   nveu32_t dma_chan)
{
	nve32_t ret = 0;

	ret = ops_p->config_l3_filters(osi_core,
				    l_filter.filter_no,
				    l_filter.filter_enb_dis,
				    type,
				    l_filter.src_dst_addr_match,
				    l_filter.perfect_inverse_match,
				    dma_routing_enable,
				    dma_chan);
	if (ret < 0) {
		OSI_CORE_ERR(OSI_NULL, OSI_LOG_ARG_HW_FAIL,
			     "failed to configure L3 filters\n", 0ULL);
		return ret;
	}

	if (type == OSI_IP6_FILTER) {
		ret = ops_p->update_ip6_addr(osi_core, l_filter.filter_no,
					  l_filter.ip6_addr);
	} else if (type == OSI_IP4_FILTER) {
		ret = ops_p->update_ip4_addr(osi_core, l_filter.filter_no,
					  l_filter.ip4_addr,
					  l_filter.src_dst_addr_match);
	} else {
		OSI_CORE_ERR(OSI_NULL, OSI_LOG_ARG_INVALID,
			     "Invalid L3 filter type\n", 0ULL);
                return -1;
	}

	return ret;
}

nve32_t osi_l3l4_filter(struct osi_core_priv_data *const osi_core,
			const struct osi_l3_l4_filter l_filter,
			const nveu32_t type, const nveu32_t dma_routing_enable,
			const nveu32_t dma_chan, const nveu32_t is_l4_filter)
{
	struct core_local *l_core = (struct core_local *)osi_core;
	nve32_t ret = -1;

	if (validate_args(osi_core, l_core) < 0) {
		return -1;
	}

	if ((dma_routing_enable == OSI_ENABLE) &&
	    (osi_core->dcs_en != OSI_ENABLE)) {
		OSI_CORE_ERR(osi_core->osd, OSI_LOG_ARG_INVALID,
			     "dma routing enabled but dcs disabled in DT\n",
			     0ULL);
		return ret;
	}

	if (is_l4_filter == OSI_ENABLE) {
		ret = helper_l4_filter(osi_core, l_core->ops_p, l_filter, type,
				       dma_routing_enable, dma_chan);
	} else {
		ret = helper_l3_filter(osi_core, l_core->ops_p, l_filter, type,
				       dma_routing_enable, dma_chan);
	}

	if (ret < 0) {
		OSI_CORE_INFO(osi_core->osd, OSI_LOG_ARG_INVALID,
			      "L3/L4 helper function failed\n", 0ULL);
		return ret;
	}

	if (osi_core->l3l4_filter_bitmask != OSI_DISABLE) {
		ret = l_core->ops_p->config_l3_l4_filter_enable(osi_core, OSI_ENABLE);
	} else {
		ret = l_core->ops_p->config_l3_l4_filter_enable(osi_core, OSI_DISABLE);
	}

	return ret;
}

nve32_t osi_config_rxcsum_offload(struct osi_core_priv_data *const osi_core,
				  const nveu32_t enable)
{
	struct core_local *l_core = (struct core_local *)osi_core;

	if (validate_args(osi_core, l_core) < 0) {
		return -1;
	}

	return l_core->ops_p->config_rxcsum_offload(osi_core, enable);
}

nve32_t osi_set_systime_to_mac(struct osi_core_priv_data *const osi_core,
			       const nveu32_t sec, const nveu32_t nsec)
{
	struct core_local *l_core = (struct core_local *)osi_core;

	if (validate_args(osi_core, l_core) < 0) {
		return -1;
	}

	return l_core->ops_p->set_systime_to_mac(osi_core, sec, nsec);
}

/**
 * @brief div_u64 - Calls a function which returns quotient
 *
 * @param[in] dividend: Dividend
 * @param[in] divisor: Divisor
 *
 * @pre MAC IP should be out of reset and need to be initialized as the
 *      requirements.
 *
 *
 * @note
 * API Group:
 * - Initialization: No
 * - Run time: Yes
 * - De-initialization: No
 * @returns Quotient
 */
static inline nveu64_t div_u64(nveu64_t dividend,
			       nveu64_t divisor)
{
	nveu64_t remain;

	return div_u64_rem(dividend, divisor, &remain);
}

nve32_t osi_adjust_freq(struct osi_core_priv_data *const osi_core, nve32_t ppb)
{
	struct core_local *l_core = (struct core_local *)osi_core;

	nveu64_t adj;
	nveu64_t temp;
	nveu32_t diff = 0;
	nveu32_t addend;
	nveu32_t neg_adj = 0;
	nve32_t ret = -1;
	nve32_t ppb1 = ppb;

	if (validate_args(osi_core, l_core) < 0) {
		return -1;
	}

	addend = osi_core->default_addend;
	if (ppb1 < 0) {
		neg_adj = 1U;
		ppb1 = -ppb1;
		adj = (nveu64_t)addend * (nveu32_t)ppb1;
	} else {
		adj = (nveu64_t)addend * (nveu32_t)ppb1;
	}

	/*
	 * div_u64 will divide the "adj" by "1000000000ULL"
	 * and return the quotient.
	 */
	temp = div_u64(adj, OSI_NSEC_PER_SEC);
	if (temp < UINT_MAX) {
		diff = (nveu32_t)temp;
	} else {
		OSI_CORE_ERR(OSI_NULL, OSI_LOG_ARG_INVALID, "temp > UINT_MAX\n",
			     0ULL);
		return ret;
	}

	if (neg_adj == 0U) {
		if (addend <= (UINT_MAX - diff)) {
			addend = (addend + diff);
		} else {
			OSI_CORE_ERR(OSI_NULL, OSI_LOG_ARG_INVALID,
				     "addend > UINT_MAX\n", 0ULL);
			return -1;
		}
	} else {
		if (addend > diff) {
			addend = addend - diff;
		} else if (addend < diff) {
			addend = diff - addend;
		} else {
			OSI_CORE_ERR(OSI_NULL, OSI_LOG_ARG_INVALID,
				     "addend = diff\n", 0ULL);
		}
	}

	return l_core->ops_p->config_addend(osi_core, addend);
}

nve32_t osi_adjust_time(struct osi_core_priv_data *const osi_core,
			nvel64_t nsec_delta)
{
	struct core_local *l_core = (struct core_local *)osi_core;
	nveu32_t neg_adj = 0;
	nveu32_t sec = 0, nsec = 0;
	nveu64_t quotient;
	nveu64_t reminder = 0;
	nveu64_t udelta = 0;
	nve32_t ret = -1;
	nvel64_t nsec_delta1 = nsec_delta;

	if (validate_args(osi_core, l_core) < 0) {
		return -1;
	}

	if (nsec_delta1 < 0) {
		neg_adj = 1;
		nsec_delta1 = -nsec_delta1;
		udelta = (nveul64_t)nsec_delta1;
	} else {
		udelta = (nveul64_t)nsec_delta1;
	}

	quotient = div_u64_rem(udelta, OSI_NSEC_PER_SEC, &reminder);
	if (quotient <= UINT_MAX) {
		sec = (nveu32_t)quotient;
	} else {
		OSI_CORE_ERR(OSI_NULL, OSI_LOG_ARG_INVALID,
			     "quotient > UINT_MAX\n", 0ULL);
		return ret;
	}

	if (reminder <= UINT_MAX) {
		nsec = (nveu32_t)reminder;
	} else {
		OSI_CORE_ERR(OSI_NULL, OSI_LOG_ARG_INVALID,
			     "reminder > UINT_MAX\n", 0ULL);
		return ret;
	}

	return l_core->ops_p->adjust_mactime(osi_core, sec, nsec, neg_adj,
					     osi_core->ptp_config.one_nsec_accuracy);
}

nve32_t osi_ptp_configuration(struct osi_core_priv_data *const osi_core,
			      const nveu32_t enable)
{
	struct core_local *l_core = (struct core_local *)osi_core;
	nve32_t ret = 0;
	nveu64_t temp = 0, temp1 = 0, temp2 = 0;
	nveu64_t ssinc = 0;

	if (validate_args(osi_core, l_core) < 0) {
		return -1;
	}

	if (enable == OSI_DISABLE) {
		/* disable hw time stamping */
		/* Program MAC_Timestamp_Control Register */
		l_core->ops_p->config_tscr(osi_core, OSI_DISABLE);
		/* Disable PTP RX Queue routing */
		ret = l_core->ops_p->config_ptp_rxq(osi_core,
					    osi_core->ptp_config.ptp_rx_queue,
					    OSI_DISABLE);
	} else {
		/* Program MAC_Timestamp_Control Register */
		l_core->ops_p->config_tscr(osi_core, osi_core->ptp_config.ptp_filter);

		if (osi_core->pre_si == OSI_ENABLE) {
			if (osi_core->mac == OSI_MAC_HW_MGBE) {
				/* FIXME: Pass it from OSD */
				osi_core->ptp_config.ptp_clock = 78125000U;
				osi_core->ptp_config.ptp_ref_clk_rate =
								 78125000U;
			} else {
				/* FIXME: Pass it from OSD */
				osi_core->ptp_config.ptp_clock = 312500000U;
				osi_core->ptp_config.ptp_ref_clk_rate =
								 312500000U;
			}
		}
		/* Program Sub Second Increment Register */
		l_core->ops_p->config_ssir(osi_core, osi_core->ptp_config.ptp_clock);

		/* formula for calculating addend value is
		 * TSAR = (2^32 * 1000) / (ptp_ref_clk_rate in MHz * SSINC)
		 * 2^x * y == (y << x), hence
		 * 2^32 * 1000 == (1000 << 32)
		 * so addend = (2^32 * 1000)/(ptp_ref_clk_rate in MHZ * SSINC);
		 */
		if ((osi_core->pre_si == OSI_ENABLE) &&
		    ((osi_core->mac == OSI_MAC_HW_MGBE) ||
		    (osi_core->mac_ver <= OSI_EQOS_MAC_4_10))) {
			ssinc = OSI_PTP_SSINC_16;
		} else {
			ssinc = OSI_PTP_SSINC_4;
		}

		temp = ((nveu64_t)1000 << 32);
		temp = (nveu64_t)temp * 1000000U;

		temp1 = div_u64(temp,
			(nveu64_t)osi_core->ptp_config.ptp_ref_clk_rate);

		temp2 = div_u64(temp1, (nveu64_t)ssinc);

		if (temp2 < UINT_MAX) {
			osi_core->default_addend = (nveu32_t)temp2;
		} else {
			OSI_CORE_ERR(OSI_NULL, OSI_LOG_ARG_INVALID,
				     "core: temp2 >= UINT_MAX\n", 0ULL);
			return -1;
		}

		/* Program addend value */
		ret = l_core->ops_p->config_addend(osi_core, osi_core->default_addend);

		/* Set current time */
		if (ret == 0) {
			ret = l_core->ops_p->set_systime_to_mac(osi_core,
						     osi_core->ptp_config.sec,
						     osi_core->ptp_config.nsec);
			if (ret == 0) {
				/* Enable PTP RX Queue routing */
				ret = l_core->ops_p->config_ptp_rxq(osi_core,
					osi_core->ptp_config.ptp_rx_queue,
					OSI_ENABLE);
			}
		}
	}

	return ret;
}

int osi_rxq_route(struct osi_core_priv_data *const osi_core,
		  const struct osi_rxq_route *rxq_route)
{
	struct core_local *l_core = (struct core_local *)osi_core;

	if (validate_args(osi_core, l_core) < 0) {
		return -1;
	}

	if (rxq_route->route_type != OSI_RXQ_ROUTE_PTP) {
		OSI_CORE_ERR(osi_core->osd, OSI_LOG_ARG_INVALID,
			     "Invalid route_type\n",
			     rxq_route->route_type);
		return -1;
	}

	return l_core->ops_p->config_ptp_rxq(osi_core,
				     rxq_route->idx,
				     rxq_route->enable);
}

nve32_t osi_read_mmc(struct osi_core_priv_data *const osi_core)
{
	struct core_local *l_core = (struct core_local *)osi_core;

	if (validate_args(osi_core, l_core) < 0) {
		return -1;
	}

	l_core->ops_p->read_mmc(osi_core);

	return 0;
}

nve32_t osi_get_mac_version(struct osi_core_priv_data *const osi_core,
		            nveu32_t *mac_ver)
{
	struct core_local *l_core = (struct core_local *)osi_core;

	if (validate_args(osi_core, l_core) < 0) {
		return -1;
	}

	if (mac_ver == OSI_NULL) {
		OSI_CORE_ERR(OSI_NULL, OSI_LOG_ARG_INVALID,
			     "mac_ver is NULL\n", 0ULL);
		return -1;
	}

	*mac_ver = ((l_core->ops_p->read_reg(osi_core, (nve32_t)MAC_VERSION)) &
		    MAC_VERSION_SNVER_MASK);

	if (is_valid_mac_version(*mac_ver) == 0) {
		OSI_CORE_ERR(OSI_NULL, OSI_LOG_ARG_INVALID,
			     "Invalid MAC version\n", (nveu64_t)*mac_ver)
		return -1;
	}

	return 0;
}

#ifndef OSI_STRIPPED_LIB
nve32_t osi_validate_core_regs(struct osi_core_priv_data *const osi_core)
{
	struct core_local *l_core = (struct core_local *)osi_core;

	if (validate_args(osi_core, l_core) < 0) {
		return -1;
	}

	if (osi_core->safety_config == OSI_NULL) {
		OSI_CORE_ERR(OSI_NULL, OSI_LOG_ARG_INVALID,
			     "CORE: Safety config is NULL\n", 0ULL);
		return -1;
	}

	return l_core->ops_p->validate_regs(osi_core);
}

nve32_t osi_flush_mtl_tx_queue(struct osi_core_priv_data *const osi_core,
			       const nveu32_t qinx)
{
	struct core_local *l_core = (struct core_local *)osi_core;

	if (validate_args(osi_core, l_core) < 0) {
		return -1;
	}

	return l_core->ops_p->flush_mtl_tx_queue(osi_core, qinx);
}

nve32_t osi_set_avb(struct osi_core_priv_data *const osi_core,
		    const struct osi_core_avb_algorithm *avb)
{
	struct core_local *l_core = (struct core_local *)osi_core;

	if (validate_args(osi_core, l_core) < 0) {
		return -1;
	}

	return l_core->ops_p->set_avb_algorithm(osi_core, avb);
}

nve32_t osi_get_avb(struct osi_core_priv_data *const osi_core,
		    struct osi_core_avb_algorithm *avb)
{
	struct core_local *l_core = (struct core_local *)osi_core;

	if (validate_args(osi_core, l_core) < 0) {
		return -1;
	}

	return l_core->ops_p->get_avb_algorithm(osi_core, avb);
}

nve32_t osi_configure_txstatus(struct osi_core_priv_data *const osi_core,
			       const nveu32_t tx_status)
{
	struct core_local *l_core = (struct core_local *)osi_core;

	if (validate_args(osi_core, l_core) < 0) {
		return -1;
	}

	/* Configure Drop Transmit Status */
	return l_core->ops_p->config_tx_status(osi_core, tx_status);
}

nve32_t osi_config_rx_crc_check(struct osi_core_priv_data *const osi_core,
				const nveu32_t crc_chk)
{
	struct core_local *l_core = (struct core_local *)osi_core;

	if (validate_args(osi_core, l_core) < 0) {
		return -1;
	}

	/* Configure CRC Checking for Received Packets */
	return l_core->ops_p->config_rx_crc_check(osi_core, crc_chk);
}

nve32_t osi_config_vlan_filtering(struct osi_core_priv_data *const osi_core,
				  const nveu32_t filter_enb_dis,
				  const nveu32_t perfect_hash_filtering,
				  const nveu32_t perfect_inverse_match)
{
	struct core_local *l_core = (struct core_local *)osi_core;

	if (validate_args(osi_core, l_core) < 0) {
		return -1;
	}

	return l_core->ops_p->config_vlan_filtering(osi_core, filter_enb_dis,
					 perfect_hash_filtering,
					 perfect_inverse_match);
}

nve32_t osi_update_vlan_id(struct osi_core_priv_data *const osi_core,
			    const nveu32_t vid)
{
	struct core_local *l_core = (struct core_local *)osi_core;
	unsigned int action = vid & VLAN_ACTION_MASK;
	unsigned short vlan_id = vid & VLAN_VID_MASK;


	if (validate_args(osi_core, l_core) < 0) {
		return -1;
	}

	if ((osi_core->mac_ver == OSI_EQOS_MAC_4_10) ||
	    (osi_core->mac_ver == OSI_EQOS_MAC_5_00)) {
		/* No VLAN ID filtering */
		return 0;
	}

	if (((action != OSI_VLAN_ACTION_ADD) &&
	    (action != OSI_VLAN_ACTION_DEL)) ||
	    (vlan_id >= VLAN_NUM_VID)) {
		OSI_CORE_ERR(OSI_NULL, OSI_LOG_ARG_INVALID,
			     "CORE: Invalid action/vlan_id\n", 0ULL);
		/* Unsupported action */
		return -1;
	}

	return update_vlan_id(osi_core, l_core->ops_p, vid);
}

nve32_t osi_reset_mmc(struct osi_core_priv_data *const osi_core)
{
	struct core_local *l_core = (struct core_local *)osi_core;

	if (validate_args(osi_core, l_core) < 0) {
		return -1;
	}

	l_core->ops_p->reset_mmc(osi_core);

	return 0;
}

nve32_t osi_configure_eee(struct osi_core_priv_data *const osi_core,
			  nveu32_t tx_lpi_enabled, nveu32_t tx_lpi_timer)
{
	struct core_local *l_core = (struct core_local *)osi_core;

	if (validate_args(osi_core, l_core) < 0) {
		return -1;
	}

	if ((tx_lpi_timer >= OSI_MAX_TX_LPI_TIMER) ||
	    (tx_lpi_timer <= OSI_MIN_TX_LPI_TIMER) ||
	    ((tx_lpi_timer % OSI_MIN_TX_LPI_TIMER) != OSI_NONE)) {
		OSI_CORE_ERR(OSI_NULL, OSI_LOG_ARG_INVALID,
			     "Invalid Tx LPI timer value\n",
			     (nveul64_t)tx_lpi_timer);
		return -1;
	}

	l_core->ops_p->configure_eee(osi_core, tx_lpi_enabled, tx_lpi_timer);

	return 0;
}

nve32_t osi_save_registers(struct osi_core_priv_data *const osi_core)
{
	struct core_local *l_core = (struct core_local *)osi_core;

	if (validate_args(osi_core, l_core) < 0) {
		return -1;
	}

	/* Call MAC save registers callback and return the value */
	return l_core->ops_p->save_registers(osi_core);
}

nve32_t osi_restore_registers(struct osi_core_priv_data *const osi_core)
{
	struct core_local *l_core = (struct core_local *)osi_core;

	if (validate_args(osi_core, l_core) < 0) {
		return -1;
	}

	/* Call MAC restore registers callback and return the value */
	return l_core->ops_p->restore_registers(osi_core);
}

nve32_t osi_configure_flow_control(struct osi_core_priv_data *const osi_core,
				   const nveu32_t flw_ctrl)
{
	struct core_local *l_core = (struct core_local *)osi_core;

	if (validate_args(osi_core, l_core) < 0) {
		return -1;
	}

	/* Configure Flow control settings */
	return l_core->ops_p->config_flow_control(osi_core, flw_ctrl);
}

int osi_configure_frp(struct osi_core_priv_data *const osi_core,
		      struct osi_core_frp_cmd *const cmd)
{
	struct core_local *l_core = (struct core_local *)osi_core;

	if (validate_args(osi_core, l_core) < 0) {
		return -1;
	}

	if (cmd == OSI_NULL) {
		OSI_CORE_ERR(OSI_NULL, OSI_LOG_ARG_INVALID,
			"Invalid argment\n", OSI_NONE);
		return -1;
	}

	/* Check for supported MAC version */
	if ((osi_core->mac == OSI_MAC_HW_EQOS) &&
	    (osi_core->mac_ver < OSI_EQOS_MAC_5_10)) {
		OSI_CORE_ERR(osi_core->osd, OSI_LOG_ARG_HW_FAIL,
			"MAC doesn't support FRP\n", OSI_NONE);
		return -1;
	}

	return setup_frp(osi_core, l_core->ops_p, cmd);
}

nve32_t osi_config_arp_offload(struct osi_core_priv_data *const osi_core,
			       const nveu32_t flags,
			       const nveu8_t *ip_addr)
{
	struct core_local *l_core = (struct core_local *)osi_core;

	if ((validate_args(osi_core, l_core) < 0)) {
		return -1;
	}

	if (ip_addr == OSI_NULL) {
		OSI_CORE_ERR(OSI_NULL, OSI_LOG_ARG_INVALID,
			     "CORE: ip_addr is NULL\n", 0ULL);
		return -1;
	}

	if ((flags != OSI_ENABLE) && (flags != OSI_DISABLE)) {
		OSI_CORE_ERR(OSI_NULL, OSI_LOG_ARG_INVALID,
			     "Invalid ARP offload enable/disable flag\n", 0ULL);
		return -1;
	}

	return l_core->ops_p->config_arp_offload(osi_core, flags, ip_addr);
}

nve32_t osi_set_mdc_clk_rate(struct osi_core_priv_data *const osi_core,
			     const nveu64_t csr_clk_rate)
{
	struct core_local *l_core = (struct core_local *)osi_core;

	if (validate_args(osi_core, l_core) < 0) {
		return -1;
	}

	l_core->ops_p->set_mdc_clk_rate(osi_core, csr_clk_rate);

	return 0;
}

int osi_config_rss(struct osi_core_priv_data *const osi_core)
{
	struct core_local *l_core = (struct core_local *)osi_core;

	if (validate_args(osi_core, l_core) < 0) {
		return -1;
	}

	return l_core->ops_p->config_rss(osi_core);
}

int osi_hw_config_est(struct osi_core_priv_data *osi_core,
		      struct osi_est_config *est)
{
	struct core_local *l_core = (struct core_local *)osi_core;

	if (validate_args(osi_core, l_core) < 0) {
		return -1;
	}

	if (est == OSI_NULL) {
		OSI_CORE_ERR(osi_core->osd, OSI_LOG_ARG_INVALID,
			     "EST data is NULL", 0ULL);
		return -1;
	}

	if ((osi_core->flow_ctrl & OSI_FLOW_CTRL_TX) ==
	     OSI_FLOW_CTRL_TX) {
		OSI_CORE_ERR(osi_core->osd, OSI_LOG_ARG_INVALID,
			     "TX Flow control enabled, please disable it",
			      0ULL);
		return -1;
	}

	return l_core->ops_p->hw_config_est(osi_core, est);
}

nve32_t osi_config_mac_loopback(struct osi_core_priv_data *const osi_core,
				const nveu32_t lb_mode)
{
	struct core_local *l_core = (struct core_local *)osi_core;

	if (validate_args(osi_core, l_core) < 0) {
		return -1;
	}

	/* don't allow only if loopback mode is other than 0 or 1 */
	if (lb_mode != OSI_ENABLE && lb_mode != OSI_DISABLE) {
		OSI_CORE_ERR(OSI_NULL, OSI_LOG_ARG_INVALID,
			     "Invalid loopback mode\n", 0ULL);
		return -1;
	}

	/* Configure MAC loopback */
	return l_core->ops_p->config_mac_loopback(osi_core, lb_mode);
}

int osi_hw_config_fpe(struct osi_core_priv_data *osi_core,
		      struct osi_fpe_config *fpe)
{
	struct core_local *l_core = (struct core_local *)osi_core;

	if (validate_args(osi_core, l_core) < 0) {
		return -1;
	}

	if (fpe == OSI_NULL) {
		OSI_CORE_ERR(osi_core->osd, OSI_LOG_ARG_INVALID,
			     "FPE data is NULL", 0ULL);
		return -1;
	}

	return l_core->ops_p->hw_config_fpe(osi_core, fpe);
}
#endif /* !OSI_STRIPPED_LIB */

nve32_t osi_get_hw_features(struct osi_core_priv_data *const osi_core,
			    struct osi_hw_features *hw_feat)
{
	struct core_local *l_core = (struct core_local *)osi_core;

	if (validate_args(osi_core, l_core) < 0) {
		return -1;
	}

	if (hw_feat == OSI_NULL) {
		OSI_CORE_ERR(OSI_NULL, OSI_LOG_ARG_INVALID,
			     "CORE: Invalid hw_feat\n", 0ULL);
		return -1;
	}

	return l_core->ops_p->get_hw_features(osi_core, hw_feat);
}
