/*
 **************************************************************************
 * Copyright (c) 2016-2019 The Linux Foundation. All rights reserved.
 *
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted, provided that the
 * above copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT
 * OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 **************************************************************************
 */

#include <common.h>
#include <asm/global_data.h>
#include "ipq6018_ppe.h"
#include "ipq6018_uniphy.h"
#include <fdtdec.h>
#include "ipq_phy.h"

DECLARE_GLOBAL_DATA_PTR;
#define pr_info(fmt, args...) printf(fmt, ##args);
/*
 * ipq6018_ppe_gpio_reg_write()
 */
static inline void ipq6018_ppe_gpio_reg_write(u32 reg, u32 val)
{
	writel(val, IPQ6018_PPE_FPGA_GPIO_BASE_ADDR + reg);
}

/*
 * ipq6018_ppe_reg_read()
 */
static inline void ipq6018_ppe_reg_read(u32 reg, u32 *val)
{
	*val = readl((void *)(IPQ6018_PPE_BASE_ADDR + reg));
}

/*
 * ipq6018_ppe_reg_write()
 */
static inline void ipq6018_ppe_reg_write(u32 reg, u32 val)
{
	writel(val, (void *)(IPQ6018_PPE_BASE_ADDR + reg));
}

void ppe_ipo_rule_reg_set(union ipo_rule_reg_u *hw_reg, int rule_id)
{
	int i;

	for (i = 0; i < 3; i++) {
		ipq6018_ppe_reg_write(IPO_CSR_BASE_ADDR + IPO_RULE_REG_ADDRESS +
			(rule_id * IPO_RULE_REG_INC) + (i * 4), hw_reg->val[i]);
	}
}

void ppe_ipo_mask_reg_set(union ipo_mask_reg_u *hw_mask, int rule_id)
{
	int i;

	for (i = 0; i < 2; i++) {
		ipq6018_ppe_reg_write((IPO_CSR_BASE_ADDR + IPO_MASK_REG_ADDRESS +
			(rule_id * IPO_MASK_REG_INC) + (i * 4)), hw_mask->val[i]);
	}
}

void ppe_ipo_action_set(union ipo_action_u *hw_act, int rule_id)
{
	int i;

	for (i = 0; i < 5; i++) {
		ipq6018_ppe_reg_write((IPE_L2_BASE_ADDR + IPO_ACTION_ADDRESS +
			(rule_id * IPO_ACTION_INC) + (i * 4)), hw_act->val[i]);
	}
}

void ipq6018_ppe_acl_set(int rule_id, int rule_type, int pkt_type, int l4_port_no, int l4_port_mask, int permit, int deny)
{
	union ipo_rule_reg_u hw_reg = {0};
	union ipo_mask_reg_u hw_mask = {0};
	union ipo_action_u hw_act = {0};

	memset(&hw_reg, 0, sizeof(hw_reg));
	memset(&hw_mask, 0, sizeof(hw_mask));
	memset(&hw_act, 0, sizeof(hw_act));

	if (rule_id < MAX_RULE) {
		if (rule_type == ADPT_ACL_HPPE_IPV4_DIP_RULE) {
			hw_reg.bf.rule_type = ADPT_ACL_HPPE_IPV4_DIP_RULE;
			hw_reg.bf.rule_field_0 = l4_port_no;
			hw_reg.bf.rule_field_1 = pkt_type<<17;
			hw_mask.bf.maskfield_0 = l4_port_mask;
			hw_mask.bf.maskfield_1 = 7<<17;
			if (permit == 0x0) {
				hw_act.bf.dest_info_change_en = 1;
				hw_act.bf.fwd_cmd = 0;/*forward*/
				hw_reg.bf.pri = 0x1;
			}

			if (deny == 0x1) {
				hw_act.bf.dest_info_change_en = 1;
				hw_act.bf.fwd_cmd = 1;/*drop*/
				hw_reg.bf.pri = 0x0;

			}
			hw_reg.bf.src_0 = 0x6;
			hw_reg.bf.src_1 = 0x7;
			ppe_ipo_rule_reg_set(&hw_reg, rule_id);
			ppe_ipo_mask_reg_set(&hw_mask, rule_id);
			ppe_ipo_action_set(&hw_act, rule_id);
		}
	}
}

/*
 * ipq6018_ppe_vp_port_tbl_set()
 */
static void ipq6018_ppe_vp_port_tbl_set(int port, int vsi)
{
	u32 addr = IPQ6018_PPE_L3_VP_PORT_TBL_ADDR +
		 (port * IPQ6018_PPE_L3_VP_PORT_TBL_INC);
	ipq6018_ppe_reg_write(addr, 0x0);
	ipq6018_ppe_reg_write(addr + 0x4 , 1 << 9 | vsi << 10);
	ipq6018_ppe_reg_write(addr + 0x8, 0x0);
}

/*
 * ipq6018_ppe_ucast_queue_map_tbl_queue_id_set()
 */
static void ipq6018_ppe_ucast_queue_map_tbl_queue_id_set(int queue, int port)
{
	uint32_t val;

	ipq6018_ppe_reg_read(IPQ6018_PPE_QM_UQM_TBL +
		 (port * IPQ6018_PPE_UCAST_QUEUE_MAP_TBL_INC), &val);

	val |= queue << 4;

	ipq6018_ppe_reg_write(IPQ6018_PPE_QM_UQM_TBL +
		 (port * IPQ6018_PPE_UCAST_QUEUE_MAP_TBL_INC), val);
}

/*
 * ipq6018_vsi_setup()
 */
static void ipq6018_vsi_setup(int vsi, uint8_t group_mask)
{
	uint32_t val = (group_mask << 24 | group_mask << 16 | group_mask << 8
							    | group_mask);

	/* Set mask */
	ipq6018_ppe_reg_write(0x061800 + (vsi * 0x10), val);

	/*  new addr lrn en | station move lrn en */
	ipq6018_ppe_reg_write(0x061804 + (vsi * 0x10), 0x9);
}

/*
 * ipq6018_gmac_port_enable()
 */
static void ipq6018_gmac_port_enable(int port)
{
	ipq6018_ppe_reg_write(IPQ6018_PPE_MAC_ENABLE + (0x200 * port), 0x70);
	ipq6018_ppe_reg_write(IPQ6018_PPE_MAC_SPEED + (0x200 * port), 0x2);
	ipq6018_ppe_reg_write(IPQ6018_PPE_MAC_MIB_CTL + (0x200 * port), 0x1);
}

void ipq6018_speed_clock_set(int port, int speed_clock1, int speed_clock2)
{
	int i;

	for (i = 0; i < 2; i++)
	{
		writel(speed_clock2, GCC_NSS_PORT1_RX_MISC + i*4 + port*0x10);
		writel(speed_clock1, GCC_NSS_PORT1_RX_CFG_RCGR + i*8 + port*0x10);
		writel(0x1, GCC_NSS_PORT1_RX_CMD_RCGR + i*8 + port*0x10);
	}
}

int phy_status_get_from_ppe(int port_id)
{
	uint32_t reg_field = 0;

	ipq6018_ppe_reg_read(PORT_PHY_STATUS_ADDRESS, &reg_field);
	if (port_id == (PORT5 - PPE_UNIPHY_INSTANCE1))
		reg_field >>= PORT_PHY_STATUS_PORT5_1_OFFSET;
	else
		reg_field >>= PORT_PHY_STATUS_PORT6_OFFSET;

	return ((reg_field >> 7) & 0x1) ? 0 : 1;
}

void ppe_port_bridge_txmac_set(int port_id, int status)
{
	uint32_t reg_value = 0;

	ipq6018_ppe_reg_read(IPE_L2_BASE_ADDR + PORT_BRIDGE_CTRL_ADDRESS +
		 (port_id * PORT_BRIDGE_CTRL_INC), &reg_value);
	if (status == 0)
		reg_value |= TX_MAC_EN;
	else
		reg_value &= ~TX_MAC_EN;

	ipq6018_ppe_reg_write(IPE_L2_BASE_ADDR + PORT_BRIDGE_CTRL_ADDRESS +
		 (port_id * PORT_BRIDGE_CTRL_INC), reg_value);

}

void ipq6018_pqsgmii_speed_set(int port, int speed, int status)
{
	ppe_port_bridge_txmac_set(port + 1, status);
	ipq6018_ppe_reg_write(IPQ6018_PPE_MAC_SPEED + (0x200 * port), speed);
	ipq6018_ppe_reg_write(IPQ6018_PPE_MAC_ENABLE + (0x200 * port), 0x73);
}



void ppe_xgmac_speed_set(uint32_t uniphy_index, int speed)
{
	uint32_t reg_value = 0;

	ipq6018_ppe_reg_read(PPE_SWITCH_NSS_SWITCH_XGMAC0 +
		 (uniphy_index * NSS_SWITCH_XGMAC_MAC_TX_CONFIGURATION), &reg_value);

	switch(speed) {
	case 0:
	case 1:
	case 2:
		reg_value &=~USS;
		reg_value |=SS(XGMAC_SPEED_SELECT_1000M);
		break;
	case 3:
		reg_value |=USS;
		reg_value |=SS(XGMAC_SPEED_SELECT_10000M);
		break;
	case 4:
		reg_value |=USS;
		reg_value |=SS(XGMAC_SPEED_SELECT_2500M);
		break;
	case 5:
		reg_value |=USS;
		reg_value |=SS(XGMAC_SPEED_SELECT_5000M);
		break;
	}
	reg_value |=JD;
	ipq6018_ppe_reg_write(PPE_SWITCH_NSS_SWITCH_XGMAC0 +
		 (uniphy_index * NSS_SWITCH_XGMAC_MAC_TX_CONFIGURATION), reg_value);

}

void ppe_xgmac_10g_r_speed_set(uint32_t uniphy_index)
{
	uint32_t reg_value = 0;

	ipq6018_ppe_reg_read(PPE_SWITCH_NSS_SWITCH_XGMAC0 +
		 (uniphy_index * NSS_SWITCH_XGMAC_MAC_TX_CONFIGURATION), &reg_value);

	reg_value |=JD;
	ipq6018_ppe_reg_write(PPE_SWITCH_NSS_SWITCH_XGMAC0 +
		 (uniphy_index * NSS_SWITCH_XGMAC_MAC_TX_CONFIGURATION), reg_value);

}

void ppe_port_txmac_status_set(uint32_t uniphy_index)
{
	uint32_t reg_value = 0;

	ipq6018_ppe_reg_read(PPE_SWITCH_NSS_SWITCH_XGMAC0 +
		 (uniphy_index * NSS_SWITCH_XGMAC_MAC_TX_CONFIGURATION), &reg_value);

	reg_value |=TE;
	ipq6018_ppe_reg_write(PPE_SWITCH_NSS_SWITCH_XGMAC0 +
		 (uniphy_index * NSS_SWITCH_XGMAC_MAC_TX_CONFIGURATION), reg_value);

}

void ppe_port_rxmac_status_set(uint32_t uniphy_index)
{
	uint32_t reg_value = 0;

	ipq6018_ppe_reg_read(PPE_SWITCH_NSS_SWITCH_XGMAC0 +
			MAC_RX_CONFIGURATION_ADDRESS +
			(uniphy_index * NSS_SWITCH_XGMAC_MAC_RX_CONFIGURATION), &reg_value);

	reg_value |= 0x5ee00c0;
	reg_value |=RE;
	reg_value |=ACS;
	reg_value |=CST;
	ipq6018_ppe_reg_write(PPE_SWITCH_NSS_SWITCH_XGMAC0 +
			MAC_RX_CONFIGURATION_ADDRESS +
			(uniphy_index * NSS_SWITCH_XGMAC_MAC_RX_CONFIGURATION), reg_value);

}

void ppe_mac_packet_filter_set(uint32_t uniphy_index)
{
	ipq6018_ppe_reg_write(PPE_SWITCH_NSS_SWITCH_XGMAC0 +
			MAC_PACKET_FILTER_ADDRESS +
			(uniphy_index * MAC_PACKET_FILTER_INC), 0x81);
}

void ipq6018_10g_r_speed_set(int port, int status)
{
	uint32_t uniphy_index;

	/* Setting the speed only for PORT5 and PORT6 */
	if (port == (PORT5 - PPE_UNIPHY_INSTANCE1))
		uniphy_index = PPE_UNIPHY_INSTANCE1;
	else if (port == (PORT6 - PPE_UNIPHY_INSTANCE1))
		uniphy_index = PPE_UNIPHY_INSTANCE2;
	else
		return;

	ppe_xgmac_10g_r_speed_set(uniphy_index - 1);
	ppe_port_bridge_txmac_set(port + 1, status);
	ppe_port_txmac_status_set(uniphy_index - 1);
	ppe_port_rxmac_status_set(uniphy_index - 1);
	ppe_mac_packet_filter_set(uniphy_index - 1);
}

void ipq6018_uxsgmii_speed_set(int port, int speed, int duplex,
				int status)
{
	uint32_t uniphy_index;

	/* Setting the speed only for PORT5 and PORT6 */
	if (port == (PORT5 - PPE_UNIPHY_INSTANCE1))
		uniphy_index = PPE_UNIPHY_INSTANCE1;
	else if (port == (PORT6 - PPE_UNIPHY_INSTANCE1))
		uniphy_index = PPE_UNIPHY_INSTANCE2;
	else
		return;

	ppe_uniphy_usxgmii_autoneg_completed(uniphy_index);
	ppe_uniphy_usxgmii_speed_set(uniphy_index, speed);
	ppe_xgmac_speed_set(uniphy_index - 1, speed);
	ppe_uniphy_usxgmii_duplex_set(uniphy_index, duplex);
	ppe_uniphy_usxgmii_port_reset(uniphy_index);
	ppe_port_bridge_txmac_set(port + 1, status);
	ppe_port_txmac_status_set(uniphy_index - 1);
	ppe_port_rxmac_status_set(uniphy_index - 1);
	ppe_mac_packet_filter_set(uniphy_index - 1);
}
/*
 * ipq6018_ppe_flow_port_map_tbl_port_num_set()
 */
static void ipq6018_ppe_flow_port_map_tbl_port_num_set(int queue, int port)
{
	ipq6018_ppe_reg_write(IPQ6018_PPE_L0_FLOW_PORT_MAP_TBL +
			queue * IPQ6018_PPE_L0_FLOW_PORT_MAP_TBL_INC, port);
	ipq6018_ppe_reg_write(IPQ6018_PPE_L1_FLOW_PORT_MAP_TBL +
			port * IPQ6018_PPE_L1_FLOW_PORT_MAP_TBL_INC, port);
}

/*
 * ipq6018_ppe_flow_map_tbl_set()
 */
static void ipq6018_ppe_flow_map_tbl_set(int queue, int port)
{
	uint32_t val = port | 0x401000; /* c_drr_wt = 1, e_drr_wt = 1 */
	ipq6018_ppe_reg_write(IPQ6018_PPE_L0_FLOW_MAP_TBL + queue * IPQ6018_PPE_L0_FLOW_MAP_TBL_INC,
									val);

	val = port | 0x100400; /* c_drr_wt = 1, e_drr_wt = 1 */
	ipq6018_ppe_reg_write(IPQ6018_PPE_L1_FLOW_MAP_TBL + port * IPQ6018_PPE_L1_FLOW_MAP_TBL_INC,
									val);
}

/*
 * ipq6018_ppe_tdm_configuration
 */
static void ipq6018_ppe_tdm_configuration(void)
{
	/*
	 * TDM is configured with instructions for each tick
	 * Port/action are configured as given below
	 *
	 * 0x5:0x5	TDM_CFG_VALID		0:idle tick
	 * 0x4:0x4	TDM_CFG_DIR		0:ingress wr
	 *					1:egress rd
	 * 0x3:0x0	TDM_CFG_PORT_NUM	0:DMA
	 *					1~4:Ethernet 1G
	 *					5~6:Ethernet 5G
	 *					7~8:Security0/1
	 */

	ipq6018_ppe_reg_write(0xc000, 0x20);
	ipq6018_ppe_reg_write(0xc010, 0x30);
	ipq6018_ppe_reg_write(0xc020, 0x25);
	ipq6018_ppe_reg_write(0xc030, 0x34);
	ipq6018_ppe_reg_write(0xc040, 0x21);
	ipq6018_ppe_reg_write(0xc050, 0x35);
	ipq6018_ppe_reg_write(0xc060, 0x26);
	ipq6018_ppe_reg_write(0xc070, 0x36);
	ipq6018_ppe_reg_write(0xc080, 0x20);
	ipq6018_ppe_reg_write(0xc090, 0x30);
	ipq6018_ppe_reg_write(0xc0a0, 0x27);
	ipq6018_ppe_reg_write(0xc0b0, 0x37);
	ipq6018_ppe_reg_write(0xc0c0, 0x24);
	ipq6018_ppe_reg_write(0xc0d0, 0x30);
	ipq6018_ppe_reg_write(0xc0e0, 0x26);
	ipq6018_ppe_reg_write(0xc0f0, 0x35);
	ipq6018_ppe_reg_write(0xc100, 0x20);
	ipq6018_ppe_reg_write(0xc110, 0x30);
	ipq6018_ppe_reg_write(0xc120, 0x22);
	ipq6018_ppe_reg_write(0xc130, 0x36);
	ipq6018_ppe_reg_write(0xc140, 0x27);
	ipq6018_ppe_reg_write(0xc150, 0x37);
	ipq6018_ppe_reg_write(0xc160, 0x25);
	ipq6018_ppe_reg_write(0xc170, 0x35);
	ipq6018_ppe_reg_write(0xc180, 0x20);
	ipq6018_ppe_reg_write(0xc190, 0x30);
	ipq6018_ppe_reg_write(0xc1a0, 0x26);
	ipq6018_ppe_reg_write(0xc1b0, 0x36);
	ipq6018_ppe_reg_write(0xc1c0, 0x27);
	ipq6018_ppe_reg_write(0xc1d0, 0x33);
	ipq6018_ppe_reg_write(0xc1e0, 0x25);
	ipq6018_ppe_reg_write(0xc1f0, 0x37);
	ipq6018_ppe_reg_write(0xc200, 0x20);
	ipq6018_ppe_reg_write(0xc210, 0x30);
	ipq6018_ppe_reg_write(0xc220, 0x26);
	ipq6018_ppe_reg_write(0xc230, 0x35);
	ipq6018_ppe_reg_write(0xc240, 0x20);
	ipq6018_ppe_reg_write(0xc250, 0x36);
	ipq6018_ppe_reg_write(0xc260, 0x27);
	ipq6018_ppe_reg_write(0xc270, 0x37);
	ipq6018_ppe_reg_write(0xc280, 0x20);
	ipq6018_ppe_reg_write(0xc290, 0x30);
	ipq6018_ppe_reg_write(0xc2a0, 0x25);
	ipq6018_ppe_reg_write(0xc2b0, 0x34);
	ipq6018_ppe_reg_write(0xc2c0, 0x26);
	ipq6018_ppe_reg_write(0xc2d0, 0x36);
	ipq6018_ppe_reg_write(0xc2e0, 0x27);
	ipq6018_ppe_reg_write(0xc2f0, 0x37);
	ipq6018_ppe_reg_write(0xc300, 0x20);
	ipq6018_ppe_reg_write(0xc310, 0x30);
	ipq6018_ppe_reg_write(0xc320, 0x24);
	ipq6018_ppe_reg_write(0xc330, 0x35);
	ipq6018_ppe_reg_write(0xc340, 0x25);
	ipq6018_ppe_reg_write(0xc350, 0x31);
	ipq6018_ppe_reg_write(0xc360, 0x26);
	ipq6018_ppe_reg_write(0xc370, 0x36);
	ipq6018_ppe_reg_write(0xc380, 0x20);
	ipq6018_ppe_reg_write(0xc390, 0x30);
	ipq6018_ppe_reg_write(0xc3a0, 0x27);
	ipq6018_ppe_reg_write(0xc3b0, 0x37);
	ipq6018_ppe_reg_write(0xc3c0, 0x20);
	ipq6018_ppe_reg_write(0xc3d0, 0x34);
	ipq6018_ppe_reg_write(0xc3e0, 0x25);
	ipq6018_ppe_reg_write(0xc3f0, 0x36);
	ipq6018_ppe_reg_write(0xc400, 0x20);
	ipq6018_ppe_reg_write(0xc410, 0x30);
	ipq6018_ppe_reg_write(0xc420, 0x26);
	ipq6018_ppe_reg_write(0xc430, 0x32);
	ipq6018_ppe_reg_write(0xc440, 0x27);
	ipq6018_ppe_reg_write(0xc450, 0x37);
	ipq6018_ppe_reg_write(0xc460, 0x25);
	ipq6018_ppe_reg_write(0xc470, 0x35);
	ipq6018_ppe_reg_write(0xc480, 0x20);
	ipq6018_ppe_reg_write(0xc490, 0x30);
	ipq6018_ppe_reg_write(0xc4a0, 0x26);
	ipq6018_ppe_reg_write(0xc4b0, 0x36);
	ipq6018_ppe_reg_write(0xc4c0, 0x23);
	ipq6018_ppe_reg_write(0xc4d0, 0x37);
	ipq6018_ppe_reg_write(0xc4e0, 0x27);
	ipq6018_ppe_reg_write(0xc4f0, 0x35);
	ipq6018_ppe_reg_write(0xc500, 0x20);
	ipq6018_ppe_reg_write(0xc510, 0x30);
	ipq6018_ppe_reg_write(0xc520, 0x25);
	ipq6018_ppe_reg_write(0xc530, 0x36);
	ipq6018_ppe_reg_write(0xc540, 0x26);
	ipq6018_ppe_reg_write(0xc550, 0x30);
	ipq6018_ppe_reg_write(0xc560, 0x27);
	ipq6018_ppe_reg_write(0xc570, 0x37);
	ipq6018_ppe_reg_write(0xc580, 0x20);
	ipq6018_ppe_reg_write(0xc590, 0x30);
	ipq6018_ppe_reg_write(0xc5a0, 0x24);
	ipq6018_ppe_reg_write(0xc5b0, 0x35);
	ipq6018_ppe_reg_write(0xc5c0, 0x26);
	ipq6018_ppe_reg_write(0xc5d0, 0x36);
	ipq6018_ppe_reg_write(0xc5e0, 0x27);
	ipq6018_ppe_reg_write(0xc5f0, 0x37);
	ipq6018_ppe_reg_write(0xb000, 0x80000060);
}

/*
 * ipq6018_ppe_sched_configuration
 */
static void ipq6018_ppe_sched_configuration(void)
{
	/*
	 * PSCH_TDM_CFG_TBL_DES_PORT : determine which egress port traffic
	 *			will be selected and transmitted out
	 * PSCH_TDM_CFG_TBL_ENS_PORT : determine which portâ€™s queue need
	 *			to be linked to scheduler at the current tick
	 * PSCH_TDM_CFG_TBL_ENS_PORT_BITMAP : determine port bitmap
	 *			for source of queue
	 *
	 * 0xf:0x8	PSCH_TDM_CFG_TBL_ENS_PORT_BITMAP	1110_1110
	 *							(Port:765-432)
	 *
	 * 0x7:0x4	PSCH_TDM_CFG_TBL_ENS_PORT		0:DMA
	 *							1~4:Ethernet 1G
	 *							5~6:Ethernet 5G
	 *							7~8:Security0/1
	 *
	 * 0x3:0x0	PSCH_TDM_CFG_TBL_DES_PORT		0:DMA
	 *							1~4:Ethernet 1G
	 *							5~6:Ethernet 5G
	 *							7~8:Security0/1
	 *
	 * For eg, 0xee60 =((IPQ6018_PPE_PORT_CRYPTO1_BITPOS | IPQ6018_PPE_PORT_XGMAC2_BITPOS |
	 *		IPQ6018_PPE_PORT_XGMAC1_BITPOS | IPQ6018_PPE_PORT_QCOM3_BITPOS |
	 *		IPQ6018_PPE_PORT_QCOM2_BITPOS | IPQ6018_PPE_PORT_QCOM1_BITPOS) << 8) |
	 *		IPQ6018_PPE_PORT_XGMAC2 | IPQ6018_PPE_PORT_EDMA);
	 */

	ipq6018_ppe_reg_write(0x47a000,  0xB706);
	ipq6018_ppe_reg_write(0x47a010,  0xBE30);
	ipq6018_ppe_reg_write(0x47a020,  0xDE65);
	ipq6018_ppe_reg_write(0x47a030,  0xDD01);
	ipq6018_ppe_reg_write(0x47a040,  0xBD56);
	ipq6018_ppe_reg_write(0x47a050,  0xBE10);
	ipq6018_ppe_reg_write(0x47a060,  0xEE64);
	ipq6018_ppe_reg_write(0x47a070,  0xCF05);
	ipq6018_ppe_reg_write(0x47a080,  0x9F46);
	ipq6018_ppe_reg_write(0x47a090,  0xBE50);
	ipq6018_ppe_reg_write(0x47a0a0,  0x7E67);
	ipq6018_ppe_reg_write(0x47a0b0,  0x5F05);
	ipq6018_ppe_reg_write(0x47a0c0,  0x9F76);
	ipq6018_ppe_reg_write(0x47a0d0,  0xBE50);
	ipq6018_ppe_reg_write(0x47a0e0,  0xFA62);
	ipq6018_ppe_reg_write(0x47a0f0,  0xBB06);
	ipq6018_ppe_reg_write(0x47a100,  0x9F25);
	ipq6018_ppe_reg_write(0x47a110,  0xCF64);
	ipq6018_ppe_reg_write(0x47a120,  0xEE50);
	ipq6018_ppe_reg_write(0x47a130,  0xBE46);
	ipq6018_ppe_reg_write(0x47a140,  0x3F07);
	ipq6018_ppe_reg_write(0x47a150,  0x5F65);
	ipq6018_ppe_reg_write(0x47a160,  0xDE70);
	ipq6018_ppe_reg_write(0x47a170,  0xBE56);
	ipq6018_ppe_reg_write(0x47a180,  0xB703);
	ipq6018_ppe_reg_write(0x47a190,  0xE764);
	ipq6018_ppe_reg_write(0x47a1a0,  0xEE30);
	ipq6018_ppe_reg_write(0x47a1b0,  0xBE46);
	ipq6018_ppe_reg_write(0x47a1c0,  0x9F05);
	ipq6018_ppe_reg_write(0x47a1d0,  0xDD61);
	ipq6018_ppe_reg_write(0x47a1e0,  0xFC50);
	ipq6018_ppe_reg_write(0x47a1f0,  0xBE16);
	ipq6018_ppe_reg_write(0x47a200,  0x9F05);
	ipq6018_ppe_reg_write(0x47a210,  0x5F67);
	ipq6018_ppe_reg_write(0x47a220,  0x7E50);
	ipq6018_ppe_reg_write(0x47a230,  0xBE76);
	ipq6018_ppe_reg_write(0x47a240,  0xAF04);
	ipq6018_ppe_reg_write(0x47a250,  0xCF65);
	ipq6018_ppe_reg_write(0x47a260,  0x9F46);
	ipq6018_ppe_reg_write(0x47a270,  0xBE50);
	ipq6018_ppe_reg_write(0x47a280,  0xFA62);
	ipq6018_ppe_reg_write(0x47a290,  0xDB05);
	ipq6018_ppe_reg_write(0x47a2a0,  0x9F26);
	ipq6018_ppe_reg_write(0x47a2b0,  0xBE50);
	ipq6018_ppe_reg_write(0x47a2c0,  0x7E67);
	ipq6018_ppe_reg_write(0x47a2d0,  0x6F04);
	ipq6018_ppe_reg_write(0x47a2e0,  0xAF76);
	ipq6018_ppe_reg_write(0x47a2f0,  0x9F45);
	ipq6018_ppe_reg_write(0x47a300,  0xDE60);
	ipq6018_ppe_reg_write(0x47a310,  0xF653);
	ipq6018_ppe_reg_write(0x400000,  0x32);
}

/*
 * ipq6018_ppe_c_sp_cfg_tbl_drr_id_set
 */
static void ipq6018_ppe_c_sp_cfg_tbl_drr_id_set(int id)
{
	ipq6018_ppe_reg_write(IPQ6018_PPE_L0_C_SP_CFG_TBL + (id * 0x80), id * 2);
	ipq6018_ppe_reg_write(IPQ6018_PPE_L1_C_SP_CFG_TBL + (id * 0x80), id * 2);
}

/*
 * ipq6018_ppe_e_sp_cfg_tbl_drr_id_set
 */
static void ipq6018_ppe_e_sp_cfg_tbl_drr_id_set(int id)
{
	ipq6018_ppe_reg_write(IPQ6018_PPE_L0_E_SP_CFG_TBL + (id * 0x80), id * 2 + 1);
	ipq6018_ppe_reg_write(IPQ6018_PPE_L1_E_SP_CFG_TBL + (id * 0x80), id * 2 + 1);
}

static void ppe_port_mux_set(int port_id, int port_type, int mode)
{
	union port_mux_ctrl_u port_mux_ctrl;

	ipq6018_ppe_reg_read(IPQ6018_PORT_MUX_CTRL,  &(port_mux_ctrl.val));
	port_mux_ctrl.bf.port4_pcs_sel = PORT4_PCS_SEL_GMII_FROM_PCS0;
	if (port_id == PORT5) {
		if (port_type == PORT_GMAC_TYPE) {
			if (mode == PORT_WRAPPER_SGMII_PLUS)
				port_mux_ctrl.bf.port5_pcs_sel = PORT5_PCS_SEL_GMII_FROM_PCS1;
			else
				port_mux_ctrl.bf.port5_pcs_sel = PORT5_PCS_SEL_GMII_FROM_PCS0;
			port_mux_ctrl.bf.port5_gmac_sel = PORT5_GMAC_SEL_GMAC;
		} else if (port_type == PORT_XGMAC_TYPE) {
			port_mux_ctrl.bf.port5_pcs_sel = PORT5_PCS_SEL_GMII_FROM_PCS1;
			port_mux_ctrl.bf.port5_gmac_sel = PORT5_GMAC_SEL_XGMAC;
		}
	} else if (port_id == PORT6) {
		if (port_type == PORT_GMAC_TYPE) {
			port_mux_ctrl.bf.port6_pcs_sel = PORT6_PCS_SEL_GMII_FROM_PCS2;
			port_mux_ctrl.bf.port6_gmac_sel = PORT6_GMAC_SEL_GMAC;
		} else if (port_type == PORT_XGMAC_TYPE) {
			port_mux_ctrl.bf.port6_pcs_sel = PORT6_PCS_SEL_GMII_FROM_PCS2;
			port_mux_ctrl.bf.port6_gmac_sel = PORT6_GMAC_SEL_XGMAC;
		}
	} else
		return;

	ipq6018_ppe_reg_write(IPQ6018_PORT_MUX_CTRL,  port_mux_ctrl.val);
}

static void ppe_port_mux_mac_type_set(int port_id, int mode)
{
	uint32_t port_type;

	switch(mode)
	{
		case PORT_WRAPPER_SGMII0_RGMII4:
			port_type = PORT_GMAC_TYPE;
			break;
		case PORT_WRAPPER_SGMII_PLUS:
			port_type = PORT_GMAC_TYPE;
			break;
		case PORT_WRAPPER_USXGMII:
			port_type = PORT_XGMAC_TYPE;
			break;
		case PORT_WRAPPER_10GBASE_R:
			port_type = PORT_XGMAC_TYPE;
			break;
		default:
			return;
	}
	ppe_port_mux_set(port_id, port_type, mode);
}



void ipq6018_ppe_interface_mode_init(void)
{
	uint32_t mode0, mode1, mode2;
	int node;

	node = fdt_path_offset(gd->fdt_blob, "/ess-switch");
	if (node < 0) {
		printf("Error: ess-switch not specified in dts");
		return;
	}

	mode0 = fdtdec_get_uint(gd->fdt_blob, node, "switch_mac_mode", -1);
	if (mode0 < 0) {
		printf("Error: switch_mac_mode not specified in dts");
		return;
	}

	mode1 = fdtdec_get_uint(gd->fdt_blob, node, "switch_mac_mode1", -1);
	if (mode1 < 0) {
		printf("Error: switch_mac_mode1 not specified in dts");
		return;
	}
	mode2 = fdtdec_get_uint(gd->fdt_blob, node, "switch_mac_mode2", -1);
	if (mode2 < 0) {
		printf("Error: switch_mac_mode2 not specified in dts");
		return;
	}

	ppe_uniphy_mode_set(PPE_UNIPHY_INSTANCE0, mode0);
	ppe_uniphy_mode_set(PPE_UNIPHY_INSTANCE1, mode1);
	ppe_uniphy_mode_set(PPE_UNIPHY_INSTANCE2, mode2);

	/* Port 1-4 are used mac type as GMAC by default but Port5 and Port6
	* can be used as GMAC or XGMAC */
	ppe_port_mux_mac_type_set(PORT5, mode1);
	ppe_port_mux_mac_type_set(PORT6, mode2);
}

/*
 * ipq6018_ppe_provision_init()
 */
void ipq6018_ppe_provision_init(void)
{
	int i;
	uint32_t queue;

	/* Port4 Port5 port mux configuration, all GMAC */
	writel(0x10, 0x3a000010);

	/* tdm/sched configuration */
	ipq6018_ppe_tdm_configuration();
	ipq6018_ppe_sched_configuration();

	/* disable clock gating */
	ipq6018_ppe_reg_write(0x000008, 0x0);

	/* flow ctrl disable */
	ipq6018_ppe_reg_write(0x200368, 0xc88);

#ifdef CONFIG_IPQ6018_BRIDGED_MODE
	/* Add CPU port 0 to VSI 2 */
	ipq6018_ppe_vp_port_tbl_set(0, 2);

	/* Add port 1 - 4 to VSI 2 */
	ipq6018_ppe_vp_port_tbl_set(1, 2);
	ipq6018_ppe_vp_port_tbl_set(2, 2);
	ipq6018_ppe_vp_port_tbl_set(3, 2);
	ipq6018_ppe_vp_port_tbl_set(4, 2);
	ipq6018_ppe_vp_port_tbl_set(5, 2);

#else
	ipq6018_ppe_vp_port_tbl_set(1, 2);
	ipq6018_ppe_vp_port_tbl_set(2, 3);
	ipq6018_ppe_vp_port_tbl_set(3, 4);
	ipq6018_ppe_vp_port_tbl_set(4, 5);
#endif

	/* Unicast priority map */
	ipq6018_ppe_reg_write(IPQ6018_PPE_QM_UPM_TBL, 0);

	/* Port0 - 7 unicast queue settings */
	for (i = 0; i < 8; i++) {
		if (i == 0)
			queue = 0;
		else
			queue = ((i * 0x10) + 0x70);

		ipq6018_ppe_ucast_queue_map_tbl_queue_id_set(queue, i);
		ipq6018_ppe_flow_port_map_tbl_port_num_set(queue, i);
		ipq6018_ppe_flow_map_tbl_set(queue, i);
		ipq6018_ppe_c_sp_cfg_tbl_drr_id_set(i);
		ipq6018_ppe_e_sp_cfg_tbl_drr_id_set(i);
	}

	/* Port0 multicast queue */
	ipq6018_ppe_reg_write(0x409000, 0x00000000);
	ipq6018_ppe_reg_write(0x403000, 0x00401000);

	/* Port1 - 7 multicast queue */
	for (i = 1; i < 8; i++) {
		ipq6018_ppe_reg_write(0x409100 + ((i - 1) * 0x40), i);
		ipq6018_ppe_reg_write(0x403100 + ((i - 1) * 0x40), 0x401000 | i);
	}

	/*
	 * Port0 - Port7 learn enable and isolation port bitmap and TX_EN
	 * Here please pay attention on bit16 (TX_EN) is not set on port7
	 */
	for (i = 0; i < 7; i++)
		ipq6018_ppe_reg_write(IPQ6018_PPE_PORT_BRIDGE_CTRL_OFFSET + (i * 4),
			      IPQ6018_PPE_PORT_BRIDGE_CTRL_PROMISC_EN |
			      IPQ6018_PPE_PORT_BRIDGE_CTRL_TXMAC_EN |
			      IPQ6018_PPE_PORT_BRIDGE_CTRL_PORT_ISOLATION_BMP |
			      IPQ6018_PPE_PORT_BRIDGE_CTRL_STATION_LRN_EN |
			      IPQ6018_PPE_PORT_BRIDGE_CTRL_NEW_ADDR_LRN_EN);

	ipq6018_ppe_reg_write(IPQ6018_PPE_PORT_BRIDGE_CTRL_OFFSET + (7 * 4),
		      IPQ6018_PPE_PORT_BRIDGE_CTRL_PROMISC_EN |
		      IPQ6018_PPE_PORT_BRIDGE_CTRL_PORT_ISOLATION_BMP |
		      IPQ6018_PPE_PORT_BRIDGE_CTRL_STATION_LRN_EN |
		      IPQ6018_PPE_PORT_BRIDGE_CTRL_NEW_ADDR_LRN_EN);

	/* Global learning */
	ipq6018_ppe_reg_write(0x060038, 0xc0);

#ifdef CONFIG_IPQ6018_BRIDGED_MODE
	ipq6018_vsi_setup(2, 0x7f);
#else
	ipq6018_vsi_setup(2, 0x03);
	ipq6018_vsi_setup(3, 0x05);
	ipq6018_vsi_setup(4, 0x09);
	ipq6018_vsi_setup(5, 0x11);
#endif

	/* Port 0-7 STP */
	for (i = 0; i < 8; i++)
		ipq6018_ppe_reg_write(IPQ6018_PPE_STP_BASE + (0x4 * i), 0x3);

	ipq6018_ppe_interface_mode_init();
	/* Port 0-5 enable */
	for (i = 0; i < 5; i++) {
		ipq6018_gmac_port_enable(i);
		ppe_port_bridge_txmac_set(i + 1, 1);
	}

	/* Allowing DHCP packets */
	ipq6018_ppe_acl_set(0, ADPT_ACL_HPPE_IPV4_DIP_RULE, UDP_PKT, 67, 0xffff, 0, 0);
	ipq6018_ppe_acl_set(1, ADPT_ACL_HPPE_IPV4_DIP_RULE, UDP_PKT, 68, 0xffff, 0, 0);
	/* Dropping all the UDP packets */
	ipq6018_ppe_acl_set(2, ADPT_ACL_HPPE_IPV4_DIP_RULE, UDP_PKT, 0, 0, 0, 1);
}
