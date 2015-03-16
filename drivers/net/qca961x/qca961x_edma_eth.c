/*
 * Copyright (c) 2015 The Linux Foundation. All rights reserved.

 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
*/

#include <common.h>
#include <net.h>
#include <asm-generic/errno.h>
#include <asm/io.h>
#include <malloc.h>
#include <phy.h>
#include <net.h>
#include <miiphy.h>
#include <asm/arch-qca961x/ess/qca961x_edma.h>
#include "qca961x_edma_eth.h"
#ifdef DEBUG
#define debugf(fmt, args...) printf(fmt, ##args);
#else
#define debugf(fmt, args...)
#endif

extern int qca961x_ess_sw_init(qca961x_edma_board_cfg_t *cfg);
uchar qca961x_def_enetaddr[6] = {0x00, 0x03, 0x7F, 0xBA, 0xDB, 0xAD};
static struct qca961x_eth_dev *qca961x_edma_dev[QCA961X_EDMA_DEV];
static int (*qca961x_switch_init)(qca961x_edma_board_cfg_t *cfg);

void qca961x_register_switch(int(*sw_init)(qca961x_edma_board_cfg_t *cfg))
{
	qca961x_switch_init = sw_init;
}

/*
 * Enable RX queue control
 */
static void qca961x_edma_enable_rx_ctrl(struct qca961x_edma_hw *hw)
{
	volatile u32 data;

	qca961x_edma_read_reg(EDMA_REG_RXQ_CTRL, &data);
	data |= EDMA_RXQ_CTRL_EN;
	qca961x_edma_write_reg(EDMA_REG_RXQ_CTRL, data);
}

/*
 * Enable TX queue control
 */
static void qca961x_edma_enable_tx_ctrl(struct qca961x_edma_hw *hw)
{
	volatile u32 data;

	qca961x_edma_read_reg(EDMA_REG_TXQ_CTRL, &data);
	data |= EDMA_TXQ_CTRL_TXQ_EN;
	qca961x_edma_write_reg(EDMA_REG_TXQ_CTRL, data);
}

/*
 * qca961x_edma_init_desc()
 * Update descriptor ring size,
 * Update buffer and producer/consumer index
 */
static void qca961x_edma_init_desc(
	struct qca961x_edma_common_info *c_info, u8 unit)
{
	struct qca961x_edma_desc_ring *rfd_ring;
	struct qca961x_edma_desc_ring *etdr;
	volatile u32 data = 0;
	u16 hw_cons_idx = 0;

	/*
	 * Set the base address of every TPD ring.
	 */
	etdr = c_info->tpd_ring[unit];
	/*
	 * Update TX descriptor ring base address.
	 */
	qca961x_edma_write_reg(EDMA_REG_TPD_BASE_ADDR_Q(unit),
		(u32)(etdr->dma & 0xffffffff));
	qca961x_edma_read_reg(EDMA_REG_TPD_IDX_Q(unit), &data);
	/*
	 * Calculate hardware consumer index for Tx.
	 */
	hw_cons_idx = (data >> EDMA_TPD_CONS_IDX_SHIFT) & 0xffff;
	etdr->sw_next_to_fill = hw_cons_idx;
	etdr->sw_next_to_clean = hw_cons_idx;
	data &= ~(EDMA_TPD_PROD_IDX_MASK << EDMA_TPD_PROD_IDX_SHIFT);
	data |= hw_cons_idx;
	/*
	 * Update producer index for Tx.
	 */
	qca961x_edma_write_reg(EDMA_REG_TPD_IDX_Q(unit), data);
	/*
	 * Update SW consumer index register for Tx.
	 */
	qca961x_edma_write_reg(EDMA_REG_TX_SW_CONS_IDX_Q(unit),
						hw_cons_idx);
	/*
	 * Set TPD ring size.
	 */
	qca961x_edma_write_reg(EDMA_REG_TPD_RING_SIZE,
		(u32)(c_info->tx_ring_count & EDMA_TPD_RING_SIZE_MASK));
	/*
	 * Configure Rx ring.
	 */
	rfd_ring = c_info->rfd_ring[unit];
	/*
	 * Update Receive Free descriptor ring base address.
	 */
	qca961x_edma_write_reg(EDMA_REG_RFD_BASE_ADDR_Q(unit),
		(u32)(rfd_ring->dma & 0xffffffff));
	qca961x_edma_read_reg(EDMA_REG_RFD_BASE_ADDR_Q(unit), &data);
	/*
	 * Update RFD ring size and RX buffer size.
	 */
	data = (c_info->rx_ring_count & EDMA_RFD_RING_SIZE_MASK)
				<< EDMA_RFD_RING_SIZE_SHIFT;
	data |= (c_info->rx_buffer_len & EDMA_RX_BUF_SIZE_MASK)
				<< EDMA_RX_BUF_SIZE_SHIFT;
	qca961x_edma_write_reg(EDMA_REG_RX_DESC0, data);
	/*
	 * Disable TX FIFO low watermark and high watermark
	 */
	qca961x_edma_write_reg(EDMA_REG_TXF_WATER_MARK, 0);
	/*
	 * Load all of base address above
	 */
	qca961x_edma_read_reg(EDMA_REG_TX_SRAM_PART, &data);
	data |= 1 << EDMA_LOAD_PTR_SHIFT;
	qca961x_edma_write_reg(EDMA_REG_TX_SRAM_PART, data);
}

/*
 * qca961x_edma_alloc_rx_buf()
 * Allocates buffer for the received packets.
 */
static int qca961x_edma_alloc_rx_buf(
	struct qca961x_edma_common_info *c_info,
        struct qca961x_edma_desc_ring *erdr,
	u32 cleaned_count, u8 queue_id)
{
	struct edma_rx_free_desc *rx_desc;
	struct edma_sw_desc *sw_desc;
	unsigned int i;
	u16 prod_idx, length;
	u32 reg_data;

	if (cleaned_count > erdr->count) {
		debugf("Incorrect cleaned_count %d", cleaned_count);
		return -1;
	}
	i = erdr->sw_next_to_fill;

	while (cleaned_count--) {
		sw_desc = &erdr->sw_desc[i];
		length = c_info->rx_buffer_len;

		sw_desc->dma = virt_to_phys(NetRxPackets[i]);
		/*
		 * Update the buffer info.
		 */
		sw_desc->data = NetRxPackets[i];
		sw_desc->length = length;
		rx_desc = (&((struct edma_rx_free_desc *)(erdr->hw_desc))[i]);
		rx_desc->buffer_addr = cpu_to_le64(sw_desc->dma);
		if (unlikely(++i == erdr->count))
			i = 0;
	}
	erdr->sw_next_to_fill = i;

	if (unlikely(i == 0))
		prod_idx = erdr->count - 1;
	else
		prod_idx = i - 1;

	/* Update the producer index */
	qca961x_edma_read_reg(EDMA_REG_RFD_IDX_Q(queue_id), &reg_data);
	reg_data &= ~EDMA_RFD_PROD_IDX_BITS;
	reg_data |= prod_idx;
	qca961x_edma_write_reg(EDMA_REG_RFD_IDX_Q(queue_id), reg_data);
	return 0;
}

/*
 * configure reception control data.
 */
static void qca961x_edma_configure_rx(
		struct qca961x_edma_common_info *c_info)
{
	struct qca961x_edma_hw *hw = &c_info->hw;
	u32 rss_type, rx_desc1, rxq_ctrl_data;

	/*
	 * Set RSS type
	 */
	rss_type = hw->rss_type;
	qca961x_edma_write_reg(EDMA_REG_RSS_TYPE, rss_type);
	/*
	 * Set RFD burst number
	 */
	rx_desc1 = (EDMA_RFD_BURST << EDMA_RXQ_RFD_BURST_NUM_SHIFT);
	/*
	 * Set RFD prefetch threshold
	 */
	rx_desc1 |= (EDMA_RFD_THR << EDMA_RXQ_RFD_PF_THRESH_SHIFT);
	/*
	 * Set RFD in host ring low threshold to generte interrupt
	 */
	rx_desc1 |= (EDMA_RFD_LTHR << EDMA_RXQ_RFD_LOW_THRESH_SHIFT);
	qca961x_edma_write_reg(EDMA_REG_RX_DESC1, rx_desc1);
	/*
	 * Set Rx FIFO threshold to start to DMA data to host
	 */
	rxq_ctrl_data = EDMA_FIFO_THRESH_128_BYTE;
	/*
	 * Set RX remove vlan bit
	 */
	rxq_ctrl_data |= EDMA_RXQ_CTRL_RMV_VLAN;
	qca961x_edma_write_reg(EDMA_REG_RXQ_CTRL, rxq_ctrl_data);
}

/*
 * Configure transmission control data
 */
static void qca961x_edma_configure_tx(
		struct qca961x_edma_common_info *c_info)
{
	u32 txq_ctrl_data;

	txq_ctrl_data = (EDMA_TPD_BURST << EDMA_TXQ_NUM_TPD_BURST_SHIFT);
	txq_ctrl_data |=
		EDMA_TXQ_CTRL_TPD_BURST_EN;
	txq_ctrl_data |=
		(EDMA_TXF_BURST << EDMA_TXQ_TXF_BURST_NUM_SHIFT);
	qca961x_edma_write_reg(EDMA_REG_TXQ_CTRL, txq_ctrl_data);
}

static int qca961x_edma_configure(
	struct qca961x_edma_common_info *c_info)
{
	struct qca961x_edma_hw *hw = &c_info->hw;
	u32 intr_modrt_data;
	u32 intr_ctrl_data = 0;

	qca961x_edma_read_reg(EDMA_REG_INTR_CTRL, &intr_ctrl_data);
	intr_ctrl_data &= ~(1 << EDMA_INTR_SW_IDX_W_TYP_SHIFT);
	intr_ctrl_data |=
		hw->intr_sw_idx_w << EDMA_INTR_SW_IDX_W_TYP_SHIFT;
	qca961x_edma_write_reg(EDMA_REG_INTR_CTRL, intr_ctrl_data);

	/* clear interrupt status */
	qca961x_edma_write_reg(EDMA_REG_RX_ISR, 0xff);
	qca961x_edma_write_reg(EDMA_REG_TX_ISR, 0xffff);
	qca961x_edma_write_reg(EDMA_REG_MISC_ISR, 0x1fff);
	qca961x_edma_write_reg(EDMA_REG_WOL_ISR, 0x1);

	/* Clear any WOL status */
	qca961x_edma_write_reg(EDMA_REG_WOL_CTRL, 0);
	intr_modrt_data = (EDMA_TX_IMT << EDMA_IRQ_MODRT_TX_TIMER_SHIFT);
	intr_modrt_data |= (EDMA_RX_IMT << EDMA_IRQ_MODRT_RX_TIMER_SHIFT);
	qca961x_edma_write_reg(EDMA_REG_IRQ_MODRT_TIMER_INIT,
						intr_modrt_data);

	qca961x_edma_configure_tx(c_info);
	qca961x_edma_configure_rx(c_info);
	return 0;
}

static void qca961x_edma_stop_rx_tx(struct qca961x_edma_hw *hw)
{
	volatile u32 data;

	qca961x_edma_read_reg(EDMA_REG_RXQ_CTRL, &data);
	data &= ~EDMA_RXQ_CTRL_EN;
	qca961x_edma_write_reg(EDMA_REG_RXQ_CTRL, data);
	qca961x_edma_read_reg(EDMA_REG_TXQ_CTRL, &data);
	data &= ~EDMA_TXQ_CTRL_TXQ_EN;
	qca961x_edma_write_reg(EDMA_REG_TXQ_CTRL, data);
}

static int qca961x_edma_reset(struct qca961x_edma_common_info *c_info)
{
	struct qca961x_edma_hw *hw = &c_info->hw;
	int i;

	for (i = 0; i < QCA961X_EDMA_DEV; i++)
		qca961x_edma_write_reg(EDMA_REG_RX_INT_MASK_Q(i), 0);

	for (i = 0; i < QCA961X_EDMA_DEV; i++)
		qca961x_edma_write_reg(EDMA_REG_TX_INT_MASK_Q(i), 0);

	qca961x_edma_write_reg(EDMA_REG_MISC_IMR, 0);
	qca961x_edma_write_reg(EDMA_REG_WOL_IMR, 0);
	qca961x_edma_write_reg(EDMA_REG_RX_ISR, 0xff);
	qca961x_edma_write_reg(EDMA_REG_TX_ISR, 0xffff);
	qca961x_edma_write_reg(EDMA_REG_MISC_ISR, 0x1fff);
	qca961x_edma_write_reg(EDMA_REG_WOL_ISR, 0x1);
	qca961x_edma_stop_rx_tx(hw);
	return 0;
}

/*
 * qca961x_edma_get_tx_buffer()
 * Get sw_desc corresponding to the TPD
 */
static struct edma_sw_desc *qca961x_edma_get_tx_buffer(
		struct qca961x_edma_common_info *c_info,
		struct edma_tx_desc *tpd, int queue_id)
{
	struct qca961x_edma_desc_ring *etdr =
		c_info->tpd_ring[queue_id];
	return &etdr->sw_desc[tpd -
		(struct edma_tx_desc *)etdr->hw_desc];
}

/*
 * edma_get_next_tpd()
 * Return a TPD descriptor for transfer
 */
static struct edma_tx_desc *qca961x_edma_get_next_tpd(
		struct qca961x_edma_common_info *c_info,
		               		int queue_id)
{
	struct qca961x_edma_desc_ring *etdr =
		c_info->tpd_ring[queue_id];
	u16 sw_next_to_fill = etdr->sw_next_to_fill;
	struct edma_tx_desc *tpd_desc =
	(&((struct edma_tx_desc *)(etdr->hw_desc))[sw_next_to_fill]);
	etdr->sw_next_to_fill =
	(etdr->sw_next_to_fill + 1) % etdr->count;
	return tpd_desc;
}

/*
 * qca961x_edma_tx_update_hw_idx()
 * update the producer index for the ring transmitted
 */
static void qca961x_edma_tx_update_hw_idx(
		struct qca961x_edma_common_info *c_info,
                void *skb, int queue_id)
{
	struct qca961x_edma_desc_ring *etdr =
		c_info->tpd_ring[queue_id];
	volatile u32 tpd_idx_data;

	/* Read and update the producer index */
	qca961x_edma_read_reg(
		EDMA_REG_TPD_IDX_Q(queue_id), &tpd_idx_data);
	tpd_idx_data &= ~EDMA_TPD_PROD_IDX_BITS;
	tpd_idx_data |=
		((etdr->sw_next_to_fill & EDMA_TPD_PROD_IDX_MASK)
		<< EDMA_TPD_PROD_IDX_SHIFT);
	qca961x_edma_write_reg(
		EDMA_REG_TPD_IDX_Q(queue_id), tpd_idx_data);
}

/*
 * qca961x_edma_tx_map_and_fill()
 * gets called from edma_xmit_frame
 * This is where the dma of the buffer to be transmitted
 * gets mapped
 */
static int qca961x_edma_tx_map_and_fill(
		struct qca961x_edma_common_info *c_info,
                void *skb, int queue_id,
		unsigned int flags_transmit,
		unsigned int length)
{
	struct edma_sw_desc *sw_desc = NULL;
	struct edma_tx_desc *tpd;
	u32 word1 = 0, word3 = 0, lso_word1 = 0, svlan_tag = 0;
	u16 buf_len = length;

	if (likely (buf_len)) {
		tpd = qca961x_edma_get_next_tpd(c_info, queue_id);
		sw_desc = qca961x_edma_get_tx_buffer(c_info, tpd, queue_id);
		sw_desc->dma = virt_to_phys(skb);


		tpd->addr = cpu_to_le32(sw_desc->dma);
		tpd->len  = cpu_to_le16(buf_len);

		word3 |= EDMA_PORT_ENABLE_ALL << EDMA_TPD_PORT_BITMAP_SHIFT;

		tpd->svlan_tag = svlan_tag;
		tpd->word1 = word1 | lso_word1;
		tpd->word3 = word3;

		/* The last buffer info contain the skb address,
		 * so it will be free after unmap
		 */
		sw_desc->length = buf_len;
		sw_desc->flags |= EDMA_SW_DESC_FLAG_SKB_HEAD;
	}
	tpd->word1 |= 1 << EDMA_TPD_EOP_SHIFT;

	sw_desc->data = skb;
	sw_desc->flags |= EDMA_SW_DESC_FLAG_LAST;
	return 0;
}

/*
 * qca961x_edma_tpd_available()
 * Check number of free TPDs
 */
static inline u16 qca961x_edma_tpd_available(
		struct qca961x_edma_common_info *c_info,
                int queue_id)
{
	struct qca961x_edma_desc_ring *etdr =
			c_info->tpd_ring[queue_id];
	u16 sw_next_to_fill;
	u16 sw_next_to_clean;
	u16 count = 0;

	sw_next_to_clean = etdr->sw_next_to_clean;
	sw_next_to_fill = etdr->sw_next_to_fill;

	if (likely(sw_next_to_clean <= sw_next_to_fill))
		count = etdr->count;

	return count + sw_next_to_clean - sw_next_to_fill - 1;
}

static inline void qca961x_edma_tx_unmap_and_free(
		struct edma_sw_desc *sw_desc)
{
	sw_desc->flags = 0;
}

/*
 * qca961x_edma_tx_complete()
 * used to clean tx queues and
 * update hardware and consumer index
 */
static void qca961x_edma_tx_complete(
		struct qca961x_edma_common_info *c_info,
		int queue_id)
{
	struct qca961x_edma_desc_ring *etdr = c_info->tpd_ring[queue_id];
	struct edma_sw_desc *sw_desc;

	u16 sw_next_to_clean = etdr->sw_next_to_clean;
	u16 hw_next_to_clean = 0;
	volatile u32 data = 0;
	qca961x_edma_read_reg(EDMA_REG_TPD_IDX_Q(queue_id), &data);
	hw_next_to_clean =
		(data >> EDMA_TPD_CONS_IDX_SHIFT) & EDMA_TPD_CONS_IDX_MASK;
	/* clean the buffer here */
	while (sw_next_to_clean != hw_next_to_clean) {
		sw_desc = &etdr->sw_desc[sw_next_to_clean];
		qca961x_edma_tx_unmap_and_free(sw_desc);
		sw_next_to_clean = (sw_next_to_clean + 1) % etdr->count;
		etdr->sw_next_to_clean = sw_next_to_clean;
	}
	/* update the TPD consumer index register */
	qca961x_edma_write_reg(
		EDMA_REG_TX_SW_CONS_IDX_Q(queue_id), sw_next_to_clean);

}

/*
 * qca961x_edma_rx_complete()
 */
static int qca961x_edma_rx_complete(
		struct qca961x_edma_common_info *c_info,
		int queue_id)
{
	u16 cleaned_count = 0;
	u16 length = 0;
	int i = 0;
	u8 rrd[16];
	volatile u32 data = 0;
	u16 hw_next_to_clean = 0;
	u16 sw_next_to_clean = 0;
	struct qca961x_edma_desc_ring *erdr = c_info->rfd_ring[queue_id];
	struct edma_sw_desc *sw_desc;
	uchar *skb;
	sw_next_to_clean = erdr->sw_next_to_clean;

	while (1) {
		sw_desc = &erdr->sw_desc[sw_next_to_clean];
		qca961x_edma_read_reg(EDMA_REG_RFD_IDX_Q(queue_id), &data);
		hw_next_to_clean = (data >> RFD_CONS_IDX_SHIFT) &
		                              RFD_CONS_IDX_MASK;

		if (hw_next_to_clean == sw_next_to_clean) {
			break;
		}
		skb = sw_desc->data;

		/* Get RRD */
		for (i = 0; i < 16; i++)
			rrd[i] = skb[i];

		/* use next descriptor */
		sw_next_to_clean = (sw_next_to_clean + 1) % erdr->count;
		cleaned_count++;

		/* Check if RRD is valid */
		if (rrd[15] & 0x80) {
			/* Get the packet size and allocate buffer */
			length = ((rrd[13] & 0x3f) << 8) + rrd[12];
			/* Get the number of RFD from RRD */
		}
		skb = (u32 *)skb + 4;
		NetReceive(skb, length);
	}
	erdr->sw_next_to_clean = sw_next_to_clean;
	/* alloc_rx_buf */
	if (cleaned_count) {
		qca961x_edma_alloc_rx_buf(c_info, erdr,
				cleaned_count, queue_id);
		qca961x_edma_write_reg(EDMA_REG_RX_SW_CONS_IDX_Q(queue_id),
			erdr->sw_next_to_clean);
	}
	return 0;
}

static int qca961x_eth_recv(struct eth_device *dev)
{
	struct qca961x_eth_dev *priv = dev->priv;
	struct qca961x_edma_common_info *c_info = priv->c_info;
	struct queue_per_cpu_info *q_cinfo = c_info->q_cinfo;
	volatile u32 reg_data;
	u32 shadow_rx_status;

	qca961x_edma_read_reg(EDMA_REG_RX_ISR, &reg_data);
	q_cinfo->rx_status |= reg_data & q_cinfo->rx_mask;
	shadow_rx_status = q_cinfo->rx_status;

	qca961x_edma_rx_complete(c_info, priv->mac_unit);
	qca961x_edma_write_reg(EDMA_REG_RX_ISR, shadow_rx_status);
	return 0;
}

static int qca961x_edma_wr_macaddr(struct eth_device *dev)
{
	return 0;
}

static int qca961x_eth_init(struct eth_device *eth_dev, bd_t *this)
{
	struct qca961x_eth_dev *priv = eth_dev->priv;
	struct qca961x_edma_common_info *c_info = priv->c_info;
	struct qca961x_edma_desc_ring *ring;
	struct qca961x_edma_hw *hw;
	int i;
	hw = &c_info->hw;
	/*
	 * Allocate the RX buffer
	 * Qid is based on mac unit.
	 */
	ring = c_info->rfd_ring[priv->mac_unit];
	qca961x_edma_alloc_rx_buf(c_info, ring, ring->count,
					priv->mac_unit);

	/* Configure RSS indirection table.
	 * 128 hash will be configured in the following
	 * pattern: hash{0,1,2,3} = {Q0,Q2,Q4,Q6} respectively
	 * and so on
	 */
	for (i = 0; i < EDMA_NUM_IDT; i++)
		qca961x_edma_write_reg(EDMA_REG_RSS_IDT(i), EDMA_RSS_IDT_VALUE);

	qca961x_edma_enable_tx_ctrl(hw);
	qca961x_edma_enable_rx_ctrl(hw);

	return 0;
}

static int qca961x_eth_snd(struct eth_device *dev, void *packet, int length)
{
	int ret, num_tpds_needed;
	struct qca961x_eth_dev *priv = dev->priv;
	struct qca961x_edma_common_info *c_info = priv->c_info;
	struct queue_per_cpu_info *q_cinfo = c_info->q_cinfo;
	unsigned int flags_transmit = 0;
	u32 shadow_tx_status, reg_data;

	num_tpds_needed = 1;

	if ((num_tpds_needed >
	qca961x_edma_tpd_available(c_info, priv->mac_unit))) {
		debugf("Not enough descriptors available");
		return NETDEV_TX_BUSY;
	}

	flags_transmit |= EDMA_HW_CHECKSUM;
	qca961x_edma_tx_map_and_fill(c_info,
			packet, priv->mac_unit,
			flags_transmit, length);

	qca961x_edma_tx_update_hw_idx(c_info,
			packet, priv->mac_unit);

	/* Check for tx dma completion */
	qca961x_edma_read_reg(EDMA_REG_TX_ISR, &reg_data);
	q_cinfo->tx_status |= reg_data & q_cinfo->tx_mask;
	shadow_tx_status = q_cinfo->tx_status;

	qca961x_edma_tx_complete(c_info, priv->mac_unit);
	qca961x_edma_write_reg(EDMA_REG_TX_ISR, shadow_tx_status);
	return 0;
}

static void qca961x_eth_halt(struct eth_device *dev)
{
	struct qca961x_eth_dev *priv = dev->priv;
	struct qca961x_edma_common_info *c_info = priv->c_info;
	qca961x_edma_reset(c_info);
}

/*
 * Free Tx and Rx rings
 */
static void qca961x_edma_free_rings(
		struct qca961x_edma_common_info *c_info)
{
	int i;
	struct qca961x_edma_desc_ring *etdr;
	struct qca961x_edma_desc_ring *rxdr;

	for (i = 0; i < c_info->num_tx_queues; i++) {
		if (!c_info->tpd_ring[i])
			continue;
		etdr = c_info->tpd_ring[i];
		if (etdr->hw_desc)
			qca961x_free_mem(etdr->hw_desc);
		if (etdr->sw_desc)
			qca961x_free_mem(etdr->sw_desc);
	}

	for (i = 0; i < c_info->num_rx_queues; i++) {
		if (!c_info->tpd_ring[i])
			continue;
		rxdr = c_info->rfd_ring[i];
		if (rxdr->hw_desc)
			qca961x_free_mem(rxdr->hw_desc);
		if (rxdr->sw_desc)
			qca961x_free_mem(rxdr->sw_desc);
	}
}

/*
 * qca961x_edma_alloc_ring()
 * allocate edma ring descriptor.
 */
static int qca961x_edma_alloc_ring(
		struct qca961x_edma_common_info *c_info,
		struct qca961x_edma_desc_ring *erd)
{
	erd->size = (sizeof(struct edma_sw_desc) * erd->count);
	erd->sw_next_to_fill = 0;
	erd->sw_next_to_clean = 0;
	/* Allocate SW descriptors */
	erd->sw_desc = qca961x_alloc_mem(erd->size);
	if (!erd->sw_desc)
		return -ENOMEM;

	 /* Alloc HW descriptors */
	erd->hw_desc = qca961x_alloc_mem(erd->size);
	erd->dma = virt_to_phys(erd->hw_desc);
	if (!erd->hw_desc) {
		qca961x_free_mem(erd->sw_desc);
		 return -ENOMEM;
	}
	return 0;

}

/*
 * qca961x_allocate_tx_rx_rings()
 */
static int qca961x_edma_alloc_tx_rx_rings(
		struct qca961x_edma_common_info *c_info)
{
	int i, ret;
	for (i = 0; i < c_info->num_tx_queues; i++) {
		ret = qca961x_edma_alloc_ring(c_info,
				c_info->tpd_ring[i]);
		if (ret)
			goto err_ring;
	}

	for (i = 0; i < c_info->num_rx_queues; i++) {
		ret = qca961x_edma_alloc_ring(c_info,
				c_info->rfd_ring[i]);
		if (ret)
			goto err_ring;
	}
	return 0;
err_ring:
	return -1;
}

/*
 * Free Tx and Rx Queues.
 */
static void qca961x_edma_free_queues(
		struct qca961x_edma_common_info *c_info)
{
	int i;
	for (i = 0; i < c_info->num_tx_queues; i++) {
		if (c_info->tpd_ring[i]) {
			qca961x_free_mem(c_info->tpd_ring[i]);
			c_info->tpd_ring[i] = NULL;
		}
	}

	for (i = 0; i < c_info->num_rx_queues; i++) {
		if (c_info->rfd_ring[i]) {
			qca961x_free_mem(c_info->rfd_ring[i]);
			c_info->rfd_ring[i] = NULL;
		}
	}

	c_info->num_tx_queues = 0;
	c_info->num_rx_queues = 0;
}

/*
 * Allocate Tx and Rx queues.
 */
static int qca961x_edma_alloc_tx_rx_queue(
		struct qca961x_edma_common_info *c_info)
{
	int i;
	struct qca961x_edma_desc_ring *etdr;
	struct qca961x_edma_desc_ring *rfd_ring;
	/* Tx queue allocation*/
	for (i = 0; i < c_info->num_tx_queues; i++) {
		etdr = qca961x_alloc_mem(
			sizeof(struct qca961x_edma_desc_ring));
		if (!etdr)
			goto err;
		etdr->count = c_info->tx_ring_count;
		c_info->tpd_ring[i] = etdr;
	}
	/* Rx Queue allocation */
	for (i = 0; i < c_info->num_rx_queues; i++) {
		rfd_ring = qca961x_alloc_mem(
			sizeof(struct qca961x_edma_desc_ring));
		if (!rfd_ring)
			goto err;
		rfd_ring->count = c_info->rx_ring_count;
		rfd_ring->queue_index = i;
		c_info->rfd_ring[i] = rfd_ring;
	}
	return 0;
err:
	return -1;
}

int qca961x_edma_init(qca961x_edma_board_cfg_t *edma_cfg)
{
	static int qca961x_ess_init_done = 0;
	static int cfg_edma  = 0;
	struct eth_device *dev[QCA961X_EDMA_DEV];
	struct qca961x_edma_common_info *c_info[QCA961X_EDMA_DEV];
	struct qca961x_edma_hw *hw[QCA961X_EDMA_DEV];
	int i;
	int ret;

	/*
	 * Register EDMA as single ethernet
	 * interface.
	 * To-do: To register as two ethernet
	 * device once we have a LAN/WAN cfg for
	 * switch core.
	 */
	for (i = 0; i < QCA961X_EDMA_DEV; edma_cfg++, i++) {
		dev[i] = qca961x_alloc_mem(sizeof(struct eth_device));
		if (!dev[i])
			goto failed;
		memset(dev[i], 0, sizeof(struct eth_device));

		c_info[i] = qca961x_alloc_mem(
			sizeof(struct qca961x_edma_common_info));
		if (!c_info[i])
			goto failed;
		memset(c_info[i], 0,
			sizeof(struct qca961x_edma_common_info));

		c_info[i]->num_tx_queues = QCA961X_EDMA_TX_QUEUE;
		c_info[i]->tx_ring_count = QCA961X_EDMA_TX_RING_SIZE;
		c_info[i]->num_rx_queues = QCA961X_EDMA_RX_QUEUE;
		c_info[i]->rx_ring_count = QCA961X_EDMA_RX_RING_SIZE;
		c_info[i]->rx_buffer_len = QCA961X_EDMA_RX_BUFF_SIZE;

		hw[i] = &c_info[i]->hw;

		hw[i]->tx_intr_mask = QCA961X_EDMA_TX_IMR_NORMAL_MASK;
		hw[i]->rx_intr_mask = QCA961X_EDMA_RX_IMR_NORMAL_MASK;
		hw[i]->rx_buff_size = QCA961X_EDMA_RX_BUFF_SIZE;
		hw[i]->misc_intr_mask = 0;
		hw[i]->wol_intr_mask = 0;
		hw[i]->intr_clear_type = QCA961X_EDMA_INTR_CLEAR_TYPE;
		hw[i]->intr_sw_idx_w = QCA961X_EDMA_INTR_SW_IDX_W_TYPE;
		hw[i]->rss_type = QCA961X_EDMA_RSS_TYPE_NONE;

		c_info[i]->hw.hw_addr = QCA961X_EDMA_CFG_BASE;

		qca961x_edma_dev[i] = qca961x_alloc_mem(
				sizeof(struct qca961x_eth_dev));
		if (!qca961x_edma_dev[i])
			goto failed;
		memset (qca961x_edma_dev[i], 0,
				sizeof(struct qca961x_eth_dev));

		dev[i]->iobase = edma_cfg->base;
		dev[i]->init = qca961x_eth_init;
		dev[i]->halt = qca961x_eth_halt;
		dev[i]->recv = qca961x_eth_recv;
		dev[i]->send = qca961x_eth_snd;
		dev[i]->write_hwaddr = qca961x_edma_wr_macaddr;
		dev[i]->priv = (void *)qca961x_edma_dev[i];

		memcpy(&dev[i]->enetaddr[0], qca961x_def_enetaddr, 6);

		sprintf(dev[i]->name, "eth%d", i);
		qca961x_edma_dev[i]->dev  = dev[i];
		qca961x_edma_dev[i]->mac_unit = edma_cfg->unit;
		qca961x_edma_dev[i]->c_info = c_info[i];
		edma_hw_addr = (unsigned long)c_info[i]->hw.hw_addr;

		ret = qca961x_edma_alloc_tx_rx_queue(c_info[i]);
		if (ret)
			goto failed;

		ret = qca961x_edma_alloc_tx_rx_rings(c_info[i]);
		if (ret)
			goto failed;

		c_info[i]->q_cinfo[i].tx_mask =
			(QCA961X_EDMA_TX_PER_CPU_MASK <<
			(i << QCA961X_EDMA_TX_PER_CPU_MASK_SHIFT));
		c_info[i]->q_cinfo[i].rx_mask =
			(QCA961X_EDMA_RX_PER_CPU_MASK <<
			(i << QCA961X_EDMA_RX_PER_CPU_MASK_SHIFT));
                c_info[i]->q_cinfo[i].tx_start =
			i << QCA961X_EDMA_TX_CPU_START_SHIFT;
		c_info[i]->q_cinfo[i].rx_start =
				i << QCA961X_EDMA_RX_CPU_START_SHIFT;
		c_info[i]->q_cinfo[i].tx_status = 0;
		c_info[i]->q_cinfo[i].rx_status = 0;
		c_info[i]->q_cinfo[i].c_info = c_info[i];

		eth_register(dev[i]);
		/*
		 * Configure EDMA This should
		 * happen Only once.
		 */
		if (!cfg_edma) {
			qca961x_edma_reset(c_info[i]);
			qca961x_edma_configure(c_info[i]);
			qca961x_edma_enable_tx_ctrl(hw[i]);
			qca961x_edma_enable_rx_ctrl(hw[i]);
			cfg_edma = 1;
		}
		/*
		 * Configure descriptor Ring based on eth_unit
		 * 1 Rx/Tx Q is maintained per eth device.
		 */
		qca961x_edma_init_desc(c_info[i],
				edma_cfg->unit);

		if (!qca961x_ess_init_done) {
			qca961x_ess_sw_init(edma_cfg);
			qca961x_ess_init_done = 1;
		}
	}
	return 0;

failed:
	printf("Error in allocating Mem\n");
	for (i = 0; i < QCA961X_EDMA_DEV; i++) {
		if (dev[i]) {
			eth_unregister(dev[i]);
			qca961x_free_mem(dev[i]);
		}
		if (c_info[i]) {
			qca961x_edma_free_rings(c_info[i]);
			qca961x_edma_free_queues(c_info[i]);
			qca961x_free_mem(c_info[i]);
		}
		if (qca961x_edma_dev[i]) {
			qca961x_free_mem(qca961x_edma_dev[i]);
		}
	}
	return -1;
}
