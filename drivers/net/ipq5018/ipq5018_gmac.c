/*
* Copyright (c) 2019 The Linux Foundation. All rights reserved.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 and
* only version 2 as published by the Free Software Foundation.
*
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
#include <asm/global_data.h>
#include <fdtdec.h>
#include "ipq_phy.h"
#include <asm/arch-ipq5018/ipq5018_gmac.h>

DECLARE_GLOBAL_DATA_PTR;

#define ipq_info        printf
#define ipq_dbg         printf
#define DESC_SIZE       (sizeof(ipq_gmac_desc_t))
#define DESC_FLUSH_SIZE (((DESC_SIZE + (CONFIG_SYS_CACHELINE_SIZE - 1)) \
                        / CONFIG_SYS_CACHELINE_SIZE) * \
                        (CONFIG_SYS_CACHELINE_SIZE))

static struct ipq_eth_dev *ipq_gmac_macs[IPQ5018_GMAC_PORT];

uchar ipq_def_enetaddr[6] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55};
phy_info_t *phy_info[IPQ5018_PHY_MAX] = {0};

extern int ipq_mdio_read(int mii_id, int regnum, ushort *data);
extern int ipq_qca8033_phy_init(struct phy_ops **ops, u32 phy_id);
extern void ipq_qca8075_phy_map_ops(struct phy_ops **ops);
extern int ipq_qca8081_phy_init(struct phy_ops **ops, u32 phy_id);
extern int ipq_sw_mdio_init(const char *);

static int ipq_eth_wr_macaddr(struct eth_device *dev)
{
	struct ipq_eth_dev *priv = dev->priv;
	struct eth_mac_regs *mac_p = (struct eth_mac_regs *)priv->mac_regs_p;
	u32 macid_lo, macid_hi;
	u8 *mac_id = &dev->enetaddr[0];

	macid_lo = mac_id[0] + (mac_id[1] << 8) +
		   (mac_id[2] << 16) + (mac_id[3] << 24);
	macid_hi = mac_id[4] + (mac_id[5] << 8);

	writel(macid_hi, &mac_p->macaddr0hi);
	writel(macid_lo, &mac_p->macaddr0lo);

	return 0;
}

static void ipq_mac_reset(struct eth_device *dev)
{
	struct ipq_eth_dev *priv = dev->priv;
	struct eth_dma_regs *dma_reg = (struct eth_dma_regs *)priv->dma_regs_p;
	u32 val;

	writel(DMAMAC_SRST, &dma_reg->busmode);
	do {
		udelay(10);
		val = readl(&dma_reg->busmode);
	} while (val & DMAMAC_SRST);

}

static void ipq_eth_mac_cfg(struct eth_device *dev)
{
	struct ipq_eth_dev *priv = dev->priv;
	struct eth_mac_regs *mac_reg = (struct eth_mac_regs *)priv->mac_regs_p;

	uint ipq_mac_cfg = 0;
	uint ipq_mac_framefilter = 0;

	ipq_mac_framefilter = PROMISCUOUS_MODE_ON;

	ipq_mac_cfg |= (FRAME_BURST_ENABLE | JUMBO_FRAME_ENABLE |
				TX_ENABLE | RX_ENABLE | FULL_DUPLEX_ENABLE);

	writel(ipq_mac_cfg, &mac_reg->conf);
	writel(ipq_mac_framefilter, &mac_reg->framefilt);
}

static void ipq_eth_dma_cfg(struct eth_device *dev)
{
	struct ipq_eth_dev *priv = dev->priv;
	struct eth_dma_regs *dma_reg = (struct eth_dma_regs *)priv->dma_regs_p;
	uint ipq_dma_bus_mode;
	uint ipq_dma_op_mode;

	ipq_dma_op_mode = DmaStoreAndForward | DmaRxThreshCtrl128 |
				DmaTxSecondFrame;
	ipq_dma_bus_mode = DmaFixedBurstEnable | DmaBurstLength16 |
				DmaDescriptorSkip0 | DmaDescriptor8Words |
				DmaArbitPr;

	writel(ipq_dma_bus_mode, &dma_reg->busmode);
	writel(ipq_dma_op_mode, &dma_reg->opmode);
}

static void ipq_eth_flw_cntl_cfg(struct eth_device *dev)
{
	struct ipq_eth_dev *priv = dev->priv;
	struct eth_mac_regs *mac_reg = (struct eth_mac_regs *)priv->mac_regs_p;
	struct eth_dma_regs *dma_reg = (struct eth_dma_regs *)priv->dma_regs_p;
	uint ipq_dma_flw_cntl;
	uint ipq_mac_flw_cntl;

	ipq_dma_flw_cntl = DmaRxFlowCtrlAct3K | DmaRxFlowCtrlDeact4K |
				DmaEnHwFlowCtrl;
	ipq_mac_flw_cntl = GmacRxFlowControl | GmacTxFlowControl | 0xFFFF0000;

	setbits_le32(&dma_reg->opmode, ipq_dma_flw_cntl);
	setbits_le32(&mac_reg->flowcontrol, ipq_mac_flw_cntl);
}

static int ipq_gmac_alloc_fifo(int ndesc, ipq_gmac_desc_t **fifo)
{
	int i;
	void *addr;

	addr = memalign((CONFIG_SYS_CACHELINE_SIZE),
			(ndesc * DESC_FLUSH_SIZE));

	for (i = 0; i < ndesc; i++) {
		fifo[i] = (ipq_gmac_desc_t *)((unsigned long)addr +
			  (i * DESC_FLUSH_SIZE));
		if (fifo[i] == NULL) {
			ipq_info("Can't allocate desc fifos\n");
			return -1;
		}
	}
	return 0;
}

static int ipq_gmac_rx_desc_setup(struct ipq_eth_dev  *priv)
{
	struct eth_dma_regs *dma_reg = (struct eth_dma_regs *)priv->dma_regs_p;
	ipq_gmac_desc_t *rxdesc;
	int i;

	for (i = 0; i < NO_OF_RX_DESC; i++) {
		rxdesc = priv->desc_rx[i];
		rxdesc->length |= ((ETH_MAX_FRAME_LEN << DescSize1Shift) &
					DescSize1Mask);
		rxdesc->buffer1 = virt_to_phys(net_rx_packets[i]);
		rxdesc->data1 = (unsigned long)priv->desc_rx[(i + 1) %
				NO_OF_RX_DESC];

		rxdesc->extstatus = 0;
		rxdesc->reserved1 = 0;
		rxdesc->timestamplow = 0;
		rxdesc->timestamphigh = 0;
		rxdesc->status = DescOwnByDma;


		flush_dcache_range((unsigned long)rxdesc,
			(unsigned long)rxdesc + DESC_SIZE);

	}
	/* Assign Descriptor base address to dmadesclist addr reg */
	writel((uint)priv->desc_rx[0], &dma_reg->rxdesclistaddr);

	return 0;
}

static int ipq_gmac_tx_rx_desc_ring(struct ipq_eth_dev  *priv)
{
	int i;
	ipq_gmac_desc_t *desc;

	if (ipq_gmac_alloc_fifo(NO_OF_TX_DESC, priv->desc_tx))
		return -1;

	for (i = 0; i < NO_OF_TX_DESC; i++) {
		desc = priv->desc_tx[i];
		memset(desc, 0, DESC_SIZE);

		desc->status =
		(i == (NO_OF_TX_DESC - 1)) ? TxDescEndOfRing : 0;

		desc->status |= TxDescChain;

		desc->data1 = (unsigned long)priv->desc_tx[(i + 1) %
				NO_OF_TX_DESC ];

		flush_dcache_range((unsigned long)desc,
			(unsigned long)desc + DESC_SIZE);

	}

	if (ipq_gmac_alloc_fifo(NO_OF_RX_DESC, priv->desc_rx))
		return -1;

	for (i = 0; i < NO_OF_RX_DESC; i++) {
		desc = priv->desc_rx[i];
		memset(desc, 0, DESC_SIZE);
		desc->length =
		(i == (NO_OF_RX_DESC - 1)) ? RxDescEndOfRing : 0;
		desc->length |= RxDescChain;

		desc->data1 = (unsigned long)priv->desc_rx[(i + 1) %
				NO_OF_RX_DESC];

		flush_dcache_range((unsigned long)desc,
			(unsigned long)desc + DESC_SIZE);

	}

	priv->next_tx = 0;
	priv->next_rx = 0;

	return 0;
}

static inline void ipq_gmac_give_to_dma(ipq_gmac_desc_t *fr)
{
	fr->status |= DescOwnByDma;
}

static inline u32 ipq_gmac_owned_by_dma(ipq_gmac_desc_t *fr)
{
	return (fr->status & DescOwnByDma);
}

static inline u32 ipq_gmac_is_desc_empty(ipq_gmac_desc_t *fr)
{
	return ((fr->length & DescSize1Mask) == 0);
}
#if 0
static int ipq_eth_update(struct eth_device *dev, bd_t *this)
{
	struct ipq_eth_dev *priv = dev->priv;
	struct phy_ops *phy_get_ops = priv->ops[priv->mac_unit];
	uint phy_status;
	uint cur_speed;
	uint cur_duplex;

	phy_status = phy_get_ops->phy_get_link_status(priv->mac_unit, priv->phy_address);
	phy_get_ops->phy_get_speed(priv->mac_unit, priv->phy_address, &curr_speed);
	phy_get_ops->phy_get_duplex(priv->mac_unit, priv->phy_address, &duplex);

		if (cur_speed != priv->speed || cur_duplex != priv->duplex) {
			ipq_info("Link %x status changed\n", priv->mac_unit);
			if (priv->duplex)
				ipq_info("Full duplex link\n");
			else
				ipq_info("Half duplex link\n");

			ipq_info("Link %x up, Phy_status = %x\n",
					priv->mac_unit, phy_status);

			switch (cur_speed) {
			case SPEED_1000M:
				ipq_info("Port:%d speed 1000Mbps\n",
					priv->mac_unit);
				priv->mac_ps = GMII_PORT_SELECT;
				break;

			case SPEED_100M:
				ipq_info("Port:%d speed 100Mbps\n",
					priv->mac_unit);
				priv->mac_ps = MII_PORT_SELECT;
				break;

			case SPEED_10M:
				ipq_info("Port:%d speed 10Mbps\n",
					priv->mac_unit);
				priv->mac_ps = MII_PORT_SELECT;
				break;

			default:
				ipq_info("Port speed unknown\n");
				return -1;
			}

			priv->speed = cur_speed;
			priv->duplex = cur_duplex;

		}

	return 0;
}
#endif
int ipq_eth_init(struct eth_device *dev, bd_t *this)
{
	struct ipq_eth_dev *priv = dev->priv;
	struct eth_dma_regs *dma_reg = (struct eth_dma_regs *)priv->dma_regs_p;
	u32 data;
#if 0
	ipq_phy_link_status(dev);
#endif
	priv->next_rx = 0;
	priv->next_tx = 0;

	ipq_mac_reset(dev);
	ipq_eth_wr_macaddr(dev);

	/* DMA, MAC configuration for Synopsys GMAC */
	ipq_eth_dma_cfg(dev);
	ipq_eth_mac_cfg(dev);
	ipq_eth_flw_cntl_cfg(dev);

	/* clear all pending interrupts if any */
	data = readl(&dma_reg->status);
	writel(data, &dma_reg->status);

	/* Setup Rx fifos and assign base address to */
	ipq_gmac_rx_desc_setup(priv);

	writel((uint)priv->desc_tx[0], &dma_reg->txdesclistaddr);
	setbits_le32(&dma_reg->opmode, (RXSTART));
	setbits_le32(&dma_reg->opmode, (TXSTART));

	return 1;
}

static int ipq_eth_send(struct eth_device *dev, void *packet, int length)
{
	struct ipq_eth_dev *priv = dev->priv;
	struct eth_dma_regs *dma_p = (struct eth_dma_regs *)priv->dma_regs_p;
	ipq_gmac_desc_t *txdesc = priv->desc_tx[priv->next_tx];
	int i;

	invalidate_dcache_range((unsigned long)txdesc,
		       (unsigned long)txdesc + DESC_FLUSH_SIZE);

	/* Check if the dma descriptor is still owned by DMA */
	if (ipq_gmac_owned_by_dma(txdesc)) {
		ipq_info("BUG: Tx descriptor is owned by DMA %p\n", txdesc);
		return NETDEV_TX_BUSY;
	}

	txdesc->length |= ((length <<DescSize1Shift) & DescSize1Mask);
	txdesc->status |= (DescTxFirst | DescTxLast | DescTxIntEnable);
	txdesc->buffer1 = virt_to_phys(packet);
	ipq_gmac_give_to_dma(txdesc);

	flush_dcache_range((unsigned long)txdesc,
			(unsigned long)txdesc + DESC_SIZE);

	flush_dcache_range((unsigned long)(txdesc->buffer1),
		(unsigned long)(txdesc->buffer1) + PKTSIZE_ALIGN);

	/* Start the transmission */
	writel(POLL_DATA, &dma_p->txpolldemand);

	for (i = 0; i < MAX_WAIT; i++) {

		udelay(10);

		invalidate_dcache_range((unsigned long)txdesc,
		(unsigned long)txdesc + DESC_FLUSH_SIZE);

		if (!ipq_gmac_owned_by_dma(txdesc))
			break;
	}

	if (i == MAX_WAIT) {
		ipq_info("Tx Timed out\n");
	}

	/* reset the descriptors */
	txdesc->status = (priv->next_tx == (NO_OF_TX_DESC - 1)) ?
	TxDescEndOfRing : 0;
	txdesc->status |= TxDescChain;
	txdesc->length = 0;
	txdesc->buffer1 = 0;

	priv->next_tx = (priv->next_tx + 1) % NO_OF_TX_DESC;

	txdesc->data1 = (unsigned long)priv->desc_tx[priv->next_tx];


	flush_dcache_range((unsigned long)txdesc,
			(unsigned long)txdesc + DESC_SIZE);

	return 0;
}

static int ipq_eth_recv(struct eth_device *dev)
{
	struct ipq_eth_dev *priv = dev->priv;
	struct eth_dma_regs *dma_p = (struct eth_dma_regs *)priv->dma_regs_p;
	int length = 0;
	ipq_gmac_desc_t *rxdesc = priv->desc_rx[priv->next_rx];
	uint status;

	invalidate_dcache_range((unsigned long)(priv->desc_rx[0]),
			(unsigned long)(priv->desc_rx[NO_OF_RX_DESC - 1]) +
			DESC_FLUSH_SIZE);

	for (rxdesc = priv->desc_rx[priv->next_rx];
		!ipq_gmac_owned_by_dma(rxdesc);
		rxdesc = priv->desc_rx[priv->next_rx]) {

		status = rxdesc->status;
		length = ((status & DescFrameLengthMask) >>
				DescFrameLengthShift);

		invalidate_dcache_range(
			(unsigned long)(net_rx_packets[priv->next_rx]),
			(unsigned long)(net_rx_packets[priv->next_rx]) +
			PKTSIZE_ALIGN);
		net_process_received_packet(net_rx_packets[priv->next_rx], length - 4);

		rxdesc->length = ((ETH_MAX_FRAME_LEN << DescSize1Shift) &
				   DescSize1Mask);

		rxdesc->length |= (priv->next_rx == (NO_OF_RX_DESC - 1)) ?
					RxDescEndOfRing : 0;
		rxdesc->length |= RxDescChain;

		rxdesc->buffer1 = virt_to_phys(net_rx_packets[priv->next_rx]);

		priv->next_rx = (priv->next_rx + 1) % NO_OF_RX_DESC;

		rxdesc->data1 = (unsigned long)priv->desc_rx[priv->next_rx];

		rxdesc->extstatus = 0;
		rxdesc->reserved1 = 0;
		rxdesc->timestamplow = 0;
		rxdesc->timestamphigh = 0;
		rxdesc->status = DescOwnByDma;

		flush_dcache_range((unsigned long)rxdesc,
			(unsigned long)rxdesc + DESC_SIZE);

		writel(POLL_DATA, &dma_p->rxpolldemand);
	}

	return length;
}

static void ipq_eth_halt(struct eth_device *dev)
{
	if (dev->state != ETH_STATE_ACTIVE)
		return;
	/* reset the mac */
	ipq_mac_reset(dev);
}

int ipq_gmac_init(ipq_gmac_board_cfg_t *gmac_cfg)
{
	struct eth_device *dev[CONFIG_IPQ_NO_MACS];
	uchar enet_addr[CONFIG_IPQ_NO_MACS * 6];
	char ethaddr[32] = "ethaddr";
	int i, phy_id;
	uint32_t phy_chip_id, phy_chip_id1, phy_chip_id2;
	int ret;
	memset(enet_addr, 0, sizeof(enet_addr));

	/* Getting the MAC address from ART partition */
	ret = get_eth_mac_address(enet_addr, CONFIG_IPQ_NO_MACS);

	for (i = 0; gmac_cfg_is_valid(gmac_cfg); gmac_cfg++, i++) {

		dev[i] = malloc(sizeof(struct eth_device));
		if (dev[i] == NULL)
			goto failed;

		ipq_gmac_macs[i] = malloc(sizeof(struct ipq_eth_dev));
		if (ipq_gmac_macs[i] == NULL)
			goto failed;

		memset(dev[i], 0, sizeof(struct eth_device));
		memset(ipq_gmac_macs[i], 0, sizeof(struct ipq_eth_dev));

		dev[i]->iobase = gmac_cfg->base;
		dev[i]->init = ipq_eth_init;
		dev[i]->halt = ipq_eth_halt;
		dev[i]->recv = ipq_eth_recv;
		dev[i]->send = ipq_eth_send;
		dev[i]->write_hwaddr = ipq_eth_wr_macaddr;
		dev[i]->priv = (void *) ipq_gmac_macs[i];

		snprintf(dev[i]->name, sizeof(dev[i]->name), "eth%d", i);

		/*
		 * Setting the Default MAC address if the MAC read from ART partition
		 * is invalid.
		 */
		if ((ret < 0) ||
			(!is_valid_ethaddr(&enet_addr[i * 6]))) {
				memcpy(&dev[i]->enetaddr[0], ipq_def_enetaddr, 6);
				dev[i]->enetaddr[5] = dev[i]->enetaddr[5] +  i;
		} else {
			memcpy(&dev[i]->enetaddr[0], &enet_addr[i * 6], 6);
		}

		ipq_info("MAC%x addr:%x:%x:%x:%x:%x:%x\n",
			gmac_cfg->unit, dev[i]->enetaddr[0],
			dev[i]->enetaddr[1],
			dev[i]->enetaddr[2],
			dev[i]->enetaddr[3],
			dev[i]->enetaddr[4],
			dev[i]->enetaddr[5]);

		snprintf(ethaddr, sizeof(ethaddr), "eth%daddr", (i + 1));

		ipq_gmac_macs[i]->dev = dev[i];
		ipq_gmac_macs[i]->mac_unit = gmac_cfg->unit;
		ipq_gmac_macs[i]->mac_regs_p =
			(struct eth_mac_regs *)(gmac_cfg->base);
		ipq_gmac_macs[i]->dma_regs_p =
			(struct eth_dma_regs *)(gmac_cfg->base + DW_DMA_BASE_OFFSET);
#if 0
		ipq_gmac_macs[i]->interface = gmac_cfg->phy;
		ipq_gmac_macs[i]->phy_address = gmac_cfg->phy_addr.addr;
		ipq_gmac_macs[i]->no_of_phys = gmac_cfg->phy_addr.count;
#endif
		ipq_gmac_macs[i]->phy_address = gmac_cfg->phy_addr;
		ipq_gmac_macs[i]->gmac_board_cfg = gmac_cfg;

#if 0
		if (get_params.gmac_port == gmac_cfg->unit) {
			ipq_gmac_macs[i]->forced_params = &get_params;
		}
#endif
		/* tx/rx Descriptor initialization */
		if (ipq_gmac_tx_rx_desc_ring(dev[i]->priv) == -1)
			goto failed;
#if 0
		if(ipq_gmac_macs[i]->mac_unit == 0)
			ipq_gmac_macs[i]->speed = SPEED_10M;
		else
			ipq_gmac_macs[i]->speed = SPEED_1000M;
#endif

		strlcpy((char *)ipq_gmac_macs[i]->phy_name, gmac_cfg->phy_name,
					sizeof(ipq_gmac_macs[i]->phy_name));

if (0){
		if (ipq_sw_mdio_init(gmac_cfg->phy_name))
			goto failed;

		for (phy_id =  0; phy_id < 2; phy_id++) {
#if 0
			if (phy_node >= 0) {
				phy_addr = phy_info[phy_id]->phy_address;
			} else {
				if (phy_id == port_8033)
					phy_addr = QCA8033_PHY_ADDR;
				else if (phy_id == aquantia_port)
					phy_addr = AQU_PHY_ADDR;
				else
					phy_addr = phy_id;
			}
#endif
			phy_chip_id1 = ipq_mdio_read(
				ipq_gmac_macs[i]->phy_address, QCA_PHY_ID1, NULL);
			phy_chip_id2 = ipq_mdio_read(
				ipq_gmac_macs[i]->phy_address, QCA_PHY_ID2, NULL);
			phy_chip_id = (phy_chip_id1 << 16) | phy_chip_id2;

			switch(phy_chip_id) {
				case QCA8033_PHY:
					ipq_qca8033_phy_init(
					&ipq_gmac_macs[i]->ops[phy_id],
						ipq_gmac_macs[i]->phy_address);
					break;
				case QCA8081_PHY:
				case QCA8081_1_1_PHY:
					ipq_qca8081_phy_init(
					&ipq_gmac_macs[i]->ops[phy_id],
						ipq_gmac_macs[i]->phy_address);
					break;
				default:
					ipq_qca8075_phy_map_ops(&ipq_gmac_macs[i]->ops[phy_id]);
					break;
			}
		}
}
	eth_register(dev[i]);
	}

	return 0;

failed:
	for (i = 0; i < IPQ5018_GMAC_PORT; i++) {
		if (dev[i]) {
			eth_unregister(dev[i]);
			free(dev[i]);
		}
		if (ipq_gmac_macs[i])
			free(ipq_gmac_macs[i]);
	}

	return -ENOMEM;
}

void ipq_gmac_common_init(ipq_gmac_board_cfg_t *gmac_cfg)
{
	return;
}

