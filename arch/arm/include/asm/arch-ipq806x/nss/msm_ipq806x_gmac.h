/* * Copyright (c) 2012 Qualcomm Atheros, Inc. * */

#ifndef __ASM_ARCH_MSM_NSS_GMAC_H
#define __ASM_ARCH_MSM_NSS_GMAC_H

/* NSS GMAC Base Addresses */
#define NSS_GMAC0_BASE                  0x37000000
#define NSS_GMAC1_BASE                  0x37200000
#define NSS_GMAC2_BASE                  0x37400000
#define NSS_GMAC3_BASE                  0x37600000
#define NSS_GMAC_REG_LEN                0x00200000

/* NSS GMAC Specific defines */
#define NSS_REG_BASE                    0x03000000
#define NSS_REG_LEN                     0x0000FFFF


/* Offsets of NSS config and status registers within NSS_REG_BASE */
/* We start the GMAC numbering from 0 */
#define NSS_CSR_REV                     0x0000
#define NSS_CSR_CFG                     0x0004
#define NSS_ETH_CLK_GATE_CTL            0x0008
#define NSS_ETH_CLK_DIV0                0x000C
#define NSS_ETH_CLK_DIV1                0x0010
#define NSS_ETH_CLK_SRC_CTL             0x0014
#define NSS_ETH_CLK_INV_CTL             0x0018
#define NSS_MACSEC_CTL                  0x0028
#define NSS_QSGMII_CLK_CTL              0x002C
#define NSS_GMAC0_CTL                   0x0030
#define NSS_GMAC1_CTL                   0x0034
#define NSS_GMAC2_CTL                   0x0038
#define NSS_GMAC3_CTL                   0x003C
#define NSS_ETH_CLK_ROOT_STAT           0x0040
#define NSS_QSGMII_STAT                 0x0044
#define NSS_ETH_SPARE_CTL               0x0088
#define NSS_ETH_SPARE_STAT              0x008C


/* Macros to calculate register offsets */
#define NSS_GMACn_CTL(n)                (NSS_GMAC0_CTL +  (n * 4))
#define NSS_ETH_CLK_CTLn(x)             (NSS_ETH_CLK_CTL0 +  (x * 4))


/* NSS_ETH_CLK_GATE_CTL bits */
#define MACSEC3_CORE_CLK                (1 << 30)
#define MACSEC2_CORE_CLK                (1 << 29)
#define MACSEC1_CORE_CLK                (1 << 28)
#define GMAC0_PTP_CLK                   (1 << 16)
#define GMAC0_RGMII_RX_CLK              (1 << 9)
#define GMAC0_RGMII_TX_CLK              (1 << 8)
#define GMAC0_GMII_RX_CLK               (1 << 4)
#define GMAC0_GMII_TX_CLK               (1 << 0)


#define GMAC0_RGMII_TX_CLK_SHIFT                8
#define GMAC0_RGMII_RX_CLK_SHIFT                9
#define GMAC0_GMII_RX_CLK_SHIFT                 4
#define GMAC0_GMII_TX_CLK_SHIFT                 0
#define GMAC0_PTP_CLK_SHIFT                     16

/* Macros to calculate bit offsets in NSS_ETH_CLK_GATE_CTL register
 * MACSEC_CORE_CLK: x = 1,2,3
 * GMII_xx_CLK: x = 0,1,2,3
 * RGMII_xx_CLK: x = 0,1
 * PTP_CLK: x = 0,1,2,3
*/
#define MACSECn_CORE_CLK(x)             (1 << (MACSEC1_CORE_CLK + x))
#define GMACn_GMII_TX_CLK(x)            (1 << (GMAC0_GMII_TX_CLK_SHIFT + x))
#define GMACn_GMII_RX_CLK(x)            (1 << (GMAC0_GMII_RX_CLK_SHIFT + x))
#define GMACn_RGMII_TX_CLK(x)           (1 << (GMAC0_RGMII_TX_CLK_SHIFT + (x * 2)))
#define GMACn_RGMII_RX_CLK(x)           (1 << (GMAC0_RGMII_RX_CLK_SHIFT + (x * 2)))
#define GMACn_PTP_CLK(x)                (1 << (GMAC0_PTP_CLK_SHIFT + x))

/* NSS_ETH_CLK_DIV0 bits ; n = 0,1,2,3 */
#define RGMII_CLK_DIV_1000                      2
#define RGMII_CLK_DIV_100                       10
#define RGMII_CLK_DIV_10                        100
#define SGMII_CLK_DIV_1000                      1
#define SGMII_CLK_DIV_100                       5
#define SGMII_CLK_DIV_10                        50
#define QSGMII_CLK_DIV_1000                     2
#define QSGMII_CLK_DIV_100                      10
#define QSGMII_CLK_DIV_10                       100
#define GMACn_CLK_DIV_SIZE                      0x7F
#define GMACn_CLK_DIV(n,val)                    (val << (n * 8))


/* NSS_ETH_CLK_SRC_CTL bits */
#define GMAC0_GMII_CLK_RGMII                    (1 << 0)
#define GMAC1_GMII_CLK_RGMII                    (1 << 1)

/* Macros to calculate bit offsets in NSS_ETH_CLK_CTL3 register */
#define GMACn_GMII_CLK_RGMII(x)                 (1 << x)

/* NSS_QSGMII_CLK_CTL bits */
#define GMAC0_TX_CLK_HALT                       (1 << 7)
#define GMAC0_RX_CLK_HALT                       (1 << 8)
#define GMAC1_TX_CLK_HALT                       (1 << 9)
#define GMAC1_RX_CLK_HALT                       (1 << 10)
#define GMAC2_TX_CLK_HALT                       (1 << 11)
#define GMAC2_RX_CLK_HALT                       (1 << 12)
#define GMAC3_TX_CLK_HALT                       (1 << 13)
#define GMAC3_RX_CLK_HALT                       (1 << 14)

#define GMAC0_QSGMII_TX_CLK_SHIFT               7
#define GMAC0_QSGMII_RX_CLK_SHIFT               8

/* Macros to calculate bit offsets in NSS_QSGMII_CLK_CTL register */
#define GMACn_QSGMII_TX_CLK(n)          (1 << (GMAC0_QSGMII_TX_CLK_SHIFT + (n * 2)))
#define GMACn_QSGMII_RX_CLK(n)          (1 << (GMAC0_QSGMII_RX_CLK_SHIFT + (n * 2)))

/* NSS_GMACn_CTL bits */
#define GMAC_IFG_CTL(x)                         (x)
#define GMAC_IFG_LIMIT(x)                       (x << 8)
#define GMAC_PHY_RGMII                          (1 << 16)
#define GMAC_PHY_QSGMII                         (0 << 16)
#define GMAC_FLOW_CTL                           (1 << 18)
#define GMAC_CSYS_REQ                           (1 << 19)
#define GMAC_PTP_TRIG                           (1 << 20)

/* GMAC min Inter Frame Gap values */
#define GMAC_IFG                                12
#define GMAC_IFG_MIN_1000                       10
#define GMAC_IFG_MIN_HALF_DUPLEX                8

/*
 * GMAC min Inter Frame Gap Limits.
 * In full duplex mode set to same value as IFG
*/
#define GMAC_IFG_LIMIT_HALF                     12

/* QSGMII Specific defines */
#define QSGMII_REG_BASE                         0x1bb00000
#define QSGMII_REG_LEN                          0x0000FFFF

/* QSGMII Register offsets */
#define PCS_QSGMII_CTL                          0x020
#define PCS_QSGMII_SGMII_MODE                   0x064
#define PCS_ALL_CH_CTL                          0x080
#define QSGMII_PHY_MODE_CTL                     0x128

/* Bit definitions for PCS_QSGMII_SGMII_MODE */
#define PCS_QSGMII_MODE_SGMII			(0x0 << 0)
#define PCS_QSGMII_MODE_QSGMII			(0x1 << 0)

/* Bit definitions for QSGMII_PHY_MODE_CTL */
#define QSGMII_PHY_MODE_SGMII			(0x0 << 0)
#define QSGMII_PHY_MODE_QSGMII			(0x1 << 0)

/* MDIO addresses for individual PHYs */
#define GMAC0_MDIO_ID				4
#define GMAC1_MDIO_ID				6
#define GMAC2_MDIO_ID				(-1) /* Does not use MDIO */
#define GMAC3_MDIO_ID				(-1)

/* Interface between GMAC and PHY */
#define GMAC_INTF_RGMII				0
#define GMAC_INTF_SGMII				1
#define GMAC_INTF_QSGMII			2

/* GMAC phy interface selection */
//#ifdef CONFIG_ARCH_IPQ806X
#if defined(CONFIG_IPQ_GMAC_PHY_PROFILE_2R_2S)
#define NSS_GMAC0_MII_MODE              GMAC_INTF_RGMII
#define NSS_GMAC1_MII_MODE              GMAC_INTF_RGMII
#define NSS_GMAC2_MII_MODE              GMAC_INTF_SGMII
#define NSS_GMAC3_MII_MODE              GMAC_INTF_SGMII
#elif defined (CONFIG_IPQ_GMAC_PHY_PROFILE_1R_3S)
#define NSS_GMAC0_MII_MODE              GMAC_INTF_RGMII
#define NSS_GMAC1_MII_MODE              GMAC_INTF_SGMII
#define NSS_GMAC2_MII_MODE              GMAC_INTF_SGMII
#define NSS_GMAC3_MII_MODE              GMAC_INTF_SGMII
#elif defined (CONFIG_IPQ_GMAC_PHY_PROFILE_QS)
#define NSS_GMAC0_MII_MODE              GMAC_INTF_QSGMII
#define NSS_GMAC1_MII_MODE              GMAC_INTF_QSGMII
#define NSS_GMAC2_MII_MODE              GMAC_INTF_QSGMII
#define NSS_GMAC3_MII_MODE              GMAC_INTF_QSGMII
#else
#define NSS_GMAC0_MII_MODE              GMAC_INTF_RGMII
#define NSS_GMAC1_MII_MODE              GMAC_INTF_RGMII
#define NSS_GMAC2_MII_MODE              GMAC_INTF_SGMII
#define NSS_GMAC3_MII_MODE              GMAC_INTF_SGMII
#endif
//#endif /* CONFIG_ARCH_IPQ806X */

struct nss_gmac_platform_data {
	uint32_t phy_mdio_addr;                 /* MDIO address of the connected PHY */
	uint32_t poll_required;                 /* [0/1] Link status poll? */
	uint32_t rgmii_delay;
};

/* Macros to calculate register offsets */

#define NSS_MAX_GMACS                           4

#endif /*__ASM_ARCH_MSM_NSS_GMAC_H */




