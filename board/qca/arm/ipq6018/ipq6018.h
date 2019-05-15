/*
 * Copyright (c) 2016-2018, The Linux Foundation. All rights reserved.
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

#ifndef _IPQ6018_CDP_H_
#define _IPQ6018_CDP_H_

#include <configs/ipq6018.h>
#include <asm/u-boot.h>
#include <asm/arch-qca-common/qca_common.h>

#define CLK_TOGGLE_ENABLE 0x1

/*
 * GCC_NSS_RCGR Registers
 */
#define GCC_NSS_PORT1_RX_CFG_RCGR		0x01868024
#define GCC_NSS_PORT1_RX_CMD_RCGR		0x01868020

#define GCC_NSS_PORT1_TX_CFG_RCGR		0x0186802C
#define GCC_NSS_PORT1_TX_CMD_RCGR		0x01868028

#define GCC_NSS_PORT2_RX_CFG_RCGR		0x01868034
#define GCC_NSS_PORT2_RX_CMD_RCGR		0x01868030

#define GCC_NSS_PORT2_TX_CFG_RCGR		0x0186803C
#define GCC_NSS_PORT2_TX_CMD_RCGR		0x01868038

#define GCC_NSS_PORT3_RX_CFG_RCGR		0x01868044
#define GCC_NSS_PORT3_RX_CMD_RCGR		0x01868040

#define GCC_NSS_PORT3_TX_CFG_RCGR		0x0186804C
#define GCC_NSS_PORT3_TX_CMD_RCGR		0x01868048

#define GCC_NSS_PORT4_RX_CFG_RCGR		0x01868054
#define GCC_NSS_PORT4_RX_CMD_RCGR		0x01868050

#define GCC_NSS_PORT4_TX_CFG_RCGR		0x0186805C
#define GCC_NSS_PORT4_TX_CMD_RCGR		0x01868058

#define GCC_NSS_PORT5_RX_CFG_RCGR		0x01868064
#define GCC_NSS_PORT5_RX_CMD_RCGR		0x01868060

#define GCC_NSS_PORT5_TX_CFG_RCGR		0x0186806C
#define GCC_NSS_PORT5_TX_CMD_RCGR		0x01868068

#define GCC_NSS_PPE_CFG_RCGR			0x01868084
#define GCC_NSS_PPE_CMD_RCGR			0x01868080

#define GCC_NSS_CRYPTO_CMD_RCGR			0x01868144
#define GCC_NSS_CRYPTO_CFG_RCGR			0x01868148

#define GCC_SNOC_NSSNOC_BFDCD_CMD_RCGR		0x01876054
#define GCC_SNOC_NSSNOC_BFDCD_CFG_RCGR		0x01876058

#define GCC_QDSS_AT_CMD_RCGR			0x0182900C
#define GCC_QDSS_AT_CFG_RCGR			0x01829010

#define GCC_NSS_CE_CMD_RCGR			0x01868098
#define GCC_NSS_CE_CFG_RCGR			0x0186809C

#define GCC_PCNOC_BFDCD_CMD_RCGR		0x01827000
#define GCC_PCNOC_BFDCD_CFG_RCGR		0x01827004

/*
 * GCC_NSS_CBCR Registers
 */
#define GCC_NSS_PORT1_RX_CBCR		0x01868240
#define GCC_NSS_PORT1_TX_CBCR		0x01868244

#define GCC_NSS_PORT2_RX_CBCR		0x01868248
#define GCC_NSS_PORT2_TX_CBCR		0x0186824C

#define GCC_NSS_PORT3_RX_CBCR		0x01868250
#define GCC_NSS_PORT3_TX_CBCR		0x01868254

#define GCC_NSS_PORT4_RX_CBCR		0x01868258
#define GCC_NSS_PORT4_TX_CBCR		0x0186825C

#define GCC_NSS_PORT5_RX_CBCR		0x01868260
#define GCC_NSS_PORT5_TX_CBCR		0x01868264

#define GCC_NSS_PPE_CBCR		0x01868190
#define GCC_NSS_PPE_CFG_CBCR		0x01868194
#define GCC_NSS_EDMA_CBCR		0x01868198
#define GCC_NSS_EDMA_CFG_CBCR		0x0186819C
#define GCC_NSS_PTP_REF_CBCR		0x0186816C
#define GCC_NSSNOC_PPE_CBCR		0x01868300
#define GCC_NSSNOC_PPE_CFG_CBCR		0x01868304

#define GCC_NSS_CRYPTO_CBCR		0x01868164

#define GCC_NSSNOC_SNOC_CBCR		0x01868188
#define GCC_SNOC_NSSNOC_CBCR		0x01826070
#define GCC_NSS_NOC_CBCR		0x01868168

#define GCC_NSSNOC_ATB_CBCR		0x0186818C

#define GCC_NSSNOC_QOSGEN_REF_CBCR	0x01868180
#define GCC_NSSNOC_TIMEOUT_REF_CBCR	0x01868184

#define GCC_NSS_CE_AXI_CBCR		0x01868170
#define GCC_NSS_CE_APB_CBCR		0x01868174
#define GCC_NSSNOC_CE_AXI_CBCR		0x01868308
#define GCC_NSSNOC_CE_APB_CBCR		0x0186830C
#define GCC_NSSNOC_UBI0_AHB_CBCR	0x01868270

#define GCC_NSS_CFG_CBCR		0x01868160

/*
 * GCC-SDCC Registers
 */
#define GCC_SDCC1_MISC		0x1842020
#define GCC_SDCC1_APPS_CBCR	0x1842018
#define GCC_SDCC1_APPS_CFG_RCGR	0x1842008
#define GCC_SDCC1_APPS_CMD_RCGR	0x1842004
#define GCC_SDCC1_APPS_M	0x184200C
#define GCC_SDCC1_APPS_N	0x1842010
#define GCC_SDCC1_APPS_D	0x1842014
#define GCC_BLSP1_UART1_APPS_CBCR       0x0180203c
#define GCC_SDCC1_BCR		0x01842000
#define GCC_SDCC1_AHB_CBCR	0x0184201C

#define GCC_BLSP1_UART2_APPS_CFG_RCGR	0x01803038
#define GCC_BLSP1_UART2_APPS_M		0x0180303C
#define GCC_BLSP1_UART2_APPS_N		0x01803040
#define GCC_BLSP1_UART2_APPS_D		0x01803044
#define GCC_BLSP1_UART2_APPS_CMD_RCGR	0x01803034
#define GCC_BLSP1_UART2_APPS_CBCR	0x0180302C

#define GCC_UART_CFG_RCGR_MODE_MASK	0x3000
#define GCC_UART_CFG_RCGR_SRCSEL_MASK	0x0700
#define GCC_UART_CFG_RCGR_SRCDIV_MASK	0x001F

#define GCC_UART_CFG_RCGR_MODE_SHIFT	12
#define GCC_UART_CFG_RCGR_SRCSEL_SHIFT	8
#define GCC_UART_CFG_RCGR_SRCDIV_SHIFT	0

#define UART2_RCGR_SRC_SEL	0x1
#define UART2_RCGR_SRC_DIV	0x0
#define UART2_RCGR_MODE		0x2
#define UART2_CMD_RCGR_UPDATE	0x1
#define UART2_CBCR_CLK_ENABLE	0x1

#define NOT_2D(two_d)		(~two_d)
#define NOT_N_MINUS_M(n,m)	(~(n - m))
#define CLOCK_UPDATE_TIMEOUT_US	1000

#define CLOCK_UPDATE_TIMEOUT_US	1000
#define KERNEL_AUTH_CMD		0x1E
#define SCM_CMD_SEC_AUTH	0x1F

/* USB Registers */
#define GCC_USB0_GDSCR			0x183E078
#define GCC_SYS_NOC_USB0_AXI_CBCR	0x1826040
#define GCC_USB0_MASTER_CFG_RCGR	0x183E010
#define GCC_USB0_MASTER_CMD_RCGR	0x183E00C
#define GCC_USB0_MASTER_CBCR		0x183E000
#define GCC_USB0_SLEEP_CBCR		0x183E004
#define GCC_USB0_MOCK_UTMI_CFG_RCGR	0x0183E024
#define GCC_USB0_MOCK_UTMI_M		0x0183E028
#define GCC_USB0_MOCK_UTMI_N		0x0183E02C
#define GCC_USB0_MOCK_UTMI_D		0x0183E030
#define GCC_USB0_MOCK_UTMI_CMD_RCGR	0x183E020
#define GCC_USB0_MOCK_UTMI_CBCR		0x183E008
#define GCC_USB0_PHY_CFG_AHB_CBCR	0x183E080
#define GCC_USB0_AUX_CBCR		0x183E044
#define GCC_USB0_PIPE_CBCR		0x183E040
#define GCC_USB_SS_REF_CLK_EN		0x0183E07C
#define GCC_USB_0_BOOT_CLOCK_CTL	0x1840000
#define GCC_USB0_BCR			0x183E070
#define GCC_QUSB2_0_PHY_BCR		0x184103C
#define GCC_USB0_PHY_BCR		0x183E034
#define GCC_USB3PHY_0_PHY_BCR		0x183E03C
#define USB30_1_GENERAL_CFG		0x8AF8808
#define USB30_1_GUCTL			0x8A0C12C
#define USB30_PHY_1_QUSB2PHY_BASE	0x79000

#define GCC_USB1_GDSCR			0x183F078
#define GCC_SNOC_BUS_TIMEOUT2_AHB_CBCR	0x01847014
#define GCC_USB1_MASTER_CBCR		0x183F000
#define GCC_USB1_SLEEP_CBCR		0x183F004
#define GCC_USB1_MOCK_UTMI_CFG_RCGR	0x183F024
#define GCC_USB1_MOCK_UTMI_M		0x183F028
#define GCC_USB1_MOCK_UTMI_N		0x0183F02C
#define GCC_USB1_MOCK_UTMI_D		0x0183F030
#define GCC_USB1_MOCK_UTMI_CMD_RCGR	0x183F020
#define GCC_USB1_MOCK_UTMI_CBCR		0x183F008
#define GCC_USB1_PHY_CFG_AHB_CBCR	0x183F080
#define GCC_USB_1_BOOT_CLOCK_CTL	0x1841000
#define GCC_USB1_BCR			0x183F070
#define GCC_QUSB2_1_PHY_BCR		0x1841040
#define USB30_PHY_2_QUSB2PHY_BASE	0x59000

#define USB30_PHY_1_USB3PHY_AHB2PHY_BASE	0x78000
#define USB30_PHY_2_USB2PHY_AHB2PHY_BASE	0x58000

#define USB3_PHY_POWER_DOWN_CONTROL	0x804
#define QSERDES_COM_SYSCLK_EN_SEL	0xac
#define QSERDES_COM_BIAS_EN_CLKBUFLR_EN	0x34
#define QSERDES_COM_CLK_SELECT		0x174
#define QSERDES_COM_BG_TRIM		0x70
#define QSERDES_RX_UCDR_FASTLOCK_FO_GAIN	0x440
#define QSERDES_COM_SVS_MODE_CLK_SEL	0x19c
#define QSERDES_COM_HSCLK_SEL		0x178
#define QSERDES_COM_CMN_CONFIG		0x194
#define QSERDES_COM_PLL_IVCO		0x048
#define QSERDES_COM_SYS_CLK_CTRL	0x3c
#define QSERDES_COM_DEC_START_MODE0	0xd0
#define QSERDES_COM_DIV_FRAC_START1_MODE0	0xdc
#define QSERDES_COM_DIV_FRAC_START2_MODE0	0xe0
#define QSERDES_COM_DIV_FRAC_START3_MODE0	0xe4
#define QSERDES_COM_CP_CTRL_MODE0		0x78
#define QSERDES_COM_PLL_RCTRL_MODE0		0x84
#define QSERDES_COM_PLL_CCTRL_MODE0		0x90
#define QSERDES_COM_INTEGLOOP_GAIN0_MODE0	0x108
#define QSERDES_COM_LOCK_CMP1_MODE0		0x4c
#define QSERDES_COM_LOCK_CMP2_MODE0		0x50
#define QSERDES_COM_LOCK_CMP3_MODE0		0x54
#define QSERDES_COM_CORE_CLK_EN			0x18c
#define QSERDES_COM_LOCK_CMP_CFG		0xcc
#define QSERDES_COM_VCO_TUNE_MAP		0x128
#define QSERDES_COM_BG_TIMER			0x0c
#define QSERDES_COM_SSC_EN_CENTER		0x10
#define QSERDES_COM_SSC_PER1			0x1c
#define QSERDES_COM_SSC_PER2			0x20
#define QSERDES_COM_SSC_ADJ_PER1		0x14
#define QSERDES_COM_SSC_ADJ_PER2		0x18
#define QSERDES_COM_SSC_STEP_SIZE1		0x24
#define QSERDES_COM_SSC_STEP_SIZE2		0x28
#define QSERDES_RX_UCDR_SO_GAIN			0x410
#define QSERDES_RX_RX_EQU_ADAPTOR_CNTRL2	0x4d8
#define QSERDES_RX_RX_EQU_ADAPTOR_CNTRL3	0x4dc
#define QSERDES_RX_RX_EQU_ADAPTOR_CNTRL4	0x4e0
#define QSERDES_RX_RX_EQ_OFFSET_ADAPTOR_CNTRL	0x508
#define QSERDES_RX_RX_OFFSET_ADAPTOR_CNTRL2	0x50c
#define QSERDES_RX_SIGDET_CNTRL			0x514
#define QSERDES_RX_SIGDET_DEGLITCH_CNTRL	0x51c
#define QSERDES_RX_SIGDET_ENABLES		0x510
#define QSERDES_TX_HIGHZ_TRANSCEIVEREN_BIAS_D	0x268
#define QSERDES_TX_RCV_DETECT_LVL_2		0x2ac
#define QSERDES_TX_LANE_MODE			0x294
#define PCS_TXDEEMPH_M6DB_V0			0x824
#define PCS_TXDEEMPH_M3P5DB_V0			0x828
#define PCS_FLL_CNTRL2				0x8c8
#define PCS_FLL_CNTRL1				0x8c4
#define PCS_FLL_CNT_VAL_L			0x8cc
#define PCS_FLL_CNT_VAL_H_TOL			0x8d0
#define PCS_FLL_MAN_CODE			0x8d4
#define PCS_LOCK_DETECT_CONFIG1			0x880
#define PCS_LOCK_DETECT_CONFIG2			0x884
#define PCS_LOCK_DETECT_CONFIG3			0x888
#define PCS_POWER_STATE_CONFIG2			0x864
#define PCS_RXEQTRAINING_WAIT_TIME		0x8b8
#define PCS_RXEQTRAINING_RUN_TIME		0x8bc
#define PCS_LFPS_TX_ECSTART_EQTLOCK		0x8b0
#define PCS_PWRUP_RESET_DLY_TIME_AUXCLK		0x8a0
#define PCS_TSYNC_RSYNC_TIME			0x88c
#define PCS_RCVR_DTCT_DLY_P1U2_L		0x870
#define PCS_RCVR_DTCT_DLY_P1U2_H		0x874
#define PCS_RCVR_DTCT_DLY_U3_L			0x878
#define PCS_RCVR_DTCT_DLY_U3_H			0x87c
#define PCS_RX_SIGDET_LVL			0x9d8
#define USB3_PCS_TXDEEMPH_M6DB_V0		0x824
#define USB3_PCS_TXDEEMPH_M3P5DB_V0		0x828
#define QSERDES_RX_SIGDET_ENABLES		0x510
#define USB3_PHY_START_CONTROL			0x808
#define USB3_PHY_SW_RESET			0x800

#define GCC_SYS_NOC_PCIE0_AXI_CLK	0x01826048
#define GCC_PCIE0_PHY_BCR		0x01875038
#define GCC_PCIE0PHY_PHY_BCR		0x0187503C
#define GCC_PCIE0_AXI_M_CBCR		0x01875008
#define GCC_PCIE0_AXI_S_CBCR		0x0187500C
#define GCC_PCIE0_AHB_CBCR		0x01875010
#define GCC_PCIE0_AUX_CBCR		0x01875014
#define GCC_PCIE0_PIPE_CBCR		0x01875018
#define GCC_PCIE0_AUX_CMD_RCGR		0x01875024
#define GCC_PCIE0_AXI_CMD_RCGR		0x01875054
#define GCC_PCIE0_AXI_CFG_RCGR		0x01875058
#define GCC_PCIE0_AXI_S_BRIDGE_CBCR	0x01875048
#define GCC_PCIE0_RCHNG_CMD_RCGR	0x01875070
#define GCC_PCIE0_RCHNG_CFG_RCGR	0x01875074

#define set_mdelay_clearbits_le32(addr, value, delay)	\
	 setbits_le32(addr, value);			\
	 mdelay(delay);					\
	 clrbits_le32(addr, value);			\

#ifdef CONFIG_SMEM_VERSION_C
#define RAM_PART_NAME_LENGTH 16

/**
 * Number of RAM partition entries which are usable by APPS.
 */
#define RAM_NUM_PART_ENTRIES 32
struct ram_partition_entry
{
	char name[RAM_PART_NAME_LENGTH];  /**< Partition name, unused for now */
	u64 start_address;             /**< Partition start address in RAM */
	u64 length;                    /**< Partition length in RAM in Bytes */
	u32 partition_attribute;       /**< Partition attribute */
	u32 partition_category;        /**< Partition category */
	u32 partition_domain;          /**< Partition domain */
	u32 partition_type;            /**< Partition type */
	u32 num_partitions;            /**< Number of partitions on device */
	u32 hw_info;                   /**< hw information such as type and frequency */
	u8 highest_bank_bit;           /**< Highest bit corresponding to a bank */
	u8 reserve0;                   /**< Reserved for future use */
	u8 reserve1;                   /**< Reserved for future use */
	u8 reserve2;                   /**< Reserved for future use */
	u32 reserved5;                 /**< Reserved for future use */
	u64 available_length;          /**< Available Partition length in RAM in Bytes */
};

struct usable_ram_partition_table
{
	u32 magic1;          /**< Magic number to identify valid RAM partition table */
	u32 magic2;          /**< Magic number to identify valid RAM partition table */
	u32 version;         /**< Version number to track structure definition changes
	                             and maintain backward compatibilities */
	u32 reserved1;       /**< Reserved for future use */

	u32 num_partitions;  /**< Number of RAM partition table entries */

	u32 reserved2;       /** < Added for 8 bytes alignment of header */

	/** RAM partition table entries */
	struct ram_partition_entry ram_part_entry[RAM_NUM_PART_ENTRIES];
};
#endif

struct smem_ram_ptn {
	char name[16];
	unsigned long long start;
	unsigned long long size;

	/* RAM Partition attribute: READ_ONLY, READWRITE etc.  */
	unsigned attr;

	/* RAM Partition category: EBI0, EBI1, IRAM, IMEM */
	unsigned category;

	/* RAM Partition domain: APPS, MODEM, APPS & MODEM (SHARED) etc. */
	unsigned domain;

	/* RAM Partition type: system, bootloader, appsboot, apps etc. */
	unsigned type;

	/* reserved for future expansion without changing version number */
	unsigned reserved2, reserved3, reserved4, reserved5;
} __attribute__ ((__packed__));

struct smem_ram_ptable {
#define _SMEM_RAM_PTABLE_MAGIC_1	0x9DA5E0A8
#define _SMEM_RAM_PTABLE_MAGIC_2	0xAF9EC4E2
	unsigned magic[2];
	unsigned version;
	unsigned reserved1;
	unsigned len;
	unsigned buf;
	struct smem_ram_ptn parts[32];
} __attribute__ ((__packed__));

int smem_ram_ptable_init(struct smem_ram_ptable *smem_ram_ptable);

typedef enum {
	SMEM_SPINLOCK_ARRAY = 7,
	SMEM_AARM_PARTITION_TABLE = 9,
	SMEM_HW_SW_BUILD_ID = 137,
	SMEM_USABLE_RAM_PARTITION_TABLE = 402,
	SMEM_POWER_ON_STATUS_INFO = 403,
	SMEM_MACHID_INFO_LOCATION = 425,
	SMEM_IMAGE_VERSION_TABLE = 469,
	SMEM_BOOT_FLASH_TYPE = 498,
	SMEM_BOOT_FLASH_INDEX = 499,
	SMEM_BOOT_FLASH_CHIP_SELECT = 500,
	SMEM_BOOT_FLASH_BLOCK_SIZE = 501,
	SMEM_BOOT_FLASH_DENSITY = 502,
	SMEM_BOOT_DUALPARTINFO = 503,
	SMEM_PARTITION_TABLE_OFFSET = 504,
	SMEM_SPI_FLASH_ADDR_LEN = 505,
	SMEM_FIRST_VALID_TYPE = SMEM_SPINLOCK_ARRAY,
	SMEM_LAST_VALID_TYPE = SMEM_SPI_FLASH_ADDR_LEN,
	SMEM_MAX_SIZE = SMEM_SPI_FLASH_ADDR_LEN + 1,
} smem_mem_type_t;

extern const char *rsvd_node;
extern const char *del_node[];
extern const add_node_t add_fdt_node[];

int ipq_board_usb_init(void);

#define MSM_SDC1_BASE           0x7800000
#define MSM_SDC1_SDHCI_BASE     0x7804000
#ifdef CONFIG_PCI_IPQ

void board_pci_init(int id);
__weak void board_pcie_clock_init(int id) {}
#endif

#endif /* _IPQ6018_CDP_H_ */
