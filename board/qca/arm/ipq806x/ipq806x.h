/*
 * Copyright (c) 2016-2017, The Linux Foundation. All rights reserved.
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

#ifndef _IPQ806X_H_
#define _IPQ806X_H_

#include <configs/ipq806x.h>
#include <asm/u-boot.h>
#include <asm/arch-ipq806x/clk.h>
#include <asm/arch-qca-common/qca_common.h>
#include "phy.h"

#define GSBI4_BASE		0x16300000
#define GMAC_AHB_RESET		0x903E24

#define KERNEL_AUTH_CMD                 0x7

#define MSM_TMR_BASE        0x0200A000

#define APCS_WDT0_EN        (MSM_TMR_BASE + 0x0040)
#define APCS_WDT0_RST       (MSM_TMR_BASE + 0x0038)
#define APCS_WDT0_BARK_TIME (MSM_TMR_BASE + 0x004C)
#define APCS_WDT0_BITE_TIME (MSM_TMR_BASE + 0x005C)

#define APCS_WDT0_CPU0_WDOG_EXPIRED_ENABLE (MSM_CLK_CTL_BASE + 0x3820)

/* Watchdog bite time set to default reset value */
#define RESET_WDT_BITE_TIME 0x31F3

/* Watchdog bark time value is kept larger than the watchdog timeout
 * of 0x31F3, effectively disabling the watchdog bark interrupt
 */
#define RESET_WDT_BARK_TIME (5 * RESET_WDT_BITE_TIME)

#define CE1_REG_USAGE           (0)
#define CE1_ADM_USAGE           (1)
#define CE1_RESOURCE            (1)

typedef struct {
	uint count;
	u8 addr[7];
} ipq_gmac_phy_addr_t;

typedef struct {
	uint base;
	int unit;
	uint is_macsec;
	uint mac_pwr0;
	uint mac_pwr1;
	uint mac_conn_to_phy;
	phy_interface_t phy;
	ipq_gmac_phy_addr_t phy_addr;
	const char phy_name[MDIO_NAME_LEN];
} ipq_gmac_board_cfg_t;

typedef struct {
	unsigned int resource;
	unsigned int channel_id;
} switch_ce_chn_buf_t;

#define gmac_board_cfg(_b, _sec, _p, _p0, _p1, _mp, _pn, ...)           \
{                                                                       \
	.base                   = NSS_GMAC##_b##_BASE,                  \
	.unit                   = _b,                                   \
	.is_macsec              = _sec,                                 \
	.phy                    = PHY_INTERFACE_MODE_##_p,              \
	.phy_addr               = { .count = _pn, { __VA_ARGS__ } },    \
	.mac_pwr0               = _p0,                                  \
	.mac_pwr1               = _p1,                                  \
	.mac_conn_to_phy        = _mp,                                  \
	.phy_name               = "IPQ MDIO"#_b                         \
}

extern ipq_gmac_board_cfg_t gmac_cfg[];
static inline int gmac_cfg_is_valid(ipq_gmac_board_cfg_t *cfg)
{
	/*
	 * 'cfg' is valid if and only if
	 *      unit number is non-negative and less than CONFIG_IPQ_NO_MACS.
	 *      'cfg' pointer lies within the array range of
	 *              board_ipq806x_params_t->gmac_cfg[]
	 */
	return ((cfg >= &gmac_cfg[0]) &&
			(cfg < &gmac_cfg[CONFIG_IPQ_NO_MACS]) &&
			(cfg->unit >= 0) && (cfg->unit < CONFIG_IPQ_NO_MACS));
}

__weak void aquantia_phy_reset(void) {}

typedef enum {
	SMEM_SPINLOCK_ARRAY = 7,
	SMEM_AARM_PARTITION_TABLE = 9,
	SMEM_APPS_BOOT_MODE = 106,
	SMEM_HW_SW_BUILD_ID = 137,
	SMEM_USABLE_RAM_PARTITION_TABLE = 402,
	SMEM_POWER_ON_STATUS_INFO = 403,
	SMEM_RLOCK_AREA = 404,
	SMEM_BOOT_INFO_FOR_APPS = 418,
	SMEM_BOOT_FLASH_TYPE = 421,
	SMEM_BOOT_FLASH_INDEX = 422,
	SMEM_BOOT_FLASH_CHIP_SELECT = 423,
	SMEM_BOOT_FLASH_BLOCK_SIZE = 424,
	SMEM_MACHID_INFO_LOCATION = 425,
	SMEM_BOOT_DUALPARTINFO = 427,
	SMEM_PARTITION_TABLE_OFFSET = 428,
	SMEM_BOOT_FLASH_DENSITY = 429,
	SMEM_IMAGE_VERSION_TABLE = 430,
	SMEM_FIRST_VALID_TYPE = SMEM_SPINLOCK_ARRAY,
	SMEM_LAST_VALID_TYPE = SMEM_IMAGE_VERSION_TABLE,
	SMEM_MAX_SIZE = SMEM_PARTITION_TABLE_OFFSET + 1,
} smem_mem_type_t;

/* Reserved-memory node names*/
extern const char *rsvd_node;
extern const char *del_node[];
extern const add_node_t add_node[];
void reset_crashdump(void);
void ipq_fdt_fixup_socinfo(void *blob);
void board_pci_init(int id);
void board_pcie_clock_init(int id);
#endif /* _IPQ806X_H_ */
