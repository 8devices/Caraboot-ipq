/*
 * Copyright (c) 2015-2017 The Linux Foundation. All rights reserved.
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
#include <environment.h>
#include <asm/arch-qca-common/smem.h>
#include <asm/arch-qca-common/uart.h>
#include <asm/arch-qca-common/gpio.h>
#include <fdtdec.h>

DECLARE_GLOBAL_DATA_PTR;

extern int nand_env_device;
extern env_t *nand_env_ptr;
extern char *nand_env_name_spec;
extern char *sf_env_name_spec;
extern int nand_saveenv(void);
extern int sf_saveenv(void);

#ifdef CONFIG_QCA_MMC
extern env_t *mmc_env_ptr;
extern char *mmc_env_name_spec;
extern int mmc_saveenv(void);
#endif

env_t *env_ptr;
char *env_name_spec;
int (*saveenv)(void);

loff_t board_env_offset;
loff_t board_env_range;
loff_t board_env_size;

__weak
int ipq_board_usb_init(void)
{
	return 0;
}
__weak
void board_usb_deinit(int id)
{
	return 0;
}
__weak
void board_pci_deinit(void)
{
	return 0;
}

int board_init(void)
{
	int ret;
	uint32_t start_blocks;
	uint32_t size_blocks;
	qca_smem_flash_info_t *sfi = &qca_smem_flash_info;

	gd->bd->bi_boot_params = QCA_BOOT_PARAMS_ADDR;
	gd->bd->bi_arch_number = smem_get_board_platform_type();

	ret = smem_get_boot_flash(&sfi->flash_type,
				  &sfi->flash_index,
				  &sfi->flash_chip_select,
				  &sfi->flash_block_size,
				  &sfi->flash_density);

	/*
	 * Should be inited, before env_relocate() is called,
	 * since env. offset is obtained from SMEM.
	 */
	switch (sfi->flash_type) {
	case SMEM_BOOT_MMC_FLASH:
	case SMEM_BOOT_NO_FLASH:
		break;
	default:
		ret = smem_ptable_init();
		if (ret < 0) {
			printf("cdp: SMEM init failed\n");
			return ret;
		}
	}

#ifndef CONFIG_ENV_IS_NOWHERE
	switch (sfi->flash_type) {
	case SMEM_BOOT_NAND_FLASH:
		nand_env_device = CONFIG_NAND_FLASH_INFO_IDX;
		break;
	case SMEM_BOOT_SPI_FLASH:
		nand_env_device = CONFIG_SPI_FLASH_INFO_IDX;
		break;
	case SMEM_BOOT_MMC_FLASH:
	case SMEM_BOOT_NO_FLASH:
		break;
	default:
		printf("BUG: unsupported flash type : %d\n", sfi->flash_type);
		BUG();
	}

	if ((sfi->flash_type != SMEM_BOOT_MMC_FLASH) && (sfi->flash_type != SMEM_BOOT_NO_FLASH))  {
		ret = smem_getpart("0:APPSBLENV", &start_blocks, &size_blocks);
		if (ret < 0) {
			printf("cdp: get environment part failed\n");
			return ret;
		}

		board_env_offset = ((loff_t) sfi->flash_block_size) * start_blocks;
		board_env_size = ((loff_t) sfi->flash_block_size) * size_blocks;
	}

	switch (sfi->flash_type) {
	case SMEM_BOOT_NAND_FLASH:
		board_env_range = CONFIG_ENV_SIZE_MAX;
		BUG_ON(board_env_size < CONFIG_ENV_SIZE_MAX);
		break;
	case SMEM_BOOT_SPI_FLASH:
		board_env_range = board_env_size;
		BUG_ON(board_env_size > CONFIG_ENV_SIZE_MAX);
		break;
#ifdef CONFIG_QCA_MMC
	case SMEM_BOOT_MMC_FLASH:
	case SMEM_BOOT_NO_FLASH:
		board_env_range = CONFIG_ENV_SIZE_MAX;
		break;
#endif
	default:
		printf("BUG: unsupported flash type : %d\n", sfi->flash_type);
		BUG();
	}

	if (sfi->flash_type == SMEM_BOOT_SPI_FLASH) {
		saveenv = sf_saveenv;
		env_name_spec = sf_env_name_spec;
#ifdef CONFIG_QCA_MMC
	} else if (sfi->flash_type == SMEM_BOOT_MMC_FLASH) {
		saveenv = mmc_saveenv;
		env_ptr = mmc_env_ptr;
		env_name_spec = mmc_env_name_spec;
#endif
	} else {
		saveenv = nand_saveenv;
		env_ptr = nand_env_ptr;
		env_name_spec = nand_env_name_spec;
	}
#endif
	ret = ipq_board_usb_init();
	if (ret < 0) {
		printf("WARN: ipq_board_usb_init failed\n");
	}

	return 0;
}

void get_kernel_fs_part_details(void)
{
	int ret, i;
	uint32_t start;         /* block number */
	uint32_t size;          /* no. of blocks */

	qca_smem_flash_info_t *smem = &qca_smem_flash_info;

	struct { char *name; qca_part_entry_t *part; } entries[] = {
		{ "0:HLOS", &smem->hlos },
		{ "rootfs", &smem->rootfs },
	};

	for (i = 0; i < ARRAY_SIZE(entries); i++) {
		ret = smem_getpart(entries[i].name, &start, &size);
		if (ret < 0) {
			qca_part_entry_t *part = entries[i].part;

			/*
			 * To set SoC specific secondary flash type to
			 * eMMC/NAND device based on the one that is enabled.
			 */
			set_flash_secondary_type(smem);

			debug("cdp: get part failed for %s\n", entries[i].name);
			part->offset = 0xBAD0FF5E;
			part->size = 0xBAD0FF5E;
		} else {
			qca_set_part_entry(entries[i].name, smem, entries[i].part, start, size);
		}
	}

	return;
}

/*
 * This function is called in the very beginning.
 * Retreive the machtype info from SMEM and map the board specific
 * parameters. Shared memory region at Dram address 0x40400000
 * contains the machine id/ board type data polulated by SBL.
 */
int board_early_init_f(void)
{
	return 0;
}

int board_late_init(void)
{
	unsigned int machid;
	qca_smem_flash_info_t *sfi = &qca_smem_flash_info;

	if (sfi->flash_type != SMEM_BOOT_MMC_FLASH) {
		get_kernel_fs_part_details();
	}

	/* get machine type from SMEM and set in env */
	machid = gd->bd->bi_arch_number;
	printf("machid: %x\n", machid);
	if (machid != 0) {
		setenv_addr("machid", (void *)machid);
		gd->bd->bi_arch_number = machid;
	}
	set_ethmac_addr();
	return 0;
}

int dram_init(void)
{
	struct smem_ram_ptable rtable;
	int i;
	int mx = ARRAY_SIZE(rtable.parts);

	if (smem_ram_ptable_init(&rtable) > 0) {
		gd->ram_size = 0;
		for (i = 0; i < mx; i++) {
			if (rtable.parts[i].category == RAM_PARTITION_SDRAM &&
			    rtable.parts[i].type == RAM_PARTITION_SYS_MEMORY) {
				gd->ram_size += rtable.parts[i].size;
			}
		}
	} else {
		gd->ram_size = fdtdec_get_uint(gd->fdt_blob, 0, "ddr_size", 256);
		gd->ram_size <<= 20;
	}
	return 0;
}

void enable_caches(void)
{
	icache_enable();
}

void disable_caches(void)
{
	icache_disable();
}

void clear_l2cache_err(void)
{
	return;
}
