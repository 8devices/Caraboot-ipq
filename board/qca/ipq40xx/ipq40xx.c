/*
 * Copyright (c) 2015, 2016 The Linux Foundation. All rights reserved.
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
#include <asm/global_data.h>
#include <asm/io.h>
#include <asm/errno.h>
#include <environment.h>
#include <configs/ipq40xx.h>
#include "ipq40xx.h"
#include "ipq40xx_board_param.h"
#include <nand.h>
#include <part.h>
#include <asm/arch-qcom-common/smem.h>
#include <asm/arch-qcom-common/scm.h>
#include <asm/arch-qcom-common/qpic_nand.h>
#include <jffs2/load_kernel.h>
#include <fdtdec.h>

DECLARE_GLOBAL_DATA_PTR;

loff_t board_env_offset;
loff_t board_env_range;
loff_t board_env_size;
extern int nand_env_device;
char *env_name_spec;
extern char *nand_env_name_spec;
int (*saveenv)(void);
env_t *env_ptr;
extern env_t *nand_env_ptr;
extern int nand_env_init(void);
extern int nand_saveenv(void);
extern void nand_env_relocate_spec(void);
extern int ipq_spi_init(u16);
extern int fdt_node_set_part_info(void *blob, int parent_offset,
				  struct mtd_device *dev);
/*
 * Don't have this as a '.bss' variable. The '.bss' and '.rel.dyn'
 * sections seem to overlap.
 *
 * $ arm-none-linux-gnueabi-objdump -h u-boot
 * . . .
 *  8 .rel.dyn      00004ba8  40630b0c  40630b0c  00038b0c  2**2
 *                  CONTENTS, ALLOC, LOAD, READONLY, DATA
 *  9 .bss          0000559c  40630b0c  40630b0c  00000000  2**3
 *                  ALLOC
 * . . .
 *
 * board_early_init_f() initializes this variable, resulting in one
 * of the relocation entries present in '.rel.dyn' section getting
 * corrupted. Hence, when relocate_code()'s 'fixrel' executes, it
 * patches a wrong address, which incorrectly modifies some global
 * variable resulting in a crash.
 *
 * Moral of the story: Global variables that are written before
 * relocate_code() gets executed cannot be in '.bss'
 */
board_ipq40xx_params_t *gboard_param = (board_ipq40xx_params_t *)0xbadb0ad;

#define DLOAD_DISABLE 0x1
#define RESERVE_ADDRESS_START 0x87B00000 /*TZAPPS, SMEM and TZ Regions */
#define RESERVE_ADDRESS_SIZE 0x500000

#define SET_MAGIC 0x1
#define CLEAR_MAGIC 0x0
#define SCM_CMD_TZ_CONFIG_HW_FOR_RAM_DUMP_ID 0x9
#define SCM_CMD_TZ_FORCE_DLOAD_ID 0x10
#define BOOT_VERSION	0
#define TZ_VERSION	1
/*******************************************************
 Function description: Board specific initialization.
 I/P : None
 O/P : integer, 0 - no error.

********************************************************/

static board_ipq40xx_params_t *get_board_param(unsigned int machid)
{
	unsigned int index;

	printf("machid: %x\n", machid);
	for (index = 0; index < NUM_IPQ40XX_BOARDS; index++) {
		if (machid == board_params[index].machid)
			return &board_params[index];
	}
	BUG_ON(index == NUM_IPQ40XX_BOARDS);
	printf("cdp: Invalid machine id 0x%x\n", machid);
	for (;;);
}

int env_init(void)
{
	int ret;

	ret = 0;

	qca_smem_flash_info_t sfi;

	smem_get_boot_flash(&sfi.flash_type,
				&sfi.flash_index,
				&sfi.flash_chip_select,
				&sfi.flash_block_size,
				&sfi.flash_density);

	if (sfi.flash_type != SMEM_BOOT_MMC_FLASH) {
		ret = nand_env_init();
#ifdef CONFIG_QCA_MMC
	} else {
		ret = mmc_env_init();
#endif
	}

	return ret;
}

void env_relocate_spec(void)
{
	qca_smem_flash_info_t sfi;

	smem_get_boot_flash(&sfi.flash_type,
				&sfi.flash_index,
				&sfi.flash_chip_select,
				&sfi.flash_block_size,
				&sfi.flash_density);

	if (sfi.flash_type != SMEM_BOOT_MMC_FLASH) {
		nand_env_relocate_spec();
#ifdef CONFIG_QCA_MMC
	} else {
		mmc_env_relocate_spec();
#endif
	}

};

int board_init(void)
{
	int ret;
	uint32_t start_blocks;
	uint32_t size_blocks;
	qca_smem_flash_info_t *sfi = &qca_smem_flash_info;

	gd->bd->bi_boot_params = QCA_BOOT_PARAMS_ADDR;
	gd->bd->bi_arch_number = smem_get_board_platform_type();
	gboard_param = get_board_param(gd->bd->bi_arch_number);

	ret = smem_get_boot_flash(&sfi->flash_type,
					&sfi->flash_index,
					&sfi->flash_chip_select,
					&sfi->flash_block_size,
					&sfi->flash_density);
	if (ret < 0) {
		printf("cdp: get boot flash failed\n");
		return ret;
	}

	/*
	 * Should be inited, before env_relocate() is called,
	 * since env. offset is obtained from SMEM.
	 */
	if (sfi->flash_type != SMEM_BOOT_MMC_FLASH) {
		ret = smem_ptable_init();
		if (ret < 0) {
			printf("cdp: SMEM init failed\n");
			return ret;
		}
	}
	switch (sfi->flash_type) {
	case SMEM_BOOT_NAND_FLASH:
		nand_env_device = CONFIG_QPIC_NAND_NAND_INFO_IDX;
		break;
	case SMEM_BOOT_SPI_FLASH:
		nand_env_device = CONFIG_IPQ_SPI_NOR_INFO_IDX;
		break;
	case SMEM_BOOT_MMC_FLASH:
		break;
	default:
		printf("BUG: unsupported flash type : %d\n", sfi->flash_type);
		BUG();
	}


	if (sfi->flash_type != SMEM_BOOT_MMC_FLASH) {
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
		board_env_range = CONFIG_ENV_SIZE_MAX;
                break;
#endif
        default:
		printf("BUG: unsupported flash type : %d\n", sfi->flash_type);
                BUG();
	}

	if (sfi->flash_type != SMEM_BOOT_MMC_FLASH) {
		saveenv = nand_saveenv;
		env_ptr = nand_env_ptr;
		env_name_spec = nand_env_name_spec;
#ifdef CONFIG_QCA_MMC
	} else {
		saveenv = mmc_saveenv;
		env_ptr = mmc_env_ptr;
		env_name_spec = mmc_env_name_spec;
#endif
	}
	return 0;
}

void qca_get_part_details(void)
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
			debug("cdp: get part failed for %s\n", entries[i].name);
			part->offset = 0xBAD0FF5E;
			part->size = 0xBAD0FF5E;
		} else {
			qca_set_part_entry(entries[i].name, smem, entries[i].part, start, size);
		}
	}

	return;
}

int board_late_init(void)
{
	unsigned int machid;
	qca_smem_flash_info_t *sfi = &qca_smem_flash_info;

	if (sfi->flash_type != SMEM_BOOT_MMC_FLASH) {
		qca_get_part_details();
	}

	/* get machine type from SMEM and set in env */
	machid = gd->bd->bi_arch_number;
	printf("machid: %x\n", machid);
	if (machid != 0) {
		setenv_addr("machid", (void *)machid);
		gd->bd->bi_arch_number = machid;
	}
	return 0;
}

/*
 * This function is called in the very beginning.
 * Retreive the machtype info from SMEM and map the board specific
 * parameters. Shared memory region at Dram address 0x40400000
 * contains the machine id/ board type data polulated by SBL.
 */
int board_early_init_f(void)
{
	/*Retrieve the machid of the board from smem*/
	gboard_param = get_board_param(smem_get_board_platform_type());
	return 0;
}

void enable_caches(void)
{
	icache_enable();
}

void clear_l2cache_err(void)
{
	return;
}

void reset_cpu(ulong addr)
{
}

static void reset_crashdump(void)
{
        unsigned int magic_cookie = CLEAR_MAGIC;
        unsigned int clear_info[] =
		{ 1 /* Disable wdog debug */, 0 /* SDI enable*/, };
        scm_call(SCM_SVC_BOOT, SCM_CMD_TZ_CONFIG_HW_FOR_RAM_DUMP_ID,
		 (const void *)&clear_info, sizeof(clear_info), NULL, 0);
        scm_call(SCM_SVC_BOOT, SCM_CMD_TZ_FORCE_DLOAD_ID, &magic_cookie,
		 sizeof(magic_cookie), NULL, 0);
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
		gboard_param->ddr_size = gd->ram_size;
	} else {
		gd->ram_size = gboard_param->ddr_size;
	}
	return 0;
}

void board_nand_init(void)
{
	struct qpic_nand_init_config config;
	gpio_func_data_t *gpio;
	int node;
	fdt_addr_t nand_base;

#ifdef CONFIG_IPQ40XX_SPI
	ipq_spi_init(CONFIG_IPQ_SPI_NOR_INFO_IDX);
#endif

	node = fdtdec_next_compatible(gd->fdt_blob, 0,
				      COMPAT_QCOM_QPIC_NAND_V1_4_20);

	if (node < 0) {
		printf("Could not find nand-flash in device tree\n");
		return;
	}

	nand_base = fdtdec_get_addr(gd->fdt_blob, node, "reg");

	if (nand_base == FDT_ADDR_T_NONE) {
		printf("No valid NAND base address found in device tree\n");
		return;
        }
	config.pipes.read_pipe = DATA_PRODUCER_PIPE;
	config.pipes.write_pipe = DATA_CONSUMER_PIPE;
	config.pipes.cmd_pipe = CMD_PIPE;

	config.pipes.read_pipe_grp = DATA_PRODUCER_PIPE_GRP;
	config.pipes.write_pipe_grp = DATA_CONSUMER_PIPE_GRP;
	config.pipes.cmd_pipe_grp = CMD_PIPE_GRP;

	config.bam_base = QPIC_BAM_CTRL_BASE;
	config.nand_base = nand_base;
	config.ee = QPIC_NAND_EE;
	config.max_desc_len = QPIC_NAND_MAX_DESC_LEN;

	gpio = gboard_param->nand_gpio;
	if (gpio) {
		qca_configure_gpio(gpio,
				   gboard_param->nand_gpio_count);
	}

	qpic_nand_init(&config);
}

int board_eth_init(bd_t *bis)
{
	return 0;
}

void qca_configure_gpio(gpio_func_data_t *gpio, uint count)
{
	int i;

	for (i = 0; i < count; i++) {
		gpio_tlmm_config(gpio->gpio, gpio->func, gpio->out,
			gpio->pull, gpio->drvstr, gpio->oe,
			gpio->gpio_vm, gpio->gpio_od_en, gpio->gpio_pu_res);
		gpio++;
	}
}

#ifdef CONFIG_OF_BOARD_SETUP
struct flash_node_info {
	const char *compat;	/* compatible string */
	int type;		/* mtd flash type */
	int idx;		/* flash index */
};

int ipq_fdt_fixup_spi_nor_params(void *blob)
{
	int nodeoff, ret;
	qca_smem_flash_info_t sfi;
	uint32_t val;

	/* Get flash parameters from smem */
	smem_get_boot_flash(&sfi.flash_type,
				&sfi.flash_index,
				&sfi.flash_chip_select,
				&sfi.flash_block_size,
				&sfi.flash_density);
	nodeoff = fdt_node_offset_by_compatible(blob, -1, "n25q128a11");

	if (nodeoff < 0) {
		printf("ipq: fdt fixup unable to find compatible node\n");
		return nodeoff;
	}

	val = cpu_to_fdt32(sfi.flash_block_size);
	ret = fdt_setprop(blob, nodeoff, "sector-size",
			&val, sizeof(uint32_t));
	if (ret) {
		printf("%s: unable to set sector size\n", __func__);
		return -1;
	}

	val = cpu_to_fdt32(sfi.flash_density);
	ret = fdt_setprop(blob, nodeoff, "density",
			&val, sizeof(uint32_t));
	if (ret) {
		printf("%s: unable to set density\n", __func__);
		return -1;
	}

	return 0;
}

void ipq_fdt_mem_rsvd_fixup(void *blob)
{
	u32 val[2], dload;
	int nodeoff, ret;
	dload = htonl(DLOAD_DISABLE);
	val[0] = htonl(RESERVE_ADDRESS_START);
	val[1] = htonl(RESERVE_ADDRESS_SIZE);

	/* Reserve only the TZ and SMEM memory region and free the rest */
	nodeoff = fdt_path_offset(blob, "/reserved-memory/rsvd2");
	if (nodeoff < 0) {
		debug("ipq: fdt fixup unable to find compatible node\n");
		return;
	}
	ret = fdt_del_node(blob, nodeoff);
	if (ret != 0) {
		debug("ipq: fdt fixup unable to delete node\n");
		return;
	}
	nodeoff = fdt_path_offset(blob, "/reserved-memory/wifi_dump");
	if (nodeoff < 0) {
		debug("ipq: fdt fixup unable to find compatible node\n");
		return;
	}
	ret = fdt_del_node(blob, nodeoff);
	if (ret != 0) {
		debug("ipq: fdt fixup unable to delete node\n");
		return;
	}
	nodeoff = fdt_path_offset(blob, "/reserved-memory/rsvd1");
	if (nodeoff < 0) {
		debug("ipq: fdt fixup unable to find compatible node\n");
		return;
	}
	ret = fdt_setprop(blob, nodeoff, "reg", val, sizeof(val));
	if (ret != 0) {
		debug("ipq: fdt fixup unable to find compatible node\n");
		return;
	}

	/* Set the dload_status to DLOAD_DISABLE */
	nodeoff = fdt_path_offset(blob, "/soc/qca,scm_restart_reason");
	if (nodeoff < 0) {
		debug("ipq: fdt fixup unable to find compatible node\n");
		return;
	}
	ret = fdt_setprop(blob, nodeoff, "dload_status", &dload, sizeof(dload));
	if (ret != 0) {
		debug("ipq: fdt fixup unable to find compatible node\n");
		return;
	}
	reset_crashdump();
}

void ipq_fdt_fixup_version(void *blob)
{
	int nodeoff, ret;
	char ver[OEM_VERSION_STRING_LENGTH + VERSION_STRING_LENGTH + 1];

	nodeoff = fdt_node_offset_by_compatible(blob, -1, "qcom,ipq40xx");

	if (nodeoff < 0) {
		debug("ipq: fdt fixup unable to find compatible node\n");
		return;
	}

	if (!smem_get_build_version(ver, sizeof(ver), BOOT_VERSION)) {
		debug("BOOT Build Version:  %s\n", ver);
		ret = fdt_setprop(blob, nodeoff, "boot_version",
				ver, strlen(ver));
		if (ret)
			debug("%s: unable to set Boot version\n", __func__);
	}

	if (!smem_get_build_version(ver, sizeof(ver), TZ_VERSION)) {
		debug("TZ Build Version:  %s\n", ver);
		ret = fdt_setprop(blob, nodeoff, "tz_version",
				ver, strlen(ver));
		if (ret)
			debug("%s: unable to set TZ version\n", __func__);
	}
}

void ipq_fdt_fixup_mtdparts(void *blob, struct flash_node_info *ni)
{
	struct mtd_device *dev;
	char *parts;
	int noff;

	parts = getenv("mtdparts");
	if (!parts)
		return;

	if (mtdparts_init() != 0)
		return;

	for (; ni->compat; ni++) {
		noff = fdt_node_offset_by_compatible(blob, -1, ni->compat);
		while (noff != -FDT_ERR_NOTFOUND) {
			dev = device_find(ni->type, ni->idx);
			if (dev) {
				if (fdt_node_set_part_info(blob, noff, dev))
					return; /* return on error */
			}

			/* Jump to next flash node */
			noff = fdt_node_offset_by_compatible(blob, noff,
							     ni->compat);
		}
	}
}
/*
 * For newer kernel that boot with device tree (3.14+), all of memory is
 * described in the /memory node, including areas that the kernel should not be
 * touching.
 *
 * By default, u-boot will walk the dram bank info and populate the /memory
 * node; here, overwrite this behavior so we describe all of memory instead.
 */
int ft_board_setup(void *blob, bd_t *bd)
{
	u64 memory_start = CONFIG_SYS_SDRAM_BASE;
	u64 memory_size = gboard_param->ddr_size;
	char *mtdparts = NULL;
	char parts_str[4096];
	qca_smem_flash_info_t *sfi = &qca_smem_flash_info;
	struct flash_node_info nodes[] = {
		{ "qcom,msm-nand", MTD_DEV_TYPE_NAND, 0 },
		{ "spinand,mt29f", MTD_DEV_TYPE_NAND, 1 },
		{ "n25q128a11", MTD_DEV_TYPE_NAND, 2 },
		{ NULL, 0, -1 },	/* Terminator */
	};

	fdt_fixup_memory_banks(blob, &memory_start, &memory_size, 1);
	ipq_fdt_fixup_version(blob);
#ifndef CONFIG_QCA_APPSBL_DLOAD
	ipq_fdt_mem_rsvd_fixup(blob);
#endif
	if (sfi->flash_type == SMEM_BOOT_NAND_FLASH) {
		sprintf(parts_str, "mtdparts=nand0");
		mtdparts = parts_str;
	} else if (sfi->flash_type == SMEM_BOOT_SPI_FLASH) {
		/* Patch NOR block size and density for
		 * generic probe case */
		ipq_fdt_fixup_spi_nor_params(blob);
		if (gboard_param->spi_nand_available &&
			get_which_flash_param("rootfs") == 0) {
			sprintf(parts_str,
				"mtdparts=nand1:0x%x@0(rootfs);spi0.0",
				IPQ_NAND_ROOTFS_SIZE);
		} else if (gboard_param->nor_nand_available &&
			get_which_flash_param("rootfs") == 0) {
			sprintf(parts_str,
				"mtdparts=nand0:0x%x@0(rootfs);spi0.0",
				IPQ_NAND_ROOTFS_SIZE);
		} else {
			sprintf(parts_str, "mtdparts=spi0.0");
		}
		mtdparts = parts_str;
	}

	if (mtdparts) {
		qca_smem_part_to_mtdparts(mtdparts);
		if (mtdparts[0] != '\0') {
			debug("mtdparts = %s\n", mtdparts);
			setenv("mtdparts", mtdparts);
		}

		debug("MTDIDS: %s\n", getenv("mtdids"));
		ipq_fdt_fixup_mtdparts(blob, nodes);
	}
	dcache_disable();
	fdt_fixup_ethernet(blob);

#ifdef CONFIG_QCA_MMC
        board_mmc_deinit();
#endif
	return 0;
}

#endif /* CONFIG_OF_BOARD_SETUP */

#ifdef CONFIG_QCA_MMC
int board_mmc_env_init(void)
{
	block_dev_desc_t *blk_dev;
	disk_partition_t disk_info;
	int ret;

	if (mmc_init(mmc_host.mmc)) {
		/* The HS mode command(cmd6) is getting timed out. So mmc card is
		 * not getting initialized properly. Since the env partition is not
		 * visible, the env default values are writing into the default
		 * partition (start of the mmc device). So do a reset again.
		 */
		if (mmc_init(mmc_host.mmc)) {
			printf("MMC init failed \n");
			return -1;
		}
	}
	blk_dev = mmc_get_dev(mmc_host.dev_num);
	ret = find_part_efi(blk_dev, "0:APPSBLENV", &disk_info);

	if (ret > 0) {
		board_env_offset = disk_info.start * disk_info.blksz;
		board_env_size = disk_info.size * disk_info.blksz;
		board_env_range = board_env_size;
		BUG_ON(board_env_size > CONFIG_ENV_SIZE_MAX);
	}
	return ret;
}

int board_mmc_init(bd_t *bis)
{
	int ret;
	qca_smem_flash_info_t *sfi = &qca_smem_flash_info;

	if (!gboard_param->mmc_gpio_count)
		return 0;

	qca_configure_gpio(gboard_param->mmc_gpio,
			gboard_param->mmc_gpio_count);

	mmc_host.base = MSM_SDC1_BASE;
	mmc_host.clk_mode = MMC_IDENTIFY_MODE;
	emmc_clock_config(mmc_host.clk_mode);

	ret = qca_mmc_init(bis, &mmc_host);

	if (!ret && sfi->flash_type == SMEM_BOOT_MMC_FLASH) {
		ret = board_mmc_env_init();
	}

	return ret;
}

void board_mmc_deinit(void)
{
	emmc_clock_disable();
}
#endif

#ifdef CONFIG_IPQ40XX_PCI
void pcie_config_gpio(pcie_params_t *cfg, int enable)
{
	int i;
	gpio_func_data_t *gpio_data;
	gpio_data = cfg->pci_gpio;

	for (i = 0; i < cfg->pci_gpio_count; i++) {
		if (enable)
			gpio_tlmm_config(gpio_data->gpio, gpio_data->func,
					gpio_data->out, gpio_data->pull,
					gpio_data->drvstr, gpio_data->oe,
					gpio_data->gpio_vm, gpio_data->gpio_od_en,
					gpio_data->gpio_pu_res);
		else
			gpio_tlmm_config(gpio_data->gpio, gpio_data->func,
					GPIO_OUT_LOW, GPIO_NO_PULL,
					GPIO_2MA, GPIO_OE_DISABLE,
					GPIO_VM_DISABLE, GPIO_OD_DISABLE,
					gpio_data->gpio_pu_res);
		gpio_data++;
	}
}

void pcie_controller_reset(int id)
{
	uint32_t val;
	pcie_params_t *cfg;
	cfg = &gboard_param->pcie_cfg[id];

	/* Enable PCIE CLKS */
	pcie_clock_enable(GCC_PCIE_SLEEP_CBCR);
	pcie_clock_enable(GCC_PCIE_AXI_M_CBCR);
	pcie_clock_enable(GCC_PCIE_AXI_S_CBCR);
	pcie_clock_enable(GCC_PCIE_AHB_CBCR);

	/* Assert cc_pcie20_core_ares */
	writel(PCIE_RST_CTRL_PIPE_ARES, cfg->pcie_rst);

	/* Assert cc_pcie20_core_sticky_area */
	val = readl(cfg->pcie_rst);
	val |= PCIE_RST_CTRL_PIPE_STICKY_ARES;
	writel(val, cfg->pcie_rst);

	/* Assert cc_pcie20_phy_ahb_ares */
	val = readl(cfg->pcie_rst);
	val |= PCIE_RST_CTRL_PIPE_PHY_AHB_ARES;
	writel(val, cfg->pcie_rst);

	gpio_set_value(PCIE_RST_GPIO, GPIO_OUT_LOW);

	/* Assert cc_pcie20_mstr_axi_ares */
	val = readl(cfg->pcie_rst);
	val |= PCIE_RST_CTRL_AXI_M_ARES;
	writel(val, cfg->pcie_rst);

	/* Assert cc_pcie20_mstr_sticky_ares */
	val = readl(cfg->pcie_rst);
	val |= PCIE_RST_CTRL_AXI_M_STICKY_ARES;
	writel(val, cfg->pcie_rst);

	/* Assert cc_pcie20_slv_axi_ares */
	val = readl(cfg->pcie_rst);
	val |= PCIE_RST_CTRL_AXI_S_ARES;
	writel(val, cfg->pcie_rst);

	/* Assert cc_pcie20_ahb_ares;  */
	val = readl(cfg->pcie_rst);
	val |= PCIE_RST_CTRL_AHB_ARES;
	writel(val, cfg->pcie_rst);

	/* DeAssert cc_pcie20_phy_ahb_ares  */
	val = readl(cfg->pcie_rst);
	val &= ~(PCIE_RST_CTRL_AHB_ARES);
	writel(val, cfg->pcie_rst);

	/* DeAssert cc_pcie20_pciephy_phy_ares*/
	val = readl(cfg->pcie_rst);
	val &= ~(PCIE_RST_CTRL_PIPE_ARES);
	writel(val, cfg->pcie_rst);

	/* DeAssert cc_pcie20_core_sticky_ares */
	val = readl(cfg->pcie_rst);
	val &= ~(PCIE_RST_CTRL_PIPE_STICKY_ARES);
	writel(val, cfg->pcie_rst);

	mdelay(5);

	gpio_set_value(PCIE_RST_GPIO, GPIO_OUT_HIGH);

	/* DeAssert cc_pcie20_mstr_axi_ares */
	val = readl(cfg->pcie_rst);
	val &= ~(PCIE_RST_CTRL_AXI_M_ARES);
	writel(val, cfg->pcie_rst);

	/* DeAssert cc_pcie20_mstr_axi_ares */
	val = readl(cfg->pcie_rst);
	val &= ~(PCIE_RST_CTRL_AXI_M_STICKY_ARES);
	writel(val, cfg->pcie_rst);

	/* DeAssert cc_pcie20_slv_axi_ares */
	val = readl(cfg->pcie_rst);
	val &= ~(PCIE_RST_CTRL_AXI_S_ARES);
	writel(val, cfg->pcie_rst);

	/* DeAssert cc_pcie20_ahb_ares */
	val = readl(cfg->pcie_rst);
	val &= ~(PCIE_RST_CTRL_PIPE_PHY_AHB_ARES);
	writel(val, cfg->pcie_rst);
}

static void ipq_pcie_config_controller(int id)
{
	pcie_params_t 	*cfg;
	cfg = &gboard_param->pcie_cfg[id];

	/*
	 * program and enable address translation region 0 (device config
	 * address space); region type config;
	 * axi config address range to device config address range
	 */
	writel(0, cfg->pcie20 + PCIE20_PLR_IATU_VIEWPORT);

	writel(4, cfg->pcie20 + PCIE20_PLR_IATU_CTRL1);
	writel((1 << 31), cfg->pcie20 + PCIE20_PLR_IATU_CTRL2);
	writel(cfg->axi_conf , cfg->pcie20 + PCIE20_PLR_IATU_LBAR);
	writel(0, cfg->pcie20 + PCIE20_PLR_IATU_UBAR);
	writel((cfg->axi_conf + PCIE_AXI_CONF_SIZE - 1),
				cfg->pcie20 + PCIE20_PLR_IATU_LAR);
	writel(MSM_PCIE_DEV_CFG_ADDR,
				cfg->pcie20 + PCIE20_PLR_IATU_LTAR);
	writel(0, cfg->pcie20 + PCIE20_PLR_IATU_UTAR);

	/*
	 * program and enable address translation region 2 (device resource
	 * address space); region type memory;
	 * axi device bar address range to device bar address range
	 */
	writel(2, cfg->pcie20 + PCIE20_PLR_IATU_VIEWPORT);

	writel(0, cfg->pcie20 + PCIE20_PLR_IATU_CTRL1);
	writel((1 << 31), cfg->pcie20 + PCIE20_PLR_IATU_CTRL2);
	writel(cfg->axi_bar_start, cfg->pcie20 + PCIE20_PLR_IATU_LBAR);
	writel(0, cfg->pcie20 + PCIE20_PLR_IATU_UBAR);
	writel((cfg->axi_bar_start + cfg->axi_bar_size
		- PCIE_AXI_CONF_SIZE - 1), cfg->pcie20 + PCIE20_PLR_IATU_LAR);
	writel(cfg->axi_bar_start, cfg->pcie20 + PCIE20_PLR_IATU_LTAR);
	writel(0, cfg->pcie20 + PCIE20_PLR_IATU_UTAR);

	/* 1K PCIE buffer setting */
	writel(0x3, cfg->pcie20 + PCIE20_AXI_MSTR_RESP_COMP_CTRL0);
	writel(0x1, cfg->pcie20 + PCIE20_AXI_MSTR_RESP_COMP_CTRL1);
}

void pcie_linkup(int id)
{
	int j, val;
	pcie_params_t 		*cfg;
	cfg = &gboard_param->pcie_cfg[id];

	pcie_clock_enable(GCC_PCIE_SLEEP_CBCR);
	pcie_clock_enable(GCC_PCIE_AXI_M_CBCR);
	pcie_clock_enable(GCC_PCIE_AXI_S_CBCR);
	pcie_clock_enable(GCC_PCIE_AHB_CBCR);

	pcie_controller_reset(id);
	mdelay(200);

	writel(SLV_ADDR_SPACE_SZ, cfg->parf + PARF_SLV_ADDR_SPACE_SIZE);
	mdelay(100);

	writel(0x0, cfg->pcie20 + PCIE_0_PORT_FORCE_REG);
	val = (L1_ENTRANCE_LATENCY(3) |
		L0_ENTRANCE_LATENCY(3) |
		COMMON_CLK_N_FTS(128) |
		ACK_N_FTS(128));
	writel(val, cfg->pcie20 + PCIE_0_ACK_F_ASPM_CTRL_REG);

	val = (FAST_TRAINING_SEQ(128) |
		NUM_OF_LANES(2) |
		DIRECT_SPEED_CHANGE);
	writel(val, cfg->pcie20 + PCIE_0_GEN2_CTRL_REG);
	writel(PCI_TYPE0_BUS_MASTER_EN,
		cfg->pcie20 + PCIE_0_TYPE0_STATUS_COMMAND_REG_1);

	writel(DBI_RO_WR_EN, cfg->pcie20 + PCIE_0_MISC_CONTROL_1_REG);
	writel(0x0002FD7F, cfg->pcie20 + 0x84);

	val = (PCIE_CAP_ASPM_OPT_COMPLIANCE |
		PCIE_CAP_LINK_BW_NOT_CAP |
		PCIE_CAP_DLL_ACTIVE_REP_CAP |
		PCIE_CAP_L1_EXIT_LATENCY(4) |
		PCIE_CAP_L0S_EXIT_LATENCY(4) |
		PCIE_CAP_MAX_LINK_WIDTH(1) |
		PCIE_CAP_MAX_LINK_SPEED(1));
	writel(val, cfg->pcie20 + PCIE_0_LINK_CAPABILITIES_REG);

	writel(PCIE_CAP_CPL_TIMEOUT_DISABLE,
		cfg->pcie20 + PCIE_0_DEVICE_CONTROL2_DEVICE_STATUS2_REG);

	writel(0x10110008, cfg->pcie20 + PCIE_0_TYPE0_LINK_CONTROL_LINK_STATUS_REG_1);

	writel(LTSSM_EN, cfg->parf + PCIE_0_PCIE20_PARF_LTSSM);

	mdelay(200);

	for (j = 0; j < 400; j++) {
		val = readl(cfg->pcie20 + PCIE_0_TYPE0_LINK_CONTROL_LINK_STATUS_REG_1);
		if (val & (1 << 29)) {
			printf("PCI%d Link Intialized\n", id);
			cfg->linkup = 1;
			break;
		}
		udelay(100);
	}
	ipq_pcie_config_controller(id);
}

void board_pci_init()
{
	int i;
	pcie_params_t *cfg;

	for (i = 0; i < PCI_MAX_DEVICES; i++) {
		cfg = &gboard_param->pcie_cfg[i];
		pcie_config_gpio(cfg, ENABLE);

		pcie_controller_reset(i);

		pcie_linkup(i);
	}
}

void board_pci_deinit(void)
{
	int i;
	pcie_params_t 		*cfg;

	for (i = 0; i < PCI_MAX_DEVICES; i++) {
		cfg = &gboard_param->pcie_cfg[i];

		writel(1, cfg->parf + PCIE20_PARF_PHY_CTRL);

		pcie_config_gpio(cfg, DISABLE);
	}

	/* Disable PCIE CLKS */
	pcie_clock_disable(GCC_PCIE_SLEEP_CBCR);
	pcie_clock_disable(GCC_PCIE_AXI_M_CBCR);
	pcie_clock_disable(GCC_PCIE_AXI_S_CBCR);
	pcie_clock_disable(GCC_PCIE_AHB_CBCR);
}
#endif /* CONFIG_IPQ40XX_PCI */
