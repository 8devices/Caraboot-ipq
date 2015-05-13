/*
 *
 * Copyright (c) 2015, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <common.h>
#include <asm/global_data.h>
#include <asm/io.h>
#include <asm/arch-qcom-common/gpio.h>
#include <asm/errno.h>
#include <asm/arch-qca961x/ess/qca961x_edma.h>
#include <environment.h>
#include "qca961x_board_param.h"
#include "qca961x_cdp.h"
#ifdef CONFIG_IPQ_NAND
#include <linux/mtd/ipq_nand.h>
#include <asm/arch-qcom-common/nand.h>
#else
#include <asm/arch-qcom-common/qpic_nand.h>
#endif
#include <asm/arch-qcom-common/clk.h>
#include <asm/arch-qca961x/smem.h>

DECLARE_GLOBAL_DATA_PTR;

loff_t board_env_offset;
loff_t board_env_range;
extern int nand_env_device;
char *env_name_spec;
extern char *mmc_env_name_spec;
extern char *nand_env_name_spec;
int (*saveenv)(void);
env_t *env_ptr;
extern env_t *mmc_env_ptr;
extern env_t *nand_env_ptr;
extern int nand_env_init(void);
extern int nand_saveenv(void);
extern void nand_env_relocate_spec(void);
extern int qca961x_edma_init(qca961x_edma_board_cfg_t *edma_cfg);
extern int ipq40xx_qca8075_phy_init(qca961x_edma_board_cfg_t *cfg);
extern int mmc_env_init(void);
extern int mmc_saveenv(void);
extern void mmc_env_relocate_spec(void);
#ifdef CONFIG_QCA_MMC
qca_mmc mmc_host;
#endif
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
board_qca961x_params_t *gboard_param = (board_qca961x_params_t *)0xbadb0ad;

/*******************************************************
 Function description: Board specific initialization.
 I/P : None
 O/P : integer, 0 - no error.

********************************************************/
static board_qca961x_params_t *get_board_param(unsigned int machid)
{
	unsigned int index;

	printf("machid : 0x%0x\n", machid);
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
	qca_smem_flash_info_t sfi;

	smem_get_boot_flash(&sfi.flash_type,
				&sfi.flash_index,
				&sfi.flash_chip_select,
				&sfi.flash_block_size);

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
				&sfi.flash_block_size);

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
	loff_t board_env_size;
	qca_smem_flash_info_t *sfi = &qca_smem_flash_info;

#ifdef CONFIG_IPQ_NAND
	nand_env_device = CONFIG_IPQ_NAND_NAND_INFO_IDX;
#else
	/* Hardcode everything for NAND */
	nand_env_device = CONFIG_QPIC_NAND_NAND_INFO_IDX;
#endif

	gd->bd->bi_boot_params = QCA_BOOT_PARAMS_ADDR;
	gd->bd->bi_arch_number = smem_get_board_platform_type();
	gboard_param = get_board_param(gd->bd->bi_arch_number);

	ret = smem_get_boot_flash(&sfi->flash_type,
					&sfi->flash_index,
					&sfi->flash_chip_select,
					&sfi->flash_block_size);
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

	if (sfi->flash_type == SMEM_BOOT_NAND_FLASH) {
		nand_env_device = CONFIG_QPIC_NAND_NAND_INFO_IDX;
	} else if (sfi->flash_type == SMEM_BOOT_SPI_FLASH) {
		nand_env_device = CONFIG_IPQ_SPI_NAND_INFO_IDX;
	} else {
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

	if (sfi->flash_type == SMEM_BOOT_NAND_FLASH) {
		board_env_range = CONFIG_ENV_SIZE_MAX;
		BUG_ON(board_env_size < CONFIG_ENV_SIZE_MAX);
	} else if (sfi->flash_type == SMEM_BOOT_SPI_FLASH) {
		board_env_range = board_env_size;
		BUG_ON(board_env_size > CONFIG_ENV_SIZE_MAX);
#ifdef CONFIG_QCA_MMC
	} else if (sfi->flash_type == SMEM_BOOT_MMC_FLASH) {
		board_env_range = CONFIG_ENV_SIZE_MAX;
#endif
	} else {
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
		{ "0:DTB", &smem->dtb },
	};

	for (i = 0; i < ARRAY_SIZE(entries); i++) {
		ret = smem_getpart(entries[i].name, &start, &size);
		if (ret < 0) {
			qca_part_entry_t *part = entries[i].part;
			printf("cdp: get part failed for %s\n", entries[i].name);
			part->offset = 0xBAD0FF5E;
			part->size = 0xBAD0FF5E;
		} else {
			qca_set_part_entry(smem, entries[i].part, start, size);
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
	/* Hardcoded board param for now. Need to retrieve from SMEM */
	gboard_param = board_params;
	return 0;
}

void clear_l2cache_err(void)
{
	return;
}

void reset_cpu(ulong addr)
{
	/* clear ps-hold bit to reset the soc */
	writel(0, GCNT_PSHOLD);
	while (1);
}

int dram_init(void)
{
	gd->ram_size = 0x10000000; /* 256 MB */
	return 0;
}

void board_nand_init(void)
{
#ifdef CONFIG_IPQ_NAND
	ipq_nand_init(IPQ_NAND_LAYOUT_LINUX, QCOM_NAND_QPIC);
#else
	struct qpic_nand_init_config config;
	config.pipes.read_pipe = DATA_PRODUCER_PIPE;
	config.pipes.write_pipe = DATA_CONSUMER_PIPE;
	config.pipes.cmd_pipe = CMD_PIPE;

	config.pipes.read_pipe_grp = DATA_PRODUCER_PIPE_GRP;
	config.pipes.write_pipe_grp = DATA_CONSUMER_PIPE_GRP;
	config.pipes.cmd_pipe_grp = CMD_PIPE_GRP;

	config.bam_base = QCA961x_QPIC_BAM_CTRL;
	config.nand_base = QCA961x_EBI2ND_BASE;
	config.ee = QPIC_NAND_EE;
	config.max_desc_len = QPIC_NAND_MAX_DESC_LEN;

	qpic_nand_init(&config);
#endif
}

int board_eth_init(bd_t *bis)
{
	u32 status;
	switch (gboard_param->machid) {
	case MACH_TYPE_IPQ40XX_AP_DK01_1_C1:
	case MACH_TYPE_IPQ40XX_AP_DK01_1_C2:
	case MACH_TYPE_IPQ40XX_AP_DK04_1_C1:
	case MACH_TYPE_IPQ40XX_AP_DK04_1_C2:
	case MACH_TYPE_IPQ40XX_AP_DK04_1_C3:
		qca961x_register_switch(ipq40xx_qca8075_phy_init);
		break;
	default:
		break;
	}
	status = qca961x_edma_init(gboard_param->edma_cfg);
	return status;
}

void qca_configure_gpio(gpio_func_data_t *gpio, uint count)
{
	int i;

	for (i = 0; i < count; i++) {
		gpio_tlmm_config(gpio->gpio, gpio->func, gpio->out,
				 gpio->pull, gpio->drvstr, gpio->oe);
		gpio++;
	}
}

#ifdef CONFIG_OF_BOARD_SETUP
/*
 * For newer kernel that boot with device tree (3.14+), all of memory is
 * described in the /memory node, including areas that the kernel should not be
 * touching.
 *
 * By default, u-boot will walk the dram bank info and populate the /memory
 * node; here, overwrite this behavior so we describe all of memory instead.
 */
void ft_board_setup(void *blob, bd_t *bd)
{
	u64 memory_start = CONFIG_SYS_SDRAM_BASE;
	u64 memory_size = gboard_param->ddr_size;

	fdt_fixup_memory_banks(blob, &memory_start, &memory_size, 1);
}
#endif /* CONFIG_OF_BOARD_SETUP */

#ifdef CONFIG_QCA_MMC
int board_mmc_env_init(void)
{
	block_dev_desc_t *blk_dev;
	disk_partition_t disk_info;
	loff_t board_env_size;
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
	int ret = 0;
	qca_smem_flash_info_t *sfi = &qca_smem_flash_info;


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
