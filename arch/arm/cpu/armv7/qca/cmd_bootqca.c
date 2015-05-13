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

#include <configs/qca961x_cdp.h>
#include <common.h>
#include <command.h>
#include <image.h>
#include <nand.h>
#include <errno.h>
#include <asm/arch-qca961x/scm.h>
#include <part.h>
#include "../../../../../board/qcom/common/qca_common.h"
#include <linux/mtd/ubi.h>
#include <asm/arch-qca961x/smem.h>
#include <mmc.h>

#define DLOAD_MAGIC_COOKIE	0x10
static int debug = 0;

DECLARE_GLOBAL_DATA_PTR;
static qca_smem_flash_info_t *sfi = &qca_smem_flash_info;
int ipq_fs_on_nand ;
extern int nand_env_device;
#ifdef CONFIG_QCA_MMC
static qca_mmc *host = &mmc_host;
#endif

/**
 * Inovke the dump routine and in case of failure, do not stop unless the user
 * requested to stop
 */
#ifdef CONFIG_QCA_APPSBL_DLOAD
static int inline do_dumpipq_data()
{
	uint64_t etime;

	if (run_command("dumpipq_data", 0) != CMD_RET_SUCCESS) {
		printf("\nAuto crashdump saving failed!"
		"\nPress any key within 10s to take control of U-Boot");

		etime = get_timer_masked() + (10 * CONFIG_SYS_HZ);
		while (get_timer_masked() < etime) {
			if (tstc())
				break;
		}

		if (get_timer_masked() < etime)
			return CMD_RET_FAILURE;
	}
	return CMD_RET_SUCCESS;
}
#endif

/*
 * Set the root device and bootargs for mounting root filesystem.
 */
static int set_fs_bootargs(int *fs_on_nand)
{
	char *bootargs;
#ifdef CONFIG_QCA_MMC
	char emmc_rootfs[30];
	block_dev_desc_t *blk_dev = mmc_get_dev(host->dev_num);
	disk_partition_t disk_info;
	int pos;
#endif

	if (sfi->flash_type == SMEM_BOOT_SPI_FLASH) {
		*fs_on_nand = 0;
	} else if (sfi->flash_type == SMEM_BOOT_NAND_FLASH) {
		*fs_on_nand = 1;
#ifdef CONFIG_QCA_MMC
	} else if (sfi->flash_type == SMEM_BOOT_MMC_FLASH) {
		pos = find_part_efi(blk_dev, IPQ_ROOT_FS_PART_NAME, &disk_info);
		if (pos > 0) {
			snprintf(emmc_rootfs, sizeof(emmc_rootfs),
				"root=/dev/mmcblk0p%d", pos);
			bootargs = emmc_rootfs;
			*fs_on_nand = 0;
		}
#endif
	} else {
		printf("bootipq: unsupported boot flash type\n");
		return -EINVAL;
	}

	if (getenv("fsbootargs") == NULL)
		setenv("fsbootargs", bootargs);

	return run_command("setenv bootargs ${bootargs} ${fsbootargs}", 0);
}



static int do_bootqca(cmd_tbl_t *cmdtp, int flag, int argc, char *const argv[])
{
#ifdef CONFIG_QCA_APPSBL_DLOAD
	uint64_t etime;
	volatile u32 val;
#endif
	int ret;
	char runcmd[256];
#ifdef CONFIG_QCA_MMC
	block_dev_desc_t *blk_dev = mmc_get_dev(host->dev_num);
	disk_partition_t disk_info;
#endif

	if (argc == 2 && strncmp(argv[1], "debug", 5) == 0)
		debug = 1;
#ifdef CONFIG_QCA_APPSBL_DLOAD
	ret = scm_call(SCM_SVC_BOOT, SCM_SVC_RD, NULL,
			0, &val, sizeof(val));
	/* check if we are in download mode */
	if (val == DLOAD_MAGIC_COOKIE) {
		/* clear the magic and run the dump command */
		val = 0x0;
		ret = scm_call(SCM_SVC_BOOT, SCM_SVC_WR,
			&val, sizeof(val), NULL, 0);
		if (ret)
			printf ("Error in reseting the Magic cookie\n");

		etime = get_timer_masked() + (10 * CONFIG_SYS_HZ);

		printf("\nCrashdump magic found."
			"\nHit any key within 10s to stop dump activity...");
		while (!tstc()) {       /* while no incoming data */
			if (get_timer_masked() >= etime) {
				if (do_dumpipq_data() == CMD_RET_FAILURE)
					return CMD_RET_FAILURE;
				break;
			}
		}
		/* reset the system, some images might not be loaded
		 * when crashmagic is found
		 */
		run_command("reset", 0);
	}
#endif

	if ((ret = set_fs_bootargs(&ipq_fs_on_nand)))
		return ret;

	if (debug) {
		run_command("printenv bootargs", 0);
		printf("Booting from flash\n");
	}

	if (sfi->flash_type == SMEM_BOOT_NAND_FLASH) {
		if (debug) {
			printf("Using nand device 0\n");
		}

		/*
		 * The kernel is in seperate partition
		 */
		if (sfi->dtb.offset == 0xBAD0FF5E || sfi->hlos.offset == 0xBAD0FF5E) {
			printf(" bad offset of dtb/hlos");
			return -1;
		}

		snprintf(runcmd, sizeof(runcmd),
			"set mtdids nand0=nand0 && "
			"set mtdparts mtdparts=nand0:${msmparts} && "
			"nand read 0x%x 0x%x 0x%x && "
			"nand read 0x%x 0x%x 0x%x && "
			"bootm 0x%x - 0x%x\n",
			CONFIG_SYS_LOAD_ADDR, (uint)sfi->hlos.offset, (uint)sfi->hlos.size,
			CONFIG_DTB_LOAD_ADDR, (uint)sfi->dtb.offset, (uint)sfi->dtb.size,
			CONFIG_SYS_LOAD_ADDR, CONFIG_DTB_LOAD_ADDR);

	} else if (sfi->flash_type == SMEM_BOOT_SPI_FLASH) {
		if (sfi->dtb.offset == 0xBAD0FF5E || sfi->hlos.offset == 0xBAD0FF5E) {
			printf(" bad offset of dtb/hlos\n");
			return -1;
		}
		/*
		 * Kernel is in a separate partition
		 */
		snprintf(runcmd, sizeof(runcmd),
			"set mtdids nand1=nand1 && "
			"set mtdparts mtdparts=nand1:${msmparts} && "
			"sf probe &&"
			"sf read 0x%x 0x%x 0x%x && "
			"sf read 0x%x 0x%x 0x%x && "
			"bootm 0x%x - 0x%x\n",
			CONFIG_SYS_LOAD_ADDR, (uint)sfi->hlos.offset, (uint)sfi->hlos.size,
			CONFIG_DTB_LOAD_ADDR, (uint)sfi->dtb.offset, (uint)sfi->dtb.size,
			CONFIG_SYS_LOAD_ADDR, CONFIG_DTB_LOAD_ADDR);
#ifdef CONFIG_QCA_MMC
	} else if (sfi->flash_type == SMEM_BOOT_MMC_FLASH) {
		if (debug) {
			printf("Using MMC device\n");
		}
		ret = find_part_efi(blk_dev, "0:HLOS", &disk_info);

		if (ret > 0) {
			snprintf(runcmd, sizeof(runcmd), "mmc read 0x%x 0x%x 0x%x",
					CONFIG_SYS_LOAD_ADDR,
					(uint)disk_info.start, (uint)disk_info.size);

			if (run_command(runcmd, 0) != CMD_RET_SUCCESS)
				return CMD_RET_FAILURE;
		}

		ret = find_part_efi(blk_dev, "0:DTB", &disk_info);

		if (ret > 0) {
			snprintf(runcmd, sizeof(runcmd), "mmc read 0x%x 0x%x 0x%x",
					CONFIG_DTB_LOAD_ADDR,
					(uint)disk_info.start, (uint)disk_info.size);

			if (run_command(runcmd, 0) != CMD_RET_SUCCESS)
				return CMD_RET_FAILURE;

			snprintf(runcmd, sizeof(runcmd),"bootm 0x%x - 0x%x",
						CONFIG_SYS_LOAD_ADDR, CONFIG_DTB_LOAD_ADDR);
		}
#endif   	/* CONFIG_QCA_MMC   */
	} else {
		printf("Unsupported BOOT flash type\n");
		return -1;
	}

#ifdef CONFIG_QCA_MMC
	board_mmc_deinit();
#endif

	if (run_command(runcmd, 0) != CMD_RET_SUCCESS) {
#ifdef CONFIG_QCA_MMC
		mmc_initialize(gd->bd);
#endif
		return CMD_RET_FAILURE;
	}

	return CMD_RET_SUCCESS;
}

U_BOOT_CMD(bootipq, 2, 0, do_bootqca,
	   "bootipq from flash device",
	   "bootipq [debug] - Load image(s) and boots the kernel\n");
