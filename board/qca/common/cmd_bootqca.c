/*
 * Copyright (c) 2015, 2016 The Linux Foundation. All rights reserved.

 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <common.h>
#include <command.h>
#include <image.h>
#include <nand.h>
#include <errno.h>
#include <asm/arch-qcom-common/scm.h>
#include <part.h>
#include <linux/mtd/ubi.h>
#include <asm/arch-qcom-common/smem.h>
#include <mmc.h>
#include <part_efi.h>
#include <fdtdec.h>
#include "fdt_info.h"

#define DLOAD_MAGIC_COOKIE 0x10
#define XMK_STR(x)#x
#define MK_STR(x)XMK_STR(x)

static int debug = 0;
static char mtdids[256];

DECLARE_GLOBAL_DATA_PTR;
static qca_smem_flash_info_t *sfi = &qca_smem_flash_info;
int ipq_fs_on_nand ;
extern int nand_env_device;

#ifdef CONFIG_QCA_MMC
static qca_mmc *host = &mmc_host;
#endif

typedef struct {
	unsigned int image_type;
	unsigned int header_vsn_num;
	unsigned int image_src;
	unsigned char *image_dest_ptr;
	unsigned int image_size;
	unsigned int code_size;
	unsigned char *signature_ptr;
	unsigned int signature_size;
	unsigned char *cert_chain_ptr;
	unsigned int cert_chain_size;
} mbn_header_t;

typedef struct {
	unsigned int kernel_load_addr;
	unsigned int kernel_load_size;
} kernel_img_info_t;

kernel_img_info_t kernel_img_info;

char dtb_config_name[64];

static int do_dumpqca_data(cmd_tbl_t *cmdtp, int flag, int argc,
					char *const argv[])
{
	char runcmd[128];
	char *serverip = NULL;
	/* dump to root of TFTP server if none specified */
	char *dumpdir;
	uint32_t memaddr;
	int indx;

	if (argc == 2) {
		serverip = argv[1];
		printf("Using given serverip %s\n", serverip);
		setenv("serverip", serverip);
	} else {
		serverip = getenv("serverip");
		if (serverip != NULL) {
			printf("Using serverip from env %s\n", serverip);
	} else {
			printf("\nServer ip not found, run dhcp or configure\n");
			return CMD_RET_FAILURE;
		}
	}
	if ((dumpdir = getenv("dumpdir")) != NULL) {
		printf("Using directory %s in TFTP server\n", dumpdir);
	} else {
		dumpdir = "";
		printf("Env 'dumpdir' not set. Using / dir in TFTP server\n");
	}

	for (indx = 0; indx < dump_entries; indx++) {
		printf("\nProcessing %s:", dumpinfo[indx].name);
		memaddr = dumpinfo[indx].start;

		if (dumpinfo[indx].is_aligned_access) {
			snprintf(runcmd, sizeof(runcmd), "cp.l 0x%x 0x%x 0x%x",
				memaddr, IPQ_TEMP_DUMP_ADDR,
				dumpinfo[indx].size);

			if (run_command(runcmd, 0) != CMD_RET_SUCCESS)
				return CMD_RET_FAILURE;

			memaddr = IPQ_TEMP_DUMP_ADDR;
		}

		snprintf(runcmd, sizeof(runcmd), "tftpput 0x%x 0x%x %s/%s",
			memaddr, dumpinfo[indx].size,
			dumpdir,  dumpinfo[indx].name);

		if (run_command(runcmd, 0) != CMD_RET_SUCCESS)
			return CMD_RET_FAILURE;
		udelay(10000); /* give some delay for server */
	}
	return CMD_RET_SUCCESS;
}

U_BOOT_CMD(dumpipq_data, 2, 0, do_dumpqca_data,
	"dumpipq_data crashdump collection from memory",
	"dumpipq_data [serverip] - Crashdump collection from memory vi tftp\n");

/**
 * Inovke the dump routine and in case of failure, do not stop unless the user
 * requested to stop
 */
#ifdef CONFIG_QCA_APPSBL_DLOAD
static int inline do_dumpipq_data(void)
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
	unsigned int active_part = 0;

#define nand_rootfs "ubi.mtd=" QCA_ROOT_FS_PART_NAME " root=mtd:ubi_rootfs rootfstype=squashfs"

	if (sfi->flash_type == SMEM_BOOT_SPI_FLASH) {
		if (((sfi->rootfs.offset == 0xBAD0FF5E) &&
		     (is_nor_emmc_available() == 0)) ||
		    get_which_flash_param("rootfs")) {
			bootargs = nand_rootfs;
			*fs_on_nand = 1;
			fdt_setprop(gd->fdt_blob, 0, "nor_nand_available", fs_on_nand, sizeof(int));
			snprintf(mtdids, sizeof(mtdids),
				 "nand%d=nand%d,nand%d=spi0.0",
				 is_spi_nand_available(),
				 is_spi_nand_available(),
				CONFIG_SPI_FLASH_INFO_IDX
				);

			if (getenv("fsbootargs") == NULL)
				setenv("fsbootargs", bootargs);
		} else {
			if (smem_bootconfig_info() == 0) {
				active_part = get_rootfs_active_partition();
				if (active_part) {
					bootargs = "rootfsname=rootfs_1";
				} else {
					bootargs = "rootfsname=rootfs";
				}
			} else {
				bootargs = "rootfsname=rootfs";
			}
			*fs_on_nand = 0;

			snprintf(mtdids, sizeof(mtdids), "nand%d=spi0.0", CONFIG_SPI_FLASH_INFO_IDX);

			if (getenv("fsbootargs") == NULL)
				setenv("fsbootargs", bootargs);
		}
	} else if (sfi->flash_type == SMEM_BOOT_NAND_FLASH) {
		bootargs = nand_rootfs;
		if (getenv("fsbootargs") == NULL)
			setenv("fsbootargs", bootargs);
		*fs_on_nand = 1;

		snprintf(mtdids, sizeof(mtdids), "nand0=nand0");

#ifdef CONFIG_QCA_MMC
	} else if (sfi->flash_type == SMEM_BOOT_MMC_FLASH) {
		if (smem_bootconfig_info() == 0) {
			active_part = get_rootfs_active_partition();
			if (active_part) {
				bootargs = "rootfsname=rootfs_1";
			} else {
				bootargs = "rootfsname=rootfs";
			}
		} else {
			bootargs = "rootfsname=rootfs";
		}

		*fs_on_nand = 0;
		if (getenv("fsbootargs") == NULL)
			setenv("fsbootargs", bootargs);
#endif
	} else {
		printf("bootipq: unsupported boot flash type\n");
		return -EINVAL;
	}

	return run_command("setenv bootargs ${bootargs} ${fsbootargs} rootwait", 0);
}

int config_select(unsigned int addr, char *rcmd, int rcmd_size)
{
	/* Selecting a config name from the list of available
	 * config names by passing them to the fit_conf_get_node()
	 * function which is used to get the node_offset with the
	 * config name passed. Based on the return value index based
	 * or board name based config is used.
	 */

	int soc_version = 0;
	const char *config = fdt_getprop(gd->fdt_blob, 0, "config_name", NULL);

	sprintf((char *)dtb_config_name, "%s", config);

	ipq_smem_get_socinfo_version((uint32_t *)&soc_version);
	if(SOCINFO_VERSION_MAJOR(soc_version) >= 2) {
		sprintf((char *)dtb_config_name, "v%d.0-%s",
			SOCINFO_VERSION_MAJOR(soc_version), dtb_config_name);
	}

	if (fit_conf_get_node((void *)addr, config) >= 0) {
		snprintf(rcmd, rcmd_size, "bootm 0x%x#%s\n", addr, dtb_config_name);
		return 0;
	}
	printf("Config not availabale\n");
	return -1;
}

void dump_func()
{
#ifdef CONFIG_QCA_APPSBL_DLOAD
	uint64_t etime;
#endif
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
	return;
}


static int do_boot_signedimg(cmd_tbl_t *cmdtp, int flag, int argc, char *const argv[])
{
#ifdef CONFIG_QCA_APPSBL_DLOAD
	volatile u32 val;
	unsigned long * dmagic1 = (unsigned long *) 0x2A03F000;
	unsigned long * dmagic2 = (unsigned long *) 0x2A03F004;
#endif
	char runcmd[256];
	int ret;
	unsigned int request;
#ifdef CONFIG_QCA_MMC
	block_dev_desc_t *blk_dev;
	disk_partition_t disk_info;
	unsigned int active_part = 0;
#endif

	if (argc == 2 && strncmp(argv[1], "debug", 5) == 0)
		debug = 1;

#ifdef CONFIG_QCA_APPSBL_DLOAD

	ret = qca_scm_call(SCM_SVC_BOOT, SCM_SVC_RD, (void *)&val, sizeof(val));
	if (ret) {
		if (*dmagic1 == 0xE47B337D && *dmagic2 == 0x0501CAB0) {
			/* clear the magic and run the dump command */
			*dmagic1 = 0;
			*dmagic2 = 0;
			dump_func();
		}
	}
	else {
		/* check if we are in download mode */
		if (val == DLOAD_MAGIC_COOKIE) {
			/* clear the magic and run the dump command */
			val = 0x0;
			ret = qca_scm_call(SCM_SVC_BOOT, SCM_SVC_WR, (void *)&val, sizeof(val));
			if (ret)
				printf ("Error in reseting the Magic cookie\n");
			dump_func();
		}
	}
#endif
	if ((ret = set_fs_bootargs(&ipq_fs_on_nand)))
		return ret;

	/* check the smem info to see which flash used for booting */
	if (sfi->flash_type == SMEM_BOOT_SPI_FLASH) {
		if (debug) {
			printf("Using nand device %d\n", CONFIG_SPI_FLASH_INFO_IDX);
		}
		sprintf(runcmd, "nand device %d", CONFIG_SPI_FLASH_INFO_IDX);
		run_command(runcmd, 0);

	} else if (sfi->flash_type == SMEM_BOOT_NAND_FLASH) {
		if (debug) {
			printf("Using nand device 0\n");
		}
	} else if (sfi->flash_type == SMEM_BOOT_MMC_FLASH) {
		if (debug) {
			printf("Using MMC device\n");
		}
	} else {
		printf("Unsupported BOOT flash type\n");
		return -1;
	}
	if (debug) {
		run_command("printenv bootargs", 0);
		printf("Booting from flash\n");
	}

	request = CONFIG_SYS_LOAD_ADDR;
	kernel_img_info.kernel_load_addr = request;

	if (ipq_fs_on_nand) {
		if (sfi->rootfs.offset == 0xBAD0FF5E) {
			sfi->rootfs.offset = 0;
			sfi->rootfs.size = IPQ_NAND_ROOTFS_SIZE;
		}
		/*
		 * The kernel will be available inside a UBI volume
		 */
		snprintf(runcmd, sizeof(runcmd),
			 "nand device %d && "
			 "setenv mtdids nand%d=nand%d && "
			 "setenv mtdparts mtdparts=nand%d:0x%llx@0x%llx(fs),${msmparts} && "
			 "ubi part fs && "
			 "ubi read 0x%x kernel && ", is_spi_nand_available(),
			 is_spi_nand_available(),
			 is_spi_nand_available(),
			 is_spi_nand_available(),
			 sfi->rootfs.size, sfi->rootfs.offset,
			 request);

		if (debug)
			printf(runcmd);

		if (run_command(runcmd, 0) != CMD_RET_SUCCESS)
			return CMD_RET_FAILURE;

		kernel_img_info.kernel_load_size =
			(unsigned int)ubi_get_volume_size("kernel");
#ifdef CONFIG_QCA_MMC
	} else if (sfi->flash_type == SMEM_BOOT_MMC_FLASH || (is_nor_emmc_available() == 1)) {
		blk_dev = mmc_get_dev(host->dev_num);
		if (smem_bootconfig_info() == 0) {
			active_part = get_rootfs_active_partition();
			if (active_part) {
				ret = get_partition_info_efi_by_name(blk_dev,
						"0:HLOS_1", &disk_info);
			} else {
				ret = get_partition_info_efi_by_name(blk_dev,
						"0:HLOS", &disk_info);
			}
		} else {
			ret = get_partition_info_efi_by_name(blk_dev,
						"0:HLOS", &disk_info);
		}

		if (ret == 0) {
			snprintf(runcmd, sizeof(runcmd), "mmc read 0x%x 0x%X 0x%X",
				 CONFIG_SYS_LOAD_ADDR,
				 (uint)disk_info.start, (uint)disk_info.size);

			if (run_command(runcmd, 0) != CMD_RET_SUCCESS)
				return CMD_RET_FAILURE;

			kernel_img_info.kernel_load_size = disk_info.size * disk_info.blksz;
		}
#endif
	} else {
		/*
		 * Kernel is in a separate partition
		 */
		snprintf(runcmd, sizeof(runcmd),
			 /* NOR is treated as psuedo NAND */
			 "nand read 0x%x 0x%llx 0x%llx && ",
			 request, sfi->hlos.offset, sfi->hlos.size);

		if (debug)
			printf(runcmd);

		if (run_command(runcmd, 0) != CMD_RET_SUCCESS)
			return CMD_RET_FAILURE;

		kernel_img_info.kernel_load_size =  sfi->hlos.size;
	}

	setenv("mtdids", mtdids);

	request += sizeof(mbn_header_t);

	ret = qca_scm_call(SCM_SVC_BOOT, KERNEL_AUTH_CMD,
			(void *)&kernel_img_info, sizeof(kernel_img_info_t));

	if (ret) {
		printf("Kernel image authentication failed \n");
		BUG();
	}

	dcache_enable();

	ret = config_select(request, runcmd, sizeof(runcmd));

	if (debug)
		printf(runcmd);

#ifdef CONFIG_QCA_MMC
	board_mmc_deinit();
#endif

	if (ret < 0 || run_command(runcmd, 0) != CMD_RET_SUCCESS) {
#ifdef CONFIG_QCA_MMC
		mmc_initialize(gd->bd);
#endif
		dcache_disable();
		return CMD_RET_FAILURE;
	}

#ifndef CONFIG_QCA_APPSBL_DLOAD
	reset_crashdump();
#endif
	return CMD_RET_SUCCESS;
}

static int do_boot_unsignedimg(cmd_tbl_t *cmdtp, int flag, int argc, char *const argv[])
{
#ifdef CONFIG_QCA_APPSBL_DLOAD
	volatile u32 val;
	unsigned long * dmagic1 = (unsigned long *) 0x2A03F000;
	unsigned long * dmagic2 = (unsigned long *) 0x2A03F004;
#endif
	int ret;
	char runcmd[256];
#ifdef CONFIG_QCA_MMC
	block_dev_desc_t *blk_dev;
	disk_partition_t disk_info;
	unsigned int active_part = 0;
#endif

	if (argc == 2 && strncmp(argv[1], "debug", 5) == 0)
		debug = 1;
#ifdef CONFIG_QCA_APPSBL_DLOAD
	ret = qca_scm_call(SCM_SVC_BOOT, SCM_SVC_RD, (void *)&val, sizeof(val));
	if (ret) {
		if (*dmagic1 == 0xE47B337D && *dmagic2 == 0x0501CAB0) {
		/* clear the magic and run the dump command */
			*dmagic1 = 0;
			*dmagic2 = 0;
			dump_func();
		}
	}
	else {
		/* check if we are in download mode */
		if (val == DLOAD_MAGIC_COOKIE) {
			/* clear the magic and run the dump command */
			val = 0x0;
			ret = qca_scm_call(SCM_SVC_BOOT, SCM_SVC_WR, (void *)&val, sizeof(val));
			if (ret)
				printf ("Error in reseting the Magic cookie\n");
			dump_func();
		}
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
		if (sfi->rootfs.offset == 0xBAD0FF5E) {
			printf(" bad offset of hlos");
			return -1;
		}

		snprintf(runcmd, sizeof(runcmd),
			 "setenv mtdids nand0=nand0 && "
			 "setenv mtdparts mtdparts=nand0:0x%llx@0x%llx(fs),${msmparts} && "
			 "ubi part fs && "
			 "ubi read 0x%x kernel && ",
			 sfi->rootfs.size, sfi->rootfs.offset,
			 CONFIG_SYS_LOAD_ADDR);

	} else if ((sfi->flash_type == SMEM_BOOT_SPI_FLASH) && (is_nor_emmc_available() == 0)) {
		if ((sfi->rootfs.offset == 0xBAD0FF5E) ||
		    get_which_flash_param("rootfs")) {
			if (sfi->rootfs.offset == 0xBAD0FF5E) {
				sfi->rootfs.offset = 0;
				sfi->rootfs.size = IPQ_NAND_ROOTFS_SIZE;
			}
			snprintf(runcmd, sizeof(runcmd),
				 "nand device %d && "
				 "setenv mtdids nand%d=nand%d && "
				 "setenv mtdparts mtdparts=nand%d:0x%llx@0x%llx(fs),${msmparts} && "
				 "ubi part fs && "
				 "ubi read 0x%x kernel && ",
				 is_spi_nand_available(),
				 is_spi_nand_available(),
				 is_spi_nand_available(),
				 is_spi_nand_available(),
				 sfi->rootfs.size, sfi->rootfs.offset,
				 CONFIG_SYS_LOAD_ADDR);
		} else {
			/*
			 * Kernel is in a separate partition
			 */
			snprintf(runcmd, sizeof(runcmd),
				 "sf probe &&"
				 "sf read 0x%x 0x%x 0x%x && ",
				 CONFIG_SYS_LOAD_ADDR, (uint)sfi->hlos.offset, (uint)sfi->hlos.size);
		}
#ifdef CONFIG_QCA_MMC
	} else if ((sfi->flash_type == SMEM_BOOT_MMC_FLASH) || (is_nor_emmc_available() == 1)) {
		if (debug) {
			printf("Using MMC device\n");
		}
		blk_dev = mmc_get_dev(host->dev_num);
		if (smem_bootconfig_info() == 0) {
			active_part = get_rootfs_active_partition();
			if (active_part) {
				ret = get_partition_info_efi_by_name(blk_dev,
						"0:HLOS_1", &disk_info);
			} else {
				ret = get_partition_info_efi_by_name(blk_dev,
						"0:HLOS", &disk_info);
			}
		} else {
			ret = get_partition_info_efi_by_name(blk_dev,
						"0:HLOS", &disk_info);
		}

		if (ret == 0) {
			snprintf(runcmd, sizeof(runcmd), "mmc read 0x%x 0x%x 0x%x",
				 CONFIG_SYS_LOAD_ADDR,
				 (uint)disk_info.start, (uint)disk_info.size);

			if (run_command(runcmd, 0) != CMD_RET_SUCCESS)
				return CMD_RET_FAILURE;
		}

#endif   /* CONFIG_QCA_MMC   */
	} else {
		printf("Unsupported BOOT flash type\n");
		return -1;
	}

	if (run_command(runcmd, 0) != CMD_RET_SUCCESS) {
#ifdef CONFIG_QCA_MMC
		mmc_initialize(gd->bd);
#endif
		return CMD_RET_FAILURE;
	}

	dcache_enable();

	setenv("mtdids", mtdids);

	ret = genimg_get_format((void *)CONFIG_SYS_LOAD_ADDR);
	if (ret == IMAGE_FORMAT_FIT) {
		ret = config_select(CONFIG_SYS_LOAD_ADDR,
				    runcmd, sizeof(runcmd));
	} else if (ret == IMAGE_FORMAT_LEGACY) {
		snprintf(runcmd, sizeof(runcmd),
			 "bootm 0x%x\n", CONFIG_SYS_LOAD_ADDR);
	} else {
		ret = genimg_get_format((void *)CONFIG_SYS_LOAD_ADDR +
					sizeof(mbn_header_t));
		if (ret == IMAGE_FORMAT_FIT) {
			ret = config_select((CONFIG_SYS_LOAD_ADDR
					     + sizeof(mbn_header_t)),
					    runcmd, sizeof(runcmd));
		} else if (ret == IMAGE_FORMAT_LEGACY) {
			snprintf(runcmd, sizeof(runcmd),
				 "bootm 0x%x\n", (CONFIG_SYS_LOAD_ADDR +
						  sizeof(mbn_header_t)));
		} else {
			dcache_disable();
			return CMD_RET_FAILURE;
		}
	}

	if (ret < 0 || run_command(runcmd, 0) != CMD_RET_SUCCESS) {
		dcache_disable();
		return CMD_RET_FAILURE;
	}

#ifndef CONFIG_QCA_APPSBL_DLOAD
	reset_crashdump();
#endif
	return CMD_RET_SUCCESS;
}

static int do_bootipq(cmd_tbl_t *cmdtp, int flag, int argc, char *const argv[])
{
	int ret;
	char buf;
	/*
	 * set fdt_high parameter so that u-boot will not load
	 * dtb above CONFIG_IPQ40XX_FDT_HIGH region.
	 */
	if (run_command("setenv fdt_high " MK_STR(CONFIG_IPQ_FDT_HIGH) "\n", 0)
	    != CMD_RET_SUCCESS) {
		return CMD_RET_FAILURE;
	}

	ret = qca_scm_call(SCM_SVC_FUSE, QFPROM_IS_AUTHENTICATE_CMD, &buf, sizeof(char));

	if (ret == 0 && buf == 1) {
		return do_boot_signedimg(cmdtp, flag, argc, argv);
	} else if (ret == 0 || ret == -EOPNOTSUPP) {
		return do_boot_unsignedimg(cmdtp, flag, argc, argv);
	}

	return CMD_RET_FAILURE;
}

U_BOOT_CMD(bootipq, 2, 0, do_bootipq,
	   "bootipq from flash device",
	   "bootipq [debug] - Load image(s) and boots the kernel\n");
