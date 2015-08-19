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
#include <nand.h>
#include <linux/mtd/nand.h>
#include <spi_flash.h>
#include <asm/errno.h>
#include "spi_flash_internal.h"
#include "spi_nand_dev.h"

#define CONFIG_SF_DEFAULT_SPEED		(48 * 1000 * 1000)
#define TIMEOUT		5000
#define MFID_ATO	0x9b

#define spi_print(...)  printf("spi_nand: " __VA_ARGS__)

struct nand_chip nand_chip[CONFIG_SYS_MAX_NAND_DEVICE];

#define mtd_to_ipq_info(m)	((struct nand_chip *)((m)->priv))->priv

static const struct spi_nand_flash_params spi_nand_flash_tbl[] = {
	{
		.mid = 0xc8,
		.devid = 0xb148,
		.page_size = 2048,
		.erase_size = 0x00020000,
		.pages_per_sector = 64,
		.nr_sectors = 1024,
		.oob_size = 64,
		.ecc_mask = 0x70,
		.ecc_err = 0x70,
		.name = "GD5F1GQ4XC",
	},
	{
		.mid = 0x9b,
		.devid = 0x12,
		.page_size = 2048,
		.erase_size = 0x00020000,
		.pages_per_sector = 64,
		.nr_sectors = 1024,
		.oob_size = 64,
		.ecc_mask = 0,
		.ecc_err = 0,
		.name = "ATO25D1GA",
	},
};

const struct spi_nand_flash_params *params;

static int spinand_waitfunc(struct mtd_info *mtd, u8 val, u8 *status)
{
	struct ipq40xx_spinand_info *info = mtd_to_ipq_info(mtd);
	struct spi_flash *flash = info->flash;
	int ret;

	ret = spi_nand_flash_cmd_wait_ready(flash, val, status, TIMEOUT);
	if (ret) {
		printf("%s Operation Timeout\n", __func__);
		return -1;
	}

	return 0;
}

static int check_offset(struct mtd_info *mtd, loff_t offs)
{
	struct ipq40xx_spinand_info *info = mtd_to_ipq_info(mtd);
	struct spi_flash *flash = info->flash;
	struct nand_chip *chip = info->chip;
	int ret = 0;

	/* Start address must align on block boundary */
	if (offs & ((1 << chip->phys_erase_shift) - 1)) {
		printf("%s: unaligned address\n", __func__);
		ret = -EINVAL;
	}

	return ret;
}

static int spi_nand_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	u8 cmd[8], len;
	u8 status;
	u32 ret;
	u32 row_addr;
	struct ipq40xx_spinand_info *info = mtd_to_ipq_info(mtd);
	struct spi_flash *flash = info->flash;
	struct nand_chip *chip = info->chip;
	u32 page = (int)(instr->addr >> chip->page_shift);

	if (check_offset(mtd, instr->addr))
		return -1;

	ret = spi_claim_bus(flash->spi);
	if (ret) {
		printf("SF: Unable to claim SPI bus\n");
		return ret;
	}

	ret = spi_flash_cmd(flash->spi, IPQ40XX_SPINAND_CMD_WREN, NULL, 0);
	if (ret) {
		printf ("Write enable failed %s\n", __func__);
		goto out;
	}
	ret = spinand_waitfunc(mtd, 0x01, &status);
	if (ret) {
		goto out;
	}

	cmd[0] = IPQ40XX_SPINAND_CMD_ERASE;
	cmd[1] = (u8)(page >> 16);
	cmd[2] = (u8)(page >> 8);
	cmd[3] = (u8)(page);
	len = 4;
	ret = spi_flash_cmd_write(flash->spi, cmd, 4, NULL, 0);
	if (ret) {
		printf("%s  failed for offset:%x \n", __func__, instr->addr);
		goto out;
	}
	ret = spinand_waitfunc(mtd, 0x01, &status);
	if (ret) {
		printf("Operation timeout\n");
		goto out;
	}

	if (status & STATUS_E_FAIL) {
		printf("Erase operation failed for 0x%x\n", page);
		ret = -EIO;
		goto out;
	}

	ret = spi_flash_cmd(flash->spi, IPQ40XX_SPINAND_CMD_WRDI, NULL, 0);

	ret = spinand_waitfunc(mtd, 0x01, &status);
	if (ret) {
		printf("%s: Write disable failed\n");
	}

out:
	spi_release_bus(flash->spi);

	return ret;
}

static int spi_nand_block_isbad(struct mtd_info *mtd, loff_t offs)
{
	struct ipq40xx_spinand_info *info = mtd_to_ipq_info(mtd);
	struct spi_flash *flash = info->flash;
	struct nand_chip *chip = info->chip;
	u8 cmd[8];
	u8 status;
	u32 value = 0xff;
	int page, column, ret;

	page = (int)(offs >> chip->page_shift) & chip->pagemask;
	column = mtd->writesize + chip->badblockpos;

	ret = spi_claim_bus(flash->spi);
	if (ret) {
		printf ("Claim bus failed. %s\n", __func__);
		return -1;
	}

	cmd[0] = IPQ40XX_SPINAND_CMD_READ;
	cmd[1] = (u8)(page >> 16);
	cmd[2] = (u8)(page >> 8);
	cmd[3] = (u8)(page);
	ret = spi_flash_cmd_write(flash->spi, cmd, 4, NULL, 0);
	if (ret) {
		printf("%s: write command failed\n", __func__);
		goto out;
	}
	ret = spinand_waitfunc(mtd, 0x01, &status);
	if (ret) {
		printf("Operation timeout\n");
		goto out;
	}

	cmd[0] = IPQ40XX_SPINAND_CMD_NORM_READ;
	cmd[1] = 0;
	cmd[2] = (u8)(column >> 8);
	cmd[3] = (u8)(column);
	ret = spi_flash_cmd_read(flash->spi, cmd, 4, &value, 1);
	if (ret) {
		printf("%s: read data failed\n", __func__);
		goto out;
	}

	if (value != 0xFF) {
		ret = 1;
		goto out;
	}

out:
	spi_release_bus(flash->spi);
	return ret;
}

static int spi_nand_read_oob(struct mtd_info *mtd, loff_t from,
			     struct mtd_oob_ops *ops)
{
	return -EINVAL;
}

static int spinand_write_oob_std(struct mtd_info *mtd, struct nand_chip *chip,  int page)
{
	int column, len, status, ret = 0, bytes;
	u_char *wbuf;
	u8 cmd[8];
	struct ipq40xx_spinand_info *info = mtd_to_ipq_info(mtd);
	struct spi_flash *flash = info->flash;

	wbuf = chip->oob_poi;
	column = mtd->writesize;

	cmd[0] = IPQ40XX_SPINAND_CMD_PLOAD;
	cmd[1] = (u8)(column >> 8);
	cmd[2] = (u8)(column);

	ret = spi_flash_cmd_write(flash->spi, cmd, 3, wbuf, 2);
	if (ret) {
		printf("%s: write command failed\n", __func__);
		ret = 1;
		goto out;
	}

	ret = spi_flash_cmd(flash->spi, IPQ40XX_SPINAND_CMD_WREN, NULL, 0);
	if (ret) {
		printf("Write enable failed in %s\n", __func__);
		goto out;
	}

	cmd[0] = IPQ40XX_SPINAND_CMD_PROG;
	cmd[1] = (u8)(page >> 16);
	cmd[2] = (u8)(page >> 8);
	cmd[3] = (u8)(page);

	ret = spi_flash_cmd_write(flash->spi, cmd, 4, NULL, 0);
	if (ret) {
		printf("PLOG failed\n");
		goto out;
	}

	ret = spinand_waitfunc(mtd, 0x01, &status);
	if (ret) {
		if (status)
			printf("Program failed\n");
	}

	ret = spi_flash_cmd(flash->spi, IPQ40XX_SPINAND_CMD_WRDI, NULL, 0);
	if (ret)
		printf("Write disable failed in %s\n", __func__);

out:
	spi_release_bus(flash->spi);
	return ret;
}

static void fill_oob_data(struct mtd_info *mtd, uint8_t *oob,
        size_t len, struct mtd_oob_ops *ops)
{
	struct ipq40xx_spinand_info *info = mtd_to_ipq_info(mtd);
	struct nand_chip *chip = info->chip;

	memset(chip->oob_poi, 0xff, mtd->oobsize);
	memcpy(chip->oob_poi + ops->ooboffs, oob, len);

	return;
}

static int spi_nand_write_oob(struct mtd_info *mtd, loff_t to,
			      struct mtd_oob_ops *ops)
{
	int page, ret;
	struct ipq40xx_spinand_info *info = mtd_to_ipq_info(mtd);
	struct spi_flash *flash = info->flash;
	struct nand_chip *chip = info->chip;

	/* Shift to get page */
	page = (int)(to >> chip->page_shift);

	fill_oob_data(mtd, ops->oobbuf, ops->ooblen, ops);

	return spinand_write_oob_std(mtd, chip, page & chip->pagemask);
}

static int spi_nand_block_markbad(struct mtd_info *mtd, loff_t offs)
{
	uint8_t buf[2]= { 0, 0 };
	int block, ret;
	struct ipq40xx_spinand_info *info = mtd_to_ipq_info(mtd);
	struct spi_flash *flash = info->flash;
	struct nand_chip *chip = info->chip;
	struct mtd_oob_ops ops;
	struct erase_info einfo;

	ret = spi_nand_block_isbad(mtd, offs);
	if (ret) {
		if (ret > 0)
			return 0;
		return ret;
	}

	/* Attempt erase before marking OOB */
	memset(&einfo, 0, sizeof(einfo));
	einfo.mtd = mtd;
	einfo.addr = offs;
	einfo.len = 1 << chip->phys_erase_shift;
	spi_nand_erase(mtd, &einfo);

	ret = spi_claim_bus(flash->spi);
	if (ret) {
		printf ("Claim bus failed. %s\n", __func__);
		return -1;
	}

	ops.datbuf = NULL;
	ops.oobbuf = buf;
	ops.ooboffs = chip->badblockpos;
	ops.len = ops.ooblen = 1;

	ret = spi_nand_write_oob(mtd, offs,&ops);
	if (ret)
		goto out;

out:
	spi_release_bus(flash->spi);
	return ret;
}

static int spi_nand_read(struct mtd_info *mtd, loff_t from, size_t len,
			 size_t *retlen, u_char *buf)
{
	struct ipq40xx_spinand_info *info = mtd_to_ipq_info(mtd);
	struct spi_flash *flash = info->flash;
	u32 ret;
	u8 cmd[8];
	u8 status;
	int realpage, page, readlen, bytes;
	int ecc_corrected = 0;

	realpage = (int)(from >> 0xB);
	page = realpage & 0xffff;
	readlen = len;

	ret = spi_claim_bus(flash->spi);
	if (ret) {
		printf ("Claim bus failed. %s\n", __func__);
		return -1;
	}
	while (1) {
		cmd[0] = IPQ40XX_SPINAND_CMD_READ;
		cmd[1] = (u8)(page >> 16);
		cmd[2] = (u8)(page >> 8);
		cmd[3] = (u8)(page);
		ret = spi_flash_cmd_write(flash->spi, cmd, 4, NULL, 0);
		if (ret) {
			printf("%s: write command failed\n", __func__);
			goto out;
		}
		ret = spinand_waitfunc(mtd, 0x01, &status);
		if (ret) {
			goto out;
		}

		if (info->params->ecc_mask != 0) {
			if ((status & info->params->ecc_mask) ==
						info->params->ecc_err) {
				mtd->ecc_stats.failed++;
				printf("ecc err(0x%x) for page read\n", status);
				ret = -EBADMSG;
				goto out;
			} else if (status & info->params->ecc_mask) {
				mtd->ecc_stats.corrected++;
				ecc_corrected = 1;
			}
		}

		bytes = ((readlen < mtd->writesize) ? readlen : mtd->writesize);
		cmd[0] = IPQ40XX_SPINAND_CMD_NORM_READ;
		cmd[2] = 0;
		cmd[3] = 0;
		ret = spi_flash_cmd_read(flash->spi, cmd, 4, buf, bytes);
		if (ret) {
			printf("%s: read data failed\n", __func__);
			return -1;
		}

		readlen -= bytes;
		if (readlen <= 0)
			break;
		buf += bytes;
		realpage++;
		page = realpage & 0xffff;
	}

	if ((ret == 0) && (ecc_corrected))
		ret = -EUCLEAN;
out:
	spi_release_bus(flash->spi);
	return ret;
}

static int spi_nand_write(struct mtd_info *mtd, loff_t to, size_t len,
			  size_t *retlen, const u_char *buf)
{
	struct ipq40xx_spinand_info *info = mtd_to_ipq_info(mtd);
	struct spi_flash *flash = info->flash;
	u8 cmd[8];
	u8 status;
	u32 ret;
	u_char *wbuf;
	int realpage, page, bytes, write_len;
	write_len = len;
	bytes = mtd->writesize;

	/* Check whether page is aligned */
	if (((to & (mtd->writesize -1)) !=0) ||
		((len & (mtd->writesize -1)) != 0)) {
		printf("Attempt to write to non page aligned data\n");
		return -EINVAL;
	}

	realpage = (int)(to >> 0xb);
	page = realpage & 0xffff;

	ret = spi_claim_bus(flash->spi);
	if (ret) {
		printf ("Write enable failed %s\n", __func__);
		return -1;
	}
	while (1) {
		wbuf = buf;
		/* buffer to be transmittted here */
		cmd[0] = IPQ40XX_SPINAND_CMD_PLOAD;
		cmd[1] = 0;
		cmd[2] = 0;
		ret = spi_flash_cmd_write(flash->spi, cmd, 3, wbuf, bytes);
		if (ret) {
			printf("%s: write command failed\n", __func__);
			goto out;
		}

		ret = spi_flash_cmd(flash->spi, IPQ40XX_SPINAND_CMD_WREN, NULL, 0);
		if (ret) {
			printf("Write enable failed\n");
			goto out;
		}

		cmd[0] = IPQ40XX_SPINAND_CMD_PROG;
		cmd[1] = (u8)(page >> 16);
		cmd[2] = (u8)(page >> 8);
		cmd[3] = (u8)(page);

		ret = spi_flash_cmd_write(flash->spi, cmd, 4, NULL, 0);
		if (ret) {
			printf("PLOG failed\n");
			goto out;
		}

		ret = spinand_waitfunc(mtd, 0x01, &status);
		if (ret) {
			printf("Operation timeout\n");
			goto out;
		}

		if (status & STATUS_P_FAIL) {
			printf("Program failure (0x%x)\n", status);
			ret = -EIO;
			goto out;
		}

		ret = spi_flash_cmd(flash->spi, IPQ40XX_SPINAND_CMD_WRDI, NULL, 0);
		if (ret) {
			printf("Write disable failed\n");
			goto out;
		}

		write_len -= bytes;
		if (!write_len)
			break;
		buf += bytes;
		realpage++;
		page = realpage & 0xffff;

	}

out:
	spi_release_bus(flash->spi);
	return ret;
}


struct spi_flash *spi_nand_flash_probe(struct spi_slave *spi,
                                                u8 *idcode)
{
	struct spi_flash *flash;
	unsigned int i;
	u16 devid;
	u32 mfid;

	mfid = idcode[0];

	if (mfid == MFID_ATO)
		devid = idcode[1];
	else
		devid = (idcode[1] << 8 | idcode[2]);

	for (i = 0; i < ARRAY_SIZE(spi_nand_flash_tbl); i++) {
		params = &spi_nand_flash_tbl[i];
		if (params->mid == mfid) {
			spi_print ("%s SF NAND MFID%x\n",
				__func__, mfid);
			if (params->devid == devid) {
				spi_print ("%s SF NAND dev ID %x\n",
					__func__, devid);
				break;
			}
		}
	}

	if (i == ARRAY_SIZE(spi_nand_flash_tbl)) {
		printf ("SF NAND unsupported Mfid:%04x devid:%04x",
			mfid, devid);
		return NULL;
	}

	flash = malloc(sizeof (*flash));
	if (!flash) {
		printf ("SF Failed to allocate memeory\n");
		return NULL;
	}

	flash->spi = spi;

	flash->name = params->name;
	flash->page_size = params->page_size;
	flash->sector_size = params->page_size;
	flash->size = (params->page_size * params->nr_sectors * params->pages_per_sector);

	return flash;
}

static int spinand_unlock_protect(struct mtd_info *mtd)
{
	struct ipq40xx_spinand_info *info = mtd_to_ipq_info(mtd);
	struct spi_flash *flash = info->flash;
	int status;
	int ret;
	u8 cmd[3];

	ret = spi_claim_bus(flash->spi);
	if (ret) {
		printf ("Write enable failed %s %d\n", __func__, __LINE__);
		return -1;
	}

	cmd[0] = IPQ40XX_SPINAND_CMD_GETFEA;
	cmd[1] = IPQ40XX_SPINAND_PROTEC_REG;

	ret = spi_flash_cmd_write(flash->spi, cmd, 2, status, 1);
	if (ret) {
		printf("Failed to read status register");
		goto out;
	}

	status &= IPQ40XX_SPINAND_PROTEC_BPx;
	cmd[0] = IPQ40XX_SPINAND_CMD_SETFEA;
	cmd[1] = IPQ40XX_SPINAND_PROTEC_REG;
	cmd[2] = (u8)status;
	ret = spi_flash_cmd_write(flash->spi, cmd, 3, NULL, 0);
	if (ret) {
		printf("Failed to unblock sectors\n");
	}
out:
	spi_release_bus(flash->spi);
	return ret;
}

void spinand_internal_ecc(struct mtd_info *mtd, int enable)
{
	struct ipq40xx_spinand_info *info = mtd_to_ipq_info(mtd);
	struct spi_flash *flash = info->flash;
	int status;
	int ret;
	u8 cmd[3];

	ret = spi_claim_bus(flash->spi);
	if (ret) {
		printf ("Write enable failed %s %d\n", __func__, __LINE__);
		return -1;
	}

	cmd[0] = IPQ40XX_SPINAND_CMD_SETFEA;
	cmd[1] = IPQ40XX_SPINAND_FEATURE_REG;
	if (enable)
		cmd[2] = IPQ40XX_SPINAND_FEATURE_ECC_EN;
	else
		cmd[2] = 0;

	ret = spi_flash_cmd_write(flash->spi, cmd, 3, NULL, 0);
	if (ret) {
		printf("Internal ECC enable failed\n");
	}

	spi_release_bus(flash->spi);
return;
}

static int spi_nand_scan_bbt_nop(struct mtd_info *mtd)
{
	return 0;
}

int spi_nand_init(void)
{
	struct spi_flash *flash;
	struct mtd_info *mtd;
	struct nand_chip *chip;
	struct ipq40xx_spinand_info *info;
	int ret;

	info = malloc (sizeof (struct ipq40xx_spinand_info));
	if (!info) {
		printf ("Error in allocating mem\n");
		return -ENOMEM;
	}

	flash = spi_flash_probe(CONFIG_SF_DEFAULT_BUS,
				CONFIG_SF_SPI_NAND_CS,
				CONFIG_SF_DEFAULT_SPEED,
				CONFIG_SF_DEFAULT_MODE);
	if (!flash) {
		spi_print("Id could not be mapped\n");
		return NULL;
	}

	mtd = &nand_info[CONFIG_IPQ_SPI_NAND_INFO_IDX];
	chip = &nand_chip[CONFIG_IPQ_SPI_NAND_INFO_IDX];

	mtd->priv = chip;
	mtd->writesize = flash->page_size;
	mtd->erasesize = params->erase_size;
	mtd->oobsize = params->oob_size;
	mtd->size = flash->size;
	mtd->type = MTD_NANDFLASH;
	mtd->flags = MTD_CAP_NANDFLASH;
	mtd->point = NULL;
	mtd->unpoint = NULL;
	mtd->read = spi_nand_read;
	mtd->write = spi_nand_write;
	mtd->erase = spi_nand_erase;
	mtd->read_oob = spi_nand_read_oob;
	mtd->write_oob = spi_nand_write_oob;
	mtd->block_isbad = spi_nand_block_isbad;
	mtd->block_markbad = spi_nand_block_markbad;

	chip->page_shift = ffs(mtd->writesize) - 1;
	chip->phys_erase_shift = ffs(mtd->erasesize) - 1;
	chip->chipsize = flash->size;
	chip->pagemask = (chip->chipsize >> chip->page_shift) - 1;
	chip->badblockpos = 0;
	chip->oob_poi = mtd->writesize;

	/* One of the NAND layer functions that the command layer
	 * tries to access directly.
	 */
	chip->scan_bbt = spi_nand_scan_bbt_nop;

	info->params = params;
	info->flash = flash;
	info->mtd = mtd;
	info->chip = chip;
	chip->priv = info;

	if ((ret = nand_register(CONFIG_IPQ_SPI_NAND_INFO_IDX)) < 0) {
		spi_print("Failed to register with MTD subsystem\n");
		return ret;
	}

	ret = spinand_unlock_protect(mtd);
	if (ret) {
		printf("Failed to unlock blocks\n");
		return -1;
	}

	spinand_internal_ecc(mtd, 1);

	return 0;
}

