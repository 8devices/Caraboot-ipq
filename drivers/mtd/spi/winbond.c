/*
 * Copyright 2008, Network Appliance Inc.
 * Author: Jason McMullan <mcmullan <at> netapp.com>
 * Licensed under the GPL-2 or later.
 */

#include <common.h>
#include <malloc.h>
#include <spi_flash.h>

#include "spi_flash_internal.h"

#define CMD_BLOCK_ERASE		0xd8	/* Block Erase */
#define CMD_BLOCK_4B_ERASE	0xdc	/* 4 byte Block Erase */
#define CMD_SECTOR_4B_ERASE	0x21	/* 4 byte Sector Erase */
#define CMD_SECTOR_ERASE	0x20	/* Sector Erase */

struct winbond_spi_flash_params {
	uint16_t	id;
	/* Log2 of page size in power-of-two mode */
	uint8_t		l2_page_size;
	uint16_t	pages_per_sector;
	uint16_t	sectors_per_block;
	uint16_t	nr_blocks;
	const char	*name;
};

static const struct winbond_spi_flash_params winbond_spi_flash_table[] = {
	{
		.id			= 0x3013,
		.l2_page_size		= 8,
		.pages_per_sector	= 16,
		.sectors_per_block	= 16,
		.nr_blocks		= 8,
		.name			= "W25X40",
	},
	{
		.id			= 0x3015,
		.l2_page_size		= 8,
		.pages_per_sector	= 16,
		.sectors_per_block	= 16,
		.nr_blocks		= 32,
		.name			= "W25X16",
	},
	{
		.id			= 0x3016,
		.l2_page_size		= 8,
		.pages_per_sector	= 16,
		.sectors_per_block	= 16,
		.nr_blocks		= 64,
		.name			= "W25X32",
	},
	{
		.id			= 0x3017,
		.l2_page_size		= 8,
		.pages_per_sector	= 16,
		.sectors_per_block	= 16,
		.nr_blocks		= 128,
		.name			= "W25X64",
	},
	{
		.id			= 0x4014,
		.l2_page_size		= 8,
		.pages_per_sector	= 16,
		.sectors_per_block	= 16,
		.nr_blocks		= 16,
		.name			= "W25Q80BL",
	},
	{
		.id			= 0x4015,
		.l2_page_size		= 8,
		.pages_per_sector	= 16,
		.sectors_per_block	= 16,
		.nr_blocks		= 32,
		.name			= "W25Q16",
	},
	{
		.id			= 0x4016,
		.l2_page_size		= 8,
		.pages_per_sector	= 16,
		.sectors_per_block	= 16,
		.nr_blocks		= 64,
		.name			= "W25Q32",
	},
	{
		.id			= 0x4017,
		.l2_page_size		= 8,
		.pages_per_sector	= 16,
		.sectors_per_block	= 16,
		.nr_blocks		= 128,
		.name			= "W25Q64",
	},
	{
		.id			= 0x4018,
		.l2_page_size		= 8,
		.pages_per_sector	= 256,
		.sectors_per_block	= 1,
		.nr_blocks		= 256,
		.name			= "W25Q128",
	},
	{
		.id                     = 0x6016,
		.l2_page_size           = 8,
		.pages_per_sector       = 256,
		.sectors_per_block      = 1,
		.nr_blocks              = 64,
		.name                   = "W25Q32",
	},
	{
		.id                     = 0x6017,
		.l2_page_size           = 8,
		.pages_per_sector       = 256,
		.sectors_per_block      = 1,
		.nr_blocks              = 128,
		.name                   = "W25Q64",
	},
	{
		.id                     = 0x4019,
		.l2_page_size           = 8,
		.pages_per_sector       = 256,
		.sectors_per_block      = 1,
		.nr_blocks              = 512,
		.name                   = "W25Q256",
	},

};

static int winbond_erase(struct spi_flash *flash, u32 offset, size_t len)
{
	u8 erase_opcode;
	
	if ((offset % flash->block_size) == 0 && (len % flash->block_size) == 0) {
		if ((flash->addr_width == 4))
			erase_opcode = CMD_BLOCK_4B_ERASE;
		else
			erase_opcode = CMD_BLOCK_ERASE;

		return spi_flash_cmd_erase_block(flash, erase_opcode, offset, len);
	}
	else {
		if ((flash->addr_width == 4))
			erase_opcode = CMD_SECTOR_4B_ERASE;
		else
			erase_opcode = CMD_SECTOR_ERASE;

		return spi_flash_cmd_erase(flash, erase_opcode, offset, len);
	}
}

struct spi_flash *spi_flash_probe_winbond(struct spi_slave *spi, u8 *idcode)
{
	const struct winbond_spi_flash_params *params;
	struct spi_flash *flash;
	unsigned int i;
	unsigned page_size;
	int ret;

	for (i = 0; i < ARRAY_SIZE(winbond_spi_flash_table); i++) {
		params = &winbond_spi_flash_table[i];
		if (params->id == ((idcode[1] << 8) | idcode[2]))
			break;
	}

	if (i == ARRAY_SIZE(winbond_spi_flash_table)) {
		debug("SF: Unsupported Winbond ID %02x%02x\n",
				idcode[1], idcode[2]);
		return NULL;
	}

	flash = malloc(sizeof(*flash));
	if (!flash) {
		debug("SF: Failed to allocate memory\n");
		return NULL;
	}

	flash->spi = spi;
	flash->name = params->name;

	/* Assuming power-of-two page size initially. */
	page_size = 1 << params->l2_page_size;

	flash->write = spi_flash_cmd_write_multi;
	flash->erase = winbond_erase;
	flash->read = spi_flash_cmd_read_fast;
	flash->page_size = page_size;
	flash->sector_size = page_size * params->pages_per_sector;
	flash->block_size = flash->sector_size * params->sectors_per_block;
	flash->size = page_size * params->pages_per_sector
				* params->sectors_per_block
				* params->nr_blocks;

	flash->read_opcode  = CMD_READ_ARRAY_FAST;
	flash->write_opcode = CMD_PAGE_PROGRAM;

	if (flash->size > 0x1000000) {
		flash->read_opcode  = CMD_4READ_ARRAY_FAST;
		flash->write_opcode = CMD_4PAGE_PROGRAM;
	}

	return flash;
}
