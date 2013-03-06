#ifndef SMEM_H
#define SMEM_H

enum {
	SMEM_BOOT_NO_FLASH        = 0,
	SMEM_BOOT_NOR_FLASH       = 1,
	SMEM_BOOT_NAND_FLASH      = 2,
	SMEM_BOOT_ONENAND_FLASH   = 3,
	SMEM_BOOT_SDC_FLASH       = 4,
	SMEM_BOOT_MMC_FLASH       = 5,
	SMEM_BOOT_SPI_FLASH       = 6,
};

int smem_ptable_init(void);
int smem_get_boot_flash(uint32_t *flash_type,
			uint32_t *flash_index,
			uint32_t *flash_chip_select,
			uint32_t *flash_block_size);
int smem_getpart(char *name, uint32_t *start, uint32_t *size);

#endif
