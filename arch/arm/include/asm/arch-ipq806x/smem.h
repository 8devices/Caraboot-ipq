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
unsigned int smem_get_board_machtype(void);

typedef struct {
	loff_t offset;
	loff_t size;
} ipq_part_entry_t;

typedef struct {
	uint32_t		flash_type;
	uint32_t		flash_index;
	uint32_t		flash_chip_select;
	uint32_t		flash_block_size;
	ipq_part_entry_t	hlos;
	ipq_part_entry_t	nss[2];
} ipq_smem_flash_info_t;

extern ipq_smem_flash_info_t ipq_smem_flash_info;

void ipq_set_part_entry(ipq_smem_flash_info_t *sfi, ipq_part_entry_t *part,
			uint32_t start, uint32_t size);
#endif
