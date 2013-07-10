/* * Copyright (c) 2012 Qualcomm Atheros, Inc. * */


#ifndef  _IPQ806X_CDP_H_
#define  _IPQ806X_CDP_H_

void configure_uart_gpio(void);
unsigned int smem_get_board_machtype(void);

typedef struct {
	unsigned int gpio;
	unsigned int func;
	unsigned int dir;
	unsigned int pull;
	unsigned int drvstr;
	unsigned int enable;
} gpio_func_data_t;

typedef struct {
	unsigned int m_value;
	unsigned int n_value;
	unsigned int d_value;
} uart_clk_mnd_t;

/* SPI Mode */

typedef enum {
	NOR_SPI_MODE_0,
	NOR_SPI_MODE_1,
	NOR_SPI_MODE_2,
	NOR_SPI_MODE_3,
} spi_mode;

/* SPI GSBI Bus number */

typedef enum {
	GSBI_BUS_5 = 0,
	GSBI_BUS_6,
	GSBI_BUS_7,
} spi_gsbi_bus_num;

/* SPI Chip selects */

typedef enum {
	SPI_CS_0 ,
	SPI_CS_1,
	SPI_CS_2,
	SPI_CS_3,
} spi_cs;

/* Flash Types */

typedef enum {
	ONLY_NAND,
	ONLY_NOR,
	NAND_NOR,
} flash_desc;

#define NO_OF_DBG_UART_GPIOS	2

#define SPI_NOR_FLASH_VENDOR_MICRON       0x1
#define SPI_NOR_FLASH_VENDOR_SPANSION     0x2

/* SPI parameters */

typedef struct {
	spi_mode mode;
	spi_gsbi_bus_num bus_number;
	spi_cs chip_select;
	int vendor;
} spinorflash_params_t;

/* Board specific parameters */

typedef struct {
	unsigned int boardid;
	unsigned int machid;
	unsigned int ddr_size;
	unsigned int uart_gsbi;
	unsigned int uart_gsbi_base;
	unsigned int uart_dm_base;
	unsigned int clk_dummy;
	uart_clk_mnd_t mnd_value;
	flash_desc flashdesc;
	spinorflash_params_t flash_param;
	gpio_func_data_t dbg_uart_gpio[NO_OF_DBG_UART_GPIOS];
} __attribute__ ((__packed__)) board_ipq806x_params_t;

unsigned int get_board_index(unsigned int machid);
#endif
