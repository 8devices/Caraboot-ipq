/*
  Copyright (C) 2018 8devices
*/

#include <common.h>
#include <malloc.h>
#include <spi.h>
#include <spi_flash.h>

#define MACMAX 5

#define ETHX_MAC_OFFSET 0x0
#define WIFI0_MAC_OFFSET 0x1006
#define WIFI1_MAC_OFFSET 0x5006

#define WIFI0_OFFSET_PART 0x1000
#define WIFI1_OFFSET_PART 0x5000
#define CALDATA_LEN_OFFSET 0x0
#define CALDATA_CRC_OFFSET 0x2
#define CALDATA_MAC_OFFSET 0x6
#define CALDATA_SIZE_LARGEST 12064

#define ART_FLASH_OFFSET 0x170000
#define ART_FLASH_SIZE 0x8000

#define MACSTR "%02X:%02X:%02X:%02X:%02X:%02X"
#define MAC2STR(a) (uint8_t)(a)[0], (uint8_t)(a)[1], (uint8_t)(a)[2], \
		   (uint8_t)(a)[3], (uint8_t)(a)[4], (uint8_t)(a)[5]

int read_mac(unsigned char *buf, int offset, unsigned char mac[])
{
	memcpy(mac, buf + offset, 6);
	return 0;
}

static int update_board_data_crc(unsigned char *buf, int offset)
{
	int i;
	unsigned short *p_half;
	unsigned short sum;

	p_half = (unsigned short *)(buf+offset);

	if (*(p_half) != CALDATA_SIZE_LARGEST) {
		printf("Failed board data size validation, got %d, expected %d\n", *p_half, CALDATA_SIZE_LARGEST);
		return -1;
	}

	/* Recompute checksum */
	sum = *(p_half + 1) = 0x0000;
	for (i = 0; i < (CALDATA_SIZE_LARGEST/2); i++) { sum ^= *p_half++; }
	sum = 0xFFFF ^ sum;

	/* Update checksum */
	memcpy(buf + offset + CALDATA_CRC_OFFSET, &sum, 2);

	return 0;
}

static int write_mac(unsigned char *buf, int offset, unsigned char mac[])
{
	if (mac[0] == 0 && mac[1] == 0 && mac[2] == 0 &&
	    mac[3] == 0 && mac[4] == 0 && mac[5] == 0) {
		return 0;
	}

	memcpy(buf + offset, mac, 6);

	return 0;
}

static int parse_mac(const char *arg, unsigned char *mac)
{
	int i;
	char *p = (char *) arg;

	if (!strcmp(arg, "-")) {
		return 0;
	}

	for (i = 0; *p && (i < 6); i++) {
		mac[i] = simple_strtoul(p, &p, 16);
		if (*p == ':')
			p++;
	}

	return i == 6 ? 0 : -1;
}

static int getmacs(void)
{
	int i, ret;
	struct spi_flash *flash;
	unsigned char *buf;
	unsigned char wifi0[6];
	unsigned char wifi1[6];
	unsigned char eths[6];

	flash = spi_flash_probe(0, 0, 1000000, SPI_MODE_3);
	if (!flash) {
		printf("Failed to initialize SPI flash\n");
		return 1;
	}
	buf = malloc(ART_FLASH_SIZE);
	if (!buf) {
		printf("Memory alloc failed\n");
		return 1;
	}
	ret = flash->read(flash, ART_FLASH_OFFSET, ART_FLASH_SIZE, buf);
	if (ret) {
		printf("SPI flash read failed\n");
		free(buf);
		return 1;
	}

	read_mac(buf, WIFI0_MAC_OFFSET, wifi0);
	read_mac(buf, WIFI1_MAC_OFFSET, wifi1);

	printf("wifi0: "MACSTR" \n", MAC2STR(wifi0));
	printf("wifi1: "MACSTR" \n", MAC2STR(wifi1));

	for (i = 0; i < MACMAX; i++) {
		read_mac(buf, ETHX_MAC_OFFSET + (i * 6), eths);

		/* Last MAC check */
		if (eths[0] == 0xFF && eths[1] == 0xFF && eths[2] == 0xFF &&
		    eths[3] == 0xFF && eths[4] == 0xFF && eths[5] == 0xFF)
			break;

		printf("eth%u: "MACSTR"\n", i, MAC2STR(eths));
	}

	free(buf);
	return 0;
}

static int setmacs(int argc, char* const argv[])
{
	int i, ret;
	int eth_cnt;
	unsigned char *buf;
	struct spi_flash *flash;
	unsigned char eth_list[6 * MACMAX];
	unsigned char wifi0[6];
	unsigned char wifi1[6];

	memset(&wifi0, 0, sizeof(wifi0));
	memset(&wifi1, 0, sizeof(wifi1));
	memset(&eth_list, 0, sizeof(eth_list));

	if (argc < 1) {
		return -1;
	}

	if (argc >= 1) {
		if (parse_mac(argv[0], wifi0)) {
			printf("Bad mac of wifi0\n");
			return -1;
		}
	}

	if (argc >= 2) {
		if (parse_mac(argv[1], wifi1)) {
			printf("Bad mac of wifi1\n");
			return -1;
		}
	}

	for (i = 2, eth_cnt = 0; i < argc && eth_cnt <= MACMAX; i++, eth_cnt++) {
		if (parse_mac(argv[i], eth_list + (eth_cnt * 6))) {
			printf("Bad mac of eth%d\n", eth_cnt);
			return -1;
		}
	}

	flash = spi_flash_probe(0, 0, 1000000, SPI_MODE_3);
	if (!flash) {
		printf("Failed to initialize SPI flash\n");
		return 1;
	}
	buf = malloc(ART_FLASH_SIZE);
	if (!buf) {
		printf("Memory alloc failed\n");
		return 1;
	}
	ret = flash->read(flash, ART_FLASH_OFFSET, ART_FLASH_SIZE, buf);
	if (ret) {
		printf("SPI flash read failed\n");
		free(buf);
		return 1;
	}

	/* write all macs */
	write_mac(buf, WIFI0_MAC_OFFSET, wifi0);
	update_board_data_crc(buf, WIFI0_OFFSET_PART);
	write_mac(buf, WIFI1_MAC_OFFSET, wifi1);
	update_board_data_crc(buf, WIFI1_OFFSET_PART);
	for (i = 0; i < eth_cnt; i++)
		write_mac(buf, ETHX_MAC_OFFSET + (i * 6), eth_list + (i * 6));

	ret = spi_flash_update(flash, ART_FLASH_OFFSET, ART_FLASH_SIZE, buf);
	if (ret) {
		printf("SPI flash update failed\n");
		free(buf);
		return 1;
	}

	free(buf);
	return 0;
}

static int do_mac(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	const char *cmd;

	if (argc < 2)
		return CMD_RET_USAGE;

	cmd = argv[1];

	if (strcmp(cmd, "get") == 0) {
		return getmacs();
	} else if (strcmp(cmd, "set") == 0) {
		argc -= 2;
		argv += 2;
		return setmacs(argc, argv);
	}

	return CMD_RET_USAGE;
}

U_BOOT_CMD(
	mac,	6,	1,	do_mac,
	"get/set MAC address",
	"mac get	- get MAC addresses\n"
	"mac set [wifi0mac|-] [wifi1mac|-] [eth0mac|-] [eth1mac|-] ...\n"
	"		- set MAC addressesi\n"
);

