/*
 * 8devices Jalapeno config
 *
 * Copyright (C) 2018 Mantas Pucka <mantas@8devices.com>
 *
 * SPDX-License-Identifier:GPL-2.0-or-later
*/

#ifndef _IPQ40XX_JALAPENO_H
#define _IPQ40XX_JALAPENO_H

#include <configs/ipq40xx_cdp.h>

#define MTDPARTS_DEFAULT	"mtdparts=nand1:0x8000000@0x0(ubi)"
#define MTDIDS_DEFAULT		"nand1=nand1"

#define CONFIG_FACTORY_IMG_FILENAME    "jalapeno.bin"

#define CONFIG_EXTRA_ENV_SETTINGS							\
	"bootcmd=run setup && run bootlinux\0"						\
	"setup=partname=1 && setenv bootargs ubi.mtd=ubi ${args_common}\0"		\
	"args_common=rootfstype=squashfs\0"						\
	"bootlinux=run boot1 boot2 boot3 boot4 boot5|| reset\0"				\
	"boot1=echo Booting from partition: ${partname}\0"				\
	"boot2=nand device 1\0"							\
	"boot3=set mtdparts mtdparts=nand1:0x8000000@0x0(ubi)\0"			\
	"boot4=ubi part ubi && ubi read 84000000 kernel\0"				\
	"boot5=cfgsel 84000000 && run bootfdtcmd\0"					\
	"do_recovery=run rec1 rec2 rec3 rec4; reset\0"					\
	"rec1=echo Doing firmware recovery!\0"						\
	"rec2=sleep 2 && tftpboot ${tftp_loadaddr} ${recovery_file}\0"			\
	"rec3=nand device 1 && nand erase.chip\0"					\
	"rec4=nand write ${fileaddr} 0x0 ${filesize}\0"					\
	"tftp_loadaddr=0x84000000\0"							\
	"recovery_file=fwupdate.bin\0"							\
	"mtdparts=" MTDPARTS_DEFAULT "\0"						\

#endif /* _IPQ40XX_JALAPENO_H */
