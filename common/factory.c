/*
 * Factory image loading
 *
 * Copyright (C) 2018 Mantas Pucka <mantas@8devices.com>
 *
 * SPDX-License-Identifier:GPL-2.0
*/

#include <common.h>
#include <config.h>

#define XMK_STR(x)        #x
#define MK_STR(x) XMK_STR(x)

static void factory_load_image(void)
{
	char *filename = CONFIG_FACTORY_IMG_FILENAME;
	int tftp_ret;

	int  argc_tftp = 3;
	char* argv_tftp[] = {"tftpboot", CONFIG_FACTORY_IMG_LOAD_ADDR, filename};

	int argc_bootm = 2;
	char* argv_bootm[] = {"bootm", CONFIG_FACTORY_IMG_LOAD_ADDR};

	if (!getenv ("ipaddr"))
		setenv("ipaddr", MK_STR(CONFIG_IPADDR));

	if (!getenv ("serverip"))
		setenv("serverip", MK_STR(CONFIG_SERVERIP));

	//Wait for eth link
	mdelay(4000);

	setenv("netretry", "once"); // Try once, reboot after
	tftp_ret=do_tftpb (NULL, 0, argc_tftp, argv_tftp);
	if (0 == tftp_ret) {
		  printf("Booting TFTP image...\n");
		  do_bootm(NULL, 0, argc_bootm, argv_bootm);
		  do_reset(NULL, 0, 0, NULL);
	}
	else{
		  printf("Error getting TFTP image. Rebooting...\n");
		  do_reset(NULL, 0, 0, NULL);
	}
	return;
}

int factory_mode_start(void)
{
	char* production_env = getenv("production");

	if (production_env){
		if (strncmp(production_env, "yes", 3) == 0){
			factory_load_image();
		}
	}
	return 0;
}
