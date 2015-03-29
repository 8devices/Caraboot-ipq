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
#include <command.h>
#include <image.h>
#include <nand.h>
#include <errno.h>
#include <asm/arch-qca961x/scm.h>
#include <part.h>
#define DLOAD_MAGIC_COOKIE	0x10
static int debug = 0;

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

static int do_bootqca(cmd_tbl_t *cmdtp, int flag, int argc, char *const argv[])
{
#ifdef CONFIG_QCA_APPSBL_DLOAD
	uint64_t etime;
	volatile u32 val;
#endif
	int ret;

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
	return CMD_RET_SUCCESS;
}

U_BOOT_CMD(bootipq, 2, 0, do_bootqca,
	   "bootipq from flash device",
	   "bootipq [debug] - Load image(s) and boots the kernel\n");
