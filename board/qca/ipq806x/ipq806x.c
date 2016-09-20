/*
 * Copyright (c) 2016, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <common.h>
#include <asm/global_data.h>
#include <asm/io.h>
#include <environment.h>
#include <asm/arch-qcom-common/gsbi.h>
#include <asm/arch-qcom-common/uart.h>
#include <asm/arch-qcom-common/gpio.h>
#include <asm/arch-qcom-common/smem.h>
#include "ipq806x.h"
#include "qca_common.h"

DECLARE_GLOBAL_DATA_PTR;

qca_mmc mmc_host;

int nand_env_device = 0;
const char *rsvd_node = "/reserved-memory";
const char *del_node[] = {NULL};
const add_node_t add_node[] = {};

unsigned long timer_read_counter(void)
{
	return 0;
}

void reset_crashdump(void)
{
	return;
}

void reset_cpu(unsigned long a)
{
	while(1);
}
void emmc_clock_config(int mode)
{
	/* TODO: To be filled */
}
int board_mmc_init(bd_t *bis)
{
	int ret;

	mmc_host.base = MSM_SDC1_BASE;
	mmc_host.clk_mode = MMC_IDENTIFY_MODE;
	emmc_clock_config(mmc_host.clk_mode);

	ret = qca_mmc_init(bis, &mmc_host);

	return ret;
}
void board_nand_init(void)
{
	/* TODO: To be filled */
}
void qca_serial_init(struct ipq_serial_platdata *plat)
{
	int serial_node, gpio_node;
	serial_node = fdt_path_offset(gd->fdt_blob, "/serial");
        if (serial_node < 0) {
             return -1;
        }

        gpio_node = fdt_subnode_offset(gd->fdt_blob, serial_node, "serial_gpio");

	qca_gpio_init(gpio_node);
	writel(GSBI_PROTOCOL_CODE_I2C_UART <<
			GSBI_CTRL_REG_PROTOCOL_CODE_S,
			GSBI_CTRL_REG(GSBI4_BASE));

	if (!(plat->m_value == -1) || ( plat->n_value == -1) || (plat->d_value == -1))
		uart_clock_config(plat->port_id,
				plat->m_value,
				plat->n_value,
				plat->d_value);
}

int ipq_fdt_fixup_socinfo(void *blob)
{
	uint32_t cpu_type;
	int nodeoff, ret;

	ret = ipq_smem_get_socinfo_cpu_type(&cpu_type);
	if (ret) {
		printf("ipq: fdt fixup cannot get socinfo\n");
		return ret;
	}
	nodeoff = fdt_node_offset_by_compatible(blob, -1, "qcom,ipq8064");

	if (nodeoff < 0) {
		printf("ipq: fdt fixup cannot find compatible node\n");
		return nodeoff;
	}
	ret = fdt_setprop(blob, nodeoff, "cpu_type",
			&cpu_type, sizeof(cpu_type));
	if (ret)
		printf("%s: cannot set cpu type %d\n", __func__, ret);
	return ret;
}
