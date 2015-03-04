/*
 *
 * Copyright (c) 2015, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <common.h>
#include <asm/global_data.h>
#include <asm/io.h>
#include <asm/arch-qcom-common/gpio.h>
#include <asm/errno.h>
#include <linux/mtd/ipq_nand.h>
#include <asm/arch-qcom-common/nand.h>
#include <asm/arch-qca961x/ess/qca961x_edma.h>
#include <environment.h>
#include "qca961x_board_param.h"
#include "qca961x_cdp.h"
#include <asm/arch-qcom-common/clk.h>

DECLARE_GLOBAL_DATA_PTR;

loff_t board_env_offset;
loff_t board_env_range;
extern int nand_env_device;
char *env_name_spec;
extern char *nand_env_name_spec;
int (*saveenv)(void);
env_t *env_ptr;
extern env_t *nand_env_ptr;
extern int nand_env_init(void);
extern int nand_saveenv(void);
extern void nand_env_relocate_spec(void);
extern int qca961x_edma_init(qca961x_edma_board_cfg_t *edma_cfg);
#ifdef CONFIG_QCA_MMC
qca_mmc mmc_host;
#endif
/*
 * Don't have this as a '.bss' variable. The '.bss' and '.rel.dyn'
 * sections seem to overlap.
 *
 * $ arm-none-linux-gnueabi-objdump -h u-boot
 * . . .
 *  8 .rel.dyn      00004ba8  40630b0c  40630b0c  00038b0c  2**2
 *                  CONTENTS, ALLOC, LOAD, READONLY, DATA
 *  9 .bss          0000559c  40630b0c  40630b0c  00000000  2**3
 *                  ALLOC
 * . . .
 *
 * board_early_init_f() initializes this variable, resulting in one
 * of the relocation entries present in '.rel.dyn' section getting
 * corrupted. Hence, when relocate_code()'s 'fixrel' executes, it
 * patches a wrong address, which incorrectly modifies some global
 * variable resulting in a crash.
 *
 * Moral of the story: Global variables that are written before
 * relocate_code() gets executed cannot be in '.bss'
 */
board_qca961x_params_t *gboard_param = (board_qca961x_params_t *)0xbadb0ad;

int env_init(void)
{
	return nand_env_init();
}

void env_relocate_spec(void)
{
	nand_env_relocate_spec();
};

int board_init(void)
{
	/* Hardcoded board param for now. Need to retrieve from SMEM */
	gboard_param = board_params;

	/* Hardcode everything for NAND */
	nand_env_device = CONFIG_IPQ_NAND_NAND_INFO_IDX;
	board_env_offset = 0x40000;
	board_env_range = CONFIG_ENV_SIZE_MAX;
	saveenv = nand_saveenv;
	env_ptr = nand_env_ptr;
	env_name_spec = nand_env_name_spec;

	return 0;
}

int board_late_init(void)
{
	return 0;
}

/*
 * This function is called in the very beginning.
 * Retreive the machtype info from SMEM and map the board specific
 * parameters. Shared memory region at Dram address 0x40400000
 * contains the machine id/ board type data polulated by SBL.
 */
int board_early_init_f(void)
{
	/* Hardcoded board param for now. Need to retrieve from SMEM */
	gboard_param = board_params;
	return 0;
}

void clear_l2cache_err(void)
{
	return;
}

void reset_cpu(ulong addr)
{
	/* clear ps-hold bit to reset the soc */
	writel(0, GCNT_PSHOLD);
	while (1);
}

int dram_init(void)
{
	gd->ram_size = 0x10000000; /* 256 MB */
	return 0;
}

void board_nand_init(void)
{
	ipq_nand_init(IPQ_NAND_LAYOUT_LINUX, QCOM_NAND_QPIC);
}

int board_eth_init(bd_t *bis)
{
	u32 status;
	switch (gboard_param->machid) {
	case MACH_TYPE_QCA961X_RUMI:
		qca961x_register_switch(NULL);
		break;
	default:
		break;
	}
	status = qca961x_edma_init(gboard_param->edma_cfg);
	return status;
}

void qca_configure_gpio(gpio_func_data_t *gpio, uint count)
{
	int i;

	for (i = 0; i < count; i++) {
		gpio_tlmm_config(gpio->gpio, gpio->func, gpio->out,
				 gpio->pull, gpio->drvstr, gpio->oe);
		gpio++;
	}
}

#ifdef CONFIG_OF_BOARD_SETUP
/*
 * For newer kernel that boot with device tree (3.14+), all of memory is
 * described in the /memory node, including areas that the kernel should not be
 * touching.
 *
 * By default, u-boot will walk the dram bank info and populate the /memory
 * node; here, overwrite this behavior so we describe all of memory instead.
 */
void ft_board_setup(void *blob, bd_t *bd)
{
	u64 memory_start = CONFIG_SYS_SDRAM_BASE;
	u64 memory_size = gboard_param->ddr_size;

	fdt_fixup_memory_banks(blob, &memory_start, &memory_size, 1);
}
#endif /* CONFIG_OF_BOARD_SETUP */

#ifdef CONFIG_QCA_MMC
int board_mmc_init(bd_t *bis)
{
	int ret = 0;

	mmc_host.base = MSM_SDC1_BASE;
	mmc_host.clk_mode = MMC_IDENTIFY_MODE;
	emmc_clock_config(mmc_host.clk_mode);

	ret = qca_mmc_init(bis, &mmc_host);

	return ret;
}

void board_mmc_deinit(void)
{
	emmc_clock_disable();
}
#endif
