/*
 * Copyright (c) 2012 - 2014 The Linux Foundation. All rights reserved.
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

#ifndef __PLATFORM_IPQ860X_CLOCK_H_
#define __PLATFORM_IPQ860X_CLOCK_H_

#include <configs/ipq806x.h>
#include <asm/io.h>

#define MSM_CLK_CTL_BASE                        0x00900000
#define GSBIn_UART_APPS_MD_REG(n)               (MSM_CLK_CTL_BASE + 0x29D0 + (0x20*((n)-1)))
#define GSBIn_UART_APPS_NS_REG(n)               (MSM_CLK_CTL_BASE + 0x29D4 + (0x20*((n)-1)))
#define GSBIn_HCLK_CTL_REG(n)                   (MSM_CLK_CTL_BASE + 0x29C0 + (0x20*((n)-1)))
#define BB_PLL_ENA_SC0_REG                      (MSM_CLK_CTL_BASE + 0x34C0)
#define PLL_LOCK_DET_STATUS_REG                 (MSM_CLK_CTL_BASE + 0x03420)
#define EBI2_CLK_CTL_REG                 	(MSM_CLK_CTL_BASE + 0x03B00)

#define MN_MODE_DUAL_EDGE                       0x2

#define BM(m, l)                                (((((unsigned int)-1) << (31-m)) >> (31-m+l)) << l)
#define BVAL(m, l, val)                         (((val) << l) & BM(m, l))

#define Uart_clk_ns_mask                        (BM(31, 16) | BM(6, 0))
#define Uart_en_mask                            BIT(11)
#define MD16(m, n)                              (BVAL(31, 16, m) | BVAL(15, 0, ~(n)))

/* NS Registers */
#define NS(n_msb, n_lsb, n, m, mde_lsb, d_msb, d_lsb, d, s_msb, s_lsb, s) \
        (BVAL(n_msb, n_lsb, ~(n-m)) \
         | (BVAL((mde_lsb+1), mde_lsb, MN_MODE_DUAL_EDGE) * !!(n)) \
         | BVAL(d_msb, d_lsb, (d-1)) | BVAL(s_msb, s_lsb, s))

#define GMAC_CORE_RESET(n)      \
                ((void *)(0x903CBC + ((n) * 0x20)))
#define GMACSEC_CORE_RESET(n)   \
                ((void *)(0x903E28 + ((n - 1) * 4)))

#define ALWAYS_ON_CLK_BRANCH_ENA(i)         ((i) << 8)
#define CLK_BRANCH_ENA(i)                   ((i) << 4)

/* Uart specific clock settings */

void uart_pll_vote_clk_enable(void);
void uart_clock_config(unsigned int gsbi_port, unsigned int m, unsigned int n,
		unsigned int d);
#endif  /*  __PLATFORM_IPQ860X_CLOCK_H_ */
