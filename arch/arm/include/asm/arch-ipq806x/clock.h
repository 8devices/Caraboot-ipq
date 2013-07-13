/*
 * Copyright (c) 2012-2013 Qualcomm Atheros, Inc.
 * Source : APQ8064 LK Boot
 *
 * Copyright (c) 2012, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
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

#ifndef __PLATFORM_IPQ860X_CLOCK_H_
#define __PLATFORM_IPQ860X_CLOCK_H_

#include <asm/io.h>
/* UART clock @ 7.3728 MHz */
#define UART_DM_CLK_RX_TX_BIT_RATE 0xCC

/* UART specific definitions */

#define BIT(s) (1<<s)
#define Uart_ns_val  NS(BIT_POS_31,BIT_POS_16,N_VALUE,M_VALUE, 5, 4, 3, 1, 2, 0,3)
#define Uart_clk_ns_mask  (BM(BIT_POS_31, BIT_POS_16) | BM(BIT_POS_6, BIT_POS_0))
#define Uart_mnd_en_mask  BIT(8) * !!(625)
#define Uart_en_mask  BIT(11)
#define MD16(m, n) (BVAL(BIT_POS_31, BIT_POS_16, m) | BVAL(BIT_POS_15, BIT_POS_0, ~(n)))
#define Uart_ns_val_rumi  NS(BIT_POS_31, BIT_POS_16, N_VALUE, M_VALUE, 5, 4, 3, 1, 2, 0,0)
#define GSBIn_UART_APPS_MD_REG(n)           REG(0x29D0+(0x20*((n)-1)))
#define GSBIn_UART_APPS_NS_REG(n)           REG(0x29D4+(0x20*((n)-1)))
#define GSBIn_HCLK_CTL_REG(n)               REG(0x29C0+(0x20*((n)-1)))
#define BB_PLL_ENA_SC0_REG                  REG(0x34C0)
#define BB_PLL8_STATUS_REG                  REG(0x3158)
#define REG(off)        (MSM_CLK_CTL_BASE + (off))
#define PLL8_STATUS_BIT                     16

#define PLL_LOCK_DET_STATUS_REG             REG(0x03420)
#define SFAB_AHB_S3_FCLK_CTL_REG            REG(0x0216C)
#define CFPB_CLK_NS_REG                     REG(0x0264C)
#define CFPB0_HCLK_CTL_REG                  REG(0x02650)
#define SFAB_CFPB_S_HCLK_CTL_REG            REG(0x026C0)
#define CFPB_SPLITTER_HCLK_CTL_REG          REG(0x026E0)
#define EBI2_CLK_CTL_REG                    REG(0x03B00)

#define ALWAYS_ON_CLK_BRANCH_ENA(i)         ((i) << 8)

#define CLK_BRANCH_ENA_MASK                 0x00000010
#define CLK_BRANCH_ENA_ENABLE               0x00000010
#define CLK_BRANCH_ENA_DISABLE              0x00000000
#define CLK_BRANCH_ENA(i)                   ((i) << 4)

/* Register: CFPB_CLK_NS */
#define CLK_DIV_MASK                        0x00000003
#define CLK_DIV_DIV_1                       0x00000000
#define CLK_DIV_DIV_2                       0x00000001
#define CLK_DIV_DIV_3                       0x00000002
#define CLK_DIV_DIV_4                       0x00000003
#define CLK_DIV(i)                          ((i) << 0)

#define MN_MODE_DUAL_EDGE 0x2
#define BIT_POS_31 31
#define BIT_POS_16 16
#define BIT_POS_6  6
#define BIT_POS_0  0
#define BIT_POS_15 15

#define BM(m, l) (((((unsigned int)-1) << (31-m)) >> (31-m+l)) << l)
#define BVAL(m, l, val)     (((val) << l) & BM(m, l))

/* MD Registers */
#define MD4(m_lsb, m, n_lsb, n) \
    (BVAL((m_lsb+3), m_lsb, m) | BVAL((n_lsb+3), n_lsb, ~(n)))

#define MD8(m_lsb, m, n_lsb, n) \
    (BVAL((m_lsb+7), m_lsb, m) | BVAL((n_lsb+7), n_lsb, ~(n)))

/* NS Registers */
#define NS(n_msb, n_lsb, n, m, mde_lsb, d_msb, d_lsb, d, s_msb, s_lsb, s) \
    (BVAL(n_msb, n_lsb, ~(n-m)) \
     | (BVAL((mde_lsb+1), mde_lsb, MN_MODE_DUAL_EDGE) * !!(n)) \
     | BVAL(d_msb, d_lsb, (d-1)) | BVAL(s_msb, s_lsb, s))

#define NS_MM(n_msb, n_lsb, n, m, d_msb, d_lsb, d, s_msb, s_lsb, s) \
    (BVAL(n_msb, n_lsb, ~(n-m)) | BVAL(d_msb, d_lsb, (d-1)) \
     | BVAL(s_msb, s_lsb, s))

#define NS_DIVSRC(d_msb , d_lsb, d, s_msb, s_lsb, s) \
    (BVAL(d_msb, d_lsb, (d-1)) | BVAL(s_msb, s_lsb, s))

#define NS_DIV(d_msb , d_lsb, d) \
    BVAL(d_msb, d_lsb, (d-1))

#define NS_SRC_SEL(s_msb, s_lsb, s) \
    BVAL(s_msb, s_lsb, s)

/* gmac specific clock settings */
#define NSS_250MHZ_CLK_SRC_CTL  0x903D60
#define NSS_250MHZ_CLK_SRC0_NS  0x903D64
#define NSS_250MHZ_CLK_SRC0_MD  0x903D6C
#define NSS_250MHZ_CLK_CTL      0x903D74
#define GMAC_CORE0_RESET        0x903CBC
#define GMAC_AHB_RESET          0x903E24
#define TCSR_PXO_SEL            0x1A4000C0
#define SRC_SEL_PLL0            (0x2 << 0)
#define MNCNTR_MODE_DUAL_EDGE   (0x2 << 5)
#define MNCNTR_ENABLE           (0x1 << 8)
#define MNCNTR_RST_ACTIVE       (0x1 << 7)
#define N_VAL                   15

void gmac_clock_config(unsigned int m, unsigned int not_n, unsigned int not_2d);

/* Uart specific clock settings */

void uart_pll_vote_clk_enable(unsigned int);
void uart_clock_config(unsigned int gsbi_port, unsigned int m, unsigned int n,
		unsigned int d, unsigned int clk_dummy);
void nand_clock_config(void);

#endif  /*  __PLATFORM_IPQ860X_CLOCK_H_ */
