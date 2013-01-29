/*
 * Copyright (c) 2012 Qualcomm Atheros, Inc.
 */

#include <common.h>
#include <asm/arch-ipq806x/clock.h>
#include <asm/arch-ipq806x/iomap.h>
#include <asm/io.h>

/**
 * uart_pll_vote_clk_enable - enables PLL8
 */
void uart_pll_vote_clk_enable(void)
{
        unsigned int  ena;

        ena = readl(BB_PLL_ENA_SC0_REG);
        ena |= BIT(8);
        writel(ena, BB_PLL_ENA_SC0_REG);
        check_pll_status(BB_PLL8_STATUS_REG,PLL8_STATUS_BIT);
}

/**
 * uart_set_rate_mnd - configures divider M and D values
 *
 * Sets the M, D parameters of the divider to generate the GSBI UART
 * apps clock.
 */
static void uart_set_rate_mnd(void)
{
        unsigned int ns_reg_val;

        /* Assert MND reset. */
        ns_reg_val = readl(GSBIn_UART_APPS_NS_REG(GSBI_PORT));
        ns_reg_val |= BIT(7);
        writel(ns_reg_val, GSBIn_UART_APPS_NS_REG(GSBI_PORT));
        /* Program M and D values. */
        writel(MD16(M_VALUE, D_VALUE), GSBIn_UART_APPS_MD_REG(GSBI_PORT));
        /* Deassert MND reset. */
        ns_reg_val &= ~BIT(7);
        writel(ns_reg_val, GSBIn_UART_APPS_NS_REG(GSBI_PORT));
}

/**
 * uart_branch_clk_enable_reg - enables branch clock
 *
 * Enables branch clock for GSBI UART port.
 */
static void uart_branch_clk_enable_reg(void)
{
        unsigned int reg_val;

        reg_val = readl((void *)GSBIn_UART_APPS_NS_REG(GSBI_PORT));
        reg_val |= BIT(9);
        writel(reg_val, (void *)GSBIn_UART_APPS_NS_REG(GSBI_PORT));
}

/**
 * uart_local_clock_enable - configures N value and enables root clocks
 *
 * Sets the N parameter of the divider and enables root clock and
 * branch clocks for GSBI UART port.
 */
static void uart_local_clock_enable(void)
{
        unsigned int reg_val;
        void *const reg = (void *)GSBIn_UART_APPS_NS_REG(GSBI_PORT);

        /*
        * Program the NS register, if applicable. NS registers are not
        * set in the set_rate path because power can be saved by deferring
        * the selection of a clocked source until the clock is enabled.
        */
        reg_val = readl(GSBIn_UART_APPS_NS_REG(GSBI_PORT)); // REG(0x29D4+(0x20*((n)-1)))
        reg_val &= ~(Uart_clk_ns_mask);

        UART_SET_NS_VALUE;
        writel(reg_val, GSBIn_UART_APPS_NS_REG(GSBI_PORT));

        /* enable MNCNTR_EN */
        reg_val = readl(reg);
        reg_val |= BIT(8);
        writel(reg_val, reg);

        /* Enable root. */
        reg_val |= Uart_en_mask;
        writel(reg_val, reg);
        uart_branch_clk_enable_reg();
}

/**
 * uart_set_gsbi_clk - enables HCLK for UART GSBI port
 */
static void uart_set_gsbi_clk(void)
{
        unsigned int reg_val;

        reg_val = readl((void *)GSBIn_HCLK_CTL_REG(GSBI_PORT));
        reg_val |= BIT(4);
        writel(reg_val, ((void *)GSBIn_HCLK_CTL_REG(GSBI_PORT)));
}

/**
 * uart_clock_config - configures UART clocks
 *
 * Configures GSBI UART dividers, enable root and branch clocks.
 */
void uart_clock_config(void)
{
        uart_set_rate_mnd();

        UART_ENABLE_PLL_CLOCK
        uart_local_clock_enable();
        uart_set_gsbi_clk();
}

/**
 * cfpb_clock_config - configure CFPB clocks
 *
 * Enable clocks to CFPB. Required for NANDC and GSBI controllers on
 * the FPB.
 */
static void cfpb_clock_config(void)
{
	writel(CLK_BRANCH_ENA_ENABLE, SFAB_AHB_S3_FCLK_CTL_REG);
	writel(CLK_DIV_DIV_2, CFPB_CLK_NS_REG);
	writel(CLK_BRANCH_ENA_ENABLE, SFAB_CFPB_S_HCLK_CTL_REG);
	writel(CLK_BRANCH_ENA_ENABLE, CFPB_SPLITTER_HCLK_CTL_REG);
}

/**
 * nand_clock_config - configure NAND controller clocks
 *
 * Enable clocks to EBI2. Must be invoked before touching EBI2
 * registers.
 */
void nand_clock_config(void)
{
	cfpb_clock_config();
	writel(CLK_BRANCH_ENA_ENABLE, CFPB0_HCLK_CTL_REG);
	writel(CLK_BRANCH_ENA_ENABLE, EBI2_CLK_CTL_REG);

	/* Wait for clock to stabilize. */
	udelay(10);
}
