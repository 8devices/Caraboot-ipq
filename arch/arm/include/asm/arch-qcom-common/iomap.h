/*
 * Copyright (c) 2015, The Linux Foundation. All rights reserved.
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

#ifndef _PLATFORM_IPQ40XX_IOMAP_H_
#define _PLATFORM_IPQ40XX_IOMAP_H_

#include <configs/ipq40xx.h>
#define CLK_CTL_BASE		0x01800000
#define GCNT_BASE		0x004a1000
#define TIMER_BASE		0x0B021000
#define UART2_DM_BASE		0x078b0000
#define UART1_DM_BASE		0x078af000
#define I2C0_BASE		0x078B7000
#define TLMM_BASE		0x01000000
#define GPIO_CONFIG_ADDR(x) (TLMM_BASE + (x)*0x1000)
#define GPIO_IN_OUT_ADDR(x) (TLMM_BASE + 0x4 + (x)*0x1000)

#define GCNT_PSHOLD		0x004AB000

#endif /* _PLATFORM_IPQ40XX_IOMAP_H_ */
