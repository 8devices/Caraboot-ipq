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

#ifndef _QCA961X_BOARD_PARAM_H_
#define _QCA961X_BOARD_PARAM_H_

#include <asm/arch-qca961x/iomap.h>
#include <asm/arch-qcom-common/gpio.h>
#include <asm/sizes.h>
#include "qca961x_cdp.h"

/* Board specific parameter Array */
board_qca961x_params_t board_params[] = {
	{
		.machid = MACH_TYPE_QCA961X_RUMI,
		.ddr_size = (256 << 20),
		.uart_dm_base = UART1_DM_BASE,
		.dbg_uart_gpio = {
			{
				.gpio = 16,
				.func = 1,
				.pull = GPIO_NO_PULL,
				.oe = GPIO_OE_ENABLE
			},
			{
				.gpio = 17,
				.func = 1,
				.pull = GPIO_NO_PULL,
				.oe = GPIO_OE_ENABLE
			},
		},
	},
	{
		.machid = MACH_TYPE_QCA961X_VIRTIO,
		.ddr_size = (256 << 20),
		.uart_dm_base = UART1_DM_BASE,
		.dbg_uart_gpio = {
			{
				.gpio = 16,
				.func = 1,
				.pull = GPIO_NO_PULL,
				.oe = GPIO_OE_ENABLE
			},
			{
				.gpio = 17,
				.func = 1,
				.pull = GPIO_NO_PULL,
				.oe = GPIO_OE_ENABLE
			},
		},

	},
};

#define NUM_QCA961X_BOARDS	ARRAY_SIZE(board_params)
#endif /* _QCA961X_BOARD_PARAM_H_ */