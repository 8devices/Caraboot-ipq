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

#ifndef _QCA961X_CDP_H_
#define _QCA961X_CDP_H_

#include <configs/qca961x_cdp.h>
#include <asm/u-boot.h>
#include "../common/qca_common.h"

#define NO_OF_DBG_UART_GPIOS	2

typedef struct {
	int gpio;
	unsigned int func;
	unsigned int out;
	unsigned int pull;
	unsigned int drvstr;
	unsigned int oe;
} gpio_func_data_t;

/* Board specific parameters */
typedef struct {
	unsigned int machid;
	unsigned int ddr_size;
	unsigned int uart_dm_base;
	gpio_func_data_t dbg_uart_gpio[NO_OF_DBG_UART_GPIOS];
} __attribute__ ((__packed__)) board_qca961x_params_t;

extern board_qca961x_params_t *gboard_param;
unsigned int get_board_index(unsigned int machid);
void qca_configure_gpio(gpio_func_data_t *gpio, uint count);


#define MSM_SDC1_BASE      0x7824000

#endif
