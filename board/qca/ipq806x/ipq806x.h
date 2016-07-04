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

#ifndef _IPQ806X_H_
#define _IPQ806X_H_

#include <configs/ipq806x.h>
#include <asm/u-boot.h>

typedef struct {
        int gpio;
        unsigned int func;
        unsigned int out;
        unsigned int pull;
        unsigned int drvstr;
        unsigned int oe;
} gpio_func_data_t;

#define GSBI4_BASE 0x16300000
void ipq_configure_gpio(gpio_func_data_t *gpio, uint count);
#endif /* _IPQ806X_H_ */
