
/* * Copyright (c) 2012-2013 Qualcomm Atheros, Inc. * */

/*
 * Adapted from lib/asm-offsets.c
 */

#include <common.h>
#include <linux/kbuild.h>

int main(void)
{
	DEFINE(GENERATED_IPQ_RESERVE_SIZE, sizeof(ipq_mem_reserve_t));

	return 0;
}

