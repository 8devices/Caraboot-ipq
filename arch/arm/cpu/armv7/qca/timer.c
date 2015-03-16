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

#include <asm/arch-qca961x/iomap.h>
#include <asm/io.h>
#include <common.h>
#include <asm/types.h>
#include <asm/arch-qca961x/timer.h>

static ulong timestamp;
static ulong lastinc;

#define GPT_FREQ_KHZ	20833
#define GPT_FREQ	(GPT_FREQ_KHZ * 1000)	/* 20.833 MHz */

/**
 * timer_init - initialize timer
 */
int timer_init(void)
{
	writel(1, GCNT_BASE);
	return 0;
}

/**
 * get_timer - returns time lapsed
 * @base: base/start time
 *
 * Returns time lapsed, since the specified base time value.
 */
ulong get_timer(ulong base)
{
	return get_timer_masked() - base;
}

/**
 * __udelay -  generates micro second delay.
 * @usec: delay duration in microseconds
 *
 * With 32KHz clock, minimum possible delay is 31.25 Micro seconds and
 * its multiples. In Rumi GPT clock is 32 KHz
 */
void __udelay(unsigned long usec)
{
	unsigned int val;
	ulong now, last;
	ulong runcount;

	usec = (usec + GPT_FREQ_KHZ - 1) / GPT_FREQ_KHZ;
	last = readl(TIMER_BASE + QTMR_CNTP_TVAL);
	runcount = last;
	val = usec + last;

	do {
		now = readl(TIMER_BASE + QTMR_CNTP_TVAL);
		if (last > now)
			runcount += ((GPT_FREQ - last) + now);
		else
			runcount += (now - last);
		last = now;
	} while (runcount < val);
}

inline ulong gpt_to_sys_freq(unsigned int gpt)
{
	/*
	 * get_timer() expects the timer increments to be in terms
	 * of CONFIG_SYS_HZ. Convert GPT timer values to CONFIG_SYS_HZ
	 * units.
	 */
	return ((ulong)gpt) / (GPT_FREQ / CONFIG_SYS_HZ);
}

/**
 * get_timer_masked - returns current ticks
 *
 * Returns the current timer ticks, since boot.
 */
ulong get_timer_masked(void)
{
	ulong now = gpt_to_sys_freq(readl(TIMER_BASE + QTMR_CNTP_TVAL));

	if (lastinc <= now) {	/* normal mode (non roll) */
		/* normal mode */
		timestamp += now - lastinc;
		/* move stamp forward with absolute diff ticks */
	} else {
		/* we have overflow of the count down timer */
		timestamp += now + (TIMER_LOAD_VAL - lastinc);
	}
	lastinc = now;

	return timestamp;
}
