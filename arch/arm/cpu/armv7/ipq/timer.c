/*
 * Copyright (c) 2012 Qualcomm Atheros, Inc. *
   Source : APQ8064 LK boot

 * Copyright (c) 2011-2012, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of Google, Inc. nor the names of its contributors
 *    may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */


#include <asm/arch-ipq806x/iomap.h>
#include <asm/io.h>
#include <common.h>
#include <asm/types.h>
#include <asm/arch-ipq806x/timer.h>


static ulong timestamp;
static ulong lastinc;

/*******************************************************
Function description: timer_init.
Arguments : None
Return : None

********************************************************/
int timer_init(void)
{
        return 0;
}

/*******************************************************
Function description: returns time lapsed from the passed
base value
Arguments: Base/start time
Return : time lapsed.

********************************************************/

ulong get_timer(ulong base)
{
        return get_timer_masked() - base;
}

/*******************************************************
Function description: Micro second delay. 33KHz clock,
minimum possible delay is 30 Micro seconds and its  multiples
Arguments : Delay in Micro seconds
Return : None
In Rumi GPT clock is 32 KHz
********************************************************/

void __udelay(unsigned long usec)
{
        unsigned int val;
        volatile unsigned int Msecvalue=0;

        usec = (usec * 32 + 1000 - 32) / 1000;

        Msecvalue = usec;

        writel(0, GPT_CLEAR);
        writel(0, GPT_ENABLE);
        do {
                val = 0;
                val = readl(GPT_COUNT_VAL);
        } while (val != 0);

        writel(GPT_ENABLE_EN, GPT_ENABLE);
        do {
                val = 0;
                val = readl(GPT_COUNT_VAL);
        } while (val < usec) ;

        writel(0, GPT_ENABLE);
        writel(0, GPT_CLEAR);
}

/*******************************************************
Function description: returns the time lapsed from last
                      read value
Arguments : None
Return : None

********************************************************/

ulong get_timer_masked(void)
{
        ulong now = READ_TIMER;	/* current tick value */

        if (lastinc <= now) {	/* normal mode (non roll) */
                /* normal mode */
                timestamp += now - lastinc;
                /* move stamp forward with absolute diff ticks */
        } else {		/* we have overflow of the count down timer */
                timestamp += now + (TIMER_LOAD_VAL - lastinc);
        }

        lastinc = now;

        return timestamp;
}
