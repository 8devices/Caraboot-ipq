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

#include <asm/arch-ipq806x/gsbi.h>
#include <asm/arch-ipq806x/clock.h>
#include <asm/arch-ipq806x/uart.h>
#include <asm/arch-ipq806x/gpio.h>
#include <asm/arch-ipq806x/iomap.h>

/*******************************************************
Function description: Initialize uart - clocks,gpio,uart
controller.
Arguments : None
Return : None

********************************************************/

void uart_dm_init()
{
        char *data = "Akronite Uboot  Bootloader - UART_DM Initialization !!!\n";
        /* Configure the uart clock */
        uart_clock_config();
        writel(GSBI_PROTOCOL_CODE_I2C_UART <<
               GSBI_CTRL_REG_PROTOCOL_CODE_S,
               GSBI_CTRL_REG(UART_GSBI_BASE));
        writel(UART_DM_CLK_RX_TX_BIT_RATE, MSM_BOOT_UART_DM_CSR(UART_DM_BASE));
        /* Intialize UART_DM */
        msm_boot_uart_dm_init((unsigned int)UART_DM_BASE);
        msm_boot_uart_dm_write( data, 53);

}

/*******************************************************
Function description: UART Receive operation  Reads a
                      word from the RX FIFO
Arguments : None
Return : None

********************************************************/

static unsigned int
msm_boot_uart_dm_read( unsigned int *data, int wait)
{
        static int rx_last_snap_count = 0;
        static int rx_chars_read_since_last_xfer = 0;
        unsigned int  base = UART_DM_BASE;

        if (data == NULL) {
                return MSM_BOOT_UART_DM_E_INVAL;
        }

        /* We will be polling RXRDY status bit */
        while (!(readl(MSM_BOOT_UART_DM_SR(base)) & MSM_BOOT_UART_DM_SR_RXRDY)) {
                /* if this is not a blocking call, we'll just return */
                if (!wait) {
                        return MSM_BOOT_UART_DM_E_RX_NOT_READY;
                }
        }

        /* Check for Overrun error. We'll just reset Error Status */
        if (readl(MSM_BOOT_UART_DM_SR(base)) & MSM_BOOT_UART_DM_SR_UART_OVERRUN) {
                writel(MSM_BOOT_UART_DM_CMD_RESET_ERR_STAT, MSM_BOOT_UART_DM_CR(base));
        }

        /* RX FIFO is ready; read a word. */
        *data = readl(MSM_BOOT_UART_DM_RF(base, 0));

        /* increment the total count of chars we've read so far */
        rx_chars_read_since_last_xfer += 4;

        /* Rx transfer ends when one of the conditions is met:
        * - The number of characters received since the end of the previous
        *   xfer equals the value written to DMRX at Transfer Initialization
        * - A stale event occurred
        */

        /* If RX transfer has not ended yet */
        if (rx_last_snap_count == 0) {
                /* Check if we've received stale event */
                if (readl(MSM_BOOT_UART_DM_MISR(base)) & MSM_BOOT_UART_DM_RXSTALE) {
                        /* Send command to reset stale interrupt */
                        writel(MSM_BOOT_UART_DM_CMD_RES_STALE_INT, MSM_BOOT_UART_DM_CR(base));
                }

                /* Check if we haven't read more than DMRX value */
                else if ((unsigned int)rx_chars_read_since_last_xfer <
                         readl(MSM_BOOT_UART_DM_DMRX(base))) {
                        /* We can still continue reading before initializing RX transfer */
                        return MSM_BOOT_UART_DM_E_SUCCESS;
                }

                /* If we've reached here it means RX
                * xfer end conditions been met
                */

                /* Read UART_DM_RX_TOTAL_SNAP register
                * to know how many valid chars
                * we've read so far since last transfer
                */
                rx_last_snap_count = readl(MSM_BOOT_UART_DM_RX_TOTAL_SNAP(base));

        }

        /* If there are still data left in FIFO we'll read them before
        * initializing RX Transfer again */
        if ((rx_last_snap_count - rx_chars_read_since_last_xfer) >= 0) {
                return MSM_BOOT_UART_DM_E_SUCCESS;
        }

//	msm_boot_uart_dm_init(base);
        msm_boot_uart_dm_init_rx_transfer(base);
        rx_last_snap_count = 0;
        rx_chars_read_since_last_xfer = 0;

        return MSM_BOOT_UART_DM_E_SUCCESS;
}

/*******************************************************
Function description: UART transmit operation.
Arguments :char *data- data to transmit
unsigned int num_of_chars - Number of bytes to transmit
Return : None

********************************************************/

static unsigned int
msm_boot_uart_dm_write(char *data, unsigned int num_of_chars)
{
        unsigned int tx_word_count = 0;
        unsigned int tx_char_left = 0, tx_char = 0;
        unsigned int tx_word = 0;
        int i = 0;
        char *tx_data = NULL;
        char new_data[1024];
        unsigned int  base = UART_DM_BASE;

        if ((data == NULL) || (num_of_chars <= 0)) {
                return MSM_BOOT_UART_DM_E_INVAL;
        }

        /* Replace line-feed (/n) with carriage-return + line-feed (/r/n) */

        msm_boot_uart_replace_lr_with_cr(data, num_of_chars, new_data, &i);

        tx_data = new_data;
        num_of_chars = i;

        /* Write to NO_CHARS_FOR_TX register number of characters
        * to be transmitted. However, before writing TX_FIFO must
        * be empty as indicated by TX_READY interrupt in IMR register
        */
        /* Check if transmit FIFO is empty.
        * If not we'll wait for TX_READY interrupt. */

        if (!(readl(MSM_BOOT_UART_DM_SR(base)) & MSM_BOOT_UART_DM_SR_TXEMT)) {
                while (!(readl(MSM_BOOT_UART_DM_ISR(base)) & MSM_BOOT_UART_DM_TX_READY)) {
                        __udelay(1);
                }
        }

        /* We are here. FIFO is ready to be written. */
        /* Write number of characters to be written */
        writel(num_of_chars, MSM_BOOT_UART_DM_NO_CHARS_FOR_TX(base));

        /* Clear TX_READY interrupt */
        writel(MSM_BOOT_UART_DM_GCMD_RES_TX_RDY_INT, MSM_BOOT_UART_DM_CR(base));

        /* We use four-character word FIFO. So we need to divide data into
        * four characters and write in UART_DM_TF register */
        tx_word_count = (num_of_chars % 4) ? ((num_of_chars / 4) + 1) :
                        (num_of_chars / 4);
        tx_char_left = num_of_chars;

        for (i = 0; i < (int)tx_word_count; i++) {
                tx_char = (tx_char_left < 4) ? tx_char_left : 4;
                PACK_CHARS_INTO_WORDS(tx_data, tx_char, tx_word);

                /* Wait till TX FIFO has space */
                while (!(readl(MSM_BOOT_UART_DM_SR(base)) & MSM_BOOT_UART_DM_SR_TXRDY)) {
                        __udelay(1);
                }

                /* TX FIFO has space. Write the chars */
                writel(tx_word, MSM_BOOT_UART_DM_TF(base, 0));
                tx_char_left = num_of_chars - (i + 1) * 4;
                tx_data = tx_data + 4;
        }

        return MSM_BOOT_UART_DM_E_SUCCESS;
}

/*******************************************************
Function description: Helper function to replace Line
Feed char "\n" with  Carriage Return "\r\n". Currently
keeping it simple than efficient

Arguments :char *data_in - data in
     int num_of_chars - Number of bytes
     char *data_out- Data out
     int *num_of_chars_out - Number of bytes out

Return : Success or Fail

********************************************************/

static unsigned int
msm_boot_uart_replace_lr_with_cr(char *data_in,
                                 int num_of_chars,
                                 char *data_out, int *num_of_chars_out)
{
        int i = 0, j = 0;

        if ((data_in == NULL) || (data_out == NULL) || (num_of_chars < 0)) {
                return MSM_BOOT_UART_DM_E_INVAL;
        }

        for (i = 0, j = 0; i < num_of_chars; i++, j++) {
                if (data_in[i] == '\n') {
                        data_out[j++] = '\r';
                }

                data_out[j] = data_in[i];
        }

        *num_of_chars_out = j;

        return MSM_BOOT_UART_DM_E_SUCCESS;
}

/*******************************************************
Function description: Initilaize uart controller.
Arguments : Uart controller base address
Return : 0 for Success

********************************************************/

static unsigned int msm_boot_uart_dm_init(unsigned int  uart_dm_base)
{
        /* Configure UART mode registers MR1 and MR2 */
        /* Hardware flow control isn't supported */
        writel(0x0, MSM_BOOT_UART_DM_MR1(uart_dm_base));

        /* 8-N-1 configuration: 8 data bits - No parity - 1 stop bit */
        writel(MSM_BOOT_UART_DM_8_N_1_MODE, MSM_BOOT_UART_DM_MR2(uart_dm_base));

        /* Configure Interrupt Mask register IMR */
        writel(MSM_BOOT_UART_DM_IMR_ENABLED, MSM_BOOT_UART_DM_IMR(uart_dm_base));

        /* Configure Tx and Rx watermarks configuration registers */
        /* TX watermark value is set to 0 - interrupt is generated when
        * FIFO level is less than or equal to 0 */
        writel(MSM_BOOT_UART_DM_TFW_VALUE, MSM_BOOT_UART_DM_TFWR(uart_dm_base));

        /* RX watermark value */
        writel(MSM_BOOT_UART_DM_RFW_VALUE, MSM_BOOT_UART_DM_RFWR(uart_dm_base));

        /* Configure Interrupt Programming Register */
        /* Set initial Stale timeout value */
        writel(MSM_BOOT_UART_DM_STALE_TIMEOUT_LSB, MSM_BOOT_UART_DM_IPR(uart_dm_base));

        /* Configure IRDA if required */
        /* Disabling IRDA mode */
        writel(0x0, MSM_BOOT_UART_DM_IRDA(uart_dm_base));

        /* Configure and enable sim interface if required */

        /* Configure hunt character value in HCR register */
        /* Keep it in reset state */
        writel(0x0, MSM_BOOT_UART_DM_HCR(uart_dm_base));

        /* Configure Rx FIFO base address */
        /* Both TX/RX shares same SRAM and default is half-n-half.
         * Sticking with default value now.
         * As such RAM size is (2^RAM_ADDR_WIDTH, 32-bit entries).
         * We have found RAM_ADDR_WIDTH = 0x7f */

        /* Issue soft reset command */
        msm_boot_uart_dm_reset(uart_dm_base);

        /* Enable/Disable Rx/Tx DM interfaces */
        /* Data Mover not currently utilized. */
        writel(0x0, MSM_BOOT_UART_DM_DMEN(uart_dm_base));

        /* Enable transmitter and receiver */
        writel(MSM_BOOT_UART_DM_CR_RX_ENABLE, MSM_BOOT_UART_DM_CR(uart_dm_base));
        writel(MSM_BOOT_UART_DM_CR_TX_ENABLE, MSM_BOOT_UART_DM_CR(uart_dm_base));

        /* Initialize Receive Path */
        msm_boot_uart_dm_init_rx_transfer(uart_dm_base);

        return 0 ;
}

/*******************************************************
Function description: Reset Uart controller
Arguments : Uart controller base address
Return : 0 for Success

********************************************************/

static unsigned int msm_boot_uart_dm_reset(unsigned int base)
{
        writel(MSM_BOOT_UART_DM_CMD_RESET_RX, MSM_BOOT_UART_DM_CR(base));
        writel(MSM_BOOT_UART_DM_CMD_RESET_TX, MSM_BOOT_UART_DM_CR(base));
        writel(MSM_BOOT_UART_DM_CMD_RESET_ERR_STAT, MSM_BOOT_UART_DM_CR(base));
        writel(MSM_BOOT_UART_DM_CMD_RES_TX_ERR, MSM_BOOT_UART_DM_CR(base));
        writel(MSM_BOOT_UART_DM_CMD_RES_STALE_INT, MSM_BOOT_UART_DM_CR(base));

        return MSM_BOOT_UART_DM_E_SUCCESS;
}


/*******************************************************
Function description: Init Rx transfer
Arguments : Uart controller base address
Return : 0 for success

********************************************************/

static unsigned int msm_boot_uart_dm_init_rx_transfer(unsigned int uart_dm_base)
{
        writel(MSM_BOOT_UART_DM_GCMD_DIS_STALE_EVT, MSM_BOOT_UART_DM_CR(uart_dm_base));
        writel(MSM_BOOT_UART_DM_CMD_RES_STALE_INT, MSM_BOOT_UART_DM_CR(uart_dm_base));
        writel(MSM_BOOT_UART_DM_DMRX_DEF_VALUE, MSM_BOOT_UART_DM_DMRX(uart_dm_base));
        writel(MSM_BOOT_UART_DM_GCMD_ENA_STALE_EVT, MSM_BOOT_UART_DM_CR(uart_dm_base));

        return MSM_BOOT_UART_DM_E_SUCCESS;
}

/*******************************************************
Function description: Uboot specific transmit a character
Arguments : None
Return : None

********************************************************/


void serial_putc (char c)
{

        msm_boot_uart_dm_write(&c, 1);

}

/*******************************************************
Function description: transmit a string of data
Arguments : const char *s - data pointer
Return : None

********************************************************/


void serial_puts (const char *s)
{
        while (*s != '\0')
                serial_putc (*s++);
}

/*******************************************************
Function description: uboot check whether data available
in RX FIFO
Arguments : None
Return : Return 1 or 0

********************************************************/


int serial_tstc (void)
{

        return (readl(MSM_BOOT_UART_DM_SR(UART_DM_BASE)) & MSM_BOOT_UART_DM_SR_RXRDY);

}


/*******************************************************
Function description: uboot serial get character
Arguments : None
Return : number of bytes

********************************************************/

int serial_getc (void)
{
        int byte;
        static unsigned int word = 0;

        if (!word) {
                /* Read from FIFO only if it's a first read or all the four
                 * characters out of a word have been read */
                if (msm_boot_uart_dm_read( &word,1) != MSM_BOOT_UART_DM_E_SUCCESS) {
                        return -1;
                }

        }

        byte = (int)word & 0xff;
        word = word >> 8;

        return byte;
}

/*******************************************************
Function description: uboot serial set baudarate
Arguments : None
Return : None

********************************************************/


void  serial_setbrg(void)
{

        return ;
}

/*******************************************************
Function description: Uboot serial init call
Arguments : None
Return : None

********************************************************/

int  serial_init(void)
{
        uart_dm_init();
        return 0;

}
