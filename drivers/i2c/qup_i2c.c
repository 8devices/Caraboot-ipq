/*
 * Copyright (c) 2015 The Linux Foundation. All rights reserved.
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

#include <common.h>
#include <i2c.h>
#include "qup_i2c.h"
#include <asm/io.h>
#include <asm/errno.h>
#include <fdtdec.h>
#include <dm/device.h>
#include <dm/root.h>
#include <mapmem.h>
#include <dm.h>

#ifdef CONFIG_IPQ807x_I2C
#include "../../board/qca/ipq807x/clk.h"
#endif

#ifdef CONFIG_IPQ40XX_I2C
#include "../../board/qca/ipq40xx/clk.h"
#endif

DECLARE_GLOBAL_DATA_PTR;

static int i2c_base_addr;
static int i2c_hw_initialized;
static int i2c_board_initialized;

struct i2c_qup_bus {
	int *i2c_base_addr;
};

/*
 * Reset entire QUP and all mini cores
 */
static void i2c_reset(void)
{
	writel(0x1, (i2c_base_addr + QUP_SW_RESET_OFFSET));
	udelay(5);
}

static int check_bit_state(uint32_t reg_addr, int bit_num, int val,
				int us_delay)
{
	unsigned int count = TIMEOUT_CNT;
	unsigned int bit_val = ((readl(reg_addr) >> bit_num) & 0x01);

	while (bit_val != val) {
		count--;
		if (count == 0) {
			return -ETIMEDOUT;
		}
		udelay(us_delay);
		bit_val = ((readl(reg_addr) >> bit_num) & 0x01);
	}

	return SUCCESS;
}

/*
 * Check whether QUP State is valid
 */
static int check_qup_state_valid(void)
{
	return check_bit_state(i2c_base_addr + QUP_STATE_OFFSET,
				QUP_STATE_VALID_BIT,
				QUP_STATE_VALID, 1);
}

/*
 * Configure QUP Core state
 */
static int config_i2c_state(unsigned int state)
{
	uint32_t val;
	int ret = SUCCESS;

	ret = check_qup_state_valid();
	if (ret != SUCCESS)
		return ret;

	/* Set the state  */
	val = readl(i2c_base_addr + QUP_STATE_OFFSET);
	val = ((val & ~QUP_STATE_MASK) | state);
	writel(val, (i2c_base_addr + QUP_STATE_OFFSET));
	ret = check_qup_state_valid();

	return ret;
}

/*
 * Configure I2C IO Mode.
 */
void config_i2c_mode(void)
{
	int cfg;

	cfg = readl(i2c_base_addr + QUP_IO_MODES_OFFSET);
	cfg |= (INPUT_FIFO_MODE | OUTPUT_FIFO_MODE | PACK_EN | UNPACK_EN);
	writel(cfg, i2c_base_addr + QUP_IO_MODES_OFFSET);
}

void i2c_qca_board_init(struct i2c_qup_bus *i2c_bus)
{
	/*Need to configure GPIO*/
	/* Configure the I2C clock */
	i2c_clock_config();

	i2c_hw_initialized = 0;
	i2c_board_initialized = 1;
}

void i2c_qup_mini_core_init(void)
{
	int cfg;

	cfg = readl(i2c_base_addr + QUP_CONFIG_OFFSET);
	cfg |= (QUP_CONFIG_MINI_CORE_I2C) | (I2C_BIT_WORD);

	writel(cfg, i2c_base_addr + QUP_CONFIG_OFFSET);

	writel(QUP_EN_VERSION_TWO_TAG, (i2c_base_addr +
					QUP_I2C_MASTER_CONFIG_OFFSET));
}

/*
 * QUP I2C Hardware Initialisation
 */
static int i2c_hw_init(void)
{
	int ret;

	/* QUP configuration */
	i2c_reset();

	/* Set the BLSP QUP state */
	ret = check_qup_state_valid();
	if (ret)
		return ret;

	writel(0,(i2c_base_addr + QUP_CONFIG_OFFSET));

	writel(QUP_APP_CLK_ON_EN | QUP_CORE_CLK_ON_EN |
		QUP_FIFO_CLK_GATE_EN, (i2c_base_addr + QUP_CONFIG_OFFSET));

	writel(0, i2c_base_addr + QUP_I2C_MASTER_CLK_CTL_OFFSET);
	writel(0, i2c_base_addr + QUP_TEST_CTRL_OFFSET);
	writel(0, i2c_base_addr + QUP_IO_MODES_OFFSET);
	writel(0, i2c_base_addr + QUP_OPERATIONAL_MASK_OFFSET);

	i2c_qup_mini_core_init();

	config_i2c_mode();

	writel(QUP_MX_READ_COUNT, i2c_base_addr + QUP_MX_READ_COUNT_OFFSET);
	writel(QUP_MX_WRITE_COUNT, i2c_base_addr + QUP_MX_WRITE_COUNT_OFFSET);


	writel(QUP_MX_INPUT_COUNT, i2c_base_addr + QUP_MX_INPUT_COUNT_OFFSET);
	writel(QUP_MX_OUTPUT_COUNT, i2c_base_addr + QUP_MX_OUTPUT_COUNT_OFFSET);

	ret = config_i2c_state(QUP_STATE_RESET);
	if (ret)
		return ret;

	i2c_hw_initialized = 1;

	return SUCCESS;
}

/*
 * Function to check wheather Input or Output FIFO
 * has data to be serviced. For invalid slaves, this
 * flag will not be set.
 */
static int check_fifo_status(uint dir)
{
	unsigned int count = TIMEOUT_CNT;
	unsigned int status_flag;
	unsigned int val;

	if (dir == READ) {
		do {
			val = readl(i2c_base_addr
				+ QUP_OPERATIONAL_OFFSET);
			count--;
			if (count == 0)
				return -ETIMEDOUT;
			status_flag = val & INPUT_SERVICE_FLAG;
			udelay(10);
		} while (!status_flag);

	} else if (dir == WRITE) {
		do {
			val = readl(i2c_base_addr
				+ QUP_OPERATIONAL_OFFSET);
			count--;
			if (count == 0)
				return -ETIMEDOUT;
			status_flag = val & OUTPUT_FIFO_FULL;
			udelay(10);
		} while (status_flag);
	}
	return SUCCESS;
}

/*
 * Check whether the values in the OUTPUT FIFO are shifted out.
 */
static int check_write_done(void)
{
	unsigned int count = TIMEOUT_CNT;
	unsigned int status_flag;
	unsigned int val;

	do {
		val = readl(i2c_base_addr
				+ QUP_OPERATIONAL_OFFSET);
		count--;
		if (count == 0)
			return -ETIMEDOUT;
		status_flag = val & OUTPUT_FIFO_NOT_EMPTY;
		udelay(10);
	} while (status_flag);

	return SUCCESS;
}

int i2c_process_read_data(uint32_t data, uchar *buffer, int len)
{
	int idx = 0;
	uint8_t data_8 = 0;
	int index = 0;
	int rd_len = len;

	while (index < 4 && rd_len) {
		data_8 = QUP_I2C_DATA(data);
		index++;
		if (data_8 == QUP_I2C_DATA_READ_AND_STOP_SEQ) {
			index++;
			data = (data >> 16);
			continue;
		}
		if (data_8 == QUP_I2C_STOP_SEQ)
			break;
		if (data_8 == QUP_I2C_NOP_PADDING) {
			data = (data >> 8);
			continue;
		}
		buffer[idx] = data_8;
		rd_len--;
		idx++;
		data = (data >> 8);
	}
	return idx;
}

uint32_t i2c_write_read_offset(uchar chip, int alen)
{
	uint32_t tag;
	uint32_t *fifo;

	fifo = (uint32_t *) (i2c_base_addr + QUP_OUTPUT_FIFO_OFFSET);
	tag = QUP_I2C_START_SEQ;
	tag |= ((QUP_I2C_ADDR(chip)) | (I2C_WRITE)) << 8;
	tag |= QUP_I2C_DATA_WRITE_SEQ << 16;
	tag |= alen << 24;
	writel(tag, fifo);

	return tag;
}

uint32_t i2c_write_read_tag(uchar chip, uint addr, int alen, int data_len)
{
	uint32_t tag = 0;
	uint32_t *fifo;

	fifo = (uint32_t *) (i2c_base_addr + QUP_OUTPUT_FIFO_OFFSET);

	if (alen == 2) {
		/* based on the slave send msb 8 bits or lsb 8 bits first */
		tag = QUP_I2C_DATA(addr);
		tag |= QUP_I2C_DATA(addr >> 8) << 8;
		tag |= QUP_I2C_START_SEQ << 16;
		tag |= ((QUP_I2C_ADDR(chip)) | (I2C_READ)) << 24;
		writel(tag, fifo);

		tag = 0;
		tag |= QUP_I2C_DATA_READ_AND_STOP_SEQ;
		tag |= data_len << 8;
		writel(tag, fifo);
	} else if (alen == 1) {
		tag = QUP_I2C_DATA(addr);
		tag |= QUP_I2C_START_SEQ << 8;
		tag |= ((QUP_I2C_ADDR(chip)) | (I2C_READ)) << 16;
		tag |= (QUP_I2C_DATA_READ_AND_STOP_SEQ << 24);
		writel(tag, fifo);

		tag = 0;
		tag |= data_len;
		writel(tag, fifo);
	} else if (alen == 0) {
		tag |= QUP_I2C_START_SEQ;
		tag |= ((QUP_I2C_ADDR(chip)) | (I2C_READ)) << 8;
		tag |= (QUP_I2C_DATA_READ_AND_STOP_SEQ << 16);
		tag |= data_len << 24;
		writel(tag, fifo);
	}
	return 0;
}

int i2c_read_data(struct i2c_qup_bus *i2c_bus, uchar chip, uint addr, int alen, uchar *buffer, int len)
{
	int ret = 0;
	int nack = 0;
	uint32_t data = 0;
	uint8_t data_len = len;
	uint32_t *fifo;
	int idx = 0;
	int cfg;

	config_i2c_state(QUP_STATE_RESET);

	if (!i2c_board_initialized) {
		i2c_qca_board_init(i2c_bus);
	}

	if (!i2c_hw_initialized) {
		i2c_hw_init();
	}

	writel(0x3C, i2c_base_addr + QUP_ERROR_FLAGS_OFFSET);
	writel(0x3C, i2c_base_addr + QUP_ERROR_FLAGS_EN_OFFSET);
	writel(0, i2c_base_addr + QUP_I2C_MASTER_STATUS_OFFSET);

	if (alen != 0)
		writel((OUT_FIFO_RD_TAG_BYTE_CNT + alen),
			i2c_base_addr + QUP_MX_WRITE_COUNT_OFFSET);
	else
		writel(OUT_FIFO_WR_TAG_BYTE_CNT,
			i2c_base_addr + QUP_MX_WRITE_COUNT_OFFSET);

	writel((IN_FIFO_TAG_BYTE_CNT + data_len),
			i2c_base_addr + QUP_MX_READ_COUNT_OFFSET);

	/* Set to RUN state */
	ret = config_i2c_state(QUP_STATE_RUN);
	if (ret != SUCCESS) {
		debug("State run failed\n");
		goto out;
	}

	/* Configure the I2C Master clock */
	cfg = (QUP_INPUT_CLK / (I2C_CLK_100KHZ * 2)) - 3;
	writel(cfg, i2c_base_addr + QUP_I2C_MASTER_CLK_CTL_OFFSET);
	/* Write to FIFO in Pause State */
	/* Set to PAUSE state */
	ret = config_i2c_state(QUP_STATE_PAUSE);
	if (ret != SUCCESS) {
		debug("State Pause failed\n");
		goto out;
	}

	fifo = (uint32_t *) (i2c_base_addr + QUP_OUTPUT_FIFO_OFFSET);

	if (alen != 0)
		data = i2c_write_read_offset(chip, alen);

	data = i2c_write_read_tag(chip, addr, alen, data_len);

	/* Set to RUN state */
	ret = config_i2c_state(QUP_STATE_RUN);
	if (ret != SUCCESS) {
		debug("State run failed\n");
		goto out;
	}

	mdelay(2);
	ret = check_write_done();
	if (ret != SUCCESS) {
		debug("Write done failed\n");
		goto out;
	}

	nack = readl(i2c_base_addr + QUP_I2C_MASTER_STATUS_OFFSET) & NACK_BIT_MASK;
	nack = nack >> NACK_BIT_SHIFT;
	if (nack == 1) {
		debug("NACK RECVD\n");
		return -ENACK;
	}

	if (readl(i2c_base_addr + QUP_OPERATIONAL_OFFSET)
			& OUTPUT_SERVICE_FLAG) {
		writel(OUTPUT_SERVICE_FLAG,
			i2c_base_addr + QUP_OPERATIONAL_OFFSET);
	}

	fifo = (uint32_t *)(i2c_base_addr + QUP_INPUT_FIFO_OFFSET);

	mdelay(2);
	ret = check_fifo_status(READ);
	if (ret != SUCCESS) {
		debug("Read status failed\n");
		goto out;
	}
	while (len) {
		/* Read the data from the FIFO */
		data = readl(fifo);

		ret = i2c_process_read_data(data, buffer + idx, len);
		if (ret) {
		    idx += ret;
		    len -= ret;
		}
	}

	if (readl(i2c_base_addr + QUP_OPERATIONAL_OFFSET)
		& INPUT_SERVICE_FLAG) {
		writel(INPUT_SERVICE_FLAG,
			i2c_base_addr + QUP_OPERATIONAL_OFFSET);
	}
	(void)config_i2c_state(QUP_STATE_RESET);
	return SUCCESS;
out:
	/*
	 * Put the I2C Core back in the Reset State to end the transfer.
	 */
	(void)config_i2c_state(QUP_STATE_RESET);
	writel(QUP_MX_READ_COUNT, i2c_base_addr + QUP_MX_READ_COUNT_OFFSET);
	return ret;
}

int create_data_byte(uint16_t *data, uchar *buffer, int len)
{
	int idx = 0;
	if (len == 0) {
		return 0;
	} else {
		*data = QUP_I2C_DATA(buffer[idx]);
		idx++;
		len--;
	}
	if (len == 0) {
	    return idx;
	} else {
		*data = (*data << 8);
		*data |= QUP_I2C_DATA(buffer[idx]);
		idx++;
		len--;
	}
	return idx;
}

uint32_t i2c_frame_wr_tag(uchar chip, uint8_t data_len, int alen)
{
	uint32_t tag;

	tag = QUP_I2C_START_SEQ;
	tag |= (((QUP_I2C_ADDR(chip)) | (I2C_WRITE)) << 8);
	tag |= (QUP_I2C_DATA_WRITE_AND_STOP_SEQ << 16);
	tag |= (data_len + alen) << 24;
	return tag;
}

int i2c_write_data(struct i2c_qup_bus *i2c_bus, uchar chip, uint addr, int alen, uchar *buffer, int len)
{
	int ret = 0;
	int nack = 0;
	int idx = 0;
	int first = 1;
	uint32_t data = 0;
	uint16_t data_lsb_16 = 0;
	uint16_t data_msb_16 = 0;
	uint8_t data_len = len;
	uint32_t *fifo;
	uint32_t cfg;

	/* Set to Reset State */
	ret = config_i2c_state(QUP_STATE_RESET);

	if (!i2c_board_initialized) {
		i2c_qca_board_init(i2c_bus);
	}

	if (!i2c_hw_initialized) {
		i2c_hw_init();
	}

	writel(0x3C, i2c_base_addr + QUP_ERROR_FLAGS_OFFSET);
	writel(0x3C, i2c_base_addr + QUP_ERROR_FLAGS_EN_OFFSET);
	writel(0, i2c_base_addr + QUP_I2C_MASTER_STATUS_OFFSET);

	writel((OUT_FIFO_WR_TAG_BYTE_CNT + len + alen),
		i2c_base_addr + QUP_MX_WRITE_COUNT_OFFSET);


	/* Set to RUN state */
	ret = config_i2c_state(QUP_STATE_RUN);
	if (ret != SUCCESS) {
		debug("State run failed\n");
		goto out;
	}

	/* Configure the I2C Master clock */
	cfg = (QUP_INPUT_CLK / (I2C_CLK_100KHZ * 2)) - 3;
	writel(cfg, i2c_base_addr + QUP_I2C_MASTER_CLK_CTL_OFFSET);


	/* Write to FIFO in Pause State */
	/* Set to PAUSE state */
	ret = config_i2c_state(QUP_STATE_PAUSE);
	if (ret != SUCCESS) {
		debug("State Pause failed\n");
		goto out;
	}
	fifo = (uint32_t *) (i2c_base_addr + QUP_OUTPUT_FIFO_OFFSET);
	data = i2c_frame_wr_tag(chip, data_len, alen);

	/* Write tags to the FIFO along with Slave address
	 * and Write len */
	writel(data, fifo);

	while (len > 0) {
		data_lsb_16 = 0;
		data_msb_16 = 0;
		data = 0;
		if ((first == 1) && (alen != 0)) {
			if (alen == 2) {
				/* based on the slave send msb 8 bits or lsb 8 bits first */
				data_lsb_16 = QUP_I2C_DATA(addr);
				data_lsb_16 |= QUP_I2C_DATA(addr >> 8) << 8;
			} else if (alen == 1) {
				data_lsb_16 = QUP_I2C_DATA(addr);
				data_lsb_16 |= QUP_I2C_DATA(buffer[idx]) << 8;
				idx++;
				len --;
			}
			first = 0;
			ret = 2;
		} else {
			ret = create_data_byte(&data_lsb_16, buffer + idx, len);
			idx += ret;
			len -= ret;
		}
		if(ret == 2) {
			ret = create_data_byte(&data_msb_16, buffer + idx, len);
			idx += ret;
			len -= ret;
		}
		data |= data_msb_16;
		data = (data << 16);
		data |= data_lsb_16;
		writel(data, fifo);
	}

	/* Set to RUN state */
	ret = config_i2c_state(QUP_STATE_RUN);
	if (ret != SUCCESS) {
		debug("State Run failed\n");
		goto out;
	}

	/* Clear Operational Flag */
	if (readl(i2c_base_addr + QUP_OPERATIONAL_OFFSET)
			& OUTPUT_SERVICE_FLAG) {
		writel(OUTPUT_SERVICE_FLAG,
			i2c_base_addr + QUP_OPERATIONAL_OFFSET);
	}

	mdelay(2);
	ret = check_write_done();
	if (ret != SUCCESS) {
		debug("Write done failed\n");
		goto out;
	}

	nack = readl(i2c_base_addr + QUP_I2C_MASTER_STATUS_OFFSET) & NACK_BIT_MASK;
	nack = nack >> NACK_BIT_SHIFT;
	if (nack == 1) {
		debug("NACK RECVD\n");
		return -ENACK;
	}
out:
	/*
	 * Put the I2C Core back in the Reset State to end the transfer.
	 */
	(void)config_i2c_state(QUP_STATE_RESET);
	return ret;
}

static int qup_i2c_xfer(struct udevice *bus, struct i2c_msg *msg,
			int nmsgs)
{
	struct i2c_qup_bus *i2c_bus = dev_get_priv(bus);
	int ret;

	debug("i2c_xfer: %d messages\n", nmsgs);
	for (; nmsgs > 0; nmsgs--, msg++) {
		debug("i2c_xfer: chip=0x%x, len=0x%x\n", msg->addr, msg->len);
		if (msg->flags & I2C_M_RD) {
		ret = i2c_read_data(i2c_bus, msg->addr, 0, 0, msg->buf,
					msg->len);
		} else {
			ret = i2c_write_data(i2c_bus, msg->addr, 0, 0, msg->buf,
					msg->len);
		}
		if (ret) {
			printf("i2c_write: error sending\n");
			return -EREMOTEIO;
		}
	}
	return 0;
}

/* Probe to see if a chip is present. */
static int qup_i2c_probe_chip(struct udevice *bus, uint chip_addr,
				uint chip_flags)
{
	uchar buf[1];

	struct i2c_qup_bus *i2c_bus = dev_get_priv(bus);
	buf[0] = 0;
	i2c_bus->i2c_base_addr = (int *)dev_get_addr(bus);
	i2c_base_addr = (int)i2c_bus->i2c_base_addr;

	return i2c_read_data(i2c_bus, chip_addr , 0x0, 0x0, buf, 0x1);
}

static const struct dm_i2c_ops qup_i2c_ops = {
	.xfer		= qup_i2c_xfer,
        .probe_chip	= qup_i2c_probe_chip,
};

static const struct udevice_id qpic_ver_ids[] = {
	{ .compatible = "qcom,qup-i2c"},
	{ },
};

U_BOOT_DRIVER(i2c_qup) = {
	.name   = "i2c_qup",
	.id     = UCLASS_I2C,
	.of_match = qpic_ver_ids,
	.priv_auto_alloc_size = sizeof(struct i2c_qup_bus),
	.ops    = &qup_i2c_ops,
};
