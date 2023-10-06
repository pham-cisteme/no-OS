/***************************************************************************//**
 *   @file   max22196.c
 *   @brief  Source file of MAX22196 Driver.
 *   @author Radu Sabau (radu.sabau@analog.com)
********************************************************************************
 * Copyright 2023(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include "max22196.h"
#include "max149x6-base.h"
#include "no_os_util.h"
#include "no_os_alloc.h"

int max22196_reg_write(struct max22196_desc *desc, uint32_t reg, uint32_t val)
{
	struct no_os_spi_msg xfer = {
		.tx_buff = desc->buff,
		.bytes_number = MAX22196_FRAME_SIZE,
		.cs_change = 1,
	};

	desc->buff[0] = no_os_field_prep(MAX22196_ADDR_MASK, desc->chip_address) |
			no_os_field_prep(MAX22196_REG_ADDR_MASK, reg) |
			no_os_field_prep(MAX22196_RW_MASK, 1);
	desc->buff[1] = val;

	if (desc->crc_en) {
		xfer.bytes_number++;
		desc->buff[2] = max149x6_crc(desc->buff, true);
	}

	return no_os_spi_transfer(desc->comm_desc, &xfer, 1);
}

int max22196_reg_read(struct max22196_desc *desc, uint32_t reg, uint32_t *val)
{
	struct no_os_spi_msg xfer = {
		.tx_buff = desc->buff,
		.rx_buff = desc->buff,
		.bytes_number = MAX22196_FRAME_SIZE,
		.cs_change = 1,
	};
	uint8_t crc;
	int ret;

	if (desc->crc_en)
		xfer.bytes_number++;

	memset(desc->buff, 0, xfer.bytes_number);
	desc->buff[0] = no_os_field_prep(MAX22196_ADDR_MASK, desc->chip_address) |
			no_os_field_prep(MAX22196_REG_ADDR_MASK, reg) |
			no_os_field_prep(MAX22196_RW_MASK, 0);

	if (desc->crc_en)
		desc->buff[2] = max149x6_crc(&desc->buff[0], true);

	ret = no_os_spi_transfer(desc->comm_desc, &xfer, 1);
	if (ret)
		return ret;

	if (desc->crc_en) {
		crc = max149x6_crc(&desc->buff[0], false);
		if (crc != desc->buff[2])
			return -EINVAL;
	}

	*val = desc->buff[1];

	return 0;
}

int max22196_reg_update(struct max22196_desc *desc, uint32_t reg, uint32_t mask,
			uint32_t val)
{
	int ret;
	uint32_t reg_val = 0;

	ret = max22196_reg_read(desc, reg, &reg_val);
	if (ret)
		return ret;

	reg_val &= ~mask;
	reg_val |= mask & val;

	return max22196_reg_write(desc, reg, reg_val);
}

int max22196_chan_cfg(struct max22196_desc *desc, uint32_t ch, uint32_t hi_thr,
		      uint32_t source, enum max22196_curr curr, uint32_t flt_en,
		      enum max22196_delay delay)
{
	uint32_t cfg_val;

	if (ch > MAX22196_CHANNELS - 1)
		return -EINVAL;

	cfg_val = no_os_field_prep(MAX22196_HITHR_MASK, hi_thr) |
		  no_os_field_prep(MAX22196_SOURCE_MASK, source) |
		  no_os_field_prep(MAX22196_CURR_MASK, curr) |
		  no_os_field_prep(MAX22196_FLTEN_MASK, flt_en) |
		  no_os_field_prep(MAX22196_DELAY_MASK, delay);

	return max22196_reg_write(desc, MAX22196_CFG_REG(ch), cfg_val);
}

int max22196_set_chan_cnt(struct max22196_desc *desc, uint32_t ch,
			  uint8_t cnt_msb_byte, uint8_t cnt_lsb_byte)
{
	int ret;

	if (ch > MAX22196_CHANNELS - 1)
		return -EINVAL;

	ret = max22196_reg_update(desc, MAX22196_START_STOP_REG,
				  MAX22196_CNT_MASK(ch),
				  no_os_field_prep(MAX22196_CNT_MASK(ch), 0));
	if (ret)
		return ret;

	ret = max22196_reg_write(desc, MAX22196_CNT_LSB_REG(ch), cnt_lsb_byte);
	if (ret)
		return ret;

	ret = max22196_reg_write(desc, MAX22196_CNT_MSB_REG(ch), cnt_msb_byte);
	if (ret)
		return ret;

	return max22196_reg_update(desc, MAX22196_START_STOP_REG,
				   MAX22196_CNT_MASK(ch),
				   no_os_field_prep(MAX22196_CNT_MASK(ch), 1));
}

int max22196_get_chan_cnt(struct max22196_desc *desc, uint32_t ch,
			  uint8_t *cnt_msb_byte, uint8_t *cnt_lsb_byte)
{
	int ret;

	if (ch > MAX22196_CHANNELS - 1)
		return -EINVAL;

	ret = max22196_reg_update(desc, MAX22196_START_STOP_REG,
				  MAX22196_CNT_MASK(ch),
				  no_os_field_prep(MAX22196_CNT_MASK(ch), 0));
	if (ret)
		return ret;

	ret = max22196_reg_read(desc, MAX22196_CNT_LSB_REG(ch), cnt_lsb_byte);
	if (ret)
		return ret;

	ret = max22196_reg_read(desc, MAX22196_CNT_MSB_REG(ch), cnt_msb_byte);
	if (ret)
		return ret;

	return max22196_reg_update(desc, MAX22196_START_STOP_REG,
				   MAX22196_CNT_MASK(ch),
				   no_os_field_prep(MAX22196_CNT_MASK(ch), 1));
}

int max22196_init(struct max22196_desc **desc,
		  struct max22196_init_param *param)
{
	struct max22196_desc *descriptor;
	uint32_t reg_val;
	int ret;
	int i;

	descriptor = no_os_calloc(1, sizeof(*descriptor));
	if (!descriptor)
		return -ENOMEM;
	ret = no_os_spi_init(&descriptor->comm_desc, param->comm_param);
	if (ret)
		goto error;

	ret = no_os_gpio_get_optional(&descriptor->crc_desc, param->crc_param);
	if (ret)
		goto spi_error;

	if (descriptor->crc_desc) {
		ret = no_os_gpio_get_value(descriptor->crc_desc, &reg_val);
		if (ret)
			goto gpio_error;
		if (reg_val == NO_OS_GPIO_HIGH)
			descriptor->crc_en = true;
		else
			descriptor->crc_en = false;
	}

	/* Clear the latched faults generated at power-up of MAX22196. */
	ret = max22196_reg_read(descriptor, MAX22196_FAULT1_REG, &reg_val);
	if (ret)
		goto gpio_error;

	ret = max22196_reg_read(descriptor, MAX22196_FAULT2_REG, &reg_val);
	if (ret)
		goto gpio_error;

	*desc = descriptor;

	return 0;
gpio_error:
	no_os_gpio_remove(descriptor->crc_desc);
spi_error:
	no_os_spi_remove(descriptor->comm_desc);
error:
	no_os_free(descriptor);

	return ret;
}

int max22196_remove(struct max22196_desc *desc)
{
	int ret, i;

	if (!desc)
		return -ENODEV;

	for (i = 0; i < MAX22196_CHANNELS; i++) {
		ret = max22196_set_chan_cnt(desc, i, 0, 0);
		if (ret)
			return ret;
	}

	no_os_spi_remove(desc->comm_desc);

	no_os_free(desc);

	return 0;
}
