/***************************************************************************//**
 *   @file   iio_max22190.c
 *   @brief  Source file of MAX22190 IIO Driver.
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
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "no_os_alloc.h"
#include "no_os_error.h"
#include "no_os_units.h"
#include "no_os_util.h"

#include "max22190.h"
#include "iio_max22190.h"

#define MAX22190_CHANNEL(_addr)			\
        {					\
            	.ch_type = IIO_VOLTAGE,		\
        	.indexed = 1,			\
		.channel = _addr,		\
	    	.address = _addr,		\
	}

static int max22190_iio_read_raw(void *dev, char *buf, uint32_t len,
				 const struct iio_ch_info *channel,
				 intptr_t priv);

static int max22190_iio_read_offset(void *dev, char *buf, uint32_t len,
				    const struct iio_ch_info *channel,
				    intptr_t priv);

static int max22190_iio_read_scale(void *dev, char *buf, uint32_t len,
				   const struct iio_ch_info *channel,
				   intptr_t priv);

static int max22190_iio_read_filter(void *dev, char *buf, uint32_t len,
				    const struct iio_ch_info *channel,
				    intptr_t priv);

static int max22190_iio_write_filter(void *dev, char *buf, uint32_t len,
				     const struct iio_ch_info *channel,
				     intptr_t priv);

static int max22190_iio_read_filter_delay(void *dev, char *buf, uint32_t len,
		const struct iio_ch_info *channel,
		intptr_t priv);

static int max22190_iio_write_filter_delay(void *dev, char *buf, uint32_t len,
		const struct iio_ch_info *channel,
		intptr_t priv);

static int max22190_iio_read_filter_available(void *dev, char *buf,
		uint32_t len,
		const struct iio_ch_info *channel,
		intptr_t priv);

static int max22190_iio_read_fault1(void *dev, char *buf, uint32_t len,
				    const struct iio_ch_info *channel,
				    intptr_t priv);

static int max22190_iio_read_fault2(void *dev, char *buf, uint32_t len,
				    const struct iio_ch_info *channel,
				    intptr_t priv);

static int max22190_iio_reg_read(struct max22190_iio_desc *dev, uint32_t reg,
				 uint32_t *readval);

static int max22190_iio_reg_write(struct max22190_iio_desc *dev, uint32_t reg,
				  uint32_t writeval);

static int max22190_iio_setup_channels(struct max22190_iio_desc *desc,
				       bool *ch_enabled);

static const uint32_t max22190_delay_avail[8] = {
	50, 100, 400, 800, 1800, 3200, 12800, 20000
};

static struct iio_attribute max22190_attrs[] = {
	{
		.name = "raw",
		.show = max22190_iio_read_raw,
	},
	{
		.name = "offset",
		.show = max22190_iio_read_offset,
	},
	{
		.name = "scale",
		.show = max22190_iio_read_scale,
	},
	{
		.name = "filter_bypass",
		.show = max22190_iio_read_filter,
		.store = max22190_iio_write_filter,
	},
	{
		.name = "filter_delay",
		.show = max22190_iio_read_filter_delay,
		.store = max22190_iio_write_filter_delay,
	},
	{
		.name = "filter_delay_available",
		.show = max22190_iio_read_filter_available,
		.shared = IIO_SHARED_BY_ALL
	},
	END_ATTRIBUTES_ARRAY
};

static struct iio_attribute max22190_debug_attrs[] = {
	{
		.name = "fault1",
		.show = max22190_iio_read_fault1,
	},
	{
		.name = "fault2",
		.show = max22190_iio_read_fault2,
	},
	END_ATTRIBUTES_ARRAY
};

static struct iio_device max22190_iio_dev = {
	.debug_reg_read = (int32_t (*)())max22190_iio_reg_read,
	.debug_reg_write = (int32_t (*)())max22190_iio_reg_write,
	.debug_attributes = max22190_debug_attrs,
};

/**
 * @brief Read the raw attribute for a specific channel
 * @param dev - The iio device structure
 * @param buf - Buffer to be filled with requested data.
 * @param len - Length of the received command buffer in bytes.
 * @param channel - Command channel info.
 * @param priv - Private descriptor.
 * @return 0 in case of succes, error code otherwise.
*/
static int max22190_iio_read_raw(void *dev, char *buf, uint32_t len,
				 const struct iio_ch_info *channel,
				 intptr_t priv)
{
	struct max22190_iio_desc *desc = dev;
	uint32_t val;
	int ret, ch;

	ch = channel->ch_num;

	ret = max22190_reg_read(desc->max22190_desc, MAX22190_DIGITAL_INPUT_REG,
				&val);
	if (ret)
		return ret;

	val = no_os_field_get(MAX22190_CH_STATE_MASK(ch), val);

	return iio_format_value(buf, len, IIO_VAL_INT, 1, (int32_t *)&val);
}

/**
 * @brief Read the offset attribute for a specific channel
 * @param dev - The iio device structure
 * @param buf - Buffer to be filled with requested data.
 * @param len - Length of the received command buffer in bytes.
 * @param channel - Command channel info.
 * @param priv - Private descriptor.
 * @return 0 in case of succes, error code otherwise.
*/
static int max22190_iio_read_offset(void *dev, char *buf, uint32_t len,
				    const struct iio_ch_info *channel,
				    intptr_t priv)
{
	int32_t val = 0;

	return iio_format_value(buf, len, IIO_VAL_INT, 1, &val);
}

/**
 * @brief Read the scale attribute for a specific channel
 * @param dev - The iio device structure
 * @param buf - Buffer to be filled with requested data.
 * @param len - Length of the received command buffer in bytes.
 * @param channel - Command channel info.
 * @param priv - Private descriptor.
 * @return 0 in case of succes, error code otherwise.
*/
static int max22190_iio_read_scale(void *dev, char *buf, uint32_t len,
				   const struct iio_ch_info *channel,
				   intptr_t priv)
{
	int32_t val = 1;

	return iio_format_value(buf, len, IIO_VAL_INT, 1, &val);
}

/**
 * @brief Read the filter bypass attribute for a specific channel
 * @param dev - The iio device structure
 * @param buf - Buffer to be filled with requested data.
 * @param len - Length of the received command buffer in bytes.
 * @param channel - Command channel info.
 * @param priv - Private descriptor.
 * @return 0 in case of succes, error code otherwise.
*/
static int max22190_iio_read_filter(void *dev, char *buf, uint32_t len,
				    const struct iio_ch_info *channel,
				    intptr_t priv)
{
	struct max22190_iio_desc *desc = dev;
	uint32_t val;
	int ret, ch;

	ch = channel->ch_num;

	ret = max22190_reg_read(desc->max22190_desc, MAX22190_FILTER_IN_REG(ch),
				&val);
	if (ret)
		return ret;

	val = no_os_field_get(MAX22190_FBP_MASK, val);

	return iio_format_value(buf, len, IIO_VAL_INT, 1, (int32_t *)&val);
}

/**
 * @brief Write the filter bypass attribute for a specific channel
 * @param dev - The iio device structure
 * @param buf - Buffer to be filled with requested data.
 * @param len - Length of the received command buffer in bytes.
 * @param channel - Command channel info.
 * @param priv - Private descriptor.
 * @return 0 in case of succes, error code otherwise.
*/
static int max22190_iio_write_filter(void *dev, char *buf, uint32_t len,
				     const struct iio_ch_info *channel,
				     intptr_t priv)
{
	struct max22190_iio_desc *desc = dev;
	int ch;
	uint32_t val;

	ch = channel->ch_num;

	iio_parse_value(buf, IIO_VAL_INT, (int32_t *)&val, NULL);

	if (val > 1)
		return -EINVAL;

	return max22190_reg_update(desc->max22190_desc,
				   MAX22190_FILTER_IN_REG(ch),
				   MAX22190_FBP_MASK, val);
}

/**
 * @brief Read the filter delay attribute for a specific channel
 * @param dev - The iio device structure
 * @param buf - Buffer to be filled with requested data.
 * @param len - Length of the received command buffer in bytes.
 * @param channel - Command channel info.
 * @param priv - Private descriptor.
 * @return 0 in case of succes, error code otherwise.
*/
static int max22190_iio_read_filter_delay(void *dev, char *buf, uint32_t len,
		const struct iio_ch_info *channel,
		intptr_t priv)
{
	struct max22190_iio_desc *desc = dev;
	int ret, ch;
	uint32_t val;

	ch = channel->ch_num;

	ret = max22190_reg_read(desc->max22190_desc, MAX22190_FILTER_IN_REG(ch),
				&val);
	if (ret)
		return ret;

	val = max22190_delay_avail[no_os_field_get(MAX22190_DELAY_MASK, val)];

	return iio_format_value(buf, len, IIO_VAL_INT, 1, (int32_t *)&val);
}

/**
 * @brief Write the filter delay attribute for a specific channel
 * @param dev - The iio device structure
 * @param buf - Buffer to be filled with requested data.
 * @param len - Length of the received command buffer in bytes.
 * @param channel - Command channel info.
 * @param priv - Private descriptor.
 * @return 0 in case of succes, error code otherwise.
*/
static int max22190_iio_write_filter_delay(void *dev, char *buf, uint32_t len,
		const struct iio_ch_info *channel,
		intptr_t priv)
{
	struct max22190_iio_desc *desc = dev;
	int ch;
	uint32_t val, i;

	ch = channel->ch_num;

	iio_parse_value(buf, IIO_VAL_INT, (int32_t *)&val, NULL);

	for (i = 0; i < NO_OS_ARRAY_SIZE(max22190_delay_avail); i++) {
		if (val == max22190_delay_avail[i])
			break;

		if (i == NO_OS_ARRAY_SIZE(max22190_delay_avail) - 1)
			return -EINVAL;
	}

	return max22190_reg_update(desc->max22190_desc,
				   MAX22190_FILTER_IN_REG(ch),
				   MAX22190_DELAY_MASK,
				   no_os_field_prep(MAX22190_DELAY_MASK, i));
}

/**
 * @brief Read the available values for filter delay attribute for a specific
 * 	  channel
 * @param dev - The iio device structure
 * @param buf - Buffer to be filled with requested data.
 * @param len - Length of the received command buffer in bytes.
 * @param channel - Command channel info.
 * @param priv - Private descriptor.
 * @return 0 in case of succes, error code otherwise.
*/
static int max22190_iio_read_filter_available(void *dev, char *buf,
		uint32_t len,
		const struct iio_ch_info *channel,
		intptr_t priv)
{
	uint32_t avail_size = NO_OS_ARRAY_SIZE(max22190_delay_avail);
	uint32_t length = 0;
	uint32_t i;

	for (i = 0; i < avail_size; i++)
		length += sprintf(buf + length, "%d ", max22190_delay_avail[i]);

	return length;
}

/**
 * @brief Read the fault1 attribute for a specific channel
 * @param dev - The iio device structure
 * @param buf - Buffer to be filled with requested data.
 * @param len - Length of the received command buffer in bytes.
 * @param channel - Command channel info.
 * @param priv - Private descriptor.
 * @return 0 in case of succes, error code otherwise.
*/
static int max22190_iio_read_fault1(void *dev, char *buf, uint32_t len,
				    const struct iio_ch_info *channel,
				    intptr_t priv)
{
	struct max22190_iio_desc *desc = dev;
	uint32_t val;
	int ret;

	ret = max22190_reg_read(desc->max22190_desc, MAX22190_FAULT1_REG, &val);
	if (ret)
		return ret;

	return iio_format_value(buf, len, IIO_VAL_INT, 1, (int32_t *)&val);
}

/**
 * @brief Read the fault2 attribute for a specific channel
 * @param dev - The iio device structure
 * @param buf - Buffer to be filled with requested data.
 * @param len - Length of the received command buffer in bytes.
 * @param channel - Command channel info.
 * @param priv - Private descriptor.
 * @return 0 in case of succes, error code otherwise.
*/
static int max22190_iio_read_fault2(void *dev, char *buf, uint32_t len,
				    const struct iio_ch_info *channel,
				    intptr_t priv)
{
	struct max22190_iio_desc *desc = dev;
	uint32_t val;
	int ret;

	ret = max22190_reg_read(desc->max22190_desc, MAX22190_FAULT2_REG, &val);
	if (ret)
		return ret;

	return iio_format_value(buf, len, IIO_VAL_INT, 1, (int32_t *)&val);
}

/**
 * @brief Register read wrapper
 * @param dev - The iio device structure.
 * @param reg - The register's address.
 * @param readval - Register value.
 * @return 0 in case of succes, error code otherwise
*/
static int max22190_iio_reg_read(struct max22190_iio_desc *dev, uint32_t reg,
				 uint32_t *readval)
{
	return max22190_reg_read(dev->max22190_desc, reg, readval);
}

/**
 * @brief Register write wrapper
 * @param dev - The iio device structure.
 * @param reg - The register's address.
 *
 * @param writeval - Register value.
 *
 * @return 0 in case of succes, error code otherwise
*/
static int max22190_iio_reg_write(struct max22190_iio_desc *dev, uint32_t reg,
				  uint32_t writeval)
{
	return max22190_reg_write(dev->max22190_desc, reg, writeval);
}

/**
 * @brief Configure a set of IIO channels based on the desired channels.
 * @param desc - MAX22190 IIO device descriptor.
 * @param ch_enabled - bool array with the requsted channels.
 * @return 0 in case of succes, negative error code otherwise.
*/
static int max22190_iio_setup_channels(struct max22190_iio_desc *desc,
				       bool *ch_enabled)
{
	struct iio_channel *max22190_iio_channels;
	uint32_t enabled_ch = 0;
	uint32_t ch_offset = 0;
	uint32_t i;

	for (i = 0; i < MAX22190_CHANNELS; i++) {
		if (ch_enabled[i])
			enabled_ch++;
	}

	max22190_iio_channels = no_os_calloc(enabled_ch,
					     sizeof(*max22190_iio_channels));
	if (!max22190_iio_channels)
		return -ENOMEM;

	for (i = 0; i < MAX22190_CHANNELS; i++) {
		if (ch_enabled[i]) {
			max22190_iio_channels[ch_offset] = (struct iio_channel)MAX22190_CHANNEL(i);
			max22190_iio_channels[ch_offset].attributes = max22190_attrs;
			max22190_iio_channels[ch_offset].ch_out = 0;
			ch_offset++;
		}
	}

	desc->iio_dev->channels = max22190_iio_channels;
	desc->iio_dev->num_ch = enabled_ch;

	return 0;
}

/**
 * @brief Initializes the MAX22190 IIO Descriptor
 * @param iio_desc - The iio device descriptor.
 * @param init_param - The structure that contains the device intial parameters.
 * @return 0 in case of succes, an error code otherwise.
*/
int max22190_iio_init(struct max22190_iio_desc **iio_desc,
		      struct max22190_iio_desc_init_param *init_param)
{
	struct max22190_iio_desc *descriptor;
	int ret, i;

	if (!init_param || !init_param->max22190_init_param)
		return -EINVAL;

	descriptor = no_os_calloc(1, sizeof(*descriptor));
	if (!descriptor)
		return -ENOMEM;

	ret = max22190_init(&descriptor->max22190_desc,
			    init_param->max22190_init_param);
	if (ret)
		goto free_desc;

	descriptor->iio_dev = &max22190_iio_dev;

	ret = max22190_iio_setup_channels(descriptor, init_param->ch_enabled);
	if (ret)
		goto free_dev;

	*iio_desc = descriptor;

	return 0;
free_dev:
	max22190_remove(descriptor->max22190_desc);
free_desc:
	no_os_free(descriptor);
	return ret;
}

/**
 * @brief Free the resources allocated by max22190_iio_init().
 * @param iio_desc - The IIO device structure.
 * @return ret - Result of the remove procedure.
*/
int max22190_iio_remove(struct max22190_iio_desc *iio_desc)
{
	if (!iio_desc)
		return -ENODEV;

	no_os_free(iio_desc->iio_dev->channels);
	max22190_remove(iio_desc->max22190_desc);
	no_os_free(iio_desc);

	return 0;
}
