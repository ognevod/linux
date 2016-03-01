/*
 * Elvees VPOUT framebuffer driver
 *
 * it66121.c - helper functions for the ITE IT66121 I2C HDMI controller
 *
 * Copyright 2016, Elvees NeoTek JSC
 *
 * based on simplefb, which was:
 * Copyright 2013, Stephen Warren
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include "vpoutfb.h"
#include "it66121.h"

#define REG_RESET 0x04
#define REG_BANK_SW 0x0F
#define REG_IRQ_FIRST 0x09
#define REG_IRQ_LAST 0x0B
#define REG_TX_RESET 0x61
#define REG_AVMUTE 0xC1
#define REG_TXFIFO_SET 0x71

#define RESET_RCLK_MASK BIT(5)
#define RESET_AUDIO_MASK BIT(4)
#define RESET_VIDEO_MASK BIT(3)
#define RESET_AFIFO_MASK BIT(2)
#define RESET_HDCP_MASK BIT(0)

static int hdmi_write_masked(struct i2c_client *client, u8 address,
			     u8 value, u8 mask)
{
	int ret = 0;

	ret = i2c_smbus_read_byte_data(client, address);

	if (ret < 0)
		return ret;

	value = (value & mask) | (ret & ~mask);
	ret = i2c_smbus_write_byte_data(client, address, value);
	return (ret < 0) ? ret : 0;
}

/* fb_info will be needed to do modesetting-related operations */

int it66121_init(struct it66121_device_data *devdata,
			struct fb_info *info,
			struct device_node *output_node)
{
	u8 ident[] = {0x54, 0x49, 0x12, 0x16};
	int ret = 0, i;

	devdata->client = of_find_i2c_device_by_node(output_node);
	devdata->gpio_reset = devm_gpiod_get(&devdata->client->dev,
					     "reset",
					     GPIOF_OUT_INIT_HIGH |
					     GPIOF_ACTIVE_LOW);
	gpiod_direction_output(devdata->gpio_reset, 0);
	it66121_reset(devdata);
	/* Here and later: delays are so that errors do not occur */
	usleep_range(1000, 2000);
	/* Verify it's the correct device */
	for (i = 0; i < 4; i++) {
		if (ident[i] != i2c_smbus_read_byte_data(devdata->client, i))
			return -ENODEV;
	}
	/* reset whole circuit */
	ret |= hdmi_write_masked(devdata->client, REG_RESET, 0x20, 0x20);
	usleep_range(1000, 2000);
	/* each circuit is off so we can switch them on one by one */
	ret |= hdmi_write_masked(devdata->client, REG_RESET,
				 RESET_RCLK_MASK | RESET_AUDIO_MASK |
				 RESET_VIDEO_MASK | RESET_AFIFO_MASK |
				 RESET_HDCP_MASK,
				 0xFF);
	usleep_range(1000, 2000);
	/* hdmi tx flipflops reset */
	ret |= hdmi_write_masked(devdata->client, REG_TX_RESET, 0x10, 0xFF);

	/* DVI mode, packet interface off */
	for (i = 0xc0; i <= 0xd0; i++)
		ret |= hdmi_write_masked(devdata->client, i, 0, 0xFF);

	/* Ignore all the interrupts */
	for (i = REG_IRQ_FIRST; i <= REG_IRQ_LAST; i++)
		ret |= hdmi_write_masked(devdata->client, i, 0xFF, 0xFF);

	/* Enable video circuit */
	ret |= hdmi_write_masked(devdata->client, REG_RESET, 0x00,
				 RESET_VIDEO_MASK);
	/* Switch avmute on */
	ret |= hdmi_write_masked(devdata->client, REG_AVMUTE, 0x01, 0xFF);

	/* Disable audio */
	ret |= hdmi_write_masked(devdata->client, 0xE0, 0x00, 0xF);
	/* Enable video clock settings for <80 MHz */
	/* TODO: for 1080p and higher, set up for >80 MHz */
	ret |= hdmi_write_masked(devdata->client, 0x62, 0x18, 0xFF);
	ret |= hdmi_write_masked(devdata->client, 0x63, 0x10, 0xFF);
	ret |= hdmi_write_masked(devdata->client, 0x64, 0x0C, 0xFF);
	/* Clear TX FIFO */
	ret |= hdmi_write_masked(devdata->client, REG_TXFIFO_SET, 0x02, 0x02);
	usleep_range(1000, 2000);
	ret |= hdmi_write_masked(devdata->client, REG_TXFIFO_SET, 0x00, 0x02);
	/* It takes a while to measure clocks, so we give it time */
	msleep(120);

	for (i = 0; i < 10; i++) {
		if (!(i2c_smbus_read_byte_data(devdata->client, 0x0E) &
		      0x10)) {
			dev_err(&devdata->client->dev, "Video not stable!\n");
			usleep_range(10000, 20000);
		} else
			break;
	}
	if (i == 10) {
		it66121_reset(devdata);
		return -EBUSY;
	}
	/* Switch avmute off */
	ret |= hdmi_write_masked(devdata->client, REG_AVMUTE, 0x00, 0xFF);
	/* Turn on display */
	ret |= hdmi_write_masked(devdata->client, 0x61, 0x03, 0xFF);
	if (ret)
		it66121_reset(devdata);
	return ret;
};

void it66121_reset(struct it66121_device_data *devdata)
{
	gpiod_set_value(devdata->gpio_reset, 1);
	gpiod_set_value(devdata->gpio_reset, 0);
};
