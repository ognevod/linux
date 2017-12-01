/*
 * Copyright 2017 RnD Center "ELVEES", JSC
 *
 * Based on Omnivision OV5642 Camera Driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/v4l2-mediabus.h>
#include <linux/videodev2.h>
#include <linux/log2.h>
#include <linux/bitops.h>
#include <linux/time.h>

#include <media/soc_camera.h>
#include <media/v4l2-clk.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-image-sizes.h>

#define OV5647_MAX_WIDTH 2592
#define OV5647_MAX_HEIGHT 1944
#define OV5647_DEFAULT_WIDTH 1920
#define OV5647_DEFAULT_HEIGHT 1080

#define REG_RESET			0x0103
#define REG_EXPOSURE_HHIGH		0x3500
#define REG_EXPOSURE_HIGH		0x3501
#define REG_EXPOSURE_LOW		0x3502
#define REG_MANUAL_CONTROL		0x3503
#define REG_AGC_HIGH			0x350A
#define REG_AGC_LOW			0x350B
#define REG_VTS_DIFF_HIGH		0x350C
#define REG_VTS_DIFF_LOW		0x350D
#define REG_TIMING_X_ADDR_START_HIGH	0x3800
#define REG_TIMING_X_ADDR_START_LOW	0x3801
#define REG_TIMING_Y_ADDR_START_HIGH	0x3802
#define REG_TIMING_Y_ADDR_START_LOW	0x3803
#define REG_TIMING_X_ADDR_END_HIGH	0x3804
#define REG_TIMING_X_ADDR_END_LOW	0x3805
#define REG_TIMING_Y_ADDR_END_HIGH	0x3806
#define REG_TIMING_Y_ADDR_END_LOW	0x3807
#define REG_TIMING_X_OUTPUT_SIZE_HIGH	0x3808
#define REG_TIMING_X_OUTPUT_SIZE_LOW	0x3809
#define REG_TIMING_Y_OUTPUT_SIZE_HIGH	0x380a
#define REG_TIMING_Y_OUTPUT_SIZE_LOW	0x380b
#define REG_TIMING_HTS_HIGH		0x380c
#define REG_TIMING_HTS_LOW		0x380d
#define REG_TIMING_VTS_HIGH		0x380e
#define REG_TIMING_VTS_LOW		0x380f
#define REG_TIMING_TC_REG20		0x3820
#define REG_TIMING_TC_REG21		0x3821
#define REG_AEC_CTRL00			0x3a00
#define REG_MIPI_CTRL00			0x4800
#define REG_MIPI_CTRL01			0x4801
#define REG_ISP_CTRL01			0x5001
#define REG_ISP_CTRL02			0x5002

/* AEC MANUAL */
#define REG_MANUAL_CONTROL_AGC		0x02
#define REG_MANUAL_CONTROL_AEC		0x01

/* ISP CONTROL1 */
#define REG_ISP_CTRL01_AWB_EN		0x01

/* ISP CONTROL2 */
#define REG_ISP_CTRL02_AWB_GAIN_EN	0x01

#define OV5647_TABLE_END		0
#define OV5647_TABLE_WAIT_MS		1

struct ov5647_reg {
	u16 addr;
	u8 val;
};

/* Register setting for 1920*1080@30fps */
static struct ov5647_reg mode_1920x1080[] = {
	{0x0103, 0x01},
	{0x0100, 0x00},
	{OV5647_TABLE_WAIT_MS, 176},

	{0x3034, 0x1a},
	{0x3035, 0x21},
	{0x3036, 0x62},
	{0x3037, 0x03},

	{0x303c, 0x11},
	{0x3106, 0xf5},

	{0x5000, 0x06},
	{0x5003, 0x08},
	{0x5a00, 0x08},
	{0x3000, 0x00},
	{0x3001, 0x00},
	{0x3002, 0x00},
	{0x3016, 0x08},
	{0x3017, 0xe0},

	{0x3018, 0x44},

	{0x3a18, 0x00},
	{0x3a19, 0xf8},
	{0x3c01, 0x80},
	{0x3b07, 0x0c},
	{0x380c, 0x09},
	{0x380d, 0x70},
	{0x3814, 0x11},
	{0x3815, 0x11},

	{0x3808, 0x07},
	{0x3809, 0x80},
	{0x380a, 0x04},
	{0x380b, 0x38},
	{0x3800, 0x01},
	{0x3801, 0x5c},
	{0x3802, 0x01},
	{0x3803, 0xb2},
	{0x3804, 0x08},
	{0x3805, 0xe3},
	{0x3806, 0x05},
	{0x3807, 0xf1},
	{0x3811, 0x04},
	{0x3813, 0x02},

	{0x3630, 0x2e},
	{0x3632, 0xe2},
	{0x3633, 0x23},
	{0x3634, 0x44},
	{0x3636, 0x06},

	{0x4837, 0x19},
	{0x4800, 0x34},

	{0x3820, 0x00},
	{0x3821, 0x06},
	{0x0100, 0x01},
	{0x380f, 0x50},
	{0x380e, 0x04},
	{0x503d, 0x00},
	{OV5647_TABLE_END},
};

struct ov5647_color_format {
	u32 code;
	enum v4l2_colorspace colorspace;
};

struct ov5647_priv {
	struct v4l2_subdev               subdev;
	struct v4l2_ctrl_handler	 hdl;
	struct {
		/* gain cluster */
		struct v4l2_ctrl *auto_gain;
		struct v4l2_ctrl *gain;
	};
	struct {
		/* exposure cluster */
		struct v4l2_ctrl *auto_exp;
		struct v4l2_ctrl *exp;
		struct v4l2_ctrl *exp_abs;
	};
	struct v4l2_ctrl		 *awb;
	struct v4l2_rect		 crop_rect;
	struct v4l2_clk			 *clk;
	const struct ov5647_color_format *cfmt;
	/* blanking information */
	int total_width;
	int total_height;
	int fps;
	int xvclk;
};

/*
 * supported color format list
 */
static const struct ov5647_color_format ov5647_cfmts[] = {
	{
		.code		= MEDIA_BUS_FMT_SBGGR8_1X8,
		.colorspace	= V4L2_COLORSPACE_SRGB,
	},
};

struct regval_list {
	u16 reg_num;
	u8 value;
};

static int reg_read(struct i2c_client *client, u16 reg, u8 *val)
{
	int ret;
	/* We have 16-bit i2c addresses - care for endianness */
	u8 data[2] = { reg >> 8, reg & 0xff };

	ret = i2c_master_send(client, data, 2);
	if (ret < 2) {
		dev_err(&client->dev, "%s: i2c read error, reg: 0x%x\n",
			__func__, reg);
		return ret < 0 ? ret : -EIO;
	}

	ret = i2c_master_recv(client, val, 1);
	if (ret < 1) {
		dev_err(&client->dev, "%s: i2c read error, reg: 0x%x\n",
				__func__, reg);
		return ret < 0 ? ret : -EIO;
	}
	return 0;
}

static int reg_write(struct i2c_client *client, u16 reg, u8 val)
{
	int ret;
	u8 data[3] = { reg >> 8, reg & 0xff, val };

	ret = i2c_master_send(client, data, 3);
	if (ret < 3) {
		dev_err(&client->dev, "%s: i2c write error, reg: 0x%x\n",
			__func__, reg);
		return ret < 0 ? ret : -EIO;
	}

	return 0;
}

static struct ov5647_priv *to_ov5647(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client),
			struct ov5647_priv, subdev);
}

static int ov5647_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad || code->index >= ARRAY_SIZE(ov5647_cfmts))
		return -EINVAL;

	code->code = ov5647_cfmts[code->index].code;
	return 0;
}

static int ov5647_cropcap(struct v4l2_subdev *sd, struct v4l2_cropcap *a)
{
	a->bounds.left			= 0;
	a->bounds.top			= 0;
	a->bounds.width			= OV5647_MAX_WIDTH;
	a->bounds.height		= OV5647_MAX_HEIGHT;
	a->defrect			= a->bounds;
	a->type				= V4L2_BUF_TYPE_VIDEO_CAPTURE;
	a->pixelaspect.numerator	= 1;
	a->pixelaspect.denominator	= 1;

	return 0;
}

static int ov5647_g_mbus_config(struct v4l2_subdev *sd,
				struct v4l2_mbus_config *cfg)
{
	cfg->type = V4L2_MBUS_CSI2;
	cfg->flags = V4L2_MBUS_CSI2_2_LANE | V4L2_MBUS_CSI2_CHANNEL_0 |
					V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
	return 0;
}

/* Find a data format by a pixel code in an array */
static const struct ov5647_color_format
			*ov5647_find_datafmt(u32 code)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ov5647_cfmts); i++)
		if (ov5647_cfmts[i].code == code)
			return &ov5647_cfmts[i];

	return NULL;
}

static int set_gain(struct i2c_client *client, s32 gain)
{
	u8 low, high, ret;

	low = (u8)(gain);
	high = (u8)((gain & 0x300) >> 8);

	ret = reg_write(client, REG_AGC_HIGH, high);
	if (ret)
		return ret;

	return reg_write(client, REG_AGC_LOW, high | low);
}

static int set_exposure(struct i2c_client *client, s32 exposure)
{
	u8 val, ret;

	val = (u8)(exposure & 0xff);
	ret = reg_write(client, REG_EXPOSURE_LOW, val);
	if (ret)
		return ret;

	val = (u8)((exposure >> 8) & 0xff);
	ret = reg_write(client, REG_EXPOSURE_HIGH, val);
	if (ret)
		return ret;

	val = (u8)((exposure >> 16) & 0xf);
	return reg_write(client, REG_EXPOSURE_HHIGH, val);
}

static int ov5647_try_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov5647_priv *priv = to_ov5647(client);
	struct v4l2_mbus_framefmt *mf = &format->format;
	const struct ov5647_color_format *fmt = ov5647_find_datafmt(mf->code);

	dev_dbg(&client->dev, "%s: %dx%d\n", __func__, mf->width, mf->height);
	mf->width = priv->crop_rect.width;
	mf->height = priv->crop_rect.height;

	if (!fmt) {
		mf->code = ov5647_cfmts[0].code;
		mf->colorspace = ov5647_cfmts[0].colorspace;
	}

	mf->field = V4L2_FIELD_NONE;

	return 0;
}

static int ov5647_s_crop(struct v4l2_subdev *sd, const struct v4l2_crop *a)
{
	return -EINVAL;
}

static int ov5647_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov5647_priv *priv = to_ov5647(client);

	if (param->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	param->parm.capture = (struct v4l2_captureparm) {
		.capability = V4L2_CAP_TIMEPERFRAME,
		.timeperframe = {
			.numerator = 1,
			.denominator = priv->fps
		}
	};

	return 0;
}

/* The exposure value in registers is in units of line period / 16. */
static u32 exposure_reg_to_100us(struct ov5647_priv *priv, u32 value)
{
	return value * (USEC_PER_SEC / 100) /
			(priv->total_height * priv->fps) / 16;
}

static u32 exposure_100us_to_reg(struct ov5647_priv *priv, u32 value)
{
	return value * (priv->total_height * priv->fps) * 16 /
			(USEC_PER_SEC / 100);
}

static int ov5647_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ov5647_priv *priv = container_of(ctrl->handler,
						struct ov5647_priv, hdl);
	struct v4l2_subdev *sd = &priv->subdev;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	u8 val, high = 1, low;

	switch (ctrl->id) {
	case V4L2_CID_AUTOGAIN:
		ret = reg_read(client, REG_AGC_LOW, &low);
		if (ret)
			return ret;
		ret = reg_read(client, REG_AGC_HIGH, &high);
		if (ret)
			return ret;

		priv->gain->val = (high << 8) | low;
		return 0;
	case V4L2_CID_EXPOSURE_AUTO: {
		u32 exposure;

		ret = reg_read(client, REG_EXPOSURE_LOW, &val);
		if (ret)
			return ret;

		exposure = val;
		ret = reg_read(client, REG_EXPOSURE_HIGH, &val);
		if (ret)
			return ret;

		exposure |= val << 8;
		ret = reg_read(client, REG_EXPOSURE_HHIGH, &val);
		if (ret)
			return ret;

		exposure |= val << 16;
		priv->exp->val = exposure;
		priv->exp_abs->val = exposure_reg_to_100us(priv, exposure);
		return 0;
	}
	}
	return -EINVAL;
}

static int ov5647_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ov5647_priv *priv = container_of(ctrl->handler,
						struct ov5647_priv, hdl);
	struct v4l2_subdev *sd = &priv->subdev;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	u8 val;

	switch (ctrl->id) {
	case V4L2_CID_AUTOGAIN:
		ret = reg_read(client, REG_MANUAL_CONTROL, &val);
		if (ret)
			return ret;
		val = ctrl->val ? val & ~REG_MANUAL_CONTROL_AGC : val |
					REG_MANUAL_CONTROL_AGC;
		ret = reg_write(client, REG_MANUAL_CONTROL, val);
		if (ret)
			return ret;

		if (ctrl->val)
			return 0;

		return set_gain(client, priv->gain->val);

	case V4L2_CID_EXPOSURE_AUTO: {
		u32 exposure;

		ret = reg_read(client, REG_MANUAL_CONTROL, &val);
		if (ret)
			return ret;

		val = (priv->auto_exp->val == V4L2_EXPOSURE_AUTO) ? val &
			~REG_MANUAL_CONTROL_AEC : val | REG_MANUAL_CONTROL_AEC;

		ret = reg_write(client, REG_MANUAL_CONTROL, val);
		if (ret)
			return ret;
		if (ctrl->val == V4L2_EXPOSURE_AUTO)
			return 0;

		if (priv->exp_abs->is_new) {
			exposure = exposure_100us_to_reg(priv,
							 priv->exp_abs->val);
			*priv->exp->p_cur.p_s32 = exposure;
		} else {
			exposure = priv->exp->val;
			*priv->exp_abs->p_cur.p_s32 =
					exposure_reg_to_100us(priv, exposure);
		}

		return set_exposure(client, exposure);
	}
	case V4L2_CID_AUTO_WHITE_BALANCE: {
		u8 ctrl01, ctrl02;

		ret = reg_read(client, REG_ISP_CTRL01, &ctrl01);
		if (ret)
			return ret;
		if (ret)
			return ret;
		ret = reg_read(client, REG_ISP_CTRL02, &ctrl02);

		if (ctrl->val) {
			ctrl01 = ctrl01 | REG_ISP_CTRL01_AWB_EN;
			ctrl02 = ctrl02 | REG_ISP_CTRL02_AWB_GAIN_EN;
		} else {
			ctrl01 = ctrl01 & ~REG_ISP_CTRL01_AWB_EN;
			ctrl02 = ctrl02 & ~REG_ISP_CTRL02_AWB_GAIN_EN;
		}

		ret = reg_write(client, REG_ISP_CTRL01, ctrl01);
		if (ret)
			return ret;
		ret = reg_write(client, REG_ISP_CTRL02, ctrl02);

		return ret;
	}
	}
	return -EINVAL;
}

static struct v4l2_subdev_video_ops ov5647_subdev_video_ops = {
	.cropcap	= ov5647_cropcap,
	.s_crop		= ov5647_s_crop,
	.g_parm		= ov5647_g_parm,
	.g_mbus_config	= ov5647_g_mbus_config,
};

static const struct v4l2_subdev_pad_ops ov5647_subdev_pad_ops = {
	.enum_mbus_code = ov5647_enum_mbus_code,
	.get_fmt	= ov5647_try_fmt,
	.set_fmt	= ov5647_try_fmt,
};

static int ov5647_s_power(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	const struct ov5647_reg *next;

	for (next = mode_1920x1080; next->addr != OV5647_TABLE_END; next++) {
		if (next->addr == OV5647_TABLE_WAIT_MS) {
			msleep(next->val);
			continue;
		}
		ret = reg_write(client, next->addr, next->val);
		if (ret)
			return ret;
	}

	return 0;
}

static const struct v4l2_ctrl_ops ov5647_ctrl_ops = {
	.s_ctrl = ov5647_s_ctrl,
	.g_volatile_ctrl = ov5647_g_volatile_ctrl,
};

static struct v4l2_subdev_core_ops ov5647_subdev_core_ops = {
	.s_power	= ov5647_s_power,
};

static struct v4l2_subdev_ops ov5647_subdev_ops = {
	.core	= &ov5647_subdev_core_ops,
	.video	= &ov5647_subdev_video_ops,
	.pad	= &ov5647_subdev_pad_ops,
};

static int ov5647_probe(struct i2c_client *client,
			const struct i2c_device_id *did)
{
	struct ov5647_priv *priv;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->crop_rect.width = OV5647_DEFAULT_WIDTH;
	priv->crop_rect.height = OV5647_DEFAULT_HEIGHT;

	priv->crop_rect.left = 436;
	priv->crop_rect.top = 10;
	priv->total_width = 2416;
	priv->total_height = 1104;

	priv->fps = 30;
	priv->cfmt = &ov5647_cfmts[0];

	/* TODO: get from device tree */
	priv->xvclk = 25000000;

	v4l2_i2c_subdev_init(&priv->subdev, client, &ov5647_subdev_ops);
	v4l2_ctrl_handler_init(&priv->hdl, 0);
	priv->auto_gain = v4l2_ctrl_new_std(&priv->hdl, &ov5647_ctrl_ops,
		V4L2_CID_AUTOGAIN, 0, 1, 1, 1);
	priv->gain = v4l2_ctrl_new_std(&priv->hdl, &ov5647_ctrl_ops,
		V4L2_CID_GAIN, 1, 256, 1, 32);
	priv->auto_exp = v4l2_ctrl_new_std_menu(&priv->hdl, &ov5647_ctrl_ops,
		V4L2_CID_EXPOSURE_AUTO,
		V4L2_EXPOSURE_MANUAL, 0, V4L2_EXPOSURE_AUTO);
	priv->auto_exp->is_private = 1;
	priv->exp = v4l2_ctrl_new_std(&priv->hdl, &ov5647_ctrl_ops,
		V4L2_CID_EXPOSURE, 1, 17600, 1, 17280);
	priv->exp_abs = v4l2_ctrl_new_std(&priv->hdl, &ov5647_ctrl_ops,
		V4L2_CID_EXPOSURE_ABSOLUTE, 1, 332, 1, 326);
	priv->awb = v4l2_ctrl_new_std(&priv->hdl, &ov5647_ctrl_ops,
		V4L2_CID_AUTO_WHITE_BALANCE, 0, 1, 1, 0);
	priv->awb->is_private = 1;
	priv->subdev.ctrl_handler = &priv->hdl;

	if (priv->hdl.error)
		return priv->hdl.error;
	v4l2_ctrl_auto_cluster(2, &priv->auto_gain, 0, true);
	v4l2_ctrl_auto_cluster(3, &priv->auto_exp, V4L2_EXPOSURE_MANUAL, true);
	v4l2_ctrl_handler_setup(&priv->hdl);
	v4l2_async_register_subdev(&priv->subdev);

	return 0;
}

static int ov5647_remove(struct i2c_client *client)
{
	struct ov5647_priv *priv = to_ov5647(client);

	v4l2_async_unregister_subdev(&priv->subdev);
	v4l2_ctrl_handler_free(&priv->hdl);
	return 0;
}

static const struct i2c_device_id ov5647_id[] = {
	{ "ov5647", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ov5647_id);

static struct i2c_driver ov5647_i2c_driver = {
	.driver = {
		.name = "ov5647",
	},
	.probe    = ov5647_probe,
	.remove   = ov5647_remove,
	.id_table = ov5647_id,
};

module_i2c_driver(ov5647_i2c_driver);

MODULE_DESCRIPTION("SoC Camera driver for ov5647");
MODULE_LICENSE("GPL v2");
