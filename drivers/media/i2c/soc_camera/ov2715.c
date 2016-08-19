/*
 * Copyright 2015 ELVEES NeoTek CJSC
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

#define OV2715_MAX_WIDTH 1920
#define OV2715_MAX_HEIGHT 1080
#define OV2715_DEFAULT_WIDTH 1920
#define OV2715_DEFAULT_HEIGHT 1080

#define REG_SYS_CTRL0			0x3008
#define REG_MIPI_CTRL00			0x300e
#define REG_PLL1_CTRL0			0x300f
#define REG_PLL1_CTRL1			0x3010
#define REG_PLL1_CTRL2			0x3011
#define REG_PAD_OUTPUT_ENABLE1		0x3017
#define REG_PAD_OUTPUT_ENABLE2		0x3018
#define REG_PLL_CLOCK_SELECT		0x3103
#define REG_AEC_PK_HHIGH		0x3500
#define REG_AEC_PK_HIGH			0x3501
#define REG_AEC_PK_LOW			0x3502
#define REG_AEC_PK_MANUAL		0x3503
#define REG_AGC_ADJ_HIGH		0x350A
#define REG_AGC_ADJ_LOW			0x350B
#define REG_AEC_PK_VTS_HIGH		0x350C
#define REG_AEC_PK_VTS_LOW		0x350D
#define REG_WINDOW_START_X_HIGH		0x3800
#define REG_WINDOW_START_X_LOW		0x3801
#define REG_WINDOW_START_Y_HIGH		0x3802
#define REG_WINDOW_START_Y_LOW		0x3803
#define REG_WINDOW_WIDTH_HIGH		0x3804
#define REG_WINDOW_WIDTH_LOW		0x3805
#define REG_WINDOW_HEIGHT_HIGH		0x3806
#define REG_WINDOW_HEIGHT_LOW		0x3807
#define REG_OUT_WIDTH_HIGH		0x3808
#define REG_OUT_WIDTH_LOW		0x3809
#define REG_OUT_HEIGHT_HIGH		0x380a
#define REG_OUT_HEIGHT_LOW		0x380b
#define REG_OUT_TOTAL_WIDTH_HIGH	0x380c
#define REG_OUT_TOTAL_WIDTH_LOW		0x380d
#define REG_OUT_TOTAL_HEIGHT_HIGH	0x380e
#define REG_OUT_TOTAL_HEIGHT_LOW	0x380f
#define REG_AEC_CONTROL			0x3a00
#define REG_AEC_B50_STEP_HIGH		0x3a08
#define REG_AEC_B50_STEP_LOW		0x3a09
#define REG_AEC_B60_STEP_HIGH		0x3a0a
#define REG_AEC_B60_STEP_LOW		0x3a0b
#define REG_AEC_CONTROLD		0x3a0d
#define REG_AEC_CONTROLE		0x3a0e
#define REG_MIPI_CTRL0			0x4800
#define REG_MIPI_CTRL1			0x4801
#define REG_ISP_CONTROL1		0x5001
#define REG_AVG_WINDOW_END_X_HIGH	0x5682
#define REG_AVG_WINDOW_END_X_LOW	0x5683
#define REG_AVG_WINDOW_END_Y_HIGH	0x5686
#define REG_AVG_WINDOW_END_Y_LOW	0x5687

/* minimum extra blanking */
#define BLANKING_EXTRA_WIDTH		500
#define BLANKING_EXTRA_HEIGHT		20

/* AEC PK MANUAL */
#define REG_AEC_PK_MANUAL_AGC		0x02
#define REG_AEC_PK_MANUAL_AEC		0x01

/* AEC CONTROL */
#define REG_AEC_CONTROL_BAND_EN		0x20

/* ISP CONTROL1 */
#define REG_ISP_CONTROL1_AWB_EN		0x01


/*
 * the sensor's autoexposure is buggy when setting total_height low.
 * It tries to expose longer than 1 frame period without taking care of it
 * and this leads to weird output. So we set 1000 lines as minimum.
 */
#define BLANKING_MIN_HEIGHT		1000

#define OV2715_TABLE_END		0
#define OV2715_TABLE_WAIT_MS		1

struct ov2715_reg {
	u16 addr;
	u8 val;
};
/* Register setting for 1920*1080@30fps */
static struct ov2715_reg mode_1920x1080[] = {
	{0x3103, 0x93},
	{0x3008, 0x82},
	{OV2715_TABLE_WAIT_MS, 20},
	{0x3008, 0x42},
	{0x3017, 0x7f},
	{0x3018, 0xfc},
	{0x3706, 0x61},
	{0x3712, 0x0c},
	{0x3630, 0x6d},
	{0x3801, 0xb4},
	{0x3621, 0x04},
	{0x3604, 0x60},
	{0x3603, 0xa7},
	{0x3631, 0x26},
	{0x3600, 0x04},
	{0x3620, 0x37},
	{0x3623, 0x00},
	{0x3702, 0x9e},
	{0x3703, 0x5c},
	{0x3704, 0x40},
	{0x370d, 0x0f},
	{0x3713, 0x9f},
	{0x3714, 0x4c},
	{0x3710, 0x9e},
	{0x3801, 0xc4},
	{0x3605, 0x05},
	{0x3606, 0x3f},
	{0x302d, 0x90},
	{0x370b, 0x40},
	{0x380d, 0x74},
	{0x5181, 0x20},
	{0x518f, 0x00},
	{0x4301, 0xff},
	{0x4303, 0x00},
	{0x3a00, 0x78},
	{0x300f, 0x88},
	{0x3011, 0x28},
	{0x3a1a, 0x06},
	{0x3a18, 0x00},
	{0x3a19, 0x7a},
	{0x3a13, 0x54},
	{0x382e, 0x0f},
	{0x381a, 0x1a},
	{0x401d, 0x02},
	{0x5688, 0x03},
	{0x5684, 0x07},
	{0x5685, 0xa0},
	{0x5686, 0x04},
	{0x5687, 0x43},
	{0x3011, 0x0a},
	{0x300f, 0x8a},
	{0x3017, 0x00},
	{0x3018, 0x00},
	{0x300e, 0x04},
	{0x4800, 0x24},
	{0x4801, 0x0f},
	{0x300f, 0xc3},
	{OV2715_TABLE_WAIT_MS, 30},
	{0x3a0f, 0x40},
	{0x3a10, 0x38},
	{0x3a1b, 0x48},
	{0x3a1e, 0x30},
	{0x3a11, 0x90},
	{0x3a1f, 0x10},
	{0x3008, 0x02},
	{OV2715_TABLE_WAIT_MS, 20},
	{},
};

struct ov2715_color_format {
	u32 code;
	enum v4l2_colorspace colorspace;
};

struct ov2715_priv {
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
	const struct ov2715_color_format *cfmt;
	/* blanking information */
	int total_width;
	int total_height;
	int fps;
	int xvclk;
};

/*
 * supported color format list
 */
static const struct ov2715_color_format ov2715_cfmts[] = {
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

static struct ov2715_priv *to_ov2715(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client),
			struct ov2715_priv, subdev);
}

static int ov2715_enum_fmt(struct v4l2_subdev *sd, unsigned int index,
			   u32 *code)
{
	if (index >= ARRAY_SIZE(ov2715_cfmts))
		return -EINVAL;

	*code = ov2715_cfmts[index].code;
	return 0;
}

static int ov2715_cropcap(struct v4l2_subdev *sd, struct v4l2_cropcap *a)
{
	a->bounds.left			= 0;
	a->bounds.top			= 0;
	a->bounds.width			= OV2715_MAX_WIDTH;
	a->bounds.height		= OV2715_MAX_HEIGHT;
	a->defrect			= a->bounds;
	a->type				= V4L2_BUF_TYPE_VIDEO_CAPTURE;
	a->pixelaspect.numerator	= 1;
	a->pixelaspect.denominator	= 1;

	return 0;
}

static int ov2715_g_mbus_config(struct v4l2_subdev *sd,
				struct v4l2_mbus_config *cfg)
{
	cfg->type = V4L2_MBUS_CSI2;
	cfg->flags = V4L2_MBUS_CSI2_1_LANE | V4L2_MBUS_CSI2_CHANNEL_0 |
					V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
	return 0;
}

/* Find a data format by a pixel code in an array */
static const struct ov2715_color_format
			*ov2715_find_datafmt(u32 code)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ov2715_cfmts); i++)
		if (ov2715_cfmts[i].code == code)
			return &ov2715_cfmts[i];

	return NULL;
}

static int ov2715_try_fmt(struct v4l2_subdev *sd,
			  struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov2715_priv *priv = to_ov2715(client);
	const struct ov2715_color_format *fmt = ov2715_find_datafmt(mf->code);

	dev_dbg(&client->dev, "%s: %dx%d\n", __func__, mf->width, mf->height);
	mf->width = priv->crop_rect.width;
	mf->height = priv->crop_rect.height;

	if (!fmt) {
		mf->code	= ov2715_cfmts[0].code;
		mf->colorspace	= ov2715_cfmts[0].colorspace;
	}

	mf->field	= V4L2_FIELD_NONE;

	return 0;
}

static int ov2715_s_crop(struct v4l2_subdev *sd, const struct v4l2_crop *a)
{
	return -EINVAL;
}

static int ov2715_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov2715_priv *priv = to_ov2715(client);

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
static u32 exposure_reg_to_100us(struct ov2715_priv *priv, u32 value)
{
	return value * (USEC_PER_SEC / 100) /
			(priv->total_height * priv->fps) / 16;
}

static u32 exposure_100us_to_reg(struct ov2715_priv *priv, u32 value)
{
	return value * (priv->total_height * priv->fps) * 16 /
			(USEC_PER_SEC / 100);
}

static int ov2715_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ov2715_priv *priv = container_of(ctrl->handler,
						struct ov2715_priv, hdl);
	struct v4l2_subdev *sd = &priv->subdev;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	u8 val, high = 1, low;

	switch (ctrl->id) {
	case V4L2_CID_AUTOGAIN:
		ret = reg_read(client, REG_AGC_ADJ_LOW, &val);
		if (ret)
			return ret;
		low = val & 0x0f;
		high = hweight8(val & 0xf0) << 4;
		ret = reg_read(client, REG_AGC_ADJ_HIGH, &val);
		if (ret)
			return ret;

		if (val & 0x01)
			high++;
		priv->gain->val = high | low;
		return 0;
	case V4L2_CID_EXPOSURE_AUTO: {
		u32 exposure;

		ret = reg_read(client, REG_AEC_PK_LOW, &val);
		if (ret)
			return ret;

		exposure = val;
		ret = reg_read(client, REG_AEC_PK_HIGH, &val);
		if (ret)
			return ret;

		exposure |= val << 8;
		ret = reg_read(client, REG_AEC_PK_HHIGH, &val);
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

static int ov2715_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ov2715_priv *priv = container_of(ctrl->handler,
						struct ov2715_priv, hdl);
	struct v4l2_subdev *sd = &priv->subdev;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	u8 val;

	switch (ctrl->id) {
	case V4L2_CID_AUTOGAIN: {
		u8 low, high;

		ret = reg_read(client, REG_AEC_PK_MANUAL, &val);
		if (ret)
			return ret;
		val = ctrl->val ? val & ~REG_AEC_PK_MANUAL_AGC : val |
					REG_AEC_PK_MANUAL_AGC;
		ret = reg_write(client, REG_AEC_PK_MANUAL, val);
		if (ret)
			return ret;

		if (ctrl->val)
			return 0;

		ret = reg_write(client, REG_AGC_ADJ_HIGH, priv->gain->val > 79);
		if (ret)
			return ret;

		low  = priv->gain->val & 0x0f;
		high = priv->gain->val >> 4;
		if (high > 4)
			high--;
		high = (0xf << high) & 0xf0;
		ret = reg_write(client, REG_AGC_ADJ_LOW, high | low);

		return ret;
	}
	case V4L2_CID_EXPOSURE_AUTO: {
		u32 exposure;

		ret = reg_read(client, REG_AEC_PK_MANUAL, &val);
		if (ret)
			return ret;

		val = (priv->auto_exp->val == V4L2_EXPOSURE_AUTO) ? val &
			~REG_AEC_PK_MANUAL_AEC : val | REG_AEC_PK_MANUAL_AEC;

		ret = reg_write(client, REG_AEC_PK_MANUAL, val);
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

		val = (u8)(exposure & 0xff);
		ret = reg_write(client, REG_AEC_PK_LOW, val);
		if (ret)
			return ret;

		val = (u8)((exposure >> 8) & 0xff);
		ret = reg_write(client, REG_AEC_PK_HIGH, val);
		if (ret)
			return ret;

		val = (u8)((exposure >> 16) & 0xf);
		ret = reg_write(client, REG_AEC_PK_HHIGH, val);
		return ret;
	}
	case V4L2_CID_AUTO_WHITE_BALANCE:
		ret = reg_read(client, REG_ISP_CONTROL1, &val);
		if (ret)
			return ret;

		val = (ctrl->val) ? val | REG_ISP_CONTROL1_AWB_EN :
				    val & ~REG_ISP_CONTROL1_AWB_EN;

		ret = reg_write(client, REG_ISP_CONTROL1, val);
		return ret;
	}
	return -EINVAL;
}

static struct v4l2_subdev_video_ops ov2715_subdev_video_ops = {
	.g_mbus_fmt	= ov2715_try_fmt,
	.s_mbus_fmt	= ov2715_try_fmt,
	.try_mbus_fmt	= ov2715_try_fmt,
	.cropcap	= ov2715_cropcap,
	.s_crop		= ov2715_s_crop,
	.g_parm		= ov2715_g_parm,
	.enum_mbus_fmt	= ov2715_enum_fmt,
	.g_mbus_config	= ov2715_g_mbus_config,
};

static int ov2715_s_power(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	u8 reg_sys_ctrl0;
	const struct ov2715_reg *next;

	dev_dbg(&client->dev, "%s: on=%d\n", __func__, on);
	ret = reg_read(client, REG_SYS_CTRL0, &reg_sys_ctrl0);
	if (ret)
		return ret;
	if (!on)
		/* sleep mode */
		return reg_write(client, REG_SYS_CTRL0, reg_sys_ctrl0 | 0x40);

	for (next = mode_1920x1080; next->addr != OV2715_TABLE_END; next++) {
		if (next->addr == OV2715_TABLE_WAIT_MS) {
			msleep(next->val);
			continue;
		}
		ret = reg_write(client, next->addr, next->val);
		if (ret)
			return ret;
	}

	return ret;
}

static const struct v4l2_ctrl_ops ov2715_ctrl_ops = {
	.s_ctrl = ov2715_s_ctrl,
	.g_volatile_ctrl = ov2715_g_volatile_ctrl,
};

static struct v4l2_subdev_core_ops ov2715_subdev_core_ops = {
	.s_power	= ov2715_s_power,
};

static struct v4l2_subdev_ops ov2715_subdev_ops = {
	.core	= &ov2715_subdev_core_ops,
	.video	= &ov2715_subdev_video_ops,
};

static int ov2715_probe(struct i2c_client *client,
			const struct i2c_device_id *did)
{
	struct ov2715_priv *priv;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->crop_rect.width = OV2715_DEFAULT_WIDTH;
	priv->crop_rect.height = OV2715_DEFAULT_HEIGHT;

	priv->crop_rect.left = 436;
	priv->crop_rect.top = 10;
	priv->total_width = 2420;
	priv->total_height = 1104;

	priv->fps = 30;
	priv->cfmt = &ov2715_cfmts[0];

	/* TODO: get from device tree */
	priv->xvclk = 24000000;

	v4l2_i2c_subdev_init(&priv->subdev, client, &ov2715_subdev_ops);
	v4l2_ctrl_handler_init(&priv->hdl, 0);
	priv->auto_gain = v4l2_ctrl_new_std(&priv->hdl, &ov2715_ctrl_ops,
		V4L2_CID_AUTOGAIN, 0, 1, 1, 1);
	priv->gain = v4l2_ctrl_new_std(&priv->hdl, &ov2715_ctrl_ops,
		V4L2_CID_GAIN, 0, 95, 1, 32);
	priv->auto_exp = v4l2_ctrl_new_std_menu(&priv->hdl, &ov2715_ctrl_ops,
		V4L2_CID_EXPOSURE_AUTO,
		V4L2_EXPOSURE_MANUAL, 0, V4L2_EXPOSURE_AUTO);
	priv->exp = v4l2_ctrl_new_std(&priv->hdl, &ov2715_ctrl_ops,
		V4L2_CID_EXPOSURE, 1, 17600, 1, 17280);
	priv->exp_abs = v4l2_ctrl_new_std(&priv->hdl, &ov2715_ctrl_ops,
		V4L2_CID_EXPOSURE_ABSOLUTE, 1, 332, 1, 326);
	priv->awb = v4l2_ctrl_new_std(&priv->hdl, &ov2715_ctrl_ops,
		V4L2_CID_AUTO_WHITE_BALANCE, 0, 1, 1, 1);
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

static int ov2715_remove(struct i2c_client *client)
{
	struct ov2715_priv *priv = to_ov2715(client);

	v4l2_async_unregister_subdev(&priv->subdev);
	v4l2_ctrl_handler_free(&priv->hdl);
	return 0;
}

static const struct i2c_device_id ov2715_id[] = {
	{ "ov2715", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ov2715_id);

static struct i2c_driver ov2715_i2c_driver = {
	.driver = {
		.name = "ov2715",
	},
	.probe    = ov2715_probe,
	.remove   = ov2715_remove,
	.id_table = ov2715_id,
};

module_i2c_driver(ov2715_i2c_driver);

MODULE_DESCRIPTION("SoC Camera driver for ov2715");
MODULE_AUTHOR("Vasiliy Zasukhin <vzasukhin@elvees.com>");
MODULE_LICENSE("GPL v2");
