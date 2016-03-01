/*
 * Elvees VPOUT framebuffer driver
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
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/platform_data/vpoutfb.h>
#include <linux/clk-provider.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include "vpoutfb.h"
#include "it66121.h"

#define LCDCSR 0x0
#define LCDDIV 0x4
#define LCDMOD 0x8
#define LCDHT0 0xc
#define LCDHT1 0x10
#define LCDVT0 0x14
#define LCDVT1 0x18
#define LCDAB0 0x2c
#define LCDAB1 0x30
#define LCDOF0 0x34
#define LCDOF1 0x38

#define MODE_HDMI 0x300
#define CSR_EN		BIT(0)
#define CSR_RUN		BIT(1)
#define CSR_INIT	BIT(2)

static struct fb_fix_screeninfo vpoutfb_fix = {
	.id		= "vpout",
	.type		= FB_TYPE_PACKED_PIXELS,
	.visual		= FB_VISUAL_TRUECOLOR,
	.accel		= FB_ACCEL_NONE,
};

static struct fb_var_screeninfo vpoutfb_var = {
	.height		= -1,
	.width		= -1,
	.activate	= FB_ACTIVATE_NOW,
	.vmode		= FB_VMODE_NONINTERLACED,
};

/*
 * Sets a fictional color palette for fbcon.
 * Needed for compatibility reasons.
 * Ignores transparency outright due to lack of support.
 */
static int vpoutfb_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
			      u_int transp, struct fb_info *info)
{
	u32 *pal = info->pseudo_palette;
	u32 cr = red >> (16 - info->var.red.length);
	u32 cg = green >> (16 - info->var.green.length);
	u32 cb = blue >> (16 - info->var.blue.length);
	u32 value;

	if (regno >= PSEUDO_PALETTE_SIZE)
		return -EINVAL;

	value = (cr << info->var.red.offset) |
		(cg << info->var.green.offset) |
		(cb << info->var.blue.offset);
	pal[regno] = value;

	return 0;
}

static void vpoutfb_destroy(struct fb_info *info)
{
	struct vpoutfb_par *par;

	par = info->par;
	if (info->screen_base)
		dma_free_coherent(info->dev, par->mem_size,
				  par->mem_virt, par->mem_phys);
	/* Reset HDMI device */
	it66121_reset(&par->hdmidata);
}

static struct fb_ops vpoutfb_ops = {
	.owner		= THIS_MODULE,
	.fb_destroy	= vpoutfb_destroy,
	.fb_setcolreg	= vpoutfb_setcolreg,
	.fb_fillrect	= sys_fillrect,
	.fb_copyarea	= sys_copyarea,
	.fb_imageblit	= sys_imageblit,
};

static struct vpoutfb_format vpoutfb_formats[] = VPOUTFB_FORMATS;


static int vpoutfb_parse_dt(struct platform_device *pdev,
			   struct vpoutfb_platform_data *pdata)
{
	struct device_node *np = pdev->dev.of_node;
	int ret;
	const char *format;
	int i;

	ret = of_property_read_u32(np, "width", &pdata->width);
	if (ret) {
		dev_err(&pdev->dev, "Can't parse width property\n");
		return ret;
	}

	ret = of_property_read_u32(np, "height", &pdata->height);
	if (ret) {
		dev_err(&pdev->dev, "Can't parse height property\n");
		return ret;
	}

	ret = of_property_read_string(np, "format", &format);
	if (ret) {
		dev_err(&pdev->dev, "Can't parse format property\n");
		return ret;
	}
	pdata->format = NULL;
	for (i = 0; i < ARRAY_SIZE(vpoutfb_formats); i++) {
		if (strcmp(format, vpoutfb_formats[i].name))
			continue;
		pdata->format = &vpoutfb_formats[i];
		break;
	}
	if (!pdata->format) {
		dev_err(&pdev->dev, "Invalid format value\n");
		return -EINVAL;
	}
	pdata->output_node = of_parse_phandle(np, "output", 0);
	if (!pdata->output_node) {
		dev_err(&pdev->dev, "Can't parse output property\n");
		return -EINVAL;
	}
	ret = of_property_read_string(pdata->output_node,
				      "compatible",
				      &pdata->output_name);
	if (ret) {
		dev_err(&pdev->dev, "Can't parse output's compatibility\n");
		return ret;
	}
	return 0;
}

#if defined CONFIG_OF && defined CONFIG_COMMON_CLK
/*
 * Clock handling code.
 *
 * Pretty impenetrable, but it just fishes clocks out of dt and
 * tries to enable them best it can.
 */
static int vpoutfb_clocks_init(struct vpoutfb_par *par,
				struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct clk *clock;
	int i, ret;

	if (dev_get_platdata(&pdev->dev) || !np)
		return 0;

	par->clk_count = of_clk_get_parent_count(np);
	if (par->clk_count <= 0)
		return 0;

	par->clks = kcalloc(par->clk_count, sizeof(struct clk *), GFP_KERNEL);
	if (!par->clks)
		return -ENOMEM;

	for (i = 0; i < par->clk_count; i++) {
		clock = of_clk_get(np, i);
		if (IS_ERR(clock)) {
			if (PTR_ERR(clock) == -EPROBE_DEFER) {
				while (--i >= 0) {
					if (par->clks[i])
						clk_put(par->clks[i]);
				}
				kfree(par->clks);
				return -EPROBE_DEFER;
			}
			dev_err(&pdev->dev, "%s: clock %d not found: %ld\n",
				__func__, i, PTR_ERR(clock));
			continue;
		}
		par->clks[i] = clock;
	}

	for (i = 0; i < par->clk_count; i++) {
		if (par->clks[i]) {
			ret = clk_prepare_enable(par->clks[i]);
			if (ret) {
				dev_err(&pdev->dev,
					"%s: failed to enable clock %d: %d\n",
					__func__, i, ret);
				clk_put(par->clks[i]);
				par->clks[i] = NULL;
			}
		}
	}

	return 0;
}

static void vpoutfb_clocks_destroy(struct vpoutfb_par *par)
{
	int i;

	if (!par->clks)
		return;

	for (i = 0; i < par->clk_count; i++) {
		if (par->clks[i]) {
			clk_disable_unprepare(par->clks[i]);
			clk_put(par->clks[i]);
		}
	}

	kfree(par->clks);
}
#else
static int vpoutfb_clocks_init(struct vpoutfb_par *par,
	struct platform_device *pdev) { return 0; }
static void vpoutfb_clocks_destroy(struct vpoutfb_par *par) { }
#endif

static int vpoutfb_probe(struct platform_device *pdev)
{
	int ret;
	u16 hsw, hgate, hgdel, hlen, vsw, vgate, vgdel, vlen;
	u32 div;
	struct vpoutfb_platform_data pdata;
	struct fb_info *info;
	struct vpoutfb_par *par;
	struct resource *devres;

	if (fb_get_options("vpoutfb", NULL))
		return -ENODEV;

	ret = -ENODEV;
	if (pdev->dev.of_node)
		ret = vpoutfb_parse_dt(pdev, &pdata);

	if (ret)
		return ret;

	devres = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!devres) {
		dev_err(&pdev->dev, "No memory resource\n");
		return -EINVAL;
	}

	info = framebuffer_alloc(sizeof(struct vpoutfb_par), &pdev->dev);
	if (!info)
		return -ENOMEM;
	platform_set_drvdata(pdev, info);

	par = info->par;
	par->mem_size = pdata.width * pdata.height *
		pdata.format->bits_per_pixel / 8;
	ret = vpoutfb_clocks_init(par, pdev);
	if (ret < 0)
		goto error_fb_release;
	par->mmio_base = devm_ioremap_resource(&pdev->dev, devres);
	if (IS_ERR(par->mmio_base)) {
		dev_err(&pdev->dev, "Cannot remap mem resource\n");
		goto error_clocks;
	}
	par->mem_virt = dma_alloc_coherent(&pdev->dev,
					   par->mem_size,
					   &par->mem_phys,
					   GFP_KERNEL);
	if (!par->mem_virt) {
		ret = -ENOMEM;
		goto error_clocks;
	}

	info->fix = vpoutfb_fix;
	info->fix.smem_start = par->mem_phys;
	info->fix.smem_len = par->mem_size;
	info->fix.line_length = pdata.width *
		pdata.format->bits_per_pixel / 8;

	info->var = vpoutfb_var;
	info->var.xres = pdata.width;
	info->var.yres = pdata.height;
	info->var.xres_virtual = pdata.width;
	info->var.yres_virtual = pdata.height;
	info->var.bits_per_pixel = pdata.format->bits_per_pixel;
	info->var.red = pdata.format->red;
	info->var.green = pdata.format->green;
	info->var.blue = pdata.format->blue;
	info->var.transp = pdata.format->transp;

	info->apertures = alloc_apertures(1);
	if (!info->apertures) {
		ret = -ENOMEM;
		goto error_unmap;
	}
	info->apertures->ranges[0].base = info->fix.smem_start;
	info->apertures->ranges[0].size = info->fix.smem_len;

	info->fbops = &vpoutfb_ops;
	info->flags = FBINFO_DEFAULT | FBINFO_MISC_FIRMWARE;
	info->screen_base = par->mem_virt;

	info->pseudo_palette = par->palette;

	if (pdata.width == 1280 && pdata.height == 720) {
		hsw = 39, hgdel = 219, hgate = 1279, hlen = 1649,
		vsw = 4, vgdel = 19, vgate = 719, vlen = 749,
		div = 5; /* 1280x720 CEA - 4 */
	} else if (pdata.width == 1920 && pdata.height == 1080) {
		hsw = 43, hgdel = 147, hgate = 1919, hlen = 2199,
		vsw = 4, vgdel = 35, vgate = 1079, vlen = 1124,
		div = 2; /* 1920x1080 CEA - 16 */
	} else {
		dev_err(&pdev->dev, "Unsupported resolution\n");
		goto error_unmap;
	}
	iowrite32(CSR_EN, par->mmio_base + LCDHT0);
	iowrite32(hgdel << 16 | hsw, par->mmio_base + LCDHT0);
	iowrite32(hlen << 16  | hgate, par->mmio_base + LCDHT1);
	iowrite32(vgdel << 16 | vsw, par->mmio_base + LCDVT0);
	iowrite32(vlen << 16  | vgate, par->mmio_base + LCDVT1);
	iowrite32(div, par->mmio_base + LCDDIV);
	iowrite32(par->mem_phys, par->mmio_base + LCDAB0);
	iowrite32(par->mem_phys, par->mmio_base + LCDAB1);
	iowrite32(MODE_HDMI + pdata.format->hw_modenum,
		  par->mmio_base + LCDMOD);
	iowrite32(CSR_INIT | CSR_EN, par->mmio_base + LCDCSR);
	while (ioread32(par->mmio_base + LCDCSR) & CSR_INIT)
		usleep_range(1000, 2000);
	iowrite32(CSR_RUN | CSR_EN, par->mmio_base + LCDCSR);
	dev_info(&pdev->dev, "framebuffer at 0x%lx, 0x%x bytes, mapped to 0x%p\n",
			     info->fix.smem_start, info->fix.smem_len,
			     info->screen_base);
	dev_info(&pdev->dev, "format=%s, mode=%dx%dx%d, linelength=%d\n",
			     pdata.format->name,
			     info->var.xres, info->var.yres,
			     info->var.bits_per_pixel, info->fix.line_length);

	ret = register_framebuffer(info);
	if (ret < 0) {
		dev_err(&pdev->dev, "Unable to register vpoutfb: %d\n", ret);
		goto error_unmap;
	}

	if (!strcmp(pdata.output_name, "ite,it66121")) {
		ret = it66121_init(&par->hdmidata, info, pdata.output_node);
		if (ret < 0) {
			dev_err(&pdev->dev,
				"Unable to init HDMI adapter: %d\n",
				ret);
			goto error_unmap;
		}
	}

	dev_info(&pdev->dev, "fb%d: vpoutfb registered!\n", info->node);

	return 0;

error_unmap:
	dma_free_coherent(&pdev->dev, par->mem_size,
			  par->mem_virt, par->mem_phys);
error_clocks:
	vpoutfb_clocks_destroy(par);
error_fb_release:
	framebuffer_release(info);
	return ret;
}

static int vpoutfb_remove(struct platform_device *pdev)
{
	struct fb_info *info = platform_get_drvdata(pdev);
	struct vpoutfb_par *par = info->par;

	unregister_framebuffer(info);
	vpoutfb_clocks_destroy(par);
	framebuffer_release(info);

	return 0;
}

static const struct of_device_id vpoutfb_of_match[] = {
	{ .compatible = "elvees,vpoutfb", },
	{ },
};
MODULE_DEVICE_TABLE(of, vpoutfb_of_match);

static struct platform_driver vpoutfb_driver = {
	.driver = {
		.name = "vpoutfb",
		.of_match_table = vpoutfb_of_match,
	},
	.probe = vpoutfb_probe,
	.remove = vpoutfb_remove,
};

module_platform_driver(vpoutfb_driver);

MODULE_AUTHOR("Oleg Kitain <okitain@elvees.com>");
MODULE_DESCRIPTION("Elvees VPOUT framebuffer driver");
MODULE_LICENSE("GPL v2");
