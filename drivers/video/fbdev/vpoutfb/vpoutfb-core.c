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

#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/fb.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/platform_data/vpoutfb.h>
#include <linux/spinlock.h>
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
#define LCDINT 0x44
#define LCDMSK 0x48

#define MODE_HDMI 0x300
#define CSR_EN		BIT(0)
#define CSR_RUN		BIT(1)
#define CSR_INIT	BIT(2)
#define CSR_CLR		BIT(3)

#define INT_DMADONE	BIT(0)
#define INT_DMAEMPTY	BIT(1)
#define INT_OUTFIFO	BIT(2)
#define INT_OUTEMPTY	BIT(3)
#define INT_VSYNC	BIT(5)

#define UNDIVPIXCLK 2315
#define MAX_BUFSIZE (1920 * 1080 * 4)
#define CLEAR_MSEC 40

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

static struct fb_videomode vpoutfb_guaranteed_modedb[] = {
	{"720p@59.94", 0, 1280, 720, 13890, 220, 110, 20, 5, 40, 5, 0,
		FB_VMODE_NONINTERLACED, 0}, /* Real: 58.1 FPS */
	{"1080p@59.94", 0, 1920, 1080, 6945, 148, 88, 36, 4, 44, 5, 0,
		FB_VMODE_NONINTERLACED, 0}, /* Real: 58.1 FPS */
	{"640x480@60", 0, 640, 480, 39355, 48, 16, 33, 10, 96, 2, 0,
		FB_VMODE_NONINTERLACED, 0}, /* Real: 60.5 FPS */
	{"480p@59.94", 0, 720, 480, 37040, 60, 16, 30, 9, 62, 6, 0,
		FB_VMODE_NONINTERLACED, 0}, /* Real: 59.94 FPS */
	{"800x600@60", 0, 800, 600, 27780, 128, 24, 22, 01, 72, 2, 0,
		FB_VMODE_NONINTERLACED, 0}, /* Real: 56.2 FPS */
	{"1024x768@60", 0, 1024, 768, 16205, 168, 8, 29, 3, 144, 6, 0,
		FB_VMODE_NONINTERLACED, 0}, /* Real: 57 FPS */
	{"1366x768@60", 0, 1366, 768, 13890, 120, 10, 14, 3, 32, 5, 0,
		FB_VMODE_NONINTERLACED, 0}, /* Real: 59.6 FPS */
};

static void vpoutfb_hwreset(unsigned long data)
{
	struct fb_info *info = (struct fb_info *) data;
	int i, j;
	struct vpoutfb_par *par;

	dev_err(info->dev, "Invalid output, choose smaller resolution\n");
	par = info->par;
	spin_lock(&par->reglock);
	for (j = 0; j < 2; j++) {
		iowrite32(CSR_EN, par->mmio_base + LCDCSR);
		iowrite32(CSR_INIT | CSR_EN, par->mmio_base + LCDCSR);
		for (i = 0; i < 250; i++) {
			if (!(ioread32(par->mmio_base + LCDCSR) & CSR_INIT))
				break;
		}
		if (i == 250)
			dev_err(info->dev, "Initialization failed\n");
	}
	iowrite32(CSR_RUN | CSR_EN, par->mmio_base + LCDCSR);
	spin_unlock(&par->reglock);
}

static int vpoutfb_check_var(struct fb_var_screeninfo *var,
			     struct fb_info *info)
{
	struct fb_var_screeninfo *oldvar;
	struct fb_videomode *modecaret;
	oldvar = &info->var;

	/* SDL Applications zero out the var.<color> fields.
	 * In order for them not to crash, we ignore changes to
	 * those fields. However, we don't ignore changes to grayscale,
	 * bits_per_pixel or transp since those mean apps want the pixel
	 * scheme changed, and we can't have that.
	 */
	if (var->bits_per_pixel != oldvar->bits_per_pixel ||
	    oldvar->grayscale != var->grayscale ||
	    memcmp(&oldvar->transp, &var->transp,
		   sizeof(struct fb_bitfield))) {
		return -EINVAL;
	}
	var->red = oldvar->red;
	var->green = oldvar->green;
	var->blue = oldvar->blue;

	/* While EDID reading is not implemented, just check via modedb */
	/* When it is implemented, should check for buffer size violation */
	for (modecaret = vpoutfb_guaranteed_modedb;
	     modecaret->name != NULL; modecaret++) {
		if (modecaret->xres == var->xres &&
		    modecaret->yres == var->yres)
			break;
	}
	/* Don't switch if not in modedb */
	if (modecaret->name == NULL)
		return -EINVAL;
	var->xres = modecaret->xres;
	var->xres_virtual = var->xres;
	var->yres = modecaret->yres;
	var->yres_virtual = var->yres;
	var->pixclock = modecaret->pixclock;
	var->left_margin = modecaret->left_margin;
	var->right_margin = modecaret->right_margin;
	var->upper_margin = modecaret->upper_margin;
	var->lower_margin = modecaret->lower_margin;
	var->hsync_len = modecaret->hsync_len;
	var->vsync_len = modecaret->vsync_len;
	return 0;
}

static int vpoutfb_set_par(struct fb_info *info)
{
	int hsw, hgdel, hgate, hlen, vsw, vgdel, vgate, vlen, div, i;
	struct fb_var_screeninfo *var;
	struct vpoutfb_par *par;

	var = &info->var;
	par = info->par;
	hsw = var->hsync_len - 1;
	vsw = var->vsync_len - 1;
	hgate = var->xres - 1;
	vgate = var->yres - 1;
	hgdel = var->left_margin - 1;
	vgdel = var->upper_margin - 1;
	hlen = var->xres + var->left_margin +
		var->right_margin + var->hsync_len - 1;
	vlen = var->yres + var->upper_margin +
		var->lower_margin + var->vsync_len - 1;
	div = var->pixclock / UNDIVPIXCLK - 1;
	info->fix.line_length = var->xres *
		par->color_fmt->bits_per_pixel / 8;
	/* If the device is currently on, clear FIFO and power it off */
	spin_lock(&par->reglock);
	if (ioread32(par->mmio_base + LCDCSR) & CSR_EN) {
		iowrite32(CSR_EN, par->mmio_base + LCDCSR);
		iowrite32(CSR_EN|CSR_CLR, par->mmio_base + LCDCSR);
		for (i = 0; i < CLEAR_MSEC; i++) {
			if (!(ioread32(par->mmio_base + LCDCSR) & CSR_CLR))
				break;
			usleep_range(1000, 2000);
		}
		if (i == CLEAR_MSEC) {
			dev_err(info->dev, "FIFO clear looped\n");
			spin_unlock(&par->reglock);
			return -EBUSY;
		}
		iowrite32(0, par->mmio_base + LCDCSR);
	}
	/* Turn on and reset the device */
	iowrite32(CSR_EN, par->mmio_base + LCDCSR);
	iowrite32(CSR_EN|CSR_CLR, par->mmio_base + LCDCSR);
	for (i = 0; i < CLEAR_MSEC; i++) {
		if (!(ioread32(par->mmio_base + LCDCSR) & CSR_CLR))
			break;
		usleep_range(1000, 2000);
	}
	if (i == CLEAR_MSEC) {
		dev_err(info->dev, "FIFO clear looped\n");
		spin_unlock(&par->reglock);
		return -EBUSY;
	}
	/* Configure video mode */
	iowrite32(hgdel << 16 | hsw, par->mmio_base + LCDHT0);
	iowrite32(hlen << 16  | hgate, par->mmio_base + LCDHT1);
	iowrite32(vgdel << 16 | vsw, par->mmio_base + LCDVT0);
	iowrite32(vlen << 16  | vgate, par->mmio_base + LCDVT1);
	iowrite32(div, par->mmio_base + LCDDIV);
	iowrite32(par->mem_phys, par->mmio_base + LCDAB0);
	iowrite32(par->mem_phys, par->mmio_base + LCDAB1);
	iowrite32(MODE_HDMI + par->color_fmt->hw_modenum,
		  par->mmio_base + LCDMOD);
	/* Finally, initialize and run the device */
	iowrite32(INT_OUTFIFO,
		  par->mmio_base + LCDMSK);
	iowrite32(CSR_INIT | CSR_EN, par->mmio_base + LCDCSR);
	for (i = 0; i < CLEAR_MSEC; i++) {
		if (!(ioread32(par->mmio_base + LCDCSR) & CSR_INIT))
			break;
		usleep_range(1000, 2000);
	}
	if (i == CLEAR_MSEC) {
		dev_err(info->dev, "Initialization failed\n");
		spin_unlock(&par->reglock);
		return -EBUSY;
	}
	iowrite32(CSR_RUN | CSR_EN, par->mmio_base + LCDCSR);
	spin_unlock(&par->reglock);
	if (par->hdmidata.client != NULL)
		return it66121_init(&par->hdmidata, info);
	return 0;
}

static irqreturn_t vpoutfb_irq_handler(int irq, void *dev_id)
{
	struct fb_info *info;
	struct vpoutfb_par *par;
	u32 irqstatus;

	info = dev_id;
	par = info->par;
	irqstatus = ioread32(par->mmio_base + LCDINT);

	if (irqstatus & INT_OUTFIFO)
		tasklet_schedule(&par->reset_tasklet);
	iowrite32(irqstatus, par->mmio_base + LCDINT);

	return IRQ_HANDLED;
}

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
	if (par->hdmidata.client != NULL) {
		it66121_reset(&par->hdmidata);
		it66121_remove(&par->hdmidata);
	}
}

static struct fb_ops vpoutfb_ops = {
	.owner		= THIS_MODULE,
	.fb_destroy	= vpoutfb_destroy,
	.fb_setcolreg	= vpoutfb_setcolreg,
	.fb_fillrect	= sys_fillrect,
	.fb_copyarea	= sys_copyarea,
	.fb_imageblit	= sys_imageblit,
	.fb_check_var	= vpoutfb_check_var,
	.fb_set_par	= vpoutfb_set_par
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

	par->clks = devm_kcalloc(&pdev->dev, par->clk_count,
				 sizeof(struct clk *), GFP_KERNEL);
	if (!par->clks)
		return -ENOMEM;

	for (i = 0; i < par->clk_count; i++) {
		clock = of_clk_get(np, i);
		if (IS_ERR(clock)) {
			while (--i >= 0) {
				if (par->clks[i])
					clk_put(par->clks[i]);
			    }
			dev_err(&pdev->dev, "%s: clock %d error: %ld\n",
				__func__, i, PTR_ERR(clock));
			return PTR_ERR(clock);
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
				return ret;
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

static int vpoutfb_create(struct fb_info *info, struct platform_device *pdev)
{
	int ret, irq;
	struct vpoutfb_par *par;

	par = info->par;
	par->mem_virt = dma_alloc_coherent(&pdev->dev,
					   par->mem_size,
					   &par->mem_phys,
					   GFP_KERNEL);
	if (!par->mem_virt) {
		ret = -ENOMEM;
		dev_err(&pdev->dev, "DMA alloc failed");
		return ret;
	}
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "Cannot parse IRQ from platform\n");
		return irq;
	}
	ret = devm_request_irq(&pdev->dev, irq, vpoutfb_irq_handler,
			       0, "vpoutfb", info);
	if (ret) {
		dev_err(&pdev->dev, "Cannot request IRQ handler\n");
		return ret;
	}
	return 0;
}

static int vpoutfb_probe(struct platform_device *pdev)
{
	int ret;
	struct vpoutfb_platform_data pdata;
	struct fb_info *info = NULL;
	struct resource *devres = NULL;
	struct vpoutfb_par *par;

	if (fb_get_options("vpoutfb", NULL))
		return -ENODEV;

	ret = -ENODEV;
	if (pdev->dev.of_node)
		ret = vpoutfb_parse_dt(pdev, &pdata);

	if (ret)
		return ret;

	info = framebuffer_alloc(sizeof(struct vpoutfb_par), &pdev->dev);
	if (!info) {
		dev_err(&pdev->dev, "FB alloc failed");
		return -ENOMEM;
	}
	platform_set_drvdata(pdev, info);
	par = info->par;

	par->mem_size = MAX_BUFSIZE;
	/* We don't allocate for modes > 1080p since bigger resolutions
	 * put more of a stress on RAM and stall out other tasks.
	 */

	tasklet_init(&par->reset_tasklet, vpoutfb_hwreset,
		     (unsigned long) info);
	ret = vpoutfb_clocks_init(par, pdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "Clock alloc failed");
		goto error_cleanup;
	}

	devres = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	if (!devres) {
		ret = -EINVAL;
		dev_err(&pdev->dev, "No memory resource\n");
		goto error_cleanup;
	}

	par->mmio_base = devm_ioremap_resource(&pdev->dev, devres);
	if (IS_ERR(par->mmio_base)) {
		ret = PTR_ERR(par->mmio_base);
		dev_err(&pdev->dev, "Cannot remap mem resource\n");
		goto error_cleanup;
	}

	ret = vpoutfb_create(info, pdev);
	if (ret)
		goto error_cleanup;

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
	par->color_fmt = pdata.format;

	spin_lock_init(&par->reglock);

	info->apertures = alloc_apertures(1);
	if (!info->apertures) {
		ret = -ENOMEM;
		goto error_cleanup;
	}
	info->apertures->ranges[0].base = info->fix.smem_start;
	info->apertures->ranges[0].size = info->fix.smem_len;

	info->fbops = &vpoutfb_ops;
	info->flags = FBINFO_DEFAULT | FBINFO_MISC_FIRMWARE;
	info->screen_base = par->mem_virt;
	vpoutfb_check_var(&info->var, info);

	info->pseudo_palette = par->palette;

	if (!strcmp(pdata.output_name, "ite,it66121"))
		it66121_probe(&par->hdmidata, pdata.output_node);

	ret = vpoutfb_set_par(info);
	if (ret < 0) {
		dev_err(&pdev->dev, "Unable to set resolution: %d\n", ret);
		ret = -EINVAL;
		goto error_cleanup;
	}

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
		goto error_cleanup;
	}

	dev_info(&pdev->dev, "fb%d: vpoutfb registered!\n", info->node);

	return 0;

error_cleanup:
	if (par->mem_virt)
		dma_free_coherent(&pdev->dev, par->mem_size,
				  par->mem_virt, par->mem_phys);
	vpoutfb_clocks_destroy(par);
	if (info)
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
