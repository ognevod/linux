/*
 * vpoutfb.h - Elvees VPOUT Framebuffer Device
 *
 * Copyright 2016 Elvees NeoTek JSC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef PLATFORM_DATA_VPOUTFB_H
#define PLATFORM_DATA_VPOUTFB_H

#include <drm/drm_fourcc.h>
#include <linux/fb.h>
#include <linux/kernel.h>
#include <linux/of.h>

/* Will not implement indexed color for now */
#define VPOUTFB_FORMATS \
{ \
	{ "r3g3b2", 8, {5, 3}, {2, 3}, {0, 2}, {0, 0}, 0 }, \
	{ "x4r4g4b4", 16, {8, 4}, {4, 4}, {0, 4}, {0, 0}, 1 }, \
	{ "x1r5g5b5", 16, {10, 5}, {5, 5}, {0, 5}, {0, 0}, 2 }, \
	{ "r5g6b5", 16, {11, 5}, {5, 6}, {0, 5}, {0, 0}, 3 }, \
	{ "r8g8b8", 24, {16, 8}, {8, 8}, {0, 8}, {0, 0}, 5 }, \
	{ "x8r8g8b8", 32, {16, 8}, {8, 8}, {0, 8}, {0, 0}, 6 }, \
}

/*
 * Data-Format for VPOUT Framebuffer
 * @name: unique 0-terminated name that can be used to identify the mode
 * @red,green,blue: Offsets and sizes of the single RGB parts
 * @transp: Offset and size of the alpha bits. length=0 means no alpha
 * @fourcc: 32bit DRM four-CC code (see drm_fourcc.h)
 * @hw_modenum: value in the hardware register bitfield
 */
struct vpoutfb_format {
	const char *name;
	u32 bits_per_pixel;
	struct fb_bitfield red;
	struct fb_bitfield green;
	struct fb_bitfield blue;
	struct fb_bitfield transp;
	u8 hw_modenum;
};

struct vpoutfb_platform_data {
	u32 width;
	u32 height;
	struct vpoutfb_format *format;
	struct device_node *output_node;
	const char *output_name;
};

#endif /* PLATFORM_DATA_VPOUTFB_H */
