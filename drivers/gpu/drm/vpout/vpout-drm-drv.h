/*
 * ELVEES VPOUT Controller DRM Driver
 *
 * Copyright 2017 RnD Center "ELVEES", JSC
 *
 * Based on tilcdc:
 * Copyright (C) 2012 Texas Instruments
 * Author: Rob Clark <robdclark@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __VPOUT_DRM_DRV_H__
#define __VPOUT_DRM_DRV_H__

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/list.h>

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>

struct vpout_drm_private {
	void __iomem *mmio;
	struct clk *clk;
	struct workqueue_struct *wq;
	struct drm_fbdev_cma *fbdev;
	struct drm_crtc *crtc;
	bool is_componentized;
	unsigned int num_encoders;
	struct drm_encoder *encoders[8];
	unsigned int num_connectors;
	struct drm_connector *connectors[8];
	const struct drm_connector_helper_funcs *connector_funcs[8];
};

struct vpout_drm_module;

struct vpout_drm_module_ops {
	int (*modeset_init)(struct vpout_drm_module *mod,
			    struct drm_device *dev);
};

struct vpout_drm_module {
	const char *name;
	struct list_head list;
	const struct vpout_drm_module_ops *funcs;
	unsigned int preferred_bpp;
};

void vpout_drm_module_init(struct vpout_drm_module *mod, const char *name,
			   const struct vpout_drm_module_ops *funcs);

void vpout_drm_module_cleanup(struct vpout_drm_module *mod);

struct vpout_drm_panel_info {
	uint32_t bpp;
	bool invert_pxl_clk;
};

int vpout_drm_crtc_mode_valid(struct drm_crtc *crtc,
			      struct drm_display_mode *mode);

void vpout_drm_crtc_set_panel_info(struct drm_crtc *crtc,
				   const struct vpout_drm_panel_info *info);

irqreturn_t vpout_drm_crtc_irq(struct drm_crtc *crtc);

struct drm_crtc *vpout_drm_crtc_create(struct drm_device *dev);

#endif /* __VPOUT_DRM_DRV_H__ */
