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

#ifndef __VPOUT_DRM_REGS_H__
#define __VPOUT_DRM_REGS_H__

#include <linux/bitops.h>

#include "vpout-drm-drv.h"

#define LCDC_CSR			0x00
#define LCDC_DIV			0x04
#define LCDC_MODE			0x08
#define LCDC_HT0			0x0C
#define LCDC_HT1			0x10
#define LCDC_VT0			0x14
#define LCDC_VT1			0x18
#define LCDC_AB0			0x2C
#define LCDC_AB1			0x30
#define LCDC_OFF0			0x34
#define LCDC_OFF1			0x38
#define LCDC_INT			0x44
#define LCDC_INTMASK			0x48

#define LCDC_CSR_CLR			BIT(3)
#define LCDC_CSR_INIT			BIT(2)
#define LCDC_CSR_RUN			BIT(1)
#define LCDC_CSR_EN			BIT(0)

#define LCDC_MODE_CLK_ON		BIT(17)
#define LCDC_MODE_VDEF			BIT(16)
#define LCDC_MODE_HDEF			BIT(15)
#define LCDC_MODE_DEN_EN		BIT(14)
#define LCDC_MODE_INSYNC		BIT(13)
#define LCDC_MODE_CCM			BIT(12)
#define LCDC_MODE_PINV			BIT(11)
#define LCDC_MODE_DINV			BIT(10)
#define LCDC_MODE_VINV			BIT(9)
#define LCDC_MODE_HINV			BIT(8)
#define LCDC_MODE_BUF_NUMB		BIT(7)
#define LCDC_MODE_BUF_MODE		BIT(6)
#define LCDC_MODE_HWC_MODE		BIT(5)
#define LCDC_MODE_HWCEN			BIT(4)
#define LCDC_MODE_INSIZE(v)		((v) << 0)

#define LCDC_MODE_INSIZE_8BPP		0
#define LCDC_MODE_INSIZE_12BPP		1
#define LCDC_MODE_INSIZE_15BPP		2
#define LCDC_MODE_INSIZE_16BPP		3
#define LCDC_MODE_INSIZE_18BPP		4
#define LCDC_MODE_INSIZE_24BPP		5
#define LCDC_MODE_INSIZE_32BPP		6

#define LCDC_HT0_HGDEL(v)		((v) << 16)
#define LCDC_HT0_HSW(v)			((v) << 0)
#define LCDC_HT1_HLEN(v)		((v) << 16)
#define LCDC_HT1_HGATE(v)		((v) << 0)

#define LCDC_VT0_VGDEL(v)		((v) << 16)
#define LCDC_VT0_VSW(v)			((v) << 0)
#define LCDC_VT1_VLEN(v)		((v) << 16)
#define LCDC_VT1_VGATE(v)		((v) << 0)

#define LCDC_INT_SYNC_DONE		BIT(5)
#define LCDC_INT_OUT_FIFO_EMPTY		BIT(3)
#define LCDC_INT_OUT_FIFO_INT		BIT(2)
#define LCDC_INT_DMA_FIFO_EMPTY		BIT(1)
#define LCDC_INT_DMA_DONE		BIT(0)

/*
 * Helpers:
 */

static inline void vpout_drm_write(struct drm_device *dev, u32 reg, u32 data)
{
	struct vpout_drm_private *priv = dev->dev_private;

	iowrite32(data, priv->mmio + reg);
}

static inline u32 vpout_drm_read(struct drm_device *dev, u32 reg)
{
	struct vpout_drm_private *priv = dev->dev_private;

	return ioread32(priv->mmio + reg);
}

static inline void vpout_drm_set(struct drm_device *dev, u32 reg, u32 mask)
{
	vpout_drm_write(dev, reg, vpout_drm_read(dev, reg) | mask);
}

static inline void vpout_drm_clear(struct drm_device *dev, u32 reg, u32 mask)
{
	vpout_drm_write(dev, reg, vpout_drm_read(dev, reg) & ~mask);
}

#endif /* __VPOUT_DRM_REGS_H__ */
