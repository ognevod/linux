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

#ifndef __VPOUT_DRM_PANEL_H__
#define __VPOUT_DRM_PANEL_H__

int vpout_drm_panel_init(void);

void vpout_drm_panel_fini(void);

#endif /* __VPOUT_DRM_PANEL_H__ */
