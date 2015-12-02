/*
 * Copyright 2015 ELVEES NeoTek CJSC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#ifndef _UAPI__LINUX_VINC_H
#define _UAPI__LINUX_VINC_H

#include <linux/types.h>

#define V4L2_CID_BAD_CORRECTION_ENABLE	(V4L2_CID_CAMERA_CLASS_BASE + 0x1000)
#define V4L2_CID_BAD_PIXELS		(V4L2_CID_CAMERA_CLASS_BASE + 0x1001)
#define V4L2_CID_BAD_ROWS		(V4L2_CID_CAMERA_CLASS_BASE + 0x1002)
#define V4L2_CID_BAD_COLS		(V4L2_CID_CAMERA_CLASS_BASE + 0x1003)
#define V4L2_CID_GAMMA_CURVE_ENABLE	(V4L2_CID_CAMERA_CLASS_BASE + 0x1004)
#define V4L2_CID_GAMMA_CURVE		(V4L2_CID_CAMERA_CLASS_BASE + 0x1005)
#define V4L2_CID_CC_ENABLE		(V4L2_CID_CAMERA_CLASS_BASE + 0x1006)
#define V4L2_CID_CC			(V4L2_CID_CAMERA_CLASS_BASE + 0x1007)
#define V4L2_CID_CT_ENABLE		(V4L2_CID_CAMERA_CLASS_BASE + 0x1008)
#define V4L2_CID_CT			(V4L2_CID_CAMERA_CLASS_BASE + 0x1009)
#define V4L2_CID_DR_ENABLE		(V4L2_CID_CAMERA_CLASS_BASE + 0x100A)
#define V4L2_CID_DR			(V4L2_CID_CAMERA_CLASS_BASE + 0x100B)

#define VINC_COMP_VALUES_COUNT		(1 << 12)
#define VINC_CC_COEFF_COUNT		9
#define VINC_CC_OFFSET_COUNT		3

struct vinc_bad_pixel {
	__u16 x;
	__u16 y;
};

struct vinc_gamma_curve {
	__u16 red[VINC_COMP_VALUES_COUNT];
	__u16 green[VINC_COMP_VALUES_COUNT];
	__u16 blue[VINC_COMP_VALUES_COUNT];
};

struct vinc_cc {
	__u16 coeff[VINC_CC_COEFF_COUNT];
	__u16 offset[VINC_CC_OFFSET_COUNT];
	__u8 scaling;
};

#endif /* _UAPI__LINUX_VINC_H */
