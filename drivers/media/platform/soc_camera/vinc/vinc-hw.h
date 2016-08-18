/*
 * Copyright 2016 ELVEES NeoTek JSC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef VINC_CONFIG_H
#define VINC_CONFIG_H

#include <linux/types.h>
#include <linux/vinc.h>

#include <media/soc_camera.h>

#include "vinc-dev.h"

void vinc_write(struct vinc_dev *priv,
		unsigned long reg_offs, u32 data);

u32 vinc_read(struct vinc_dev *priv, unsigned long reg_offs);

void vinc_configure_input(struct vinc_stream *stream);

void vinc_configure(struct vinc_dev *priv, struct soc_camera_device *icd);

void set_cc_ct(struct vinc_dev *priv, u8 devnum, struct vinc_cc *cc,
	       int is_ct);

void vinc_stat_start(struct vinc_stream *stream);

void set_stat_zone(struct vinc_stream *stream, u32 zone_id,
		   struct vinc_stat_zone *zone);

#endif /* VINC_CONFIG_H */
