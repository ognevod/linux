/*
 * Copyright 2016 ELVEES NeoTek JSC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef VINC_CTRLS_H
#define VINC_CTRLS_H

#include <media/v4l2-ctrls.h>

#include "vinc-dev.h"

void vinc_stat_tasklet(unsigned long data);

int vinc_create_controls(struct v4l2_ctrl_handler *hdl,
			 struct vinc_stream *stream);

#endif /* VINC_CTRLS_H */
