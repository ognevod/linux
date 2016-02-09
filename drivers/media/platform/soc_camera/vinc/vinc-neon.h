/*
 * Copyright 2016 ELVEES NeoTek JSC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef VINC_NEON_H
#define VINC_NEON_H

#include <linux/videodev2.h>
#include <linux/vinc.h>
#include <media/v4l2-ctrls.h>

#define COEFF_FLOAT_TO_U16(coeff, scaling) ((u16)((s16)((coeff) * \
		(1 << (15 - (scaling))) + ((coeff) < 0 ? -0.5 : 0.5))))

struct matrix {
	double coeff[9];
};

void vinc_calculate_gamma_curve(int value, struct vinc_gamma_curve
			*gamma_ptr);

void vinc_calculate_wb_matrix(u32 sum_r, u32 sum_g, u32 sum_b,
	void *matrix);
void vinc_calculate_cc(void *coeffs[1], struct vinc_cc *cc);

#endif /* VINC_NEON_H */
