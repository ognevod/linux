/*
 * Copyright 2016 ELVEES NeoTek JSC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/vinc.h>
#include <linux/log2.h>
#include "vinc-neon.h"
#include "math/math.h"

void vinc_calculate_gamma_curve(int value, struct vinc_gamma_curve *gamma)
{
	double gamma_dbl, cur_dbl;
	int i, cur;

	gamma_dbl = (value / 32.0) / (1 - value / 32.0);
	for (i = 0; i < 4096; i++) {
		cur_dbl = 4095 * (pow(i / 4095.0, gamma_dbl));
		cur = (int) rint(cur_dbl);
		gamma->red[i]   = cur;
		gamma->green[i] = cur;
		gamma->blue[i]  = cur;
	}
}

void vinc_calculate_wb_matrix(u32 sum_r, u32 sum_g, u32 sum_b,
			      void *matrix)
{
	struct matrix *wb = (struct matrix *)matrix;

	memset(wb, 0, sizeof(struct matrix));
	wb->coeff[0] = ((double) sum_g) / sum_r;
	wb->coeff[4] = 1;
	wb->coeff[8] = ((double) sum_g) / sum_b;
}

void vinc_calculate_cc(void *coeffs[1], struct vinc_cc *cc)
{
	int i;
	double max = 0, cur;
	int scaler;
	struct matrix *wb = (struct matrix *)coeffs[0];

	for (i = 0;  i < 9; i++) {
		cur = fabs(wb->coeff[i]);
		if (max < cur)
			max = cur;
	}

	scaler = ilog2((u16)max) + 1;

	for (i = 0; i < 9; i++)
		cc->coeff[i] = COEFF_FLOAT_TO_U16(wb->coeff[i], scaler);

	cc->offset[0] = 0;
	cc->offset[1] = 0;
	cc->offset[2] = 0;
	cc->scaling = scaler;
}
