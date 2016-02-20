/*
 * Copyright 2016 ELVEES NeoTek JSC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/vinc.h>
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
