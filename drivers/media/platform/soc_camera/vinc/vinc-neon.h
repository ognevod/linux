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

#include <linux/vinc.h>

void vinc_calculate_gamma_curve(int value, struct vinc_gamma_curve *gamma_ptr);

#endif /* VINC_NEON_H */
