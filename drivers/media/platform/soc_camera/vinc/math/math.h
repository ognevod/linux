/*
 * Copyright 2016 ELVEES NeoTek JSC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef VINC_MATH_H
#define VINC_MATH_H

#ifndef __STDC__
#define __STDC__
#endif /*__STDC__ */

#ifndef _IEEE_LIBM
#define _IEEE_LIBM
#endif /*  _IEEE_LIBM */

#define PI 3.141592653589793238

#define __HI(x) (*(1+(int *)&x))
#define __LO(x) (*(int *)&x)
#define __HIp(x) (*(1+(int *)x))
#define __LOp(x) (*(int *)x)

double __ieee754_pow(double x, double y);
double __ieee754_sqrt(double x);
double __kernel_sin(double x, double y, int iy);
double __kernel_cos(double x, double y);
double __kernel_tan(double x, double y, int iy);
double rint(double x);
double copysign(double x, double y);
double scalbn(double x, int n);
double fabs(double x);
double sqrt(double x);
double pow(double x, double y);
double cos(double x);
double sin(double x);
double tan(double x);
double floor(double x);
int __ieee754_rem_pio2(double x, double *y);
int __kernel_rem_pio2(double *x, double *y, int e0, int nx, int prec,
			const int *ipio2);

#endif /* VINC_MATH_H */
