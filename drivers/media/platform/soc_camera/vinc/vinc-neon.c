/*
 * Copyright 2016 ELVEES NeoTek JSC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/log2.h>
#include "vinc-neon.h"
#include "math/math.h"

#define OFFSET_MAX 4096.0

/* Matrices for RGB->YCbCr conversion depends on YCbCr ENCODING.
 * Output QUANTIZATION is always in FULL RANGE.
 * Table of coefficients:

   coefficient || R  G  B
   =======================
     for Y     || 0  1  2
     for Cb    || 3  4  5
     for Cr    || 6  7  8 */
const struct matrix m_ycbcr[4] = {
	[VINC_YCBCR_ENC_601] = {
		.coeff[0] =  0.299,
		.coeff[1] =  0.587,
		.coeff[2] =  0.114,
		.coeff[3] = -0.168735891647856,
		.coeff[4] = -0.331264108352144,
		.coeff[5] =  0.5,
		.coeff[6] =  0.5,
		.coeff[7] = -0.418687589158345,
		.coeff[8] = -0.0813124108416548,
	},
	[VINC_YCBCR_ENC_709] = {
		.coeff[0] =  0.2126,
		.coeff[1] =  0.7152,
		.coeff[2] =  0.0722,
		.coeff[3] = -0.11457210605734,
		.coeff[4] = -0.38542789394266,
		.coeff[5] =  0.5,
		.coeff[6] =  0.5,
		.coeff[7] = -0.454152908305817,
		.coeff[8] = -0.0458470916941834,
	},
	[VINC_YCBCR_ENC_BT2020] = {
		.coeff[0] =  0.2627,
		.coeff[1] =  0.678,
		.coeff[2] =  0.0593,
		.coeff[3] = -0.139630062719252,
		.coeff[4] = -0.38542789394266,
		.coeff[5] =  0.5,
		.coeff[6] =  0.5,
		.coeff[7] = -0.459785704597857,
		.coeff[8] = -0.040214295402143,
	},
	[VINC_YCBCR_ENC_SYCC] = {
		.coeff[0] =  0.299,
		.coeff[1] =  0.587,
		.coeff[2] =  0.114,
		.coeff[3] = -0.1687,
		.coeff[4] = -0.3313,
		.coeff[5] =  0.5,
		.coeff[6] =  0.5,
		.coeff[7] = -0.4187,
		.coeff[8] = -0.0813,
	}
};

/* Offsets for RGB->YCbCr conversion.
 * Output QUANTIZATION is always in FULL RANGE.
 * Vector elements go in this order:
 * 0: Y offset
 * 1: Cb offset
 * 2: Cr offset */
const struct vector v_ycbcr = {
	.offset[0] = 0,
	.offset[1] = 2048,
	.offset[2] = 2048,
};

/* Matrices for YCbCr->RGB conversion.
 * Depend on YCbCr ENCODING and input QUANTIZATION.
 * Table of coefficients:

   coefficient || Y Cb Cr
   =======================
     for R     || 0  1  2
     for G     || 3  4  5
     for B     || 6  7  8 */
const struct matrix m_rgb[4][2] = {
	[VINC_YCBCR_ENC_601][VINC_QUANTIZATION_LIM_RANGE] = {
		.coeff[0] =  1.16438356164384,
		.coeff[1] =  5.59894392753486e-17,
		.coeff[2] =  1.59602678571429,
		.coeff[3] =  1.16438356164384,
		.coeff[4] = -0.391762290094914,
		.coeff[5] = -0.81296764723777,
		.coeff[6] =  1.16438356164384,
		.coeff[7] =  2.01723214285714,
		.coeff[8] =  1.11022302462516e-16,

	},
	[VINC_YCBCR_ENC_601][VINC_QUANTIZATION_FULL_RANGE] = {
		.coeff[0] =  1,
		.coeff[1] =  4.91828799908944e-17,
		.coeff[2] =  1.402,
		.coeff[3] =  1,
		.coeff[4] = -0.344136286201022,
		.coeff[5] = -0.714136286201022,
		.coeff[6] =  1,
		.coeff[7] =  1.772,
		.coeff[8] =  0,
	},

	[VINC_YCBCR_ENC_709][VINC_QUANTIZATION_LIM_RANGE] = {
		.coeff[0] =  1.16438356164384,
		.coeff[1] =  0,
		.coeff[2] =  1.79274107142857,
		.coeff[3] =  1.16438356164384,
		.coeff[4] = -0.21324861427373,
		.coeff[5] = -0.532909328559444,
		.coeff[6] =  1.16438356164384,
		.coeff[7] =  2.11240178571429,
		.coeff[8] = -5.55111512312578e-17,
	},
	[VINC_YCBCR_ENC_709][VINC_QUANTIZATION_FULL_RANGE] = {
		.coeff[0] =  1,
		.coeff[1] =  0,
		.coeff[2] =  1.5748,
		.coeff[3] =  1,
		.coeff[4] = -0.187324272930649,
		.coeff[5] = -0.468124272930649,
		.coeff[6] =  1,
		.coeff[7] =  1.8556,
		.coeff[8] =  5.55111512312578e-17,
	},

	[VINC_YCBCR_ENC_BT2020][VINC_QUANTIZATION_LIM_RANGE] = {
		.coeff[0] =  1.16438356164384,
		.coeff[1] = -5.94461236188718e-17,
		.coeff[2] =  1.67867410714286,
		.coeff[3] =  1.16438356164384,
		.coeff[4] = -0.187326104219343,
		.coeff[5] = -0.650424318505057,
		.coeff[6] =  1.16438356164384,
		.coeff[7] =  2.14177232142857,
		.coeff[8] =  0,
	},
	[VINC_YCBCR_ENC_BT2020][VINC_QUANTIZATION_FULL_RANGE] = {
		.coeff[0] =  1,
		.coeff[1] = -2.61096699816221e-17,
		.coeff[2] =  1.4746,
		.coeff[3] =  1,
		.coeff[4] = -0.164553126843658,
		.coeff[5] = -0.571353126843658,
		.coeff[6] =  1,
		.coeff[7] =  1.8814,
		.coeff[8] =  0,
	},

	[VINC_YCBCR_ENC_SYCC][VINC_QUANTIZATION_LIM_RANGE] = {
		.coeff[0] =  1.16438356164384,
		.coeff[1] = -4.19156139872783e-05,
		.coeff[2] =  1.59601264338613,
		.coeff[3] =  1.16438356164384,
		.coeff[4] = -0.391736101495071,
		.coeff[5] = -0.812930689215866,
		.coeff[6] =  1.16438356164384,
		.coeff[7] =  2.01720723110692,
		.coeff[8] = -0.000153208795953375,
	},
	[VINC_YCBCR_ENC_SYCC][VINC_QUANTIZATION_FULL_RANGE] = {
		.coeff[0] =  1,
		.coeff[1] = -3.68199903261289e-05,
		.coeff[2] =  1.40198757693526,
		.coeff[3] =  1,
		.coeff[4] = -0.344113281313317,
		.coeff[5] = -0.714103821115113,
		.coeff[6] =  1,
		.coeff[7] =  1.77197811673706,
		.coeff[8] = -0.000134583412916056,
	}
};

/* Offsets for YCbCr->RGB conversion.
 * Depend on YCbCr ENCODING and input QUANTIZATION
 * Vector elements go in this order:
 * 0: R offset
 * 1: G offset
 * 2: B offset */
const struct vector v_rgb[4][2] = {
	[VINC_YCBCR_ENC_601][VINC_QUANTIZATION_LIM_RANGE] = {
		.offset[0] = -3566.74504892368,
		.offset[1] =  2169.20471987651,
		.offset[2] = -4429.37362035225,
	},
	[VINC_YCBCR_ENC_601][VINC_QUANTIZATION_FULL_RANGE] = {
		.offset[0] = -2871.296,
		.offset[1] =  2167.34222827939,
		.offset[2] = -3629.056,
	},

	[VINC_YCBCR_ENC_709][VINC_QUANTIZATION_LIM_RANGE] = {
		.offset[0] = -3969.61590606654,
		.offset[1] =  1230.04927514152,
		.offset[2] = -4624.28104892368,
	},
	[VINC_YCBCR_ENC_709][VINC_QUANTIZATION_FULL_RANGE] = {
		.offset[0] = -3225.1904,
		.offset[1] =  1342.35862192394,
		.offset[2] = -3800.2688,
	},

	[VINC_YCBCR_ENC_BT2020][VINC_QUANTIZATION_LIM_RANGE] = {
		.offset[0] = -3736.00676320939,
		.offset[1] =  1417.63067395875,
		.offset[2] = -4684.43190606654,
	},
	[VINC_YCBCR_ENC_BT2020][VINC_QUANTIZATION_FULL_RANGE] = {
		.offset[0] = -3019.9808,
		.offset[1] =  1507.13600755162,
		.offset[2] = -3853.1072,
	},

	[VINC_YCBCR_ENC_SYCC][VINC_QUANTIZATION_LIM_RANGE] = {
		.offset[0] = -3566.63024225816,
		.offset[1] =  2169.07539559518,
		.offset[2] = -4429.00882947368,
	},
	[VINC_YCBCR_ENC_SYCC][VINC_QUANTIZATION_FULL_RANGE] = {
		.offset[0] = -2871.19515022323,
		.offset[1] =  2167.22862577343,
		.offset[2] = -3628.73555624785,
	}
};

/*
 * This function clips float value to [-OFFSET_MAX; OFFSET_MAX - 1]
 * interval and converts it to u16 with fbits fractional bits.
 */
static u16 offset_float_to_u16(double offset, u8 fbits)
{
	offset = clamp(offset, -OFFSET_MAX, OFFSET_MAX - 1);
	return (u16)((s16)(offset * (1 << fbits)));
}

#define TEMP_TABLE_STEP 200
#define TEMP_TABLE_MIN 2000
#define TEMP_TABLE_MAX 9000
#define NUM_TABLE ((TEMP_TABLE_MAX - TEMP_TABLE_MIN) / TEMP_TABLE_STEP + 1)

static const double t2rgb[NUM_TABLE][3] = {
	{1.000, 0.234, 0.063}, {1.000, 0.281, 0.063}, {1.000, 0.327, 0.063},
	{1.000, 0.371, 0.082}, {1.000, 0.413, 0.115}, {1.000, 0.454, 0.151},
	{1.000, 0.494, 0.190}, {1.000, 0.531, 0.232}, {1.000, 0.567, 0.276},
	{1.000, 0.602, 0.322}, {1.000, 0.635, 0.370}, {1.000, 0.666, 0.419},
	{1.000, 0.696, 0.468}, {1.000, 0.724, 0.519}, {1.000, 0.752, 0.569},
	{1.000, 0.778, 0.620}, {1.000, 0.802, 0.671}, {1.000, 0.826, 0.721},
	{1.000, 0.848, 0.771}, {1.000, 0.870, 0.820}, {1.000, 0.890, 0.869},
	{1.000, 0.909, 0.917}, {1.000, 0.928, 0.965}, {0.989, 0.935, 1.000},
	{0.946, 0.910, 1.000}, {0.907, 0.888, 1.000}, {0.841, 0.848, 1.000},
	{0.813, 0.831, 1.000}, {0.786, 0.815, 1.000}, {0.762, 0.800, 1.000},
	{0.740, 0.785, 1.000}, {0.720, 0.772, 1.000}, {0.701, 0.760, 1.000},
	{0.684, 0.749, 1.000}, {0.668, 0.738, 1.000}, {0.668, 0.738, 1.000}
};

void vinc_neon_calculate_gamma_curve(int value, struct vinc_gamma_curve *gamma)
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

/* Matrices and vectors for CC controls */
void vinc_neon_calculate_v_bri(void *vector, s32 val)
{
	struct vector *bri = (struct vector *)vector;

	*bri = (struct vector) {
		.offset[0] = (double)val
	};
}

void vinc_neon_calculate_m_con(void *matrix, s32 val)
{
	struct matrix *con = (struct matrix *)matrix;

	*con = (struct matrix) {
		.coeff[0] = tan((double)val * PI / 512),
		.coeff[4] = 1,
		.coeff[8] = 1
	};
}

void vinc_neon_calculate_m_sat(void *matrix, s32 val)
{
	struct matrix *sat = (struct matrix *)matrix;

	*sat = (struct matrix) {
		.coeff[0] = 1,
		.coeff[4] = tan((double)val * PI / 512),
		.coeff[8] = tan((double)val * PI / 512)
	};
}

void vinc_neon_calculate_m_hue(void *matrix, s32 val)
{
	struct matrix *hue = (struct matrix *)matrix;

	*hue = (struct matrix) {
		.coeff[0] =  1,
		.coeff[4] =  cos((double)val * PI / 512),
		.coeff[5] = -sin((double)val * PI / 512),
		.coeff[7] =  sin((double)val * PI / 512),
		.coeff[8] =  cos((double)val * PI / 512)
	};
}

static void t2rgb_interpolate(u32 t, double rgb[3])
{
	u32 i, t_index, t_max;
	double delta[3];
	double coeff_min[3], coeff_max[3];

	for (t_index = 1; t_index < NUM_TABLE; t_index++) {
		t_max = TEMP_TABLE_MIN + TEMP_TABLE_STEP * t_index;
		if ((t < t_max) || (t_max == TEMP_TABLE_MAX))
			break;
	}

	for (i = 0; i < 3; i++) {
		coeff_min[i] = t2rgb[t_index-1][i];
		coeff_max[i] = t2rgb[t_index][i];
		delta[i] = (coeff_max[i] - coeff_min[i]) / TEMP_TABLE_STEP;
		rgb[i] = coeff_max[i] - delta[i] * (t_max - t);
	}
}

void vinc_neon_wb_stat(u32 red, u32 green, u32 blue, u32 t, s32 *cptr[])
{
	double green_level = 1.0;
	double rgb[3], Kr, Kg, Kb;

	if (t) {
		t2rgb_interpolate(t, rgb);
		green_level = (rgb[1] / rgb[0]) / ((double)green / red);
		rgb[1] /= green_level;
		Kg = 1.0 / rgb[1];
		Kr = (1.0 / rgb[0]) / Kg;
		Kb = (1.0 / rgb[2]) / Kg;
	} else {
		Kr = (double)green / red;
		Kb = (double)green / blue;
	}

	*cptr[0] = rint((Kr / (Kr + 1)) * 256 - 128);
	*cptr[1] = rint((Kb / (Kb + 1)) * 256 - 128);
}

void vinc_neon_calculate_m_wb(u32 rb, u32 bb, void *matrix)
{
	struct matrix *wb = (struct matrix *)matrix;

	*wb = (struct matrix) {
		.coeff[0] = ((rb + 128) / 256.0) / (1 - (rb + 128) / 256.0),
		.coeff[4] = 1,
		.coeff[8] = ((bb + 128) / 256.0) / (1 - (bb + 128) / 256.0)
	};
}

void vinc_neon_calculate_m_ck(void *matrix, s32 val)
{
	struct matrix *ck = (struct matrix *)matrix;

	*ck = (struct matrix) {
		.coeff[0] =  1,
		.coeff[4] =  1 - (double)val,
		.coeff[8] =  1 - (double)val
	};
}

/*  Matrix and matrix multiplication
 *  m1 - left matrix, m2 - right matrix. Order of multiplication matters. */
static void mxm_mult(struct matrix *prod, const struct matrix *m1,
		     const struct matrix *m2)
{
	u8 i, j, k;
	double sum;

	for (i = 0; i < VINC_CC_OFFSET_COUNT; i++) {
		for (j = 0; j < VINC_CC_OFFSET_COUNT; j++) {
			sum = 0;
			for (k = 0; k < VINC_CC_OFFSET_COUNT; k++)
				sum += m1->coeff[k + VINC_CC_OFFSET_COUNT * i]
				     * m2->coeff[j + VINC_CC_OFFSET_COUNT * k];
			prod->coeff[j + VINC_CC_OFFSET_COUNT * i] = sum;
		}
	}
}

/*  Matrix and vector multiplication
 *  m - left matrix, v - right vector. */
static void mxv_mult(struct vector *prod, const struct matrix *m,
		     const struct vector *v)
{
	u8 i, j;
	double sum;

	for (i = 0; i < VINC_CC_OFFSET_COUNT; i++) {
		sum = 0;
		for (j = 0; j < VINC_CC_OFFSET_COUNT; j++)
			sum += m->coeff[j + VINC_CC_OFFSET_COUNT * i] *
			       v->offset[j];
		prod->offset[i] = sum;
	}
}

/*  Matrix addition. Matrices must have the same size */
static void mxm_add(struct matrix *sum, const struct matrix *m1,
		    const struct matrix *m2)
{
	u8 i;

	for (i = 0; i < VINC_CC_COEFF_COUNT; i++)
		sum->coeff[i] = m1->coeff[i] + m2->coeff[i];
}

/*  Vector addition. Vectors must have the same size */
static void vxv_add(struct vector *sum, const struct vector *v1,
		    const struct vector *v2)
{
	u8 i;

	for (i = 0; i < VINC_CC_OFFSET_COUNT; i++)
		sum->offset[i] = v1->offset[i] + v2->offset[i];
}

/*  Calculate CC coefficient matrix. Uses controls matrices and vectors */
static void cc_matrix_calc(struct matrix *coeff, struct ctrl_priv *ctrl_privs,
		enum vinc_ycbcr_encoding ycbcr_enc)
{
	struct matrix tmp1;
	struct matrix tmp2;

	struct matrix *m_wb = (struct matrix *)ctrl_privs->dowb;
	struct matrix *m_con = (struct matrix *)ctrl_privs->contrast;
	struct matrix *m_sat = (struct matrix *)ctrl_privs->saturation;
	struct matrix *m_hue = (struct matrix *)ctrl_privs->hue;
	struct matrix *m_ck  = (struct matrix *)ctrl_privs->ck;

	/* M_cc = M_rgb * M_ck * M_sat * M_con * M_hue * M_ycbcr * M_wb
	 *              6      5       4       3       2         1   */
	mxm_mult(&tmp1, &m_ycbcr[ycbcr_enc], m_wb);   /* [1] */
	mxm_mult(&tmp2, m_hue, &tmp1);		     /* [2] */
	mxm_mult(&tmp1, m_con, &tmp2);		     /* [3] */
	mxm_mult(&tmp2, m_sat, &tmp1);		     /* [4] */
	mxm_mult(&tmp1, m_ck, &tmp2);		     /* [5] */
	mxm_mult(coeff, &m_rgb[ycbcr_enc][VINC_QUANTIZATION_FULL_RANGE],
		 &tmp1);			     /* [6] */
}

/* Calculate CC offset vector according to control matrices and vectors */
static void cc_vector_calc(struct vector *offset, struct ctrl_priv *ctrl_privs,
		enum vinc_ycbcr_encoding ycbcr_enc)
{
	struct vector tmp1;
	struct vector tmp2;
	struct vector half_plus = {
		.offset[0] = 2048,
		.offset[1] = 2048,
		.offset[2] = 2048
	};
	struct vector half_minus = {
		.offset[0] = -2048,
		.offset[1] = -2048,
		.offset[2] = -2048
	};
	struct vector *v_bri = (struct vector *)ctrl_privs->brightness;
	struct matrix *m_con = (struct matrix *)ctrl_privs->contrast;
	struct matrix *m_sat = (struct matrix *)ctrl_privs->saturation;
	struct matrix *m_hue = (struct matrix *)ctrl_privs->hue;
	struct matrix *m_ck  = (struct matrix *)ctrl_privs->ck;

	/* Vcc = M_rgb*(Mck*Msat*Mcon*Mhue*(Vycbcr-Vhalf)+Vbri+Vhalf)+Vrgb
	 *            8    5    4    3    2       1      6    7      9  */
	vxv_add(&tmp1, &v_ycbcr, &half_minus);   /* [1] */
	mxv_mult(&tmp2, m_hue, &tmp1);		 /* [2] */
	mxv_mult(&tmp1, m_con, &tmp2);		 /* [3] */
	mxv_mult(&tmp2, m_sat, &tmp1);		 /* [4] */
	mxv_mult(&tmp1, m_ck, &tmp2);		 /* [5] */
	vxv_add(&tmp2, v_bri, &tmp1);		 /* [6] */
	vxv_add(&tmp1, &half_plus, &tmp2);	 /* [7] */
	mxv_mult(&tmp2, &m_rgb[ycbcr_enc][VINC_QUANTIZATION_FULL_RANGE],
		 &tmp1);			 /* [8] */
	vxv_add(offset, &v_rgb[ycbcr_enc][VINC_QUANTIZATION_FULL_RANGE],
		&tmp2);				 /* [9] */
}

/* Color Correction coefficient matrix, offset vector and scaling register
 * calculation routine */
void vinc_neon_calculate_cc(struct ctrl_priv *ctrl_privs,
		enum vinc_ycbcr_encoding ycbcr_enc, struct vinc_cc *cc)
{
	struct matrix coeff;
	struct vector offset;
	u8 scaling;

	u16 i;
	double max_abs = 0;

	cc_matrix_calc(&coeff, ctrl_privs, ycbcr_enc);
	cc_vector_calc(&offset, ctrl_privs, ycbcr_enc);
	/*  Scaling calculation */
	for (i = 0; i < VINC_CC_COEFF_COUNT; i++)
		if (fabs(coeff.coeff[i]) > max_abs)
			max_abs = fabs(coeff.coeff[i]);

	if (max_abs < 1)
		scaling = 0;
	else
		scaling = ilog2((u16)max_abs) + 1;

	max_abs = max_abs + pow(2, scaling - 16);

	if (max_abs < 1)
		scaling = 0;
	else
		scaling = ilog2((u16)max_abs) + 1;

	/* Vector, matrix and scaling write to CC cache */
	cc->scaling = scaling;
	for (i = 0; i < VINC_CC_COEFF_COUNT; i++)
		cc->coeff[i] = COEFF_FLOAT_TO_U16(coeff.coeff[i], scaling);
	for (i = 0; i < VINC_CC_OFFSET_COUNT; i++)
		cc->offset[i] = offset_float_to_u16(offset.offset[i], 0);
}
