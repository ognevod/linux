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

/* Matrices for RGB->YCbCr conversion depends on YCbCr ENCODING.
 * Output QUANTIZATION is always in FULL RANGE.
 * Table of coefficients:

   coefficient || R  G  B
   =======================
     for Y     || 0  1  2
     for Cb    || 3  4  5
     for Cr    || 6  7  8 */
const struct matrix m_ycbcr[4] = {
	/* BT 601 */
	[0] = {
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
	/* REC 709 */
	[1] = {
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
	/* BT 2020 */
	[2] = {
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
	/* sRGB */
	[3] = {
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
	/* BT 601, LIMITED RANGE */
	[0][0] = {
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
	/* BT 601, FULL RANGE */
	[0][1] = {
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

	/* REC 709, LIMITED RANGE */
	[1][0] = {
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
	/* REC 709, FULL RANGE */
	[1][1] = {
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

	/* BT 2020, LIMITED RANGE */
	[2][0] = {
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
	/* BT 2020, FULL RANGE */
	[2][1] = {
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

	/* sRGB, LIMITED RANGE */
	[3][0] = {
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
	/* sRGB, FULL RANGE */
	[3][1] = {
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
	/* BT 601, LIMITED RANGE */
	[0][0] = {
		.offset[0] = -3566.74504892368,
		.offset[1] =  2169.20471987651,
		.offset[2] = -4429.37362035225,
	},
	/* BT 601, FULL RANGE */
	[0][1] = {
		.offset[0] = -2871.296,
		.offset[1] =  2167.34222827939,
		.offset[2] = -3629.056,
	},

	/* REC 709, LIMITED RANGE */
	[1][0] = {
		.offset[0] = -3969.61590606654,
		.offset[1] =  1230.04927514152,
		.offset[2] = -4624.28104892368,
	},
	/* REC 709, FULL RANGE */
	[1][1] = {
		.offset[0] = -3225.1904,
		.offset[1] =  1342.35862192394,
		.offset[2] = -3800.2688,
	},

	/* BT 2020, LIMITED RANGE */
	[2][0] = {
		.offset[0] = -3736.00676320939,
		.offset[1] =  1417.63067395875,
		.offset[2] = -4684.43190606654,
	},
	/* BT 2020, FULL RANGE */
	[2][1] = {
		.offset[0] = -3019.9808,
		.offset[1] =  1507.13600755162,
		.offset[2] = -3853.1072,
	},

	/* sRGB, LIMITED RANGE */
	[3][0] = {
		.offset[0] = -3566.63024225816,
		.offset[1] =  2169.07539559518,
		.offset[2] = -4429.00882947368,
	},
	/* sRGB, FULL RANGE */
	[3][1] = {
		.offset[0] = -2871.19515022323,
		.offset[1] =  2167.22862577343,
		.offset[2] = -3628.73555624785,
	}
};

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

/* Matrices and vectors for CC controls */
void vinc_calculate_v_bri(void *vector, s32 val)
{
	u8 i;
	struct vector *v = (struct vector *)vector;

	v->offset[0] = (double)val;
	for (i = 1; i < VINC_CC_OFFSET_COUNT; i++)
		v->offset[i] = 0;
}

void vinc_calculate_m_wb(u32 sum_r, u32 sum_g, u32 sum_b,
			      void *matrix)
{
	struct matrix *wb = (struct matrix *)matrix;

	memset(wb, 0, sizeof(struct matrix));
	wb->coeff[0] = ((double) sum_g) / sum_r;
	wb->coeff[4] = 1;
	wb->coeff[8] = ((double) sum_g) / sum_b;
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
static void cc_matrix_calc(struct matrix *coeff, void *ctrl_privs[],
				u8 vinc_enc, u8 vinc_qnt)
{
	struct matrix tmp1;
	struct matrix tmp2;

	struct matrix *wb = (struct matrix *)ctrl_privs[0];

	/* V4L2_CID_DO_WHITE_BALANCE control matrix and RGB->YCbCr matrix
	 * multiplication */
	mxm_mult(&tmp2, &m_ycbcr[vinc_enc], wb);

	/* YCbCr->RGB matrix multiplication */
	mxm_mult(&tmp1, &m_rgb[vinc_enc][vinc_qnt], &tmp2);

	*coeff = tmp1;
}

/* Calculate CC offset vector according to control matrices and vectors */
static void cc_vector_calc(struct vector *offset, void *ctrl_privs[],
				u8 vinc_enc, u8 vinc_qnt)
{
	struct vector tmp1;
	struct vector tmp2;

	struct vector *v_bri = (struct vector *)ctrl_privs[1];

	/* RGB->YCbCr vector and V4L2_CID_BRIGHTNESS control vector addition */
	vxv_add(&tmp2, &v_ycbcr, v_bri);

	/* YCbCr->RGB matrix multiplication */
	mxv_mult(&tmp1, &m_rgb[vinc_enc][vinc_qnt], &tmp2);

	/* YCbCr->RGB vector addition */
	vxv_add(&tmp2, &v_rgb[vinc_enc][vinc_qnt], &tmp1);

	*offset = tmp2;
}

/* Color Correction coefficient matrix, offset vector and scaling register
 * calculation routine */
void vinc_calculate_cc(void *ctrl_privs[], enum v4l2_ycbcr_encoding ycbcr_enc,
		       enum v4l2_quantization quantization, struct vinc_cc *cc)
{
	struct matrix coeff;
	struct vector offset;
	u8 scaling;

	u16 i;
	double max_abs = 0;
	u8 vinc_enc;
	u8 vinc_qnt;

	/* Select proper YCbCr<-> RGB coefficients and offsets according to
	 * YCbCr encoding and quantization */

	/* BUG: Add proper color spaces handler */
	switch (ycbcr_enc) {
	case V4L2_YCBCR_ENC_709:
		vinc_enc = 1;
		break;
	case V4L2_YCBCR_ENC_BT2020:
		vinc_enc = 2;
		break;
	case V4L2_YCBCR_ENC_SYCC:
		vinc_enc = 3;
		break;
	default:   /* V4L2_YCBCR_ENC_601 */
		vinc_enc = 0;
		break;
	}

	switch (quantization) {
	case V4L2_QUANTIZATION_FULL_RANGE:
		vinc_qnt = 1;
		break;
	default:   /* V4L2_QUANTIZATION_LIM_RANGE */
		vinc_qnt = 0;
		break;
	}

	cc_matrix_calc(&coeff, ctrl_privs, vinc_enc, vinc_qnt);
	cc_vector_calc(&offset, ctrl_privs, vinc_enc, vinc_qnt);
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
		cc->offset[i] = (u16)((s16)offset.offset[i]);
}
