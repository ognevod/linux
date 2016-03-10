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

/* Matrices for RGB->YCbCr conversion depends on YCbCr ENCODING and QUANTIZATION
 * Matrices are written in GBR format (not RGB)
 * because VINC uses GBR for internal processing
 * Table of coefficients:

   coefficient || G  B  R
   =======================
     for Y     || 0  1  2
     for Cb    || 3  4  5
     for Cr    || 6  7  8 */
const struct matrix rgb_to_ycbcr_m[3][2] = {
	/* BT 601, LIMITED RANGE */
	[0][0] = {
		.coeff[0] =  0.504129411764706,
		.coeff[1] =  0.0979058823529412,
		.coeff[2] =  0.256788235294118,
		.coeff[3] = -0.290992785376001,
		.coeff[4] =  0.43921568627451,
		.coeff[5] = -0.148222900898508,
		.coeff[6] = -0.367788313613605,
		.coeff[7] = -0.0714273726609046,
		.coeff[8] =  0.43921568627451,
	},
	/* BT 601, FULL RANGE */
	[0][1] = {
		.coeff[0] =  0.587,
		.coeff[1] =  0.114,
		.coeff[2] =  0.299,
		.coeff[3] = -0.331264108352144,
		.coeff[4] =  0.5,
		.coeff[5] = -0.168735891647856,
		.coeff[6] = -0.418687589158345,
		.coeff[7] = -0.0813124108416548,
		.coeff[8] =  0.5,
	},

	/* REC 709, LIMITED RANGE */
	[1][0] = {
		.coeff[0] =  0.614230588235294,
		.coeff[1] =  0.0620070588235294,
		.coeff[2] =  0.182585882352941,
		.coeff[3] = -0.338571953894729,
		.coeff[4] =  0.43921568627451,
		.coeff[5] = -0.100643732379781,
		.coeff[6] = -0.398942162590207,
		.coeff[7] = -0.0402735236843023,
		.coeff[8] =  0.43921568627451,
	},
	/* REC 709, FULL RANGE */
	[1][1] = {
		.coeff[0] =  0.7152,
		.coeff[1] =  0.0722,
		.coeff[2] =  0.2126,
		.coeff[3] = -0.38542789394266,
		.coeff[4] =  0.5,
		.coeff[5] = -0.11457210605734,
		.coeff[6] = -0.454152908305817,
		.coeff[7] = -0.0458470916941834,
		.coeff[8] =  0.5,
	},

	/* BT 2020, LIMITED RANGE */
	[2][0] = {
		.coeff[0] =  0.582282352941176,
		.coeff[1] =  0.0509282352941176,
		.coeff[2] =  0.225612941176471,
		.coeff[3] = -0.316560258630932,
		.coeff[4] =  0.43921568627451,
		.coeff[5] = -0.122655427643578,
		.coeff[6] = -0.403890187568314,
		.coeff[7] = -0.0353254987061962,
		.coeff[8] =  0.43921568627451,
	},
	/* BT 2020, FULL RANGE */
	[2][1] = {
		.coeff[0] =  0.678,
		.coeff[1] =  0.0593,
		.coeff[2] =  0.2627,
		.coeff[3] = -0.38542789394266,
		.coeff[4] =  0.5,
		.coeff[5] = -0.139630062719252,
		.coeff[6] = -0.459785704597857,
		.coeff[7] = -0.040214295402143,
		.coeff[8] =  0.5,
	}
};

/* Offsets for RGB->YCbCr conversion depend on COLOR SPACE and QUANTIZATION
 * Vectors are written in GBR format (not RGB)
 * because VINC uses GBR for internal processing
 * Vector elements go in this order:
 * 0: Y offset
 * 1: Cb offset
 * 2: Cr offset */
const struct vector rgb_to_ycbcr_v[3][2] = {
	/* BT 601, LIMITED RANGE */
	[0][0] = {
		.offset[0] = 256,
		.offset[1] = 2048,
		.offset[2] = 2048,
	},
	/* BT 601, FULL RANGE */
	[0][1] = {
		.offset[0] = 0,
		.offset[1] = 2048,
		.offset[2] = 2048,
	},

	/* REC 709, LIMITED RANGE */
	[1][0] = {
		.offset[0] = 256,
		.offset[1] = 2048,
		.offset[2] = 2048,
	},
	/* REC 709, FULL RANGE */
	[1][1] = {
		.offset[0] = 0,
		.offset[1] = 2048,
		.offset[2] = 2048,
	},

	/* BT 2020, LIMITED RANGE */
	[2][0] = {
		.offset[0] = 256,
		.offset[1] = 2048,
		.offset[2] = 2048,
	},
	/* BT 2020, FULL RANGE */
	[2][1] = {
		.offset[0] = 0,
		.offset[1] = 2048,
		.offset[2] = 2048,
	}
};

/* Matrices for YCbCr->RGB conversion depends on COLOR SPACE and QUANTIZATION
 * Matrices are written in GBR format (not RGB)
 * because VINC uses GBR for internal processing
 * Table of coefficients:

   coefficient || Y Cb Cr
   =======================
     for G     || 0  1  2
     for B     || 3  4  5
     for R     || 6  7  8 */
const struct matrix ycbcr_to_rgb_m[3][2] = {
	/* BT 601, LIMITED RANGE */
	[0][0] = {
		.coeff[0] =  1.16438356164384,
		.coeff[1] = -0.391762290094914,
		.coeff[2] = -0.81296764723777,
		.coeff[3] =  1.16438356164384,
		.coeff[4] =  2.01723214285714,
		.coeff[5] =  1.11022302462516e-16,
		.coeff[6] =  1.16438356164384,
		.coeff[7] =  5.59894392753486e-17,
		.coeff[8] =  1.59602678571429,
	},
	/* BT 601, FULL RANGE */
	[0][1] = {
		.coeff[0] =  1,
		.coeff[1] = -0.344136286201022,
		.coeff[2] = -0.714136286201022,
		.coeff[3] =  1,
		.coeff[4] =  1.772,
		.coeff[5] =  0,
		.coeff[6] =  1,
		.coeff[7] =  4.91828799908944e-17,
		.coeff[8] =  1.402,
	},

	/* REC 709, LIMITED RANGE */
	[1][0] = {
		.coeff[0] =  1.16438356164384,
		.coeff[1] = -0.21324861427373,
		.coeff[2] = -0.532909328559444,
		.coeff[3] =  1.16438356164384,
		.coeff[4] =  2.11240178571429,
		.coeff[5] = -5.55111512312578e-17,
		.coeff[6] =  1.16438356164384,
		.coeff[7] =  0,
		.coeff[8] =  1.79274107142857,
	},
	/* REC 709, FULL RANGE */
	[1][1] = {
		.coeff[0] =  1,
		.coeff[1] = -0.187324272930649,
		.coeff[2] = -0.468124272930649,
		.coeff[3] =  1,
		.coeff[4] =  1.8556,
		.coeff[5] =  5.55111512312578e-17,
		.coeff[6] =  1,
		.coeff[7] =  0,
		.coeff[8] =  1.5748,
	},

	/* BT 2020, LIMITED RANGE */
	[2][0] = {
		.coeff[0] =  1.16438356164384,
		.coeff[1] = -0.187326104219343,
		.coeff[2] = -0.650424318505057,
		.coeff[3] =  1.16438356164384,
		.coeff[4] =  2.14177232142857,
		.coeff[5] =  0,
		.coeff[6] =  1.16438356164384,
		.coeff[7] = -5.94461236188718e-17,
		.coeff[8] =  1.67867410714286,
	},
	/* BT 2020, FULL RANGE */
	[2][1] = {
		.coeff[0] =  1,
		.coeff[1] = -0.164553126843658,
		.coeff[2] = -0.571353126843658,
		.coeff[3] =  1,
		.coeff[4] =  1.8814,
		.coeff[5] =  0,
		.coeff[6] =  1,
		.coeff[7] = -2.61096699816221e-17,
		.coeff[8] =  1.4746,
	}
};

/* Offsets for YCbCr->RGB conversion depend on COLOR SPACE and QUANTIZATION
 * Vectors are written in GBR format (not RGB)
 * because VINC uses GBR for internal processing.
 * Vector elements go in this order:
 * 0: G offset
 * 1: B offset
 * 2: R offset */
const struct vector ycbcr_to_rgb_v[3][2] = {
	/* BT 601, LIMITED RANGE */
	[0][0] = {
		.offset[0] =  2169.20471987651,
		.offset[1] = -4429.37362035225,
		.offset[2] = -3566.74504892368,
	},
	/* BT 601, FULL RANGE */
	[0][1] = {
		.offset[0] =  2167.34222827939,
		.offset[1] = -3629.056,
		.offset[2] = -2871.296,
	},

	/* REC 709, LIMITED RANGE */
	[1][0] = {
		.offset[0] =  1230.04927514152,
		.offset[1] = -4624.28104892368,
		.offset[2] = -3969.61590606654,
	},
	/* REC 709, FULL RANGE */
	[1][1] = {
		.offset[0] =  1342.35862192394,
		.offset[1] = -3800.2688,
		.offset[2] = -3225.1904,
	},

	/* BT 2020, LIMITED RANGE */
	[2][0] = {
		.offset[0] =  1417.63067395875,
		.offset[1] = -4684.43190606654,
		.offset[2] = -3736.00676320939,
	},
	/* BT 2020, FULL RANGE */
	[2][1] = {
		.offset[0] =  1507.13600755162,
		.offset[1] = -3853.1072,
		.offset[2] = -3019.9808,
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

void vinc_calculate_wb_matrix(u32 sum_r, u32 sum_g, u32 sum_b,
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
static void cc_matrix_calc(struct matrix *coeff, void *coeffs[], u8 vinc_enc,
				u8 vinc_qnt)
{
	struct matrix tmp1;
	struct matrix tmp2;

	struct matrix *wb = (struct matrix *)coeffs[0];

	/* V4L2_CID_DO_WHITE_BALANCE control matrix and RGB->YCbCr matrix
	 * multiplication */
	mxm_mult(&tmp2, &rgb_to_ycbcr_m[vinc_enc][vinc_qnt], wb);

	/* YCbCr->RGB matrix multiplication */
	mxm_mult(&tmp1, &ycbcr_to_rgb_m[vinc_enc][vinc_qnt], &tmp2);

	*coeff = tmp1;
}

/* Calculate CC offset vector according to control matrices and vectors */
static void cc_vector_calc(struct vector *offset, void *coeffs[], u8 vinc_enc,
				u8 vinc_qnt)
{
	struct vector tmp1;
	struct vector tmp2;

	struct vector *v_bri = (struct vector *)coeffs[1];

	/* RGB->YCbCr vector and V4L2_CID_BRIGHTNESS control vector addition */
	vxv_add(&tmp2, &rgb_to_ycbcr_v[vinc_enc][vinc_qnt], v_bri);

	/* YCbCr->RGB matrix multiplication */
	mxv_mult(&tmp1, &ycbcr_to_rgb_m[vinc_enc][vinc_qnt], &tmp2);

	/* YCbCr->RGB vector addition */
	vxv_add(&tmp2, &ycbcr_to_rgb_v[vinc_enc][vinc_qnt], &tmp1);

	*offset = tmp2;
}

/* Color Correction coefficient matrix, offset vector and scaling register
 * calculation routine */
void vinc_calculate_cc(void *coeffs[], enum v4l2_ycbcr_encoding ycbcr_enc,
		       enum v4l2_quantization quantization, struct vinc_cc *cc)
{
	struct matrix coeff;
	struct vector offset;
	u8 scaling;

	u16 i;
	u16 max_coeff = 0;
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
	default:   /* V4L2_YCBCR_ENC_601 */
		vinc_enc = 0;
		break;
	}

	switch (quantization) {
	case V4L2_QUANTIZATION_FULL_RANGE:
		vinc_qnt = 1;
		break;
	/* default quantization is set to limited,
	 * cause all current color spaces have limited range by default
	 * if you need to add more color spaces,
	 * check default quantization for them */
	default:
		vinc_qnt = 0;
		break;
	}

	cc_matrix_calc(&coeff, coeffs, vinc_enc, vinc_qnt);
	cc_vector_calc(&offset, coeffs, vinc_enc, vinc_qnt);
	/*  Scaling calculation */
	for (i = 0; i < VINC_CC_COEFF_COUNT; i++)
		if ((u16)fabs(coeff.coeff[i]) > max_coeff)
			max_coeff = (u16)fabs(coeff.coeff[i]);

	if (max_coeff == 0)
		scaling = 0;
	else
		scaling = ilog2(max_coeff) + 1;

	/* Vector, matrix and scaling write to CC cache */
	cc->scaling = scaling;
	for (i = 0; i < VINC_CC_COEFF_COUNT; i++)
		cc->coeff[i] = COEFF_FLOAT_TO_U16(coeff.coeff[i], scaling);
	for (i = 0; i < VINC_CC_OFFSET_COUNT; i++)
		cc->offset[i] = (u16)((s16)offset.offset[i]);
}
