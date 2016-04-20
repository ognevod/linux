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
#include <media/v4l2-ctrls.h>

#define COEFF_FLOAT_TO_U16(coeff, scaling) ((u16)((s16)((coeff) *  \
			(1 << (15 - (scaling))) + ((coeff) < 0 ? -0.5 : 0.5))))
#define CT_OFFSET_FLOAT_TO_U16(coeff) ((u16)((s16)((coeff) << 2) +  \
			((coeff) < 0 ? -0.5 : 0.5)))

enum vinc_ycbcr_encoding {
	VINC_YCBCR_ENC_601            = 0,
	VINC_YCBCR_ENC_709            = 1,
	VINC_YCBCR_ENC_BT2020         = 2,
	VINC_YCBCR_ENC_SYCC           = 3
};

enum vinc_quantization {
	VINC_QUANTIZATION_LIM_RANGE   = 0,
	VINC_QUANTIZATION_FULL_RANGE  = 1
};

struct matrix {
	double coeff[VINC_CC_COEFF_COUNT];
};

struct vector {
	double offset[VINC_CC_OFFSET_COUNT];
};

struct ctrl_priv {
	void *dowb;
	void *brightness;
	void *contrast;
	void *saturation;
	void *hue;
	void *ck;
};

void vinc_neon_calculate_v_bri(void *vector, s32 val);

void vinc_neon_calculate_m_con(void *matrix, s32 val);

void vinc_neon_calculate_m_sat(void *matrix, s32 val);

void vinc_neon_calculate_m_hue(void *matrix, s32 val);

void vinc_neon_wb_stat(u32 red, u32 green, u32 blue, s32 *cptr[]);

void vinc_neon_calculate_m_wb(u32 rb, u32 bb, void *matrix);

void vinc_neon_calculate_m_ck(void *matrix, s32 val);

void vinc_neon_calculate_gamma_curve(int value,
		struct vinc_gamma_curve *gamma_ptr);

/* Calculate CC matrix and offset */
void vinc_neon_calculate_cc(struct ctrl_priv *ctrl_privs,
		enum vinc_ycbcr_encoding ycbcr_enc, struct vinc_cc *cc);

#endif /* VINC_NEON_H */
