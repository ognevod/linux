/*
 * Copyright 2015 ELVEES NeoTek CJSC
 *
 * Based on V4L2 Driver for SuperH Mobile CEU
 * interface - "sh_mobile_ceu_camera.c"
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/time.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/videodev2.h>
#include <linux/vinc.h>
#include <linux/pm_runtime.h>
#include <linux/sched.h>

#include <media/v4l2-async.h>
#include <media/v4l2-common.h>
#include <media/v4l2-dev.h>
#include <media/soc_camera.h>
#include <media/videobuf2-dma-contig.h>
#include <media/v4l2-mediabus.h>
#include <media/soc_mediabus.h>

#include "soc_scale_crop.h"

#define MODULE_NAME "vinc"

/* VINC registers */
#define ID				0x0
#define AXI_MASTER_CFG			0x4
#define AXI_MASTER_STATUS		0x8
#define INTERRUPT			0x20
#define INTERRUPT_RESET			0x24
#define INTERRUPT_MASK			0x28

#define PPORT_BASE(p)			(0x80 * p + 0x100)
#define PPORT_CFG(p)			(PPORT_BASE(p) + 0x0)

#define PINTERFACE_BASE(p)		(0x100 * p + 0x300)
#define PINTERFACE_CFG(p)		(PINTERFACE_BASE(p) + 0x0)
#define PINTERFACE_CCMOV(p, c)		(PINTERFACE_BASE(p) + 0x20 + (c * 4))
#define PINTERFACE_HVFSYNC(p)		(PINTERFACE_BASE(p) + 0x80)

#define PPORT_INP_MUX_CFG		0x700
#define PPORT_STATUS			0x704
#define PPORT_TEST_SRC			0x720

#define CMOS_BASE(c)			(0x20 * c + 0x780)
#define CMOS_CTR(c)			(CMOS_BASE(c) + 0x0)
#define CMOS_TIMER_HIGH(c)		(CMOS_BASE(c) + 0x4)
#define CMOS_TIMER_LOW(c)		(CMOS_BASE(c) + 0x8)

#define CSI2_PORT_BASE(c)		(0x200 * c + 0x800)

#define CSI2_DEVICE_READY(c)		(CSI2_PORT_BASE(c) + 0x0)
#define CSI2_INTR(c)			(CSI2_PORT_BASE(c) + 0x4)
#define CSI2_INTR_EN(c)			(CSI2_PORT_BASE(c) + 0x8)
#define CSI2_FUNC_PROG(c)		(CSI2_PORT_BASE(c) + 0xc)
#define CSI2_DPHY_TIM3(c)		(CSI2_PORT_BASE(c) + 0x10)
#define CSI2_DPHY_TIM1(c)		(CSI2_PORT_BASE(c) + 0x14)
#define CSI2_DPHY_TIM2(c)		(CSI2_PORT_BASE(c) + 0x18)
#define CSI2_FSLS(c)			(CSI2_PORT_BASE(c) + 0x1c)
#define CSI2_LSDV(c)			(CSI2_PORT_BASE(c) + 0x20)
#define CSI2_DVLE(c)			(CSI2_PORT_BASE(c) + 0x24)
#define CSI2_LEFE(c)			(CSI2_PORT_BASE(c) + 0x28)
#define CSI2_FEFS(c)			(CSI2_PORT_BASE(c) + 0x2c)
#define CSI2_LELS(c)			(CSI2_PORT_BASE(c) + 0x30)
#define CSI2_TWO_PIX_EN(c)		(CSI2_PORT_BASE(c) + 0x34)
#define CSI2_TRIM0(c)			(CSI2_PORT_BASE(c) + 0x38)
#define CSI2_SYNC_COUNT(c)		(CSI2_PORT_BASE(c) + 0x3c)
#define CSI2_RCV_COUNT(c)		(CSI2_PORT_BASE(c) + 0x40)
#define CSI2_TRIM1(c)			(CSI2_PORT_BASE(c) + 0x44)
#define CSI2_VCH01(c)			(CSI2_PORT_BASE(c) + 0x48)
#define CSI2_VCH02(c)			(CSI2_PORT_BASE(c) + 0x4c)
#define CSI2_VCH11(c)			(CSI2_PORT_BASE(c) + 0x50)
#define CSI2_VCH12(c)			(CSI2_PORT_BASE(c) + 0x54)
#define CSI2_VCH21(c)			(CSI2_PORT_BASE(c) + 0x58)
#define CSI2_VCH22(c)			(CSI2_PORT_BASE(c) + 0x5c)
#define CSI2_VCH31(c)			(CSI2_PORT_BASE(c) + 0x60)
#define CSI2_VCH32(c)			(CSI2_PORT_BASE(c) + 0x64)
#define CSI2_RAW8(c)			(CSI2_PORT_BASE(c) + 0x68)
#define CSI2_TRIM2(c)			(CSI2_PORT_BASE(c) + 0x6c)
#define CSI2_TRIM3(c)			(CSI2_PORT_BASE(c) + 0x70)
#define CSI2_LOOP_BACK(c)		(CSI2_PORT_BASE(c) + 0x80)

#define CSI2_PORT_SYS_CTR(c)		(CSI2_PORT_BASE(c) + 0x180)
#define CSI2_PORT_SYS_STATUS(c)		(CSI2_PORT_BASE(c) + 0x184)
#define CSI2_PORT_SYS_LB_CTRL(c)	(CSI2_PORT_BASE(c) + 0x18c)
#define CSI2_PORT_GENFIFO_CTR(c)	(CSI2_PORT_BASE(c) + 0x1c0)
#define CSI2_PORT_GENFIFO_DATA(c)	(CSI2_PORT_BASE(c) + 0x1c4)
#define CSI2_PORT_GENFIFO_STATUS(c)	(CSI2_PORT_BASE(c) + 0x1c8)

#define STREAM_BASE(s)			(0x400 * s + 0x1000)
#define STREAM_INP_CFG(s)		(STREAM_BASE(s) + 0x0)
#define STREAM_INP_HCROP_CTR(s)		(STREAM_BASE(s) + 0x4)
#define STREAM_INP_VCROP_CTR(s)		(STREAM_BASE(s) + 0x8)
#define STREAM_INP_VCROP_ODD_CTR(s)	(STREAM_BASE(s) + 0xc)
#define STREAM_INP_DECIM_CTR(s)		(STREAM_BASE(s) + 0x10)
#define STREAM_INP_MIN_SPACE_CTR(s)	(STREAM_BASE(s) + 0x14)
#define STREAM_STATUS(s)		(STREAM_BASE(s) + 0x20)
#define STREAM_INTERRUPT(s)		(STREAM_BASE(s) + 0x40)
#define STREAM_INTERRUPT_RESET(s)	(STREAM_BASE(s) + 0x44)
#define STREAM_INTERRUPT_MASK(s)	(STREAM_BASE(s) + 0x48)
#define STREAM_PROC_CFG(s)		(STREAM_BASE(s) + 0x50)
#define STREAM_PROC_CTR(s)		(STREAM_BASE(s) + 0x58)
#define STREAM_PROC_CLEAR(s)		(STREAM_BASE(s) + 0x60)
#define STREAM_PROC_BP_MAP_CTR(s)	(STREAM_BASE(s) + 0x68)
#define STREAM_PROC_BP_MAP_DATA(s)	(STREAM_BASE(s) + 0x6c)
#define STREAM_PROC_BP_BAD_LINE(s, l)	(STREAM_BASE(s) + 0x70 + (l * 4))
#define STREAM_PROC_BP_BAD_COLUMN(s, c)	(STREAM_BASE(s) + 0x90 + (c * 4))
#define STREAM_PROC_DR_CTR(s)		(STREAM_BASE(s) + 0xb0)
#define STREAM_PROC_DR_DATA(s)		(STREAM_BASE(s) + 0xb4)
#define STREAM_PROC_DR_COUNT(s)		(STREAM_BASE(s) + 0xb8)
#define STREAM_PROC_CC_COEFF(s, c)	(STREAM_BASE(s) + 0xc0 + (c * 4))
#define STREAM_PROC_CC_OFFSET(s, o)	(STREAM_BASE(s) + 0xd4 + (o * 4))
#define STREAM_PROC_GC_CTR(s)		(STREAM_BASE(s) + 0xe0)
#define STREAM_PROC_GC_DATA(s)		(STREAM_BASE(s) + 0xe4)
#define STREAM_PROC_CT_COEFF(s, c)	(STREAM_BASE(s) + 0xf0 + (c * 4))
#define STREAM_PROC_CT_OFFSET(s, o)	(STREAM_BASE(s) + 0x104 + (o * 4))

#define STREAM_PROC_STAT_ZONE_BASE(s, z) (0xc * z + STREAM_BASE(s) + 0x110)
#define STREAM_PROC_STAT_ZONE_LT(s, z)	(STREAM_PROC_STAT_ZONE_BASE(s, z) + 0x0)
#define STREAM_PROC_STAT_ZONE_RB(s, z)	(STREAM_PROC_STAT_ZONE_BASE(s, z) + 0x4)

#define STREAM_PROC_STAT_CTR(s)		(STREAM_BASE(s) + 0x140)
#define STREAM_PROC_STAT_DATA(s)	(STREAM_BASE(s) + 0x144)
#define STREAM_PROC_STAT_MIN(s)		(STREAM_BASE(s) + 0x148)
#define STREAM_PROC_STAT_MAX(s)		(STREAM_BASE(s) + 0x14c)
#define STREAM_PROC_STAT_SUM_B(s)	(STREAM_BASE(s) + 0x150)
#define STREAM_PROC_STAT_SUM_G(s)	(STREAM_BASE(s) + 0x154)
#define STREAM_PROC_STAT_SUM_R(s)	(STREAM_BASE(s) + 0x158)
#define STREAM_PROC_STAT_SUM2_B(s)	(STREAM_BASE(s) + 0x15c)
#define STREAM_PROC_STAT_SUM2_G(s)	(STREAM_BASE(s) + 0x160)
#define STREAM_PROC_STAT_SUM2_R(s)	(STREAM_BASE(s) + 0x164)
#define STREAM_PROC_STAT_SUM2_HI(s)	(STREAM_BASE(s) + 0x168)
#define STREAM_PROC_STAT_TH(s)		(STREAM_BASE(s) + 0x16c)
#define STREAM_PROC_STAT_HSOBEL(s)	(STREAM_BASE(s) + 0x170)
#define STREAM_PROC_STAT_VSOBEL(s)	(STREAM_BASE(s) + 0x174)
#define STREAM_PROC_STAT_LSOBEL(s)	(STREAM_BASE(s) + 0x178)
#define STREAM_PROC_STAT_RSOBEL(s)	(STREAM_BASE(s) + 0x17c)

#define STREAM_DMA_BASE(s, d)		(0x100 * d + STREAM_BASE(s) + 0x200)
#define STREAM_DMA_FBUF_CFG(s, d)	(STREAM_DMA_BASE(s, d) + 0x0)
#define STREAM_DMA_PIXEL_FMT(s, d)	(STREAM_DMA_BASE(s, d) + 0x4)
#define STREAM_DMA_FBUF_HORIZ(s, d)	(STREAM_DMA_BASE(s, d) + 0x8)
#define STREAM_DMA_FBUF_VERT(s, d)	(STREAM_DMA_BASE(s, d) + 0xc)
#define STREAM_DMA_FBUF_VERT_ODD(s, d)	(STREAM_DMA_BASE(s, d) + 0x10)
#define STREAM_DMA_FBUF_DECIM(s, d)	(STREAM_DMA_BASE(s, d) + 0x14)

#define STREAM_DMA_FBUF_BASE(s, d, b)	(0x10 * b + STREAM_DMA_BASE(s, d)\
					+ 0x20)
#define STREAM_DMA_FBUF_LSTEP(s, d, b)	(STREAM_DMA_FBUF_BASE(s, d, b) + 0x4)
#define STREAM_DMA_FBUF_FSTEP(s, d, b)	(STREAM_DMA_FBUF_BASE(s, d, b) + 0x8)

#define STREAM_DMA_WR_CTR(s, d)		(STREAM_DMA_BASE(s, d) + 0x60)
#define STREAM_DMA_WR_STATUS(s, d)	(STREAM_DMA_BASE(s, d) + 0x64)
#define STREAM_DMA_WR_COUNT(s, d, c)	(STREAM_DMA_BASE(s, d) + 0x68 + (c * 4))
#define STREAM_DMA_CUR_ADDR(s, d, a)	(STREAM_DMA_BASE(s, d) + 0x70 + (a * 4))
#define STREAM_DMA_TEST_DATA(s, d)	(STREAM_DMA_BASE(s, d) + 0x80)
#define STREAM_DMA_TEST_CTR(s, d)	(STREAM_DMA_BASE(s, d) + 0x84)

#define STREAM_CTR			0x3f00

#define CC_CT_OFFSET_COEFF8		0x10
#define CC_CT_OFFSET_OFFSET0_1		0x14
#define CC_CT_OFFSET_OFFSET2		0x18

/* Bits for AXI_MASTER_CFG register */
#define AXI_MASTER_CFG_MAX_BURST(v)	(v & 0x7)
#define AXI_MASTER_CFG_MAX_WR_ID(v)	((v & 0xF) << 4)
#define AXI_MASTER_CFG_BUF_LAYOUT(v)	((v & 0xF) << 8)
#define AXI_MASTER_CFG_4K_BOUND_EN	BIT(19)
#define AXI_MASTER_CFG_GLOBAL_EN	BIT(31)

/* Bits for INTERRUPT register */
#define INTERRUPT_AXI_ERROR		BIT(0)
#define INTERRUPT_PPORT_ERROR		BIT(12)
#define INTERRUPT_CSI0			BIT(16)
#define INTERRUPT_CSI0_GEN		BIT(17)
#define INTERRUPT_CSI1			BIT(18)
#define INTERRUPT_CSI1_GEN		BIT(19)

/* Bits for PORT_CFG register */
#define PORT_CFG_PIXEL_MODE(v)		(v & 0x3)
#define PORT_CFG_PCLK_DIV		BIT(2)
#define PORT_CFG_PCLK_NEG_DIV		BIT(3)
#define PORT_CFG_PCLK_HALF_CLK		BIT(4)
#define PORT_CFG_DIGITAL_DELAY(v)	((v & 0x7) << 5)
#define PORT_CFG_VIN_SRC(v)		((v & 0x3) << 12)

/* Bits for PINTERFACE_CFG register */
#define PINTERFACE_CFG_CYCLE_NUM(v)	(v & 0x7)
#define PINTERFACE_CFG_PIXEL_NUM_EVEN(v)	((v & 0x3) << 4)
#define PINTERFACE_CFG_PIXEL_NUM_ODD(v)	((v & 0x3) << 8)
#define PINTERFACE_CFG_PV_ALGN_MODE(v)	((v & 0xF) << 12)
#define PINTERFACE_CFG_PORT_NUM_SYNC(v)	((v & 0x3) << 16)
#define PINTERFACE_CFG_EMB_SYNC		BIT(18)
#define PINTERFACE_CFG_EMB_SYNC_CORR	BIT(19)
#define PINTERFACE_CFG_PHASE_CORR(v)	((v & 0x7) << 20)
#define PINTERFACE_CFG_FORW_H(v)	((v & 0xFF) << 24)

/* Bits for PINTERFACE_HVFSYNCregister */
#define PINTERFACE_HVFSYNC_INVERS_H	BIT(0)
#define PINTERFACE_HVFSYNC_INVERS_V	BIT(1)
#define PINTERFACE_HVFSYNC_INVERS_F	BIT(2)
#define PINTERFACE_HVFSYNC_BUILT_MODE(v)	((v & 0x7) << 3)
#define PINTERFACE_HVFSYNC_DELAY_F_EN	BIT(6)
#define PINTERFACE_HVFSYNC_DELAY_V(v)	((v & 0x1F) << 8)
#define PINTERFACE_HVFSYNC_DELAY_F(v)	((v & 0x3F) << 16)
#define PINTERFACE_HVFSYNC_PRE_DELAY_V(v)	((v & 0x1F) << 24)
#define PINTERFACE_HVFSYNC_DELAY_VF_ODD_OFS(v)	((v & 0x7) << 29)



/* Bits for CMOS_CTR register */
#define CMOS_CTR_RESET			BIT(0)
#define CMOS_CTR_PCLK_EN		BIT(1)
#define CMOS_CTR_PCLK_SRC(v)		((v & 0x3) << 2)
#define CMOS_CTR_CLK_DIV(v)		((v & 0xF) << 4)
#define CMOS_CTR_FSYNC_EN		BIT(8)

/* Bits for CSI2_PORT_SYS_CTR register */
#define CSI2_PORT_SYS_CTR_ENABLE	BIT(0)
#define CSI2_PORT_SYS_CTR_TWO_PORTS	BIT(1)
#define CSI2_PORT_SYS_CTR_FREQ_RATIO(v)	((v & 0x3F) << 8)

/* Bits for CSI2_DPHY_TIM1 register */
#define CSI2_TIM1_DLN_CNT_HS_PREP(v)	(v & 0xFF)
#define CSI2_TIM1_DLN_CNT_HS_ZERO(v)	((v & 0xFF) << 8)
#define CSI2_TIM1_DLN_CNT_HS_TRAIL(v)	((v & 0xFF) << 16)
#define CSI2_TIM1_DLN_CNT_HS_EXIT(v)	((v & 0xFF) << 24)

/* Bits for CSI2_DPHY_TIM2 register */
#define CSI2_TIM2_CLN_CNT_HS_PREP(v)	(v & 0xFF)
#define CSI2_TIM2_CLN_CNT_HS_ZERO(v)	((v & 0xFF) << 8)
#define CSI2_TIM2_CLN_CNT_HS_TRAIL(v)	((v & 0xFF) << 16)
#define CSI2_TIM2_CLN_CNT_HS_EXIT(v)	((v & 0xFF) << 24)

/* Bits for CSI2_DPHY_TIM3 register */
#define CSI2_TIM3_CLN_CNT_LPX(v)	(v & 0xFF)
#define CSI2_TIM3_DLN_CNT_LPX(v)	((v & 0xFF) << 8)
#define CSI2_TIM3_CLN_CNT_PLL(v)	((v & 0xFFFF) << 16)

/* Bits for STREAM_INTERRUPT register */
#define STREAM_INTERRUPT_PROC		BIT(0)
#define STREAM_INTERRUPT_DMA0		BIT(8)
#define STREAM_INTERRUPT_DMA1		BIT(9)

/* Bits for STREAM_PROC_CFG register */
#define STREAM_PROC_CFG_BPC_EN		BIT(0)
#define STREAM_PROC_CFG_ADR_EN		BIT(1)
#define STREAM_PROC_CFG_CFA_EN		BIT(2)
#define STREAM_PROC_CFG_CC_EN		BIT(3)
#define STREAM_PROC_CFG_GC_EN		BIT(4)
#define STREAM_PROC_CFG_CT_EN		BIT(5)
#define STREAM_PROC_CFG_422TO444_EN	BIT(6)
#define STREAM_PROC_CFG_444TO422_EN	BIT(7)
#define STREAM_PROC_CFG_422TO420_EN	BIT(8)
#define STREAM_PROC_CFG_STT_EN(v)	((v & 0x7) << 9)
#define STREAM_PROC_CFG_STT_ZONE_OFFSET	12
#define STREAM_PROC_CFG_STT_ZONE_EN(v)	((v & 0xF) << 12)
#define STREAM_PROC_CFG_IM_EN		BIT(16)
#define STREAM_PROC_CFG_CT_SRC(v)	((v & 0x1) << 23)
#define STREAM_PROC_CFG_444TO422_SRC(v)	((v & 0x1) << 24)
#define STREAM_PROC_CFG_422TO420_SRC(v)	((v & 0x1) << 25)
#define STREAM_PROC_CFG_DMA0_SRC(v)	((v & 0x7) << 26)
#define STREAM_PROC_CFG_DMA1_SRC(v)	((v & 0x7) << 29)

#define STT_EN_HIST			0x1
#define STT_EN_AF			0x2
#define STT_EN_ADD			0x4

/* Bits for STREAM_PROC_CTR register */
#define STREAM_PROC_CTR_BAYER_MONO	BIT(0)
#define STREAM_PROC_CTR_BAYER_MODE(v)	((v & 0x3) << 1)
#define STREAM_PROC_CTR_422TO444ALG(v)	((v & 0x3) << 6)
#define STREAM_PROC_CTR_422TO444FILL	BIT(8)
#define STREAM_PROC_CTR_444TO422ALG(v)	((v & 0x3) << 9)
#define STREAM_PROC_CTR_444TO422FILL	BIT(11)
#define STREAM_PROC_CTR_422TO420ALG	BIT(12)
#define STREAM_PROC_CTR_HIST_THR	BIT(13)
#define STREAM_PROC_CTR_AF_THR		BIT(14)
#define STREAM_PROC_CTR_ADD_THR		BIT(15)
#define STREAM_PROC_CTR_AF_COLOR(v)	((v & 0x3) << 16)
#define STREAM_PROC_CTR_IM_COLOR(v)	((v & 0x3) << 21)

/* Bits for STREAM_PROC_CLEAR register */
#define STREAM_PROC_CLEAR_AF_CLR	BIT(0)
#define STREAM_PROC_CLEAR_ADD_CLR	BIT(1)
#define STREAM_PROC_CLEAR_THR_CLR	BIT(2)

/* Bits for STREAM_PROC_STAT_CTR register */
#define STREAM_PROC_STAT_CTR_ADDR_HIST(v)	(v & 0xFF)
#define STREAM_PROC_STAT_CTR_NUM_ZONE(v)	((v & 0x3) << 16)
#define STREAM_PROC_STAT_CTR_COLOR_HIST(v)	((v & 0x3) << 18)

/* Bits for STREAM_DMA_PIXEL_FMT register */
#define STREAM_DMA_PIXEL_FMT_PLANES(v)	((v) & 0xF)
#define STREAM_DMA_PIXEL_FMT_WIDTH(v)	(((v) & 0xF) << 4)
#define STREAM_DMA_PIXEL_FMT_FORMAT(v)	(((v) & 0xF) << 8)
#define STREAM_DMA_PIXEL_FMT_WITH_ALPHA	BIT(12)
#define STREAM_DMA_PIXEL_FMT_PACK_TYPE(v)	(((v) & 0x1) << 16)

#define PLANES_SINGLE			1
#define PLANES_DUAL			3
#define WIDTH_8				0
#define WIDTH_16			8
#define FORMAT_BGR			0x0
#define FORMAT_RGB			0x1
#define FORMAT_444			0x4
#define FORMAT_422			0x5
#define FORMAT_420			0x6
#define FORMAT_BAYER			0x8
#define FORMAT_MONO			0xC

/* Bits for STREAM_DMA_WR_CTR register */
#define DMA_WR_CTR_DMA_EN		BIT(0)
#define DMA_WR_CTR_LINE_INT_PERIOD(v)	((v & 0xFFF) << 1)
#define DMA_WR_CTR_FIELD_EN		BIT(13)
#define DMA_WR_CTR_FRAME_END_EN		BIT(14)
#define DMA_WR_CTR_FRAME_BEGIN_EN	BIT(15)
#define DMA_WR_CTR_TEST_MODE		BIT(16)

/* Bits for STREAM_DMA_WR_STATUS register */
#define DMA_WR_STATUS_LINE		BIT(0)
#define DMA_WR_STATUS_FIELD		BIT(1)
#define DMA_WR_STATUS_FRAME_END		BIT(2)
#define DMA_WR_STATUS_FRAME_BEGIN	BIT(3)
#define DMA_WR_STATUS_DMA_OVF		BIT(4)

/* Bits for STREAM_CTR register */
#define STREAM_CTR_STREAM0_ENABLE	BIT(0)
#define STREAM_CTR_STREAM1_ENABLE	BIT(1)
#define STREAM_CTR_DMA_CHANNELS_ENABLE	BIT(8)

#define CTRL_BAD_PIXELS_COUNT		4096
#define CTRL_BAD_ROWSCOLS_COUNT		16
#define CTRL_GC_ELEMENTS_COUNT		4096
#define CTRL_DR_ELEMENTS_COUNT		4096

#define MAX_WIDTH_HEIGHT		4095
#define MAX_COMP_VALUE			4095

#define COEFF_FLOAT_TO_U16(coeff, scaling) ((u16)((s16)((coeff) * \
		(1 << (15 - (scaling))) + ((coeff) < 0 ? -0.5 : 0.5))))

enum vinc_input_format { UNKNOWN, BAYER, RGB, YCbCr };

enum vinc_ctrls {
	CTRL_BP_EN,
	CTRL_BP_PIX,
	CTRL_BP_ROW,
	CTRL_BP_COL,
	CTRL_GC_EN,
	CTRL_GC,
	CTRL_CC_EN,
	CTRL_CC,
	CTRL_CT_EN,
	CTRL_CT,
	CTRL_DR_EN,
	CTRL_DR,
	CTRL_STAT_EN,
	CTRL_STAT_AF_COLOR,
	CTRL_STAT_AF_TH,
	CTRL_STAT_ZONE0,
	CTRL_STAT_ZONE1,
	CTRL_STAT_ZONE2,
	CTRL_STAT_ZONE3,
	CTRL_STAT_HIST0,
	CTRL_STAT_HIST1,
	CTRL_STAT_HIST2,
	CTRL_STAT_HIST3,
	CTRL_STAT_AF0,
	CTRL_STAT_AF1,
	CTRL_STAT_AF2,
	CTRL_STAT_AF3,
	CTRL_STAT_ADD0,
	CTRL_STAT_ADD1,
	CTRL_STAT_ADD2,
	CTRL_STAT_ADD3,
	CTRL_TEST_PATTERN,
	CTRLS_COUNT
};

struct vinc_dev {
	struct soc_camera_host ici;

	unsigned int irq_vio;
	unsigned int irq_s0;
	unsigned int irq_s1;
	void __iomem *base;
	struct tasklet_struct stat_tasklet;

	spinlock_t lock;		/* Protects video buffer lists */
	struct list_head capture;
	struct vb2_buffer *active;
	struct vb2_alloc_ctx *alloc_ctx;

	struct completion complete;

	enum v4l2_mbus_type video_source;
	int csi2_lanes;
	enum vinc_input_format input_format;
	u32 bayer_mode;

	struct v4l2_ctrl *ctrls[CTRLS_COUNT];
	int bp_change:1;
	int gc_change:1;
	int cc_change:1;
	int ct_change:1;
	int dr_change:1;
	int stat_odd:1;

	struct v4l2_crop crop1;
	struct v4l2_crop crop2;

	int sequence;
};

struct vinc_cam {
	/* Client output */
	unsigned int width;
	unsigned int height;
	u32 code;
};

struct vinc_ctrl_cfg {
	enum vinc_ctrls ctrl_id;
	struct v4l2_ctrl_config cfg;
};

static struct soc_mbus_pixelfmt vinc_formats[] = {
	{
		.name = "BGR 8+8+8+8",
		.fourcc = V4L2_PIX_FMT_BGR32,
		.packing = SOC_MBUS_PACKING_NONE,
		.order = SOC_MBUS_ORDER_LE,
		.layout = SOC_MBUS_LAYOUT_PACKED,
		.bits_per_sample = 32,
	},
	{
		.name = "YUV 4:2:0 2 lines Y, 1 line UV interleaved",
		.fourcc = V4L2_PIX_FMT_M420,
		.packing = SOC_MBUS_PACKING_NONE,
		.order = SOC_MBUS_ORDER_LE,
		.layout = SOC_MBUS_LAYOUT_PACKED,
		.bits_per_sample = 12,
	},
};

/* per video frame buffer */
struct vinc_buffer {
	struct vb2_buffer vb; /* v4l buffer must be first */
	struct list_head queue;
};

static void vinc_write(struct vinc_dev *priv,
		      unsigned long reg_offs, u32 data)
{
	iowrite32(data, priv->base + reg_offs);
}

static u32 vinc_read(struct vinc_dev *priv, unsigned long reg_offs)
{
	return ioread32(priv->base + reg_offs);
}

/*
 * .queue_setup() is called to check, whether the driver can accept the
 *		  requested number of buffers and to fill in plane sizes
 *		  for the current frame format if required
 */
static int vinc_queue_setup(struct vb2_queue *vq,
			    const struct v4l2_format *fmt,
			    unsigned int *count, unsigned int *num_planes,
			    unsigned int sizes[], void *alloc_ctxs[])
{
	struct soc_camera_device *icd = container_of(vq,
			struct soc_camera_device, vb2_vidq);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct vinc_dev *priv = ici->priv;

	if (fmt)
		return -EINVAL;

	dev_dbg(icd->parent, "Requested %u buffers\n", *count);

	sizes[0] = icd->sizeimage;
	dev_dbg(icd->parent, "%s: image_size=%d\n", __func__,
		icd->sizeimage);

	alloc_ctxs[0] = priv->alloc_ctx;

	*num_planes = 1;

	return 0;
}

static struct vinc_buffer *to_vinc_vb(struct vb2_buffer *vb)
{
	return container_of(vb, struct vinc_buffer, vb);
}

static int vinc_buf_prepare(struct vb2_buffer *vb)
{
	return 0;
}

static void vinc_start_capture(struct vinc_dev *priv)
{
	dma_addr_t phys_addr_top;
	u32 stream_ctr = vinc_read(priv, STREAM_CTR);

	stream_ctr &= ~STREAM_CTR_DMA_CHANNELS_ENABLE;
	vinc_write(priv, STREAM_CTR, stream_ctr);
	if (!priv->active)
		return;
	phys_addr_top = vb2_dma_contig_plane_dma_addr(priv->active, 0);
	switch (priv->ici.icd->current_fmt->host_fmt->fourcc) {
	case V4L2_PIX_FMT_BGR32:
		vinc_write(priv, STREAM_DMA_FBUF_BASE(0, 0, 0), phys_addr_top);
		break;
	case V4L2_PIX_FMT_M420:
		vinc_write(priv, STREAM_DMA_FBUF_BASE(0, 0, 0),
			   phys_addr_top + (priv->crop2.c.width << 1));
		vinc_write(priv, STREAM_DMA_FBUF_BASE(0, 0, 1), phys_addr_top);
		break;
	default:
		dev_err(priv->ici.v4l2_dev.dev, "Unknown output fourcc %#x",
			priv->ici.icd->current_fmt->host_fmt->fourcc);
		return;
	}
	stream_ctr |= STREAM_CTR_DMA_CHANNELS_ENABLE;
	vinc_write(priv, STREAM_CTR, stream_ctr);
}

static void vinc_buf_queue(struct vb2_buffer *vb)
{
	struct soc_camera_device *icd = container_of(vb->vb2_queue,
			struct soc_camera_device, vb2_vidq);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct vinc_dev *priv = ici->priv;
	struct vinc_buffer *buf = to_vinc_vb(vb);
	unsigned long size = icd->sizeimage;

	dev_dbg(icd->parent, "Add buffer #%u to queue\n",
		buf->vb.v4l2_buf.index);

	if (vb2_plane_size(vb, 0) < size) {
		dev_err(icd->parent, "Buffer #%u too small (%lu < %lu)\n",
		       vb->v4l2_buf.index, vb2_plane_size(vb, 0), size);
		vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
	}
	vb2_set_plane_payload(vb, 0, size);

	spin_lock_irq(&priv->lock);
	list_add_tail(&buf->queue, &priv->capture);
	if (!priv->active) {
		priv->active = vb;
		vinc_start_capture(priv);
	}
	spin_unlock_irq(&priv->lock);
}

static int vinc_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct soc_camera_device *icd = container_of(q,
			struct soc_camera_device, vb2_vidq);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct vinc_dev *priv = ici->priv;

	dev_dbg(icd->parent, "Start streaming (count: %u)\n", count);

	vinc_write(priv, CSI2_PORT_SYS_CTR(0), CSI2_PORT_SYS_CTR_FREQ_RATIO(2) |
			CSI2_PORT_SYS_CTR_ENABLE);
	vinc_write(priv, STREAM_DMA_WR_CTR(0, 0), DMA_WR_CTR_FRAME_END_EN |
			DMA_WR_CTR_DMA_EN);
	vinc_write(priv, STREAM_CTR, STREAM_CTR_STREAM0_ENABLE);

	priv->sequence = 0;
	spin_lock_irq(&priv->lock);
	if (priv->active)
		vinc_start_capture(priv);
	spin_unlock_irq(&priv->lock);

	return 0;
}

static void vinc_stop_streaming(struct vb2_queue *q)
{
	struct soc_camera_device *icd = container_of(q,
			struct soc_camera_device, vb2_vidq);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct vinc_dev *priv = ici->priv;
	struct vinc_buffer *buf, *tmp;
	u32 csi2_port_sys_ctr;

	dev_dbg(icd->parent, "Stop streaming\n");

	vinc_write(priv, STREAM_CTR, 0);
	vinc_write(priv, STREAM_DMA_WR_CTR(0, 0), 0x0);
	csi2_port_sys_ctr = vinc_read(priv, CSI2_PORT_SYS_CTR(0));
	vinc_write(priv, CSI2_PORT_SYS_CTR(0),
		   csi2_port_sys_ctr ^ STREAM_CTR_STREAM0_ENABLE);
	/* GLOBAL_ENABLE still enable for sensor clocks */

	spin_lock_irq(&priv->lock);

	priv->active = NULL;

	list_for_each_entry_safe(buf, tmp, &priv->capture, queue) {
		list_del_init(&buf->queue);
		if (buf->vb.state == VB2_BUF_STATE_ACTIVE)
			vb2_buffer_done(&buf->vb, VB2_BUF_STATE_ERROR);
	}

	spin_unlock_irq(&priv->lock);
}

static struct vb2_ops vinc_videobuf_ops = {
	.queue_setup		= vinc_queue_setup,
	.buf_prepare		= vinc_buf_prepare,
	.buf_queue		= vinc_buf_queue,
	.wait_prepare		= vb2_ops_wait_prepare,
	.wait_finish		= vb2_ops_wait_finish,
	.start_streaming	= vinc_start_streaming,
	.stop_streaming		= vinc_stop_streaming,
};

static void set_bad_pixels(struct vinc_dev *priv, struct vinc_bad_pixel *bp)
{
	int i;

	vinc_write(priv, STREAM_PROC_BP_MAP_CTR(0), 0);
	for (i = 0; i < CTRL_BAD_PIXELS_COUNT; i++)
		vinc_write(priv, STREAM_PROC_BP_MAP_DATA(0),
			   (bp[i].y << 12) | bp[i].x);
}

static void set_bad_rows_cols(struct vinc_dev *priv, u16 *data, int is_cols)
{
	int i;
	u32 reg_start = is_cols ? STREAM_PROC_BP_BAD_COLUMN(0, 0) :
			STREAM_PROC_BP_BAD_LINE(0, 0);

	for (i = 0; i < (CTRL_BAD_ROWSCOLS_COUNT / 2); i++)
		vinc_write(priv, reg_start + i * sizeof(u32),
			   (data[i * 2] & 0xFFF) |
			   ((data[i * 2 + 1] & 0xFFF) << 16));
}

static void set_gc_curve(struct vinc_dev *priv, struct vinc_gamma_curve *gc)
{
	int i;

	vinc_write(priv, STREAM_PROC_GC_CTR(0), 0);
	for (i = 0; i < CTRL_GC_ELEMENTS_COUNT; i++)
		vinc_write(priv, STREAM_PROC_GC_DATA(0),
			   (gc->green[i] << 16) | gc->red[i]);
	vinc_write(priv, STREAM_PROC_GC_CTR(0), 0x1000);
	for (i = 0; i < CTRL_GC_ELEMENTS_COUNT; i++)
		vinc_write(priv, STREAM_PROC_GC_DATA(0),
			   gc->blue[i]);
}

static void set_cc_ct(struct vinc_dev *priv, struct vinc_cc *cc, int is_ct)
{
	int i;
	u32 start_reg = is_ct ? STREAM_PROC_CT_COEFF(0, 0) :
			STREAM_PROC_CC_COEFF(0, 0);
	u32 val;

	for (i = 0; i < 4; i++) {
		val = cc->coeff[i * 2] | (cc->coeff[i * 2 + 1] << 16);
		vinc_write(priv, start_reg + i * 4, val);
	}
	vinc_write(priv, start_reg + CC_CT_OFFSET_COEFF8, cc->coeff[8]);
	val = cc->offset[0] | (cc->offset[1] << 16);
	vinc_write(priv, start_reg + CC_CT_OFFSET_OFFSET0_1, val);
	val = cc->offset[2] | (cc->scaling << 16);
	vinc_write(priv, start_reg + CC_CT_OFFSET_OFFSET2, val);
}

static void set_dr(struct vinc_dev *priv, u16 *dr)
{
	int i;

	vinc_write(priv, STREAM_PROC_DR_CTR(0), 0);
	for (i = 0; i < CTRL_DR_ELEMENTS_COUNT; i++)
		vinc_write(priv, STREAM_PROC_DR_DATA(0), dr[i]);
}

static void set_stat_af_color(struct vinc_dev *priv, u32 color)
{
	u32 proc_ctr = vinc_read(priv, STREAM_PROC_CTR(0));

	proc_ctr &= ~STREAM_PROC_CTR_AF_COLOR(0x3);
	proc_ctr |= STREAM_PROC_CTR_AF_COLOR(color);
	vinc_write(priv, STREAM_PROC_CTR(0), proc_ctr);
}

static void vinc_stat_start(struct vinc_dev *priv)
{
	priv->stat_odd = 0;
	vinc_write(priv, STREAM_PROC_CLEAR(0),
		   STREAM_PROC_CLEAR_THR_CLR);
}

static void set_stat_zone(struct vinc_dev *priv, u32 zone_id,
		     struct vinc_stat_zone *zone)
{
	u32 proc_cfg = vinc_read(priv, STREAM_PROC_CFG(0));

	if (zone->enable) {
		u32 lt = (zone->x_lt + priv->crop2.c.left) |
				((zone->y_lt + priv->crop2.c.top) << 16);
		u32 rb = (zone->x_rb + priv->crop2.c.left) |
				((zone->y_rb + priv->crop2.c.top) << 16);

		vinc_write(priv, STREAM_PROC_STAT_ZONE_LT(0, zone_id), lt);
		vinc_write(priv, STREAM_PROC_STAT_ZONE_RB(0, zone_id), rb);
		proc_cfg |= BIT(STREAM_PROC_CFG_STT_ZONE_OFFSET + zone_id);
	} else
		proc_cfg &= ~(BIT(STREAM_PROC_CFG_STT_ZONE_OFFSET + zone_id));
	vinc_write(priv, STREAM_PROC_CFG(0), proc_cfg);
}

static void vinc_configure_input(struct vinc_dev *priv)
{
	if (priv->ctrls[CTRL_TEST_PATTERN]->val) {
		u32 test_src = 0;

		test_src |= priv->crop1.c.width + priv->crop1.c.left;
		test_src |= (priv->crop1.c.height + priv->crop1.c.top) << 12;
		test_src |= 5 << 24;
		test_src |= (priv->ctrls[CTRL_TEST_PATTERN]->val - 1) << 29;

		vinc_write(priv, PPORT_INP_MUX_CFG, 0x101);
		vinc_write(priv, PPORT_CFG(0),
				PORT_CFG_PIXEL_MODE(1) | PORT_CFG_VIN_SRC(1));
		vinc_write(priv, PPORT_CFG(1),
				PORT_CFG_PIXEL_MODE(1) | PORT_CFG_VIN_SRC(2));
		vinc_write(priv, PPORT_CFG(2),
				PORT_CFG_PIXEL_MODE(1) | PORT_CFG_VIN_SRC(3));
		vinc_write(priv, PPORT_TEST_SRC, test_src);
		vinc_write(priv, PINTERFACE_CFG(0),
				PINTERFACE_CFG_CYCLE_NUM(1) |
				PINTERFACE_CFG_PIXEL_NUM_EVEN(1) |
				PINTERFACE_CFG_PORT_NUM_SYNC(2));
		vinc_write(priv, PINTERFACE_CCMOV(0, 0), 0x321);
		vinc_write(priv, PINTERFACE_HVFSYNC(0),
				PINTERFACE_HVFSYNC_DELAY_V(0x11) |
				PINTERFACE_HVFSYNC_PRE_DELAY_V(1));
		vinc_write(priv, STREAM_INP_CFG(0), 0x0);
	} else if (priv->video_source == V4L2_MBUS_CSI2) {
		vinc_write(priv, CSI2_INTR(0), 0x0007FFFF);
		vinc_write(priv, PPORT_INP_MUX_CFG, 0x0);
		vinc_write(priv, PPORT_CFG(0), 0x0);
		vinc_write(priv, PPORT_CFG(1), 0x0);
		vinc_write(priv, PPORT_CFG(2), 0x0);

		vinc_write(priv, CSI2_PORT_SYS_CTR(0),
			   CSI2_PORT_SYS_CTR_ENABLE |
			   CSI2_PORT_SYS_CTR_FREQ_RATIO(2));
		vinc_write(priv, CSI2_PORT_GENFIFO_CTR(0), 0x0);

		/* 1lane, timeout=max */
		vinc_write(priv, CSI2_FUNC_PROG(0), 0x1ffffc |
				((priv->csi2_lanes - 1) & 0x3));
		vinc_write(priv, CSI2_DPHY_TIM3(0),
			   CSI2_TIM3_CLN_CNT_LPX(0xBF) |
			   CSI2_TIM3_DLN_CNT_LPX(0x12));
		vinc_write(priv, CSI2_SYNC_COUNT(0), 0x14141414);
		vinc_write(priv, CSI2_RCV_COUNT(0), 0x04040404);

		vinc_write(priv, CSI2_FSLS(0), 0x2);
		vinc_write(priv, CSI2_LSDV(0), 0x2);
		vinc_write(priv, CSI2_DVLE(0), 0x2);
		vinc_write(priv, CSI2_LEFE(0), 0x2);
		vinc_write(priv, CSI2_FEFS(0), 0x2);
		vinc_write(priv, CSI2_LELS(0), 0x4);
		vinc_write(priv, CSI2_LOOP_BACK(0), 0x0);
		vinc_write(priv, CSI2_RAW8(0), 0x0);
		vinc_write(priv, CSI2_DPHY_TIM1(0),
			   CSI2_TIM1_DLN_CNT_HS_PREP(0x06) |
			   CSI2_TIM1_DLN_CNT_HS_ZERO(0x04) |
			   CSI2_TIM1_DLN_CNT_HS_TRAIL(0x07) |
			   CSI2_TIM1_DLN_CNT_HS_EXIT(0x02));
		vinc_write(priv, CSI2_DPHY_TIM2(0),
			   CSI2_TIM2_CLN_CNT_HS_PREP(0x06) |
			   CSI2_TIM2_CLN_CNT_HS_ZERO(0x04) |
			   CSI2_TIM2_CLN_CNT_HS_TRAIL(0x11) |
			   CSI2_TIM2_CLN_CNT_HS_EXIT(0x03));
		vinc_write(priv, CSI2_TRIM0(0), 0x02000000);
		vinc_write(priv, CSI2_DEVICE_READY(0), 0x1);
		vinc_write(priv, STREAM_INP_CFG(0), 0x2);
	} else
		dev_err(priv->ici.v4l2_dev.dev, "Unknown input format %#x",
			priv->video_source);
}

static int vinc_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct soc_camera_device *icd = container_of(ctrl->handler,
			struct soc_camera_device, ctrl_handler);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct vinc_dev *priv = ici->priv;
	u32 proc_cfg, stream_ctr;

	switch (ctrl->id) {
	case V4L2_CID_BAD_CORRECTION_ENABLE:
		/* For programming BP block we need to stop video */
		stream_ctr = vinc_read(priv, STREAM_CTR);
		vinc_write(priv, STREAM_CTR, 0);
		proc_cfg = vinc_read(priv, STREAM_PROC_CFG(0));
		if (ctrl->val)
			proc_cfg |= STREAM_PROC_CFG_BPC_EN;
		else
			proc_cfg &= ~STREAM_PROC_CFG_BPC_EN;
		vinc_write(priv, STREAM_PROC_CFG(0), proc_cfg);
		if (ctrl->val && priv->bp_change) {
			set_bad_pixels(priv, priv->ctrls[CTRL_BP_PIX]->p_cur.p);
			set_bad_rows_cols(priv,
					  priv->ctrls[CTRL_BP_ROW]->p_cur.p_u16,
					  0);
			set_bad_rows_cols(priv,
					  priv->ctrls[CTRL_BP_COL]->p_cur.p_u16,
					  1);
			priv->bp_change = 0;
		}
		vinc_write(priv, STREAM_CTR, stream_ctr);
		break;
	case V4L2_CID_BAD_PIXELS:
		if (priv->ctrls[CTRL_BP_EN]->cur.val) {
			stream_ctr = vinc_read(priv, STREAM_CTR);
			vinc_write(priv, STREAM_CTR, 0);
			set_bad_pixels(priv, ctrl->p_new.p);
			vinc_write(priv, STREAM_CTR, stream_ctr);
		} else
			priv->bp_change = 1;
		break;
	case V4L2_CID_BAD_ROWS:
	case V4L2_CID_BAD_COLS:
		if (priv->ctrls[CTRL_BP_EN]->cur.val) {
			stream_ctr = vinc_read(priv, STREAM_CTR);
			vinc_write(priv, STREAM_CTR, 0);
			set_bad_rows_cols(priv, ctrl->p_new.p_u16,
					  ctrl->id == V4L2_CID_BAD_ROWS ?
							  0 : 1);
			vinc_write(priv, STREAM_CTR, stream_ctr);
		} else
			priv->bp_change = 1;
		break;
	case V4L2_CID_GAMMA_CURVE_ENABLE:
		proc_cfg = vinc_read(priv, STREAM_PROC_CFG(0));
		if (ctrl->val)
			proc_cfg |= STREAM_PROC_CFG_GC_EN;
		else
			proc_cfg &= ~STREAM_PROC_CFG_GC_EN;
		vinc_write(priv, STREAM_PROC_CFG(0), proc_cfg);
		if (ctrl->val && priv->gc_change) {
			set_gc_curve(priv, priv->ctrls[CTRL_GC]->p_cur.p);
			priv->gc_change = 0;
		}
		break;
	case V4L2_CID_GAMMA_CURVE:
		if (priv->ctrls[CTRL_GC_EN]->cur.val)
			set_gc_curve(priv, ctrl->p_new.p);
		else
			priv->gc_change = 1;
		break;
	case V4L2_CID_CC_ENABLE:
		proc_cfg = vinc_read(priv, STREAM_PROC_CFG(0));
		if (ctrl->val)
			proc_cfg |= STREAM_PROC_CFG_CC_EN;
		else
			proc_cfg &= ~STREAM_PROC_CFG_CC_EN;
		vinc_write(priv, STREAM_PROC_CFG(0), proc_cfg);
		if (ctrl->val && priv->cc_change) {
			set_cc_ct(priv, priv->ctrls[CTRL_CC]->p_cur.p, 0);
			priv->cc_change = 0;
		}
		break;
	case V4L2_CID_CC:
		if (priv->ctrls[CTRL_CC_EN]->cur.val)
			set_cc_ct(priv, ctrl->p_new.p, 0);
		else
			priv->cc_change = 1;
		break;
	case V4L2_CID_CT_ENABLE:
		proc_cfg = vinc_read(priv, STREAM_PROC_CFG(0));
		if (ctrl->val)
			proc_cfg |= STREAM_PROC_CFG_CT_EN;
		else
			proc_cfg &= ~STREAM_PROC_CFG_CT_EN;
		vinc_write(priv, STREAM_PROC_CFG(0), proc_cfg);
		if (ctrl->val && priv->ct_change) {
			set_cc_ct(priv, priv->ctrls[CTRL_CT]->p_cur.p, 1);
			priv->ct_change = 0;
		}
		break;
	case V4L2_CID_CT:
		if (priv->ctrls[CTRL_CT_EN]->cur.val)
			set_cc_ct(priv, ctrl->p_new.p, 1);
		else
			priv->ct_change = 1;
		break;
	case V4L2_CID_DR_ENABLE:
		proc_cfg = vinc_read(priv, STREAM_PROC_CFG(0));
		if (ctrl->val)
			proc_cfg |= STREAM_PROC_CFG_ADR_EN;
		else
			proc_cfg &= ~STREAM_PROC_CFG_ADR_EN;
		vinc_write(priv, STREAM_PROC_CFG(0), proc_cfg);
		if (ctrl->val && priv->dr_change) {
			set_dr(priv, priv->ctrls[CTRL_DR]->p_cur.p_u16);
			priv->dr_change = 0;
		}
		break;
	case V4L2_CID_DR:
		if (priv->ctrls[CTRL_DR_EN]->cur.val)
			set_dr(priv, ctrl->p_new.p_u16);
		else
			priv->dr_change = 1;
		break;
	case V4L2_CID_STAT_ENABLE:
		stream_ctr = vinc_read(priv, STREAM_CTR);
		vinc_write(priv, STREAM_CTR, 0);
		proc_cfg = vinc_read(priv, STREAM_PROC_CFG(0));
		proc_cfg &= ~STREAM_PROC_CFG_STT_EN(0x7);
		proc_cfg |= STREAM_PROC_CFG_STT_EN(ctrl->val);
		if (ctrl->val)
			vinc_stat_start(priv);
		vinc_write(priv, STREAM_PROC_CFG(0), proc_cfg);
		vinc_write(priv, STREAM_CTR, stream_ctr);
		break;
	case V4L2_CID_STAT_AF_COLOR:
		set_stat_af_color(priv, ctrl->val);
		break;
	case V4L2_CID_STAT_AF_TH:
		vinc_write(priv, STREAM_PROC_STAT_TH(0), ctrl->val);
		break;
	case V4L2_CID_STAT_ZONE0:
	case V4L2_CID_STAT_ZONE1:
	case V4L2_CID_STAT_ZONE2:
	case V4L2_CID_STAT_ZONE3:
		stream_ctr = vinc_read(priv, STREAM_CTR);
		vinc_write(priv, STREAM_CTR, 0);
		set_stat_zone(priv, ctrl->id - V4L2_CID_STAT_ZONE0,
			      ctrl->p_new.p);
		vinc_write(priv, STREAM_CTR, stream_ctr);
		break;
	case V4L2_CID_TEST_PATTERN:
		stream_ctr = vinc_read(priv, STREAM_CTR);
		vinc_write(priv, STREAM_CTR, 0);
		proc_cfg = vinc_read(priv, STREAM_PROC_CFG(0));
		if (priv->input_format == BAYER && !ctrl->val)
			proc_cfg |= STREAM_PROC_CFG_CFA_EN;
		else
			proc_cfg &= ~STREAM_PROC_CFG_CFA_EN;
		vinc_configure_input(priv);
		vinc_write(priv, STREAM_PROC_CFG(0), proc_cfg);
		vinc_write(priv, STREAM_CTR, stream_ctr);
		break;
	default:
		return -EINVAL;
	}
	dev_dbg(priv->ici.v4l2_dev.dev, "%s: %#x (%s), is_ptr: %d, val: %d\n",
		__func__, ctrl->id, ctrl->name, ctrl->is_ptr, ctrl->val);

	return 0;
}

/* This function will be call only for controls
 * with V4L2_CTRL_FLAG_VOLATILE flag */
static int vinc_g_ctrl(struct v4l2_ctrl *ctrl)
{
	struct vinc_dev *priv = ctrl->priv;

	dev_dbg(priv->ici.v4l2_dev.dev, "%s: %#x (%s)\n",
		__func__, ctrl->id, ctrl->name);
	return 0;
}

static int vinc_try_ctrl(struct v4l2_ctrl *ctrl)
{
	struct vinc_bad_pixel *pixel;
	struct vinc_gamma_curve *gc;
	struct vinc_stat_zone *zone;
	int i;

	switch (ctrl->id) {
	case V4L2_CID_BAD_CORRECTION_ENABLE:
	case V4L2_CID_GAMMA_CURVE_ENABLE:
	case V4L2_CID_CC_ENABLE:
	case V4L2_CID_CT_ENABLE:
	case V4L2_CID_DR_ENABLE:
		return (u32)ctrl->val < 2 ? 0 : -ERANGE;
	case V4L2_CID_BAD_PIXELS:
		pixel = ctrl->p_new.p;
		for (i = 0; i < CTRL_BAD_PIXELS_COUNT; i++) {
			if (((pixel[i].x > MAX_WIDTH_HEIGHT) ||
			     (pixel[i].y > MAX_WIDTH_HEIGHT)) &&
			    (pixel[i].x != 0xFFFF) && (pixel[i].y != 0xFFFF))
				return -ERANGE;
		}
		return 0;
	case V4L2_CID_BAD_ROWS:
	case V4L2_CID_BAD_COLS:
		for (i = 0; i < CTRL_BAD_ROWSCOLS_COUNT; i++) {
			if (ctrl->p_new.p_u16[i] > MAX_WIDTH_HEIGHT &&
			    ctrl->p_new.p_u16[i] != 0xFFFF)
				return -ERANGE;
		}
		return 0;
	case V4L2_CID_GAMMA_CURVE:
		gc = ctrl->p_new.p;
		for (i = 0; i < CTRL_GC_ELEMENTS_COUNT; i++) {
			if (gc->red[i] > MAX_COMP_VALUE ||
			    gc->green[i] > MAX_COMP_VALUE ||
			    gc->blue[i] > MAX_COMP_VALUE)
				return -ERANGE;
		}
		return 0;
	case V4L2_CID_STAT_ZONE0:
		zone = ctrl->p_new.p;
		if (!zone->enable)
			return 0;
		if (zone->x_lt > MAX_WIDTH_HEIGHT ||
				zone->y_lt > MAX_WIDTH_HEIGHT ||
				zone->x_rb > MAX_WIDTH_HEIGHT ||
				zone->y_rb > MAX_WIDTH_HEIGHT ||
				zone->x_lt >= zone->x_rb ||
				zone->y_lt >= zone->y_rb)
			return -ERANGE;
		return 0;
	case V4L2_CID_CC:
	case V4L2_CID_CT:
	case V4L2_CID_DR:
	case V4L2_CID_STAT_ENABLE:
	case V4L2_CID_STAT_AF_COLOR:
	case V4L2_CID_STAT_AF_TH:
	case V4L2_CID_TEST_PATTERN:
		return 0;
	default:
		return -EINVAL;
	}
}

static struct v4l2_ctrl_ops ctrl_ops = {
	.g_volatile_ctrl = vinc_g_ctrl,
	.s_ctrl = vinc_s_ctrl,
	.try_ctrl = vinc_try_ctrl
};

static const char * const vinc_af_color_menu[] = {
	"Red/Cr",
	"Green/Y",
	"Blue/Cb",
};

static const char * const vinc_test_pattern_menu[] = {
	"Disabled",
	"Vertical bars",
	"Diagonal stripes",
	"Horizontal bars",
	"Increment",
};

static struct vinc_ctrl_cfg ctrl_cfg[] = {
	{
		.ctrl_id = CTRL_BP_EN,
		.cfg = {
			.ops = &ctrl_ops,
			.id = V4L2_CID_BAD_CORRECTION_ENABLE,
			.name = "Bad pixels/rows/columns repair enable",
			.type = V4L2_CTRL_TYPE_BOOLEAN,
			.min = 0,
			.max = 1,
			.step = 1,
			.def = 0,
			.flags = 0
		}
	},
	{
		.ctrl_id = CTRL_BP_PIX,
		.cfg = {
			.ops = &ctrl_ops,
			.id = V4L2_CID_BAD_PIXELS,
			.name = "Bad pixels",
			.type = V4L2_CTRL_COMPOUND_TYPES,
			.min = 0,
			.max = 0xFF,
			.step = 1,
			.def = 0,
			.dims[0] = sizeof(struct vinc_bad_pixel) *
					CTRL_BAD_PIXELS_COUNT,
			.flags = V4L2_CTRL_FLAG_HAS_PAYLOAD
		}
	},
	{
		.ctrl_id = CTRL_BP_ROW,
		.cfg = {
			.ops = &ctrl_ops,
			.id = V4L2_CID_BAD_ROWS,
			.name = "Bad rows",
			.type = V4L2_CTRL_TYPE_U16,
			.min = 0,
			.max = 0xFFF,
			.step = 1,
			.def = 0,
			.dims[0] = CTRL_BAD_ROWSCOLS_COUNT,
			.flags = V4L2_CTRL_FLAG_HAS_PAYLOAD
		}
	},
	{
		.ctrl_id = CTRL_BP_COL,
		.cfg = {
			.ops = &ctrl_ops,
			.id = V4L2_CID_BAD_COLS,
			.name = "Bad columns",
			.type = V4L2_CTRL_TYPE_U16,
			.min = 0,
			.max = 0xFFF,
			.step = 1,
			.def = 0,
			.dims[0] = CTRL_BAD_ROWSCOLS_COUNT,
			.flags = V4L2_CTRL_FLAG_HAS_PAYLOAD
		}
	},
	{
		.ctrl_id = CTRL_GC_EN,
		.cfg = {
			.ops = &ctrl_ops,
			.id = V4L2_CID_GAMMA_CURVE_ENABLE,
			.name = "Gamma curve enable",
			.type = V4L2_CTRL_TYPE_BOOLEAN,
			.min = 0,
			.max = 1,
			.step = 1,
			.def = 0,
			.flags = 0
		}
	},
	{
		.ctrl_id = CTRL_GC,
		.cfg = {
			.ops = &ctrl_ops,
			.id = V4L2_CID_GAMMA_CURVE,
			.name = "Gamma curve",
			.type = V4L2_CTRL_COMPOUND_TYPES,
			.min = 0,
			.max = 0xFF,
			.step = 1,
			.def = 0,
			.dims[0] = sizeof(struct vinc_gamma_curve),
			.flags = V4L2_CTRL_FLAG_HAS_PAYLOAD
		}
	},
	{
		.ctrl_id = CTRL_CC_EN,
		.cfg = {
			.ops = &ctrl_ops,
			.id = V4L2_CID_CC_ENABLE,
			.name = "Color correction enable",
			.type = V4L2_CTRL_TYPE_BOOLEAN,
			.min = 0,
			.max = 1,
			.step = 1,
			.def = 0,
			.flags = 0
		}
	},
	{
		.ctrl_id = CTRL_CC,
		.cfg = {
			.ops = &ctrl_ops,
			.id = V4L2_CID_CC,
			.name = "Color correction",
			.type = V4L2_CTRL_COMPOUND_TYPES,
			.min = 0,
			.max = 0xFF,
			.step = 1,
			.def = 0,
			.dims[0] = sizeof(struct vinc_cc),
			.flags = V4L2_CTRL_FLAG_HAS_PAYLOAD
		}
	},
	{
		.ctrl_id = CTRL_CT_EN,
		.cfg = {
			.ops = &ctrl_ops,
			.id = V4L2_CID_CT_ENABLE,
			.name = "Color tranformation enable",
			.type = V4L2_CTRL_TYPE_BOOLEAN,
			.min = 0,
			.max = 1,
			.step = 1,
			.def = 0,
			.flags = 0
		}
	},
	{
		.ctrl_id = CTRL_CT,
		.cfg = {
			.ops = &ctrl_ops,
			.id = V4L2_CID_CT,
			.name = "Color tranformation",
			.type = V4L2_CTRL_COMPOUND_TYPES,
			.min = 0,
			.max = 0xFF,
			.step = 1,
			.def = 0,
			.dims[0] = sizeof(struct vinc_cc),
			.flags = V4L2_CTRL_FLAG_HAS_PAYLOAD
		}
	},
	{
		.ctrl_id = CTRL_DR_EN,
		.cfg = {
			.ops = &ctrl_ops,
			.id = V4L2_CID_DR_ENABLE,
			.name = "Dynamic range enable",
			.type = V4L2_CTRL_TYPE_BOOLEAN,
			.min = 0,
			.max = 1,
			.step = 1,
			.def = 0,
			.flags = 0
		}
	},
	{
		.ctrl_id = CTRL_DR,
		.cfg = {
			.ops = &ctrl_ops,
			.id = V4L2_CID_DR,
			.name = "Dynamic range",
			.type = V4L2_CTRL_TYPE_U16,
			.min = 0,
			.max = 0xFFF,
			.step = 1,
			.def = 0,
			.dims[0] = CTRL_DR_ELEMENTS_COUNT,
			.flags = V4L2_CTRL_FLAG_HAS_PAYLOAD
		}
	},
	{
		.ctrl_id = CTRL_STAT_EN,
		.cfg = {
			.ops = &ctrl_ops,
			.id = V4L2_CID_STAT_ENABLE,
			.name = "Statistics enable",
			.type = V4L2_CTRL_TYPE_BITMASK,
			.min = 0,
			.max = 0x7,
			.step = 0,
			.def = 0,
			.flags = 0
		}
	},
	{
		.ctrl_id = CTRL_STAT_AF_COLOR,
		.cfg = {
			.ops = &ctrl_ops,
			.id = V4L2_CID_STAT_AF_COLOR,
			.name = "Autofocus component",
			.type = V4L2_CTRL_TYPE_MENU,
			.min = 0,
			.max = ARRAY_SIZE(vinc_af_color_menu) - 1,
			.step = 0,
			.def = 0,
			.qmenu = vinc_af_color_menu
		}
	},
	{
		.ctrl_id = CTRL_STAT_AF_TH,
		.cfg = {
			.ops = &ctrl_ops,
			.id = V4L2_CID_STAT_AF_TH,
			.name = "Autofocus threshold",
			.type = V4L2_CTRL_TYPE_INTEGER,
			.min = 0,
			.max = 0x7FF,
			.step = 1,
			.def = 0,
			.flags = 0
		}
	},
	{
		.ctrl_id = CTRL_STAT_ZONE0,
		.cfg = {
			.ops = &ctrl_ops,
			.id = V4L2_CID_STAT_ZONE0,
			.name = "Window of zone 0",
			.type = V4L2_CTRL_COMPOUND_TYPES,
			.min = 0,
			.max = 0xFF,
			.step = 1,
			.def = 0,
			.dims[0] = sizeof(struct vinc_stat_zone),
			.flags = V4L2_CTRL_FLAG_HAS_PAYLOAD
		}
	},
	{
		.ctrl_id = CTRL_STAT_ZONE1,
		.cfg = {
			.ops = &ctrl_ops,
			.id = V4L2_CID_STAT_ZONE1,
			.name = "Window of zone 1",
			.type = V4L2_CTRL_COMPOUND_TYPES,
			.min = 0,
			.max = 0xFF,
			.step = 1,
			.def = 0,
			.dims[0] = sizeof(struct vinc_stat_zone),
			.flags = V4L2_CTRL_FLAG_HAS_PAYLOAD
		}
	},
	{
		.ctrl_id = CTRL_STAT_ZONE2,
		.cfg = {
			.ops = &ctrl_ops,
			.id = V4L2_CID_STAT_ZONE2,
			.name = "Window of zone 2",
			.type = V4L2_CTRL_COMPOUND_TYPES,
			.min = 0,
			.max = 0xFF,
			.step = 1,
			.def = 0,
			.dims[0] = sizeof(struct vinc_stat_zone),
			.flags = V4L2_CTRL_FLAG_HAS_PAYLOAD
		}
	},
	{
		.ctrl_id = CTRL_STAT_ZONE3,
		.cfg = {
			.ops = &ctrl_ops,
			.id = V4L2_CID_STAT_ZONE3,
			.name = "Window of zone 3",
			.type = V4L2_CTRL_COMPOUND_TYPES,
			.min = 0,
			.max = 0xFF,
			.step = 1,
			.def = 0,
			.dims[0] = sizeof(struct vinc_stat_zone),
			.flags = V4L2_CTRL_FLAG_HAS_PAYLOAD
		}
	},
	{
		.ctrl_id = CTRL_STAT_HIST0,
		.cfg = {
			.ops = &ctrl_ops,
			.id = V4L2_CID_STAT_HIST0,
			.name = "Histogram in zone 0",
			.type = V4L2_CTRL_COMPOUND_TYPES,
			.min = 0,
			.max = 0xFF,
			.step = 1,
			.def = 0,
			.dims[0] = sizeof(struct vinc_stat_hist),
			.flags = V4L2_CTRL_FLAG_HAS_PAYLOAD |
				 V4L2_CTRL_FLAG_READ_ONLY
		}
	},
	{
		.ctrl_id = CTRL_STAT_HIST1,
		.cfg = {
			.ops = &ctrl_ops,
			.id = V4L2_CID_STAT_HIST1,
			.name = "Histogram in zone 1",
			.type = V4L2_CTRL_COMPOUND_TYPES,
			.min = 0,
			.max = 0xFF,
			.step = 1,
			.def = 0,
			.dims[0] = sizeof(struct vinc_stat_hist),
			.flags = V4L2_CTRL_FLAG_HAS_PAYLOAD |
				 V4L2_CTRL_FLAG_READ_ONLY
		}
	},
	{
		.ctrl_id = CTRL_STAT_HIST2,
		.cfg = {
			.ops = &ctrl_ops,
			.id = V4L2_CID_STAT_HIST2,
			.name = "Histogram in zone 2",
			.type = V4L2_CTRL_COMPOUND_TYPES,
			.min = 0,
			.max = 0xFF,
			.step = 1,
			.def = 0,
			.dims[0] = sizeof(struct vinc_stat_hist),
			.flags = V4L2_CTRL_FLAG_HAS_PAYLOAD |
				 V4L2_CTRL_FLAG_READ_ONLY
		}
	},
	{
		.ctrl_id = CTRL_STAT_HIST3,
		.cfg = {
			.ops = &ctrl_ops,
			.id = V4L2_CID_STAT_HIST3,
			.name = "Histogram in zone 3",
			.type = V4L2_CTRL_COMPOUND_TYPES,
			.min = 0,
			.max = 0xFF,
			.step = 1,
			.def = 0,
			.dims[0] = sizeof(struct vinc_stat_hist),
			.flags = V4L2_CTRL_FLAG_HAS_PAYLOAD |
				 V4L2_CTRL_FLAG_READ_ONLY
		}
	},
	{
		.ctrl_id = CTRL_STAT_AF0,
		.cfg = {
			.ops = &ctrl_ops,
			.id = V4L2_CID_STAT_AF0,
			.name = "Autofocus in zone 0",
			.type = V4L2_CTRL_COMPOUND_TYPES,
			.min = 0,
			.max = 0xFF,
			.step = 1,
			.def = 0,
			.dims[0] = sizeof(struct vinc_stat_af),
			.flags = V4L2_CTRL_FLAG_HAS_PAYLOAD |
				 V4L2_CTRL_FLAG_READ_ONLY
		}
	},
	{
		.ctrl_id = CTRL_STAT_AF1,
		.cfg = {
			.ops = &ctrl_ops,
			.id = V4L2_CID_STAT_AF1,
			.name = "Autofocus in zone 1",
			.type = V4L2_CTRL_COMPOUND_TYPES,
			.min = 0,
			.max = 0xFF,
			.step = 1,
			.def = 0,
			.dims[0] = sizeof(struct vinc_stat_af),
			.flags = V4L2_CTRL_FLAG_HAS_PAYLOAD |
				 V4L2_CTRL_FLAG_READ_ONLY
		}
	},
	{
		.ctrl_id = CTRL_STAT_AF2,
		.cfg = {
			.ops = &ctrl_ops,
			.id = V4L2_CID_STAT_AF2,
			.name = "Autofocus in zone 2",
			.type = V4L2_CTRL_COMPOUND_TYPES,
			.min = 0,
			.max = 0xFF,
			.step = 1,
			.def = 0,
			.dims[0] = sizeof(struct vinc_stat_af),
			.flags = V4L2_CTRL_FLAG_HAS_PAYLOAD |
				 V4L2_CTRL_FLAG_READ_ONLY
		}
	},
	{
		.ctrl_id = CTRL_STAT_AF3,
		.cfg = {
			.ops = &ctrl_ops,
			.id = V4L2_CID_STAT_AF3,
			.name = "Autofocus in zone 3",
			.type = V4L2_CTRL_COMPOUND_TYPES,
			.min = 0,
			.max = 0xFF,
			.step = 1,
			.def = 0,
			.dims[0] = sizeof(struct vinc_stat_af),
			.flags = V4L2_CTRL_FLAG_HAS_PAYLOAD |
				 V4L2_CTRL_FLAG_READ_ONLY
		}
	},
	{
		.ctrl_id = CTRL_STAT_ADD0,
		.cfg = {
			.ops = &ctrl_ops,
			.id = V4L2_CID_STAT_ADD0,
			.name = "Additional statistics in zone 0",
			.type = V4L2_CTRL_COMPOUND_TYPES,
			.min = 0,
			.max = 0xFF,
			.step = 1,
			.def = 0,
			.dims[0] = sizeof(struct vinc_stat_add),
			.flags = V4L2_CTRL_FLAG_HAS_PAYLOAD |
				 V4L2_CTRL_FLAG_READ_ONLY
		}
	},
	{
		.ctrl_id = CTRL_STAT_ADD1,
		.cfg = {
			.ops = &ctrl_ops,
			.id = V4L2_CID_STAT_ADD1,
			.name = "Additional statistics in zone 1",
			.type = V4L2_CTRL_COMPOUND_TYPES,
			.min = 0,
			.max = 0xFF,
			.step = 1,
			.def = 0,
			.dims[0] = sizeof(struct vinc_stat_add),
			.flags = V4L2_CTRL_FLAG_HAS_PAYLOAD |
				 V4L2_CTRL_FLAG_READ_ONLY
		}
	},
	{
		.ctrl_id = CTRL_STAT_ADD2,
		.cfg = {
			.ops = &ctrl_ops,
			.id = V4L2_CID_STAT_ADD2,
			.name = "Additional statistics in zone 2",
			.type = V4L2_CTRL_COMPOUND_TYPES,
			.min = 0,
			.max = 0xFF,
			.step = 1,
			.def = 0,
			.dims[0] = sizeof(struct vinc_stat_add),
			.flags = V4L2_CTRL_FLAG_HAS_PAYLOAD |
				 V4L2_CTRL_FLAG_READ_ONLY
		}
	},
	{
		.ctrl_id = CTRL_STAT_ADD3,
		.cfg = {
			.ops = &ctrl_ops,
			.id = V4L2_CID_STAT_ADD3,
			.name = "Additional statistics in zone 3",
			.type = V4L2_CTRL_COMPOUND_TYPES,
			.min = 0,
			.max = 0xFF,
			.step = 1,
			.def = 0,
			.dims[0] = sizeof(struct vinc_stat_add),
			.flags = V4L2_CTRL_FLAG_HAS_PAYLOAD |
				 V4L2_CTRL_FLAG_READ_ONLY
		}
	},
	{
		.ctrl_id = CTRL_TEST_PATTERN,
		.cfg = {
			.ops = &ctrl_ops,
			.id = V4L2_CID_TEST_PATTERN,
			.type = V4L2_CTRL_TYPE_MENU,
			.min = 0,
			.max = ARRAY_SIZE(vinc_test_pattern_menu) - 1,
			.step = 0,
			.def = 0,
			.qmenu = vinc_test_pattern_menu
		}
	},
};

static int vinc_create_controls(struct v4l2_ctrl_handler *hdl,
				struct vinc_dev *priv)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ctrl_cfg); i++) {
		priv->ctrls[ctrl_cfg[i].ctrl_id] = v4l2_ctrl_new_custom(hdl,
				&ctrl_cfg[i].cfg, NULL);
		if (!priv->ctrls[ctrl_cfg[i].ctrl_id]) {
			dev_err(priv->ici.v4l2_dev.dev,
				"Can not create control %#x\n",
				ctrl_cfg[i].cfg.id);
			return hdl->error;
		}
	}

	priv->bp_change = 1;
	priv->gc_change = 1;
	priv->cc_change = 1;
	priv->ct_change = 1;
	priv->dr_change = 1;

	memset(priv->ctrls[CTRL_BP_PIX]->p_cur.p, 0xFF,
	       sizeof(struct vinc_bad_pixel) * CTRL_BAD_PIXELS_COUNT);
	memset(priv->ctrls[CTRL_BP_PIX]->p_cur.p, 0xFF,
	       sizeof(u16) * CTRL_BAD_ROWSCOLS_COUNT);
	memset(priv->ctrls[CTRL_BP_PIX]->p_cur.p, 0xFF,
	       sizeof(u16) * CTRL_BAD_ROWSCOLS_COUNT);

	return hdl->error;
}

static int vinc_get_formats(struct soc_camera_device *icd, unsigned int idx,
			    struct soc_camera_format_xlate *xlate)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct device *dev = icd->parent;
	struct soc_camera_host *ici = to_soc_camera_host(dev);
	struct vinc_dev *priv = ici->priv;
	struct vinc_cam *cam;
	u32 code;
	int ret, i;
	int formats_count;

	ret = v4l2_subdev_call(sd, video, enum_mbus_fmt, idx, &code);
	if (ret < 0)
		return ret;

	switch (code) {
	case MEDIA_BUS_FMT_SBGGR8_1X8:
	case MEDIA_BUS_FMT_SGBRG8_1X8:
	case MEDIA_BUS_FMT_SGRBG8_1X8:
	case MEDIA_BUS_FMT_SRGGB8_1X8:
	case MEDIA_BUS_FMT_SBGGR10_1X10:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SRGGB10_1X10:
	case MEDIA_BUS_FMT_SBGGR12_1X12:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
	case MEDIA_BUS_FMT_SRGGB12_1X12:
		formats_count = ARRAY_SIZE(vinc_formats);
		if (xlate) {
			for (i = 0; i < formats_count; i++) {
				xlate->host_fmt = &vinc_formats[i];
				xlate->code = code;
				xlate++;
			}
		}
		break;
	default:
		formats_count = 0;
		break;
	}

	if (!icd->host_priv) {
		struct v4l2_mbus_config mbus_cfg;
		struct v4l2_mbus_framefmt mbus_fmt;

		ret = vinc_create_controls(&icd->ctrl_handler, priv);
		if (ret)
			return ret;

		ret = v4l2_subdev_call(sd, video, g_mbus_config, &mbus_cfg);
		if (ret >= 0) {
			priv->video_source = mbus_cfg.type;

			if (mbus_cfg.type != V4L2_MBUS_CSI2) {
				dev_err(dev,
					"Interface type %d is not supported\n",
					mbus_cfg.type);
				return -EINVAL;
			}

			if (mbus_cfg.flags & V4L2_MBUS_CSI2_4_LANE)
				priv->csi2_lanes = 4;
			else if (mbus_cfg.flags & V4L2_MBUS_CSI2_3_LANE)
				priv->csi2_lanes = 3;
			else if (mbus_cfg.flags & V4L2_MBUS_CSI2_2_LANE)
				priv->csi2_lanes = 2;
			else if (mbus_cfg.flags & V4L2_MBUS_CSI2_1_LANE)
				priv->csi2_lanes = 1;
		} else {
			dev_err(dev, "Failed to get mbus config from sensor\n");
			return ret;
		}

		cam = kzalloc(sizeof(*cam), GFP_KERNEL);
		if (!cam)
			return -ENOMEM;

		ret = v4l2_subdev_call(sd, video, g_mbus_fmt, &mbus_fmt);
		if (ret < 0)
			return ret;
		cam->width = mbus_fmt.width;
		cam->height = mbus_fmt.height;
		cam->code = mbus_fmt.code;

		icd->host_priv = cam;
	}

	return formats_count;
}

static struct soc_mbus_pixelfmt *vinc_get_mbus_pixelfmt(u32 fourcc)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(vinc_formats); i++) {
		if (fourcc == vinc_formats[i].fourcc)
			return &vinc_formats[i];
	}
	return NULL;
}

static int __vinc_try_fmt(struct soc_camera_device *icd, struct v4l2_format *f,
			  struct v4l2_mbus_framefmt *mbus_fmt)
{
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct soc_mbus_pixelfmt *pixelfmt;
	const struct soc_camera_format_xlate *xlate;
	u32 width, height;
	int ret;

	pixelfmt = vinc_get_mbus_pixelfmt(pix->pixelformat);
	if (!pixelfmt)
		return -EINVAL;

	xlate = soc_camera_xlate_by_fourcc(icd, pix->pixelformat);
	if (!xlate) {
		dev_warn(icd->parent, "Format %#x not found\n",
			 pix->pixelformat);
		return -EINVAL;
	}

	v4l_bound_align_image(&pix->width, 16, MAX_WIDTH_HEIGHT, 3,
			      &pix->height, 2, MAX_WIDTH_HEIGHT, 1, 0);

	width = pix->width;
	height = pix->height;
	mbus_fmt->code = xlate->code;
	mbus_fmt->colorspace = pix->colorspace;
	mbus_fmt->width = pix->width;
	mbus_fmt->height = pix->height;
	mbus_fmt->field = pix->field;
	mbus_fmt->ycbcr_enc = pix->ycbcr_enc;
	mbus_fmt->quantization = pix->quantization;
	ret = v4l2_subdev_call(sd, video, try_mbus_fmt, mbus_fmt);
	if (ret) {
		dev_warn(icd->parent, "Sensor can not negotiate format\n");
		return ret;
	}

	if (mbus_fmt->width != width || mbus_fmt->height != height) {
		/* If userspace request HD resolution then we will use
		 * this resolution, even if sensor does not support it */
		pix->width = min_t(u32, min_t(u32, 1280, mbus_fmt->width),
				   MAX_WIDTH_HEIGHT);
		pix->height = min_t(u32, min_t(u32, 720, mbus_fmt->height),
				    MAX_WIDTH_HEIGHT);
	} else {
		pix->width = mbus_fmt->width;
		pix->height = mbus_fmt->height;
	}
	dev_dbg(icd->parent,
		"%s : result resolution: %dx%d, pixelformat: %#x (%s)\n",
		__func__, pix->width, pix->height, pix->pixelformat,
		pixelfmt->name);

	return 0;
}

static int vinc_try_fmt(struct soc_camera_device *icd, struct v4l2_format *f)
{
	struct v4l2_mbus_framefmt mbus_fmt;

	return __vinc_try_fmt(icd, f, &mbus_fmt);
}

static int vinc_set_fmt(struct soc_camera_device *icd, struct v4l2_format *f)
{
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct vinc_dev *priv = ici->priv;
	struct v4l2_subdev *sd = soc_camera_to_subdev(priv->ici.icd);
	struct v4l2_mbus_framefmt mbus_fmt;
	const struct soc_camera_format_xlate *xlate;
	int ret;
	int offset_x, offset_y;

	ret = __vinc_try_fmt(icd, f, &mbus_fmt);
	if (ret)
		return ret;

	switch (mbus_fmt.code) {
	case MEDIA_BUS_FMT_SRGGB8_1X8:
	case MEDIA_BUS_FMT_SRGGB10_1X10:
	case MEDIA_BUS_FMT_SRGGB12_1X12:
		priv->input_format = BAYER;
		priv->bayer_mode = 0;
		break;
	case MEDIA_BUS_FMT_SGRBG8_1X8:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
		priv->input_format = BAYER;
		priv->bayer_mode = 1;
		break;
	case MEDIA_BUS_FMT_SGBRG8_1X8:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
		priv->input_format = BAYER;
		priv->bayer_mode = 2;
		break;
	case MEDIA_BUS_FMT_SBGGR8_1X8:
	case MEDIA_BUS_FMT_SBGGR10_1X10:
	case MEDIA_BUS_FMT_SBGGR12_1X12:
		priv->input_format = BAYER;
		priv->bayer_mode = 3;
		break;
	default:
		priv->input_format = UNKNOWN;
		dev_warn(icd->parent,
			 "Sensor reported invalid media bus format %#x\n",
			 mbus_fmt.code);
		return -EINVAL;
	}

	offset_x = (mbus_fmt.width - pix->width) / 2;
	offset_y = (mbus_fmt.height - pix->height) / 2;

	ret = v4l2_subdev_call(sd, video, s_mbus_fmt, &mbus_fmt);
	if (ret) {
		dev_warn(icd->parent, "Sensor can't set format %#x, %dx%d\n",
			 mbus_fmt.code, mbus_fmt.width, mbus_fmt.height);
		return ret;
	}

	xlate = soc_camera_xlate_by_fourcc(icd, pix->pixelformat);
	if (!xlate) {
		dev_warn(icd->parent, "Format %#x not found\n",
			 pix->pixelformat);
		return -EINVAL;
	}
	icd->current_fmt = xlate;

	priv->crop1.c.width = pix->width + offset_x;
	priv->crop1.c.height = pix->height + offset_y;
	priv->crop1.c.left = 0;
	priv->crop1.c.top = 0;

	priv->crop2.c.width = pix->width;
	priv->crop2.c.height = pix->height;
	priv->crop2.c.left = offset_x;
	priv->crop2.c.top = offset_y;

	dev_dbg(icd->parent, "crop1: %dx%d (%d,%d)\n",
		priv->crop1.c.width, priv->crop1.c.height,
		priv->crop1.c.left, priv->crop1.c.top);
	dev_dbg(icd->parent, "crop2: %dx%d (%d,%d)\n",
		priv->crop2.c.width, priv->crop2.c.height,
		priv->crop2.c.left, priv->crop2.c.top);

	return 0;
}

static unsigned int vinc_poll(struct file *file, poll_table *pt)
{
	struct soc_camera_device *icd = file->private_data;

	return vb2_poll(&icd->vb2_vidq, file, pt);
}

static int vinc_querycap(struct soc_camera_host *ici,
			 struct v4l2_capability *cap)
{
	/* cap->driver is filled in soc_camera_querycap() using ici->drv_name
	 * cap->version is filled in v4l_querycap() using LINUX_VERSION_CODE
	 */
	strlcpy(cap->card, "VINC", sizeof(cap->card));
	cap->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
	return 0;
}

static void vinc_configure_bgr(struct vinc_dev *priv)
{
	u32 proc_cfg, lstep, fstep;

	proc_cfg = vinc_read(priv, STREAM_PROC_CFG(0));
	proc_cfg |= STREAM_PROC_CFG_DMA0_SRC(2);
	vinc_write(priv, STREAM_PROC_CFG(0), proc_cfg);

	vinc_write(priv, STREAM_DMA_FBUF_CFG(0, 0), 0x00001);
	lstep = priv->crop2.c.width * 4;
	fstep = lstep * priv->crop2.c.height;
	vinc_write(priv, STREAM_DMA_FBUF_LSTEP(0, 0, 0), lstep);
	vinc_write(priv, STREAM_DMA_FBUF_FSTEP(0, 0, 0), fstep);
	vinc_write(priv, STREAM_DMA_PIXEL_FMT(0, 0),
		   STREAM_DMA_PIXEL_FMT_PLANES(PLANES_SINGLE) |
		   STREAM_DMA_PIXEL_FMT_FORMAT(FORMAT_BGR));
}

static void vinc_configure_m420(struct vinc_dev *priv)
{
	u32 proc_cfg, lstep, fstep;
	struct vinc_cc *ct = priv->ctrls[CTRL_CT]->p_cur.p;
	int i;

	ct->scaling = 0;
	ct->coeff[0] = COEFF_FLOAT_TO_U16(0.587005, 0);
	ct->coeff[1] = COEFF_FLOAT_TO_U16(0.113983, 0);
	ct->coeff[2] = COEFF_FLOAT_TO_U16(0.299011, 0);
	ct->coeff[3] = COEFF_FLOAT_TO_U16(-0.338836, 0);
	ct->coeff[4] = COEFF_FLOAT_TO_U16(0.511413, 0);
	ct->coeff[5] = COEFF_FLOAT_TO_U16(-0.172576, 0);
	ct->coeff[6] = COEFF_FLOAT_TO_U16(-0.428253, 0);
	ct->coeff[7] = COEFF_FLOAT_TO_U16(-0.083160, 0);
	ct->coeff[8] = COEFF_FLOAT_TO_U16(0.511413, 0);
	ct->offset[0] = 0x0;
	ct->offset[1] = 0x2000;
	ct->offset[2] = 0x2000;
	set_cc_ct(priv, ct, 1);
	priv->ctrls[CTRL_CT_EN]->cur.val = 1;

	proc_cfg = vinc_read(priv, STREAM_PROC_CFG(0));
	proc_cfg |= STREAM_PROC_CFG_CT_EN |
			STREAM_PROC_CFG_444TO422_EN |
			STREAM_PROC_CFG_422TO420_EN |
			STREAM_PROC_CFG_444TO422_SRC(1) |
			STREAM_PROC_CFG_422TO420_SRC(1) |
			STREAM_PROC_CFG_DMA0_SRC(5);
	vinc_write(priv, STREAM_PROC_CFG(0), proc_cfg);

	vinc_write(priv, STREAM_DMA_FBUF_CFG(0, 0), 0x00001);
	lstep = priv->crop2.c.width;
	fstep = lstep * priv->crop2.c.height;
	for (i = 0; i < 2; i++) {
		vinc_write(priv, STREAM_DMA_FBUF_LSTEP(0, 0, i),
			   lstep | BIT(31));
		vinc_write(priv, STREAM_DMA_FBUF_FSTEP(0, 0, i), fstep);
	}
	vinc_write(priv, STREAM_DMA_PIXEL_FMT(0, 0),
		   STREAM_DMA_PIXEL_FMT_PLANES(PLANES_DUAL) |
		   STREAM_DMA_PIXEL_FMT_FORMAT(FORMAT_420));
}

static void vinc_configure(struct vinc_dev *priv)
{
	u32 axi_master_cfg, proc_cfg, proc_ctr;
	int i;

	vinc_write(priv, STREAM_CTR, 0);

	vinc_configure_input(priv);

	vinc_write(priv, STREAM_INP_HCROP_CTR(0),
		   (priv->crop1.c.width << 16) | priv->crop1.c.left);
	vinc_write(priv, STREAM_INP_VCROP_CTR(0),
		   (priv->crop1.c.height << 16) | priv->crop1.c.top);
	vinc_write(priv, STREAM_INP_VCROP_ODD_CTR(0), 0);
	vinc_write(priv, STREAM_INP_DECIM_CTR(0), 0);

	proc_cfg = STREAM_PROC_CFG_STT_EN(priv->ctrls[CTRL_STAT_EN]->val);
	if (priv->input_format == BAYER && !priv->ctrls[CTRL_TEST_PATTERN]->val)
		proc_cfg |= STREAM_PROC_CFG_CFA_EN;

	if (priv->ctrls[CTRL_BP_EN]->cur.val)
		proc_cfg |= STREAM_PROC_CFG_BPC_EN;
	if (priv->ctrls[CTRL_GC_EN]->cur.val)
		proc_cfg |= STREAM_PROC_CFG_GC_EN;
	if (priv->ctrls[CTRL_CC_EN]->cur.val)
		proc_cfg |= STREAM_PROC_CFG_CC_EN;
	if (priv->ctrls[CTRL_CT_EN]->cur.val)
		proc_cfg |= STREAM_PROC_CFG_CT_EN;
	if (priv->ctrls[CTRL_DR_EN]->cur.val)
		proc_cfg |= STREAM_PROC_CFG_ADR_EN;

	for (i = 0; i < 4; i++) {
		struct vinc_stat_zone *zone =
				priv->ctrls[CTRL_STAT_ZONE0 + i]->p_cur.p;

		if (zone->enable)
			proc_cfg |= BIT(STREAM_PROC_CFG_STT_ZONE_OFFSET + i);
	}

	vinc_write(priv, STREAM_PROC_CFG(0), proc_cfg);
	proc_ctr = STREAM_PROC_CTR_BAYER_MODE(priv->bayer_mode);
	proc_ctr |= STREAM_PROC_CTR_AF_COLOR(
			priv->ctrls[CTRL_STAT_AF_COLOR]->val);
	proc_ctr |= STREAM_PROC_CTR_HIST_THR |
			STREAM_PROC_CTR_AF_THR |
			STREAM_PROC_CTR_ADD_THR;
	vinc_write(priv, STREAM_PROC_CTR(0), proc_ctr);
	if (priv->ctrls[CTRL_STAT_EN]->val)
		vinc_stat_start(priv);

	vinc_write(priv, STREAM_DMA_FBUF_HORIZ(0, 0),
		   (priv->crop2.c.width << 16) | priv->crop2.c.left);
	vinc_write(priv, STREAM_DMA_FBUF_VERT(0, 0),
		   (priv->crop2.c.height << 16) | priv->crop2.c.top);
	vinc_write(priv, STREAM_DMA_FBUF_DECIM(0, 0), 0x10000);

	switch (priv->ici.icd->current_fmt->host_fmt->fourcc) {
	case V4L2_PIX_FMT_BGR32:
		vinc_configure_bgr(priv);
		break;
	case V4L2_PIX_FMT_M420:
		vinc_configure_m420(priv);
		break;
	default:
		dev_warn(priv->ici.v4l2_dev.dev, "Unknown output format %#x\n",
			 priv->ici.icd->current_fmt->host_fmt->fourcc);
		break;
	}

	axi_master_cfg = vinc_read(priv, AXI_MASTER_CFG);
	vinc_write(priv, AXI_MASTER_CFG,
		   axi_master_cfg | AXI_MASTER_CFG_GLOBAL_EN);
}

static int vinc_set_bus_param(struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct vinc_dev *priv = ici->priv;

	vinc_configure(priv);

	return 0;
}

static int vinc_init_videobuf(struct vb2_queue *q,
			      struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);

	dev_dbg(icd->parent, "%s\n", __func__);

	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP;
	q->drv_priv = icd;
	q->ops = &vinc_videobuf_ops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->buf_struct_size = sizeof(struct vinc_buffer);
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->lock = &ici->host_lock;

	return vb2_queue_init(q);
}

static int vinc_get_parm(struct soc_camera_device *icd,
			 struct v4l2_streamparm *parm)
{
	if (parm->type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		parm->parm.capture.capability = 0;
		parm->parm.capture.capturemode = 0;
		parm->parm.capture.extendedmode = 0;
	}
	return 0;
}

static struct soc_camera_host_ops vinc_host_ops = {
	.owner		= THIS_MODULE,
	.get_formats	= vinc_get_formats,
	.set_fmt	= vinc_set_fmt,
	.try_fmt	= vinc_try_fmt,
	.poll		= vinc_poll,
	.querycap	= vinc_querycap,
	.set_bus_param	= vinc_set_bus_param,
	.init_videobuf2	= vinc_init_videobuf,
	.get_parm       = vinc_get_parm,
};


static irqreturn_t vinc_irq_vio(int irq, void *data)
{
	struct vinc_dev *priv = data;
	u32 int_status = vinc_read(priv, INTERRUPT);

	dev_dbg(priv->ici.v4l2_dev.dev, "Interrupt vio: 0x%x\n", int_status);
	if (int_status & INTERRUPT_PPORT_ERROR) {
		u32 int_pport = vinc_read(priv, PPORT_STATUS);

		dev_warn(priv->ici.v4l2_dev.dev,
			 "Pport interrupt. status: 0x%x\n", int_pport);
	}
	vinc_write(priv, INTERRUPT_RESET, int_status);

	return IRQ_HANDLED;
}

static void vinc_next_buffer(struct vinc_dev *priv, enum vb2_buffer_state state)
{
	struct vb2_buffer *vb = priv->active;

	if (!vb)
		return;
	spin_lock(&priv->lock);
	list_del_init(&to_vinc_vb(vb)->queue);

	if (!list_empty(&priv->capture))
		priv->active = &list_entry(priv->capture.next,
					   struct vinc_buffer, queue)->vb;
	else
		priv->active = NULL;

	vinc_start_capture(priv);
	v4l2_get_timestamp(&vb->v4l2_buf.timestamp);

	vb->v4l2_buf.sequence = priv->sequence++;

	vb2_buffer_done(vb, state);
	spin_unlock(&priv->lock);
}

static void vinc_stat_tasklet(unsigned long data)
{
	struct vinc_dev *priv = (struct vinc_dev *)data;
	struct vinc_stat_zone *zone;
	struct vinc_stat_hist *hist;
	struct vinc_stat_af *af;
	struct vinc_stat_add *add;
	u32 stat_en;
	u32 reg;
	int z, i;

	stat_en = priv->ctrls[CTRL_STAT_EN]->val;
	if (!stat_en)
		return;

	for (z = 0; z < 4; z++) {
		zone = priv->ctrls[CTRL_STAT_ZONE0 + z]->p_cur.p;
		if (!zone->enable)
			continue;

		hist = priv->ctrls[CTRL_STAT_HIST0 + z]->p_cur.p;
		if (stat_en & STT_EN_HIST) {
			__u32 *component[3] = { hist->red, hist->green,
						hist->blue };
			int c;

			for (c = 0; c < ARRAY_SIZE(component); c++) {
				vinc_write(priv, STREAM_PROC_STAT_CTR(0),
					   STREAM_PROC_STAT_CTR_NUM_ZONE(z) |
					   STREAM_PROC_STAT_CTR_COLOR_HIST(c));
				for (i = 0; i < VINC_STAT_HIST_COUNT; i++)
					component[c][i] = vinc_read(priv,
						STREAM_PROC_STAT_DATA(0));
			}
		}
		if (stat_en & STT_EN_AF) {
			af = priv->ctrls[CTRL_STAT_AF0 + z]->p_cur.p;
			vinc_write(priv, STREAM_PROC_STAT_CTR(0),
				   STREAM_PROC_STAT_CTR_NUM_ZONE(z));
			af->hsobel = vinc_read(priv,
					STREAM_PROC_STAT_HSOBEL(0));
			af->vsobel = vinc_read(priv,
					STREAM_PROC_STAT_VSOBEL(0));
			af->lsobel = vinc_read(priv,
					STREAM_PROC_STAT_LSOBEL(0));
			af->rsobel = vinc_read(priv,
					STREAM_PROC_STAT_RSOBEL(0));
			vinc_write(priv, STREAM_PROC_CLEAR(0),
				   STREAM_PROC_CLEAR_AF_CLR);
		}
		if (stat_en & STT_EN_ADD) {
			add = priv->ctrls[CTRL_STAT_ADD0 + z]->p_cur.p;
			reg = vinc_read(priv, STREAM_PROC_STAT_MIN(0));
			add->min_b = reg & 0xFF;
			add->min_g = (reg >> 8) & 0xFF;
			add->min_r = (reg >> 16) & 0xFF;
			reg = vinc_read(priv, STREAM_PROC_STAT_MAX(0));
			add->max_b = reg & 0xFF;
			add->max_g = (reg >> 8) & 0xFF;
			add->max_r = (reg >> 16) & 0xFF;
			add->sum_b = vinc_read(priv, STREAM_PROC_STAT_SUM_B(0));
			add->sum_g = vinc_read(priv, STREAM_PROC_STAT_SUM_G(0));
			add->sum_r = vinc_read(priv, STREAM_PROC_STAT_SUM_R(0));
			reg = vinc_read(priv, STREAM_PROC_STAT_SUM2_HI(0));
			add->sum2_b = reg & 0xFF;
			add->sum2_g = (reg >> 8) & 0xFF;
			add->sum2_r = (reg >> 16) & 0xFF;
			add->sum2_b = (add->sum2_b << 32) |
				vinc_read(priv, STREAM_PROC_STAT_SUM2_B(0));
			add->sum2_g = (add->sum2_g << 32) |
				vinc_read(priv, STREAM_PROC_STAT_SUM2_G(0));
			add->sum2_r = (add->sum2_r << 32) |
				vinc_read(priv, STREAM_PROC_STAT_SUM2_R(0));
			vinc_write(priv, STREAM_PROC_CLEAR(0),
				   STREAM_PROC_CLEAR_ADD_CLR);
		}
	}
}

static void vinc_eof_handler(struct vinc_dev *priv)
{
	if (priv->ctrls[CTRL_STAT_EN]->val) {
		/* TODO: Tasklet must complete before the next frame starts.
		 * Otherwise it will read broken statistic. We need to take
		 * into account that tasklet can run when the next frame starts
		 * (or protect ourselves from this situation). */
		if (priv->stat_odd)
			tasklet_schedule(&priv->stat_tasklet);
		priv->stat_odd = ~priv->stat_odd;
	}
	if (priv->active) {
		dev_dbg(priv->ici.v4l2_dev.dev, "Frame end\n");
		vinc_next_buffer(priv, VB2_BUF_STATE_DONE);
	} else
		dev_warn(priv->ici.v4l2_dev.dev,
			 "Unexpected interrupt. VINC started without driver?\n");
}

static irqreturn_t vinc_irq_s0(int irq, void *data)
{
	struct vinc_dev *priv = data;
	u32 int_status = vinc_read(priv, STREAM_INTERRUPT(0));

	dev_dbg(priv->ici.v4l2_dev.dev, "Interrupt stream0 0x%x\n",
		int_status);
	if (int_status & STREAM_INTERRUPT_PROC) {
		u32 int_proc = vinc_read(priv, STREAM_STATUS(0));
		u32 stream_ctr = vinc_read(priv, STREAM_CTR);

		vinc_write(priv, STREAM_CTR, 0);
		stream_ctr &= ~STREAM_CTR_DMA_CHANNELS_ENABLE;
		vinc_write(priv, STREAM_CTR, stream_ctr);
		vinc_next_buffer(priv, VB2_BUF_STATE_ERROR);

		dev_warn(priv->ici.v4l2_dev.dev,
			 "Short frame/line. Stream0_status: 0x%x\n", int_proc);
	}
	if (int_status & STREAM_INTERRUPT_DMA0) {
		u32 int_d0 = vinc_read(priv, STREAM_DMA_WR_STATUS(0, 0));

		if (int_d0 & DMA_WR_STATUS_FRAME_END)
			vinc_eof_handler(priv);
		if (int_d0 & DMA_WR_STATUS_DMA_OVF) {
			u32 stream_ctr = vinc_read(priv, STREAM_CTR);

			stream_ctr &= ~STREAM_CTR_DMA_CHANNELS_ENABLE;
			vinc_write(priv, STREAM_CTR, stream_ctr);
			vinc_next_buffer(priv, VB2_BUF_STATE_ERROR);
			dev_warn(priv->ici.v4l2_dev.dev, "s0d0: DMA overflow\n");
		}
	}
	if (int_status & STREAM_INTERRUPT_DMA1) {
		u32 int_d1 = vinc_read(priv, STREAM_DMA_WR_STATUS(0, 1));

		if (int_d1 & DMA_WR_STATUS_DMA_OVF)
			dev_warn(priv->ici.v4l2_dev.dev, "s0d1: DMA overflow\n");
	}
	vinc_write(priv, STREAM_INTERRUPT_RESET(0), int_status);
	return IRQ_HANDLED;
}

static irqreturn_t vinc_irq_s1(int irq, void *data)
{
	struct vinc_dev *priv = data;
	u32 int_status = vinc_read(priv, STREAM_INTERRUPT(1));

	dev_dbg(priv->ici.v4l2_dev.dev, "Interrupt stream1 0x%x\n",
		int_status);
	if (int_status & STREAM_INTERRUPT_PROC) {
		u32 int_proc = vinc_read(priv, STREAM_STATUS(1));

		dev_warn(priv->ici.v4l2_dev.dev,
			 "Short frame/line. Stream1_status: 0x%x\n", int_proc);
	}
	if (int_status & STREAM_INTERRUPT_DMA0)
		vinc_read(priv, STREAM_DMA_WR_STATUS(1, 0));
	if (int_status & STREAM_INTERRUPT_DMA1)
		vinc_read(priv, STREAM_DMA_WR_STATUS(1, 1));
	vinc_write(priv, STREAM_INTERRUPT_RESET(1), int_status);
	return IRQ_HANDLED;
}

static int vinc_probe(struct platform_device *pdev)
{
	struct vinc_dev *priv;
	struct resource *res;
	int err;
	u32 id;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Failed to get memory resource\n");
		return -ENODEV;
	}

	priv->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	id = vinc_read(priv, ID);
	if (id != 0x76494e01)
		dev_err(&pdev->dev, "Bad magic: %#08x\n", id);

	INIT_LIST_HEAD(&priv->capture);
	spin_lock_init(&priv->lock);
	init_completion(&priv->complete);

	priv->video_source = V4L2_MBUS_CSI2;
	priv->input_format = BAYER;

	priv->ici.priv = priv;
	priv->ici.v4l2_dev.dev = &pdev->dev;
	priv->ici.nr = pdev->id;
	priv->ici.drv_name = "vinc";
	priv->ici.ops = &vinc_host_ops;
	priv->ici.capabilities = SOCAM_HOST_CAP_STRIDE;

	priv->alloc_ctx = vb2_dma_contig_init_ctx(&pdev->dev);
	if (IS_ERR(priv->alloc_ctx))
		return PTR_ERR(priv->alloc_ctx);

	priv->irq_vio = platform_get_irq(pdev, 0);
	priv->irq_s0 = platform_get_irq(pdev, 1);
	priv->irq_s1 = platform_get_irq(pdev, 2);
	if (priv->irq_vio < 0 || priv->irq_s0 < 0 || priv->irq_s1 < 0) {
		dev_err(&pdev->dev, "Failed to get required IRQs\n");
		return -ENODEV;
	}

	/* request irq */
	err = devm_request_irq(&pdev->dev, priv->irq_vio, vinc_irq_vio,
			       0, dev_name(&pdev->dev), priv);
	err |= devm_request_irq(&pdev->dev, priv->irq_s0, vinc_irq_s0,
			       0, dev_name(&pdev->dev), priv);
	err |= devm_request_irq(&pdev->dev, priv->irq_s1, vinc_irq_s1,
			       0, dev_name(&pdev->dev), priv);
	if (err) {
		dev_err(&pdev->dev, "Failed to request required IRQs\n");
		return err;
	}
	tasklet_init(&priv->stat_tasklet, vinc_stat_tasklet,
		     (unsigned long)priv);

	vinc_write(priv, CMOS_CTR(0), CMOS_CTR_PCLK_EN | CMOS_CTR_PCLK_SRC(0) |
			CMOS_CTR_CLK_DIV(4) | CMOS_CTR_FSYNC_EN);
	vinc_write(priv, CMOS_TIMER_HIGH(0), 1);
	vinc_write(priv, CMOS_TIMER_LOW(0), 1);

	/* GLOBAL_ENABLE need for generate clocks to sensor */
	vinc_write(priv, AXI_MASTER_CFG, AXI_MASTER_CFG_MAX_BURST(2) |
			AXI_MASTER_CFG_MAX_WR_ID(1) |
			AXI_MASTER_CFG_BUF_LAYOUT(0x1) |
			AXI_MASTER_CFG_4K_BOUND_EN |
			AXI_MASTER_CFG_GLOBAL_EN);

	err = soc_camera_host_register(&priv->ici);
	if (err) {
		dev_err(&pdev->dev, "Failed to register SoC camera host\n");
		return err;
	}

	return 0;
}

static int vinc_remove(struct platform_device *pdev)
{
	struct soc_camera_host *soc_host = to_soc_camera_host(&pdev->dev);
	struct vinc_dev *priv = container_of(soc_host, struct vinc_dev, ici);
	u32 reg = vinc_read(priv, AXI_MASTER_CFG);

	reg &= ~AXI_MASTER_CFG_GLOBAL_EN;
	vinc_write(priv, AXI_MASTER_CFG, reg);

	if (priv->ici.icd && priv->ici.icd->host_priv)
		kfree(priv->ici.icd->host_priv);
	soc_camera_host_unregister(soc_host);
	if (platform_get_resource(pdev, IORESOURCE_MEM, 1))
		dma_release_declared_memory(&pdev->dev);
	vb2_dma_contig_cleanup_ctx(priv->alloc_ctx);

	return 0;
}

static const struct of_device_id vinc_of_match[] = {
	{ .compatible = "elvees,vinc" },
	{ }
};
MODULE_DEVICE_TABLE(of, vinc_of_match);

static struct platform_driver vinc_driver = {
	.driver		= {
		.name	= "vinc",
		.of_match_table = vinc_of_match,
	},
	.probe		= vinc_probe,
	.remove		= vinc_remove,
};

module_platform_driver_probe(vinc_driver, vinc_probe);

MODULE_DESCRIPTION("VINC driver");
MODULE_AUTHOR("Vasiliy Zasukhin <vzasukhin@elvees.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1.0");
MODULE_ALIAS("platform:vinc");
MODULE_SUPPORTED_DEVICE("video");
