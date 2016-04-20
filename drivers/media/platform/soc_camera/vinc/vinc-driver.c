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
#include <linux/clk.h>

#include <media/v4l2-async.h>
#include <media/v4l2-common.h>
#include <media/v4l2-dev.h>
#include <media/soc_camera.h>
#include <media/videobuf2-dma-contig.h>
#include <media/v4l2-mediabus.h>
#include <media/soc_mediabus.h>
#include <asm/neon.h>

#include "vinc-neon.h"

#define MODULE_NAME "vinc"

/* VINC registers */
#define ID				0x0
#define AXI_MASTER_CFG			0x4
#define AXI_MASTER_STATUS		0x8
#define INTERRUPT			0x20
#define INTERRUPT_RESET			0x24
#define INTERRUPT_MASK			0x28

#define PPORT_BASE(p)			(0x80 * (p) + 0x100)
#define PPORT_CFG(p)			(PPORT_BASE(p) + 0x0)

#define PINTERFACE_BASE(p)		(0x100 * (p) + 0x300)
#define PINTERFACE_CFG(p)		(PINTERFACE_BASE(p) + 0x0)
#define PINTERFACE_CCMOV(p, c)		(PINTERFACE_BASE(p) + 0x20 + ((c) * 4))
#define PINTERFACE_HVFSYNC(p)		(PINTERFACE_BASE(p) + 0x80)

#define PPORT_INP_MUX_CFG		0x700
#define PPORT_STATUS			0x704
#define PPORT_TEST_SRC			0x720

#define CMOS_BASE(c)			(0x20 * (c) + 0x780)
#define CMOS_CTR(c)			(CMOS_BASE(c) + 0x0)
#define CMOS_TIMER_HIGH(c)		(CMOS_BASE(c) + 0x4)
#define CMOS_TIMER_LOW(c)		(CMOS_BASE(c) + 0x8)

#define CSI2_PORT_BASE(c)		(0x200 * (c) + 0x800)

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

#define STREAM_BASE(s)			(0x400 * (s) + 0x1000)
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
#define STREAM_PROC_BP_BAD_LINE(s, l)	(STREAM_BASE(s) + 0x70 + ((l) * 4))
#define STREAM_PROC_BP_BAD_COLUMN(s, c)	(STREAM_BASE(s) + 0x90 + ((c) * 4))
#define STREAM_PROC_DR_CTR(s)		(STREAM_BASE(s) + 0xb0)
#define STREAM_PROC_DR_DATA(s)		(STREAM_BASE(s) + 0xb4)
#define STREAM_PROC_DR_COUNT(s)		(STREAM_BASE(s) + 0xb8)
#define STREAM_PROC_CC_COEFF(s, c)	(STREAM_BASE(s) + 0xc0 + ((c) * 4))
#define STREAM_PROC_CC_OFFSET(s, o)	(STREAM_BASE(s) + 0xd4 + ((o) * 4))
#define STREAM_PROC_GC_CTR(s)		(STREAM_BASE(s) + 0xe0)
#define STREAM_PROC_GC_DATA(s)		(STREAM_BASE(s) + 0xe4)
#define STREAM_PROC_CT_COEFF(s, c)	(STREAM_BASE(s) + 0xf0 + ((c) * 4))
#define STREAM_PROC_CT_OFFSET(s, o)	(STREAM_BASE(s) + 0x104 + ((o) * 4))

#define STREAM_PROC_STAT_ZONE_BASE(s, z) (0xc * (z) + STREAM_BASE(s) + 0x110)
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

#define STREAM_DMA_BASE(s, d)		(0x100 * (d) + STREAM_BASE(s) + 0x200)
#define STREAM_DMA_FBUF_CFG(s, d)	(STREAM_DMA_BASE(s, d) + 0x0)
#define STREAM_DMA_PIXEL_FMT(s, d)	(STREAM_DMA_BASE(s, d) + 0x4)
#define STREAM_DMA_FBUF_HORIZ(s, d)	(STREAM_DMA_BASE(s, d) + 0x8)
#define STREAM_DMA_FBUF_VERT(s, d)	(STREAM_DMA_BASE(s, d) + 0xc)
#define STREAM_DMA_FBUF_VERT_ODD(s, d)	(STREAM_DMA_BASE(s, d) + 0x10)
#define STREAM_DMA_FBUF_DECIM(s, d)	(STREAM_DMA_BASE(s, d) + 0x14)

#define STREAM_DMA_FBUF_BASE(s, d, b)	(0x10 * (b) + STREAM_DMA_BASE(s, d)\
					+ 0x20)
#define STREAM_DMA_FBUF_LSTEP(s, d, b)	(STREAM_DMA_FBUF_BASE(s, d, b) + 0x4)
#define STREAM_DMA_FBUF_FSTEP(s, d, b)	(STREAM_DMA_FBUF_BASE(s, d, b) + 0x8)

#define STREAM_DMA_WR_CTR(s, d)		(STREAM_DMA_BASE(s, d) + 0x60)
#define STREAM_DMA_WR_STATUS(s, d)	(STREAM_DMA_BASE(s, d) + 0x64)
#define STREAM_DMA_WR_COUNT(s, d, c)	(STREAM_DMA_BASE(s, d) + 0x68\
					+ ((c) * 4))
#define STREAM_DMA_CUR_ADDR(s, d, a)	(STREAM_DMA_BASE(s, d) + 0x70\
					+ ((a) * 4))
#define STREAM_DMA_TEST_DATA(s, d)	(STREAM_DMA_BASE(s, d) + 0x80)
#define STREAM_DMA_TEST_CTR(s, d)	(STREAM_DMA_BASE(s, d) + 0x84)

#define STREAM_CTR			0x3f00

#define CC_CT_OFFSET_COEFF8		0x10
#define CC_CT_OFFSET_OFFSET0_1		0x14
#define CC_CT_OFFSET_OFFSET2		0x18

/* Bits for AXI_MASTER_CFG register */
#define AXI_MASTER_CFG_MAX_BURST(v)	((v) & 0x7)
#define AXI_MASTER_CFG_MAX_WR_ID(v)	(((v) & 0xF) << 4)
#define AXI_MASTER_CFG_BUF_LAYOUT(v)	(((v) & 0xF) << 8)
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
#define PORT_CFG_PIXEL_MODE(v)		((v) & 0x3)
#define PORT_CFG_PCLK_DIV		BIT(2)
#define PORT_CFG_PCLK_NEG_DIV		BIT(3)
#define PORT_CFG_PCLK_HALF_CLK		BIT(4)
#define PORT_CFG_DIGITAL_DELAY(v)	(((v) & 0x7) << 5)
#define PORT_CFG_VIN_SRC(v)		(((v) & 0x3) << 12)

/* Bits for PINTERFACE_CFG register */
#define PINTERFACE_CFG_CYCLE_NUM(v)	((v) & 0x7)
#define PINTERFACE_CFG_PIXEL_NUM_EVEN(v)	(((v) & 0x3) << 4)
#define PINTERFACE_CFG_PIXEL_NUM_ODD(v)	(((v) & 0x3) << 8)
#define PINTERFACE_CFG_PV_ALGN_MODE(v)	(((v) & 0xF) << 12)
#define PINTERFACE_CFG_PORT_NUM_SYNC(v)	(((v) & 0x3) << 16)
#define PINTERFACE_CFG_EMB_SYNC		BIT(18)
#define PINTERFACE_CFG_EMB_SYNC_CORR	BIT(19)
#define PINTERFACE_CFG_PHASE_CORR(v)	(((v) & 0x7) << 20)
#define PINTERFACE_CFG_FORW_H(v)	(((v) & 0xFF) << 24)

/* Bits for PINTERFACE_HVFSYNCregister */
#define PINTERFACE_HVFSYNC_INVERS_H	BIT(0)
#define PINTERFACE_HVFSYNC_INVERS_V	BIT(1)
#define PINTERFACE_HVFSYNC_INVERS_F	BIT(2)
#define PINTERFACE_HVFSYNC_BUILT_MODE(v)	(((v) & 0x7) << 3)
#define PINTERFACE_HVFSYNC_DELAY_F_EN	BIT(6)
#define PINTERFACE_HVFSYNC_DELAY_V(v)	(((v) & 0x1F) << 8)
#define PINTERFACE_HVFSYNC_DELAY_F(v)	(((v) & 0x3F) << 16)
#define PINTERFACE_HVFSYNC_PRE_DELAY_V(v)	(((v) & 0x1F) << 24)
#define PINTERFACE_HVFSYNC_DELAY_VF_ODD_OFS(v)	(((v) & 0x7) << 29)



/* Bits for CMOS_CTR register */
#define CMOS_CTR_RESET			BIT(0)
#define CMOS_CTR_PCLK_EN		BIT(1)
#define CMOS_CTR_PCLK_SRC(v)		(((v) & 0x3) << 2)
#define CMOS_CTR_CLK_DIV(v)		(((v) & 0xF) << 4)
#define CMOS_CTR_FSYNC_EN		BIT(8)

/* Bits for CSI2_PORT_SYS_CTR register */
#define CSI2_PORT_SYS_CTR_ENABLE	BIT(0)
#define CSI2_PORT_SYS_CTR_TWO_PORTS	BIT(1)
#define CSI2_PORT_SYS_CTR_FREQ_RATIO(v)	(((v) & 0x3F) << 8)

/* Bits for CSI2_DPHY_TIM1 register */
#define CSI2_TIM1_DLN_CNT_HS_PREP(v)	((v) & 0xFF)
#define CSI2_TIM1_DLN_CNT_HS_ZERO(v)	(((v) & 0xFF) << 8)
#define CSI2_TIM1_DLN_CNT_HS_TRAIL(v)	(((v) & 0xFF) << 16)
#define CSI2_TIM1_DLN_CNT_HS_EXIT(v)	(((v) & 0xFF) << 24)

/* Bits for CSI2_DPHY_TIM2 register */
#define CSI2_TIM2_CLN_CNT_HS_PREP(v)	((v) & 0xFF)
#define CSI2_TIM2_CLN_CNT_HS_ZERO(v)	(((v) & 0xFF) << 8)
#define CSI2_TIM2_CLN_CNT_HS_TRAIL(v)	(((v) & 0xFF) << 16)
#define CSI2_TIM2_CLN_CNT_HS_EXIT(v)	(((v) & 0xFF) << 24)

/* Bits for CSI2_DPHY_TIM3 register */
#define CSI2_TIM3_CLN_CNT_LPX(v)	((v) & 0xFF)
#define CSI2_TIM3_DLN_CNT_LPX(v)	(((v) & 0xFF) << 8)
#define CSI2_TIM3_CLN_CNT_PLL(v)	(((v) & 0xFFFF) << 16)

/* Bits for STREAM_INP_DECIM_CTR register */
#define STREAM_INP_DECIM_HDECIM(v)	((v) & 0xF)
#define STREAM_INP_DECIM_VDECIM(v)	(((v) & 0xF) << 4)
#define STREAM_INP_DECIM_FDECIM(v)	(((v) & 0x3F) << 8)
#define STREAM_INP_DECIM_INTERLACE_MODE	BIT(16)

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
#define STREAM_PROC_CFG_STT_EN(v)	(((v) & 0x7) << 9)
#define STREAM_PROC_CFG_STT_ZONE_OFFSET	12
#define STREAM_PROC_CFG_STT_ZONE_EN(v)	(((v) & 0xF) << 12)
#define STREAM_PROC_CFG_IM_EN		BIT(16)
#define STREAM_PROC_CFG_CT_SRC(v)	(((v) & 0x1) << 23)
#define STREAM_PROC_CFG_444TO422_SRC(v)	(((v) & 0x1) << 24)
#define STREAM_PROC_CFG_422TO420_SRC(v)	(((v) & 0x1) << 25)
#define STREAM_PROC_CFG_DMA0_SRC(v)	(((v) & 0x7) << 26)
#define STREAM_PROC_CFG_DMA1_SRC(v)	(((v) & 0x7) << 29)

#define STT_EN_HIST			0x1
#define STT_EN_AF			0x2
#define STT_EN_ADD			0x4

#define DMA_SRC_CROP			0
#define DMA_SRC_DR			0x1
#define DMA_SRC_444			0x2
#define DMA_SRC_CT			0x3
#define DMA_SRC_422			0x4
#define DMA_SRC_420			0x5
#define DMA_SRC_IS			0x6
#define DMA_SRC_MASK			0x7

/* Bits for STREAM_PROC_CTR register */
#define STREAM_PROC_CTR_BAYER_MONO	BIT(0)
#define STREAM_PROC_CTR_BAYER_MODE(v)	(((v) & 0x3) << 1)
#define STREAM_PROC_CTR_422TO444ALG(v)	(((v) & 0x3) << 6)
#define STREAM_PROC_CTR_422TO444FILL	BIT(8)
#define STREAM_PROC_CTR_444TO422ALG(v)	(((v) & 0x3) << 9)
#define STREAM_PROC_CTR_444TO422FILL	BIT(11)
#define STREAM_PROC_CTR_422TO420ALG	BIT(12)
#define STREAM_PROC_CTR_HIST_THR	BIT(13)
#define STREAM_PROC_CTR_AF_THR		BIT(14)
#define STREAM_PROC_CTR_ADD_THR		BIT(15)
#define STREAM_PROC_CTR_AF_COLOR(v)	(((v) & 0x3) << 16)
#define STREAM_PROC_CTR_IM_COLOR(v)	(((v) & 0x3) << 21)

/* Bits for STREAM_PROC_CLEAR register */
#define STREAM_PROC_CLEAR_AF_CLR	BIT(0)
#define STREAM_PROC_CLEAR_ADD_CLR	BIT(1)
#define STREAM_PROC_CLEAR_THR_CLR	BIT(2)

/* Bits for STREAM_PROC_STAT_CTR register */
#define STREAM_PROC_STAT_CTR_ADDR_HIST(v)	((v) & 0xFF)
#define STREAM_PROC_STAT_CTR_NUM_ZONE(v)	(((v) & 0x3) << 16)
#define STREAM_PROC_STAT_CTR_COLOR_HIST(v)	(((v) & 0x3) << 18)

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
#define DMA_WR_CTR_LINE_INT_PERIOD(v)	(((v) & 0xFFF) << 1)
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
#define STREAM_CTR_STREAM_ENABLE(n)	BIT(n)
#define STREAM_CTR_DMA_CHANNELS_ENABLE	BIT(8)

#define CTRL_BAD_PIXELS_COUNT		4096
#define CTRL_BAD_ROWSCOLS_COUNT		16
#define CTRL_GC_ELEMENTS_COUNT		4096
#define CTRL_DR_ELEMENTS_COUNT		4096

#define MAX_WIDTH_HEIGHT		4095
#define MAX_COMP_VALUE			4095

#define CLUSTER_SIZE(c) (sizeof(c) / sizeof(struct v4l2_ctrl *))

enum vinc_input_format { UNKNOWN, BAYER, RGB, YCbCr };

struct vinc_cluster_bp {
	struct v4l2_ctrl *enable;
	struct v4l2_ctrl *pix;
	struct v4l2_ctrl *row;
	struct v4l2_ctrl *col;
};

struct vinc_cluster_gamma {
	struct v4l2_ctrl *enable;
	struct v4l2_ctrl *curve;
	struct v4l2_ctrl *gamma;
};

struct vinc_cluster_cc {
	struct v4l2_ctrl *enable;
	struct v4l2_ctrl *cc;
	struct v4l2_ctrl *brightness;
	struct v4l2_ctrl *contrast;
	struct v4l2_ctrl *saturation;
	struct v4l2_ctrl *hue;
	struct v4l2_ctrl *dowb;
};

struct vinc_cluster_ct {
	struct v4l2_ctrl *enable;
	struct v4l2_ctrl *ct;
};

struct vinc_cluster_dr {
	struct v4l2_ctrl *enable;
	struct v4l2_ctrl *dr;
};

struct vinc_cluster_stat {
	struct v4l2_ctrl *enable;
	struct v4l2_ctrl *af_color;
	struct v4l2_ctrl *af_th;
	struct v4l2_ctrl *zone[4];
	struct v4l2_ctrl *hist[4];
	struct v4l2_ctrl *af[4];
	struct v4l2_ctrl *add[4];
};

struct vinc_cluster {
	struct vinc_cluster_bp bp;
	struct vinc_cluster_gamma gamma;
	struct vinc_cluster_cc cc;
	struct vinc_cluster_ct ct;
	struct vinc_cluster_dr dr;
	struct vinc_cluster_stat stat;
};

struct vinc_stream {
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

	struct vinc_cluster cluster;
	struct v4l2_ctrl *test_pattern;
	int stat_odd:1;

	struct v4l2_crop crop1;
	struct v4l2_crop crop2;
	u32 fdecim;

	enum v4l2_ycbcr_encoding ycbcr_enc;
	enum v4l2_quantization quantization;

	int sequence;

	u8 devnum;
};

struct vinc_dev {
	struct soc_camera_host ici;

	int irq_vio;
	int irq_stream[2];
	void __iomem *base;

	struct vinc_stream stream[2];

	u32 reset_active;

	struct clk *pclk;
	struct clk *aclk;
	struct clk *sclk;
};

struct vinc_cam {
	/* Client output */
	unsigned int width;
	unsigned int height;
	u32 code;
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
		.bits_per_sample = 8,
	},
};

/* per video frame buffer */
struct vinc_buffer {
	struct vb2_buffer vb; /* v4l buffer must be first */
	struct list_head queue;
};

static uint memory_per_stream = CONFIG_VIDEO_VINC_MEMORY_PER_STREAM_MB * SZ_1M;
module_param(memory_per_stream, uint, 0644);
MODULE_PARM_DESC(memory_per_stream,
		 "Maximum memory for video buffers in bytes");

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
	unsigned int max_count;

	if (fmt)
		return -EINVAL;

	dev_dbg(icd->parent, "Requested %u buffers\n", *count);

	sizes[0] = icd->sizeimage;
	dev_dbg(icd->parent, "%s: image_size=%d\n", __func__,
		icd->sizeimage);

	alloc_ctxs[0] = priv->stream[icd->devnum].alloc_ctx;

	if (memory_per_stream) {
		max_count = memory_per_stream / icd->sizeimage;
		if (*count > max_count)
			*count = max_count;
	}
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

static void vinc_start_capture(struct vinc_dev *priv,
			       struct soc_camera_device *icd)
{
	dma_addr_t phys_addr_top;
	const u8 devnum = icd->devnum;
	struct vinc_stream * const stream = &priv->stream[devnum];
	u32 wr_ctr = vinc_read(priv, STREAM_DMA_WR_CTR(devnum, 0));

	wr_ctr &= ~DMA_WR_CTR_DMA_EN;
	vinc_write(priv, STREAM_DMA_WR_CTR(devnum, 0), wr_ctr);
	if (!stream->active)
		return;
	phys_addr_top = vb2_dma_contig_plane_dma_addr(stream->active, 0);
	switch (icd->current_fmt->host_fmt->fourcc) {
	case V4L2_PIX_FMT_BGR32:
		vinc_write(priv, STREAM_DMA_FBUF_BASE(devnum, 0, 0),
			   phys_addr_top);
		break;
	case V4L2_PIX_FMT_M420:
		vinc_write(priv, STREAM_DMA_FBUF_BASE(devnum, 0, 0),
			   phys_addr_top + (stream->crop2.c.width << 1));
		vinc_write(priv, STREAM_DMA_FBUF_BASE(devnum, 0, 1),
			   phys_addr_top);
		break;
	default:
		dev_err(priv->ici.v4l2_dev.dev, "Unknown output fourcc %#x",
			icd->current_fmt->host_fmt->fourcc);
		return;
	}
	wr_ctr |= DMA_WR_CTR_DMA_EN;
	vinc_write(priv, STREAM_DMA_WR_CTR(devnum, 0), wr_ctr);
}

static void vinc_buf_queue(struct vb2_buffer *vb)
{
	struct soc_camera_device *icd = container_of(vb->vb2_queue,
			struct soc_camera_device, vb2_vidq);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct vinc_dev *priv = ici->priv;
	struct vinc_buffer *buf = to_vinc_vb(vb);
	unsigned long size = icd->sizeimage;
	const u8 devnum = icd->devnum;
	struct vinc_stream * const stream = &priv->stream[devnum];

	dev_dbg(icd->parent, "Add buffer #%u to queue\n",
		buf->vb.v4l2_buf.index);

	if (vb2_plane_size(vb, 0) < size) {
		dev_err(icd->parent, "Buffer #%u too small (%lu < %lu)\n",
		       vb->v4l2_buf.index, vb2_plane_size(vb, 0), size);
		vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
	}
	vb2_set_plane_payload(vb, 0, size);

	spin_lock_irq(&stream->lock);
	list_add_tail(&buf->queue, &stream->capture);
	if (!stream->active) {
		stream->active = vb;
		vinc_start_capture(priv, icd);
	}
	spin_unlock_irq(&stream->lock);
}

static void vinc_configure_input(struct vinc_stream *stream)
{
	const u8 devnum = stream->devnum;
	struct vinc_dev *priv = container_of(stream, struct vinc_dev,
					     stream[devnum]);

	if (stream->test_pattern->val) {
		u32 test_src = 0;

		test_src |= stream->crop1.c.width + stream->crop1.c.left;
		test_src |= (stream->crop1.c.height +
				stream->crop1.c.top) << 12;
		test_src |= 5 << 24;
		test_src |= (stream->test_pattern->val - 1) << 29;

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
	} else if (stream->video_source == V4L2_MBUS_CSI2) {
		vinc_write(priv, PPORT_INP_MUX_CFG, 0x0);
		vinc_write(priv, PPORT_CFG(0), 0x0);
		vinc_write(priv, PPORT_CFG(1), 0x0);
		vinc_write(priv, PPORT_CFG(2), 0x0);

		vinc_write(priv, CSI2_DEVICE_READY(devnum), 0x0);
		vinc_write(priv, CSI2_INTR(devnum), 0x0007FFFF);
		vinc_write(priv, CSI2_PORT_GENFIFO_CTR(devnum), 0x0);

		/* 1lane, timeout=max */
		vinc_write(priv, CSI2_FUNC_PROG(devnum), 0x1ffffc |
				((stream->csi2_lanes - 1) & 0x3));
		vinc_write(priv, CSI2_DPHY_TIM3(devnum),
			   CSI2_TIM3_CLN_CNT_LPX(0xBF) |
			   CSI2_TIM3_DLN_CNT_LPX(0x12));
		vinc_write(priv, CSI2_SYNC_COUNT(devnum), 0x14141414);
		vinc_write(priv, CSI2_RCV_COUNT(devnum), 0x04040404);

		vinc_write(priv, CSI2_FSLS(devnum), 0x2);
		vinc_write(priv, CSI2_LSDV(devnum), 0x2);
		vinc_write(priv, CSI2_DVLE(devnum), 0x2);
		vinc_write(priv, CSI2_LEFE(devnum), 0x2);
		vinc_write(priv, CSI2_FEFS(devnum), 0x2);
		vinc_write(priv, CSI2_LELS(devnum), 0x4);
		vinc_write(priv, CSI2_LOOP_BACK(devnum), 0x0);
		vinc_write(priv, CSI2_RAW8(devnum), 0x0);
		vinc_write(priv, CSI2_DPHY_TIM1(devnum),
			   CSI2_TIM1_DLN_CNT_HS_PREP(0x06) |
			   CSI2_TIM1_DLN_CNT_HS_ZERO(0x04) |
			   CSI2_TIM1_DLN_CNT_HS_TRAIL(0x07) |
			   CSI2_TIM1_DLN_CNT_HS_EXIT(0x02));
		vinc_write(priv, CSI2_DPHY_TIM2(devnum),
			   CSI2_TIM2_CLN_CNT_HS_PREP(0x06) |
			   CSI2_TIM2_CLN_CNT_HS_ZERO(0x04) |
			   CSI2_TIM2_CLN_CNT_HS_TRAIL(0x11) |
			   CSI2_TIM2_CLN_CNT_HS_EXIT(0x03));
		vinc_write(priv, CSI2_TRIM0(devnum), 0x02000000);
		vinc_write(priv, CSI2_DEVICE_READY(devnum), 0x1);
		vinc_write(priv, STREAM_INP_CFG(devnum), 0x2 + devnum);
	} else if (stream->video_source == V4L2_MBUS_PARALLEL) {
		vinc_write(priv, PPORT_INP_MUX_CFG, 0);
		vinc_write(priv, PPORT_CFG(devnum), PORT_CFG_PIXEL_MODE(1));
		vinc_write(priv, PPORT_CFG(2), PORT_CFG_PIXEL_MODE(0));
		if (stream->input_format == BAYER) {
			vinc_write(priv, PINTERFACE_CFG(devnum),
					PINTERFACE_CFG_CYCLE_NUM(1) |
					PINTERFACE_CFG_PIXEL_NUM_EVEN(1) |
					PINTERFACE_CFG_PORT_NUM_SYNC(devnum));
			vinc_write(priv, PINTERFACE_CCMOV(devnum, 0),
				   0x1 + devnum);
		} else
			dev_err(priv->ici.v4l2_dev.dev, "Unknown input format %#x",
				stream->input_format);

		vinc_write(priv, PINTERFACE_HVFSYNC(devnum),
				PINTERFACE_HVFSYNC_DELAY_V(0x11) |
				PINTERFACE_HVFSYNC_PRE_DELAY_V(1));
		vinc_write(priv, STREAM_INP_CFG(devnum), devnum);
	} else
		dev_err(priv->ici.v4l2_dev.dev, "Unknown input source %#x",
			stream->video_source);
}

static int vinc_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct soc_camera_device *icd = container_of(q,
			struct soc_camera_device, vb2_vidq);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct vinc_dev *priv = ici->priv;
	int retry_count = 10;
	unsigned long timeout;
	const u8 devnum = icd->devnum;
	struct vinc_stream * const stream = &priv->stream[devnum];
	u32 csi2_port_sys_ctr = vinc_read(priv, CSI2_PORT_SYS_CTR(0));
	u32 csi2_intr;
	u32 stream_ctr;

	dev_dbg(icd->parent, "Start streaming (count: %u)\n", count);

	if (stream->video_source == V4L2_MBUS_CSI2) {
		/* Workaround for mcom issue rf#1361 (see errata)
		 * Check that VINC captures video and reenable MIPI port
		 * otherwise. */
		do {
			vinc_write(priv, CSI2_PORT_SYS_CTR(devnum),
				   csi2_port_sys_ctr &
				   ~CSI2_PORT_SYS_CTR_ENABLE);
			vinc_write(priv, CSI2_PORT_SYS_CTR(devnum),
				   csi2_port_sys_ctr |
				   CSI2_PORT_SYS_CTR_ENABLE);
			vinc_configure_input(stream);

			timeout = jiffies + msecs_to_jiffies(30);
			do {
				csi2_intr = vinc_read(priv, CSI2_INTR(devnum));
				if (!(csi2_intr & BIT(9)))
					schedule();
				else
					break;
			} while (time_before(jiffies, timeout));
		} while (!(csi2_intr & BIT(9)) && retry_count--);

		if (retry_count < 0) {
			struct vinc_buffer *buf, *tmp;

			list_for_each_entry_safe(buf, tmp,
						 &stream->capture, queue) {
				list_del_init(&buf->queue);
				vb2_buffer_done(&buf->vb, VB2_BUF_STATE_QUEUED);
			}
			dev_err(icd->parent,
				"Can not receive video from sensor\n");
			return -EIO;
		}
	} else if (stream->video_source == V4L2_MBUS_PARALLEL)
		vinc_configure_input(stream);

	vinc_write(priv, STREAM_DMA_WR_CTR(devnum, 0), DMA_WR_CTR_FRAME_END_EN);
	stream_ctr = vinc_read(priv, STREAM_CTR);
	stream_ctr |= STREAM_CTR_DMA_CHANNELS_ENABLE;
	stream_ctr |= STREAM_CTR_STREAM_ENABLE(devnum);
	vinc_write(priv, STREAM_CTR, stream_ctr);

	stream->sequence = 0;
	spin_lock_irq(&stream->lock);
	if (stream->active)
		vinc_start_capture(priv, icd);
	spin_unlock_irq(&stream->lock);

	return 0;
}

static void vinc_stop_streaming(struct vb2_queue *q)
{
	struct soc_camera_device *icd = container_of(q,
			struct soc_camera_device, vb2_vidq);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct vinc_dev *priv = ici->priv;
	struct vinc_buffer *buf, *tmp;
	const u8 devnum = icd->devnum;
	struct vinc_stream * const stream = &priv->stream[devnum];
	u32 csi2_port_sys_ctr;
	u32 stream_ctr = vinc_read(priv, STREAM_CTR);

	dev_dbg(icd->parent, "Stop streaming\n");

	stream_ctr &= ~STREAM_CTR_STREAM_ENABLE(devnum);
	vinc_write(priv, STREAM_CTR, stream_ctr);

	vinc_write(priv, STREAM_DMA_WR_CTR(devnum, 0), 0x0);
	csi2_port_sys_ctr = vinc_read(priv, CSI2_PORT_SYS_CTR(devnum));
	vinc_write(priv, CSI2_PORT_SYS_CTR(devnum),
		   csi2_port_sys_ctr & ~CSI2_PORT_SYS_CTR_ENABLE);
	/* GLOBAL_ENABLE still enable for sensor clocks */

	spin_lock_irq(&stream->lock);

	stream->active = NULL;

	list_for_each_entry_safe(buf, tmp,
				 &stream->capture, queue) {
		list_del_init(&buf->queue);
		if (buf->vb.state == VB2_BUF_STATE_ACTIVE)
			vb2_buffer_done(&buf->vb, VB2_BUF_STATE_ERROR);
	}

	spin_unlock_irq(&stream->lock);
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

static u32 vinc_get_dma_src(struct vinc_dev *priv,
			    struct soc_camera_device *icd)
{
	switch (icd->current_fmt->host_fmt->fourcc) {
	case V4L2_PIX_FMT_BGR32:
		return (priv->stream[icd->devnum].cluster.ct.enable->val ?
				DMA_SRC_CT : DMA_SRC_444);
	case V4L2_PIX_FMT_M420:
		return DMA_SRC_420;
	default:
		dev_warn(priv->ici.v4l2_dev.dev, "Unknown output format %#x\n",
			 icd->current_fmt->host_fmt->fourcc);
		return DMA_SRC_CROP;
	}
}

static void set_bad_pixels(struct vinc_dev *priv, u8 devnum,
			   struct vinc_bad_pixel *bp)
{
	int i;

	vinc_write(priv, STREAM_PROC_BP_MAP_CTR(devnum), 0);
	for (i = 0; i < CTRL_BAD_PIXELS_COUNT; i++)
		vinc_write(priv, STREAM_PROC_BP_MAP_DATA(devnum),
			   (bp[i].y << 12) | bp[i].x);
}

static void set_bad_rows_cols(struct vinc_dev *priv, u8 devnum, u16 *data,
			      int is_cols)
{
	int i;
	u32 reg_start = is_cols ? STREAM_PROC_BP_BAD_COLUMN(devnum, 0) :
			STREAM_PROC_BP_BAD_LINE(devnum, 0);

	for (i = 0; i < (CTRL_BAD_ROWSCOLS_COUNT / 2); i++)
		vinc_write(priv, reg_start + i * sizeof(u32),
			   (data[i * 2] & 0xFFF) |
			   ((data[i * 2 + 1] & 0xFFF) << 16));
}

static void set_gc_curve(struct vinc_dev *priv, u8 devnum,
			 struct vinc_gamma_curve *gc)
{
	int i;

	vinc_write(priv, STREAM_PROC_GC_CTR(devnum), 0);
	for (i = 0; i < CTRL_GC_ELEMENTS_COUNT; i++)
		vinc_write(priv, STREAM_PROC_GC_DATA(devnum),
			   (gc->green[i] << 16) | gc->red[i]);
	vinc_write(priv, STREAM_PROC_GC_CTR(devnum), 0x1000);
	for (i = 0; i < CTRL_GC_ELEMENTS_COUNT; i++)
		vinc_write(priv, STREAM_PROC_GC_DATA(devnum),
			   gc->blue[i]);
}

static void set_cc_ct(struct vinc_dev *priv, u8 devnum, struct vinc_cc *cc,
		      int is_ct)
{
	int i;
	u32 start_reg = is_ct ? STREAM_PROC_CT_COEFF(devnum, 0) :
			STREAM_PROC_CC_COEFF(devnum, 0);
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

static void set_dr(struct vinc_dev *priv, u8 devnum, u16 *dr)
{
	int i;

	vinc_write(priv, STREAM_PROC_DR_CTR(devnum), 0);
	for (i = 0; i < CTRL_DR_ELEMENTS_COUNT; i++)
		vinc_write(priv, STREAM_PROC_DR_DATA(devnum), dr[i]);
}

static void set_stat_af_color(struct vinc_dev *priv, u8 devnum, u32 color)
{
	u32 proc_ctr = vinc_read(priv, STREAM_PROC_CTR(devnum));

	proc_ctr &= ~STREAM_PROC_CTR_AF_COLOR(0x3);
	proc_ctr |= STREAM_PROC_CTR_AF_COLOR(color);
	vinc_write(priv, STREAM_PROC_CTR(devnum), proc_ctr);
}

static void vinc_stat_start(struct vinc_stream *stream)
{
	struct vinc_dev *priv = container_of(stream, struct vinc_dev,
					     stream[stream->devnum]);

	stream->stat_odd = 0;
	vinc_write(priv, STREAM_PROC_CLEAR(0),
		   STREAM_PROC_CLEAR_THR_CLR);
}

static void set_stat_zone(struct vinc_stream *stream, u32 zone_id,
			  struct vinc_stat_zone *zone)
{
	const u8 devnum = stream->devnum;
	struct vinc_dev *priv = container_of(stream, struct vinc_dev,
					     stream[devnum]);
	u32 proc_cfg = vinc_read(priv, STREAM_PROC_CFG(devnum));

	if (zone->enable) {
		u32 lt = (zone->x_lt + stream->crop2.c.left) |
				((zone->y_lt +
				stream->crop2.c.top) << 16);
		u32 rb = (zone->x_rb + stream->crop2.c.left) |
				((zone->y_rb +
				stream->crop2.c.top) << 16);

		vinc_write(priv, STREAM_PROC_STAT_ZONE_LT(devnum, zone_id), lt);
		vinc_write(priv, STREAM_PROC_STAT_ZONE_RB(devnum, zone_id), rb);
		proc_cfg |= BIT(STREAM_PROC_CFG_STT_ZONE_OFFSET + zone_id);
	} else
		proc_cfg &= ~(BIT(STREAM_PROC_CFG_STT_ZONE_OFFSET + zone_id));
	vinc_write(priv, STREAM_PROC_CFG(devnum), proc_cfg);
}

/* TODO: Split this function into two: the one that activates cluster and
 * the other that enables/disables HW block. */
static void cluster_activate(struct vinc_dev *priv, u8 devnum, u32 block_mask,
			     struct v4l2_ctrl **cluster)
{
	u32 proc_cfg = vinc_read(priv, STREAM_PROC_CFG(0));
	struct v4l2_ctrl *master = cluster[0];
	int i;

	if (master->val)
		proc_cfg |= block_mask;
	else
		proc_cfg &= ~block_mask;
	vinc_write(priv, STREAM_PROC_CFG(devnum), proc_cfg);
	for (i = 1; i < master->ncontrols; i++)
		v4l2_ctrl_activate(cluster[i], master->val);
}

static void cluster_activate_only(struct vinc_dev *priv,
				  struct v4l2_ctrl **cluster)
{
	struct v4l2_ctrl *master = cluster[0];
	int i;

	for (i = 1; i < master->ncontrols; i++)
		v4l2_ctrl_activate(cluster[i], master->val);
}

static void enable_block(struct vinc_dev *priv, u8 devnum, u32 block_mask,
			 bool const enable)
{
	u32 proc_cfg = vinc_read(priv, STREAM_PROC_CFG(devnum));

	if (enable)
		proc_cfg |= block_mask;
	else
		proc_cfg &= ~block_mask;

	vinc_write(priv, STREAM_PROC_CFG(devnum), proc_cfg);
}

static int vinc_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct soc_camera_device *icd = container_of(ctrl->handler,
			struct soc_camera_device, ctrl_handler);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct vinc_dev *priv = ici->priv;
	struct vinc_cluster_bp *bp;
	struct vinc_cluster_gamma *gamma;
	struct vinc_cluster_cc *cc;
	struct vinc_cluster_ct *ct;
	struct vinc_cluster_dr *dr;
	struct vinc_cluster_stat *stat;
	const u8 devnum = icd->devnum;
	struct vinc_stream * const stream = &priv->stream[devnum];
	u32 proc_cfg, stream_ctr;
	int i, init, std_is_new;

	switch (ctrl->id) {
	case V4L2_CID_BAD_CORRECTION_ENABLE:
		bp = (struct vinc_cluster_bp *)ctrl->cluster;
		/* For programming BP block we need to stop video */
		stream_ctr = vinc_read(priv, STREAM_CTR);
		vinc_write(priv, STREAM_CTR,
			   stream_ctr & ~STREAM_CTR_STREAM_ENABLE(devnum));
		cluster_activate(priv, devnum, STREAM_PROC_CFG_BPC_EN,
				 ctrl->cluster);
		if (bp->enable->val) {
			if (bp->enable->is_new || bp->pix->is_new)
				set_bad_pixels(priv, devnum, bp->pix->p_new.p);
			if (bp->enable->is_new || bp->row->is_new)
				set_bad_rows_cols(priv, devnum,
						  bp->row->p_new.p_u16, 0);
			if (bp->enable->is_new || bp->col->is_new)
				set_bad_rows_cols(priv, devnum,
						  bp->col->p_new.p_u16, 1);
		}
		vinc_write(priv, STREAM_CTR, stream_ctr);
		break;
	case V4L2_CID_GAMMA_CURVE_ENABLE: {
		struct vinc_gamma_curve *p_gamma;

		gamma = (struct vinc_cluster_gamma *)ctrl->cluster;
		p_gamma = gamma->curve->p_cur.p;
		if (gamma->gamma->is_new) {
			kernel_neon_begin();
			vinc_neon_calculate_gamma_curve(gamma->gamma->val,
					      gamma->curve->p_cur.p);
			kernel_neon_end();
			gamma->gamma->flags &= ~V4L2_CTRL_FLAG_WRITE_ONLY;
			gamma->curve->flags |= V4L2_CTRL_FLAG_UPDATE;
		} else if (gamma->curve->is_new) {
			gamma->gamma->flags |= V4L2_CTRL_FLAG_WRITE_ONLY;
			gamma->curve->flags &= ~V4L2_CTRL_FLAG_UPDATE;
			p_gamma = gamma->curve->p_new.p;
		}

		if (gamma->enable->is_new)
			cluster_activate_only(priv, ctrl->cluster);

		if (gamma->enable->val && (gamma->gamma->val != 16 ||
			gamma->gamma->flags & V4L2_CTRL_FLAG_WRITE_ONLY)) {
			enable_block(priv, devnum, STREAM_PROC_CFG_GC_EN, true);
			set_gc_curve(priv, devnum, p_gamma);
		} else {
			enable_block(priv, devnum, STREAM_PROC_CFG_GC_EN,
				     false);
		}
		break;
	}
	case V4L2_CID_CC_ENABLE: {
		struct vinc_cc *p_cc;
		cc = (struct vinc_cluster_cc *)ctrl->cluster;

		p_cc = cc->cc->p_cur.p;
		/*TODO: is_new flags for other cc controls must be
		added to std_is_new condition */
		std_is_new = cc->dowb->is_new     | cc->brightness->is_new |
			     cc->contrast->is_new | cc->saturation->is_new |
			     cc->hue->is_new;
		init = cc->enable->is_new & cc->cc->is_new &
			std_is_new;
		cluster_activate(priv, devnum, STREAM_PROC_CFG_CC_EN,
				 ctrl->cluster);
		if (cc->dowb->is_new && !init) {
			struct vinc_stat_add *add;

			add = stream->cluster.stat.add[3]->p_cur.p;
			kernel_neon_begin();
			vinc_neon_calculate_m_wb(add->sum_r,
				add->sum_g, add->sum_b, cc->dowb->priv);
			kernel_neon_end();
		}
		if (cc->brightness->is_new) {
			kernel_neon_begin();
			vinc_neon_calculate_v_bri(cc->brightness->priv,
						cc->brightness->val);
			kernel_neon_end();
		}
		if (cc->contrast->is_new) {
			kernel_neon_begin();
			vinc_neon_calculate_m_con(cc->contrast->priv,
						  cc->contrast->val);
			kernel_neon_end();
		}
		if (cc->saturation->is_new) {
			kernel_neon_begin();
			vinc_neon_calculate_m_sat(cc->saturation->priv,
						  cc->saturation->val);
			kernel_neon_end();
		}
		if (cc->hue->is_new) {
			kernel_neon_begin();
			vinc_neon_calculate_m_hue(cc->hue->priv, cc->hue->val);
			kernel_neon_end();
		}
		if (std_is_new) {
			struct ctrl_priv ctrl_privs = {
				.dowb = cc->dowb->priv,
				.brightness = cc->brightness->priv,
				.contrast = cc->contrast->priv,
				.saturation = cc->saturation->priv,
				.hue = cc->hue->priv
			};
			kernel_neon_begin();
			vinc_neon_calculate_cc(&ctrl_privs, stream->ycbcr_enc,
					       cc->cc->p_cur.p);
			kernel_neon_end();

			cc->brightness->flags &= ~V4L2_CTRL_FLAG_WRITE_ONLY &
					~V4L2_CTRL_FLAG_EXECUTE_ON_WRITE;
			cc->contrast->flags &= ~V4L2_CTRL_FLAG_WRITE_ONLY &
					~V4L2_CTRL_FLAG_EXECUTE_ON_WRITE;
			cc->saturation->flags &= ~V4L2_CTRL_FLAG_WRITE_ONLY &
					~V4L2_CTRL_FLAG_EXECUTE_ON_WRITE;
			cc->hue->flags &= ~V4L2_CTRL_FLAG_WRITE_ONLY &
					~V4L2_CTRL_FLAG_EXECUTE_ON_WRITE;
			cc->cc->flags |= V4L2_CTRL_FLAG_UPDATE;
		} else if (cc->cc->is_new) {
			p_cc = cc->cc->p_new.p;

			cc->brightness->flags |= V4L2_CTRL_FLAG_WRITE_ONLY |
					V4L2_CTRL_FLAG_EXECUTE_ON_WRITE;
			cc->contrast->flags |= V4L2_CTRL_FLAG_WRITE_ONLY |
					V4L2_CTRL_FLAG_EXECUTE_ON_WRITE;
			cc->saturation->flags |= V4L2_CTRL_FLAG_WRITE_ONLY |
					V4L2_CTRL_FLAG_EXECUTE_ON_WRITE;
			cc->hue->flags |= V4L2_CTRL_FLAG_WRITE_ONLY |
					V4L2_CTRL_FLAG_EXECUTE_ON_WRITE;
			cc->cc->flags &= ~V4L2_CTRL_FLAG_UPDATE;
		}
		set_cc_ct(priv, devnum, p_cc, 0);
		break;
	}
	case V4L2_CID_CT_ENABLE:
		ct = (struct vinc_cluster_ct *)ctrl->cluster;
		proc_cfg = vinc_read(priv, STREAM_PROC_CFG(devnum));
		if (ctrl->val)
			proc_cfg |= STREAM_PROC_CFG_CT_EN;
		else
			proc_cfg &= ~STREAM_PROC_CFG_CT_EN;
		proc_cfg &= ~STREAM_PROC_CFG_DMA0_SRC(DMA_SRC_MASK);
		proc_cfg |= STREAM_PROC_CFG_DMA0_SRC(
				vinc_get_dma_src(priv, icd));
		vinc_write(priv, STREAM_PROC_CFG(devnum), proc_cfg);

		v4l2_ctrl_activate(ct->ct, ct->enable->val);
		if (ct->ct->is_new)
			set_cc_ct(priv, devnum, ct->ct->p_new.p, 1);
		break;
	case V4L2_CID_DR_ENABLE:
		dr = (struct vinc_cluster_dr *)ctrl->cluster;
		/* To enable/disable DR block we need to stop video */
		stream_ctr = vinc_read(priv, STREAM_CTR);
		vinc_write(priv, STREAM_CTR,
			   stream_ctr & ~STREAM_CTR_STREAM_ENABLE(devnum));
		cluster_activate(priv, devnum, STREAM_PROC_CFG_ADR_EN,
				 ctrl->cluster);
		vinc_write(priv, STREAM_CTR, stream_ctr);
		if (dr->enable->val && (dr->enable->is_new || dr->dr->is_new))
			set_dr(priv, devnum, dr->dr->p_new.p_u16);
		break;
	case V4L2_CID_STAT_ENABLE:
		stat = (struct vinc_cluster_stat *)ctrl->cluster;
		if (stat->enable->is_new) {
			stream_ctr = vinc_read(priv, STREAM_CTR);
			vinc_write(priv, STREAM_CTR,
				   stream_ctr & ~STREAM_CTR_STREAM_ENABLE(
						   devnum));
			proc_cfg = vinc_read(priv, STREAM_PROC_CFG(devnum));
			proc_cfg &= ~STREAM_PROC_CFG_STT_EN(0x7);
			proc_cfg |= STREAM_PROC_CFG_STT_EN(stat->enable->val);
			if (stat->enable->val)
				vinc_stat_start(stream);
			vinc_write(priv, STREAM_PROC_CFG(devnum), proc_cfg);
			vinc_write(priv, STREAM_CTR, stream_ctr);
		}
		set_stat_af_color(priv, devnum, stat->af_color->val);
		vinc_write(priv, STREAM_PROC_STAT_TH(devnum),
			   stat->af_th->val);
		for (i = 0; i < 4; i++) {
			if (!stat->zone[i]->is_new)
				continue;
			stream_ctr = vinc_read(priv, STREAM_CTR);
			vinc_write(priv, STREAM_CTR,
				   stream_ctr & ~STREAM_CTR_STREAM_ENABLE(
						   devnum));
			set_stat_zone(stream,
				      stat->zone[i]->id - V4L2_CID_STAT_ZONE0,
				      stat->zone[i]->p_new.p);
			vinc_write(priv, STREAM_CTR, stream_ctr);
		}
		break;
	case V4L2_CID_TEST_PATTERN:
		stream_ctr = vinc_read(priv, STREAM_CTR);
		vinc_write(priv, STREAM_CTR,
			   stream_ctr & ~STREAM_CTR_STREAM_ENABLE(devnum));
		proc_cfg = vinc_read(priv, STREAM_PROC_CFG(0));
		if (stream->input_format == BAYER && !ctrl->val)
			proc_cfg |= STREAM_PROC_CFG_CFA_EN;
		else
			proc_cfg &= ~STREAM_PROC_CFG_CFA_EN;
		vinc_configure_input(stream);
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
	struct vinc_cluster_bp *bp;
	struct vinc_cluster_gamma *gamma;
	struct vinc_cluster_stat *stat;
	struct vinc_gamma_curve *gc;
	struct vinc_stat_zone *zone;
	int i;

	switch (ctrl->id) {
	case V4L2_CID_BAD_CORRECTION_ENABLE:
		bp = (struct vinc_cluster_bp *)ctrl->cluster;
		if (bp->pix->is_new) {
			struct vinc_bad_pixel *pixel = bp->pix->p_new.p;

			for (i = 0; i < CTRL_BAD_PIXELS_COUNT; i++) {
				if (((pixel[i].x > MAX_WIDTH_HEIGHT) ||
				     (pixel[i].y > MAX_WIDTH_HEIGHT)) &&
				     (pixel[i].x != 0xFFFF) &&
				     (pixel[i].y != 0xFFFF))
					return -ERANGE;
			}
		}
		if (bp->row->is_new) {
			u16 *row = bp->row->p_new.p_u16;

			for (i = 0; i < CTRL_BAD_ROWSCOLS_COUNT; i++) {
				if (row[i] > MAX_WIDTH_HEIGHT &&
				    row[i] != 0xFFFF)
					return -ERANGE;
			}
		}
		if (bp->col->is_new) {
			u16 *col = bp->col->p_new.p_u16;

			for (i = 0; i < CTRL_BAD_ROWSCOLS_COUNT; i++) {
				if (col[i] > MAX_WIDTH_HEIGHT &&
				    col[i] != 0xFFFF)
					return -ERANGE;
			}
		}
		return 0;
	case V4L2_CID_GAMMA_CURVE_ENABLE:
		gamma = (struct vinc_cluster_gamma *)ctrl->cluster;
		if (gamma->curve->is_new) {
			gc = gamma->curve->p_new.p;
			for (i = 0; i < CTRL_GC_ELEMENTS_COUNT; i++) {
				if (gc->red[i] > MAX_COMP_VALUE ||
				    gc->green[i] > MAX_COMP_VALUE ||
				    gc->blue[i] > MAX_COMP_VALUE)
					return -ERANGE;
			}
		}
		return 0;
	case V4L2_CID_STAT_ENABLE:
		stat = (struct vinc_cluster_stat *)ctrl->cluster;
		if ((u32)stat->enable->val > 7)
			return -ERANGE;

		for (i = 0; i < 4; i++) {
			zone = stat->zone[i]->p_new.p;
			if (!stat->zone[i]->is_new || !zone->enable)
				continue;
			if (zone->x_lt > MAX_WIDTH_HEIGHT ||
					zone->y_lt > MAX_WIDTH_HEIGHT ||
					zone->x_rb > MAX_WIDTH_HEIGHT ||
					zone->y_rb > MAX_WIDTH_HEIGHT ||
					zone->x_lt >= zone->x_rb ||
					zone->y_lt >= zone->y_rb)
				return -ERANGE;
		}
		return 0;
	case V4L2_CID_CT_ENABLE:
	case V4L2_CID_DR_ENABLE:
	case V4L2_CID_CC_ENABLE:
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

static struct v4l2_ctrl_config ctrl_cfg[] = {
	{
		.ops   = &ctrl_ops,
		.id    = V4L2_CID_BRIGHTNESS,
		.name  = "Brightness",
		.type  = V4L2_CTRL_TYPE_INTEGER,
		.min   = -2048,
		.max   =  2048,
		.step  =  1,
		.def   =  0,
		.flags =  V4L2_CTRL_FLAG_UPDATE
	},
	{
		.ops   = &ctrl_ops,
		.id    = V4L2_CID_CONTRAST,
		.name  = "Contrast",
		.type  = V4L2_CTRL_TYPE_INTEGER,
		.min   = 0,
		.max   = 255,
		.step  = 1,
		.def   = 128,
		.flags = V4L2_CTRL_FLAG_UPDATE
	},
	{
		.ops   = &ctrl_ops,
		.id    = V4L2_CID_SATURATION,
		.name  = "Saturation",
		.type  = V4L2_CTRL_TYPE_INTEGER,
		.min   = 0,
		.max   = 255,
		.step  = 1,
		.def   = 128,
		.flags = V4L2_CTRL_FLAG_UPDATE
	},
	{
		.ops   = &ctrl_ops,
		.id    = V4L2_CID_HUE,
		.name  = "Hue",
		.type  = V4L2_CTRL_TYPE_INTEGER,
		.min   = -128,
		.max   =  127,
		.step  =  1,
		.def   =  0,
		.flags = V4L2_CTRL_FLAG_UPDATE
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_DO_WHITE_BALANCE,
		.name = "Do white balance",
		.type = V4L2_CTRL_TYPE_BUTTON,
		.flags = V4L2_CTRL_FLAG_UPDATE
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_GAMMA,
		.name = "Gamma",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 1,
		.max = 31,
		.step = 1,
		.def = 16,
		.flags = V4L2_CTRL_FLAG_UPDATE | V4L2_CTRL_FLAG_EXECUTE_ON_WRITE
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_BAD_CORRECTION_ENABLE,
		.name = "Bad pixels/rows/columns repair enable",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = 0,
		.max = 1,
		.step = 1,
		.def = 0,
		.flags = 0
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_BAD_PIXELS,
		.name = "Bad pixels",
		.type = V4L2_CTRL_COMPOUND_TYPES,
		.min = 0,
		.max = 0xFF,
		.step = 1,
		.def = 0xFF,
		.dims[0] = sizeof(struct vinc_bad_pixel) *
				CTRL_BAD_PIXELS_COUNT,
		.flags = V4L2_CTRL_FLAG_HAS_PAYLOAD
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_BAD_ROWS,
		.name = "Bad rows",
		.type = V4L2_CTRL_TYPE_U16,
		.min = 0,
		.max = 0xFFF,
		.step = 1,
		.def = 0xFFF,
		.dims[0] = CTRL_BAD_ROWSCOLS_COUNT,
		.flags = V4L2_CTRL_FLAG_HAS_PAYLOAD
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_BAD_COLS,
		.name = "Bad columns",
		.type = V4L2_CTRL_TYPE_U16,
		.min = 0,
		.max = 0xFFF,
		.step = 1,
		.def = 0xFFF,
		.dims[0] = CTRL_BAD_ROWSCOLS_COUNT,
		.flags = V4L2_CTRL_FLAG_HAS_PAYLOAD
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_GAMMA_CURVE_ENABLE,
		.name = "Gamma curve enable",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = 0,
		.max = 1,
		.step = 1,
		.def = 1,
		.flags = V4L2_CTRL_FLAG_UPDATE
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_GAMMA_CURVE,
		.name = "Gamma curve",
		.type = V4L2_CTRL_COMPOUND_TYPES,
		.min = 0,
		.max = 0xFF,
		.step = 1,
		.def = 0,
		.dims[0] = sizeof(struct vinc_gamma_curve),
		.flags = V4L2_CTRL_FLAG_HAS_PAYLOAD | V4L2_CTRL_FLAG_UPDATE
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_CC_ENABLE,
		.name = "Color correction enable",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = 0,
		.max = 1,
		.step = 1,
		.def = 1,
		.flags = V4L2_CTRL_FLAG_UPDATE
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_CC,
		.name = "Color correction",
		.type = V4L2_CTRL_COMPOUND_TYPES,
		.min = 0,
		.max = 0xFF,
		.step = 1,
		.def = 0,
		.dims[0] = sizeof(struct vinc_cc),
		.flags = V4L2_CTRL_FLAG_HAS_PAYLOAD | V4L2_CTRL_FLAG_UPDATE
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_CT_ENABLE,
		.name = "Color tranformation enable",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = 0,
		.max = 1,
		.step = 1,
		.def = 0,
		.flags = 0
	},
	{
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
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_DR_ENABLE,
		.name = "Dynamic range enable",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = 0,
		.max = 1,
		.step = 1,
		.def = 0,
		.flags = 0
	},
	{
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
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_STAT_ENABLE,
		.name = "Statistics enable",
		.type = V4L2_CTRL_TYPE_BITMASK,
		.min = 0,
		.max = 0x7,
		.step = 0,
		.def = 0,
		.flags = 0
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_STAT_AF_COLOR,
		.name = "Autofocus component",
		.type = V4L2_CTRL_TYPE_MENU,
		.min = 0,
		.max = ARRAY_SIZE(vinc_af_color_menu) - 1,
		.step = 0,
		.def = 0,
		.qmenu = vinc_af_color_menu
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_STAT_AF_TH,
		.name = "Autofocus threshold",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 0x7FF,
		.step = 1,
		.def = 0,
		.flags = 0
	},
	{
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
	},
	{
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
	},
	{
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
	},
	{
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
	},
	{
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
	},
	{
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
	},
	{
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
	},
	{
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
	},
	{
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
	},
	{
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
	},
	{
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
	},
	{
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
	},
	{
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
	},
	{
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
	},
	{
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
	},
	{
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
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_TEST_PATTERN,
		.type = V4L2_CTRL_TYPE_MENU,
		.min = 0,
		.max = ARRAY_SIZE(vinc_test_pattern_menu) - 1,
		.step = 0,
		.def = 0,
		.qmenu = vinc_test_pattern_menu
	},
};

static int vinc_create_controls(struct v4l2_ctrl_handler *hdl,
				struct vinc_stream *stream)
{
	int i;
	struct v4l2_ctrl *tmp_ctrl;
	const u8 devnum = stream->devnum;
	struct vinc_dev *priv = container_of(stream, struct vinc_dev,
					     stream[devnum]);

	for (i = 0; i < ARRAY_SIZE(ctrl_cfg); i++) {
		tmp_ctrl = v4l2_ctrl_new_custom(hdl, &ctrl_cfg[i], NULL);
		if (!tmp_ctrl) {
			dev_err(priv->ici.v4l2_dev.dev,
				"Can not create control %#x\n", ctrl_cfg[i].id);
			return hdl->error;
		}
	}

	stream->cluster.bp.enable = v4l2_ctrl_find(hdl,
			V4L2_CID_BAD_CORRECTION_ENABLE);
	stream->cluster.bp.pix = v4l2_ctrl_find(hdl, V4L2_CID_BAD_PIXELS);
	stream->cluster.bp.row = v4l2_ctrl_find(hdl, V4L2_CID_BAD_ROWS);
	stream->cluster.bp.col = v4l2_ctrl_find(hdl, V4L2_CID_BAD_COLS);
	v4l2_ctrl_cluster(CLUSTER_SIZE(struct vinc_cluster_bp),
			  &stream->cluster.bp.enable);

	stream->cluster.gamma.enable = v4l2_ctrl_find(hdl,
			V4L2_CID_GAMMA_CURVE_ENABLE);
	stream->cluster.gamma.curve = v4l2_ctrl_find(hdl, V4L2_CID_GAMMA_CURVE);
	stream->cluster.gamma.gamma = v4l2_ctrl_find(hdl, V4L2_CID_GAMMA);
	v4l2_ctrl_cluster(CLUSTER_SIZE(struct vinc_cluster_gamma),
			  &stream->cluster.gamma.enable);

	stream->cluster.cc.enable = v4l2_ctrl_find(hdl, V4L2_CID_CC_ENABLE);
	stream->cluster.cc.cc = v4l2_ctrl_find(hdl, V4L2_CID_CC);
	stream->cluster.cc.brightness = v4l2_ctrl_find(hdl,
			V4L2_CID_BRIGHTNESS);
	stream->cluster.cc.contrast = v4l2_ctrl_find(hdl, V4L2_CID_CONTRAST);
	stream->cluster.cc.saturation = v4l2_ctrl_find(hdl,
						       V4L2_CID_SATURATION);
	stream->cluster.cc.hue = v4l2_ctrl_find(hdl, V4L2_CID_HUE);
	stream->cluster.cc.dowb = v4l2_ctrl_find(hdl,
			V4L2_CID_DO_WHITE_BALANCE);
	v4l2_ctrl_cluster(CLUSTER_SIZE(struct vinc_cluster_cc),
			  &stream->cluster.cc.enable);

	stream->cluster.ct.enable = v4l2_ctrl_find(hdl, V4L2_CID_CT_ENABLE);
	stream->cluster.ct.ct = v4l2_ctrl_find(hdl, V4L2_CID_CT);
	v4l2_ctrl_cluster(CLUSTER_SIZE(struct vinc_cluster_ct),
			  &stream->cluster.ct.enable);

	stream->cluster.dr.enable = v4l2_ctrl_find(hdl, V4L2_CID_DR_ENABLE);
	stream->cluster.dr.dr = v4l2_ctrl_find(hdl, V4L2_CID_DR);
	v4l2_ctrl_cluster(CLUSTER_SIZE(struct vinc_cluster_dr),
			  &stream->cluster.dr.enable);

	stream->cluster.stat.enable = v4l2_ctrl_find(hdl, V4L2_CID_STAT_ENABLE);
	stream->cluster.stat.af_color = v4l2_ctrl_find(hdl,
			V4L2_CID_STAT_AF_COLOR);
	stream->cluster.stat.af_th = v4l2_ctrl_find(hdl, V4L2_CID_STAT_AF_TH);
	for (i = 0; i < 4; i++) {
		stream->cluster.stat.zone[i] = v4l2_ctrl_find(hdl,
				V4L2_CID_STAT_ZONE0 + i);
		stream->cluster.stat.hist[i] = v4l2_ctrl_find(hdl,
				V4L2_CID_STAT_HIST0 + i);
		stream->cluster.stat.af[i] = v4l2_ctrl_find(hdl,
				V4L2_CID_STAT_AF0 + i);
		stream->cluster.stat.add[i] = v4l2_ctrl_find(hdl,
				V4L2_CID_STAT_ADD0 + i);
	}
	v4l2_ctrl_cluster(CLUSTER_SIZE(struct vinc_cluster_stat),
			  &stream->cluster.stat.enable);

	stream->test_pattern = v4l2_ctrl_find(hdl, V4L2_CID_TEST_PATTERN);
	stream->cluster.cc.brightness->priv = devm_kmalloc(
			priv->ici.v4l2_dev.dev,
			sizeof(struct vector), GFP_KERNEL);
	stream->cluster.cc.contrast->priv = devm_kmalloc(
			priv->ici.v4l2_dev.dev, sizeof(struct matrix),
			GFP_KERNEL);
	stream->cluster.cc.saturation->priv = devm_kmalloc(
			priv->ici.v4l2_dev.dev, sizeof(struct matrix),
			GFP_KERNEL);
	stream->cluster.cc.hue->priv = devm_kmalloc(
			priv->ici.v4l2_dev.dev, sizeof(struct matrix),
			GFP_KERNEL);
	stream->cluster.cc.dowb->priv = kzalloc(sizeof(struct matrix),
						GFP_KERNEL);
	kernel_neon_begin();
	vinc_neon_calculate_m_wb(1, 1, 1, stream->cluster.cc.dowb->priv);
	kernel_neon_end();
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
	const u8 devnum = icd->devnum;
	struct vinc_stream * const stream = &priv->stream[devnum];
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

		ret = vinc_create_controls(&icd->ctrl_handler, stream);
		if (ret)
			return ret;

		ret = v4l2_subdev_call(sd, video, g_mbus_config, &mbus_cfg);
		if (ret >= 0) {
			stream->video_source = mbus_cfg.type;

			if (mbus_cfg.type != V4L2_MBUS_CSI2 &&
					mbus_cfg.type != V4L2_MBUS_PARALLEL) {
				dev_err(dev,
					"Interface type %d is not supported\n",
					mbus_cfg.type);
				return -EINVAL;
			}

			if (mbus_cfg.flags & V4L2_MBUS_CSI2_4_LANE)
				stream->csi2_lanes = 4;
			else if (mbus_cfg.flags & V4L2_MBUS_CSI2_3_LANE)
				stream->csi2_lanes = 3;
			else if (mbus_cfg.flags & V4L2_MBUS_CSI2_2_LANE)
				stream->csi2_lanes = 2;
			else if (mbus_cfg.flags & V4L2_MBUS_CSI2_1_LANE)
				stream->csi2_lanes = 1;
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

static void color_space_adjust(u32 *colorspace, u32 *ycbcr_enc,
				u32 *quantization)
{
	switch (*colorspace) {
	case V4L2_COLORSPACE_REC709:
	case V4L2_COLORSPACE_BT2020:
	case V4L2_COLORSPACE_SRGB:
		break;
	/* SMPTE 170M */
	default:
		*colorspace = V4L2_COLORSPACE_SMPTE170M;
		break;
	}

	switch (*ycbcr_enc) {
	case V4L2_YCBCR_ENC_601:
	case V4L2_YCBCR_ENC_709:
	case V4L2_YCBCR_ENC_BT2020:
	case V4L2_YCBCR_ENC_SYCC:
		break;
	default:
		switch (*colorspace) {
		case V4L2_COLORSPACE_REC709:
			*ycbcr_enc = V4L2_YCBCR_ENC_709;
			break;
		case V4L2_COLORSPACE_BT2020:
			*ycbcr_enc = V4L2_YCBCR_ENC_BT2020;
			break;
		case V4L2_COLORSPACE_SRGB:
			*ycbcr_enc = V4L2_YCBCR_ENC_SYCC;
			break;
		default:
			*ycbcr_enc = V4L2_YCBCR_ENC_601;
			break;
		}
		break;
	}

	switch (*quantization) {
	case V4L2_QUANTIZATION_FULL_RANGE:
	case V4L2_QUANTIZATION_LIM_RANGE:
		break;
	default:
		switch (*colorspace) {
		case V4L2_COLORSPACE_SRGB:
			*quantization = V4L2_QUANTIZATION_FULL_RANGE;
			break;
		default:
			*quantization = V4L2_QUANTIZATION_LIM_RANGE;
			break;
		}
		break;
	}
}

static int __vinc_try_fmt(struct soc_camera_device *icd, struct v4l2_format *f,
			  struct v4l2_mbus_framefmt *mbus_fmt)
{
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);

	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct vinc_dev *priv = ici->priv;

	struct soc_mbus_pixelfmt *pixelfmt;
	const struct soc_camera_format_xlate *xlate;
	const u8 devnum = icd->devnum;
	u32 width, height;
	u32 min_bytesperline;
	int ret;

	pix->field = V4L2_FIELD_NONE;

	color_space_adjust(&pix->colorspace, &pix->ycbcr_enc,
			   &pix->quantization);
	priv->stream[devnum].ycbcr_enc = pix->ycbcr_enc;
	priv->stream[devnum].quantization = pix->quantization;

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

	if (pix->bytesperline > 0xFFF8)
		pix->bytesperline = 0xFFF8;

	pix->bytesperline &= ~0x7;
	min_bytesperline = (pix->width * pixelfmt->bits_per_sample) / 8;
	if (pix->bytesperline < min_bytesperline)
		pix->bytesperline = min_bytesperline;

	switch (pixelfmt->fourcc) {
	case V4L2_PIX_FMT_M420:
		pix->sizeimage = pix->bytesperline * ((pix->height * 3) / 2);
		break;
	default:
		pix->sizeimage = pix->bytesperline * pix->height;
		break;
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
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct v4l2_mbus_framefmt mbus_fmt;
	const struct soc_camera_format_xlate *xlate;
	const u8 devnum = icd->devnum;
	struct vinc_stream * const stream = &priv->stream[devnum];
	int ret;
	int offset_x, offset_y;

	ret = __vinc_try_fmt(icd, f, &mbus_fmt);
	if (ret)
		return ret;

	switch (mbus_fmt.code) {
	case MEDIA_BUS_FMT_SRGGB8_1X8:
	case MEDIA_BUS_FMT_SRGGB10_1X10:
	case MEDIA_BUS_FMT_SRGGB12_1X12:
		stream->input_format = BAYER;
		stream->bayer_mode = 0;
		break;
	case MEDIA_BUS_FMT_SGRBG8_1X8:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
		stream->input_format = BAYER;
		stream->bayer_mode = 1;
		break;
	case MEDIA_BUS_FMT_SGBRG8_1X8:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
		stream->input_format = BAYER;
		stream->bayer_mode = 2;
		break;
	case MEDIA_BUS_FMT_SBGGR8_1X8:
	case MEDIA_BUS_FMT_SBGGR10_1X10:
	case MEDIA_BUS_FMT_SBGGR12_1X12:
		stream->input_format = BAYER;
		stream->bayer_mode = 3;
		break;
	default:
		stream->input_format = UNKNOWN;
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

	stream->crop1.c.width = pix->width + offset_x;
	stream->crop1.c.height = pix->height + offset_y;
	stream->crop1.c.left = 0;
	stream->crop1.c.top = 0;

	stream->crop2.c.width = pix->width;
	stream->crop2.c.height = pix->height;
	stream->crop2.c.left = offset_x;
	stream->crop2.c.top = offset_y;

	dev_dbg(icd->parent, "crop1: %dx%d (%d,%d)\n",
		stream->crop1.c.width, stream->crop1.c.height,
		stream->crop1.c.left, stream->crop1.c.top);
	dev_dbg(icd->parent, "crop2: %dx%d (%d,%d)\n",
		stream->crop2.c.width, stream->crop2.c.height,
		stream->crop2.c.left, stream->crop2.c.top);

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

static void vinc_configure_bgr(struct vinc_dev *priv,
			       struct soc_camera_device *icd)
{
	const u8 devnum = icd->devnum;

	v4l2_ctrl_s_ctrl(priv->stream[devnum].cluster.ct.enable, 0);

	vinc_write(priv, STREAM_DMA_FBUF_CFG(devnum, 0), 0x00001);
	vinc_write(priv, STREAM_DMA_FBUF_LSTEP(devnum, 0, 0),
		   icd->bytesperline);
	vinc_write(priv, STREAM_DMA_FBUF_FSTEP(devnum, 0, 0), icd->sizeimage);
	vinc_write(priv, STREAM_DMA_PIXEL_FMT(devnum, 0),
		   STREAM_DMA_PIXEL_FMT_PLANES(PLANES_SINGLE) |
		   STREAM_DMA_PIXEL_FMT_FORMAT(FORMAT_BGR));
}

static void vinc_configure_m420(struct vinc_dev *priv,
				struct soc_camera_device *icd)
{
	u32 proc_cfg;
	const u8 devnum = icd->devnum;
	struct vinc_stream * const stream = &priv->stream[devnum];
	struct vinc_cc *ct = stream->cluster.ct.ct->p_cur.p;
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
	set_cc_ct(priv, devnum, ct, 1);
	v4l2_ctrl_s_ctrl(stream->cluster.ct.enable, 1);

	proc_cfg = vinc_read(priv, STREAM_PROC_CFG(devnum));
	proc_cfg |= STREAM_PROC_CFG_CT_EN |
			STREAM_PROC_CFG_444TO422_EN |
			STREAM_PROC_CFG_422TO420_EN |
			STREAM_PROC_CFG_444TO422_SRC(1) |
			STREAM_PROC_CFG_422TO420_SRC(1);
	vinc_write(priv, STREAM_PROC_CFG(devnum), proc_cfg);

	vinc_write(priv, STREAM_DMA_FBUF_CFG(devnum, 0), 0x00001);
	for (i = 0; i < 2; i++) {
		vinc_write(priv, STREAM_DMA_FBUF_LSTEP(devnum, 0, i),
			   icd->bytesperline | BIT(31));
		vinc_write(priv, STREAM_DMA_FBUF_FSTEP(devnum, 0, i),
			   icd->sizeimage);
	}
	vinc_write(priv, STREAM_DMA_PIXEL_FMT(devnum, 0),
		   STREAM_DMA_PIXEL_FMT_PLANES(PLANES_DUAL) |
		   STREAM_DMA_PIXEL_FMT_FORMAT(FORMAT_420));
}

static void vinc_configure(struct vinc_dev *priv, struct soc_camera_device *icd)
{
	u32 axi_master_cfg, proc_ctr, proc_cfg;
	struct vinc_stat_zone *zone;
	const u8 devnum = icd->devnum;
	struct vinc_stream * const stream = &priv->stream[devnum];

	vinc_write(priv, STREAM_INP_HCROP_CTR(devnum),
		   (stream->crop1.c.width << 16) | stream->crop1.c.left);
	vinc_write(priv, STREAM_INP_VCROP_CTR(devnum),
		   (stream->crop1.c.height << 16) | stream->crop1.c.top);
	vinc_write(priv, STREAM_INP_VCROP_ODD_CTR(devnum), 0);
	vinc_write(priv, STREAM_INP_DECIM_CTR(devnum),
		   STREAM_INP_DECIM_FDECIM(stream->fdecim - 1));

	zone = stream->cluster.stat.zone[3]->p_cur.p;
	zone->enable = 1;
	zone->x_lt = stream->crop1.c.left;
	zone->y_lt = stream->crop1.c.top;
	zone->x_rb = stream->crop1.c.left + stream->crop1.c.width - 1;
	zone->y_rb = stream->crop1.c.top + stream->crop1.c.height - 1;
	stream->cluster.stat.enable->val = 0x4;

	proc_cfg = vinc_read(priv, STREAM_PROC_CFG(devnum));
	proc_cfg |= STREAM_PROC_CFG_STT_EN(stream->cluster.stat.enable->val);
	vinc_write(priv, STREAM_PROC_CFG(devnum), proc_cfg);
	set_stat_zone(stream, 3, stream->cluster.stat.zone[3]->p_cur.p);

	proc_ctr = STREAM_PROC_CTR_BAYER_MODE(stream->bayer_mode);
	proc_ctr |= STREAM_PROC_CTR_AF_COLOR(
			stream->cluster.stat.af_color->val);
	proc_ctr |= STREAM_PROC_CTR_HIST_THR |
			STREAM_PROC_CTR_AF_THR |
			STREAM_PROC_CTR_ADD_THR;
	vinc_write(priv, STREAM_PROC_CTR(devnum), proc_ctr);
	if (stream->cluster.stat.enable->val)
		vinc_stat_start(stream);

	vinc_write(priv, STREAM_DMA_FBUF_HORIZ(devnum, 0),
		   (stream->crop2.c.width << 16) | stream->crop2.c.left);
	vinc_write(priv, STREAM_DMA_FBUF_VERT(devnum, 0),
		   (stream->crop2.c.height << 16) | stream->crop2.c.top);
	vinc_write(priv, STREAM_DMA_FBUF_DECIM(devnum, 0), 0x10000);

	switch (icd->current_fmt->host_fmt->fourcc) {
	case V4L2_PIX_FMT_BGR32:
		vinc_configure_bgr(priv, icd);
		break;
	case V4L2_PIX_FMT_M420:
		vinc_configure_m420(priv, icd);
		break;
	default:
		dev_warn(priv->ici.v4l2_dev.dev, "Unknown output format %#x\n",
			 icd->current_fmt->host_fmt->fourcc);
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

	vinc_configure(priv, icd);

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
	int ret;
	struct v4l2_streamparm sensor_parm = {
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
	};
	struct v4l2_subdev_frame_interval sensor_interval = {0};
	struct v4l2_fract *tpf = &parm->parm.capture.timeperframe;
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct vinc_dev *priv = ici->priv;

	parm->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
	parm->parm.capture.capturemode = 0;
	parm->parm.capture.extendedmode = 0;
	tpf->denominator = 0;
	tpf->numerator = 0;

	ret = v4l2_subdev_call(sd, video, g_parm, &sensor_parm);
	if (!ret && (sensor_parm.parm.capture.capability &
			V4L2_CAP_TIMEPERFRAME))
		*tpf = sensor_parm.parm.capture.timeperframe;

	if (!tpf->denominator || !tpf->numerator) {
		ret = v4l2_subdev_call(sd, video, g_frame_interval,
				       &sensor_interval);
		if (!ret)
			*tpf = sensor_interval.interval;
	}

	if (!tpf->denominator || !tpf->numerator) {
		dev_notice(icd->parent,
			   "Can not get framerate from sensor. Fallback to 30 FPS.\n");
		tpf->denominator = 30;
		tpf->numerator = 1;
	}
	tpf->numerator *= priv->stream[icd->devnum].fdecim;
	/* TODO: Add fraction reduction */

	return 0;
}

static int vinc_set_parm(struct soc_camera_device *icd,
			 struct v4l2_streamparm *parm)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct vinc_dev *priv = ici->priv;
	struct v4l2_fract tpf, *current_tpf = &parm->parm.capture.timeperframe;
	u32 decim_ctr = vinc_read(priv, STREAM_INP_DECIM_CTR(0));
	const u8 devnum = icd->devnum;
	struct vinc_stream * const stream = &priv->stream[devnum];

	if (parm->parm.capture.extendedmode)
		return -EINVAL;

	stream->fdecim = 1;
	tpf = parm->parm.capture.timeperframe;
	vinc_get_parm(icd, parm);

	/*
	 * Here we have:
	 * tpf - timeperframe requested by user (can contain zeros)
	 * current_tpf - timeperframe returned by sensor (can not contain zeros)
	 */
	if (tpf.denominator)
		stream->fdecim = (tpf.numerator * current_tpf->denominator) /
				(tpf.denominator * current_tpf->numerator);

	stream->fdecim = clamp_val(stream->fdecim, 1, 64);

	current_tpf->numerator *= stream->fdecim;
	/* TODO: Add fraction reduction */

	decim_ctr &= ~STREAM_INP_DECIM_FDECIM(0x3F);
	decim_ctr |= STREAM_INP_DECIM_FDECIM(stream->fdecim - 1);
	vinc_write(priv, STREAM_INP_DECIM_CTR(0), decim_ctr);
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
	.set_parm       = vinc_set_parm,
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

static void vinc_next_buffer(struct vinc_stream *stream,
			     enum vb2_buffer_state state)
{
	struct vb2_buffer *vb = stream->active;
	struct vinc_dev *priv = container_of(stream, struct vinc_dev,
					     stream[stream->devnum]);

	if (!vb)
		return;
	spin_lock(&stream->lock);
	list_del_init(&to_vinc_vb(vb)->queue);

	if (!list_empty(&stream->capture))
		stream->active = &list_entry(stream->capture.next,
					     struct vinc_buffer, queue)->vb;
	else
		stream->active = NULL;

	vinc_start_capture(priv, priv->ici.icds[stream->devnum]);
	v4l2_get_timestamp(&vb->v4l2_buf.timestamp);

	vb->v4l2_buf.sequence = stream->sequence++;

	vb2_buffer_done(vb, state);
	spin_unlock(&stream->lock);
}

static void vinc_stat_tasklet(unsigned long data)
{
	struct vinc_stream *stream = (struct vinc_stream *)data;
	const u8 devnum = stream->devnum;
	struct vinc_dev *priv = container_of(stream, struct vinc_dev,
					     stream[devnum]);
	struct vinc_stat_zone *zone;
	struct vinc_stat_hist *hist;
	struct vinc_stat_af *af;
	struct vinc_stat_add *add;
	u32 stat_en;
	u32 reg;
	int z, i;

	stat_en = stream->cluster.stat.enable->val;
	if (!stat_en)
		return;

	for (z = 0; z < 4; z++) {
		zone = stream->cluster.stat.zone[z]->p_cur.p;
		if (!zone->enable)
			continue;

		hist = stream->cluster.stat.hist[z]->p_cur.p;
		if (stat_en & STT_EN_HIST) {
			__u32 *component[3] = { hist->red, hist->green,
						hist->blue };
			int c;

			for (c = 0; c < ARRAY_SIZE(component); c++) {
				vinc_write(priv, STREAM_PROC_STAT_CTR(devnum),
					   STREAM_PROC_STAT_CTR_NUM_ZONE(z) |
					   STREAM_PROC_STAT_CTR_COLOR_HIST(c));
				for (i = 0; i < VINC_STAT_HIST_COUNT; i++)
					component[c][i] = vinc_read(priv,
						STREAM_PROC_STAT_DATA(devnum));
			}
		}
		if (stat_en & STT_EN_AF) {
			af = stream->cluster.stat.af[z]->p_cur.p;
			vinc_write(priv, STREAM_PROC_STAT_CTR(devnum),
				   STREAM_PROC_STAT_CTR_NUM_ZONE(z));
			af->hsobel = vinc_read(priv,
					STREAM_PROC_STAT_HSOBEL(devnum));
			af->vsobel = vinc_read(priv,
					STREAM_PROC_STAT_VSOBEL(devnum));
			af->lsobel = vinc_read(priv,
					STREAM_PROC_STAT_LSOBEL(devnum));
			af->rsobel = vinc_read(priv,
					STREAM_PROC_STAT_RSOBEL(devnum));
			vinc_write(priv, STREAM_PROC_CLEAR(devnum),
				   STREAM_PROC_CLEAR_AF_CLR);
		}
		if (stat_en & STT_EN_ADD) {
			add = stream->cluster.stat.add[z]->p_cur.p;
			vinc_write(priv, STREAM_PROC_STAT_CTR(devnum),
				   STREAM_PROC_STAT_CTR_NUM_ZONE(z));
			reg = vinc_read(priv, STREAM_PROC_STAT_MIN(devnum));
			add->min_b = reg & 0xFF;
			add->min_g = (reg >> 8) & 0xFF;
			add->min_r = (reg >> 16) & 0xFF;
			reg = vinc_read(priv, STREAM_PROC_STAT_MAX(devnum));
			add->max_b = reg & 0xFF;
			add->max_g = (reg >> 8) & 0xFF;
			add->max_r = (reg >> 16) & 0xFF;
			add->sum_b = vinc_read(priv,
					       STREAM_PROC_STAT_SUM_B(devnum));
			add->sum_g = vinc_read(priv,
					       STREAM_PROC_STAT_SUM_G(devnum));
			add->sum_r = vinc_read(priv,
					       STREAM_PROC_STAT_SUM_R(devnum));
			reg = vinc_read(priv, STREAM_PROC_STAT_SUM2_HI(devnum));
			add->sum2_b = reg & 0xFF;
			add->sum2_g = (reg >> 8) & 0xFF;
			add->sum2_r = (reg >> 16) & 0xFF;
			add->sum2_b = (add->sum2_b << 32) |
				vinc_read(priv,
					  STREAM_PROC_STAT_SUM2_B(devnum));
			add->sum2_g = (add->sum2_g << 32) |
				vinc_read(priv,
					  STREAM_PROC_STAT_SUM2_G(devnum));
			add->sum2_r = (add->sum2_r << 32) |
				vinc_read(priv,
					  STREAM_PROC_STAT_SUM2_R(devnum));
			vinc_write(priv, STREAM_PROC_CLEAR(devnum),
				   STREAM_PROC_CLEAR_ADD_CLR);
		}
	}
}

static void vinc_eof_handler(struct vinc_stream *stream)
{
	struct vinc_dev *priv = container_of(stream, struct vinc_dev,
					     stream[stream->devnum]);

	if (stream->cluster.stat.enable->val) {
		/* TODO: Tasklet must complete before the next frame starts.
		 * Otherwise it will read broken statistic. We need to take
		 * into account that tasklet can run when the next frame starts
		 * (or protect ourselves from this situation). */
		if (stream->stat_odd)
			tasklet_schedule(&stream->stat_tasklet);
		stream->stat_odd = ~stream->stat_odd;
	}
	if (stream->active) {
		dev_dbg(priv->ici.v4l2_dev.dev, "Frame end\n");
		vinc_next_buffer(stream, VB2_BUF_STATE_DONE);
	} else
		dev_warn(priv->ici.v4l2_dev.dev,
			 "Unexpected interrupt. VINC started without driver?\n");
}

static irqreturn_t vinc_irq_stream(int irq, void *data)
{
	struct vinc_stream *stream = (struct vinc_stream *)data;
	const u8 devnum = stream->devnum;
	struct vinc_dev *priv = container_of(stream, struct vinc_dev,
					     stream[devnum]);
	u32 int_status = vinc_read(priv, STREAM_INTERRUPT(devnum));

	dev_dbg(priv->ici.v4l2_dev.dev, "Interrupt stream%d 0x%x\n", devnum,
		int_status);
	if (int_status & STREAM_INTERRUPT_PROC) {
		u32 int_proc = vinc_read(priv, STREAM_STATUS(devnum));
		u32 stream_ctr = vinc_read(priv, STREAM_CTR);
		u32 wr_ctr = vinc_read(priv, STREAM_DMA_WR_CTR(devnum, 0));

		stream_ctr &= ~STREAM_CTR_STREAM_ENABLE(devnum);
		vinc_write(priv, STREAM_CTR, stream_ctr);

		wr_ctr &= ~DMA_WR_CTR_DMA_EN;
		vinc_write(priv, STREAM_DMA_WR_CTR(devnum, 0), wr_ctr);

		vinc_next_buffer(stream, VB2_BUF_STATE_ERROR);

		dev_warn(priv->ici.v4l2_dev.dev,
			 "Short frame/line. Stream%d_status: 0x%x\n", devnum,
			 int_proc);

		stream_ctr |= STREAM_CTR_STREAM_ENABLE(devnum);
		vinc_write(priv, STREAM_CTR, stream_ctr);
	}
	if (int_status & STREAM_INTERRUPT_DMA0) {
		u32 int_d0 = vinc_read(priv, STREAM_DMA_WR_STATUS(devnum, 0));

		if (int_d0 & DMA_WR_STATUS_FRAME_END)
			vinc_eof_handler(stream);
		if (int_d0 & DMA_WR_STATUS_DMA_OVF) {
			u32 stream_ctr = vinc_read(priv, STREAM_CTR);

			stream_ctr &= ~STREAM_CTR_DMA_CHANNELS_ENABLE;
			vinc_write(priv, STREAM_CTR, stream_ctr);
			vinc_next_buffer(stream, VB2_BUF_STATE_ERROR);
			dev_warn(priv->ici.v4l2_dev.dev,
				 "s%dd0: DMA overflow\n", devnum);
			stream_ctr |= STREAM_CTR_DMA_CHANNELS_ENABLE;
			vinc_write(priv, STREAM_CTR, stream_ctr);
		}
	}
	if (int_status & STREAM_INTERRUPT_DMA1) {
		u32 int_d1 = vinc_read(priv, STREAM_DMA_WR_STATUS(devnum, 1));

		if (int_d1 & DMA_WR_STATUS_DMA_OVF)
			dev_warn(priv->ici.v4l2_dev.dev,
				 "s%dd1: DMA overflow\n", devnum);
	}
	vinc_write(priv, STREAM_INTERRUPT_RESET(devnum), int_status);

	return IRQ_HANDLED;
}

static int vinc_clk_init(struct vinc_dev *priv, struct platform_device *pdev)
{
	int err;

	priv->pclk = devm_clk_get(&pdev->dev, "pclk");
	if (IS_ERR(priv->pclk)) {
		err = PTR_ERR(priv->pclk);
		dev_err(&pdev->dev,
			"failed to get pclk (%u)\n", err);
		return err;
	}

	priv->aclk = devm_clk_get(&pdev->dev, "aclk");
	if (IS_ERR(priv->aclk)) {
		err = PTR_ERR(priv->aclk);
		dev_err(&pdev->dev,
			"failed to get aclk (%u)\n", err);
		return err;
	}

	priv->sclk = devm_clk_get(&pdev->dev, "sclk");
	if (IS_ERR(priv->sclk)) {
		err = PTR_ERR(priv->sclk);
		dev_err(&pdev->dev,
			"failed to get sclk (%u)\n", err);
		return err;
	}

	err = clk_prepare_enable(priv->pclk);
	if (err) {
		dev_err(&pdev->dev,
			"failed to enable pclk (%u)\n", err);
		return err;
	}

	err = clk_prepare_enable(priv->aclk);
	if (err) {
		dev_err(&pdev->dev,
			"failed to enable aclk (%u)\n", err);
		goto disable_pclk;
	}

	err = clk_prepare_enable(priv->sclk);
	if (err) {
		dev_err(&pdev->dev,
			"failed to enable sclk (%u)\n", err);
		goto disable_aclk;
	}

	return 0;

disable_aclk:
	clk_disable_unprepare(priv->aclk);
disable_pclk:
	clk_disable_unprepare(priv->pclk);

	return err;
}

static int vinc_probe(struct platform_device *pdev)
{
	struct vinc_dev *priv;
	struct resource *res;
	struct device_node *np = pdev->dev.of_node;
	int err;
	u32 id;
	u32 cmos_ctr;
	int i;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Failed to get memory resource\n");
		return -ENODEV;
	}

	/* Try to get and enable clocks */
	err = vinc_clk_init(priv, pdev);
	if (err)
		return err;

	priv->reset_active = 1;
	if (np)
		of_property_read_u32(np, "reset-active", &priv->reset_active);

	priv->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	id = vinc_read(priv, ID);
	if (id != 0x76494e01)
		dev_err(&pdev->dev, "Bad magic: %#08x\n", id);

	for (i = 0; i < 2; i++) {
		INIT_LIST_HEAD(&priv->stream[i].capture);
		spin_lock_init(&priv->stream[i].lock);
		init_completion(&priv->stream[i].complete);

		priv->stream[i].video_source = V4L2_MBUS_CSI2;
		priv->stream[i].input_format = BAYER;
		priv->stream[i].fdecim = 1;
		priv->stream[i].devnum = i;
	}

	priv->ici.priv = priv;
	priv->ici.v4l2_dev.dev = &pdev->dev;
	priv->ici.nr = pdev->id;
	priv->ici.drv_name = "vinc";
	priv->ici.ops = &vinc_host_ops;
	priv->ici.capabilities = SOCAM_HOST_CAP_STRIDE;

	for (i = 0; i < 2; i++) {
		priv->stream[i].alloc_ctx = vb2_dma_contig_init_ctx(&pdev->dev);
		if (IS_ERR(priv->stream[i].alloc_ctx))
			return PTR_ERR(priv->stream[i].alloc_ctx);
	}

	priv->irq_vio = platform_get_irq(pdev, 0);
	err = priv->irq_vio;
	for (i = 0; i < 2; i++) {
		priv->irq_stream[i] = platform_get_irq(pdev, i + 1);
		err |= priv->irq_stream[i];
	}
	if (err < 0) {
		dev_err(&pdev->dev, "Failed to get required IRQs\n");
		return -ENODEV;
	}

	/* request irq */
	err = devm_request_irq(&pdev->dev, priv->irq_vio, vinc_irq_vio,
			       0, dev_name(&pdev->dev), priv);
	for (i = 0; i < 2; i++)
		err |= devm_request_irq(&pdev->dev, priv->irq_stream[i],
					vinc_irq_stream, 0,
					dev_name(&pdev->dev), &priv->stream[i]);
	if (err) {
		dev_err(&pdev->dev, "Failed to request required IRQs\n");
		return err;
	}
	for (i = 0; i < 2; i++)
		tasklet_init(&priv->stream[i].stat_tasklet, vinc_stat_tasklet,
			     (unsigned long)&priv->stream[i]);

	cmos_ctr = CMOS_CTR_PCLK_EN | CMOS_CTR_PCLK_SRC(0) |
			CMOS_CTR_CLK_DIV(4) | CMOS_CTR_FSYNC_EN;
	if (priv->reset_active == 0)
		cmos_ctr |= CMOS_CTR_RESET;
	vinc_write(priv, CMOS_CTR(0), cmos_ctr);
	vinc_write(priv, CMOS_TIMER_HIGH(0), 1);
	vinc_write(priv, CMOS_TIMER_LOW(0), 1);

	/* GLOBAL_ENABLE need for generate clocks to sensor */
	vinc_write(priv, AXI_MASTER_CFG, AXI_MASTER_CFG_MAX_BURST(2) |
			AXI_MASTER_CFG_MAX_WR_ID(1) |
			AXI_MASTER_CFG_BUF_LAYOUT(0x5) |
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
	int i;

	reg &= ~AXI_MASTER_CFG_GLOBAL_EN;
	vinc_write(priv, AXI_MASTER_CFG, reg);

	for (i = 0; i < 2; i++) {
		if (priv->ici.icds[i] && priv->ici.icds[i]->host_priv)
			kfree(priv->ici.icds[i]->host_priv);
	}
	soc_camera_host_unregister(soc_host);
	if (platform_get_resource(pdev, IORESOURCE_MEM, 1))
		dma_release_declared_memory(&pdev->dev);
	for (i = 0; i < 2; i++)
		vb2_dma_contig_cleanup_ctx(priv->stream[i].alloc_ctx);

	clk_disable_unprepare(priv->pclk);
	clk_disable_unprepare(priv->aclk);
	clk_disable_unprepare(priv->sclk);

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

module_platform_driver(vinc_driver);

MODULE_DESCRIPTION("VINC driver");
MODULE_AUTHOR("Vasiliy Zasukhin <vzasukhin@elvees.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1.0");
MODULE_ALIAS("platform:vinc");
MODULE_SUPPORTED_DEVICE("video");
