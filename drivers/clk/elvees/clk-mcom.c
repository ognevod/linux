/*
 *  Copyright 2015 ELVEES NeoTek CJSC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This is the temporary version of MCom CMCTR driver. It will be replaced
 * by full-featured version in future.
 */

#include <linux/bitops.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/io.h>
#include <linux/clk-provider.h>
#include <linux/slab.h>
#include <linux/log2.h>

#define DIV_MPU_CTR     0x004
#define DIV_ATB_CTR     0x008
#define DIV_APB_CTR     0x00C
#define GATE_MPU_CTR    0x014
#define DIV_SYS1_CTR    0x040
#define DIV_SYS2_CTR    0x044
#define GATE_CORE_CTR   0x048
#define GATE_SYS_CTR    0x04C
#define GATE_DSP_CTR    0x068

#define SEL_APLL        0x100
#define SEL_CPLL        0x104
#define SEL_DPLL        0x108
#define SEL_SPLL        0x10C
#define SEL_VPLL        0x110
#define SEL_UPLL        0x114

#define GATE_CORE_CTR_L0_EN    BIT(0)
#define GATE_CORE_CTR_DDR0_EN  BIT(1)
#define GATE_CORE_CTR_DDR1_EN  BIT(2)
#define GATE_CORE_CTR_VPIN_EN  BIT(3)
#define GATE_CORE_CTR_VPOUT_EN BIT(4)
#define GATE_CORE_CTR_VPU_EN   BIT(5)
#define GATE_CORE_CTR_GPU_EN   BIT(6)

#define GATE_SYS_CTR_I2S_EN    BIT(1)
#define GATE_SYS_CTR_SDMMC0_EN BIT(2)
#define GATE_SYS_CTR_SDMMC1_EN BIT(3)
#define GATE_SYS_CTR_EMAC_EN   BIT(4)
#define GATE_SYS_CTR_USB_EN    BIT(5)
#define GATE_SYS_CTR_UART0_EN  BIT(12)
#define GATE_SYS_CTR_UART1_EN  BIT(13)
#define GATE_SYS_CTR_UART2_EN  BIT(14)
#define GATE_SYS_CTR_UART3_EN  BIT(15)
#define GATE_SYS_CTR_I2C0_EN   BIT(16)
#define GATE_SYS_CTR_I2C1_EN   BIT(17)
#define GATE_SYS_CTR_I2C2_EN   BIT(18)
#define GATE_SYS_CTR_SPI0_EN   BIT(19)
#define GATE_SYS_CTR_SPI1_EN   BIT(20)

#define GATE_DSP_CTR_DSP0_EN   BIT(0)
#define GATE_DSP_CTR_DSP1_EN   BIT(1)
#define GATE_DSP_CTR_DSPEXT_EN BIT(2)
#define GATE_DSP_CTR_DSPENC_EN BIT(3)

#define cmctr_readl(port, reg) __raw_readl((port)->reg_base + reg)
#define cmctr_writel(port, reg, value) \
	__raw_writel((value), (port)->reg_base + reg)

struct cmctr_pdata {
	void __iomem *reg_base;
};

static void __init enable_clocks(struct cmctr_pdata *pd)
{
	u32 reg;

	reg = cmctr_readl(pd, GATE_SYS_CTR);
	reg |= (GATE_SYS_CTR_I2S_EN |
		GATE_SYS_CTR_SDMMC0_EN |
		GATE_SYS_CTR_EMAC_EN |
		GATE_SYS_CTR_USB_EN |
		GATE_SYS_CTR_UART0_EN |
		GATE_SYS_CTR_UART1_EN |
		GATE_SYS_CTR_UART2_EN |
		GATE_SYS_CTR_UART3_EN |
		GATE_SYS_CTR_I2C0_EN |
		GATE_SYS_CTR_I2C1_EN |
		GATE_SYS_CTR_I2C2_EN |
		GATE_SYS_CTR_SPI0_EN |
		GATE_SYS_CTR_SPI1_EN);
	cmctr_writel(pd, GATE_SYS_CTR, reg);

	reg = cmctr_readl(pd, GATE_CORE_CTR);
	reg |= (GATE_CORE_CTR_L0_EN |
		GATE_CORE_CTR_DDR0_EN |
		GATE_CORE_CTR_VPU_EN |
		GATE_CORE_CTR_GPU_EN);
	cmctr_writel(pd, GATE_CORE_CTR, reg);

	reg = cmctr_readl(pd, GATE_DSP_CTR);
	reg |= (GATE_DSP_CTR_DSP0_EN |
		GATE_DSP_CTR_DSP1_EN |
		GATE_DSP_CTR_DSPEXT_EN);
	cmctr_writel(pd, GATE_DSP_CTR, reg);
}

static u32 __init read_property(char const *const name, char const *const path)
{
	struct device_node *node;
	u32 read_value;
	int rc;

	node = of_find_node_by_path(path);
	if (!node) {
		pr_err("Can not find %s in DTS\n", path);
		return 0;
	}

	rc = of_property_read_u32(node, name, &read_value);
	if (rc) {
		pr_err("%s: Can not read %s property\n", path, name);
		return 0;
	}

	return read_value;
}

static void __init set_pll(struct cmctr_pdata *const pd, unsigned const sel,
			   char const *const path)
{
	u32 clock_mult;

	clock_mult = read_property("clock-mult", path);

	if (clock_mult == 0 || clock_mult > 256) {
		pr_err("%s: Incorrect clock-mult (%u)\n", path, clock_mult);
		return;
	}

	/* We must write (clock_mult - 1) to sel in order to set
	 * 24*clock_mult clock frequency.
	 */

	cmctr_writel(pd, sel, clock_mult - 1);
	if (clock_mult != 1) {
		do {
			clock_mult = cmctr_readl(pd, sel);
		} while (!((clock_mult >> 31) & 1));
	}
}

static void __init set_divider(struct cmctr_pdata *const pd, u32 const reg,
			       u32 const max, char const *const path)
{
	u32 clock_div;

	clock_div = read_property("clock-div", path);

	if (clock_div == 0 || clock_div > max || !is_power_of_2(clock_div)) {
		pr_err("Incorrect clock-div (%u) on %s\n", clock_div, path);
		return;
	}

	cmctr_writel(pd, reg, fls(clock_div) - 1);
}

static void __init mcom_clk_init(struct device_node *np)
{
	struct cmctr_pdata *pd;

	pd = kmalloc(sizeof(struct cmctr_pdata), GFP_KERNEL);
	if (!pd)
		return;

	pd->reg_base = of_iomap(np, 0);
	if (!pd->reg_base) {
		pr_err("%s: Can not map requested memory region\n", np->name);
		return;
	}

	set_divider(pd, DIV_APB_CTR, 2, "/clocks/apclk");
	set_divider(pd, DIV_ATB_CTR, 8, "/clocks/atclk");
	set_divider(pd, DIV_MPU_CTR, 2, "/clocks/mpspclk");
	set_divider(pd, DIV_SYS1_CTR, 2, "/clocks/l1_hclk");
	set_divider(pd, DIV_SYS2_CTR, 2, "/clocks/l3_pclk");

	set_pll(pd, SEL_APLL, "/clocks/apllclk");
	/* Do not set CPLL, because it clocks DDR. */
	set_pll(pd, SEL_DPLL, "/clocks/dpllclk");
	set_pll(pd, SEL_SPLL, "/clocks/spllclk");
	set_pll(pd, SEL_VPLL, "/clocks/vpllclk");
	set_pll(pd, SEL_UPLL, "/clocks/upllclk");

	enable_clocks(pd);
}

CLK_OF_DECLARE(mcom_cmctr, "elvees,mcom-cmctr", mcom_clk_init);
