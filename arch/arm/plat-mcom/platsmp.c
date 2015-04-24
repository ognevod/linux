/*
 *  Copyright 2015 ELVEES NeoTek CJSC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/init.h>
#include <linux/smp.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <asm/smp_scu.h>
#include <asm/smp_plat.h>

static void __iomem *spram_base_addr;

static void __init mcom_prepare_spram(void)
{
	struct device_node *node;

	node = of_find_compatible_node(NULL, NULL, "elvees,mcom-spram");
	if (!node) {
		pr_err("%s: could not find spram dt node\n", __func__);
		return;
	}

	spram_base_addr = of_iomap(node, 0);
	if (!spram_base_addr) {
		pr_err("%s: could not map spram registers\n", __func__);
		return;
	}
}

static int __cpuinit mcom_boot_secondary(unsigned int cpu,
					   struct task_struct *idle)
{
	/* FIX ME
	 *
	 * The code in this block is actualy a hack.
	 * We should use power management controller (PMCTR) to power up all
	 * cores except core1. Instead of that we simply write magic value and
	 * address of secondary_startup function to spram. Secondary core polls
	 * spram and when the magic value is read calls secondary_startup
	 * function.
	*/

	const u32 magic = 0xdeadbeef;
	const u32 offset_for_magic = 0xfff8;
	const u32 offset_for_addr = 0xfff4;


	if (!spram_base_addr) {
		pr_err("%s: spram_base_addr is missing for cpu boot\n",
		       __func__);
		return -ENXIO;
	}

	__raw_writel(magic, spram_base_addr + offset_for_magic);
	__raw_writel((unsigned int)&secondary_startup, spram_base_addr +
		     offset_for_addr);

	return 0;
}

static void __init mcom_smp_prepare_cpus(unsigned int max_cpus)
{
	mcom_prepare_spram();
	scu_enable(ioremap(scu_a9_get_base(), SZ_4K));
}

static struct smp_operations mcom_smp_ops __initdata = {
	.smp_prepare_cpus   = mcom_smp_prepare_cpus,
	.smp_boot_secondary = mcom_boot_secondary,
};
CPU_METHOD_OF_DECLARE(mcom_smp, "elvees,mcom-smp", &mcom_smp_ops);
