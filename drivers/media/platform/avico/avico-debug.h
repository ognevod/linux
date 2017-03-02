/*
 * ELVEES Avico (a.k.a. VELcore-01) driver - Debug helpers
 *
 * Copyright 2015 ELVEES NeoTek JSC
 * Copyright 2017 RnD Center "ELVEES", JSC
 *
 * Author: Anton Leontiev <aleontiev@elvees.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef AVICO_DEBUG_H
#define AVICO_DEBUG_H

#include <linux/types.h>

void avico_regs_ctrl_dump(void __iomem *const base);
void avico_regs_thread_dump(void __iomem *const base);
void avico_regs_vdma_sys_dump(void __iomem *const base);
void avico_regs_vdma_channel_dump(void __iomem *const base);
void avico_regs_vdma_dump(void __iomem *const base, unsigned channel);
void avico_regs_ec_taskctrc_dump(void __iomem *const base);
void avico_regs_ec_vramctrc_dump(void __iomem *const base);
void avico_regs_ec_cavlc_dump(void __iomem *const base);
void avico_regs_ec_hdrc_dump(void __iomem *const base);
void avico_regs_ec_packer_dump(void __iomem *const base);
void avico_regs_ec_regc_dump(void __iomem *const base);
void avico_regs_ec_dump(void __iomem *const base);
void avico_regs_dump(void __iomem *const base, unsigned const thread);

#endif /* AVICO_DEBUG_H */
