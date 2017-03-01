/*
 * Avico driver
 *
 * Copyright 2015 ELVEES NeoTek JSC
 * Author: Anton Leontiev <aleontiev@elvees.com>
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
