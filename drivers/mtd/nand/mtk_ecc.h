/*
 * MTK SDG1 ECC controller
 *
 * Copyright (c) 2016 Mediatek
 * Authors:	Xiaolei Li		<xiaolei.li@mediatek.com>
 *		Jorge Ramirez-Ortiz	<jorge.ramirez-ortiz@linaro.org>
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 */

#ifndef __DRIVERS_MTD_NAND_MTK_ECC_H__
#define __DRIVERS_MTD_NAND_MTK_ECC_H__

#include <linux/types.h>

#define ECC_PARITY_BITS		(14)

struct device_node;
struct mtk_ecc;

enum mtk_ecc_codec {ECC_ENC, ECC_DEC};

enum mtk_ecc_mode {
	ECC_DMA_MODE = 0,
	ECC_NFI_MODE = 1};
struct mtk_ecc_stats {
	u32 corrected;
	u32 bitflips;
	u32 failed;
};

struct mtk_ecc_config {
	u32 strength;
	u32 enc_len;
	u32 dec_len;
	u32 sec_mask;
	enum mtk_ecc_mode ecc_mode;
	enum mtk_ecc_codec codec;
	dma_addr_t addr;
};

void mtk_ecc_enable(struct mtk_ecc *, struct mtk_ecc_config *);
void mtk_ecc_disable(struct mtk_ecc *, struct mtk_ecc_config *);

int mtk_ecc_encode_non_nfi_mode(struct mtk_ecc *, struct mtk_ecc_config *, u8 *data, u32 bytes);

int mtk_ecc_wait_irq_done(struct mtk_ecc *, enum mtk_ecc_codec codec);

void mtk_ecc_get_stats(struct mtk_ecc *, struct mtk_ecc_stats *, int sectors);

void mtk_ecc_hw_init(struct mtk_ecc *);
void mtk_ecc_update_strength(u32 *);

struct mtk_ecc *of_mtk_ecc_get(struct device_node *);
void mtk_ecc_release(struct mtk_ecc *);

#endif
