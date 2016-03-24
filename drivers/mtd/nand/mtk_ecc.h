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

struct device_node;
struct nand_chip;
struct mtd_info;
struct mtk_ecc;
struct device;

/**
 * @len: number of bytes in the data buffer
 * @data: pointer to memory holding the data
 * @strength: number of correctable bits
 */
struct mtk_ecc_enc_data {
	unsigned len;
	int strength;
	u8 *data;
};

struct mtk_ecc_stats {
	u32 corrected;
	u32 bitflips;
	u32 failed;
};

struct mtk_ecc_config {
	u32 strength;
	u32 step_len;
};

void mtk_ecc_enable_decode(struct mtk_ecc *);
void mtk_ecc_disable_decode(struct mtk_ecc *);

int mtk_ecc_wait_decode(struct mtk_ecc *);
void mtk_ecc_enable_encode(struct mtk_ecc *);
void mtk_ecc_disable_encode(struct mtk_ecc *);
int mtk_ecc_start_encode(struct mtk_ecc *, struct mtk_ecc_enc_data *);
void mtk_ecc_hw_init(struct mtk_ecc *);
int mtk_ecc_config(struct mtk_ecc *, struct mtk_ecc_config *);
void mtk_ecc_release(struct mtk_ecc *);
struct mtk_ecc *of_mtk_ecc_get(struct device_node *);

void mtk_ecc_start_decode(struct mtk_ecc *, int sectors);
void mtk_ecc_get_stats(struct mtk_ecc *, struct mtk_ecc_stats *, int sectors);
#endif
