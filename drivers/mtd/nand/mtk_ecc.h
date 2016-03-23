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
	u32 sectors;
	u32 corrected;
	u32 bitflips;
	u32 failed;
};

void mtk_ecc_enable_decode(struct mtk_ecc *ecc);
void mtk_ecc_disable_decode(struct mtk_ecc *ecc);
void mtk_ecc_start_decode(struct mtk_ecc *ecc, int sectors);
int mtk_ecc_wait_decode(struct mtk_ecc *ecc);
void mtk_ecc_enable_encode(struct mtk_ecc *ecc);
void mtk_ecc_disable_encode(struct mtk_ecc *ecc);
int mtk_ecc_start_encode(struct mtk_ecc *ecc, struct mtk_ecc_enc_data *d);
void mtk_ecc_get_stats(struct mtk_ecc *ecc, struct mtk_ecc_stats *stats);
void mtk_ecc_hw_init(struct mtk_ecc *ecc);
int mtk_ecc_config(struct mtk_ecc *ecc, int strength, int step_len);
void mtk_ecc_release(struct mtk_ecc *ecc);
struct mtk_ecc *of_mtk_ecc_get(struct device_node *);

#endif
