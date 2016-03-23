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

#ifndef __DRIVERS_MTD_NAND_MTKSDG1_ECC_H__
#define __DRIVERS_MTD_NAND_MTKSDG1_ECC_H__

#include <linux/types.h>

struct device_node;
struct nand_chip;
struct mtd_info;
struct sdg1_ecc;
struct device;

/**
 * @len: number of bytes in the data buffer
 * @data: pointer to memory holding the data
 * @strength: number of correctable bits
 */
struct sdg1_enc_data {
	unsigned len;
	int strength;
	u8 *data;
};

struct sdg1_ecc_stats {
	u32 sectors;
	u32 corrected;
	u32 bitflips;
	u32 failed;
};

void sdg1_ecc_enable_decode(struct sdg1_ecc *ecc);
void sdg1_ecc_disable_decode(struct sdg1_ecc *ecc);
void sdg1_ecc_start_decode(struct sdg1_ecc *ecc, int sectors);
int sdg1_ecc_wait_decode(struct sdg1_ecc *ecc);
void sdg1_ecc_enable_encode(struct sdg1_ecc *ecc);
void sdg1_ecc_disable_encode(struct sdg1_ecc *ecc);
int sdg1_ecc_start_encode(struct sdg1_ecc *ecc, struct sdg1_enc_data *d);
void sdg1_ecc_get_stats(struct sdg1_ecc *ecc, struct sdg1_ecc_stats *stats);
void sdg1_ecc_hw_init(struct sdg1_ecc *ecc);
int sdg1_ecc_config(struct sdg1_ecc *ecc, int strength, int step_len);
void sdg1_ecc_release(struct sdg1_ecc *ecc);
struct sdg1_ecc *of_sdg1_ecc_get(struct device_node *);

#endif
