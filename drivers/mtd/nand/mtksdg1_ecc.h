/*
 * MTK SDG1 ECC controller
 *
 * Copyright (c) 2016 Mediatek
 * Author: Jorge Ramirez-Ortiz <jorge.ramirez-ortiz@linaro.org>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 */

#ifndef __DRIVERS_MTD_NAND_MTKSDG1_ECC_H__
#define __DRIVERS_MTD_NAND_MTKSDG1_ECC_H__

#include <linux/types.h>

struct sdg1_ecc_if;
struct device_node;
struct nand_chip;
struct mtd_info;
struct device;

enum sdg1_ecc_ctrl {
	disable_encoder,
	disable_decoder,
	enable_encoder,
	enable_decoder,
	start_decoder
};

struct sdg1_encode_params {
	unsigned len;
	int strength;
	u8 *data;
};

typedef int (*config) (struct sdg1_ecc_if *, struct mtd_info *, unsigned);
typedef int (*encode)(struct sdg1_ecc_if *, struct sdg1_encode_params *);
typedef void (*control)(struct sdg1_ecc_if *, enum sdg1_ecc_ctrl, int);
typedef int (*check)(struct sdg1_ecc_if *, struct mtd_info *, u32);
typedef void (*release)(struct sdg1_ecc_if *);
typedef int (*decode)(struct sdg1_ecc_if *);
typedef void (*init) (struct sdg1_ecc_if *);

struct sdg1_ecc_if {
	release	release;
	control control;
	encode encode;
	decode decode;
	config config;
	check check;
	init init;
};

/* these functions will go into the new API */
struct sdg1_ecc_if *of_sdg1_ecc_get(struct device_node *);

#endif
