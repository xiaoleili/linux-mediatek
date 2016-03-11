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

#define KB(x)			((x) * 1024UL)
#define MB(x)			(KB(x) * 1024UL)

#define SECTOR_SHIFT		(10)
#define SECTOR_SIZE		(1UL << SECTOR_SHIFT)
#define BYTES_TO_SECTORS(x)	((x) >> SECTOR_SHIFT)
#define SECTORS_TO_BYTES(x)	((x) << SECTOR_SHIFT)

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


typedef int (*config_f) (struct sdg1_ecc_if *, struct mtd_info *, unsigned);
typedef void (*control_f)(struct sdg1_ecc_if *, enum sdg1_ecc_ctrl, int);
typedef void (*init_f) (struct sdg1_ecc_if *);

/**
 * MTK specific functions
 *
 * @author jramirez (3/10/2016)
 */
struct sdg1_ecc_if {
	control_f control;
	config_f config;
	init_f init;
};

/* these functions will go into the new API */
struct sdg1_ecc_if *of_sdg1_ecc_get(struct device_node *);
void sdg1_ecc_release(struct sdg1_ecc_if *);
int sdg1_ecc_encode(struct sdg1_ecc_if *, struct sdg1_encode_params *);
int sdg1_ecc_decode(struct sdg1_ecc_if *);
int sdg1_ecc_check(struct sdg1_ecc_if *, struct mtd_info *, u32);

#endif
