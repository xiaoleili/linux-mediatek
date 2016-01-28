/*
 * MTK smart device ECC engine register.
 * Copyright (C) 2015-2016 MediaTek Inc.
 * Author: Xiaolei.Li <xiaolei.li@mediatek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef MTKSDG1_NAND_ECC_H
#define MTKSDG1_NAND_ECC_H

/* ECC engine register definition */
#define MTKSDG1_ECC_ENCCON 0x00
#define MTKSDG1_ECC_ENCCNFG 0x04
#define MTKSDG1_ECC_ENCIDLE 0x0C
#define MTKSDG1_ECC_ENCPAR0 0x10
#define MTKSDG1_ECC_ENCSTA 0x7C
#define MTKSDG1_ECC_DECCON 0x100
#define MTKSDG1_ECC_DECCNFG 0x104
#define MTKSDG1_ECC_DECIDLE 0x10C
#define MTKSDG1_ECC_DECFER 0x110
#define MTKSDG1_ECC_DECENUM0 0x114
#define MTKSDG1_ECC_DECDONE 0x124
#define MTKSDG1_ECC_DECEL0 0x128
#define MTKSDG1_ECC_DECFSM 0x208


#endif
