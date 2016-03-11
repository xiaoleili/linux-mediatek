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

#ifndef MTKSDG1_ECC_REGS_H
#define MTKSDG1_ECC_REGS_H

/* ECC engine register definition */
#define MTKSDG1_ECC_ENCCON		(0x00)
#define		ENC_EN			(1)
#define		ENC_DE			(0)

#define MTKSDG1_ECC_ENCCNFG		(0x04)
#define		ECC_CNFG_4BIT		(0)
#define		ECC_CNFG_12BIT		(4)
#define		ECC_NFI_MODE		BIT(5)
#define		ECC_DMA_MODE		(0)
#define		ECC_ENC_MODE_MASK	(0x3 << 5)
#define		ECC_MS_SHIFT		(16)

#define MTKSDG1_ECC_ENCDIADDR		(0x08)

#define MTKSDG1_ECC_ENCIDLE		(0x0C)
#define		ENC_IDLE		BIT(0)

#define MTKSDG1_ECC_ENCPAR0		(0x10)
#define MTKSDG1_ECC_ENCSTA		(0x7C)

#define MTKSDG1_ECC_ENCIRQ_EN		(0x80)
#define		ENC_IRQEN		BIT(0)

#define MTKSDG1_ECC_ENCIRQ_STA		(0x84)

#define MTKSDG1_ECC_DECCON		(0x100)
#define		DEC_EN			(1)
#define		DEC_DE			(0)

#define MTKSDG1_ECC_DECCNFG		(0x104)
#define		DEC_EMPTY_EN		BIT(31)
#define		DEC_CNFG_FER		(0x1 << 12)
#define		DEC_CNFG_EL		(0x2 << 12)
#define		DEC_CNFG_CORRECT	(0x3 << 12)

#define MTKSDG1_ECC_DECIDLE		(0x10C)
#define		DEC_IDLE		BIT(0)

#define MTKSDG1_ECC_DECFER		(0x110)

#define MTKSDG1_ECC_DECENUM0		(0x114)
#define		ERR_MASK		(0x3f)

#define MTKSDG1_ECC_DECDONE		(0x124)

#define MTKSDG1_ECC_DECEL0		(0x128)

#define MTKSDG1_ECC_DECIRQ_EN		(0x200)
#define		DEC_IRQEN		BIT(0)

#define MTKSDG1_ECC_DECIRQ_STA		(0x204)

#define MTKSDG1_ECC_DECFSM		(0x208)
#define		DECFSM_MASK		(0x7f0f0f0f)
#define		DECFSM_IDLE		(0x01010101)
#endif
