/*
 * MTK smart device NAND Flash controller register.
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

#ifndef MTKSDG1_NFI_REGS_H
#define MTKSDG1_NFI_REGS_H

/* NAND controller register definition */
#define MTKSDG1_NFI_CNFG		(0x00)
#define		CNFG_AHB		BIT(0)
#define		CNFG_READ_EN		BIT(1)
#define		CNFG_DMA_BURST_EN	BIT(2)
#define		CNFG_BYTE_RW		BIT(6)
#define		CNFG_HW_ECC_EN		BIT(8)
#define		CNFG_AUTO_FMT_EN	BIT(9)
#define		CNFG_OP_IDLE		(0 << 12)
#define		CNFG_OP_READ		(1 << 12)
#define		CNFG_OP_SRD		(2 << 12)
#define		CNFG_OP_PRGM		(3 << 12)
#define		CNFG_OP_ERASE		(4 << 12)
#define		CNFG_OP_RESET		(5 << 12)
#define		CNFG_OP_CUST		(6 << 12)

#define MTKSDG1_NFI_PAGEFMT		(0x04)
#define		PAGEFMT_FDM_ECC_SHIFT	(12)
#define		PAGEFMT_FDM_SHIFT	(8)
#define		PAGEFMT_SPARE_16	(0)
#define		PAGEFMT_SPARE_28	(3)
#define		PAGEFMT_SPARE_SHIFT	(4)
#define		PAGEFMT_SEC_SEL_512	BIT(2)
#define		PAGEFMT_512_2K		(0)
#define		PAGEFMT_2K_4K		(1)
#define		PAGEFMT_4K_8K		(2)
#define		PAGEFMT_8K_16K		(3)
/* NFI control */
#define MTKSDG1_NFI_CON			(0x08)
#define		CON_FIFO_FLUSH		BIT(0)
#define		CON_NFI_RST		BIT(1)
#define		CON_SRD			BIT(4)	/* single read */
#define		CON_BRD			BIT(8)  /* burst  read */
#define		CON_BWR			BIT(9)	/* burst  write */
#define		CON_SEC_SHIFT		(12)

/* Timming control register */
#define MTKSDG1_NFI_ACCCON		(0x0C)

#define MTKSDG1_NFI_INTR_EN		(0x10)
#define		INTR_RD_DONE_EN		BIT(0)
#define		INTR_WR_DONE_EN		BIT(1)
#define		INTR_RST_DONE_EN	BIT(2)
#define		INTR_ERS_DONE_EN	BIT(3)
#define		INTR_BUSY_RT_EN		BIT(4)
#define		INTR_AHB_DONE_EN	BIT(6)

#define MTKSDG1_NFI_INTR_STA		(0x14)

#define MTKSDG1_NFI_CMD			(0x20)

#define MTKSDG1_NFI_ADDRNOB		(0x30)
#define		ADDR_ROW_NOB_SHIFT	(4)

#define MTKSDG1_NFI_COLADDR		(0x34)
#define MTKSDG1_NFI_ROWADDR		(0x38)
#define MTKSDG1_NFI_STRDATA		(0x40)
#define MTKSDG1_NFI_CNRNB		(0x44)
#define MTKSDG1_NFI_DATAW		(0x50)
#define MTKSDG1_NFI_DATAR		(0x54)
#define MTKSDG1_NFI_PIO_DIRDY		(0x58)
#define		PIO_DI_RDY		(0x01)

/* NFI state*/
#define MTKSDG1_NFI_STA			(0x60)
#define		STA_CMD			BIT(0)
#define		STA_ADDR		BIT(1)
#define		STA_DATAR		BIT(2)
#define		STA_DATAW		BIT(3)
#define		STA_BUSY		BIT(8)
#define		STA_EMP_PAGE		BIT(12)
#define		NFI_FSM_CUSTDATA	(0xe << 16)
#define		NFI_FSM_MASK		(0xf << 16)

#define MTKSDG1_NFI_FIFOSTA		(0x64)

#define MTKSDG1_NFI_ADDRCNTR		(0x70)
#define		CNTR_MASK		GENMASK(16, 12)

#define MTKSDG1_NFI_STRADDR		(0x80)
#define MTKSDG1_NFI_BYTELEN		(0x84)
#define MTKSDG1_NFI_CSEL		(0x90)
#define MTKSDG1_NFI_IOCON		(0x94)

/* FDM data for sector: FDM0[L,H] - FDMF[L,H] */
#define MTKSDG1_NFI_FDM_MAX_SEC		(0x10)
#define MTKSDG1_NFI_FDM_REG_SIZE	(8)
#define MTKSDG1_NFI_FDM0L		(0xA0)
#define MTKSDG1_NFI_FDM0M		(0xA4)


#define MTKSDG1_NFI_FIFODATA0		(0x190)
#define MTKSDG1_NFI_DEBUG_CON1		(0x220)
#define MTKSDG1_NFI_MASTER_STA		(0x224)
#define		MASTER_STA_MASK		(0x0FFF)

#define MTKSDG1_NFI_RANDOM_CNFG		(0x238)
#define MTKSDG1_NFI_EMPTY_THRESH	(0x23C)
#define MTKSDG1_NFI_NAND_TYPE		(0x240)
#define MTKSDG1_NFI_ACCCON1		(0x244)
#define MTKSDG1_NFI_DELAY_CTRL		(0x248)

#endif
