/*
 * MTK smart device NAND Flash controller driver.
 * Copyright (C) 2016 MediaTek Inc.
 * Authors:	Xiaolei Li		<xiaolei.li@mediatek.com>
 *		Jorge Ramirez-Ortiz	<jorge.ramirez-ortiz@linaro.org>
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

#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/of_mtd.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/mtd.h>
#include <linux/module.h>
#include <linux/iopoll.h>

#include "mtksdg1_ecc.h"

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


#define MTK_NAME		"mtksdg1-nand"
#define KB(x)			((x) * 1024UL)
#define MB(x)			(KB(x) * 1024UL)

#define MTK_TIMEOUT		(500000)
#define MTK_RESET_TIMEOUT	(1000000)
#define MTK_MAX_SECTOR		(16)
#define MTK_NAND_MAX_NSELS	(2)

struct mtk_nfc_clk {
	struct clk *nfi_clk;
	struct clk *pad_clk;
};

struct mtk_nfc_saved_reg {
	u32 emp_thresh;
	u16 pagefmt;
	u32 acccon;
	u16 cnrnb;
	u16 csel;
};

struct mtk_nfc_nand_chip {
	struct list_head node;
	struct nand_chip nand;
	u32 sparesize;
	int nsels;
	u8 sels[0];
};

struct mtk_nfc {
	struct nand_hw_control controller;
	struct mtk_nfc_clk clk;
	struct sdg1_ecc *ecc;

	struct device *dev;
	void __iomem *regs;

	struct completion done;
	struct list_head chips;

	bool switch_oob;
	u8 *buffer;

#ifdef CONFIG_PM_SLEEP
	struct mtk_nfc_saved_reg saved_reg;
#endif
};

static inline struct mtk_nfc_nand_chip *to_mtk_nand(struct nand_chip *nand)
{
	return container_of(nand, struct mtk_nfc_nand_chip, nand);
}

static inline int data_len(struct nand_chip *chip)
{
	return chip->ecc.size;
}

static inline int sdg1_oob_len(void)
{
	return MTKSDG1_NFI_FDM_REG_SIZE;
}

static inline uint8_t *data_ptr(struct nand_chip *chip, const uint8_t *p, int i)
{
	return (uint8_t *) p + i * data_len(chip);
}

static inline uint8_t *oob_ptr(struct nand_chip *chip, int i)
{
	return chip->oob_poi + i * sdg1_oob_len();
}

static inline int sdg1_data_len(struct nand_chip *chip)
{
	struct mtd_info *mtd = nand_to_mtd(chip);

	return data_len(chip) + mtd->oobsize / chip->ecc.steps;
}

static inline uint8_t *sdg1_data_ptr(struct nand_chip *chip,  int i)
{
	struct mtk_nfc *nfc = nand_get_controller_data(chip);

	return nfc->buffer + i * sdg1_data_len(chip);
}

static inline uint8_t *sdg1_oob_ptr(struct nand_chip *chip, int i)
{
	struct mtk_nfc *nfc = nand_get_controller_data(chip);

	return nfc->buffer + i * sdg1_data_len(chip) + data_len(chip);
}

static inline int sdg1_step_len(struct nand_chip *chip)
{
	/* CRC protected per step */
	return data_len(chip) + sdg1_oob_len();
}

/* NFI register access */
static inline void sdg1_writel(struct mtk_nfc *nfc, u32 val, u32 reg)
{
	writel(val, nfc->regs + reg);
}

static inline void sdg1_writew(struct mtk_nfc *nfc, u16 val, u32 reg)
{
	writew(val, nfc->regs + reg);
}

static inline void sdg1_writeb(struct mtk_nfc *nfc, u8 val, u32 reg)
{
	writeb(val, nfc->regs + reg);
}

static inline u32 sdg1_readl(struct mtk_nfc *nfc, u32 reg)
{
	return readl_relaxed(nfc->regs + reg);
}

static inline u16 sdg1_readw(struct mtk_nfc *nfc, u32 reg)
{
	return readw_relaxed(nfc->regs + reg);
}

static inline u8 sdg1_readb(struct mtk_nfc *nfc, u32 reg)
{
	return readb_relaxed(nfc->regs + reg);
}

static void mtk_nfc_hw_reset(struct mtk_nfc *nfc)
{
	struct device *dev = nfc->dev;
	u32 val;
	int ret;

	sdg1_writel(nfc, CON_FIFO_FLUSH | CON_NFI_RST, MTKSDG1_NFI_CON);

	ret = readl_poll_timeout(nfc->regs + MTKSDG1_NFI_MASTER_STA, val,
			!(val & MASTER_STA_MASK), 50, MTK_RESET_TIMEOUT);
	if (ret)
		dev_warn(dev, "master active in reset [0x%x] = 0x%x\n",
			MTKSDG1_NFI_MASTER_STA, val);

	sdg1_writew(nfc, 0, MTKSDG1_NFI_STRDATA);
}

static int mtk_nfc_send_command(struct mtk_nfc *nfc, u8 command)
{
	struct device *dev = nfc->dev;
	u32 val;
	int ret;

	sdg1_writel(nfc, command, MTKSDG1_NFI_CMD);

	ret = readl_poll_timeout_atomic(nfc->regs + MTKSDG1_NFI_STA, val,
					!(val & STA_CMD), 10,  MTK_TIMEOUT);
	if (ret) {
		dev_warn(dev, "nfi core timed out entering command mode\n");
		return -EIO;
	}

	return 0;
}

static int mtk_nfc_send_address(struct mtk_nfc *nfc, int addr)
{
	struct device *dev = nfc->dev;
	u32 val;
	int ret;

	sdg1_writel(nfc, addr, MTKSDG1_NFI_COLADDR);
	sdg1_writel(nfc, 0, MTKSDG1_NFI_ROWADDR);
	sdg1_writew(nfc, 1, MTKSDG1_NFI_ADDRNOB);

	ret = readl_poll_timeout_atomic(nfc->regs + MTKSDG1_NFI_STA, val,
					!(val & STA_ADDR), 10, MTK_TIMEOUT);
	if (ret) {
		dev_warn(dev, "nfi core timed out entering address mode\n");
		return -EIO;
	}

	return 0;
}

static int mtk_nfc_hw_runtime_config(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct mtk_nfc_nand_chip *mtk_nand = to_mtk_nand(chip);
	struct mtk_nfc *nfc = nand_get_controller_data(chip);
	u32 fmt, spare = mtk_nand->sparesize;

	switch (mtd->writesize) {
	case 512:
		fmt = PAGEFMT_512_2K | PAGEFMT_SEC_SEL_512;
		break;
	case KB(2):
		fmt = PAGEFMT_512_2K;
		break;
	case KB(4):
		fmt = PAGEFMT_2K_4K;
		break;
	case KB(8):
		fmt = PAGEFMT_4K_8K;
		break;
	default:
		dev_err(nfc->dev, "invalid page len: %d\n", mtd->writesize);
		return -EINVAL;
	}

	mtd->oobsize = mtk_nand->sparesize * (mtd->writesize / data_len(chip));

	if (mtd->writesize > 512)
		spare >>= 1;

	switch (spare) {
	case 16:
		fmt |= (PAGEFMT_SPARE_16 << PAGEFMT_SPARE_SHIFT);
		break;
	case 28:
		fmt |= (PAGEFMT_SPARE_28 << PAGEFMT_SPARE_SHIFT);
		break;
	default:
		break;
	}
	fmt |= MTKSDG1_NFI_FDM_REG_SIZE << PAGEFMT_FDM_SHIFT;
	fmt |= MTKSDG1_NFI_FDM_REG_SIZE << PAGEFMT_FDM_ECC_SHIFT;
	sdg1_writew(nfc, fmt, MTKSDG1_NFI_PAGEFMT);

	sdg1_ecc_config(nfc->ecc, chip->ecc.strength, sdg1_step_len(chip));

	return 0;
}

static void mtk_nfc_select_chip(struct mtd_info *mtd, int chip)
{
	struct nand_chip *nand = mtd_to_nand(mtd);
	struct mtk_nfc *nfc = nand_get_controller_data(nand);
	struct mtk_nfc_nand_chip *mtk_nand = to_mtk_nand(nand);

	if (chip < 0)
		return;

	mtk_nfc_hw_runtime_config(mtd);

	sdg1_writel(nfc, mtk_nand->sels[chip], MTKSDG1_NFI_CSEL);
}

static int mtk_nfc_dev_ready(struct mtd_info *mtd)
{
	struct mtk_nfc *nfc = nand_get_controller_data(mtd_to_nand(mtd));

	if (sdg1_readl(nfc, MTKSDG1_NFI_STA) & STA_BUSY)
		return 0;

	return 1;
}

static void mtk_nfc_cmd_ctrl(struct mtd_info *mtd, int dat, unsigned int ctrl)
{
	struct mtk_nfc *nfc = nand_get_controller_data(mtd_to_nand(mtd));

	if (ctrl & NAND_ALE) {
		mtk_nfc_send_address(nfc, dat);
	} else if (ctrl & NAND_CLE) {
		mtk_nfc_hw_reset(nfc);

		sdg1_writew(nfc, CNFG_OP_CUST, MTKSDG1_NFI_CNFG);
		mtk_nfc_send_command(nfc, dat);
	}
}

static inline void mtk_nfc_wait_ioready(struct mtk_nfc *nfc)
{
	int rc;
	u8 val;

	rc = readb_poll_timeout_atomic(nfc->regs + MTKSDG1_NFI_PIO_DIRDY, val,
					val & PIO_DI_RDY, 10, MTK_TIMEOUT);
	if (rc < 0)
		dev_err(nfc->dev, "data not ready\n");
}

static inline uint8_t mtk_nfc_read_byte(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct mtk_nfc *nfc = nand_get_controller_data(chip);
	u32 reg;

	reg = sdg1_readl(nfc, MTKSDG1_NFI_STA) & NFI_FSM_MASK;
	if (reg != NFI_FSM_CUSTDATA) {
		reg = sdg1_readw(nfc, MTKSDG1_NFI_CNFG);
		reg |= CNFG_BYTE_RW | CNFG_READ_EN;
		sdg1_writew(nfc, reg, MTKSDG1_NFI_CNFG);

		reg = (MTK_MAX_SECTOR << CON_SEC_SHIFT) | CON_BRD;
		sdg1_writel(nfc, reg, MTKSDG1_NFI_CON);

		sdg1_writew(nfc, 1, MTKSDG1_NFI_STRDATA);
		udelay(10);
	}

	mtk_nfc_wait_ioready(nfc);

	return sdg1_readb(nfc, MTKSDG1_NFI_DATAR);
}

static void mtk_nfc_read_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
	int i;

	for (i = 0; i < len; i++)
		buf[i] = mtk_nfc_read_byte(mtd);
}

static void mtk_nfc_write_byte(struct mtd_info *mtd, uint8_t byte)
{
	struct mtk_nfc *nfc = nand_get_controller_data(mtd_to_nand(mtd));
	u32 reg;

	reg = sdg1_readl(nfc, MTKSDG1_NFI_STA) & NFI_FSM_MASK;

	if (reg != NFI_FSM_CUSTDATA) {
		reg = sdg1_readw(nfc, MTKSDG1_NFI_CNFG) | CNFG_BYTE_RW;
		sdg1_writew(nfc, reg, MTKSDG1_NFI_CNFG);

		reg = MTK_MAX_SECTOR << CON_SEC_SHIFT | CON_BWR;
		sdg1_writel(nfc, reg, MTKSDG1_NFI_CON);

		sdg1_writew(nfc, 1, MTKSDG1_NFI_STRDATA);
	}

	mtk_nfc_wait_ioready(nfc);
	sdg1_writeb(nfc, byte, MTKSDG1_NFI_DATAW);
}

static int mtk_nfc_sector_encode(struct nand_chip *chip, u8 *data)
{
	struct mtk_nfc *nfc = nand_get_controller_data(chip);
	struct sdg1_enc_data enc_data = {
		.strength = chip->ecc.strength,
		.len = sdg1_step_len(chip),
		.data = data,
	};

	return sdg1_ecc_start_encode(nfc->ecc, &enc_data);
}

static int mtk_nfc_format_subpage(struct mtd_info *mtd, uint32_t offset,
			uint32_t len, const uint8_t *buf, int oob_on)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct mtk_nfc *nfc = nand_get_controller_data(chip);
	u32 start, end;
	int i, ret;

	start = offset / data_len(chip);
	end = DIV_ROUND_UP(offset + len, data_len(chip));

	memset(nfc->buffer, 0xff, mtd->writesize + mtd->oobsize);
	for (i = 0; i < chip->ecc.steps; i++) {

		memcpy(sdg1_data_ptr(chip, i), data_ptr(chip, buf, i),
			data_len(chip));

		if (i < start || i >= end)
			continue;

		if (oob_on)
			memcpy(sdg1_oob_ptr(chip, i), oob_ptr(chip, i),
				sdg1_oob_len());

		/* program the CRC back to the OOB */
		ret = mtk_nfc_sector_encode(chip, sdg1_data_ptr(chip, i));
		if (ret < 0)
			return ret;
	}

	return 0;
}

static void mtk_nfc_format_page(struct mtd_info *mtd, const uint8_t *buf,
				int oob_on)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct mtk_nfc *nfc = nand_get_controller_data(chip);
	u32 i;

	memset(nfc->buffer, 0xff, mtd->writesize + mtd->oobsize);
	for (i = 0; i < chip->ecc.steps; i++) {
		if (buf)
			memcpy(sdg1_data_ptr(chip, i), data_ptr(chip, buf, i),
				data_len(chip));
		if (oob_on)
			memcpy(sdg1_oob_ptr(chip, i), oob_ptr(chip, i),
				sdg1_oob_len());
	}
}

static inline void mtk_nfc_read_fdm(struct nand_chip *chip, u32 sectors)
{
	struct mtk_nfc *nfc = nand_get_controller_data(chip);
	u32 *p;
	int i;

	for (i = 0; i < sectors; i++) {
		p = (u32 *) oob_ptr(chip, i);
		p[0] = sdg1_readl(nfc, MTKSDG1_NFI_FDM0L + i * sdg1_oob_len());
		p[1] = sdg1_readl(nfc, MTKSDG1_NFI_FDM0M + i * sdg1_oob_len());
	}
}

static inline void mtk_nfc_write_fdm(struct nand_chip *chip)
{
	struct mtk_nfc *nfc = nand_get_controller_data(chip);
	u32 *p;
	int i;

	for (i = 0; i < chip->ecc.steps ; i++) {
		p = (u32 *) oob_ptr(chip, i);
		sdg1_writel(nfc, p[0], MTKSDG1_NFI_FDM0L + i * sdg1_oob_len());
		sdg1_writel(nfc, p[1], MTKSDG1_NFI_FDM0M + i * sdg1_oob_len());
	}
}

static int mtk_nfc_do_write_page(struct mtd_info *mtd, struct nand_chip *chip,
	const uint8_t *buf, int page, int len)
{

	struct mtk_nfc *nfc = nand_get_controller_data(chip);
	struct device *dev = nfc->dev;
	dma_addr_t addr;
	u32 reg;
	int ret;

	addr = dma_map_single(dev, (void *) buf, len, DMA_TO_DEVICE);
	ret = dma_mapping_error(nfc->dev, addr);
	if (ret) {
		dev_err(nfc->dev, "dma mapping error\n");
		return -EINVAL;
	}

	reg = sdg1_readw(nfc, MTKSDG1_NFI_CNFG) | CNFG_AHB | CNFG_DMA_BURST_EN;
	sdg1_writew(nfc, reg, MTKSDG1_NFI_CNFG);

	sdg1_writel(nfc, chip->ecc.steps << CON_SEC_SHIFT, MTKSDG1_NFI_CON);
	sdg1_writel(nfc, lower_32_bits(addr), MTKSDG1_NFI_STRADDR);
	sdg1_writew(nfc, INTR_AHB_DONE_EN, MTKSDG1_NFI_INTR_EN);

	init_completion(&nfc->done);

	reg = sdg1_readl(nfc, MTKSDG1_NFI_CON) | CON_BWR;
	sdg1_writel(nfc, reg, MTKSDG1_NFI_CON);
	sdg1_writew(nfc, 1, MTKSDG1_NFI_STRDATA);

	ret = wait_for_completion_timeout(&nfc->done, msecs_to_jiffies(500));
	if (!ret) {
		dev_err(dev, "program ahb done timeout\n");
		sdg1_writew(nfc, 0, MTKSDG1_NFI_INTR_EN);
		ret = -ETIMEDOUT;
		goto timeout;
	}

	ret = readl_poll_timeout_atomic(nfc->regs + MTKSDG1_NFI_ADDRCNTR, reg,
			(reg & CNTR_MASK) >= chip->ecc.steps, 10, MTK_TIMEOUT);
	if (ret)
		dev_err(dev, "hwecc write timeout\n");

timeout:

	dma_unmap_single(nfc->dev, addr, len, DMA_TO_DEVICE);
	sdg1_writel(nfc, 0, MTKSDG1_NFI_CON);

	return ret;
}

static int mtk_nfc_write_page(struct mtd_info *mtd, struct nand_chip *chip,
			const uint8_t *buf, int oob_on, int page, int raw)
{
	struct mtk_nfc *nfc = nand_get_controller_data(chip);
	size_t len;
	u32 reg;
	int ret;

	if (!raw) {
		/* OOB => FDM: from register,  ECC: from HW */
		reg = sdg1_readw(nfc, MTKSDG1_NFI_CNFG) | CNFG_AUTO_FMT_EN;
		sdg1_writew(nfc, reg | CNFG_HW_ECC_EN, MTKSDG1_NFI_CNFG);

		sdg1_ecc_enable_encode(nfc->ecc);

		/* write OOB into the FDM registers (OOB area in MTK NAND) */
		if (oob_on)
			mtk_nfc_write_fdm(chip);
	}

	len = mtd->writesize + (raw ? mtd->oobsize : 0);
	ret = mtk_nfc_do_write_page(mtd, chip, buf, page, len);

	if (!raw)
		sdg1_ecc_disable_encode(nfc->ecc);

	return ret;
}

static int mtk_nfc_write_page_hwecc(struct mtd_info *mtd,
			struct nand_chip *chip, const uint8_t *buf,
			int oob_on, int page)
{
	return mtk_nfc_write_page(mtd, chip, buf, oob_on, page, 0);
}

static int mtk_nfc_write_page_raw(struct mtd_info *mtd, struct nand_chip *chip,
					const uint8_t *buf, int oob_on, int pg)
{
	struct mtk_nfc *nfc = nand_get_controller_data(chip);

	mtk_nfc_format_page(mtd, buf, oob_on);
	return mtk_nfc_write_page(mtd, chip, nfc->buffer, 0, pg, 1);
}

static int mtk_nfc_write_subpage_hwecc(struct mtd_info *mtd,
		struct nand_chip *chip, uint32_t offset, uint32_t data_len,
		const uint8_t *buf, int oob_on, int page)
{
	struct mtk_nfc *nfc = nand_get_controller_data(chip);
	int ret;

	ret = mtk_nfc_format_subpage(mtd, offset, data_len, buf, oob_on);
	if (ret < 0)
		return ret;

	/* use the data in the private buffer (now with FDM and CRC) */
	return mtk_nfc_write_page(mtd, chip, nfc->buffer, 0, page, 1);
}

static int mtk_nfc_write_oob(struct mtd_info *mtd, struct nand_chip *chip,
				int page)
{
	u8 *data = chip->buffers->databuf;
	int ret;

	memset(data, 0xff, mtd->writesize);

	chip->cmdfunc(mtd, NAND_CMD_SEQIN, 0x00, page);

	ret = mtk_nfc_write_page_hwecc(mtd, chip, data, 1, page);
	if (ret < 0)
		return -EIO;

	chip->cmdfunc(mtd, NAND_CMD_PAGEPROG, -1, -1);
	ret = chip->waitfunc(mtd, chip);

	return ret & NAND_STATUS_FAIL ? -EIO : 0;
}

static int mtk_nfc_write_oob_raw(struct mtd_info *mtd, struct nand_chip *chip,
					int page)
{
	int ret;

	chip->cmdfunc(mtd, NAND_CMD_SEQIN, 0x00, page);
	ret = mtk_nfc_write_page_raw(mtd, chip, NULL, 1, page);
	if (ret < 0)
		return -EIO;

	chip->cmdfunc(mtd, NAND_CMD_PAGEPROG, -1, -1);
	ret = chip->waitfunc(mtd, chip);

	return ret & NAND_STATUS_FAIL ? -EIO : 0;
}

static int mtk_nfc_update_oob(struct mtd_info *mtd, struct nand_chip *chip,
				u8 *buf, u32 sectors)
{
	struct mtk_nfc *nfc = nand_get_controller_data(chip);
	struct sdg1_ecc_stats stats = { .sectors = sectors };
	int rc, i;

	rc = sdg1_readl(nfc, MTKSDG1_NFI_STA) & STA_EMP_PAGE;
	if (rc) {
		memset(buf, 0xff, sectors * data_len(chip));

		for (i = 0; i < sectors; i++)
			memset(oob_ptr(chip, i), 0xff, sdg1_oob_len());

		return 0;
	}

	mtk_nfc_read_fdm(chip, sectors);

	sdg1_ecc_get_stats(nfc->ecc, &stats);
	mtd->ecc_stats.corrected += stats.corrected;
	mtd->ecc_stats.failed += stats.failed;

	return stats.bitflips;
}

static int mtk_nfc_block_markbad(struct mtd_info *mtd, loff_t ofs)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	u8 *buf = chip->buffers->databuf;
	int page, rc, i;

	memset(buf, 0x00, mtd->writesize + mtd->oobsize);

	if (chip->bbt_options & NAND_BBT_SCANLASTPAGE)
		ofs += mtd->erasesize - mtd->writesize;

	i = 0;
	do {
		page = (int)(ofs >> chip->page_shift);
		chip->cmdfunc(mtd, NAND_CMD_SEQIN, 0x00, page);
		rc = mtk_nfc_write_page(mtd, chip, buf, 0, page, 1);
		if (rc < 0)
			return rc;

		chip->cmdfunc(mtd, NAND_CMD_PAGEPROG, -1, -1);
		rc = chip->waitfunc(mtd, chip);
		rc = rc & NAND_STATUS_FAIL ? -EIO : 0;
		if (rc < 0)
			return rc;

		ofs += mtd->writesize;
		i++;

	} while ((chip->bbt_options & NAND_BBT_SCAN2NDPAGE) && i < 2);

	return 0;
}

static int mtk_nfc_read_subpage(struct mtd_info *mtd, struct nand_chip *chip,
		uint32_t data_offs, uint32_t readlen, uint8_t *bufpoi,
		int page, int raw)
{
	struct mtk_nfc *nfc = nand_get_controller_data(chip);
	u32 column, spare, sectors, start, end, reg;
	dma_addr_t addr;
	int bitflips;
	size_t len;
	u8 *buf;
	int rc;

	start = data_offs / data_len(chip);
	end = DIV_ROUND_UP(data_offs + readlen, data_len(chip));

	sectors = end - start;
	spare = mtd->oobsize / chip->ecc.steps;
	column =  start * (data_len(chip) + spare);

	len = sectors * data_len(chip) + (raw ? sectors * spare : 0);
	buf = bufpoi + start * data_len(chip);

	if (column != 0)
		chip->cmdfunc(mtd, NAND_CMD_RNDOUT, column, -1);

	addr = dma_map_single(nfc->dev, buf, len, DMA_FROM_DEVICE);
	rc = dma_mapping_error(nfc->dev, addr);
	if (rc) {
		dev_err(nfc->dev, "dma mapping error\n");
		return -EINVAL;
	}

	reg = sdg1_readw(nfc, MTKSDG1_NFI_CNFG);
	reg |= CNFG_READ_EN | CNFG_DMA_BURST_EN | CNFG_AHB;
	if (!raw) {
		reg |= CNFG_AUTO_FMT_EN | CNFG_HW_ECC_EN;
		sdg1_writew(nfc, reg, MTKSDG1_NFI_CNFG);

		sdg1_ecc_enable_decode(nfc->ecc);
	} else
		sdg1_writew(nfc, reg, MTKSDG1_NFI_CNFG);

	sdg1_writel(nfc, sectors << CON_SEC_SHIFT, MTKSDG1_NFI_CON);
	sdg1_writew(nfc, INTR_AHB_DONE_EN, MTKSDG1_NFI_INTR_EN);
	sdg1_writel(nfc, lower_32_bits(addr), MTKSDG1_NFI_STRADDR);

	if (!raw)
		sdg1_ecc_start_decode(nfc->ecc, sectors);

	init_completion(&nfc->done);
	reg = sdg1_readl(nfc, MTKSDG1_NFI_CON) | CON_BRD;
	sdg1_writel(nfc, reg, MTKSDG1_NFI_CON);
	sdg1_writew(nfc, 1, MTKSDG1_NFI_STRDATA);

	rc = wait_for_completion_timeout(&nfc->done, msecs_to_jiffies(500));
	if (!rc)
		dev_warn(nfc->dev, "read ahb/dma done timeout\n");

	rc = readl_poll_timeout_atomic(nfc->regs + MTKSDG1_NFI_BYTELEN, reg,
				(reg & CNTR_MASK) >= sectors, 10, MTK_TIMEOUT);
	if (rc < 0) {
		dev_err(nfc->dev, "subpage done timeout\n");
		bitflips = -EIO;
	} else {
		bitflips = 0;
		if (!raw) {
			rc = sdg1_ecc_wait_decode(nfc->ecc);
			bitflips = rc < 0 ? -ETIMEDOUT :
				mtk_nfc_update_oob(mtd, chip, buf, sectors);
		}
	}

	dma_unmap_single(nfc->dev, addr, len, DMA_FROM_DEVICE);

	if (!raw)
		sdg1_ecc_disable_decode(nfc->ecc);

	sdg1_writel(nfc, 0, MTKSDG1_NFI_CON);

	return bitflips;
}

static int mtk_nfc_read_subpage_hwecc(struct mtd_info *mtd,
	struct nand_chip *chip, uint32_t off, uint32_t len, uint8_t *p, int pg)
{
	return mtk_nfc_read_subpage(mtd, chip, off, len, p, pg, 0);
}

static int mtk_nfc_read_page_hwecc(struct mtd_info *mtd,
	struct nand_chip *chip, uint8_t *p, int oob_on, int pg)
{
	return mtk_nfc_read_subpage_hwecc(mtd, chip, 0, mtd->writesize, p, pg);
}

static int mtk_nfc_read_page_raw(struct mtd_info *mtd, struct nand_chip *chip,
				uint8_t *buf, int oob_on, int page)
{
	struct mtk_nfc *nfc = nand_get_controller_data(chip);
	int i, ret;

	memset(nfc->buffer, 0xff, mtd->writesize + mtd->oobsize);
	ret = mtk_nfc_read_subpage(mtd, chip, 0, mtd->writesize, nfc->buffer,
					page, 1);
	if (ret < 0)
		return ret;

	for (i = 0; i < chip->ecc.steps; i++) {
		if (buf)
			memcpy(data_ptr(chip, buf, i), sdg1_data_ptr(chip, i),
							data_len(chip));
		if (oob_on)
			memcpy(oob_ptr(chip, i), sdg1_oob_ptr(chip, i),
							sdg1_oob_len());
	}

	return ret;
}

static void mtk_nfc_switch_oob(struct mtd_info *mtd, struct nand_chip *chip,
					uint8_t *buf)
{
	u32 *poi, *oob;
	size_t spare;
	u32 sectors;
	int i;

	spare = mtd->oobsize / chip->ecc.steps;
	sectors = mtd->writesize / (data_len(chip) + spare);

	poi = (u32 *) (buf + mtd->writesize - sectors * spare);
	oob = (u32 *) oob_ptr(chip, 0);

	for (i = 0; i < sdg1_oob_len() / sizeof(u32); i++) {
		poi[i] = poi[i] ^ oob[i];
		oob[i] = poi[i] ^ oob[i];
		poi[i] = poi[i] ^ oob[i];
	}

}

static int mtk_nfc_read_oob(struct mtd_info *mtd, struct nand_chip *chip,
				int page)
{
	struct mtk_nfc *nfc = nand_get_controller_data(chip);
	u8 *buf = chip->buffers->databuf;
	struct mtd_ecc_stats stats;
	int ret;

	stats = mtd->ecc_stats;

	memset(buf, 0xff, mtd->writesize);
	chip->cmdfunc(mtd, NAND_CMD_READ0, 0, page);

	ret = mtk_nfc_read_page_hwecc(mtd, chip, buf, 1, page);

	if (nfc->switch_oob)
		mtk_nfc_switch_oob(mtd, chip, buf);

	if (ret < mtd->bitflip_threshold)
		mtd->ecc_stats.corrected = stats.corrected;

	return ret;
}

static int mtk_nfc_read_oob_raw(struct mtd_info *mtd, struct nand_chip *chip,
				int page)
{
	chip->cmdfunc(mtd, NAND_CMD_READ0, 0, page);

	return mtk_nfc_read_page_raw(mtd, chip, NULL, 1, page);
}


static inline void mtk_nfc_hw_init(struct mtk_nfc *nfc)
{
	sdg1_writel(nfc, 0x10804211, MTKSDG1_NFI_ACCCON);
	sdg1_writew(nfc, 0xf1, MTKSDG1_NFI_CNRNB);
	sdg1_writew(nfc, PAGEFMT_8K_16K, MTKSDG1_NFI_PAGEFMT);

	mtk_nfc_hw_reset(nfc);

	sdg1_readl(nfc, MTKSDG1_NFI_INTR_STA);
	sdg1_writel(nfc, 0, MTKSDG1_NFI_INTR_EN);

	sdg1_ecc_hw_init(nfc->ecc);
}

static irqreturn_t mtk_nfc_irq(int irq, void *id)
{
	struct mtk_nfc *nfc = id;
	u16 sta, ien;

	sta = sdg1_readw(nfc, MTKSDG1_NFI_INTR_STA);
	ien = sdg1_readw(nfc, MTKSDG1_NFI_INTR_EN);

	if (!(sta & ien))
		return IRQ_NONE;

	sdg1_writew(nfc, ~sta & ien, MTKSDG1_NFI_INTR_EN);
	complete(&nfc->done);

	return IRQ_HANDLED;
}

static int mtk_nfc_enable_clk(struct device *dev, struct mtk_nfc_clk *clk)
{
	int ret;

	ret = clk_prepare_enable(clk->nfi_clk);
	if (ret) {
		dev_err(dev, "failed to enable nfi clk\n");
		return ret;
	}

	ret = clk_prepare_enable(clk->pad_clk);
	if (ret) {
		dev_err(dev, "failed to enable pad clk\n");
		clk_disable_unprepare(clk->nfi_clk);
		return ret;
	}

	return 0;
}

static void mtk_nfc_disable_clk(struct mtk_nfc_clk *clk)
{
	clk_disable_unprepare(clk->nfi_clk);
	clk_disable_unprepare(clk->pad_clk);
}

static int mtk_nfc_ooblayout_free(struct mtd_info *mtd, int section,
				struct mtd_oob_region *oob_region)
{
	struct nand_chip *chip = mtd_to_nand(mtd);

	if (section)
		return -ERANGE;

	oob_region->length = sdg1_oob_len() * chip->ecc.steps;
	oob_region->offset = 0;

	return 0;
}

static int mtk_nfc_ooblayout_ecc(struct mtd_info *mtd, int section,
				struct mtd_oob_region *oob_region)
{
	struct mtd_oob_region free_region;

	if (section)
		return -ERANGE;

	mtk_nfc_ooblayout_free(mtd, 0, &free_region);

	oob_region->length = mtd->oobsize - free_region.length;
	oob_region->offset = free_region.length;

	return 0;
}

static const struct mtd_ooblayout_ops mtk_nfc_ooblayout_ops = {
	.free = mtk_nfc_ooblayout_free,
	.ecc = mtk_nfc_ooblayout_ecc,
};

static int mtk_nfc_nand_chip_init(struct device *dev, struct mtk_nfc *nfc,
				struct device_node *np)
{
	struct mtk_nfc_nand_chip *chip;
	struct nand_chip *nand;
	struct mtd_info *mtd;
	int nsels, len;
	u32 tmp;
	int ret;
	int i;

	if (!of_get_property(np, "reg", &nsels))
		return -EINVAL;

	nsels /= sizeof(u32);
	if (!nsels || nsels > MTK_NAND_MAX_NSELS) {
		dev_err(dev, "invalid reg property size %d\n", nsels);
		return -EINVAL;
	}

	chip = devm_kzalloc(dev,
			sizeof(*chip) + nsels * sizeof(u8), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->nsels = nsels;
	for (i = 0; i < nsels; i++) {
		ret = of_property_read_u32_index(np, "reg", i, &tmp);
		if (ret) {
			dev_err(dev, "reg property failure : %d\n", ret);
			return ret;
		}
		chip->sels[i] = tmp;
	}

	of_property_read_u32(np, "spare_per_sector", &chip->sparesize);

	nand = &chip->nand;
	nand->controller = &nfc->controller;

	nand_set_flash_node(nand, np);
	nand_set_controller_data(nand, nfc);

	nand->options |= NAND_USE_BOUNCE_BUFFER | NAND_SUBPAGE_READ;
	nand->block_markbad = mtk_nfc_block_markbad;
	nand->dev_ready = mtk_nfc_dev_ready;
	nand->select_chip = mtk_nfc_select_chip;
	nand->write_byte = mtk_nfc_write_byte;
	nand->read_byte = mtk_nfc_read_byte;
	nand->read_buf = mtk_nfc_read_buf;
	nand->cmd_ctrl = mtk_nfc_cmd_ctrl;
	nand->ecc.mode = NAND_ECC_HW;

	nand->ecc.write_subpage = mtk_nfc_write_subpage_hwecc;
	nand->ecc.write_page_raw = mtk_nfc_write_page_raw;
	nand->ecc.write_page = mtk_nfc_write_page_hwecc;
	nand->ecc.write_oob_raw = mtk_nfc_write_oob_raw;
	nand->ecc.write_oob = mtk_nfc_write_oob;

	nand->ecc.read_subpage = mtk_nfc_read_subpage_hwecc;
	nand->ecc.read_page_raw = mtk_nfc_read_page_raw;
	nand->ecc.read_oob_raw = mtk_nfc_read_oob_raw;
	nand->ecc.read_page = mtk_nfc_read_page_hwecc;
	nand->ecc.read_oob = mtk_nfc_read_oob;

	mtd = nand_to_mtd(nand);
	mtd->owner = THIS_MODULE;
	mtd->dev.parent = dev;
	mtd->name = MTK_NAME;
	mtd_set_ooblayout(mtd, &mtk_nfc_ooblayout_ops);

	mtk_nfc_hw_init(nfc);

	ret = nand_scan_ident(mtd, nsels, NULL);
	if (ret)
		return -ENODEV;

	len = mtd->writesize + mtd->oobsize;
	nfc->buffer = devm_kzalloc(dev, len, GFP_KERNEL);
	if (!nfc->buffer)
		return  -ENOMEM;

	/* create bbt table if not present */
	nfc->switch_oob = true;
	ret = nand_scan_tail(mtd);
	nfc->switch_oob = false;
	if (ret)
		return -ENODEV;

	ret = mtd_device_parse_register(mtd, NULL, NULL, NULL, 0);
	if (ret) {
		dev_err(dev, "mtd parse partition error\n");
		nand_release(mtd);
		return ret;
	}

	list_add_tail(&chip->node, &nfc->chips);

	return 0;
}

static int mtk_nfc_nand_chips_init(struct device *dev, struct mtk_nfc *nfc)
{
	struct device_node *np = dev->of_node;
	struct device_node *nand_np;
	int ret;

	for_each_child_of_node(np, nand_np) {
		ret = mtk_nfc_nand_chip_init(dev, nfc, nand_np);
		if (ret) {
			of_node_put(nand_np);
			return ret;
		}
	}

	return 0;
}

static int mtk_nfc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct mtk_nfc *nfc;
	struct resource *res;
	int ret, irq;

	nfc = devm_kzalloc(dev, sizeof(*nfc), GFP_KERNEL);
	if (!nfc)
		return -ENOMEM;

	spin_lock_init(&nfc->controller.lock);
	init_waitqueue_head(&nfc->controller.wq);
	INIT_LIST_HEAD(&nfc->chips);

	/* probe defer if not ready */
	nfc->ecc = of_sdg1_ecc_get(np);
	if (IS_ERR(nfc->ecc))
		return PTR_ERR(nfc->ecc);
	else if (!nfc->ecc)
		return -ENODEV;

	nfc->dev = dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	nfc->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(nfc->regs)) {
		ret = PTR_ERR(nfc->regs);
		dev_err(dev, "no nfi base\n");
		goto release_ecc;
	}

	nfc->clk.nfi_clk = devm_clk_get(dev, "nfi_clk");
	if (IS_ERR(nfc->clk.nfi_clk)) {
		dev_err(dev, "no clk\n");
		ret = PTR_ERR(nfc->clk.nfi_clk);
		goto release_ecc;
	}

	nfc->clk.pad_clk = devm_clk_get(dev, "pad_clk");
	if (IS_ERR(nfc->clk.pad_clk)) {
		dev_err(dev, "no pad clk\n");
		ret = PTR_ERR(nfc->clk.pad_clk);
		goto release_ecc;
	}

	ret = mtk_nfc_enable_clk(dev, &nfc->clk);
	if (ret)
		goto release_ecc;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(dev, "no nfi irq resource\n");
		ret = -EINVAL;
		goto clk_disable;
	}

	ret = devm_request_irq(dev, irq, mtk_nfc_irq, 0x0, "sdg1-nand", nfc);
	if (ret) {
		dev_err(dev, "failed to request nfi irq\n");
		goto clk_disable;
	}

	ret = dma_set_mask(dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(dev, "failed to set dma mask\n");
		goto clk_disable;
	}

	platform_set_drvdata(pdev, nfc);

	ret = mtk_nfc_nand_chips_init(dev, nfc);
	if (ret) {
		dev_err(dev, "failed to init nand chips\n");
		goto clk_disable;
	}

	return 0;

clk_disable:
	mtk_nfc_disable_clk(&nfc->clk);

release_ecc:
	sdg1_ecc_release(nfc->ecc);

	return ret;
}

static int mtk_nfc_remove(struct platform_device *pdev)
{
	struct mtk_nfc *nfc = platform_get_drvdata(pdev);
	struct mtk_nfc_nand_chip *chip;

	while (!list_empty(&nfc->chips)) {
		chip = list_first_entry(&nfc->chips, struct mtk_nfc_nand_chip,
					node);
		nand_release(nand_to_mtd(&chip->nand));
		list_del(&chip->node);
	}

	sdg1_ecc_release(nfc->ecc);
	mtk_nfc_disable_clk(&nfc->clk);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int mtk_nfc_suspend(struct device *dev)
{
	struct mtk_nfc *nfc = dev_get_drvdata(dev);
	struct mtk_nfc_saved_reg *reg = &nfc->saved_reg;

	reg->emp_thresh = sdg1_readl(nfc, MTKSDG1_NFI_EMPTY_THRESH);
	reg->pagefmt = sdg1_readw(nfc, MTKSDG1_NFI_PAGEFMT);
	reg->acccon = sdg1_readl(nfc, MTKSDG1_NFI_ACCCON);
	reg->cnrnb = sdg1_readw(nfc, MTKSDG1_NFI_CNRNB);
	reg->csel = sdg1_readw(nfc, MTKSDG1_NFI_CSEL);

	mtk_nfc_disable_clk(&nfc->clk);

	return 0;
}

static int mtk_nfc_resume(struct device *dev)
{
	struct mtk_nfc *nfc = dev_get_drvdata(dev);
	struct mtk_nfc_saved_reg *reg = &nfc->saved_reg;
	struct mtk_nfc_nand_chip *chip;
	struct nand_chip *nand;
	struct mtd_info *mtd;
	int ret;
	u32 i;

	udelay(200);

	ret = mtk_nfc_enable_clk(dev, &nfc->clk);
	if (ret)
		return ret;

	mtk_nfc_hw_init(nfc);

	list_for_each_entry(chip, &nfc->chips, node) {
		nand = &chip->nand;
		mtd = nand_to_mtd(nand);
		for (i = 0; i < chip->nsels; i++) {
			nand->select_chip(mtd, i);
			nand->cmdfunc(mtd, NAND_CMD_RESET, -1, -1);
		}
	}

	sdg1_writel(nfc, reg->emp_thresh, MTKSDG1_NFI_EMPTY_THRESH);
	sdg1_writew(nfc, reg->pagefmt, MTKSDG1_NFI_PAGEFMT);
	sdg1_writel(nfc, reg->acccon, MTKSDG1_NFI_ACCCON);
	sdg1_writew(nfc, reg->cnrnb, MTKSDG1_NFI_CNRNB);
	sdg1_writew(nfc, reg->csel, MTKSDG1_NFI_CSEL);

	return 0;
}
static SIMPLE_DEV_PM_OPS(mtk_nfc_pm_ops, mtk_nfc_suspend, mtk_nfc_resume);
#endif

static const struct of_device_id mtk_nfc_id_table[] = {
	{ .compatible = "mediatek,mt2701-nfc" },
	{}
};
MODULE_DEVICE_TABLE(of, mtk_nfc_id_table);

static struct platform_driver mtk_nfc_driver = {
	.probe  = mtk_nfc_probe,
	.remove = mtk_nfc_remove,
	.driver = {
		.name  = MTK_NAME,
		.of_match_table = mtk_nfc_id_table,
#ifdef CONFIG_PM_SLEEP
		.pm = &mtk_nfc_pm_ops,
#endif
	},
};

module_platform_driver(mtk_nfc_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Xiaolei Li <xiaolei.li@mediatek.com>");
MODULE_AUTHOR("Jorge Ramirez-Ortiz <jorge.ramirez-ortiz@linaro.org>");
MODULE_DESCRIPTION("MTK Nand Flash Controller Driver");
