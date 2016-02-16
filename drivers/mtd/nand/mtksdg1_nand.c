/*
 * MTK smart device NAND Flash controller driver.
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

#include <linux/of.h>
#include <linux/of_mtd.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include "mtksdg1_nand_nfc.h"	/* MTK NAND controller register */
#include "mtksdg1_nand_ecc.h"	/* MTK ECC engine register */

#define MTK_NAND_MAX_CHIP 2
#define MTK_DEFAULT_TIMEOUT (1 * HZ)
#define MTK_RESET_TIMEOUT_US 500
#define MTK_ECC_PARITY_BIT 14

struct mtk_nfc_clk {
	struct clk *nfi_clk;
	struct clk *nfiecc_clk;
	struct clk *pad_clk;
};

struct mtk_nfc_saved_reg {
	u16 nfi_pagefmt;
	u16 nfi_csel;
	u16 nfi_cnrnb;
	u32 nfi_emp_thresh;
	u32 ecc_enccnfg;
	u32 ecc_deccnfg;
	u32 nfi_acccon;
};

struct mtk_nfc_host {
	struct nand_chip chip;
	struct mtd_info mtd;
	struct mtk_nfc_saved_reg saved_reg;
	struct device *dev;
	void __iomem *nfi_base;
	void __iomem *nfiecc_base;
	struct mtk_nfc_clk clk;
	struct completion nfi_complete;
	struct completion ecc_complete;
	u32 sectorsize_shift;
	u32 fdm_size;
	u32 rownob;
	int exchange_oob; /* just enable during scan_bbt, to get real oob*/
	u32 ecc_sector; /* number of sector to do ecc correct*/
	u32 eccdone_sector; /* number of sectors ecc operation done*/
};

static void mtk_nfc_wait_sta_ready(struct mtk_nfc_host *host, u32 status)
{
	unsigned long timeout = jiffies + MTK_DEFAULT_TIMEOUT;

	do {
		if (!(readl(host->nfi_base + MTKSDG1_NFC_STA) & status))
			return;
	} while (time_before(jiffies, timeout));

	dev_err(host->dev, "wait for status %d timedout\n", status);
}

static void mtk_nfc_wait_pio_dirdy(struct mtk_nfc_host *host)
{
	unsigned long timeout = jiffies + MTK_DEFAULT_TIMEOUT;

	do {
		if (readb(host->nfi_base + MTKSDG1_NFC_PIO_DIRDY) & 0x1)
			return;
	} while (time_before(jiffies, timeout));

	dev_err(host->dev, "wait for PIO DIRDY timedout\n");
}

static void mtk_nfc_wait_cntrdone(struct mtk_nfc_host *host, u32 sector)
{
	unsigned long timeout = jiffies + MTK_DEFAULT_TIMEOUT;

	do {
		if (((readl(host->nfi_base + MTKSDG1_NFC_ADDRCNTR)
			>> CNTR_SHIFT) & CNTR_MASK) == sector)
			return;
	} while (time_before(jiffies, timeout));

	dev_err(host->dev, "wait for cntr %d done timedout\n", sector);
}

static void mtk_nfc_wait_bytelen(struct mtk_nfc_host *host, u32 sector)
{
	unsigned long timeout = jiffies + MTK_DEFAULT_TIMEOUT;

	do {
		if (((readl(host->nfi_base + MTKSDG1_NFC_BYTELEN)
			>> CNTR_SHIFT) & CNTR_MASK) == sector)
			return;
	} while (time_before(jiffies, timeout));

	dev_err(host->dev, "wait for bytelen %d done timedout\n", sector);
}

static void mtk_nfc_wait_encidle(struct mtk_nfc_host *host)
{
	unsigned long timeout = jiffies + MTK_DEFAULT_TIMEOUT;

	do {
		if (readl(host->nfiecc_base + MTKSDG1_ECC_ENCIDLE) & ENC_IDLE)
			return;
	} while (time_before(jiffies, timeout));

	dev_err(host->dev, "wait for encode idle timedout\n");
}

static void mtk_nfc_wait_decidle(struct mtk_nfc_host *host)
{
	unsigned long timeout = jiffies + MTK_DEFAULT_TIMEOUT;

	do {
		if (readl(host->nfiecc_base + MTKSDG1_ECC_DECIDLE) & DEC_IDLE)
			return;
	} while (time_before(jiffies, timeout));

	dev_err(host->dev, "wait for decode idle timedout\n");
}

static void mtk_nfc_set_command(struct mtk_nfc_host *host, u8 command)
{
	writel(command, host->nfi_base + MTKSDG1_NFC_CMD);
	mtk_nfc_wait_sta_ready(host, STA_CMD);
}

static void mtk_nfc_set_address(struct mtk_nfc_host *host, u32 column, u32 row,
		u8 colnob, u8 rownob)
{
	writel(column, host->nfi_base + MTKSDG1_NFC_COLADDR);
	writel(row, host->nfi_base + MTKSDG1_NFC_ROWADDR);
	writel(colnob | (rownob << ADDR_ROW_NOB_SHIFT),
		host->nfi_base + MTKSDG1_NFC_ADDRNOB);
	mtk_nfc_wait_sta_ready(host, STA_ADDR);
}

static struct nand_ecclayout nand_4k_128 = {
	.oobavail = 32,
	.oobfree = { {0, 32} },
};

static void mtk_nfc_hw_config(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	struct mtk_nfc_host *host = chip->priv;
	u32 spare_per_sector, ecc_bit, spare_bit, pagesize_bit, reg_val;
	u32 dec_size, enc_size, ecc_level;

	host->sectorsize_shift = 10;
	host->fdm_size = 8;
	if (chip->chipsize > (128 << 20))
		host->rownob = 3;
	else if (chip->chipsize > (32 << 20))
		host->rownob = 2;
	else
		host->rownob = 1;

	spare_per_sector =
		mtd->oobsize / (mtd->writesize >> host->sectorsize_shift);

	switch (spare_per_sector) {
	case 16:
		spare_bit = PAGEFMT_SPARE_16;
		ecc_bit = ECC_CNFG_4BIT;
		ecc_level = 4;
		break;
	case 32:
		spare_bit = PAGEFMT_SPARE_16;
		ecc_bit = ECC_CNFG_12BIT;
		ecc_level = 12;
		break;
	default:
		dev_err(host->dev,
			"spare size %d is invalid\n", spare_per_sector);
		break;
	}

	chip->ecc.size = 1 << host->sectorsize_shift;
	chip->ecc.strength = ecc_level;

	switch (mtd->writesize) {
	case 2048:
		pagesize_bit = PAGEFMT_512_2K;
		break;
	case 4096:
		pagesize_bit = PAGEFMT_2K_4K;
		chip->ecc.layout = &nand_4k_128;
		break;
	case 8192:
		pagesize_bit = PAGEFMT_4K_8K;
		break;
	default:
		dev_err(host->dev,
			"page size %d is invalid\n", mtd->writesize);
		break;
	}

	reg_val = spare_bit << PAGEFMT_SPARE_SHIFT | pagesize_bit;
	reg_val |= host->fdm_size << PAGEFMT_FDM_SHIFT;
	reg_val |= host->fdm_size << PAGEFMT_FDM_ECC_SHIFT;
	writew(reg_val, host->nfi_base + MTKSDG1_NFC_PAGEFMT);

	enc_size = ((1 << host->sectorsize_shift) + host->fdm_size) << 3;
	dec_size = enc_size + ecc_level * MTK_ECC_PARITY_BIT;
	reg_val = ecc_bit | ECC_NFI_MODE | (enc_size << ECC_MS_SHIFT);
	writel(reg_val, host->nfiecc_base + MTKSDG1_ECC_ENCCNFG);
	reg_val = ecc_bit | ECC_NFI_MODE | (dec_size << ECC_MS_SHIFT);
	reg_val |= (DEC_CNFG_CORRECT | DEC_EMPTY_EN);
	writel(reg_val, host->nfiecc_base + MTKSDG1_ECC_DECCNFG);
}

static void mtk_nfc_hw_reset(struct mtk_nfc_host *host)
{
	unsigned long timeout = jiffies + MTK_DEFAULT_TIMEOUT;

	writel(CON_FIFO_FLUSH | CON_NFI_RST, host->nfi_base + MTKSDG1_NFC_CON);
	do {
		if (!(readl(host->nfi_base + MTKSDG1_NFC_MASTER_STA) & 0x0fff))
			break;
	} while (time_before(jiffies, timeout));
	if (time_after_eq(jiffies, timeout))
		dev_err(host->dev, "controller reset timeout\n");
	writel(CON_FIFO_FLUSH | CON_NFI_RST, host->nfi_base + MTKSDG1_NFC_CON);
};

static void mtk_nfc_device_reset(struct mtk_nfc_host *host)
{
	u16 chip;
	int ret;

	mtk_nfc_hw_reset(host);
	writew(INTR_RST_DONE_EN, host->nfi_base + MTKSDG1_NFC_INTR_EN);
	init_completion(&host->nfi_complete);
	writew(CNFG_OP_RESET, host->nfi_base + MTKSDG1_NFC_CNFG);
	mtk_nfc_set_command(host, NAND_CMD_RESET);
	ret = wait_for_completion_timeout(&host->nfi_complete,
					usecs_to_jiffies(MTK_RESET_TIMEOUT_US));
	chip = readw(host->nfi_base + MTKSDG1_NFC_CSEL);
	if (!ret)
		dev_err(host->dev, "device(%d) reset timeout!\n", chip);
}

static irqreturn_t mtk_nfi_irq_handle(int irq, void *devid)
{
	struct mtk_nfc_host *host = devid;
	u16 sta, ien;

	sta = readw(host->nfi_base + MTKSDG1_NFC_INTR_STA);
	ien = readw(host->nfi_base + MTKSDG1_NFC_INTR_EN);

	if (!(sta & ien))
		return IRQ_NONE;
	if ((sta & ien) == ien)
		complete(&host->nfi_complete);
	writew(~sta & ien, host->nfi_base + MTKSDG1_NFC_INTR_EN);
	return IRQ_HANDLED;
}

static irqreturn_t mtk_ecc_irq_handle(int irq, void *devid)
{
	struct mtk_nfc_host *host = devid;

	if (readw(host->nfiecc_base + MTKSDG1_ECC_DECIRQ_STA) & 0x1)
		host->eccdone_sector++;

	if (host->eccdone_sector == host->ecc_sector) {
		complete(&host->ecc_complete);
		writew(0, host->nfiecc_base + MTKSDG1_ECC_DECIRQ_EN);
	} else {
		return IRQ_NONE;
	}
	return IRQ_HANDLED;
}

static int mtk_nfc_enable_clk(struct device *dev, struct mtk_nfc_clk *clk)
{
	int ret;

	ret = clk_prepare_enable(clk->nfi_clk);
	if (ret) {
		dev_err(dev, "failed to enable nfi clk\n");
		goto out_nfi_clk_disable;
	}

	ret = clk_prepare_enable(clk->nfiecc_clk);
	if (ret) {
		dev_err(dev, "failed to enable nfiecc clk\n");
		goto out_nfiecc_clk_disable;
	}

	ret = clk_prepare_enable(clk->pad_clk);
	if (ret) {
		dev_err(dev, "failed to enable pad clk\n");
		goto out_pad_clk_disable;
	}

	return 0;

out_pad_clk_disable:
	clk_disable_unprepare(clk->nfiecc_clk);
out_nfiecc_clk_disable:
	clk_disable_unprepare(clk->nfi_clk);
out_nfi_clk_disable:
	return ret;
}

static void mtk_nfc_disable_clk(struct mtk_nfc_clk *clk)
{
	clk_disable_unprepare(clk->nfi_clk);
	clk_disable_unprepare(clk->nfiecc_clk);
	clk_disable_unprepare(clk->pad_clk);
}

static void mtk_nfc_select_chip(struct mtd_info *mtd, int chip)
{
	struct nand_chip *nand = mtd->priv;
	struct mtk_nfc_host *host = nand->priv;

	if ((chip >= 0) && (chip < MTK_NAND_MAX_CHIP))
		writel(chip, host->nfi_base + MTKSDG1_NFC_CSEL);
}

static void mtk_nfc_cmdfunc(struct mtd_info *mtd, unsigned command, int column,
		int page_addr)
{
	struct nand_chip *chip = mtd->priv;
	struct mtk_nfc_host *host = chip->priv;
	int ret;

	switch (command) {
	case NAND_CMD_RESET:
		mtk_nfc_device_reset(host);
		break;
	case NAND_CMD_READID:
		mtk_nfc_hw_reset(host);
		writew(CNFG_READ_EN | CNFG_BYTE_RW | CNFG_OP_SRD,
			host->nfi_base + MTKSDG1_NFC_CNFG);
		mtk_nfc_set_command(host, NAND_CMD_READID);
		mtk_nfc_set_address(host, column, 0, 1, 0);
		writel(CON_SRD, host->nfi_base + MTKSDG1_NFC_CON);
		break;
	case NAND_CMD_STATUS:
		mtk_nfc_hw_reset(host);
		writew(CNFG_READ_EN | CNFG_BYTE_RW | CNFG_OP_SRD,
			host->nfi_base + MTKSDG1_NFC_CNFG);
		mtk_nfc_set_command(host, NAND_CMD_STATUS);
		writel(CON_SRD, host->nfi_base + MTKSDG1_NFC_CON);
		break;
	case NAND_CMD_READOOB:
		mtk_nfc_hw_reset(host);
		writew(CNFG_READ_EN | CNFG_BYTE_RW | CNFG_OP_READ,
			host->nfi_base + MTKSDG1_NFC_CNFG);
		column += mtd->writesize;
		mtk_nfc_set_command(host, NAND_CMD_READ0);
		mtk_nfc_set_address(host, column, page_addr, 2, host->rownob);
		writel(CON_BRD | (1 << CON_SEC_SHIFT),
			host->nfi_base + MTKSDG1_NFC_CON);
		break;
	case NAND_CMD_ERASE1:
		mtk_nfc_hw_reset(host);
		writew(INTR_ERS_DONE_EN, host->nfi_base + MTKSDG1_NFC_INTR_EN);
		init_completion(&host->nfi_complete);
		writew(CNFG_OP_ERASE, host->nfi_base + MTKSDG1_NFC_CNFG);
		mtk_nfc_set_command(host, NAND_CMD_ERASE1);
		mtk_nfc_set_address(host, 0, page_addr, 0, host->rownob);
		break;
	case NAND_CMD_ERASE2:
		mtk_nfc_set_command(host, NAND_CMD_ERASE2);
		ret = wait_for_completion_timeout(&host->nfi_complete,
							MTK_DEFAULT_TIMEOUT);
		if (!ret)
			dev_err(host->dev, "erase timeout!\n");
		break;
	case NAND_CMD_SEQIN:
		mtk_nfc_hw_reset(host);
		writew(CNFG_OP_PRGM, host->nfi_base + MTKSDG1_NFC_CNFG);
		mtk_nfc_set_command(host, NAND_CMD_SEQIN);
		mtk_nfc_set_address(host, column, page_addr, 2, host->rownob);
		break;
	case NAND_CMD_PAGEPROG:
	case NAND_CMD_CACHEDPROG:
		writew(INTR_BUSY_RT_EN, host->nfi_base + MTKSDG1_NFC_INTR_EN);
		init_completion(&host->nfi_complete);
		mtk_nfc_set_command(host, command);
		ret = wait_for_completion_timeout(&host->nfi_complete,
							MTK_DEFAULT_TIMEOUT);
		if (!ret)
			dev_err(host->dev, "program busy return timeout!\n");
		break;
	case NAND_CMD_READ0:
		mtk_nfc_hw_reset(host);
		writew(CNFG_OP_READ | CNFG_READ_EN,
			host->nfi_base + MTKSDG1_NFC_CNFG);
		mtk_nfc_set_command(host, NAND_CMD_READ0);
		break;
	default:
		dev_err(host->dev, "command 0x%x is invalid\n", command);
		break;
	}
}

static uint8_t mtk_nfc_read_byte(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	struct mtk_nfc_host *host = chip->priv;

	mtk_nfc_wait_pio_dirdy(host);

	return readb(host->nfi_base + MTKSDG1_NFC_DATAR);
}

static void mtk_nfc_exchange_oob(struct mtd_info *mtd, struct nand_chip *chip,
					uint8_t *buf)
{
	struct mtk_nfc_host *host = chip->priv;
	u32 spare_per_sector, sector, sectorsize;
	u64 temp;
	u8 *bufpoi;

	spare_per_sector =
		mtd->oobsize / (mtd->writesize >> host->sectorsize_shift);
	sectorsize = (1 << host->sectorsize_shift) + spare_per_sector;
	sector = mtd->writesize / sectorsize;
	bufpoi = buf + mtd->writesize - (sector * spare_per_sector);

	memcpy(&temp, bufpoi, 8);
	memcpy(bufpoi, chip->oob_poi, 8);
	memcpy(chip->oob_poi, &temp, 8);
}

static void mtk_nfc_write_fdm(struct nand_chip *chip, u32 sectors,
				int oob_required)
{
	struct mtk_nfc_host *host = chip->priv;
	u32 fdm[2], i;

	for (i = 0; i < sectors; i++) {
		memset(fdm, 0xff, 8);
		if (oob_required)
			memcpy(fdm, chip->oob_poi + i * host->fdm_size,
				host->fdm_size);
		writel(fdm[0], host->nfi_base + MTKSDG1_NFC_FDM0L + (i << 3));
		writel(fdm[1], host->nfi_base + MTKSDG1_NFC_FDM0M + (i << 3));
	}
}

static int mtk_nfc_write_page_hwecc(struct mtd_info *mtd,
					struct nand_chip *chip,
					const uint8_t *buf, int oob_required,
					int page)
{
	struct mtk_nfc_host *host = chip->priv;
	u32 sectors = mtd->writesize >> host->sectorsize_shift;
	u32 reg_val;
	dma_addr_t dma_addr;
	int ret = 0;

	reg_val = readw(host->nfi_base + MTKSDG1_NFC_CNFG);
	reg_val |= CNFG_AUTO_FMT_EN | CNFG_HW_ECC_EN;
	reg_val |= CNFG_DMA_BURST_EN | CNFG_AHB;
	writew(reg_val, host->nfi_base + MTKSDG1_NFC_CNFG);
	mtk_nfc_wait_encidle(host);
	writew(ENC_EN, host->nfiecc_base + MTKSDG1_ECC_ENCCON);

	writel(sectors << CON_SEC_SHIFT, host->nfi_base + MTKSDG1_NFC_CON);
	writew(INTR_AHB_DONE_EN, host->nfi_base + MTKSDG1_NFC_INTR_EN);

	mtk_nfc_write_fdm(chip, sectors, oob_required);

	dma_addr = dma_map_single(host->dev, buf, mtd->writesize,
					DMA_TO_DEVICE);
	writel(lower_32_bits(dma_addr), host->nfi_base + MTKSDG1_NFC_STRADDR);

	init_completion(&host->nfi_complete);
	reg_val = readl(host->nfi_base + MTKSDG1_NFC_CON);
	reg_val |= CON_BWR;
	writel(reg_val, host->nfi_base + MTKSDG1_NFC_CON);
	ret = wait_for_completion_timeout(&host->nfi_complete,
						MTK_DEFAULT_TIMEOUT);
	if (!ret) {
		dev_err(host->dev, "program ahb done timeout!\n");
		ret = -ETIMEDOUT;
		goto timeout;
	} else {
		ret = 0;
	}
	mtk_nfc_wait_cntrdone(host, sectors);
	dma_unmap_single(host->dev, dma_addr, mtd->writesize, DMA_TO_DEVICE);
	mtk_nfc_wait_encidle(host);

timeout:
	writew(0, host->nfiecc_base + MTKSDG1_ECC_ENCCON);
	writel(0, host->nfi_base + MTKSDG1_NFC_CON);

	return ret;
}

static int mtk_nfc_write_subpage_hwecc(struct mtd_info *mtd,
					struct nand_chip *chip, uint32_t offset,
					uint32_t data_len, const uint8_t *buf,
					int oob_required, int page)
{
	return mtk_nfc_write_page_hwecc(mtd, chip, buf, oob_required, page);
}

static int mtk_nfc_write_page_raw(struct mtd_info *mtd, struct nand_chip *chip,
					const uint8_t *buf, int oob_required,
					int page)
{
	struct mtk_nfc_host *host = chip->priv;
	u32 sectors = mtd->writesize >> host->sectorsize_shift;
	u8 *oob = chip->oob_poi;
	u32 i, j, spare_per_sector, reg_val, sectorsize, sectorsparesize;

	spare_per_sector =
		mtd->oobsize / (mtd->writesize >> host->sectorsize_shift);
	sectorsize = 1 << host->sectorsize_shift;
	sectorsparesize = sectorsize + spare_per_sector;

	reg_val = readw(host->nfi_base + MTKSDG1_NFC_CNFG);
	reg_val |= CNFG_BYTE_RW;
	writew(reg_val, host->nfi_base + MTKSDG1_NFC_CNFG);

	writel((sectors << CON_SEC_SHIFT) | CON_BWR,
		host->nfi_base + MTKSDG1_NFC_CON);

	/* mtk one page data layout
	 * page = sector + sector + sector + ...
	 * sector = main data + fdm data + ecc parity data + pad dummy data
	 */
	for (i = 0; i < sectors; i++) {
		for (j = 0; j < sectorsparesize; j++) {
			mtk_nfc_wait_pio_dirdy(host);
			if (j < sectorsize)
				writel(*buf++,
					host->nfi_base + MTKSDG1_NFC_DATAW);
			else if (j < (sectorsize + host->fdm_size))
				writel(*oob++,
					host->nfi_base + MTKSDG1_NFC_DATAW);
			else
				writel(0xff,
					host->nfi_base + MTKSDG1_NFC_DATAW);
		}
	}
	mtk_nfc_wait_cntrdone(host, sectors);
	writel(0, host->nfi_base + MTKSDG1_NFC_CON);

	return 0;
}

static int mtk_nfc_write_oob(struct mtd_info *mtd, struct nand_chip *chip,
				int page)
{
	u8 *buf = chip->buffers->databuf;
	int status = 0;

	memset(buf, 0xff, mtd->writesize);
	chip->cmdfunc(mtd, NAND_CMD_SEQIN, 0x00, page);
	mtk_nfc_write_page_hwecc(mtd, chip, buf, 1, page);
	chip->cmdfunc(mtd, NAND_CMD_PAGEPROG, -1, -1);
	status = chip->waitfunc(mtd, chip);

	return status & NAND_STATUS_FAIL ? -EIO : 0;
}

static int mtk_nfc_write_oob_raw(struct mtd_info *mtd, struct nand_chip *chip,
					int page)
{
	u8 *buf = chip->buffers->databuf;
	int status = 0;

	memset(buf, 0xff, mtd->writesize);
	chip->cmdfunc(mtd, NAND_CMD_SEQIN, 0x00, page);
	mtk_nfc_write_page_raw(mtd, chip, buf, 1, page);
	chip->cmdfunc(mtd, NAND_CMD_PAGEPROG, -1, -1);
	status = chip->waitfunc(mtd, chip);

	return status & NAND_STATUS_FAIL ? -EIO : 0;
}

static void mtk_nfc_read_fdm(struct nand_chip *chip, u32 sectors)
{
	struct mtk_nfc_host *host = chip->priv;
	u32 fdm[2], i;

	for (i = 0; i < sectors; i++) {
		fdm[0] = readl(host->nfi_base + MTKSDG1_NFC_FDM0L + (i << 3));
		fdm[1] = readl(host->nfi_base + MTKSDG1_NFC_FDM0M + (i << 3));
		memcpy(chip->oob_poi + i * host->fdm_size, fdm, host->fdm_size);
	}
}

static int mtk_nfc_ecc_check(struct mtd_info *mtd, struct nand_chip *chip,
				u8 *buf, u32 sectors)
{
	struct mtk_nfc_host *host = chip->priv;
	struct nand_oobfree *free = chip->ecc.layout->oobfree;
	u32 i, err_num, max_bitflip = 0;

	if ((readl(host->nfi_base + MTKSDG1_NFC_STA) & STA_EMP_PAGE) != 0) {
		memset(buf, 0xff, sectors << host->sectorsize_shift);
		for (i = 0; i < sectors; i++)
			memset(chip->oob_poi + free[i].offset, 0xff,
				host->fdm_size);
		return 0;
	}

	for (i = 0; i < sectors; i++) {
		err_num = readl(host->nfiecc_base + MTKSDG1_ECC_DECENUM0
				+ ((i >> 2) << 2)) >> ((i % 4) << 3);
		err_num &= ERR_MASK;
		if (err_num == ERR_MASK) {
			mtd->ecc_stats.failed++;
		} else {
			mtd->ecc_stats.corrected += err_num;
			max_bitflip = max_t(u32, max_bitflip, err_num);
		}
	}

	return max_bitflip;
}

static int mtk_nfc_read_subpage(struct mtd_info *mtd, struct nand_chip *chip,
			uint32_t data_offs, uint32_t readlen, uint8_t *bufpoi,
			int page)
{
	struct mtk_nfc_host *host = chip->priv;
	u32 start_sector = data_offs >> host->sectorsize_shift;
	u32 sectors = (readlen + (1 << host->sectorsize_shift) - 1)
			>> host->sectorsize_shift;
	u32 reg_val, column, spare_per_sector, readsize;
	u8 *buf = bufpoi;
	dma_addr_t dma_addr;
	int ret = 0;

	spare_per_sector =
		mtd->oobsize / (mtd->writesize >> host->sectorsize_shift);
	column = start_sector
			* ((1 << host->sectorsize_shift) + spare_per_sector);
	readsize = sectors << host->sectorsize_shift;
	buf += column;

	reg_val = readw(host->nfi_base + MTKSDG1_NFC_CNFG);
	reg_val |= CNFG_AUTO_FMT_EN | CNFG_HW_ECC_EN;
	reg_val |= CNFG_DMA_BURST_EN | CNFG_AHB;
	writew(reg_val, host->nfi_base + MTKSDG1_NFC_CNFG);
	mtk_nfc_wait_decidle(host);
	writel(DEC_EN, host->nfiecc_base + MTKSDG1_ECC_DECCON);

	writel(sectors << CON_SEC_SHIFT, host->nfi_base + MTKSDG1_NFC_CON);

	writew(INTR_BUSY_RT_EN, host->nfi_base + MTKSDG1_NFC_INTR_EN);
	init_completion(&host->nfi_complete);
	mtk_nfc_set_address(host, column, page, 2, host->rownob);
	mtk_nfc_set_command(host, NAND_CMD_READSTART);
	ret = wait_for_completion_timeout(&host->nfi_complete,
						MTK_DEFAULT_TIMEOUT);
	if (!ret) {
		dev_err(host->dev, "read busy return timeout!\n");
		ret = -ETIMEDOUT;
		goto timeout;
	}
	writew(INTR_AHB_DONE_EN, host->nfi_base + MTKSDG1_NFC_INTR_EN);
	writew(DEC_IRQEN, host->nfiecc_base + MTKSDG1_ECC_DECIRQ_EN);
	dma_addr = dma_map_single(host->dev, buf, readsize, DMA_FROM_DEVICE);
	writel(lower_32_bits(dma_addr), host->nfi_base + MTKSDG1_NFC_STRADDR);

	init_completion(&host->nfi_complete);
	init_completion(&host->ecc_complete);
	host->ecc_sector = sectors;
	host->eccdone_sector = 0;
	reg_val = readl(host->nfi_base + MTKSDG1_NFC_CON);
	reg_val |= CON_BRD;
	writel(reg_val, host->nfi_base + MTKSDG1_NFC_CON);
	ret = wait_for_completion_timeout(&host->nfi_complete,
						MTK_DEFAULT_TIMEOUT);
	if (!ret) {
		dev_err(host->dev, "read ahb done timeout!\n");
		ret = -ETIMEDOUT;
		goto timeout;
	}
	mtk_nfc_wait_bytelen(host, sectors);
	ret = wait_for_completion_timeout(&host->ecc_complete,
						MTK_DEFAULT_TIMEOUT);
	host->ecc_sector = 0;
	host->eccdone_sector = 0;
	if (!ret) {
		dev_err(host->dev, "ecc decode done timeout!\n");
		ret = -ETIMEDOUT;
		goto timeout;
	}
	mtk_nfc_read_fdm(chip, sectors);
	ret = mtk_nfc_ecc_check(mtd, chip, buf, sectors);
	dma_unmap_single(host->dev, dma_addr, readsize, DMA_FROM_DEVICE);
	mtk_nfc_wait_decidle(host);

timeout:
	writew(0, host->nfiecc_base + MTKSDG1_ECC_DECCON);
	writel(0, host->nfi_base + MTKSDG1_NFC_CON);

	return ret;
}

static int mtk_nfc_read_page_hwecc(struct mtd_info *mtd, struct nand_chip *chip,
				uint8_t *buf, int oob_required, int page)
{
	return mtk_nfc_read_subpage(mtd, chip, 0, mtd->writesize, buf, page);
}

static int mtk_nfc_read_page_raw(struct mtd_info *mtd, struct nand_chip *chip,
				uint8_t *buf, int oob_required, int page)
{
	struct mtk_nfc_host *host = chip->priv;
	u32 sectors = mtd->writesize >> host->sectorsize_shift;
	u8 *oob = chip->oob_poi;
	u32 i, j, spare_per_sector, reg_val, sectorsize, sectorsparesize;
	int ret = 0;

	spare_per_sector =
		mtd->oobsize / (mtd->writesize >> host->sectorsize_shift);
	sectorsize = 1 << host->sectorsize_shift;
	sectorsparesize = sectorsize + spare_per_sector;

	reg_val = readw(host->nfi_base + MTKSDG1_NFC_CNFG);
	reg_val |= CNFG_BYTE_RW;
	writew(reg_val, host->nfi_base + MTKSDG1_NFC_CNFG);

	writew(INTR_BUSY_RT_EN, host->nfi_base + MTKSDG1_NFC_INTR_EN);
	init_completion(&host->nfi_complete);
	mtk_nfc_set_address(host, 0, page, 2, host->rownob);
	mtk_nfc_set_command(host, NAND_CMD_READSTART);
	ret = wait_for_completion_timeout(&host->nfi_complete,
						MTK_DEFAULT_TIMEOUT);
	if (!ret) {
		dev_err(host->dev, "read busy return timeout!\n");
		ret = -ETIMEDOUT;
		goto timeout;
	} else {
		ret = 0;
	}
	writel((sectors << CON_SEC_SHIFT) | CON_BRD,
		host->nfi_base + MTKSDG1_NFC_CON);
	for (i = 0; i < sectors; i++) {
		for (j = 0; j < sectorsparesize; j++) {
			mtk_nfc_wait_pio_dirdy(host);
			if (j < sectorsize)
				*buf++ =
				readl(host->nfi_base + MTKSDG1_NFC_DATAR);
			else if (j < (sectorsize + host->fdm_size))
				*oob++ =
				readl(host->nfi_base + MTKSDG1_NFC_DATAR);
			else
				readl(host->nfi_base + MTKSDG1_NFC_DATAR);
		}
	}
	mtk_nfc_wait_bytelen(host, sectors);

timeout:
	writel(0, host->nfi_base + MTKSDG1_NFC_CON);

	return ret;
}

static int mtk_nfc_read_oob(struct mtd_info *mtd, struct nand_chip *chip,
				int page)
{
	struct mtk_nfc_host *host = chip->priv;
	u8 *buf = chip->buffers->databuf;

	memset(buf, 0xff, mtd->writesize);
	chip->cmdfunc(mtd, NAND_CMD_READ0, 0, page);
	mtk_nfc_read_page_hwecc(mtd, chip, buf, 1, page);
	if (host->exchange_oob)
		mtk_nfc_exchange_oob(mtd, chip, buf);
	return 0;
}

static int mtk_nfc_read_oob_raw(struct mtd_info *mtd, struct nand_chip *chip,
				int page)
{
	u8 *buf = chip->buffers->databuf;

	memset(buf, 0xff, mtd->writesize);
	chip->cmdfunc(mtd, NAND_CMD_READ0, 0, page);
	return mtk_nfc_read_page_raw(mtd, chip, buf, 1, page);
}

static void mtk_nfc_hw_init(struct mtk_nfc_host *host)
{
	writel(0x10804222, host->nfi_base + MTKSDG1_NFC_ACCCON);
	writew(0xf1, host->nfi_base + MTKSDG1_NFC_CNRNB);
	writel(8, host->nfi_base + MTKSDG1_NFC_EMPTY_THRESH);

	mtk_nfc_hw_reset(host);

	/* clear interrupt */
	readl(host->nfi_base + MTKSDG1_NFC_INTR_STA);
	writel(0, host->nfi_base + MTKSDG1_NFC_INTR_EN);

	mtk_nfc_wait_encidle(host);
	writew(0, host->nfiecc_base + MTKSDG1_ECC_ENCCON);
	mtk_nfc_wait_decidle(host);
	writel(0, host->nfiecc_base + MTKSDG1_ECC_DECCON);
}

static int mtk_nfc_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct mtk_nfc_host *host;
	struct device *dev = &pdev->dev;
	struct nand_chip *chip;
	struct mtd_info *mtd;
	int ret, irq, max_chip = MTK_NAND_MAX_CHIP;
	struct mtd_part_parser_data ppdata;
	struct device_node *np = dev->of_node;

	host = devm_kzalloc(dev, sizeof(*host), GFP_KERNEL);
	if (!host)
		return -ENOMEM;
	host->dev = dev;
	chip = &host->chip;
	mtd = &host->mtd;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	host->nfi_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(host->nfi_base)) {
		ret = PTR_ERR(host->nfi_base);
		dev_err(dev, "no nfi base\n");
		return ret;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	host->nfiecc_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(host->nfiecc_base)) {
		ret = PTR_ERR(host->nfiecc_base);
		dev_err(dev, "no nfiecc base\n");
		return ret;
	}

	host->clk.nfi_clk = devm_clk_get(dev, "nfi_clk");
	if (IS_ERR(host->clk.nfi_clk)) {
		dev_err(dev, "no nfi clk\n");
		ret = PTR_ERR(host->clk.nfi_clk);
		return ret;
	}

	host->clk.nfiecc_clk = devm_clk_get(dev, "nfiecc_clk");
	if (IS_ERR(host->clk.nfiecc_clk)) {
		dev_err(dev, "no nfiecc clk\n");
		ret = PTR_ERR(host->clk.nfiecc_clk);
		return ret;
	}

	host->clk.pad_clk = devm_clk_get(dev, "pad_clk");
	if (IS_ERR(host->clk.pad_clk)) {
		dev_err(dev, "no pad clk\n");
		ret = PTR_ERR(host->clk.pad_clk);
		return ret;
	}

	ret = mtk_nfc_enable_clk(dev, &host->clk);
	if (ret)
		return ret;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(dev, "no NFI IRQ resource\n");
		ret = -EINVAL;
		goto clk_disable;
	}
	ret = devm_request_irq(dev, irq, mtk_nfi_irq_handle, 0x0, "mtk-nand",
				host);
	if (ret) {
		dev_err(dev, "failed to request NFI IRQ\n");
		goto clk_disable;
	}

	irq = platform_get_irq(pdev, 1);
	if (irq < 0) {
		dev_err(dev, "no ECC IRQ resource\n");
		ret = -EINVAL;
		goto clk_disable;
	}
	ret = devm_request_irq(dev, irq, mtk_ecc_irq_handle, 0x0, "mtk-nandecc",
				host);
	if (ret) {
		dev_err(dev, "failed to request ECC IRQ\n");
		goto clk_disable;
	}

	ret = dma_set_mask(dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(dev, "failed to set dma mask\n");
		goto clk_disable;
	}

	platform_set_drvdata(pdev, host);

	mtd->priv = chip;
	mtd->owner = THIS_MODULE;
	mtd->name = "mtk-nand";
	mtd->dev.parent = dev;

	chip->priv = host;
	chip->options = NAND_USE_BOUNCE_BUFFER;
	chip->options |= NAND_SUBPAGE_READ;
	chip->select_chip = mtk_nfc_select_chip;
	chip->cmdfunc = mtk_nfc_cmdfunc;
	chip->read_byte = mtk_nfc_read_byte;
	chip->ecc.mode = NAND_ECC_HW;
	chip->ecc.write_page = mtk_nfc_write_page_hwecc;
	chip->ecc.write_subpage = mtk_nfc_write_subpage_hwecc;
	chip->ecc.write_page_raw = mtk_nfc_write_page_raw;
	chip->ecc.write_oob = mtk_nfc_write_oob;
	chip->ecc.write_oob_raw = mtk_nfc_write_oob_raw;
	chip->ecc.read_page = mtk_nfc_read_page_hwecc;
	chip->ecc.read_page_raw = mtk_nfc_read_page_raw;
	chip->ecc.read_subpage = mtk_nfc_read_subpage;
	chip->ecc.read_oob = mtk_nfc_read_oob;
	chip->ecc.read_oob_raw = mtk_nfc_read_oob_raw;

	if (of_get_nand_on_flash_bbt(np))
		chip->bbt_options |= NAND_BBT_USE_FLASH;

	mtk_nfc_hw_init(host);

	ret = nand_scan_ident(mtd, max_chip, NULL);
	if (ret) {
		ret = -ENODEV;
		goto clk_disable;
	}

	mtk_nfc_hw_config(mtd);

	host->exchange_oob = 1;
	ret = nand_scan_tail(mtd);
	if (ret) {
		ret = -ENODEV;
		goto clk_disable;
	}
	host->exchange_oob = 0;

	ppdata.of_node = np;
	ret = mtd_device_parse_register(mtd, NULL, &ppdata, NULL, 0);
	if (ret) {
		dev_err(dev, "mtd parse partition error\n");
		goto nand_free;
	}

	return 0;

nand_free:
	nand_release(mtd);
clk_disable:
	mtk_nfc_disable_clk(&host->clk);
	return ret;
}

static int mtk_nfc_remove(struct platform_device *pdev)
{
	struct mtk_nfc_host *host = platform_get_drvdata(pdev);
	struct mtd_info *mtd = &host->mtd;

	nand_release(mtd);

	mtk_nfc_disable_clk(&host->clk);

	kfree(host);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int mtk_nfc_suspend(struct device *dev)
{
	struct mtk_nfc_host *host = dev_get_drvdata(dev);
	struct mtk_nfc_saved_reg *reg = &host->saved_reg;

	reg->nfi_pagefmt = readw(host->nfi_base + MTKSDG1_NFC_PAGEFMT);
	reg->nfi_acccon = readl(host->nfi_base + MTKSDG1_NFC_ACCCON);
	reg->nfi_csel = readw(host->nfi_base + MTKSDG1_NFC_CSEL);
	reg->nfi_acccon = readl(host->nfi_base + MTKSDG1_NFC_ACCCON);
	reg->nfi_csel = readw(host->nfi_base + MTKSDG1_NFC_CSEL);
	reg->ecc_enccnfg = readl(host->nfiecc_base + MTKSDG1_ECC_ENCCNFG);
	reg->ecc_deccnfg = readl(host->nfiecc_base + MTKSDG1_ECC_DECCNFG);
	reg->nfi_cnrnb = readw(host->nfi_base + MTKSDG1_NFC_CNRNB);
	reg->nfi_emp_thresh = readl(host->nfi_base + MTKSDG1_NFC_EMPTY_THRESH);

	mtk_nfc_disable_clk(&host->clk);
	return 0;
}

static int mtk_nfc_resume(struct device *dev)
{
	struct mtk_nfc_host *host = dev_get_drvdata(dev);
	struct mtd_info *mtd = &host->mtd;
	struct nand_chip *chip = &host->chip;
	struct mtk_nfc_saved_reg *reg = &host->saved_reg;
	u32 i;
	int ret;

	/* delay 200us for POR flow */
	udelay(200);

	for (i = 0; i < chip->numchips; i++) {
		chip->select_chip(mtd, i);
		chip->cmdfunc(mtd, NAND_CMD_RESET, -1, -1);
	}

	ret = mtk_nfc_enable_clk(dev, &host->clk);
	if (ret)
		return ret;

	writew(reg->nfi_pagefmt, host->nfi_base + MTKSDG1_NFC_PAGEFMT);
	writel(reg->nfi_acccon, host->nfi_base + MTKSDG1_NFC_ACCCON);
	writew(reg->nfi_csel, host->nfi_base + MTKSDG1_NFC_CSEL);
	writel(reg->ecc_enccnfg, host->nfiecc_base + MTKSDG1_ECC_ENCCNFG);
	writel(reg->ecc_deccnfg, host->nfiecc_base + MTKSDG1_ECC_DECCNFG);
	writew(reg->nfi_cnrnb, host->nfi_base + MTKSDG1_NFC_CNRNB);
	writel(reg->nfi_emp_thresh, host->nfi_base + MTKSDG1_NFC_EMPTY_THRESH);

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
		.name  = "mtk-nand",
		.of_match_table = mtk_nfc_id_table,
#ifdef CONFIG_PM_SLEEP
		.pm = &mtk_nfc_pm_ops,
#endif
	},
};

module_platform_driver(mtk_nfc_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Xiaolei Li <xiaolei.li@mediatek.com>");
MODULE_DESCRIPTION("MTK Nand Flash Controller Driver");

