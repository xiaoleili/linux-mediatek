/*
 * MTK smart device NAND Flash controller driver.
 * Copyright (C) 2015-2016 MediaTek Inc.
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

#include "mtksdg1_nfi_regs.h"
#include "mtksdg1_ecc.h"

#define MTK_IRQ_NFI		"mtksdg1-nand-nfi"
#define MTK_NAME		"mtksdg1-nand"

#define MTK_TIMEOUT		(500000)
#define MTK_RESET_TIMEOUT	(1000000)

#define MTK_NAND_MAX_NSELS	(2)

/* raw accesses do not use ECC (ecc = !raw) */
#define MTK_ECC_OFF		(1)
#define MTK_ECC_ON		(0)
#define MTK_OOB_ON		(1)
#define MTK_OOB_OFF		(0)

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
	int nsels;
	u8 sels[0];
};

static inline struct mtk_nfc_nand_chip *to_mtk_nand(struct nand_chip *nand)
{
	return container_of(nand, struct mtk_nfc_nand_chip, nand);
}

struct mtk_nfc {
	struct nand_hw_control controller;
	struct sdg1_ecc_if *ecc;
	struct mtk_nfc_clk clk;

	struct device *dev;
	void __iomem *regs;

	struct completion done;
	struct list_head chips;

	u32 fdm_reg[MTKSDG1_NFI_FDM_REG_SIZE / sizeof(u32)];
	bool switch_oob;
	u32 row_nob;
	u8 *buffer;

#ifdef CONFIG_PM_SLEEP
	struct mtk_nfc_saved_reg saved_reg;
#endif
};

static inline struct mtk_nfc *to_mtk_nfc(struct nand_hw_control *ctrl)
{
	return container_of(ctrl, struct mtk_nfc, controller);
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
		(!(val & MASTER_STA_MASK)), 50, MTK_RESET_TIMEOUT);
	if (ret)
		dev_warn(dev, "master active in reset [0x%x] = 0x%x\n",
			MTKSDG1_NFI_MASTER_STA, val);
}

static int mtk_nfc_set_command(struct mtk_nfc *nfc, u8 command)
{
	struct device *dev = nfc->dev;
	u32 val;
	int ret;

	sdg1_writel(nfc, command, MTKSDG1_NFI_CMD);

	ret = readl_poll_timeout_atomic(nfc->regs + MTKSDG1_NFI_STA, val,
		(!(val & STA_CMD)), 10,  MTK_TIMEOUT);
	if (ret) {
		dev_warn(dev, "nfi core timed out entering command mode\n");
		return -EIO;
	}

	return 0;
}

static int mtk_nfc_set_address(struct mtk_nfc *nfc, u32 column, u32 row,
		u8 colnob, u8 row_nob)
{
	struct device *dev = nfc->dev;
	u32 addr_nob, val;
	int ret;

	addr_nob = colnob | (row_nob << ADDR_ROW_NOB_SHIFT);
	sdg1_writel(nfc, column, MTKSDG1_NFI_COLADDR);
	sdg1_writel(nfc, row, MTKSDG1_NFI_ROWADDR);
	sdg1_writel(nfc, addr_nob, MTKSDG1_NFI_ADDRNOB);

	/* wait for the NFI core to enter address mode */
	ret = readl_poll_timeout_atomic(nfc->regs + MTKSDG1_NFI_STA, val,
		(!(val & STA_ADDR)), 10, MTK_TIMEOUT);
	if (ret) {
		dev_warn(dev, "nfi core timed out entering address mode\n");
		return -EIO;
	}

	return 0;
}

static int mtk_nfc_transfer_done(struct mtk_nfc *nfc, u32 sectors)
{
	int val;
	int ret;

	ret = readl_poll_timeout_atomic(nfc->regs + MTKSDG1_NFI_ADDRCNTR, val,
		((val & CNTR_MASK) >= sectors), 10, MTK_TIMEOUT);
	if (ret)
		return -EIO;

	return  0;
}

static int mtk_nfc_subpage_done(struct mtk_nfc *nfc, u32 sectors)
{
	u32 val;
	int ret;

	ret = readl_poll_timeout_atomic(nfc->regs + MTKSDG1_NFI_BYTELEN, val,
		((val & CNTR_MASK) >= sectors), 10, MTK_TIMEOUT);
	if (ret)
		return -EIO;

	return 0;
}

static inline int mtk_nfc_data_ready(struct mtk_nfc *nfc)
{
	u8 val;
	int ret;

	ret = readb_poll_timeout_atomic(nfc->regs + MTKSDG1_NFI_PIO_DIRDY, val,
		(val & PIO_DI_RDY), 10, MTK_TIMEOUT);
	if (ret)
		return -EIO;

	return 0;
}

static int mtk_nfc_hw_runtime_config(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct mtk_nfc *nfc = nand_get_controller_data(chip);
	struct device *dev = nfc->dev;
	u32 fmt;

	nfc->row_nob = 1;
	if (chip->chipsize > MB(32))
		nfc->row_nob = chip->chipsize > MB(128) ? 3 : 2;

	switch (mtd->writesize) {
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
		dev_err(dev, "invalid page size: %d\n", mtd->writesize);
		return -EINVAL;
	}

	fmt |= PAGEFMT_SPARE_16 << PAGEFMT_SPARE_SHIFT;
	fmt |= MTKSDG1_NFI_FDM_REG_SIZE << PAGEFMT_FDM_SHIFT;
	fmt |= MTKSDG1_NFI_FDM_REG_SIZE << PAGEFMT_FDM_ECC_SHIFT;
	sdg1_writew(nfc, fmt, MTKSDG1_NFI_PAGEFMT);

	nfc->ecc->config(nfc->ecc, mtd, SECTOR_SIZE + MTKSDG1_NFI_FDM_REG_SIZE);

	return 0;
}

static void mtk_nfc_device_reset(struct mtk_nfc *nfc)
{
	unsigned long timeout = msecs_to_jiffies(500);
	struct device *dev = nfc->dev;
	u16 csel;
	int rc;

	mtk_nfc_hw_reset(nfc);

	/* enable reset done interrupt */
	sdg1_writew(nfc, INTR_RST_DONE_EN, MTKSDG1_NFI_INTR_EN);

	/* configure FSM for reset operation */
	sdg1_writew(nfc, CNFG_OP_RESET, MTKSDG1_NFI_CNFG);

	init_completion(&nfc->done);

	mtk_nfc_set_command(nfc, NAND_CMD_RESET);
	rc = wait_for_completion_timeout(&nfc->done, timeout);
	if (!rc) {
		csel = sdg1_readw(nfc, MTKSDG1_NFI_CSEL);
		dev_err(dev, "device csel(%d) reset timeout\n", csel);
	}
}

static void mtk_nfc_select_chip(struct mtd_info *mtd, int chip)
{
	struct nand_chip *nand = mtd_to_nand(mtd);
	struct mtk_nfc *nfc = nand_get_controller_data(nand);
	struct mtk_nfc_nand_chip *mtk_nand = to_mtk_nand(nand);

	if (chip < 0 )
		return;

	if (chip >= mtk_nand->nsels)
		return;

	sdg1_writel(nfc, mtk_nand->sels[chip], MTKSDG1_NFI_CSEL);
}

static inline bool mtk_nfc_cmd_supported(unsigned command)
{
	switch (command) {
	case NAND_CMD_RESET:
	case NAND_CMD_READID:
	case NAND_CMD_STATUS:
	case NAND_CMD_READOOB:
	case NAND_CMD_ERASE1:
	case NAND_CMD_ERASE2:
	case NAND_CMD_SEQIN:
	case NAND_CMD_PAGEPROG:
	case NAND_CMD_CACHEDPROG:
	case NAND_CMD_READ0:
		return true;
	default:
		return false;
	}
}

static void mtk_nfc_cmdfunc(struct mtd_info *mtd, unsigned command, int column,
		int page_addr)
{
	struct mtk_nfc *nfc = nand_get_controller_data(mtd_to_nand(mtd));
	unsigned long const cmd_timeout = msecs_to_jiffies(500);
	struct completion *p = &nfc->done;
	u32 val;
	int rc;

	if (mtk_nfc_cmd_supported(command))
		mtk_nfc_hw_reset(nfc);

	switch (command) {
	case NAND_CMD_RESET:
		mtk_nfc_device_reset(nfc);
		break;
	case NAND_CMD_READID:
		val = CNFG_READ_EN | CNFG_BYTE_RW | CNFG_OP_SRD;
		sdg1_writew(nfc, val, MTKSDG1_NFI_CNFG);
		mtk_nfc_set_command(nfc, NAND_CMD_READID);
		mtk_nfc_set_address(nfc, column, 0, 1, 0);
		sdg1_writel(nfc, CON_SRD, MTKSDG1_NFI_CON);
		break;
	case NAND_CMD_STATUS:
		val = CNFG_READ_EN | CNFG_BYTE_RW | CNFG_OP_SRD;
		sdg1_writew(nfc, val, MTKSDG1_NFI_CNFG);
		mtk_nfc_set_command(nfc, NAND_CMD_STATUS);
		sdg1_writel(nfc, CON_SRD, MTKSDG1_NFI_CON);
		break;
	case NAND_CMD_READOOB:
		val = CNFG_READ_EN | CNFG_BYTE_RW | CNFG_OP_READ;
		sdg1_writew(nfc, val, MTKSDG1_NFI_CNFG);
		mtk_nfc_set_command(nfc, NAND_CMD_READ0);
		column += mtd->writesize;
		mtk_nfc_set_address(nfc, column, page_addr, 2, nfc->row_nob);
		val = CON_BRD | (1 << CON_SEC_SHIFT);
		sdg1_writel(nfc, val, MTKSDG1_NFI_CON);
		break;
	case NAND_CMD_ERASE1:
		sdg1_writew(nfc, INTR_ERS_DONE_EN, MTKSDG1_NFI_INTR_EN);
		sdg1_writew(nfc, CNFG_OP_ERASE, MTKSDG1_NFI_CNFG);
		mtk_nfc_set_command(nfc, NAND_CMD_ERASE1);
		mtk_nfc_set_address(nfc, 0, page_addr, 0, nfc->row_nob);
		break;
	case NAND_CMD_ERASE2:
		init_completion(p);
		mtk_nfc_set_command(nfc, NAND_CMD_ERASE2);
		rc = wait_for_completion_timeout(p, cmd_timeout);
		if (!rc)
			dev_err(nfc->dev, "erase command timeout\n");
		break;
	case NAND_CMD_SEQIN:
		sdg1_writew(nfc, CNFG_OP_PRGM, MTKSDG1_NFI_CNFG);
		mtk_nfc_set_command(nfc, NAND_CMD_SEQIN);
		mtk_nfc_set_address(nfc, column, page_addr, 2, nfc->row_nob);
		break;
	case NAND_CMD_PAGEPROG:
	case NAND_CMD_CACHEDPROG:
		sdg1_writew(nfc, INTR_BUSY_RT_EN, MTKSDG1_NFI_INTR_EN);
		init_completion(p);
		mtk_nfc_set_command(nfc, command);
		rc = wait_for_completion_timeout(p, cmd_timeout);
		if (!rc)
			dev_err(nfc->dev, "pageprogr command timeout\n");
		break;
	case NAND_CMD_READ0:
		val = CNFG_OP_READ | CNFG_READ_EN;
		sdg1_writew(nfc, val, MTKSDG1_NFI_CNFG);
		mtk_nfc_set_command(nfc, NAND_CMD_READ0);
		break;
	default:
		dev_warn(nfc->dev, "command 0x%x not supported\n", command);
		break;
	}
}

static uint8_t mtk_nfc_read_byte(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct mtk_nfc *nfc = nand_get_controller_data(chip);
	int rc;

	/* todo return code */

	rc = mtk_nfc_data_ready(nfc);
	if (rc < 0) {
		dev_err(nfc->dev, "data not ready\n");
		return NAND_STATUS_FAIL;
	}

	return sdg1_readb(nfc, MTKSDG1_NFI_DATAR);
}

static void mtk_nfc_read_fdm(struct nand_chip *chip, u32 sectors)
{
	struct mtk_nfc *nfc = nand_get_controller_data(chip);
	int i, j, reg;
	u8 *dst, *src;

	/* todo endianess */

	for (i = 0; i < sectors; i++) {
		/* read FDM register into host memory */
		for (j = 0; j < ARRAY_SIZE(nfc->fdm_reg); j++) {
			reg = MTKSDG1_NFI_FDM0L + i * MTKSDG1_NFI_FDM_REG_SIZE;
			reg += j * sizeof(nfc->fdm_reg[0]);
			nfc->fdm_reg[j] = sdg1_readl(nfc, reg);
		}

		/* copy FDM register from host to OOB */
		src = (u8 *)nfc->fdm_reg;
		dst = chip->oob_poi + i * MTKSDG1_NFI_FDM_REG_SIZE;
		memcpy(dst, src, MTKSDG1_NFI_FDM_REG_SIZE);
	}
}

static void mtk_nfc_write_fdm(struct nand_chip *chip, u32 sectors)
{
	struct mtk_nfc *nfc = nand_get_controller_data(chip);
	u8 *src, *dst;
	int i, j, reg;

	for (i = 0; i < sectors ; i++) {
		/* read FDM from OOB into private area */
		src = chip->oob_poi + i * MTKSDG1_NFI_FDM_REG_SIZE;
		dst = (u8 *)nfc->fdm_reg;
		memcpy(dst, src, MTKSDG1_NFI_FDM_REG_SIZE);

		/* write FDM to registers */
		for (j = 0; j < ARRAY_SIZE(nfc->fdm_reg); j++) {
			reg = MTKSDG1_NFI_FDM0L + i * MTKSDG1_NFI_FDM_REG_SIZE;
			reg += j * sizeof(nfc->fdm_reg[0]);
			sdg1_writel(nfc, nfc->fdm_reg[j], reg);
		}
	}
}

static int mtk_nfc_write_page(struct mtd_info *mtd,
			struct nand_chip *chip, const uint8_t *buf,
			int oob_on, int page, int raw)
{

	struct mtk_nfc *nfc = nand_get_controller_data(chip);
	struct completion *nfi = &nfc->done;
	struct device *dev = nfc->dev;
	const bool use_ecc = !raw;
	void *q = (void *) buf;
	dma_addr_t dma_addr;
	size_t dmasize;
	u32 reg;
	int ret;

	dmasize = mtd->writesize + (raw ? mtd->oobsize : 0);

	dma_addr = dma_map_single(dev, q, dmasize, DMA_TO_DEVICE);
	if (dma_mapping_error(nfc->dev, dma_addr)) {
		dev_err(nfc->dev, "dma mapping error\n");
		return -EINVAL;
	}

	reg = sdg1_readw(nfc, MTKSDG1_NFI_CNFG);
	reg |= CNFG_AHB | CNFG_DMA_BURST_EN;
	if (use_ecc) {
		/**
		 * OOB will be generated
		 *  - FDM: from register
		 *  - ECC: from HW
		 */
		reg |= CNFG_AUTO_FMT_EN | CNFG_HW_ECC_EN;
		sdg1_writew(nfc, reg, MTKSDG1_NFI_CNFG);
		nfc->ecc->control(nfc->ecc, enable_encoder, 0);

		/* write OOB into the FDM registers (OOB area in MTK NAND) */
		if (oob_on)
			mtk_nfc_write_fdm(chip, chip->ecc.steps);
	} else {
		/* OOB is part of the DMA transfer */
		sdg1_writew(nfc, reg, MTKSDG1_NFI_CNFG);
	}

	sdg1_writel(nfc, chip->ecc.steps << CON_SEC_SHIFT, MTKSDG1_NFI_CON);
	sdg1_writel(nfc, lower_32_bits(dma_addr), MTKSDG1_NFI_STRADDR);
	sdg1_writew(nfc, INTR_AHB_DONE_EN, MTKSDG1_NFI_INTR_EN);

	init_completion(nfi);

	/* start DMA */
	reg = sdg1_readl(nfc, MTKSDG1_NFI_CON) | CON_BWR;
	sdg1_writel(nfc, reg, MTKSDG1_NFI_CON);

	ret = wait_for_completion_timeout(nfi, msecs_to_jiffies(500));
	if (!ret) {
		dev_err(dev, "program ahb done timeout\n");
		sdg1_writew(nfc, 0, MTKSDG1_NFI_INTR_EN);
		ret = -ETIMEDOUT;
		goto timeout;
	}

	ret = mtk_nfc_transfer_done(nfc, chip->ecc.steps);
	if (ret < 0)
		dev_err(dev, "hwecc write timeout\n");
timeout:
	dma_unmap_single(nfc->dev, dma_addr, dmasize, DMA_TO_DEVICE);

	if (use_ecc)
		nfc->ecc->control(nfc->ecc, disable_encoder, 0);

	sdg1_writel(nfc, 0, MTKSDG1_NFI_CON);

	return ret;
}

static int mtk_nfc_write_page_hwecc(struct mtd_info *mtd,
			struct nand_chip *chip, const uint8_t *buf,
			int oob_on, int page)
{
	return mtk_nfc_write_page(mtd, chip, buf, oob_on, page, MTK_ECC_ON);
}

static int mtk_nfc_write_page_raw(struct mtd_info *mtd, struct nand_chip *chip,
					const uint8_t *buf, int oob_on, int pg)
{
	struct mtk_nfc *nfc = nand_get_controller_data(chip);
	uint8_t *src, *dst;
	size_t len;
	u32 i;

	memset(nfc->buffer, 0xff, mtd->writesize + mtd->oobsize);

	/* MTK internal 4KB page data layout:
	 * ----------------------------------
	 * PAGE = 4KB, SECTOR = 1KB, OOB=128B
	 * page = sector_oob1 + sector_oob2 + sector_oob3 + sector_oob4
	 * sector_oob = data (1KB) + FDM (8B) + ECC parity (21B) + free (3B)
	 *
	 */
	len = SECTOR_SIZE + mtd->oobsize / chip->ecc.steps;

	for (i = 0; i < chip->ecc.steps; i++) {

		if (buf) {
			src = (uint8_t *) buf + i * SECTOR_SIZE;
			dst = nfc->buffer + i * len;
			memcpy(dst, src, SECTOR_SIZE);
		}

		if (oob_on) {
			src = chip->oob_poi + i * MTKSDG1_NFI_FDM_REG_SIZE;
			dst = nfc->buffer + i * len + SECTOR_SIZE;
			memcpy(dst, src, MTKSDG1_NFI_FDM_REG_SIZE);
		}
	}

	return mtk_nfc_write_page(mtd, chip, nfc->buffer, MTK_OOB_OFF, pg,
				MTK_ECC_OFF);
}

static int mtk_nfc_sector_encode(struct nand_chip *chip, u8 *data)
{
	struct mtk_nfc *nfc = nand_get_controller_data(chip);
	struct sdg1_encode_params p = {
		.len = SECTOR_SIZE + MTKSDG1_NFI_FDM_REG_SIZE,
		.strength = chip->ecc.strength,
		.data = data,
	};

	return sdg1_ecc_encode(nfc->ecc, &p);
}

static int mtk_nfc_write_subpage_hwecc(struct mtd_info *mtd,
		struct nand_chip *chip, uint32_t offset, uint32_t data_len,
		const uint8_t *buf, int oob_on, int pg)
{
	struct mtk_nfc *nfc = nand_get_controller_data(chip);
	uint8_t *src, *dst;
	u32 start, end;
	size_t len;
	int i, ret;

	start = BYTES_TO_SECTORS(offset);
	end = BYTES_TO_SECTORS(offset + data_len + SECTOR_SIZE - 1);

	len = SECTOR_SIZE + mtd->oobsize / chip->ecc.steps;

	memset(nfc->buffer, 0xff, mtd->writesize + mtd->oobsize);
	for (i = 0; i < chip->ecc.steps; i++) {

		/* write data */
		src = (uint8_t *) buf + i * SECTOR_SIZE;
		dst = nfc->buffer + i * len;
		memcpy(dst, src, SECTOR_SIZE);

		if (i < start)
			continue;

		if (i >= end)
			continue;

		/* write fdm */
		if (oob_on) {
			src = chip->oob_poi + i * MTKSDG1_NFI_FDM_REG_SIZE;
			dst = nfc->buffer + i * len + SECTOR_SIZE;
			memcpy(dst, src, MTKSDG1_NFI_FDM_REG_SIZE);
		}

		/* point to the start of data */
		src = nfc->buffer + i * len;

		/* program the CRC back to the OOB */
		ret = mtk_nfc_sector_encode(chip, src);
		if (ret < 0)
			return ret;
	}

	/* use the data in the private buffer (now with FDM and CRC) to perform
	 * a raw write
	 */
	src = nfc->buffer;
	return mtk_nfc_write_page(mtd, chip, src, MTK_OOB_OFF, pg, MTK_ECC_OFF);
}

static int mtk_nfc_write_oob(struct mtd_info *mtd, struct nand_chip *chip,
				int page)
{
	u8 *buf = chip->buffers->databuf;
	int ret;

	memset(buf, 0xff, mtd->writesize);
	chip->cmdfunc(mtd, NAND_CMD_SEQIN, 0x00, page);
	ret = mtk_nfc_write_page_hwecc(mtd, chip, buf, MTK_OOB_ON, page);
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
	ret = mtk_nfc_write_page_raw(mtd, chip, NULL, MTK_OOB_ON, page);
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
	int i, bitflips = 0;

	/* if the page is empty, no bitflips and clear data and oob */
	if (sdg1_readl(nfc, MTKSDG1_NFI_STA) & STA_EMP_PAGE) {
		memset(buf, 0xff, SECTORS_TO_BYTES(sectors));

		/* empty page: update OOB with 0xFF */
		for (i = 0; i < sectors; i++) {
			memset(chip->oob_poi + i * MTKSDG1_NFI_FDM_REG_SIZE,
				0xff, MTKSDG1_NFI_FDM_REG_SIZE);
		}
	} else {
		/* update OOB with HW info */
		mtk_nfc_read_fdm(chip, sectors);

		/* return the bitflips */
		bitflips = sdg1_ecc_check(nfc->ecc, mtd, sectors);
	}

	return bitflips;
}

static int mtk_nfc_block_markbad(struct mtd_info *mtd, loff_t ofs)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	u8 *buf = chip->buffers->databuf;
	int rc, i, pg;

	/* block_markbad writes 0x00 at data and OOB */
	memset(buf, 0x00, mtd->writesize + mtd->oobsize);

	/* Write to first/last page(s) if necessary */
	if (chip->bbt_options & NAND_BBT_SCANLASTPAGE)
		ofs += mtd->erasesize - mtd->writesize;

	i = 0;
	do {
		pg = (int)(ofs >> chip->page_shift);

		/**
		 *  write 0x00 to DATA & OOB in flash
		 *  No need to reorganize the page since it is all 0x00
		 */
		chip->cmdfunc(mtd, NAND_CMD_SEQIN, 0x00, pg);
		rc = mtk_nfc_write_page(mtd, chip, buf, MTK_OOB_OFF, pg,
			MTK_ECC_OFF);
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
	unsigned long timeout = msecs_to_jiffies(500);
	u32 reg, column, spare, sectors, start, end;
	struct completion *nfi;
	const bool use_ecc = !raw;
	int bitflips = -EIO;
	dma_addr_t dma_addr;
	size_t len;
	u8 *buf;
	int rc;

	nfi = &nfc->done;

	start = BYTES_TO_SECTORS(data_offs);
	end = BYTES_TO_SECTORS(data_offs + readlen + SECTOR_SIZE - 1);
	sectors = end - start;

	spare = mtd->oobsize / chip->ecc.steps;
	column =  start * (SECTOR_SIZE + spare);

	len = SECTORS_TO_BYTES(sectors) + (raw ? sectors * spare : 0);
	buf = bufpoi + SECTORS_TO_BYTES(start);

	/* map the device memory */
	dma_addr = dma_map_single(nfc->dev, buf, len, DMA_FROM_DEVICE);
	if (dma_mapping_error(nfc->dev, dma_addr)) {
		dev_err(nfc->dev, "dma mapping error\n");
		return -EINVAL;
	}

	/* configure the transfer  */
	reg = sdg1_readw(nfc, MTKSDG1_NFI_CNFG);
	reg |= CNFG_DMA_BURST_EN | CNFG_AHB;
	if (use_ecc) {
		reg |= CNFG_AUTO_FMT_EN | CNFG_HW_ECC_EN;
		sdg1_writew(nfc, reg, MTKSDG1_NFI_CNFG);
		nfc->ecc->control(nfc->ecc, enable_decoder, 0);
	} else
		sdg1_writew(nfc, reg, MTKSDG1_NFI_CNFG);

	sdg1_writel(nfc, sectors << CON_SEC_SHIFT, MTKSDG1_NFI_CON);
	sdg1_writew(nfc, INTR_BUSY_RT_EN, MTKSDG1_NFI_INTR_EN);

	init_completion(nfi);
	mtk_nfc_set_address(nfc, column, page, 2, nfc->row_nob);
	mtk_nfc_set_command(nfc, NAND_CMD_READSTART);

	rc = wait_for_completion_timeout(nfi, timeout);
	if (!rc) {
		dev_err(nfc->dev, "read busy return timeout\n");
		goto error;
	}

	sdg1_writew(nfc, INTR_AHB_DONE_EN, MTKSDG1_NFI_INTR_EN);
	sdg1_writel(nfc, lower_32_bits(dma_addr), MTKSDG1_NFI_STRADDR);

	if (use_ecc)
		nfc->ecc->control(nfc->ecc, start_decoder, sectors);

	/* start DMA */
	init_completion(nfi);
	reg = sdg1_readl(nfc, MTKSDG1_NFI_CON) | CON_BRD;
	sdg1_writel(nfc, reg, MTKSDG1_NFI_CON);

	rc = wait_for_completion_timeout(nfi, timeout);
	if (!rc)
		dev_warn(nfc->dev, "read ahb/dma done timeout\n");

	/* DMA interrupt didn't trigger, check page done just in case */
	rc = mtk_nfc_subpage_done(nfc, sectors);
	if (rc < 0) {
		dev_err(nfc->dev, "subpage done timeout\n");
		goto error;
	}

	if (use_ecc) {
		rc = sdg1_ecc_decode(nfc->ecc);
		if (rc < 0) {
			bitflips = -ETIMEDOUT;
			goto error;
		}
		bitflips = mtk_nfc_update_oob(mtd, chip, buf, sectors);
	} else
		bitflips = 0;

error:
	dma_unmap_single(nfc->dev, dma_addr, len, DMA_FROM_DEVICE);

	if (use_ecc)
		nfc->ecc->control(nfc->ecc, disable_decoder, 0);

	sdg1_writel(nfc, 0, MTKSDG1_NFI_CON);

	return bitflips;
}

static int mtk_nfc_read_subpage_hwecc(struct mtd_info *mtd,
				struct nand_chip *chip, uint32_t data_offs,
				uint32_t readlen, uint8_t *bufpoi, int page)
{
	return mtk_nfc_read_subpage(mtd, chip, data_offs, readlen,
					bufpoi, page, MTK_ECC_ON);
}

static int mtk_nfc_read_page_hwecc(struct mtd_info *mtd, struct nand_chip *chip,
				uint8_t *buf, int oob_on, int page)
{
	return mtk_nfc_read_subpage_hwecc(mtd, chip, 0, mtd->writesize,
						buf, page);
}

static int mtk_nfc_read_page_raw(struct mtd_info *mtd, struct nand_chip *chip,
				uint8_t *buf, int oob_on, int page)
{
	struct mtk_nfc *nfc = nand_get_controller_data(chip);
	uint8_t *src, *dst;
	int i, ret;
	size_t len;

	dst = nfc->buffer;
	memset(dst, 0xff, mtd->writesize + mtd->oobsize);
	ret = mtk_nfc_read_subpage(mtd, chip, 0, mtd->writesize, dst, page, MTK_ECC_OFF);
	if (ret < 0)
		return ret;

	len = SECTOR_SIZE + mtd->oobsize / chip->ecc.steps;

	/* copy to the output buffer */
	for (i = 0; i < chip->ecc.steps; i++) {

		/* copy sector data */
		if (buf) {
			src = nfc->buffer + i * len;
			dst = buf + i * SECTOR_SIZE;
			memcpy(dst, src, SECTOR_SIZE);
		}

		/* copy FDM data to OOB */
		if (oob_on) {
			src = nfc->buffer + i * len + SECTOR_SIZE;
			dst = chip->oob_poi + i * MTKSDG1_NFI_FDM_REG_SIZE;
			memcpy(dst, src, MTKSDG1_NFI_FDM_REG_SIZE);
		}
	}

	return ret;
}

static void mtk_nfc_switch_oob(struct mtd_info *mtd, struct nand_chip *chip,
					uint8_t *buf)
{
	struct mtk_nfc *nfc = nand_get_controller_data(chip);
	size_t spare;
	u32 sectors;
	u8 *bufpoi;
	int len;

	spare = mtd->oobsize / chip->ecc.steps;
	sectors = mtd->writesize / (SECTOR_SIZE + spare);

	/**
	 * MTK: DATA+oob1, DATA+oob2, DATA+oob3 ...
	 * LNX: DATA+OOB
	 */
	/* point to the last oob_i from the NAND device*/
	bufpoi = buf + mtd->writesize - (sectors * spare);
	len = sizeof(nfc->fdm_reg);

	/* copy NAND oob to private area */
	memcpy(nfc->fdm_reg, bufpoi, len);

	/* copy oob_poi to NAND */
	memcpy(bufpoi, chip->oob_poi, len);

	/* copy NAND oob to oob_poi */
	memcpy(chip->oob_poi, nfc->fdm_reg, sizeof(nfc->fdm_reg));
	memset(nfc->fdm_reg, 0x00, len);
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

	return mtk_nfc_read_page_raw(mtd, chip, NULL, MTK_OOB_ON, page);
}

static inline void mtk_nfc_hw_init(struct mtk_nfc *nfc)
{
	sdg1_writel(nfc, 0x10804211, MTKSDG1_NFI_ACCCON);
	sdg1_writew(nfc, 0xf1, MTKSDG1_NFI_CNRNB);
	mtk_nfc_hw_reset(nfc);

	/* clear interrupt */
	sdg1_readl(nfc, MTKSDG1_NFI_INTR_STA);
	sdg1_writel(nfc, 0, MTKSDG1_NFI_INTR_EN);

	nfc->ecc->init(nfc->ecc);
}

static irqreturn_t mtk_nfc_irq(int irq, void *devid)
{
	struct mtk_nfc *nfc = devid;
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

static int mtk_nfc_nand_chip_init(struct device *dev, struct mtk_nfc *nfc,
				struct device_node *np)
{
	struct mtk_nfc_nand_chip *chip;
	struct nand_chip *nand;
	struct mtd_info *mtd;
	int nsels, len;
	int ret;
	int i;
	u32 tmp;

	if (!of_get_property(np, "reg", &nsels))
		return -EINVAL;

	nsels /= sizeof(u32);
	if (!nsels || nsels > MTK_NAND_MAX_NSELS) {
		dev_err(dev, "invalid reg property size %d \n", nsels);
		return -EINVAL;
	}

	chip = devm_kzalloc(dev,
			    sizeof(*chip) + nsels * sizeof(u8), GFP_KERNEL);
	if (!chip) {
		dev_err(dev, "could not allocate chip\n");
		return -ENOMEM;
	}

	chip->nsels = nsels;

	for (i = 0; i < nsels; i++) {
		ret = of_property_read_u32_index(np, "reg", i, &tmp);
		if (ret) {
			dev_err(dev, "could not retrieve reg property: %d\n",
				ret);
			return ret;
		}
		chip->sels[i] = tmp;
	}

	nand = &chip->nand;
	nand->controller = &nfc->controller;

	nand_set_flash_node(nand, np);
	nand_set_controller_data(nand, nfc);

	nand->options |= NAND_USE_BOUNCE_BUFFER | NAND_SUBPAGE_READ;
	nand->block_markbad = mtk_nfc_block_markbad;
	nand->select_chip = mtk_nfc_select_chip;
	nand->read_byte = mtk_nfc_read_byte;
	nand->cmdfunc = mtk_nfc_cmdfunc;
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

	mtk_nfc_hw_init(nfc);

	ret = nand_scan_ident(mtd, nsels, NULL);
	if (ret)
		return -ENODEV;

	ret = mtk_nfc_hw_runtime_config(mtd);
	if (ret < 0)
		return -ENODEV;

	len = mtd->writesize + mtd->oobsize;
	nfc->buffer = devm_kzalloc(dev, len, GFP_KERNEL);
	if (!nfc->buffer)
		return  -ENOMEM;

	/* required to create bbt table if not present */
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

	ret = devm_request_irq(dev, irq, mtk_nfc_irq, 0x0, MTK_IRQ_NFI, nfc);
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

