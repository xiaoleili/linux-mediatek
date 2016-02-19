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

#include "mtksdg1_nand_nfi.h"	/* NAND controller NFI registers	*/
#include "mtksdg1_nand_ecc.h"	/* ECC engine registers			*/

#define MTK_NAME		"mtk-nand"
#define KB(x)			((x) * 1024UL)
#define MB(x)			(KB(x) * 1024UL)

#define SECTOR_SHIFT		(10)
#define SECTOR_SIZE		(1UL << SECTOR_SHIFT)
#define BYTES_TO_SECTORS(x)	((x) >> SECTOR_SHIFT)
#define SECTORS_TO_BYTES(x)	((x) << SECTOR_SHIFT)

#define MTK_TIMEOUT		(500)
#define MTK_RESET_TIMEOUT	(1 * HZ)

#define MTK_ECC_PARITY_BITS	(14)
#define MTK_NAND_MAX_CHIP	(2)

struct mtk_nfc_clk {
	struct clk *nfiecc_clk;
	struct clk *nfi_clk;
	struct clk *pad_clk;
};

struct mtk_nfc_saved_reg {
	u32 ecc_enccnfg;
	u32 ecc_deccnfg;
	u16 nfi_pagefmt;
	u32 nfi_acccon;
	u16 nfi_csel;
};

struct mtk_nfc_host {
	struct mtk_nfc_clk clk;
	struct nand_chip chip;
	struct mtd_info mtd;
	struct device *dev;
	struct completion complete;
	void __iomem *nfiecc_base;
	void __iomem *nfi_base;
	u32 fdm_reg[MTKSDG1_NFI_FDM_REG_SIZE / sizeof(u32)];
	u32 rownob;
#ifdef CONFIG_PM_SLEEP
	struct mtk_nfc_saved_reg saved_reg;
#endif
};

static struct nand_ecclayout nand_4k_128 = {
	.oobfree = { {0, 32} },
	.oobavail = 32,
};

/* NFI register access */
static inline void mtk_nfi_writel(struct mtk_nfc_host *host, u32 val, u32 reg)
{
	writel(val, host->nfi_base + reg);
}
static inline void mtk_nfi_writew(struct mtk_nfc_host *host, u16 val, u32 reg)
{
	writew(val, host->nfi_base + reg);
}
static inline u32 mtk_nfi_readl(struct mtk_nfc_host *host, u32 reg)
{
	return readl_relaxed(host->nfi_base + reg);
}
static inline u16 mtk_nfi_readw(struct mtk_nfc_host *host, u32 reg)
{
	return readw_relaxed(host->nfi_base + reg);
}
static inline u8 mtk_nfi_readb(struct mtk_nfc_host *host, u32 reg)
{
	return readb_relaxed(host->nfi_base + reg);
}

/* ECC register access */
static inline void mtk_ecc_writel(struct mtk_nfc_host *host, u32 val, u32 reg)
{
	writel(val, host->nfiecc_base + reg);
}
static inline void mtk_ecc_writew(struct mtk_nfc_host *host, u16 val, u32 reg)
{
	writew(val, host->nfiecc_base + reg);
}
static inline u32 mtk_ecc_readl(struct mtk_nfc_host *host, u32 reg)
{
	return readl_relaxed(host->nfiecc_base + reg);
}
static inline u16 mtk_ecc_readw(struct mtk_nfc_host *host, u32 reg)
{
	return readw_relaxed(host->nfiecc_base + reg);
}

static void mtk_nfc_hw_reset(struct mtk_nfc_host *host)
{
	unsigned long timeout = MTK_RESET_TIMEOUT;
	struct device *dev = host->dev;
	u32 val;

	/* reset the state machine, data fifo and fdm data */
	mtk_nfi_writel(host, CON_FIFO_FLUSH | CON_NFI_RST, MTKSDG1_NFI_CON);
	timeout += jiffies;
	do {
		val = mtk_nfi_readl(host, MTKSDG1_NFI_MASTER_STA);
		val &= MASTER_STA_MASK;
		if (!val)
			return;
		usleep_range(100, 200);

	} while (time_before(jiffies, timeout));

	dev_warn(dev, "nfi master active after in reset [0x%x] = 0x%x\n",
		MTKSDG1_NFI_MASTER_STA, val);
};

static int mtk_nfc_set_command(struct mtk_nfc_host *host, u8 command)
{
	unsigned long timeout = msecs_to_jiffies(MTK_TIMEOUT);
	struct device *dev = host->dev;
	u32 val;

	mtk_nfi_writel(host, command, MTKSDG1_NFI_CMD);

	/* wait for the NFI core to enter command mode */
	timeout += jiffies;
	do {
		val = mtk_nfi_readl(host, MTKSDG1_NFI_STA);
		val &= STA_CMD;
		if (!val)
			return 0;
		cpu_relax();

	} while (time_before(jiffies, timeout));
	dev_warn(dev, "nfi core timed out entering command mode\n");

	return -EIO;
}

static int mtk_nfc_set_address(struct mtk_nfc_host *host, u32 column, u32 row,
		u8 colnob, u8 rownob)
{
	unsigned long timeout = msecs_to_jiffies(MTK_TIMEOUT);
	struct device *dev = host->dev;
	u32 addr_nob, val;

	addr_nob = colnob | (rownob << ADDR_ROW_NOB_SHIFT);
	mtk_nfi_writel(host, column, MTKSDG1_NFI_COLADDR);
	mtk_nfi_writel(host, row, MTKSDG1_NFI_ROWADDR);
	mtk_nfi_writel(host, addr_nob, MTKSDG1_NFI_ADDRNOB);

	/* wait for the NFI core to enter address mode */
	timeout += jiffies;
	do {
		val = mtk_nfi_readl(host, MTKSDG1_NFI_STA);
		val &= STA_ADDR;
		if (!val)
			return 0;
		cpu_relax();

	} while (time_before(jiffies, timeout));

	dev_warn(dev, "nfi core timed out entering address mode\n");

	return -EIO;
}

static inline void mtk_ecc_encoder_idle(struct mtk_nfc_host *host)
{
	unsigned long timeout = msecs_to_jiffies(MTK_TIMEOUT);
	struct device *dev = host->dev;
	u32 val;

	timeout += jiffies;
	do {
		val = mtk_ecc_readl(host, MTKSDG1_ECC_ENCIDLE);
		val &= ENC_IDLE;
		if (val)
			return;
		cpu_relax();

	} while (time_before(jiffies, timeout));

	dev_warn(dev, "hw init ecc encoder not idle\n");
}

static inline void mtk_ecc_decoder_idle(struct mtk_nfc_host *host)
{
	unsigned long timeout = msecs_to_jiffies(MTK_TIMEOUT);
	struct device *dev = host->dev;
	u32 val;

	timeout += jiffies;
	do {
		val = mtk_ecc_readw(host, MTKSDG1_ECC_DECIDLE);
		val &= DEC_IDLE;
		if (val)
			return;
		cpu_relax();

	} while (time_before(jiffies, timeout));

	dev_warn(dev, "hw init ecc decoder not idle\n");
}

static int mtk_nfc_transfer_done(struct mtk_nfc_host *host, u32 sectors)
{
	unsigned long timeout = msecs_to_jiffies(MTK_TIMEOUT);
	u32 cnt;

	/* wait for the sector count */
	timeout += jiffies;
	do {
		cnt = mtk_nfi_readl(host, MTKSDG1_NFI_ADDRCNTR);
		cnt &= CNTR_MASK;
		if (cnt >= sectors)
			return 0;
		cpu_relax();

	} while (time_before(jiffies, timeout));

	return  -EIO;
}

static inline int mtk_nfc_data_ready(struct mtk_nfc_host *host)
{
	unsigned long timeout = msecs_to_jiffies(MTK_TIMEOUT);
	u8 val;

	timeout += jiffies;
	do {
		val = mtk_nfi_readw(host, MTKSDG1_NFI_PIO_DIRDY);
		val &= PIO_DI_RDY;
		if (val)
			return 0;
		cpu_relax();

	} while (time_before(jiffies, timeout));

	/* data MUST not be accessed */
	return -EIO;
}

static inline int mtk_nfc_read_pio(struct mtk_nfc_host *host, uint8_t *buf)
{
	int rc;

	rc = mtk_nfc_data_ready(host);
	if (rc < 0) {
		dev_err(host->dev, "cant access data, aborting\n");
		return -EIO;
	}
	if (buf)
		*buf = mtk_nfi_readl(host, MTKSDG1_NFI_DATAR);
	else
		mtk_nfi_readl(host, MTKSDG1_NFI_DATAR);

	return 0;
}

static inline int mtk_nfc_write_pio(struct mtk_nfc_host *host,
				const uint8_t *buf)
{
	int rc;

	rc = mtk_nfc_data_ready(host);
	if (rc < 0) {
		dev_err(host->dev, "cant access data, aborting\n");
		return -EIO;
	}
	if (buf)
		mtk_nfi_writel(host, *buf,  MTKSDG1_NFI_DATAW);
	else
		mtk_nfi_writel(host, 0xff,  MTKSDG1_NFI_DATAW);

	return 0;

}

static void mtk_nfc_hw_config(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	struct mtk_nfc_host *host = chip->priv;
	struct device *dev = host->dev;
	u32 spare, spare_bit, pagesize_bit;
	u32 dec_size, enc_size;
	u32 ecc_bit, ecc_level;
	u32 reg;

	host->rownob = 1;
	if (chip->chipsize > MB(32))
		host->rownob = chip->chipsize > MB(128) ? 3 : 2;

	spare = mtd->oobsize / BYTES_TO_SECTORS(mtd->writesize);
	spare_bit = PAGEFMT_SPARE_16;

	switch (spare) {
	case 16:
		ecc_bit = ECC_CNFG_4BIT;
		ecc_level = 4;
		break;
	case 32:
		ecc_bit = ECC_CNFG_12BIT;
		ecc_level = 12;
		break;
	default:
		dev_err(dev, "invalid spare size per sector: %d\n", spare);
		ecc_bit = ECC_CNFG_4BIT;
		ecc_level = 4;
		break;
	}

	chip->ecc.size = SECTOR_SIZE;
	chip->ecc.strength = ecc_level;

	switch (mtd->writesize) {
	case KB(2):
		pagesize_bit = PAGEFMT_512_2K;
		break;
	case KB(4):
		pagesize_bit = PAGEFMT_2K_4K;
		chip->ecc.layout = &nand_4k_128;
		break;
	case KB(8):
		pagesize_bit = PAGEFMT_4K_8K;
		break;
	default:
		dev_err(dev, "invalid page size: %d\n", mtd->writesize);
		pagesize_bit = PAGEFMT_2K_4K;
		break;
	}

	/* configure PAGE FMT */
	reg = pagesize_bit;
	reg |= spare_bit << PAGEFMT_SPARE_SHIFT;
	reg |= MTKSDG1_NFI_FDM_REG_SIZE << PAGEFMT_FDM_SHIFT;
	reg |= MTKSDG1_NFI_FDM_REG_SIZE << PAGEFMT_FDM_ECC_SHIFT;
	mtk_nfi_writew(host, reg, MTKSDG1_NFI_PAGEFMT);

	/* configure ECC encoder */
	enc_size = (SECTOR_SIZE + MTKSDG1_NFI_FDM_REG_SIZE) << 3;
	dec_size = enc_size + ecc_level * MTK_ECC_PARITY_BITS;
	reg = ecc_bit | ECC_NFI_MODE | (enc_size << ECC_MS_SHIFT);
	mtk_ecc_writel(host, reg, MTKSDG1_ECC_ENCCNFG);

	/* configure ECC decoder */
	reg = ecc_bit | ECC_NFI_MODE | (dec_size << ECC_MS_SHIFT);
	reg |= (DEC_CNFG_CORRECT | DEC_EMPTY_EN);
	mtk_ecc_writel(host, reg, MTKSDG1_ECC_DECCNFG);
}

static void mtk_nfc_device_reset(struct mtk_nfc_host *host)
{
	unsigned long timeout = msecs_to_jiffies(MTK_TIMEOUT);
	struct device *dev = host->dev;
	u16 chip;
	int rc;

	mtk_nfc_hw_reset(host);
	init_completion(&host->complete);

	/* enable reset done interrupt */
	mtk_nfi_writew(host, INTR_RST_DONE_EN, MTKSDG1_NFI_INTR_EN);

	/* configure FSM for reset operation */
	mtk_nfi_writew(host, CNFG_OP_RESET, MTKSDG1_NFI_CNFG);

	mtk_nfc_set_command(host, NAND_CMD_RESET);
	rc = wait_for_completion_timeout(&host->complete, timeout);
	if (!rc) {
		chip = mtk_nfi_readw(host, MTKSDG1_NFI_CSEL);
		dev_err(dev, "device(%d) reset timeout\n", chip);
	}
}

static void mtk_nfc_select_chip(struct mtd_info *mtd, int chip)
{
	struct nand_chip *nand = mtd->priv;
	struct mtk_nfc_host *host = nand->priv;

	if (chip < 0)
		return;

	mtk_nfi_writel(host, chip, MTKSDG1_NFI_CSEL);
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
	struct mtk_nfc_host *host = ((struct nand_chip *)mtd->priv)->priv;
	unsigned long cmd_timeout = msecs_to_jiffies(MTK_TIMEOUT);
	struct completion *p = &host->complete;
	u32 val;
	int rc;

	if (mtk_nfc_cmd_supported(command))
		mtk_nfc_hw_reset(host);

	switch (command) {
	case NAND_CMD_RESET:
		mtk_nfc_device_reset(host);
		break;
	case NAND_CMD_READID:
		val = CNFG_READ_EN | CNFG_BYTE_RW | CNFG_OP_SRD;
		mtk_nfi_writew(host, val, MTKSDG1_NFI_CNFG);
		mtk_nfc_set_command(host, NAND_CMD_READID);
		mtk_nfc_set_address(host, column, 0, 1, 0);
		mtk_nfi_writel(host, CON_SRD, MTKSDG1_NFI_CON);
		break;
	case NAND_CMD_STATUS:
		val = CNFG_READ_EN | CNFG_BYTE_RW | CNFG_OP_SRD;
		mtk_nfi_writew(host, val, MTKSDG1_NFI_CNFG);
		mtk_nfc_set_command(host, NAND_CMD_STATUS);
		mtk_nfi_writel(host, CON_SRD, MTKSDG1_NFI_CON);
		break;
	case NAND_CMD_READOOB:
		val = CNFG_READ_EN | CNFG_BYTE_RW | CNFG_OP_READ;
		mtk_nfi_writew(host, val, MTKSDG1_NFI_CNFG);
		mtk_nfc_set_command(host, NAND_CMD_READ0);
		column += mtd->writesize;
		mtk_nfc_set_address(host, column, page_addr, 2, host->rownob);
		val = CON_BRD | (1 << CON_SEC_SHIFT);
		mtk_nfi_writel(host, val, MTKSDG1_NFI_CON);
		break;
	case NAND_CMD_ERASE1:
		mtk_nfi_writew(host, INTR_ERS_DONE_EN, MTKSDG1_NFI_INTR_EN);
		mtk_nfi_writew(host, CNFG_OP_ERASE, MTKSDG1_NFI_CNFG);
		mtk_nfc_set_command(host, NAND_CMD_ERASE1);
		mtk_nfc_set_address(host, 0, page_addr, 0, host->rownob);
		break;
	case NAND_CMD_ERASE2:
		init_completion(p);
		mtk_nfc_set_command(host, NAND_CMD_ERASE2);
		rc = wait_for_completion_timeout(p, cmd_timeout);
		if (!rc)
			dev_err(host->dev, "erase command timeout\n");
		break;
	case NAND_CMD_SEQIN:
		mtk_nfi_writew(host, CNFG_OP_PRGM, MTKSDG1_NFI_CNFG);
		mtk_nfc_set_command(host, NAND_CMD_SEQIN);
		mtk_nfc_set_address(host, column, page_addr, 2, host->rownob);
		break;
	case NAND_CMD_PAGEPROG:
	case NAND_CMD_CACHEDPROG:
		mtk_nfi_writew(host, INTR_BUSY_RT_EN, MTKSDG1_NFI_INTR_EN);
		init_completion(p);
		mtk_nfc_set_command(host, command);
		rc = wait_for_completion_timeout(p, cmd_timeout);
		if (!rc)
			dev_err(host->dev, "pageprogr command timeout\n");
		break;
	case NAND_CMD_READ0:
		val = CNFG_OP_READ | CNFG_READ_EN;
		mtk_nfi_writew(host, val, MTKSDG1_NFI_CNFG);
		mtk_nfc_set_command(host, NAND_CMD_READ0);
		break;
	default:
		dev_warn(host->dev, "command 0x%x not supported\n", command);
		break;
	}
}

static uint8_t mtk_nfc_read_byte(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	struct mtk_nfc_host *host = chip->priv;
	int rc;

	rc = mtk_nfc_data_ready(host);
	if (rc < 0) {
		dev_err(host->dev, "data not ready\n");
		return NAND_STATUS_FAIL;
	}

	return mtk_nfi_readb(host, MTKSDG1_NFI_DATAR);
}

static void mtk_nfc_write_fdm(struct nand_chip *chip, u32 sectors, int oob)
{
	struct mtk_nfc_host *host = chip->priv;
	int i, j, reg;
	size_t len;

	/* maximum number of fdm sector registers */
	if (sectors > MTKSDG1_NFI_FDM_MAX_SEC)
		sectors = MTKSDG1_NFI_FDM_MAX_SEC;

	len = MTKSDG1_NFI_FDM_REG_SIZE;
	for (i = 0; i < sectors ; i++) {
		memcpy(host->fdm_reg, chip->oob_poi + i * len, len);
		for (j = 0; j < ARRAY_SIZE(host->fdm_reg); j++) {
			reg = MTKSDG1_NFI_FDM0L;
			reg += j * sizeof(host->fdm_reg[0]);
			reg += i * MTKSDG1_NFI_FDM_REG_SIZE;
			mtk_nfi_writel(host, host->fdm_reg[j], reg);
		}
	}
}

static int mtk_nfc_write_page_hwecc(struct mtd_info *mtd,
			struct nand_chip *chip, const uint8_t *buf,
			int oob_required, int page)
{

	struct mtk_nfc_host *host = chip->priv;
	struct completion *p = &host->complete;
	u32 sectors = BYTES_TO_SECTORS(mtd->writesize);
	struct device *dev = host->dev;
	void *q = (void *) buf;
	dma_addr_t dma_addr;
	u32 reg;
	int ret;

	reg = mtk_nfi_readw(host, MTKSDG1_NFI_CNFG);
	reg |= CNFG_AUTO_FMT_EN | CNFG_HW_ECC_EN | CNFG_AHB | CNFG_DMA_BURST_EN;
	mtk_nfi_writew(host, reg, MTKSDG1_NFI_CNFG);

	/* enable ecc encoder*/
	mtk_ecc_encoder_idle(host);
	mtk_ecc_writew(host, ENC_EN, MTKSDG1_ECC_ENCCON);

	mtk_nfi_writel(host, sectors << CON_SEC_SHIFT, MTKSDG1_NFI_CON);
	mtk_nfi_writew(host, INTR_AHB_DONE_EN, MTKSDG1_NFI_INTR_EN);
	mtk_nfc_write_fdm(chip, sectors, oob_required);

	/* start dma */
	dma_addr = dma_map_single(dev, q, mtd->writesize, DMA_TO_DEVICE);
	mtk_nfi_writel(host, lower_32_bits(dma_addr), MTKSDG1_NFI_STRADDR);
	init_completion(p);
	reg = mtk_nfi_readl(host, MTKSDG1_NFI_CON);
	reg |= CON_BWR;
	mtk_nfi_writel(host, reg, MTKSDG1_NFI_CON);

	ret = wait_for_completion_timeout(p, msecs_to_jiffies(MTK_TIMEOUT));
	if (!ret)
		dev_warn(dev, "program ahb done timeout\n");

	/* um, dma interrupt didn't trigger...check transfer just in case*/
	ret = mtk_nfc_transfer_done(host, sectors);
	if (ret < 0)
		dev_err(dev, "hwecc write timeout\n");

	dma_unmap_single(host->dev, dma_addr, mtd->writesize, DMA_TO_DEVICE);

	/* disable ecc encoder */
	mtk_ecc_encoder_idle(host);
	mtk_ecc_writew(host, ENC_DE, MTKSDG1_ECC_ENCCON);

	/* clear config */
	mtk_nfi_writel(host, 0, MTKSDG1_NFI_CON);

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
	u32 sectors = BYTES_TO_SECTORS(mtd->writesize);
	struct mtk_nfc_host *host = chip->priv;
	struct device *dev = host->dev;
	u32 i, j, spare, reg;
	u8 *oob = chip->oob_poi;
	int rc;

	reg = mtk_nfi_readw(host, MTKSDG1_NFI_CNFG);
	reg |= CNFG_BYTE_RW;
	reg &= ~CNFG_AHB;
	mtk_nfi_writew(host, reg, MTKSDG1_NFI_CNFG);

	/* enable burst writes */
	reg = (sectors << CON_SEC_SHIFT) | CON_BWR;
	mtk_nfi_writel(host, reg, MTKSDG1_NFI_CON);

	spare = mtd->oobsize / sectors;
	rc = 0;
	for (i = 0; i < sectors && !rc; i++) {
		for (j = 0; j < SECTOR_SIZE && !rc; j++)
			rc = mtk_nfc_write_pio(host, buf++);
		for (j = 0; j < MTKSDG1_NFI_FDM_REG_SIZE && !rc; j++)
			rc = mtk_nfc_write_pio(host, oob++);
		for (j = 0; j < spare - MTKSDG1_NFI_FDM_REG_SIZE && !rc; j++)
			rc = mtk_nfc_write_pio(host, NULL);
	}

	if (rc < 0) {
		dev_err(dev, "nfc not ready for writing\n");
		goto done;
	}

	rc = mtk_nfc_transfer_done(host, sectors);
	if (rc < 0)
		dev_err(dev, "raw write timeout\n");
done:
	mtk_nfi_writel(host, 0, MTKSDG1_NFI_CON);

	return rc;
}

static int mtk_nfc_write_oob(struct mtd_info *mtd, struct nand_chip *chip,
				int page)
{
	u8 *buf = chip->buffers->databuf;
	int ret;

	memset(buf, 0xff, mtd->writesize);
	chip->cmdfunc(mtd, NAND_CMD_SEQIN, 0x00, page);

	ret = mtk_nfc_write_page_hwecc(mtd, chip, buf, 1, page);
	if (ret < 0)
		return -EIO;

	chip->cmdfunc(mtd, NAND_CMD_PAGEPROG, -1, -1);
	ret = chip->waitfunc(mtd, chip);

	return ret & NAND_STATUS_FAIL ? -EIO : 0;
}

static int mtk_nfc_write_oob_raw(struct mtd_info *mtd, struct nand_chip *chip,
					int page)
{
	u8 *buf = chip->buffers->databuf;
	int ret;

	memset(buf, 0xff, mtd->writesize);
	chip->cmdfunc(mtd, NAND_CMD_SEQIN, 0x00, page);

	ret = mtk_nfc_write_page_raw(mtd, chip, buf, 1, page);
	if (ret < 0)
		return -EIO;

	chip->cmdfunc(mtd, NAND_CMD_PAGEPROG, -1, -1);
	ret = chip->waitfunc(mtd, chip);

	return ret & NAND_STATUS_FAIL ? -EIO : 0;
}

static void mtk_nfc_read_fdm(struct nand_chip *chip, u32 sectors)
{
	struct mtk_nfc_host *host = chip->priv;
	int i, j, reg;
	size_t len;

	if (sectors > MTKSDG1_NFI_FDM_MAX_SEC) {
		dev_warn(host->dev, "read fdm can't process all sectors\n");
		sectors = MTKSDG1_NFI_FDM_MAX_SEC;
	}

	len = MTKSDG1_NFI_FDM_REG_SIZE;
	for (i = 0; i < sectors ; i++) {
		for (j = 0; j < ARRAY_SIZE(host->fdm_reg); j++) {
			reg = MTKSDG1_NFI_FDM0L;
			reg += j * sizeof(host->fdm_reg[0]);
			reg += i * MTKSDG1_NFI_FDM_REG_SIZE;
			host->fdm_reg[j] = mtk_nfi_readl(host, reg);
		}
		memcpy(chip->oob_poi + i * len, host->fdm_reg, len);
	}
}

static int mtk_nfc_ecc_check(struct mtd_info *mtd, struct nand_chip *chip,
				u8 *buf, u32 sectors)
{
	struct nand_oobfree *oob_free = chip->ecc.layout->oobfree;
	u32 empty_page, offset, i, err_num, max_bitflip = 0;
	struct mtk_nfc_host *host = chip->priv;
	uint8_t *p;

	/* check empty page */
	empty_page = mtk_nfi_readl(host, MTKSDG1_NFI_STA);
	empty_page &= STA_EMP_PAGE;
	if (empty_page) {
		memset(buf, 0xff, SECTORS_TO_BYTES(sectors));
		for (i = 0; i < sectors; i++) {
			p = chip->oob_poi + oob_free[i].offset;
			memset(p, 0xff, MTKSDG1_NFI_FDM_REG_SIZE);
		}

		return 0;
	}

	for (i = 0; i < sectors; i++) {
		offset = (i >> 2) << 2;
		err_num = mtk_ecc_readl(host, MTKSDG1_ECC_DECENUM0 + offset);
		err_num = err_num >> ((i % 4) * 8);
		err_num &= ERR_MASK;
		if (err_num == ERR_MASK) {
			/* uncorrectable errors */
			mtd->ecc_stats.failed++;
			continue;
		}

		mtd->ecc_stats.corrected += err_num;
		max_bitflip = max_t(u32, max_bitflip, err_num);
	}

	return max_bitflip;
}

static int mtk_nfc_subpage_done(struct mtk_nfc_host *host, int sectors)
{
	bool dma_done = false, dec_done = false, dec_fsm_idle = false;
	unsigned long timeout = jiffies + msecs_to_jiffies(MTK_TIMEOUT);
	u32 reg;

	do {
		if (!dma_done) {
			/* check the sector count at the DMA bytelen register */
			reg = mtk_nfi_readl(host, MTKSDG1_NFI_BYTELEN);
			reg &= CNTR_MASK;
			dma_done = reg >= sectors ? true : false;
		}
		if (!dec_done) {
			/* check that the ECC decoder completed */
			reg = mtk_ecc_readw(host, MTKSDG1_ECC_DECDONE);
			reg &= (1 << (sectors - 1));
			dec_done = reg ? true : false;
		}
		if (!dec_fsm_idle) {
			/* check that the ECC FSM is idle */
			reg = mtk_ecc_readl(host, MTKSDG1_ECC_DECFSM);
			reg &= DECFSM_MASK;
			dec_fsm_idle = reg == DECFSM_IDLE ? true : false;
		}
		if (dma_done && dec_done && dec_fsm_idle)
			return 0;

		cpu_relax();

	} while (time_before(jiffies, timeout));

	return -EIO;
}

static int mtk_nfc_read_subpage(struct mtd_info *mtd, struct nand_chip *chip,
		uint32_t data_offs, uint32_t readlen, uint8_t *bufpoi, int page)
{
	u32 start_sector = BYTES_TO_SECTORS(data_offs);
	struct mtk_nfc_host *host = chip->priv;
	struct completion *p = &host->complete;
	u32 sectors = BYTES_TO_SECTORS(readlen);
	u32 reg, column, spare;
	int bitflips = -EIO;
	dma_addr_t d;
	int rc;

	spare = mtd->oobsize / BYTES_TO_SECTORS(mtd->writesize);
	column =  start_sector * (SECTOR_SIZE + spare);

	/* configure the transfer  */
	reg = mtk_nfi_readw(host, MTKSDG1_NFI_CNFG);
	reg |= CNFG_AUTO_FMT_EN | CNFG_HW_ECC_EN | CNFG_DMA_BURST_EN | CNFG_AHB;
	mtk_nfi_writew(host, reg, MTKSDG1_NFI_CNFG);

	/* enable ecc decoder */
	mtk_ecc_decoder_idle(host);
	mtk_ecc_writel(host, DEC_EN, MTKSDG1_ECC_DECCON);

	/* select the sector number from the device */
	mtk_nfi_writel(host, sectors << CON_SEC_SHIFT, MTKSDG1_NFI_CON);
	mtk_nfi_writew(host, INTR_BUSY_RT_EN, MTKSDG1_NFI_INTR_EN);

	init_completion(p);
	mtk_nfc_set_address(host, column, page, 2, host->rownob);
	mtk_nfc_set_command(host, NAND_CMD_READSTART);
	rc = wait_for_completion_timeout(p, msecs_to_jiffies(MTK_TIMEOUT));
	if (!rc) {
		dev_err(host->dev, "read busy return timeout\n");
		goto read_error;
	}

	/* prepare the dma */
	mtk_nfi_writew(host, INTR_AHB_DONE_EN, MTKSDG1_NFI_INTR_EN);
	d = dma_map_single(host->dev, bufpoi, mtd->writesize, DMA_FROM_DEVICE);
	mtk_nfi_writel(host, lower_32_bits(d), MTKSDG1_NFI_STRADDR);

	init_completion(p);
	/* enable the data read operation: start dma */
	reg = mtk_nfi_readl(host, MTKSDG1_NFI_CON);
	reg |= CON_BRD;
	mtk_nfi_writel(host, reg, MTKSDG1_NFI_CON);
	rc = wait_for_completion_timeout(p, msecs_to_jiffies(MTK_TIMEOUT));
	if (!rc)
		dev_warn(host->dev, "read ahb/dma done timeout\n");

	/* um dma interrupt didn't trigger, check page done just in case */
	rc = mtk_nfc_subpage_done(host, sectors);
	if (rc < 0) {
		dev_err(host->dev, "subpage done timeout\n");
		goto dma_error;
	}

	mtk_nfc_read_fdm(chip, sectors);

	bitflips = mtk_nfc_ecc_check(mtd, chip, bufpoi, sectors);
	if (bitflips)
		dev_dbg(host->dev, "ecc bitflips = 0x%x\n", bitflips);
dma_error:

	dma_unmap_single(host->dev, d, mtd->writesize, DMA_FROM_DEVICE);

read_error:

	/* disable ecc decoder */
	mtk_ecc_decoder_idle(host);
	mtk_ecc_writew(host, DEC_DE, MTKSDG1_ECC_DECCON);

	mtk_nfi_writel(host, 0, MTKSDG1_NFI_CON);

	return bitflips;
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
	u32 sectors = BYTES_TO_SECTORS(mtd->writesize);
	struct completion *p = &host->complete;
	struct device *dev = host->dev;
	u32 i, j, spare, reg;
	u8 *oob = chip->oob_poi;
	int rc;

	/* configure read in PIO mode (byte at a time) */
	reg = mtk_nfi_readw(host, MTKSDG1_NFI_CNFG);
	reg |= CNFG_BYTE_RW;
	reg &= ~CNFG_AHB;
	mtk_nfi_writew(host, reg, MTKSDG1_NFI_CNFG);
	mtk_nfi_writew(host, INTR_BUSY_RT_EN, MTKSDG1_NFI_INTR_EN);

	init_completion(p);
	mtk_nfc_set_address(host, 0, page, 2, host->rownob);
	mtk_nfc_set_command(host, NAND_CMD_READSTART);
	rc = wait_for_completion_timeout(p, msecs_to_jiffies(MTK_TIMEOUT));
	if (!rc) {
		dev_err(host->dev, "raw read command timeout\n");
		goto error;
	}

	/* enable burst reads */
	reg = (sectors << CON_SEC_SHIFT) | CON_BRD;
	mtk_nfi_writel(host, reg, MTKSDG1_NFI_CON);

	spare = mtd->oobsize / sectors;
	rc = 0;
	for (i = 0; i < sectors && !rc; i++) {
		for (j = 0; j < SECTOR_SIZE && !rc; j++)
			rc = mtk_nfc_read_pio(host, buf++);
		for (j = 0; j < MTKSDG1_NFI_FDM_REG_SIZE && !rc; j++)
			rc = mtk_nfc_read_pio(host, oob++);
		for (j = 0; j < spare - MTKSDG1_NFI_FDM_REG_SIZE && !rc; j++)
			rc = mtk_nfc_read_pio(host, NULL);
	}

	if (rc < 0) {
		dev_err(dev, "nfc not ready for reading\n");
		goto error;
	}

	rc = mtk_nfc_transfer_done(host, sectors);
	if (rc < 0)
		dev_err(dev, "raw read transfer timeout\n");
error:
	mtk_nfi_writel(host, 0, MTKSDG1_NFI_CON);

	return rc;
}

static int mtk_nfc_read_oob(struct mtd_info *mtd, struct nand_chip *chip,
				int page)
{
	u8 *buf = chip->buffers->databuf;

	memset(buf, 0xff, mtd->writesize);
	chip->cmdfunc(mtd, NAND_CMD_READ0, 0, page);

	return mtk_nfc_read_page_hwecc(mtd, chip, buf, 1, page);
}

static int mtk_nfc_read_oob_raw(struct mtd_info *mtd, struct nand_chip *chip,
				int page)
{
	u8 *buf = chip->buffers->databuf;

	memset(buf, 0xff, mtd->writesize);
	chip->cmdfunc(mtd, NAND_CMD_READ0, 0, page);

	return mtk_nfc_read_page_raw(mtd, chip, buf, 1, page);
}

static inline void mtk_nfc_hw_init(struct mtk_nfc_host *host)
{
	/* read and write operations: proposed 2 wait states	*/
	/* 0001 - 000010 - 000000 - 0100.0010.0010.0010		*/
	/* todo: wait states vs performance			*/
	mtk_nfi_writel(host, 0x10804222, MTKSDG1_NFI_ACCCON);
	mtk_nfi_writew(host, 0xf1, MTKSDG1_NFI_CNRNB);
	mtk_nfc_hw_reset(host);

	/* clear interrupt */
	mtk_nfi_readl(host, MTKSDG1_NFI_INTR_STA);
	mtk_nfi_writel(host, 0, MTKSDG1_NFI_INTR_EN);

	/* ecc encoder init */
	mtk_ecc_encoder_idle(host);
	mtk_ecc_writew(host, ENC_DE, MTKSDG1_ECC_ENCCON);

	/* ecc decoder init */
	mtk_ecc_decoder_idle(host);
	mtk_ecc_writel(host, DEC_DE, MTKSDG1_ECC_DECCON);
}

static irqreturn_t mtk_nfc_irq(int irq, void *devid)
{
	struct mtk_nfc_host *host = devid;
	u16 sta, ien;

	sta = mtk_nfi_readw(host, MTKSDG1_NFI_INTR_STA);
	ien = mtk_nfi_readw(host, MTKSDG1_NFI_INTR_EN);

	if (!(sta & ien))
		return IRQ_NONE;

	mtk_nfi_writew(host, ~sta & ien, MTKSDG1_NFI_INTR_EN);
	complete(&host->complete);

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

	return ret;
}

static void mtk_nfc_disable_clk(struct mtk_nfc_clk *clk)
{
	clk_disable_unprepare(clk->nfi_clk);
	clk_disable_unprepare(clk->nfiecc_clk);
	clk_disable_unprepare(clk->pad_clk);
}

static int mtk_nfc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct mtd_part_parser_data ppdata;
	struct mtk_nfc_host *host;
	struct nand_chip *chip;
	struct mtd_info *mtd;
	struct resource *res;
	int ret, irq;

	host = devm_kzalloc(dev, sizeof(*host), GFP_KERNEL);
	if (!host)
		return -ENOMEM;

	chip = &host->chip;
	mtd = &host->mtd;
	host->dev = dev;

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
		dev_err(dev, "no ecc base\n");
		return ret;
	}

	host->clk.nfi_clk = devm_clk_get(dev, "nfi_clk");
	if (IS_ERR(host->clk.nfi_clk)) {
		dev_err(dev, "no clk\n");
		ret = PTR_ERR(host->clk.nfi_clk);
		return ret;
	}

	host->clk.nfiecc_clk = devm_clk_get(dev, "nfiecc_clk");
	if (IS_ERR(host->clk.nfiecc_clk)) {
		dev_err(dev, "no ecc clk\n");
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
		dev_err(dev, "no irq resource\n");
		ret = -EINVAL;
		goto clk_disable;
	}

	ret = devm_request_irq(dev, irq, mtk_nfc_irq, 0x0, MTK_NAME, host);
	if (ret) {
		dev_err(dev, "failed to request irq\n");
		goto clk_disable;
	}

	ret = dma_set_mask(dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(dev, "failed to set dma mask\n");
		goto clk_disable;
	}

	platform_set_drvdata(pdev, host);

	mtd->owner = THIS_MODULE;
	mtd->dev.parent = dev;
	mtd->name = MTK_NAME;
	mtd->priv = chip;

	chip->select_chip = mtk_nfc_select_chip;
	chip->read_byte = mtk_nfc_read_byte;
	chip->cmdfunc = mtk_nfc_cmdfunc;
	chip->ecc.mode = NAND_ECC_HW;
	chip->priv = host;

	chip->ecc.write_subpage = mtk_nfc_write_subpage_hwecc;
	chip->ecc.write_page_raw = mtk_nfc_write_page_raw;
	chip->ecc.write_page = mtk_nfc_write_page_hwecc;
	chip->ecc.write_oob_raw = mtk_nfc_write_oob_raw;
	chip->ecc.write_oob = mtk_nfc_write_oob;

	chip->ecc.read_page_raw = mtk_nfc_read_page_raw;
	chip->ecc.read_subpage = mtk_nfc_read_subpage;
	chip->ecc.read_oob_raw = mtk_nfc_read_oob_raw;
	chip->ecc.read_page = mtk_nfc_read_page_hwecc;
	chip->ecc.read_oob = mtk_nfc_read_oob;

	if (of_get_nand_on_flash_bbt(np))
		chip->bbt_options |= NAND_BBT_USE_FLASH;

	mtk_nfc_hw_init(host);

	ret = nand_scan_ident(mtd, MTK_NAND_MAX_CHIP, NULL);
	if (ret) {
		ret = -ENODEV;
		goto clk_disable;
	}

	mtk_nfc_hw_config(mtd);

	ret = nand_scan_tail(mtd);
	if (ret) {
		ret = -ENODEV;
		goto clk_disable;
	}

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

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int mtk_nfc_suspend(struct device *dev)
{
	struct mtk_nfc_host *host = dev_get_drvdata(dev);
	struct mtk_nfc_saved_reg *reg = &host->saved_reg;

	reg->nfi_pagefmt = mtk_nfi_readw(host, MTKSDG1_NFI_PAGEFMT);
	reg->nfi_acccon = mtk_nfi_readl(host, MTKSDG1_NFI_ACCCON);
	reg->nfi_csel = mtk_nfi_readw(host, MTKSDG1_NFI_CSEL);

	reg->ecc_enccnfg = mtk_ecc_readl(host, MTKSDG1_ECC_ENCCNFG);
	reg->ecc_deccnfg = mtk_ecc_readl(host, MTKSDG1_ECC_DECCNFG);

	mtk_nfc_disable_clk(&host->clk);

	return 0;
}

static int mtk_nfc_resume(struct device *dev)
{
	struct mtk_nfc_host *host = dev_get_drvdata(dev);
	struct mtk_nfc_saved_reg *reg = &host->saved_reg;
	struct nand_chip *chip = &host->chip;
	struct mtd_info *mtd = &host->mtd;
	int ret;
	u32 i;

	/* todo: delay 200us for POR flow? why? */
	udelay(200);

	for (i = 0; i < chip->numchips; i++) {
		chip->select_chip(mtd, i);
		chip->cmdfunc(mtd, NAND_CMD_RESET, -1, -1);
	}

	ret = mtk_nfc_enable_clk(dev, &host->clk);
	if (ret)
		return ret;

	/* nfi registers */
	mtk_nfi_writew(host, reg->nfi_pagefmt, MTKSDG1_NFI_PAGEFMT);
	mtk_nfi_writel(host, reg->nfi_acccon, MTKSDG1_NFI_ACCCON);
	mtk_nfi_writew(host, reg->nfi_csel, MTKSDG1_NFI_CSEL);

	/* ecc registers */
	mtk_ecc_writel(host, reg->ecc_enccnfg, MTKSDG1_ECC_ENCCNFG);
	mtk_ecc_writel(host, reg->ecc_deccnfg, MTKSDG1_ECC_DECCNFG);

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
MODULE_DESCRIPTION("MTK Nand Flash Controller Driver");

