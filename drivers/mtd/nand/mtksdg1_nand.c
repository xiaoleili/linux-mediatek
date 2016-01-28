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

struct mtk_nfc_clk {
	struct clk *nfi_clk;
	struct clk *nfiecc_clk;
	struct clk *pad_clk;
};

struct mtk_nfc_host {
	struct nand_chip chip;
	struct mtd_info mtd;
	struct device *dev;
	void __iomem *nfi_base;
	void __iomem *nfiecc_base;
	struct mtk_nfc_clk clk;
	struct completion complete;
	bool fmt_initialized;
};

static void mtk_nfc_set_command(struct mtk_nfc_host *host, u8 command)
{
	writel(command, host->nfi_base + MTKSDG1_NFC_CMD);
	while ((readl(host->nfi_base + MTKSDG1_NFC_STA) & STA_CMD) != 0)
		;
}

static void mtk_nfc_set_address(struct mtk_nfc_host *host, u32 column, u32 row,
		u8 colnob, u8 rownob)
{
	writel(column, host->nfi_base + MTKSDG1_NFC_COLADDR);
	writel(row, host->nfi_base + MTKSDG1_NFC_ROWADDR);
	writel(colnob | (rownob << ADDR_ROW_NOB_SHIFT), host->nfi_base + MTKSDG1_NFC_ADDRNOB);
	while ((readl(host->nfi_base + MTKSDG1_NFC_STA) & STA_ADDR) != 0)
		;
}

static void mtk_nfc_hw_reset(struct mtk_nfc_host *host)
{
	writel(CON_FIFO_FLUSH | CON_NFI_RST, host->nfi_base + MTKSDG1_NFC_CON);
	while ((readl(host->nfi_base + MTKSDG1_NFC_MASTER_STA) & 0x0fff) != 0)
		;
	writel(CON_FIFO_FLUSH | CON_NFI_RST, host->nfi_base + MTKSDG1_NFC_CON);
	mtk_nfc_set_command(host, NAND_CMD_RESET);
};

static void mtk_nfc_device_reset(struct mtk_nfc_host *host)
{
	int ret;

	mtk_nfc_hw_reset(host);
	writew(INTR_RST_DONE_EN, host->nfi_base + MTKSDG1_NFC_INTR_EN);
	writew(CNFG_OP_RESET, host->nfi_base + MTKSDG1_NFC_CNFG);
	writew(0xf1, host->nfi_base + MTKSDG1_NFC_CNRNB);
	init_completion(&host->complete);
	ret = wait_for_completion_timeout(&host->complete, MTK_DEFAULT_TIMEOUT);
	if (!ret)
		dev_err(host->dev, "device reset timeout!\n");
}

static irqreturn_t mtk_irq_handle(int irq, void *devid)
{
	struct mtk_nfc_host *host = devid;
	u16 sta, ien;

	sta = readw(host->nfi_base + MTKSDG1_NFC_INTR_STA);
	ien = readw(host->nfi_base + MTKSDG1_NFC_INTR_EN);

	pr_err("[xl] sta 0x%x ien 0x%x\n",sta ,ien);

	if (!(sta & ien))
		return IRQ_NONE;
	if ((sta & ien) == ien)
		complete(&host->complete);
	writew(~sta & ien, host->nfi_base + MTKSDG1_NFC_INTR_EN);
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

	if ((chip > 0) && (chip < MTK_NAND_MAX_CHIP))
		writel(chip, host->nfi_base + MTKSDG1_NFC_CSEL);
}

static void mtk_nfc_cmdfunc(struct mtd_info *mtd, unsigned command, int column,
		int page_addr)
{
	struct nand_chip *chip = mtd->priv;
	struct mtk_nfc_host *host = chip->priv;

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
		while ((readl(host->nfi_base + MTKSDG1_NFC_STA) & STA_DATAR) != 0)
			;
	default:
		break;
	}
}

static uint8_t mtk_nfc_read_byte(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	struct mtk_nfc_host *host = chip->priv;

	while (readb(host->nfi_base + MTKSDG1_NFC_PIO_DIRDY) == 0)
		;
	return readb(host->nfi_base + MTKSDG1_NFC_DATAR);
}

static void mtk_nfc_hw_init(struct mtk_nfc_host *host)
{
	writel(0x10804222, host->nfi_base + MTKSDG1_NFC_ACCCON);
	mtk_nfc_hw_reset(host);
	/* clear interrupt */
	readl(host->nfi_base + MTKSDG1_NFC_INTR_STA);
	writel(0, host->nfi_base + MTKSDG1_NFC_INTR_EN);
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
	ret = devm_request_irq(dev, irq, mtk_irq_handle, 0x0, "mtk-nand", host);
	if (ret) {
		dev_err(dev, "failed to request NFI IRQ\n");
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
	chip->select_chip = mtk_nfc_select_chip;
	chip->cmdfunc = mtk_nfc_cmdfunc;
	chip->read_byte = mtk_nfc_read_byte;

	mtk_nfc_hw_init(host);

	ret = nand_scan(mtd, max_chip);
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
	return 0;
}

static int mtk_nfc_resume(struct device *dev)
{
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

