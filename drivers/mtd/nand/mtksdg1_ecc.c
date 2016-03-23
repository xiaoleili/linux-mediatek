/*
 * MTK smart device ECC  controller driver.
 * Copyright (C) 2016  MediaTek Inc.
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
#include <linux/clk.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/mtd.h>
#include <linux/module.h>
#include <linux/iopoll.h>
#include <linux/of.h>
#include <linux/of_platform.h>

#include "mtksdg1_ecc.h"
#define MTKSDG1_ECC_ENCCON		(0x00)
#define		ENC_EN			(1)
#define		ENC_DE			(0)
#define MTKSDG1_ECC_ENCCNFG		(0x04)
#define		ECC_CNFG_4BIT		(0)
#define		ECC_CNFG_12BIT		(4)
#define		ECC_CNFG_24BIT		(10)
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


#define ECC_TIMEOUT		(500000)
#define MTK_ECC_PARITY_BITS	(14)

struct sdg1_pm_regs {
	u32 enccnfg;
	u32 deccnfg;
};

struct sdg1_ecc {
	struct device *dev;
	void __iomem *regs;
	struct mutex lock;
	struct clk *clk;

	struct completion done;
	u32 sec_mask;

#ifdef CONFIG_PM_SLEEP
	struct sdg1_pm_regs pm_regs;
#endif

};

static inline void sdg1_ecc_encoder_idle(struct sdg1_ecc *ecc)
{
	struct device *dev = ecc->dev;
	u32 val;
	int ret;

	ret = readl_poll_timeout_atomic(ecc->regs + MTKSDG1_ECC_ENCIDLE, val,
					val & ENC_IDLE, 10, ECC_TIMEOUT);
	if (ret)
		dev_warn(dev, "encoder NOT idle\n");
}

static inline void sdg1_ecc_decoder_idle(struct sdg1_ecc *ecc)
{
	struct device *dev = ecc->dev;
	u32 val;
	int ret;

	ret = readl_poll_timeout_atomic(ecc->regs + MTKSDG1_ECC_DECIDLE, val,
					val & DEC_IDLE, 10, ECC_TIMEOUT);
	if (ret)
		dev_warn(dev, "decoder NOT idle\n");
}

static irqreturn_t sdg1_ecc_irq(int irq, void *id)
{
	struct sdg1_ecc *ecc = id;
	u32 dec, enc;

	dec = readw(ecc->regs + MTKSDG1_ECC_DECIRQ_STA) & DEC_IRQEN;
	enc = readl(ecc->regs + MTKSDG1_ECC_ENCIRQ_STA) & ENC_IRQEN;

	if (!(dec || enc))
		return IRQ_NONE;

	if (dec) {
		dec = readw(ecc->regs + MTKSDG1_ECC_DECDONE);
		if (dec & ecc->sec_mask) {
			ecc->sec_mask = 0;
			complete(&ecc->done);
			writew(0, ecc->regs + MTKSDG1_ECC_DECIRQ_EN);
		}
	} else {
		complete(&ecc->done);
		writel(0, ecc->regs + MTKSDG1_ECC_ENCIRQ_EN);
	}

	return IRQ_HANDLED;
}

void sdg1_ecc_get_stats(struct sdg1_ecc *ecc, struct sdg1_ecc_stats *stats)
{
	u32 offset, i, err;
	u32 bitflips = 0;

	stats->corrected = 0;
	stats->failed = 0;

	for (i = 0; i < stats->sectors; i++) {
		offset = (i >> 2) << 2;
		err = readl(ecc->regs + MTKSDG1_ECC_DECENUM0 + offset);
		err = err >> ((i % 4) * 8);
		err &= ERR_MASK;
		if (err == ERR_MASK) {
			/* uncorrectable errors */
			stats->failed++;
			continue;
		}

		stats->corrected += err;
		bitflips = max_t(u32, bitflips, err);
	}

	stats->bitflips = bitflips;
}
EXPORT_SYMBOL(sdg1_ecc_get_stats);

void sdg1_ecc_release(struct sdg1_ecc *ecc)
{
	clk_disable_unprepare(ecc->clk);
	put_device(ecc->dev);
}
EXPORT_SYMBOL(sdg1_ecc_release);

static struct sdg1_ecc *sdg1_ecc_get(struct device_node *np)
{
	struct platform_device *pdev;
	struct sdg1_ecc *ecc;

	pdev = of_find_device_by_node(np);
	if (!pdev || !platform_get_drvdata(pdev))
		return ERR_PTR(-EPROBE_DEFER);

	get_device(&pdev->dev);
	ecc = platform_get_drvdata(pdev);

	clk_prepare_enable(ecc->clk);

	return ecc;
}

struct sdg1_ecc *of_sdg1_ecc_get(struct device_node *of_node)
{
	struct sdg1_ecc *ecc = NULL;
	struct device_node *np;

	np = of_parse_phandle(of_node, "mediatek,ecc-controller", 0);
	if (np) {
		ecc = sdg1_ecc_get(np);
		of_node_put(np);
	}

	return ecc;
}
EXPORT_SYMBOL(of_sdg1_ecc_get);

void sdg1_ecc_enable_encode(struct sdg1_ecc *ecc)
{
	sdg1_ecc_encoder_idle(ecc);
	writew(ENC_EN, ecc->regs + MTKSDG1_ECC_ENCCON);
}
EXPORT_SYMBOL(sdg1_ecc_enable_encode);

void sdg1_ecc_disable_encode(struct sdg1_ecc *ecc)
{
	writew(0, ecc->regs + MTKSDG1_ECC_ENCIRQ_EN);
	sdg1_ecc_encoder_idle(ecc);
	writew(ENC_DE, ecc->regs + MTKSDG1_ECC_ENCCON);
}
EXPORT_SYMBOL(sdg1_ecc_disable_encode);

void sdg1_ecc_enable_decode(struct sdg1_ecc *ecc)
{
	sdg1_ecc_decoder_idle(ecc);
	writel(DEC_EN, ecc->regs + MTKSDG1_ECC_DECCON);
}
EXPORT_SYMBOL(sdg1_ecc_enable_decode);

void sdg1_ecc_disable_decode(struct sdg1_ecc *ecc)
{
	writew(0, ecc->regs + MTKSDG1_ECC_DECIRQ_EN);
	sdg1_ecc_decoder_idle(ecc);
	writel(DEC_DE, ecc->regs + MTKSDG1_ECC_DECCON);
}
EXPORT_SYMBOL(sdg1_ecc_disable_decode);

void sdg1_ecc_start_decode(struct sdg1_ecc *ecc, int sectors)
{
	ecc->sec_mask = 1 << (sectors - 1);
	init_completion(&ecc->done);
	writew(DEC_IRQEN, ecc->regs + MTKSDG1_ECC_DECIRQ_EN);
}
EXPORT_SYMBOL(sdg1_ecc_start_decode);

int sdg1_ecc_wait_decode(struct sdg1_ecc *ecc)
{
	int ret;

	ret = wait_for_completion_timeout(&ecc->done, msecs_to_jiffies(500));
	if (!ret) {
		dev_err(ecc->dev, "decode timeout\n");
		return -ETIMEDOUT;
	}

	return 0;
}
EXPORT_SYMBOL(sdg1_ecc_wait_decode);

int sdg1_ecc_start_encode(struct sdg1_ecc *ecc, struct sdg1_enc_data *d)
{
	dma_addr_t addr;
	u32 *p, len;
	u32 reg, i;
	int rc, ret = 0;

	addr = dma_map_single(ecc->dev, d->data, d->len, DMA_TO_DEVICE);
	rc = dma_mapping_error(ecc->dev, addr);
	if (rc) {
		dev_err(ecc->dev, "dma mapping error\n");
		return -EINVAL;
	}

	/* enable the encoder in DMA mode to calculate the ECC bytes  */
	reg = readl(ecc->regs + MTKSDG1_ECC_ENCCNFG) & ~ECC_ENC_MODE_MASK;
	reg |= ECC_DMA_MODE;
	writel(reg, ecc->regs + MTKSDG1_ECC_ENCCNFG);

	writel(ENC_IRQEN, ecc->regs + MTKSDG1_ECC_ENCIRQ_EN);
	writel(lower_32_bits(addr), ecc->regs + MTKSDG1_ECC_ENCDIADDR);

	init_completion(&ecc->done);
	writew(ENC_EN, ecc->regs + MTKSDG1_ECC_ENCCON);

	rc = wait_for_completion_timeout(&ecc->done, msecs_to_jiffies(500));
	if (!rc) {
		dev_err(ecc->dev, "encode timeout\n");
		writel(0, ecc->regs + MTKSDG1_ECC_ENCIRQ_EN);
		ret = -ETIMEDOUT;
		goto timeout;
	}

	sdg1_ecc_encoder_idle(ecc);

	/* Program ECC bytes to OOB: per sector oob = FDM + ECC + SPARE */
	len = (d->strength * MTK_ECC_PARITY_BITS + 7) >> 3;
	p = (u32 *) (d->data + d->len);

	/* write the parity bytes generated by the ECC back to the OOB region */
	for (i = 0; i < len; i++)
		p[i] = readl(ecc->regs + MTKSDG1_ECC_ENCPAR0 + i * sizeof(u32));

timeout:

	dma_unmap_single(ecc->dev, addr, d->len, DMA_TO_DEVICE);

	writew(0, ecc->regs + MTKSDG1_ECC_ENCCON);
	reg = readl(ecc->regs + MTKSDG1_ECC_ENCCNFG) & ~ECC_ENC_MODE_MASK;
	reg |= ECC_NFI_MODE;
	writel(reg, ecc->regs + MTKSDG1_ECC_ENCCNFG);

	return ret;
}
EXPORT_SYMBOL(sdg1_ecc_start_encode);

void sdg1_ecc_hw_init(struct sdg1_ecc *ecc)
{
	sdg1_ecc_encoder_idle(ecc);
	writew(ENC_DE, ecc->regs + MTKSDG1_ECC_ENCCON);

	sdg1_ecc_decoder_idle(ecc);
	writel(DEC_DE, ecc->regs + MTKSDG1_ECC_DECCON);
}
EXPORT_SYMBOL(sdg1_ecc_hw_init);

int sdg1_ecc_config(struct sdg1_ecc *ecc, int strength, int step_len)
{
	u32 ecc_bit, dec_sz, enc_sz;
	u32 reg;

	switch (strength) {
	case 4:
		ecc_bit = ECC_CNFG_4BIT;
		break;
	case 12:
		ecc_bit = ECC_CNFG_12BIT;
		break;
	case 24:
		ecc_bit = ECC_CNFG_24BIT;
		break;
	default:
		dev_err(ecc->dev, "invalid spare len per sector\n");
		return -EINVAL;
	}

	/* configure ECC encoder (in bits) */
	enc_sz = step_len << 3;
	reg = ecc_bit | ECC_NFI_MODE | (enc_sz << ECC_MS_SHIFT);
	writel(reg, ecc->regs + MTKSDG1_ECC_ENCCNFG);

	/* configure ECC decoder (in bits) */
	dec_sz = enc_sz + strength * MTK_ECC_PARITY_BITS;
	reg = ecc_bit | ECC_NFI_MODE | (dec_sz << ECC_MS_SHIFT);
	reg |= DEC_CNFG_CORRECT | DEC_EMPTY_EN;
	writel(reg, ecc->regs + MTKSDG1_ECC_DECCNFG);

	return 0;
}
EXPORT_SYMBOL(sdg1_ecc_config);

static int sdg1_ecc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sdg1_ecc *ecc;
	struct resource *res;
	int irq, ret;

	ecc = devm_kzalloc(dev, sizeof(*ecc), GFP_KERNEL);
	if (!ecc)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ecc->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(ecc->regs)) {
		dev_err(dev, "failed to map regs: %ld\n", PTR_ERR(ecc->regs));
		return PTR_ERR(ecc->regs);
	}

	ecc->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(ecc->clk)) {
		dev_err(dev, "failed to get clock: %ld\n", PTR_ERR(ecc->clk));
		return PTR_ERR(ecc->clk);
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(dev, "failed to get irq\n");
		return -EINVAL;
	}

	ret = dma_set_mask(dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(dev, "failed to set DMA mask\n");
		return ret;
	}

	ret = devm_request_irq(dev, irq, sdg1_ecc_irq, 0x0, "sdg1-ecc", ecc);
	if (ret) {
		dev_err(dev, "failed to request irq\n");
		return -EINVAL;
	}

	mutex_init(&ecc->lock);

	ecc->dev = dev;

	platform_set_drvdata(pdev, ecc);

	dev_info(dev, "driver probed\n");

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int sdg1_ecc_suspend(struct device *dev)
{
	struct sdg1_ecc *ecc = dev_get_drvdata(dev);
	struct sdg1_pm_regs *regs = &ecc->pm_regs;

	regs->enccnfg = readl(ecc->regs + MTKSDG1_ECC_ENCCNFG);
	regs->deccnfg = readl(ecc->regs + MTKSDG1_ECC_DECCNFG);

	clk_disable_unprepare(ecc->clk);

	return 0;
}

static int sdg1_ecc_resume(struct device *dev)
{
	struct sdg1_ecc *ecc = dev_get_drvdata(dev);
	struct sdg1_pm_regs *regs = &ecc->pm_regs;
	int ret;

	ret = clk_prepare_enable(ecc->clk);
	if (ret) {
		dev_err(dev, "failed to enable clk\n");
		return ret;
	}

	writel(regs->enccnfg, ecc->regs + MTKSDG1_ECC_ENCCNFG);
	writel(regs->deccnfg, ecc->regs + MTKSDG1_ECC_DECCNFG);

	return 0;
}

static SIMPLE_DEV_PM_OPS(sdg1_ecc_pm_ops, sdg1_ecc_suspend, sdg1_ecc_resume);
#endif

static const struct of_device_id sdg1_ecc_dt_match[] = {
	{ .compatible = "mediatek,mt2701-ecc" },
	{},
};

MODULE_DEVICE_TABLE(of, sdg1_ecc_dt_match);

static struct platform_driver sdg1_ecc_driver = {
	.probe  = sdg1_ecc_probe,
	.driver = {
		.name  = "mtksdg1-ecc",
		.of_match_table = of_match_ptr(sdg1_ecc_dt_match),
#ifdef CONFIG_PM_SLEEP
		.pm = &sdg1_ecc_pm_ops,
#endif

	},
};

module_platform_driver(sdg1_ecc_driver);

MODULE_AUTHOR("Xiaolei Li <xiaolei.li@mediatek.com>");
MODULE_AUTHOR("Jorge Ramirez-Ortiz <jorge.ramirez-ortiz@linaro.org>");
MODULE_DESCRIPTION("MTK Nand ECC Driver");
MODULE_LICENSE("GPL");
