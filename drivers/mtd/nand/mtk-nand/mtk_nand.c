/******************************************************************************
* mtk_nand.c - MTK NAND Flash Device Driver
 *
* Copyright 2009-2012 MediaTek Co.,Ltd.
 *
* DESCRIPTION:
*	This file provid the other drivers nand relative functions
 *
* modification history
* ----------------------------------------
* v3.0, 11 Feb 2010, mtk
* ----------------------------------------
******************************************************************************/

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/slab.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/dma-mapping.h>
#include <linux/jiffies.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/time.h>
#include <linux/mm.h>
#include <asm/io.h>
#include <asm/cacheflush.h>
#include <asm/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/gpio.h>
#include "mtk_nand.h"
#include <linux/rtc.h>
#include <nand_device_define.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>

#ifndef FALSE
#define FALSE (0)
#endif

#ifndef TRUE
#define TRUE  (1)
#endif

#ifndef NULL
#define NULL  (0)
#endif

#define READ_REGISTER_UINT8(reg) \
	(*(volatile unsigned char * const)(reg))

#define READ_REGISTER_UINT16(reg) \
	(*(volatile unsigned short * const)(reg))

#define READ_REGISTER_UINT32(reg) \
	(*(volatile unsigned int * const)(reg))


#define INREG8(x)			READ_REGISTER_UINT8((unsigned char *)((void *)(x)))
#define INREG16(x)			READ_REGISTER_UINT16((unsigned short *)((void *)(x)))
#define INREG32(x)			READ_REGISTER_UINT32((unsigned int *)((void *)(x)))
#define DRV_Reg8(addr)				INREG8(addr)
#define DRV_Reg16(addr)				INREG16(addr)
#define DRV_Reg32(addr)				INREG32(addr)
#define DRV_Reg(addr)				DRV_Reg16(addr)

#define WRITE_REGISTER_UINT8(reg, val) \
	((*(volatile unsigned char * const)(reg)) = (val))
#define WRITE_REGISTER_UINT16(reg, val) \
	((*(volatile unsigned short * const)(reg)) = (val))
#define WRITE_REGISTER_UINT32(reg, val) \
	((*(volatile unsigned int * const)(reg)) = (val))


#define OUTREG8(x, y)		WRITE_REGISTER_UINT8((unsigned char *)((void *)(x)), (unsigned char)(y))
#define OUTREG16(x, y)		WRITE_REGISTER_UINT16((unsigned short *)((void *)(x)), (unsigned short)(y))
#define OUTREG32(x, y)		WRITE_REGISTER_UINT32((unsigned int *)((void *)(x)), (unsigned int)(y))
#define DRV_WriteReg8(addr, data)	OUTREG8(addr, data)
#define DRV_WriteReg16(addr, data)	OUTREG16(addr, data)
#define DRV_WriteReg32(addr, data)	OUTREG32(addr, data)
#define DRV_WriteReg(addr, data)	DRV_WriteReg16(addr, data)


static const flashdev_info_t gen_FlashTable_p[] = {
	{{0x2C, 0xDC, 0x90, 0xA6, 0x54, 0x00}, 5, 5, IO_8BIT, 512, 256, 4096, 128,
	 0x31C08222, 0xC03222, 0x101, 80, VEND_MICRON, 1024, "MT29F4G08ABAEA", 0,
	 {MICRON_8K, {0xEF, 0xEE, 0xFF, 7, 0x89, 0, 1, RTYPE_MICRON, {0x1, 0x14}, {0x1, 0x5} },
	 {RAND_TYPE_SAMSUNG, {0x2D2D, 1, 1, 1, 1, 1} } } },
};

static unsigned int flash_number = sizeof(gen_FlashTable_p) / sizeof(flashdev_info_t);
#define NFI_DEFAULT_CS				(0)

#define mtk_nand_assert(expr)  do { \
	if (unlikely(!(expr))) { \
		pr_crit("MTK nand assert failed in %s at %u (pid %d)\n", \
			   __func__, __LINE__, current->pid); \
		dump_stack();	\
	}	\
} while (0)

struct clk *nfi_hclk;
struct clk *nfiecc_bclk;
struct clk *nfi_bclk;
struct clk *onfi_sel_clk;
struct clk *onfi_26m_clk;
struct clk *onfi_mode5;
struct clk *onfi_mode4;
struct clk *nfi_bclk_sel;
struct clk *nfi_ahb_clk;
struct clk *nfi_1xpad_clk;
struct clk *nfi_ecc_pclk;
struct clk *nfi_pclk;
struct clk *onfi_pad_clk;
struct regulator *mtk_nand_regulator;

#define VERSION	"v2.1 Fix AHB virt2phys error"
#define MODULE_NAME	"# MTK NAND #"
#define PROCNAME	"driver/nand"
#define _MTK_NAND_DUMMY_DRIVER_
#define __INTERNAL_USE_AHB_MODE__	(1)
#define CFG_FPGA_PLATFORM (0)	/* for fpga by bean */
#define CFG_RANDOMIZER	  (1)	/* for randomizer code */
#define CFG_PERFLOG_DEBUG (0)	/* for performance log */
#define CFG_2CS_NAND	(1)	/* for 2CS nand */
#define CFG_COMBO_NAND	  (1)	/* for Combo nand */

#define NFI_TRICKY_CS  (1)	/* must be 1 or > 1? */

void __iomem *mtk_nfi_base;
void __iomem *mtk_nfiecc_base;
struct device_node *mtk_nfiecc_node;
unsigned int nfi_irq;
#define MT_NFI_IRQ_ID nfi_irq

void __iomem *mtk_gpio_base;
struct device_node *mtk_gpio_node;
#define GPIO_BASE	mtk_gpio_base

void __iomem *mtk_infra_base;
struct device_node *mtk_infra_node;

/*
 * NFI controller version define
 *
 * 1: MT8127
 * 2: MT8163
 * Reserved.
 */
struct mtk_nfi_compatible {
	unsigned char chip_ver;
};

static const struct mtk_nfi_compatible mt2701_compat = {
	.chip_ver = 1,
};

static const struct mtk_nfi_compatible mt8163_compat = {
	.chip_ver = 2,
};

static const struct of_device_id mtk_nfi_of_match[] = {
	{ .compatible = "mediatek,mt2701-nfi", .data = &mt2701_compat },
	{}
};

const struct mtk_nfi_compatible *mtk_nfi_dev_comp;

struct device *mtk_dev;
struct scatterlist mtk_sg;
enum dma_data_direction mtk_dir;

#define ERR_RTN_SUCCESS   1
#define ERR_RTN_FAIL	  0
#define ERR_RTN_BCH_FAIL -1

#define NFI_SET_REG32(reg, value) \
do {	\
	g_value = (DRV_Reg32(reg) | (value));\
	DRV_WriteReg32(reg, g_value); \
} while (0)

#define NFI_SET_REG16(reg, value) \
do {	\
	g_value = (DRV_Reg16(reg) | (value));\
	DRV_WriteReg16(reg, g_value); \
} while (0)

#define NFI_CLN_REG32(reg, value) \
do {	\
	g_value = (DRV_Reg32(reg) & (~(value)));\
	DRV_WriteReg32(reg, g_value); \
} while (0)

#define NFI_CLN_REG16(reg, value) \
do {	\
	g_value = (DRV_Reg16(reg) & (~(value)));\
	DRV_WriteReg16(reg, g_value); \
} while (0)

#define NFI_WAIT_STATE_DONE(state) do {; } while (__raw_readl(NFI_STA_REG32) & state)
#define NFI_WAIT_TO_READY()  do {; } while (!(__raw_readl(NFI_STA_REG32) & STA_BUSY2READY))
#define FIFO_PIO_READY(x)  (0x1 & x)
#define WAIT_NFI_PIO_READY(timeout) \
do {\
	while ((!FIFO_PIO_READY(DRV_Reg(NFI_PIO_DIRDY_REG16))) && (--timeout)) \
		;\
} while (0)


#define NAND_SECTOR_SIZE (512)
#define OOB_PER_SECTOR		(16)
#define OOB_AVAI_PER_SECTOR (8)

u8 ecc_threshold;
#define PMT_POOL_SIZE	(2)

#define TIMEOUT_1	0x1fff
#define TIMEOUT_2	0x8ff
#define TIMEOUT_3	0xffff
#define TIMEOUT_4	0xffff	/* 5000   //PIO */

#define NFI_ISSUE_COMMAND(cmd, col_addr, row_addr, col_num, row_num) \
	do { \
		DRV_WriteReg(NFI_CMD_REG16, cmd);\
		while (DRV_Reg32(NFI_STA_REG32) & STA_CMD_STATE)\
			;\
		DRV_WriteReg32(NFI_COLADDR_REG32, col_addr);\
		DRV_WriteReg32(NFI_ROWADDR_REG32, row_addr);\
		DRV_WriteReg(NFI_ADDRNOB_REG16, col_num | (row_num<<ADDR_ROW_NOB_SHIFT))\
			;\
		while (DRV_Reg32(NFI_STA_REG32) & STA_ADDR_STATE)\
			;\
	} while (0)

/* ------------------------------------------------------------------------------- */
static struct completion g_comp_AHB_Done;
static struct NAND_CMD g_kCMD;
bool g_bInitDone;
static int g_i4Interrupt;
static bool g_bcmdstatus;
/* static bool g_brandstatus; */
static u32 g_value;
static int g_page_size;
static int g_block_size;
static u32 PAGES_PER_BLOCK = 255;
static bool g_bSyncOrToggle;
static int g_iNFI2X_CLKSRC;

bool g_b2Die_CS = FALSE;	/* for nand base */
static bool g_bTricky_CS = FALSE;
static u32 g_nanddie_pages;

unsigned char g_bHwEcc = true;
#define LPAGE 16384
#define LSPARE 2048

static u8 *local_buffer_16_align;	/* 16 byte aligned buffer, for HW issue */
__aligned(64)
static u8 local_buffer[LPAGE + LSPARE];
static u8 *temp_buffer_16_align;	/* 16 byte aligned buffer, for HW issue */
__aligned(64)
static u8 temp_buffer[LPAGE + LSPARE];

static int mtk_nand_cs_check(struct mtd_info *mtd, u8 *id, u16 cs);
static u32 mtk_nand_cs_on(struct nand_chip *nand_chip, u16 cs, u32 page);

struct mtk_nand_host *host;
static u8 g_running_dma;

int manu_id;
int dev_id;

static u8 local_oob_buf[LSPARE];

int dummy_driver_debug;

flashdev_info_t devinfo;

enum NAND_TYPE_MASK {
	TYPE_ASYNC = 0x0,
	TYPE_TOGGLE = 0x1,
	TYPE_SYNC = 0x2,
	TYPE_RESERVED = 0x3,
	TYPE_MLC = 0x4,		/* 1b0 */
	TYPE_SLC = 0x4,		/* 1b1 */
};


typedef u32(*GetLowPageNumber) (u32 pageNo);
typedef u32(*TransferPageNumber) (u32 pageNo, bool high_to_low);

u32 SANDISK_TRANSFER(u32 pageNo)
{
	if (pageNo == 0)
		return pageNo;
	else
		return pageNo + pageNo - 1;
}

u32 HYNIX_TRANSFER(u32 pageNo)
{
	u32 temp;

	if (pageNo < 4)
		return pageNo;
	temp = pageNo + (pageNo & 0xFFFFFFFE) - 2;
	return temp;
}


u32 MICRON_TRANSFER(u32 pageNo)
{
	u32 temp;

	if (pageNo < 4)
		return pageNo;
	temp = (pageNo - 4) & 0xFFFFFFFE;
	if (pageNo <= 130)
		return (pageNo + temp);
	else
		return (pageNo + temp - 2);
}

u32 sandisk_pairpage_mapping(u32 page, bool high_to_low)
{
	if (high_to_low == TRUE) {
		if (page == 255)
			return page - 2;
		if ((page == 0) || ((page % 2) == 1))
			return page;
		if (page == 2)
			return 0;
		else
			return (page - 3);
	} else {
		if ((page != 0) && ((page % 2) == 0))
			return page;
		if (page == 255)
			return page;
		if (page == 0 || page == 253)
			return page + 2;
		else
			return page + 3;
	}
}

u32 hynix_pairpage_mapping(u32 page, bool high_to_low)
{
	u32 offset;

	if (high_to_low == TRUE) {
		/* Micron 256pages */
		if (page < 4)
			return page;

		offset = page % 4;
		if (offset == 2 || offset == 3)
			return page;

		if (page == 4 || page == 5 || page == 254 || page == 255)
			return page - 4;
		else
			return page - 6;
	} else {
		if (page > 251)
			return page;
		if (page == 0 || page == 1)
			return page + 4;
		offset = page % 4;
		if (offset == 0 || offset == 1)
			return page;
		else
			return page + 6;
	}
}

u32 micron_pairpage_mapping(u32 page, bool high_to_low)
{
	u32 offset;

	if (high_to_low == TRUE) {
		/* Micron 256pages */
		if ((page < 4) || (page > 251))
			return page;

		offset = page % 4;
		if (offset == 0 || offset == 1)
			return page;
		else
			return page - 6;
	} else {
		if ((page == 2) || (page == 3) || (page > 247))
			return page;
		offset = page % 4;
		if (offset == 0 || offset == 1)
			return page + 6;
		else
			return page;
	}
}

GetLowPageNumber functArray[] = {
	MICRON_TRANSFER,
	HYNIX_TRANSFER,
	SANDISK_TRANSFER,
};

TransferPageNumber fsFuncArray[] = {
	micron_pairpage_mapping,
	hynix_pairpage_mapping,
	sandisk_pairpage_mapping,
};

int mtk_nand_paired_page_transfer(u32 pageNo, bool high_to_low)
{
	if (devinfo.vendor != VEND_NONE)
		return fsFuncArray[devinfo.feature_set.ptbl_idx] (pageNo, high_to_low);
	else
		return 0xFFFFFFFF;
}

#define PWR_DOWN 0
#define PWR_ON	 1
void nand_prepare_clock(void)
{
	clk_prepare(nfi_hclk);
	clk_prepare(nfiecc_bclk);
	clk_prepare(nfi_bclk);
	if (mtk_nfi_dev_comp->chip_ver == 2) {
		clk_prepare(nfi_pclk);
		clk_prepare(nfi_ecc_pclk);
	}
}

void nand_unprepare_clock(void)
{
	clk_unprepare(nfi_hclk);
	clk_unprepare(nfiecc_bclk);
	clk_unprepare(nfi_bclk);
	if (mtk_nfi_dev_comp->chip_ver == 2) {
		clk_unprepare(nfi_pclk);
		clk_unprepare(nfi_ecc_pclk);
	}
}

void nand_enable_clock(void)
{
	clk_enable(nfi_hclk);
	clk_enable(nfiecc_bclk);
	clk_enable(nfi_bclk);
	if (mtk_nfi_dev_comp->chip_ver == 2) {
		clk_enable(nfi_pclk);
		clk_enable(nfi_ecc_pclk);
	}
}

void nand_disable_clock(void)
{
	clk_disable(nfi_hclk);
	clk_disable(nfiecc_bclk);
	clk_disable(nfi_bclk);
	if (mtk_nfi_dev_comp->chip_ver == 2) {
		clk_disable(nfi_pclk);
		clk_disable(nfi_ecc_pclk);
	}
}

static struct nand_ecclayout nand_oob_16 = {
	.eccbytes = 8,
	.eccpos = {8, 9, 10, 11, 12, 13, 14, 15},
	.oobfree = {{1, 6}, {0, 0} }
};

struct nand_ecclayout nand_oob_64 = {
	.eccbytes = 32,
	.eccpos = {32, 33, 34, 35, 36, 37, 38, 39,
		   40, 41, 42, 43, 44, 45, 46, 47,
		   48, 49, 50, 51, 52, 53, 54, 55,
		   56, 57, 58, 59, 60, 61, 62, 63},
	.oobfree = {{1, 7}, {9, 7}, {17, 7}, {25, 6}, {0, 0} }
};

struct nand_ecclayout nand_oob_128 = {
	.eccbytes = 64,
	.eccpos = {
		   64, 65, 66, 67, 68, 69, 70, 71,
		   72, 73, 74, 75, 76, 77, 78, 79,
		   80, 81, 82, 83, 84, 85, 86, 86,
		   88, 89, 90, 91, 92, 93, 94, 95,
		   96, 97, 98, 99, 100, 101, 102, 103,
		   104, 105, 106, 107, 108, 109, 110, 111,
		   112, 113, 114, 115, 116, 117, 118, 119,
		   120, 121, 122, 123, 124, 125, 126, 127},
	.oobfree = {{1, 7}, {9, 7}, {17, 7}, {25, 7}, {33, 7}, {41, 7}, {49, 7}, {57, 6} }
};

/**************************************************************************
*  Randomizer
**************************************************************************/
#define SS_SEED_NUM 128
#define EFUSE_RANDOM_ENABLE 0x00000004
static bool use_randomizer = FALSE;
static bool pre_randomizer = FALSE;

static unsigned short SS_RANDOM_SEED[SS_SEED_NUM] = {
	/* for page 0~127 */
	0x576A, 0x05E8, 0x629D, 0x45A3, 0x649C, 0x4BF0, 0x2342, 0x272E,
	0x7358, 0x4FF3, 0x73EC, 0x5F70, 0x7A60, 0x1AD8, 0x3472, 0x3612,
	0x224F, 0x0454, 0x030E, 0x70A5, 0x7809, 0x2521, 0x484F, 0x5A2D,
	0x492A, 0x043D, 0x7F61, 0x3969, 0x517A, 0x3B42, 0x769D, 0x0647,
	0x7E2A, 0x1383, 0x49D9, 0x07B8, 0x2578, 0x4EEC, 0x4423, 0x352F,
	0x5B22, 0x72B9, 0x367B, 0x24B6, 0x7E8E, 0x2318, 0x6BD0, 0x5519,
	0x1783, 0x18A7, 0x7B6E, 0x7602, 0x4B7F, 0x3648, 0x2C53, 0x6B99,
	0x0C23, 0x67CF, 0x7E0E, 0x4D8C, 0x5079, 0x209D, 0x244A, 0x747B,
	0x350B, 0x0E4D, 0x7004, 0x6AC3, 0x7F3E, 0x21F5, 0x7A15, 0x2379,
	0x1517, 0x1ABA, 0x4E77, 0x15A1, 0x04FA, 0x2D61, 0x253A, 0x1302,
	0x1F63, 0x5AB3, 0x049A, 0x5AE8, 0x1CD7, 0x4A00, 0x30C8, 0x3247,
	0x729C, 0x5034, 0x2B0E, 0x57F2, 0x00E4, 0x575B, 0x6192, 0x38F8,
	0x2F6A, 0x0C14, 0x45FC, 0x41DF, 0x38DA, 0x7AE1, 0x7322, 0x62DF,
	0x5E39, 0x0E64, 0x6D85, 0x5951, 0x5937, 0x6281, 0x33A1, 0x6A32,
	0x3A5A, 0x2BAC, 0x743A, 0x5E74, 0x3B2E, 0x7EC7, 0x4FD2, 0x5D28,
	0x751F, 0x3EF8, 0x39B1, 0x4E49, 0x746B, 0x6EF6, 0x44BE, 0x6DB7
};



void dump_nfi(void)
{
	pr_debug("~~~~Dump NFI Register in Kernel~~~~\n");
	pr_debug("NFI_CNFG_REG16: 0x%x\n", DRV_Reg16(NFI_CNFG_REG16));
	if (mtk_nfi_dev_comp->chip_ver == 1)
		pr_debug("NFI_PAGEFMT_REG16: 0x%x\n", DRV_Reg32(NFI_PAGEFMT_REG16));
	else if (mtk_nfi_dev_comp->chip_ver == 2)
		pr_debug("NFI_PAGEFMT_REG32: 0x%x\n", DRV_Reg32(NFI_PAGEFMT_REG32));
	pr_debug("NFI_CON_REG16: 0x%x\n", DRV_Reg16(NFI_CON_REG16));
	pr_debug("NFI_ACCCON_REG32: 0x%x\n", DRV_Reg32(NFI_ACCCON_REG32));
	pr_debug("NFI_INTR_EN_REG16: 0x%x\n", DRV_Reg16(NFI_INTR_EN_REG16));
	pr_debug("NFI_INTR_REG16: 0x%x\n", DRV_Reg16(NFI_INTR_REG16));
	pr_debug("NFI_CMD_REG16: 0x%x\n", DRV_Reg16(NFI_CMD_REG16));
	pr_debug("NFI_ADDRNOB_REG16: 0x%x\n", DRV_Reg16(NFI_ADDRNOB_REG16));
	pr_debug("NFI_COLADDR_REG32: 0x%x\n", DRV_Reg32(NFI_COLADDR_REG32));
	pr_debug("NFI_ROWADDR_REG32: 0x%x\n", DRV_Reg32(NFI_ROWADDR_REG32));
	pr_debug("NFI_STRDATA_REG16: 0x%x\n", DRV_Reg16(NFI_STRDATA_REG16));
	pr_debug("NFI_DATAW_REG32: 0x%x\n", DRV_Reg32(NFI_DATAW_REG32));
	pr_debug("NFI_DATAR_REG32: 0x%x\n", DRV_Reg32(NFI_DATAR_REG32));
	pr_debug("NFI_PIO_DIRDY_REG16: 0x%x\n", DRV_Reg16(NFI_PIO_DIRDY_REG16));
	pr_debug("NFI_STA_REG32: 0x%x\n", DRV_Reg32(NFI_STA_REG32));
	pr_debug("NFI_FIFOSTA_REG16: 0x%x\n", DRV_Reg16(NFI_FIFOSTA_REG16));
	pr_debug("NFI_ADDRCNTR_REG16: 0x%x\n", DRV_Reg16(NFI_ADDRCNTR_REG16));
	pr_debug("NFI_STRADDR_REG32: 0x%x\n", DRV_Reg32(NFI_STRADDR_REG32));
	pr_debug("NFI_BYTELEN_REG16: 0x%x\n", DRV_Reg16(NFI_BYTELEN_REG16));
	pr_debug("NFI_CSEL_REG16: 0x%x\n", DRV_Reg16(NFI_CSEL_REG16));
	pr_debug("NFI_IOCON_REG16: 0x%x\n", DRV_Reg16(NFI_IOCON_REG16));
	pr_debug("NFI_FDM0L_REG32: 0x%x\n", DRV_Reg32(NFI_FDM0L_REG32));
	pr_debug("NFI_FDM0M_REG32: 0x%x\n", DRV_Reg32(NFI_FDM0M_REG32));
	pr_debug("NFI_LOCK_REG16: 0x%x\n", DRV_Reg16(NFI_LOCK_REG16));
	pr_debug("NFI_LOCKCON_REG32: 0x%x\n", DRV_Reg32(NFI_LOCKCON_REG32));
	pr_debug("NFI_LOCKANOB_REG16: 0x%x\n", DRV_Reg16(NFI_LOCKANOB_REG16));
	pr_debug("NFI_FIFODATA0_REG32: 0x%x\n", DRV_Reg32(NFI_FIFODATA0_REG32));
	pr_debug("NFI_FIFODATA1_REG32: 0x%x\n", DRV_Reg32(NFI_FIFODATA1_REG32));
	pr_debug("NFI_FIFODATA2_REG32: 0x%x\n", DRV_Reg32(NFI_FIFODATA2_REG32));
	pr_debug("NFI_FIFODATA3_REG32: 0x%x\n", DRV_Reg32(NFI_FIFODATA3_REG32));
	pr_debug("NFI_MASTERSTA_REG16: 0x%x\n", DRV_Reg16(NFI_MASTERSTA_REG16));
	pr_debug("NFI_DEBUG_CON1_REG16: 0x%x\n", DRV_Reg16(NFI_DEBUG_CON1_REG16));
	pr_debug("ECC_ENCCON_REG16	  :%x\n", *ECC_ENCCON_REG16);
	pr_debug("ECC_ENCCNFG_REG32	:%x\n", *ECC_ENCCNFG_REG32);
	pr_debug("ECC_ENCDIADDR_REG32	:%x\n", *ECC_ENCDIADDR_REG32);
	pr_debug("ECC_ENCIDLE_REG32	:%x\n", *ECC_ENCIDLE_REG32);
	pr_debug("ECC_ENCPAR0_REG32	:%x\n", *ECC_ENCPAR0_REG32);
	pr_debug("ECC_ENCPAR1_REG32	:%x\n", *ECC_ENCPAR1_REG32);
	pr_debug("ECC_ENCPAR2_REG32	:%x\n", *ECC_ENCPAR2_REG32);
	pr_debug("ECC_ENCPAR3_REG32	:%x\n", *ECC_ENCPAR3_REG32);
	pr_debug("ECC_ENCPAR4_REG32	:%x\n", *ECC_ENCPAR4_REG32);
	pr_debug("ECC_ENCPAR5_REG32	:%x\n", *ECC_ENCPAR5_REG32);
	pr_debug("ECC_ENCPAR6_REG32	:%x\n", *ECC_ENCPAR6_REG32);
	pr_debug("ECC_ENCSTA_REG32	:%x\n", *ECC_ENCSTA_REG32);
	pr_debug("ECC_ENCIRQEN_REG16	:%x\n", *ECC_ENCIRQEN_REG16);
	pr_debug("ECC_ENCIRQSTA_REG16 :%x\n", *ECC_ENCIRQSTA_REG16);
	pr_debug("ECC_DECCON_REG16	:%x\n", *ECC_DECCON_REG16);
	pr_debug("ECC_DECCNFG_REG32	:%x\n", *ECC_DECCNFG_REG32);
	pr_debug("ECC_DECDIADDR_REG32 :%x\n", *ECC_DECDIADDR_REG32);
	pr_debug("ECC_DECIDLE_REG16	:%x\n", *ECC_DECIDLE_REG16);
	pr_debug("ECC_DECFER_REG16	:%x\n", *ECC_DECFER_REG16);
	pr_debug("ECC_DECENUM0_REG32	:%x\n", *ECC_DECENUM0_REG32);
	pr_debug("ECC_DECENUM1_REG32	:%x\n", *ECC_DECENUM1_REG32);
	pr_debug("ECC_DECDONE_REG16	:%x\n", *ECC_DECDONE_REG16);
	pr_debug("ECC_DECEL0_REG32	:%x\n", *ECC_DECEL0_REG32);
	pr_debug("ECC_DECEL1_REG32	:%x\n", *ECC_DECEL1_REG32);
	pr_debug("ECC_DECEL2_REG32	:%x\n", *ECC_DECEL2_REG32);
	pr_debug("ECC_DECEL3_REG32	:%x\n", *ECC_DECEL3_REG32);
	pr_debug("ECC_DECEL4_REG32	:%x\n", *ECC_DECEL4_REG32);
	pr_debug("ECC_DECEL5_REG32	:%x\n", *ECC_DECEL5_REG32);
	pr_debug("ECC_DECEL6_REG32	:%x\n", *ECC_DECEL6_REG32);
	pr_debug("ECC_DECEL7_REG32	:%x\n", *ECC_DECEL7_REG32);
	pr_debug("ECC_DECIRQEN_REG16	:%x\n", *ECC_DECIRQEN_REG16);
	pr_debug("ECC_DECIRQSTA_REG16 :%x\n", *ECC_DECIRQSTA_REG16);
	pr_debug("ECC_DECFSM_REG32	:%x\n", *ECC_DECFSM_REG32);
	pr_debug("ECC_BYPASS_REG32	:%x\n", *ECC_BYPASS_REG32);
}

u8 NFI_DMA_status(void)
{
	return g_running_dma;
}
EXPORT_SYMBOL(NFI_DMA_status);

u32 NFI_DMA_address(void)
{
	return DRV_Reg32(NFI_STRADDR_REG32);
}
EXPORT_SYMBOL(NFI_DMA_address);

unsigned long nand_virt_to_phys_add(unsigned long va)
{}
EXPORT_SYMBOL(nand_virt_to_phys_add);

bool get_device_info(u8 *id, flashdev_info_t *devinfo)
{
	u32 i, m, n, mismatch;
	int target = -1;
	u8 target_id_len = 0;

	for (i = 0; i < flash_number; i++) {
		mismatch = 0;
		for (m = 0; m < gen_FlashTable_p[i].id_length; m++) {
			if (id[m] != gen_FlashTable_p[i].id[m]) {
				mismatch = 1;
				break;
			}
		}
		if (mismatch == 0 && gen_FlashTable_p[i].id_length > target_id_len) {
			target = i;
			target_id_len = gen_FlashTable_p[i].id_length;
		}
	}

	if (target != -1) {
		pr_debug("Recognize NAND: ID [");
		for (n = 0; n < gen_FlashTable_p[target].id_length; n++) {
			devinfo->id[n] = gen_FlashTable_p[target].id[n];
			pr_debug("%x ", devinfo->id[n]);
		}
		pr_debug("], Device Name [%s], Page Size [%d]B Spare Size [%d]B Total Size [%d]MB\n",
			gen_FlashTable_p[target].devciename, gen_FlashTable_p[target].pagesize,
			gen_FlashTable_p[target].sparesize, gen_FlashTable_p[target].totalsize);
		devinfo->id_length = gen_FlashTable_p[target].id_length;
		devinfo->blocksize = gen_FlashTable_p[target].blocksize;
		devinfo->addr_cycle = gen_FlashTable_p[target].addr_cycle;
		devinfo->iowidth = gen_FlashTable_p[target].iowidth;
		devinfo->timmingsetting = gen_FlashTable_p[target].timmingsetting;
		devinfo->advancedmode = gen_FlashTable_p[target].advancedmode;
		devinfo->pagesize = gen_FlashTable_p[target].pagesize;
		devinfo->sparesize = gen_FlashTable_p[target].sparesize;
		devinfo->totalsize = gen_FlashTable_p[target].totalsize;
		devinfo->sectorsize = gen_FlashTable_p[target].sectorsize;
		devinfo->s_acccon = gen_FlashTable_p[target].s_acccon;
		devinfo->s_acccon1 = gen_FlashTable_p[target].s_acccon1;
		devinfo->freq = gen_FlashTable_p[target].freq;
		devinfo->vendor = gen_FlashTable_p[target].vendor;
		/* devinfo->ttarget = gen_FlashTable[target].ttarget; */
		memcpy((u8 *) &devinfo->feature_set, (u8 *) &gen_FlashTable_p[target].feature_set,
			   sizeof(struct MLC_feature_set));
		memcpy(devinfo->devciename, gen_FlashTable_p[target].devciename,
			   sizeof(devinfo->devciename));
		return true;
	}
	pr_err("Not Found NAND: ID [");
	for (n = 0; n < NAND_MAX_ID; n++)
		pr_err("%x ", id[n]);
	pr_err("]\n");
	return false;
}

static bool mtk_nand_reset(void);

u32 mtk_nand_page_transform(struct mtd_info *mtd, struct nand_chip *chip, u32 page, u32 *blk,
				u32 *map_blk)
{
	u32 block_size = 1 << (chip->phys_erase_shift);
	u32 page_size = (1 << chip->page_shift);
	loff_t start_address;
	u32 idx;
	u32 block;
	u32 page_in_block;
	u32 mapped_block;
	bool translate = FALSE;
	loff_t logical_address = (loff_t) page * (1 << chip->page_shift);

	block = page / (block_size / page_size);
	mapped_block = block;
	page_in_block = page % (block_size / page_size);
	*blk = block;
	*map_blk = mapped_block;
	return page_in_block;
}

bool mtk_nand_IsRawPartition(loff_t logical_address)
{
	return false;
}

static int mtk_nand_interface_config(struct mtd_info *mtd)
{
	u32 timeout;
	u32 val;
	u32 acccon1;
	struct gFeatureSet *feature_set = &(devinfo.feature_set.FeatureSet);

	g_bSyncOrToggle = false;
	pr_notice("[%s] legacy interface\n", __func__);
	return 0;
}

static int mtk_nand_turn_on_randomizer(u32 page, int type, int fgPage)
{
	u32 u4NFI_CFG = 0;
	u32 u4NFI_RAN_CFG = 0;

	u4NFI_CFG = DRV_Reg32(NFI_CNFG_REG16);

	DRV_WriteReg32(NFI_ENMPTY_THRESH_REG32, 40);	/* empty threshold 40 */

	if (type) {		/* encode */
		DRV_WriteReg32(NFI_RANDOM_ENSEED01_TS_REG32, 0);
		DRV_WriteReg32(NFI_RANDOM_ENSEED02_TS_REG32, 0);
		DRV_WriteReg32(NFI_RANDOM_ENSEED03_TS_REG32, 0);
		DRV_WriteReg32(NFI_RANDOM_ENSEED04_TS_REG32, 0);
		DRV_WriteReg32(NFI_RANDOM_ENSEED05_TS_REG32, 0);
		DRV_WriteReg32(NFI_RANDOM_ENSEED06_TS_REG32, 0);
	} else {
		DRV_WriteReg32(NFI_RANDOM_DESEED01_TS_REG32, 0);
		DRV_WriteReg32(NFI_RANDOM_DESEED02_TS_REG32, 0);
		DRV_WriteReg32(NFI_RANDOM_DESEED03_TS_REG32, 0);
		DRV_WriteReg32(NFI_RANDOM_DESEED04_TS_REG32, 0);
		DRV_WriteReg32(NFI_RANDOM_DESEED05_TS_REG32, 0);
		DRV_WriteReg32(NFI_RANDOM_DESEED06_TS_REG32, 0);
	}
	u4NFI_CFG |= CNFG_RAN_SEL;
	if (PAGES_PER_BLOCK <= SS_SEED_NUM) {
		if (type) {
			u4NFI_RAN_CFG |=
				RAN_CNFG_ENCODE_SEED(SS_RANDOM_SEED[page & (PAGES_PER_BLOCK - 1)]) |
				RAN_CNFG_ENCODE_EN;
		} else {
			u4NFI_RAN_CFG |=
				RAN_CNFG_DECODE_SEED(SS_RANDOM_SEED[page & (PAGES_PER_BLOCK - 1)]) |
				RAN_CNFG_DECODE_EN;
		}
	} else {
		if (type) {
			u4NFI_RAN_CFG |=
				RAN_CNFG_ENCODE_SEED(SS_RANDOM_SEED[page & (SS_SEED_NUM - 1)]) |
				RAN_CNFG_ENCODE_EN;
		} else {
			u4NFI_RAN_CFG |=
				RAN_CNFG_DECODE_SEED(SS_RANDOM_SEED[page & (SS_SEED_NUM - 1)]) |
				RAN_CNFG_DECODE_EN;
		}
	}


	if (fgPage)		/* reload seed for each page */
		u4NFI_CFG &= ~CNFG_RAN_SEC;
	else			/* reload seed for each sector */
		u4NFI_CFG |= CNFG_RAN_SEC;

	DRV_WriteReg32(NFI_CNFG_REG16, u4NFI_CFG);
	DRV_WriteReg32(NFI_RANDOM_CNFG_REG32, u4NFI_RAN_CFG);
	/* MSG(INIT, "[K]ran turn on type:%d 0x%x 0x%x\n", type, DRV_Reg32(NFI_RANDOM_CNFG_REG32), page); */
	return 0;
}

static bool mtk_nand_israndomizeron(void)
{
	u32 nfi_ran_cnfg = 0;

	nfi_ran_cnfg = DRV_Reg32(NFI_RANDOM_CNFG_REG32);
	if (nfi_ran_cnfg & (RAN_CNFG_ENCODE_EN | RAN_CNFG_DECODE_EN))
		return TRUE;

	return FALSE;
}

static void mtk_nand_turn_off_randomizer(void)
{
	u32 u4NFI_CFG = DRV_Reg32(NFI_CNFG_REG16);

	u4NFI_CFG &= ~CNFG_RAN_SEL;
	u4NFI_CFG &= ~CNFG_RAN_SEC;
	DRV_WriteReg32(NFI_RANDOM_CNFG_REG32, 0);
	DRV_WriteReg32(NFI_CNFG_REG16, u4NFI_CFG);
	/* MSG(INIT, "[K]ran turn off\n"); */
}

/******************************************************************************
 * mtk_nand_irq_handler
 *
 * DESCRIPTION:
 *	 NAND interrupt handler!
 *
 * PARAMETERS:
 *	 int irq
 *	 void *dev_id
 *
 * RETURNS:
 *	 IRQ_HANDLED : Successfully handle the IRQ
 *
 * NOTES:
 *	 None
 *
 ******************************************************************************/
/* Modified for TCM used */

static irqreturn_t mtk_nand_irq_handler(int irqno, void *dev_id)
{
	u16 u16IntStatus = DRV_Reg16(NFI_INTR_REG16);
	(void)irqno;

	if (u16IntStatus & (u16) INTR_AHB_DONE_EN)
		complete(&g_comp_AHB_Done);
	return IRQ_HANDLED;
}

/******************************************************************************
 * ECC_Config
 *
 * DESCRIPTION:
 *	 Configure HW ECC!
 *
 * PARAMETERS:
 *	 struct mtk_nand_host_hw *hw
 *
 * RETURNS:
 *	 None
 *
 * NOTES:
 *	 None
 *
 ******************************************************************************/
static void ECC_Config(struct mtk_nand_host_hw *hw, u32 ecc_bit)
{
	u32 u4ENCODESize;
	u32 u4DECODESize;
	u32 ecc_bit_cfg = ECC_CNFG_ECC4;

	switch (ecc_bit) {
	case 4:
		ecc_bit_cfg = ECC_CNFG_ECC4;
		break;
	case 8:
		ecc_bit_cfg = ECC_CNFG_ECC8;
		break;
	case 10:
		ecc_bit_cfg = ECC_CNFG_ECC10;
		break;
	case 12:
		ecc_bit_cfg = ECC_CNFG_ECC12;
		break;
	case 14:
		ecc_bit_cfg = ECC_CNFG_ECC14;
		break;
	case 16:
		ecc_bit_cfg = ECC_CNFG_ECC16;
		break;
	case 18:
		ecc_bit_cfg = ECC_CNFG_ECC18;
		break;
	case 20:
		ecc_bit_cfg = ECC_CNFG_ECC20;
		break;
	case 22:
		ecc_bit_cfg = ECC_CNFG_ECC22;
		break;
	case 24:
		ecc_bit_cfg = ECC_CNFG_ECC24;
		break;
	case 28:
		ecc_bit_cfg = ECC_CNFG_ECC28;
		break;
	case 32:
		ecc_bit_cfg = ECC_CNFG_ECC32;
		break;
	case 36:
		ecc_bit_cfg = ECC_CNFG_ECC36;
		break;
	case 40:
		ecc_bit_cfg = ECC_CNFG_ECC40;
		break;
	case 44:
		ecc_bit_cfg = ECC_CNFG_ECC44;
		break;
	case 48:
		ecc_bit_cfg = ECC_CNFG_ECC48;
		break;
	case 52:
		ecc_bit_cfg = ECC_CNFG_ECC52;
		break;
	case 56:
		ecc_bit_cfg = ECC_CNFG_ECC56;
		break;
	case 60:
		ecc_bit_cfg = ECC_CNFG_ECC60;
		break;
	default:
		break;

	}
	DRV_WriteReg16(ECC_DECCON_REG16, DEC_DE);
	do {
		;
	} while (!DRV_Reg16(ECC_DECIDLE_REG16));

	DRV_WriteReg16(ECC_ENCCON_REG16, ENC_DE);
	do {
		;
	} while (!DRV_Reg32(ECC_ENCIDLE_REG32));

	/* setup FDM register base */
	/* DRV_WriteReg32(ECC_FDMADDR_REG32, NFI_FDM0L_REG32); */

	/* Sector + FDM */
	u4ENCODESize = (hw->nand_sec_size + 8) << 3;
	/* Sector + FDM + YAFFS2 meta data bits */
	u4DECODESize = ((hw->nand_sec_size + 8) << 3) + ecc_bit * ECC_PARITY_BIT;

	/* configure ECC decoder && encoder */
	DRV_WriteReg32(ECC_DECCNFG_REG32,
			   ecc_bit_cfg | DEC_CNFG_NFI | DEC_CNFG_EMPTY_EN | (u4DECODESize <<
									 DEC_CNFG_CODE_SHIFT));

	DRV_WriteReg32(ECC_ENCCNFG_REG32,
			   ecc_bit_cfg | ENC_CNFG_NFI | (u4ENCODESize << ENC_CNFG_MSG_SHIFT));
	NFI_SET_REG32(ECC_DECCNFG_REG32, DEC_CNFG_CORRECT);
}

/******************************************************************************
 * ECC_Decode_Start
 *
 * DESCRIPTION:
 *	 HW ECC Decode Start !
 *
 * PARAMETERS:
 *	 None
 *
 * RETURNS:
 *	 None
 *
 * NOTES:
 *	 None
 *
 ******************************************************************************/
static void ECC_Decode_Start(void)
{
	/* wait for device returning idle */
	while (!(DRV_Reg16(ECC_DECIDLE_REG16) & DEC_IDLE))
		;
	DRV_WriteReg16(ECC_DECCON_REG16, DEC_EN);
}

/******************************************************************************
 * ECC_Decode_End
 *
 * DESCRIPTION:
 *	 HW ECC Decode End !
 *
 * PARAMETERS:
 *	 None
 *
 * RETURNS:
 *	 None
 *
 * NOTES:
 *	 None
 *
 ******************************************************************************/
static void ECC_Decode_End(void)
{
	/* wait for device returning idle */
	while (!(DRV_Reg16(ECC_DECIDLE_REG16) & DEC_IDLE))
		;
	DRV_WriteReg16(ECC_DECCON_REG16, DEC_DE);
}

/******************************************************************************
 * ECC_Encode_Start
 *
 * DESCRIPTION:
 *	 HW ECC Encode Start !
 *
 * PARAMETERS:
 *	 None
 *
 * RETURNS:
 *	 None
 *
 * NOTES:
 *	 None
 *
 ******************************************************************************/
static void ECC_Encode_Start(void)
{
	/* wait for device returning idle */
	while (!(DRV_Reg32(ECC_ENCIDLE_REG32) & ENC_IDLE))
		;
	/* trigger Encoder after ECC Engine idle */
	mb();
	DRV_WriteReg16(ECC_ENCCON_REG16, ENC_EN);
}

/******************************************************************************
 * ECC_Encode_End
 *
 * DESCRIPTION:
 *	 HW ECC Encode End !
 *
 * PARAMETERS:
 *	 None
 *
 * RETURNS:
 *	 None
 *
 * NOTES:
 *	 None
 *
 ******************************************************************************/
static void ECC_Encode_End(void)
{
	/* wait for device returning idle */
	while (!(DRV_Reg32(ECC_ENCIDLE_REG32) & ENC_IDLE))
		;
	/* trigger Encoder after ECC Engine idle */
	mb();
	DRV_WriteReg16(ECC_ENCCON_REG16, ENC_DE);
}

/******************************************************************************
 * mtk_nand_check_bch_error
 *
 * DESCRIPTION:
 *	 Check BCH error or not !
 *
 * PARAMETERS:
 *	 struct mtd_info *mtd
 *	 u8* pDataBuf
 *	 u32 u4SecIndex
 *	 u32 u4PageAddr
 *
 * RETURNS:
 *	 None
 *
 * NOTES:
 *	 None
 *
 ******************************************************************************/
static bool mtk_nand_check_bch_error(struct mtd_info *mtd, u8 *pDataBuf, u8 *spareBuf,
					 u32 u4SecIndex, u32 u4PageAddr, u32 *bitmap)
{
	bool ret = true;
	u16 u2SectorDoneMask = 1 << u4SecIndex;
	u32 u4ErrorNumDebug0, u4ErrorNumDebug1, i, u4ErrNum;
	u32 timeout = 0xFFFF;
	u32 correct_count = 0;
	u32 page_size = (u4SecIndex + 1) * host->hw->nand_sec_size;
	u32 sec_num = u4SecIndex + 1;
	/* u32 bitflips = sec_num * 39; */
	u16 failed_sec = 0;
	u32 maxSectorBitErr = 0;
	u32 ERR_NUM0 = 0;

	if (mtk_nfi_dev_comp->chip_ver == 1)
		ERR_NUM0 = ERR_NUM0_V1;
	else if (mtk_nfi_dev_comp->chip_ver == 2)
		ERR_NUM0 = ERR_NUM0_V2;

	while (0 == (u2SectorDoneMask & DRV_Reg16(ECC_DECDONE_REG16))) {
		timeout--;
		if (timeout == 0)
			return false;
	}

	if (0 == (DRV_Reg32(NFI_STA_REG32) & STA_READ_EMPTY)) {
		u4ErrorNumDebug0 = DRV_Reg32(ECC_DECENUM0_REG32);
		u4ErrorNumDebug1 = DRV_Reg32(ECC_DECENUM1_REG32);
		if (0 != (u4ErrorNumDebug0 & 0xFFFFFFFF) || 0 != (u4ErrorNumDebug1 & 0xFFFFFFFF)) {
			for (i = 0; i <= u4SecIndex; ++i) {
				u4ErrNum = (DRV_Reg32((ECC_DECENUM0_REG32 + (i / 4))) >> ((i % 4) * 8)) & ERR_NUM0;

				if (u4ErrNum == ERR_NUM0) {
					failed_sec++;
					ret = false;
					pr_debug("UnCorrectable ECC errors at PageAddr=%d, Sector=%d\n", u4PageAddr, i);
					continue;
				}
				if (bitmap)
					*bitmap |= 1 << i;
				if (u4ErrNum) {
					if (maxSectorBitErr < u4ErrNum)
						maxSectorBitErr = u4ErrNum;
					correct_count += u4ErrNum;
				}
			}
			mtd->ecc_stats.failed += failed_sec;
			if ((maxSectorBitErr > ecc_threshold) && (ret != FALSE)) {
				pr_debug("ECC bit flips (0x%x) exceed eccthreshold (0x%x),u4PageAddr 0x%x\n",
					maxSectorBitErr, ecc_threshold, u4PageAddr);
				mtd->ecc_stats.corrected++;
			}
		}
	}

	if (0 != (DRV_Reg32(NFI_STA_REG32) & STA_READ_EMPTY)) {
		ret = true;
		/* MSG(INIT, "empty page, empty buffer returned\n"); */
		memset(pDataBuf, 0xff, page_size);
		memset(spareBuf, 0xff, sec_num * 8);
		maxSectorBitErr = 0;
		failed_sec = 0;
	}
	return ret;
}

/******************************************************************************
 * mtk_nand_RFIFOValidSize
 *
 * DESCRIPTION:
 *	 Check the Read FIFO data bytes !
 *
 * PARAMETERS:
 *	 u16 u2Size
 *
 * RETURNS:
 *	 None
 *
 * NOTES:
 *	 None
 *
 ******************************************************************************/
static bool mtk_nand_RFIFOValidSize(u16 u2Size)
{
	u32 timeout = 0xFFFF;

	while (FIFO_RD_REMAIN(DRV_Reg16(NFI_FIFOSTA_REG16)) < u2Size) {
		timeout--;
		if (timeout == 0)
			return false;
	}
	return true;
}

/******************************************************************************
 * mtk_nand_WFIFOValidSize
 *
 * DESCRIPTION:
 *	 Check the Write FIFO data bytes !
 *
 * PARAMETERS:
 *	 u16 u2Size
 *
 * RETURNS:
 *	 None
 *
 * NOTES:
 *	 None
 *
 ******************************************************************************/
static bool mtk_nand_WFIFOValidSize(u16 u2Size)
{
	u32 timeout = 0xFFFF;

	while (FIFO_WR_REMAIN(DRV_Reg16(NFI_FIFOSTA_REG16)) > u2Size) {
		timeout--;
		if (timeout == 0)
			return false;
	}
	return true;
}

/******************************************************************************
 * mtk_nand_status_ready
 *
 * DESCRIPTION:
 *	 Indicate the NAND device is ready or not !
 *
 * PARAMETERS:
 *	 u32 u4Status
 *
 * RETURNS:
 *	 None
 *
 * NOTES:
 *	 None
 *
 ******************************************************************************/
static bool mtk_nand_status_ready(u32 u4Status)
{
	u32 timeout = 0xFFFF;

	while ((DRV_Reg32(NFI_STA_REG32) & u4Status) != 0) {
		timeout--;
		if (timeout == 0)
			return false;
	}
	return true;
}

/******************************************************************************
 * mtk_nand_reset
 *
 * DESCRIPTION:
 *	 Reset the NAND device hardware component !
 *
 * PARAMETERS:
 *	 struct mtk_nand_host *host (Initial setting data)
 *
 * RETURNS:
 *	 None
 *
 * NOTES:
 *	 None
 *
 ******************************************************************************/
static bool mtk_nand_reset(void)
{
	/* HW recommended reset flow */
	int timeout = 0xFFFF;

	if (DRV_Reg16(NFI_MASTERSTA_REG16) & 0xFFF) {	/* master is busy */
		mb();
		DRV_WriteReg32(NFI_CON_REG16, CON_FIFO_FLUSH | CON_NFI_RST);
		while (DRV_Reg16(NFI_MASTERSTA_REG16) & 0xFFF) {
			timeout--;
			if (!timeout)
				pr_notice("Wait for NFI_MASTERSTA timeout\n");
		}
	}
	/* issue reset operation */
	mb();
	DRV_WriteReg32(NFI_CON_REG16, CON_FIFO_FLUSH | CON_NFI_RST);

	return mtk_nand_status_ready(STA_NFI_FSM_MASK | STA_NAND_BUSY) && mtk_nand_RFIFOValidSize(0)
		&& mtk_nand_WFIFOValidSize(0);
}

/******************************************************************************
 * mtk_nand_set_mode
 *
 * DESCRIPTION:
 *	  Set the oepration mode !
 *
 * PARAMETERS:
 *	 u16 u2OpMode (read/write)
 *
 * RETURNS:
 *	 None
 *
 * NOTES:
 *	 None
 *
 ******************************************************************************/
static void mtk_nand_set_mode(u16 u2OpMode)
{
	u16 u2Mode = DRV_Reg16(NFI_CNFG_REG16);

	u2Mode &= ~CNFG_OP_MODE_MASK;
	u2Mode |= u2OpMode;
	DRV_WriteReg16(NFI_CNFG_REG16, u2Mode);
}

/******************************************************************************
 * mtk_nand_set_autoformat
 *
 * DESCRIPTION:
 *	  Enable/Disable hardware autoformat !
 *
 * PARAMETERS:
 *	 bool bEnable (Enable/Disable)
 *
 * RETURNS:
 *	 None
 *
 * NOTES:
 *	 None
 *
 ******************************************************************************/
static void mtk_nand_set_autoformat(bool bEnable)
{
	if (bEnable)
		NFI_SET_REG16(NFI_CNFG_REG16, CNFG_AUTO_FMT_EN);
	else
		NFI_CLN_REG16(NFI_CNFG_REG16, CNFG_AUTO_FMT_EN);
}

/******************************************************************************
 * mtk_nand_configure_fdm
 *
 * DESCRIPTION:
 *	 Configure the FDM data size !
 *
 * PARAMETERS:
 *	 u16 u2FDMSize
 *
 * RETURNS:
 *	 None
 *
 * NOTES:
 *	 None
 *
 ******************************************************************************/
static void mtk_nand_configure_fdm(u16 u2FDMSize)
{
	if (mtk_nfi_dev_comp->chip_ver == 1) {
		NFI_CLN_REG16(NFI_PAGEFMT_REG16, PAGEFMT_FDM_MASK | PAGEFMT_FDM_ECC_MASK);
		NFI_SET_REG16(NFI_PAGEFMT_REG16, u2FDMSize << PAGEFMT_FDM_SHIFT);
		NFI_SET_REG16(NFI_PAGEFMT_REG16, u2FDMSize << PAGEFMT_FDM_ECC_SHIFT);
	} else if (mtk_nfi_dev_comp->chip_ver == 2) {
		NFI_CLN_REG32(NFI_PAGEFMT_REG32, PAGEFMT_FDM_MASK | PAGEFMT_FDM_ECC_MASK);
		NFI_SET_REG32(NFI_PAGEFMT_REG32, u2FDMSize << PAGEFMT_FDM_SHIFT);
		NFI_SET_REG32(NFI_PAGEFMT_REG32, u2FDMSize << PAGEFMT_FDM_ECC_SHIFT);
	} else {
		pr_err("[mtk_nand_configure_fdm] ERROR, mtk_nfi_dev_comp->chip_ver=%d\n",
			mtk_nfi_dev_comp->chip_ver);
	}
}

static bool mtk_nand_pio_ready(void)
{
	int count = 0;

	while (!(DRV_Reg16(NFI_PIO_DIRDY_REG16) & 1)) {
		count++;
		if (count > 0xffff) {
			pr_info("PIO_DIRDY timeout\n");
			return false;
		}
	}

	return true;
}

/******************************************************************************
 * mtk_nand_set_command
 *
 * DESCRIPTION:
 *	  Send hardware commands to NAND devices !
 *
 * PARAMETERS:
 *	 u16 command
 *
 * RETURNS:
 *	 None
 *
 * NOTES:
 *	 None
 *
 ******************************************************************************/
static bool mtk_nand_set_command(u16 command)
{
	/* Write command to device */
	mb();
	DRV_WriteReg16(NFI_CMD_REG16, command);
	return mtk_nand_status_ready(STA_CMD_STATE);
}

/******************************************************************************
 * mtk_nand_set_address
 *
 * DESCRIPTION:
 *	  Set the hardware address register !
 *
 * PARAMETERS:
 *	 struct nand_chip *nand, u32 u4RowAddr
 *
 * RETURNS:
 *	 None
 *
 * NOTES:
 *	 None
 *
 ******************************************************************************/
static bool mtk_nand_set_address(u32 u4ColAddr, u32 u4RowAddr, u16 u2ColNOB, u16 u2RowNOB)
{
	/* fill cycle addr */
	mb();
	DRV_WriteReg32(NFI_COLADDR_REG32, u4ColAddr);
	DRV_WriteReg32(NFI_ROWADDR_REG32, u4RowAddr);
	DRV_WriteReg16(NFI_ADDRNOB_REG16, u2ColNOB | (u2RowNOB << ADDR_ROW_NOB_SHIFT));
	return mtk_nand_status_ready(STA_ADDR_STATE);
}

/* ------------------------------------------------------------------------------- */
static bool mtk_nand_device_reset(void)
{
	u32 timeout = 0xFFFF;

	mtk_nand_reset();

	DRV_WriteReg(NFI_CNFG_REG16, CNFG_OP_RESET);

	mtk_nand_set_command(NAND_CMD_RESET);

	while (!(DRV_Reg32(NFI_STA_REG32) & STA_NAND_BUSY_RETURN) && (timeout--))
		;

	if (!timeout)
		return FALSE;
	else
		return TRUE;
}

/* ------------------------------------------------------------------------------- */

/******************************************************************************
 * mtk_nand_check_RW_count
 *
 * DESCRIPTION:
 *	  Check the RW how many sectors !
 *
 * PARAMETERS:
 *	 u16 u2WriteSize
 *
 * RETURNS:
 *	 None
 *
 * NOTES:
 *	 None
 *
 ******************************************************************************/
static bool mtk_nand_check_RW_count(u16 u2WriteSize)
{
	u32 timeout = 0xFFFF;
	u16 u2SecNum = u2WriteSize >> host->hw->nand_sec_shift;

	while (ADDRCNTR_CNTR(DRV_Reg32(NFI_ADDRCNTR_REG16)) < u2SecNum) {
		timeout--;
		if (timeout == 0) {
			pr_info("[%s] timeout\n", __func__);
			return false;
		}
	}
	return true;
}

/******************************************************************************
 * mtk_nand_ready_for_read
 *
 * DESCRIPTION:
 *	  Prepare hardware environment for read !
 *
 * PARAMETERS:
 *	 struct nand_chip *nand, u32 u4RowAddr
 *
 * RETURNS:
 *	 None
 *
 * NOTES:
 *	 None
 *
 ******************************************************************************/
static bool mtk_nand_ready_for_read(struct nand_chip *nand, u32 u4RowAddr, u32 u4ColAddr,
					u16 sec_num, bool full, u8 *buf)
{
	/* Reset NFI HW internal state machine and flush NFI in/out FIFO */
	bool bRet = false;
	/* u16 sec_num = 1 << (nand->page_shift - host->hw->nand_sec_shift); */
	u32 col_addr = u4ColAddr;
	u32 colnob = 2, rownob = devinfo.addr_cycle - 2;

	/* u32 reg_val = DRV_Reg32(NFI_MASTERRST_REG32); */
	unsigned int phys = 0;

	if (full) {
		mtk_dir = DMA_FROM_DEVICE;
		sg_init_one(&mtk_sg, buf, (sec_num * (1 << host->hw->nand_sec_shift)));
		dma_map_sg(mtk_dev, &mtk_sg, 1, mtk_dir);
		phys = mtk_sg.dma_address;
		/* pr_debug("[xl] phys va 0x%x\n", phys); */
	}

	if (DRV_Reg32(NFI_NAND_TYPE_CNFG_REG32) & 0x3) {
		NFI_SET_REG16(NFI_MASTERRST_REG32, PAD_MACRO_RST);	/* reset */
		NFI_CLN_REG16(NFI_MASTERRST_REG32, PAD_MACRO_RST);	/* dereset */
	}

	if (nand->options & NAND_BUSWIDTH_16)
		col_addr /= 2;

	if (!mtk_nand_reset())
		goto cleanup;
	if (g_bHwEcc) {
		/* Enable HW ECC */
		NFI_SET_REG32(NFI_CNFG_REG16, CNFG_HW_ECC_EN);
	} else {
		NFI_CLN_REG32(NFI_CNFG_REG16, CNFG_HW_ECC_EN);
	}

	mtk_nand_set_mode(CNFG_OP_READ);
	NFI_SET_REG16(NFI_CNFG_REG16, CNFG_READ_EN);
	DRV_WriteReg32(NFI_CON_REG16, sec_num << CON_NFI_SEC_SHIFT);

	if (full) {
		NFI_SET_REG16(NFI_CNFG_REG16, CNFG_AHB);
		NFI_SET_REG16(NFI_CNFG_REG16, CNFG_DMA_BURST_EN);
		/* phys = nand_virt_to_phys_add((unsigned long) buf); */

		if (!phys) {
			pr_err("[mtk_nand_ready_for_read]convert virt addr (%lx) to phys add (%x)fail!!!",
				   (unsigned long)buf, phys);
			return false;
		}
		DRV_WriteReg32(NFI_STRADDR_REG32, phys);

		if (g_bHwEcc)
			NFI_SET_REG16(NFI_CNFG_REG16, CNFG_HW_ECC_EN);
		else
			NFI_CLN_REG16(NFI_CNFG_REG16, CNFG_HW_ECC_EN);

	} else {
		NFI_CLN_REG16(NFI_CNFG_REG16, CNFG_HW_ECC_EN);
		NFI_CLN_REG16(NFI_CNFG_REG16, CNFG_AHB);
	}

	mtk_nand_set_autoformat(full);
	if (full) {
		if (g_bHwEcc)
			ECC_Decode_Start();
	}
	if (!mtk_nand_set_command(NAND_CMD_READ0))
		goto cleanup;
	if (!mtk_nand_set_address(col_addr, u4RowAddr, colnob, rownob))
		goto cleanup;

	if (!mtk_nand_set_command(NAND_CMD_READSTART))
		goto cleanup;

	if (!mtk_nand_status_ready(STA_NAND_BUSY))
		goto cleanup;

	bRet = true;

cleanup:
	return bRet;
}

/******************************************************************************
 * mtk_nand_ready_for_write
 *
 * DESCRIPTION:
 *	  Prepare hardware environment for write !
 *
 * PARAMETERS:
 *	 struct nand_chip *nand, u32 u4RowAddr
 *
 * RETURNS:
 *	 None
 *
 * NOTES:
 *	 None
 *
 ******************************************************************************/
static bool mtk_nand_ready_for_write(struct nand_chip *nand, u32 u4RowAddr, u32 col_addr, bool full,
					 u8 *buf)
{
	bool bRet = false;
	u32 sec_num = 1 << (nand->page_shift - host->hw->nand_sec_shift);
	u32 colnob = 2, rownob = devinfo.addr_cycle - 2;
	unsigned int phys = 0;

	if (full) {
		mtk_dir = DMA_TO_DEVICE;
		sg_init_one(&mtk_sg, buf, (1 << nand->page_shift));
		dma_map_sg(mtk_dev, &mtk_sg, 1, mtk_dir);
		phys = mtk_sg.dma_address;
		/* pr_debug("[xl] phys va 0x%x\n", phys); */
	}

	if (nand->options & NAND_BUSWIDTH_16)
		col_addr /= 2;

	/* Reset NFI HW internal state machine and flush NFI in/out FIFO */
	if (!mtk_nand_reset()) {
		pr_err("[Bean]mtk_nand_ready_for_write (mtk_nand_reset) fail!\n");
		return false;
	}

	mtk_nand_set_mode(CNFG_OP_PRGM);

	NFI_CLN_REG16(NFI_CNFG_REG16, CNFG_READ_EN);

	DRV_WriteReg32(NFI_CON_REG16, sec_num << CON_NFI_SEC_SHIFT);

	if (full) {
		NFI_SET_REG16(NFI_CNFG_REG16, CNFG_AHB);
		NFI_SET_REG16(NFI_CNFG_REG16, CNFG_DMA_BURST_EN);
		/* phys = nand_virt_to_phys_add((unsigned long) buf); */
		/* T_phys=__virt_to_phys(buf); */
		if (!phys) {
			pr_err("[mt65xx_nand_ready_for_write]convert virt addr (%lx) to phys add fail!!!",
				   (unsigned long)buf);
			return false;
		}
		DRV_WriteReg32(NFI_STRADDR_REG32, phys);
		if (g_bHwEcc)
			NFI_SET_REG16(NFI_CNFG_REG16, CNFG_HW_ECC_EN);
		else
			NFI_CLN_REG16(NFI_CNFG_REG16, CNFG_HW_ECC_EN);
	} else {
		NFI_CLN_REG16(NFI_CNFG_REG16, CNFG_HW_ECC_EN);
		NFI_CLN_REG16(NFI_CNFG_REG16, CNFG_AHB);
	}

	mtk_nand_set_autoformat(full);

	if (full) {
		if (g_bHwEcc)
			ECC_Encode_Start();
	}

	if (!mtk_nand_set_command(NAND_CMD_SEQIN)) {
		pr_err("[Bean]mtk_nand_ready_for_write (mtk_nand_set_command) fail!\n");
		goto cleanup;
	}
	/* 1 FIXED ME: For Any Kind of AddrCycle */
	if (!mtk_nand_set_address(col_addr, u4RowAddr, colnob, rownob)) {
		pr_err("[Bean]mtk_nand_ready_for_write (mtk_nand_set_address) fail!\n");
		goto cleanup;
	}

	if (!mtk_nand_status_ready(STA_NAND_BUSY)) {
		pr_err("[Bean]mtk_nand_ready_for_write (mtk_nand_status_ready) fail!\n");
		goto cleanup;
	}

	bRet = true;
cleanup:

	return bRet;
}

static bool mtk_nand_check_dececc_done(u32 u4SecNum)
{
	u32 dec_mask;
	u32 fsm_mask;
	u32 ECC_DECFSM_IDLE;
	struct timeval timer_timeout, timer_cur;

	do_gettimeofday(&timer_timeout);

	timer_timeout.tv_usec += 800 * 1000;	/* 500ms */
	if (timer_timeout.tv_usec >= 1000000) {	/* 1 second */
		timer_timeout.tv_usec -= 1000000;
		timer_timeout.tv_sec += 1;
	}

	dec_mask = (1 << (u4SecNum - 1));
	while (dec_mask != (DRV_Reg(ECC_DECDONE_REG16) & dec_mask)) {
		do_gettimeofday(&timer_cur);
		if (timeval_compare(&timer_cur, &timer_timeout) >= 0) {
			pr_notice("ECC_DECDONE: timeout 0x%x %d\n", DRV_Reg(ECC_DECDONE_REG16),
				u4SecNum);
			dump_nfi();
			return false;
		}
	}

	if (mtk_nfi_dev_comp->chip_ver == 1) {
		fsm_mask = 0x7F0F0F0F;
		ECC_DECFSM_IDLE = ECC_DECFSM_IDLE_V1;
	} else if (mtk_nfi_dev_comp->chip_ver == 2) {
		fsm_mask = 0x3F3FFF0F;
		ECC_DECFSM_IDLE = ECC_DECFSM_IDLE_V2;
	} else {
		fsm_mask = 0xFFFFFFFF;
		ECC_DECFSM_IDLE = 0xFFFFFFFF;
		pr_err("[mtk_nand_check_dececc_done] ERROR, mtk_nfi_dev_comp->chip_ver=%d\n",
			mtk_nfi_dev_comp->chip_ver);
	}

	while ((DRV_Reg32(ECC_DECFSM_REG32) & fsm_mask) != ECC_DECFSM_IDLE) {
		do_gettimeofday(&timer_cur);
		if (timeval_compare(&timer_cur, &timer_timeout) >= 0) {
			pr_notice("ECC_DECDONE: timeout 0x%x 0x%x %d\n",
				DRV_Reg32(ECC_DECFSM_REG32), DRV_Reg(ECC_DECDONE_REG16), u4SecNum);
			dump_nfi();
			return false;
		}
	}
	return true;
}

/******************************************************************************
 * mtk_nand_read_page_data
 *
 * DESCRIPTION:
 *	 Fill the page data into buffer !
 *
 * PARAMETERS:
 *	 u8* pDataBuf, u32 u4Size
 *
 * RETURNS:
 *	 None
 *
 * NOTES:
 *	 None
 *
 ******************************************************************************/
static bool mtk_nand_dma_read_data(struct mtd_info *mtd, u8 *buf, u32 length)
{
	int interrupt_en = g_i4Interrupt;
	int timeout = 0xfffff;
	/* struct scatterlist sg; */
	/* enum dma_data_direction dir = DMA_FROM_DEVICE; */
	/* pr_debug("[xl] dma read buf in 0x%lx\n", (unsigned long)buf); */
	/* sg_init_one(&sg, buf, length); */
	/* pr_debug("[xl] dma read buf out 0x%lx\n", (unsigned long)buf); */
	/* dma_map_sg(&(mtd->dev), &sg, 1, dir); */

	NFI_CLN_REG16(NFI_CNFG_REG16, CNFG_BYTE_RW);
	/* DRV_WriteReg32(NFI_STRADDR_REG32, __virt_to_phys(pDataBuf)); */

	if ((unsigned long)buf % 16) {	/* TODO: can not use AHB mode here */
		pr_debug("Un-16-aligned address\n");
		NFI_CLN_REG16(NFI_CNFG_REG16, CNFG_DMA_BURST_EN);
	} else {
		NFI_SET_REG16(NFI_CNFG_REG16, CNFG_DMA_BURST_EN);
	}

	NFI_SET_REG16(NFI_CNFG_REG16, CNFG_DMA_BURST_EN);

	DRV_Reg16(NFI_INTR_REG16);
	DRV_WriteReg16(NFI_INTR_EN_REG16, INTR_AHB_DONE_EN);

	if (interrupt_en)
		init_completion(&g_comp_AHB_Done);
	/* dmac_inv_range(pDataBuf, pDataBuf + u4Size); */
	mb();
	NFI_SET_REG32(NFI_CON_REG16, CON_NFI_BRD);
	g_running_dma = 1;

	if (interrupt_en) {
		/* Wait 10ms for AHB done */
		if (!wait_for_completion_timeout(&g_comp_AHB_Done, 50)) {
			pr_notice("wait for completion timeout happened @ [%s]: %d\n", __func__,
				__LINE__);
			dump_nfi();
			g_running_dma = 0;
			return false;
		}
		g_running_dma = 0;
		while ((length >> host->hw->nand_sec_shift) >
			   ((DRV_Reg32(NFI_BYTELEN_REG16) & 0x1f000) >> 12)) {
			timeout--;
			if (timeout == 0) {
				pr_err("[%s] poll BYTELEN error\n", __func__);
				g_running_dma = 0;
				return false;	/* 4  // AHB Mode Time Out! */
			}
		}
	} else {
		while (!DRV_Reg16(NFI_INTR_REG16)) {
			timeout--;
			if (timeout == 0) {
				pr_err("[%s] poll nfi_intr error\n", __func__);
				dump_nfi();
				g_running_dma = 0;
				return false;	/* 4  // AHB Mode Time Out! */
			}
		}
		g_running_dma = 0;
		while ((length >> host->hw->nand_sec_shift) >
			   ((DRV_Reg32(NFI_BYTELEN_REG16) & 0x1f000) >> 12)) {
			timeout--;
			if (timeout == 0) {
				pr_err("[%s] poll BYTELEN error\n", __func__);
				dump_nfi();
				g_running_dma = 0;
				return false;	/* 4  // AHB Mode Time Out! */
			}
		}
	}

	/* dma_unmap_sg(&(mtd->dev), &sg, 1, dir); */
	return true;
}

static bool mtk_nand_mcu_read_data(struct mtd_info *mtd, u8 *buf, u32 length)
{
	int timeout = 0xffff;
	u32 i;
	u32 *buf32 = (u32 *) buf;

	if ((unsigned long)buf % 4 || length % 4)
		NFI_SET_REG16(NFI_CNFG_REG16, CNFG_BYTE_RW);
	else
		NFI_CLN_REG16(NFI_CNFG_REG16, CNFG_BYTE_RW);

	/* DRV_WriteReg32(NFI_STRADDR_REG32, 0); */
	mb();
	NFI_SET_REG32(NFI_CON_REG16, CON_NFI_BRD);

	if ((unsigned long)buf % 4 || length % 4) {
		for (i = 0; (i < (length)) && (timeout > 0);) {
			/* if (FIFO_RD_REMAIN(DRV_Reg16(NFI_FIFOSTA_REG16)) >= 4) */
			if (DRV_Reg16(NFI_PIO_DIRDY_REG16) & 1) {
				*buf++ = (u8) DRV_Reg32(NFI_DATAR_REG32);
				i++;
			} else {
				timeout--;
			}
			if (timeout == 0) {
				pr_err("[%s] timeout\n", __func__);
				dump_nfi();
				return false;
			}
		}
	} else {
		for (i = 0; (i < (length >> 2)) && (timeout > 0);) {
			/* if (FIFO_RD_REMAIN(DRV_Reg16(NFI_FIFOSTA_REG16)) >= 4) */
			if (DRV_Reg16(NFI_PIO_DIRDY_REG16) & 1) {
				*buf32++ = DRV_Reg32(NFI_DATAR_REG32);
				i++;
			} else {
				timeout--;
			}
			if (timeout == 0) {
				pr_err("[%s] timeout\n", __func__);
				dump_nfi();
				return false;
			}
		}
	}
	return true;
}

static bool mtk_nand_read_page_data(struct mtd_info *mtd, u8 *pDataBuf, u32 u4Size)
{
	return mtk_nand_dma_read_data(mtd, pDataBuf, u4Size);
}

/******************************************************************************
 * mtk_nand_write_page_data
 *
 * DESCRIPTION:
 *	 Fill the page data into buffer !
 *
 * PARAMETERS:
 *	 u8* pDataBuf, u32 u4Size
 *
 * RETURNS:
 *	 None
 *
 * NOTES:
 *	 None
 *
 ******************************************************************************/
static bool mtk_nand_dma_write_data(struct mtd_info *mtd, u8 *pDataBuf, u32 u4Size)
{
	int i4Interrupt = 0;	/* g_i4Interrupt; */
	u32 timeout = 0xFFFF;
	/* struct scatterlist sg; */
	/* enum dma_data_direction dir = DMA_TO_DEVICE; */
	/* pr_debug("[xl] dma write buf in 0x%lx\n", (unsigned long)pDataBuf); */
	/* sg_init_one(&sg, pDataBuf, u4Size); */
	/* pr_debug("[xl] dma write buf out 0x%lx\n", (unsigned long)pDataBuf); */
	/* dma_map_sg(&(mtd->dev), &sg, 1, dir); */
	NFI_CLN_REG16(NFI_CNFG_REG16, CNFG_BYTE_RW);
	DRV_Reg16(NFI_INTR_REG16);
	DRV_WriteReg16(NFI_INTR_EN_REG16, 0);
	/* DRV_WriteReg32(NFI_STRADDR_REG32, (u32*)virt_to_phys(pDataBuf)); */

	if ((unsigned long)pDataBuf % 16) {	/* TODO: can not use AHB mode here */
		pr_debug("Un-16-aligned address\n");
		NFI_CLN_REG16(NFI_CNFG_REG16, CNFG_DMA_BURST_EN);
	} else {
		NFI_SET_REG16(NFI_CNFG_REG16, CNFG_DMA_BURST_EN);
	}

	NFI_SET_REG16(NFI_CNFG_REG16, CNFG_DMA_BURST_EN);

	if (i4Interrupt) {
		init_completion(&g_comp_AHB_Done);
		DRV_Reg16(NFI_INTR_REG16);
		DRV_WriteReg16(NFI_INTR_EN_REG16, INTR_AHB_DONE_EN);
	}
	/* dmac_clean_range(pDataBuf, pDataBuf + u4Size); */
	mb();
	NFI_SET_REG32(NFI_CON_REG16, CON_NFI_BWR);
	g_running_dma = 3;
	if (i4Interrupt) {
		/* Wait 10ms for AHB done */
		if (!wait_for_completion_timeout(&g_comp_AHB_Done, 10)) {
			pr_notice("wait for completion timeout happened @ [%s]: %d\n", __func__,
				__LINE__);
			dump_nfi();
			g_running_dma = 0;
			return false;
		}
		g_running_dma = 0;
		/* wait_for_completion(&g_comp_AHB_Done); */
	} else {
		while ((u4Size >> host->hw->nand_sec_shift) >
			   ((DRV_Reg32(NFI_BYTELEN_REG16) & 0x1f000) >> 12)) {
			timeout--;
			if (timeout == 0) {
				pr_err("[%s] poll BYTELEN error\n", __func__);
				g_running_dma = 0;
				return false;	/* 4  // AHB Mode Time Out! */
			}
		}
		g_running_dma = 0;
	}

	/* dma_unmap_sg(&(mtd->dev), &sg, 1, dir); */
	return true;
}

static bool mtk_nand_mcu_write_data(struct mtd_info *mtd, const u8 *buf, u32 length)
{
	u32 timeout = 0xFFFF;
	u32 i;
	u32 *pBuf32;

	NFI_CLN_REG16(NFI_CNFG_REG16, CNFG_BYTE_RW);
	NFI_SET_REG32(NFI_CON_REG16, CON_NFI_BWR);
	pBuf32 = (u32 *) buf;

	if ((unsigned long)buf % 4 || length % 4)
		NFI_SET_REG16(NFI_CNFG_REG16, CNFG_BYTE_RW);
	else
		NFI_CLN_REG16(NFI_CNFG_REG16, CNFG_BYTE_RW);

	if ((unsigned long)buf % 4 || length % 4) {
		for (i = 0; (i < (length)) && (timeout > 0);) {
			if (DRV_Reg16(NFI_PIO_DIRDY_REG16) & 1) {
				DRV_WriteReg32(NFI_DATAW_REG32, *buf++);
				i++;
			} else {
				timeout--;
			}
			if (timeout == 0) {
				pr_err("[%s] timeout\n", __func__);
				dump_nfi();
				return false;
			}
		}
	} else {
		for (i = 0; (i < (length >> 2)) && (timeout > 0);) {
			/* if (FIFO_WR_REMAIN(DRV_Reg16(NFI_FIFOSTA_REG16)) <= 12) */
			if (DRV_Reg16(NFI_PIO_DIRDY_REG16) & 1) {
				DRV_WriteReg32(NFI_DATAW_REG32, *pBuf32++);
				i++;
			} else {
				timeout--;
			}
			if (timeout == 0) {
				pr_err("[%s] timeout\n", __func__);
				dump_nfi();
				return false;
			}
		}
	}

	return true;
}

static bool mtk_nand_write_page_data(struct mtd_info *mtd, u8 *buf, u32 size)
{
	return mtk_nand_dma_write_data(mtd, buf, size);
}

/******************************************************************************
 * mtk_nand_read_fdm_data
 *
 * DESCRIPTION:
 *	 Read a fdm data !
 *
 * PARAMETERS:
 *	 u8* pDataBuf, u32 u4SecNum
 *
 * RETURNS:
 *	 None
 *
 * NOTES:
 *	 None
 *
 ******************************************************************************/
static void mtk_nand_read_fdm_data(u8 *pDataBuf, u32 u4SecNum)
{
	u32 i;
	u32 *pBuf32 = (u32 *) pDataBuf;

	if (pBuf32) {
		for (i = 0; i < u4SecNum; ++i) {
			*pBuf32++ = DRV_Reg32(NFI_FDM0L_REG32 + (i << 1));
			*pBuf32++ = DRV_Reg32(NFI_FDM0M_REG32 + (i << 1));
			/* *pBuf32++ = DRV_Reg32((u32)NFI_FDM0L_REG32 + (i<<3)); */
			/* *pBuf32++ = DRV_Reg32((u32)NFI_FDM0M_REG32 + (i<<3)); */
		}
	}
}

/******************************************************************************
 * mtk_nand_write_fdm_data
 *
 * DESCRIPTION:
 *	 Write a fdm data !
 *
 * PARAMETERS:
 *	 u8* pDataBuf, u32 u4SecNum
 *
 * RETURNS:
 *	 None
 *
 * NOTES:
 *	 None
 *
 ******************************************************************************/
static u8 fdm_buf[128];
static void mtk_nand_write_fdm_data(struct nand_chip *chip, u8 *pDataBuf, u32 u4SecNum)
{
	u32 i, j;
	u8 checksum = 0;
	bool empty = true;
	struct nand_oobfree *free_entry;
	u32 *pBuf32;

	memcpy(fdm_buf, pDataBuf, u4SecNum * 8);

	free_entry = chip->ecc.layout->oobfree;
	for (i = 0; i < MTD_MAX_OOBFREE_ENTRIES && free_entry[i].length; i++) {
		for (j = 0; j < free_entry[i].length; j++) {
			if (pDataBuf[free_entry[i].offset + j] != 0xFF)
				empty = false;
			checksum ^= pDataBuf[free_entry[i].offset + j];
		}
	}

	if (!empty)
		fdm_buf[free_entry[i - 1].offset + free_entry[i - 1].length] = checksum;

	pBuf32 = (u32 *) fdm_buf;
	for (i = 0; i < u4SecNum; ++i) {
		DRV_WriteReg32(NFI_FDM0L_REG32 + (i << 1), *pBuf32++);
		DRV_WriteReg32(NFI_FDM0M_REG32 + (i << 1), *pBuf32++);
		/* DRV_WriteReg32((u32)NFI_FDM0L_REG32 + (i<<3), *pBuf32++); */
		/* DRV_WriteReg32((u32)NFI_FDM0M_REG32 + (i<<3), *pBuf32++); */
	}
}

/******************************************************************************
 * mtk_nand_stop_read
 *
 * DESCRIPTION:
 *	 Stop read operation !
 *
 * PARAMETERS:
 *	 None
 *
 * RETURNS:
 *	 None
 *
 * NOTES:
 *	 None
 *
 ******************************************************************************/
static void mtk_nand_stop_read(void)
{
	NFI_CLN_REG32(NFI_CON_REG16, CON_NFI_BRD);
	mtk_nand_reset();
	if (g_bHwEcc)
		ECC_Decode_End();
	DRV_WriteReg16(NFI_INTR_EN_REG16, 0);
}

/******************************************************************************
 * mtk_nand_stop_write
 *
 * DESCRIPTION:
 *	 Stop write operation !
 *
 * PARAMETERS:
 *	 None
 *
 * RETURNS:
 *	 None
 *
 * NOTES:
 *	 None
 *
 ******************************************************************************/
static void mtk_nand_stop_write(void)
{
	NFI_CLN_REG32(NFI_CON_REG16, CON_NFI_BWR);
	if (g_bHwEcc)
		ECC_Encode_End();
	DRV_WriteReg16(NFI_INTR_EN_REG16, 0);
}

/* --------------------------------------------------------------------------- */
#define STATUS_READY			(0x40)
#define STATUS_FAIL				(0x01)
#define STATUS_WR_ALLOW			(0x80)

bool mtk_nand_SetFeature(struct mtd_info *mtd, u16 cmd, u32 addr, u8 *value, u8 bytes)
{
	u16 reg_val = 0;
	u8 write_count = 0;
	u32 reg = 0;
	u32 timeout = TIMEOUT_3;	/* 0xffff; */
	/* u32			 status; */
	/* struct nand_chip *chip = (struct nand_chip *)mtd->priv; */

	mtk_nand_reset();

	reg = DRV_Reg32(NFI_NAND_TYPE_CNFG_REG32);
	if (!(reg & TYPE_SLC))
		bytes <<= 1;

	reg_val |= (CNFG_OP_CUST | CNFG_BYTE_RW);
	DRV_WriteReg(NFI_CNFG_REG16, reg_val);

	mtk_nand_set_command(cmd);
	mtk_nand_set_address(addr, 0, 1, 0);

	mtk_nand_status_ready(STA_NFI_OP_MASK);

	DRV_WriteReg32(NFI_CON_REG16, 1 << CON_NFI_SEC_SHIFT);
	NFI_SET_REG32(NFI_CON_REG16, CON_NFI_BWR);
	DRV_WriteReg(NFI_STRDATA_REG16, 0x1);
	/* pr_debug("Bytes=%d\n", bytes); */
	while ((write_count < bytes) && timeout) {
		WAIT_NFI_PIO_READY(timeout);
		if (timeout == 0)
			break;
		if (reg & TYPE_SLC) {
			/* pr_debug("VALUE1:0x%2X\n", *value); */
			DRV_WriteReg8(NFI_DATAW_REG32, *value++);
		} else if (write_count % 2) {
			/* pr_debug("VALUE2:0x%2X\n", *value); */
			DRV_WriteReg8(NFI_DATAW_REG32, *value++);
		} else {
			/* pr_debug("VALUE3:0x%2X\n", *value); */
			DRV_WriteReg8(NFI_DATAW_REG32, *value);
		}
		write_count++;
		timeout = TIMEOUT_3;
	}
	*NFI_CNRNB_REG16 = 0x81;
	if (!mtk_nand_status_ready(STA_NAND_BUSY_RETURN))
		return FALSE;
	/* mtk_nand_read_status(); */
	/* if(status& 0x1) */
	/* return FALSE; */
	return TRUE;
}

bool mtk_nand_GetFeature(struct mtd_info *mtd, u16 cmd, u32 addr, u8 *value, u8 bytes)
{
	u16 reg_val = 0;
	u8 read_count = 0;
	u32 timeout = TIMEOUT_3;	/* 0xffff; */
	/* struct nand_chip *chip = (struct nand_chip *)mtd->priv; */

	mtk_nand_reset();

	reg_val |= (CNFG_OP_CUST | CNFG_BYTE_RW | CNFG_READ_EN);
	DRV_WriteReg(NFI_CNFG_REG16, reg_val);

	mtk_nand_set_command(cmd);
	mtk_nand_set_address(addr, 0, 1, 0);
	mtk_nand_status_ready(STA_NFI_OP_MASK);
	*NFI_CNRNB_REG16 = 0x81;
	mtk_nand_status_ready(STA_NAND_BUSY_RETURN);

	/* DRV_WriteReg32(NFI_CON_REG16, 0 << CON_NFI_SEC_SHIFT); */
	reg_val = DRV_Reg32(NFI_CON_REG16);
	reg_val &= ~CON_NFI_NOB_MASK;
	reg_val |= ((4 << CON_NFI_NOB_SHIFT) | CON_NFI_SRD);
	DRV_WriteReg32(NFI_CON_REG16, reg_val);
	DRV_WriteReg(NFI_STRDATA_REG16, 0x1);
	/* bytes = 20; */
	while ((read_count < bytes) && timeout) {
		WAIT_NFI_PIO_READY(timeout);
		if (timeout == 0)
			break;
		*value++ = DRV_Reg8(NFI_DATAR_REG32);
		/* pr_debug("Value[0x%02X]\n", DRV_Reg8(NFI_DATAR_REG32)); */
		read_count++;
		timeout = TIMEOUT_3;
	}
	/* chip->cmdfunc(mtd, NAND_CMD_STATUS, -1, -1); */
	/* mtk_nand_read_status(); */
	if (timeout != 0)
		return TRUE;
	else
		return FALSE;

}

const u8 data_tbl[8][5] = {
	{0x04, 0x04, 0x7C, 0x7E, 0x00},
	{0x00, 0x7C, 0x78, 0x78, 0x00},
	{0x7C, 0x76, 0x74, 0x72, 0x00},
	{0x08, 0x08, 0x00, 0x00, 0x00},
	{0x0B, 0x7E, 0x76, 0x74, 0x00},
	{0x10, 0x76, 0x72, 0x70, 0x00},
	{0x02, 0x7C, 0x7E, 0x70, 0x00},
	{0x00, 0x00, 0x00, 0x00, 0x00}
};

static void mtk_nand_modeentry_rrtry(void)
{
	mtk_nand_reset();

	mtk_nand_set_mode(CNFG_OP_CUST);

	mtk_nand_set_command(0x5C);
	mtk_nand_set_command(0xC5);

	mtk_nand_status_ready(STA_NFI_OP_MASK);
}

static void mtk_nand_rren_rrtry(bool needB3)
{
	mtk_nand_reset();

	mtk_nand_set_mode(CNFG_OP_CUST);

	if (needB3)
		mtk_nand_set_command(0xB3);
	mtk_nand_set_command(0x26);
	mtk_nand_set_command(0x5D);

	mtk_nand_status_ready(STA_NFI_OP_MASK);
}

static void mtk_nand_sprmset_rrtry(u32 addr, u32 data)/* single parameter setting */
{
	u16 reg_val = 0;
	/* u8 write_count = 0; */
	/* u32 reg = 0; */
	u32 timeout = TIMEOUT_3;	/* 0xffff; */

	mtk_nand_reset();

	reg_val |= (CNFG_OP_CUST | CNFG_BYTE_RW);
	DRV_WriteReg(NFI_CNFG_REG16, reg_val);

	mtk_nand_set_command(0x55);
	mtk_nand_set_address(addr, 0, 1, 0);

	mtk_nand_status_ready(STA_NFI_OP_MASK);

	DRV_WriteReg32(NFI_CON_REG16, 1 << CON_NFI_SEC_SHIFT);
	NFI_SET_REG32(NFI_CON_REG16, CON_NFI_BWR);
	DRV_WriteReg(NFI_STRDATA_REG16, 0x1);


	WAIT_NFI_PIO_READY(timeout);
	timeout = TIMEOUT_3;
	DRV_WriteReg8(NFI_DATAW_REG32, data);

	while (!(DRV_Reg32(NFI_STA_REG32) & STA_NAND_BUSY_RETURN) && (timeout--))
		;
}

static void mtk_nand_toshiba_rrtry(struct mtd_info *mtd, flashdev_info_t deviceinfo, u32 retryCount,
				   bool defValue)
{
}

static void mtk_nand_micron_rrtry(struct mtd_info *mtd, flashdev_info_t deviceinfo, u32 feature,
				  bool defValue)
{
}

static int g_sandisk_retry_case;	/* for new read retry table case 1,2,3,4 */
static void mtk_nand_sandisk_rrtry(struct mtd_info *mtd, flashdev_info_t deviceinfo, u32 feature,
				   bool defValue)
{
}

u16 sandisk_19nm_rr_table[18] = {
	0x0000,
	0xFF0F, 0xEEFE, 0xDDFD, 0x11EE,	/* 04h[7:4] | 07h[7:4] | 04h[3:0] | 05h[7:4] */
	0x22ED, 0x33DF, 0xCDDE, 0x01DD,
	0x0211, 0x1222, 0xBD21, 0xAD32,
	0x9DF0, 0xBCEF, 0xACDC, 0x9CFF,
	0x0000			/* align */
};

static void sandisk_19nm_rr_init(void)
{
	u32 reg_val = 0;
	u32 count = 0;
	u32 timeout = 0xffff;
	/* u32 u4RandomSetting; */
	u32 acccon;

	acccon = DRV_Reg32(NFI_ACCCON_REG32);
	DRV_WriteReg32(NFI_ACCCON_REG32, 0x31C08669);	/* to fit read retry timing */

	mtk_nand_reset();

	reg_val = (CNFG_OP_CUST | CNFG_BYTE_RW);
	DRV_WriteReg(NFI_CNFG_REG16, reg_val);
	mtk_nand_set_command(0x3B);
	mtk_nand_set_command(0xB9);

	for (count = 0; count < 9; count++) {
		mtk_nand_set_command(0x53);
		mtk_nand_set_address((0x04 + count), 0, 1, 0);
		DRV_WriteReg(NFI_CON_REG16, (CON_NFI_BWR | (1 << CON_NFI_SEC_SHIFT)));
		DRV_WriteReg(NFI_STRDATA_REG16, 1);
		timeout = 0xffff;
		WAIT_NFI_PIO_READY(timeout);
		DRV_WriteReg32(NFI_DATAW_REG32, 0x00);
		mtk_nand_reset();
	}

	DRV_WriteReg32(NFI_ACCCON_REG32, acccon);
}

static void sandisk_19nm_rr_loading(u32 retryCount, bool defValue)
{
}

static void mtk_nand_sandisk_19nm_rrtry(struct mtd_info *mtd, flashdev_info_t deviceinfo,
					u32 retryCount, bool defValue)
{
	if ((retryCount == 0) && (!defValue))
		sandisk_19nm_rr_init();
	sandisk_19nm_rr_loading(retryCount, defValue);
}

#define HYNIX_RR_TABLE_SIZE  (1026)	/* hynix read retry table size */
#define SINGLE_RR_TABLE_SIZE (64)

#define READ_RETRY_STEP (devinfo.feature_set.FeatureSet.readRetryCnt + \
	devinfo.feature_set.FeatureSet.readRetryStart)	/* 8 step or 12 step to fix read retry table */
#define HYNIX_16NM_RR_TABLE_SIZE  ((READ_RETRY_STEP == 12)?(784):(528))	/* hynix read retry table size */
#define SINGLE_RR_TABLE_16NM_SIZE  ((READ_RETRY_STEP == 12)?(48):(32))

u8 nand_hynix_rr_table[(HYNIX_RR_TABLE_SIZE + 16) / 16 * 16];	/* align as 16 byte */

#define NAND_HYX_RR_TBL_BUF nand_hynix_rr_table

static u8 real_hynix_rr_table_idx;
static u32 g_hynix_retry_count;

static bool hynix_rr_table_select(u8 table_index, flashdev_info_t *deviceinfo)
{
	u32 i;
	u32 table_size = (deviceinfo->feature_set.FeatureSet.rtype ==
		 RTYPE_HYNIX_16NM) ? SINGLE_RR_TABLE_16NM_SIZE : SINGLE_RR_TABLE_SIZE;

	for (i = 0; i < table_size; i++) {
		u8 *temp_rr_table = (u8 *) NAND_HYX_RR_TBL_BUF + table_size * table_index * 2 + 2;
		u8 *temp_inversed_rr_table = (u8 *) NAND_HYX_RR_TBL_BUF + table_size * table_index * 2 + table_size + 2;

		if (deviceinfo->feature_set.FeatureSet.rtype == RTYPE_HYNIX_16NM) {
			temp_rr_table += 14;
			temp_inversed_rr_table += 14;
		}
		if (0xFF != (temp_rr_table[i] ^ temp_inversed_rr_table[i]))
			return FALSE;	/* error table */
	}
	if (deviceinfo->feature_set.FeatureSet.rtype == RTYPE_HYNIX_16NM)
		table_size += 16;
	else
		table_size += 2;
	for (i = 0; i < table_size; i++) {
		pr_debug("%02X ", NAND_HYX_RR_TBL_BUF[i]);
		if ((i + 1) % 8 == 0)
			pr_debug("\n");
	}
	return TRUE;		/* correct table */
}

static void HYNIX_RR_TABLE_READ(flashdev_info_t *deviceinfo)
{
	u32 reg_val = 0;
	u32 read_count = 0, max_count = HYNIX_RR_TABLE_SIZE;
	u32 timeout = 0xffff;
	u8 *rr_table = (u8 *) (NAND_HYX_RR_TBL_BUF);
	u8 table_index = 0;
	u8 add_reg1[3] = { 0xFF, 0xCC };
	u8 data_reg1[3] = { 0x40, 0x4D };
	u8 cmd_reg[6] = { 0x16, 0x17, 0x04, 0x19, 0x00 };
	u8 add_reg2[6] = { 0x00, 0x00, 0x00, 0x02, 0x00 };
	bool RR_TABLE_EXIST = TRUE;

	if (deviceinfo->feature_set.FeatureSet.rtype == RTYPE_HYNIX_16NM) {
		read_count = 1;
		add_reg1[1] = 0x38;
		data_reg1[1] = 0x52;
		max_count = HYNIX_16NM_RR_TABLE_SIZE;
		if (READ_RETRY_STEP == 12)
			add_reg2[2] = 0x1F;
	}
	mtk_nand_device_reset();
	/* take care under sync mode. need change nand device inferface xiaolei */

	mtk_nand_reset();

	DRV_WriteReg(NFI_CNFG_REG16, (CNFG_OP_CUST | CNFG_BYTE_RW));

	mtk_nand_set_command(0x36);

	for (; read_count < 2; read_count++) {
		mtk_nand_set_address(add_reg1[read_count], 0, 1, 0);
		DRV_WriteReg(NFI_CON_REG16, (CON_NFI_BWR | (1 << CON_NFI_SEC_SHIFT)));
		DRV_WriteReg(NFI_STRDATA_REG16, 1);
		timeout = 0xffff;
		WAIT_NFI_PIO_READY(timeout);
		DRV_WriteReg32(NFI_DATAW_REG32, data_reg1[read_count]);
		mtk_nand_reset();
	}

	for (read_count = 0; read_count < 5; read_count++)
		mtk_nand_set_command(cmd_reg[read_count]);
	for (read_count = 0; read_count < 5; read_count++)
		mtk_nand_set_address(add_reg2[read_count], 0, 1, 0);
	mtk_nand_set_command(0x30);
	DRV_WriteReg(NFI_CNRNB_REG16, 0xF1);
	timeout = 0xffff;
	while (!(DRV_Reg32(NFI_STA_REG32) & STA_NAND_BUSY_RETURN) && (timeout--))
		;

	reg_val = (CNFG_OP_CUST | CNFG_BYTE_RW | CNFG_READ_EN);
	DRV_WriteReg(NFI_CNFG_REG16, reg_val);
	DRV_WriteReg(NFI_CON_REG16, (CON_NFI_BRD | (2 << CON_NFI_SEC_SHIFT)));
	DRV_WriteReg(NFI_STRDATA_REG16, 0x1);
	timeout = 0xffff;
	read_count = 0;		/* how???? */
	while ((read_count < max_count) && timeout) {
		WAIT_NFI_PIO_READY(timeout);
		*rr_table++ = (unsigned char) DRV_Reg32(NFI_DATAR_REG32);
		read_count++;
		timeout = 0xFFFF;
	}

	mtk_nand_device_reset();
	/* take care under sync mode. need change nand device inferface xiaolei */

	reg_val = (CNFG_OP_CUST | CNFG_BYTE_RW);
	if (deviceinfo->feature_set.FeatureSet.rtype == RTYPE_HYNIX_16NM) {
		DRV_WriteReg(NFI_CNFG_REG16, reg_val);
		mtk_nand_set_command(0x36);
		mtk_nand_set_address(0x38, 0, 1, 0);
		DRV_WriteReg(NFI_CON_REG16, (CON_NFI_BWR | (1 << CON_NFI_SEC_SHIFT)));
		DRV_WriteReg(NFI_STRDATA_REG16, 1);
		WAIT_NFI_PIO_READY(timeout);
		DRV_WriteReg32(NFI_DATAW_REG32, 0x00);
		mtk_nand_reset();
		mtk_nand_set_command(0x16);
		mtk_nand_set_command(0x00);
		mtk_nand_set_address(0x00, 0, 1, 0);	/* dummy read, add don't care */
		mtk_nand_set_command(0x30);
	} else {
		DRV_WriteReg(NFI_CNFG_REG16, reg_val);
		mtk_nand_set_command(0x38);
	}
	timeout = 0xffff;
	while (!(DRV_Reg32(NFI_STA_REG32) & STA_NAND_BUSY_RETURN) && (timeout--))
		;
	rr_table = (u8 *) (NAND_HYX_RR_TBL_BUF);
	if (deviceinfo->feature_set.FeatureSet.rtype == RTYPE_HYNIX) {
		if ((rr_table[0] != 8) || (rr_table[1] != 8)) {
			RR_TABLE_EXIST = FALSE;
			mtk_nand_assert(0);
		}
	} else if (deviceinfo->feature_set.FeatureSet.rtype == RTYPE_HYNIX_16NM) {
		for (read_count = 0; read_count < 8; read_count++) {
			if ((rr_table[read_count] != 8) || (rr_table[read_count + 8] != 4)) {
				RR_TABLE_EXIST = FALSE;
				break;
			}
		}
	}
	if (RR_TABLE_EXIST) {
		for (table_index = 0; table_index < 8; table_index++) {
			if (hynix_rr_table_select(table_index, deviceinfo)) {
				real_hynix_rr_table_idx = table_index;
				pr_debug("Hynix rr_tbl_id %d\n", real_hynix_rr_table_idx);
				break;
			}
		}
		if (table_index == 8)
			mtk_nand_assert(0);
	} else {
		pr_err("Hynix RR table index error!\n");
	}
}

static void HYNIX_Set_RR_Para(u32 rr_index, flashdev_info_t *deviceinfo)
{
	/* u32 reg_val = 0; */
	u32 timeout = 0xffff;
	u8 count, max_count = 8;
	u8 add_reg[9] = { 0xCC, 0xBF, 0xAA, 0xAB, 0xCD, 0xAD, 0xAE, 0xAF };
	u8 *hynix_rr_table =
		(u8 *) NAND_HYX_RR_TBL_BUF + SINGLE_RR_TABLE_SIZE * real_hynix_rr_table_idx * 2 + 2;
	if (deviceinfo->feature_set.FeatureSet.rtype == RTYPE_HYNIX_16NM) {
		add_reg[0] = 0x38;	/* 0x38, 0x39, 0x3A, 0x3B */
		for (count = 1; count < 4; count++)
			add_reg[count] = add_reg[0] + count;
		hynix_rr_table += 14;
		max_count = 4;
	}
	mtk_nand_reset();

	DRV_WriteReg(NFI_CNFG_REG16, (CNFG_OP_CUST | CNFG_BYTE_RW));
	/* mtk_nand_set_command(0x36); */

	for (count = 0; count < max_count; count++) {
		mtk_nand_set_command(0x36);
		mtk_nand_set_address(add_reg[count], 0, 1, 0);
		DRV_WriteReg(NFI_CON_REG16, (CON_NFI_BWR | (1 << CON_NFI_SEC_SHIFT)));
		DRV_WriteReg(NFI_STRDATA_REG16, 1);
		timeout = 0xffff;
		WAIT_NFI_PIO_READY(timeout);
		if (timeout == 0) {
			pr_notice("HYNIX_Set_RR_Para timeout\n");
			break;
		}
		DRV_WriteReg32(NFI_DATAW_REG32, hynix_rr_table[rr_index * max_count + count]);
		mtk_nand_reset();
	}
	mtk_nand_set_command(0x16);
}

static void mtk_nand_hynix_rrtry(struct mtd_info *mtd, flashdev_info_t deviceinfo, u32 retryCount,
				 bool defValue)
{
	if (defValue == FALSE) {
		if (g_hynix_retry_count == READ_RETRY_STEP)
			g_hynix_retry_count = 0;

		pr_debug("Hynix Retry %d\n", g_hynix_retry_count);
		HYNIX_Set_RR_Para(g_hynix_retry_count, &deviceinfo);
		/* HYNIX_Get_RR_Para(g_hynix_retry_count, &deviceinfo); */
		g_hynix_retry_count++;
	}
}

static void mtk_nand_hynix_16nm_rrtry(struct mtd_info *mtd, flashdev_info_t deviceinfo,
					  u32 retryCount, bool defValue)
{
	if (defValue == FALSE) {
		if (g_hynix_retry_count == READ_RETRY_STEP)
			g_hynix_retry_count = 0;

		pr_debug("Hynix 16nm Retry %d\n", g_hynix_retry_count);
		HYNIX_Set_RR_Para(g_hynix_retry_count, &deviceinfo);
		/* mb(); */
		/* HYNIX_Get_RR_Para(g_hynix_retry_count, &deviceinfo); */
		g_hynix_retry_count++;

	}
}

u32 special_rrtry_setting[37] = {
	0x00000000, 0x7C00007C, 0x787C0004, 0x74780078,
	0x7C007C08, 0x787C7C00, 0x74787C7C, 0x70747C00,
	0x7C007800, 0x787C7800, 0x74787800, 0x70747800,
	0x6C707800, 0x00040400, 0x7C000400, 0x787C040C,
	0x7478040C, 0x7C000810, 0x00040810, 0x04040C0C,
	0x00040C10, 0x00081014, 0x000C1418, 0x7C040C0C,
	0x74787478, 0x70747478, 0x6C707478, 0x686C7478,
	0x74787078, 0x70747078, 0x686C7078, 0x6C707078,
	0x6C706C78, 0x686C6C78, 0x64686C78, 0x686C6874,
	0x64686874,
};

static u32 mtk_nand_rrtry_setting(flashdev_info_t deviceinfo, enum readRetryType type,
				  u32 retryStart, u32 loopNo)
{
	u32 value;
	/* if(RTYPE_MICRON == type || RTYPE_SANDISK== type || RTYPE_TOSHIBA== type || RTYPE_HYNIX== type) */
	{
		if (retryStart != 0xFFFFFFFF)
			value = retryStart + loopNo;
		else
			value = special_rrtry_setting[loopNo];
	}

	return value;
}

typedef void (*rrtryFunctionType) (struct mtd_info *mtd, flashdev_info_t deviceinfo, u32 feature,
				   bool defValue);

static rrtryFunctionType rtyFuncArray[] = {
	mtk_nand_micron_rrtry,
	mtk_nand_sandisk_rrtry,
	mtk_nand_sandisk_19nm_rrtry,
	mtk_nand_toshiba_rrtry,
	mtk_nand_hynix_rrtry,
	mtk_nand_hynix_16nm_rrtry
};


static void mtk_nand_rrtry_func(struct mtd_info *mtd, flashdev_info_t deviceinfo, u32 feature,
				bool defValue)
{
	rtyFuncArray[deviceinfo.feature_set.FeatureSet.rtype] (mtd, deviceinfo, feature, defValue);
}

/******************************************************************************
 * mtk_nand_exec_read_page
 *
 * DESCRIPTION:
 *	 Read a page data !
 *
 * PARAMETERS:
 *	 struct mtd_info *mtd, u32 u4RowAddr, u32 u4PageSize,
 *	 u8* pPageBuf, u8* pFDMBuf
 *
 * RETURNS:
 *	 None
 *
 * NOTES:
 *	 None
 *
 ******************************************************************************/
int mtk_nand_exec_read_page(struct mtd_info *mtd, u32 u4RowAddr, u32 u4PageSize, u8 *pPageBuf,
				u8 *pFDMBuf)
{
	u8 *buf;
	int bRet = ERR_RTN_SUCCESS;
	struct nand_chip *nand = mtd->priv;
	u32 u4SecNum = u4PageSize >> host->hw->nand_sec_shift;
	u32 backup_corrected, backup_failed;
	bool readRetry = FALSE;
	int retryCount = 0;
	/* u32 val; */
	u32 tempBitMap;

	tempBitMap = 0;

	/* flush_icache_range(pPageBuf, (pPageBuf + u4PageSize));//flush_cache_all();//cache flush */

	if (((unsigned long)pPageBuf % 16) && local_buffer_16_align) {
		buf = local_buffer_16_align;
		/* pr_debug("[xl] read buf (1) 0x%lx\n",(unsigned long)buf); */
	} else {
		if (virt_addr_valid(pPageBuf) == 0) {	/* It should be allocated by vmalloc */
			buf = local_buffer_16_align;
			/* pr_debug("[xl] read buf (2) 0x%lx\n",(unsigned long)buf); */
		} else {
			buf = pPageBuf;
			/* pr_debug("[xl] read buf (3) 0x%lx\n",(unsigned long)buf); */
		}
	}
	backup_corrected = mtd->ecc_stats.corrected;
	backup_failed = mtd->ecc_stats.failed;

	if (g_bTricky_CS)
		u4RowAddr = mtk_nand_cs_on(nand, NFI_TRICKY_CS, u4RowAddr);

	mtk_nand_turn_on_randomizer(u4RowAddr, 0, 0);
	if (mtk_nand_ready_for_read(nand, u4RowAddr, 0, u4SecNum, true, buf)) {
		if (!mtk_nand_read_page_data(mtd, buf, u4PageSize)) {
			pr_err("mtk_nand_read_page_data fail\n");
			bRet = ERR_RTN_FAIL;
	}
		dma_unmap_sg(mtk_dev, &mtk_sg, 1, mtk_dir);
		if (!mtk_nand_status_ready(STA_NAND_BUSY)) {
			pr_err("mtk_nand_status_ready fail\n");
			bRet = ERR_RTN_FAIL;
		}
		if (g_bHwEcc) {
			if (!mtk_nand_check_dececc_done(u4SecNum)) {
				pr_err("mtk_nand_check_dececc_done fail\n");
				bRet = ERR_RTN_FAIL;
			}
		}
		mtk_nand_read_fdm_data(pFDMBuf, u4SecNum);
		if (g_bHwEcc) {
			if (!mtk_nand_check_bch_error(mtd, buf, pFDMBuf,
					u4SecNum - 1, u4RowAddr, &tempBitMap)) {
				pr_debug("mtk_nand_check_bch_error fail, retryCount:%d\n",
					retryCount);
				bRet = ERR_RTN_BCH_FAIL;
			} else {
				if (0 != (DRV_Reg32(NFI_STA_REG32) & STA_READ_EMPTY)
					&& 0 != retryCount) {	/* if empty */
					pr_err("NFI read retry read empty page, return as uncorrectable\n");
					mtd->ecc_stats.failed += u4SecNum;
					bRet = ERR_RTN_BCH_FAIL;
				}
			}
		}
		mtk_nand_stop_read();
	} else {
		dma_unmap_sg(mtk_dev, &mtk_sg, 1, mtk_dir);
	}
	mtk_nand_turn_off_randomizer();

	/* flush_icache_range(pPageBuf, (pPageBuf + u4PageSize));//flush_cache_all();//cache flush */

	if (buf == local_buffer_16_align) {
		memcpy(pPageBuf, buf, u4PageSize);
		/* pr_debug("[xl] mtk_nand_exec_read_page memcpy 0x%x 0x%x\n", pPageBuf[0],buf[0]); */
	}
	/* else */
	/* pr_debug("[xl] mtk_nand_exec_read_page no memcpy 0x%x 0x%x\n", pPageBuf[0],buf[0]); */
	if (bRet != ERR_RTN_SUCCESS) {
		pr_err("ECC uncorrectable , fake buffer returned\n");
		memset(pPageBuf, 0xff, u4PageSize);
		memset(pFDMBuf, 0xff, u4SecNum * 8);
	}

	return bRet;
}

bool mtk_nand_exec_read_sector(struct mtd_info *mtd, u32 u4RowAddr, u32 u4ColAddr, u32 u4PageSize,
				   u8 *pPageBuf, u8 *pFDMBuf, int subpageno)
{
	u8 *buf;
	int bRet = ERR_RTN_SUCCESS;
	struct nand_chip *nand = mtd->priv;
	u32 u4SecNum = subpageno;
	u32 backup_corrected, backup_failed;
	bool readRetry = FALSE;
	int retryCount = 0;
	u32 tempBitMap;
	/* MSG(INIT, "mtk_nand_exec_read_page, host->hw->nand_sec_shift: %d\n", host->hw->nand_sec_shift); */

	/* flush_icache_range(pPageBuf, (pPageBuf + u4PageSize));//flush_cache_all();//cache flush */

	if (((unsigned long)pPageBuf % 16) && local_buffer_16_align) {
		buf = local_buffer_16_align;
	} else {
		if (virt_addr_valid(pPageBuf) == 0) {	/* It should be allocated by vmalloc */
			buf = local_buffer_16_align;
		} else {
			buf = pPageBuf;
		}
	}
	backup_corrected = mtd->ecc_stats.corrected;
	backup_failed = mtd->ecc_stats.failed;
	if (g_bTricky_CS)
		u4RowAddr = mtk_nand_cs_on(nand, NFI_TRICKY_CS, u4RowAddr);

	mtk_nand_turn_on_randomizer(u4RowAddr, 0, 0);
	if (mtk_nand_ready_for_read(nand, u4RowAddr, u4ColAddr, u4SecNum, true, buf)) {
		if (!mtk_nand_read_page_data(mtd, buf, u4PageSize)) {
			pr_err("mtk_nand_read_page_data fail\n");
			bRet = ERR_RTN_FAIL;
		}
		dma_unmap_sg(mtk_dev, &mtk_sg, 1, mtk_dir);
		if (!mtk_nand_status_ready(STA_NAND_BUSY)) {
			pr_err("mtk_nand_status_ready fail\n");
			bRet = ERR_RTN_FAIL;
		}
		if (g_bHwEcc) {
			if (!mtk_nand_check_dececc_done(u4SecNum)) {
				pr_err("mtk_nand_check_dececc_done fail\n");
				bRet = ERR_RTN_FAIL;
			}
		}
		mtk_nand_read_fdm_data(pFDMBuf, u4SecNum);
		if (g_bHwEcc) {
			if (!mtk_nand_check_bch_error(mtd, buf, pFDMBuf, u4SecNum - 1, u4RowAddr, NULL)) {
				if (devinfo.vendor != VEND_NONE)
					readRetry = TRUE;
				pr_debug("mtk_nand_check_bch_error fail, retryCount:%d\n",
					retryCount);
				bRet = ERR_RTN_BCH_FAIL;
			} else {
				if (0 != (DRV_Reg32(NFI_STA_REG32) & STA_READ_EMPTY)
					&& 0 != retryCount) {	/* if empty */
					pr_notice("NFI read retry read empty page, return as uncorrectable\n");
					mtd->ecc_stats.failed += u4SecNum;
					bRet = ERR_RTN_BCH_FAIL;
				}
			}
		}
		mtk_nand_stop_read();
	} else {
		dma_unmap_sg(mtk_dev, &mtk_sg, 1, mtk_dir);
	}
	mtk_nand_turn_off_randomizer();

	/* flush_icache_range(pPageBuf, (pPageBuf + u4PageSize));//flush_cache_all();//cache flush */

	if (buf == local_buffer_16_align)
		memcpy(pPageBuf, buf, u4PageSize);

	if (bRet != ERR_RTN_SUCCESS) {
		pr_err("ECC uncorrectable , fake buffer returned\n");
		memset(pPageBuf, 0xff, u4PageSize);
		memset(pFDMBuf, 0xff, u4SecNum * 8);
	}
	return bRet;
}

/******************************************************************************
 * mtk_nand_exec_write_page
 *
 * DESCRIPTION:
 *	 Write a page data !
 *
 * PARAMETERS:
 *	 struct mtd_info *mtd, u32 u4RowAddr, u32 u4PageSize,
 *	 u8* pPageBuf, u8* pFDMBuf
 *
 * RETURNS:
 *	 None
 *
 * NOTES:
 *	 None
 *
 ******************************************************************************/
int mtk_nand_exec_write_page(struct mtd_info *mtd, u32 u4RowAddr, u32 u4PageSize, u8 *pPageBuf,
				 u8 *pFDMBuf)
{
	struct nand_chip *chip = mtd->priv;
	u32 u4SecNum = u4PageSize >> host->hw->nand_sec_shift;
	u8 *buf;
	u8 status;
	/* MSG(INIT, "mtk_nand_exec_write_page, page: 0x%x\n", u4RowAddr); */
	if (g_bTricky_CS)
		u4RowAddr = mtk_nand_cs_on(chip, NFI_TRICKY_CS, u4RowAddr);

	mtk_nand_turn_on_randomizer(u4RowAddr, 1, 0);

	/* flush_icache_range(pPageBuf, (pPageBuf + u4PageSize));//flush_cache_all();//cache flush */

	if (((unsigned long)pPageBuf % 16) && local_buffer_16_align) {
		pr_info("Data buffer not 16 bytes aligned: %p\n", pPageBuf);
		memcpy(local_buffer_16_align, pPageBuf, mtd->writesize);
		buf = local_buffer_16_align;
	} else {
		if (virt_addr_valid(pPageBuf) == 0) {	/* It should be allocated by vmalloc */
			memcpy(local_buffer_16_align, pPageBuf, mtd->writesize);
			buf = local_buffer_16_align;
		} else {
			buf = pPageBuf;
		}
	}

	if (mtk_nand_ready_for_write(chip, u4RowAddr, 0, true, buf)) {
		mtk_nand_write_fdm_data(chip, pFDMBuf, u4SecNum);
		(void)mtk_nand_write_page_data(mtd, buf, u4PageSize);
		dma_unmap_sg(mtk_dev, &mtk_sg, 1, mtk_dir);
		(void)mtk_nand_check_RW_count(u4PageSize);
		mtk_nand_stop_write();
		(void)mtk_nand_set_command(NAND_CMD_PAGEPROG);
		while (DRV_Reg32(NFI_STA_REG32) & STA_NAND_BUSY)
			;
	} else {
		dma_unmap_sg(mtk_dev, &mtk_sg, 1, mtk_dir);
		pr_err("[Bean]mtk_nand_ready_for_write fail!\n");
	}

	mtk_nand_turn_off_randomizer();
	/* flush_icache_range(pPageBuf, (pPageBuf + u4PageSize));//flush_cache_all();//cache flush */

	status = chip->waitfunc(mtd, chip);
	/* pr_debug("[Bean]status:%d\n", status); */
	if (status & NAND_STATUS_FAIL)
		return -EIO;
	else
		return 0;
}

/******************************************************************************
 *
 * Write a page to a logical address
 *
 *****************************************************************************/
static int mtk_nand_write_page(struct mtd_info *mtd, struct nand_chip *chip,
				   uint32_t offset, int data_len, const uint8_t *buf,
				   int oob_required, int page, int cached, int raw)
{
	/* int block_size = 1 << (chip->phys_erase_shift); */
	int page_per_block = 1 << (chip->phys_erase_shift - chip->page_shift);
	u32 block;
	u32 page_in_block;
	u32 mapped_block;

	page_in_block = mtk_nand_page_transform(mtd, chip, page, &block, &mapped_block);
	/* MSG(INIT,"[WRITE] %d, %d, %d %d\n",mapped_block, block, page_in_block, page_per_block); */
	/* write bad index into oob */
	/* pr_debug("[xiaolei] mtk_nand_write_page 0x%x\n", (u32)buf); */
	if (mtk_nand_exec_write_page(mtd, page_in_block + mapped_block * page_per_block, mtd->writesize, (u8 *) buf,
		 chip->oob_poi)) {
		pr_err("write fail at block: 0x%x, page: 0x%x\n", mapped_block, page_in_block);
		return -EIO;
	}
	return 0;
}

/******************************************************************************
 * mtk_nand_command_bp
 *
 * DESCRIPTION:
 *	 Handle the commands from MTD !
 *
 * PARAMETERS:
 *	 struct mtd_info *mtd, unsigned int command, int column, int page_addr
 *
 * RETURNS:
 *	 None
 *
 * NOTES:
 *	 None
 *
 ******************************************************************************/
static void mtk_nand_command_bp(struct mtd_info *mtd, unsigned int command, int column,
				int page_addr)
{
	struct nand_chip *nand = mtd->priv;

	switch (command) {
	case NAND_CMD_SEQIN:
		memset(g_kCMD.au1OOB, 0xFF, sizeof(g_kCMD.au1OOB));
		g_kCMD.pDataBuf = NULL;
		/* } */
		g_kCMD.u4RowAddr = page_addr;
		g_kCMD.u4ColAddr = column;
		break;

	case NAND_CMD_PAGEPROG:
		if (g_kCMD.pDataBuf || (g_kCMD.au1OOB[0] != 0xFF)) {
			u8 *pDataBuf = g_kCMD.pDataBuf ? g_kCMD.pDataBuf : nand->buffers->databuf;
			/* pr_debug("[xiaolei] mtk_nand_command_bp 0x%x\n", (u32)pDataBuf); */
			mtk_nand_exec_write_page(mtd, g_kCMD.u4RowAddr, mtd->writesize, pDataBuf,
						 g_kCMD.au1OOB);
			g_kCMD.u4RowAddr = (u32) -1;
			g_kCMD.u4OOBRowAddr = (u32) -1;
		}
		break;

	case NAND_CMD_READOOB:
		g_kCMD.u4RowAddr = page_addr;
		g_kCMD.u4ColAddr = column + mtd->writesize;
		break;

	case NAND_CMD_READ0:
		g_kCMD.u4RowAddr = page_addr;
		g_kCMD.u4ColAddr = column;
		break;

	case NAND_CMD_ERASE1:
		(void)mtk_nand_reset();
		mtk_nand_set_mode(CNFG_OP_ERASE);
		(void)mtk_nand_set_command(NAND_CMD_ERASE1);
		(void)mtk_nand_set_address(0, page_addr, 0, devinfo.addr_cycle - 2);
		break;

	case NAND_CMD_ERASE2:
		(void)mtk_nand_set_command(NAND_CMD_ERASE2);
		while (DRV_Reg32(NFI_STA_REG32) & STA_NAND_BUSY)
			;
		break;

	case NAND_CMD_STATUS:
		(void)mtk_nand_reset();
		if (mtk_nand_israndomizeron()) {
			/* g_brandstatus = TRUE; */
			mtk_nand_turn_off_randomizer();
		}
		NFI_CLN_REG16(NFI_CNFG_REG16, CNFG_BYTE_RW);
		mtk_nand_set_mode(CNFG_OP_SRD);
		mtk_nand_set_mode(CNFG_READ_EN);
		NFI_CLN_REG16(NFI_CNFG_REG16, CNFG_AHB);
		NFI_CLN_REG16(NFI_CNFG_REG16, CNFG_HW_ECC_EN);
		(void)mtk_nand_set_command(NAND_CMD_STATUS);
		NFI_CLN_REG32(NFI_CON_REG16, CON_NFI_NOB_MASK);
		/* clear byte number before SRD */
		mb();
		DRV_WriteReg32(NFI_CON_REG16, CON_NFI_SRD | (1 << CON_NFI_NOB_SHIFT));
		g_bcmdstatus = true;
		break;

	case NAND_CMD_RESET:
		mtk_nand_device_reset();
		break;

	case NAND_CMD_READID:
		/* Issue NAND chip reset command */
		/* NFI_ISSUE_COMMAND (NAND_CMD_RESET, 0, 0, 0, 0); */

		/* timeout = TIMEOUT_4; */

		/* while (timeout) */
		/* timeout--; */

		mtk_nand_reset();
		/* Disable HW ECC */
		NFI_CLN_REG16(NFI_CNFG_REG16, CNFG_HW_ECC_EN);
		NFI_CLN_REG16(NFI_CNFG_REG16, CNFG_AHB);

		/* Disable 16-bit I/O */
		/* NFI_CLN_REG16(NFI_PAGEFMT_REG16, PAGEFMT_DBYTE_EN); */

		NFI_SET_REG16(NFI_CNFG_REG16, CNFG_READ_EN | CNFG_BYTE_RW);
		(void)mtk_nand_reset();
		mtk_nand_set_mode(CNFG_OP_SRD);
		(void)mtk_nand_set_command(NAND_CMD_READID);
		(void)mtk_nand_set_address(0, 0, 1, 0);
		DRV_WriteReg32(NFI_CON_REG16, CON_NFI_SRD);
		while (DRV_Reg32(NFI_STA_REG32) & STA_DATAR_STATE)
			;
		break;

	default:
		BUG();
		break;
	}
}

/******************************************************************************
 * mtk_nand_select_chip
 *
 * DESCRIPTION:
 *	 Select a chip !
 *
 * PARAMETERS:
 *	 struct mtd_info *mtd, int chip
 *
 * RETURNS:
 *	 None
 *
 * NOTES:
 *	 None
 *
 ******************************************************************************/
static void mtk_nand_select_chip(struct mtd_info *mtd, int chip)
{
	if (chip == -1 && false == g_bInitDone) {
		struct nand_chip *nand = mtd->priv;

		struct mtk_nand_host *host = nand->priv;
		struct mtk_nand_host_hw *hw = host->hw;
		u32 spare_per_sector = mtd->oobsize / (mtd->writesize / hw->nand_sec_size);
		u32 ecc_bit = 4;
		u32 spare_bit = PAGEFMT_SPARE_16;

		switch (spare_per_sector) {
		case 16:
			spare_bit = PAGEFMT_SPARE_16;
			ecc_bit = 4;
			spare_per_sector = 16;
			break;
		case 26:
		case 27:
		case 28:
			spare_bit = PAGEFMT_SPARE_26;
			ecc_bit = 10;
			spare_per_sector = 26;
			break;
		case 32:
			ecc_bit = 12;
			if (devinfo.sectorsize == 1024)
				spare_bit = PAGEFMT_SPARE_32_1KS;
			else
				spare_bit = PAGEFMT_SPARE_32;
			spare_per_sector = 32;
			break;
		case 40:
			ecc_bit = 18;
			spare_bit = PAGEFMT_SPARE_40;
			spare_per_sector = 40;
			break;
		case 44:
			ecc_bit = 20;
			spare_bit = PAGEFMT_SPARE_44;
			spare_per_sector = 44;
			break;
		case 48:
		case 49:
			ecc_bit = 22;
			spare_bit = PAGEFMT_SPARE_48;
			spare_per_sector = 48;
			break;
		case 50:
		case 51:
			ecc_bit = 24;
			spare_bit = PAGEFMT_SPARE_50;
			spare_per_sector = 50;
			break;
		case 52:
		case 54:
		case 56:
			ecc_bit = 24;
			if (devinfo.sectorsize == 1024)
				spare_bit = PAGEFMT_SPARE_52_1KS;
			else
				spare_bit = PAGEFMT_SPARE_52;
			spare_per_sector = 52;
			break;
		case 62:
		case 63:
			ecc_bit = 28;
			spare_bit = PAGEFMT_SPARE_62;
			spare_per_sector = 62;
			break;
		case 64:
			ecc_bit = 32;
			if (devinfo.sectorsize == 1024)
				spare_bit = PAGEFMT_SPARE_64_1KS;
			else
				spare_bit = PAGEFMT_SPARE_64;
			spare_per_sector = 64;
			break;
		case 72:
			ecc_bit = 36;
			if (devinfo.sectorsize == 1024)
				spare_bit = PAGEFMT_SPARE_72_1KS;
			spare_per_sector = 72;
			break;
		case 80:
			ecc_bit = 40;
			if (devinfo.sectorsize == 1024)
				spare_bit = PAGEFMT_SPARE_80_1KS;
			spare_per_sector = 80;
			break;
		case 88:
			ecc_bit = 44;
			if (devinfo.sectorsize == 1024)
				spare_bit = PAGEFMT_SPARE_88_1KS;
			spare_per_sector = 88;
			break;
		case 96:
		case 98:
			ecc_bit = 48;
			if (devinfo.sectorsize == 1024)
				spare_bit = PAGEFMT_SPARE_96_1KS;
			spare_per_sector = 96;
			break;
		case 100:
		case 102:
		case 104:
			ecc_bit = 52;
			if (devinfo.sectorsize == 1024)
				spare_bit = PAGEFMT_SPARE_100_1KS;
			spare_per_sector = 100;
			break;
		case 124:
		case 126:
		case 128:
			ecc_bit = 60;
			if (devinfo.sectorsize == 1024)
				spare_bit = PAGEFMT_SPARE_124_1KS;
			spare_per_sector = 124;
			break;
		default:
			pr_notice("[NAND]: NFI not support oobsize: %x\n", spare_per_sector);
			mtk_nand_assert(0);
		}

		mtd->oobsize = spare_per_sector * (mtd->writesize / hw->nand_sec_size);
		pr_debug("[NAND]select ecc bit:%d, sparesize :%d\n", ecc_bit, mtd->oobsize);
		/* Setup PageFormat */
		if (mtk_nfi_dev_comp->chip_ver == 1) {
			if (mtd->writesize == 16384) {
				NFI_SET_REG16(NFI_PAGEFMT_REG16,
						  (spare_bit << PAGEFMT_SPARE_SHIFT_V1) | PAGEFMT_16K_1KS);
				nand->cmdfunc = mtk_nand_command_bp;
			} else if (mtd->writesize == 8192) {
				NFI_SET_REG16(NFI_PAGEFMT_REG16,
						  (spare_bit << PAGEFMT_SPARE_SHIFT_V1) | PAGEFMT_8K_1KS);
				nand->cmdfunc = mtk_nand_command_bp;
			} else if (mtd->writesize == 4096) {
				if (devinfo.sectorsize == 512)
					NFI_SET_REG16(NFI_PAGEFMT_REG16,
							  (spare_bit << PAGEFMT_SPARE_SHIFT_V1) | PAGEFMT_4K);
				else
					NFI_SET_REG16(NFI_PAGEFMT_REG16,
							  (spare_bit << PAGEFMT_SPARE_SHIFT_V1) | PAGEFMT_4K_1KS);
				nand->cmdfunc = mtk_nand_command_bp;
			} else if (mtd->writesize == 2048) {
				if (devinfo.sectorsize == 512)
					NFI_SET_REG16(NFI_PAGEFMT_REG16,
							  (spare_bit << PAGEFMT_SPARE_SHIFT_V1) | PAGEFMT_2K);
				else
					NFI_SET_REG16(NFI_PAGEFMT_REG16,
							  (spare_bit << PAGEFMT_SPARE_SHIFT_V1) | PAGEFMT_2K_1KS);
				nand->cmdfunc = mtk_nand_command_bp;
			}
		} else if (mtk_nfi_dev_comp->chip_ver == 2) {
			if (mtd->writesize == 16384) {
				NFI_SET_REG32(NFI_PAGEFMT_REG32,
						  (spare_bit << PAGEFMT_SPARE_SHIFT) | PAGEFMT_16K_1KS);
				nand->cmdfunc = mtk_nand_command_bp;
			} else if (mtd->writesize == 8192) {
				NFI_SET_REG32(NFI_PAGEFMT_REG32,
						  (spare_bit << PAGEFMT_SPARE_SHIFT) | PAGEFMT_8K_1KS);
				nand->cmdfunc = mtk_nand_command_bp;
			} else if (mtd->writesize == 4096) {
				if (devinfo.sectorsize == 512)
					NFI_SET_REG32(NFI_PAGEFMT_REG32,
							  (spare_bit << PAGEFMT_SPARE_SHIFT) | PAGEFMT_4K);
				else
					NFI_SET_REG32(NFI_PAGEFMT_REG32,
							  (spare_bit << PAGEFMT_SPARE_SHIFT) | PAGEFMT_4K_1KS);
				nand->cmdfunc = mtk_nand_command_bp;
			} else if (mtd->writesize == 2048) {
				if (devinfo.sectorsize == 512)
					NFI_SET_REG32(NFI_PAGEFMT_REG32,
							  (spare_bit << PAGEFMT_SPARE_SHIFT) | PAGEFMT_2K);
				else
					NFI_SET_REG32(NFI_PAGEFMT_REG32,
							  (spare_bit << PAGEFMT_SPARE_SHIFT) | PAGEFMT_2K_1KS);
				nand->cmdfunc = mtk_nand_command_bp;
			}
		} else {
			pr_err("[mtk_nand_select_chip] ERROR, mtk_nfi_dev_comp->chip_ver=%d\n",
				mtk_nfi_dev_comp->chip_ver);
		}
		ecc_threshold = ecc_bit * 4 / 5;
		ECC_Config(hw, ecc_bit);
		g_bInitDone = true;

		/* xiaolei for kernel3.10 */
		nand->ecc.strength = ecc_bit;
		mtd->bitflip_threshold = nand->ecc.strength;
	}
	switch (chip) {
	case -1:
		break;
	case 0:
		DRV_WriteReg16(NFI_CSEL_REG16, 0);
		break;
	case 1:
		DRV_WriteReg16(NFI_CSEL_REG16, chip);
		break;
	}
}

/******************************************************************************
 * mtk_nand_read_byte
 *
 * DESCRIPTION:
 *	 Read a byte of data !
 *
 * PARAMETERS:
 *	 struct mtd_info *mtd
 *
 * RETURNS:
 *	 None
 *
 * NOTES:
 *	 None
 *
 ******************************************************************************/
static uint8_t mtk_nand_read_byte(struct mtd_info *mtd)
{
	uint8_t retval = 0;

	if (!mtk_nand_pio_ready()) {
		pr_err("pio ready timeout\n");
		retval = false;
	}

	if (g_bcmdstatus) {
		retval = DRV_Reg8(NFI_DATAR_REG32);
		NFI_CLN_REG32(NFI_CON_REG16, CON_NFI_NOB_MASK);
		mtk_nand_reset();
		NFI_SET_REG16(NFI_CNFG_REG16, CNFG_AHB);
		NFI_SET_REG16(NFI_CNFG_REG16, CNFG_DMA_BURST_EN);
		if (g_bHwEcc)
			NFI_SET_REG16(NFI_CNFG_REG16, CNFG_HW_ECC_EN);
		else
			NFI_CLN_REG16(NFI_CNFG_REG16, CNFG_HW_ECC_EN);
		g_bcmdstatus = false;
	} else
		retval = DRV_Reg8(NFI_DATAR_REG32);

	return retval;
}

/******************************************************************************
 * mtk_nand_read_buf
 *
 * DESCRIPTION:
 *	 Read NAND data !
 *
 * PARAMETERS:
 *	 struct mtd_info *mtd, uint8_t *buf, int len
 *
 * RETURNS:
 *	 None
 *
 * NOTES:
 *	 None
 *
 ******************************************************************************/
static void mtk_nand_read_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
	struct nand_chip *nand = (struct nand_chip *)mtd->priv;
	struct NAND_CMD *pkCMD = &g_kCMD;
	u32 u4ColAddr = pkCMD->u4ColAddr;
	u32 u4PageSize = mtd->writesize;

	if (u4ColAddr < u4PageSize) {
		if ((u4ColAddr == 0) && (len >= u4PageSize)) {
			mtk_nand_exec_read_page(mtd, pkCMD->u4RowAddr, u4PageSize, buf,
						pkCMD->au1OOB);
			if (len > u4PageSize) {
				u32 u4Size = min(len - u4PageSize, (u32) (sizeof(pkCMD->au1OOB)));

				memcpy(buf + u4PageSize, pkCMD->au1OOB, u4Size);
			}
		} else {
			mtk_nand_exec_read_page(mtd, pkCMD->u4RowAddr, u4PageSize,
						nand->buffers->databuf, pkCMD->au1OOB);
			memcpy(buf, nand->buffers->databuf + u4ColAddr, len);
		}
		pkCMD->u4OOBRowAddr = pkCMD->u4RowAddr;
	} else {
		u32 u4Offset = u4ColAddr - u4PageSize;
		u32 u4Size = min(len - u4Offset, (u32) (sizeof(pkCMD->au1OOB)));

		if (pkCMD->u4OOBRowAddr != pkCMD->u4RowAddr) {
			mtk_nand_exec_read_page(mtd, pkCMD->u4RowAddr, u4PageSize,
						nand->buffers->databuf, pkCMD->au1OOB);
			pkCMD->u4OOBRowAddr = pkCMD->u4RowAddr;
		}
		memcpy(buf, pkCMD->au1OOB + u4Offset, u4Size);
	}
	pkCMD->u4ColAddr += len;
}

/******************************************************************************
 * mtk_nand_write_buf
 *
 * DESCRIPTION:
 *	 Write NAND data !
 *
 * PARAMETERS:
 *	 struct mtd_info *mtd, const uint8_t *buf, int len
 *
 * RETURNS:
 *	 None
 *
 * NOTES:
 *	 None
 *
 ******************************************************************************/
static void mtk_nand_write_buf(struct mtd_info *mtd, const uint8_t *buf, int len)
{
	struct NAND_CMD *pkCMD = &g_kCMD;
	u32 u4ColAddr = pkCMD->u4ColAddr;
	u32 u4PageSize = mtd->writesize;
	int i4Size, i;

	if (u4ColAddr >= u4PageSize) {
		u32 u4Offset = u4ColAddr - u4PageSize;
		u8 *pOOB = pkCMD->au1OOB + u4Offset;

		i4Size = min(len, (int)(sizeof(pkCMD->au1OOB) - u4Offset));

		for (i = 0; i < i4Size; i++)
			pOOB[i] &= buf[i];
	} else {
		pkCMD->pDataBuf = (u8 *) buf;
	}

	pkCMD->u4ColAddr += len;
}

/******************************************************************************
 * mtk_nand_write_page_hwecc
 *
 * DESCRIPTION:
 *	 Write NAND data with hardware ecc !
 *
 * PARAMETERS:
 *	 struct mtd_info *mtd, struct nand_chip *chip, const uint8_t *buf
 *
 * RETURNS:
 *	 None
 *
 * NOTES:
 *	 None
 *
 ******************************************************************************/
static int mtk_nand_write_page_hwecc(struct mtd_info *mtd, struct nand_chip *chip,
					 const uint8_t *buf, int oob_required)
{
	mtk_nand_write_buf(mtd, buf, mtd->writesize);
	mtk_nand_write_buf(mtd, chip->oob_poi, mtd->oobsize);
	return 0;
}

/******************************************************************************
 * mtk_nand_read_page_hwecc
 *
 * DESCRIPTION:
 *	 Read NAND data with hardware ecc !
 *
 * PARAMETERS:
 *	 struct mtd_info *mtd, struct nand_chip *chip, uint8_t *buf
 *
 * RETURNS:
 *	 None
 *
 * NOTES:
 *	 None
 *
 ******************************************************************************/
static int mtk_nand_read_page_hwecc(struct mtd_info *mtd, struct nand_chip *chip, uint8_t *buf,
					int oob_required, int page)
{
	struct NAND_CMD *pkCMD = &g_kCMD;
	u32 u4ColAddr = pkCMD->u4ColAddr;
	u32 u4PageSize = mtd->writesize;

	if (u4ColAddr == 0) {
		mtk_nand_exec_read_page(mtd, pkCMD->u4RowAddr, u4PageSize, buf, chip->oob_poi);
		pkCMD->u4ColAddr += u4PageSize + mtd->oobsize;
	}

	return 0;
}

/******************************************************************************
 *
 * Read a page to a logical address
 *
 *****************************************************************************/
static int mtk_nand_read_page(struct mtd_info *mtd, struct nand_chip *chip, u8 *buf, int page)
{
	/* int block_size = 1 << (chip->phys_erase_shift); */
	int page_per_block = 1 << (chip->phys_erase_shift - chip->page_shift);
	/* int page_per_block1 = page_per_block; */
	u32 block;
	u32 page_in_block;
	u32 mapped_block;
	int bRet = ERR_RTN_SUCCESS;

	page_in_block = mtk_nand_page_transform(mtd, chip, page, &block, &mapped_block);
	/* MSG(INIT,"[READ] %d, %d, %d %d\n",mapped_block, block, page_in_block, page_per_block); */

	/* pr_debug("[xl] mtk_nand_read_page buf 0x%lx\n", (unsigned long)buf); */

	bRet =
		mtk_nand_exec_read_page(mtd, page_in_block + mapped_block * page_per_block,
					mtd->writesize, buf, chip->oob_poi);
	if (bRet == ERR_RTN_SUCCESS)
		return 0;
	return 0;
}

static int mtk_nand_read_subpage(struct mtd_info *mtd, struct nand_chip *chip, u8 *buf, int page,
				 int subpage, int subpageno)
{
	/* int block_size = 1 << (chip->phys_erase_shift); */
	int page_per_block = 1 << (chip->phys_erase_shift - chip->page_shift);
	/* int page_per_block1 = page_per_block; */
	u32 block;
	int coladdr;
	u32 page_in_block;
	u32 mapped_block;
	/* bool readRetry = FALSE; */
	/* int retryCount = 0; */
	int bRet = ERR_RTN_SUCCESS;
	int sec_num = 1 << (chip->page_shift - host->hw->nand_sec_shift);
	int spare_per_sector = mtd->oobsize / sec_num;

	page_in_block = mtk_nand_page_transform(mtd, chip, page, &block, &mapped_block);
	coladdr = subpage * (devinfo.sectorsize + spare_per_sector);
	/* coladdr = subpage*(devinfo.sectorsize); */
	/* MSG(INIT,"[Read Subpage] %d, %d, %d %d\n",mapped_block, block, page_in_block, page_per_block); */

	bRet = mtk_nand_exec_read_sector(mtd, page_in_block + mapped_block * page_per_block, coladdr,
					  devinfo.sectorsize * subpageno, buf, chip->oob_poi,
					  subpageno);
	/* memset(bean_buffer, 0xFF, LPAGE); */
	/* bRet = mtk_nand_exec_read_page(mtd, page, mtd->writesize, bean_buffer, chip->oob_poi); */
	if (bRet == ERR_RTN_SUCCESS)
		return 0;
	/* memcpy(buf, bean_buffer+coladdr, mtd->writesize); */
	return 0;
}


/******************************************************************************
 *
 * Erase a block at a logical address
 *
 *****************************************************************************/
int mtk_nand_erase_hw(struct mtd_info *mtd, int page)
{
	int result;
	struct nand_chip *chip = (struct nand_chip *)mtd->priv;

	if (g_bTricky_CS)
		page = mtk_nand_cs_on(chip, NFI_TRICKY_CS, page);
	result = chip->erase(mtd, page);
	return result;
}

static int mtk_nand_erase(struct mtd_info *mtd, int page)
{
	int status;
	struct nand_chip *chip = mtd->priv;
	/* int block_size = 1 << (chip->phys_erase_shift); */
	int page_per_block = 1 << (chip->phys_erase_shift - chip->page_shift);
	u32 block;
	u32 page_in_block;
	u32 mapped_block;

	page_in_block = mtk_nand_page_transform(mtd, chip, page, &block, &mapped_block);
	/* MSG(INIT, "[ERASE] 0x%x 0x%x\n", mapped_block, page); */
	status = mtk_nand_erase_hw(mtd, page_in_block + page_per_block * mapped_block);

	if (status & NAND_STATUS_FAIL) {
		pr_notice("Erase fail at block: 0x%x, update BMT fail\n", mapped_block);
		return NAND_STATUS_FAIL;
	}

	return 0;
}

/******************************************************************************
 * mtk_nand_read_oob_raw
 *
 * DESCRIPTION:
 *	 Read oob data
 *
 * PARAMETERS:
 *	 struct mtd_info *mtd, const uint8_t *buf, int addr, int len
 *
 * RETURNS:
 *	 None
 *
 * NOTES:
 *	 this function read raw oob data out of flash, so need to re-organise
 *	 data format before using.
 *	 len should be times of 8, call this after nand_get_device.
 *	 Should notice, this function read data without ECC protection.
 *
 *****************************************************************************/
static int mtk_nand_read_oob_raw(struct mtd_info *mtd, uint8_t *buf, int page_addr, int len)
{
	struct nand_chip *chip = (struct nand_chip *)mtd->priv;
	u32 col_addr = 0;
	u32 sector = 0;
	int res = 0;
	u32 colnob = 2, rawnob = devinfo.addr_cycle - 2;
	int randomread = 0;
	int read_len = 0;
	int sec_num = 1 << (chip->page_shift - host->hw->nand_sec_shift);
	int spare_per_sector = mtd->oobsize / sec_num;
	u32 sector_size = NAND_SECTOR_SIZE;

	if (devinfo.sectorsize == 1024)
		sector_size = 1024;

	if (len > NAND_MAX_OOBSIZE || len % OOB_AVAI_PER_SECTOR || !buf) {
		pr_warn("[%s] invalid parameter, len: %d, buf: %p\n", __func__, len,
			   buf);
		return -EINVAL;
	}
	if (len > spare_per_sector)
		randomread = 1;

	if (!randomread || !(devinfo.advancedmode & RAMDOM_READ)) {
		while (len > 0) {
			read_len = min(len, spare_per_sector);
			col_addr = sector_size +
				sector * (sector_size + spare_per_sector);	/* TODO: Fix this hard-code 16 */
			if (!mtk_nand_ready_for_read(chip,
					page_addr, col_addr, sec_num, false, NULL)) {
				pr_warn("mtk_nand_ready_for_read return failed\n");
				res = -EIO;
				goto error;
			}
			if (!mtk_nand_mcu_read_data(mtd,
					buf + spare_per_sector * sector, read_len)) {	/* TODO: and this 8 */
				pr_warn("mtk_nand_mcu_read_data return failed\n");
				res = -EIO;
				goto error;
			}
			mtk_nand_stop_read();
			/* dump_data(buf + 16 * sector,16); */
			sector++;
			len -= read_len;

		}
	} else {		/* should be 64 */

		col_addr = sector_size;
		if (chip->options & NAND_BUSWIDTH_16)
			col_addr /= 2;

		if (!mtk_nand_reset())
			goto error;

		mtk_nand_set_mode(0x6000);
		NFI_SET_REG16(NFI_CNFG_REG16, CNFG_READ_EN);
		DRV_WriteReg32(NFI_CON_REG16, 4 << CON_NFI_SEC_SHIFT);

		NFI_CLN_REG16(NFI_CNFG_REG16, CNFG_AHB);
		NFI_CLN_REG16(NFI_CNFG_REG16, CNFG_HW_ECC_EN);

		mtk_nand_set_autoformat(false);

		if (!mtk_nand_set_command(NAND_CMD_READ0))
			goto error;
		/* 1 FIXED ME: For Any Kind of AddrCycle */
		if (!mtk_nand_set_address(col_addr, page_addr, colnob, rawnob))
			goto error;

		if (!mtk_nand_set_command(NAND_CMD_READSTART))
			goto error;
		if (!mtk_nand_status_ready(STA_NAND_BUSY))
			goto error;

		read_len = min(len, spare_per_sector);
		if (!mtk_nand_mcu_read_data(mtd, buf + spare_per_sector * sector, read_len)) {	/* TODO: and this 8 */
			pr_warn("mtk_nand_mcu_read_data return failed first 16\n");
			res = -EIO;
			goto error;
		}
		sector++;
		len -= read_len;
		mtk_nand_stop_read();
		while (len > 0) {
			read_len = min(len, spare_per_sector);
			if (!mtk_nand_set_command(0x05))
				goto error;

			col_addr = sector_size + sector * (sector_size + 16);	/* :TODO_JP careful 16 */
			if (chip->options & NAND_BUSWIDTH_16)
				col_addr /= 2;
			DRV_WriteReg32(NFI_COLADDR_REG32, col_addr);
			DRV_WriteReg16(NFI_ADDRNOB_REG16, 2);
			DRV_WriteReg32(NFI_CON_REG16, 4 << CON_NFI_SEC_SHIFT);

			if (!mtk_nand_status_ready(STA_ADDR_STATE))
				goto error;

			if (!mtk_nand_set_command(0xE0))
				goto error;
			if (!mtk_nand_status_ready(STA_NAND_BUSY))
				goto error;
			if (!mtk_nand_mcu_read_data(mtd,
					buf + spare_per_sector * sector, read_len)) {	/* TODO: and this 8 */
				pr_warn("mtk_nand_mcu_read_data return failed first 16\n");
				res = -EIO;
				goto error;
			}
			mtk_nand_stop_read();
			sector++;
			len -= read_len;
		}
		/* dump_data(&testbuf[16],16); */
		/* pr_debug(KERN_ERR "\n"); */
	}
error:
	NFI_CLN_REG32(NFI_CON_REG16, CON_NFI_BRD);
	return res;
}

static int mtk_nand_write_oob_raw(struct mtd_info *mtd, const uint8_t *buf, int page_addr, int len)
{
	struct nand_chip *chip = mtd->priv;
	/* int i; */
	u32 col_addr = 0;
	u32 sector = 0;
	/* int res = 0; */
	/* u32 colnob=2, rawnob=devinfo.addr_cycle-2; */
	/* int randomread =0; */
	int write_len = 0;
	int status;
	int sec_num = 1 << (chip->page_shift - host->hw->nand_sec_shift);
	int spare_per_sector = mtd->oobsize / sec_num;
	u32 sector_size = NAND_SECTOR_SIZE;

	if (devinfo.sectorsize == 1024)
		sector_size = 1024;

	if (len > NAND_MAX_OOBSIZE || len % OOB_AVAI_PER_SECTOR || !buf) {
		pr_warn("[%s] invalid parameter, len: %d, buf: %p\n", __func__, len,
			   buf);
		return -EINVAL;
	}

	while (len > 0) {
		write_len = min(len, spare_per_sector);
		col_addr = sector * (sector_size + spare_per_sector) + sector_size;
		if (!mtk_nand_ready_for_write(chip, page_addr, col_addr, false, NULL))
			return -EIO;

		if (!mtk_nand_mcu_write_data(mtd, buf + sector * spare_per_sector, write_len))
			return -EIO;

		(void)mtk_nand_check_RW_count(write_len);
		NFI_CLN_REG32(NFI_CON_REG16, CON_NFI_BWR);
		(void)mtk_nand_set_command(NAND_CMD_PAGEPROG);

		while (DRV_Reg32(NFI_STA_REG32) & STA_NAND_BUSY)
			;

		status = chip->waitfunc(mtd, chip);
		if (status & NAND_STATUS_FAIL) {
			pr_debug("status: %d\n", status);
			return -EIO;
		}

		len -= write_len;
		sector++;
	}

	return 0;
}

static int mtk_nand_write_oob_hw(struct mtd_info *mtd, struct nand_chip *chip, int page)
{
	/* u8 *buf = chip->oob_poi; */
	int i, iter;

	int sec_num = 1 << (chip->page_shift - host->hw->nand_sec_shift);
	int spare_per_sector = mtd->oobsize / sec_num;

	memcpy(local_oob_buf, chip->oob_poi, mtd->oobsize);

	/* copy ecc data */
	for (i = 0; i < chip->ecc.layout->eccbytes; i++) {
		iter = (i / OOB_AVAI_PER_SECTOR) * spare_per_sector + OOB_AVAI_PER_SECTOR +
			i % OOB_AVAI_PER_SECTOR;
		local_oob_buf[iter] = chip->oob_poi[chip->ecc.layout->eccpos[i]];
		/* chip->oob_poi[chip->ecc.layout->eccpos[i]] = local_oob_buf[iter]; */
	}

	/* copy FDM data */
	for (i = 0; i < sec_num; i++) {
		memcpy(&local_oob_buf[i * spare_per_sector],
			   &chip->oob_poi[i * OOB_AVAI_PER_SECTOR], OOB_AVAI_PER_SECTOR);
	}

	return mtk_nand_write_oob_raw(mtd, local_oob_buf, page, mtd->oobsize);
}

static int mtk_nand_write_oob(struct mtd_info *mtd, struct nand_chip *chip, int page)
{
	/* int block_size = 1 << (chip->phys_erase_shift); */
	int page_per_block = 1 << (chip->phys_erase_shift - chip->page_shift);
	/* int page_per_block1 = page_per_block; */
	u32 block;
	u16 page_in_block;
	u32 mapped_block;

	/* block = page / page_per_block1; */
	/* mapped_block = get_mapping_block_index(block); */
	page_in_block = mtk_nand_page_transform(mtd, chip, page, &block, &mapped_block);

	if (mtk_nand_write_oob_hw(mtd, chip, page_in_block + mapped_block * page_per_block /* page */)) {
		pr_err("write oob fail at block: 0x%x, page: 0x%x\n", mapped_block,
			page_in_block);
		return -EIO;
	}

	return 0;
}

int mtk_nand_block_markbad_hw(struct mtd_info *mtd, loff_t offset)
{
	struct nand_chip *chip = mtd->priv;
	int block = (int)(offset >> chip->phys_erase_shift);
	int page = block * (1 << (chip->phys_erase_shift - chip->page_shift));
	int ret;

	u8 buf[8];

	memset(buf, 0xFF, 8);
	buf[0] = 0;

	ret = mtk_nand_write_oob_raw(mtd, buf, page, 8);
	return ret;
}

static int mtk_nand_block_markbad(struct mtd_info *mtd, loff_t offset)
{
	struct nand_chip *chip = mtd->priv;
	u32 block = (u32) (offset >> chip->phys_erase_shift);
	int page = block * (1 << (chip->phys_erase_shift - chip->page_shift));
	u32 mapped_block;
	int ret;

	/* mapped_block = get_mapping_block_index(block); */
	page = mtk_nand_page_transform(mtd, chip, page, &block, &mapped_block);
	ret = mtk_nand_block_markbad_hw(mtd, mapped_block << chip->phys_erase_shift);
	return ret;
}

int mtk_nand_read_oob_hw(struct mtd_info *mtd, struct nand_chip *chip, int page)
{
	int i;
	u8 iter = 0;

	int sec_num = 1 << (chip->page_shift - host->hw->nand_sec_shift);
	int spare_per_sector = mtd->oobsize / sec_num;

	if (mtk_nand_read_oob_raw(mtd, chip->oob_poi, page, mtd->oobsize)) {
		/* pr_debug(KERN_ERR "[%s]mtk_nand_read_oob_raw return failed\n", __FUNCTION__); */
		return -EIO;
	}

	/* adjust to ecc physical layout to memory layout */
	/*********************************************************/
	/* FDM0 | ECC0 | FDM1 | ECC1 | FDM2 | ECC2 | FDM3 | ECC3 */
	/*	8B	|  8B  |  8B  |  8B  |	8B	|  8B  |  8B  |  8B  */
	/*********************************************************/

	memcpy(local_oob_buf, chip->oob_poi, mtd->oobsize);

	/* copy ecc data */
	for (i = 0; i < chip->ecc.layout->eccbytes; i++) {
		iter = (i / OOB_AVAI_PER_SECTOR) * spare_per_sector + OOB_AVAI_PER_SECTOR +
			i % OOB_AVAI_PER_SECTOR;
		chip->oob_poi[chip->ecc.layout->eccpos[i]] = local_oob_buf[iter];
	}

	/* copy FDM data */
	for (i = 0; i < sec_num; i++) {
		memcpy(&chip->oob_poi[i * OOB_AVAI_PER_SECTOR],
			   &local_oob_buf[i * spare_per_sector], OOB_AVAI_PER_SECTOR);
	}

	return 0;
}

static int mtk_nand_read_oob(struct mtd_info *mtd, struct nand_chip *chip, int page)
{
	/* int block_size = 1 << (chip->phys_erase_shift); */
	/* int page_per_block = 1 << (chip->phys_erase_shift - chip->page_shift); */
	/* int block; */
	/* u16 page_in_block; */
	/* int mapped_block; */
	/* u8* buf = (u8*)kzalloc(mtd->writesize, GFP_KERNEL); */

	/* page = mtk_nand_page_transform(mtd,chip,page,&block,&mapped_block); */

	mtk_nand_read_page(mtd, chip, temp_buffer_16_align, page);
	/* kfree(buf); */

	return 0;		/* the return value is sndcmd */
}

int mtk_nand_block_bad_hw(struct mtd_info *mtd, loff_t ofs)
{
	struct nand_chip *chip = (struct nand_chip *)mtd->priv;
	int page_addr = (int)(ofs >> chip->page_shift);
	u32 block, mapped_block;
	int ret;
	unsigned int page_per_block = 1 << (chip->phys_erase_shift - chip->page_shift);

	/* unsigned char oob_buf[128]; */
	/* char* buf = (char*) kmalloc(mtd->writesize + mtd->oobsize, GFP_KERNEL); */

	/* page_addr = mtk_nand_page_transform(mtd, chip, page_addr, &block, &mapped_block); */

	page_addr &= ~(page_per_block - 1);

	/* ret = mtk_nand_read_page(mtd,chip,buf,(ofs >> chip->page_shift)); */
	memset(temp_buffer_16_align, 0xFF, LPAGE);
	ret = mtk_nand_read_subpage(mtd, chip, temp_buffer_16_align, (ofs >> chip->page_shift), 0, 1);
	page_addr = mtk_nand_page_transform(mtd, chip, page_addr, &block, &mapped_block);
	/* ret = mtk_nand_exec_read_page(mtd, page_addr+mapped_block*page_per_block, mtd->writesize, buf, oob_buf); */
	if (ret != 0) {
		pr_warn("mtk_nand_read_oob_raw return error %d\n", ret);
	/* kfree(buf); */
		return 1;
	}

	if (chip->oob_poi[0] != 0xff) {
		pr_debug("Bad block detected at 0x%x, oob_buf[0] is 0x%x\n",
			   block * page_per_block, chip->oob_poi[0]);
		/* kfree(buf); */
		/* dump_nfi(); */
		return 1;
	}
	/* kfree(buf); */
	return 0;		/* everything is OK, good block */
}

static int mtk_nand_block_bad(struct mtd_info *mtd, loff_t ofs, int getchip)
{
	int chipnr = 0;

	struct nand_chip *chip = (struct nand_chip *)mtd->priv;
	int block = (int)(ofs >> chip->phys_erase_shift);
	int mapped_block;
	int page = (int)(ofs >> chip->page_shift);
	int page_in_block;
	int page_per_block = 1 << (chip->phys_erase_shift - chip->page_shift);

	int ret;

	if (getchip) {
		chipnr = (int)(ofs >> chip->chip_shift);
		/* Select the NAND device */
		chip->select_chip(mtd, chipnr);
	}
	/* page = mtk_nand_page_transform(mtd, chip, page, &block, &mapped_block); */
	/* mapped_block = get_mapping_block_index(block); */

	ret = mtk_nand_block_bad_hw(mtd, ofs);
	page_in_block = mtk_nand_page_transform(mtd, chip, page, &block, &mapped_block);

	if (ret)
		pr_debug("Unmapped bad block: 0x%x %d\n", mapped_block, ret);

	return ret;
}

/******************************************************************************
 * mtk_nand_init_size
 *
 * DESCRIPTION:
 *	 initialize the pagesize, oobsize, blocksize
 *
 * PARAMETERS:
 *	 struct mtd_info *mtd, struct nand_chip *this, u8 *id_data
 *
 * RETURNS:
 *	 Buswidth
 *
 * NOTES:
 *	 None
 *
 ******************************************************************************/

static int mtk_nand_init_size(struct mtd_info *mtd, struct nand_chip *this, u8 *id_data)
{
	/* Get page size */
	mtd->writesize = devinfo.pagesize;

	/* Get oobsize */
	mtd->oobsize = devinfo.sparesize;

	/* Get blocksize. */
	mtd->erasesize = devinfo.blocksize * 1024;
	/* Get buswidth information */
	if (devinfo.iowidth == 16)
		return NAND_BUSWIDTH_16;
	else
		return 0;
}

/******************************************************************************
 * mtk_nand_init_hw
 *
 * DESCRIPTION:
 *	 Initial NAND device hardware component !
 *
 * PARAMETERS:
 *	 struct mtk_nand_host *host (Initial setting data)
 *
 * RETURNS:
 *	 None
 *
 * NOTES:
 *	 None
 *
 ******************************************************************************/
static void mtk_nand_init_hw(struct mtk_nand_host *host)
{
	struct mtk_nand_host_hw *hw = host->hw;


	g_bInitDone = false;
	g_kCMD.u4OOBRowAddr = (u32) -1;

	/* Set default NFI access timing control */
	DRV_WriteReg32(NFI_ACCCON_REG32, 0x10804333/*hw->nfi_access_timing*/);
	pr_err("ACCCON = 0x%x\n", DRV_Reg32(NFI_ACCCON_REG32));
	DRV_WriteReg16(NFI_CNFG_REG16, 0);
	if (mtk_nfi_dev_comp->chip_ver == 1) {
		DRV_WriteReg16(NFI_PAGEFMT_REG16, 4);
	} else if (mtk_nfi_dev_comp->chip_ver == 2) {
		DRV_WriteReg32(NFI_PAGEFMT_REG32, 4);
	} else {
		pr_err("[mtk_nand_init_hw] ERROR, mtk_nfi_dev_comp->chip_ver=%d\n",
			mtk_nfi_dev_comp->chip_ver);
	}
	DRV_WriteReg32(NFI_ENMPTY_THRESH_REG32, 40);

	/* Reset the state machine and data FIFO, because flushing FIFO */
	(void)mtk_nand_reset();

	/* Set the ECC engine */
	if (hw->nand_ecc_mode == NAND_ECC_HW) {
		pr_notice("Use HW ECC\n");
		if (g_bHwEcc)
			NFI_SET_REG32(NFI_CNFG_REG16, CNFG_HW_ECC_EN);

		ECC_Config(host->hw, 4);
		mtk_nand_configure_fdm(8);
	}

	/* Initialize interrupt. Clear interrupt, read clear. */
	DRV_Reg16(NFI_INTR_REG16);

	/* Interrupt arise when read data or program data to/from AHB is done. */
	DRV_WriteReg16(NFI_INTR_EN_REG16, 0);

	/* Enable automatic disable ECC clock when NFI is busy state */
	DRV_WriteReg16(NFI_DEBUG_CON1_REG16, (NFI_BYPASS | WBUF_EN | HWDCM_SWCON_ON));

	host->saved_para.suspend_flag = 0;
}

/* ------------------------------------------------------------------------------- */
static int mtk_nand_dev_ready(struct mtd_info *mtd)
{
	return !(DRV_Reg32(NFI_STA_REG32) & STA_NAND_BUSY);
}

/******************************************************************************
 * mtk_nand_proc_read
 *
 * DESCRIPTION:
 *	 Read the proc file to get the interrupt scheme setting !
 *
 * PARAMETERS:
 *	 char *page, char **start, off_t off, int count, int *eof, void *data
 *
 * RETURNS:
 *	 None
 *
 * NOTES:
 *	 None
 *
 ******************************************************************************/
int mtk_nand_proc_read(struct file *file, char *buffer, size_t count, loff_t *ppos)
{
	char *p = buffer;
	int len = 0;
	int i;

	p += sprintf(p, "ID:");
	for (i = 0; i < devinfo.id_length; i++)
		p += sprintf(p, " 0x%x", devinfo.id[i]);

	p += sprintf(p, "\n");
	p += sprintf(p, "total size: %dMiB; part number: %s\n", devinfo.totalsize,
			 devinfo.devciename);
	p += sprintf(p, "Current working in %s mode\n", g_i4Interrupt ? "interrupt" : "polling");
	len = p - buffer;

	return len < count ? len : count;
}

/******************************************************************************
 * mtk_nand_proc_write
 *
 * DESCRIPTION:
 *	 Write the proc file to set the interrupt scheme !
 *
 * PARAMETERS:
 *	 struct file* file, const char* buffer,	unsigned long count, void *data
 *
 * RETURNS:
 *	 None
 *
 * NOTES:
 *	 None
 *
 ******************************************************************************/
ssize_t mtk_nand_proc_write(struct file *file, const char __user *buffer, size_t count,
				loff_t *data)
{
	struct mtd_info *mtd = &host->mtd;
	char buf[16];
	char cmd;
	int value;
	int len = count;	/* , n; */

	if (len >= sizeof(buf))
		len = sizeof(buf) - 1;

	if (copy_from_user(buf, buffer, len))
		return -EFAULT;

	if (sscanf(buf, "%c%x", &cmd, &value) != 2)
		return -EINVAL;

	switch (cmd) {
	case 'A':		/* NFIA driving setting */
		break;
	case 'B':		/* NFIB driving setting */
		break;
	case 'D':
		break;
	case 'I':		/* Interrupt control */
		if ((value > 0 && !g_i4Interrupt) || (value == 0 && g_i4Interrupt)) {
			g_i4Interrupt = value;

			if (g_i4Interrupt) {
				DRV_Reg16(NFI_INTR_REG16);
				enable_irq(MT_NFI_IRQ_ID);
			} else
				disable_irq(MT_NFI_IRQ_ID);
		}
		break;
	case 'P':		/* Reset Performance monitor counter */
		break;
	case 'R':		/* Reset NFI performance log */
		break;
	case 'T':		/* ACCCON Setting */
		DRV_WriteReg32(NFI_ACCCON_REG32, value);
		break;
	default:
		break;
	}

	return len;
}

static int mtk_nand_cs_check(struct mtd_info *mtd, u8 *id, u16 cs)
{
	u8 ids[NAND_MAX_ID];
	int i = 0;
	/* if(devinfo.ttarget == TTYPE_2DIE) */
	/* { */
	/* MSG(INIT,"2 Die Flash\n"); */
	/* g_bTricky_CS = TRUE; */
	/* return 0; */
	/* } */
	DRV_WriteReg16(NFI_CSEL_REG16, cs);
	mtk_nand_command_bp(mtd, NAND_CMD_READID, 0, -1);
	for (i = 0; i < NAND_MAX_ID; i++) {
		ids[i] = mtk_nand_read_byte(mtd);
		if (ids[i] != id[i]) {
			pr_notice("Nand cs[%d] not support(%d,%x)\n", cs, i, ids[i]);
			DRV_WriteReg16(NFI_CSEL_REG16, NFI_DEFAULT_CS);

			return 0;
		}
	}
	DRV_WriteReg16(NFI_CSEL_REG16, NFI_DEFAULT_CS);
	return 1;
}

static u32 mtk_nand_cs_on(struct nand_chip *nand_chip, u16 cs, u32 page)
{
	u32 cs_page = page / g_nanddie_pages;

	if (cs_page) {
		DRV_WriteReg16(NFI_CSEL_REG16, cs);
		/* if(devinfo.ttarget == TTYPE_2DIE) */
		/* return page;//return (page | CHIP_ADDRESS); */
		return (page - g_nanddie_pages);
	}
	DRV_WriteReg16(NFI_CSEL_REG16, NFI_DEFAULT_CS);
	return page;
}

static int mtk_nand_probe(struct platform_device *pdev)
{

	struct mtk_nand_host_hw *hw;
	struct mtd_info *mtd;
	struct nand_chip *nand_chip;
	/*struct resource *res = pdev->resource; */
	int err = 0;
	 int ret = 0;
	u8 id[NAND_MAX_ID];
	int i;
	u32 sector_size = NAND_SECTOR_SIZE;
	int bmt_sz = 0;

	const struct of_device_id *of_id;

	of_id = of_match_node(mtk_nfi_of_match, pdev->dev.of_node);
	if (!of_id)
		return -EINVAL;

	mtk_nfi_dev_comp = of_id->data;
	/* dt modify */
	mtk_nfi_base = of_iomap(pdev->dev.of_node, 0);
	pr_err("of_iomap for nfi base @ 0x%p\n", mtk_nfi_base);

	if (mtk_nfiecc_node == NULL) {
		if (mtk_nfi_dev_comp->chip_ver == 1)
			mtk_nfiecc_node = of_find_compatible_node(NULL, NULL, "mediatek,mt2701-nfiecc");
		else if (mtk_nfi_dev_comp->chip_ver == 2)
			mtk_nfiecc_node = of_find_compatible_node(NULL, NULL, "mediatek,mt8163-nfiecc");
		mtk_nfiecc_base = of_iomap(mtk_nfiecc_node, 0);
		pr_err("of_iomap for nfiecc base @ 0x%p\n", mtk_nfiecc_base);
	}
	nfi_irq = irq_of_parse_and_map(pdev->dev.of_node, 0);

	if (mtk_gpio_node == NULL) {
		/* mtk_gpio_node = of_find_compatible_node(NULL, NULL, "mediatek,GPIO"); */
		if (mtk_nfi_dev_comp->chip_ver == 1)
			mtk_gpio_node = of_find_compatible_node(NULL, NULL, "mediatek,mt2701-pinctrl");
		else if (mtk_nfi_dev_comp->chip_ver == 2)
			mtk_gpio_node = of_find_compatible_node(NULL, NULL, "mediatek,mt8163-pctl-a-syscfg");
		mtk_gpio_base = of_iomap(mtk_gpio_node, 0);
		pr_debug("of_iomap for gpio base @ 0x%p\n", mtk_gpio_base);
	}

	if (mtk_nfi_dev_comp->chip_ver == 1) {
		nfi_hclk = devm_clk_get(&pdev->dev, "nfi_ck");
		BUG_ON(IS_ERR(nfi_hclk));
		nfiecc_bclk = devm_clk_get(&pdev->dev, "nfi_ecc_ck");
		BUG_ON(IS_ERR(nfiecc_bclk));
		nfi_bclk = devm_clk_get(&pdev->dev, "nfi_pad_ck");
		BUG_ON(IS_ERR(nfi_bclk));
	} else if (mtk_nfi_dev_comp->chip_ver == 2) {
		nfi_hclk = devm_clk_get(&pdev->dev, "nfi_hclk");
		BUG_ON(IS_ERR(nfi_hclk));
		nfiecc_bclk = devm_clk_get(&pdev->dev, "nfiecc_bclk");
		BUG_ON(IS_ERR(nfiecc_bclk));
		nfi_bclk = devm_clk_get(&pdev->dev, "nfi_bclk");
		BUG_ON(IS_ERR(nfi_bclk));
		onfi_sel_clk = devm_clk_get(&pdev->dev, "onfi_sel");
		BUG_ON(IS_ERR(onfi_sel_clk));
		onfi_26m_clk = devm_clk_get(&pdev->dev, "onfi_clk26m");
		BUG_ON(IS_ERR(onfi_26m_clk));
		onfi_mode5 = devm_clk_get(&pdev->dev, "onfi_mode5");
		BUG_ON(IS_ERR(onfi_mode5));
		onfi_mode4 = devm_clk_get(&pdev->dev, "onfi_mode4");
		BUG_ON(IS_ERR(onfi_mode4));
		nfi_bclk_sel = devm_clk_get(&pdev->dev, "nfi_bclk_sel");
		BUG_ON(IS_ERR(nfi_bclk_sel));
		nfi_ahb_clk = devm_clk_get(&pdev->dev, "nfi_ahb_clk");
		BUG_ON(IS_ERR(nfi_ahb_clk));
		nfi_1xpad_clk = devm_clk_get(&pdev->dev, "nfi_1xpad_clk");
		BUG_ON(IS_ERR(nfi_1xpad_clk));
		nfi_ecc_pclk = devm_clk_get(&pdev->dev, "nfiecc_pclk");
		BUG_ON(IS_ERR(nfi_ecc_pclk));
		nfi_pclk = devm_clk_get(&pdev->dev, "nfi_pclk");
		BUG_ON(IS_ERR(nfi_pclk));
		onfi_pad_clk = devm_clk_get(&pdev->dev, "onfi_pad_clk");
		BUG_ON(IS_ERR(onfi_pad_clk));
		mtk_nand_regulator = devm_regulator_get(&pdev->dev, "vmch");
		BUG_ON(IS_ERR(mtk_nand_regulator));
	}

	hw = (struct mtk_nand_host_hw *)pdev->dev.platform_data;
	hw = devm_kzalloc(&pdev->dev, sizeof(*hw), GFP_KERNEL);
	BUG_ON(!hw);
	hw->nfi_bus_width = 8;
	hw->nfi_access_timing = 0x4333;
	hw->nfi_cs_num = 2;
	hw->nand_sec_size = 512;
	hw->nand_sec_shift = 9;
	hw->nand_ecc_size = 2048;
	hw->nand_ecc_bytes = 32;
	hw->nand_ecc_mode = 2;

	/* Allocate memory for the device structure (and zero it) */
	host = kzalloc(sizeof(struct mtk_nand_host), GFP_KERNEL);
	if (!host) {
		/* pr_err("failed to allocate device structure.\n"); */
		return -ENOMEM;
	}

	/* Allocate memory for 16 byte aligned buffer */
	local_buffer_16_align = local_buffer;
	temp_buffer_16_align = temp_buffer;
	/* pr_debug(KERN_INFO "Allocate 16 byte aligned buffer: %p\n", local_buffer_16_align); */

	host->hw = hw;

	/* init mtd data structure */
	nand_chip = &host->nand_chip;
	nand_chip->priv = host;	/* link the private data structures */

	mtd = &host->mtd;
	mtd->priv = nand_chip;
	mtd->owner = THIS_MODULE;
	mtd->name = "MTK-Nand";
	mtd->eraseregions = host->erase_region;

	hw->nand_ecc_mode = NAND_ECC_HW;

	/* Set address of NAND IO lines */
	nand_chip->IO_ADDR_R = (void __iomem *)NFI_DATAR_REG32;
	nand_chip->IO_ADDR_W = (void __iomem *)NFI_DATAW_REG32;
	nand_chip->chip_delay = 20;	/* 20us command delay time */
	nand_chip->ecc.mode = hw->nand_ecc_mode;	/* enable ECC */

	nand_chip->read_byte = mtk_nand_read_byte;
	nand_chip->read_buf = mtk_nand_read_buf;
	nand_chip->write_buf = mtk_nand_write_buf;
	nand_chip->select_chip = mtk_nand_select_chip;
	nand_chip->dev_ready = mtk_nand_dev_ready;
	nand_chip->cmdfunc = mtk_nand_command_bp;
	nand_chip->ecc.read_page = mtk_nand_read_page_hwecc;
	nand_chip->ecc.write_page = mtk_nand_write_page_hwecc;

	nand_chip->ecc.layout = &nand_oob_64;
	nand_chip->ecc.size = hw->nand_ecc_size;	/* 2048 */
	nand_chip->ecc.bytes = hw->nand_ecc_bytes;	/* 32 */

	nand_chip->options = NAND_SKIP_BBTSCAN;

	/* For BMT, we need to revise driver architecture */
	nand_chip->write_page = mtk_nand_write_page;
	nand_chip->ecc.write_oob = mtk_nand_write_oob;
	nand_chip->ecc.read_oob = mtk_nand_read_oob;
	/* need to add nand_get_device()/nand_release_device(). */
	nand_chip->block_markbad = mtk_nand_block_markbad;
	nand_chip->block_bad = mtk_nand_block_bad;

	/* MSG(INIT, "[NAND]Enable NFI and NFIECC Clock\n"); */
	nand_prepare_clock();
	nand_enable_clock();

	mtk_nand_init_hw(host);
	/* Select the device */
	nand_chip->select_chip(mtd, NFI_DEFAULT_CS);

	/*
	 * Reset the chip, required by some chips (e.g. Micron MT29FxGxxxxx)
	 * after power-up
	 */
	nand_chip->cmdfunc(mtd, NAND_CMD_RESET, -1, -1);

	/* Send the command for reading device ID */
	nand_chip->cmdfunc(mtd, NAND_CMD_READID, 0x00, -1);

	for (i = 0; i < NAND_MAX_ID; i++)
		id[i] = nand_chip->read_byte(mtd);

	manu_id = id[0];
	dev_id = id[1];

	if (!get_device_info(id, &devinfo)) {
		pr_err("Not Support this Device! \r\n");
		goto out;
	}

	if (mtk_nand_cs_check(mtd, id, NFI_TRICKY_CS)) {
		pr_info("Twins Nand\n");
		g_bTricky_CS = TRUE;
		g_b2Die_CS = TRUE;
	}

	if (devinfo.pagesize == 16384) {
		nand_chip->ecc.layout = &nand_oob_128;
		hw->nand_ecc_size = 16384;
	} else if (devinfo.pagesize == 8192) {
		nand_chip->ecc.layout = &nand_oob_128;
		hw->nand_ecc_size = 8192;
	} else if (devinfo.pagesize == 4096) {
		nand_chip->ecc.layout = &nand_oob_128;
		hw->nand_ecc_size = 4096;
	} else if (devinfo.pagesize == 2048) {
		nand_chip->ecc.layout = &nand_oob_64;
		hw->nand_ecc_size = 2048;
	} else if (devinfo.pagesize == 512) {
		nand_chip->ecc.layout = &nand_oob_16;
		hw->nand_ecc_size = 512;
	}
	if (devinfo.sectorsize == 1024) {
		sector_size = 1024;
		hw->nand_sec_shift = 10;
		hw->nand_sec_size = 1024;
		if (mtk_nfi_dev_comp->chip_ver == 1) {
			NFI_CLN_REG16(NFI_PAGEFMT_REG16, PAGEFMT_SECTOR_SEL);
		} else if (mtk_nfi_dev_comp->chip_ver == 2) {
			NFI_CLN_REG32(NFI_PAGEFMT_REG32, PAGEFMT_SECTOR_SEL);
		} else {
			pr_err("[mtk_nand_init_hw] ERROR, mtk_nfi_dev_comp->chip_ver=%d\n",
				mtk_nfi_dev_comp->chip_ver);
		}
	}
	if (devinfo.pagesize <= 4096) {
		nand_chip->ecc.layout->eccbytes =
			devinfo.sparesize - OOB_AVAI_PER_SECTOR * (devinfo.pagesize / sector_size);
		hw->nand_ecc_bytes = nand_chip->ecc.layout->eccbytes;
		/* Modify to fit device character */
		nand_chip->ecc.size = hw->nand_ecc_size;
		nand_chip->ecc.bytes = hw->nand_ecc_bytes;
	} else {
		/* devinfo.sparesize-OOB_AVAI_PER_SECTOR*(devinfo.pagesize/sector_size); */
		nand_chip->ecc.layout->eccbytes = 64;
		hw->nand_ecc_bytes = nand_chip->ecc.layout->eccbytes;
		/* Modify to fit device character */
		nand_chip->ecc.size = hw->nand_ecc_size;
		nand_chip->ecc.bytes = hw->nand_ecc_bytes;
	}
	nand_chip->subpagesize = devinfo.sectorsize;

	for (i = 0; i < nand_chip->ecc.layout->eccbytes; i++) {
		nand_chip->ecc.layout->eccpos[i] =
			OOB_AVAI_PER_SECTOR * (devinfo.pagesize / sector_size) + i;
	}
	/* MSG(INIT, "[NAND] pagesz:%d , oobsz: %d,eccbytes: %d\n", */
	/* devinfo.pagesize,  sizeof(g_kCMD.au1OOB),nand_chip->ecc.layout->eccbytes); */


	/* MSG(INIT, "Support this Device in MTK table! %x \r\n", id); */
	pr_notice("EFUSE RANDOM CFG is ON\n");
	use_randomizer = TRUE;
	pre_randomizer = TRUE;

	if ((devinfo.feature_set.FeatureSet.rtype == RTYPE_HYNIX_16NM)
		|| (devinfo.feature_set.FeatureSet.rtype == RTYPE_HYNIX))
		HYNIX_RR_TABLE_READ(&devinfo);

	hw->nfi_bus_width = devinfo.iowidth;

	DRV_WriteReg32(NFI_ACCCON_REG32, 0x10804333/*devinfo.timmingsetting*/);
	pr_err("ACCCON = 0x%x\n", DRV_Reg32(NFI_ACCCON_REG32));

	/* MSG(INIT, "Kernel Nand Timing:0x%x!\n", DRV_Reg32(NFI_ACCCON_REG32)); */

	/* 16-bit bus width */
	if (hw->nfi_bus_width == 16) {
		pr_notice("Set the 16-bit I/O settings!\n");
		nand_chip->options |= NAND_BUSWIDTH_16;
	}

	mtk_dev = &pdev->dev;
	pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;
	if (dma_set_mask(&pdev->dev, DMA_BIT_MASK(32))) {
		dev_err(&pdev->dev, "set dma mask fail\n");
		pr_err("set dma mask fail\n");
	} else
		pr_notice("set dma mask ok\n");

	mtd->oobsize = devinfo.sparesize;

	/* Scan to find existence of the device */
	if (nand_scan(mtd, hw->nfi_cs_num)) {
		pr_err("nand_scan fail.\n");
		err = -ENXIO;
		goto out;
	}

	g_page_size = mtd->writesize;
	g_block_size = devinfo.blocksize << 10;
	PAGES_PER_BLOCK = (u32) (g_block_size / g_page_size);
	/* MSG(INIT, "g_page_size(%d) g_block_size(%d)\n",g_page_size, g_block_size); */
	g_nanddie_pages = (u32) (nand_chip->chipsize >> nand_chip->page_shift);
	/* if(devinfo.ttarget == TTYPE_2DIE) */
	/* { */
	/* g_nanddie_pages = g_nanddie_pages / 2; */
	/* } */
	if (g_b2Die_CS) {
		nand_chip->chipsize <<= 1;
		/* MSG(INIT, "[Bean]%dMB\n", (u32)(nand_chip->chipsize/1024/1024)); */
	}

	platform_set_drvdata(pdev, host);

	if (hw->nfi_bus_width == 16) {
		if (mtk_nfi_dev_comp->chip_ver == 1) {
			NFI_SET_REG16(NFI_PAGEFMT_REG16, PAGEFMT_DBYTE_EN);
		} else if (mtk_nfi_dev_comp->chip_ver == 2) {
			NFI_SET_REG32(NFI_PAGEFMT_REG32, PAGEFMT_DBYTE_EN);
		} else {
			pr_err("[mtk_nand_init_hw] ERROR, mtk_nfi_dev_comp->chip_ver=%d\n",
				mtk_nfi_dev_comp->chip_ver);
		}
	}

	nand_chip->select_chip(mtd, 0);

	mtd->size = nand_chip->chipsize;

	/* Successfully!! */
	if (!err) {
		/* MSG(INIT, "[mtk_nand] probe successfully!\n"); */
		nand_disable_clock();
		return err;
	}

	/* Fail!! */
out:
	pr_err("[NFI] mtk_nand_probe fail, err = %d!\n", err);
	nand_release(mtd);
	platform_set_drvdata(pdev, NULL);
	kfree(host);
	nand_disable_clock();
	nand_unprepare_clock();
	return err;
}

/******************************************************************************
 * mtk_nand_suspend
 *
 * DESCRIPTION:
 *	 Suspend the nand device!
 *
 * PARAMETERS:
 *	 struct platform_device *pdev : device structure
 *
 * RETURNS:
 *	 0 : Success
 *
 * NOTES:
 *	 None
 *
 ******************************************************************************/
static int mtk_nand_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct mtk_nand_host *host = platform_get_drvdata(pdev);
	  int ret = 0;
	/* struct mtd_info *mtd = &host->mtd; */
	/* backup register */
	if (host->saved_para.suspend_flag == 0) {
		nand_enable_clock();
		/* Save NFI register */
		host->saved_para.sNFI_CNFG_REG16 = DRV_Reg16(NFI_CNFG_REG16);
		if (mtk_nfi_dev_comp->chip_ver == 1) {
			host->saved_para.sNFI_PAGEFMT_REG16 = DRV_Reg16(NFI_PAGEFMT_REG16);
		} else if (mtk_nfi_dev_comp->chip_ver == 2) {
			host->saved_para.sNFI_PAGEFMT_REG32 = DRV_Reg32(NFI_PAGEFMT_REG32);
		} else {
			pr_err("[NFI] Suspend ERROR, mtk_nfi_dev_comp->chip_ver=%d\n",
				mtk_nfi_dev_comp->chip_ver);
		}
		host->saved_para.sNFI_CON_REG16 = DRV_Reg32(NFI_CON_REG16);
		host->saved_para.sNFI_ACCCON_REG32 = DRV_Reg32(NFI_ACCCON_REG32);
		host->saved_para.sNFI_INTR_EN_REG16 = DRV_Reg16(NFI_INTR_EN_REG16);
		host->saved_para.sNFI_IOCON_REG16 = DRV_Reg16(NFI_IOCON_REG16);
		host->saved_para.sNFI_CSEL_REG16 = DRV_Reg16(NFI_CSEL_REG16);
		host->saved_para.sNFI_DEBUG_CON1_REG16 = DRV_Reg16(NFI_DEBUG_CON1_REG16);

		/* save ECC register */
		host->saved_para.sECC_ENCCNFG_REG32 = DRV_Reg32(ECC_ENCCNFG_REG32);
		/* host->saved_para.sECC_FDMADDR_REG32 = DRV_Reg32(ECC_FDMADDR_REG32); */
		host->saved_para.sECC_DECCNFG_REG32 = DRV_Reg32(ECC_DECCNFG_REG32);
		nand_disable_clock();
		nand_unprepare_clock();
		host->saved_para.suspend_flag = 1;
	} else {
		pr_debug("[NFI] Suspend twice !\n");
	}

	pr_debug("[NFI] Suspend !\n");
	return 0;
}

/******************************************************************************
 * mtk_nand_resume
 *
 * DESCRIPTION:
 *	 Resume the nand device!
 *
 * PARAMETERS:
 *	 struct platform_device *pdev : device structure
 *
 * RETURNS:
 *	 0 : Success
 *
 * NOTES:
 *	 None
 *
 ******************************************************************************/
static int mtk_nand_resume(struct platform_device *pdev)
{
	struct mtk_nand_host *host = platform_get_drvdata(pdev);
	int ret = 0;
	/* struct mtd_info *mtd = &host->mtd;  //for test */
	/* struct nand_chip *chip = mtd->priv; */
	/* struct gFeatureSet *feature_set = &(devinfo.feature_set.FeatureSet); //for test */
	/* int val = -1;   // for test */


	if (host->saved_para.suspend_flag == 1) {
		/* restore NFI register */
		udelay(200);
		pr_debug("[NFI] delay 200us for power on reset flow!\n");
		nand_prepare_clock();
		nand_enable_clock();
		DRV_WriteReg16(NFI_CNFG_REG16, host->saved_para.sNFI_CNFG_REG16);
		if (mtk_nfi_dev_comp->chip_ver == 1) {
			DRV_WriteReg16(NFI_PAGEFMT_REG16, host->saved_para.sNFI_PAGEFMT_REG16);
		} else if (mtk_nfi_dev_comp->chip_ver == 2) {
			DRV_WriteReg32(NFI_PAGEFMT_REG32, host->saved_para.sNFI_PAGEFMT_REG32);
		} else {
			pr_err("[NFI] Resume ERROR, mtk_nfi_dev_comp->chip_ver=%d\n",
				mtk_nfi_dev_comp->chip_ver);
		}
		DRV_WriteReg32(NFI_CON_REG16, host->saved_para.sNFI_CON_REG16);
		DRV_WriteReg32(NFI_ACCCON_REG32, host->saved_para.sNFI_ACCCON_REG32);
		DRV_WriteReg16(NFI_IOCON_REG16, host->saved_para.sNFI_IOCON_REG16);
		DRV_WriteReg16(NFI_CSEL_REG16, host->saved_para.sNFI_CSEL_REG16);
		DRV_WriteReg16(NFI_DEBUG_CON1_REG16, host->saved_para.sNFI_DEBUG_CON1_REG16);

		/* restore ECC register */
		DRV_WriteReg32(ECC_ENCCNFG_REG32, host->saved_para.sECC_ENCCNFG_REG32);
		/* DRV_WriteReg32(ECC_FDMADDR_REG32 ,host->saved_para.sECC_FDMADDR_REG32); */
		DRV_WriteReg32(ECC_DECCNFG_REG32, host->saved_para.sECC_DECCNFG_REG32);

		/* Reset NFI and ECC state machine */
		/* Reset the state machine and data FIFO, because flushing FIFO */
		(void)mtk_nand_reset();
		/* Reset ECC */
		DRV_WriteReg16(ECC_DECCON_REG16, DEC_DE);
		while (!DRV_Reg16(ECC_DECIDLE_REG16))
			;

		DRV_WriteReg16(ECC_ENCCON_REG16, ENC_DE);
		while (!DRV_Reg32(ECC_ENCIDLE_REG32))
			;


		/* Initialize interrupt. Clear interrupt, read clear. */
		DRV_Reg16(NFI_INTR_REG16);

		DRV_WriteReg16(NFI_INTR_EN_REG16, host->saved_para.sNFI_INTR_EN_REG16);

		/* mtk_nand_interface_config(&host->mtd); */
		/* mtk_nand_GetFeature(mtd, feature_set->gfeatureCmd, \ */
		/* feature_set->Interface.address, (u8 *)&val,4); */
		/* MSG(POWERCTL, "[NFI] Resume feature %d!\n", val); */

		mtk_nand_device_reset();

		nand_disable_clock();
		host->saved_para.suspend_flag = 0;
	} else {
		pr_debug("[NFI] Resume twice !\n");
	}
	pr_debug("[NFI] Resume !\n");
	return 0;
}

/******************************************************************************
 * mtk_nand_remove
 *
 * DESCRIPTION:
 *	 unregister the nand device file operations !
 *
 * PARAMETERS:
 *	 struct platform_device *pdev : device structure
 *
 * RETURNS:
 *	 0 : Success
 *
 * NOTES:
 *	 None
 *
 ******************************************************************************/

static int mtk_nand_remove(struct platform_device *pdev)
{
	struct mtk_nand_host *host = platform_get_drvdata(pdev);
	struct mtd_info *mtd = &host->mtd;

	nand_release(mtd);

	kfree(host);

	nand_disable_clock();
	nand_unprepare_clock();
	return 0;
}

static struct platform_driver mtk_nand_driver = {
	.probe = mtk_nand_probe,
	.remove = mtk_nand_remove,
	.suspend = mtk_nand_suspend,
	.resume = mtk_nand_resume,
	.driver = {
		   .name = "mtk-nand",
		   .owner = THIS_MODULE,
		   .of_match_table = mtk_nfi_of_match,
		   },
};

/******************************************************************************
 * mtk_nand_init
 *
 * DESCRIPTION:
 *	 Init the device driver !
 *
 * PARAMETERS:
 *	 None
 *
 * RETURNS:
 *	 None
 *
 * NOTES:
 *	 None
 *
 ******************************************************************************/
#define SEQ_printf(m, x...)		\
do {			\
	if (m)			\
		seq_printf(m, x);	\
	else			\
		pr_debug(x);		\
} while (0)

int mtk_nand_proc_show(struct seq_file *m, void *v)
{
	int i;

	SEQ_printf(m, "ID:");
	for (i = 0; i < devinfo.id_length; i++)
		SEQ_printf(m, " 0x%x", devinfo.id[i]);

	SEQ_printf(m, "\n");
	SEQ_printf(m, "total size: %dMiB; part number: %s\n", devinfo.totalsize,
		   devinfo.devciename);
	SEQ_printf(m, "Current working in %s mode\n", g_i4Interrupt ? "interrupt" : "polling");
	SEQ_printf(m, "NFI_ACCON=0x%x\n", DRV_Reg32(NFI_ACCCON_REG32));
	SEQ_printf(m, "NFI_NAND_TYPE_CNFG_REG32= 0x%x\n", DRV_Reg32(NFI_NAND_TYPE_CNFG_REG32));
	return 0;
}


static int mt_nand_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, mtk_nand_proc_show, inode->i_private);
}


static const struct file_operations mtk_nand_fops = {
	.open = mt_nand_proc_open,
	.write = mtk_nand_proc_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int __init mtk_nand_init(void)
{
	struct proc_dir_entry *entry;

	g_i4Interrupt = 0;

	entry = proc_create(PROCNAME, 0664, NULL, &mtk_nand_fops);

	return platform_driver_register(&mtk_nand_driver);
}

/******************************************************************************
 * mtk_nand_exit
 *
 * DESCRIPTION:
 *	 Free the device driver !
 *
 * PARAMETERS:
 *	 None
 *
 * RETURNS:
 *	 None
 *
 * NOTES:
 *	 None
 *
 ******************************************************************************/
static void __exit mtk_nand_exit(void)
{
	pr_debug("MediaTek Nand driver exit, version %s\n", VERSION);
	platform_driver_unregister(&mtk_nand_driver);
	remove_proc_entry(PROCNAME, NULL);
}
late_initcall(mtk_nand_init);
module_exit(mtk_nand_exit);
MODULE_LICENSE("GPL");
