/*
 * Copyright (C) 2021 Alibaba Group Holding Limited
 * Author: Shenwuyi <shenwuyi.swy@alibaba-inc.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/regmap.h>
#include <linux/of_reserved_mem.h>
#include <linux/io.h>
#include "vi_pre.h"
#include "vi_pre_ioctl.h"

#define check_retval(x)\
	do {\
		if ((x))\
			return -EIO;\
	} while (0)

static void vi_pre_write(struct vi_pre_dev *dec, unsigned int addr, unsigned int val)
{
	writel(val, dec->reg_base + addr);
}

static int vi_pre_write_reg(struct vi_pre_dev *dev, void *__user args)
{
	struct vi_pre_reg_t reg, reg1;
	check_retval(copy_from_user(&reg, args, sizeof(reg)));
	writel(reg.value, dev->reg_base + reg.offset);
	pr_info("%s  addr 0x%08x val 0x%08x\n", __func__, reg.offset, reg.value);

	reg1.value = readl(dev->reg_base + reg.offset);
	pr_info("%s  rad back addr 0x%08x val 0x%08x\n", __func__, reg.offset, reg1.value);
	return 0;
}

static int vi_pre_read_reg(struct vi_pre_dev *dev, void *__user args)
{
	struct vi_pre_reg_t reg;
	check_retval(copy_from_user(&reg, args, sizeof(reg)));
	reg.value = readl(dev->reg_base + reg.offset);
	check_retval(copy_to_user(args, &reg, sizeof(reg)));
	pr_info("%s  addr 0x%08x val 0x%08x\n", __func__, reg.offset, reg.value);
	return 0;
}
static unsigned int vi_pre_read(struct vi_pre_dev *dev, unsigned int addr)
{
	return readl(dev->reg_base + addr);
}

static int vi_pre_set_hdrpro(struct vi_pre_dev *dev, void *args)
{
	int ret = 0;
	unsigned int val = 0;

	val = vi_pre_read(dev, HDRPRO_CTRL(1));
	val |= HDRPRO_EN;
	val |= HDRPRO_CTRL1_BAYER_MODSEL_BAYER_C;
	val |= HDRPRO_CTRL1_COLER_MODSEL_BLACK_NODOWN_2X2;
	val &= ~HDRPRO_CTRL1_HDRPRO_RAWMOD_MASK;
	val |= HDRPRO_CTRL1_RAW10;
	vi_pre_write(dev, HDRPRO_CTRL(1), val);

	val = vi_pre_read(dev, HDRPRO_CTRL(2));
	val &= ~HDRPRO_CTRL2_VERTICAL_MASK;
	val |= (480 << HDRPRO_CTRL2_VERTICAL_SHIFT);
	val &= ~HDRPRO_CTRL2_HORIZON_MASK;
	val |= (640 << HDRPRO_CTRL2_HORIZON_SHIFT);
	vi_pre_write(dev, HDRPRO_CTRL(2), val);

	val = vi_pre_read(dev, HDRPRO_COE(1));
	val |= (8 << HDRPRO_COE1_BE_BLACK_WR_SHIFT);
	val |= (6 << HDRPRO_COE1_BE_BLACK_WB_SHIFT);
	val |= (4 << HDRPRO_COE1_BE_BLACK_WG0_SHIFT);
	val |= (2 << HDRPRO_COE1_BE_BLACK_WG1_SHIFT);
	val |= (1 << HDRPRO_COE1_BE_BLACK_WG0_00_SHIFT);
	val |= (2 << HDRPRO_COE1_BE_BLACK_WG0_01_SHIFT);
	val |= (3 << HDRPRO_COE1_BE_BLACK_WG0_02_SHIFT);
	val |= (4 << HDRPRO_COE1_BE_BLACK_WG0_10_SHIFT);
	vi_pre_write(dev, HDRPRO_COE(1), val);

	val = vi_pre_read(dev, HDRPRO_COE(2));
	val |= (5 << HDRPRO_COE2_BE_BLACK_WG0_11_SHIFT);
	val |= (6 << HDRPRO_COE2_BE_BLACK_WG0_12_SHIFT);
	val |= (7 << HDRPRO_COE2_BE_BLACK_WG0_20_SHIFT);
	val |= (8 << HDRPRO_COE2_BE_BLACK_WG0_21_SHIFT);
	val |= (9 << HDRPRO_COE2_BE_BLACK_WG0_22_SHIFT);
	val |= (1 << HDRPRO_COE2_BE_BLACK_WG1_00_SHIFT);
	val |= (2 << HDRPRO_COE2_BE_BLACK_WG1_01_SHIFT);
	val |= (3 << HDRPRO_COE2_BE_BLACK_WG1_02_SHIFT);
	vi_pre_write(dev, HDRPRO_COE(2), val);

	val = vi_pre_read(dev, HDRPRO_COE(3));
	val |= (4 << HDRPRO_COE3_BE_BLACK_WG1_10_SHIFT);
	val |= (5 << HDRPRO_COE3_BE_BLACK_WG1_11_SHIFT);
	val |= (6 << HDRPRO_COE3_BE_BLACK_WG1_12_SHIFT);
	val |= (7 << HDRPRO_COE3_BE_BLACK_WG1_20_SHIFT);
	val |= (8 << HDRPRO_COE3_BE_BLACK_WG1_21_SHIFT);
	val |= (9 << HDRPRO_COE3_BE_BLACK_WG1_22_SHIFT);
	val |= (1 << HDRPRO_COE3_BE_BLACK_WR_00_SHIFT);
	val |= (2 << HDRPRO_COE3_BE_BLACK_WR_01_SHIFT);
	vi_pre_write(dev, HDRPRO_COE(3), val);

	val = vi_pre_read(dev, HDRPRO_COE(4));
	val |= (3 << HDRPRO_COE4_BE_BLACK_WR_02_SHIFT);
	val |= (4 << HDRPRO_COE4_BE_BLACK_WR_10_SHIFT);
	val |= (5 << HDRPRO_COE4_BE_BLACK_WR_11_SHIFT);
	val |= (6 << HDRPRO_COE4_BE_BLACK_WR_12_SHIFT);
	val |= (7 << HDRPRO_COE4_BE_BLACK_WR_20_SHIFT);
	val |= (8 << HDRPRO_COE4_BE_BLACK_WR_21_SHIFT);
	val |= (9 << HDRPRO_COE4_BE_BLACK_WR_22_SHIFT);
	val |= (1 << HDRPRO_COE4_BE_BLACK_WB_00_SHIFT);
	vi_pre_write(dev, HDRPRO_COE(4), val);

	val = vi_pre_read(dev, HDRPRO_COE(5));
	val |= (2 << HDRPRO_COE5_BE_BLACK_WB_02_SHIFT);
	val |= (3 << HDRPRO_COE5_BE_BLACK_WB_10_SHIFT);
	val |= (4 << HDRPRO_COE5_BE_BLACK_WB_11_SHIFT);
	val |= (5 << HDRPRO_COE5_BE_BLACK_WB_12_SHIFT);
	val |= (6 << HDRPRO_COE5_BE_BLACK_WB_20_SHIFT);
	val |= (7 << HDRPRO_COE5_BE_BLACK_WB_21_SHIFT);
	val |= (8 << HDRPRO_COE5_BE_BLACK_WB_22_SHIFT);
	val |= (9 << HDRPRO_COE5_BE_BLACK_WB_01_SHIFT);
	vi_pre_write(dev, HDRPRO_COE(5), val);

	val = vi_pre_read(dev, HDRPRO_LINW(1));
	val |= (0x100 << HDRPRO_LINW1_W1_SHIFT);
	vi_pre_write(dev, HDRPRO_LINW(1), val);

	val = vi_pre_read(dev, HDRPRO_LINA(1));
	val |= (0 << HDRPRO_LINA1_A1_SHIFT);
	vi_pre_write(dev, HDRPRO_LINA(1), val);

	val = vi_pre_read(dev, HDRPRO_LINB(1));
	val |= (0x1000 << HDRPRO_LINB1_B1_SHIFT);
	vi_pre_write(dev, HDRPRO_LINB(1), val);

	val = vi_pre_read(dev, HDRPRO_LINW(2));
	val |= (0x100 << HDRPRO_LINW1_W2_SHIFT);
	vi_pre_write(dev, HDRPRO_LINW(2), val);

	val = vi_pre_read(dev, HDRPRO_LINA(2));
	val |= (0x1fff << HDRPRO_LINA2_A2_SHIFT);
	vi_pre_write(dev, HDRPRO_LINA(2), val);

	val = vi_pre_read(dev, HDRPRO_LINB(2));
	val |= (0x1f00 << HDRPRO_LINB2_B2_SHIFT);
	vi_pre_write(dev, HDRPRO_LINB(2), val);

	val = vi_pre_read(dev, HDRPRO_LINW(3));
	val |= (0x100 << HDRPRO_LINW2_W3_SHIFT);
	vi_pre_write(dev, HDRPRO_LINW(3), val);

	val = vi_pre_read(dev, HDRPRO_LINA(3));
	val |= (0x2fff << HDRPRO_LINA3_A3_SHIFT);
	vi_pre_write(dev, HDRPRO_LINA(3), val);

	val = vi_pre_read(dev, HDRPRO_LINB(3));
	val |= (0x2000 << HDRPRO_LINB3_B3_SHIFT);
	vi_pre_write(dev, HDRPRO_LINB(3), val);

	val = vi_pre_read(dev, HDRPRO_LINW(4));
	val |= (0x100 << HDRPRO_LINW2_W4_SHIFT);
	vi_pre_write(dev, HDRPRO_LINW(4), val);

	val = vi_pre_read(dev, HDRPRO_LINA(4));
	val |= (0x3fff << HDRPRO_LINA4_A4_SHIFT);
	vi_pre_write(dev, HDRPRO_LINA(4), val);

	val = vi_pre_read(dev, HDRPRO_LINB(4));
	val |= (0x3000 << HDRPRO_LINB4_B4_SHIFT);
	vi_pre_write(dev, HDRPRO_LINB(4), val);

	val = vi_pre_read(dev, HDRPRO_LINW(5));
	val |= (0x100 << HDRPRO_LINW3_W5_SHIFT);
	vi_pre_write(dev, HDRPRO_LINW(5), val);

	val = vi_pre_read(dev, HDRPRO_LINA(5));
	val |= (0x4fff << HDRPRO_LINA5_A5_SHIFT);
	vi_pre_write(dev, HDRPRO_LINA(5), val);

	val = vi_pre_read(dev, HDRPRO_LINB(5));
	val |= (0x4000 << HDRPRO_LINB5_B5_SHIFT);
	vi_pre_write(dev, HDRPRO_LINB(5), val);

	val = vi_pre_read(dev, HDRPRO_LINW(6));
	val |= (0x100 << HDRPRO_LINW3_W6_SHIFT);
	vi_pre_write(dev, HDRPRO_LINW(6), val);

	val = vi_pre_read(dev, HDRPRO_LINA(6));
	val |= (0x5fff << HDRPRO_LINA6_A6_SHIFT);
	vi_pre_write(dev, HDRPRO_LINA(6), val);

	val = vi_pre_read(dev, HDRPRO_LINB(6));
	val |= (0x5000 << HDRPRO_LINB6_B6_SHIFT);
	vi_pre_write(dev, HDRPRO_LINB(6), val);

	val = vi_pre_read(dev, HDRPRO_LINW(7));
	val |= (0x100 << HDRPRO_LINW4_W7_SHIFT);
	vi_pre_write(dev, HDRPRO_LINW(7), val);

	val = vi_pre_read(dev, HDRPRO_LINA(7));
	val |= (0x6fff << HDRPRO_LINA7_A7_SHIFT);
	vi_pre_write(dev, HDRPRO_LINA(7), val);

	val = vi_pre_read(dev, HDRPRO_LINB(7));
	val |= (0x6000 << HDRPRO_LINB7_B7_SHIFT);
	vi_pre_write(dev, HDRPRO_LINB(7), val);

	val = vi_pre_read(dev, HDRPRO_LINW(8));
	val |= (0x100 << HDRPRO_LINW4_W8_SHIFT);
	vi_pre_write(dev, HDRPRO_LINW(8), val);

	val = vi_pre_read(dev, HDRPRO_LINA(8));
	val |= (0x7fff << HDRPRO_LINA8_A8_SHIFT);
	vi_pre_write(dev, HDRPRO_LINA(8), val);

	val = vi_pre_read(dev, HDRPRO_LINB(8));
	val |= (0x7000 << HDRPRO_LINB8_B8_SHIFT);
	vi_pre_write(dev, HDRPRO_LINB(8), val);

	val = vi_pre_read(dev, HDRPRO_LINW(9));
	val |= (0x100 << HDRPRO_LINW5_W9_SHIFT);
	vi_pre_write(dev, HDRPRO_LINW(9), val);

	val = vi_pre_read(dev, HDRPRO_LINA(9));
	val |= (0x8fff << HDRPRO_LINA9_A9_SHIFT);
	vi_pre_write(dev, HDRPRO_LINA(9), val);

	val = vi_pre_read(dev, HDRPRO_LINB(9));
	val |= (0x8000 << HDRPRO_LINB9_B9_SHIFT);
	vi_pre_write(dev, HDRPRO_LINB(9), val);

	val = vi_pre_read(dev, HDRPRO_LINW(10));
	val |= (0x100 << HDRPRO_LINW5_W10_SHIFT);
	vi_pre_write(dev, HDRPRO_LINW(10), val);

	val = vi_pre_read(dev, HDRPRO_LINA(10));
	val |= (0x9fff << HDRPRO_LINA10_A10_SHIFT);
	vi_pre_write(dev, HDRPRO_LINA(10), val);

	val = vi_pre_read(dev, HDRPRO_LINB(10));
	val |= (0x9000 << HDRPRO_LINB10_B10_SHIFT);
	vi_pre_write(dev, HDRPRO_LINB(10), val);

	val = vi_pre_read(dev, HDRPRO_LINW(11));
	val |= (0x100 << HDRPRO_LINW6_W11_SHIFT);
	vi_pre_write(dev, HDRPRO_LINW(11), val);

	val = vi_pre_read(dev, HDRPRO_LINA(11));
	val |= (0xafff << HDRPRO_LINA11_A11_SHIFT);
	vi_pre_write(dev, HDRPRO_LINA(11), val);

	val = vi_pre_read(dev, HDRPRO_LINB(11));
	val |= (0xa000 << HDRPRO_LINB11_B11_SHIFT);
	vi_pre_write(dev, HDRPRO_LINB(11), val);

	val = vi_pre_read(dev, HDRPRO_LINW(12));
	val |= (0x100 << HDRPRO_LINW6_W12_SHIFT);
	vi_pre_write(dev, HDRPRO_LINW(12), val);

	val = vi_pre_read(dev, HDRPRO_LINA(12));
	val |= (0xbfff << HDRPRO_LINA12_A12_SHIFT);
	vi_pre_write(dev, HDRPRO_LINA(12), val);

	val = vi_pre_read(dev, HDRPRO_LINB(12));
	val |= (0xb000 << HDRPRO_LINB12_B12_SHIFT);
	vi_pre_write(dev, HDRPRO_LINB(12), val);

	val = vi_pre_read(dev, HDRPRO_LINW(13));
	val |= (0x100 << HDRPRO_LINW7_W13_SHIFT);
	vi_pre_write(dev, HDRPRO_LINW(13), val);

	val = vi_pre_read(dev, HDRPRO_LINA(13));
	val |= (0xcfff << HDRPRO_LINA13_A13_SHIFT);
	vi_pre_write(dev, HDRPRO_LINA(13), val);

	val = vi_pre_read(dev, HDRPRO_LINB(13));
	val |= (0xc000 << HDRPRO_LINB13_B13_SHIFT);
	vi_pre_write(dev, HDRPRO_LINB(13), val);

	val = vi_pre_read(dev, HDRPRO_LINW(14));
	val |= (0x100 << HDRPRO_LINW7_W14_SHIFT);
	vi_pre_write(dev, HDRPRO_LINW(14), val);

	val = vi_pre_read(dev, HDRPRO_LINA(14));
	val |= (0xdfff << HDRPRO_LINA14_A14_SHIFT);
	vi_pre_write(dev, HDRPRO_LINA(14), val);

	val = vi_pre_read(dev, HDRPRO_LINB(14));
	val |= (0xd000 << HDRPRO_LINB14_B14_SHIFT);
	vi_pre_write(dev, HDRPRO_LINB(14), val);

	val = vi_pre_read(dev, HDRPRO_LINW(15));
	val |= (0x100 << HDRPRO_LINW8_W15_SHIFT);
	vi_pre_write(dev, HDRPRO_LINW(15), val);

	val = vi_pre_read(dev, HDRPRO_LINA(15));
	val |= (0xefff << HDRPRO_LINA15_A15_SHIFT);
	vi_pre_write(dev, HDRPRO_LINA(15), val);

	val = vi_pre_read(dev, HDRPRO_LINB(15));
	val |= (0xe000 << HDRPRO_LINB15_B15_SHIFT);
	vi_pre_write(dev, HDRPRO_LINB(15), val);

	val = vi_pre_read(dev, HDRPRO_LINW(16));
	val |= (0x100 << HDRPRO_LINW8_W16_SHIFT);
	vi_pre_write(dev, HDRPRO_LINW(16), val);

	val = vi_pre_read(dev, HDRPRO_LINA(16));
	val |= (0xffff << HDRPRO_LINA16_A16_SHIFT);
	vi_pre_write(dev, HDRPRO_LINA(16), val);

	val = vi_pre_read(dev, HDRPRO_LINB(16));
	val |= (0xffff << HDRPRO_LINB16_B16_SHIFT);
	vi_pre_write(dev, HDRPRO_LINB(16), val);

	return ret;
}

static int vi_pre_set_mipi2dma_n_line(struct vi_pre_dev *dev, void *args)
{
#define WIDTH 640
#define HEIGHT 480
#define RAW_SIZE 16 //RAW12
#define BURST_LEN 16
#define OUT_STANDING 1024
//K > N, N>2
#define K_LINE 32
#define N_LINE 16

	int ret = 0;
	unsigned int val = 0;
	unsigned int stride = 0;
	unsigned int horizon_cnt128 = 0;
	unsigned int readnum = 0;

	stride = WIDTH * RAW_SIZE / 8;
	horizon_cnt128 = stride / 16;
	readnum = horizon_cnt128 / BURST_LEN;

	//base mode config
	//val = vi_pre_read(dev, MIPI2DMA_CTRL(0));
	val = 0;
	val |= MIPI2DMA_CTRL0_MODE_N_LINE;
	val |= MIPI2DMA_CTRL0_RAW10;
	val |= MIPI2DMA_CTRL0_HIGH_BIT_MODE;
	//val |= MIPI2DMA_CTRL0_4_FRAME;
	vi_pre_write(dev, MIPI2DMA_CTRL(0), val);

	//val = vi_pre_read(dev, MIPI2DMA_CTRL(1));
	val = 0;
	val |= MIPI2DMA_CTRL1_WBURSTLEN_16;
	val |= ((OUT_STANDING - 1) << MIPI2DMA_CTRL1_WOSNUM_SHIFT);
	vi_pre_write(dev, MIPI2DMA_CTRL(1), val);

	//val = vi_pre_read(dev, MIPI2DMA_CTRL(3));
	val = 0;
	val |= (WIDTH << MIPI2DMA_CTRL3_HORIZON_SHIFT);
	val |= (HEIGHT << MIPI2DMA_CTRL3_VERTICAL_SHIFT);
	vi_pre_write(dev, MIPI2DMA_CTRL(3), val);

	//val = vi_pre_read(dev, MIPI2DMA_CTRL(4));
	val = 0;
	//need calculate BURSTREM by unsing the function from vipre doc
	val |= (BURST_LEN << MIPI2DMA_CTRL4_BURSTREM_SHIFT);
	val |= (readnum << MIPI2DMA_CTRL4_READNUM_SHIFT);
	val |= (horizon_cnt128 << MIPI2DMA_CTRL4_HORIZON_CNT128_SHIFT);
	vi_pre_write(dev, MIPI2DMA_CTRL(4), val);

	//val = vi_pre_read(dev, MIPI2DMA_CTRL(5));
	val = 0;
	val |= (N_LINE << MIPI2DMA_CTRL5_N_NLINENUM_SHIFT);
	val |= (K_LINE << MIPI2DMA_CTRL5_N_LINENUM_SHIFT);
	vi_pre_write(dev, MIPI2DMA_CTRL(5), val);

	//val = vi_pre_read(dev, MIPI2DMA_CTRL(6));
	val = 0;
	val |= (stride << MIPI2DMA_CTRL6_N_STRIDE_SHIFT);
	vi_pre_write(dev, MIPI2DMA_CTRL(6), val);

	val = vi_pre_read(dev, MIPI2DMA_CTRL(51));
	val |= MIPI2DMA_CTRL51_CROSS_4K_EN;
	vi_pre_write(dev, MIPI2DMA_CTRL(51), val);

	//feedback config to default
	//clear INV_FLAG
	val = 0;
	vi_pre_write(dev, MIPI2DMA_CTRL(11), val);

	//clear N_LINENUM_NEW_ID0
	val = 0;
	vi_pre_write(dev, MIPI2DMA_CTRL(12), val);
	//clear N_LINENUM_NEW_ID1
	val = 0;
	vi_pre_write(dev, MIPI2DMA_CTRL(13), val);
	//clear N_LINENUM_NEW_ID2
	val = 0;
	vi_pre_write(dev, MIPI2DMA_CTRL(14), val);

	//clear N_num_done_idx
	val = 0;
	vi_pre_write(dev, MIPI2DMA_CTRL(21), val);
	val = 0;
	vi_pre_write(dev, MIPI2DMA_CTRL(22), val);

	//N ID0 address
	//val = vi_pre_read(dev, MIPI2DMA_CTRL(15));
	val = 0;
	val |= (0x0 << MIPI2DMA_CTRL15_N_SADDR_ID0_H_SHIFT);
	vi_pre_write(dev, MIPI2DMA_CTRL(15), val);

	//val = vi_pre_read(dev, MIPI2DMA_CTRL(16));
	val = 0;
	val |= (0xf0000000 << MIPI2DMA_CTRL16_N_SADDR_ID0_L_SHIFT);
	vi_pre_write(dev, MIPI2DMA_CTRL(16), val);

	//N ID1 address
	//val = vi_pre_read(dev, MIPI2DMA_CTRL(17));
	val = 0;
	val |= (0x0 << MIPI2DMA_CTRL17_N_SADDR_ID1_H_SHIFT);
	vi_pre_write(dev, MIPI2DMA_CTRL(17), val);

	//val = vi_pre_read(dev, MIPI2DMA_CTRL(18));
	val = 0;
	val |= (0xf1000000 << MIPI2DMA_CTRL18_N_SADDR_ID1_L_SHIFT);
	vi_pre_write(dev, MIPI2DMA_CTRL(18), val);

	//N ID2 address
	//val = vi_pre_read(dev, MIPI2DMA_CTRL(19));
	val = 0;
	val |= (0x0 << MIPI2DMA_CTRL19_N_SADDR_ID2_H_SHIFT);
	vi_pre_write(dev, MIPI2DMA_CTRL(19), val);

	//val = vi_pre_read(dev, MIPI2DMA_CTRL(20));
	val = 0;
	val |= (0xf2000000 << MIPI2DMA_CTRL20_N_SADDR_ID2_L_SHIFT);
	vi_pre_write(dev, MIPI2DMA_CTRL(20), val);

	// int status clean & mask
	//val = vi_pre_read(dev, MIPI2DMA_CTRL(44));
	//enable all interrupt
	val = 0;
	vi_pre_write(dev, MIPI2DMA_CTRL(44), val);

	val = vi_pre_read(dev, MIPI2DMA_CTRL(10));
	val |= MIPI2DMA_START;
	vi_pre_write(dev, MIPI2DMA_CTRL(10), val);

	return ret;
}

static int vi_pre_set_mipi2dma_m_frame(struct vi_pre_dev *dev, void *args)
{
#define WIDTH 640
#define HEIGHT 480
#define RAW_SIZE 16 //RAW12
#define BURST_LEN 16
#define OUT_STANDING 1024

	int ret = 0;
	unsigned int val = 0;
	unsigned int stride = 0;
	unsigned int horizon_cnt128 = 0;
	unsigned int readnum = 0;

	stride = WIDTH * RAW_SIZE / 8;
	horizon_cnt128 = stride / 16;
	readnum = horizon_cnt128 / BURST_LEN;

	//base mode config
	//val = vi_pre_read(dev, MIPI2DMA_CTRL(0));
	val = 0;
	val |= MIPI2DMA_CTRL0_MODE_M_FRAME;
	val |= MIPI2DMA_CTRL0_RAW10;
	val |= MIPI2DMA_CTRL0_HIGH_BIT_MODE;
	val |= MIPI2DMA_CTRL0_4_FRAME;
	vi_pre_write(dev, MIPI2DMA_CTRL(0), val);

	//val = vi_pre_read(dev, MIPI2DMA_CTRL(1));
	val = 0;
	val |= MIPI2DMA_CTRL1_WBURSTLEN_16;
	val |= ((OUT_STANDING - 1) << MIPI2DMA_CTRL1_WOSNUM_SHIFT);
	vi_pre_write(dev, MIPI2DMA_CTRL(1), val);

	//val = vi_pre_read(dev, MIPI2DMA_CTRL(3));
	val = 0;
	val |= (WIDTH << MIPI2DMA_CTRL3_HORIZON_SHIFT);
	val |= (HEIGHT << MIPI2DMA_CTRL3_VERTICAL_SHIFT);
	vi_pre_write(dev, MIPI2DMA_CTRL(3), val);

	//val = vi_pre_read(dev, MIPI2DMA_CTRL(4));
	val = 0;
	//need calculate BURSTREM by unsing the function from vipre doc
	val |= (BURST_LEN << MIPI2DMA_CTRL4_BURSTREM_SHIFT);
	val |= (readnum << MIPI2DMA_CTRL4_READNUM_SHIFT);
	val |= (horizon_cnt128 << MIPI2DMA_CTRL4_HORIZON_CNT128_SHIFT);
	vi_pre_write(dev, MIPI2DMA_CTRL(4), val);

	//val = vi_pre_read(dev, MIPI2DMA_CTRL(7));
	val = 0;
	val |= (stride << MIPI2DMA_CTRL7_M0_STRIDE_SHIFT);
	val |= (stride << MIPI2DMA_CTRL7_M1_STRIDE_SHIFT);
	vi_pre_write(dev, MIPI2DMA_CTRL(7), val);

	//val = vi_pre_read(dev, MIPI2DMA_CTRL(8));
	val = 0;
	val |= (stride << MIPI2DMA_CTRL8_M2_STRIDE_SHIFT);
	val |= (stride << MIPI2DMA_CTRL8_M3_STRIDE_SHIFT);
	vi_pre_write(dev, MIPI2DMA_CTRL(8), val);

	//M0 ADDR
	val = vi_pre_read(dev, MIPI2DMA_CTRL(25));
	val |= (0x0 << MIPI2DMA_CTRL25_M0_SADDR_ID0_H_SHIFT);
	vi_pre_write(dev, MIPI2DMA_CTRL(25), val);

	val = vi_pre_read(dev, MIPI2DMA_CTRL(26));
	val |= (0xF0000000 << MIPI2DMA_CTRL26_M0_SADDR_ID0_L_SHIFT);
	vi_pre_write(dev, MIPI2DMA_CTRL(26), val);

	val = vi_pre_read(dev, MIPI2DMA_CTRL(27));
	val |= (0x0 << MIPI2DMA_CTRL27_M0_SADDR_ID1_H_SHIFT);
	vi_pre_write(dev, MIPI2DMA_CTRL(27), val);

	val = vi_pre_read(dev, MIPI2DMA_CTRL(28));
	val |= (0xF1000000 << MIPI2DMA_CTRL28_M0_SADDR_ID1_L_SHIFT);
	vi_pre_write(dev, MIPI2DMA_CTRL(28), val);

	val = vi_pre_read(dev, MIPI2DMA_CTRL(29));
	val |= (0x0 << MIPI2DMA_CTRL29_M0_SADDR_ID2_H_SHIFT);
	vi_pre_write(dev, MIPI2DMA_CTRL(29), val);

	val = vi_pre_read(dev, MIPI2DMA_CTRL(30));
	val |= (0xF2000000 << MIPI2DMA_CTRL30_M0_SADDR_ID2_L_SHIFT);
	vi_pre_write(dev, MIPI2DMA_CTRL(30), val);

	//M1 ADDR
	val = vi_pre_read(dev, MIPI2DMA_CTRL(31));
	val |= (0x0 << MIPI2DMA_CTRL31_M1_SADDR_ID0_H_SHIFT);
	vi_pre_write(dev, MIPI2DMA_CTRL(31), val);

	val = vi_pre_read(dev, MIPI2DMA_CTRL(32));
	val |= (0xF3000000 << MIPI2DMA_CTRL32_M1_SADDR_ID0_L_SHIFT);
	vi_pre_write(dev, MIPI2DMA_CTRL(32), val);

	val = vi_pre_read(dev, MIPI2DMA_CTRL(33));
	val |= (0x0 << MIPI2DMA_CTRL33_M1_SADDR_ID1_H_SHIFT);
	vi_pre_write(dev, MIPI2DMA_CTRL(33), val);

	val = vi_pre_read(dev, MIPI2DMA_CTRL(34));
	val |= (0xF4000000 << MIPI2DMA_CTRL34_M1_SADDR_ID1_L_SHIFT);
	vi_pre_write(dev, MIPI2DMA_CTRL(34), val);

	val = vi_pre_read(dev, MIPI2DMA_CTRL(35));
	val |= (0x0 << MIPI2DMA_CTRL35_M1_SADDR_ID2_H_SHIFT);
	vi_pre_write(dev, MIPI2DMA_CTRL(35), val);

	val = vi_pre_read(dev, MIPI2DMA_CTRL(36));
	val |= (0xF5000000 << MIPI2DMA_CTRL36_M1_SADDR_ID2_L_SHIFT);
	vi_pre_write(dev, MIPI2DMA_CTRL(36), val);

	//M2 ADDR
	val = vi_pre_read(dev, MIPI2DMA_CTRL(37));
	val |= (0x0 << MIPI2DMA_CTRL37_M2_SADDR_ID0_H_SHIFT);
	vi_pre_write(dev, MIPI2DMA_CTRL(37), val);

	val = vi_pre_read(dev, MIPI2DMA_CTRL(38));
	val |= (0xF6000000 << MIPI2DMA_CTRL38_M2_SADDR_ID0_L_SHIFT);
	vi_pre_write(dev, MIPI2DMA_CTRL(38), val);

	val = vi_pre_read(dev, MIPI2DMA_CTRL(39));
	val |= (0x0 << MIPI2DMA_CTRL39_M2_SADDR_ID1_H_SHIFT);
	vi_pre_write(dev, MIPI2DMA_CTRL(39), val);

	val = vi_pre_read(dev, MIPI2DMA_CTRL(40));
	val |= (0xF7000000 << MIPI2DMA_CTRL40_M2_SADDR_ID1_L_SHIFT);
	vi_pre_write(dev, MIPI2DMA_CTRL(40), val);

	val = vi_pre_read(dev, MIPI2DMA_CTRL(41));
	val |= (0x0 << MIPI2DMA_CTRL41_M2_SADDR_ID2_H_SHIFT);
	vi_pre_write(dev, MIPI2DMA_CTRL(41), val);

	val = vi_pre_read(dev, MIPI2DMA_CTRL(42));
	val |= (0xF8000000 << MIPI2DMA_CTRL42_M2_SADDR_ID2_L_SHIFT);
	vi_pre_write(dev, MIPI2DMA_CTRL(42), val);

	//M3 ADDR
	val = vi_pre_read(dev, MIPI2DMA_CTRL(45));
	val |= (0x0 << MIPI2DMA_CTRL45_M3_SADDR_ID0_H_SHIFT);
	vi_pre_write(dev, MIPI2DMA_CTRL(45), val);

	val = vi_pre_read(dev, MIPI2DMA_CTRL(46));
	val |= (0xF9000000 << MIPI2DMA_CTRL46_M3_SADDR_ID0_L_SHIFT);
	vi_pre_write(dev, MIPI2DMA_CTRL(46), val);

	val = vi_pre_read(dev, MIPI2DMA_CTRL(47));
	val |= (0x0 << MIPI2DMA_CTRL47_M3_SADDR_ID1_H_SHIFT);
	vi_pre_write(dev, MIPI2DMA_CTRL(47), val);

	val = vi_pre_read(dev, MIPI2DMA_CTRL(48));
	val |= (0xFa000000 << MIPI2DMA_CTRL48_M3_SADDR_ID1_L_SHIFT);
	vi_pre_write(dev, MIPI2DMA_CTRL(48), val);

	val = vi_pre_read(dev, MIPI2DMA_CTRL(49));
	val |= (0x0 << MIPI2DMA_CTRL49_M3_SADDR_ID2_H_SHIFT);
	vi_pre_write(dev, MIPI2DMA_CTRL(49), val);

	val = vi_pre_read(dev, MIPI2DMA_CTRL(50));
	val |= (0xFb000000 << MIPI2DMA_CTRL50_M3_SADDR_ID2_L_SHIFT);
	vi_pre_write(dev, MIPI2DMA_CTRL(50), val);

	val = vi_pre_read(dev, MIPI2DMA_CTRL(51));
	val |= MIPI2DMA_CTRL51_CROSS_4K_EN;
	vi_pre_write(dev, MIPI2DMA_CTRL(51), val);

	// feedback config to default
	val = 0;
	vi_pre_write(dev, MIPI2DMA_CTRL(11), val);

	val = 0;
	vi_pre_write(dev, MIPI2DMA_CTRL(23), val);

	val = 0;
	vi_pre_write(dev, MIPI2DMA_CTRL(24), val);

	// int status clean & mask
	//val = vi_pre_read(dev, MIPI2DMA_CTRL(44));
	//enable all interrupt
	val = 0;  //open all interrupt
	vi_pre_write(dev, MIPI2DMA_CTRL(44), val);

	val = vi_pre_read(dev, MIPI2DMA_CTRL(10));
	val |= MIPI2DMA_START;
	vi_pre_write(dev, MIPI2DMA_CTRL(10), val);

	return ret;
}

static u16 read_interrupt_status(struct vi_pre_dev *dev)
{
    return vi_pre_read(dev, MIPI2DMA_CTRL(44)) & 0x1ff;
}

static u32 mframe_frame_done_flag(struct vi_pre_dev *dev)
{
    return vi_pre_read(dev, MIPI2DMA_CTRL(43)) & 0xfff;
}

static void mframe_frame_done_flag_clear(struct vi_pre_dev *dev, u32 mask)
{
     u32 reg_val  = vi_pre_read(dev, MIPI2DMA_CTRL(43));
     reg_val |= mask;
     vi_pre_write(dev, MIPI2DMA_CTRL(43), mask);
}
static int get_frame_num(struct vi_pre_dev *dev)
{
    u32 reg_val = vi_pre_read(dev, MIPI2DMA_CTRL(0));
    reg_val &= (3 << 2);
    return (reg_val >> 2) + 1;
}

static void dma_period_clear(struct vi_pre_dev *dev, int ch)
{
    u32 reg_val = vi_pre_read(dev, MIPI2DMA_CTRL(11));
    if (reg_val & (1 << ch)) {
        reg_val &= ~(1 << ch);
    } else {
        reg_val |= (1 << ch);
    }
    vi_pre_write(dev, MIPI2DMA_CTRL(11), reg_val);
}

void vi_pre_interrupt_handler(struct vi_pre_dev *dev)
{
    u32 status = read_interrupt_status(dev);
    u32 f_done_sta = 0;
    int i = 0;

    if (status & VIPRE_FRAME_DONE) {
        f_done_sta = mframe_frame_done_flag(dev);
        for(i = 11; i >= 0; i--) {
            if (f_done_sta & (1 << i)) {
                if (i >= 8) {
                    dev->cnt[0]++;
                } else if (i >= 4) {
                    dev->cnt[1]++;
                } else {
                    dev->cnt[2]++;
                }
            }
        }
        mframe_frame_done_flag_clear(dev, f_done_sta);
        /*TODO send event*/
        //printk("frame done %x\n", f_done_sta);
    } else if (status & VIPRE_LINE_DONE) {
        //u32 nline_done_sta =;
        //pdriver_dev->cnt++;
        //printk("line done %x\n", f_done_sta);
    }
    //u32 line_cnt = pdriver_dev->cnt * get_nline_int_period(pdriver_dev);
    if (dev->is_mframe_mode) {
        for(i = 0; i < 3; i++) {
            if(dev->cnt[i] < get_frame_num(dev)) {
                continue;
            }
            dev->cnt[i] = 0;
            dma_period_clear(dev, i);
        }
    }
}

unsigned int vi_pre_priv_ioctl(struct vi_pre_dev *dev, unsigned int cmd, void *args)
{
	int ret = -1;

	if (!dev) {
		pr_err("%s invalid para\n", __func__);
		return ret;
	}

	switch (cmd) {
	case VI_PRE_IOCTL_RESET:
		ret = 0;
		break;
	case VI_PRE_IOCTL_WRITE_REG:
		ret = vi_pre_write_reg(dev, (void *)args);
		break;
	case VI_PRE_IOCTL_READ_REG:
		ret = vi_pre_read_reg(dev, (void *)args);
		break;
	case VI_PRE_IOCTL_SET_HDRPRO:
		ret = vi_pre_set_hdrpro(dev, args);
		break;
	case VI_PRE_IOCTL_SET_MIPI2DMA_M_FRMAE:
		ret = vi_pre_set_mipi2dma_m_frame(dev, args);
		break;
	case VI_PRE_IOCTL_SET_MIPI2DMA_N_LINE:
		ret = vi_pre_set_mipi2dma_n_line(dev, args);
		break;
	default:
		pr_err("unsupported command %d\n", cmd);
		break;
	}

	return ret;
}
