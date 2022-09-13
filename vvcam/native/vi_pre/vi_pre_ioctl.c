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
#include "vi_pre_reg.h"

#define check_retval(x)\
	do {\
		if ((x))\
			return -EIO;\
	} while (0)

static void vi_pre_write(struct vi_pre_dev *dec, unsigned int addr, unsigned int val)
{
	writel(val, dec->reg_base + addr);
}

static unsigned int vi_pre_read(struct vi_pre_dev *dev, unsigned int addr)
{
	return readl(dev->reg_base + addr);
}

static int vi_pre_write_reg(struct vi_pre_dev *dev, void *__user args)
{
	struct vi_pre_reg_t reg;
    unsigned int val;

	check_retval(copy_from_user(&reg, args, sizeof(reg)));
    vi_pre_write(dev, reg.offset, reg.value);
	pr_info("%s  write addr 0x%08x val 0x%08x\n", __func__, reg.offset, reg.value);

    val = vi_pre_read(dev, reg.offset);
	pr_info("%s  rad back addr 0x%08x val 0x%08x\n", __func__, reg.offset, val);

	return 0;
}

static int vi_pre_read_reg(struct vi_pre_dev *dev, void *__user args)
{
	struct vi_pre_reg_t reg;

	check_retval(copy_from_user(&reg, args, sizeof(reg)));
    reg.value = vi_pre_read(dev, reg.offset);
	check_retval(copy_to_user(args, &reg, sizeof(reg)));

	//pr_info("%s  addr 0x%08x val 0x%08x\n", __func__, reg.offset, reg.value);
	return 0;
}

static int vi_pre_set_hdrpro_merge_para(struct vi_pre_dev *dev, void *args)
{
    int ret = 0;
    unsigned int i = 0, j = 0, val = 0;
    struct hdrpro_merge_para merge_para;
    struct hdrpro_para *w;
	struct hdrpro_para *b;
	struct hdrpro_para *a;

	check_retval(copy_from_user(&merge_para, args, sizeof(merge_para)));
    a = &merge_para.a;
    b = &merge_para.b;
    w = &merge_para.w;

    for (i = HDRPRO_LINW(1); i <= HDRPRO_LINW(8); i += 4) {
        val = 0;
        val |= (w->para[j] << 16);
        j++;
        val |= w->para[j];
        j++;
	    vi_pre_write(dev, i, val);
    }

    j = 0;
    for (i = HDRPRO_LINA(1); i <= HDRPRO_LINA(16); i += 4) {
        val = 0;
        val |= a->para[j];
	    vi_pre_write(dev, i, val);
        j++;
    }

    j = 0;
    for (i = HDRPRO_LINB(1); i <= HDRPRO_LINB(16); i += 4) {
        val = 0;
        val |= b->para[j];
	    vi_pre_write(dev, i, val);
        j++;
    }

    return ret;
}

static int vi_pre_set_hdrpro_color_para(struct vi_pre_dev *dev, void *args)
{
	int ret = 0;
	unsigned int val = 0;
	struct hdrpro_color_para color_para;

	check_retval(copy_from_user(&color_para, args, sizeof(color_para)));
	val = vi_pre_read(dev, HDRPRO_COE(1));
	val &= ~HDRPRO_COE1_BE_COLOR_WG0_00_MASK;
	val |= (color_para.wg0.pattern_00 << HDRPRO_COE1_BE_COLOR_WG0_00_SHIFT);
	val &= ~HDRPRO_COE1_BE_COLOR_WG0_01_MASK;
	val |= (color_para.wg0.pattern_01 << HDRPRO_COE1_BE_COLOR_WG0_01_SHIFT);
	val &= ~HDRPRO_COE1_BE_COLOR_WG0_02_MASK;
	val |= (color_para.wg0.pattern_02 << HDRPRO_COE1_BE_COLOR_WG0_02_SHIFT);
	val &= ~HDRPRO_COE1_BE_COLOR_WG0_10_MASK;
	val |= (color_para.wg0.pattern_10 << HDRPRO_COE1_BE_COLOR_WG0_10_SHIFT);
	vi_pre_write(dev, HDRPRO_COE(1), val);

	//val = vi_pre_read(dev, HDRPRO_COE(2));
	val = 0;
	val |= (color_para.wg0.pattern_11 << HDRPRO_COE2_BE_COLOR_WG0_11_SHIFT);
	val |= (color_para.wg0.pattern_12 << HDRPRO_COE2_BE_COLOR_WG0_12_SHIFT);
	val |= (color_para.wg0.pattern_20 << HDRPRO_COE2_BE_COLOR_WG0_20_SHIFT);
	val |= (color_para.wg0.pattern_21 << HDRPRO_COE2_BE_COLOR_WG0_21_SHIFT);
	val |= (color_para.wg0.pattern_22 << HDRPRO_COE2_BE_COLOR_WG0_22_SHIFT);
	val |= (color_para.wg1.pattern_00 << HDRPRO_COE2_BE_COLOR_WG1_00_SHIFT);
	val |= (color_para.wg1.pattern_01 << HDRPRO_COE2_BE_COLOR_WG1_01_SHIFT);
	val |= (color_para.wg1.pattern_02 << HDRPRO_COE2_BE_COLOR_WG1_02_SHIFT);
	vi_pre_write(dev, HDRPRO_COE(2), val);

	//val = vi_pre_read(dev, HDRPRO_COE(3));
	val = 0;
	val |= (color_para.wg1.pattern_10 << HDRPRO_COE3_BE_COLOR_WG1_10_SHIFT);
	val |= (color_para.wg1.pattern_11 << HDRPRO_COE3_BE_COLOR_WG1_11_SHIFT);
	val |= (color_para.wg1.pattern_12 << HDRPRO_COE3_BE_COLOR_WG1_12_SHIFT);
	val |= (color_para.wg1.pattern_20 << HDRPRO_COE3_BE_COLOR_WG1_20_SHIFT);
	val |= (color_para.wg1.pattern_21 << HDRPRO_COE3_BE_COLOR_WG1_21_SHIFT);
	val |= (color_para.wg1.pattern_22 << HDRPRO_COE3_BE_COLOR_WG1_22_SHIFT);
	val |= (color_para.wr.pattern_00 << HDRPRO_COE3_BE_COLOR_WR_00_SHIFT);
	val |= (color_para.wr.pattern_01 << HDRPRO_COE3_BE_COLOR_WR_01_SHIFT);
	vi_pre_write(dev, HDRPRO_COE(3), val);

	//val = vi_pre_read(dev, HDRPRO_COE(4));
	val = 0;
	val |= (color_para.wr.pattern_02 << HDRPRO_COE4_BE_COLOR_WR_02_SHIFT);
	val |= (color_para.wr.pattern_10 << HDRPRO_COE4_BE_COLOR_WR_10_SHIFT);
	val |= (color_para.wr.pattern_11 << HDRPRO_COE4_BE_COLOR_WR_11_SHIFT);
	val |= (color_para.wr.pattern_12 << HDRPRO_COE4_BE_COLOR_WR_12_SHIFT);
	val |= (color_para.wr.pattern_20 << HDRPRO_COE4_BE_COLOR_WR_20_SHIFT);
	val |= (color_para.wr.pattern_21 << HDRPRO_COE4_BE_COLOR_WR_21_SHIFT);
	val |= (color_para.wr.pattern_22 << HDRPRO_COE4_BE_COLOR_WR_22_SHIFT);
	val |= (color_para.wb.pattern_00 << HDRPRO_COE4_BE_COLOR_WB_00_SHIFT);
	vi_pre_write(dev, HDRPRO_COE(4), val);

	//val = vi_pre_read(dev, HDRPRO_COE(5));
	val = 0;
	val |= (color_para.wb.pattern_01 << HDRPRO_COE5_BE_COLOR_WB_01_SHIFT);
	val |= (color_para.wb.pattern_02 << HDRPRO_COE5_BE_COLOR_WB_02_SHIFT);
	val |= (color_para.wb.pattern_10 << HDRPRO_COE5_BE_COLOR_WB_10_SHIFT);
	val |= (color_para.wb.pattern_11 << HDRPRO_COE5_BE_COLOR_WB_11_SHIFT);
	val |= (color_para.wb.pattern_12 << HDRPRO_COE5_BE_COLOR_WB_12_SHIFT);
	val |= (color_para.wb.pattern_20 << HDRPRO_COE5_BE_COLOR_WB_20_SHIFT);
	val |= (color_para.wb.pattern_21 << HDRPRO_COE5_BE_COLOR_WB_21_SHIFT);
	val |= (color_para.wb.pattern_22 << HDRPRO_COE5_BE_COLOR_WB_22_SHIFT);
	vi_pre_write(dev, HDRPRO_COE(5), val);

err:
	return ret;
}

static int vi_pre_set_hdrpro_black_para(struct vi_pre_dev *dev, void *args)
{
	int ret = 0;
	unsigned int val = 0;
	struct hdrpro_black_para black_para;

	check_retval(copy_from_user(&black_para, args, sizeof(black_para)));
	val = vi_pre_read(dev, HDRPRO_COE(1));
	val &= ~HDRPRO_COE1_BE_BLACK_WR_MASK;
	val |= (black_para.weight_para.wr << HDRPRO_COE1_BE_BLACK_WR_SHIFT);
	val &= ~HDRPRO_COE1_BE_BLACK_WB_MASK;
	val |= (black_para.weight_para.wb << HDRPRO_COE1_BE_BLACK_WB_SHIFT);
	val &= ~HDRPRO_COE1_BE_BLACK_WG0_MASK;
	val |= (black_para.weight_para.wg0 << HDRPRO_COE1_BE_BLACK_WG0_SHIFT);
	val &= ~HDRPRO_COE1_BE_BLACK_WG1_MASK;
	val |= (black_para.weight_para.wg1 << HDRPRO_COE1_BE_BLACK_WG1_SHIFT);
	vi_pre_write(dev, HDRPRO_COE(1), val);

err:
	return ret;
}

static int vi_pre_set_hdrpro_resolution(struct vi_pre_dev *dev, void *args)
{
	int ret = 0;
	unsigned int val = 0;
	struct hdrpro_resolution resolution;

	check_retval(copy_from_user(&resolution, args, sizeof(resolution)));
	val = vi_pre_read(dev, HDRPRO_CTRL(2));
	val &= ~HDRPRO_CTRL2_VERTICAL_MASK;
	val |= (resolution.height << HDRPRO_CTRL2_VERTICAL_SHIFT);
	val &= ~HDRPRO_CTRL2_HORIZON_MASK;
	val |= (resolution.width << HDRPRO_CTRL2_HORIZON_SHIFT);
	vi_pre_write(dev, HDRPRO_CTRL(2), val);

	return ret;
}

static int vi_pre_set_hdrpro_mode(struct vi_pre_dev *dev, void *args)
{
	int ret = 0;
	unsigned int val = 0;
	struct hdrpro_mode mode;

	check_retval(copy_from_user(&mode, args, sizeof(mode)));
	val = vi_pre_read(dev, HDRPRO_CTRL(1));
	val &= ~HDRPRO_CTRL1_BAYER_MODSEL_MASK;
	val |= mode.bayer_modesel;
	val &= ~HDRPRO_CTRL1_COLOR_MODSEL_MASK;
	val |= mode.color_modesel;
	val &= ~HDRPRO_CTRL1_HDRPRO_RAWMOD_MASK;
	val |= mode.raw_mode;
	vi_pre_write(dev, HDRPRO_CTRL(1), val);

	return ret;
}

static int vi_pre_hdrpro_en(struct vi_pre_dev *dev, int en)
{
	int ret = 0;
	unsigned int val = 0;
	val = vi_pre_read(dev, HDRPRO_CTRL(1));
	val &= ~HDRPRO_CTRL1_HDRPRO_EN_MASK;
	val |= en;
	vi_pre_write(dev, HDRPRO_CTRL(1), val);

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

#if 0
extern void test(struct vi_pre_dev *pdev);
static int vi_pre_set_mipi2dma_m_frame(struct vi_pre_dev *dev, void *args)
{
	int ret = 0;
    test(dev);
#if 0
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
	val |= (0x1 << MIPI2DMA_CTRL25_M0_SADDR_ID0_H_SHIFT);
	vi_pre_write(dev, MIPI2DMA_CTRL(25), val);

	val = vi_pre_read(dev, MIPI2DMA_CTRL(26));
	val |= (0x00000000 << MIPI2DMA_CTRL26_M0_SADDR_ID0_L_SHIFT);
	vi_pre_write(dev, MIPI2DMA_CTRL(26), val);

	val = vi_pre_read(dev, MIPI2DMA_CTRL(27));
	val |= (0x1 << MIPI2DMA_CTRL27_M0_SADDR_ID1_H_SHIFT);
	vi_pre_write(dev, MIPI2DMA_CTRL(27), val);

	val = vi_pre_read(dev, MIPI2DMA_CTRL(28));
	val |= (0x04000000 << MIPI2DMA_CTRL28_M0_SADDR_ID1_L_SHIFT);
	vi_pre_write(dev, MIPI2DMA_CTRL(28), val);

	val = vi_pre_read(dev, MIPI2DMA_CTRL(29));
	val |= (0x1 << MIPI2DMA_CTRL29_M0_SADDR_ID2_H_SHIFT);
	vi_pre_write(dev, MIPI2DMA_CTRL(29), val);

	val = vi_pre_read(dev, MIPI2DMA_CTRL(30));
	val |= (0x08000000 << MIPI2DMA_CTRL30_M0_SADDR_ID2_L_SHIFT);
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
#endif
	return ret;
}
#endif

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
#if 0
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
#endif

typedef struct {
    int glue_idx;
    int h;
    int v;
} vipre_resolution_cfg_t;

int vi_pre_set_resolution(struct vi_pre_dev *dev, void *arg)
{
    u32 val = 0;
    vipre_resolution_cfg_t cfg;
	check_retval(copy_from_user(&cfg, arg, sizeof(cfg)));

    val = (cfg.h << 16) | cfg.v;

    if(cfg.glue_idx == 0) {
	    vi_pre_write(dev, G0_RESCFG, val);
	    vi_pre_write(dev, G0_RESCFG2, val);
	    vi_pre_write(dev, G0_RESCFG3, val);
    } else if(cfg.glue_idx == 1) {
	    vi_pre_write(dev, G1_RESCFG, val);
	    vi_pre_write(dev, G1_RESCFG2, val);
	    vi_pre_write(dev, G1_RESCFG3, val);
    } else if(cfg.glue_idx == 2) {
	    vi_pre_write(dev, G2_RESCFG, val);
	    vi_pre_write(dev, G2_RESCFG2, val);
	    vi_pre_write(dev, G2_RESCFG3, val);
    } else {
        return -1;
    }

    return 0;
}

typedef enum {
    DDR,
    ISP1,
    ISP2,
}img_processor_e;

typedef struct {
    int csi_idx;
    int glue_idx;
    img_processor_e processor;
} vipre_pipline_t;

static void glue0_select_csi(struct vi_pre_dev *dev, int csi_idx)
{
    unsigned int val = 0;
    val = vi_pre_read(dev, G1_MUX3_2);
    val &= ~(3 << 4);
    val |= (csi_idx << 4);
	vi_pre_write(dev, G1_MUX3_2, val);
}

static void glue1_select_csi(struct vi_pre_dev *dev, int csi_idx)
{
    unsigned int val = 0;
    val = vi_pre_read(dev, G1_MUX3_2);
    val &= ~(3 << 2);
    val |= (csi_idx << 2);
	vi_pre_write(dev, G1_MUX3_2, val);
}

static void glue2_select_csi(struct vi_pre_dev *dev, int csi_idx)
{
    unsigned int val = 0;
    val = vi_pre_read(dev, G1_MUX3_2);
    val &= ~3;
    val |= csi_idx;
	vi_pre_write(dev, G1_MUX3_2, val);
}

static void isp2_select_glue(struct vi_pre_dev *dev, int glue_idx)
{
    unsigned int val = 0;
    val = vi_pre_read(dev, G1_MUX3_2);
    val &= ~(1 << 7);
    if (glue_idx == 1) {
        val |= (1 << 7);
    }
	vi_pre_write(dev, G1_MUX3_2, val);
}

static void isp1_select_glue(struct vi_pre_dev *dev, int glue_idx)
{
    unsigned int val = 0;
    val = vi_pre_read(dev, G1_MUX3_2);
    val &= ~(1 << 6);
    if (glue_idx == 2) {
        val |= (1 << 6);
    }
	vi_pre_write(dev, G1_MUX3_2, val);
}

static void ipi_enable(struct vi_pre_dev *dev, int ipi_idx)
{
    unsigned int val = 0;
    val = vi_pre_read(dev, G1_MUX3_2);
    val |= (1 << (10 - ipi_idx));
	vi_pre_write(dev, G1_MUX3_2, val);
}

static void ipi_disable(struct vi_pre_dev *dev, int ipi_idx)
{
    unsigned int val = 0;
    val = vi_pre_read(dev, G1_MUX3_2);
    val &= ~(1 << (10 - ipi_idx));
	vi_pre_write(dev, G1_MUX3_2, val);
}

static int vi_pre_reset(struct vi_pre_dev *dev, void *args)
{
    uint32_t ipi_idx  = 0;
    unsigned int val = 0;
	check_retval(copy_from_user(&ipi_idx, args, sizeof(ipi_idx)));

    val = vi_pre_read(dev, MIPI2DMA_CTRL9);
    val |= 1 << ipi_idx;
	vi_pre_write(dev, MIPI2DMA_CTRL9, val);

    while(vi_pre_read(dev, MIPI2DMA_CTRL9)) {
        ;
    }

    val = (0x3f << 16);
	vi_pre_write(dev, MIPI2DMA_CTRL44, val);

	vi_pre_write(dev, MIPI2DMA_CTRL23, 0);
	vi_pre_write(dev, MIPI2DMA_CTRL24, 0);
	vi_pre_write(dev, MIPI2DMA_CTRL11, 0);
	vi_pre_write(dev, MIPI2DMA_CTRL12, 0);
	vi_pre_write(dev, MIPI2DMA_CTRL13, 0);
	vi_pre_write(dev, MIPI2DMA_CTRL14, 0);

    val = vi_pre_read(dev, MIPI2DMA_CTRL43);
    vi_pre_write(dev,MIPI2DMA_CTRL43,val);
    return  0;
}

static int vi_pre_press_interror(struct vi_pre_dev *dev, void *args)
{
    uint32_t stat = (uint32_t)args;
    int err_flag = 0;

    if (stat & VIPRE_BUS_ERR) {
		pr_err("%s, %d,  vipre bus error!\n", __func__, __LINE__);
        err_flag = 1;
    }

    if (stat & VIPRE_FIFO_OVER) {
		pr_err("%s, %d,  vipre fifo overflow!\n", __func__, __LINE__);
        err_flag = 1;
    }

    if (stat & VIPRE_NMOVERFLOW) {
		pr_err("%s, %d,  vipre nmoverflow!\n", __func__, __LINE__);
        err_flag = 1;
    }

    if(err_flag) {
        return vi_pre_reset(dev, NULL);
    }

    return  0;
}

int vi_pre_set_pipline(struct vi_pre_dev *dev, void *arg)
{

    vipre_pipline_t cfg;
	check_retval(copy_from_user(&cfg, arg, sizeof(cfg)));

    if (cfg.glue_idx == 0) {
        glue0_select_csi(dev, cfg.csi_idx);
    } else if (cfg.glue_idx == 1) {
        glue1_select_csi(dev, cfg.csi_idx);
        if (cfg.processor == DDR) {
            return -1;
        }
    } else if (cfg.glue_idx == 2) {
        glue2_select_csi(dev, cfg.csi_idx);
        if (cfg.processor == DDR) {
            return -1;
        }
    }

    if (cfg.processor == ISP1) {
        isp1_select_glue(dev, cfg.glue_idx);
    } else if (cfg.processor == ISP2) {
        isp2_select_glue(dev, cfg.glue_idx);
    }
    return 0;
}

int vi_pre_set_ipi_mode(struct vi_pre_dev *dev, void *arg)
{
    ipi_mode_cfg_t ipi_mode;
	check_retval(copy_from_user(&ipi_mode, arg, sizeof(ipi_mode)));

    switch(ipi_mode.glue_idx) {
        case 0:
	        vi_pre_write(dev, G0_MODSEL, ipi_mode.mode);
            break;
        case 1:
	        vi_pre_write(dev, G1_MODSEL, ipi_mode.mode);
            break;
        case 2:
	        vi_pre_write(dev, G2_MODSEL, ipi_mode.mode);
            break;
        default:
            return -1;
    }

    return 0;
}

int vi_pre_set_ipi_idnum(struct vi_pre_dev *dev, void *arg)
{
    ipi_idnum_cfg_t cfg;
    uint32_t val = 0;
	check_retval(copy_from_user(&cfg, arg, sizeof(cfg)));
    val = cfg.id_1 | (cfg.id_2 << 2) | (cfg.id_3 << 4)
          | (cfg.id_first_sync << 6) | (cfg.id_second_sync << 8)  | (cfg.id_third_sync << 10);

    switch(cfg.glue_idx) {
        case 0:
	        vi_pre_write(dev, G0_IDNUM, val);
            break;
        case 1:
	        vi_pre_write(dev, G1_IDNUM, val);
            break;
        case 2:
	        vi_pre_write(dev, G2_IDNUM, val);
            break;
        default:
            return -1;
    }

    return 0;
}

int vi_pre_enable_ipi(struct vi_pre_dev *dev, void *arg)
{
    int ipi_idx;
	check_retval(copy_from_user(&ipi_idx, arg, sizeof(ipi_idx)));
    ipi_enable(dev, ipi_idx);

    return 0;
}

int vi_pre_disable_ipi(struct vi_pre_dev *dev, void *arg)
{
    int ipi_idx;
	check_retval(copy_from_user(&ipi_idx, arg, sizeof(ipi_idx)));
    ipi_disable(dev, ipi_idx);

    return 0;
}

extern int vi_pre_dma_config(struct vi_pre_dev *pdriver_dev, void *arg);
extern int vi_pre_dma_start(struct vi_pre_dev *pdriver_dev);
extern int vi_pre_dma_stop(struct vi_pre_dev *pdriver_dev);


unsigned int vi_pre_priv_ioctl(struct vi_pre_dev *dev, unsigned int cmd, void *args)
{
	int ret = -1;

	if (!dev) {
		pr_err("%s invalid para\n", __func__);
		return ret;
	}

	switch (cmd) {
	case VI_PRE_IOCTL_RESET:
		ret = vi_pre_reset(dev, (void *)args);
		break;
	case VI_PRE_IOCTL_WRITE_REG:
		ret = vi_pre_write_reg(dev, (void *)args);
		break;
	case VI_PRE_IOCTL_READ_REG:
		ret = vi_pre_read_reg(dev, (void *)args);
		break;
	case VI_PRE_IOCTL_SET_IPI_RESOLUTION:
        ret = vi_pre_set_resolution(dev, (void*)args);
		break;
	case VI_PRE_IOCTL_SET_PIPLINE:
        ret = vi_pre_set_pipline(dev, (void*)args);
		break;
    case VI_PRE_IOCTL_SET_IPI_MODE:
        ret = vi_pre_set_ipi_mode(dev, (void*)args);
		break;
    case VI_PRE_IOCTL_SET_IPI_IDNUM:
        ret = vi_pre_set_ipi_idnum(dev, (void*)args);
		break;
	case VI_PRE_IOCTL_ENABLE_IPI:
        ret = vi_pre_enable_ipi(dev, (void *)args);
        break;
	case VI_PRE_IOCTL_DISABLE_IPI:
        ret = vi_pre_disable_ipi(dev, (void *)args);
        break;
	case VI_PRE_IOCTL_SET_MIPI2DMA_M_FRMAE:
        ret = vi_pre_dma_config(dev, (void*)args);
		break;
	case VI_PRE_IOCTL_SET_MIPI2DMA_START:
        ret = vi_pre_dma_start(dev);
		break;
	case VI_PRE_IOCTL_SET_MIPI2DMA_STOP:
        ret = vi_pre_dma_stop(dev);
		break;
	case VI_PRE_IOCTL_SET_MIPI2DMA_N_LINE:
		//ret = vi_pre_set_mipi2dma_n_line(dev, args);
        ret = vi_pre_dma_config(dev, (void*)args);
		break;
	case VI_PRE_IOCTL_SET_HDRPRO_MODE:
		ret = vi_pre_set_hdrpro_mode(dev, args);
		break;
	case VI_PRE_IOCTL_HDRPRO_ENABLE:
        ret = vi_pre_hdrpro_en(dev, 1);
		break;
	case VI_PRE_IOCTL_HDRPRO_DISABLE:
        ret = vi_pre_hdrpro_en(dev, 0);
		break;
	case VI_PRE_IOCTL_SET_HDRPRO_RESOLUTION:
		ret = vi_pre_set_hdrpro_resolution(dev, args);
		break;
	case VI_PRE_IOCTL_SET_HDRPRO_BLACK_PARA:
		ret = vi_pre_set_hdrpro_black_para(dev, args);
		break;
	case VI_PRE_IOCTL_SET_HDRPRO_COLOR_PARA:
		ret = vi_pre_set_hdrpro_color_para(dev, args);
		break;
	case VI_PRE_IOCTL_SET_HDRPRO_MERGE_PARA:
        ret = vi_pre_set_hdrpro_merge_para(dev, args);
		break;
	case VI_PRE_IOCTL_PRESS_INTERROR:
        ret = vi_pre_press_interror(dev, args);
		break;
	default:
		pr_err("unsupported command %d\n", cmd);
		break;
	}

	return ret;
}
