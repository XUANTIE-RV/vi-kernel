/*
 * Copyright (C) 2021 Alibaba Group Holding Limited
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/regmap.h>
#include <linux/of_reserved_mem.h>
#include <linux/io.h>
#include "dec400.h"
#include "dec400_ioctl.h"

#define DEC400_WRITE_PHYSICAL_ADDR 0xc2a00000
//#define DEC400_WRITE_PHYSICAL_ADDR 0xe10b8000

#define DEC400_READ_PHYSICAL_ADDR 0xe0ac8000

#define RES 3
#define DEBUG

static void dec400_write(struct dec400_dev *dec, unsigned int addr, unsigned int val)
{
	writel(val, dec->reg_base + addr);
}

static void dec400_set_bypass(struct dec400_dev *dec, void *args)
{
	dec400_write(dec, DEC_CONTROL, DEC_BYPASS);
}

static int dec400_compress_init(struct dec400_dev *dec, void *args)
{
	unsigned int val = 0;

	//val  = 0;
	//val |= DEC_EN;
	//val |= 0x00FC0000;  //need confirm with VSI, no any description
	//dec400_write(dec, DEC_CONTROL, val); 	//val 0x00FC0000
	dec400_write(dec, DEC_CONTROL, 0x100008c); //0x101008c

	//val = 0;
	//val |= WRITE_MISS_POLICY;
	//dec400_write(dec, DEC_CONTROL_EX, val);//0x00080000
	dec400_write(dec, DEC_CONTROL_EX, 0xa0000);

	//Tile status cache's AXI read/write ID.
	//val = 0x00410104;
	//dec400_write(dec, DEC_CONTROL_EX2, val);
	dec400_write(dec, DEC_CONTROL_EX2, 0x3fe840);

	//enable intr
	dec400_write(dec, 0x80c, 0xffffffff);
	dec400_write(dec, 0x810, 0xffffffff);
	dec400_write(dec, 0x814, 0xffffffff);

	return 0;
}

static int dec400_mmu_config(struct dec400_dev *dec, void *args)
{
	#define ISP0_PAGE_TABLE_ARRAY_ADDR 	0xEEFFFFF0
	unsigned int val = 0;

	val  = 0;
	val |= DEC_EN;
	val |= 0x00FC0000;  //need confirm with VSI, no any description
	dec400_write(dec, DEC_CONTROL, val); 	//val 0x00FC0000

	dec400_write(dec, 0x0388, 0x00000001); //0xE2 gcregMMUAHBControlRegAddrs

	// table array base address
	//0xFEFFFFF0
	dec400_write(dec, 0x038C, ISP0_PAGE_TABLE_ARRAY_ADDR); // 0xE3 gcregMMUAHBTableArrayBaseAddressLowRegAddrs

	//WriteRegister(reg_base, 0x00184, 0x00000001);//gcregMMUConfigurationRegAddrs, flush
	dec400_write(dec, 0x01AC, 0x00000000);//dcreMMUConfigRegAddrs

	return 0;
}

static int dec400_compress_set_buffer(struct dec400_dev *dec, void *args)
{
	unsigned int y_start = 0, y_end = 0, y_cache = 0;
	unsigned int uv_start = 0, uv_end = 0, uv_cache = 0;
	unsigned int val = 0;

	y_start = DEC400_WRITE_PHYSICAL_ADDR;
	y_end = y_start + dec_timing[RES].y_total - 1;
	y_cache = y_start + dec_timing[RES].buffer_offset;

	uv_start = y_start + dec_timing[RES].y_total;
	uv_end = uv_start + dec_timing[RES].uv_total - 1;
	uv_cache = y_cache + dec_timing[RES].uv_total;

#ifdef DEBUG
	pr_info("%s, dec400 y_start = 0x%x, y_end = 0x%x, uv_start = 0x%x, uv_end = 0x%x\n",
			__func__, y_start, y_end, uv_start, uv_end);
	pr_info("%s, dec400 y_cache = 0x%x, uv_cache = 0x%x\n", __func__, y_cache, uv_cache);
#endif

	val = 0;
	val |= DEC_COMPRESS_EN;
	val |= DEC_FMT_YUV_ONLY;
	val |= DEC_ALIGN_32_BYTE;
	val |= DEC_RASTER128X1;
	dec400_write(dec, DEC_WRITE_CFG_CH(0), val);
	dec400_write(dec, DEC_WRITE_EX_CFG_CH(0), 0);
	dec400_write(dec, DEC_WRITE_BUF_BASE_CH(0), y_start);
	dec400_write(dec, DEC_WRITE_BUF_END_CH(0), y_end);
	dec400_write(dec, DEC_WRITE_CACHE_BASE_CH(0), y_cache);
	dec400_write(dec, 0xf80, 0xffffffff);

#ifdef DEBUG
	pr_info("%s, dec400 0x%x = 0x%x, 0x%x = 0x%x, 0x%x = 0x%x, 0x%x = 0x%x, 0x%x = 0x%x\n",
		__func__,
		DEC_WRITE_CFG_CH(0), val,
		DEC_WRITE_EX_CFG_CH(0), 0,
		DEC_WRITE_BUF_BASE_CH(0), y_start,
		DEC_WRITE_BUF_END_CH(0), y_end,
		DEC_WRITE_CACHE_BASE_CH(0), y_cache);
#endif


	val = 0;
	val |= DEC_COMPRESS_EN;
	val |= DEC_FMT_UV_MIX;
	val |= DEC_ALIGN_32_BYTE;
	val |= DEC_RASTER64X1;
	dec400_write(dec, DEC_WRITE_CFG_CH(1), val);
	dec400_write(dec, DEC_WRITE_EX_CFG_CH(1), 0);
	dec400_write(dec, DEC_WRITE_BUF_BASE_CH(1), uv_start);
	dec400_write(dec, DEC_WRITE_BUF_END_CH(1), uv_end);
	dec400_write(dec, DEC_WRITE_CACHE_BASE_CH(1), uv_cache);
	dec400_write(dec, 0xf84, 0xffffffff);

#ifdef DEBUG
	pr_info("%s, dec400+1 0x%x = 0x%x, 0x%x = 0x%x, 0x%x = 0x%x, 0x%x = 0x%x, 0x%x = 0x%x\n",
		__func__,
		DEC_WRITE_CFG_CH(1), val,
		DEC_WRITE_EX_CFG_CH(1), 0,
		DEC_WRITE_BUF_BASE_CH(1), uv_start,
		DEC_WRITE_BUF_END_CH(1), uv_end,
		DEC_WRITE_CACHE_BASE_CH(1), uv_cache);
#endif

	return 0;
}

static int dec400_decompress_init(struct dec400_dev *dec, void *args)
{
	unsigned int val = 0;

	//val  = 0;
	//val |= DEC_EN;
	//val |= 0x00FC0000;  //need confirm with VSI, no any description
	//dec400_write(dec, DEC_CONTROL, val); 	//val 0x00FC0000
	dec400_write(dec, DEC_CONTROL, 0x100008c); //0x101008c

	//val = 0;
	//val |= WRITE_MISS_POLICY;
	//dec400_write(dec, DEC_CONTROL_EX, val);//0x00080000
	dec400_write(dec, DEC_CONTROL_EX, 0xa0000);

	//Tile status cache's AXI read/write ID.
	//val = 0x00410104;
	//dec400_write(dec, DEC_CONTROL_EX2, val);
	dec400_write(dec, DEC_CONTROL_EX2, 0x3fe840);

	//enable intr
	dec400_write(dec, 0x80c, 0xffffffff);
	dec400_write(dec, 0x810, 0xffffffff);
	dec400_write(dec, 0x814, 0xffffffff);

	return 0;
}

static int dec400_decompress_set_buffer(struct dec400_dev *dec, void *args)
{
	unsigned int y_start = 0, y_end = 0, y_cache = 0;
	unsigned int uv_start = 0, uv_end = 0, uv_cache = 0;
	unsigned int val = 0;

	y_start = DEC400_READ_PHYSICAL_ADDR;
	y_end = y_start + dec_timing[RES].y_total - 1;
	//y_cache = y_start + dec_timing[RES].buffer_offset;
	y_cache = DEC400_WRITE_PHYSICAL_ADDR + dec_timing[RES].buffer_offset;

	uv_start = y_start + dec_timing[RES].y_total;
	uv_end = uv_start + dec_timing[RES].uv_total - 1;
	uv_cache = y_cache + dec_timing[RES].uv_total;

#ifdef DEBUG
	pr_info("%s, dec400 y_start = 0x%x, y_end = 0x%x, uv_start = 0x%x, uv_end = 0x%x\n",
			__func__, y_start, y_end, uv_start, uv_end);
	pr_info("%s, dec400 y_cache = 0x%x, uv_cache = 0x%x\n", __func__, y_cache, uv_cache);
#endif
#if	0
	dec400_write(dec, 0x00000880, 0x14020029);
	dec400_write(dec, 0x00000900, 0x00000000);
	dec400_write(dec, 0x00000A80, y_start);
	dec400_write(dec, 0x00001080, y_cache);
	dec400_write(dec, 0x00000B80, y_end);
	dec400_write(dec, 0x00000884, 0x1E020031);
	dec400_write(dec, 0x00000904, 0x00000000);
	dec400_write(dec, 0x00000A84, uv_start);
	dec400_write(dec, 0x00001084, uv_cache);
	dec400_write(dec, 0x00000B84, uv_end);
#endif
	val = 0;
	val |= DEC_COMPRESS_EN;
	val |= DEC_FMT_YUV_ONLY;
	val |= DEC_ALIGN_32_BYTE;
	val |= DEC_RASTER128X1;
	dec400_write(dec, DEC_READ_CFG_CH(0), val);
	dec400_write(dec, DEC_READ_EX_CFG_CH(0), 0);
	dec400_write(dec, DEC_READ_BUF_BASE_CH(0), y_start);
	dec400_write(dec, DEC_READ_BUF_END_CH(0), y_end);
	dec400_write(dec, DEC_READ_CACHE_BASE_CH(0), y_cache);

#ifdef DEBUG
	pr_info("%s, dec400+1 0x%x = 0x%x, 0x%x = 0x%x, 0x%x = 0x%x, 0x%x = 0x%x, 0x%x = 0x%x\n",
		__func__,
		DEC_READ_CFG_CH(0), val,
		DEC_READ_EX_CFG_CH(0), 0,
		DEC_READ_BUF_BASE_CH(0), y_start,
		DEC_READ_BUF_END_CH(0), y_end,
		DEC_READ_CACHE_BASE_CH(0), y_cache);
#endif
	val = 0;
	val |= DEC_COMPRESS_EN;
	val |= DEC_FMT_UV_MIX;
	val |= DEC_ALIGN_32_BYTE;
	val |= DEC_RASTER64X1;
	dec400_write(dec, DEC_READ_CFG_CH(1), val);
	dec400_write(dec, DEC_READ_EX_CFG_CH(1), 0);
	dec400_write(dec, DEC_READ_BUF_BASE_CH(1), uv_start);
	dec400_write(dec, DEC_READ_BUF_END_CH(1), uv_end);
	dec400_write(dec, DEC_READ_CACHE_BASE_CH(1), uv_cache);

#ifdef DEBUG
	pr_info("%s, dec400+1 0x%x = 0x%x, 0x%x = 0x%x, 0x%x = 0x%x, 0x%x = 0x%x, 0x%x = 0x%x\n",
		__func__,
		DEC_READ_CFG_CH(1), val,
		DEC_READ_EX_CFG_CH(1), 0,
		DEC_READ_BUF_BASE_CH(1), uv_start,
		DEC_READ_BUF_END_CH(1), uv_end,
		DEC_READ_CACHE_BASE_CH(1), uv_cache);
#endif
	return 0;
}

unsigned int dec400_priv_ioctl(struct dec400_dev *dev, unsigned int cmd, void *args)
{
	int ret = -1;

	if (!dev) {
		pr_err("%s invalid para\n", __func__);
		return ret;
	}

	switch (cmd) {
	case DEC400IOC_RESET:
		ret = 0;
		break;
	case DEC400IOC_COMPRESS_INIT:
		ret = dec400_compress_init(dev, args);
		break;
	case DEC400IOC_COMPRESS_SET_BUFFER:
		ret = dec400_compress_set_buffer(dev, args);
		break;
	case DEC400IOC_DECOMPRESS_INIT:
		ret = dec400_decompress_init(dev, args);
		break;
	case DEC400IOC_DECOMPRESS_SET_BUFFER:
		ret = dec400_decompress_set_buffer(dev, args);
		break;
	case DEC400IOC_MMU_CONFIG:
		ret = dec400_mmu_config(dev, args);
		break;
	default:
		pr_err("unsupported command %d\n", cmd);
		break;
	}

	return ret;
}
