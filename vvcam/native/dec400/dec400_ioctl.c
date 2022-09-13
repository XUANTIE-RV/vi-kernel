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

//#define DEBUG

static void dec400_write(struct dec400_dev *dec, unsigned int addr, unsigned int val)
{
	writel(val, dec->reg_base + addr);
}

static int dec400_mmu_config(struct dec400_dev *dec, void *args)
{
	unsigned int val = 0;
	struct dec400_mmu_para para = {0};

	copy_from_user(&para, args, sizeof(para));

	pr_err("%s, entry\n", __func__);
	val  = 0;
	val |= 0x100008c;  //need confirm with VSI, no any description
	dec400_write(dec, DEC_CONTROL, val); 	//val 0x100008c

	dec400_write(dec, 0x0388, 0x00000001); //0xE2 gcregMMUAHBControlRegAddrs

	// table array base address
	dec400_write(dec, 0x038C, para.page_bable_addr); // 0xE3 gcregMMUAHBTableArrayBaseAddressLowRegAddrs

	//WriteRegister(reg_base, 0x00184, 0x00000001);//gcregMMUConfigurationRegAddrs, flush
	dec400_write(dec, 0x01AC, 0x00000000);//dcreMMUConfigRegAddrs

	return 0;
}

static int dec400_compress_init(struct dec400_dev *dec, void *args)
{
	unsigned int val = 0;
	struct dec400_compress_init compress = {0};

	copy_from_user(&compress, args, sizeof(compress));

	val = 0x100008c;
	val |= (compress.enable_global_bypass) << DEC_CONTROL_DISABLE_COMPRESSION_SHIFT;
	val |= (!compress.enable_hw_flush) << DEC_CONTROL_DISABLE_HW_FLUSH_SHIFT;
	dec400_write(dec, DEC_CONTROL, val); //0x101008c

	//info from VSI cmodel, don't modify
	dec400_write(dec, DEC_CONTROL_EX, 0xa0000);

	//set tile status read/write ID, don't modity
	val = 0x3fe840;
	dec400_write(dec, DEC_CONTROL_EX2, val);

	//enable intr
	dec400_write(dec, DEC_INTR_EN, 0xffffffff);
	dec400_write(dec, DEC_INTR_EN_EX, 0xffffffff);
	dec400_write(dec, DEC_INTR_EN_EX2, 0xffffffff);

	return 0;
}

static int dec400_decompress_init(struct dec400_dev *dec, void *args)
{
	unsigned int val = 0;
	struct dec400_compress_init compress = {0};

	copy_from_user(&compress, args, sizeof(compress));

	val = 0x100008c;
	val |= (compress.enable_global_bypass) << DEC_CONTROL_DISABLE_COMPRESSION_SHIFT;
	val |= (!compress.enable_hw_flush) << DEC_CONTROL_DISABLE_HW_FLUSH_SHIFT;
	dec400_write(dec, DEC_CONTROL, val); //0x100008c

	//info from VSI cmodel, don't modify
	dec400_write(dec, DEC_CONTROL_EX, 0xa0000);

	//set tile status read/write ID, don't modity
	val = 0x3fe840;
	dec400_write(dec, DEC_CONTROL_EX2, val);

	//enable intr
	dec400_write(dec, DEC_INTR_EN, 0xffffffff);
	dec400_write(dec, DEC_INTR_EN_EX, 0xffffffff);
	dec400_write(dec, DEC_INTR_EN_EX2, 0xffffffff);

	return 0;
}

static int dec400_compress_set_buffer(struct dec400_dev *dec, void *args)
{
	unsigned int val = 0;
	struct dec400_compress_para para = {0};

	copy_from_user(&para, args, sizeof(para));

	val = 0;
	val |= (para.enable) << DEC_WRITE_CONFIG_COMPRESS_ENABLE_SHIFT;
	val |= (para.format) << DEC_WRITE_CONFIG_COMPRESS_FORMAT_SHIFT;
	val |= (para.align_mode) << DEC_WRITE_CONFIG_COMPRESS_ALIGN_SHIFT;
	val |= (para.tile_mode) << DEC_WRITE_CONFIG_COMPRESS_TILE_SHIFT;
	dec400_write(dec, DEC_WRITE_CFG_CH(para.channel), val);
	dec400_write(dec, DEC_WRITE_EX_CFG_CH(para.channel), 0);
	dec400_write(dec, DEC_WRITE_BUF_BASE_CH(para.channel), para.physical_stream_start);
	dec400_write(dec, DEC_WRITE_BUF_END_CH(para.channel), para.physical_stream_end);
	dec400_write(dec, DEC_WRITE_CACHE_BASE_CH(para.channel), para.physical_tile_start);
	dec400_write(dec, DEC_FLUSH_CACHE_CH(para.channel), 0xffffffff);

#ifdef DEBUG
	pr_info("%s, dec400 compress0x%x = 0x%x, 0x%x = 0x%x, 0x%x = 0x%x, 0x%x = 0x%x, 0x%x = 0x%x\n",
		__func__,
		DEC_WRITE_CFG_CH(para.channel), val,
		DEC_WRITE_EX_CFG_CH(para.channel), 0,
		DEC_WRITE_BUF_BASE_CH(para.channel), para.physical_stream_start,
		DEC_WRITE_BUF_END_CH(para.channel), para.physical_stream_end,
		DEC_WRITE_CACHE_BASE_CH(para.channel), para.physical_tile_start);
#endif

	return 0;
}

static int dec400_decompress_set_buffer(struct dec400_dev *dec, void *args)
{
	unsigned int val = 0;
	struct dec400_decompress_para para = {0};

	copy_from_user(&para, args, sizeof(para));

	val = 0;
	val |= (para.enable) << DEC_READ_CONFIG_DECOMPRESS_ENABLE_SHIFT;
	val |= (para.format) << DEC_READ_CONFIG_DECOMPRESS_FORMAT_SHIFT;
	val |= (para.align_mode) << DEC_READ_CONFIG_DECOMPRESS_ALIGN_SHIFT;
	val |= (para.tile_mode) << DEC_READ_CONFIG_DECOMPRESS_TILE_SHIFT;
	dec400_write(dec, DEC_READ_CFG_CH(para.channel), val);
	dec400_write(dec, DEC_READ_EX_CFG_CH(para.channel), 0);
	dec400_write(dec, DEC_READ_BUF_BASE_CH(para.channel), para.physical_stream_start);
	dec400_write(dec, DEC_READ_BUF_END_CH(para.channel), para.physical_stream_end);
	dec400_write(dec, DEC_READ_CACHE_BASE_CH(para.channel), para.physical_tile_start);

#ifdef DEBUG
	pr_info("%s, dec400 decompress 0x%x = 0x%x, 0x%x = 0x%x, 0x%x = 0x%x, 0x%x = 0x%x, 0x%x = 0x%x\n",
		__func__,
		DEC_READ_CFG_CH(para.channel), val,
		DEC_READ_EX_CFG_CH(para.channel), 0,
		DEC_READ_BUF_BASE_CH(para.channel), para.physical_stream_start,
		DEC_READ_BUF_END_CH(para.channel), para.physical_stream_end,
		DEC_READ_CACHE_BASE_CH(para.channel), para.physical_tile_start);
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
