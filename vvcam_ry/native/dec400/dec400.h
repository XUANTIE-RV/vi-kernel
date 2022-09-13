/*
 * Verisilicon DEC400 Driver for FalconLite.
 *
 * Author: Wei Weiyu <Weiyu.Wei@verisilicon.com>
 *
 * Copyright (C) 2020 VeriSilicon Microelectronics (Shanghai) Co., Ltd.
 *
 */

#ifndef __DEC_H__
#define __DEC_H__

#include <linux/io.h>
#include <linux/of.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/of_irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_dma.h>
#include <linux/device.h>
#include <linux/videodev2.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>

#define DEC_DEV_NAME			"thead,dec400"
#define DEC400_NAME "dec"
#define DEC400_MAXCNT 3

#define DEC_CHIP_REV			0x00
#define DEC_CHIP_DATA			0x04
#define DEC_CONTROL			0x0800
#define DEC_CONTROL_EX			0x0804
#define DEC_CONTROL_EX2			0x0808
#define DEC_INTR_EN			0x080c
#define DEC_INTR_EN_EX			0x0810
#define DEC_INTR_EN_EX2			0x0814
#define DEC_WRITE_CFG_CH(n)		(0x0980 + (n) * 4)
#define DEC_WRITE_EX_CFG_CH(n)		(0x0a00 + (n) * 4)
#define DEC_WRITE_BUF_BASE_CH(n)	(0x0d80 + (n) * 4)
#define DEC_WRITE_BUF_END_CH(n)		(0x0e80 + (n) * 4)
#define DEC_WRITE_CACHE_BASE_CH(n)	(0x1180 + (n) * 4)
#define DEC_READ_CFG_CH(n)		(0x0880 + (n) * 4)
#define DEC_READ_EX_CFG_CH(n)		(0x0900 + (n) * 4)
#define DEC_READ_BUF_BASE_CH(n)	(0x0A80 + (n) * 4)
#define DEC_READ_BUF_END_CH(n)		(0x0B80 + (n) * 4)
#define DEC_READ_CACHE_BASE_CH(n)	(0x1080 + (n) * 4)


/* dec register bit config */
enum dec_control_regval {
	DEC_FLUSH = 1 << 0,
	DEC_EN = 0 << 1,
	DEC_BYPASS = 1 << 1,
	DEC_RAM_CLK_GATING_EN = 1 << 2,
	DEC_DEBUG_REG_EN = 0 << 3,
	DEC_DEBUG_REG_DIS = 1 << 3,
	DEC_SOFT_RESET = 1 << 4,
	DEC_WRITE_ALIGN_MODE = 1 << 8,
	DEC_HW_FLUSH_DIS = 1 << 16,
	DEC_HW_FLUSH_EN = 0 << 16
};

/* dec register bit config ex */
enum dec_control_ex_regval {
	WRITE_MISS_POLICY = 1 << 19,
	READ_MISS_POLICY = 1 << 29,
};

enum dec_intr_regval {
	DEC_INTR_AXI_BUS_ERR = 1 << 31
};

enum dec_write_cfg_regval {
	DEC_COMPRESS_EN = 1 << 0,
	DEC_FMT_ARGB8 = 0 << 3,
	DEC_FMT_XRGB8 = 1 << 3,
	DEC_FMT_AYUV = 2 << 3,
	DEC_FMT_UYVY = 3 << 3,
	DEC_FMT_YUY2 = 4 << 3,
	DEC_FMT_YUV_ONLY = 5 << 3,
	DEC_FMT_UV_MIX = 6 << 3,
	DEC_FMT_ARGB4 = 7 << 3,
	DEC_FMT_XRGB4 = 8 << 3,
	DEC_FMT_A1RGB5 = 9 << 3,
	DEC_FMT_X1RGB5 = 10 << 3,
	DEC_FMT_R5G6B5 = 11 << 3,
	DEC_FMT_A2R10G10B10 = 15 << 3,
	DEC_FMT_BAYER = 16 << 3,
	DEC_FMT_COEFFICIENT = 18 << 3,
	DEC_FMT_ARGB16 = 19 << 3,
	DEC_FMT_X2RGB10 = 21 << 3,
	DEC_ALIGN_1_BYTE = 0 << 16,
	DEC_ALIGN_16_BYTE = 1 << 16,
	DEC_ALIGN_32_BYTE = 2 << 16,
	DEC_ALIGN_64_BYTE = 3 << 16,
	DEC_TILE8X8_XMAJOR = 0 << 25,
	DEC_TILE8X8_YMAJOR = 1 << 25,
	DEC_TILE16X4 = 2 << 25,
	DEC_TILE8X4 = 3 << 25,
	DEC_TILE4X8 = 4 << 25,
	DEC_RASTER16X4 = 6 << 25,
	DEC_TILE64X4 = 7 << 25,
	DEC_TILE32X4 = 8 << 25,
	DEC_RASTER256X1 = 9 << 25,
	DEC_RASTER128X1 = 10 << 25,
	DEC_RASTER64X4 = 11 << 25,
	DEC_RASTER256X2 = 12 << 25,
	DEC_RASTER128X2 = 13 << 25,
	DEC_RASTER128X4 = 14 << 25,
	DEC_RASTER64X1 = 15 << 25,
	DEC_TILE16X8 = 16 << 25,
	DEC_TILE8X16 = 17 << 25,
	DEC_RASTER512X1 = 18 << 25,
	DEC_RASTER32X4 = 19 << 25,
	DEC_RASTER64X2 = 20 << 25,
	DEC_RASTER32X2 = 21 << 25,
	DEC_RASTER32X1 = 22 << 25,
	DEC_RASTER16X1 = 23 << 25,
	DEC_TILE128X4 = 24 << 25,
	DEC_TILE256X4 = 25 << 25,
	DEC_TILE512X4 = 26 << 25,
	DEC_TILE16X16 = 27 << 25,
	DEC_TILE32X16 = 28 << 25,
	DEC_TILE64X16 = 29 << 25,
	DEC_TILE128X8 = 30 << 25,
	DEC_TILE8X4_S = 31 << 25,
	DEC_TILE16X4_S = 32 << 25,
	DEC_TILE32X4_S = 33 << 25,
	DEC_TILE16X4_LSB = 34 << 25,
	DEC_TILE32X4_LSB = 35 << 25,
	DEC_TILE32X8 = 36 << 25
};

enum dec_write_ex_cfg_regval {
	DEC_BIT_DEPTH_8 = 0 << 16,
	DEC_BIT_DEPTH_10 = 1 << 16,
	DEC_BIT_DEPTH_12 = 2 << 16,
	DEC_BIT_DEPTH_14 = 3 << 16,
	DEC_BIT_DEPTH_16 = 4 << 16
};

enum dec_enable {
	DEC_ON = 0,
	DEC_OFF = 1
};

struct dec_timming {
	u32 h_len;
	u32 h_stride;

	u32 v_len;
	u32 y_total;
	u32 uv_total;
	u32 buffer_offset;
};

static struct dec_timming dec_timing[] = {
	{ 0x280, 0x280, 0x1e0, 0x4b000, 0x25800, 0x96000 }, //640x480
	{ 0x200, 0x200, 0x200, 0x40000, 0x20000, 0x80000 }, //512x512
	{ 0x500, 0x500, 0x2d0, 0xe1000, 0x70800, 0x1C2000 }, //1280x720
	{ 0x780, 0x780, 0x438, 0x1fa400, 0xfd200, 0x3f4800 } //1920x1080
};

struct dec400_dev {
	int id;
	//int irq;
	dev_t devt;
	struct class *class;
	struct cdev cdev;
	struct mutex mutex;

	//enum dec_enable en;

	//enum vs_format fmt;
	//enum vs_vdieo_resolution res;

	//struct device *dev;
	void __iomem *reg_base;

	//struct v4l2_subdev sd;
	//struct vs_video_device *vdev;

	//struct workqueue_struct *work_queue;
	//struct delayed_work q_buf_wk;
};

#endif
