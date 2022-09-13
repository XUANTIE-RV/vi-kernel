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
#define DEC_FLUSH_CACHE_CH(n)		(0x0F80 + (n) * 4)
#define DEC_READ_CACHE_BASE_CH(n)	(0x1080 + (n) * 4)

//gcregAHBDECControl
#define DEC_CONTROL_DISABLE_COMPRESSION_SHIFT 1
#define DEC_CONTROL_DISABLE_RAM_CLOCK_GATING_SHIFT 2
#define DEC_CONTROL_DISABLE_DISABLE_DEBUG_REG_SHIFT 3
#define DEC_CONTROL_DISABLE_HW_FLUSH_SHIFT 16

//gcregAHBDECControlEx2
#define DEC_CONTROLEX2_TILE_STATUS_READ_ID 0
#define DEC_CONTROLEX2_TILE_STATUS_WRITE_ID 7
#define DEC_CONTROLEX2_WR_OT_CNT 14

#define DEC_READ_CONFIG_DECOMPRESS_ENABLE_SHIFT 0
#define DEC_READ_CONFIG_DECOMPRESS_SIZE_SHIFT 1
#define DEC_READ_CONFIG_DECOMPRESS_FORMAT_SHIFT 3
#define DEC_READ_CONFIG_DECOMPRESS_ALIGN_SHIFT 16
#define DEC_READ_CONFIG_DECOMPRESS_TILE_SHIFT 25

#define DEC_WRITE_CONFIG_COMPRESS_ENABLE_SHIFT 0
#define DEC_WRITE_CONFIG_COMPRESS_SIZE_SHIFT 1
#define DEC_WRITE_CONFIG_COMPRESS_FORMAT_SHIFT 3
#define DEC_WRITE_CONFIG_COMPRESS_ALIGN_SHIFT 16
#define DEC_WRITE_CONFIG_COMPRESS_TILE_SHIFT 25


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
