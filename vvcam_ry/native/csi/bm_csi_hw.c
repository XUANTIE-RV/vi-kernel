/*
 * Copyright (C) 2021 Alibaba Group Holding Limited
 * Author: liuyitong <yitong.lyt@alibaba-inc.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <asm/io.h>

#include <linux/cdev.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/mm.h>

#include <linux/timer.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/ioctl.h>
#include <linux/poll.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/debugfs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>

#include <linux/of.h>

#include "bm_printk.h"
#include "bm_csi_ioctl.h"
#include "bm_csi_hw.h"
#include "dw-dphy-rx.h"
#include "bm_csi_dphy.h"

#define check_retval(x)\
	do {\
		if ((x))\
			return -EIO;\
	} while (0)

int bm_csi_write_reg(struct bm_csi_drvdata *drvdata, void *__user args)
{
	struct bm_csi_reg_t reg;
	check_retval(copy_from_user(&reg, args, sizeof(reg)));
	writel(reg.value, drvdata->base + reg.offset);
	bm_info("%s  addr 0x%08x val 0x%08x\n", __func__, reg.offset, reg.value);
	return 0;
}

int bm_csi_read_reg(struct bm_csi_drvdata *drvdata, void *__user args)
{
	struct bm_csi_reg_t reg;
	check_retval(copy_from_user(&reg, args, sizeof(reg)));
	reg.value = readl(drvdata->base + reg.offset);
	check_retval(copy_to_user(args, &reg, sizeof(reg)));
	bm_info("%s  addr 0x%08x val 0x%08x\n", __func__, reg.offset, reg.value);
	return 0;
}

int bm_csi_init(struct bm_csi_drvdata *drvdata, void *__user args)
{
	bm_info("%s: csi init\n", __func__);
	return 0;
}

int bm_csi_exit(struct bm_csi_drvdata *drvdata, void *__user args)
{
	bm_info("%s: csi exit\n", __func__);
	return 0;
}

int bm_csi_reset(struct bm_csi_drvdata *drvdata, void *__user args)
{
	bm_info("%s: csi reset success\n", __func__);
    dw_csi_soc_reset(drvdata->reset);
    //bm_csi_dphy_reset();
	return 0;
}

static int csi_power_on_sta = 0;
int bm_csi_set_power(struct bm_csi_drvdata *drvdata, void *__user args)
{
	bm_info("%s: csi set power\n", __func__);
	check_retval(copy_from_user(&csi_power_on_sta, args, sizeof(csi_power_on_sta)));
    dw_mipi_csi_s_power(&drvdata->csi_dev, csi_power_on_sta);
	return 0;
}

int bm_csi_get_power(struct bm_csi_drvdata *drvdata, void *__user args)
{
	bm_info("%s: csi get power\n", __func__);
	check_retval(copy_to_user(args, &csi_power_on_sta, sizeof(csi_power_on_sta)));
	return 0;
}

int bm_csi_set_clock(struct bm_csi_drvdata *drvdata, void *__user args)
{
    bm_info("%s: \n", __func__);
	check_retval(copy_to_user(args, &csi_power_on_sta, sizeof(csi_power_on_sta)));
	return 0;
}

int bm_csi_get_clock(struct bm_csi_drvdata *drvdata, void *__user args)
{
	bm_info("%s: \n", __func__);
	return 0;
}

int bm_csi_set_stream(struct bm_csi_drvdata *drvdata, void *__user args)
{
	bm_info("%s: \n", __func__);
	return 0;
}

int bm_csi_get_stream(struct bm_csi_drvdata *drvdata, void *__user args)
{
	bm_info("%s: \n", __func__);
	return 0;
}

int bm_csi_set_fmt(struct bm_csi_drvdata *drvdata, void *__user args)
{
	bm_info("%s: \n", __func__);
	return 0;
}

int bm_csi_get_fmt(struct bm_csi_drvdata *drvdata, void *__user args)
{
	bm_info("%s: \n", __func__);
	return 0;
}

int bm_csi_set_vc_select(struct bm_csi_drvdata *drvdata, void *__user args)
{
	bm_info("%s: \n", __func__);
	return 0;
}

int bm_csi_get_vc_select(struct bm_csi_drvdata *drvdata, void *__user args)
{
	bm_info("%s: \n", __func__);
	return 0;
}

int bm_csi_set_lane_cfg(struct bm_csi_drvdata *drvdata, void *__user args)
{
	bm_info("%s: \n", __func__);
	return 0;
}
