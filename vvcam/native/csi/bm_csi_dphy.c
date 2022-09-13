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
#include "bm_csi_dphy.h"
#include "dw-dphy-rx.h"

#define check_retval(x)\
	do {\
		if ((x))\
			return -EIO;\
	} while (0)

#define REG_DPHY_OFFSET 0x40

int bm_csi_dphy_write_reg(struct bm_csi_drvdata *drvdata, void *__user args)
{
	struct bm_csi_reg_t reg;
	check_retval(copy_from_user(&reg, args, sizeof(reg)));
    struct dw_dphy_rx *dphy = drvdata->dphy;

	writel(reg.value, dphy->base_address + reg.offset);
	bm_info("%s  addr 0x%08x val 0x%08x\n", __func__, reg.offset, reg.value);
}

int bm_csi_dphy_init(struct bm_csi_drvdata *drvdata, void *__user args)
{
    struct dw_dphy_rx *dphy = drvdata->dphy;
/*
    dphy->dphy_freq = 20000000;
    dphy->phy_type = 1;
    dphy->dphy_te_len = BIT12;
    dphy->lanes_config = CTRL_4_LANES;
    dphy->dphy_gen = GEN3;
    dphy->max_lanes = CTRL_4_LANES;
	dphy->lp_time = 1000; //ns
	dphy->base_address = drvdata->base + REG_DPHY_OFFSET;
    //dphy->dphy1_if_addr =visysreg
    dw_dphy_reset(dphy);
    dw_dphy_power_on(dphy);
    dw_dphy_init(dphy);
    */
    return 0;
}

int bm_csi_dphy_uinit(struct bm_csi_drvdata *drvdata, void *__user args)
{
    struct dw_dphy_rx *dphy = &drvdata->dphy;
    dw_dphy_reset(dphy);
    dw_dphy_power_off(dphy);
}

int bm_csi_dphy_reset(struct bm_csi_drvdata *drvdata, void *__user args)
{
    struct dw_dphy_rx *dphy = &drvdata->dphy;
    dw_dphy_reset(dphy);
}
