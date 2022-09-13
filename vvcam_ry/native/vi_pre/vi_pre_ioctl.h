/*
 * Copyright (C) 2021 Alibaba Group Holding Limited
 * Author: Shenwuyi <shenwuyi.swy@alibaba-inc.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _VI_PRE_IOC_H_
#define _VI_PRE_IOC_H_
#include <linux/ioctl.h>

struct vi_pre_reg_t {
	unsigned int offset;
	unsigned int value;
};

enum {
	VI_PRE_IOCTL_RESET = 0,
	VI_PRE_IOCTL_WRITE_REG,
	VI_PRE_IOCTL_READ_REG,
	VI_PRE_IOCTL_SET_MIPI2DMA_M_FRMAE,
	VI_PRE_IOCTL_SET_MIPI2DMA_N_LINE,
	VI_PRE_IOCTL_SET_HDRPRO,
	VI_PRE_IOCTL_MAX
};
extern unsigned int vi_pre_priv_ioctl(struct vi_pre_dev *dev, unsigned int cmd, void *args);

#endif /* _VI_PRE_IOC_H_ */
