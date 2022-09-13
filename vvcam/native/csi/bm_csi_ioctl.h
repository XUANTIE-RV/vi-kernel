/*
 * Copyright (C) 2021 Alibaba Group Holding Limited
 * Author: liuyitong <yitong.lyt@alibaba-inc.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef _BM_CSI_IOCTL_H_
#define _BM_CSI_IOCTL_H_

#include <linux/ioctl.h>

enum {
    BMCSI_IOC_S_RESET = 0x100,
    BMCSI_IOC_S_POWER,
    BMCSI_IOC_G_POWER,
    BMCSI_IOC_S_CLOCK,
    BMCSI_IOC_G_CLOCK,
    BMCSI_IOC_S_STREAM,
    BMCSI_IOC_G_STREAM,
    BMCSI_IOC_S_FMT,
    BMCSI_IOC_G_FMT,
    BMCSI_IOC_S_VC_SELECT,
    BMCSI_IOC_G_VC_SELECT,
    BMCSI_IOC_S_LANE_CFG,
    BMCSI_IOC_INIT,
    BMCSI_IOC_EXIT,
    BMCSI_IOC_WRITE_REG,
    BMCSI_IOC_READ_REG,
    BMCSI_IOC_MAX,
};

struct bm_csi_reg_t {
	unsigned int offset;
	unsigned int value;
};

#endif /* _BM_CSI_IOCTL_H_*/
