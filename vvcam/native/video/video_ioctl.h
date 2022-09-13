/*
 * Copyright (C) 2021 Alibaba Group Holding Limited
 * Author: liuyitong <yitong.lyt@alibaba-inc.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef _THEAD_VIDEO_IOCTL_H_
#define _THEAD_VIDEO_IOCTL_H_

#include <linux/ioctl.h>

enum {
    VIDEO_GET_PIPLANES = _IO('r', 0),
    VIDEO_GET_PIPLANES_NUM,
    VIDEO_GET_DEV_PARAM,
    VIDEO_SET_PATH_TYPE,
    VIDEO_GET_MEM_POOL_REGION_ID,
};

#endif /* _THEAD_VIDEO_IOCTL_H_*/
