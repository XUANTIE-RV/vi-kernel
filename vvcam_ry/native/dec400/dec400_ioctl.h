/*
 * Copyright (C) 2021 Alibaba Group Holding Limited
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef _DEC400_IOC_H_
#define _DEC400_IOC_H_
#include <linux/ioctl.h>

enum {
	DEC400IOC_RESET = 0,
	DEC400IOC_WRITE_REG,
	DEC400IOC_READ_REG,
	DEC400IOC_COMPRESS_INIT,
	DEC400IOC_DECOMPRESS_INIT,
	DEC400IOC_COMPRESS_SET_BUFFER,
	DEC400IOC_DECOMPRESS_SET_BUFFER,
	DEC400IOC_MMU_CONFIG
};
extern unsigned int dec400_priv_ioctl(struct dec400_dev *dev, unsigned int cmd, void *args);

#endif /* _DEC400_IOC_H_ */
