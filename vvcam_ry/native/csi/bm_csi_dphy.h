/*
 * Copyright (C) 2021 Alibaba Group Holding Limited
 * Author: liuyitong <yitong.lyt@alibaba-inc.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef _BM_CSI_DPHY_H_
#define _BM_CSI_DPHY_H_

#include <linux/platform_device.h>

int dw_dphy_rx_probe(struct platform_device *pdev, void __iomem *dphy1_if_addr);
int dw_dphy_rx_remove(struct platform_device *pdev);

#endif /*_BM_CSI_DPHY_H_ */


