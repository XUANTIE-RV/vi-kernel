/****************************************************************************
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 VeriSilicon Holdings Co., Ltd.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 *****************************************************************************
 *
 * The GPL License (GPL)
 *
 * Copyright (c) 2020 VeriSilicon Holdings Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program;
 *
 *****************************************************************************
 *
 * Note: This software is released under dual MIT and GPL licenses. A
 * recipient may use this file under the terms of either the MIT license or
 * GPL License. If you wish to use only one license not the other, you can
 * indicate your decision by deleting one of the above license notices in your
 * version of this file.
 *
 *****************************************************************************/
#include <linux/module.h>
#include <linux/uaccess.h>
#if defined(ISP8000L_V2008) || defined(ISP8000_V2009)
#include <linux/io.h>   //Fix thead compile error.
#endif


#include "nwl_regs.h"
#include "csi_ioctl.h"

static int nwl_register_write(void * dev,unsigned int addr, unsigned int data)
{
	void __iomem *base_addr;
	struct vvcam_csi_dev *nwl_csi_dev;

	if (dev == NULL)
		return -1;
	nwl_csi_dev = dev;
	base_addr = nwl_csi_dev->base;

	writel(data, base_addr + addr);

	return 0;
}

#if 0
static int nwl_register_read(void * dev,unsigned int addr, unsigned int *data)
{
	void __iomem *base_addr;
	struct vvcam_csi_dev *nwl_csi_dev;

	if (dev == NULL)
		return -1;
	nwl_csi_dev = dev;
	base_addr = nwl_csi_dev->base;

	*data = readl(base_addr + addr);

	return 0;
}
#endif

int vvnative_csi_set_stream_control(void * dev)
{
	struct vvcam_csi_dev *nwl_csi_dev;
	u32 clock_status;
	u32 data_status;

	if (dev == NULL)
		return -1;
	nwl_csi_dev = dev;

	if (nwl_csi_dev->streaming_enable)
	{
		clock_status = 0x01;
		data_status  = 0xFF;
	}
	else
	{
		clock_status = 0x00;
		data_status  = 0x00;
	}

	nwl_register_write(dev,MRV_MIPICSI_LANES_CLK, clock_status);
	nwl_register_write(dev,MRV_MIPICSI_LANES_DATA, data_status);
	return 0;
}

int vvnative_csi_set_cfg(void * dev)
{
	struct vvcam_csi_dev *nwl_csi_dev;

	if (dev == NULL)
		return -1;
	nwl_csi_dev = dev;

	nwl_register_write(dev,MRV_MIPICSI_NUM_LANES, nwl_csi_dev->csi_lane_cfg.mipi_lane_num);

	return 0;
}

int vvnative_csi_module_init(void * dev)
{
	struct vvcam_csi_dev *nwl_csi_dev;

	if (dev == NULL)
		return -1;
	nwl_csi_dev = dev;

	nwl_register_write(dev,MRV_MIPICSI_NUM_LANES, 0x04);
	nwl_register_write(dev,MRV_MIPICSI_LANES_CLK, 0x01);
	nwl_register_write(dev,MRV_MIPICSI_LANES_DATA, 0x0F);
	nwl_register_write(dev,MRV_MIPICSI_IGNORE_VC, 0x01);

	nwl_register_write(dev,MRV_MIPICSI_FIFO_SENSD_LEVEL, 0x41);
	nwl_register_write(dev,MRV_MIPICSI_VID_VSYNC, 0x20);
	nwl_register_write(dev,MRV_MIPICSI_VID_HSYNC_FP, 0x20);
	nwl_register_write(dev,MRV_MIPICSI_VID_HSYNC, 0x01);
	nwl_register_write(dev,MRV_MIPICSI_VID_HSYNC_BP, 0x20);

	if (nwl_csi_dev->device_idx == 0)
	{
		nwl_register_write(dev,MRV_MIPICSI0_CTRL, 0x4);
	}else
	{
		nwl_register_write(dev,MRV_MIPICSI1_CTRL, 0x4);
	}

	return 0;
}

int vvnative_csi_module_exit(void * dev)
{

	return 0;
}
