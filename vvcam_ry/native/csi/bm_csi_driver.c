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
//#include "dw-csi-plat.h"

#define BM_DRIVER_NAME "vivcsi"
#define BM_DRIVER_MAXCNT 3

static struct class *bm_driver_class;
static unsigned int bm_driver_major = 0;
static unsigned int bm_driver_minor = 0;
static unsigned int device_register_index = 0;

#define check_retval(x)\
	do {\
		if ((x))\
			return -EIO;\
	} while (0)

static unsigned int bm_csi_poll(struct file * filp, poll_table *wait)
{
	return 0;
}

void bm_csi_work(struct work_struct *work)
{
}

irqreturn_t bm_csi_irq(int irq, void *dev_id)
{
	bm_info("enter %s\n", __func__);
	return IRQ_HANDLED;
}

static int bm_csi_open(struct inode * inode, struct file * file)
{
	struct bm_csi_drvdata *drvdata;

	bm_info("enter %s\n", __func__);

	drvdata = container_of(inode->i_cdev, struct bm_csi_drvdata, cdev);
	file->private_data = drvdata;

	return 0;
};

static long bm_csi_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret = 0;
	struct bm_csi_drvdata *drvdata;
	bm_info("enter %s\n", __func__);

	drvdata = file->private_data;
	if (drvdata == NULL) {
		bm_err("%s:file private is null point error\n", __func__);
		return  -ENOMEM;
	}

	mutex_lock(&drvdata->mutex);
	switch (cmd) {
	case BMCSI_IOC_WRITE_REG:
		ret = bm_csi_write_reg(drvdata, (void *)arg);
		break;
	case BMCSI_IOC_READ_REG:
		ret = bm_csi_read_reg(drvdata, (void *)arg);
		break;
    case BMCSI_IOC_INIT:
        ret = bm_csi_init(drvdata, (void *)arg);
        break;
    case BMCSI_IOC_EXIT:
        ret = bm_csi_exit(drvdata, (void *)arg);
        break;
    case BMCSI_IOC_S_RESET:
        ret = bm_csi_reset(drvdata, (void *)arg);
		break;
    case BMCSI_IOC_S_POWER:
        ret = bm_csi_set_power(drvdata, (void *)arg);
		break;
    case BMCSI_IOC_G_POWER:
        ret = bm_csi_get_power(drvdata, (void *)arg);
		break;
    case BMCSI_IOC_S_CLOCK:
        ret = bm_csi_set_clock(drvdata, (void *)arg);
		break;
    case BMCSI_IOC_G_CLOCK:
        ret = bm_csi_get_clock(drvdata, (void *)arg);
		break;
    case BMCSI_IOC_S_STREAM:
        ret = bm_csi_set_stream(drvdata, (void *)arg);
		break;
    case BMCSI_IOC_G_STREAM:
        ret = bm_csi_get_stream(drvdata, (void *)arg);
		break;
    case BMCSI_IOC_S_FMT:
        ret = bm_csi_set_fmt(drvdata, (void *)arg);
		break;
    case BMCSI_IOC_G_FMT:
        ret = bm_csi_get_fmt(drvdata, (void *)arg);
		break;
    case BMCSI_IOC_S_VC_SELECT:
        ret = bm_csi_set_vc_select(drvdata, (void *)arg);
		break;
    case BMCSI_IOC_G_VC_SELECT:
        ret = bm_csi_get_vc_select(drvdata, (void *)arg);
		break;
    case BMCSI_IOC_S_LANE_CFG:
        ret = bm_csi_set_lane_cfg(drvdata, (void *)arg);
		break;
    case BMCSI_IOC_MAX:
		break;
	default:
		ret = -EPERM;
		bm_err("%s: unsupported command %d", __func__, cmd);
		break;
	}
	mutex_unlock(&drvdata->mutex);
	return ret;
};

static int bm_csi_release(struct inode * inode, struct file * file)
{
	bm_info("enter %s\n", __func__);
	return 0;
};

static int bm_csi_mmap(struct file *pFile, struct vm_area_struct *vma)
{
	bm_info("enter %s\n", __func__);
	return 0;
};

struct file_operations bm_csi_fops = {
	.owner = THIS_MODULE,
	.open = bm_csi_open,
	.release = bm_csi_release,
	.unlocked_ioctl = bm_csi_ioctl,
	.mmap = bm_csi_mmap,
	.poll = bm_csi_poll,
};

static int bm_csi_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct bm_csi_drvdata *drvdata;
	struct resource *iores_mem;
    void __iomem *visys_addr;

	u32 value;

	bm_info("enter %s\n", __func__);
	pdev->id = device_register_index;
	if (pdev->id >= BM_DRIVER_MAXCNT) {
		bm_err("%s:pdev id is %d error\n", __func__, pdev->id);
		return  -EINVAL;
	}

	drvdata = devm_kzalloc(&pdev->dev,sizeof(struct bm_csi_drvdata), GFP_KERNEL);
	if (drvdata == NULL) {
		bm_err("%s:alloc struct drvdata error\n", __func__);
		return  -ENOMEM;
	}

	iores_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	drvdata->base = devm_ioremap_resource(&pdev->dev, iores_mem);
	bm_info("%s: [%s%d]: drvdata->base=0x%px, phy_addr base=0x%llx\n", __func__,
		BM_DRIVER_NAME, pdev->id, drvdata->base, iores_mem->start);
	drvdata->device_idx = pdev->id;
	mutex_init(&drvdata->mutex);
	drvdata->irq_num = platform_get_irq(pdev, 0);
	bm_info("%s:[%s%d]: pdriver_dev->irq_num=%d\n", __func__, "BM_CSI", pdev->id, drvdata->irq_num);

	platform_set_drvdata(pdev, drvdata);

	if (pdev->id == 0) {
		if (bm_driver_major == 0) {
			ret = alloc_chrdev_region(&drvdata->devt, 0, BM_DRIVER_MAXCNT, BM_DRIVER_NAME);
			if (ret != 0) {
				bm_err("%s:alloc_chrdev_region error\n", __func__);
				return ret;
			}
			bm_driver_major = MAJOR(drvdata->devt);
			bm_driver_minor = MINOR(drvdata->devt);
		} else {
			drvdata->devt = MKDEV(bm_driver_major, bm_driver_minor);
			ret = register_chrdev_region(drvdata->devt, BM_DRIVER_MAXCNT, BM_DRIVER_NAME);
			if (ret) {
				bm_err("%s:register_chrdev_region error\n", __func__);
				return ret;
			}
		}

		bm_driver_class = class_create(THIS_MODULE, BM_DRIVER_NAME);
		if (IS_ERR(bm_driver_class)) {
			bm_err("%s[%d]:class_create error!\n", __func__, __LINE__);
			return -EINVAL;
		}
	}

	drvdata->devt = MKDEV(bm_driver_major, bm_driver_minor + pdev->id);
	cdev_init(&drvdata->cdev, &bm_csi_fops);
	ret = cdev_add(&drvdata->cdev, drvdata->devt, 1);
	if ( ret ) {
		bm_err("%s[%d]:cdev_add error!\n", __func__, __LINE__);
		return ret;
	} else {
		bm_info("%s[%d]:cdev_add OK, major=%d, minor=%d\n", __func__, __LINE__,
			bm_driver_major, bm_driver_minor + pdev->id);
	}
	drvdata->class = bm_driver_class;
	device_create(drvdata->class, NULL, drvdata->devt,
		      drvdata, "%s%d", BM_DRIVER_NAME, pdev->id);

    /*read version*/
	value = readl(drvdata->base + 0x0);
	bm_info("offset=04, value is:0x%08x\n", value);

	//visys_addr = platform_get_resource(pdev, IORESOURCE_MEM, 1);

	visys_addr = devm_platform_ioremap_resource(pdev, 1);
    dw_dphy_rx_probe(pdev, visys_addr);

	drvdata->reset = visys_addr;
    dw_csi_probe(pdev);

	device_register_index++;
	bm_info("exit %s:[%s%d]\n", __func__, BM_DRIVER_NAME, pdev->id);

	return 0;
}

static int bm_csi_remove(struct platform_device *pdev)
{
	struct bm_csi_drvdata *drvdata;

	bm_info("enter %s\n", __func__);
    dw_dphy_rx_remove(pdev);
    dw_csi_remove(pdev);

	device_register_index--;
	drvdata = platform_get_drvdata(pdev);
	free_irq(drvdata->irq_num, drvdata);
	cdev_del(&drvdata->cdev);
	device_destroy(drvdata->class, drvdata->devt);
	unregister_chrdev_region(drvdata->devt, BM_DRIVER_MAXCNT);
	mutex_destroy(&drvdata->mutex);
	if (device_register_index == 0) {
		class_destroy(drvdata->class);
	}
	devm_kfree(&pdev->dev, drvdata);

	bm_info("exit %s\n", __func__);
	return 0;
}

static const struct of_device_id bm_csi_of_match[] = {
	{ .compatible = "thead,light-bm-csi", },
	{ /* sentinel */ },
};

static struct platform_driver bm_csi_driver = {
	.probe		= bm_csi_probe,
	.remove		= bm_csi_remove,
	.driver = {
		.name  = BM_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(bm_csi_of_match),
	}
};

static int __init bm_csi_init_module(void)
{
	int ret = 0;

	bm_info("enter %s\n", __func__);
	ret = platform_driver_register(&bm_csi_driver);
	if (ret) {
		bm_err("register platform driver failed.\n");
		return ret;
	}

	return ret;
}

static void __exit bm_csi_exit_module(void)
{
	bm_info("enter %s\n", __func__);
	platform_driver_unregister(&bm_csi_driver);
}

module_init(bm_csi_init_module);
module_exit(bm_csi_exit_module);

MODULE_AUTHOR("Liu Yitong");
MODULE_DESCRIPTION("BAREMETAL-CSI");
MODULE_LICENSE("GPL");
