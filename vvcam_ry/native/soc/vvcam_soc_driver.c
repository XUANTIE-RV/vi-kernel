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

#include <asm/io.h>
#include <linux/cdev.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/debugfs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>

#include "soc_ioctl.h"

#define VIVCAM_SOC_NAME "vivsoc"
#define VIVCAM_SOC_MAXCNT 2


struct vvcam_soc_driver_dev
{
	struct cdev cdev;
	dev_t devt;
	struct class *class;
	struct mutex vvmutex;
	void *private;
};

static unsigned int vvcam_soc_major = 0;
static unsigned int vvcam_soc_minor = 0;
static struct class *vvcam_soc_class;
static unsigned int devise_register_index = 0;


static int vvcam_soc_open(struct inode * inode, struct file * file)
{
	struct vvcam_soc_driver_dev *pdriver_dev;

	pdriver_dev = container_of(inode->i_cdev, struct vvcam_soc_driver_dev, cdev);
	file->private_data = pdriver_dev;

	return 0;
};

static long vvcam_soc_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret = 0;
	struct vvcam_soc_driver_dev *pdriver_dev;
	struct vvcam_soc_dev* psoc_dev;
	pdriver_dev = file->private_data;
	if (pdriver_dev == NULL)
	{
		pr_err("%s:file private is null point error\n", __func__);
		return  -ENOMEM;
	}
	psoc_dev = pdriver_dev->private;
	ret =  soc_priv_ioctl(psoc_dev, cmd , (void __user *)arg);
	return ret;
};

static int vvcam_soc_release(struct inode * inode, struct file * file)
{
	return 0;
};

static struct file_operations vvcam_soc_fops = {
	.owner = THIS_MODULE,
	.open = vvcam_soc_open,
	.release = vvcam_soc_release,
	.unlocked_ioctl = vvcam_soc_ioctl,
};

static int vvcam_soc_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct vvcam_soc_driver_dev *pdriver_dev;
	struct vvcam_soc_dev * psoc_dev;
//	struct resource		*mem;

	pr_info("enter %s\n", __func__);

	if (pdev->id >= VIVCAM_SOC_MAXCNT)
	{
		pr_err("%s:pdev id is %d error\n", __func__,pdev->id);
		return  -EINVAL;
	}

	pdriver_dev = devm_kzalloc(&pdev->dev,sizeof(struct vvcam_soc_driver_dev), GFP_KERNEL);
	if (pdriver_dev == NULL)
	{
		pr_err("%s:alloc struct vvcam_soc_driver_dev error\n", __func__);
		return  -ENOMEM;
	}
	memset(pdriver_dev,0,sizeof(struct vvcam_soc_driver_dev ));


	psoc_dev = devm_kzalloc(&pdev->dev,sizeof(struct vvcam_soc_dev), GFP_KERNEL);
	if (psoc_dev == NULL)
	{
		pr_err("%s:alloc struct vvcam_soc_dev error\n", __func__);
		return  -ENOMEM;
	}
	memset(psoc_dev,0,sizeof(struct vvcam_soc_dev ));

	//mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	//psoc_dev->base = devm_ioremap_resource(&pdev->dev, mem);
	psoc_dev->base = ioremap(VVCTRL_BASE, VVCTRL_SIZE);
	if (IS_ERR(psoc_dev->base ))
		return PTR_ERR(psoc_dev->base );

	pdriver_dev->private = psoc_dev;
	mutex_init(&pdriver_dev->vvmutex);
	platform_set_drvdata(pdev, pdriver_dev);

	vvnative_soc_init(psoc_dev);
	if (devise_register_index == 0)
	{
		if (vvcam_soc_major == 0)
		{
			ret = alloc_chrdev_region(&pdriver_dev->devt, 0, VIVCAM_SOC_MAXCNT, VIVCAM_SOC_NAME);
			if (ret != 0)
			{
				pr_err("%s:alloc_chrdev_region error\n", __func__);
				return ret;
			}
			vvcam_soc_major = MAJOR(pdriver_dev->devt);
			vvcam_soc_minor = MINOR(pdriver_dev->devt);
		}
		else
		{
			pdriver_dev->devt = MKDEV(vvcam_soc_major, vvcam_soc_minor);
			ret = register_chrdev_region(pdriver_dev->devt, VIVCAM_SOC_MAXCNT, VIVCAM_SOC_NAME);
			if (ret)
			{
				pr_err("%s:register_chrdev_region error\n", __func__);
				return ret;
			}
		}

		vvcam_soc_class = class_create(THIS_MODULE, VIVCAM_SOC_NAME);
		if (IS_ERR(vvcam_soc_class))
		{
			pr_err("%s[%d]:class_create error!\n", __func__, __LINE__);
			return -EINVAL;
		}
	}
	pdriver_dev->devt = MKDEV(vvcam_soc_major, vvcam_soc_minor + pdev->id);

	cdev_init(&pdriver_dev->cdev, &vvcam_soc_fops);
	ret = cdev_add(&pdriver_dev->cdev, pdriver_dev->devt, 1);
	if ( ret )
	{
		pr_err("%s[%d]:cdev_add error!\n", __func__, __LINE__);
		return ret;
	}
	pdriver_dev->class = vvcam_soc_class;
	device_create(pdriver_dev->class, NULL, pdriver_dev->devt,
			pdriver_dev, "%s%d", VIVCAM_SOC_NAME, pdev->id);

	devise_register_index++;
	pr_info("exit %s\n", __func__);
	return ret;
}

static int vvcam_soc_remove(struct platform_device *pdev)
{
	struct vvcam_soc_driver_dev *pdriver_dev;
	struct vvcam_soc_dev * psoc_dev;

	pr_info("enter %s\n", __func__);
	devise_register_index--;
	pdriver_dev = platform_get_drvdata(pdev);

	psoc_dev = pdriver_dev->private;
	iounmap(psoc_dev->base);

	cdev_del(&pdriver_dev->cdev);
	device_destroy(pdriver_dev->class, pdriver_dev->devt);
	unregister_chrdev_region(pdriver_dev->devt, VIVCAM_SOC_MAXCNT);
	if (devise_register_index == 0)
	{
		class_destroy(pdriver_dev->class);
	}

	return 0;
}

static struct platform_driver vvcam_soc_driver = {
	.probe		= vvcam_soc_probe,
	.remove		= vvcam_soc_remove,
	.driver = {
		.name  = VIVCAM_SOC_NAME,
		.owner = THIS_MODULE,
	}
};

static void vvcam_soc_pdev_release(struct device *dev)
{
	pr_info("enter %s\n", __func__);
}


static struct resource vvcam_soc_resource[] = {
	[0] = {
		.start = VVCTRL_BASE,
		.end   = VVCTRL_BASE + VVCTRL_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
};
static struct platform_device vvcam_soc_pdev = {
	.name = VIVCAM_SOC_NAME,
	.id   = 0,
	.resource = vvcam_soc_resource,
	.num_resources = 1,
	.dev.release = vvcam_soc_pdev_release,
};

static int __init vvcam_soc_init_module(void)
{
	int ret = 0;

	pr_info("enter %s\n", __func__);

	ret = platform_device_register(&vvcam_soc_pdev);
	if (ret)
    {
		pr_err("register platform device failed.\n");
		return ret;
	}

	ret = platform_driver_register(&vvcam_soc_driver);
	if (ret)
    {
		pr_err("register platform driver failed.\n");
		platform_device_unregister(&vvcam_soc_pdev);
		return ret;
	}

	return ret;
}

static void __exit vvcam_soc_exit_module(void)
{
	pr_info("enter %s\n", __func__);

	platform_driver_unregister(&vvcam_soc_driver);
	platform_device_unregister(&vvcam_soc_pdev);
}

module_init(vvcam_soc_init_module);
module_exit(vvcam_soc_exit_module);

MODULE_DESCRIPTION("ISP");
MODULE_LICENSE("GPL");