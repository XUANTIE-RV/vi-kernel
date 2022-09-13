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

#include "vvsensor.h"
#include "sensor_common.h"

#define VIVCAM_SENSOR_NAME "vivcam"
#define VIVCAM_SENSOR_MAXCNT 2


struct vvcam_sensor_driver_dev
{
	struct cdev cdev;
	dev_t devt;
	struct class *class;
	struct mutex vvmutex;
	void *private;
};

static unsigned int vvcam_sensor_major = 0;
static unsigned int vvcam_sensor_minor = 0;
struct class *vvcam_sensor_class;
static unsigned int devise_register_index = 0;


static int vvcam_sensor_open(struct inode * inode, struct file * file)
{
	struct vvcam_sensor_driver_dev *pdriver_dev;

	pdriver_dev = container_of(inode->i_cdev, struct vvcam_sensor_driver_dev, cdev);
	file->private_data = pdriver_dev;

	return 0;
};

static long vvcam_sensor_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret = 0;
	struct vvcam_sensor_driver_dev *pdriver_dev;
	struct vvcam_sensor_dev * psensor_dev;

	pdriver_dev = file->private_data;
	if (pdriver_dev == NULL)
	{
		pr_err("%s:file private is null point error\n", __func__);
		return  -ENOMEM;
	}
	psensor_dev = pdriver_dev->private;
	pr_info("%s:pdriver_dev =0x%p\n", __func__,pdriver_dev);
	pr_info("%s:sensor[%d] psensor_dev =0x%p\n", __func__,psensor_dev->device_idx,psensor_dev);

	mutex_lock(&pdriver_dev->vvmutex);
	ret = sensor_priv_ioctl(psensor_dev, cmd ,(void *)arg);
	mutex_unlock(&pdriver_dev->vvmutex);

	return ret;
};

static int vvcam_sensor_release(struct inode * inode, struct file * file)
{
	return 0;
};

struct file_operations vvcam_sensor_fops = {
	.owner = THIS_MODULE,
	.open = vvcam_sensor_open,
	.release = vvcam_sensor_release,
	.unlocked_ioctl = vvcam_sensor_ioctl,
};

static int vvcam_sensor_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct vvcam_sensor_driver_dev *pdriver_dev;
	struct vvcam_sensor_dev * psensor_dev;

	pr_info("enter %s\n", __func__);

	if (pdev->id >= VIVCAM_SENSOR_MAXCNT)
	{
		pr_err("%s:pdev id is %d error\n", __func__,pdev->id);
		return  -EINVAL;
	}

	pdriver_dev = devm_kzalloc(&pdev->dev,sizeof(struct vvcam_sensor_driver_dev), GFP_KERNEL);
	if (pdriver_dev == NULL)
	{
		pr_err("%s:alloc struct vvcam_sensor_driver_dev error\n", __func__);
		return  -ENOMEM;
	}
	memset(pdriver_dev,0,sizeof(struct vvcam_sensor_driver_dev ));
	pr_info("%s:sensor[%d]: pdriver_dev =0x%p\n", __func__,pdev->id,pdriver_dev);

	psensor_dev = devm_kzalloc(&pdev->dev,sizeof(struct vvcam_sensor_dev), GFP_KERNEL);
	if (psensor_dev == NULL)
	{
		pr_err("%s:alloc struct vvcam_sensor_dev error\n", __func__);
		return  -ENOMEM;
	}
	memset(psensor_dev,0,sizeof(struct vvcam_sensor_dev ));
	pr_info("%s:sensor[%d]: psensor_dev =0x%p\n", __func__,pdev->id,psensor_dev);
	psensor_dev->device_idx = pdev->id;

	ret = vvnative_sensor_init(psensor_dev);
	if (ret != 0)
	{
		pr_err("%s:vvnative_sensor_init error\n", __func__);
		return  -EIO;
	}

	pdriver_dev->private = psensor_dev;
	mutex_init(&pdriver_dev->vvmutex);
	platform_set_drvdata(pdev, pdriver_dev);

	if (devise_register_index == 0)
	{
		if (vvcam_sensor_major == 0)
		{
			ret = alloc_chrdev_region(&pdriver_dev->devt, 0, VIVCAM_SENSOR_MAXCNT, VIVCAM_SENSOR_NAME);
			if (ret != 0)
			{
				pr_err("%s:alloc_chrdev_region error\n", __func__);
				return ret;
			}
			vvcam_sensor_major = MAJOR(pdriver_dev->devt);
			vvcam_sensor_minor = MINOR(pdriver_dev->devt);
		}
		else
		{
			pdriver_dev->devt = MKDEV(vvcam_sensor_major, vvcam_sensor_minor);
			ret = register_chrdev_region(pdriver_dev->devt, VIVCAM_SENSOR_MAXCNT, VIVCAM_SENSOR_NAME);
			if (ret)
			{
				pr_err("%s:register_chrdev_region error\n", __func__);
				return ret;
			}
		}

		vvcam_sensor_class = class_create(THIS_MODULE, VIVCAM_SENSOR_NAME);
		if (IS_ERR(vvcam_sensor_class))
		{
			pr_err("%s[%d]:class_create error!\n", __func__, __LINE__);
			return -EINVAL;
		}
	}
	pdriver_dev->devt = MKDEV(vvcam_sensor_major, vvcam_sensor_minor + pdev->id);

	cdev_init(&pdriver_dev->cdev, &vvcam_sensor_fops);
	ret = cdev_add(&pdriver_dev->cdev, pdriver_dev->devt, 1);
	if ( ret )
	{
		pr_err("%s[%d]:cdev_add error!\n", __func__, __LINE__);
		return ret;
	}
	pdriver_dev->class = vvcam_sensor_class;
	device_create(pdriver_dev->class, NULL, pdriver_dev->devt,
			pdriver_dev, "%s%d", VIVCAM_SENSOR_NAME, pdev->id);

	devise_register_index++;
	pr_info("exit %s\n", __func__);
	return ret;
}

static int vvcam_sensor_remove(struct platform_device *pdev)
{
	struct vvcam_sensor_driver_dev *pdriver_dev;
	struct vvcam_sensor_dev * psensor_dev;

	pr_info("enter %s\n", __func__);
	devise_register_index--;
	pdriver_dev = platform_get_drvdata(pdev);

	if (pdriver_dev == NULL)
	{
		pr_err("%s:file private is null point error\n", __func__);
		return  -ENOMEM;
	}

	psensor_dev = pdriver_dev->private;
	vvnative_sensor_deinit(psensor_dev);

	cdev_del(&pdriver_dev->cdev);
	device_destroy(pdriver_dev->class, pdriver_dev->devt);
	unregister_chrdev_region(pdriver_dev->devt, VIVCAM_SENSOR_MAXCNT);
	if (devise_register_index == 0)
	{
		class_destroy(pdriver_dev->class);
	}

	return 0;
}

static struct platform_driver vvcam_sensor_driver = {
	.probe		= vvcam_sensor_probe,
	.remove		= vvcam_sensor_remove,
	.driver = {
		.name  = VIVCAM_SENSOR_NAME,
		.owner = THIS_MODULE,
	}
};

static void vvcam_sensor_pdev_release(struct device *dev)
{
	pr_info("enter %s\n", __func__);
}

#ifdef WITH_VVCAM
static struct platform_device vvcam_sensor_pdev = {
	.name = VIVCAM_SENSOR_NAME,
	.id   = 0,
	.dev.release = vvcam_sensor_pdev_release,
};
#endif

#ifdef WITH_VVCAM_DUAL
static struct platform_device vvcam_sensor_dual_pdev = {
	.name = VIVCAM_SENSOR_NAME,
	.id   = 1,
	.dev.release = vvcam_sensor_pdev_release,
};
#endif

static int __init vvcam_sensor_init_module(void)
{
	int ret = 0;

	pr_info("enter %s\n", __func__);

#ifdef WITH_VVCAM
	ret = platform_device_register(&vvcam_sensor_pdev);
	if (ret)
    {
		pr_err("register platform device failed.\n");
		return ret;
	}
#endif

#ifdef WITH_VVCAM_DUAL
	ret = platform_device_register(&vvcam_sensor_dual_pdev);
	if (ret)
    {
		pr_err("register platform device failed.\n");
		return ret;
	}
#endif

	ret = platform_driver_register(&vvcam_sensor_driver);
	if (ret)
    {
		pr_err("register platform driver failed.\n");
		return ret;
	}

	return ret;
}

static void __exit vvcam_sensor_exit_module(void)
{
	pr_info("enter %s\n", __func__);

	platform_driver_unregister(&vvcam_sensor_driver);

#ifdef WITH_VVCAM
	platform_device_unregister(&vvcam_sensor_pdev);
#endif

#ifdef WITH_VVCAM_DUAL
	platform_device_unregister(&vvcam_sensor_dual_pdev);
#endif

}

module_init(vvcam_sensor_init_module);
module_exit(vvcam_sensor_exit_module);

MODULE_DESCRIPTION("SENSOR");
MODULE_LICENSE("GPL");
