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
#include <linux/regulator/consumer.h>

#include "vvsensor.h"
#include "sensor_common.h"

#define VIVCAM_SENSOR_NAME "vivcam"
#define VIVCAM_SENSOR_MAXCNT 30

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
	//pr_info("%s:pdriver_dev =0x%px\n", __func__,pdriver_dev);
	//pr_info("%s:sensor[%d] psensor_dev =0x%px\n", __func__,psensor_dev->device_idx,psensor_dev);

	mutex_lock(&pdriver_dev->vvmutex);
	ret = sensor_priv_ioctl(psensor_dev, cmd ,(void __user *)arg);
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

static int vvcam_sensor_of_parse(struct platform_device *pdev)
{
	int ret = 0, regulator_num = 0;
	struct device_node *np = pdev->dev.of_node;
	struct vvcam_sensor_driver_dev *pdriver_dev = platform_get_drvdata(pdev);
	struct vvcam_sensor_dev * psensor_dev = pdriver_dev->private;

	ret = of_property_read_string(np, "sensor_name", &psensor_dev->sensor_name);
	if (ret < 0) {
		pr_err("%s:property sensor_name not defined for %s\n", __func__, pdev->name);
		return ret;
	}
	psensor_dev->regulators.num = of_property_count_strings(np, "sensor_regulators");
	if (psensor_dev->regulators.num <= 0) {
		pr_err("%s:property sensor_regulators not defined for %s\n", __func__, pdev->name);
	} else {
		pr_info("%s num_of_regulators %d\n", __func__, psensor_dev->regulators.num);
		ret = of_property_read_string_array(np, "sensor_regulators",
				psensor_dev->regulators.name, psensor_dev->regulators.num);
		if (ret != psensor_dev->regulators.num) {
			pr_err("%s:fail to read property sensor_regulators\n", __func__);
			return -1;
		};
		ret = of_property_read_u32_array(np, "sensor_regulator_timing_us",
				psensor_dev->regulators.delay_us, psensor_dev->regulators.num);
		if (ret != 0) {
			pr_err("%s:fail to read property sensor_regulator_timing_us\n", __func__);
			return ret;
		}
		regulator_num = psensor_dev->regulators.num;
		while (regulator_num) {
			pr_info("%s regulator %s delay %d\n", psensor_dev->sensor_name, psensor_dev->regulators.name[regulator_num - 1],
				psensor_dev->regulators.delay_us[regulator_num - 1]);
			psensor_dev->regulators.supply[regulator_num - 1] = devm_regulator_get(&pdev->dev,
						psensor_dev->regulators.name[regulator_num - 1]);
			if (IS_ERR(psensor_dev->regulators.supply[regulator_num - 1])) {
				pr_err("%s:fail to devm_regulator_get %s\n", __func__, psensor_dev->regulators.name[regulator_num - 1]);
				return -1;
			}
			regulator_num--;
		}
	}

	psensor_dev->pdn_pin = of_get_named_gpio(np, "sensor_pdn", 0);
	if (psensor_dev->pdn_pin >= 0) {
		ret = devm_gpio_request(&pdev->dev, psensor_dev->pdn_pin,
					"sensor_pdn");
		if (ret < 0) {
			pr_err("%s:sensor_pdn request failed\n", __func__);
		}
	} else {
		pr_err("sensor_pdn not defined for %s\n", psensor_dev->sensor_name);
	}

	ret = of_property_read_u32(np, "sensor_pdn_delay_us",
				&psensor_dev->pdn_delay_us);
	if (ret != 0) {
		pr_err("fail to read property sensor_pdn_delay_us, assume zero\n");
	}

	psensor_dev->rst_pin = of_get_named_gpio(np, "sensor_rst", 0);
	if (psensor_dev->rst_pin >= 0) {
		ret = devm_gpio_request(&pdev->dev, psensor_dev->rst_pin,
					"sensor_rst");
		if (ret < 0) {
			pr_err("%s:sensor_rst request failed\n", __func__);
		}
	} else {
		pr_err("sensor_rst not defined for %s\n", psensor_dev->sensor_name);
	}

	ret = of_property_read_u8(np, "i2c_addr",
				&psensor_dev->sensor_sccb_cfg.slave_addr);
	if (ret != 0) {
		pr_err("fail to read property i2c_addr, refer to %s.c\n", psensor_dev->sensor_name);
	}
	ret = of_property_read_u8(np, "i2c_reg_width",
				&psensor_dev->sensor_sccb_cfg.addr_byte);
	if (ret != 0) {
		pr_err("fail to read property i2c_reg_width, refer to %s.c\n", psensor_dev->sensor_name);
	}
	ret = of_property_read_u8(np, "i2c_data_width",
				&psensor_dev->sensor_sccb_cfg.data_byte);
	if (ret != 0) {
		pr_err("fail to read property i2c_data_width, refer to %s.c\n", psensor_dev->sensor_name);
	}
	ret = of_property_read_u8(np, "i2c_bus",
				&psensor_dev->i2c_bus);
	if (ret != 0) {
		pr_err("fail to read property i2c_bus, refer to %s.c\n", psensor_dev->sensor_name);
		psensor_dev->i2c_bus = UNDEFINED_IN_DTS;
        return -1;
	}

	return 0;
}

static int vvcam_sensor_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct vvcam_sensor_driver_dev *pdriver_dev;
	struct vvcam_sensor_dev * psensor_dev;
	struct device_node *np = pdev->dev.of_node;

	pr_info("enter %s\n", __func__);

	pdev->id = of_alias_get_id(np, "vivcam");

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
	pr_info("%s:sensor[%d]: pdriver_dev =0x%px\n", __func__,pdev->id,pdriver_dev);

	psensor_dev = devm_kzalloc(&pdev->dev,sizeof(struct vvcam_sensor_dev), GFP_KERNEL);
	if (psensor_dev == NULL)
	{
		pr_err("%s:alloc struct vvcam_sensor_dev error\n", __func__);
		return  -ENOMEM;
	}
	memset(psensor_dev,0,sizeof(struct vvcam_sensor_dev ));
	pr_info("%s:sensor[%d]: psensor_dev =0x%px\n", __func__,pdev->id,psensor_dev);
	psensor_dev->device_idx = pdev->id;

	pdriver_dev->private = psensor_dev;
	mutex_init(&pdriver_dev->vvmutex);
	platform_set_drvdata(pdev, pdriver_dev);

	ret = vvcam_sensor_of_parse(pdev);
	if (ret < 0)
	{
		pr_err("%s:vvcam_sensor_of_parse error\n", __func__);
		return  ret;
	}

	ret = vvnative_sensor_init(psensor_dev);
	if (ret != 0)
	{
		pr_err("%s:vvnative_sensor_init error\n", __func__);
		return  -EIO;
	}

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

    extern int sensor_create_capabilities_sysfs(struct platform_device *pdev);
    sensor_create_capabilities_sysfs(pdev);
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

    extern int sensor_remove_capabilities_sysfs(struct platform_device *pdev);
    sensor_remove_capabilities_sysfs(pdev);

	cdev_del(&pdriver_dev->cdev);
	device_destroy(pdriver_dev->class, pdriver_dev->devt);
	unregister_chrdev_region(pdriver_dev->devt, VIVCAM_SENSOR_MAXCNT);
	if (devise_register_index == 0)
	{
		class_destroy(pdriver_dev->class);
	}

    devm_kfree(&pdev->dev, pdriver_dev);
    devm_kfree(&pdev->dev, psensor_dev);


	return 0;
}

static const struct of_device_id vvcam_sensor_of_match[] = {
	{.compatible = "thead,light-vvcam-sensor"},
};

static struct platform_driver vvcam_sensor_driver = {
	.probe		= vvcam_sensor_probe,
	.remove		= vvcam_sensor_remove,
	.driver = {
		.name  = VIVCAM_SENSOR_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(vvcam_sensor_of_match),
	}
};

static void vvcam_sensor_pdev_release(struct device *dev)
{
	pr_info("enter %s\n", __func__);
}

#if 0//def WITH_VVCAM
static struct platform_device vvcam_sensor_pdev = {
	.name = VIVCAM_SENSOR_NAME,
	.id   = 0,
	.dev.release = vvcam_sensor_pdev_release,
};
#endif

#if 0//def WITH_VVCAM_DUAL
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

#if 0//def WITH_VVCAM
	ret = platform_device_register(&vvcam_sensor_pdev);
	if (ret)
    {
		pr_err("register platform device failed.\n");
		return ret;
	}
#endif

#if 0//def WITH_VVCAM_DUAL
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

#if 0//def WITH_VVCAM
	platform_device_unregister(&vvcam_sensor_pdev);
#endif

#if 0//def WITH_VVCAM_DUAL
	platform_device_unregister(&vvcam_sensor_dual_pdev);
#endif

}

module_init(vvcam_sensor_init_module);
module_exit(vvcam_sensor_exit_module);

MODULE_DESCRIPTION("SENSOR");
MODULE_LICENSE("GPL");
