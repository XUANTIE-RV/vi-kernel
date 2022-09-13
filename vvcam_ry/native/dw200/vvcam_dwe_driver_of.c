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
#include <linux/interrupt.h>
#include "dw200_ioctl.h"
#include "vivdw200_irq_queue.h"

#define VIVCAM_DWE_NAME "vivdw200"
#define VIVCAM_DWE_MAXCNT 1


struct vvcam_dwe_driver_dev
{
	struct cdev cdev;
	dev_t devt;
	struct class *class;
	struct mutex vvmutex;
	void *private;
	unsigned int irq_num[2];
	bool  irq_trigger;
	wait_queue_head_t irq_wait;
};

static unsigned int vvcam_dwe_major = 0;
static unsigned int vvcam_dwe_minor = 0;
struct class *vvcam_dwe_class;
static unsigned int devise_register_index = 0;

static unsigned int vvcam_dwe_poll(struct file * filp, poll_table *wait)
{
	unsigned int mask = 0;
	struct vvcam_dwe_driver_dev *pdriver_dev = filp->private_data;
	poll_wait(filp, &pdriver_dev->irq_wait, wait);

	pr_info("poll dwe_irq %d\n", pdriver_dev->irq_trigger);

	if (pdriver_dev->irq_trigger) {
		mask |= POLLIN |POLLRDNORM;
		pr_info("poll notify user space\n");
		pdriver_dev->irq_trigger = false;
	}

	return mask;
}

irqreturn_t vivdw200_interrupt(int irq, void* dev_id)
{
	pr_info(" %s enter\n", __func__);
	vivdw200_mis_t node;
	unsigned int dwe_mis, vse_mis;
	struct vvcam_dwe_driver_dev *pdriver_dev = dev_id;
	struct dw200_subdev *pdw200 = (struct dw200_subdev *)pdriver_dev->private;
	dwe_mis = 0;
	vse_mis = 0;

	dwe_read_irq((struct dw200_subdev *)pdw200, &dwe_mis);
	dwe_mis = dwe_mis & (~0xff00);

	if (0 != dwe_mis) {
		pr_info(" %s dwe mis 0x%08x\n", __func__, dwe_mis);
		dwe_clear_irq((struct dw200_subdev *)pdw200,  dwe_mis <<24);

		node.val = dwe_mis;
		vivdw200_write_circle_queue(&node, &pdw200->dwe_circle_list);
		pdriver_dev->irq_trigger |= true;

	}

	vse_read_irq((struct dw200_subdev *)pdw200, &vse_mis);

	if (0 != vse_mis) {
		pr_info(" %s vse mis 0x%08x\n", __func__, vse_mis);
		vse_clear_irq((struct dw200_subdev *)pdw200, vse_mis);
		node.val = vse_mis;
		vivdw200_write_circle_queue(&node, &pdw200->vse_circle_list);
		pdriver_dev->irq_trigger |= true;
	}
	if (dwe_mis || vse_mis){
		wake_up_interruptible(&pdriver_dev->irq_wait);
	} else {
		return IRQ_NONE;
	}
	pr_info(" %s exit\n", __func__);
	return IRQ_HANDLED;
}

static int vvcam_dwe_open(struct inode * inode, struct file * file)
{
	struct vvcam_dwe_driver_dev *pdriver_dev;
	struct dw200_subdev * pdw200;
	pdriver_dev = container_of(inode->i_cdev, struct vvcam_dwe_driver_dev, cdev);
	file->private_data = pdriver_dev;

	pdw200 = (struct dw200_subdev *)pdriver_dev->private;
	/*create circle queue*/
	vivdw200_create_circle_queue(&(pdw200->dwe_circle_list), QUEUE_NODE_COUNT);
	vivdw200_create_circle_queue(&(pdw200->vse_circle_list), QUEUE_NODE_COUNT);

	return 0;
};

static long vvcam_dwe_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret = 0;
	struct vvcam_dwe_driver_dev *pdriver_dev;
	struct dw200_subdev* pdwe_dev;
	pdriver_dev = file->private_data;
	if (pdriver_dev == NULL)
	{
		pr_err("%s:file private is null point error\n", __func__);
		return  -ENOMEM;
	}
	pdwe_dev = pdriver_dev->private;

	ret =  dw200_priv_ioctl(pdwe_dev, cmd ,(void *)arg);
	return ret;
};

static int vvcam_dwe_release(struct inode * inode, struct file * file)
{
	struct vvcam_dwe_driver_dev *pdriver_dev;
	pdriver_dev = file->private_data;
	struct dw200_subdev * pdw200;
	pdw200 = (struct dw200_subdev *)pdriver_dev->private;

	/*destory circle queue*/
	vivdw200_destroy_circle_queue(&(pdw200->dwe_circle_list));
	vivdw200_destroy_circle_queue(&(pdw200->vse_circle_list));

	return 0;
};

struct file_operations vvcam_dwe_fops = {
	.owner = THIS_MODULE,
	.open = vvcam_dwe_open,
	.release = vvcam_dwe_release,
	.unlocked_ioctl = vvcam_dwe_ioctl,
	.poll = vvcam_dwe_poll,
};

static int vvcam_dwe_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct vvcam_dwe_driver_dev *pdriver_dev;
	struct dw200_subdev * pdwe_dev;
//	struct resource		*mem;

	pr_info("enter %s\n", __func__);

	if (pdev->id >= VIVCAM_DWE_MAXCNT)
	{
		pr_err("%s:pdev id is %d error\n", __func__,pdev->id);
		return  -EINVAL;
	}

	pdriver_dev = devm_kzalloc(&pdev->dev,sizeof(struct vvcam_dwe_driver_dev), GFP_KERNEL);
	if (pdriver_dev == NULL)
	{
		pr_err("%s:alloc struct vvcam_soc_driver_dev error\n", __func__);
		return  -ENOMEM;
	}
	memset(pdriver_dev,0,sizeof(struct vvcam_dwe_driver_dev ));

	pdwe_dev = devm_kzalloc(&pdev->dev,sizeof(struct dw200_subdev), GFP_KERNEL);
	if (pdwe_dev == NULL)
	{
		pr_err("%s:alloc struct vvcam_soc_dev error\n", __func__);
		return  -ENOMEM;
	}
	memset(pdwe_dev,0,sizeof(struct dw200_subdev ));

	pdwe_dev->dwe_base  = ioremap(DWE_REG_BASE, DWE_REG_SIZE);
	pdwe_dev->vse_base  = ioremap(VSE_REG_BASE, VSE_REG_SIZE);
#ifdef DWE_REG_RESET
	pdwe_dev->dwe_reset = ioremap(DWE_REG_RESET, 4);
#endif
#ifdef VSE_REG_RESET
	pdwe_dev->vse_reset = ioremap(VSE_REG_RESET, 4);
#endif
	pdriver_dev->private = pdwe_dev;
	mutex_init(&pdriver_dev->vvmutex);
	platform_set_drvdata(pdev, pdriver_dev);
	pdriver_dev->irq_num[0] = platform_get_irq(pdev, 0);
	if (pdriver_dev->irq_num[0] == 0) {
		pr_err("%s:dw200_[%d]: could not map IRQ\n", __func__, pdev->id);
		dev_err(&pdev->dev, "could not map IRQ.\n");
		return -ENXIO;
	}
	pr_info("%s:dw200_[%d]: pdriver_dev->irq_num[0]=%d\n", __func__, pdev->id, pdriver_dev->irq_num[0]);

	pdriver_dev->irq_num[1] = platform_get_irq(pdev, 1);
	if (pdriver_dev->irq_num[1] == 0) {
		pr_err("%s:dw200_[%d]: could not map IRQ\n", __func__, pdev->id);
		dev_err(&pdev->dev, "could not map IRQ.\n");
		return -ENXIO;
	}
	pr_info("%s:dw200_[%d]: pdriver_dev->irq_num[1]=%d\n", __func__, pdev->id, pdriver_dev->irq_num[1]);

	ret = request_irq(pdriver_dev->irq_num[0], vivdw200_interrupt,
			IRQF_SHARED|IRQF_TRIGGER_RISING, "DEWARP_IRQ", (char *)pdriver_dev);
	if (ret) {
		pr_err("%s[%d]:request irq error!\n", __func__, __LINE__);
		return ret;
	}

	ret = request_irq(pdriver_dev->irq_num[1], vivdw200_interrupt,
			IRQF_SHARED|IRQF_TRIGGER_RISING, "SCALAR_IRQ", (char *)pdriver_dev);
	if (ret) {
		pr_err("%s[%d]:request irq error!\n", __func__, __LINE__);
		return ret;
	}

#if	0
	ret = devm_request_irq(&pdev->dev, 16, (irq_handler_t)vivdw200_interrupt, IRQF_SHARED|IRQF_TRIGGER_RISING, VIVCAM_DWE_NAME, (void*)pdriver_dev);// pdriver_dev->irq_num
	if (ret != 0) {
		pr_err("%s:request irq error\n", __func__);
		return ret;
	}
#endif
	init_waitqueue_head(&pdriver_dev->irq_wait);

	if (devise_register_index == 0)
	{
		if (vvcam_dwe_major == 0)
		{
			ret = alloc_chrdev_region(&pdriver_dev->devt, 0, VIVCAM_DWE_MAXCNT, VIVCAM_DWE_NAME);
			if (ret != 0)
			{
				pr_err("%s:alloc_chrdev_region error\n", __func__);
				return ret;
			}
			vvcam_dwe_major = MAJOR(pdriver_dev->devt);
			vvcam_dwe_minor = MINOR(pdriver_dev->devt);
		}
		else
		{
			pdriver_dev->devt = MKDEV(vvcam_dwe_major, vvcam_dwe_minor);
			ret = register_chrdev_region(pdriver_dev->devt, VIVCAM_DWE_MAXCNT, VIVCAM_DWE_NAME);
			if (ret)
			{
				pr_err("%s:register_chrdev_region error\n", __func__);
				return ret;
			}
		}

		vvcam_dwe_class = class_create(THIS_MODULE, VIVCAM_DWE_NAME);
		if (IS_ERR(vvcam_dwe_class))
		{
			pr_err("%s[%d]:class_create error!\n", __func__, __LINE__);
			return -EINVAL;
		}
	}
	pdriver_dev->devt = MKDEV(vvcam_dwe_major, vvcam_dwe_minor + pdev->id);

	cdev_init(&pdriver_dev->cdev, &vvcam_dwe_fops);
	ret = cdev_add(&pdriver_dev->cdev, pdriver_dev->devt, 1);
	if ( ret )
	{
		pr_err("%s[%d]:cdev_add error!\n", __func__, __LINE__);
		return ret;
	}
	pdriver_dev->class = vvcam_dwe_class;
	device_create(pdriver_dev->class, NULL, pdriver_dev->devt,
			pdriver_dev, "%s", VIVCAM_DWE_NAME);

	devise_register_index++;
	pr_info("exit %s:[%d]\n", __func__, pdev->id);
	return ret;
}

static int vvcam_dwe_remove(struct platform_device *pdev)
{
	struct vvcam_dwe_driver_dev *pdriver_dev;
	struct dw200_subdev * pdwe_dev;

	pr_info("enter %s\n", __func__);
	devise_register_index--;
	pdriver_dev = platform_get_drvdata(pdev);

	free_irq(pdriver_dev->irq_num[0], pdriver_dev);
	free_irq(pdriver_dev->irq_num[1], pdriver_dev);

	pdwe_dev = pdriver_dev->private;
	iounmap(pdwe_dev->dwe_base);
	iounmap(pdwe_dev->vse_base);
	iounmap(pdwe_dev->dwe_reset);
	iounmap(pdwe_dev->vse_reset);

	cdev_del(&pdriver_dev->cdev);
	device_destroy(pdriver_dev->class, pdriver_dev->devt);
	unregister_chrdev_region(pdriver_dev->devt, VIVCAM_DWE_MAXCNT);
	if (devise_register_index == 0)
	{
		class_destroy(pdriver_dev->class);
	}

	return 0;
}

static const struct of_device_id dewarp_of_match[] = {
	{ .compatible = "thead,light-dewarp", },
	{ /* sentinel */ },
};

static struct platform_driver vvcam_dwe_driver = {
	.probe		= vvcam_dwe_probe,
	.remove		= vvcam_dwe_remove,
	.driver = {
		.name  = VIVCAM_DWE_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(dewarp_of_match),
	}
};

static int __init vvcam_dwe_init_module(void)
{
	int ret = 0;

	pr_info("enter %s\n", __func__);

	ret = platform_driver_register(&vvcam_dwe_driver);
	if (ret)
    {
		pr_err("register platform driver failed.\n");
		return ret;
	}

	return ret;
}

static void __exit vvcam_dwe_exit_module(void)
{
	pr_info("enter %s\n", __func__);

	platform_driver_unregister(&vvcam_dwe_driver);
}

module_init(vvcam_dwe_init_module);
module_exit(vvcam_dwe_exit_module);

MODULE_DESCRIPTION("DWE");
MODULE_LICENSE("GPL");
