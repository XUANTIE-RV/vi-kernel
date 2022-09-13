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
#include <linux/pm_runtime.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/debugfs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/of_address.h>
#include "dw200_ioctl.h"
#include "vivdw200_irq_queue.h"

#define VIVCAM_DWE_NAME "vivdw200"
#define VIVCAM_DWE_MAXCNT 1
//#undef USE_RESERVED_MEM

#ifdef __KERNEL__
#define dw_info(...)
#else
#define dw_info(...)
#endif

struct vvcam_dwe_driver_dev
{
	struct cdev cdev;
	dev_t devt;
	struct class *class;
	struct mutex vvmutex;
	unsigned int irq_num[2];
	struct vvcam_dwe_per_file_dev *current_file;
	unsigned long long int dw200_userid;
	struct platform_device *pdev;
#ifdef USE_RESERVED_MEM
	struct resource mem;
#endif
	struct clk *aclk;
	struct clk *hclk;
	struct clk *vseclk;
	struct clk *dweclk;
};

struct vvcam_dwe_per_file_dev
{
	wait_queue_head_t irq_wait;
	bool  irq_trigger;
	void *private; // point to dw200_subdev{}
	int id;
	struct vvcam_dwe_driver_dev *pdriver_dev; // point to global vvcam_dwe_driver_dev
};

static unsigned int vvcam_dwe_major = 0;
static unsigned int vvcam_dwe_minor = 0;
struct class *vvcam_dwe_class;
static unsigned int devise_register_index = 0;

static unsigned int vvcam_dwe_poll(struct file * filp, poll_table *wait)
{
	unsigned int mask = 0;
	struct vvcam_dwe_per_file_dev *per_file = (struct vvcam_dwe_per_file_dev *)filp->private_data;
	poll_wait(filp, &per_file->irq_wait, wait);

	dw_info("poll dwe_irq %d for irq_wait %p id=%d\n", per_file->irq_trigger, &per_file->irq_wait, per_file->id);

	if (per_file->irq_trigger) {
		mask |= POLLIN |POLLRDNORM;
		dw_info("poll notify user space\n");
		per_file->irq_trigger = false;
	}

	return mask;
}

irqreturn_t vivdw200_interrupt(int irq, void* dev_id)
{
	dw_info(" %s enter\n", __func__);
	vivdw200_mis_t node;
	unsigned int dwe_mis, vse_mis;
	struct vvcam_dwe_driver_dev *pdriver_dev = dev_id;
	struct vvcam_dwe_per_file_dev *current_file = pdriver_dev->current_file;
	struct dw200_subdev *pdw200 = (struct dw200_subdev *)current_file->private;
	dw_info("%s current per_file=%p,pdw200=%p id=%d\n", __func__, current_file, pdw200, current_file->id);
	dwe_mis = 0;
	vse_mis = 0;

	dwe_read_irq((struct dw200_subdev *)pdw200, &dwe_mis);
	dwe_mis = dwe_mis & (~0xff00);

	if (0 != dwe_mis) {
		dw_info(" %s dwe mis 0x%08x\n", __func__, dwe_mis);
		dwe_clear_irq((struct dw200_subdev *)pdw200,  dwe_mis <<24);

		node.val = dwe_mis;
		vivdw200_write_circle_queue(&node, &pdw200->dwe_circle_list);
		current_file->irq_trigger |= true;

	}

	vse_read_irq((struct dw200_subdev *)pdw200, &vse_mis);

	if (0 != vse_mis) {
		dw_info(" %s vse mis 0x%08x\n", __func__, vse_mis);
		vse_clear_irq((struct dw200_subdev *)pdw200, vse_mis);
		node.val = vse_mis;
		vivdw200_write_circle_queue(&node, &pdw200->vse_circle_list);
		current_file->irq_trigger |= true;
	}
	if (dwe_mis || vse_mis){
		wake_up_interruptible(&current_file->irq_wait);
	} else {
		return IRQ_HANDLED; // return IRQ_NONE;
	}
	dw_info(" %s exit\n", __func__);
	return IRQ_HANDLED;
}

static int vvcam_dwe_open(struct inode * inode, struct file * file)
{
	struct vvcam_dwe_driver_dev *pdriver_dev = container_of(inode->i_cdev, struct vvcam_dwe_driver_dev, cdev);
	struct dw200_subdev * pdw200 = kzalloc(sizeof(struct dw200_subdev), GFP_KERNEL);
	struct vvcam_dwe_per_file_dev *per_file = kzalloc(sizeof(struct vvcam_dwe_per_file_dev), GFP_KERNEL);
	per_file->id = pdriver_dev->dw200_userid++;
	dw_info("%s per_file=%p,pdw200=%p id=%d\n", __func__, per_file, pdw200, per_file->id);
	pdw200->dwe_base  = ioremap(DWE_REG_BASE, DWE_REG_SIZE);
	pdw200->vse_base  = ioremap(VSE_REG_BASE, VSE_REG_SIZE);
#ifdef DWE_REG_RESET
	pdwe_dev->dwe_reset = ioremap(DWE_REG_RESET, 4);
#endif
#ifdef VSE_REG_RESET
	pdwe_dev->vse_reset = ioremap(VSE_REG_RESET, 4);
#endif
	file->private_data = per_file;
	per_file->private = pdw200;
	per_file->pdriver_dev = pdriver_dev;
	/*create circle queue*/
	vivdw200_create_circle_queue(&(pdw200->dwe_circle_list), QUEUE_NODE_COUNT);
	vivdw200_create_circle_queue(&(pdw200->vse_circle_list), QUEUE_NODE_COUNT);
	init_waitqueue_head(&per_file->irq_wait);
	pdw200->vvmutex = &pdriver_dev->vvmutex;
	return 0;
};

static long vvcam_dwe_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret = 0;
	struct dw200_subdev* pdwe_dev;
	struct vvcam_dwe_per_file_dev *per_file = (struct vvcam_dwe_per_file_dev *)file->private_data;
	struct vvcam_dwe_driver_dev *pdriver_dev = per_file->pdriver_dev;
	struct device *dev = &pdriver_dev->pdev->dev;
	
	if (per_file == NULL)
	{
		pr_err("%s:file private is null point error\n", __func__);
		return  -ENOMEM;
	}
	pdwe_dev = per_file->private;
	dw_info("%s cmd=0x%x id=%d", __func__, cmd, per_file->id);
	switch (cmd) {
		case DW200IOC_UPDATECURR:
			pdriver_dev->current_file = per_file;
			dw_info("DW200IOC_UPDATECURR\n");
			break;
#ifdef USE_RESERVED_MEM
		case DW200IOC_GET_RESERVE_ADDR:
			copy_to_user(arg, &pdriver_dev->mem.start, sizeof(uint32_t));
			dw_info("DW200IOC_GET_RESERVE_ADDR copied pdriver_dev->mem.start 0x%x\n", pdriver_dev->mem.start);
			break;
#endif
		case DW200IOC_RUNTIME_RESUME:
			pm_runtime_resume_and_get(dev);
			break;
		case DW200IOC_RUNTIME_SUSPEND:
			pm_runtime_put_sync(dev);
			break;
	}
	ret =  dw200_priv_ioctl(pdwe_dev, cmd ,(void *)arg);
	return ret;
};

static int vvcam_dwe_release(struct inode * inode, struct file * file)
{
	struct vvcam_dwe_per_file_dev *per_file = (struct vvcam_dwe_per_file_dev *)file->private_data;
	struct dw200_subdev * pdw200;
	pdw200 = (struct dw200_subdev *)per_file->private;
	mutex_unlock(pdw200->vvmutex);
	/*destory circle queue*/
	vivdw200_destroy_circle_queue(&(pdw200->dwe_circle_list));
	vivdw200_destroy_circle_queue(&(pdw200->vse_circle_list));
	kfree(per_file);
	return 0;
};

static int vvcam_dwe_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct vvcam_dwe_per_file_dev *per_file = (struct vvcam_dwe_per_file_dev *)file->private_data;
	struct vvcam_dwe_driver_dev *pdriver_dev = per_file->pdriver_dev;
	int ret = 0;
#ifdef USE_RESERVED_MEM
	unsigned long pfn_start = phys_to_pfn(pdriver_dev->mem.start) + vma->vm_pgoff;
	unsigned long size = vma->vm_end - vma->vm_start;
	dw_info("phy: 0x%lx, size: 0x%lx PAGE_SHIFT: %d vma->vm_pgoff: 0x%lx vma->vm_start: 0x%lx\n", pfn_to_phys(pfn_start), size, PAGE_SHIFT, vma->vm_pgoff, vma->vm_start);
	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
	if (remap_pfn_range(vma, vma->vm_start, pfn_start, size, vma->vm_page_prot))
	{
		pr_err("-->%s: remap_pfn_range error!\n", __func__);
		return -EIO;
	}
#else
	ret = -EIO;
#endif
	return ret;
};

struct file_operations vvcam_dwe_fops = {
	.owner = THIS_MODULE,
	.open = vvcam_dwe_open,
	.release = vvcam_dwe_release,
	.unlocked_ioctl = vvcam_dwe_ioctl,
	.poll = vvcam_dwe_poll,
	.mmap = vvcam_dwe_mmap,
};

static int vvcam_dwe_runtime_suspend(struct device *dev)
{
	struct vvcam_dwe_driver_dev *pdriver_dev = dev_get_drvdata(dev);

	if (IS_ERR(pdriver_dev->dweclk) || IS_ERR(pdriver_dev->vseclk) 
		|| IS_ERR(pdriver_dev->hclk) || IS_ERR(pdriver_dev->aclk)) {
			return 0;
	}
	
	dw_info("%s\n", __func__);

	clk_disable_unprepare(pdriver_dev->aclk);
	clk_disable_unprepare(pdriver_dev->hclk);
	clk_disable_unprepare(pdriver_dev->vseclk);
	clk_disable_unprepare(pdriver_dev->dweclk);

	return 0;
}

static int vvcam_dwe_runtime_resume(struct device *dev)
{
	struct vvcam_dwe_driver_dev *pdriver_dev = dev_get_drvdata(dev);
	int ret = 0;

	if (IS_ERR(pdriver_dev->dweclk) || IS_ERR(pdriver_dev->vseclk) 
		|| IS_ERR(pdriver_dev->hclk) || IS_ERR(pdriver_dev->aclk)) {
			return 0;
	}

	ret = clk_prepare_enable(pdriver_dev->dweclk);
	if (ret < 0) {
		dev_err(dev, "could not prepare or enable dwe clock\n");
	}

	ret = clk_prepare_enable(pdriver_dev->vseclk);
	if (ret < 0) {
		dev_err(dev, "could not prepare or enable vse clock\n");
		clk_disable_unprepare(pdriver_dev->dweclk);
		//return ret;
	}

	ret = clk_prepare_enable(pdriver_dev->hclk);
	if (ret < 0) {
		dev_err(dev, "could not prepare or enable ahb clock\n");
		clk_disable_unprepare(pdriver_dev->dweclk);
		clk_disable_unprepare(pdriver_dev->vseclk);
		//return ret;
	}	

	ret = clk_prepare_enable(pdriver_dev->aclk);
	if (ret < 0) {
		dev_err(dev, "could not prepare or enable axi clock\n");
		clk_disable_unprepare(pdriver_dev->dweclk);
		clk_disable_unprepare(pdriver_dev->vseclk);
		clk_disable_unprepare(pdriver_dev->hclk);
		//return ret;
	}	

	dw_info("%s Enabled clock\n", __func__);
	return ret;
}

static const struct dev_pm_ops vvcam_dwe_runtime_pm_ops = {
	SET_RUNTIME_PM_OPS(vvcam_dwe_runtime_suspend, vvcam_dwe_runtime_resume, NULL)
};

void __iomem *visys_sw_rst;
#ifdef USE_RESERVED_MEM
static int vvcam_dwe_of_parse(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np;
	int ret = 0;
	struct vvcam_dwe_driver_dev *pdriver_dev = platform_get_drvdata(pdev);

	np = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (!np) {
		dev_err(dev, "No %s specified\n", "memory-region");
		return -EINVAL;
	}

	ret = of_address_to_resource(np, 0, &pdriver_dev->mem);
	if (ret) {
		dev_err(dev, "of_address_to_resource fail\n");
		return ret;
	}
	dw_info("%s got mem start 0x%x size 0x%x\n", __func__, pdriver_dev->mem.start, resource_size(&pdriver_dev->mem));

	return ret;
}
#endif
static int vvcam_dwe_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct vvcam_dwe_driver_dev *pdriver_dev;

	dw_info("enter %s\n", __func__);

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

	visys_sw_rst = ioremap(0xffe4040100, 0x00001000);
#ifdef DWE_REG_RESET
	pdwe_dev->dwe_reset = ioremap(DWE_REG_RESET, 4);
#endif
#ifdef VSE_REG_RESET
	pdwe_dev->vse_reset = ioremap(VSE_REG_RESET, 4);
#endif
	mutex_init(&pdriver_dev->vvmutex);
	platform_set_drvdata(pdev, pdriver_dev);
	pdriver_dev->pdev = pdev;
	pdriver_dev->irq_num[0] = platform_get_irq(pdev, 0);
	if (pdriver_dev->irq_num[0] == 0) {
		pr_err("%s:dw200_[%d]: could not map IRQ\n", __func__, pdev->id);
		dev_err(&pdev->dev, "could not map IRQ.\n");
		return -ENXIO;
	}
	dw_info("%s:dw200_[%d]: pdriver_dev->irq_num[0]=%d\n", __func__, pdev->id, pdriver_dev->irq_num[0]);

	pdriver_dev->irq_num[1] = platform_get_irq(pdev, 1);
	if (pdriver_dev->irq_num[1] == 0) {
		pr_err("%s:dw200_[%d]: could not map IRQ\n", __func__, pdev->id);
		dev_err(&pdev->dev, "could not map IRQ.\n");
		return -ENXIO;
	}
	dw_info("%s:dw200_[%d]: pdriver_dev->irq_num[1]=%d\n", __func__, pdev->id, pdriver_dev->irq_num[1]);

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
#ifdef USE_RESERVED_MEM
	ret = vvcam_dwe_of_parse(pdev);
	if (ret) {
		pr_err("%s[%d]: vvcam_dwe_of_parse error!\n", __func__, __LINE__);
		return ret;
	}
#else
	pr_info("%s:disable vvcam_dwe_of_parse reserved-memory\n", __func__);
#endif
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
	pdriver_dev->devt = MKDEV(vvcam_dwe_major, vvcam_dwe_minor + pdev->id + 1); // just in case platform instance num is -1

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

	pdriver_dev->dweclk = devm_clk_get(&pdev->dev, "dweclk");
	if (IS_ERR(pdriver_dev->dweclk)) {
		dev_err(&pdev->dev, "failed to get dwe clk");
		//return -1;
	}

	pdriver_dev->vseclk = devm_clk_get(&pdev->dev, "vseclk");
	if (IS_ERR(pdriver_dev->vseclk)) {
		dev_err(&pdev->dev, "failed to get vse clk");
		//return -1;
	}

	pdriver_dev->hclk = devm_clk_get(&pdev->dev, "hclk");
	if (IS_ERR(pdriver_dev->hclk)) {
		dev_err(&pdev->dev, "failed to get hclk");
		//return -1;
	}

	pdriver_dev->aclk = devm_clk_get(&pdev->dev, "aclk");
	if (IS_ERR(pdriver_dev->aclk)) {
		dev_err(&pdev->dev, "failed to get aclk");
		//return -1;
	}

	pm_runtime_enable(&pdev->dev);
	pm_runtime_resume_and_get(&pdev->dev);
	pm_runtime_put_sync(&pdev->dev);

	dw_info("exit %s:[%d]\n", __func__, pdev->id);
	return ret;
}

static int vvcam_dwe_remove(struct platform_device *pdev)
{
	struct vvcam_dwe_driver_dev *pdriver_dev;
	//struct dw200_subdev * pdwe_dev;

	dw_info("enter %s\n", __func__);
	devise_register_index--;
	pdriver_dev = platform_get_drvdata(pdev);

	free_irq(pdriver_dev->irq_num[0], pdriver_dev);
	free_irq(pdriver_dev->irq_num[1], pdriver_dev);

	//pdwe_dev = pdriver_dev->private;
	//iounmap(pdwe_dev->dwe_base);
	//iounmap(pdwe_dev->vse_base);
	//iounmap(pdwe_dev->dwe_reset);
	//iounmap(pdwe_dev->vse_reset);
	pm_runtime_disable(&pdev->dev);
	if (!pm_runtime_status_suspended(&pdev->dev)) { // turn off the power regardless of the ref cnt
		vvcam_dwe_runtime_suspend(&pdev->dev);
	}
	
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
		.pm = &vvcam_dwe_runtime_pm_ops,
	}
};

static int __init vvcam_dwe_init_module(void)
{
	int ret = 0;

	dw_info("enter %s\n", __func__);

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
	dw_info("enter %s\n", __func__);

	platform_driver_unregister(&vvcam_dwe_driver);
}

module_init(vvcam_dwe_init_module);
module_exit(vvcam_dwe_exit_module);

MODULE_DESCRIPTION("DWE");
MODULE_LICENSE("GPL");
