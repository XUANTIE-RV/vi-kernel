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

#include <linux/timer.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/debugfs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/of_reserved_mem.h>

#include "isp_ioctl.h"
#include "mrv_all_regs.h"
#include "isp_wdr.h"

extern MrvAllRegister_t *all_regs;

static ulong vvImgBufBase = 0x10000000;
module_param(vvImgBufBase, ulong, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(vvImgBufBase, "Base addrss of memory reserved for ISP");

static ulong vvImgBufSize = 0x10000000;
module_param(vvImgBufSize, ulong, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(vvImgBufSize, "Size of memory reserved for ISP");

#define VIVCAM_ISP_NAME "vivisp"
#define VIVCAM_ISP_MAXCNT 2
#define VIVCAM_ISP_IRQ_NUMBER 16

struct vvcam_isp_driver_dev
{
	struct cdev cdev;
	dev_t devt;
	struct class *class;
	struct mutex vvmutex;
	unsigned int device_idx;
    struct timer_list isp_timer;
    struct work_struct vvnative_wq;
    wait_queue_head_t irq_wait;
    int irq_num;
	void *private;
};

static unsigned int vvcam_isp_major = 0;
static unsigned int vvcam_isp_minor = 0;
static struct class *vvcam_isp_class;
static unsigned int devise_register_index = 0;
static bool isp_irq = false;
static unsigned int vvnative_isp_poll(struct file * filp, poll_table *wait)
{
    unsigned int mask = 0;
    struct vvcam_isp_driver_dev *pdriver_dev = filp->private_data;
    poll_wait(filp, &pdriver_dev->irq_wait, wait);

    //pr_info("poll isp_irq %d\n", isp_irq);

    if (isp_irq) {
        mask |= POLLIN |POLLRDNORM;
        isp_irq = false;
     }
    return mask;
}

static void vvnative_isp_work(struct work_struct *work)
{
   /*Todo update those module that does not have shandow register*/
   struct vvcam_isp_driver_dev *pdriver_dev = container_of(work, struct vvcam_isp_driver_dev, vvnative_wq);

   struct isp_ic_dev * pisp_dev = pdriver_dev->private;
   pr_info("%s enter \n", __func__);

  isp_irq = true;
  wake_up_interruptible(&pdriver_dev->irq_wait);
  if (pisp_dev->isp_mis & MRV_ISP_MIS_FRAME_MASK) {
    if (pisp_dev->wdr.changed) {
      pr_info("%s pisp_dev->wdr.changed %d\n", __func__,
       pisp_dev->wdr.changed);
      isp_s_wdr(pisp_dev);
    }
    if (pisp_dev->flt.changed) {
      pr_info("%s pisp_dev->flt.changed %d\n", __func__,
       pisp_dev->flt.changed);
      isp_s_flt(pisp_dev);
    }
#ifndef ISP_CPROC_SHD_RY
    if (pisp_dev->cproc.changed) {
       pr_info("%s pisp_dev->cproc.changed %d\n", __func__,
       pisp_dev->cproc.changed);
       isp_s_cproc(pisp_dev);
    }
#endif
    if (pisp_dev->gamma_out.changed) {
      pr_info("%s pisp_dev->gamma_out.changed %d\n", __func__,
       pisp_dev->gamma_out.changed);
       isp_s_gamma_out(pisp_dev);
    }

  }

}

static irqreturn_t vvcam_isp_irq(int irq, void *dev_id)
{
	struct vvcam_isp_driver_dev *pdriver_dev ;
	struct isp_ic_dev * pisp_dev;
    u32 isp_mis, mi_mis, mi_mis_addr, mi_icr_addr;
#ifdef ISP_MIV2_RY
	u32 miv2_mis1, miv2_mis3;
#endif
 #if defined(ISP_MI_PP_READ_RY) || defined (ISP_3DNR_V3) || defined (ISP_MI_PP_WRITE_RY) ||  defined (ISP_MI_HDR_RY)
   u32 miv2_mis2;
#endif
	pdriver_dev = (struct vvcam_isp_driver_dev *)dev_id;
	pisp_dev = pdriver_dev->private;
    isp_mis = isp_read_reg(pisp_dev, REG_ADDR(isp_mis));
#ifdef ISP_MIV2_RY
	mi_icr_addr = REG_ADDR(miv2_icr);
	mi_mis_addr = REG_ADDR(miv2_mis);
    miv2_mis1 = isp_read_reg(pisp_dev, REG_ADDR(miv2_mis1));
	if (miv2_mis1) {
		pr_info("%s mi mis1 0x%08x\n", __func__, miv2_mis1);
  		isp_write_reg(pisp_dev, REG_ADDR(miv2_icr1), miv2_mis1);
	}

    miv2_mis3 = isp_read_reg(pisp_dev, REG_ADDR(miv2_mis3));
	if (miv2_mis3) {
		pr_info("%s mi mis3 0x%08x\n", __func__, miv2_mis3);
  		isp_write_reg(pisp_dev, REG_ADDR(miv2_icr3), miv2_mis3);
	}
#elif defined(ISP_MIV1)
	mi_icr_addr = REG_ADDR(mi_icr);
	mi_mis_addr = REG_ADDR(mi_mis);
#endif

    mi_mis = isp_read_reg(pisp_dev, mi_mis_addr);
 #if defined(ISP_MI_PP_READ_RY) || defined (ISP_3DNR_V3) || defined (ISP_MI_PP_WRITE_RY) ||  defined (ISP_MI_HDR_RY)
	miv2_mis2 = isp_read_reg(pisp_dev, REG_ADDR(miv2_mis2));
   pr_info("%s isp mis 0x%08x, mi mis 0x%08x  post mis 0x%08x\n", __func__,  \
 		  isp_mis, mi_mis,  miv2_mis2);
#else
   pr_info("%s isp mis 0x%08x, mi mis 0x%08x\n", __func__,  \
 		  isp_mis, mi_mis);
#endif

  if (isp_mis) {
	isp_mis_t mis_data;
	mis_data.irq_src = SRC_ISP_IRQ;
	mis_data.val = isp_mis;
	pisp_dev->isp_mis = isp_mis;
	isp_irq_write_circle_queue(&mis_data, &pisp_dev->circle_list);
  	isp_write_reg(pisp_dev, REG_ADDR(isp_icr), isp_mis);
	if(isp_mis & MRV_ISP_ISR_ISP_OFF_MASK)
	isp_write_reg(pisp_dev, REG_ADDR(isp_imsc), isp_mis& (~MRV_ISP_ISR_ISP_OFF_MASK));
  }

  if (mi_mis) {
	isp_mis_t mis_data;
	mis_data.irq_src = SRC_MI_IRQ;
	mis_data.val = mi_mis;
	isp_irq_write_circle_queue(&mis_data, &pisp_dev->circle_list);

  	isp_write_reg(pisp_dev, mi_icr_addr, mi_mis);
  }
#ifdef ISP_MIV2_RY
  if (miv2_mis1) {
       isp_mis_t mis_data;
       mis_data.irq_src = SRC_MI1_IRQ;
       mis_data.val = mi_mis;
       isp_irq_write_circle_queue(&mis_data, &pisp_dev->circle_list);

       isp_write_reg(pisp_dev, mi_icr_addr, mi_mis);
  }
#endif

 #if defined(ISP_MI_PP_READ_RY) || defined (ISP_3DNR_V3) || defined (ISP_MI_PP_WRITE_RY) ||  defined (ISP_MI_HDR_RY)
  if (miv2_mis2) {
	isp_mis_t mis_data;
	mis_data.irq_src = SRC_MI2_IRQ;
	mis_data.val = miv2_mis2;
	isp_irq_write_circle_queue(&mis_data, &pisp_dev->circle_list);
  	isp_write_reg(pisp_dev, REG_ADDR(miv2_icr2), miv2_mis2);
  }
#endif

 #if defined(ISP_MI_PP_READ_RY) || defined (ISP_3DNR_V3) || defined (ISP_MI_PP_WRITE_RY) ||  defined (ISP_MI_HDR_RY)
   if (isp_mis != 0 ||mi_mis != 0 || miv2_mis2 != 0) {
 #else
  if (isp_mis != 0 ||mi_mis != 0 ) {
#endif
    schedule_work(&pdriver_dev->vvnative_wq);
  } else {
	return IRQ_NONE;
  }
    return IRQ_HANDLED;
}

static int vvcam_isp_open(struct inode * inode, struct file * file)
{
	struct vvcam_isp_driver_dev *pdriver_dev;
	struct isp_ic_dev * pisp_dev;
	pdriver_dev = container_of(inode->i_cdev, struct vvcam_isp_driver_dev, cdev);
	file->private_data = pdriver_dev;
	pisp_dev = pdriver_dev->private;
	/*create circle queue*/
	isp_irq_create_circle_queue(&(pisp_dev->circle_list), QUEUE_NODE_COUNT);
	return 0;
};

static long vvcam_isp_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret = 0;
	struct vvcam_isp_driver_dev *pdriver_dev;
	struct isp_ic_dev * pisp_dev;
	pr_err("%s:cmd = %d\n", __func__, cmd);

	pdriver_dev = file->private_data;
	if (pdriver_dev == NULL)
	{
		pr_err("%s:file private is null point error\n", __func__);
		return  -ENOMEM;
	}

	pisp_dev = pdriver_dev->private;
	//pr_info("%s:isp[%d] pdriver_dev =0x%px\n", __func__,pdriver_dev->device_idx,pdriver_dev);
	//pr_info("%s:pisp_dev =0x%px\n", __func__,pisp_dev);

	mutex_lock(&pdriver_dev->vvmutex);
	ret = isp_priv_ioctl(pisp_dev, cmd ,(void __user *)arg);
	mutex_unlock(&pdriver_dev->vvmutex);

	return ret;
};

static int vvcam_isp_release(struct inode * inode, struct file * file)
{
	struct vvcam_isp_driver_dev *pdriver_dev;
	struct isp_ic_dev *pisp_dev;

	pdriver_dev = container_of(inode->i_cdev, struct vvcam_isp_driver_dev, cdev);
	file->private_data = pdriver_dev;
	pisp_dev = pdriver_dev->private;
	pr_info("enter %s\n", __func__);
	isp_irq_destroy_circle_queue(&(pisp_dev->circle_list));

	return 0;
};

static int vvcam_isp_mmap(struct file *pFile, struct vm_area_struct *vma)
{
	int ret = 0;

	ulong phy_base_addr = 0;

	unsigned long pfn_start = (phy_base_addr >> PAGE_SHIFT) + vma->vm_pgoff;
	unsigned long size = vma->vm_end - vma->vm_start;
	pr_info("phy: 0x%lx, size: 0x%lx\n", pfn_start << PAGE_SHIFT, size);
	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
	if (remap_pfn_range(vma, vma->vm_start,pfn_start,size, vma->vm_page_prot))
	{
		pr_err("-->%s: remap_pfn_range error!\n", __func__);
		return -EIO;
	}

	return ret;
};

static struct file_operations vvcam_isp_fops = {
	.owner = THIS_MODULE,
	.open = vvcam_isp_open,
	.release = vvcam_isp_release,
	.unlocked_ioctl = vvcam_isp_ioctl,
	.mmap = vvcam_isp_mmap,
    .poll = vvnative_isp_poll,
};

static int vvcam_isp_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct vvcam_isp_driver_dev *pdriver_dev;
	struct isp_ic_dev * pisp_dev;
	struct resource		*mem;

	pr_info("enter %s\n", __func__);

	if (pdev->id >= VIVCAM_ISP_MAXCNT)
	{
		pr_err("%s:pdev id is %d error\n", __func__,pdev->id);
		return  -EINVAL;
	}

	pdriver_dev = devm_kzalloc(&pdev->dev,sizeof(struct vvcam_isp_driver_dev), GFP_KERNEL);
	if (pdriver_dev == NULL)
	{
		pr_err("%s:alloc struct vvcam_isp_driver_dev error\n", __func__);
		return  -ENOMEM;
	}
	memset(pdriver_dev,0,sizeof(struct vvcam_isp_driver_dev ));
	pr_info("%s:isp[%d]: pdriver_dev =0x%px\n", __func__,pdev->id,pdriver_dev);

	pisp_dev = devm_kzalloc(&pdev->dev,sizeof(struct isp_ic_dev), GFP_KERNEL);
	if (pisp_dev == NULL)
	{
		pr_err("%s:alloc struct isp_ic_dev error\n", __func__);
		return  -ENOMEM;
	}
	memset(pisp_dev,0,sizeof(struct isp_ic_dev ));
	pr_info("%s:isp[%d]: psensor_dev =0x%px\n", __func__,pdev->id,pisp_dev);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pisp_dev->base = devm_ioremap_resource(&pdev->dev, mem);
	pisp_dev->reset = NULL;

	pdriver_dev->private = pisp_dev;
	pdriver_dev->device_idx = pdev->id;
	mutex_init(&pdriver_dev->vvmutex);
	platform_set_drvdata(pdev, pdriver_dev);

    pdriver_dev->irq_num = platform_get_irq(pdev, 0);

	if (devise_register_index == 0)
	{
	    /*init work queue*/
	    INIT_WORK(&pdriver_dev->vvnative_wq,  vvnative_isp_work);

	    ret = devm_request_irq(&pdev->dev, pdriver_dev->irq_num, vvcam_isp_irq,
	    IRQF_TRIGGER_RISING|IRQF_SHARED, "ISP_IRQ", (char *)pdriver_dev);
	    if (ret) {
	    	pr_err("%s[%d]:request irq error!\n", __func__, __LINE__);
	    	return ret;
	    }
        init_waitqueue_head(&pdriver_dev->irq_wait);
		if (vvcam_isp_major == 0)
		{
			ret = alloc_chrdev_region(&pdriver_dev->devt, 0, VIVCAM_ISP_MAXCNT, VIVCAM_ISP_NAME);
			if (ret != 0)
			{
				pr_err("%s:alloc_chrdev_region error\n", __func__);
				return ret;
			}
			vvcam_isp_major = MAJOR(pdriver_dev->devt);
			vvcam_isp_minor = MINOR(pdriver_dev->devt);
		}
		else
		{
			pdriver_dev->devt = MKDEV(vvcam_isp_major, vvcam_isp_minor);
			ret = register_chrdev_region(pdriver_dev->devt, VIVCAM_ISP_MAXCNT, VIVCAM_ISP_NAME);
			if (ret)
			{
				pr_err("%s:register_chrdev_region error\n", __func__);
				return ret;
			}
		}
		vvcam_isp_class = class_create(THIS_MODULE, VIVCAM_ISP_NAME);
		if (IS_ERR(vvcam_isp_class))
		{
			pr_err("%s[%d]:class_create error!\n", __func__, __LINE__);
			return -EINVAL;
		}
	}
	pdriver_dev->devt = MKDEV(vvcam_isp_major, vvcam_isp_minor + pdev->id);

	cdev_init(&pdriver_dev->cdev, &vvcam_isp_fops);
	ret = cdev_add(&pdriver_dev->cdev, pdriver_dev->devt, 1);
	if ( ret )
	{
		pr_err("%s[%d]:cdev_add error!\n", __func__, __LINE__);
		return ret;
	}
	pdriver_dev->class = vvcam_isp_class;
	device_create(pdriver_dev->class, NULL, pdriver_dev->devt,
			pdriver_dev, "%s%d", VIVCAM_ISP_NAME, pdev->id);


	devise_register_index++;
	pr_info("exit %s\n", __func__);
	return ret;
}

static int vvcam_isp_remove(struct platform_device *pdev)
{
	struct vvcam_isp_driver_dev *pdriver_dev;

	pr_info("enter %s\n", __func__);
	devise_register_index--;
	pdriver_dev = platform_get_drvdata(pdev);

   // free_irq(pdriver_dev->irq_num, pdriver_dev);

	cdev_del(&pdriver_dev->cdev);
	device_destroy(pdriver_dev->class, pdriver_dev->devt);
	unregister_chrdev_region(pdriver_dev->devt, VIVCAM_ISP_MAXCNT);
	if (devise_register_index == 0)
	{
		class_destroy(pdriver_dev->class);
	}
	return 0;
}

static struct platform_driver vvcam_isp_driver = {
	.probe		= vvcam_isp_probe,
	.remove		= vvcam_isp_remove,
	.driver = {
		.name  = VIVCAM_ISP_NAME,
		.owner = THIS_MODULE,
	}
};

static void vvcam_isp_pdev_release(struct device *dev)
{
	pr_info("enter %s\n", __func__);
}

#ifdef WITH_VVCAM
static struct resource vvcam_isp0_resource[] = {
	[0] = {
		.start = VVISP0_BASE,
		.end   = VVISP0_BASE + ISP_REG_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
    [1] = {
        .start = VIVCAM_ISP_IRQ_NUMBER,
        .end   = VIVCAM_ISP_IRQ_NUMBER,
        .flags = IORESOURCE_IRQ,
    },
};
static struct platform_device vvcam_isp_pdev = {
	.name = VIVCAM_ISP_NAME,
	.id   = 0,
	.resource = vvcam_isp0_resource,
	.num_resources = ARRAY_SIZE(vvcam_isp0_resource),
	.dev.release = vvcam_isp_pdev_release,
};
#endif

#ifdef WITH_VVCAM_DUAL
static struct resource vvcam_isp1_resource[] = {
	[0] = {
		.start = VVISP1_BASE,
		.end   = VVISP1_BASE + ISP_REG_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
    [1] = {
        .start = VIVCAM_ISP_IRQ_NUMBER,
        .end   = VIVCAM_ISP_IRQ_NUMBER,
        .flags = IORESOURCE_IRQ,
    },
};

static struct platform_device vvcam_isp_dual_pdev = {
	.name = VIVCAM_ISP_NAME,
	.id   = 1,
	.resource = vvcam_isp1_resource,
	.num_resources = ARRAY_SIZE(vvcam_isp1_resource),
	.dev.release = vvcam_isp_pdev_release,
};
#endif

static int __init vvcam_isp_init_module(void)
{
	int ret = 0;

	pr_info("enter %s\n", __func__);
#ifdef WITH_VVCAM
	ret = platform_device_register(&vvcam_isp_pdev);
	if (ret)
    {
		pr_err("register platform device failed.\n");
		return ret;
	}
#endif

#ifdef WITH_VVCAM_DUAL
	ret = platform_device_register(&vvcam_isp_dual_pdev);
	if (ret)
    {
		pr_err("register platform device failed.\n");
		return ret;
	}
#endif

	ret = platform_driver_register(&vvcam_isp_driver);
	if (ret)
    {
		pr_err("register platform driver failed.\n");
		return ret;
	}

	return ret;
}

static void __exit vvcam_isp_exit_module(void)
{
	pr_info("enter %s\n", __func__);

	platform_driver_unregister(&vvcam_isp_driver);
#ifdef WITH_VVCAM
	platform_device_unregister(&vvcam_isp_pdev);
#endif

#ifdef WITH_VVCAM_DUAL
	platform_device_unregister(&vvcam_isp_dual_pdev);
#endif
}

module_init(vvcam_isp_init_module);
module_exit(vvcam_isp_exit_module);

MODULE_DESCRIPTION("ISP");
MODULE_LICENSE("GPL");
