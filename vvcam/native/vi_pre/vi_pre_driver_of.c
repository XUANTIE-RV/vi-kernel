/*
 * Copyright (C) 2021 Alibaba Group Holding Limited
 * Author: Shenwuyi <shenwuyi.swy@alibaba-inc.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/poll.h>
#include <linux/pm_runtime.h>
#include "vi_pre.h"
#include "vi_pre_ioctl.h"

static unsigned int vi_pre_major = 0;
static unsigned int vi_pre_minor = 0;
struct class *vi_pre_class;
static unsigned int device_register_index = 0;

static int vipre_runtime_suspend(struct device *dev)
{
    struct vi_pre_dev *pdriver_dev = dev_get_drvdata(dev);
    pr_info("enter %s\n", __func__);

    if (pdriver_dev->aclk!= NULL) {
        clk_disable_unprepare(pdriver_dev->aclk);
    }

    if (pdriver_dev->pclk != NULL) {
        clk_disable_unprepare(pdriver_dev->pclk);
    }

    if (pdriver_dev->pixclk!= NULL) {
        clk_disable_unprepare(pdriver_dev->pixclk);
    }

    pr_info("exit %s\n", __func__);
    return 0;
}
static int vipre_runtime_resume(struct device *dev)
{
    int ret = 0;
    struct vi_pre_dev *pdriver_dev = dev_get_drvdata(dev);

    if (pdriver_dev->pclk != NULL) {
        ret = clk_prepare_enable(pdriver_dev->pclk);
        if (ret < 0) {
            dev_err(dev, "could not prepare or enable csi pclk\n");
            clk_disable_unprepare(pdriver_dev->pclk);
        }
    }

    if (pdriver_dev->pixclk!= NULL) {
        ret = clk_prepare_enable(pdriver_dev->pixclk);
        if (ret < 0) {
            dev_err(dev, "could not prepare or enable  pixclk\n");
            clk_disable_unprepare(pdriver_dev->pixclk);
        }
    }

    if (pdriver_dev->aclk!= NULL) {
        ret = clk_prepare_enable(pdriver_dev->aclk);
        if (ret < 0) {
            dev_err(dev, "could not prepare or enable aclk\n");
            clk_disable_unprepare(pdriver_dev->aclk);
        }
    }

    pr_info("%s Enabled clock\n", __func__);
    return ret;
}

static const struct dev_pm_ops vipre_runtime_pm_ops = {
    SET_RUNTIME_PM_OPS(vipre_runtime_suspend, vipre_runtime_resume, NULL)
};

static int vi_pre_open(struct inode * inode, struct file * file)
{
    int ret = 0;
	struct vi_pre_dev *pdriver_dev;
    struct device *dev;

	pr_info("entry %s\n", __func__);
	pdriver_dev = container_of(inode->i_cdev, struct vi_pre_dev, cdev);
	file->private_data = pdriver_dev;

    dev = &pdriver_dev->pdev->dev;
    if (pm_runtime_get_sync(dev)) {
        ret = vipre_runtime_resume(dev);
        if (ret)
            pr_err("fail to resume csi %s %d\n", __func__, __LINE__);
    }

	pr_info("exit %s\n", __func__);
	return 0;
};

static long vi_pre_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret = 0;
	struct vi_pre_dev *pdriver_dev;

	pr_info("enter %s\n", __func__);
	//printk("%s, %d, vipre_cmd %u\n", __func__, __LINE__, cmd);
	pdriver_dev = file->private_data;
	if (pdriver_dev == NULL) {
		pr_err("%s:file private is null point error\n", __func__);
		return  -ENOMEM;
	}

	mutex_lock(&pdriver_dev->mutex);

	ret = vi_pre_priv_ioctl(pdriver_dev, cmd ,(void *)arg);
	mutex_unlock(&pdriver_dev->mutex);

	pr_info("exit %s\n", __func__);

	return ret;
};

static int vi_pre_release(struct inode * inode, struct file * file)
{
    int ret = 0;
	struct vi_pre_dev *pdriver_dev;
    struct device *dev;

	pr_info("enter %s\n", __func__);

	pdriver_dev = container_of(inode->i_cdev, struct vi_pre_dev, cdev);
	file->private_data = pdriver_dev;

    dev = &pdriver_dev->pdev->dev;

    ret = pm_runtime_put_sync(dev);
    if (ret) {
        pr_info("fail to resume vipre %s %d\n", __func__, __LINE__);
    }

	pr_info("exit %s\n", __func__);

	return 0;
};

static int vi_pre_mmap(struct file *pFile, struct vm_area_struct *vma)
{
    #define img_buf 0xf0000000

	pr_info("enter %s\n", __func__);
    vma->vm_flags |= VM_IO;
    vma->vm_flags |= (VM_DONTEXPAND | VM_DONTDUMP);
    if(remap_pfn_range(vma,
                       vma->vm_start,
                       img_buf >> PAGE_SHIFT,
                       vma->vm_end - vma->vm_start,
                       vma->vm_page_prot)) {
        return -EAGAIN;
    }

	pr_info("exit %s\n", __func__);
	return 0;
};

static DECLARE_WAIT_QUEUE_HEAD(vipre_waitq);
static unsigned vi_pre_poll(struct file *file, poll_table *wait)
{
    extern int *vi_pre_event(void);
    int *event = 0;
    unsigned int mask = 0;
    event = vi_pre_event();
    poll_wait(file, &vipre_waitq, wait);

    if (*event) {
        mask |= POLLIN | POLLRDNORM;
        *event = 0;
    }

    return mask;
}

struct file_operations vi_pre_fops = {
	.owner = THIS_MODULE,
	.open = vi_pre_open,
	.release = vi_pre_release,
	.unlocked_ioctl = vi_pre_ioctl,
	.mmap = vi_pre_mmap,
	.poll = vi_pre_poll,
};

static irqreturn_t vi_pre_irq(int irq, void *dev_id)
{
    extern void vi_pre_dma_interrupt_handler(struct vi_pre_dev *pdriver_dev);
	struct vi_pre_dev *pdriver_dev = dev_id;
    vi_pre_dma_interrupt_handler(pdriver_dev);
    wake_up_interruptible(&vipre_waitq);

    return IRQ_HANDLED;
}

static int vi_pre_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct vi_pre_dev *pdriver_dev;
	struct resource *mem;
    struct device *dev = NULL;

	pr_info("enter %s\n", __func__);
    dev = &pdev->dev;
	pdev->id = device_register_index;
	if (pdev->id >= VI_PRE_MAXCNT) {
		pr_err("%s:pdev id is %d error\n", __func__, pdev->id);
		return  -EINVAL;
	}

	pdriver_dev = devm_kzalloc(&pdev->dev, sizeof(struct vi_pre_dev), GFP_KERNEL);
	if (pdriver_dev == NULL) {
		pr_err("%s:alloc struct vi_pre_dev error\n", __func__);
		return  -ENOMEM;
	}
	pr_info("%s:isp[%d]: pdriver_dev =0x%px\n", __func__, pdev->id, pdriver_dev);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pdriver_dev->reg_base = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(pdriver_dev->reg_base))
		return PTR_ERR(pdriver_dev->reg_base);
	pr_info("%s:isp[%d]: pdriver_dev->base=0x%px\n", __func__,
		pdev->id, pdriver_dev->reg_base);

	pdriver_dev->id = pdev->id;
    pdriver_dev->pdev = pdev;
	mutex_init(&pdriver_dev->mutex);

   /*parse clk info from dts*/
    pdriver_dev->pclk = devm_clk_get(&pdev->dev, "pclk");
    if (IS_ERR(pdriver_dev->pclk)) {
        dev_err(&pdev->dev, "failed to get pclk");
        pdriver_dev->pclk = NULL;
    }

    pdriver_dev->pixclk = devm_clk_get(&pdev->dev, "pixclk");
    if (IS_ERR(pdriver_dev->pixclk)) {
        dev_err(&pdev->dev, "failed to get pixclk");
        pdriver_dev->pixclk = NULL;
    }

    pdriver_dev->aclk = devm_clk_get(&pdev->dev, "aclk");
    if (IS_ERR(pdriver_dev->aclk)) {
        dev_err(&pdev->dev, "failed to get aclk");
        pdriver_dev->aclk = NULL;
    }

	platform_set_drvdata(pdev, pdriver_dev);

	if (device_register_index == 0) {
		int ret;

		if (vi_pre_major == 0) {
			ret = alloc_chrdev_region(&pdriver_dev->devt, 0, VI_PRE_MAXCNT, VI_PRE_NAME);
			if (ret != 0) {

                    pr_err("%s:alloc_chrdev_region error\n", __func__);
				return ret;
			}
			vi_pre_major = MAJOR(pdriver_dev->devt);
			vi_pre_minor = MINOR(pdriver_dev->devt);
		}
		else
		{
			pdriver_dev->devt = MKDEV(vi_pre_major, vi_pre_minor);
			ret = register_chrdev_region(pdriver_dev->devt, VI_PRE_MAXCNT, VI_PRE_NAME);
			if (ret) {
				pr_err("%s:register_chrdev_region error\n", __func__);
				return ret;
			}
		}
		vi_pre_class = class_create(THIS_MODULE, VI_PRE_NAME);
		if (IS_ERR(vi_pre_class)) {
			pr_err("%s[%d]:class_create error!\n", __func__, __LINE__);
			return -EINVAL;
		}
	}
	pdriver_dev->devt = MKDEV(vi_pre_major, vi_pre_minor + pdev->id);

	cdev_init(&pdriver_dev->cdev, &vi_pre_fops);
	ret = cdev_add(&pdriver_dev->cdev, pdriver_dev->devt, 1);
	if ( ret ) {
		pr_err("%s[%d]:cdev_add error!\n", __func__, __LINE__);
		return ret;
	}
	pdriver_dev->class = vi_pre_class;
	device_create(pdriver_dev->class, NULL, pdriver_dev->devt,
		      pdriver_dev, "%s%d", VI_PRE_NAME, pdev->id);

    pdriver_dev->irq = platform_get_irq(pdev, 0);
    if (pdriver_dev->irq < 0) {
        dev_err(dev, "vipre get irq error: %d\n", pdriver_dev->id);
        ret = pdriver_dev->irq;
        goto end;
    }

    spin_lock_init(&pdriver_dev->slock);
    if (device_property_read_bool(dev, "vi_pre_irq_en")) {
        ret = devm_request_irq(dev, pdriver_dev->irq,
                               vi_pre_irq, IRQF_SHARED,
                               dev_name(dev), pdriver_dev);
        if (ret) {
            dev_err(dev, "irq vipre failed\n");
            goto end;
        }
    }

    pm_runtime_enable(&pdev->dev);
    ret = vipre_runtime_resume(&pdev->dev);
    if (ret < 0) {
        dev_err(&pdev->dev, "fail to resume vipre\n");
    }

    vipre_runtime_suspend(&pdev->dev);
    if (ret < 0) {
        dev_err(&pdev->dev, "fail to suspend vipre\n");
    }

	device_register_index++;
	pr_info("exit %s:[%d]\n", __func__, pdev->id);

end:
	return ret;
}


static int vi_pre_remove(struct platform_device *pdev)
{
	struct vi_pre_dev *pdriver_dev;

	pr_info("enter %s\n", __func__);
	device_register_index--;
	pdriver_dev = platform_get_drvdata(pdev);

	cdev_del(&pdriver_dev->cdev);
	device_destroy(pdriver_dev->class, pdriver_dev->devt);
	unregister_chrdev_region(pdriver_dev->devt, VI_PRE_MAXCNT);
	if (device_register_index == 0) {
		class_destroy(pdriver_dev->class);
	}
    devm_kfree(&pdev->dev, pdriver_dev);

	return 0;
}


static const struct of_device_id vi_pre_of_match_table[] = {
	{ .compatible = "thead,vi_pre", },
	{ },
};

static struct platform_driver vi_pre_driver = {
	.probe  = vi_pre_probe,
	.remove = vi_pre_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name  = VI_PRE_DEV_NAME,
		.of_match_table = vi_pre_of_match_table,
        .pm = &vipre_runtime_pm_ops,
	},
};

static int __init vi_pre_init_module(void)
{
	int ret = 0;

	pr_info("enter %s\n", __func__);

	ret = platform_driver_register(&vi_pre_driver);
	if (ret) {
		pr_err("register platform driver failed.\n");
		return ret;
	}
	pr_info("exit %s\n", __func__);

	return ret;
}

static void __exit vi_pre_exit_module(void)
{
	pr_info("enter %s\n", __func__);

	platform_driver_unregister(&vi_pre_driver);
	pr_info("exit %s\n", __func__);
}

module_init(vi_pre_init_module);
module_exit(vi_pre_exit_module);

MODULE_DESCRIPTION("VI_PRE");
MODULE_LICENSE("GPL");
