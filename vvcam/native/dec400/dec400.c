/*
 * Verisilicon DEC400 Driver for FalconLite.
 *
 * Author: Wei Weiyu <Weiyu.Wei@verisilicon.com>
 *
 * Copyright (C) 2020 VeriSilicon Microelectronics (Shanghai) Co., Ltd.
 *
 */

#include "dec400.h"
#include "dec400_ioctl.h"

static unsigned int dec400_major = 0;
static unsigned int dec400_minor = 0;
struct class *dec400_class;
static unsigned int device_register_index = 0;

extern unsigned int dec400_priv_ioctl(struct dec400_dev *dev, unsigned int cmd, void *args);

static int dec400_open(struct inode * inode, struct file * file)
{
	struct dec400_dev *pdriver_dev;

	pr_info("entry %s\n", __func__);
	pdriver_dev = container_of(inode->i_cdev, struct dec400_dev, cdev);
	file->private_data = pdriver_dev;

	pr_info("exit %s\n", __func__);
	return 0;
};

static long dec400_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret = 0;
	struct dec400_dev *pdriver_dev;

	/*pr_info("enter %s\n", __func__);*/
	pdriver_dev = file->private_data;
	if (pdriver_dev == NULL) {
		pr_err("%s:file private is null point error\n", __func__);
		return  -ENOMEM;
	}

	mutex_lock(&pdriver_dev->mutex);
	ret = dec400_priv_ioctl(pdriver_dev, cmd ,(void *)arg);
	mutex_unlock(&pdriver_dev->mutex);

	/*pr_info("exit %s\n", __func__);*/

	return ret;
};

static int dec400_release(struct inode * inode, struct file * file)
{
	struct dec400_dev *pdriver_dev;

	pr_info("enter %s\n", __func__);

	pdriver_dev = container_of(inode->i_cdev, struct dec400_dev, cdev);
	file->private_data = pdriver_dev;

	pr_info("exit %s\n", __func__);

	return 0;
};

struct file_operations dec400_fops = {
	.owner = THIS_MODULE,
	.open = dec400_open,
	.release = dec400_release,
	.unlocked_ioctl = dec400_ioctl,
	.mmap = NULL,
	.poll = NULL,
};

static int dec400_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct dec400_dev *pdriver_dev;
	struct resource *mem;

	pr_info("enter %s\n", __func__);
	pdev->id = device_register_index;
	if (pdev->id >= DEC400_MAXCNT) {
		pr_err("%s:pdev id is %d error\n", __func__, pdev->id);
		return  -EINVAL;
	}

	pdriver_dev = devm_kzalloc(&pdev->dev, sizeof(struct dec400_dev), GFP_KERNEL);
	if (pdriver_dev == NULL) {
		pr_err("%s:alloc struct dec400_dev error\n", __func__);
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
	mutex_init(&pdriver_dev->mutex);
	platform_set_drvdata(pdev, pdriver_dev);

	if (device_register_index == 0) {
		int ret;

		if (dec400_major == 0) {
			ret = alloc_chrdev_region(&pdriver_dev->devt, 0, DEC400_MAXCNT, DEC400_NAME);
			if (ret != 0) {
				pr_err("%s:alloc_chrdev_region error\n", __func__);
				return ret;
			}
			dec400_major = MAJOR(pdriver_dev->devt);
			dec400_minor = MINOR(pdriver_dev->devt);
		}
		else
		{
			pdriver_dev->devt = MKDEV(dec400_major, dec400_minor);
			ret = register_chrdev_region(pdriver_dev->devt, DEC400_MAXCNT, DEC400_NAME);
			if (ret) {
				pr_err("%s:register_chrdev_region error\n", __func__);
				return ret;
			}
		}
		dec400_class = class_create(THIS_MODULE, DEC400_NAME);
		if (IS_ERR(dec400_class)) {
			pr_err("%s[%d]:class_create error!\n", __func__, __LINE__);
			return -EINVAL;
		}
	}
	pdriver_dev->devt = MKDEV(dec400_major, dec400_minor + pdev->id);

	cdev_init(&pdriver_dev->cdev, &dec400_fops);
	ret = cdev_add(&pdriver_dev->cdev, pdriver_dev->devt, 1);
	if ( ret ) {
		pr_err("%s[%d]:cdev_add error!\n", __func__, __LINE__);
		return ret;
	}
	pdriver_dev->class = dec400_class;
	device_create(pdriver_dev->class, NULL, pdriver_dev->devt,
		      pdriver_dev, "%s%d", DEC400_NAME, pdev->id);

	device_register_index++;
	pr_info("exit %s:[%d]\n", __func__, pdev->id);

	return ret;
}


static int dec400_remove(struct platform_device *pdev)
{
	struct dec400_dev *pdriver_dev;

	pr_info("enter %s\n", __func__);
	device_register_index--;
	pdriver_dev = platform_get_drvdata(pdev);

	cdev_del(&pdriver_dev->cdev);
	device_destroy(pdriver_dev->class, pdriver_dev->devt);
	unregister_chrdev_region(pdriver_dev->devt, DEC400_MAXCNT);
	if (device_register_index == 0) {
		class_destroy(pdriver_dev->class);
	}
	return 0;
}


static const struct of_device_id dec400_of_match_table[] = {
	{ .compatible = "thead,dec400", },
	{ },
};

static struct platform_driver dec400_driver = {
	.probe  = dec400_probe,
	.remove = dec400_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name  = DEC_DEV_NAME,
		.of_match_table = dec400_of_match_table,
	},
};
		
static int __init dec400_init_module(void)
{
	int ret = 0;

	pr_info("enter %s\n", __func__);

	ret = platform_driver_register(&dec400_driver);
	if (ret) {
		pr_err("register platform driver failed.\n");
		return ret;
	}

	return ret;
}

static void __exit dec400_exit_module(void)
{
	pr_info("enter %s\n", __func__);

	platform_driver_unregister(&dec400_driver);
}

module_init(dec400_init_module);
module_exit(dec400_exit_module);

MODULE_DESCRIPTION("DEC400");
MODULE_LICENSE("GPL");
