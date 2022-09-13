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
#include "video.h"
#include "video_ioctl.h"

#define VIDEO_DRIVER_NAME "video"
#define VIDEO_DRIVER_MAXCNT 100


static struct class *video_driver_class;
static unsigned int video_driver_major = 0;
static unsigned int video_driver_minor = 0;
static unsigned int device_register_index = 0;

#define check_retval(x)\
	do {\
		if ((x))\
			return -EIO;\
	} while (0)

static unsigned int video_poll(struct file * filp, poll_table *wait)
{
	return 0;
}

void video_work(struct work_struct *work)
{

}

irqreturn_t video_irq(int irq, void *dev_id)
{
	video_info("enter %s\n", __func__);
	return IRQ_HANDLED;
}

static int video_open(struct inode * inode, struct file * file)
{
	struct video_drvdata *drvdata;
	drvdata = container_of(inode->i_cdev, struct video_drvdata, cdev);
	file->private_data = drvdata;

	return 0;
};

static int video_get_pipelines(struct video_drvdata *drvdata, void *__user args)
{
    pipline_t *usr_pip = (pipline_t *)args;
    pipline_t *cam_pip = drvdata->piplines;

    check_retval(copy_to_user(usr_pip, cam_pip, sizeof(pipline_t) * drvdata->pipline_num));
	video_info("%s, %d, pipnmu: %d\n", __func__, __LINE__, drvdata->pipline_num);
    return 0;
}

static int video_get_pipelines_num (struct video_drvdata *drvdata, void *__user args)
{
    check_retval(copy_to_user(args, &drvdata->pipline_num, sizeof(int)));
    return 0;
}

static int video_set_path_type(struct video_drvdata *drvdata, void *__user args)
{
	pathtype_set_t pathtype_set;
	check_retval(copy_from_user(&pathtype_set, args, sizeof(pathtype_set_t)));
	video_info("VIDEO_SET_PATH_TYPE pipeline=%d sensor=%s path_type%d ", pathtype_set.pipeline_id , pathtype_set.sensor_name, pathtype_set.path_type);
	return 0;
}

static int video_get_mem_pool_region_id(struct video_drvdata *drvdata, void *__user args)
{
	check_retval(copy_to_user(args, &drvdata->vi_mem_pool_region_id, sizeof(int)));
	return 0;
}

static long video_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret = 0;
	struct video_drvdata *drvdata;

	drvdata = file->private_data;
	if (drvdata == NULL) {
		video_err("%s:file private is null point error\n", __func__);
		return  -ENOMEM;
	}

	mutex_lock(&drvdata->mutex);
	switch (cmd) {
	case VIDEO_GET_PIPLANES:
		ret = video_get_pipelines(drvdata, (void *)arg);
		break;
	case VIDEO_GET_PIPLANES_NUM:
		ret = video_get_pipelines_num(drvdata, (void *)arg);
		break;
	case VIDEO_SET_PATH_TYPE:
		ret = video_set_path_type(drvdata, (void *)arg);
		break;
	case VIDEO_GET_MEM_POOL_REGION_ID:
		ret = video_get_mem_pool_region_id(drvdata, (void *)arg);
		break;
	default:
		ret = -EPERM;
		video_err("%s: unsupported command %d", __func__, cmd);
		break;
	}
	mutex_unlock(&drvdata->mutex);
	return ret;
};

static int video_release(struct inode * inode, struct file * file)
{
	return 0;
};

static int video_mmap(struct file *pFile, struct vm_area_struct *vma)
{
	video_info("enter %s\n", __func__);
	return 0;
};

struct file_operations video_fops = {
	.owner = THIS_MODULE,
	.open = video_open,
	.release = video_release,
	.unlocked_ioctl = video_ioctl,
	.mmap = video_mmap,
	.poll = video_poll,
};

static int video_press_pipline(pipline_t *pipline, struct fwnode_handle *pip_node)
{
    int ret = 0;
    const char *dev_name;
	const char *path_name;
    struct fwnode_handle *nodes;
    sub_dev_info_t *sub_dev = pipline->sub_dev;


    fwnode_for_each_available_child_node(pip_node, nodes) {
        ret = fwnode_property_read_string(nodes, "subdev_name", &dev_name);
        if (ret != 0) {
            video_err("dts press error!!!!!, %s, %d\n", __func__, __LINE__);
            return -1;
        }

        strcpy(sub_dev->name, dev_name);

        ret = fwnode_property_read_u32_array(nodes, "idx", &sub_dev->idx, 1);
        if (ret != 0) {
	        video_err("dts press  device :%s error!!!!!, %d, %s\n", __func__, __LINE__,dev_name);
            return -1;
        }

		ret = fwnode_property_read_string(nodes, "path_type", &path_name);
		if (ret != 0) {
	        video_err("%s, %d, dts parse %s->%s error!!!!!\n", __func__, __LINE__, dev_name, path_name);
            return -1;
        }

        video_info("%s, %d, dev: %s->%s !!!!!\n", __func__, __LINE__, dev_name, path_name);
		sub_dev->path = vedio_get_path_type(dev_name, path_name);
		if(sub_dev->path < 0) {
            video_err("%s, %d, %s, parse path(%s) error!!!!!\n", __func__, __LINE__, dev_name, path_name);
            return -1;
		}
        video_info("subdev:%s-%d,path:(%s,%d)\n",dev_name,sub_dev->idx, path_name,sub_dev->path);

        ret = video_subdev_press_dts(nodes, sub_dev);
        if (ret != 0) {
	        video_err("dts press error!!!!!, %s, %d\n", __func__, __LINE__);
            return -1;
        }

        sub_dev++;
        pipline->sub_dev_num++;
    }

    return 0;
}

static int video_press_dts(struct platform_device *pdev)
{
	int ret = 0;
	struct device *dev = &pdev->dev;
	struct fwnode_handle *child;
	struct video_drvdata *drvdata;
	pipline_t *pipline;
	int pool_region_id;

	drvdata  = platform_get_drvdata(pdev);
	pipline = drvdata->piplines;

	//if (device_property_read_u32(&(pdev->dev), "vi_mem_pool_region", &pool_region_id) != 0) {
	if (of_property_read_s32(dev->of_node, "vi_mem_pool_region", &pool_region_id) != 0) {
		video_warn("%s, dev(%s) can't get vi_mem_pool_region id from dts, set to default -1(cma)\n",
				   __func__, pdev->name);
		pool_region_id = -1;
	}
	drvdata->vi_mem_pool_region_id = pool_region_id;

	device_for_each_child_node(dev, child) {
		video_info("pipleline(%d):",drvdata->pipline_num);
		ret = video_press_pipline(pipline, child);
		if (ret != 0) {
			video_err("%s, %d, dts press pipe:%d error!\n", __func__, __LINE__, drvdata->pipline_num);
			return -1;
		}
		pipline++;
		drvdata->pipline_num++;
	}

	return 0;
}

static int video_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct video_drvdata *drvdata;
	struct device_node *np = pdev->dev.of_node;

	video_info("enter %s\n", __func__);
	pdev->id = of_alias_get_id(np, "viv_video");

	video_info("%s:pdev id is %d\n", __func__, pdev->id);

	if (pdev->id >= VIDEO_DRIVER_MAXCNT) {
		video_err("%s:pdev id is %d error\n", __func__, pdev->id);
		return  -EINVAL;
	}

	drvdata = devm_kzalloc(&pdev->dev,sizeof(struct video_drvdata), GFP_KERNEL);
	if (drvdata == NULL) {
		video_err("%s:alloc struct drvdata error\n", __func__);
		return  -ENOMEM;
	}

	drvdata->device_idx = pdev->id;
	mutex_init(&drvdata->mutex);
	platform_set_drvdata(pdev, drvdata);

	if (device_register_index == 0) {
		if (video_driver_major == 0) {
			ret = alloc_chrdev_region(&drvdata->devt, 0, VIDEO_DRIVER_MAXCNT, VIDEO_DRIVER_NAME);
			if (ret != 0) {
				video_err("%s:alloc_chrdev_region error\n", __func__);
				return ret;
			}
			video_driver_major = MAJOR(drvdata->devt);
			video_driver_minor = MINOR(drvdata->devt);
		} else {
			drvdata->devt = MKDEV(video_driver_major, video_driver_minor);
			ret = register_chrdev_region(drvdata->devt, VIDEO_DRIVER_MAXCNT, VIDEO_DRIVER_NAME);
			if (ret) {
				video_err("%s:register_chrdev_region error\n", __func__);
				return ret;
			}
		}

		video_driver_class = class_create(THIS_MODULE, VIDEO_DRIVER_NAME);
		if (IS_ERR(video_driver_class)) {
			video_err("%s[%d]:class_create error!\n", __func__, __LINE__);
			return -EINVAL;
		}
	}

	drvdata->devt = MKDEV(video_driver_major, video_driver_minor + pdev->id);
	cdev_init(&drvdata->cdev, &video_fops);
	ret = cdev_add(&drvdata->cdev, drvdata->devt, 1);
	if ( ret ) {
		video_err("%s[%d]:cdev_add error!\n", __func__, __LINE__);
		return ret;
	} else {
		video_info("%s[%d]:cdev_add OK, major=%d, minor=%d\n", __func__, __LINE__,
			video_driver_major, video_driver_minor + pdev->id);
	}

	drvdata->class = video_driver_class;
	device_create(drvdata->class, NULL, drvdata->devt,
		      drvdata, "%s%d", VIDEO_DRIVER_NAME, pdev->id);

    ret = video_press_dts(pdev);
	if (ret) {
		video_err("%s[%d]:dts error!\n", __func__, __LINE__);
		return ret;
    }

    video_create_capabilities_sysfs(pdev);
	device_register_index++;

	video_info("exit %s:[%s%d]\n", __func__, VIDEO_DRIVER_NAME, pdev->id);

	return 0;
}

static int video_remove(struct platform_device *pdev)
{
	struct video_drvdata *drvdata;

	video_info("enter %s\n", __func__);
	device_register_index--;
	drvdata = platform_get_drvdata(pdev);
	cdev_del(&drvdata->cdev);
	device_destroy(drvdata->class, drvdata->devt);
	unregister_chrdev_region(drvdata->devt, VIDEO_DRIVER_MAXCNT);
	mutex_destroy(&drvdata->mutex);
    video_remove_capabilities_sysfs(pdev);
	if (device_register_index == 0) {
		class_destroy(drvdata->class);
	}
	devm_kfree(&pdev->dev, drvdata);
	video_info("exit %s\n", __func__);
	return 0;
}

static const struct of_device_id video_of_match[] = {
	{ .compatible = "thead,video", },
	{ /* sentinel */ },
};

static struct platform_driver video_driver = {
	.probe		= video_probe,
	.remove		= video_remove,
	.driver = {
		.name  = VIDEO_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(video_of_match),
	}
};

static int __init video_init_module(void)
{
	int ret = 0;

	video_info("enter %s\n", __func__);
	ret = platform_driver_register(&video_driver);
	if (ret) {
		video_err("register platform driver failed.\n");
		return ret;
	}

	return ret;
}

static void __exit video_exit_module(void)
{
	video_info("enter %s\n", __func__);
	platform_driver_unregister(&video_driver);
}

module_init(video_init_module);
module_exit(video_exit_module);

MODULE_AUTHOR("Liu Yitong");
MODULE_DESCRIPTION("THEAD-VIDEO");
MODULE_LICENSE("GPL");
