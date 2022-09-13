#ifndef _THEAD_VIDEO_H_
#define _THEAD_VIDEO_H_

#include <linux/cdev.h>
#include <linux/ioctl.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/phy/phy.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/ratelimit.h>
#include <linux/reset.h>
#include <linux/videodev2.h>
#include <linux/wait.h>
#include "subdev.h"

#define video_info(fmt, ...) printk(KERN_DEBUG   pr_fmt(fmt), ##__VA_ARGS__)
#define video_warn(fmt, ...) printk(KERN_WARNING pr_fmt(fmt), ##__VA_ARGS__)
#define video_err(fmt, ...)  printk(KERN_ERR     pr_fmt(fmt), ##__VA_ARGS__)

typedef struct {
	int sub_dev_num;
	sub_dev_info_t sub_dev[20];
} pipline_t;

typedef struct video_drvdata {
	struct cdev cdev;
	dev_t devt;
	struct class *class;
	int device_idx;
	struct mutex mutex;
	int pipline_num;
	pipline_t piplines[20];
	int vi_mem_pool_region_id;
	void *private;
} video_drvdata_t;

typedef struct {
	char *subdev_name;
	char *sensor_name;
	unsigned int path_type;
	unsigned int pipeline_id;
} pathtype_set_t;

int video_create_capabilities_sysfs(struct platform_device *pdev);
int video_remove_capabilities_sysfs(struct platform_device *pdev);
int vedio_get_path_type(const char *subdev_name, const char *path_name);

#endif /* _THEAD_VIDEO_H_ */
