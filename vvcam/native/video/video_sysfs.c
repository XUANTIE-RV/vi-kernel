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
#include "video.h"
#include "video_kernel_defs.h"
#if 0
static ssize_t n_lanes_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	int ret;
	unsigned long lanes;

	struct platform_device *pdev = to_platform_device(dev);
    struct bm_csi_drvdata *drvdata  = platform_get_drvdata(pdev);
	struct dw_csi *csi_dev = &drvdata->csi_dev;

	ret = kstrtoul(buf, 10, &lanes);
	if (ret < 0)
		return ret;

	if (lanes > 8) {
		dev_err(dev, "Invalid number of lanes %lu\n", lanes);
		return count;
	}

	dev_info(dev, "Lanes %lu\n", lanes);
	csi_dev->hw.num_lanes = lanes;

	return count;
}

static ssize_t n_lanes_show(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
    struct platform_device *pdev = to_platform_device(dev);
    struct video_drvdata *drvdata  = platform_get_drvdata(pdev);


	char buffer[10];

	snprintf(buffer, 10, "%d\n", csi_dev->hw.num_lanes);

	return strlcpy(buf, buffer, PAGE_SIZE);
}
#endif

static void show_subdev(pipline_t *pipline)
{
    int i = 0;
    sub_dev_info_t *sub_dev;

    sub_dev = pipline->sub_dev;
    while(i < pipline->sub_dev_num) {
        video_info("dev[%d]: %s%d\n", i, sub_dev->name, sub_dev->idx);
        sub_dev++;
        i++;
    }
}

static ssize_t piplines_show(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
    int i = 0;
    struct platform_device *pdev = to_platform_device(dev);
    struct video_drvdata *drvdata  = platform_get_drvdata(pdev);
    pipline_t *pipline = drvdata->piplines;
    while(i < drvdata->pipline_num) {
        video_info("/*pipline[%d]*******/\n", i);
        show_subdev(pipline);
        video_info("\n\n");
        pipline++;
        i++;
    }

    return 0;
}

//static DEVICE_ATTR_RW(n_lanes);
static DEVICE_ATTR_RO(piplines);

int video_create_capabilities_sysfs(struct platform_device *pdev)
{
	device_create_file(&pdev->dev, &dev_attr_piplines);
	return 0;
}

int video_remove_capabilities_sysfs(struct platform_device *pdev)
{
	device_remove_file(&pdev->dev, &dev_attr_piplines);
	return 0;
}

