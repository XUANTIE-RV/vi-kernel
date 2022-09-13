// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2018-2019 Synopsys, Inc. and/or its affiliates.
 *
 * Synopsys DesignWare MIPI D-PHY controller driver.
 * SysFS components for the platform driver
 *
 * Author: Luis Oliveira <luis.oliveira@synopsys.com>
 */


#include "sensor_common.h"

static ssize_t cam_info_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct vvcam_sensor_driver_dev *pdriver_dev;
	struct vvcam_sensor_dev * psensor_dev;
   	char buffer[15];

    pdriver_dev = platform_get_drvdata(pdev);
	psensor_dev = (struct vvcam_sensor_dev *)pdriver_dev->private;

    printk("<0>""sensor name: %s\n", psensor_dev->sensor_func.sensor_name);
	snprintf(buffer,
		 sizeof(buffer),
		 "i2c id is :%d\n", psensor_dev->i2c_bus);

	return strlcpy(buf, buffer, PAGE_SIZE);
}

static DEVICE_ATTR_RO(cam_info);

int sensor_create_capabilities_sysfs(struct platform_device *pdev)
{
	device_create_file(&pdev->dev, &dev_attr_cam_info);
	return 0;
}

int sensor_remove_capabilities_sysfs(struct platform_device *pdev)
{
	device_remove_file(&pdev->dev, &dev_attr_cam_info);
	return 0;
}
