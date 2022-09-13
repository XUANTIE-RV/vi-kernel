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

#ifndef _SENSOR_COMMON_H_
#define _SENSOR_COMMON_H_
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/phy/phy.h>
#include <linux/phy/phy-mipi-dphy.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>


#include "vvsensor.h"

#define UNDEFINED_IN_DTS	0xFF

struct vvcam_sensor_driver_dev
{
	struct cdev cdev;
	dev_t devt;
	struct class *class;
	struct mutex vvmutex;
	void *private;
};

struct vvcam_sensor_function_s
{
	uint8_t sensor_name[16];
	uint32_t reserve_id;
	uint32_t  sensor_clk;
	struct sensor_mipi_info mipi_info;

	int32_t (*sensor_get_chip_id)   (void *ctx, uint32_t *chip_id);
	int32_t (*sensor_init)          (void *ctx, struct vvcam_mode_info *pmode);
	int32_t (*sensor_set_stream)    (void *ctx, uint32_t status);
	int32_t (*sensor_set_exp)       (void *ctx, uint32_t exp_line);
	int32_t (*sensor_set_vs_exp)    (void *ctx, uint32_t exp_line);
	int32_t (*sensor_set_gain)      (void *ctx, uint32_t gain);
	int32_t (*sensor_set_vs_gain)   (void *ctx, uint32_t gain);
	int32_t (*sensor_set_fps)       (void *ctx, uint32_t fps);
	int32_t (*sensor_set_resolution)(void *ctx, uint32_t width, uint32_t height);
	int32_t (*sensor_set_hdr_mode)  (void *ctx, uint32_t hdr_mode);
	int32_t (*sensor_query)         (void *ctx, struct vvcam_mode_info_array *pmode_info_arry);
};

#define VVCAM_SENSOR_MAX_REGULATORS 	10
struct vvcam_sensor_regulators {
	struct regulator *supply[VVCAM_SENSOR_MAX_REGULATORS];
	const char *name[VVCAM_SENSOR_MAX_REGULATORS];
	unsigned int delay_us[VVCAM_SENSOR_MAX_REGULATORS];
	int num;
};

struct vvcam_sensor_dev {
	long phy_addr;
	long reg_size;
	void __iomem *base;
	int32_t device_idx;

	uint8_t i2c_bus;
	void* i2c_client;
	struct vvcam_sccb_cfg_s sensor_sccb_cfg;
	struct vvcam_sccb_cfg_s focus_sccb_cfg;
	struct vvcam_sensor_function_s sensor_func;

	struct vvcam_mode_info sensor_mode;
	struct vvcam_ae_info_s ae_info;
    const char *sensor_name;
	struct vvcam_sensor_regulators regulators;
	int pdn_pin;
	unsigned int pdn_delay_us;
	int rst_pin;
};

extern int32_t sensor_reset(void *dev);
extern int32_t sensor_set_clk(void *dev, uint32_t clk);
extern int32_t sensor_get_clk(void *dev, uint32_t *pclk);
extern int32_t sensor_set_power(void *dev, uint32_t power);
extern int32_t sensor_get_power(void *dev, uint32_t *ppower);

int32_t vvcam_sensor_i2c_write(struct vvcam_sensor_dev *dev, uint32_t address, uint32_t data);
int32_t vvcam_sensor_i2c_read(struct vvcam_sensor_dev *dev, uint32_t address, uint32_t *pdata);

int vvnative_sensor_init(struct vvcam_sensor_dev *dev);
int vvnative_sensor_deinit(struct vvcam_sensor_dev *dev);
long sensor_priv_ioctl(struct vvcam_sensor_dev *dev, unsigned int cmd, void __user *args);


#endif
