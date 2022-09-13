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
#include <linux/regulator/consumer.h>
#include <linux/time.h>
#include <linux/timex.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/spinlock.h>
#include <linux/dma-mapping.h>
#include <linux/dma-buf.h>
#include <linux/iio/consumer.h>
#include "flash_led_driver.h"
#include "flash_led_ioctl.h"


#define FLASH_LED_NAME "flash_led"
#define FLASH_LED_MAXCNT 10

static unsigned int flash_led_major = 0;
static unsigned int flash_led_minor = 0;
struct class *flash_led_class;
static unsigned int devise_register_index = 0;
extern int get_ntc_temperature(int mv);


static int flash_led_open(struct inode * inode, struct file * file)
{
    int ret = 0;
	struct flash_led_driver_dev *pdriver_dev;
	struct flash_led_ctrl *pflash_led_dev;

	pdriver_dev = container_of(inode->i_cdev, struct flash_led_driver_dev, cdev);
	file->private_data = pdriver_dev;

	pflash_led_dev = (struct flash_led_ctrl*)pdriver_dev->private;

    struct flash_led_dev *floodlight = &pflash_led_dev->floodlight;
    struct flash_led_dev *projection = &pflash_led_dev->projection;

    if (floodlight->flash_led_func != NULL) {
        ret = floodlight->flash_led_func->init(floodlight);
        if (ret != 0) {
		    pr_err("%s, %d, floodlight init error\n", __func__, __LINE__);
            return ret;
        }
    }

    if (projection->flash_led_func != NULL) {
        ret = projection->flash_led_func->init(projection);
        if (ret != 0) {
		    pr_err("%s, %d, projection init error\n", __func__, __LINE__);
            return ret;
        }
    }

	return 0;
};

static long flash_led_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret = 0;
	struct flash_led_driver_dev *pdriver_dev;
	struct flash_led_ctrl *pflash_led_dev;

	pdriver_dev = file->private_data;
	if (pdriver_dev == NULL)
	{
		pr_err("%s:file private is null point error\n", __func__);
		return  -ENOMEM;
	}
	pflash_led_dev = (struct flash_led_ctrl*)pdriver_dev->private;

	mutex_lock(&pdriver_dev->vvmutex);
    ret = flash_led_priv_ioctl(pflash_led_dev, cmd, (void __user *)arg);
	mutex_unlock(&pdriver_dev->vvmutex);

	return ret;
};

static int flash_led_release(struct inode * inode, struct file * file)
{

	struct flash_led_driver_dev *pdriver_dev;
	pdriver_dev = container_of(inode->i_cdev, struct flash_led_driver_dev, cdev);
    struct flash_led_ctrl *pflash_led_dev = (struct flash_led_ctrl*)pdriver_dev->private;

    struct flash_led_dev *floodlight = &pflash_led_dev->floodlight;
    struct flash_led_dev *projection = &pflash_led_dev->projection;

    pflash_led_dev->enable = 0;

    if (projection->flash_led_func != NULL) {
        projection->flash_led_func->disable_channel(projection, 3);
    }

    if (floodlight->flash_led_func != NULL) {
        floodlight->flash_led_func->disable_channel(floodlight, 3);
    }

	return 0;
};

struct file_operations flash_led_fops = {
	.owner = THIS_MODULE,
	.open = flash_led_open,
	.release = flash_led_release,
	.unlocked_ioctl = flash_led_ioctl,
};

static int flash_led_of_parse(struct platform_device *pdev)
{
	int ret = 0, regulator_num = 0;
	struct device_node *np = pdev->dev.of_node;
	struct flash_led_driver_dev *pdriver_dev = platform_get_drvdata(pdev);
	struct flash_led_ctrl *pflash_led_dev = pdriver_dev->private;
    struct flash_led_dev *floodlight = &pflash_led_dev->floodlight;
    struct flash_led_dev *projection = &pflash_led_dev->projection;

	ret = of_property_read_string(np, "flash_led_name", &pflash_led_dev->flash_led_name);
	if (ret < 0) {
		pr_err("%s:property flash_led_name not defined for %s\n", __func__, pdev->name);
		return ret;
	}

    // the flash led touch pin used touch the io interrupt
	pflash_led_dev->touch_pin = of_get_named_gpio(np, "flash_led_touch_pin", 0);
	if (pflash_led_dev->touch_pin >= 0) {
		ret = devm_gpio_request(&pdev->dev, pflash_led_dev->touch_pin,
					"flash_led_touch_pin");
		if (ret < 0) {
			pr_err("%s:flash_led_touch request failed\n", __func__);
		}
	} else {
		pr_err("flash_led_touch not defined for %s\n", pflash_led_dev->flash_led_name);
	}

    pflash_led_dev->floodlight_en_pin = of_get_named_gpio(np, "floodlight_en_pin", 0);
	if (pflash_led_dev->floodlight_en_pin >= 0) {
		ret = devm_gpio_request(&pdev->dev, pflash_led_dev->floodlight_en_pin,
					"floodlight_en_pin");
		if (ret < 0) {
			pr_err("%s:floodlight_en_pin request failed\n", __func__);
		}
	} else {
        pflash_led_dev->floodlight_en_pin = -1;
		pr_err("floodlight_en_pin not defined for %s\n", pflash_led_dev->flash_led_name);
	}

    pflash_led_dev->projection_en_pin = of_get_named_gpio(np, "projection_en_pin", 0);
	if (pflash_led_dev->projection_en_pin >= 0) {
		ret = devm_gpio_request(&pdev->dev, pflash_led_dev->projection_en_pin,
					"projection_en_pin");
		if (ret < 0) {
			pr_err("%s:projection_en_pin request failed\n", __func__);
		}
	} else {
        pflash_led_dev->projection_en_pin = -1;
		pr_err("projection_en_pin not defined for %s\n", pflash_led_dev->flash_led_name);
	}

    floodlight->regulators.num = of_property_count_strings(np, "floodlight_regulators");
	if (floodlight->regulators.num <= 0) {
		pr_info("%s:property floodlight_regulators not defined for %s\n", __func__, pdev->name);
	} else {
		pr_info("%s num_of_regulators %d\n", __func__, floodlight->regulators.num);
		ret = of_property_read_string_array(np, "floodlight_regulators",
				floodlight->regulators.name, floodlight->regulators.num);
		if (ret != floodlight->regulators.num) {
			pr_err("%s:fail to read property floodlight_regulators\n", __func__);
			return -1;
		};

		regulator_num = floodlight->regulators.num;
		while (regulator_num) {
			floodlight->regulators.supply[regulator_num - 1] = devm_regulator_get(&pdev->dev,
						floodlight->regulators.name[regulator_num - 1]);
			if (IS_ERR(floodlight->regulators.supply[regulator_num - 1])) {
				pr_err("%s:fail to devm_regulator_get %s\n", __func__, floodlight->regulators.name[regulator_num - 1]);
				return -1;
			}
			regulator_num--;
		}
	}

    projection->regulators.num = of_property_count_strings(np, "projection_regulators");
	if (projection->regulators.num <= 0) {
		pr_info("%s:property projection_regulators not defined for %s\n", __func__, pdev->name);
	} else {
		pr_info("%s num_of_regulators %d\n", __func__, projection->regulators.num);
		ret = of_property_read_string_array(np, "projection_regulators",
				projection->regulators.name, projection->regulators.num);
		if (ret != projection->regulators.num) {
			pr_err("%s:fail to read property projection_regulators\n", __func__);
			return -1;
		};

		regulator_num = projection->regulators.num;
		while (regulator_num) {
			projection->regulators.supply[regulator_num - 1] = devm_regulator_get(&pdev->dev,
						projection->regulators.name[regulator_num - 1]);
			if (IS_ERR(projection->regulators.supply[regulator_num - 1])) {
				pr_err("%s:fail to devm_regulator_get %s\n", __func__, projection->regulators.name[regulator_num - 1]);
				return -1;
			}
			regulator_num--;
		}
	}

	ret = of_property_read_u8(np, "floodlight_i2c_bus",
				&floodlight->i2c_bus);
	if (ret != 0) {
		pr_err("fail to read property floodlight_i2c_bus\n");
		floodlight->i2c_bus = UNDEFINED_IN_DTS;
	}

    ret = of_property_read_u8(np, "projection_i2c_bus",
				&projection->i2c_bus);
	if (ret != 0) {
		pr_err("fail to read property projection_i2c_bus\n");
		projection->i2c_bus = UNDEFINED_IN_DTS;
	}

    pflash_led_dev->projection_adc = iio_channel_get(&pdev->dev, "projection_adc");
	if (IS_ERR(pflash_led_dev->projection_adc)) {
		pr_err("not define projection_adc adc\n");
	}

    pflash_led_dev->floodlight_adc = iio_channel_get(&pdev->dev, "floodlight_adc");
	if (IS_ERR(pflash_led_dev->floodlight_adc)) {
		pr_err("not define iio floodlight_adc\n");
	}

	return 0;
}

static volatile uint64_t falling_time_us[2] = {0};

uint64_t touch_pin_int_num(struct flash_led_ctrl *pflash_led_dev)
{
    return pflash_led_dev->frame_mark->frame_irq_cnt;
}

void frame_irq_cnt_clear(struct flash_led_ctrl *pflash_led_dev)
{
    pflash_led_dev->frame_mark->frame_irq_cnt = 0;
}

uint64_t touch_pin_high_time_us(void)
{
    return 0;
}

uint64_t touch_pin_falling_int(void)
{
    return 0;
}

uint64_t touch_pin_rising_int(void)
{
    return 0;
}

uint64_t touch_pin_preiod_time_us(void)
{

    if (falling_time_us[0] < falling_time_us[1]) {
        return falling_time_us[1] - falling_time_us[0];
    }

    return falling_time_us[0] - falling_time_us[1];
}

uint64_t get_us_time(void)
{
    struct timespec64 ts;
    static uint64_t us = 0;
    ktime_get_real_ts64(&ts);
    us = timespec64_to_ns(&ts) / 1000UL;
    return us;
}

static void flash_led_interrupt_func(struct work_struct *work)
{
    struct flash_led_ctrl *pflash_led_dev = container_of(work, struct flash_led_ctrl, flash_led_work);
    struct flash_led_dev *floodlight = &pflash_led_dev->floodlight;
    struct flash_led_dev *projection = &pflash_led_dev->projection;
    frame_mark_t *frame_mark = pflash_led_dev->frame_mark;

    frame_mark->frame_irq_cnt += 1;
    falling_time_us[frame_mark->frame_irq_cnt % 2] = get_us_time();
    frame_mark->frame_time_us = falling_time_us[frame_mark->frame_irq_cnt % 2];

    if (!pflash_led_dev->enable) {
        if (projection->flash_led_func != NULL) {
            projection->flash_led_func->disable_channel(projection, 3);
        }

        if (floodlight->flash_led_func != NULL) {
            floodlight->flash_led_func->disable_channel(floodlight, 3);
        }
        return;
    }

    if ((frame_mark->frame_irq_cnt % 2) == 0) {
        if ((pflash_led_dev->enable & FLOODLIGHT_EN) && floodlight->flash_led_func != NULL) {
            floodlight->flash_led_func->enable_channel(floodlight, 3);
        }

        if ((pflash_led_dev->enable & PROJECTION_EN) && projection->flash_led_func != NULL) {
            projection->flash_led_func->disable_channel(projection, 3);
        }
        if (!IS_ERR(pflash_led_dev->floodlight_adc)) {
            iio_read_channel_processed(pflash_led_dev->floodlight_adc,
                                           &frame_mark->floodlight_temperature);
            frame_mark->floodlight_temperature = get_ntc_temperature(frame_mark->floodlight_temperature);
        }

    } else {
        if ((pflash_led_dev->enable & PROJECTION_EN) && projection->flash_led_func != NULL) {
            projection->flash_led_func->enable_channel(projection, 3);
        }

        if ((pflash_led_dev->enable & FLOODLIGHT_EN) && floodlight->flash_led_func != NULL) {
            floodlight->flash_led_func->disable_channel(floodlight, 3);
        }

        if (!IS_ERR(pflash_led_dev->projection_adc)) {
            iio_read_channel_processed(pflash_led_dev->projection_adc,
                                       &frame_mark->projection_temperature);
            frame_mark->projection_temperature = get_ntc_temperature(frame_mark->projection_temperature);
        }
    }
}

static irqreturn_t touch_pin_isr(int irq, void *dev)
{
    struct flash_led_ctrl *pflash_led_dev = dev;

    schedule_work(&pflash_led_dev->flash_led_work);

    return IRQ_HANDLED;
}

static int flash_pin_init(struct flash_led_ctrl *dev)
{
    int irq = gpio_to_irq(dev->touch_pin);
    gpio_request(dev->touch_pin, "flash led touch pin");

    INIT_WORK(&dev->flash_led_work, flash_led_interrupt_func);

    request_irq(irq, touch_pin_isr, IRQF_TRIGGER_FALLING,
                "flash led touch pin",
                dev);

    if (dev->floodlight_en_pin != -1) {
        gpio_request(dev->floodlight_en_pin, "floodlight enable pin");
        if (gpio_is_valid(dev->floodlight_en_pin)) {
            gpio_direction_output(dev->floodlight_en_pin, 1);
        }
    }

    if (dev->projection_en_pin != -1) {
        gpio_request(dev->projection_en_pin, "projection enable pin");
        if (gpio_is_valid(dev->projection_en_pin)) {
            gpio_direction_output(dev->projection_en_pin, 1);
        }
    }

    return 0;
}

static int touch_pin_uinit(struct flash_led_ctrl *dev)
{
    int irq = gpio_to_irq(dev->touch_pin);
    dev->enable = 0;
    free_irq(irq, dev);
    cancel_work_sync(&dev->flash_led_work);
    gpio_free(dev->touch_pin);
    return 0;
}

static int flash_led_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct flash_led_driver_dev *pdriver_dev;
    struct flash_led_ctrl *pflash_led_dev;
	struct device_node *np = pdev->dev.of_node;

	pr_info("enter %s\n", __func__);

	if (pdev->id >= FLASH_LED_MAXCNT) {
		pr_err("%s:pdev id is %d error\n", __func__,pdev->id);
		return  -EINVAL;
	}

	pdev->id = of_alias_get_id(np, "flash_led");

	pdriver_dev = devm_kzalloc(&pdev->dev,sizeof(struct flash_led_driver_dev), GFP_KERNEL);
	if (pdriver_dev == NULL) {
		pr_err("%s:alloc struct flash_led_driver_dev error\n", __func__);
		return  -ENOMEM;
	}

	memset(pdriver_dev,0,sizeof(struct flash_led_driver_dev ));
	pr_info("%s:flash_led[%d]: pdriver_dev =0x%px\n", __func__, pdev->id, pdriver_dev);

	pflash_led_dev = devm_kzalloc(&pdev->dev,sizeof(*pflash_led_dev), GFP_KERNEL);
	if (pflash_led_dev == NULL) {
		pr_err("%s:alloc struct flash_led_dev error\n", __func__);
		return  -ENOMEM;
	}

	memset(pflash_led_dev,0,sizeof(*pflash_led_dev));
	pr_info("%s:flash_led[%d]: pflash_led_dev =0x%px\n", __func__,pdev->id,pflash_led_dev);
	pflash_led_dev->device_idx = pdev->id;

	pdriver_dev->private = pflash_led_dev;
	mutex_init(&pdriver_dev->vvmutex);

    frame_mark_t *frame_mark;
    frame_mark = dma_alloc_coherent(&pdev->dev, sizeof(frame_mark_t), &pflash_led_dev->frame_mark_info_addr, GFP_KERNEL);
    if (frame_mark == NULL ) {
        pr_err("dma_alloc_coherent error\n");
        return -1;
    }
    memset(frame_mark, 0, sizeof(frame_mark_t));
    pflash_led_dev->frame_mark = frame_mark;

	platform_set_drvdata(pdev, pdriver_dev);

	ret = flash_led_of_parse(pdev);
	if (ret < 0) {
		pr_err("%s:flash_led_of_parse error\n", __func__);
		return  ret;
	}

    ret = flash_pin_init(pflash_led_dev);
    if (ret != 0) {
		pr_err("%s:flash_pin_init error\n", __func__);
	}

	ret = flash_led_init(pflash_led_dev);
	if (ret != 0) {
		pr_err("%s:vvnative_flash_led_init error\n", __func__);
	}

	if (devise_register_index == 0) {
		if (flash_led_major == 0) {
			ret = alloc_chrdev_region(&pdriver_dev->devt, 0, FLASH_LED_MAXCNT, FLASH_LED_NAME);
			if (ret != 0) {
				pr_err("%s:alloc_chrdev_region error\n", __func__);
				return ret;
			}
			flash_led_major = MAJOR(pdriver_dev->devt);
			flash_led_minor = MINOR(pdriver_dev->devt);
		} else {
			pdriver_dev->devt = MKDEV(flash_led_major, flash_led_minor);
			ret = register_chrdev_region(pdriver_dev->devt, FLASH_LED_MAXCNT, FLASH_LED_NAME);
			if (ret) {
				pr_err("%s:register_chrdev_region error\n", __func__);
				return ret;
			}
		}

		flash_led_class = class_create(THIS_MODULE, FLASH_LED_NAME);
		if (IS_ERR(flash_led_class)) {
			pr_err("%s[%d]:class_create error!\n", __func__, __LINE__);
			return -EINVAL;
		}
	}

	pdriver_dev->devt = MKDEV(flash_led_major, flash_led_minor + pdev->id);

	cdev_init(&pdriver_dev->cdev, &flash_led_fops);
	ret = cdev_add(&pdriver_dev->cdev, pdriver_dev->devt, 1);
	if (ret) {
		pr_err("%s[%d]:cdev_add error!\n", __func__, __LINE__);
		return ret;
	}

	pdriver_dev->class = flash_led_class;
	device_create(pdriver_dev->class, NULL, pdriver_dev->devt,
			pdriver_dev, "%s%d", FLASH_LED_NAME, pdev->id);

    extern int flash_led_create_capabilities_sysfs(struct platform_device *pdev);
    flash_led_create_capabilities_sysfs(pdev);
	devise_register_index++;
	pr_info("exit %s\n", __func__);
	return ret;
}

static int flash_led_remove(struct platform_device *pdev)
{
	struct flash_led_driver_dev *pdriver_dev;
	struct flash_led_ctrl *pflash_led_dev;

	pr_info("enter %s\n", __func__);
	devise_register_index--;
	pdriver_dev = platform_get_drvdata(pdev);

	if (pdriver_dev == NULL) {
		pr_err("%s:file private is null point error\n", __func__);
		return  -ENOMEM;
	}

	pflash_led_dev = pdriver_dev->private;
	flash_led_deinit(pflash_led_dev);
    touch_pin_uinit(pflash_led_dev);

    dma_free_coherent(&pdev->dev, sizeof(*pflash_led_dev->frame_mark), pflash_led_dev->frame_mark, pflash_led_dev->frame_mark_info_addr);

    if (!IS_ERR(pflash_led_dev->projection_adc)) {
        iio_channel_release(pflash_led_dev->projection_adc);
    }

    if (!IS_ERR(pflash_led_dev->floodlight_adc)) {
        iio_channel_release(pflash_led_dev->floodlight_adc);
    }

    extern int flash_led_remove_capabilities_sysfs(struct platform_device *pdev);
    flash_led_remove_capabilities_sysfs(pdev);

	cdev_del(&pdriver_dev->cdev);
	device_destroy(pdriver_dev->class, pdriver_dev->devt);
	unregister_chrdev_region(pdriver_dev->devt, FLASH_LED_MAXCNT);
	if (devise_register_index == 0) {
		class_destroy(pdriver_dev->class);
	}

    devm_kfree(&pdev->dev, pdriver_dev);
    devm_kfree(&pdev->dev, pflash_led_dev);

	pr_info("exit %s\n", __func__);

	return 0;
}

static const struct of_device_id flash_led_of_match[] = {
	{.compatible = "thead,light-vvcam-flash_led"},
};

static struct platform_driver flash_led_driver = {
	.probe		= flash_led_probe,
	.remove		= flash_led_remove,
	.driver = {
		.name  = FLASH_LED_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(flash_led_of_match),
	}
};

/*
static void flash_led_pdev_release(struct device *dev)
{
	pr_info("enter %s\n", __func__);
}
*/

static int __init flash_led_init_module(void)
{
	int ret = 0;

	pr_info("enter %s\n", __func__);

	ret = platform_driver_register(&flash_led_driver);
	if (ret) {
		pr_err("register platform driver failed.\n");
		return ret;
	}

	return ret;
}

static void __exit flash_led_exit_module(void)
{
	pr_info("enter %s\n", __func__);

	platform_driver_unregister(&flash_led_driver);
}

module_init(flash_led_init_module);
module_exit(flash_led_exit_module);

MODULE_DESCRIPTION("FLASH_LED");
MODULE_LICENSE("GPL");
