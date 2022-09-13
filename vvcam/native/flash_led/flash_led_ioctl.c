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

#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
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
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include "flash_led_ioctl.h"
#include "flash_led_driver.h"

void frame_irq_cnt_clear(struct flash_led_ctrl *pflash_led_dev);
extern struct flash_led_function_s aw36515_function;
extern struct flash_led_function_s aw36413_function;

static struct flash_led_function_s *floodlight_func = &aw36413_function;
static struct flash_led_function_s *projection_func = &aw36515_function;

#define check_retval(x)\
	do {\
		if ((x))\
			return -EIO;\
	} while (0)

/*
static int32_t flash_led_sccb_config(struct flash_led_dev *dev, struct flash_led_sccb_cfg_s *sccb_config)
{
	dev->flash_led_sccb_cfg.slave_addr = sccb_config->slave_addr;
	dev->flash_led_sccb_cfg.addr_byte  = sccb_config->addr_byte;
	dev->flash_led_sccb_cfg.data_byte  = sccb_config->data_byte;
	return 0;
}
*/

static int i2c_write_reg(struct i2c_client *client,unsigned int slave_address,
						unsigned int reg_addr,unsigned int reg_length,
						unsigned int data, unsigned int data_length)
{
	int ret;
	unsigned int i;
	struct i2c_msg msgs[2];
	unsigned char sendbuf[16];
	unsigned int send_len    = 0;

	if (client == NULL)
		return -1;
	memset(msgs,0,sizeof(msgs));
	memset(sendbuf,0,sizeof(sendbuf));

	for (i=0; i < reg_length; i++)
	{
		sendbuf[send_len++] = (reg_addr >> ((reg_length -1 - i)<<3)) & 0xff;
	}
	for (i=0; i < data_length; i++)
	{
		sendbuf[send_len++] = (data >> ((data_length -1 - i)<<3)) & 0xff;
	}

	msgs[0].addr  = slave_address;
	msgs[0].flags = client->flags & I2C_M_TEN;
	msgs[0].len   = send_len;
	msgs[0].buf   = sendbuf;

	ret = i2c_transfer(client->adapter, msgs, 1);
	if (ret != 1)
	{
		return -1;
	}

	return 0;
}

static int i2c_read_reg(struct i2c_client *client,unsigned int slave_address,
						unsigned int reg_addr,unsigned int reg_length,
						unsigned char * pdata, unsigned int data_length)
{
	int ret;
	unsigned int i;
	struct i2c_msg msgs[2];
	unsigned char sendbuf[16];
	unsigned int send_len = 0;
	unsigned char readbuf[16];

	if (client == NULL || pdata == NULL)
		return -1;

	memset(msgs,0,sizeof(msgs));
	memset(sendbuf,0,sizeof(sendbuf));
	memset(readbuf,0,sizeof(readbuf));

	for (i=0; i < reg_length; i++)
	{
		sendbuf[send_len++] = (reg_addr >> ((reg_length -1 - i)<<3)) & 0xff;
	}

	msgs[0].addr  = slave_address;
	msgs[0].flags = client->flags & I2C_M_TEN;
	msgs[0].len   = send_len;
	msgs[0].buf   = sendbuf;

	msgs[1].addr  = slave_address;
	msgs[1].flags = client->flags & I2C_M_TEN;
	msgs[1].flags |= I2C_M_RD;
	msgs[1].len   = data_length;
	msgs[1].buf   = readbuf;

	ret = i2c_transfer(client->adapter, msgs, 2);
	if (ret != 2) {
		return -1;
	}

	for (i = 0; i < data_length; i++) {
		pdata[i] = readbuf[data_length -1 - i];
	}

	return 0;
}

int32_t flash_led_i2c_write(struct flash_led_dev *dev, uint32_t address, uint32_t data)
{
	int32_t ret = 0;

	if((NULL == dev))
	{
		return -1;
	}

	ret = i2c_write_reg(dev->i2c_client,dev->flash_led_sccb_cfg.slave_addr,
	                         address,dev->flash_led_sccb_cfg.addr_byte,
	                         data,dev->flash_led_sccb_cfg.data_byte);

	return ret;
}

int32_t flash_led_i2c_read(struct flash_led_dev *dev, uint32_t address, uint32_t *pdata)
{
	int32_t ret = 0;

	if((NULL == dev))
	{
		return -1;
	}

	ret = i2c_read_reg(dev->i2c_client,dev->flash_led_sccb_cfg.slave_addr,
	                         address, dev->flash_led_sccb_cfg.addr_byte,
	                         (unsigned char *)pdata, dev->flash_led_sccb_cfg.data_byte);

	return ret;
}

long flash_led_priv_ioctl(struct flash_led_ctrl *dev, unsigned int cmd, void __user *args)
{
	int ret = -1;
    flash_bright_cfg_t bright;
    flash_led_reg_t reg;
    flash_ch_t ch;
    flash_mode_t mode;
    flash_led_enable_mask_t flash_led;

    struct flash_led_dev *floodlight = &dev->floodlight;
    struct flash_led_dev *projection = &dev->projection;

	if (!dev) {
		pr_err("-->%s: null point!\n", __func__);
		return ret;
	}

    switch (cmd) {
        case FLASH_LED_IOCTL_ENABLE_CH:
            check_retval(copy_from_user(&ch, args, sizeof(ch)));
            if (ch.type == FLOODLIGHT && floodlight->flash_led_func != NULL) {
                ret = floodlight->flash_led_func->enable_channel(floodlight, ch.channel);
            } else if (projection->flash_led_func != NULL) {
                ret = projection->flash_led_func->enable_channel(projection, ch.channel);
            }
            break;
        case FLASH_LED_IOCTL_DISABLE_CH:
            check_retval(copy_from_user(&ch, args, sizeof(ch)));
            if (ch.type == FLOODLIGHT && floodlight->flash_led_func != NULL) {
                ret = floodlight->flash_led_func->disable_channel(floodlight, ch.channel);
            } else if (projection->flash_led_func != NULL) {
                ret = projection->flash_led_func->disable_channel(projection, ch.channel);
            }
            break;
        case FLASH_LED_IOCTL_SET_MODE:
            check_retval(copy_from_user(&mode, args, sizeof(mode)));
            if (mode.type == FLOODLIGHT && floodlight->flash_led_func != NULL) {
                ret = floodlight->flash_led_func->set_mode(floodlight, mode.mode);
            } else if (projection->flash_led_func != NULL) {
                ret = projection->flash_led_func->set_mode(projection, mode.mode);
            }
            break;
        case FLASH_LED_IOCTL_SET_FLASH_BRIGHT:
            check_retval(copy_from_user(&bright, args, sizeof(bright)));
            if (bright.type == FLOODLIGHT && floodlight->flash_led_func != NULL) {
                ret = floodlight->flash_led_func->set_flash_brightness(floodlight, bright.channel, bright.value);
            } else if (projection->flash_led_func != NULL) {
                ret = projection->flash_led_func->set_flash_brightness(projection, bright.channel, bright.value);
            }
            break;
        case FLASH_LED_IOCTL_SET_TORCH_BRIGHT:
            check_retval(copy_from_user(&bright, args, sizeof(bright)));
            if (bright.type == FLOODLIGHT && floodlight->flash_led_func != NULL) {
                ret = floodlight->flash_led_func->set_torch_brightness(floodlight, bright.channel, bright.value);
            } else if (projection->flash_led_func != NULL) {
                ret = projection->flash_led_func->set_torch_brightness(projection, bright.channel, bright.value);
            }
            break;
        case FLASH_LED_IOCTL_WRITE_REG:
            check_retval(copy_from_user(&reg, args, sizeof(reg)));
            if (reg.type == FLOODLIGHT && floodlight->i2c_bus != UNDEFINED_IN_DTS) {
                ret = flash_led_i2c_write(floodlight, reg.offset, reg.value);
            } else if(projection->i2c_bus != UNDEFINED_IN_DTS) {
                ret = flash_led_i2c_write(projection, reg.offset, reg.value);
            }
            break;
        case FLASH_LED_IOCTL_READ_REG:
            check_retval(copy_from_user(&reg, args, sizeof(reg)));
            if (reg.type == FLOODLIGHT && floodlight->i2c_bus != UNDEFINED_IN_DTS) {
                ret = flash_led_i2c_read(floodlight, reg.offset, &reg.value);
            } else if(projection->i2c_bus != UNDEFINED_IN_DTS) {
                ret = flash_led_i2c_read(projection, reg.offset, &reg.value);
            }
            check_retval(copy_to_user(args, &reg, sizeof(reg)));
            break;
        case FLASH_LED_IOCTL_ENABLE:
            check_retval(copy_from_user(&flash_led, args, sizeof(flash_led)));
            frame_irq_cnt_clear(dev);
            dev->enable |= flash_led;
            ret = 0;
            break;
        case FLASH_LED_IOCTL_DISABLE:
            check_retval(copy_from_user(&flash_led, args, sizeof(flash_led)));
            dev->enable &= ~flash_led;
            if (flash_led & FLOODLIGHT_EN && floodlight->flash_led_func != NULL) {
                floodlight->flash_led_func->disable_channel(floodlight, 3);
            }

            if (flash_led & PROJECTION_EN && projection->flash_led_func != NULL) {
                projection->flash_led_func->disable_channel(projection, 3);
            }
            ret = 0;
            break;
        case FLASH_LED_IOCTL_GET_FRAME_MASK_INFO_ADDR: {
            unsigned long addr;
            addr = dev->frame_mark_info_addr;
            pr_info("FLASH_LED_IOCTL_GET_FRAME_MASK_INFO_ADDR 0x%lx\n", addr);
            check_retval(copy_to_user(args, &addr, sizeof(addr)));
            ret = 0;
        }
            break;
        default:
            printk("%s, %d, flash led cmd error, cmd %u\n", __func__, __LINE__, cmd);
            return -ENOTTY;
    }

	return ret;
}

static int register_i2c_client(struct flash_led_dev *dev)
{
	struct i2c_adapter *adap;
    struct i2c_board_info flash_led_i2c_info = {
        .type = "flash_led",
        .addr = dev->flash_led_sccb_cfg.slave_addr,
    };

    if (dev->i2c_bus == UNDEFINED_IN_DTS) {
        return 0;
    }

    flash_led_i2c_info.addr &= 0xff;

	adap = i2c_get_adapter(dev->i2c_bus);
	if (adap == NULL)
	{
		pr_err("[%s]:i2c_get_adapter i2c_bus %d failed\n",__func__,dev->i2c_bus);
		return -1;
	}

    strscpy(flash_led_i2c_info.type, dev->flash_led_func->flash_led_name, I2C_NAME_SIZE);

    dev->i2c_client = i2c_new_client_device(adap, &flash_led_i2c_info);

	i2c_put_adapter(adap);

	if (dev->i2c_client == NULL) {
		pr_err("[%s]:i2c_new_client_device i2c_bus %d failed\n",__func__,dev->i2c_bus);
		return -1;
	}
	return 0;
}

static void unregister_i2c_client(struct flash_led_dev *dev)
{
	i2c_unregister_device(dev->i2c_client);
}

int flash_led_init(struct flash_led_ctrl *dev)
{
	int ret = 0;
    struct flash_led_dev *floodlight = &dev->floodlight;
    struct flash_led_dev *projection = &dev->projection;

    floodlight->flash_led_sccb_cfg.slave_addr = dev->device_idx+1;
    projection->flash_led_sccb_cfg.slave_addr = dev->device_idx+2;

    if (floodlight->i2c_bus == UNDEFINED_IN_DTS && projection->i2c_bus == UNDEFINED_IN_DTS) {
		pr_err("%s, %d, flash_led i2c bus invalid\n",__func__, __LINE__);
        return -1;
    }

    floodlight->flash_led_func = NULL;
    projection->flash_led_func = NULL;

    if (floodlight->i2c_bus != UNDEFINED_IN_DTS) {
        floodlight->flash_led_func = floodlight_func;
    }

    if (projection->i2c_bus != UNDEFINED_IN_DTS) {
        projection->flash_led_func = projection_func;
    }

    if (floodlight->i2c_bus == projection->i2c_bus) {
	    pr_info("%s, %d, floodlight->i2c_bus = %d projection->i2c_bus %d\n", __func__, __LINE__, floodlight->i2c_bus, projection->i2c_bus);
        ret = register_i2c_client(floodlight);
	    if (ret != 0) {
		    pr_err("[%s]: register_i2c_client flash_led_idx = %d failed\n",__func__, dev->device_idx);
		    return -1;
	    }

        projection->i2c_client = floodlight->i2c_client;
    } else {
        if (floodlight->i2c_bus != UNDEFINED_IN_DTS) {
	        pr_info("%s, %d, floodlight->i2c_bus = %d \n", __func__, __LINE__, floodlight->i2c_bus);
            ret = register_i2c_client(floodlight);
	        if (ret != 0) {
		        pr_err("[%s]: floodlight register_i2c_client flash_led_idx = %d failed\n",__func__, dev->device_idx);
		        return -1;
	        }
        }

        if (projection->i2c_bus != UNDEFINED_IN_DTS) {
	        pr_info("%s, %d, projection->i2c_bus %d\n", __func__, __LINE__, projection->i2c_bus);
            ret = register_i2c_client(projection);
	        if (ret != 0) {
		        pr_err("[%s]: projection register_i2c_client flash_led_idx = %d failed\n",__func__, dev->device_idx);
		        return -1;
	        }
        }
    }

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

    printk("%s,%d, exit\n", __func__, __LINE__);

	return ret;
}

int flash_led_deinit(struct flash_led_ctrl *dev)
{
	int ret = 0, err = 0;
    struct flash_led_dev *floodlight = &dev->floodlight;
    struct flash_led_dev *projection = &dev->projection;

    if (floodlight->flash_led_func != NULL) {
        ret = floodlight->flash_led_func->uninit(floodlight);
        if (ret != 0) {
            err |= ret;
        }
    }

    if (projection->flash_led_func != NULL) {
        ret = projection->flash_led_func->uninit(projection);
        if (ret != 0) {
            err |= ret;
        }
    }

    if (floodlight->i2c_bus == projection->i2c_bus) {
 	    unregister_i2c_client(floodlight);
    } else {
        if (floodlight->i2c_bus != UNDEFINED_IN_DTS) {
 	        unregister_i2c_client(floodlight);
        }

        if (projection->i2c_bus != UNDEFINED_IN_DTS) {
 	        unregister_i2c_client(projection);
        }
    }

	return err;
}

