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
#include "vvsensor.h"
#include "sensor_common.h"

extern struct vvcam_sensor_function_s SENSR0_FUNCTION;
extern struct vvcam_sensor_function_s SENSR1_FUNCTION;

#define check_retval(x)\
	do {\
		if ((x))\
			return -EIO;\
	} while (0)

static int32_t vvcam_sensor_sccb_config(struct vvcam_sensor_dev *dev, struct vvcam_sccb_cfg_s *sccb_config)
{
	dev->sensor_sccb_cfg.slave_addr = sccb_config->slave_addr;
	dev->sensor_sccb_cfg.addr_byte  = sccb_config->addr_byte;
	dev->sensor_sccb_cfg.data_byte  = sccb_config->data_byte;
	return 0;
}
static int32_t vvcam_focus_sccb_config(struct vvcam_sensor_dev *dev, struct vvcam_sccb_cfg_s *sccb_config)
{
	dev->sensor_sccb_cfg.slave_addr = sccb_config->slave_addr;
	dev->sensor_sccb_cfg.addr_byte  = sccb_config->addr_byte;
	dev->sensor_sccb_cfg.data_byte  = sccb_config->data_byte;
	return 0;
}

static int vvcam_i2c_write_reg(struct i2c_client *client,unsigned int slave_address,
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
	//pr_info("-->%s: slave_address[0x%x] addr[0x%04x] = 0x%04x addr_byte[%d] data_byte[%d]!\n", __func__,slave_address,reg_addr,data,reg_length,data_length);
	return 0;
}

static int vvcam_i2c_read_reg(struct i2c_client *client,unsigned int slave_address,
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
	if (ret != 2)
	{
		return -1;
	}

	for (i=0; i < data_length; i++)
	{
		pdata[i] = readbuf[data_length -1 - i];
	}

	return 0;
}

int32_t vvcam_sensor_i2c_write(struct vvcam_sensor_dev *dev, uint32_t address, uint32_t data)
{
	int32_t ret = 0;
	struct vvcam_sccb_data;

	if((NULL == dev))
	{
		return -1;
	}

	ret = vvcam_i2c_write_reg(dev->i2c_client,dev->sensor_sccb_cfg.slave_addr,
	                         address,dev->sensor_sccb_cfg.addr_byte,
	                         data,dev->sensor_sccb_cfg.data_byte);

	return ret;
}

int32_t vvcam_sensor_i2c_read(struct vvcam_sensor_dev *dev, uint32_t address, uint32_t *pdata)
{
	int32_t ret = 0;
	struct vvcam_sccb_data;

	if((NULL == dev))
	{
		return -1;
	}

	ret = vvcam_i2c_read_reg(dev->i2c_client,dev->sensor_sccb_cfg.slave_addr,
	                         address, dev->sensor_sccb_cfg.addr_byte,
	                         (unsigned char *)pdata, dev->sensor_sccb_cfg.data_byte);

	return ret;
}

static int32_t vvcam_sensor_i2c_write_array(struct vvcam_sensor_dev *dev, void __user *args)
{
	int ret = 0;
	int index =0;
	struct vvcam_sccb_array array;
	struct vvcam_sccb_data sccb_data;

	if((NULL == dev)||(NULL == args))
	{
		return -1;
	}

	copy_from_user(&array, args, sizeof(struct vvcam_sccb_array));

	for (index = 0; index < array.count; index++)
	{
		copy_from_user(&sccb_data, &(array.sccb_data[index]), sizeof(struct vvcam_sccb_data));
		ret = vvcam_sensor_i2c_write(dev, sccb_data.addr, sccb_data.data);
		if (ret != 0)
		{
#if 1
            int i = 10;
            while(i > 0) {
		        ret = vvcam_sensor_i2c_write(dev, sccb_data.addr, sccb_data.data);
                if(ret == 0) {
                    break;
                }
                mdelay(10);
                i--;
            }
#endif

            if (ret != 0) {
                printk("<0>""!!!!!!!write array error, sccb_data.addr: 0x%x, sccb_data.data: 0x%x\n", sccb_data.addr, sccb_data.data);
			    return ret;
            }
		}
	}

	return 0;
}

static int vvcam_sensor_i2c_read_array(struct vvcam_sensor_dev *dev, void __user *args)
{
	int ret = 0;
	int index =0;
	struct vvcam_sccb_array array;
	struct vvcam_sccb_data sccb_data;

	if((NULL == dev)||(NULL == args))
	{
		return -1;
	}

	copy_from_user(&array, args, sizeof(struct vvcam_sccb_array));

	for (index = 0; index < array.count; index++)
	{
		copy_from_user(&sccb_data,  &(array.sccb_data[index]), sizeof(struct vvcam_sccb_data));
		ret = vvcam_sensor_i2c_read(dev, sccb_data.addr, &sccb_data.data);
		if (ret != 0)
		{
			return ret;
		}
		copy_to_user(&(array.sccb_data[index]), &sccb_data, sizeof(struct vvcam_sccb_data));
	}

	return 0;

}

static int32_t vvcam_focus_i2c_write(struct vvcam_sensor_dev *dev, uint32_t address, uint32_t data)
{
	int32_t ret = 0;
	struct vvcam_sccb_data;

	if((NULL == dev))
	{
		return -1;
	}

	ret = vvcam_i2c_write_reg(dev->i2c_client,dev->focus_sccb_cfg.slave_addr,
	                         address,dev->focus_sccb_cfg.addr_byte,
	                         data,dev->focus_sccb_cfg.data_byte);

	return ret;
}

static int32_t vvcam_focus_i2c_read(struct vvcam_sensor_dev *dev, uint32_t address, uint32_t *pdata)
{
	int32_t ret = 0;
	struct vvcam_sccb_data;

	if((NULL == dev))
	{
		return -1;
	}

	ret = vvcam_i2c_read_reg(dev->i2c_client,dev->focus_sccb_cfg.slave_addr,
	                         address,dev->focus_sccb_cfg.addr_byte,
	                         (unsigned char *)pdata,dev->focus_sccb_cfg.data_byte);

	return ret;
}

long sensor_priv_ioctl(struct vvcam_sensor_dev *dev, unsigned int cmd, void __user *args)
{
	int ret = -1;

	if (!dev)
	{
		pr_err("-->%s: null point!\n", __func__);
		return ret;
	}

	//printk("-->%s: cmd = %d!\n", __func__,cmd);
	switch (cmd)
	{
		case VVSENSORIOC_RESET:
		{
			ret = sensor_reset(dev);
			break;
		}

		case VVSENSORIOC_S_CLK:
		{
			uint32_t clk;
			check_retval(copy_from_user(&clk, args, sizeof(clk)));
			ret = sensor_set_clk(dev, clk);
			break;
		}

		case VVSENSORIOC_G_CLK:
		{
			uint32_t clk;
			ret = sensor_get_clk(dev, &clk);
			check_retval(copy_to_user(args, &clk, sizeof(clk)));
			break;
		}

		case VVSENSORIOC_S_POWER:
		{
			uint32_t power;
			check_retval(copy_from_user(&power, args, sizeof(power)));
			ret = sensor_set_power(dev, power);
			break;
		}

		case VVSENSORIOC_G_POWER:
		{
			uint32_t power;
			ret = sensor_get_power(dev, &power);
			check_retval(copy_to_user(args, &power, sizeof(power)));
			break;
		}

		case VVSENSORIOC_SENSOR_SCCB_CFG:
		{
			struct vvcam_sccb_cfg_s sccb_config;
			check_retval(copy_from_user(&sccb_config, args, sizeof(sccb_config)));
			ret = vvcam_sensor_sccb_config(dev,&sccb_config);
			break;
		}

		case VVSENSORIOC_FOCUS_SCCB_CFG:
		{
			struct vvcam_sccb_cfg_s sccb_config;
			check_retval(copy_from_user(&sccb_config, args, sizeof(sccb_config)));
			ret = vvcam_focus_sccb_config(dev,&sccb_config);
			break;
		}

		case VVSENSORIOC_WRITE_REG:
		{
			struct vvcam_sccb_data sccb_data;
			check_retval(copy_from_user(&sccb_data, args, sizeof(sccb_data)));
			ret = vvcam_sensor_i2c_write(dev, sccb_data.addr, sccb_data.data);
			break;
		}

		case VVSENSORIOC_READ_REG:
		{
			struct vvcam_sccb_data sccb_data;
			check_retval(copy_from_user(&sccb_data, args, sizeof(sccb_data)));
			ret = vvcam_sensor_i2c_read(dev, sccb_data.addr, &sccb_data.data);
			check_retval(copy_to_user(args, &sccb_data, sizeof(sccb_data)));
			break;
		}

		case VVSENSORIOC_WRITE_ARRAY:
		{
			ret = vvcam_sensor_i2c_write_array(dev, args);
			break;
		}

		case VVSENSORIOC_READ_ARRAY:
		{
			ret = vvcam_sensor_i2c_read_array(dev, args);
			break;
		}

		case VVSENSORIOC_AF_WRITE_REG:
		{
			struct vvcam_sccb_data sccb_data;
			check_retval(copy_from_user(&sccb_data, args, sizeof(sccb_data)));
			ret = vvcam_focus_i2c_write(dev, sccb_data.addr, sccb_data.data);
			break;
		}

		case VVSENSORIOC_AF_READ_REG:
		{
			struct vvcam_sccb_data sccb_data;
			check_retval(copy_from_user(&sccb_data, args, sizeof(sccb_data)));
			ret = vvcam_focus_i2c_read(dev, sccb_data.addr, &sccb_data.data);
			check_retval(copy_to_user(args, &sccb_data, sizeof(sccb_data)));
			break;
		}

		case VVSENSORIOC_G_MIPI:
		{
			ret = 0;
			dev->sensor_func.mipi_info.sensor_data_bit = dev->sensor_mode.bit_width;
			check_retval(copy_to_user(args,&(dev->sensor_func.mipi_info),sizeof(struct sensor_mipi_info)));
			break;
		}

		case VVSENSORIOC_G_NAME:
		{
			ret = 0;
			check_retval(copy_to_user(args,dev->sensor_name,strlen(dev->sensor_name) + 1));
			break;
		}

		case VVSENSORIOC_G_RESERVE_ID:
		{
			ret = 0;
			check_retval(copy_to_user(args,&(dev->sensor_func.reserve_id),sizeof(uint32_t)));
			break;
		}

		case VVSENSORIOC_G_CHIP_ID:
		{
			uint32_t chip_id = 0;
			if (dev->sensor_func.sensor_get_chip_id == NULL)
			{
				return -1;
			}
			ret = dev->sensor_func.sensor_get_chip_id(dev,&chip_id);
			check_retval(copy_to_user(args, &chip_id, sizeof(chip_id)));
			break;
		}

		case VVSENSORIOC_S_INIT:
		{

			struct vvcam_mode_info sensor_mode;
			check_retval(copy_from_user(&sensor_mode, args, sizeof(struct vvcam_mode_info)));
			if (dev->sensor_func.sensor_init == NULL)
			{
				return -1;
			}

			ret = dev->sensor_func.sensor_init(dev,&sensor_mode);
			break;
		}

		case VVSENSORIOC_S_STREAM:
		{
			uint32_t stream_status;
			check_retval(copy_from_user(&stream_status, args, sizeof(stream_status)));
			if (dev->sensor_func.sensor_set_stream == NULL)
			{
				return -1;
			}
			ret = dev->sensor_func.sensor_set_stream(dev, stream_status);
			break;
		}

		case VVSENSORIOC_S_EXP:
		{
			uint32_t exp_line;
			check_retval(copy_from_user(&exp_line, args, sizeof(exp_line)));

			if (dev->sensor_func.sensor_set_exp == NULL)
			{
				return -1;
			}

			ret = dev->sensor_func.sensor_set_exp(dev,exp_line);

			break;
		}

		case VVSENSORIOC_S_VSEXP:
		{
			uint32_t exp_line;
			check_retval(copy_from_user(&exp_line, args, sizeof(exp_line)));

			if (dev->sensor_func.sensor_set_vs_exp == NULL)
			{
				return -1;
			}

			ret = dev->sensor_func.sensor_set_vs_exp(dev,exp_line);

			break;
		}

		case VVSENSORIOC_S_GAIN:
		{
			uint32_t gain;
			check_retval(copy_from_user(&gain, args, sizeof(gain)));

			if (dev->sensor_func.sensor_set_gain == NULL)
			{
				return -1;
			}

			ret = dev->sensor_func.sensor_set_gain(dev, gain);

			break;
		}

		case VVSENSORIOC_S_VSGAIN:
		{
			uint32_t gain;
			check_retval(copy_from_user(&gain, args, sizeof(gain)));

			if (dev->sensor_func.sensor_set_vs_gain == NULL)
			{
				return -1;
			}

			ret = dev->sensor_func.sensor_set_vs_gain(dev, gain);
			break;
		}

		case VVSENSORIOC_S_FPS:
		{
			uint32_t fps;
			check_retval(copy_from_user(&fps, args, sizeof(fps)));

			if (dev->sensor_func.sensor_set_fps == NULL)
			{
				return -1;
			}

			ret = dev->sensor_func.sensor_set_fps(dev,fps);

			break;
		}

		case VVSENSORIOC_G_FPS:
		{
			ret = 0;
			check_retval(copy_to_user(args, &(dev->ae_info.cur_fps),sizeof(uint32_t)));
			break;
		}

		case VVSENSORIOC_S_FRAMESIZE:
		{
			ret = 0;
			break;
		}

		case VVSENSORIOC_ENUM_FRAMESIZES:
		{
			ret = 0;
			break;
		}

		case VVSENSORIOC_S_HDR_MODE:
		{
			uint32_t hdr_mode;
			check_retval(copy_from_user(&hdr_mode, args, sizeof(hdr_mode)));

			if (dev->sensor_func.sensor_set_hdr_mode == NULL)
			{
				return -1;
			}
			ret = dev->sensor_func.sensor_set_hdr_mode(dev, hdr_mode);

			break;
		}

		case VVSENSORIOC_G_HDR_MODE:
		{
			ret = 0;
			check_retval(copy_to_user(args,&(dev->sensor_mode.hdr_mode),sizeof(uint32_t)));
			break;
		}

		case VVSENSORIOC_S_HDR_RADIO:
		{
			ret = 0;
			check_retval(copy_from_user(&(dev->ae_info.hdr_radio),args,sizeof(uint32_t)));
			break;

		}

		case VVSENSORIOC_G_AE_INFO:
		{
			ret = 0;
			check_retval(copy_to_user(args,&(dev->ae_info),sizeof(struct vvcam_ae_info_s)));
			break;
		}

		case VVSENSORIOC_QUERY:
		{
			struct vvcam_mode_info_array sensor_mode_info_arry;
			memset(&sensor_mode_info_arry,0,sizeof(sensor_mode_info_arry));

			if (dev->sensor_func.sensor_query == NULL)
			{
				return -1;
			}
			ret = dev->sensor_func.sensor_query(dev, &sensor_mode_info_arry);
			if (ret == 0)
			{
			check_retval(copy_to_user(args, &sensor_mode_info_arry, sizeof(sensor_mode_info_arry)));
			}

			break;
		}

		case VVSENSORIOC_G_SENSOR_MODE:
		{
			check_retval(copy_to_user(args, &(dev->sensor_mode), sizeof(struct vvcam_mode_info)));
			break;
		}

		default:
		{
			pr_err("unsupported command %d\n", cmd);
			break;
		}

	}

	return ret;
}

static int vvcam_register_i2c_client(struct vvcam_sensor_dev *dev)
{
	struct i2c_adapter *adap;
    struct i2c_board_info sensor_i2c_info = {
        .type = "sensor",
        .addr = dev->device_idx + 1
    };

	adap = i2c_get_adapter(dev->i2c_bus);
	if (adap == NULL)
	{
		pr_err("[%s]:i2c_get_adapter i2c_bus %d failed\n",__func__,dev->i2c_bus);
		return -1;
	}

    if (dev->i2c_bus != UNDEFINED_IN_DTS) {
        strscpy(sensor_i2c_info.type, dev->sensor_name, I2C_NAME_SIZE);
    }

    dev->i2c_client = i2c_new_client_device(adap, &sensor_i2c_info);

	i2c_put_adapter(adap);

	if (dev->i2c_client == NULL)
	{
		pr_err("[%s]:i2c_new_client_device i2c_bus %d failed\n",__func__,dev->i2c_bus);
		return -1;
	}
	return 0;
}

static void vvcam_unregister_i2c_client(struct vvcam_sensor_dev *dev)
{
	i2c_unregister_device(dev->i2c_client);
}

int vvnative_sensor_init(struct vvcam_sensor_dev *dev)
{
	int ret = 0;

    if (dev->i2c_bus == UNDEFINED_IN_DTS) {
        dev->sensor_sccb_cfg.addr_byte  = 2;
        dev->sensor_sccb_cfg.data_byte  = 1;
    }

	ret = vvcam_register_i2c_client(dev);
	if (ret != 0) {
		pr_err("[%s]: vvcam_register_i2c_client sensor_idx = %d failed\n",__func__, dev->device_idx);
		return -1;
	}

	return ret;
}

int vvnative_sensor_deinit(struct vvcam_sensor_dev *dev)
{
	int ret = 0;

 	vvcam_unregister_i2c_client(dev);

	return ret;
}
