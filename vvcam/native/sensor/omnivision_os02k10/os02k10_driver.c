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
#include "vvsensor.h"
#include "sensor_common.h"
#include "os02k10_reg_cfg.h"

#define SENSOR_CLK 63000000

static struct vvcam_mode_info pos02k10_mode_info[] = {
	{
		.index     = 0,
		.width     = 1920,
		.height    = 1080,
		.fps       = 30,
		.hdr_mode  = SENSOR_MODE_LINEAR,
		.bit_width = 10,
		.bayer_pattern = BAYER_GBRG,
	},
	{
		.index     = 1,
		.width    = 1920,
		.height   = 1080,
		.fps      = 30,
		.hdr_mode = SENSOR_MODE_HDR_STITCH,
		.stitching_mode = SENSOR_STITCHING_3DOL,
		.bit_width = 10,
		.bayer_pattern = BAYER_GBRG,
	},
	{
		.index     = 2,
		.width    = 1280,
		.height   = 720,
		.fps      = 60,
		.hdr_mode = SENSOR_MODE_LINEAR,
		.bit_width = 10,
		.bayer_pattern = BAYER_GBRG,
	},
};


static int32_t sensor_query(void *ctx, struct vvcam_mode_info_array *pmode_info_arry)
{
	if (!pmode_info_arry)
	{
		return -1;
	}
	pmode_info_arry->count = ARRAY_SIZE(pos02k10_mode_info);

	memcpy(pmode_info_arry->modes,pos02k10_mode_info,sizeof(pos02k10_mode_info));

	return 0;
}

static int32_t sensor_write_reg(void *ctx, uint32_t reg, uint32_t val)
{
	int32_t ret = 0;
	struct vvcam_sensor_dev *dev = ctx;

	ret = vvcam_sensor_i2c_write(dev, reg, val);

	return ret;
}
static int32_t sensor_read_reg(void *ctx, uint32_t reg, uint32_t *pval)
{
	int32_t ret = 0;
	struct vvcam_sensor_dev *dev = ctx;

	ret = vvcam_sensor_i2c_read(dev, reg, pval);

	return ret;
}

static int32_t sensor_write_reg_arry(void *ctx, struct vvcam_sccb_array *parray)
{
	int32_t ret = 0;
	int32_t index = 0;
	struct vvcam_sensor_dev *dev = ctx;

	for (index = 0; index < parray->count; index++)
	{
		ret = vvcam_sensor_i2c_write(dev, parray->sccb_data[index].addr, parray->sccb_data[index].data);
		if (ret != 0)
		{
			return ret;
		}
	}

	return ret;
}

static int32_t sensor_get_chip_id(void *ctx, uint32_t *chip_id)
{
	int32_t ret = 0;
	int32_t chip_id_high = 0;
	int32_t chip_id_low = 0;
	ret = sensor_read_reg(ctx, 0x300a, &chip_id_high);
	ret |= sensor_read_reg(ctx, 0x300b, &chip_id_low);

	*chip_id = ((chip_id_high & 0xff)<<8) | (chip_id_low & 0xff);

	return ret;
}

static int32_t sensor_init(void *ctx, struct vvcam_mode_info *pmode)
{
	int32_t ret = 0;
	int32_t i = 0;
	struct vvcam_sensor_dev *dev = ctx;
	struct vvcam_mode_info *psensor_mode = NULL;

	for (i=0; i < sizeof(pos02k10_mode_info)/ sizeof(struct vvcam_mode_info); i++)
	{
		if (pos02k10_mode_info[i].index == pmode->index)
        {
            psensor_mode = &(pos02k10_mode_info[i]);
            break;
        }
	}

	if (psensor_mode == NULL)
	{
		return -1;
	}

	memcpy(&(dev->sensor_mode),psensor_mode,sizeof(struct vvcam_mode_info));

	switch(dev->sensor_mode.index)
	{
		case 0:
			ret = sensor_write_reg_arry(ctx,&os02k10_mipi4lane_1080p_30fps_linear_arry);

			dev->ae_info.DefaultFrameLengthLines = 0x466;
			dev->ae_info.CurFrameLengthLines = dev->ae_info.DefaultFrameLengthLines;
			dev->ae_info.one_line_exp_time_ns = 69607;//ns

			dev->ae_info.max_integration_time = dev->ae_info.CurFrameLengthLines -4;
			dev->ae_info.min_integration_time = 1;
			dev->ae_info.integration_accuracy = 1;

			dev->ae_info.gain_accuracy = 1024;
			dev->ae_info.max_gain = 22 * dev->ae_info.gain_accuracy;
			dev->ae_info.min_gain = 3 * dev->ae_info.gain_accuracy;

			dev->ae_info.cur_fps = dev->sensor_mode.fps;
			break;
		case 1:
			ret = sensor_write_reg_arry(ctx,&os02k10_mipi4lane_1080p_30fps_linear_arry);

			dev->ae_info.DefaultFrameLengthLines = 0x466;
			dev->ae_info.CurFrameLengthLines = dev->ae_info.DefaultFrameLengthLines;
			dev->ae_info.one_line_exp_time_ns = 69607;//ns

			dev->ae_info.max_integration_time = dev->ae_info.CurFrameLengthLines -4;
			dev->ae_info.min_integration_time = 1;
			dev->ae_info.integration_accuracy = 1;

			dev->ae_info.gain_accuracy = 1024;
			dev->ae_info.max_gain = 22 * dev->ae_info.gain_accuracy;
			dev->ae_info.min_gain = 3 * dev->ae_info.gain_accuracy;

			dev->ae_info.cur_fps = dev->sensor_mode.fps;
			break;
		default:
			break;
	}

	return ret;
}

static int32_t sensor_set_stream(void *ctx, uint32_t status)
{
	int32_t ret = 0;
	if (status)
	{
		ret = sensor_write_reg(ctx, 0x3012, 0x01);
	}else
	{
		ret = sensor_write_reg(ctx, 0x3012, 0x00);
	}

	return ret;
}

static int32_t sensor_set_exp(void *ctx, uint32_t exp_line)
{
	int32_t ret = 0;
	ret  = sensor_write_reg(ctx, 0x3467, 0x00);
	ret |= sensor_write_reg(ctx, 0x3464, 0x04);
	ret |= sensor_write_reg(ctx, 0x30b6, (exp_line>>8) & 0xff);
	ret |= sensor_write_reg(ctx, 0x30b7, exp_line & 0xff);
	ret |= sensor_write_reg(ctx, 0x3464, 0x14);
	ret |= sensor_write_reg(ctx, 0x3467, 0x01);
	return ret;
}

static int32_t sensor_set_vs_exp(void *ctx, uint32_t exp_line)
{
	int32_t ret = 0;
	ret  = sensor_write_reg(ctx, 0x3467, 0x00);
	ret |= sensor_write_reg(ctx, 0x3464, 0x04);
	ret |= sensor_write_reg(ctx, 0x30b8, (exp_line>>8) & 0xff);
	ret |= sensor_write_reg(ctx, 0x30b9, exp_line & 0xff);
	ret |= sensor_write_reg(ctx, 0x3464, 0x14);
	ret |= sensor_write_reg(ctx, 0x3467, 0x01);
	return ret;

}

static int32_t sensor_calc_gain(uint32_t total_gain, uint32_t *pagain, uint32_t *pdgain, uint32_t *phcg)
{
	uint32_t sensor_gain;

	sensor_gain = total_gain;

	if(sensor_gain <= 3072)// 3 * gain_accuracy
	{
		sensor_gain = 3072;
	}
	else if((sensor_gain >= 22528) && (sensor_gain < 23552)) //gain >22*gain_accuracy && gain < 23*gain_accuracy
	{
		sensor_gain = 22528;
	}

	if (sensor_gain < 4480) //gain < 4.375 * gain_accuracy
	{
		*pagain = 1;
		*phcg = 1;
	}
	else if(sensor_gain < 8960)//gain < 8.75 * gain_accuracy
	{
		*pagain = 2;
		*phcg = 1;
	}
	else if(sensor_gain < 22528)//gain < 22 * gain_accuracy
	{
		*pagain = 3;
		*phcg = 1;
	}
	else if(sensor_gain < 44990)//gain < 44 * gain_accuracy
	{
		*pagain = 0;
		*phcg = 11;
	}
	else if (sensor_gain < 89320)//gain < 88 * gain_accuracy
	{
		*pagain = 1;
		*phcg = 11;
	}
	else if (sensor_gain < 179498)//gain < 176 * gain_accuracy
	{
		*pagain = 2;
		*phcg = 11;
	}else
	{
		*pagain = 3;
		*phcg = 11;
	}
	*pdgain = ((sensor_gain << 8) >> (*pagain)) / (1024 * (*phcg));
	return 0;

}

static int32_t sensor_set_gain(void *ctx, uint32_t gain)
{
	int32_t ret = 0;
	uint32_t again = 0;
	uint32_t dgain = 0;
	uint32_t hcg = 1;

	uint32_t hcg_gain;
	uint32_t lcg_gain;

	uint32_t hcg_again = 0;
	uint32_t hcg_dgain = 0;
	uint32_t lcg_again = 0;
	uint32_t lcg_dgain = 0;

	uint32_t reg_val = 0;
	uint32_t hdr_radio;

	struct vvcam_sensor_dev *dev = ctx;

	hdr_radio= dev->ae_info.hdr_radio;

	switch(dev->sensor_mode.index)
	{
		case 0:
			sensor_calc_gain(gain, &again, &dgain, &hcg);

			ret = sensor_read_reg(ctx, 0x30bb, &reg_val);
			if (hcg == 1)
			{
				reg_val &= ~(1<<6); //LCG
			}else
			{
				reg_val |= (1<<6);  //HCG
			}
			reg_val &= ~0x03;
			reg_val |= again;
			ret  = sensor_write_reg(ctx, 0x3467, 0x00);
			ret |= sensor_write_reg(ctx, 0x3464, 0x04);
			ret |= sensor_write_reg(ctx, 0x315a, (dgain>>8) & 0xff);
			ret |= sensor_write_reg(ctx, 0x315b, dgain & 0xff);
			ret |= sensor_write_reg(ctx, 0x30bb, reg_val);
			ret |= sensor_write_reg(ctx, 0x3464, 0x14);
			ret |= sensor_write_reg(ctx, 0x3467, 0x01);
			break;
		case 1:
			hcg_gain = gain * hdr_radio / 11;
			sensor_calc_gain(gain, &hcg_again, &hcg_dgain, &hcg);
			lcg_gain = gain;
			sensor_calc_gain(gain, &lcg_again, &lcg_dgain, &hcg);

			ret = sensor_read_reg(ctx, 0x30bb, &reg_val);
			reg_val &= ~0x0f;
			reg_val |= (lcg_again<<2)&0x03;
			reg_val |= hcg_again  & 0x03;

			ret  = sensor_write_reg(ctx, 0x3467, 0x00);
			ret |= sensor_write_reg(ctx, 0x3464, 0x04);

			ret |= sensor_write_reg(ctx, 0x315a, (hcg_dgain>>8) & 0xff);
			ret |= sensor_write_reg(ctx, 0x315b, hcg_dgain & 0xff);

			ret |= sensor_write_reg(ctx, 0x315c, (lcg_dgain>>8) & 0xff);
			ret |= sensor_write_reg(ctx, 0x315d, lcg_dgain & 0xff);

			ret |= sensor_write_reg(ctx, 0x30bb, reg_val);
			ret |= sensor_write_reg(ctx, 0x3464, 0x14);
			ret |= sensor_write_reg(ctx, 0x3467, 0x01);
			break;
		default:
			break;
	}

	return ret;
}

static int32_t sensor_set_vs_gain(void *ctx, uint32_t gain)
{
	int32_t ret = 0;
	uint32_t again = 0;
	uint32_t dgain = 0;
	uint32_t sensor_gain;
	uint32_t hcg = 1;
	uint32_t reg_val = 0;

	sensor_gain = gain;

	sensor_calc_gain(gain, &again, &dgain, &hcg);

	ret = sensor_read_reg(ctx, 0x30bb, &reg_val);

	reg_val &= ~0x30;
	reg_val |= (again & 0x03) << 4;
	ret  = sensor_write_reg(ctx, 0x3467, 0x00);
	ret |= sensor_write_reg(ctx, 0x3464, 0x04);
	ret |= sensor_write_reg(ctx, 0x315e, (dgain>>8) & 0xff);
	ret |= sensor_write_reg(ctx, 0x315f, dgain & 0xff);
	ret |= sensor_write_reg(ctx, 0x30bb, reg_val);
	ret |= sensor_write_reg(ctx, 0x3464, 0x14);
	ret |= sensor_write_reg(ctx, 0x3467, 0x01);

	return ret;

}

static int32_t sensor_set_fps(void *ctx, uint32_t fps)
{
	int32_t ret = 0;
	uint32_t FrameLengthLines = 0;
	struct vvcam_sensor_dev *dev = ctx;

	if (fps > dev->sensor_mode.fps)
	{
		return -1;
	}
	FrameLengthLines = dev->sensor_mode.fps * dev->ae_info.DefaultFrameLengthLines / fps;

	if (FrameLengthLines != dev->ae_info.CurFrameLengthLines)
	{
		ret = sensor_write_reg(ctx, 0x30b2, (FrameLengthLines >> 8) & 0xff);
		ret |= sensor_write_reg(ctx, 0x30b3, FrameLengthLines  & 0xff);
		if (ret != 0)
		{
			return -1;
		}
		dev->ae_info.CurFrameLengthLines = FrameLengthLines;
		dev->ae_info.max_integration_time = FrameLengthLines-3;
		dev->ae_info.cur_fps = fps;
	}
	return ret;
}

static int32_t sensor_set_resolution(void *ctx, uint32_t width, uint32_t height)
{
	return 0;
}

static int32_t sensor_set_hdr_mode(void *ctx, uint32_t hdr_mode)
{
	int32_t ret = 0;
	struct vvcam_sensor_dev *dev = ctx;

	if (hdr_mode == dev->sensor_mode.hdr_mode)
	{
		return 0;
	}
	if (hdr_mode == SENSOR_MODE_LINEAR)
	{
		ret = sensor_write_reg(ctx, 0x3190, 0x08);
		dev->sensor_mode.hdr_mode = SENSOR_MODE_LINEAR;
	}
	else if(hdr_mode == SENSOR_MODE_HDR_STITCH)
	{
		ret = sensor_write_reg(ctx, 0x3190, 0x05);
		dev->sensor_mode.hdr_mode = SENSOR_MODE_HDR_STITCH;
	}
	return ret;
}

struct vvcam_sensor_function_s os02k10_function =
{
	.sensor_name = "os02k10",
	.reserve_id  = 0x2770,
	.sensor_clk  = SENSOR_CLK,
	.mipi_info.mipi_lane = 2,

	.sensor_get_chip_id    = sensor_get_chip_id,
	.sensor_init           = sensor_init,
	.sensor_set_stream     = sensor_set_stream,
	.sensor_set_exp        = sensor_set_exp,
	.sensor_set_vs_exp     = sensor_set_vs_exp,
	.sensor_set_gain       = sensor_set_gain,
	.sensor_set_vs_gain    = sensor_set_vs_gain,
	.sensor_set_fps        = sensor_set_fps,
	.sensor_set_resolution = sensor_set_resolution,
	.sensor_set_hdr_mode   = sensor_set_hdr_mode,
	.sensor_query          = sensor_query,
};
