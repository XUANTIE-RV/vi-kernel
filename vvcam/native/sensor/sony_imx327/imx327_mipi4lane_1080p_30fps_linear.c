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

#include "vvsensor.h"
#include "sensor_common.h"


struct vvcam_sccb_data imx327_mipi4lane_1080p_30fps_linear_reg[] = {
			 
{0x3000, 0x01},
{0x3001, 0x00},
{0x3002, 0x01},
{0x3003, 0x00},
{0x300C, 0x3B},
{0x300D, 0x2A},
{0x3030, 0xc4},
{0x3031, 0x1c},
{0x3034, 0xEC},
{0x3035, 0x0a},
{0x3058, 0x72},
{0x3059, 0x06},
{0x30E8, 0x14},
{0x314C, 0x29},
{0x314D, 0x01},
{0x315A, 0x03},
{0x3168, 0xA0},
{0x316A, 0x7E},
{0x31A1, 0x00},
{0x3288, 0x21},
{0x328A, 0x02},
{0x3414, 0x05},
{0x3416, 0x18},
{0x35AC, 0x0E},
{0x3648, 0x01},
{0x364A, 0x04},
{0x364C, 0x04},
{0x3678, 0x01},
{0x367C, 0x31},
{0x367E, 0x31},
{0x3708, 0x02},
{0x3714, 0x01},
{0x3715, 0x02},
{0x3716, 0x02},
{0x3717, 0x02},
{0x371C, 0x3D},
{0x371D, 0x3F},
{0x372C, 0x00},
{0x372D, 0x00},
{0x372E, 0x46},
{0x372F, 0x00},
{0x3730, 0x89},
{0x3731, 0x00},
{0x3732, 0x08},
{0x3733, 0x01},
{0x3734, 0xFE},
{0x3735, 0x05},
{0x375D, 0x00},
{0x375E, 0x00},
{0x375F, 0x61},
{0x3760, 0x06},
{0x3768, 0x1B},
{0x3769, 0x1B},
{0x376A, 0x1A},
{0x376B, 0x19},
{0x376C, 0x18},
{0x376D, 0x14},
{0x376E, 0x0F},
{0x3776, 0x00},
{0x3777, 0x00},
{0x3778, 0x46},
{0x3779, 0x00},
{0x377A, 0x08},
{0x377B, 0x01},
{0x377C, 0x45},
{0x377D, 0x01},
{0x377E, 0x23},
{0x377F, 0x02},
{0x3780, 0xD9},
{0x3781, 0x03},
{0x3782, 0xF5},
{0x3783, 0x06},
{0x3784, 0xA5},
{0x3788, 0x0F},
{0x378A, 0xD9},
{0x378B, 0x03},
{0x378C, 0xEB},
{0x378D, 0x05},
{0x378E, 0x87},
{0x378F, 0x06},
{0x3790, 0xF5},
{0x3792, 0x43},
{0x3794, 0x7A},
{0x3796, 0xA1},
{0x3E04, 0x0E},
};

struct vvcam_sccb_array imx327_mipi4lane_1080p_30fps_linear_arry = {
    .count = sizeof(imx327_mipi4lane_1080p_30fps_linear_reg) / sizeof(struct vvcam_sccb_data),
    .sccb_data = imx327_mipi4lane_1080p_30fps_linear_reg,
};
