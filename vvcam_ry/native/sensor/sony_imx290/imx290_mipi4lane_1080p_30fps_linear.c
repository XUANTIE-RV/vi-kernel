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


struct vvcam_sccb_data imx290_mipi4lane_1080p_30fps_linear_reg[] = {
	{0x3000, 0x01},
	{0x3002, 0x01},
	{0x3005, 0x01},
	{0x3129, 0x00},
	{0x317c, 0x00},
	{0x31ec, 0x0e},
	{0x3441, 0x0c},
	{0x3442, 0x0c},
	{0x3007, 0x00},
	{0x300c, 0x00},
	{0x300f, 0x00},
	{0x3010, 0x21},
	{0x3012, 0x64},
	{0x3016, 0x09},
	{0x3017, 0x00},
	{0x3009, 0x12},
	{0x3014, 0x0b},
	{0x3018, 0x47},
	{0x3019, 0x05},
	{0x3020, 0x01},
	{0x3021, 0x00},
	{0x3024, 0x00},
	{0x3025, 0x00},
	{0x3028, 0x00},
	{0x3029, 0x00},
	{0x3030, 0x00},
	{0x3031, 0x00},
	{0x3034, 0x00},
	{0x3035, 0x00},
	{0x305c, 0x18},
	{0x305d, 0x03},
	{0x305e, 0x20},
	{0x305f, 0x01},
	{0x3070, 0x02},
	{0x3071, 0x11},
	{0x309b, 0x10},
	{0x309c, 0x22},
	{0x30a2, 0x02},
	{0x30a6, 0x20},
	{0x30a8, 0x20},
	{0x30aa, 0x20},
	{0x30ac, 0x20},
	{0x30b0, 0x43},
	{0x3119, 0x9e},
	{0x311c, 0x1e},
	{0x311e, 0x08},
	{0x3128, 0x05},
	{0x313d, 0x83},
	{0x3150, 0x03},
	{0x317e, 0x00},
	{0x315e, 0x1a},
	{0x3164, 0x1a},
	{0x32b8, 0x50},
	{0x32b9, 0x10},
	{0x32ba, 0x00},
	{0x32bb, 0x04},
	{0x32c8, 0x50},
	{0x32c9, 0x10},
	{0x32ca, 0x00},
	{0x32cb, 0x04},
	{0x332c, 0xd3},
	{0x332d, 0x10},
	{0x332e, 0x0d},
	{0x3358, 0x06},
	{0x3359, 0xe1},
	{0x335a, 0x11},
	{0x3360, 0x1e},
	{0x3361, 0x61},
	{0x3362, 0x10},
	{0x33b0, 0x50},
	{0x33b2, 0x1a},
	{0x33b3, 0x04},
	{0x3414, 0x0a},
	{0x3418, 0x49},
	{0x3419, 0x04},
	{0x3444, 0x20},
	{0x3445, 0x25},
	{0x3446, 0x47},
	{0x3447, 0x00},
	{0x3448, 0x1f},
	{0x3449, 0x00},
	{0x344a, 0x17},
	{0x344b, 0x00},
	{0x344c, 0x0f},
	{0x344d, 0x00},
	{0x344e, 0x17},
	{0x344f, 0x00},
	{0x3450, 0x47},
	{0x3451, 0x00},
	{0x3452, 0x0f},
	{0x3453, 0x00},
	{0x3454, 0x0f},
	{0x3455, 0x00},
	{0x3480, 0x49},
	{0x3000, 0x00},
	{0x3002, 0x00},
	{0x304b, 0x0a},
};

struct vvcam_sccb_array imx290_mipi4lane_1080p_30fps_linear_arry = {
    .count = sizeof(imx290_mipi4lane_1080p_30fps_linear_reg) / sizeof(struct vvcam_sccb_data),
    .sccb_data = imx290_mipi4lane_1080p_30fps_linear_reg,
};
