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
#ifdef __KERNEL__
#include <linux/io.h>
#include <linux/module.h>
#endif
#include "mrv_all_bits.h"
#include "isp_ioctl.h"
#include "isp_types.h"

#ifdef ISP_WDR_V4
#define WDR4_WW						(32)
#define WDR4_HH						(32)
#define WDR4_MAX_VALUE				(1048575)
#define WDR4_GAIN_SHIFT				(2)
#define WDR4_NORMALIZE				(1024)
#define WDR4_NORMALIZE_SHIFT		(10)
#ifdef ISP_WDR_V4_20BIT
#define MODULE_INPUT_BIT_DEPTH		(20)
#else
#define MODULE_INPUT_BIT_DEPTH		(12)
#endif
#define MODULE_OUTPUT_BIT_DEPTH		(12)

extern MrvAllRegister_t *all_regs;

static void wdr4_config_curve(struct isp_ic_dev *dev)
{
	struct isp_wdr4_context *wdr4 = &dev->wdr4;
    u32 pos, i , val;
	//u32 isp_wdr4_shift_0;
	//u32 isp_wdr4_shift0_0;
    //initialize the histogram && entropy && gamma_pre && gamma_up && gamma_down && invert_linear
	for (i = 0; i < 7; i++) {
        u32  val_high_bit, val_low_bit;
		pos = i * 3;
        //get the curve that combined with high && low data
        val_high_bit = (wdr4->histogram[pos] >> 10) & (0x3ff);
        val_low_bit = wdr4->histogram[pos] & (0x3ff);

		val = val_low_bit << WDR4_HISTOGRAM_CURVE0_0_SHIFT ;
		val |= wdr4->histogram[pos +1];
		isp_write_reg(dev, REG_ADDR(isp_wdr4_histogram_0) + i * 2 * ISP_REG_GAP, val);

		val = val_high_bit << WDR4_HISTOGRAM_CURVE0_0_SHIFT ;

        if (pos + 2 < WDR4_BIN) {
            val |= wdr4->histogram[pos +2];
        }
		isp_write_reg(dev, REG_ADDR(isp_wdr4_histogram_0) + (i *2 + 1) * ISP_REG_GAP, val);

        //get the value that combined with high && low data
        val_high_bit = (wdr4->gamma_pre[pos] >> 10) & 0x3ff;
        val_low_bit = (wdr4->gamma_pre[pos]  & 0x3ff) ;

		val = val_low_bit << WDR4_GAMMA_PRE_CURVE0_1_SHIFT ;
		val |= wdr4->gamma_pre[pos +1];
		isp_write_reg(dev, REG_ADDR(isp_wdr4_gamma_pre_0) + (i * 2) * ISP_REG_GAP, val);

		val = val_high_bit << WDR4_GAMMA_PRE_CURVE0_1_SHIFT ;

        if (pos + 2 < WDR4_BIN)
        val |= wdr4->gamma_pre[pos +2];
		isp_write_reg(dev, REG_ADDR(isp_wdr4_gamma_pre_0) + (i *2 + 1) * ISP_REG_GAP,  val);


        //get the value that combined with high && low data
        val_high_bit = (wdr4->gamma_up[pos] >> 10) & 0x3ff;
        val_low_bit = (wdr4->gamma_up[pos] & 0x3ff);

		val = val_low_bit <<WDR4_GAMMA_UP_CURVE0_0_SHIFT;
		val |= wdr4->gamma_up[pos +1];
		isp_write_reg(dev, REG_ADDR(isp_wdr4_gamma_up_0) + (i * 2) * ISP_REG_GAP, val);
		val = val_high_bit  <<WDR4_GAMMA_UP_CURVE0_1_SHIFT;
        if (pos + 2 < WDR4_BIN)
        val |= wdr4->gamma_up[pos +2];
		isp_write_reg(dev, REG_ADDR(isp_wdr4_gamma_up_0) + (i *2 + 1) * ISP_REG_GAP,  val);


         //get the value that combined with high && low data
        val_high_bit = (wdr4->invert_linear[pos] >> 9) & 0x1ff;
        val_low_bit = (wdr4->invert_linear[pos] & 0x1ff);

        val = val_low_bit  << WDR4_LINEAR_CURVE_INVERT0_0_SHIFT;
        val |= wdr4->invert_linear[pos +1];
        isp_write_reg(dev, REG_ADDR(isp_wdr4_invert_linear_0) + (i * 2) * ISP_REG_GAP, val);
        val = val_high_bit << WDR4_LINEAR_CURVE_INVERT0_0_SHIFT;
        if (pos + 2 < WDR4_BIN)
        val |= wdr4->invert_linear[pos +2];
        isp_write_reg(dev, REG_ADDR(isp_wdr4_invert_linear_0) + (i *2 + 1) * ISP_REG_GAP,  val);

		if (pos + 2 < WDR4_BIN) {
			val = wdr4->entropy[pos] << WDR4_ENTROPY_CONVERT0_SHIFT ;
			val |= wdr4->entropy[pos +1] <<  WDR4_ENTROPY_CONVERT1_SHIFT;

			val |= wdr4->entropy[pos +2] <<  WDR4_ENTROPY_CONVERT2_SHIFT;
		} else {     //the lasw two entropy
			val = wdr4->entropy[pos] << WDR4_ENTROPY_CONVERT18_SHIFT ;
			val |= wdr4->entropy[pos +1] <<  WDR4_ENTROPY_CONVERT19_SHIFT;
		}
		isp_write_reg(dev, REG_ADDR(isp_wdr4_entropy_0) + i * ISP_REG_GAP, val);

		if (pos + 2 < WDR4_BIN)  {
			val = wdr4->gamma_down[pos] << WDR4_GAMMA_DOWN_CURVE0_SHIFT ;
			val |= wdr4->gamma_down[pos +1] <<  WDR4_GAMMA_DOWN_CURVE1_SHIFT;

			val |= wdr4->gamma_down[pos +2] <<  WDR4_GAMMA_DOWN_CURVE2_SHIFT;
		} else {   //the lasw two entropy
			val = wdr4->gamma_down[pos] << WDR4_GAMMA_DOWN_CURVE18_SHIFT ;
			val |= wdr4->gamma_down[pos +1] <<  WDR4_GAMMA_DOWN_CURVE19_SHIFT;

		}
		isp_write_reg(dev, REG_ADDR(isp_wdr4_gamma_down_0) + i * ISP_REG_GAP, val);
	}

    //config the distance weight && different weight && SMOOTH INVERT
    for (i = 0 ; i < 5  ; i++) {
        pos = i * ISP_REG_GAP;
		val = wdr4->distance_weight[pos] << WDR4_DISTANCE_WEIGHT_CURVE0_SHIFT ;
		val |= wdr4->distance_weight[pos +1] <<  WDR4_DISTANCE_WEIGHT_CURVE1_SHIFT;
		val |= wdr4->distance_weight[pos +2] << WDR4_DISTANCE_WEIGHT_CURVE2_SHIFT;
		val |= wdr4->distance_weight[pos +3] << WDR4_DISTANCE_WEIGHT_CURVE3_SHIFT;
		isp_write_reg(dev, REG_ADDR(isp_wdr4_distance_weight_0) + i * ISP_REG_GAP, val);

		val = wdr4->difference_weight[pos] << WDR4_DIFFERENCE_WEIGHT_CURVE0_SHIFT ;
		val |= wdr4->difference_weight[pos +1] <<  WDR4_DIFFERENCE_WEIGHT_CURVE1_SHIFT;
		val |= wdr4->difference_weight[pos +2] << WDR4_DIFFERENCE_WEIGHT_CURVE2_SHIFT;
		val |= wdr4->difference_weight[pos +3] << WDR4_DIFFERENCE_WEIGHT_CURVE3_SHIFT;


        isp_write_reg(dev, REG_ADDR(isp_wdr4_difference_weight_0) + i * ISP_REG_GAP, val);


		val = wdr4->smooth_invert[pos] << WDR4_SMOOTH_INVERT_CURVE0_SHIFT ;
		val |= wdr4->smooth_invert[pos +1] <<  WDR4_SMOOTH_INVERT_CURVE1_SHIFT;
		val |= wdr4->smooth_invert[pos +2] << WDR4_SMOOTH_INVERT_CURVE2_SHIFT;
		val |= wdr4->smooth_invert[pos +3] << WDR4_SMOOTH_INVERT_CURVE3_SHIFT;
		isp_write_reg(dev, REG_ADDR(isp_wdr4_smooth_invert_0) + i * ISP_REG_GAP, val);

    }
	for (i = 0; i < 10; i++) {
		val = 0;
		REG_SET_SLICE(val, WDR4_GLOBAL_CURVE_INVERT0,
			      wdr4->invert_curve[i * 2]);
		REG_SET_SLICE(val, WDR4_GLOBAL_CURVE_INVERT1,
			      wdr4->invert_curve[i * 2 + 1]);
		isp_write_reg(dev, REG_ADDR(isp_wdr4_invert_curve_0) + i * ISP_REG_GAP, val);

	}

    for (i = 0 ; i < 4; i++) {
        val = 0 ;
        pos = i * 6;
        if (pos + 3 > WDR4_BIN) {
            REG_SET_SLICE(val, WDR4_HISTOGRAM_SHIFT18, wdr4->shift[ pos +0]);
            REG_SET_SLICE(val, WDR4_HISTOGRAM_SHIFT19, wdr4->shift[ pos +1]);
        } else {
            REG_SET_SLICE(val, WDR4_HISTOGRAM_SHIFT0, wdr4->shift[ pos +0]);
            REG_SET_SLICE(val, WDR4_HISTOGRAM_SHIFT1, wdr4->shift[ pos +1]);
            REG_SET_SLICE(val, WDR4_HISTOGRAM_SHIFT2, wdr4->shift[ pos +2]);
            REG_SET_SLICE(val, WDR4_HISTOGRAM_SHIFT3, wdr4->shift[ pos +3]);
            REG_SET_SLICE(val, WDR4_HISTOGRAM_SHIFT4, wdr4->shift[ pos +4]);
            REG_SET_SLICE(val, WDR4_HISTOGRAM_SHIFT5, wdr4->shift[ pos +5]);
        }
        isp_write_reg(dev, REG_ADDR(isp_wdr4_shift_0) + i * ISP_REG_GAP,  val);

    }

    for (i = 0 ; i < 3; i++) {
        val = 0 ;
        pos = i * 8;
        if (pos + 5 > WDR4_BIN) {
            REG_SET_SLICE(val, WDR4_HISTOGRAM0_SHIFT16, wdr4->shift0[ pos +0]);
            REG_SET_SLICE(val, WDR4_HISTOGRAM0_SHIFT17, wdr4->shift0[ pos +1]);
            REG_SET_SLICE(val, WDR4_HISTOGRAM0_SHIFT18, wdr4->shift0[ pos +2]);
            REG_SET_SLICE(val, WDR4_HISTOGRAM0_SHIFT19, wdr4->shift0[ pos +3]);
        } else {
            REG_SET_SLICE(val, WDR4_HISTOGRAM0_SHIFT0, wdr4->shift0[ pos +0]);
            REG_SET_SLICE(val, WDR4_HISTOGRAM0_SHIFT1, wdr4->shift0[ pos +1]);
            REG_SET_SLICE(val, WDR4_HISTOGRAM0_SHIFT2, wdr4->shift0[ pos +2]);
            REG_SET_SLICE(val, WDR4_HISTOGRAM0_SHIFT3, wdr4->shift0[ pos +3]);
            REG_SET_SLICE(val, WDR4_HISTOGRAM0_SHIFT4, wdr4->shift0[ pos +4]);
            REG_SET_SLICE(val, WDR4_HISTOGRAM0_SHIFT5, wdr4->shift0[ pos +5]);
            REG_SET_SLICE(val, WDR4_HISTOGRAM0_SHIFT6, wdr4->shift0[ pos +6]);
            REG_SET_SLICE(val, WDR4_HISTOGRAM0_SHIFT7, wdr4->shift0[ pos +7]);
        }

        isp_write_reg(dev, REG_ADDR(isp_wdr4_shift0_0) + i * ISP_REG_GAP,  val);
    }

}
static void wdr4_hw_init(struct isp_ic_dev *dev)
{
	// struct isp_wdr4_context *wdr4 = &dev->wdr4;
	u32 width, height;
	u32 slice_sigma_height;
	u32 slice_sigma_width;
	u32 slice_sigma_value;
	u32 slice_block_width;
	u32 slice_block_height;
	u32 isp_wdr4_block_size;
	u32 slice_block_area_inverse;
	u32 isp_wdr4_value_weight;
	u32 isp_wdr4_pixel_slope;
	u32 isp_wdr4_entropy_slope;
	u32 isp_wdr4_sigma_width;
	u32 isp_wdr4_sigma_height;
	u32 isp_wdr4_sigma_value;
	u32 isp_wdr4_block_flag_width;
	u32 isp_wdr4_block_flag_height;
	u32 isp_wdr4_strength;
	u32 isp_wdr4_block_area_factor;
	u32 width_left;
	u32 width_count = 0;
	u32 height_left;
	u32 height_count = 0;
    u32 isp_wdr4_strength_0 = 0, isp_wdr4_strength_1 = 0, isp_wdr4_contrast = 0;
	int i;
    u8 isp_wdr4_low_strength, isp_wdr4_high_strength,  isp_wdr4_global_strength;
    u8  isp_wdr4_local_strength;

	u32 slice_pixel_shift;
	u32 slice_output_shift;// isp_wdr4_normalize_shift;
	u32 isp_wdr4_shift;// isp_wdr4_gain_shift_bit;
	pr_info("enter %s\n", __func__);

	width = isp_read_reg(dev, REG_ADDR(isp_acq_h_size));
	height = isp_read_reg(dev, REG_ADDR(isp_acq_v_size));

	pr_info("isp_wdr4 res: %d %d \n", width, height);
	/* firware initilization */

    isp_wdr4_strength = 128;         // valid values: [0,128]; isp_wdr4 will merge original and enhanced image
                                     // together based on this value.
                                    // 128: use enhanced image completely; 0: use original image completely
    isp_wdr4_low_strength = 16;     // Valid values: [0,255]; if hdr off, limit the maximum gain for image enhancement
                                     // if hdr on, control low light area information. Higher:haver more low light information
    isp_wdr4_high_strength = 60;     //valid values:[0, 128]: control high light area information. Higher:have more high
                                     // light information
    isp_wdr4_global_strength = 128;  // Valid values: [0,128]
                                     // isp_wdr4 will increase global contrast based on this value.
                                     // 128: use strongest contrast; 0: on contrast increasing.
	 isp_wdr4_local_strength = 128;   // valid values: [0,128]
                                  // wdr4 will increase local contrast based on this value.
                                  // 128: use strongest contrast; 0: on contrast increasing.

    isp_wdr4_strength_0 = isp_read_reg(dev, REG_ADDR(isp_wdr4_strength_0));
    REG_SET_SLICE(isp_wdr4_strength_0, WDR4_LOCAL_STRENGTH, isp_wdr4_local_strength);
    REG_SET_SLICE(isp_wdr4_strength_0, WDR4_GLOBAL_STRENGTH, isp_wdr4_global_strength);
    REG_SET_SLICE(isp_wdr4_strength_0, WDR4_LOW_STRENGTH, isp_wdr4_low_strength);
	REG_SET_SLICE(isp_wdr4_strength_0, WDR4_TOTAL_STRENGTH,isp_wdr4_strength);
	isp_write_reg(dev, REG_ADDR(isp_wdr4_strength_0), isp_wdr4_strength_0);

    isp_wdr4_strength_1 = isp_read_reg(dev, REG_ADDR(isp_wdr4_strength_1));
    REG_SET_SLICE(isp_wdr4_strength_1, WDR4_HIGH_STRENGTH, isp_wdr4_high_strength);
    REG_SET_SLICE(isp_wdr4_strength_1, WDR4_DRC_BAYER_RATIO, 14);
    REG_SET_SLICE(isp_wdr4_strength_1, WDR4_DRC_BAYER_RATIOLSVS, 14);
   	isp_write_reg(dev, REG_ADDR(isp_wdr4_strength_1), isp_wdr4_strength_1);

    isp_wdr4_contrast = isp_read_reg(dev, REG_ADDR(isp_wdr4_contrast));
    REG_SET_SLICE(isp_wdr4_contrast , WDR4_CONTRAST ,0); //valid values:[-1023~1023]:control contrast of image,.higher:image have stronger contrast
    REG_SET_SLICE(isp_wdr4_contrast , WDR4_FLAT_STRENGTH ,8);//valid values:[0, 19]:adjust flat area. Higher:flat area stretched more strong
    REG_SET_SLICE(isp_wdr4_contrast , WDR4_FLAT_THR ,1); //valid values:[0, 20]:judge flat region.

    isp_write_reg(dev, REG_ADDR(isp_wdr4_contrast), isp_wdr4_contrast);

    isp_wdr4_pixel_slope = isp_read_reg(dev, REG_ADDR(isp_wdr4_pixel_slope));
     REG_SET_SLICE(isp_wdr4_pixel_slope , WDR4_PIXEL_MERGE_SLOPE , 128);
     REG_SET_SLICE(isp_wdr4_pixel_slope , WDR4_PIXEL_MERGE_BASE ,  128);
     REG_SET_SLICE(isp_wdr4_pixel_slope , WDR4_PIXEL_ADJUST_SLOPE ,128);
     REG_SET_SLICE(isp_wdr4_pixel_slope , WDR4_PIXEL_ADJUST_BASE , 128);

     isp_write_reg(dev, REG_ADDR(isp_wdr4_pixel_slope), isp_wdr4_pixel_slope);

    isp_wdr4_entropy_slope = isp_read_reg(dev, REG_ADDR(isp_wdr4_entropy_slope));
     REG_SET_SLICE(isp_wdr4_entropy_slope , WDR4_ENTROPY_SLOPE , 0xc8);
     REG_SET_SLICE(isp_wdr4_entropy_slope , WDR4_ENTROPY_BASE ,0x02bc);
     isp_write_reg(dev, REG_ADDR(isp_wdr4_entropy_slope), isp_wdr4_entropy_slope);

	isp_wdr4_value_weight = isp_read_reg(dev, REG_ADDR(isp_wdr4_value_weight));
	REG_SET_SLICE(isp_wdr4_value_weight , WDR4_VALUE_WEIGHT_0 , 6);
	REG_SET_SLICE(isp_wdr4_value_weight , WDR4_VALUE_WEIGHT_1 , 5);
	REG_SET_SLICE(isp_wdr4_value_weight , WDR4_VALUE_WEIGHT_2 , 5);
	REG_SET_SLICE(isp_wdr4_value_weight , WDR4_VALUE_WEIGHT_3 , 16);
	isp_write_reg(dev, REG_ADDR(isp_wdr4_value_weight), isp_wdr4_value_weight);

	slice_block_width = width / WDR4_WW;
	slice_block_height = height / WDR4_HH;
	isp_wdr4_block_size = 0;
	REG_SET_SLICE(isp_wdr4_block_size, WDR4_BLOCK_WIDTH, slice_block_width);
	REG_SET_SLICE(isp_wdr4_block_size, WDR4_BLOCK_HEIGHT, slice_block_height);
	isp_write_reg(dev, REG_ADDR(isp_wdr4_block_size), isp_wdr4_block_size);

	slice_block_area_inverse =
	    WDR4_NORMALIZE * WDR4_NORMALIZE / (slice_block_width *
					       slice_block_height);
	isp_wdr4_block_area_factor = isp_read_reg(dev, REG_ADDR(isp_wdr4_block_area_factor));
    REG_SET_SLICE(isp_wdr4_block_area_factor, WDR4_BLOCK_AREA_INVERSE, slice_block_area_inverse);
 	isp_write_reg(dev, REG_ADDR(isp_wdr4_block_area_factor), isp_wdr4_block_area_factor);

	slice_sigma_height =
	    WDR4_NORMALIZE * WDR4_NORMALIZE / slice_block_height;
	slice_sigma_width = WDR4_NORMALIZE * WDR4_NORMALIZE / slice_block_width;
	slice_sigma_value = WDR4_NORMALIZE  * WDR4_NORMALIZE/ WDR4_MAX_VALUE;

	isp_wdr4_sigma_width =
	    isp_read_reg(dev, REG_ADDR(isp_wdr4_sigma_width));
	REG_SET_SLICE(isp_wdr4_sigma_width, WDR4_BILITERAL_WIDTH_SIGMA,
		      slice_sigma_width);
	isp_write_reg(dev, REG_ADDR(isp_wdr4_sigma_width),
		      isp_wdr4_sigma_width);

	isp_wdr4_sigma_height = isp_read_reg(dev, REG_ADDR(isp_wdr4_sigma_height));
	REG_SET_SLICE(isp_wdr4_sigma_height, WDR4_BILITERAL_HEIGHT_SIGMA,
		      slice_sigma_height);
	isp_write_reg(dev, REG_ADDR(isp_wdr4_sigma_height),
		      isp_wdr4_sigma_height);

	isp_wdr4_sigma_value = isp_read_reg(dev, REG_ADDR(isp_wdr4_sigma_value));
	REG_SET_SLICE(isp_wdr4_sigma_value, WDR4_BILITERAL_VALUE_SIGMA,
		      slice_sigma_value);
	isp_write_reg(dev, REG_ADDR(isp_wdr4_sigma_value),
		      isp_wdr4_sigma_value);

	/* block flag configuration */
	width_left = width - slice_block_width * WDR4_WW;
	height_left = height - slice_block_height * WDR4_HH;


    // isp_wdr4_gain_shift_bit = 8;
    // isp_wdr4_normalize_shift = 10;
	slice_pixel_shift =  MODULE_INPUT_BIT_DEPTH - 15;
	slice_output_shift = MODULE_INPUT_BIT_DEPTH - MODULE_OUTPUT_BIT_DEPTH;


	isp_wdr4_shift = isp_read_reg(dev, REG_ADDR(isp_wdr4_shift));
	REG_SET_SLICE(isp_wdr4_shift, WDR4_PIXEL_SHIFT_BIT, slice_pixel_shift);
	REG_SET_SLICE(isp_wdr4_shift, WDR4_OUTPUT_SHIFT_BIT,
		      slice_output_shift);
	isp_write_reg(dev, REG_ADDR(isp_wdr4_shift), isp_wdr4_shift);


	/* block flag configuration */
	width_left = width - slice_block_width * WDR4_WW;
	height_left = height - slice_block_height * WDR4_HH;

	isp_wdr4_block_flag_width = 0;
	isp_wdr4_block_flag_height = 0;

	for (i = 0, width_count = 0;
	     (i < WDR4_WW) && (width_count < width_left); i++, width_count++) {
		isp_wdr4_block_flag_width |= (1 << i);
	}
	isp_write_reg(dev, REG_ADDR(isp_wdr4_block_flag_width), isp_wdr4_block_flag_width);

	for (i = 0, height_count = 0;
	     (i < WDR4_HH) && (height_count < height_left);
	     i++, height_count++) {
		isp_wdr4_block_flag_height |= (1 << i);
	}
	isp_write_reg(dev, REG_ADDR(isp_wdr4_block_flag_height), isp_wdr4_block_flag_height);

	isp_wdr4_block_size = isp_read_reg(dev, REG_ADDR(isp_wdr4_block_size));
	REG_SET_SLICE(isp_wdr4_block_size, WDR4_BLOCK_WIDTH, slice_block_width);
	REG_SET_SLICE(isp_wdr4_block_size, WDR4_BLOCK_HEIGHT,
		      slice_block_height);
	isp_write_reg(dev, REG_ADDR(isp_wdr4_block_size), isp_wdr4_block_size);

	wdr4_config_curve(dev);
#if 1
	{
	uint32_t isp_wdr4_ctrl = isp_read_reg(dev, REG_ADDR(isp_wdr4_ctrl));
	REG_SET_SLICE(isp_wdr4_ctrl, WDR4_ENABLE, 1);
	isp_write_reg(dev, REG_ADDR(isp_wdr4_ctrl), isp_wdr4_ctrl);
	}
#endif
}

int isp_enable_wdr4(struct isp_ic_dev *dev)
{
#ifndef ISP_WDR_V4
	pr_err("unsupported function: %s", __func__);
	return -EINVAL;
#else
	int32_t isp_wdr4_ctrl;
	struct isp_wdr4_context *wdr4 = &dev->wdr4;
	isp_read_reg(dev, REG_ADDR(isp_pre_filt_ctrl));
	isp_read_reg(dev, REG_ADDR(isp_dmsc_cac_y_norm_shd));

	isp_wdr4_ctrl = isp_read_reg(dev, REG_ADDR(isp_wdr4_ctrl));
	wdr4->enable = true;
	REG_SET_SLICE(isp_wdr4_ctrl, WDR4_ENABLE, 1);
	isp_write_reg(dev, REG_ADDR(isp_wdr4_ctrl), isp_wdr4_ctrl);

	return 0;
#endif
}

int isp_disable_wdr4(struct isp_ic_dev *dev)
{
#ifndef ISP_WDR_V4
	pr_err("unsupported function: %s", __func__);
	return -EINVAL;
#else
	struct isp_wdr4_context *wdr4 = &dev->wdr4;
	int32_t isp_wdr4_ctrl = isp_read_reg(dev, REG_ADDR(isp_wdr4_ctrl));
	wdr4->enable = false;
	REG_SET_SLICE(isp_wdr4_ctrl, WDR4_ENABLE, 0);
	isp_write_reg(dev, REG_ADDR(isp_wdr4_ctrl), isp_wdr4_ctrl);

	return 0;
#endif
}

int isp_u_wdr4(struct isp_ic_dev *dev)
{
#ifndef ISP_WDR_V4
	pr_err("unsupported function: %s", __func__);
	return -EINVAL;
#else
	wdr4_hw_init(dev);
	return 0;
#endif
}

int isp_s_wdr4(struct isp_ic_dev *dev)
{

#ifndef ISP_WDR_V4
	pr_err("unsupported function: %s", __func__);
	return -EINVAL;
#else
	struct isp_wdr4_context *wdr4 = &dev->wdr4;
	u32 isp_wdr4_strength_0 = isp_read_reg(dev, REG_ADDR(isp_wdr4_strength_0));
	u32 isp_wdr4_strength_1 = isp_read_reg(dev, REG_ADDR(isp_wdr4_strength_1));
	u32 isp_wdr4_contrast = isp_read_reg(dev, REG_ADDR(isp_wdr4_contrast));
	u32 width = isp_read_reg(dev, REG_ADDR(isp_acq_h_size));
	u32 height = isp_read_reg(dev, REG_ADDR(isp_acq_v_size));
	width /= 32;
	height /= 32;

	REG_SET_SLICE(isp_wdr4_strength_0, WDR4_LOW_STRENGTH, wdr4->low_gain);
	REG_SET_SLICE(isp_wdr4_strength_0, WDR4_GLOBAL_STRENGTH,
		      wdr4->global_strength);
	REG_SET_SLICE(isp_wdr4_strength_0, WDR4_LOCAL_STRENGTH, 128);
	REG_SET_SLICE(isp_wdr4_strength_0, WDR4_TOTAL_STRENGTH, wdr4->strength);
	REG_SET_SLICE(isp_wdr4_strength_1, WDR4_HIGH_STRENGTH, wdr4->high_strength);

    REG_SET_SLICE(isp_wdr4_contrast , WDR4_CONTRAST, wdr4->contrast); //valid values:[-1023~1023]:control contrast of image,.higher:image have stronger contrast
	REG_SET_SLICE(isp_wdr4_contrast , WDR4_FLAT_STRENGTH, wdr4->flat_strength);//valid values:[0, 19]:adjust flat area. Higher:flat area stretched more strong
    REG_SET_SLICE(isp_wdr4_contrast , WDR4_FLAT_THR, wdr4->flat_threshold); //valid values:[0, 20]:judge flat region.
    isp_write_reg(dev, REG_ADDR(isp_wdr4_contrast), isp_wdr4_contrast);

	isp_write_reg(dev, REG_ADDR(isp_wdr4_block_size),
		      width | (height << 9));
	isp_write_reg(dev, REG_ADDR(isp_wdr4_strength_0), isp_wdr4_strength_0);
	isp_write_reg(dev, REG_ADDR(isp_wdr4_strength_1), isp_wdr4_strength_1);
	isp_write_reg(dev, REG_ADDR(isp_wdr4_strength_0_shd), isp_wdr4_strength_0);	/* cmodel use */
	isp_write_reg(dev, REG_ADDR(isp_wdr4_strength_1_shd), isp_wdr4_strength_1);	/* cmodel use */

	return 0;
#endif
}
#endif

