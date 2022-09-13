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
#include "isp_ioctl.h"
#include "mrv_all_bits.h"

#include "isp_types.h"
#ifdef ISP_3DNR_V3
extern MrvAllRegister_t *all_regs;

int isp_tdnr_cfg_gamma(struct isp_ic_dev *dev)
{
	//write to regs
	struct isp_tdnr_context *tdnr = &dev->tdnr;
	u32 regVal;
	u8 bit_width;
	u8 i;
	u16 val_a, val_b;

	//const u8 tdnrRegGap = 8;
	bit_width = 12;

    pr_info("enter %s\n", __func__);

	for (i = 0; i < DENOISE3D_GAMMA_BIN - 3; i += 5)
	{
		val_a = tdnr->curve.preGamma_y[i + 2] >> 6; val_b =  tdnr->curve.preGamma_y[i + 2] & 0x3f;
		regVal = (val_a << 24) | ((tdnr->curve.preGamma_y[i + 1] & 0xfff) << bit_width) | (tdnr->curve.preGamma_y[i] & 0xfff);
		isp_write_reg(dev, REG_ADDR(isp_denoise3d2_pregamma_y_0) + (i / 5) *8 , regVal);

		regVal = (val_b << 24) | ((tdnr->curve.preGamma_y[i + 4] & 0xfff) << bit_width) | (tdnr->curve.preGamma_y[i + 3] & 0xfff);
		isp_write_reg(dev, REG_ADDR(isp_denoise3d2_pregamma_y_1) +  (i / 5 ) *8, regVal);
	}

	val_a = tdnr->curve.preGamma_y[i + 2] >> 6; val_b = tdnr->curve.preGamma_y[i + 2] & 0x3f;
	regVal = (val_a << 24) | ((tdnr->curve.preGamma_y[i + 1] & 0xfff) << bit_width) | (tdnr->curve.preGamma_y[i] & 0xfff);
	isp_write_reg(dev, REG_ADDR(isp_denoise3d2_pregamma_y_12), regVal);
	isp_write_reg(dev, REG_ADDR(isp_denoise3d2_pregamma_y_13), val_b);


	i = 0;
	for (i = 0; i < DENOISE3D_GAMMA_BIN - 3; i += 5)
	{
		val_a = tdnr->curve.invGamma_y[i + 2] >> 6; val_b = tdnr->curve.invGamma_y[i + 2] & 0x3f;
		regVal = (val_a << 24) | ((tdnr->curve.invGamma_y[i + 1] & 0xfff) << bit_width) | (tdnr->curve.invGamma_y[i] & 0xfff);
		isp_write_reg(dev, REG_ADDR(isp_denoise3d2_invgamma_y_0) + (i / 5) *8 , regVal);

		regVal = (val_b << 24) | ((tdnr->curve.invGamma_y[i + 4] & 0xfff) << bit_width) | (tdnr->curve.invGamma_y[i + 3] & 0xfff);
		isp_write_reg(dev, REG_ADDR(isp_denoise3d2_invgamma_y_1) + (i / 5) *8 , regVal);
	}

	val_a = tdnr->curve.invGamma_y[i + 2] >> 6; val_b = tdnr->curve.invGamma_y[i + 2] & 0x3f;
	regVal = (val_a << 24) | ((tdnr->curve.invGamma_y[i + 1] & 0xfff) << bit_width) | (tdnr->curve.invGamma_y[i] & 0xfff);
    isp_write_reg(dev, REG_ADDR(isp_denoise3d2_invgamma_y_12), regVal);
	isp_write_reg(dev, REG_ADDR(isp_denoise3d2_invgamma_y_13),  val_b);
	return 0;
}

int  isp_s_tdnr(struct isp_ic_dev *dev)
{
	u32 isp_denoise3d2_ctrl;
	struct isp_tdnr_context *tdnr = &dev->tdnr;
	u32 regVal = 0;
	/* spacial */
	u32 strength = tdnr->strength;

    pr_info("enter %s\n", __func__);
	strength = MIN(MAX(strength, 0), 128);

	isp_denoise3d2_ctrl = isp_read_reg(dev, REG_ADDR(isp_denoise3d2_ctrl));
#if 0
	if (!tdnr->enable) {
			REG_SET_SLICE(isp_denoise3d2_ctrl, DENOISE3D_V20_ENABLE, 0);
			isp_write_reg(dev, REG_ADDR(isp_denoise3d2_ctrl), isp_denoise3d2_ctrl);
			return 0;
	}
#endif

#if 1
	if (dev->tdnr.enable_tnr) {
		isp_write_reg(dev, REG_ADDR(isp_denoise3d2_strength), strength);
	} else {
		isp_write_reg(dev, REG_ADDR(isp_denoise3d2_strength), 0);
		if (dev->tdnr.frames == 2) 
			return 0 ;
	}
#endif
	//isp_write_reg(dev, REG_ADDR(isp_denoise3d2_strength), strength);
	regVal = 0;
	REG_SET_SLICE(regVal, DENOISE3D_V20_NOISE_LEVEL, tdnr->noise_level);
	REG_SET_SLICE(regVal, DENOISE3D_V20_NOISE_MEAN, tdnr->noise_mean);
	isp_write_reg(dev, REG_ADDR(isp_denoise3d2_noise), regVal);

	regVal = 0;
	REG_SET_SLICE(regVal, DENOISE3D_V20_NOISE_LEVEL, tdnr->noise_threshold);
	REG_SET_SLICE(regVal, DENOISE3D_V20_NOISE_MEAN, tdnr->motion_mean);
	isp_write_reg(dev, REG_ADDR(isp_denoise3d2_motion), regVal);

	regVal = 0;
	REG_SET_SLICE(regVal, DENOISE3D_V20_TNR_RANGE_H, tdnr->range_h);
	REG_SET_SLICE(regVal, DENOISE3D_V20_TNR_RANGE_V, tdnr->range_v);
	REG_SET_SLICE(regVal, DENOISE3D_V20_TNR_DILATE_RANGE_H, tdnr->dilate_range_h);
	REG_SET_SLICE(regVal, DENOISE3D_V20_TNR_DILATE_RANGE_V, tdnr->dilate_range_v);
	isp_write_reg(dev, REG_ADDR(isp_denoise3d2_range), regVal);

	regVal = 0;
	REG_SET_SLICE(regVal, DENOISE3D_V20_MOTION_INV, tdnr->motion_inv_factor);
	isp_write_reg(dev, REG_ADDR(isp_denoise3d2_motion_inv), regVal);

	regVal = 0;
	REG_SET_SLICE(regVal, DENOISE3D_V20_THR_UPDATE, tdnr->update_factor);
	REG_SET_SLICE(regVal, DENOISE3D_V20_MOTION_THR_UPDATE, tdnr->motion_update_factor);
	isp_write_reg(dev, REG_ADDR(isp_denoise3d2_update), regVal);

	regVal = 0;
	REG_SET_SLICE(regVal, DENOISE3D_V20_MOTION_PRE_WEIGHT, tdnr->pre_motion_weight);
	REG_SET_SLICE(regVal, DENOISE3D_V20_MOTION_SLOPE, tdnr->motion_slope);
	isp_write_reg(dev, REG_ADDR(isp_denoise3d2_tnr), regVal);

	regVal = 0;
	REG_SET_SLICE(regVal, DENOISE3D_V20_MOTION_CONV_SHIFT, tdnr->motion_converage_shift);
	REG_SET_SLICE(regVal, DENOISE3D_V20_MOTION_CONV_MAX, tdnr->motion_converage_max);
	REG_SET_SLICE(regVal, DENOISE3D_V20_TNR_VAL_SHIFT_BIT, tdnr->tnr_val_shift_bit);
	REG_SET_SLICE(regVal, DENOISE3D_V20_TNR_DIFF_NORM_SHIFT_BIT, tdnr->tnr_diff_shift_bit);
	REG_SET_SLICE(regVal, DENOISE3D_V20_SAD_WEIGHT, tdnr->sad_weight);
	isp_write_reg(dev, REG_ADDR(isp_denoise3d2_shift), regVal);

	regVal = 0;
	REG_SET_SLICE(regVal, DENOISE3D_V20_PIXEL_VAL_SHIFT, tdnr->luma_pixel_val_shift);
	REG_SET_SLICE(regVal, DENOISE3D_V20_PIXEL_SLOP, tdnr->luma_pixel_slope);
	REG_SET_SLICE(regVal, DENOISE3D_V20_PIXEL_SLOP_SHIFT_BIT, tdnr->luma_pixel_slope_shift);
	REG_SET_SLICE(regVal, DENOISE3D_V20_PIXEL_SLOP_MIN, tdnr->luma_pixel_slope_min);
	isp_write_reg(dev, REG_ADDR(isp_denoise3d2_luma), regVal);

	isp_write_reg(dev, REG_ADDR(isp_denoise3d2_dummy_hblank), 0x63);

	REG_SET_SLICE(isp_denoise3d2_ctrl, DENOISE3D_V20_INVGAMMA_EN, tdnr->enable_inv_gamma);
	REG_SET_SLICE(isp_denoise3d2_ctrl, DENOISE3D_V20_PREGAMMA_EN, tdnr->enable_pre_gamma);
	REG_SET_SLICE(isp_denoise3d2_ctrl, DENOISE3D_V20_ERODE_EN, tdnr->enable_motion_erosion);
	REG_SET_SLICE(isp_denoise3d2_ctrl, DENOISE3D_V20_MOTION_CONV_EN, tdnr->enable_motion_converage);
	REG_SET_SLICE(isp_denoise3d2_ctrl, DENOISE3D_V20_INV_DGAIN_EN, 0);
	REG_SET_SLICE(isp_denoise3d2_ctrl, DENOISE3D_V20_INV_AWB_GAIN_EN, 1);
	REG_SET_SLICE(isp_denoise3d2_ctrl, DENOISE3D_V20_COMP_LUMA_EN, 0);
	REG_SET_SLICE(isp_denoise3d2_ctrl, DENOISE3D_V20_MOTION_DILATE_ENABLE, tdnr->enable_motion_dilation);
	REG_SET_SLICE(isp_denoise3d2_ctrl, DENOISE3D_V20_NLM_ENABLE, dev->tdnr.enable_2dnr);
	//REG_SET_SLICE(isp_denoise3d2_ctrl, DENOISE3D_V20_NLM_ENABLE, 1);
	REG_SET_SLICE(isp_denoise3d2_ctrl, DENOISE3D_V20_TNR_ENABLE, dev->tdnr.enable_tnr);

	if (!dev->tdnr.enable_tnr) {
		REG_SET_SLICE(isp_denoise3d2_ctrl, DENOISE3D_V20_REF_RESET, 1);
	}
	REG_SET_SLICE(isp_denoise3d2_ctrl, DENOISE3D_V20_ENABLE, 1);

	isp_write_reg(dev, REG_ADDR(isp_denoise3d2_ctrl), isp_denoise3d2_ctrl);

	return 0;
}

int isp_tdnr_set_strength(struct isp_ic_dev *dev)
{
	u32 isp_denoise3d2_strength;
	struct isp_tdnr_context *tdnr = &dev->tdnr;
	pr_info("enter %s\n", __func__);

	isp_denoise3d2_strength = isp_read_reg(dev, REG_ADDR(isp_denoise3d2_strength));
	REG_SET_SLICE(isp_denoise3d2_strength, DENOISE3D_V20_TNR_STRENGTH, tdnr->strength);

	isp_write_reg(dev, REG_ADDR(isp_denoise3d2_strength), isp_denoise3d2_strength);

	return 0;
}

int isp_tdnr_set_motion(struct isp_ic_dev *dev)
{
	struct isp_tdnr_context *tdnr = &dev->tdnr;
	u32 isp_denoise3d2_motion_inv;
	pr_info("enter %s\n", __func__);

	isp_denoise3d2_motion_inv = isp_read_reg(dev, REG_ADDR(isp_denoise3d2_motion_inv));
	REG_SET_SLICE(isp_denoise3d2_motion_inv, DENOISE3D_V20_MOTION_INV, tdnr->motion_inv_factor);
	isp_write_reg(dev, REG_ADDR(isp_denoise3d2_motion_inv), isp_denoise3d2_motion_inv);

	return 0;
}

int isp_tdnr_enable(struct isp_ic_dev *dev)
{
	u32 regVal = 0;
	u32 strength = dev->tdnr.strength;
	pr_info("enter %s\n", __func__);
	regVal = isp_read_reg(dev, REG_ADDR(isp_denoise3d2_ctrl));
	REG_SET_SLICE(regVal, DENOISE3D_V20_ENABLE, 1);
	// REG_SET_SLICE(regVal, DENOISE3D_V20_REF_RESET, 1);
	REG_SET_SLICE(regVal, DENOISE3D_V20_TNR_ENABLE, 1);
	isp_write_reg(dev, REG_ADDR(isp_denoise3d2_ctrl), regVal);

	strength = MIN(MAX(strength, 0), 128);
	isp_write_reg(dev, REG_ADDR(isp_denoise3d2_strength), strength); //clear strength
	dev->tdnr.enable = true;

	return 0;
}

int isp_tdnr_disable(struct isp_ic_dev *dev)
{
	u32 regVal = 0;

	pr_info("enter %s\n", __func__);
	regVal = isp_read_reg(dev, REG_ADDR(isp_denoise3d2_ctrl));
	// REG_SET_SLICE(regVal, DENOISE3D_V20_ENABLE, 0);
	isp_write_reg(dev, REG_ADDR(isp_denoise3d2_ctrl), regVal);

	isp_write_reg(dev, REG_ADDR(isp_denoise3d2_strength), 0); //clear strength
	dev->tdnr.enable = false;

	return 0;
}

int isp_tdnr_enable_tdnr(struct isp_ic_dev *dev)
{
	u32 regVal = 0;

	pr_info("enter %s\n", __func__);
	regVal = isp_read_reg(dev, REG_ADDR(isp_denoise3d2_ctrl));
	REG_SET_SLICE(regVal, DENOISE3D_V20_TNR_ENABLE, 1);
	isp_write_reg(dev, REG_ADDR(isp_denoise3d2_ctrl), regVal);

	dev->tdnr.enable_tnr= true;

	return 0;
}

int isp_tdnr_disable_tdnr(struct isp_ic_dev *dev)
{
	u32 regVal = 0;

	pr_info("enter %s\n", __func__);
	regVal = isp_read_reg(dev, REG_ADDR(isp_denoise3d2_ctrl));
	REG_SET_SLICE(regVal, DENOISE3D_V20_TNR_ENABLE, 0);
	isp_write_reg(dev, REG_ADDR(isp_denoise3d2_ctrl), regVal);
	dev->tdnr.enable = false;

	return 0;
}

int isp_tdnr_enable_2dnr(struct isp_ic_dev *dev)
{
	//u32 regVal = 0;

	pr_info("enter %s\n", __func__);
	/*regVal = isp_read_reg(dev, REG_ADDR(isp_denoise3d2_ctrl));
	REG_SET_SLICE(regVal, DENOISE3D_V20_NLM_ENABLE, 1);
	isp_write_reg(dev, REG_ADDR(isp_denoise3d2_ctrl), regVal);*/

	dev->tdnr.enable_tnr= true;

	return 0;
}

int isp_tdnr_disable_2dnr(struct isp_ic_dev *dev)
{
	//u32 regVal = 0;

	pr_info("enter %s\n", __func__);
	/*regVal = isp_read_reg(dev, REG_ADDR(isp_denoise3d2_ctrl));
	REG_SET_SLICE(regVal, DENOISE3D_V20_NLM_ENABLE, 0);
	isp_write_reg(dev, REG_ADDR(isp_denoise3d2_ctrl), regVal);*/
	dev->tdnr.enable = false;

	return 0;
}



int isp_tdnr_g_stats(struct isp_ic_dev *dev, struct isp_tdnr_stats *stats)
{
	pr_info("enter %s\n", __func__);
	if (!dev || !stats) {
		return -EINVAL;
	}

	stats->bg_sum = isp_read_reg(dev, REG_ADDR(isp_denoise3d2_bg_val_sum));
	stats->motion_sum = isp_read_reg(dev, REG_ADDR(isp_denoise3d2_mo_val_sum));
	stats->bg_pixel_cnt = isp_read_reg(dev, REG_ADDR(isp_denoise3d2_bg_cnt));
	stats->motion_pixel_cnt = isp_read_reg(dev, REG_ADDR(isp_denoise3d2_mo_cnt));
	stats->frame_avg = isp_read_reg(dev, REG_ADDR(isp_denoise3d2_frame_avg));
	return 0;
}

int isp_tdnr_u_noise(struct isp_ic_dev *dev)
{
	u32 regVal = 0;
	struct isp_tdnr_context *tdnr = &dev->tdnr;
	pr_info("enter %s\n", __func__);

	regVal = 0;
	REG_SET_SLICE(regVal, DENOISE3D_V20_NOISE_LEVEL, tdnr->noise_level);
	REG_SET_SLICE(regVal, DENOISE3D_V20_NOISE_MEAN, tdnr->noise_mean);
	isp_write_reg(dev, REG_ADDR(isp_denoise3d2_noise), regVal);

	regVal = 0;
	REG_SET_SLICE(regVal, DENOISE3D_V20_NOISE_LEVEL, tdnr->noise_threshold);
	REG_SET_SLICE(regVal, DENOISE3D_V20_NOISE_MEAN, tdnr->motion_mean);
	isp_write_reg(dev, REG_ADDR(isp_denoise3d2_motion), regVal);

	return 0;

}

int isp_tdnr_u_thr(struct isp_ic_dev *dev)
{
	u32 regVal = 0;
	struct isp_tdnr_context *tdnr = &dev->tdnr;
	pr_info("enter %s\n", __func__);

	regVal = 0;
	REG_SET_SLICE(regVal, DENOISE3D_V20_THR_UPDATE, tdnr->update_factor);
	REG_SET_SLICE(regVal, DENOISE3D_V20_MOTION_THR_UPDATE, tdnr->motion_update_factor);
	isp_write_reg(dev, REG_ADDR(isp_denoise3d2_update), regVal);

	return 0;

}

int isp_tdnr_s_buf(struct isp_ic_dev *dev)
{
    struct isp_tdnr_buffer* buf = &dev->tdnr.buf;
    u32 in_width, in_height;
	u32 size, lval;
	u32 miv2_sp2_bus_id ;
	u32 miv2_sp2_fmt;
	u32 miv2_imsc2;

	u32 miv2_ctrl = isp_read_reg(dev, REG_ADDR(miv2_ctrl));
	u32 miv2_imsc = isp_read_reg(dev, REG_ADDR(miv2_imsc));
	u32 miv2_sp2_ctrl = isp_read_reg(dev, REG_ADDR(miv2_sp2_ctrl));
	u32 isp_mi_sp2_raw2_ctrl = isp_read_reg(dev, REG_ADDR(isp_mi_sp2_raw2_ctrl));
	pr_info("enter %s\n", __func__);

	/*

		ENABLE MIV2 SP2 RAW1 RAW2 WRITE/READ

	*/

	REG_SET_SLICE(miv2_ctrl, SP2_RAW_RDMA_PATH_ENABLE, 1);
	REG_SET_SLICE(miv2_ctrl, SP2_RAW_PATH_ENABLE, 1);
    miv2_ctrl |= 0x00100000;  // sp2_raw2_path_enable

	isp_write_reg(dev, REG_ADDR(miv2_ctrl), miv2_ctrl);

	in_width = isp_read_reg(dev, REG_ADDR(isp_out_h_size));
	in_height = isp_read_reg(dev, REG_ADDR(isp_out_v_size));

	lval = (in_width * 12 + 127)/ 128;
	lval <<= 4;
	size = in_height * lval;  //raw12 unaligned
	//write reference frame config
	isp_write_reg(dev, REG_ADDR(miv2_sp2_raw_base_ad_init), buf->pa_refer);
	isp_write_reg(dev, REG_ADDR(miv2_sp2_raw_size_init), size);
	isp_write_reg(dev, REG_ADDR(miv2_sp2_raw_offs_cnt_init), 0);
	isp_write_reg(dev, REG_ADDR(miv2_sp2_raw_llength), lval);
	isp_write_reg(dev, REG_ADDR(miv2_sp2_raw_pic_width), in_width);
	isp_write_reg(dev, REG_ADDR(miv2_sp2_raw_pic_height), in_height);
	isp_write_reg(dev, REG_ADDR(miv2_sp2_raw_pic_size), size);

	isp_write_reg(dev, REG_ADDR(miv2_sp2_dma_raw_pic_start_ad), buf->pa_refer);
	isp_write_reg(dev, REG_ADDR(miv2_sp2_dma_raw_pic_width), in_width);
	isp_write_reg(dev, REG_ADDR(miv2_sp2_dma_raw_pic_llength), lval);
	isp_write_reg(dev, REG_ADDR(miv2_sp2_dma_raw_pic_lval), lval);
	isp_write_reg(dev, REG_ADDR(miv2_sp2_dma_raw_pic_size), size);

	in_width /= 2;
	in_height /= 2;
	lval = (in_width); // RAW8 output
	size = in_height * lval;
	pr_info("%s 0x%08x\n", __func__, REG_ADDR(isp_mi_sp2_raw2_base_ad_init));
	isp_write_reg(dev, REG_ADDR(isp_mi_sp2_raw2_base_ad_init), buf->pa_motion);
	isp_write_reg(dev, REG_ADDR(isp_mi_sp2_raw2_size_init), size);
	isp_write_reg(dev, REG_ADDR(isp_mi_sp2_raw2_offs_cnt_init), 0);
	isp_write_reg(dev, REG_ADDR(isp_mi_sp2_raw2_llength), lval);
	isp_write_reg(dev, REG_ADDR(isp_mi_sp2_raw2_pic_width), in_width);
	isp_write_reg(dev, REG_ADDR(isp_mi_sp2_raw2_pic_height), in_height);
	isp_write_reg(dev, REG_ADDR(isp_mi_sp2_raw2_pic_size), size);

	isp_write_reg(dev, REG_ADDR(isp_mi_sp2_dma_raw2_pic_start_ad), buf->pa_motion);
	isp_write_reg(dev, REG_ADDR(isp_mi_sp2_dma_raw2_pic_width), in_width);
	isp_write_reg(dev, REG_ADDR(isp_mi_sp2_dma_raw2_pic_llength), lval);
	isp_write_reg(dev, REG_ADDR(isp_mi_sp2_dma_raw2_pic_lval), lval);  // align to 128
	isp_write_reg(dev, REG_ADDR(isp_mi_sp2_dma_raw2_pic_size), size);

#if 0
	isp_write_reg(dev, 0x5620, buf->pa_motion);
	isp_write_reg(dev, 0x5624, size);
	isp_write_reg(dev, 0x5628, 0);
	isp_write_reg(dev, 0x562c, lval);
	isp_write_reg(dev, 0x5630, in_width);
	isp_write_reg(dev, 0x5634, in_height);
	isp_write_reg(dev, 0x5638, size);

	isp_write_reg(dev, 0x5660, buf->pa_motion);
	isp_write_reg(dev, 0x5664, in_width);
	isp_write_reg(dev, 0x5668, lval);
	isp_write_reg(dev, 0x5680, lval);  // align to 128
	isp_write_reg(dev, 0x566c, size);
#endif
	miv2_sp2_bus_id = isp_read_reg(dev, REG_ADDR(miv2_sp2_bus_id));

	REG_SET_SLICE(miv2_sp2_bus_id, SP2_WR_ID_EN, 1);
	REG_SET_SLICE(miv2_sp2_bus_id, SP2_RD_ID_EN, 1);
	REG_SET_SLICE(miv2_sp2_bus_id, SP2_RD_BURST_LEN, 2); //sp2 rd burst lenghth 16
	REG_SET_SLICE(miv2_sp2_bus_id, SP2_BUS_SW_EN, 1);

	isp_write_reg(dev, REG_ADDR(miv2_sp2_bus_id), miv2_sp2_bus_id);

	miv2_sp2_fmt = isp_read_reg(dev, REG_ADDR(miv2_sp2_fmt));
	REG_SET_SLICE(miv2_sp2_fmt, SP2_WR_RAW_BIT, 2);  //raw12
	REG_SET_SLICE(miv2_sp2_fmt, SP2_WR_RAW_ALIGNED, 0);  //unaligned

	REG_SET_SLICE(miv2_sp2_fmt, SP2_RD_RAW_BIT, 2);   //raw12
	REG_SET_SLICE(miv2_sp2_fmt, SP2_RD_RAW_ALIGNED, 0);  //unaligned

	isp_write_reg(dev, REG_ADDR(miv2_sp2_fmt), miv2_sp2_fmt);
	isp_write_reg(dev,  REG_ADDR(isp_mi_sp2_raw2_fmt), 0);

	REG_SET_SLICE(miv2_sp2_ctrl, SP2_RD_RAW_CFG_UPDATE, 1);
	//REG_SET_SLICE(miv2_sp2_ctrl, SP2_RD_RAW_AUTO_UPDATE, 1);
	REG_SET_SLICE(miv2_sp2_ctrl, SP2_MI_CFG_UPD, 1);
	REG_SET_SLICE(miv2_sp2_ctrl, SP2_AUTO_UPDATE, 1);

	miv2_sp2_ctrl |= (SP2_INIT_BASE_EN_MASK | SP2_INIT_OFFSET_EN_MASK);
	//miv2_sp2_ctrl |= 0x2fa;
	isp_write_reg(dev, REG_ADDR(miv2_sp2_ctrl), miv2_sp2_ctrl);

	isp_mi_sp2_raw2_ctrl |= 0x23a;
	isp_write_reg(dev,  REG_ADDR(isp_mi_sp2_raw2_ctrl), isp_mi_sp2_raw2_ctrl);
	miv2_imsc |= SP2_RAW_FRAME_END_MASK|SP2_DMA_RAW_READY_MASK;

	isp_write_reg(dev, REG_ADDR(miv2_imsc), miv2_imsc);
	miv2_imsc2 = isp_read_reg(dev, REG_ADDR(miv2_imsc2));

	miv2_imsc2 |= SP2_RAW2_FRAME_END_MASK | SP2_RAW2_DMA_READY_MASK; //0x41000
	isp_write_reg(dev, REG_ADDR(miv2_imsc2), miv2_imsc2);

	isp_write_reg(dev, REG_ADDR(isp_denoise3d_ctrl), 0x480);

//	isp_write_reg(dev, 0x00000730, 0x00001edf);
	//isp_write_reg(dev, 0x000014ec, 0x04b30000);
	//isp_write_reg(dev, 0x000014f0, 0x0807e521);
	return 0;
}

int isp_r_tdnr_refer(struct isp_ic_dev *dev)
{
	struct isp_tdnr_buffer* buf = &dev->tdnr.buf;
	u32 miv2_ctrl = isp_read_reg(dev, REG_ADDR(miv2_ctrl));
	u32 miv2_imsc = isp_read_reg(dev, REG_ADDR(miv2_imsc));
	u32 miv2_sp2_ctrl = isp_read_reg(dev, REG_ADDR(miv2_sp2_ctrl));
	u32 isp_mi_sp2_raw2_ctrl = isp_read_reg(dev, REG_ADDR(isp_mi_sp2_raw2_ctrl));
	pr_info("enter %s\n", __func__);
	isp_write_reg(dev, REG_ADDR(miv2_sp2_dma_raw_pic_start_ad), buf->pa_refer);
	isp_write_reg(dev, REG_ADDR(isp_mi_sp2_dma_raw2_pic_start_ad), buf->pa_motion);

	REG_SET_SLICE(miv2_sp2_ctrl, SP2_RD_RAW_CFG_UPDATE, 1);
	REG_SET_SLICE(miv2_sp2_ctrl, SP2_RD_RAW_AUTO_UPDATE, 1);
	REG_SET_SLICE(miv2_sp2_ctrl, SP2_MI_CFG_UPD, 1);

	miv2_sp2_ctrl |= (SP2_INIT_BASE_EN_MASK | SP2_INIT_OFFSET_EN_MASK);
	miv2_sp2_ctrl |= 0xf0;
	isp_write_reg(dev, REG_ADDR(miv2_sp2_ctrl), miv2_sp2_ctrl);

	isp_mi_sp2_raw2_ctrl |= 0x238;

	isp_write_reg(dev, REG_ADDR(isp_mi_sp2_raw2_ctrl), isp_mi_sp2_raw2_ctrl);

	REG_SET_SLICE(miv2_ctrl, SP2_RAW_RDMA_PATH_ENABLE, 1);
	REG_SET_SLICE(miv2_ctrl, SP2_RAW_RDMA_START_CON, 1);
	REG_SET_SLICE(miv2_ctrl, SP2_RAW_RDMA_START, 1);
	REG_SET_SLICE(miv2_ctrl, SP2_RAW2_RDMA_START, 1);
	REG_SET_SLICE(miv2_ctrl, SP2_RAW2_RDMA_START_CON, 1);
	isp_write_reg(dev, REG_ADDR(miv2_ctrl), miv2_ctrl);

	miv2_imsc |= SP2_DMA_RAW_READY_MASK;
	isp_write_reg(dev, REG_ADDR(miv2_imsc), miv2_imsc);
	return 0;
}

int isp_r_tdnr_motion(struct isp_ic_dev *dev)
{
	struct isp_tdnr_buffer* buf = &dev->tdnr.buf;
	u32 miv2_ctrl = isp_read_reg(dev, REG_ADDR(miv2_ctrl));
	u32 isp_mi_sp2_raw2_ctrl = isp_read_reg(dev, REG_ADDR(isp_mi_sp2_raw2_ctrl));
	pr_info("enter %s\n", __func__);
	isp_write_reg(dev, REG_ADDR(isp_mi_sp2_dma_raw2_pic_start_ad), buf->pa_motion);

	REG_SET_SLICE(miv2_ctrl, SP2_RAW2_RDMA_START, 1);
	REG_SET_SLICE(miv2_ctrl, SP2_RAW2_RDMA_START_CON, 1);
	isp_write_reg(dev, REG_ADDR(miv2_ctrl), miv2_ctrl);

	isp_mi_sp2_raw2_ctrl |= 0x238;
	isp_write_reg(dev, REG_ADDR(isp_mi_sp2_raw2_ctrl), isp_mi_sp2_raw2_ctrl);

	return 0;
}
#endif
