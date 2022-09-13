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

extern MrvAllRegister_t *all_regs;
enum {
    ISP_RGBIR_OUT_BAT_RGGB = 0,
    ISP_RGBIR_OUT_BAT_GRBG,
    ISP_RGBIR_OUT_BAT_GBRG,
    ISP_RGBIR_OUT_BAT_BGGR
};

enum {
    ISP_RGBIR_BAT_BGGIR = 0,
    ISP_RGBIR_BAT_GRIRG,
    ISP_RGBIR_BAT_RGGIR,
    ISP_RGBIR_BAT_GBIRG,
    ISP_RGBIR_BAT_GIRRG,
    ISP_RGBIR_BAT_IRGGB,
    ISP_RGBIR_BAT_GIRBG,
    ISP_RGBIR_BAT_IRGGR,
    ISP_RGBIR_BAT_RGIRB,
    ISP_RGBIR_BAT_GRBIR,
    ISP_RGBIR_BAT_IRBRG,
    ISP_RGBIR_BAT_BIRGR,
    ISP_RGBIR_BAT_BGIRR,
    ISP_RGBIR_BAT_GBRIR,
    ISP_RGBIR_BAT_IRRBG,
    ISP_RGBIR_BAT_RIRGB,
};

int isp_rgbir_s_bls(struct isp_ic_dev *dev)
{
#ifndef ISP_RGBIR
	isp_err("unsupported function %s", __func__);
	return -1;
#else
	struct isp_rgbir_context rgbir = *(&dev->rgbir);
    struct isp_rgbir_bls_context* bls = &rgbir.bls;
	isp_info("enter %s\n", __func__);

	isp_write_reg(dev, REG_ADDR(isp_pre_filt_bls_a), bls->a);
	isp_write_reg(dev, REG_ADDR(isp_pre_filt_bls_b), bls->b);
	isp_write_reg(dev, REG_ADDR(isp_pre_filt_bls_c), bls->c);
	isp_write_reg(dev, REG_ADDR(isp_pre_filt_bls_d), bls->d);
	return 0;
#endif
}

int isp_rgbir_s_gain(struct isp_ic_dev *dev)
{
#ifndef ISP_RGBIR
	isp_err("unsupported function %s", __func__);
	return -1;
#else

	struct isp_rgbir_context rgbir = *(&dev->rgbir);
    struct isp_rgbir_rgb_gain_context* rgb_gain = &rgbir.rgb_gain;
	isp_info("enter %s\n", __func__);

	isp_write_reg(dev, REG_ADDR(isp_pre_filt_gain_r), rgb_gain->r);
	isp_write_reg(dev, REG_ADDR(isp_pre_filt_gain_g), rgb_gain->g);
	isp_write_reg(dev, REG_ADDR(isp_pre_filt_gain_b), rgb_gain->b);

	return 0;
#endif
}

int isp_rgbir_s_dpcc(struct isp_ic_dev *dev)
{
#ifndef ISP_RGBIR
	isp_err("unsupported function %s", __func__);
	return -1;
#else
	struct isp_rgbir_dpcc_context *dpcc = &dev->rgbir.dpcc;
	int i;
    u32 isp_pre_filt_dpc_th_r;

	isp_info("enter %s\n", __func__);

	for (i = 0; i < 4; i++) {
        isp_pre_filt_dpc_th_r = isp_read_reg(dev, REG_ADDR(isp_pre_filt_dpc_th_r) + i * ISP_REG_GAP);
        REG_SET_SLICE(isp_pre_filt_dpc_th_r, ISP_PRE_FILT_DPC_TH_MED_R, dpcc->median_thr[i]);
        REG_SET_SLICE(isp_pre_filt_dpc_th_r, ISP_PRE_FILT_DPC_TH_AVG_R, dpcc->avg_thr[i]);

		isp_write_reg(dev, REG_ADDR(isp_pre_filt_dpc_th_r) + i * ISP_REG_GAP, isp_pre_filt_dpc_th_r);

	}
	return 0;
#endif
}

int isp_rgbir_s_cc_matrix(struct isp_ic_dev *dev)
{
#ifndef ISP_RGBIR
	isp_err("unsupported function %s", __func__);
	return -1;
#else
	struct isp_rgbir_cc_context *cc = &dev->rgbir.cc; //cc color correct
	int i;
    const u8 reg_gap = 8;
    u32 isp_pre_filt_cc_00_01;
	isp_info("enter %s\n", __func__);

	for (i = 0; i < 3; i++) {

        isp_pre_filt_cc_00_01 = isp_read_reg(dev, REG_ADDR(isp_pre_filt_cc_00_01) + i * reg_gap);
        REG_SET_SLICE(isp_pre_filt_cc_00_01, ISP_PRE_FILT_CC_00, (u32)(cc->mtx[i][0] ));
        REG_SET_SLICE(isp_pre_filt_cc_00_01, ISP_PRE_FILT_CC_01, (u32)(cc->mtx[i][1] ));
        isp_write_reg(dev, REG_ADDR(isp_pre_filt_cc_00_01) + i * reg_gap, isp_pre_filt_cc_00_01);

        isp_pre_filt_cc_00_01 = isp_read_reg(dev, REG_ADDR(isp_pre_filt_cc_00_01) + i * reg_gap + ISP_REG_GAP);
        REG_SET_SLICE(isp_pre_filt_cc_00_01, ISP_PRE_FILT_CC_02, (u32)(cc->mtx[i][2] ));
        REG_SET_SLICE(isp_pre_filt_cc_00_01, ISP_PRE_FILT_CC_03, (u32)(cc->mtx[i][3] ));
        isp_write_reg(dev, REG_ADDR(isp_pre_filt_cc_00_01) + (i  * reg_gap) + ISP_REG_GAP, isp_pre_filt_cc_00_01);
	}
	return 0;
#endif
}

int isp_rgbir_s_des(struct isp_ic_dev *dev)   //de-saturation
{
#ifndef ISP_RGBIR
	isp_err("unsupported function %s", __func__);
	return -1;
#else
	struct isp_rgbir_des_context *des = &dev->rgbir.des; //cc color correct
	int i;
    u32 isp_pre_filt_ir_des_pd1; // isp_pre_filt_ir_des_px1, isp_pre_filt_ir_des_py1;
    u32 isp_pre_filt_l_des_pd1; //  isp_pre_filt_l_des_px1, isp_pre_filt_l_des_py1;

	isp_info("enter %s\n", __func__);
	//register value
    for (i = 0 ; i < 4; i++) {
            isp_pre_filt_ir_des_pd1 = isp_read_reg(dev, REG_ADDR(isp_pre_filt_ir_des_pd1) + i * ISP_REG_GAP);
            if (i != 3) {
                isp_pre_filt_ir_des_pd1 = des->ir_pd[i *4] | des->ir_pd[i *4 + 1] << 8 |  des->ir_pd[i * 4 + 2] << 16 |\
                des->ir_pd[i*4 + 3] << 24;
            } else {
                isp_pre_filt_ir_des_pd1 = des->ir_pd[i *4] | des->ir_pd[i *4 + 1] << 8 |  des->ir_pd[i*4 + 2] << 16;
             }
            isp_write_reg(dev, REG_ADDR(isp_pre_filt_ir_des_pd1) + i * ISP_REG_GAP, isp_pre_filt_ir_des_pd1);
    }

	isp_write_reg(dev, REG_ADDR(isp_pre_filt_ir_des_px1), (des->ir_px1 & 0xffff));
	for (i = 0; i < 8; i++)
	{
		isp_write_reg(dev, REG_ADDR(isp_pre_filt_ir_des_py1) + i * ISP_REG_GAP, des->ir_py[i]);
	}

    for (i = 0 ; i < 4; i++) {
            isp_pre_filt_l_des_pd1 = isp_read_reg(dev, REG_ADDR(isp_pre_filt_l_des_pd1) + i * ISP_REG_GAP);
            if (i != 3) {
                isp_pre_filt_l_des_pd1 = des->l_pd[i *4] | des->l_pd[i*4 + 1] << 8 |  des->l_pd[i*4 + 2] << 16 |\
                des->l_pd[i*4 + 3] << 24;
            } else {
                isp_pre_filt_l_des_pd1 = des->l_pd[i *4] | des->l_pd[i*4 + 1] << 8 |  des->l_pd[i*4+2] << 16;
             }
            isp_write_reg(dev, REG_ADDR(isp_pre_filt_l_des_pd1) + i * ISP_REG_GAP, isp_pre_filt_l_des_pd1);
    }

	isp_write_reg(dev, REG_ADDR(isp_pre_filt_l_des_px1), (des->l_px1 & 0xffff));
	for ( i = 0; i < 8; i++)
	{
		isp_write_reg(dev, REG_ADDR(isp_pre_filt_l_des_py1) + i * 4, des->l_py[i]);
	}
	return 0;
#endif
}

int isp_rgbir_s_sharpen(struct isp_ic_dev *dev)
{
#ifndef ISP_RGBIR
	isp_err("unsupported function %s", __func__);
	return -1;
#else
	struct isp_rgbir_sharpen_context *sharpen = &dev->rgbir.sharpen;
     u8 sharpl = sharpen->sharpen_lvl;
    u32 isp_pre_filt_lum_weight; // isp_pre_filt_thresh_sh0, isp_pre_filt_thresh_sh1;
	// u32 isp_pre_filt_thresh_bl0, isp_pre_filt_thresh_bl1;
    // u32  isp_pre_filt_fac_sh0, isp_pre_filt_fac_sh1, isp_pre_filt_fac_mid;
    // u32  isp_pre_filt_fac_bl0, isp_pre_filt_fac_bl1;
	const unsigned int sharplevel[5][11] = {
		{ 0x4, 0x08, 0x0C, 0x10, 0x16, 0x1B, 0x20, 0x26, 0x2C, 0x30, 0x3F },
		{ 0x4, 0x07, 0x0A, 0x0C, 0x10, 0x14, 0x1A, 0x1E, 0x24, 0x2A, 0x30 },
		{ 0x4, 0x06, 0x08, 0x0A, 0x0C, 0x10, 0x13, 0x17, 0x1D, 0x22, 0x28 },
		{ 0x2, 0x02, 0x04, 0x06, 0x08, 0x0A, 0x0C, 0x10, 0x15, 0x1A, 0x24 },
		{ 0x0, 0x00, 0x00, 0x02, 0x04, 0x04, 0x06, 0x08, 0x0D, 0x14, 0x20 } };

	isp_info("enter %s\n", __func__);


	//filter factor sharp
    isp_pre_filt_lum_weight = isp_read_reg(dev, REG_ADDR(isp_pre_filt_lum_weight));

    REG_SET_SLICE(isp_pre_filt_lum_weight, LUM_WEIGHT_GAIN, sharpen->lum_weight_gain);
    REG_SET_SLICE(isp_pre_filt_lum_weight, LUM_WEIGHT_KINK, sharpen->lum_weight_kink);
    REG_SET_SLICE(isp_pre_filt_lum_weight, LUM_WEIGHT_MIN, sharpen->lum_weight_min);

	isp_write_reg(dev, REG_ADDR(isp_pre_filt_lum_weight), isp_pre_filt_lum_weight);
	isp_write_reg(dev, REG_ADDR(isp_pre_filt_fac_mid), sharpen->middle);

	isp_write_reg(dev, REG_ADDR(isp_pre_filt_thresh_sh0), sharpen->thresh_sh0);
	isp_write_reg(dev, REG_ADDR(isp_pre_filt_thresh_sh1), sharpen->thresh_sh1);
	isp_write_reg(dev, REG_ADDR(isp_pre_filt_thresh_bl0), sharpen->thresh_bl0);
	isp_write_reg(dev, REG_ADDR(isp_pre_filt_thresh_bl1), sharpen->thresh_bl1);

	isp_write_reg(dev, REG_ADDR(isp_pre_filt_fac_sh1), (unsigned int)(sharplevel[0][sharpl]));
	isp_write_reg(dev, REG_ADDR(isp_pre_filt_fac_sh0), (unsigned int)(sharplevel[1][sharpl]));
	isp_write_reg(dev, REG_ADDR(isp_pre_filt_fac_mid), (unsigned int)(sharplevel[2][sharpl]));
	isp_write_reg(dev, REG_ADDR(isp_pre_filt_fac_bl0), (unsigned int)(sharplevel[3][sharpl]));
	isp_write_reg(dev, REG_ADDR(isp_pre_filt_fac_bl1), (unsigned int)(sharplevel[4][sharpl]));
	return 0;
#endif
}

int isp_rgbir_s_ir_dnr(struct isp_ic_dev *dev)
{
#ifndef ISP_RGBIR
	isp_err("unsupported function %s", __func__);
	return -1;
#else
    struct isp_rgbir_ir_dnr_context* ir_dnr = &dev->rgbir.ir_dnr;
	//u16 imgwidth, imgheight;
	u32 ir_denoise_reg_03;
	u32 ir_denoise_reg_45;

	ir_denoise_reg_03 = (ir_dnr->winweight[0] | (ir_dnr->winweight[1] << 8) | (ir_dnr->winweight[2] << 16) | (ir_dnr->winweight[3] << 24));
	ir_denoise_reg_45 = (ir_dnr->winweight[4] | (ir_dnr->winweight[5] << 8));

	isp_write_reg(dev, REG_ADDR(isp_pre_filt_ir_denoise_sw_03), (unsigned int)(ir_denoise_reg_03));
	isp_write_reg(dev, REG_ADDR(isp_pre_filt_ir_denoise_sw_45), ((unsigned int)(ir_denoise_reg_45) & 0xffff));

	//imgwidth = isp_read_reg(dev, REG_ADDR(isp_out_h_size));
	//imgheight = isp_read_reg(dev, REG_ADDR(isp_out_v_size));
	isp_write_reg(dev, REG_ADDR(isp_pre_filt_h_size), ir_dnr->width& 0x7fff);
	isp_write_reg(dev, REG_ADDR(isp_pre_filt_v_size), ir_dnr->height & 0x3fff);
	return 0;
#endif
}

int isp_rgbir_hw_init(struct isp_ic_dev *dev)
{
#ifndef ISP_RGBIR
	isp_err("unsupported function %s", __func__);
	return -1;
#else
    u32  isp_pre_filt_ctrl;
	struct isp_rgbir_context *rgbir = &dev->rgbir;
    isp_info("enter %s\n", __func__);

    isp_rgbir_s_cc_matrix(dev);
    isp_rgbir_s_bls(dev);
    isp_rgbir_s_dpcc(dev);
    isp_rgbir_s_des(dev);
    isp_rgbir_s_gain(dev);
    isp_rgbir_s_ir_dnr(dev);
    isp_rgbir_s_sharpen(dev);

    isp_pre_filt_ctrl = isp_read_reg(dev, REG_ADDR(isp_pre_filt_ctrl));//pre filt ctrl
	REG_SET_SLICE(isp_pre_filt_ctrl , ISP_PRE_FILT_ENABLE, rgbir->prefilt_enable);
	REG_SET_SLICE(isp_pre_filt_ctrl , GREEN_FILT_MODE, rgbir->green_filt_mode);
	REG_SET_SLICE(isp_pre_filt_ctrl , GREEN_FILT_ENABLE, rgbir->green_filt_enable);
	REG_SET_SLICE(isp_pre_filt_ctrl , RGBIR_BAYER_PATTERN, rgbir->rgbirPattern );
	REG_SET_SLICE(isp_pre_filt_ctrl , OUT_RGB_BAYER_PATTERN, rgbir->out_rgb_pattern);
	REG_SET_SLICE(isp_pre_filt_ctrl ,  STAGE1_SELECT, rgbir->green_filt_stage1_select);
	REG_SET_SLICE(isp_pre_filt_ctrl ,  DEMOSAIC_THRESHOLD, rgbir->demosaic_threshold);
	REG_SET_SLICE(isp_pre_filt_ctrl ,  PART_ONE_ENABLE, rgbir->part1_enable);
	REG_SET_SLICE(isp_pre_filt_ctrl ,  PART_TWO_ENABLE,rgbir->part2_enable);

    isp_write_reg(dev, REG_ADDR(isp_pre_filt_ctrl), (unsigned int)(isp_pre_filt_ctrl));//pre filt ctrl
	return 0;
#endif
}

int isp_enable_rgbir(struct isp_ic_dev *dev)
{
#ifndef ISP_RGBIR
	isp_err("unsupported function %s", __func__);
	return -1;
#else
	struct isp_rgbir_context *rgbir = &dev->rgbir;
    u32 isp_pre_filt_ctrl;
    isp_info("enter %s\n", __func__);

    isp_pre_filt_ctrl = isp_read_reg(dev, REG_ADDR(isp_pre_filt_ctrl));//pre filt ctrl
	REG_SET_SLICE(isp_pre_filt_ctrl , ISP_PRE_FILT_ENABLE, rgbir->prefilt_enable);
    isp_write_reg(dev, REG_ADDR(isp_pre_filt_ctrl), isp_pre_filt_ctrl);//pre filt ctrl
	return 0;
#endif
}

int isp_rgbir_out_ir_raw(struct isp_ic_dev *dev)
{
#ifndef ISP_RGBIR
	isp_err("unsupported function %s", __func__);
	return -1;
#else
	struct isp_rgbir_context *rgbir = &dev->rgbir;
	u32 isp_ctrl;
    isp_info("enter %s\n", __func__);

    isp_ctrl = isp_read_reg(dev, REG_ADDR(isp_ctrl));//isp ctrl
	REG_SET_SLICE(isp_ctrl , ISP_IR_RAW_OUT, rgbir->enable_ir_raw_out);
	REG_SET_SLICE(isp_ctrl , MRV_ISP_ISP_MODE,  MRV_ISP_ISP_MODE_RGB); //there need to config rgb mode.
    isp_write_reg(dev, REG_ADDR(isp_ctrl), isp_ctrl);//isp_ctrl
	return 0;
#endif
}

int isp_s_rgbir(struct isp_ic_dev *dev)
{

#ifndef ISP_WDR_V4
	pr_err("unsupported function: %s", __func__);
	return -EINVAL;
#else
	struct isp_rgbir_context *rgbir = &dev->rgbir;
	u32 isp_pre_filt_ctrl;
    isp_info("enter %s\n", __func__);
    isp_rgbir_s_cc_matrix(dev);
    isp_rgbir_s_bls(dev);
    isp_rgbir_s_dpcc(dev);
    isp_rgbir_s_des(dev);
    isp_rgbir_s_gain(dev);
    isp_rgbir_s_ir_dnr(dev);
    isp_rgbir_s_sharpen(dev);

    isp_pre_filt_ctrl = isp_read_reg(dev, REG_ADDR(isp_pre_filt_ctrl));//pre filt ctrl
	REG_SET_SLICE(isp_pre_filt_ctrl , ISP_PRE_FILT_ENABLE, rgbir->prefilt_enable);
	REG_SET_SLICE(isp_pre_filt_ctrl , GREEN_FILT_MODE, rgbir->green_filt_mode);
	REG_SET_SLICE(isp_pre_filt_ctrl , GREEN_FILT_ENABLE, rgbir->green_filt_enable);
	REG_SET_SLICE(isp_pre_filt_ctrl , RGBIR_BAYER_PATTERN, rgbir->rgbirPattern );
	REG_SET_SLICE(isp_pre_filt_ctrl , OUT_RGB_BAYER_PATTERN, rgbir->out_rgb_pattern);
	REG_SET_SLICE(isp_pre_filt_ctrl ,  STAGE1_SELECT, rgbir->green_filt_stage1_select);
	REG_SET_SLICE(isp_pre_filt_ctrl ,  DEMOSAIC_THRESHOLD, rgbir->demosaic_threshold);
	REG_SET_SLICE(isp_pre_filt_ctrl ,  PART_ONE_ENABLE, rgbir->part1_enable);
	REG_SET_SLICE(isp_pre_filt_ctrl ,  PART_TWO_ENABLE,rgbir->part2_enable);

    isp_write_reg(dev, REG_ADDR(isp_pre_filt_ctrl), (unsigned int)(isp_pre_filt_ctrl));//pre filt ctrl
	return 0;
#endif
}


