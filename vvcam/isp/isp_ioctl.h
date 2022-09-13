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
#ifndef _ISP_IOC_H_
#define _ISP_IOC_H_
#include "ic_dev.h"
#include <linux/ioctl.h>
enum {
	ISPIOC_RESET				= 0x100,
	ISPIOC_WRITE_REG			= 0x101,
	ISPIOC_READ_REG 			= 0x102,
	ISPIOC_S_INPUT				= 0x103,
	ISPIOC_ENABLE				= 0x104,
	ISPIOC_DISABLE				= 0x105,
	ISPIOC_ISP_STATUS			= 0x106,
	ISPIOC_ISP_STOP 			= 0x107,
	ISPIOC_START_CAPTURE		= 0x108,
	ISPIOC_DISABLE_ISP_OFF		= 0x109,
	ISPIOC_SET_BUFFER			= 0x10A,
	ISPIOC_SET_BP_BUFFER		= 0x10B,
	ISPIOC_START_DMA_READ		= 0x10C,
	ISPIOC_MI_START 			= 0x10D,
	ISPIOC_MI_STOP				= 0x10E,
	ISPIOC_ENABLE_TPG			= 0x10F,
	ISPIOC_DISABLE_TPG			= 0x110,
	ISPIOC_S_TPG				= 0x111,
	ISPIOC_S_MCM_WR				= 0x112,
	ISPIOC_ENABLE_BLS			= 0x113,
	ISPIOC_DISABLE_BLS			= 0x114,
	ISPIOC_S_BLS				= 0x115,
	ISPIOC_S_MUX				= 0x116,
	ISPIOC_ENABLE_AWB			= 0x117,
	ISPIOC_DISABLE_AWB			= 0x118,
	ISPIOC_S_AWB				= 0x119,
	ISPIOC_G_AWBMEAN			= 0x11A,
	ISPIOC_S_IS 				= 0x11B,
	ISPIOC_S_RAW_IS 			= 0x11C,
	ISPIOC_S_CNR				= 0x11D,
	ISPIOC_S_CC 				= 0x11E,
	ISPIOC_S_XTALK				= 0x11F,
	ISPIOC_S_GAMMA_OUT			= 0x120,
	ISPIOC_ENABLE_LSC			= 0x121,
	ISPIOC_DISABLE_LSC			= 0x122,
	ISPIOC_S_LSC_TBL			= 0x123,
	ISPIOC_S_LSC_SEC			= 0x124,
	ISPIOC_S_DPF				= 0x125,
	ISPIOC_S_EE 				= 0x126,
	ISPIOC_S_EXP				= 0x127,
	ISPIOC_S_HDREXP				= 0x128,
	ISPIOC_G_EXPMEAN			= 0x129,
	ISPIOC_G_HDREXPMEAN			= 0x12A,
	ISPIOC_S_HIST				= 0x12B,
	ISPIOC_G_HISTMEAN			= 0x12C,
	ISPIOC_S_HDRHIST			= 0x12D,
	ISPIOC_G_HDRHISTMEAN        = 0x12E,
	ISPIOC_S_HIST64             = 0x12F,
	ISPIOC_G_HIST64MEAN         = 0x130,
	ISPIOC_G_HIST64VSTART_STATUS= 0x131,
	ISPIOC_U_HIST64 			= 0x132,
	ISPIOC_S_DPCC				= 0x133,
	ISPIOC_S_FLT				= 0x134,
	ISPIOC_S_CAC				= 0x135,
	ISPIOC_S_DEG				= 0x136,
	ISPIOC_S_AFM				= 0x137,
	ISPIOC_G_AFM				= 0x138,
	ISPIOC_S_VSM				= 0x139,
	ISPIOC_G_VSM				= 0x13A,
	ISPIOC_S_IE 				= 0x13B,
	ISPIOC_ENABLE_WDR3			= 0x13C,
	ISPIOC_DISABLE_WDR3 		= 0x13D,
	ISPIOC_U_WDR3				= 0x13E,
	ISPIOC_S_WDR3				= 0x13F,
	ISPIOC_S_EXP2               = 0x140,
	ISPIOC_S_EXP2_INPUTSEL      = 0x141,
	ISPIOC_S_EXP2_SIZERATIO     = 0x142,
	ISPIOC_S_2DNR				= 0x143,
	ISPIOC_S_3DNR				= 0x144,
	ISPIOC_G_3DNR				= 0x145, /* get last avg */
	ISPIOC_U_3DNR				= 0x146, /* update */
	ISPIOC_R_3DNR				= 0x147, /* read back 3dnr reference image. */
	ISPIOC_S_3DNR_CMP			= 0x148, /*config 3dnr compress */
	ISPIOC_U_3DNR_STRENGTH 		= 0x149,
	ISPIOC_S_3DNR_MOT			= 0x14A,  /*config 3dnr motion*/
	ISPIOC_S_3DNR_DLT  			= 0x14B,/*config 3dnr delta*/
	ISPIOC_S_HDR				= 0x14C,
	ISPIOC_S_COMP				= 0x14D,
	ISPIOC_S_CPROC				= 0x14E,
	ISPIOC_S_SIMP				= 0x14F,
	ISPIOC_S_ELAWB				= 0x150,
	ISPIOC_S_HDR_WB 			= 0x151,
	ISPIOC_S_HDR_BLS			= 0x152,
	ISPIOC_S_HDR_DIGITAL_GAIN	= 0x153,
	ISPIOC_ENABLE_WB			= 0x154,
	ISPIOC_DISABLE_WB			= 0x155,
	ISPIOC_DISABLE_HDR			= 0x156,
	ISPIOC_ENABLE_HDR			= 0x157,
	ISPIOC_ENABLE_GAMMA_OUT 	= 0x158,
	ISPIOC_DISABLE_GAMMA_OUT	= 0x159,
	ISPIOC_G_STATUS 			= 0x15A,
	ISPIOC_G_FEATURE			= 0x15B,
	ISPIOC_G_FEATURE_VERSION	= 0x15C,
	ISPIOC_ENABLE_GCMONO		= 0x15D,
	ISPIOC_DISABLE_GCMONO		= 0x15E,
	ISPIOC_S_GCMONO 			= 0x15F,
	ISPIOC_ENABLE_RGBGAMMA		= 0x160,
	ISPIOC_DISABLE_RGBGAMMA 	= 0x161,
	ISPIOC_S_RGBGAMMA			= 0x162,
	ISPIOC_S_DEMOSAIC			= 0x163,
	ISPIOC_S_DMSC_INTP			= 0x164,
	ISPIOC_S_DMSC_DMOI			= 0x165,
	ISPIOC_S_DMSC_SKIN			= 0x166,
	ISPIOC_S_DMSC_CAC			= 0x167,
	ISPIOC_S_DMSC_SHAP			= 0x168,
	ISPIOC_S_DMSC_SHAP_LINE		= 0x169,
	ISPIOC_S_DMSC_DEPURPLE      = 0x16A,
	ISPIOC_S_DMSC_GFILTER       = 0x16B,
	ISPIOC_S_DMSC				= 0x16C,
	ISPIOC_S_GREENEQUILIBRATE	= 0x16D,
	ISPIOC_S_COLOR_ADJUST		= 0x16E,
	ISPIOC_S_DIGITAL_GAIN		= 0x16F,
	ISPIOC_G_QUERY_EXTMEM		= 0x170,
#ifdef ISP_WDR_V4
	ISPIOC_ENABLE_WDR4          = 0x171,
	ISPIOC_DISABLE_WDR4         = 0x172,
	ISPIOC_U_WDR4               = 0x173,
	ISPIOC_S_WDR4               = 0x174,
#endif
	ISPIOC_WDR_CONFIG	    	= 0x175,
	ISPIOC_S_WDR_CURVE	    	= 0x176,
	ISPIOC_ENABLE_RGBIR	    	= 0x177,
	ISPIOC_S_RGBIR		    	= 0x178,
	ISPIOC_RGBIR_HW_INIT		= 0x179,
	ISPIOC_RGBIR_S_IR_DNR		= 0x17A,
	ISPIOC_RGBIR_S_SHARPEN		= 0x17B,
	ISPIOC_RGBIR_S_DES	    	= 0x17C,
	ISPIOC_RGBIR_S_CC_MATRIX	= 0x17D,
	ISPIOC_RGBIR_S_DPCC	    	= 0x17E,
	ISPIOC_RGBIR_S_GAIN	    	= 0x17F,
	ISPIOC_RGBIR_S_BLS	    	= 0x180,
	ISPIOC_RGBIR_S_IR_RAW_OUT	= 0x181,
	ISPIOC_S_TDNR               = 0x182,
	ISPIOC_TDNR_ENABLE  		= 0x183,
	ISPIOC_TDNR_DISABLE         = 0x184,
	ISPIOC_TDNR_ENABLE_TDNR     = 0x185,
	ISPIOC_TDNR_DISABLE_TDNR    = 0x186,
	ISPIOC_TDNR_ENABLE_2DNR     = 0x187,
	ISPIOC_TDNR_DISABLE_2DNR    = 0x188,
	ISPIOC_S_TDNR_CURVE         = 0x189,
	ISPIOC_G_TDNR               = 0x18A,
	ISPIOC_S_TDNR_STRENGTH      = 0x18B,
	ISPIOC_U_TDNR_NOISE         = 0x18C,
	ISPIOC_U_TDNR_THR           = 0x18D,
    ISPIOC_S_TDNR_BUF           = 0x18E,  // refer and motion
    ISPIOC_R_TDNR_REFER         = 0x18F,
    ISPIOC_R_TDNR_MOTION        = 0x190,
	ISPIOC_GET_MIS				= 0x191,
	ISPIOC_CFG_DMA				= 0x192,
	ISPIOC_BYPASS_MCM			= 0x193,
	ISPIOC_SET_PPW_LINE_NUM     = 0x194,
	ISPIOC_GET_PPW_LINE_CNT     = 0x195,
	ISPIOC_CFG_DMA_LINE_ENTRY   = 0x196,
	ISPIOC_S_CROP   			= 0x197,
    ISPIOC_GET_FRAME_MASK_INFO_ADDR = 0x198,
};

#define  ISP_GEN_CFG_UPDATE(dev)	{                                \
	uint32_t isp_ctrl = isp_read_reg(dev, REG_ADDR(isp_ctrl));       \
	REG_SET_SLICE(isp_ctrl, MRV_ISP_ISP_GEN_CFG_UPD, 1);			\
	isp_write_reg(dev, REG_ADDR(isp_ctrl), isp_ctrl);               \
	}

#define CONFIG_VSI_ISP_DEBUG 1
#ifdef CONFIG_VSI_ISP_DEBUG
#define isp_info(fmt, ...)  pr_info(fmt, ##__VA_ARGS__)
#define isp_debug(fmt, ...)  pr_debug(fmt, ##__VA_ARGS__)
#define isp_err(fmt, ...)  pr_err(fmt, ##__VA_ARGS__)
#else
#define isp_info(fmt, ...)
#define isp_debug(fmt, ...)
#define isp_err(fmt, ...)  pr_err(fmt, ##__VA_ARGS__)
#endif

#define ISP_REG_GAP 4
#define MI_QOS 0x44440444
#define MI_QOS2 0x44
long isp_priv_ioctl(struct isp_ic_dev *dev, unsigned int cmd, void __user *args);
long isp_copy_data(void *dst, void *src, int size);

/* internal functions, can called by v4l2 video device and ioctl */
int isp_reset(struct isp_ic_dev *dev);
int isp_enable_tpg(struct isp_ic_dev *dev);
int isp_disable_tpg(struct isp_ic_dev *dev);
int isp_enable_bls(struct isp_ic_dev *dev);
int isp_disable_bls(struct isp_ic_dev *dev);
int isp_enable(struct isp_ic_dev *dev);
int isp_disable(struct isp_ic_dev *dev);
bool is_isp_enable(struct isp_ic_dev *dev);
int isp_enable_lsc(struct isp_ic_dev *dev);
int isp_disable_lsc(struct isp_ic_dev *dev);
int isp_s_input(struct isp_ic_dev *dev);
int isp_s_digital_gain(struct isp_ic_dev *dev);
int isp_s_demosaic(struct isp_ic_dev *dev);
int isp_s_tpg(struct isp_ic_dev *dev);
int isp_s_mcm_wr(struct isp_ic_dev *dev);
int isp_bypass_mcm(struct isp_ic_dev *dev);
int isp_s_mux(struct isp_ic_dev *dev);
int isp_s_bls(struct isp_ic_dev *dev);
int isp_enable_awb(struct isp_ic_dev *dev);
int isp_disable_awb(struct isp_ic_dev *dev);
int isp_s_awb(struct isp_ic_dev *dev);
int isp_g_awbmean(struct isp_ic_dev *dev, struct isp_awb_mean *mean);
int isp_s_is(struct isp_ic_dev *dev);
int isp_s_raw_is(struct isp_ic_dev *dev);
int isp_s_cnr(struct isp_ic_dev *dev);
int isp_start_stream(struct isp_ic_dev *dev, u32 framenum);
int isp_stop_stream(struct isp_ic_dev *dev);
int isp_s_cc(struct isp_ic_dev *dev);
int isp_s_xtalk(struct isp_ic_dev *dev);
int isp_enable_wb(struct isp_ic_dev *dev, bool bEnable);
int isp_enable_gamma_out(struct isp_ic_dev *dev, bool bEnable);
int isp_s_gamma_out(struct isp_ic_dev *dev);
int isp_s_lsc_sec(struct isp_ic_dev *dev);
int isp_s_lsc_tbl(struct isp_ic_dev *dev);
int isp_ioc_disable_isp_off(struct isp_ic_dev *dev, void __user *args);
int isp_s_dpf(struct isp_ic_dev *dev);
int isp_s_ee(struct isp_ic_dev *dev);
int isp_s_exp(struct isp_ic_dev *dev);
int isp_s_hdrexp(struct isp_ic_dev *dev);
int isp_g_expmean(struct isp_ic_dev *dev, u8 * mean);
int isp_g_hdrexpmean(struct isp_ic_dev *dev, u8 * mean);
int isp_s_hist(struct isp_ic_dev *dev);
int isp_g_histmean(struct isp_ic_dev *dev, u32 * mean);
int isp_s_hdrhist(struct isp_ic_dev *dev);
int isp_g_hdrhistmean(struct isp_ic_dev *dev, u32 * mean);
#ifdef ISP_HIST64
int isp_s_hist64(struct isp_ic_dev *dev);
int isp_g_hist64mean(struct isp_ic_dev *dev, u32 *mean);
int isp_g_hist64_vstart(struct isp_ic_dev *dev, u32 start_line);
int isp_g_hist64_vstart_status(struct isp_ic_dev *dev, u32 *status);
int isp_update_hist64(struct isp_ic_dev *dev);
#endif
int isp_s_dpcc(struct isp_ic_dev *dev);
int isp_s_flt(struct isp_ic_dev *dev);
int isp_s_cac(struct isp_ic_dev *dev);
int isp_s_deg(struct isp_ic_dev *dev);
int isp_s_ie(struct isp_ic_dev *dev);
int isp_s_vsm(struct isp_ic_dev *dev);
int isp_g_vsm(struct isp_ic_dev *dev, struct isp_vsm_result *vsm);
int isp_s_afm(struct isp_ic_dev *dev);
int isp_g_afm(struct isp_ic_dev *dev, struct isp_afm_result *afm);
int isp_enable_wdr3(struct isp_ic_dev *dev);
int isp_disable_wdr3(struct isp_ic_dev *dev);
int isp_u_wdr3(struct isp_ic_dev *dev);
int isp_s_wdr3(struct isp_ic_dev *dev);
#ifdef ISP_WDR_V4
int isp_enable_wdr4(struct isp_ic_dev *dev);
int isp_disable_wdr4(struct isp_ic_dev *dev);
int isp_u_wdr4(struct isp_ic_dev *dev);
int isp_s_wdr4(struct isp_ic_dev *dev);
#endif
int isp_s_exp2(struct isp_ic_dev *dev);
int isp_s_exp2_inputsel(struct isp_ic_dev *dev);
int isp_s_exp2_sizeratio(struct isp_ic_dev *dev, u32 ratio);
int isp_s_hdr(struct isp_ic_dev *dev);
int isp_s_hdr_wb(struct isp_ic_dev *dev);
int isp_s_hdr_bls(struct isp_ic_dev *dev);
//int isp_s_hdr_digal_gain(struct isp_ic_dev *dev);
int isp_enable_hdr(struct isp_ic_dev *dev);
int isp_disable_hdr(struct isp_ic_dev *dev);
#ifdef ISP_2DNR_V5
int isp_tdnr_s_2dnr(struct isp_ic_dev *dev);	
#else
int isp_s_2dnr(struct isp_ic_dev *dev);
#endif
#if defined(ISP_3DNR) || defined(ISP_3DNR_V2)
int isp_s_3dnr(struct isp_ic_dev *dev);
int isp_g_3dnr(struct isp_ic_dev *dev, u32 * avg);
int isp_u_3dnr(struct isp_ic_dev *dev, struct isp_3dnr_update *dnr3_update);
int isp_s_3dnr_motion(struct isp_ic_dev *dev);
int isp_s_3dnr_delta(struct isp_ic_dev *dev);
#endif

#if defined(ISP_3DNR) || defined(ISP_3DNR_V2_V1)
int isp_r_3dnr(struct isp_ic_dev *dev);
#endif
#ifdef ISP_3DNR_V2
int isp_s_3dnr_cmp(struct isp_ic_dev *dev);
#endif
int isp_u_3dnr_strength(struct isp_ic_dev *dev);
int isp_s_comp(struct isp_ic_dev *dev);
int isp_s_simp(struct isp_ic_dev *dev);
int isp_s_cproc(struct isp_ic_dev *dev);
int isp_s_elawb(struct isp_ic_dev *dev);
int isp_ioc_qcap(struct isp_ic_dev *dev, void __user *args);
int isp_ioc_g_status(struct isp_ic_dev *dev, void __user *args);

int isp_enable_gcmono(struct isp_ic_dev *dev);
int isp_disable_gcmono(struct isp_ic_dev *dev);
int isp_s_gcmono(struct isp_ic_dev *dev, struct isp_gcmono_data *data);	/* set curve */
int isp_enable_rgbgamma(struct isp_ic_dev *dev);
int isp_disable_rgbgamma(struct isp_ic_dev *dev);
int isp_s_rgbgamma(struct isp_ic_dev *dev, struct isp_rgbgamma_data *data);

u32 isp_read_mi_irq(struct isp_ic_dev *dev);
void isp_reset_mi_irq(struct isp_ic_dev *dev, u32 icr);

int isp_ioc_cfg_dma(struct isp_ic_dev *dev, void __user *args);
int isp_ioc_start_dma_read(struct isp_ic_dev *dev, void __user *args);
int isp_mi_start(struct isp_ic_dev *dev);
int isp_mi_stop(struct isp_ic_dev *dev);
int isp_set_buffer(struct isp_ic_dev *dev, struct isp_buffer_context *buf);
int isp_set_bp_buffer(struct isp_ic_dev *dev,
		      struct isp_bp_buffer_context *buf);

int isp_enable_dmsc(struct isp_ic_dev *dev);
int isp_disable_dmsc(struct isp_ic_dev *dev);
int isp_set_dmsc_intp(struct isp_ic_dev *dev);
int isp_set_dmsc_skin(struct isp_ic_dev *dev);
int isp_set_dmsc_gfilter(struct isp_ic_dev *dev);
int isp_set_dmsc_depurple(struct isp_ic_dev *dev);
int isp_set_dmsc_cac(struct isp_ic_dev *dev);
int isp_set_dmsc_sharpen(struct isp_ic_dev *dev);
int isp_set_dmsc_sharpen_line(struct isp_ic_dev *dev);
int isp_set_dmsc_dmoi(struct isp_ic_dev *dev);
int isp_s_dmsc(struct isp_ic_dev *dev);
int isp_s_ge(struct isp_ic_dev *dev);
int isp_s_ca(struct isp_ic_dev *dev);
int isp_s_color_adjust(struct isp_ic_dev *dev);
int isp_config_dummy_hblank(struct isp_ic_dev *dev);

int isp_s_rgbir(struct isp_ic_dev *dev);
int isp_enable_rgbir(struct isp_ic_dev *dev);
int isp_rgbir_hw_init(struct isp_ic_dev *dev);
int isp_rgbir_s_ir_dnr(struct isp_ic_dev *dev);
int isp_rgbir_s_sharpen(struct isp_ic_dev *dev);
int isp_rgbir_s_des(struct isp_ic_dev *dev);
int isp_rgbir_s_cc_matrix(struct isp_ic_dev *dev);
int isp_rgbir_s_dpcc(struct isp_ic_dev *dev);
int isp_rgbir_s_gain(struct isp_ic_dev *dev);
int isp_rgbir_s_bls(struct isp_ic_dev *dev);
int isp_rgbir_out_ir_raw(struct isp_ic_dev *dev);

#ifdef ISP_3DNR_V3
int isp_tdnr_cfg_gamma(struct isp_ic_dev *dev);
int isp_s_tdnr(struct isp_ic_dev *dev);
int isp_tdnr_set_strength(struct isp_ic_dev *dev);
int isp_tdnr_set_motion(struct isp_ic_dev *dev);
int isp_tdnr_enable(struct isp_ic_dev *dev);
int isp_tdnr_disable(struct isp_ic_dev *dev);
int isp_tdnr_enable_tdnr(struct isp_ic_dev *dev);
int isp_tdnr_disable_tdnr(struct isp_ic_dev *dev);
int isp_tdnr_enable_2dnr(struct isp_ic_dev *dev);
int isp_tdnr_disable_2dnr(struct isp_ic_dev *dev);
int isp_tdnr_g_stats(struct isp_ic_dev *dev, struct isp_tdnr_stats *avg);
int isp_tdnr_u_noise(struct isp_ic_dev *dev);
int isp_r_tdnr_refer(struct isp_ic_dev *dev);
int isp_r_tdnr_motion(struct isp_ic_dev *dev);
int isp_tdnr_u_thr(struct isp_ic_dev *dev);
int isp_tdnr_s_buf(struct isp_ic_dev *dev);
#endif

#ifdef ISP_MI_PP_WRITE
int  isp_set_ppw_line_num(struct isp_ic_dev *dev);
int  isp_get_ppw_pic_cnt(struct isp_ic_dev *dev, u16* pic_cnt);
#endif

#ifdef ISP_MI_PP_READ
int  isp_cfg_pp_dma_line_entry(struct isp_ic_dev *dev);
#endif
#ifdef __KERNEL__
int clean_dma_buffer(struct isp_ic_dev *dev);
irqreturn_t isp_hw_isr(int irq, void *data);
void isp_clear_interrupts(struct isp_ic_dev *dev);
#endif
/*get irq mis value from store array*/
u32 isp_read_mis(struct isp_ic_dev *dev, u32 irq_src);
int isp_ioc_read_mis(struct isp_ic_dev *dev, void __user *args);

/*set scaler*/
int isp_set_scaling(int id, struct isp_ic_dev *dev, bool stabilization, bool crop);
int isp_set_crop(struct isp_ic_dev *dev);


int isp_ioc_g_feature(struct isp_ic_dev *dev, void __user *args);
int isp_ioc_g_feature_veresion(struct isp_ic_dev *dev, void __user *args);

#endif /* _ISP_IOC_H_ */
