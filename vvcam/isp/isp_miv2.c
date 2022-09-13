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

#ifdef ISP_MIV2

static void mi_set_slice(u32* val, u32 mask, u32 slice);
extern MrvAllRegister_t *all_regs;

static int getRawBit(u32 type, u32 *bit, u32 *len)
{
	*len = 16;
	switch (type) {
	case ISP_PICBUF_TYPE_RAW8:
		*bit = 0;
		*len = 8;
		break;
#if 0				/* normal process,  need pass type from engine. */
	case ISP_PICBUF_TYPE_RAW10:
		*bit = 1;
		break;
	case ISP_PICBUF_TYPE_RAW12:
		*bit = 2;
		break;
	case ISP_PICBUF_TYPE_RAW14:
		*bit = 3;
		break;
	case ISP_PICBUF_TYPE_RAW16:
		*bit = 4;
		break;
#else /* WA */
	case ISP_PICBUF_TYPE_RAW10:
	case ISP_PICBUF_TYPE_RAW12:
	case ISP_PICBUF_TYPE_RAW14:
	case ISP_PICBUF_TYPE_RAW16:
		*bit = 4;
		break;
#endif
	default:
		pr_err("unsupport raw formt: %d\n", type);
		return -1;
	}
	return 0;
}

static bool isYuv(int type)
{
	return (type == ISP_PICBUF_TYPE_YCbCr444) ||
	    (type == ISP_PICBUF_TYPE_YCbCr422) ||
	    (type == ISP_PICBUF_TYPE_YCbCr420) ||
	    (type == ISP_PICBUF_TYPE_YCbCr400);
}

static bool isRaw(u32 type)
{
	return (type == ISP_PICBUF_TYPE_RAW8) ||
	    (type == ISP_PICBUF_TYPE_RAW10) ||
	    (type == ISP_PICBUF_TYPE_RAW12) ||
	    (type == ISP_PICBUF_TYPE_RAW14) || (type == ISP_PICBUF_TYPE_RAW16);

}

static void set_rgb_buffer(struct isp_ic_dev *dev, struct isp_buffer_context *buf)
{
	u32 addr = buf->path == 0 ? REG_ADDR(miv2_mp_y_base_ad_init) :
	    (buf->path ==
	     1) ? REG_ADDR(miv2_sp1_y_base_ad_init) :
	    REG_ADDR(miv2_sp2_y_base_ad_init);
	if (buf->type == ISP_PICBUF_TYPE_RGB888) {
		isp_write_reg(dev, addr, (buf->addr_y & MP_Y_BASE_AD_MASK));
		isp_write_reg(dev, addr + 1 * 4,
			      (buf->size_y & MP_Y_SIZE_MASK));
		isp_write_reg(dev, addr + 2 * 4, 0);
		isp_write_reg(dev, addr + 7 * 4,
			      (buf->addr_cb & MP_CB_BASE_AD_MASK));
		isp_write_reg(dev, addr + 8 * 4,
			      (buf->size_cb & MP_CB_SIZE_MASK));
		isp_write_reg(dev, addr + 9 * 4, 0);
		isp_write_reg(dev, addr + 10 * 4,
			      (buf->addr_cr & MP_CR_BASE_AD_MASK));
		isp_write_reg(dev, addr + 11 * 4,
			      (buf->size_cr & MP_CR_SIZE_MASK));
		isp_write_reg(dev, addr + 12 * 4, 0);
	}
}
/*#define PP_LINE_ENTRY*/
#define PP_LINE_NUM		80*2

static void set_yuv_buffer(struct isp_ic_dev *dev, struct isp_buffer_context *buf)
{

	u32 addr = buf->path == 0 ? REG_ADDR(miv2_mp_y_base_ad_init) :
			   (buf->path == 1) ? REG_ADDR(miv2_sp1_y_base_ad_init) :
			   (buf->path == 2) ? REG_ADDR(miv2_sp2_y_base_ad_init) :
			   REG_ADDR(isp_mi_pp_y_base_ad_init);

	if (isYuv(buf->type)) {
		isp_write_reg(dev, addr, (buf->addr_y & MP_Y_BASE_AD_MASK));
		isp_write_reg(dev, addr + 1 * 4,
			      (buf->size_y & MP_Y_SIZE_MASK));
		isp_write_reg(dev, addr + 2 * 4, 0);
		isp_write_reg(dev, addr + 7 * 4,
			      (buf->addr_cb & MP_CB_BASE_AD_MASK));
		isp_write_reg(dev, addr + 8 * 4,
			      (buf->size_cb & MP_CB_SIZE_MASK));
		isp_write_reg(dev, addr + 9 * 4, 0);
		isp_write_reg(dev, addr + 10 * 4,
			      (buf->addr_cr & MP_CR_BASE_AD_MASK));
		isp_write_reg(dev, addr + 11 * 4,
			      (buf->size_cr & MP_CR_SIZE_MASK));
		isp_write_reg(dev, addr + 12 * 4, 0);


	}
}

static void set_raw_buffer(struct isp_ic_dev *dev, struct isp_buffer_context *buf)
{
	u32 addr = buf->path == ISP_MI_PATH_MP ? REG_ADDR(miv2_mp_raw_base_ad_init) :
			   (buf->path == ISP_MI_PATH_SP2_BP) ? REG_ADDR(miv2_sp2_raw_base_ad_init) :
			   (buf->path == ISP_MI_PATH_PP) ?REG_ADDR(isp_mi_pp_y_base_ad_init):
#ifdef ISP_MI_MCM_WR
			   (buf->path == ISP_MI_MCM_WR0) ?REG_ADDR(miv2_mcm_raw0_base_ad_init):
			   (buf->path == ISP_MI_MCM_WR1) ?REG_ADDR(miv2_mcm_raw1_base_ad_init):
#endif
#ifdef ISP_MI_HDR
			   (buf->path == ISP_MI_HDR_L) ?REG_ADDR(isp_mi_hdr_l_base_ad_init):
			   (buf->path == ISP_MI_HDR_S) ?REG_ADDR(isp_mi_hdr_s_base_ad_init):
			   (buf->path == ISP_MI_HDR_VS) ?REG_ADDR(isp_mi_hdr_vs_base_ad_init):
#endif
				0;
		uint32_t line_num = isp_read_reg(dev, REG_ADDR(mi_sp1_ppw_ycbcr_entry_line_num));
		if ((buf->path == ISP_MI_PATH_PP) && (line_num != 0)) {
			buf->addr_y = dev->pp_write.buf_addr;
			buf->size_y = dev->pp_write.buf_size;
		}
	/*pr_info("%s path %d type %d addr %08x line_num = %d buf_addr = 0x%x, buf_size = %d\n",
		__func__, buf->path, buf->type, addr, line_num, buf->addr_y, buf->size_y);*/
	if (isRaw(buf->type)) {
		if (addr != 0) {
			isp_write_reg(dev, addr,
					(buf->addr_y & MP_RAW_BASE_AD_MASK));
			isp_write_reg(dev, addr + 4,
					(buf->size_y & MP_RAW_SIZE_MASK));
			isp_write_reg(dev, addr + 2 * 4, 0);
		}
	}

}

int isp_set_buffer(struct isp_ic_dev *dev, struct isp_buffer_context *buf)
{
	if (!dev || !buf) {
		pr_err("NULL pointer %s\n", __func__);
		return -1;
	}

	set_yuv_buffer(dev, buf);
	set_raw_buffer(dev, buf);
	set_rgb_buffer(dev, buf);
	return 0;
}

static int calc_raw_lval(int width, int out_mode, int align_mode)
{
	u32 lval = 0;

	if (align_mode == ISP_MI_DATA_ALIGN_16BIT_MODE) {
		if ((out_mode == IC_MI_DATAMODE_RAW10) ||
		    (out_mode == IC_MI_DATAMODE_RAW12) ||
		    (out_mode == IC_MI_DATAMODE_RAW14)) {
			lval = (width + 7) / 8;
		}
	} else if (align_mode == ISP_MI_DATA_ALIGN_128BIT_MODE) {
		if (out_mode == IC_MI_DATAMODE_RAW10){
			lval = (width + 12 - 1)/12;
		}else if(out_mode == IC_MI_DATAMODE_RAW12){
			lval = (width + 10 - 1)/10;
		}else if(out_mode == IC_MI_DATAMODE_RAW14){
			lval = (width + 9 - 1)/9;
		}else{
			lval = (width + 128 - 1)/128;
		}
	} else {
		if (out_mode == IC_MI_DATAMODE_RAW10) {
			lval = (width * 10 + 127) / 128;
		} else if (out_mode == IC_MI_DATAMODE_RAW12) {
			lval = (width * 12 + 127) / 128;
		} else if (out_mode == IC_MI_DATAMODE_RAW14) {
			lval = (width * 14 + 127) / 128;
		} else if (out_mode == IC_MI_DATAMODE_RAW16) {
			lval = (width * 16 + 127) / 128;
		} else {
			lval = (width * 8 + 127) / 128;
		}
	}

	return lval;
}

struct isp_dma_path_context{
	u32 ctrl_addr;
	u32 fmt_addr;
	u32 bus_cfg_addr;
	u32 bus_id_addr;
	u32 mi_imsc_addr;
	u32 rd_fmt_align;
	u32 rd_raw_bit;
	u32 rd_cfg_upd;
	u32 rd_auto_upd;
	u32 pic_start_addr;
	u32 pic_width;
	u32 pic_height;
	u32 pic_llength;
	u32 pic_size;
	u32 pic_lval;
	u32 path_enable_mask;
	u32 path_dma_start_mask;
};

int isp_ioc_cfg_dma(struct isp_ic_dev *dev, void __user *args)
{
	u32 llength, miv2_imsc, miv2_ctrl,path_ctrl,path_fmt;
	struct isp_dma_context dma;
	//u32 path_bus_cfg;
	u32 bus_id;
	u32 path_rd_fmt_bit = 0;
	u8 id;
	struct isp_dma_path_context  dma_path_ctx[] =
#ifndef ISP_MI_PP_READ
		{{REG_ADDR(miv2_mcm_ctrl), REG_ADDR(miv2_mcm_fmt), REG_ADDR(miv2_mcm_bus_cfg), REG_ADDR(miv2_mcm_bus_id),
		REG_ADDR(miv2_imsc), MCM_RD_FMT_ALIGNED_MASK, MCM_RD_RAW_BIT_MASK, MCM_RD_CFG_UPD_MASK, MCM_RD_AUTO_UPDATE_MASK, \
		REG_ADDR(miv2_mcm_dma_raw_pic_start_ad), REG_ADDR(miv2_mcm_dma_raw_pic_width),    0,          \
		REG_ADDR(miv2_mcm_dma_raw_pic_llength), REG_ADDR(miv2_mcm_dma_raw_pic_size),                \
		REG_ADDR(miv2_mcm_dma_raw_pic_lval), MCM_RAW_RDMA_PATH_ENABLE_MASK, MCM_RAW_RDMA_START_MASK},
#ifdef ISP_MI_HDR
		{REG_ADDR(isp_mi_hdr_ctrl), REG_ADDR(isp_mi_hdr_fmt), 0, REG_ADDR(isp_mi_hdr_ret_bus_id), REG_ADDR(miv2_imsc2),               \
		HDR_RD_RAW_ALIGNED_MASK, HDR_RD_L_BIT_MASK, HDR_RD_RAW_CFG_UPDATE_MASK,HDR_RD_RAW_CFG_UPDATE_MASK,      \
		REG_ADDR(isp_mi_hdr_dma_l_base_ad_init), REG_ADDR(isp_mi_hdr_dma_pic_width), REG_ADDR(isp_mi_hdr_dma_pic_height),           \
		REG_ADDR(isp_mi_hdr_dma_l_llength), REG_ADDR(isp_mi_hdr_dma_l_size_init),                \
		REG_ADDR(isp_mi_hdr_dma_l_lval), 0, HDR_DMA_START_MASK},

		{REG_ADDR(isp_mi_hdr_ctrl), REG_ADDR(isp_mi_hdr_fmt), 0, REG_ADDR(isp_mi_hdr_ret_bus_id), REG_ADDR(miv2_imsc2),               \
		HDR_RD_RAW_ALIGNED_MASK, HDR_RD_S_BIT_MASK, HDR_RD_RAW_CFG_UPDATE_MASK,HDR_RD_RAW_CFG_UPDATE_MASK,      \
		REG_ADDR(isp_mi_hdr_dma_s_base_ad_init), REG_ADDR(isp_mi_hdr_dma_pic_width), REG_ADDR(isp_mi_hdr_dma_pic_height),       \
		REG_ADDR(isp_mi_hdr_dma_s_llength), REG_ADDR(isp_mi_hdr_dma_s_size_init),                \
		REG_ADDR(isp_mi_hdr_dma_s_lval), 0, HDR_DMA_START_MASK},

		{REG_ADDR(isp_mi_hdr_ctrl), REG_ADDR(isp_mi_hdr_fmt), 0, REG_ADDR(isp_mi_hdr_ret_bus_id), REG_ADDR(miv2_imsc2),               \
		HDR_RD_RAW_ALIGNED_MASK, HDR_RD_VS_BIT_MASK, HDR_RD_RAW_CFG_UPDATE_MASK,HDR_RD_RAW_CFG_UPDATE_MASK,      \
		REG_ADDR(isp_mi_hdr_dma_vs_base_ad_init), REG_ADDR(isp_mi_hdr_pic_width), REG_ADDR(isp_mi_hdr_dma_pic_height),       \
		REG_ADDR(isp_mi_hdr_dma_vs_llength), REG_ADDR(isp_mi_hdr_dma_vs_size_init),                \
		REG_ADDR(isp_mi_hdr_dma_vs_lval), 0, HDR_DMA_START_MASK}
#endif
		};
#else
		{{REG_ADDR(isp_mi_pp_ctrl), REG_ADDR(isp_mi_pp_fmt), 0, 0, REG_ADDR(miv2_imsc2),               \
		PP_RD_RAW_ALIGNED_MASK, PP_RD_RAW_BIT_MASK, PP_MI_CFG_UPD_MASK, PP_RD_YUV_CFG_UPDATE_MASK,      \
		REG_ADDR(isp_mi_pp_dma_y_pic_start_ad), REG_ADDR(isp_mi_pp_dma_y_pic_width),      0,        \
		REG_ADDR(isp_mi_pp_dma_y_pic_llength), REG_ADDR(isp_mi_pp_dma_y_pic_size),                \
		REG_ADDR(isp_mi_pp_dma_y_pic_lval), 0, PP_DMA_START_MASK}};
#endif
	if (dev == NULL || args == NULL) {
		pr_info("input wrong parameter %s\n", __func__);
		return -1;
	}

	pr_info("enter %s\n", __func__);

	viv_check_retval(copy_from_user(&dma, args, sizeof(dma)));
	id = dma.id;
	if ( id > ISP_MI_DMA_ID_MAX){
		pr_info("id config wrong %s\n", __func__);
		return -1;
	}
	path_fmt = isp_read_reg(dev,  dma_path_ctx[id].fmt_addr);
#ifdef ISP_MI_PP_READ
	path_fmt |= 0x80000000;
#endif
	miv2_imsc = isp_read_reg(dev, dma_path_ctx[id].mi_imsc_addr);
	miv2_ctrl = isp_read_reg(dev, REG_ADDR(miv2_ctrl));
	path_ctrl = isp_read_reg(dev,  dma_path_ctx[id].ctrl_addr);

	//if ( dma_path_ctx[id].bus_cfg_addr != 0) {
	//	path_bus_cfg = isp_read_reg(dev,  dma_path_ctx[id].bus_cfg_addr);
	//}
	if ( dma_path_ctx[id].bus_id_addr != 0) {
		bus_id = isp_read_reg(dev,  dma_path_ctx[id].bus_id_addr);
	}

	if ( dma.align == ISP_MI_DATA_UNALIGN_MODE ) {

		switch(dma.type){
			case ISP_PICBUF_TYPE_RAW10:
				llength = calc_raw_lval(dma.width,IC_MI_DATAMODE_RAW10, ISP_MI_DATA_UNALIGN_MODE);
				mi_set_slice(&path_fmt, dma_path_ctx[id].rd_raw_bit, 1);
				break;
			case ISP_PICBUF_TYPE_RAW12:
				llength = calc_raw_lval(dma.width,IC_MI_DATAMODE_RAW12,ISP_MI_DATA_UNALIGN_MODE);
				if (id >= ISP_MI_DMA_ID_HDR_L && id <= ISP_MI_DMA_ID_HDR_VS) {
					mi_set_slice(&path_fmt, dma_path_ctx[id].rd_raw_bit, 0);     //mi dma hdr stitch  raw only support raw12 and raw16
				} else {
					mi_set_slice(&path_fmt, dma_path_ctx[id].rd_raw_bit, 2);
				}
				break;
			case ISP_PICBUF_TYPE_RAW14:
				llength = calc_raw_lval(dma.width,IC_MI_DATAMODE_RAW14, ISP_MI_DATA_UNALIGN_MODE);
				mi_set_slice(&path_fmt, dma_path_ctx[id].rd_raw_bit, 3);
				break;
			case ISP_PICBUF_TYPE_RAW16:
				llength = calc_raw_lval(dma.width,IC_MI_DATAMODE_RAW16, ISP_MI_DATA_UNALIGN_MODE);
				if (id >= ISP_MI_DMA_ID_HDR_L && id <= ISP_MI_DMA_ID_HDR_VS) {
					mi_set_slice(&path_fmt, dma_path_ctx[id].rd_raw_bit, 1);     //mi dma hdr stitch  raw only support raw12 and raw16
				} else {
					mi_set_slice(&path_fmt, dma_path_ctx[id].rd_raw_bit, 4);
				}

				break;
			default:
				return -EFAULT;
		}

		llength <<= 4;
		mi_set_slice(&path_fmt, dma_path_ctx[id].rd_fmt_align,  dma.align);

	} else {
		getRawBit(dma.type, &path_rd_fmt_bit, &llength);        //The old version load all kinds of raw format with raw16 format except raw8
		llength = dma.width * llength / 8;
		if (id >= ISP_MI_DMA_ID_HDR_L && id <= ISP_MI_DMA_ID_HDR_VS) {
			path_rd_fmt_bit = 1;
		}
		mi_set_slice(&path_fmt, dma_path_ctx[id].rd_raw_bit, path_rd_fmt_bit);
	}

/*	if (llength != 8)
		REG_SET_SLICE(mcm_bus_cfg, MCM_RD_SWAP_RAW, 1);*/

	//path_ctrl |=  PP_RD_YUV_CFG_UPDATE_MASK;  //PP_INIT_OFFSET_EN_MASK | PP_INIT_BASE_EN_MASK |
	isp_write_reg(dev, dma_path_ctx[id].pic_start_addr, (MCM_DMA_RAW_PIC_START_AD_MASK & dma.base));
	isp_write_reg(dev,dma_path_ctx[id].pic_width, (MCM_DMA_RAW_PIC_WIDTH_MASK & dma.width));

	if (dma_path_ctx[id].pic_height) {
		isp_write_reg(dev,dma_path_ctx[id].pic_height, dma.height);
	}

	isp_write_reg(dev, dma_path_ctx[id].pic_llength, (MCM_DMA_RAW_PIC_LLENGTH_MASK & llength));
	isp_write_reg(dev,dma_path_ctx[id].pic_size, (MCM_DMA_RAW_PIC_SIZE_MASK & (llength * dma.height)));
	isp_write_reg(dev, dma_path_ctx[id].pic_lval, (MCM_DMA_RAW_PIC_WIDTH_MASK & llength));
	isp_write_reg(dev, dma_path_ctx[id].fmt_addr, path_fmt);

	//isp_write_reg(dev, REG_ADDR(miv2_mcm_bus_cfg), path_bus_cfg);
#ifdef  ISP_MI_PP_READ
	isp_write_reg(dev, REG_ADDR(mi_pp_dma_y_pic_height),  dma.height);
	isp_write_reg(dev, REG_ADDR(mi_pp_y_lval_bytes), llength);

    //    isp_write_reg(dev, 0x55c0,  dma.height);
    //    isp_write_reg(dev, 0x55c4,  dma.height);
    //    isp_write_reg(dev, 0x55c8,  dma.height);
    //    isp_write_reg(dev, 0x55cc, llength);

	isp_write_reg(dev, dma_path_ctx[id].mi_imsc_addr, miv2_imsc | PPR_DMA_READY_MASK);	/* enabled pp dma */
#else
	isp_write_reg(dev, dma_path_ctx[id].mi_imsc_addr, miv2_imsc | 0x01800025);	/* enabled jdp, sp2_raw, mp_raw, mcm */
#endif

	if(id == ISP_MI_DMA_ID_MCM_PP) {
		miv2_ctrl |= (dma_path_ctx[id].path_enable_mask );//| dma_path_ctx[id].path_dma_start_mask);
		//path_ctrl |= 0xfa;
		mi_set_slice(&path_ctrl, dma_path_ctx[id].rd_cfg_upd, 1);
		mi_set_slice(&path_ctrl,  dma_path_ctx[id].rd_auto_upd, 1);

		REG_SET_SLICE(bus_id, MCM_BUS_SW_EN, 1);
		REG_SET_SLICE(bus_id, MCM_RD_ID_EN, 1);
		if (dma_path_ctx[id].bus_id_addr) {
			isp_write_reg(dev, dma_path_ctx[id].bus_id_addr, bus_id);
		}
		isp_write_reg(dev,dma_path_ctx[id].ctrl_addr, path_ctrl);
		isp_write_reg(dev, REG_ADDR(miv2_ctrl), miv2_ctrl);
#ifdef ISP_MI_HDR
	} else {
		REG_SET_SLICE(path_fmt, HDR_RD_STR, dma.rd_wr_str);
		//config wr str, l,s,vs bit and wr raw aligned same with rd  str,l,s,vs bit and wr  raw aligned
		path_fmt |= ((path_fmt >> HDR_RD_STR_SHIFT) &0x3ff) ;
		isp_write_reg(dev, dma_path_ctx[id].fmt_addr, path_fmt);

		/*hdr mi dma path enable in mi hdr ctrl register*/
		path_ctrl |= (dma_path_ctx[id].path_enable_mask) ; //| dma_path_ctx[id].path_dma_start_mask);
		REG_SET_SLICE(path_ctrl, HDR_INIT_OFFSET_EN, 1);
		REG_SET_SLICE(path_ctrl, HDR_INIT_BASE_EN, 1);

		REG_SET_SLICE(bus_id, HDR_BUS_SW_EN, 1);
		REG_SET_SLICE(bus_id, HDR_RD_ID_EN, 1);
		if (dma_path_ctx[id].bus_id_addr) {
			isp_write_reg(dev, dma_path_ctx[id].bus_id_addr, bus_id);
		}
		isp_write_reg(dev,dma_path_ctx[id].ctrl_addr, path_ctrl);
#endif
	}


	return 0;

}
/* only support read raw */
int isp_ioc_start_dma_read(struct isp_ic_dev *dev, void __user *args)
{
	start_dma_path_t dma_path;
	 u32 mi_hdr_fmt;
	 u32 mi_path_ctrl;
	 u32 mi_hdr_ret_ctrl;
	 u32 rd_wr_str;
	if (dev == NULL || args == NULL) {
		pr_info("input wrong parameter %s\n", __func__);
		return -1;
	}

	pr_info("enter %s\n", __func__);
	viv_check_retval(copy_from_user(&dma_path, args, sizeof(dma_path)));
	if (dma_path == ISP_MI_DMA_PATH_MCM_PP) {
#ifndef ISP_MI_PP_READ
		mi_path_ctrl = isp_read_reg(dev,REG_ADDR(miv2_ctrl));
		REG_SET_SLICE(mi_path_ctrl, MCM_RAW_RDMA_START, 1);
		isp_write_reg(dev,REG_ADDR(miv2_ctrl), mi_path_ctrl);
#else
		mi_path_ctrl = isp_read_reg(dev,REG_ADDR(miv2_ctrl));
		REG_SET_SLICE(mi_path_ctrl, PP_DMA_START, 1);
		isp_write_reg(dev,REG_ADDR(miv2_ctrl), mi_path_ctrl);
#endif
	} else if (dma_path == ISP_MI_DMA_PATH_HDR) {

			isp_write_reg(dev, REG_ADDR(isp_mi_hdr_ret_bus_timeo), 0x00000001);   //disable bus time out interrupt

			mi_hdr_ret_ctrl = isp_read_reg(dev,REG_ADDR(isp_hdr_ret_ctrl));
			REG_SET_SLICE(mi_hdr_ret_ctrl, HDR_RT_VSYNC_POL, 1);
			REG_SET_SLICE(mi_hdr_ret_ctrl, HDR_RETIMING_ENABLE, 1);
			REG_SET_SLICE(mi_hdr_ret_ctrl, DUMP_MODE_EN, 1);
			mi_hdr_fmt = isp_read_reg(dev,REG_ADDR(isp_mi_hdr_fmt));
			rd_wr_str = (mi_hdr_fmt & HDR_RD_STR_MASK)>>HDR_RD_STR_SHIFT;
			if (rd_wr_str == 0) {
				REG_SET_SLICE(mi_hdr_ret_ctrl, EXPOSURE_COUNT, 1);
			} else if (rd_wr_str == 1|| rd_wr_str == 3) {
				REG_SET_SLICE(mi_hdr_ret_ctrl, EXPOSURE_COUNT, 2);
				if (rd_wr_str == 3) {
					REG_SET_SLICE(mi_hdr_ret_ctrl, L_VS_COMBINING_ENABLE, 1);
				}
			} else if (rd_wr_str == 2) {
				REG_SET_SLICE(mi_hdr_ret_ctrl, EXPOSURE_COUNT, 0);
			}
			isp_write_reg(dev, REG_ADDR(isp_mi_hdr_dma_start_by_lines), 0x360);//0x10); //the written lines count of the last appearing frame after which start dma read
			isp_write_reg(dev,REG_ADDR(isp_hdr_ret_ctrl), mi_hdr_ret_ctrl);
			mi_path_ctrl = isp_read_reg(dev,REG_ADDR(isp_mi_hdr_ctrl));
			REG_SET_SLICE(mi_path_ctrl, HDR_RD_RAW_CFG_UPDATE, 1);
			REG_SET_SLICE(mi_path_ctrl, HDR_RD_RAW_AUTO_UPDATE, 1);

			REG_SET_SLICE(mi_path_ctrl, HDR_DMA_START, 1);
			isp_write_reg(dev,REG_ADDR(isp_mi_hdr_ctrl), mi_path_ctrl);

	}
	return 0;
}

#define PATHNUM  ISP_MI_PATH_ID_MAX// hw related

// only config write bits,  SP2 read bit at 3dnr.c
// read defined is same as write
struct miv2_format_bit {
	u32 nyv, nv12;
	u32 raw_aligned, yuv_aligned;
	u32 raw_bit, yuv_str;
	u32 yuv_fmt, yuv_bit, jdp_fmt;
};

static struct miv2_format_bit fmt_bit[PATHNUM] = {
	{
		.nyv = 3 << 13, .nv12 = 1 << 12,
		.raw_aligned = 3 << 10,
		.yuv_aligned = 1 << 9,
		.raw_bit = 7 << 6,
		.yuv_str = 3 << 4,
		.yuv_fmt = 3 << 2,
		.yuv_bit = 1 << 1,
		.jdp_fmt = 1,
	},
	{
		.nyv = 3 << 7,
		.nv12 =  1 << 6,
		.yuv_aligned = 1 << 5,
		.yuv_str = 3 << 3,
		.yuv_fmt = 3 << 1,
		.yuv_bit = 1,
	},
	{
		.nyv = 3 << 12,
		.nv12 =  1 << 11,
		.raw_aligned = 3 << 9,
		.yuv_aligned = 1 << 8,
		.raw_bit = 7 << 5,
		.yuv_str = 3 << 3,
		.yuv_fmt = 3 << 1,
		.yuv_bit = 1,
	},
#ifdef ISP_MI_MCM_WR
	{
		.nyv =  0,
		.nv12 = 0,
		.raw_aligned = 3 << 14,
		.raw_bit = 3 << 4,
		.yuv_aligned = 0,
		.yuv_str = 0,
		.yuv_fmt = 0,
		.yuv_bit = 0,
	},
	{
		.nyv =  0,
		.nv12 = 0,
		.raw_aligned = 3 << 16,
		.raw_bit = 3 << 8,
		.yuv_aligned = 0,
		.yuv_str = 0,
		.yuv_fmt = 0,
		.yuv_bit = 0,
	},
#endif
#ifdef ISP_MI_PP_WRITE
	{
		.nyv =  3 << 7,
		.nv12 =   1 << 6,
		.raw_aligned = 3 << 14,
		.raw_bit = 7 << 11,
		.yuv_aligned = 1 << 5,
		.yuv_str = 3 << 3,
		.yuv_fmt = 3 << 1,
		.yuv_bit = 1,
	},
#endif
#ifdef ISP_MI_HDR
	{
		.nyv =  0,
		.nv12 = 0,
		.raw_aligned = 3 << 8,
		.raw_bit = 3 << 2,
		.yuv_aligned = 0,
		.yuv_str = 0,
		.yuv_fmt = 0,
		.yuv_bit = 0,
	},
	{
		.nyv =  0,
		.nv12 = 0,
		.raw_aligned = 3 << 8,
		.raw_bit = 3 << 4,
		.yuv_aligned = 0,
		.yuv_str = 0,
		.yuv_fmt = 0,
		.yuv_bit = 0,
	},
	{
		.nyv =  0,
		.nv12 = 0,
		.raw_aligned = 3 << 8,
		.raw_bit = 3 << 6,
		.yuv_aligned = 0,
		.yuv_str = 0,
		.yuv_fmt = 0,
		.yuv_bit = 0,
	},
#endif
};

static u32 bit_shift(u32 i) {
	u32 shift = 0;
	while(!(i&1)) {
		shift++;
		i >>= 1;
	}
	return shift;
}

void mi_set_slice(u32* val, u32 mask, u32 slice)
{
	// mp, sp1, sp2 mcm postpath have different masks.
	if (mask) {
		*val &= ~mask;
		*val |= (slice << bit_shift(mask));
	}
}

struct miv2_path_address {
	u32 bus_cfg_addr;
	u32 bus_id_addr;
	u32 bus_timeo_addr;			//axi bus time out waiting ctrl register
	u32 path_ctrl_addr;
	u32 format_addr;
	u32 y_length_addr;
	u32 raw_llength_addr;
	u32 raw_pic_width_addr;
	u32 raw_pic_height_addr;
	u32 raw_pic_size_addr;
	u32 ycbcr_enable_bit;
	u32 raw_enable_bit;
	u32 format_conv_ctrl;
	u32 wr_raw_swap_bit;
};

static void disable_bus_timeo_intr(struct isp_ic_dev *dev, u32 bus_addr)
{
	u32 val;
	pr_info("%s  bus timeo interrupt register addr 0x%08x\n", __func__, bus_addr);
	val = isp_read_reg(dev, bus_addr);
	REG_SET_SLICE(val, MP_BUS_TIMEO_INTERRUPT_DISABLE, 1);
	isp_write_reg(dev, bus_addr, val);
}

static void set_qos(struct isp_ic_dev *dev)
{
	pr_info("isp %s enter\n", __func__);

	isp_write_reg(dev, 0x1308, MI_QOS);
	isp_write_reg(dev, 0x130C, MI_QOS2);
	pr_info("isp %s exit\n", __func__);
}

static void set_data_path(int id, struct isp_mi_data_path_context *path,
		   struct isp_ic_dev *dev)
{
	u32 bus_cfg, bus_id;
	u32 format;
	u32 miv2_ctrl;
	u32 path_ctrl;
	u32 lval;
	u32 acq_proc;
	u32 mcm_bus_cfg = isp_read_reg(dev, REG_ADDR(miv2_mcm_bus_cfg));
	u32 conv_format_ctrl = 0;
	u32 y_length_addr;
	u32 y_llength = 0;
#ifdef ISP_MI_PP_WRITE
	u32 isp_ctrl;
#endif
	// please take care the register order
#if 0
    struct miv2_path_address  path_list[PATHNUM] = {
		{ 0x1318, 0x131c, 0x1310, 0x1314, 0x1330, 0x13a0, 0x13a4, 0x13a8, 0x13ac, 1, 0x0c6c },
		{ 0x1434, 0x1438, 0x142c, 0x1430, 0x144c, 0, 0, 0, 8, 0x106c },
		{ 0x14ec, 0x14f0, 0x14e4, 0x14e8, 0x1504, 0x1574, 0x1578, 0x157c, 0x1580, 0x10, 0x116c },
	};
#else
	//id 0 is mp path;id 1 is sp path;id 2 is sp2 path;
	//id 3 is post process path write;
	struct miv2_path_address  path_list[PATHNUM] = {
		{
			REG_ADDR(miv2_mp_bus_cfg), REG_ADDR(miv2_mp_bus_id), REG_ADDR(miv2_mp_bus_timeo), REG_ADDR(miv2_mp_ctrl),
			REG_ADDR(miv2_mp_fmt), REG_ADDR(miv2_mp_y_llength), REG_ADDR(miv2_mp_raw_llength),
			REG_ADDR(miv2_mp_raw_pic_width), REG_ADDR(miv2_mp_raw_pic_height), REG_ADDR(miv2_mp_raw_pic_size),
			MP_YCBCR_PATH_ENABLE_MASK, MP_RAW_PATH_ENABLE_MASK, REG_ADDR(mrsz_format_conv_ctrl), MP_WR_SWAP_RAW_MASK
		},
		{
			REG_ADDR(miv2_sp1_bus_cfg), REG_ADDR(miv2_sp1_bus_id), REG_ADDR(miv2_sp1_bus_timeo), REG_ADDR(miv2_sp1_ctrl),
			REG_ADDR(miv2_sp1_fmt), REG_ADDR(miv2_sp1_y_llength), 0,
			0, 0, 0,
			SP1_YCBCR_PATH_ENABLE_MASK, 0, REG_ADDR(srsz_phase_format_conv_ctr), 0,
		},
		{
			REG_ADDR(miv2_sp2_bus_cfg), REG_ADDR(miv2_sp2_bus_id), REG_ADDR(miv2_sp2_bus_timeo), REG_ADDR(miv2_sp2_ctrl),
			REG_ADDR(miv2_sp2_fmt), REG_ADDR(miv2_sp2_y_llength), REG_ADDR(miv2_sp2_raw_llength),
			REG_ADDR(miv2_sp2_raw_pic_width), REG_ADDR(miv2_sp2_raw_pic_height), REG_ADDR(miv2_sp2_raw_pic_size),
			SP2_YCBCR_PATH_ENABLE_MASK, SP2_RAW_PATH_ENABLE_MASK, REG_ADDR(srsz2_phase_format_conv_ctr),
			SP2_WR_SWAP_RAW_MASK
		},
#ifdef ISP_MI_MCM_WR
		{
			REG_ADDR(miv2_mcm_bus_cfg), REG_ADDR(miv2_mcm_bus_id), REG_ADDR(miv2_mcm_bus_timeo), REG_ADDR(miv2_mcm_ctrl),
			REG_ADDR(miv2_mcm_fmt), 0, REG_ADDR(miv2_mcm_raw0_llength),
			REG_ADDR(miv2_mcm_raw0_pic_width), REG_ADDR(miv2_mcm_raw0_pic_height), REG_ADDR(miv2_mcm_raw0_pic_size),
			0, MCM_RAW0_PATH_ENABLE_MASK, 0, MCM_WR0_SWAP_RAW_MASK
		},

		{
			REG_ADDR(miv2_mcm_bus_cfg), REG_ADDR(miv2_mcm_bus_id), REG_ADDR(miv2_mcm_bus_timeo), REG_ADDR(miv2_mcm_ctrl),
			REG_ADDR(miv2_mcm_fmt), 0, REG_ADDR(miv2_mcm_raw1_llength),
			REG_ADDR(miv2_mcm_raw1_pic_width), REG_ADDR(miv2_mcm_raw1_pic_height), REG_ADDR(miv2_mcm_raw1_pic_size),
			0, MCM_RAW1_PATH_ENABLE_MASK, 0, MCM_WR1_SWAP_RAW_MASK
		},
#endif
		{
			REG_ADDR(isp_mi_pp_data_swap), REG_ADDR(miv2_sp1_bus_id), REG_ADDR(miv2_sp1_bus_timeo), REG_ADDR(isp_mi_pp_ctrl),
			REG_ADDR(isp_mi_pp_fmt), REG_ADDR(isp_mi_pp_y_llength), REG_ADDR(mi_pp_y_lval_bytes),
			REG_ADDR(isp_mi_pp_y_pic_width), REG_ADDR(isp_mi_pp_y_pic_height), REG_ADDR(isp_mi_pp_y_pic_size),
			PP_WRITE_PATH_ENABLE_MASK, PP_WRITE_PATH_ENABLE_MASK, 0,PP_WR_SWAP_Y_MASK
		},
#ifdef ISP_MI_HDR
		{
			REG_ADDR(isp_mi_hdr_ret_bus_cfg), REG_ADDR(isp_mi_hdr_ret_bus_id), REG_ADDR(isp_mi_hdr_ret_bus_timeo),
			REG_ADDR(isp_mi_hdr_ctrl), REG_ADDR(isp_mi_hdr_fmt), 0, REG_ADDR(isp_mi_hdr_l_llength),
			REG_ADDR(isp_mi_hdr_pic_width), REG_ADDR(isp_mi_hdr_pic_height), REG_ADDR(isp_mi_hdr_l_size_init),
			0, HDR_WR_ENABLE_MASK, 0, HDR_WR_SWAP_RAW_MASK
		},

		{
			REG_ADDR(isp_mi_hdr_ret_bus_cfg), REG_ADDR(isp_mi_hdr_ret_bus_id), REG_ADDR(isp_mi_hdr_ret_bus_timeo),
			REG_ADDR(isp_mi_hdr_ctrl), REG_ADDR(isp_mi_hdr_fmt), 0, REG_ADDR(isp_mi_hdr_s_llength),
			REG_ADDR(isp_mi_hdr_pic_width), REG_ADDR(isp_mi_hdr_pic_height), REG_ADDR(isp_mi_hdr_s_size_init),
			0, HDR_WR_ENABLE_MASK, 0, HDR_WR_SWAP_RAW_MASK
		},
		{
			REG_ADDR(isp_mi_hdr_ret_bus_cfg), REG_ADDR(isp_mi_hdr_ret_bus_id), REG_ADDR(isp_mi_hdr_ret_bus_timeo),
			REG_ADDR(isp_mi_hdr_ctrl), REG_ADDR(isp_mi_hdr_fmt), 0, REG_ADDR(isp_mi_hdr_vs_llength),
			REG_ADDR(isp_mi_hdr_pic_width), REG_ADDR(isp_mi_hdr_pic_height), REG_ADDR(isp_mi_hdr_vs_size_init),
			0, HDR_WR_ENABLE_MASK, 0, HDR_WR_SWAP_RAW_MASK
		},
#endif

	};
#endif

	if (!path->enable) {
		disable_bus_timeo_intr(dev, path_list[id].bus_timeo_addr);
		return;
	}


	if (path->hscale || path->vscale || dev->is.enable) {
		if (id <= ISP_MI_PATH_SP2_BP)
			isp_set_scaling(id, dev, dev->is.enable, dev->crop[id].enabled);
		else
			isp_set_scaling(id, dev, dev->is.enable, 0);

	}

	miv2_ctrl = isp_read_reg(dev, REG_ADDR(miv2_ctrl));
	if(path_list[id].bus_cfg_addr)
	bus_cfg = isp_read_reg(dev, path_list[id].bus_cfg_addr);
	format = isp_read_reg(dev, path_list[id].format_addr);
	if (path_list[id].format_conv_ctrl) {
		conv_format_ctrl = isp_read_reg(dev, path_list[id].format_conv_ctrl);
	}
	pr_err("mi %s  id %d  fmt_bit[id].raw_bit 0x%08x miv2_ctrl 0x%08x ", __func__, id, path_list[id].raw_enable_bit, miv2_ctrl);

	path_ctrl = isp_read_reg(dev, path_list[id].path_ctrl_addr);
	switch (path->out_mode) {
	case IC_MI_DATAMODE_YUV444:
		mi_set_slice(&format, fmt_bit[id].yuv_fmt, 2);
		miv2_ctrl |= path_list[id].ycbcr_enable_bit;
		REG_SET_SLICE(conv_format_ctrl, MRV_MRSZ_COVERT_OUTPUT, 3);
		REG_SET_SLICE(conv_format_ctrl, MRV_MRSZ_COVERT_INPUT, 2);
		break;
	case IC_MI_DATAMODE_YUV422:
		mi_set_slice(&format, fmt_bit[id].yuv_fmt, 1);
		miv2_ctrl |= path_list[id].ycbcr_enable_bit;
		break;
	case IC_MI_DATAMODE_YUV420:
		mi_set_slice(&format, fmt_bit[id].yuv_fmt, 0);
		miv2_ctrl |= path_list[id].ycbcr_enable_bit;
		break;
	case IC_MI_DATAMODE_YUV400:
	case IC_MI_DATAMODE_JPEG:
		mi_set_slice(&format, fmt_bit[id].jdp_fmt, 1);
		REG_SET_SLICE(miv2_ctrl, MP_JDP_PATH_ENABLE, 1);
		break;
	case IC_MI_DATAMODE_RAW8:
		mi_set_slice(&format, fmt_bit[id].raw_bit, 0);
		miv2_ctrl |= path_list[id].raw_enable_bit;
		break;
	case IC_MI_DATAMODE_RAW10:
		mi_set_slice(&format, fmt_bit[id].raw_bit, 1);
		miv2_ctrl |= path_list[id].raw_enable_bit;

		mi_set_slice(&bus_cfg, path_list[id].wr_raw_swap_bit, 1);
		break;
	case IC_MI_DATAMODE_RAW12:
#ifdef ISP_MI_HDR
		if (id >= ISP_MI_HDR_L && id <= ISP_MI_HDR_VS) {
			mi_set_slice(&format, fmt_bit[id].raw_bit, 0);
			path_ctrl |= path_list[id].raw_enable_bit;
			//mi_set_slice(&bus_cfg, path_list[id].wr_raw_swap_bit, 1);
		}
#endif
		if (id <= ISP_MI_PATH_PP) {
			mi_set_slice(&format, fmt_bit[id].raw_bit, 2);
			miv2_ctrl |= path_list[id].raw_enable_bit;
			mi_set_slice(&bus_cfg, path_list[id].wr_raw_swap_bit, 1);
		}
		break;
	case IC_MI_DATAMODE_RAW14:
		mi_set_slice(&format, fmt_bit[id].raw_bit, 3);
		miv2_ctrl |= path_list[id].raw_enable_bit;

		mi_set_slice(&bus_cfg, path_list[id].wr_raw_swap_bit, 1);
		break;
	case IC_MI_DATAMODE_RAW16:
#ifdef ISP_MI_HDR
		if (id >= ISP_MI_HDR_L && id <= ISP_MI_HDR_VS) {
			mi_set_slice(&format, fmt_bit[id].raw_bit, 1);
			path_ctrl |= path_list[id].raw_enable_bit;
			//mi_set_slice(&bus_cfg, path_list[id].wr_raw_swap_bit, 1);
		}
#endif
		if (id <= ISP_MI_PATH_PP) {
			mi_set_slice(&format, fmt_bit[id].raw_bit, 4);
			miv2_ctrl |= path_list[id].raw_enable_bit;
			mi_set_slice(&bus_cfg, path_list[id].wr_raw_swap_bit, 1);
		}
		break;
	case IC_MI_DATAMODE_RGB888:
		mi_set_slice(&format, fmt_bit[id].yuv_fmt, 2);
		REG_SET_SLICE(conv_format_ctrl, MRV_MRSZ_COVERT_OUTPUT, 6);
		REG_SET_SLICE(conv_format_ctrl, MRV_MRSZ_COVERT_INPUT, 2);
		miv2_ctrl |= path_list[id].ycbcr_enable_bit;
		break;
	default:
		pr_err("mi %s unsupport format: %d", __func__, path->out_mode);
		return;
	}

	switch (path->data_layout) {
	case IC_MI_DATASTORAGE_PLANAR:
		mi_set_slice(&format, fmt_bit[id].yuv_str, 2);
		break;
	case IC_MI_DATASTORAGE_SEMIPLANAR:
		mi_set_slice(&format, fmt_bit[id].yuv_str, 0);
		break;
	case IC_MI_DATASTORAGE_INTERLEAVED:
		mi_set_slice(&format, fmt_bit[id].yuv_str, 1);
		break;
	default:
		break;
	}

	mi_set_slice(&format, fmt_bit[id].yuv_bit, path->yuv_bit);
	if(path->yuv_bit){
		if(path->data_alignMode){
			mi_set_slice(&format, fmt_bit[id].yuv_aligned, 1);
		}else{
			mi_set_slice(&format, fmt_bit[id].yuv_aligned, 0);
		}
		REG_SET_SLICE(conv_format_ctrl, MRV_MRSZ_COVERT_10_ENABLE, 1);
		REG_SET_SLICE(conv_format_ctrl, MRV_MRSZ_COVERT_10_METHOD, 0);
	}

	mi_set_slice(&format, fmt_bit[id].raw_aligned, path->data_alignMode);
	if (id == ISP_MI_PATH_MP) {
		REG_SET_SLICE(bus_cfg, MP_WR_BURST_LEN, dev->mi.burst_len);
	} else if (id == ISP_MI_PATH_SP) {
		REG_SET_SLICE(bus_cfg, SP1_WR_BURST_LEN, dev->mi.burst_len);
	} else if (id == ISP_MI_PATH_SP2_BP) {
		REG_SET_SLICE(bus_cfg, SP2_WR_BURST_LEN, dev->mi.burst_len);
	}
	REG_SET_SLICE(mcm_bus_cfg, MCM_WR_BURST_LEN, dev->mi.burst_len);

	if(path->yuv_bit){//
		if(path->data_alignMode){//aligned mode
			lval = (path->out_width + 12 - 1)/12;
			//printf("zw debug lval = 0x%x\n",lval);
		}else{                   //unaligned mode
			lval = (path->out_width * 10 + 127)/128;
		}
		y_llength =lval<<4;
		//printf("zw debug y_llength = 0x%x\n",y_llength);
	}else{    //8bit output
		y_llength = ALIGN_16BYTE(path->out_width);
    }

	y_length_addr = path_list[id].y_length_addr;
	if (y_length_addr) {
		isp_write_reg(dev, y_length_addr, y_llength);
		isp_write_reg(dev, y_length_addr + 4, path->out_width);
		isp_write_reg(dev, y_length_addr + 8, path->out_height);
		isp_write_reg(dev, y_length_addr + 12,
				y_llength * path->out_height);
	}
	// aev2, 3dnr
	if (id == ISP_MI_PATH_MP) {
		if (dev->exp2.enable) {
			REG_SET_SLICE(miv2_ctrl, MP_JDP_PATH_ENABLE, 1);
#ifdef ISP_AEV2_V2
			REG_SET_SLICE(format, MP_WR_JDP_DP_BIT, 1);
#endif
		} else {
			REG_SET_SLICE(miv2_ctrl, MP_JDP_PATH_ENABLE, 0);
		}
	}

	if ((id == ISP_MI_PATH_MP && (miv2_ctrl & MP_RAW_PATH_ENABLE_MASK))   ||
		(id == ISP_MI_PATH_SP2_BP && (miv2_ctrl & SP2_RAW_PATH_ENABLE_MASK))
#ifdef ISP_MI_MCM_WR
		|| (id == ISP_MI_MCM_WR0 && (miv2_ctrl & MCM_RAW0_PATH_ENABLE_MASK)) ||
		(id == ISP_MI_MCM_WR1 && (miv2_ctrl & MCM_RAW1_PATH_ENABLE_MASK))
#endif
#ifdef ISP_MI_PP_WRITE
		|| (id == ISP_MI_PATH_PP && ((path->out_mode == IC_MI_DATAMODE_RAW8) ||
		(path->out_mode == IC_MI_DATAMODE_RAW10) || (path->out_mode == IC_MI_DATAMODE_RAW12) ||
		(path->out_mode == IC_MI_DATAMODE_RAW14) || (path->out_mode == IC_MI_DATAMODE_RAW16)))
#endif
#ifdef ISP_MI_HDR
		|| (id >= ISP_MI_HDR_L && id <= ISP_MI_HDR_VS)
#endif
		) {
		lval =
			calc_raw_lval(path->out_width, path->out_mode,
				  path->data_alignMode);
		y_llength = lval <<4;
		isp_write_reg(dev, path_list[id].raw_llength_addr, y_llength);
		isp_write_reg(dev, path_list[id].raw_pic_width_addr, path->out_width);
		isp_write_reg(dev, path_list[id].raw_pic_height_addr, path->out_height);
		isp_write_reg(dev, path_list[id].raw_pic_size_addr, path->out_height * y_llength );
#ifdef ISP_MI_MCM_WR
		if (id == ISP_MI_MCM_WR0 || id == ISP_MI_MCM_WR1) {
			isp_write_reg(dev, REG_ADDR(miv2_mcm_raw0_lval_bytes), y_llength);
		}
#endif
#ifdef ISP_MI_PP_WRITE
		//ppw line entry mode, llength need to align with 256.
		//llength is line length, lval is line availble data.
#if 1/*should check by VV, if align with 256 is needed*/
		uint32_t line_num = isp_read_reg(dev, REG_ADDR(mi_sp1_ppw_ycbcr_entry_line_num));
        /*TODO shenweiyi to enable */
		// if (line_num != 0) {
		// 	y_llength = y_llength & 0xff ? (y_llength & 0xffffff00 + 0x100):y_llength;
		// }
		pr_info("%s:line_num = %d y_llength = 0x%x\n", __func__, line_num, y_llength);
		isp_write_reg(dev, REG_ADDR(isp_mi_pp_y_llength), y_llength);
		isp_write_reg(dev, path_list[id].raw_pic_size_addr, path->out_height * y_llength );
#else
		if (id == ISP_MI_PATH_PP) {
			uint32_t line_num = isp_read_reg(dev, REG_ADDR(mi_sp1_ppw_ycbcr_entry_line_num));
			if (line_num != 0) {
				y_llength = (y_llength & 0xff) ? ((y_llength + 0xff) &(~0xff)):y_llength;
			}
			isp_write_reg(dev, REG_ADDR(isp_mi_pp_y_llength), y_llength);
			isp_write_reg(dev, path_list[id].raw_pic_size_addr, path->out_height * y_llength );
		}
#endif
#endif
#if defined(ISP_MI_HDR)
		if (id >= ISP_MI_HDR_L && id <= ISP_MI_HDR_VS)
		isp_write_reg(dev, path_list[id].raw_llength_addr+4, y_llength);
#endif
	}

#ifdef ISP_MI_HDR
	if (id >= ISP_MI_HDR_L && id <= ISP_MI_HDR_VS) {
		REG_SET_SLICE(path_ctrl, HDR_MI_CFG_UPD, 1);
		REG_SET_SLICE(path_ctrl, HDR_AUTO_UPDATE, 1);
		path_ctrl |= (HDR_INIT_OFFSET_EN_MASK | HDR_INIT_BASE_EN_MASK);
	}
#endif
	if (id <= ISP_MI_PATH_PP) {
		REG_SET_SLICE(path_ctrl, MP_MI_CFG_UPD, 1);
		REG_SET_SLICE(path_ctrl, MP_AUTO_UPDATE, 1);
		//path_ctrl |= 0x05;
		path_ctrl |= (MP_INIT_BASE_EN_MASK | MP_INIT_OFFSET_EN_MASK);
	}
#ifdef ISP_MI_MCM_WR
	if (id <= ISP_MI_MCM_WR1 && id >= ISP_MI_MCM_WR0) {
		REG_SET_SLICE(path_ctrl, MCM_WR_CFG_UPD, 1);
		REG_SET_SLICE(path_ctrl, MCM_WR_AUTO_UPDATE, 1);
		//path_ctrl |= 0x05;
		path_ctrl |= (MCM_INIT_BASE_EN_MASK | MCM_INIT_OFFSET_EN_MASK);
	}
#endif

	pr_info("%s:path_ctrl 0x%08x\n", __func__, path_ctrl);
	acq_proc = isp_read_reg(dev, REG_ADDR(isp_acq_prop));
	isp_write_reg(dev, REG_ADDR(isp_acq_prop),
		      acq_proc & ~MRV_ISP_LATENCY_FIFO_SELECTION_MASK);

	bus_id = isp_read_reg(dev, path_list[id].bus_id_addr);
	if (id == ISP_MI_PATH_SP) {
		bus_id <<= 4;
	}
	bus_id |= MP_WR_ID_EN_MASK;
	if (id == ISP_MI_PATH_SP2_BP) {
		bus_id |= SP2_BUS_SW_EN_MASK;
		// REG_SET_SLICE(bus_cfg, SP2_WR_SWAP_Y, 1);
#ifdef ISP_MI_HDR
	} else if (id >= ISP_MI_HDR_L && id <= ISP_MI_HDR_VS ) {
		bus_id |= HDR_BUS_SW_EN_MASK;
#endif
	} else  {
		bus_id |= MP_BUS_SW_EN_MASK;
	}
	if (path_list[id].bus_id_addr)
	isp_write_reg(dev, path_list[id].bus_id_addr, bus_id);

	if (path_list[id].bus_cfg_addr)
	isp_write_reg(dev, path_list[id].bus_cfg_addr, bus_cfg);

	isp_write_reg(dev, REG_ADDR(miv2_mcm_bus_cfg), mcm_bus_cfg);
#ifdef ISP_MI_PP_WRITE
	isp_ctrl = isp_read_reg(dev, REG_ADDR(isp_ctrl));
	if (id == ISP_MI_PATH_PP && ((path->out_mode == IC_MI_DATAMODE_RAW8) ||
		  (path->out_mode == IC_MI_DATAMODE_RAW10) || (path->out_mode == IC_MI_DATAMODE_RAW12) ||
		  (path->out_mode == IC_MI_DATAMODE_RAW14) || (path->out_mode == IC_MI_DATAMODE_RAW16)))  {  //pp path output raw
		format |= PP_WR_RAW_SEL_MASK;
		REG_SET_SLICE(format, PP_WR_RAW_SEL, 1);
		REG_SET_SLICE(isp_ctrl, PP_WRITE_SEL, 0);
		isp_write_reg(dev, REG_ADDR(isp_ctrl), isp_ctrl);

	} else {
		REG_SET_SLICE(format, PP_WR_RAW_SEL, 0);
		REG_SET_SLICE(isp_ctrl, PP_WRITE_SEL, 1);
		isp_write_reg(dev, REG_ADDR(isp_ctrl), isp_ctrl);
	}
#endif
	isp_write_reg(dev, path_list[id].format_addr, format);
	isp_write_reg(dev, REG_ADDR(miv2_ctrl), miv2_ctrl);
	isp_write_reg(dev, path_list[id].path_ctrl_addr, path_ctrl);

	/*config qos for isp*/
	set_qos(dev);

	if (path_list[id].format_conv_ctrl)
		isp_write_reg(dev, path_list[id].format_conv_ctrl, conv_format_ctrl);

}

int isp_mi_start(struct isp_ic_dev *dev)
{
	int i;
	struct isp_mi_context mi = *(&dev->mi);
	u32 imsc, miv2_mcm_bus_id;
	pr_info("enter %s\n", __func__);

	miv2_mcm_bus_id = isp_read_reg(dev, REG_ADDR(miv2_mcm_bus_id));
	miv2_mcm_bus_id |= MCM_BUS_SW_EN_MASK;
	isp_write_reg(dev, REG_ADDR(miv2_mcm_bus_id), miv2_mcm_bus_id);

    for (i = 0; i < ISP_MI_PATH_ID_MAX; i++) {
        set_data_path(i, &mi.path[i], dev);
    }
    uint32_t line_num = isp_read_reg(dev, REG_ADDR(mi_sp1_ppw_ycbcr_entry_line_num));

    imsc = isp_read_reg(dev, REG_ADDR(miv2_imsc));
    if (line_num == 0) {
    isp_write_reg(dev, REG_ADDR(miv2_imsc),
		         imsc | (MP_YCBCR_FRAME_END_MASK | MP_RAW_FRAME_END_MASK |
			     WRAP_MP_Y_MASK | WRAP_MP_CB_MASK | WRAP_MP_CR_MASK |
			     WRAP_MP_RAW_MASK | WRAP_MP_JDP_MASK | MCM_RAW0_FRAME_END_MASK |
			     SP1_YCBCR_FRAME_END_MASK | WRAP_SP1_Y_MASK |MCM_RAW1_FRAME_END_MASK|
			     WRAP_SP1_CB_MASK | WRAP_SP1_CR_MASK |
			     SP2_YCBCR_FRAME_END_MASK | WRAP_SP2_Y_MASK |
			     WRAP_SP2_CB_MASK | WRAP_SP2_CR_MASK |
                 SP2_RAW_FRAME_END_MASK | MP_JDP_FRAME_END_MASK));
	} else {
    isp_write_reg(dev, REG_ADDR(miv2_imsc),
		         imsc | (MP_YCBCR_FRAME_END_MASK | MP_RAW_FRAME_END_MASK |
			     WRAP_MP_Y_MASK | WRAP_MP_CB_MASK | WRAP_MP_CR_MASK |
			     WRAP_MP_RAW_MASK | WRAP_MP_JDP_MASK | MCM_RAW0_FRAME_END_MASK |
			     MCM_RAW1_FRAME_END_MASK|
			     SP2_YCBCR_FRAME_END_MASK | WRAP_SP2_Y_MASK |
			     WRAP_SP2_CB_MASK | WRAP_SP2_CR_MASK |
                 SP2_RAW_FRAME_END_MASK | MP_JDP_FRAME_END_MASK));
	}


	//isp_write_reg(dev, REG_ADDR(miv2_imsc1), 0x7ffffff);
	isp_write_reg(dev, REG_ADDR(miv2_imsc1), 0);
#ifdef ISP_MI_PP_WRITE
	imsc = isp_read_reg(dev, REG_ADDR(miv2_imsc2));
    if (line_num == 0) {
		isp_write_reg(dev, REG_ADDR(miv2_imsc2),
			      imsc | ( PPW_U_BUF_FULL_MASK | PPW_Y_BUF_FULL_MASK |
				  PPW_V_BUF_FULL_MASK | PPR_Y_BUF_FULL_MASK | SP2_RAW2_W_BUF_FULL_MASK |
				  SP2_RAW2_R_BUF_FULL_MASK | HDR_W_BUF_FULL_MASK | HDR_R_BUF_FULL_MASK |
				  WRAP_SP2_RAW_MASK | WRAP_PPW_CR_MASK | WRAP_PPW_CB_MASK | //WRAP_PPW_Y_MASK |
				  SP2_RAW2_FRAME_END_MASK | PPW_FRAME_END_MASK | HDR_VS_DMA_READY_MASK |
	              HDR_S_DMA_READY_MASK | HDR_L_DMA_READY_MASK | HDR_L_DMA_READY_MASK |
				  WRAP_HDR_VS_MASK | WRAP_HDR_S_MASK | WRAP_HDR_L_MASK | HDR_VS_FRAME_END_MASK |
				  HDR_S_FRAME_END_MASK | HDR_L_FRAME_END_MASK | MI_RT_BUS_BUSERR_MASK |
				  MI_RT_BUS_TIMEO_MASK));
    } else {
		isp_write_reg(dev, REG_ADDR(miv2_imsc2),
				  imsc | (SP2_RAW2_W_BUF_FULL_MASK |
				  SP2_RAW2_R_BUF_FULL_MASK | HDR_W_BUF_FULL_MASK | HDR_R_BUF_FULL_MASK |
				  WRAP_SP2_RAW_MASK |PPW_FRAME_END_MASK|
				  SP2_RAW2_FRAME_END_MASK | HDR_VS_DMA_READY_MASK |
				  HDR_S_DMA_READY_MASK | HDR_L_DMA_READY_MASK | HDR_L_DMA_READY_MASK |
				  WRAP_HDR_VS_MASK | WRAP_HDR_S_MASK | WRAP_HDR_L_MASK | HDR_VS_FRAME_END_MASK |
				  HDR_S_FRAME_END_MASK | HDR_L_FRAME_END_MASK | MI_RT_BUS_BUSERR_MASK |
				  MI_RT_BUS_TIMEO_MASK));
	}
#endif
	isp_write_reg(dev, REG_ADDR(miv2_imsc3),  0x3f);
	return 0;
}

int isp_mi_stop(struct isp_ic_dev *dev)
{
	pr_info("enter %s\n", __func__);
#ifdef ISP_MI_PP_WRITE
	isp_write_reg(dev, REG_ADDR(miv2_imsc2), 0);
#endif
	isp_write_reg(dev, REG_ADDR(miv2_imsc3), 0);
	isp_write_reg(dev, REG_ADDR(miv2_imsc), 0);
	isp_write_reg(dev, REG_ADDR(miv2_imsc1), 0);
	isp_write_reg(dev, REG_ADDR(miv2_ctrl), 0UL);
	return 0;
}

u32 isp_read_mi_irq(struct isp_ic_dev *dev)
{
	return isp_read_reg(dev, REG_ADDR(miv2_mis));
}

void isp_reset_mi_irq(struct isp_ic_dev *dev, u32 icr)
{
	isp_write_reg(dev, REG_ADDR(miv2_icr), icr);
}

int isp_set_bp_buffer(struct isp_ic_dev *dev, struct isp_bp_buffer_context *buf)
{
	return 0;
}
#ifdef ISP_MI_PP_WRITE

int  isp_set_ppw_line_num(struct isp_ic_dev *dev)
{
	if (dev == NULL) {
		pr_err("Wrong input %s\n", __func__);
		return -1;
	}
	pr_info("enter %s\n", __func__);

	isp_write_reg(dev, REG_ADDR(mi_sp1_ppw_ycbcr_entry_line_num), dev->pp_write.entry_line_num);
	pr_info("exit %s\n", __func__);
	return 0;
}
int  isp_get_ppw_pic_cnt(struct isp_ic_dev *dev, u16* pic_cnt)
{

	if (dev == NULL) {
		pr_err("Wrong input %s\n", __func__);
		return -1;
	}
	pr_info("enter %s\n", __func__);
	*pic_cnt = isp_read_reg(dev, REG_ADDR(mi_sp1_ppw_ycbcr_entry_pic_cnt));
	pr_info("exit %s\n", __func__);
	return 0;
}
#endif

#ifdef ISP_MI_PP_READ

int  isp_cfg_pp_dma_line_entry(struct isp_ic_dev *dev)
{
	pp_dma_line_entry_t* pp_dam_line_entry = &dev->pp_dma_line_entry;
	if (dev == NULL) {
		pr_err("Wrong input %s\n", __func__);
		return -1;
	}
	pr_info("enter %s\n", __func__);
	isp_write_reg(dev, REG_ADDR(mi_pp_dma_y_entry_line_num), pp_dam_line_entry->entry_line_num);
	isp_write_reg(dev, REG_ADDR(mi_pp_dma_y_buf_line_num), pp_dam_line_entry->buf_line_num);
	pr_info("exit %s\n", __func__);
	return 0;
}
#endif
#endif
