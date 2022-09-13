/*
 * Copyright (C) 2021 Alibaba Group Holding Limited
 * Author: liuyitong <yitong.lyt@alibaba-inc.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef _THEAD_VIDEO_KERNEL_DEFS_H_
#define _THEAD_VIDEO_KERNEL_DEFS_H_

typedef enum sensor_path_type{
    SENSOR_VGA_RAW12_LINER = 0,
    SENSOR_1080P_RAW12_LINER,
    SENSOR_4K_RAW12_LINER,
    SENSOR_VGA_RAW10_LINER = 20,
    SENSOR_1080P_RAW10_LINER,
    SENSOR_4K_RAW10_LINER,
    SENSOR_2592x1944_RAW10_LINER,
    SENSOR_1280x720_RAW10_LINER,
    SENSOR_VGA_RAW12_HDR_2DOL,
    SENSOR_VGA_RAW12_HDR_3DOL,
    SENSOR_PATH_MAX,
} sensor_path_type_e;

typedef enum vipre_path_type{
    VIPRE_CSI0_DDR,
    VIPRE_CSI1_DDR,
    VIPRE_CSI2_DDR,
    VIPRE_CSI0_ISP0,
    VIPRE_CSI1_ISP0,
    VIPRE_CSI2_ISP0,
    VIPRE_CSI0_ISP1,
    VIPRE_CSI1_ISP1,
    VIPRE_CSI2_ISP1,
    VIPRE_CSI0_LOW_COAST_HDR_ISP0,
    VIPRE_CSI1_LOW_COAST_HDR_ISP0,
    VIPRE_CSI2_LOW_COAST_HDR_ISP0,
    VIPRE_PATH_MAX,

} vipre_path_type_e;

typedef enum isp_path_type{
    // 指示IP 内部进出的通路，也可表征内部数据对齐等方式
	ISP_MI_PATH_MP = 0,
	ISP_MI_PATH_SP,
	ISP_MI_PATH_SP2_BP,
#ifdef ISP_MI_MCM_WR
	ISP_MI_MCM_WR0,
	ISP_MI_MCM_WR1,
#endif
	ISP_MI_PATH_PP,
#ifdef ISP_MI_HDR
	ISP_MI_HDR_L,
	ISP_MI_HDR_S,
	ISP_MI_HDR_VS,
	ISP_MI_MAX,
#endif
} isp_path_type_e;

typedef enum ry_path_type{
	ISP_RY_MI_PATH_MP = 0,
	ISP_RY_MI_PATH_SP,
	ISP_RY_MI_PATH_SP2_BP,
	ISP_RY_PATH_MAX,
} isp_ry_path_type_e;

typedef enum dsp_path_type{
    DSP_PATH_ISP_RY = 0,
    DSP_PATH_ISP_CPU,
    DSP_PATH_VIPRE_DDR,

    DSP_PATH_VIPRE_EVEN,
    DSP_PATH_VIPRE_ODD,
    DSP_PATH_VIPRE_RY,
    DSP_PATH_CPU_CPU,
    DSP_PATH_MAX,
} dsp_path_type_e;

typedef enum dw_path_type{
    DW_DWE_VSE0 = 0,
    DW_DWE_VSE1,
    DW_DWE_VSE2,
    DW_PATH_MAX,
} dw_path_type_e;

typedef enum vedio_ip_type{
    SENSOR = 0,
    VIPRE,
    ISP,
    DW,
    RY,
    DSP,
}vedio_ip_type_e;

#endif /* _THEAD_VIDEO_KERNEL_DEFS_H_*/
