/*
 * Copyright (c) 2021 Alibaba Group. All rights reserved.
 * License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#ifndef _THEAD_SUBDEV_H_
#define _THEAD_SUBDEV_H_

#ifdef __KERNEL__
#include <asm/io.h>
#include <linux/cdev.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/mm.h>

#include <linux/timer.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/ioctl.h>
#include <linux/poll.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/debugfs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/of.h>

typedef struct {
	char name[40];
	int path;
    int idx;
    int param_size;
    char param[400];
} sub_dev_info_t;

typedef struct {
    char subdev_name[40];
    int (*press)(struct fwnode_handle *node, sub_dev_info_t *subdev);
} subdev_dts_press_t;

int video_subdev_press_dts(struct fwnode_handle *node, sub_dev_info_t *subdev);

#endif  // __KERNEL__

typedef enum {
	VIPRE_MFRAME,
    VIPRE_NLINE,
} vipre_dam_mode_e;

typedef struct {
    vipre_dam_mode_e dma_mode;
} vipre_ipi_dts_arg_t;

typedef struct {
	// Converting refer to TuningRyChlCfg
	// Field					Default Value (init_dts_frame_cfg)	Sample
	uint32_t idx;				// default: 0						idx = <0>;
	uint32_t max_width;			// default: 0 (same as input)		max_width = <640>;
	uint32_t max_height;		// default: 0 (same as input)		max_height = <480>;
	uint32_t frame_count;		// default: 2						frame_count = <2>;
	uint32_t bit_per_pixel;		// default: 12						bit_per_pixel = 12;
	char     output_name[32];	// default: "unknown"				name = "isp0:output[0]";
} subdev_dts_frame_cfg_t;

typedef struct {
	subdev_dts_frame_cfg_t dts_path_frame_cfg;
} isp_dts_arg_t;

typedef struct {
	subdev_dts_frame_cfg_t dts_path_frame_cfg;
} ry_dts_arg_t;

typedef struct {
	uint32_t dw_dst_depth;
} dw_dts_arg_t;

typedef struct {
	subdev_dts_frame_cfg_t dts_path_frame_cfg;
} dsp_dts_arg_t;

#endif /*_THEAD_SUBDEV_H_*/
