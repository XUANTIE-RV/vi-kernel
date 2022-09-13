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
#include <linux/string.h>
#include "video.h"
#include "video_ioctl.h"
#include "subdev.h"

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(arr)     ( sizeof(arr) / sizeof((arr)[0]) )
#endif

static void dump_dts_frame_cfg(subdev_dts_frame_cfg_t *frame_cfg)
{
	video_info(">>>> dump_dts_frame_cfg begin >>>>\n");

	video_info("\t idx = %u\n", frame_cfg->idx);
	video_info("\t name = %s\n", frame_cfg->output_name);
	video_info("\t max_width = %u\n", frame_cfg->max_width);
	video_info("\t max_height = %u\n", frame_cfg->max_height);
	video_info("\t bit_per_pixel = %d\n", frame_cfg->bit_per_pixel);
	video_info("\t frame_count = %u\n", frame_cfg->frame_count);

	video_info("<<<< dump_dts_frame_cfg end   <<<<\n");
}

static void init_dts_frame_cfg(subdev_dts_frame_cfg_t *frame_cfg)
{
	if (frame_cfg == NULL) {
		video_err("frame_cfg is NULL\n");
		return;
	}

	frame_cfg->idx = -1;
	frame_cfg->max_width = 0;
	frame_cfg->max_height = 0;
	frame_cfg->bit_per_pixel = 12;
	frame_cfg->frame_count = 2;
	strcpy(frame_cfg->output_name, "unknown");
}

static int subdev_get_dts_frame_cfg(struct fwnode_handle *node, subdev_dts_frame_cfg_t *frame_cfg)
{
	if (frame_cfg == NULL) {
		video_err("frame_cfg is NULL\n");
		return -1;
	}

	fwnode_property_read_u32_array(node, "max_width", &frame_cfg->max_width, 1);
	fwnode_property_read_u32_array(node, "max_height", &frame_cfg->max_height, 1);
	fwnode_property_read_u32_array(node, "bit_per_pixel", &frame_cfg->bit_per_pixel, 1);
	fwnode_property_read_u32_array(node, "frame_count", &frame_cfg->frame_count, 1);
	return 0;
}

static int sensor_press_dts(struct fwnode_handle *node, sub_dev_info_t *subdev)
{
    //int ret = 0;
    //const char *path;
/*
    ret = fwnode_property_read_string(node, "path_type", &path);
    if (ret != 0) {
        video_err("sensor dts press error!!!!!, %s, %d\n", __func__, __LINE__);
        goto error;
    }
    */
    int ret = 0;

    uint32_t csi_idx, flash_led_idx;

    ret = fwnode_property_read_u32_array(node, "csi_idx", &csi_idx, 1);
    if(ret != 0) {
        return -1;
    }

    ret = fwnode_property_read_u32_array(node, "flash_led_idx", &flash_led_idx, 1);
    if(ret != 0) {
        flash_led_idx = -1;
    }

	video_info("flash_led idx is %u\n", flash_led_idx);

    subdev->param[0] = (char)csi_idx;
    subdev->param[1] = fwnode_property_read_bool(node, "skip_init");
    subdev->param[2] = (char)flash_led_idx;
    video_info("subdev->param[0] is %d\n", subdev->param[0]);
    video_info("subdev->param[2] is %d\n", subdev->param[2]);
    subdev->param_size = 3;
    return 0;
}

static int vipre_press_dts(struct fwnode_handle *node, sub_dev_info_t *subdev)
{
    //int ret = 0;
    //const char *path;
    vipre_ipi_dts_arg_t param;
    param.dma_mode = 111;
/*
    ret = fwnode_property_read_string(node, "path_type", &path);
    if (ret != 0) {
        video_err("sensor dts press error!!!!!, %s, %d\n", __func__, __LINE__);
        goto error;
    }
    */

    memcpy(subdev->param, &param, sizeof(vipre_ipi_dts_arg_t));
    subdev->param_size = sizeof(vipre_ipi_dts_arg_t);

    return 0;
}

static int isp_press_dts(struct fwnode_handle *node, sub_dev_info_t *subdev)
{
	int ret = 0;
	int idx;
	const char *path;

	isp_dts_arg_t param;
	struct fwnode_handle *child;

	subdev_dts_frame_cfg_t *dts_frame_cfg = &(param.dts_path_frame_cfg);
	init_dts_frame_cfg(dts_frame_cfg);

	child = fwnode_get_named_child_node(node, "output");
	if (child == NULL) {
		subdev->param_size = 0;
		return 0;   // return with all default values
	}

	ret = subdev_get_dts_frame_cfg(child, dts_frame_cfg);
	if (ret == 0) {
		fwnode_property_read_u32_array(node, "idx", &idx, 1);
		dts_frame_cfg->idx = idx;

		fwnode_property_read_string(node, "path_type", &path);
		snprintf(dts_frame_cfg->output_name, sizeof(dts_frame_cfg->output_name), "isp%d:%s", idx, path);

		dump_dts_frame_cfg(dts_frame_cfg);
	}
	else {
		video_err("%s, subdev_get_dts_frame_cfg() failed, ret=%d\n", __func__, ret);
		subdev->param_size = 0;
		return -1;
	}

	memcpy(subdev->param, &param, sizeof(subdev_dts_frame_cfg_t));
	subdev->param_size = sizeof(isp_dts_arg_t);

	return 0;
}

static int ry_press_dts(struct fwnode_handle *node, sub_dev_info_t *subdev)
{
	int ret = 0;
	int idx;
	const char *path;

	ry_dts_arg_t param;
	struct fwnode_handle *child;

	subdev_dts_frame_cfg_t *dts_frame_cfg = &(param.dts_path_frame_cfg);
	init_dts_frame_cfg(dts_frame_cfg);

	child = fwnode_get_named_child_node(node, "output");
	if (child == NULL) {
		subdev->param_size = 0;
		return 0;   // return with all default values
	}

	ret = subdev_get_dts_frame_cfg(child, dts_frame_cfg);
	if (ret == 0) {
		fwnode_property_read_u32_array(node, "idx", &idx, 1);
		dts_frame_cfg->idx = idx;

		fwnode_property_read_string(node, "path_type", &path);
		snprintf(dts_frame_cfg->output_name, sizeof(dts_frame_cfg->output_name), "ry%d:%s", idx, path);

		dump_dts_frame_cfg(dts_frame_cfg);
	}
	else {
		video_err("%s, subdev_get_dts_frame_cfg() failed, ret=%d\n", __func__, ret);
		subdev->param_size = 0;
		return -1;
	}

	memcpy(subdev->param, &param, sizeof(subdev_dts_frame_cfg_t));
	subdev->param_size = sizeof(ry_dts_arg_t);

	return 0;
}

static int dw_parse_dts(struct fwnode_handle *node, sub_dev_info_t *subdev)
{
	int idx;
	const char *path;
	int dw_dst_depth;

	dw_dts_arg_t param;

	dw_dst_depth = 2; // init

	fwnode_property_read_u32_array(node, "idx", &idx, 1);
	fwnode_property_read_string(node, "path_type", &path);
	fwnode_property_read_u32_array(node, "dw_dst_depth", &dw_dst_depth, 1);
	param.dw_dst_depth = dw_dst_depth;
	video_info(">>>> dump dw_dts_arg_t begin >>>>\n");
	video_info("\t idx = %d\n", idx);
	video_info("\t path_type = %s\n", path);
	video_info("\t dw_dst_depth = %d\n", dw_dst_depth);
	video_info("<<<< dump dw_dts_arg_t end   <<<<\n");

	memcpy(subdev->param, &param, sizeof(dw_dts_arg_t));
	subdev->param_size = sizeof(dw_dts_arg_t);

	return 0;
}

static int dsp_press_dts(struct fwnode_handle *node, sub_dev_info_t *subdev)
{
	int ret = 0;
	int idx;
	const char *path;

	dsp_dts_arg_t param;
	struct fwnode_handle *child;

	subdev_dts_frame_cfg_t *dts_frame_cfg = &(param.dts_path_frame_cfg);
	init_dts_frame_cfg(dts_frame_cfg);

	child = fwnode_get_named_child_node(node, "output");
	if (child == NULL) {
		subdev->param_size = 0;
		return 0;   // return with all default values
	}

	ret = subdev_get_dts_frame_cfg(child, dts_frame_cfg);
	if (ret == 0) {
		fwnode_property_read_u32_array(node, "idx", &idx, 1);
		dts_frame_cfg->idx = idx;

		fwnode_property_read_string(node, "path_type", &path);
		snprintf(dts_frame_cfg->output_name, sizeof(dts_frame_cfg->output_name), "dsp%d:%s", idx, path);

		dump_dts_frame_cfg(dts_frame_cfg);
	}
	else {
		video_err("%s, subdev_get_dts_frame_cfg() failed, ret=%d\n", __func__, ret);
		subdev->param_size = 0;
		return -1;
	}

	memcpy(subdev->param, &param, sizeof(dsp_dts_arg_t));
	subdev->param_size = sizeof(dsp_dts_arg_t);

	return 0;

}

static subdev_dts_press_t subdev_dts_press[] = {
    {.subdev_name = "vivcam", .press = sensor_press_dts},
    {.subdev_name = "vipre", .press = vipre_press_dts},
    {.subdev_name = "isp", .press = isp_press_dts},
    {.subdev_name = "ry", .press = ry_press_dts},
    {.subdev_name = "dw", .press = dw_parse_dts},
    {.subdev_name = "dsp", .press = dsp_press_dts},
};

int video_subdev_press_dts(struct fwnode_handle *node, sub_dev_info_t *subdev)
{
    int i = 0;
    for (i = 0; i < sizeof(subdev_dts_press) / sizeof(subdev_dts_press[0]); i++) {
        if (strcmp(subdev_dts_press[i].subdev_name, subdev->name) == 0) {
            return subdev_dts_press[i].press(node, subdev);
        }
    }

    return -1;
}
