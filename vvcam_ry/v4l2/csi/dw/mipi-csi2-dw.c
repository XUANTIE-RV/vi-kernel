/*
 * Copyright (C) 2021 Alibaba Group Holding Limited
 * Author: LuChongzhi <chongzhi.lcz@alibaba-inc.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/clk/clk-conf.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/videodev2.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-device.h>
#include <linux/reset.h>

#include "vvcsioc.h"
#include "mipi-csi2-dw.h"

#define CSIS_DRIVER_NAME		"mipi-csi2-dw"
#define CSIS_SUBDEV_NAME		"mipi-csi2-dw-subdev"
#define CSIS_MAX_ENTITIES		2
#define CSIS0_MAX_LANES			4
#define CSIS1_MAX_LANES			2

#define MIPI_CSIS_OF_NODE_NAME		"csi"

#define MIPI_CSIS_VC0_PAD_SINK		0
#define MIPI_CSIS_VC1_PAD_SINK		1
#define MIPI_CSIS_VC2_PAD_SINK		2
#define MIPI_CSIS_VC3_PAD_SINK		3

#define MIPI_CSIS_VC0_PAD_SOURCE	4
#define MIPI_CSIS_VC1_PAD_SOURCE	5
#define MIPI_CSIS_VC2_PAD_SOURCE	6
#define MIPI_CSIS_VC3_PAD_SOURCE	7
#define MIPI_CSIS_VCX_PADS_NUM		8

#define MIPI_CSIS_DEF_PIX_WIDTH		1920
#define MIPI_CSIS_DEF_PIX_HEIGHT	1080

#define CSI_HW_INTERRUPT

struct mipi_csis_event {
	u32 mask;
	const char *const name;
	unsigned int counter;
};

/**
 * struct csis_pix_format - CSIS pixel format description
 * @pix_width_alignment: horizontal pixel alignment, width will be
 *					   multiple of 2^pix_width_alignment
 * @code: corresponding media bus code
 * @fmt_reg: MIPI_CSIS_CONFIG register value
 * @data_alignment: MIPI-CSI data alignment in bits
 */
struct csis_pix_format {
	unsigned int pix_width_alignment;
	u32 code;
	u32 fmt_reg;
	u8 data_alignment;
};

struct csis_pktbuf {
	u32 *data;
	unsigned int len;
};

struct csis_hw_reset1 {
	struct regmap *src;
	u8 req_src;
	u8 rst_bit;
};

struct csi_state;
typedef int (*mipi_csis_phy_reset_t) (struct csi_state *state);

static const struct mipi_csis_event mipi_csis_events[] = {
	/* Errors */
	{MIPI_CSIS_INTSRC_ERR_SOT_HS, "SOT Error"},
	{MIPI_CSIS_INTSRC_ERR_LOST_FS, "Lost Frame Start Error"},
	{MIPI_CSIS_INTSRC_ERR_LOST_FE, "Lost Frame End Error"},
	{MIPI_CSIS_INTSRC_ERR_OVER, "FIFO Overflow Error"},
	{MIPI_CSIS_INTSRC_ERR_ECC, "ECC Error"},
	{MIPI_CSIS_INTSRC_ERR_CRC, "CRC Error"},
	{MIPI_CSIS_INTSRC_ERR_UNKNOWN, "Unknown Error"},
	/* Non-image data receive events */
	{MIPI_CSIS_INTSRC_EVEN_BEFORE, "Non-image data before even frame"},
	{MIPI_CSIS_INTSRC_EVEN_AFTER, "Non-image data after even frame"},
	{MIPI_CSIS_INTSRC_ODD_BEFORE, "Non-image data before odd frame"},
	{MIPI_CSIS_INTSRC_ODD_AFTER, "Non-image data after odd frame"},
	/* Frame start/end */
	{MIPI_CSIS_INTSRC_FRAME_START, "Frame Start"},
	{MIPI_CSIS_INTSRC_FRAME_END, "Frame End"},
};

#define MIPI_CSIS_NUM_EVENTS ARRAY_SIZE(mipi_csis_events)

/**
 * struct csi_state - the driver's internal state data structure
 * @lock: mutex serializing the subdev and power management operations,
 *		protecting @format and @flags members
 * @sd: v4l2_subdev associated with CSIS device instance
 * @index: the hardware instance index
 * @pdev: CSIS platform device
 * @phy: pointer to the CSIS generic PHY
 * @regs: mmaped I/O registers memory
 * @supplies: CSIS regulator supplies
 * @clock: CSIS clocks
 * @irq: requested s5p-mipi-csis irq number
 * @flags: the state variable for power and streaming control
 * @clock_frequency: device bus clock frequency
 * @hs_settle: HS-RX settle time
 * @clk_settle: Clk settle time
 * @num_lanes: number of MIPI-CSI data lanes used
 * @max_num_lanes: maximum number of MIPI-CSI data lanes supported
 * @wclk_ext: CSI wrapper clock: 0 - bus clock, 1 - external SCLK_CAM
 * @csis_fmt: current CSIS pixel format
 * @format: common media bus format for the source and sink pad
 * @slock: spinlock protecting structure members below
 * @pkt_buf: the frame embedded (non-image) data buffer
 * @events: MIPI-CSIS event (error) counters
 */
struct csi_state {
	struct v4l2_subdev sd;
	struct mutex lock;
	struct device *dev;
	struct v4l2_device *v4l2_dev;

	struct media_pad pads[MIPI_CSIS_VCX_PADS_NUM];

	u8 index;
	struct platform_device *pdev;
	struct phy *phy;
	void __iomem *regs;
	struct clk *mipi_clk;
	struct clk *phy_clk;
	struct clk *disp_axi;
	struct clk *disp_apb;
	int irq;
	u32 flags;

	u32 clk_frequency;
	u32 hs_settle;
	u32 clk_settle;
	u32 num_lanes;
	u32 max_num_lanes;
	u8 wclk_ext;

	u8 vchannel;
	const struct csis_pix_format *csis_fmt;
	struct v4l2_mbus_framefmt format;

	spinlock_t slock;
	struct csis_pktbuf pkt_buf;
	struct mipi_csis_event events[MIPI_CSIS_NUM_EVENTS];

	struct v4l2_async_subdev asd;
	struct v4l2_async_notifier subdev_notifier;
	struct v4l2_async_subdev *async_subdevs[2];

	struct csis_hw_reset1 hw_reset;
	struct regulator *mipi_phy_regulator;

	struct regmap *gasket;
	struct regmap *gpr;
	struct regmap *mix_gpr;
	struct reset_control *soft_resetn;
	struct reset_control *clk_enable;
	struct reset_control *mipi_reset;

	mipi_csis_phy_reset_t phy_reset_fn;
	bool hdr;
};

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-2)");

static const struct csis_pix_format mipi_csis_formats[] = {
	{
	 .code = MEDIA_BUS_FMT_SBGGR10_1X10,
	 .fmt_reg = MIPI_CSIS_ISPCFG_FMT_RAW10,
	 .data_alignment = 16,
	},
	{
	 .code = MEDIA_BUS_FMT_SGBRG10_1X10,
	 .fmt_reg = MIPI_CSIS_ISPCFG_FMT_RAW10,
	 .data_alignment = 16,
	},
	{
	 .code = MEDIA_BUS_FMT_SGRBG10_1X10,
	 .fmt_reg = MIPI_CSIS_ISPCFG_FMT_RAW10,
	 .data_alignment = 16,
	},
	{
	 .code = MEDIA_BUS_FMT_SRGGB10_1X10,
	 .fmt_reg = MIPI_CSIS_ISPCFG_FMT_RAW10,
	 .data_alignment = 16,
	},
	{
	 .code = MEDIA_BUS_FMT_SBGGR12_1X12,
	 .fmt_reg = MIPI_CSIS_ISPCFG_FMT_RAW12,
	 .data_alignment = 16,
	},
	{
	 .code = MEDIA_BUS_FMT_SGBRG12_1X12,
	 .fmt_reg = MIPI_CSIS_ISPCFG_FMT_RAW12,
	 .data_alignment = 16,
	},
	{
	 .code = MEDIA_BUS_FMT_SGRBG12_1X12,
	 .fmt_reg = MIPI_CSIS_ISPCFG_FMT_RAW12,
	 .data_alignment = 16,
	},
	{
	 .code = MEDIA_BUS_FMT_SRGGB12_1X12,
	 .fmt_reg = MIPI_CSIS_ISPCFG_FMT_RAW12,
	 .data_alignment = 16,
	},
	{
	 .code = MEDIA_BUS_FMT_YUYV8_2X8,
	 .fmt_reg = MIPI_CSIS_ISPCFG_FMT_YCBCR422_8BIT,
	 .data_alignment = 16,
	 },
	{
	 .code = MEDIA_BUS_FMT_RGB888_1X24,
	 .fmt_reg = MIPI_CSIS_ISPCFG_FMT_RGB888,
	 .data_alignment = 24,
	 },
	{
	 .code = MEDIA_BUS_FMT_YUYV8_2X8,
	 .fmt_reg = MIPI_CSIS_ISPCFG_FMT_YCBCR422_8BIT,
	 .data_alignment = 16,
	 },
	{
	 .code = MEDIA_BUS_FMT_VYUY8_2X8,
	 .fmt_reg = MIPI_CSIS_ISPCFG_FMT_YCBCR422_8BIT,
	 .data_alignment = 16,
	 },
	{
	 .code = MEDIA_BUS_FMT_SBGGR8_1X8,
	 .fmt_reg = MIPI_CSIS_ISPCFG_FMT_RAW8,
	 .data_alignment = 8,
	 }
};

typedef int (*mipi_csis_phy_reset_t) (struct csi_state *state);

#define mipi_csis_write(__csis, __r, __v) writel(__v, __csis->regs + __r)
#define mipi_csis_read(__csis, __r) readl(__csis->regs + __r)

static void dump_csis_regs(struct csi_state *state, const char *label)
{
	v4l2_dbg(2, debug, &state->sd, "--- %s ---\n", label);
	// TODO:
}

static void dump_gasket_regs(struct csi_state *state, const char *label)
{
	v4l2_dbg(2, debug, &state->sd, "--- %s ---\n", label);
	// TODO:
}

static inline struct csi_state *mipi_sd_to_csi_state(struct v4l2_subdev *sdev)
{
	pr_info("enter %s\n", __func__);
	return container_of(sdev, struct csi_state, sd);
}

static inline struct csi_state *notifier_to_mipi_dev(
					struct v4l2_async_notifier *n)
{
	pr_debug("enter %s\n", __func__);
	return container_of(n, struct csi_state, subdev_notifier);
}

static struct media_pad *csis_get_remote_sensor_pad(struct csi_state *state)
{
	struct v4l2_subdev *subdev = &state->sd;
	struct media_pad *sink_pad, *source_pad;
	int i;

	pr_debug("enter %s\n", __func__);

	while (1) {
		source_pad = NULL;
		for (i = 0; i < subdev->entity.num_pads; i++) {
			sink_pad = &subdev->entity.pads[i];

			if (sink_pad->flags & MEDIA_PAD_FL_SINK) {
				source_pad = media_entity_remote_pad(sink_pad);
				if (source_pad)
					break;
			}
		}
		/* return first pad point in the loop  */
		return source_pad;
	}

	if (i == subdev->entity.num_pads)
		v4l2_err(&state->sd, "%s, No remote pad found!\n", __func__);

	return NULL;
}

static struct v4l2_subdev *csis_get_remote_subdev(struct csi_state *state,
						  const char *const label)
{
	struct media_pad *source_pad;
	struct v4l2_subdev *sen_sd;

	pr_debug("enter %s\n", __func__);

	/* Get remote source pad */
	source_pad = csis_get_remote_sensor_pad(state);
	if (!source_pad) {
		v4l2_err(&state->sd, "%s, No remote pad found!\n", label);
		return NULL;
	}

	/* Get remote source pad subdev */
	sen_sd = media_entity_to_v4l2_subdev(source_pad->entity);
	if (!sen_sd) {
		v4l2_err(&state->sd, "%s, No remote subdev found!\n", label);
		return NULL;
	}

	return sen_sd;
}

const struct csis_pix_format *find_csis_format(u32 code)
{
	int i;
	pr_debug("enter %s\n", __func__);

	for (i = 0; i < ARRAY_SIZE(mipi_csis_formats); i++)
		if (code == mipi_csis_formats[i].code)
			return &mipi_csis_formats[i];
	return NULL;
}

static void mipi_csis_clean_irq(struct csi_state *state)
{
	pr_debug("enter %s\n", __func__);
	// TODO
}

static void mipi_csis_enable_interrupts(struct csi_state *state, bool on)
{
	pr_debug("enter %s\n", __func__);
	// TODO
}

void mipi_csis_sw_reset(struct csi_state *state)
{
	pr_debug("enter %s\n", __func__);
	// TODO
}

static int mipi_csis_phy_init(struct csi_state *state)
{
	pr_debug("enter %s\n", __func__);
	// TODO
	return 0;
}

static void mipi_csis_phy_reset(struct csi_state *state)
{
	pr_debug("enter %s\n", __func__);
	// TODO
}

static void mipi_csis_system_enable(struct csi_state *state, int on)
{
	pr_debug("enter %s\n", __func__);
	// TODO
}

/* Called with the state.lock mutex held */
static void __mipi_csis_set_format(struct csi_state *state)
{
	struct v4l2_mbus_framefmt *mf = &state->format;

	v4l2_dbg(1, debug, &state->sd, "fmt: %#x, %d x %d\n",
		 mf->code, mf->width, mf->height);

	// TODO
}

static void mipi_csis_set_hsync_settle(struct csi_state *state)
{
	pr_debug("enter %s\n", __func__);
	// TODO
}

void mipi_csis_set_params(struct csi_state *state)
{
	pr_debug("enter %s\n", __func__);
	// TODO
}

static int mipi_csis_clk_enable(struct csi_state *state)
{
	pr_debug("enter %s\n", __func__);
	// TODO
}

static void mipi_csis_clk_disable(struct csi_state *state)
{
	pr_debug("enter %s\n", __func__);
	// TODO
}

static int mipi_csis_clk_get(struct csi_state *state)
{
	pr_debug("enter %s\n", __func__);
	// TODO
	return 0;
}

static int disp_mix_sft_rstn(struct reset_control *reset, bool enable)
{
	pr_debug("enter %s\n", __func__);
	// TODO
	return 0;
}

static int disp_mix_clks_enable(struct reset_control *reset, bool enable)
{
	pr_debug("enter %s\n", __func__);
	// TODO
	return 0;
}

void disp_mix_gasket_config(struct csi_state *state)
{
	/* gasket is imx's pincontrol related configuration.
	   Refer from: git@github.com:Avnet/linux-imx.git
	               drivers/pinctrl/freescale/pinctrl-imx8mp.c
	 */
	 pr_debug("enter %s\n", __func__);
}

static void disp_mix_gasket_enable(struct csi_state *state, bool enable)
{
	pr_debug("enter %s\n", __func__);
}

static void mipi_csis_start_stream(struct csi_state *state)
{
	pr_debug("enter %s\n", __func__);
	mipi_csis_sw_reset(state);

	disp_mix_gasket_config(state);
	mipi_csis_set_params(state);

	mipi_csis_system_enable(state, true);
	disp_mix_gasket_enable(state, true);
	mipi_csis_enable_interrupts(state, true);

	msleep(5);
}

static void mipi_csis_stop_stream(struct csi_state *state)
{
	pr_debug("enter %s\n", __func__);
	mipi_csis_enable_interrupts(state, false);

	mipi_csis_system_enable(state, false);
	disp_mix_gasket_enable(state, false);
}

static void mipi_csis_clear_counters(struct csi_state *state)
{
	unsigned long flags;
	int i;
	pr_debug("enter %s\n", __func__);
	spin_lock_irqsave(&state->slock, flags);
	for (i = 0; i < MIPI_CSIS_NUM_EVENTS; i++)
		state->events[i].counter = 0;
	spin_unlock_irqrestore(&state->slock, flags);
}

static void mipi_csis_log_counters(struct csi_state *state, bool non_errors)
{
	int i = non_errors ? MIPI_CSIS_NUM_EVENTS : MIPI_CSIS_NUM_EVENTS - 4;
	unsigned long flags;
	pr_debug("enter %s\n", __func__);
	spin_lock_irqsave(&state->slock, flags);

	for (i--; i >= 0; i--) {
		if (state->events[i].counter > 0 || debug)
			v4l2_info(&state->sd, "%s events: %d\n",
				  state->events[i].name,
				  state->events[i].counter);
	}
	spin_unlock_irqrestore(&state->slock, flags);
}

static int mipi_csi2_link_setup(struct media_entity *entity,
				const struct media_pad *local,
				const struct media_pad *remote, u32 flags)
{
	pr_debug("enter %s\n", __func__);
	return 0;
}

static const struct media_entity_operations mipi_csi2_sd_media_ops = {
	.link_setup = mipi_csi2_link_setup,
};

/*
 * V4L2 subdev operations
 */
int mipi_csis_s_power(struct v4l2_subdev *mipi_sd, int on)
{
	struct csi_state *state = mipi_sd_to_csi_state(mipi_sd);
	struct v4l2_subdev *sen_sd;
	pr_debug("enter %s\n", __func__);

	/* Get remote source pad subdev */
	sen_sd = csis_get_remote_subdev(state, __func__);
	if (!sen_sd) {
		v4l2_err(&state->sd, "%s, No remote subdev found!\n", __func__);
		return -EINVAL;
	}

	return v4l2_subdev_call(sen_sd, core, s_power, on);
}

int mipi_csis_s_stream(struct v4l2_subdev *mipi_sd, int enable)
{
	struct csi_state *state = mipi_sd_to_csi_state(mipi_sd);

	v4l2_dbg(1, debug, mipi_sd, "%s: %d, state: 0x%x\n",
		 __func__, enable, state->flags);

	if (enable) {
		pm_runtime_get_sync(state->dev);
		mipi_csis_clear_counters(state);
		mipi_csis_start_stream(state);
		dump_csis_regs(state, __func__);
		dump_gasket_regs(state, __func__);
	} else {
		mipi_csis_stop_stream(state);
		if (debug > 0)
			mipi_csis_log_counters(state, true);
		pm_runtime_put(state->dev);
	}

	return 0;
}

static int mipi_csis_set_fmt(struct v4l2_subdev *mipi_sd,
			     struct v4l2_subdev_pad_config *cfg,
			     struct v4l2_subdev_format *format)
{
	struct csi_state *state = mipi_sd_to_csi_state(mipi_sd);
	struct v4l2_mbus_framefmt *mf = &format->format;
	struct csis_pix_format const *csis_fmt;
	struct media_pad *source_pad;
	struct v4l2_subdev *sen_sd;
	int ret;

	/* Get remote source pad */
	source_pad = csis_get_remote_sensor_pad(state);
	if (!source_pad) {
		v4l2_err(&state->sd, "%s, No remote pad found!\n", __func__);
		return -EINVAL;
	}

	/* Get remote source pad subdev */
	sen_sd = csis_get_remote_subdev(state, __func__);
	if (!sen_sd) {
		v4l2_err(&state->sd, "%s, No remote subdev found!\n", __func__);
		return -EINVAL;
	}

	format->pad = source_pad->index;
	mf->code = MEDIA_BUS_FMT_UYVY8_2X8;
	ret = v4l2_subdev_call(sen_sd, pad, set_fmt, NULL, format);
	if (ret < 0) {
		v4l2_err(&state->sd, "%s, set sensor format fail\n", __func__);
		return -EINVAL;
	}

	csis_fmt = find_csis_format(mf->code);
	if (!csis_fmt) {
		csis_fmt = &mipi_csis_formats[0];
		mf->code = csis_fmt->code;
	}

	return 0;
}

static int mipi_csis_get_fmt(struct v4l2_subdev *mipi_sd,
			     struct v4l2_subdev_pad_config *cfg,
			     struct v4l2_subdev_format *format)
{
	struct csi_state *state = mipi_sd_to_csi_state(mipi_sd);
	struct v4l2_mbus_framefmt *mf = &state->format;
	struct media_pad *source_pad;
	struct v4l2_subdev *sen_sd;
	int ret;

	/* Get remote source pad */
	source_pad = csis_get_remote_sensor_pad(state);
	if (!source_pad) {
		v4l2_err(&state->sd, "%s, No remote pad found!\n", __func__);
		return -EINVAL;
	}

	/* Get remote source pad subdev */
	sen_sd = csis_get_remote_subdev(state, __func__);
	if (!sen_sd) {
		v4l2_err(&state->sd, "%s, No remote subdev found!\n", __func__);
		return -EINVAL;
	}

	format->pad = source_pad->index;
	ret = v4l2_subdev_call(sen_sd, pad, get_fmt, NULL, format);
	if (ret < 0) {
		v4l2_err(&state->sd, "%s, call get_fmt of subdev failed!\n",
			 __func__);
		return ret;
	}

	memcpy(mf, &format->format, sizeof(struct v4l2_mbus_framefmt));
	return 0;
}

static int mipi_csis_s_rx_buffer(struct v4l2_subdev *mipi_sd, void *buf,
				 unsigned int *size)
{
	struct csi_state *state = mipi_sd_to_csi_state(mipi_sd);
	unsigned long flags;

	*size = min_t(unsigned int, *size, MIPI_CSIS_PKTDATA_SIZE);

	spin_lock_irqsave(&state->slock, flags);
	state->pkt_buf.data = buf;
	state->pkt_buf.len = *size;
	spin_unlock_irqrestore(&state->slock, flags);

	return 0;
}

static int mipi_csis_s_frame_interval(struct v4l2_subdev *mipi_sd, struct v4l2_subdev_frame_interval
				      *interval)
{
	struct csi_state *state = mipi_sd_to_csi_state(mipi_sd);
	struct v4l2_subdev *sen_sd;

	/* Get remote source pad subdev */
	sen_sd = csis_get_remote_subdev(state, __func__);
	if (!sen_sd) {
		v4l2_err(&state->sd, "%s, No remote subdev found!\n", __func__);
		return -EINVAL;
	}

	return v4l2_subdev_call(sen_sd, video, s_frame_interval, interval);
}

static int mipi_csis_g_frame_interval(struct v4l2_subdev *mipi_sd,
				struct v4l2_subdev_frame_interval *interval)
{
	struct csi_state *state = mipi_sd_to_csi_state(mipi_sd);
	struct v4l2_subdev *sen_sd;

	/* Get remote source pad subdev */
	sen_sd = csis_get_remote_subdev(state, __func__);
	if (!sen_sd) {
		v4l2_err(&state->sd, "%s, No remote subdev found!\n", __func__);
		return -EINVAL;
	}

	return v4l2_subdev_call(sen_sd, video, g_frame_interval, interval);
}

static int mipi_csis_enum_framesizes(struct v4l2_subdev *mipi_sd,
				     struct v4l2_subdev_pad_config *cfg,
				     struct v4l2_subdev_frame_size_enum *fse)
{
	struct csi_state *state = mipi_sd_to_csi_state(mipi_sd);
	struct v4l2_subdev *sen_sd;

	/* Get remote source pad subdev */
	sen_sd = csis_get_remote_subdev(state, __func__);
	if (!sen_sd) {
		v4l2_err(&state->sd, "%s, No remote subdev found!\n", __func__);
		return -EINVAL;
	}

	return v4l2_subdev_call(sen_sd, pad, enum_frame_size, NULL, fse);
}

static int mipi_csis_enum_frameintervals(struct v4l2_subdev *mipi_sd,
					 struct v4l2_subdev_pad_config *cfg,
					 struct v4l2_subdev_frame_interval_enum
					 *fie)
{
	struct csi_state *state = mipi_sd_to_csi_state(mipi_sd);
	struct v4l2_subdev *sen_sd;

	/* Get remote source pad subdev */
	sen_sd = csis_get_remote_subdev(state, __func__);
	if (!sen_sd) {
		v4l2_err(&state->sd, "%s, No remote subdev found!\n", __func__);
		return -EINVAL;
	}

	return v4l2_subdev_call(sen_sd, pad, enum_frame_interval, NULL, fie);
}

static int mipi_csis_log_status(struct v4l2_subdev *mipi_sd)
{
	struct csi_state *state = mipi_sd_to_csi_state(mipi_sd);

	mutex_lock(&state->lock);
	mipi_csis_log_counters(state, true);
	if (debug) {
		dump_csis_regs(state, __func__);
		dump_gasket_regs(state, __func__);
	}
	mutex_unlock(&state->lock);
	return 0;
}


int csis_s_fmt(struct v4l2_subdev *sd, struct csi_sam_format *fmt)
{
	u32 code;
	const struct csis_pix_format *csis_format;
	struct csi_state *state = container_of(sd, struct csi_state, sd);

	switch (fmt->format) {
	case V4L2_PIX_FMT_SBGGR10:
	    code = MEDIA_BUS_FMT_SBGGR10_1X10;
	    break;
	case V4L2_PIX_FMT_SGBRG10:
	    code = MEDIA_BUS_FMT_SGBRG10_1X10;
	    break;
	case V4L2_PIX_FMT_SGRBG10:
	    code = MEDIA_BUS_FMT_SGRBG10_1X10;
	    break;
	case V4L2_PIX_FMT_SRGGB10:
	    code = MEDIA_BUS_FMT_SRGGB10_1X10;
	    break;
	case V4L2_PIX_FMT_SBGGR12:
	    code = MEDIA_BUS_FMT_SBGGR12_1X12;
	    break;
	case V4L2_PIX_FMT_SGBRG12:
	    code = MEDIA_BUS_FMT_SGBRG12_1X12;
	    break;
	case V4L2_PIX_FMT_SGRBG12:
	    code = MEDIA_BUS_FMT_SGRBG12_1X12;
	    break;
	case V4L2_PIX_FMT_SRGGB12:
	    code = MEDIA_BUS_FMT_SRGGB12_1X12;
	    break;
	default:
		return -EINVAL;
	}
	csis_format = find_csis_format(code);
	if (csis_format == NULL)
		return -EINVAL;

	state->csis_fmt = csis_format;
	state->format.width = fmt->width;
	state->format.height = fmt->height;
	disp_mix_gasket_config(state);
	mipi_csis_set_params(state);
	return 0;
}

int csis_s_hdr(struct v4l2_subdev *sd, bool enable)
{
	struct csi_state *state = container_of(sd, struct csi_state, sd);
	pr_debug("enter %s: %d\n", __func__, enable);
	state->hdr = enable;
	return 0;
}

int csis_ioc_qcap(struct v4l2_subdev *dev, void *args)
{
	struct csi_state *state = mipi_sd_to_csi_state(dev);
	struct v4l2_capability *cap = (struct v4l2_capability *)args;
	pr_info("enter %s\n", __func__);
	strcpy((char *)cap->driver, "csi_dw_subdev");
	cap->bus_info[0] = state->index;
	pr_info("%s: cap->driver=%s, state->index=%d\n",
		__func__, (char *)cap->driver, state->index);
	return 0;
}

#ifdef __KERNEL__
#define USER_TO_KERNEL(TYPE) \
	do {\
		TYPE tmp; \
		arg = (void *)(&tmp); \
		copy_from_user(arg, arg_user, sizeof(TYPE));\
	} while (0)

#define KERNEL_TO_USER(TYPE) \
		copy_to_user(arg_user, arg, sizeof(TYPE));
#else
#define USER_TO_KERNEL(TYPE)
#define KERNEL_TO_USER(TYPE)
#endif
long csis_priv_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg_user)
{
	int ret = 1;
	struct csi_state *state = container_of(sd, struct csi_state, sd);
	void *arg = arg_user;

	pr_debug("enter %s\n", __func__);
	switch (cmd) {
	case VVCSIOC_RESET:
		mipi_csis_sw_reset(state);
		ret = 0;
		break;
	case VVCSIOC_POWERON:
		ret = mipi_csis_s_power(sd, 1);
		break;
	case VVCSIOC_POWEROFF:
		ret = mipi_csis_s_power(sd, 0);
		break;
	case VVCSIOC_STREAMON:
		ret = mipi_csis_s_stream(sd, 1);
		break;
	case VVCSIOC_STREAMOFF:
		ret = mipi_csis_s_stream(sd, 0);
		break;
	case VVCSIOC_S_FMT: {
		USER_TO_KERNEL(struct csi_sam_format);
		ret = csis_s_fmt(sd, (struct csi_sam_format *)arg);
		break;
	}
	case VVCSIOC_S_HDR: {
		USER_TO_KERNEL(bool);
		ret = csis_s_hdr(sd, *(bool *) arg);
		break;
	}
	case VIDIOC_QUERYCAP:
		ret = csis_ioc_qcap(sd, arg);
		break;
	default:
		pr_err("unsupported csi-sam command %d.", cmd);
		break;
	}

	return ret;
}

static struct v4l2_subdev_core_ops mipi_csis_core_ops = {
	.s_power = mipi_csis_s_power,
	.log_status = mipi_csis_log_status,
	.ioctl = csis_priv_ioctl,
};

static struct v4l2_subdev_video_ops mipi_csis_video_ops = {
	.s_rx_buffer = mipi_csis_s_rx_buffer,
	.s_stream = mipi_csis_s_stream,

	.g_frame_interval = mipi_csis_g_frame_interval,
	.s_frame_interval = mipi_csis_s_frame_interval,
};

static const struct v4l2_subdev_pad_ops mipi_csis_pad_ops = {
	.enum_frame_size = mipi_csis_enum_framesizes,
	.enum_frame_interval = mipi_csis_enum_frameintervals,
	.get_fmt = mipi_csis_get_fmt,
	.set_fmt = mipi_csis_set_fmt,
};

static struct v4l2_subdev_ops mipi_csis_subdev_ops = {
	.core = &mipi_csis_core_ops,
	.video = &mipi_csis_video_ops,
	.pad = &mipi_csis_pad_ops,
};

static irqreturn_t mipi_csis_irq_handler(int irq, void *dev_id)
{
	pr_debug("enter %s\n", __func__);
	// TODO
	return IRQ_HANDLED;
}

static int mipi_csis_parse_dt(struct platform_device *pdev,
			      struct csi_state *state)
{
	pr_info("enter %s\n", __func__);
	struct device_node *node = pdev->dev.of_node;

	state->index = of_alias_get_id(node, "csi");	// dts: csi0 = &mipi_csi_0;
	pr_info("[%s:%d] state->index=%d\n", __func__, __LINE__, state->index);
	state->index = 0;	// dts: csi0 = &mipi_csi_0;
	pr_info("[%s:%d] state->index=%d (hardcode)\n", __func__, __LINE__, state->index);

	if (of_property_read_u32
	    (node, "clock-frequency", &state->clk_frequency))
		state->clk_frequency = DEFAULT_SCLK_CSIS_FREQ;
	pr_info("[%s:%d] clock-frequency=%d\n", __func__, __LINE__, state->clk_frequency);

	if (of_property_read_u32(node, "bus-width", &state->max_num_lanes))
		return -EINVAL;
	pr_info("[%s:%d] bus-width=%d\n", __func__, __LINE__, state->max_num_lanes);

	node = of_graph_get_next_endpoint(node, NULL);
	if (!node) {
		dev_err(&pdev->dev, "No port node at %s\n", node->full_name);
		return -EINVAL;
	}

	/* Get MIPI CSI-2 bus configration from the endpoint node. */
	of_property_read_u32(node, "csis-hs-settle", &state->hs_settle);
	of_property_read_u32(node, "csis-clk-settle", &state->clk_settle);
	of_property_read_u32(node, "data-lanes", &state->num_lanes);
	pr_info("[%s:%d] hs_settle=%d, clk_settle=%d, num_lanes=%d\n",
		__func__, __LINE__,
		state->hs_settle, state->clk_settle, state->num_lanes);

	state->wclk_ext = of_property_read_bool(node, "csis-wclk");
	pr_info("[%s:%d] csis-wclk=%d\n", __func__, __LINE__, state->wclk_ext);

	of_node_put(node);
	return 0;
}

static const struct of_device_id mipi_csis_of_match[];

/* init subdev */
static int mipi_csis_subdev_init(struct v4l2_subdev *mipi_sd,
				 struct platform_device *pdev,
				 const struct v4l2_subdev_ops *ops)
{
	struct csi_state *state = platform_get_drvdata(pdev);
	int ret = 0;

	v4l2_subdev_init(mipi_sd, ops);
	mipi_sd->owner = THIS_MODULE;
	snprintf(mipi_sd->name, sizeof(mipi_sd->name), "%s.%d",
		 CSIS_SUBDEV_NAME, state->index);
	mipi_sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	mipi_sd->entity.function = MEDIA_ENT_F_IO_V4L;
	mipi_sd->dev = &pdev->dev;

	state->csis_fmt = &mipi_csis_formats[0];
	state->format.code = mipi_csis_formats[0].code;
	state->format.width = MIPI_CSIS_DEF_PIX_WIDTH;
	state->format.height = MIPI_CSIS_DEF_PIX_HEIGHT;

	/* This allows to retrieve the platform device id by the host driver */
	v4l2_set_subdevdata(mipi_sd, state);

	state->v4l2_dev = kzalloc(sizeof(*state->v4l2_dev), GFP_KERNEL);
	if (WARN_ON(!state->v4l2_dev)) {
		ret = -ENOMEM;
		goto end;
	}

	ret = v4l2_device_register(&(pdev->dev), state->v4l2_dev);
	if (WARN_ON(ret < 0))
		goto end;

	ret = v4l2_device_register_subdev(state->v4l2_dev, mipi_sd);

	if (ret) {
		pr_err("failed to register csis-subdev %d\n", ret);
		goto end;
	}

	ret = v4l2_device_register_subdev_nodes(state->v4l2_dev);
end:
	return ret;
}

static int mipi_csis_of_parse_resets(struct csi_state *state)
{
	return 0;
}

static int mipi_csis_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct v4l2_subdev *mipi_sd;
	struct resource *mem_res;
	struct csi_state *state;
	const struct of_device_id *of_id;
	mipi_csis_phy_reset_t phy_reset_fn;
	int ret = -ENOMEM;
	pr_info("enter %s\n", __func__);

	state = devm_kzalloc(dev, sizeof(*state), GFP_KERNEL);
	if (!state)
		return -ENOMEM;
	pr_info("[%s:%d]\n", __func__, __LINE__);

	mutex_init(&state->lock);
	spin_lock_init(&state->slock);

	/* WA on VSI platform, need insmod dynamically for debug */
	pdev->dev.of_node =
	    of_find_compatible_node(NULL, NULL, "thead,light-mipi-csi");

	state->pdev = pdev;
	mipi_sd = &state->sd;
	state->dev = dev;

	ret = mipi_csis_parse_dt(pdev, state);
	if (ret < 0)
		return ret;
	pr_info("[%s:%d]\n", __func__, __LINE__);

	/* Hardcode here */
	state->num_lanes = 4;
	state->hs_settle = 0x10;

	if (state->num_lanes == 0 || state->num_lanes > state->max_num_lanes) {
		dev_err(dev, "Unsupported number of data lanes: %d (max. %d)\n",
			state->num_lanes, state->max_num_lanes);
		return -EINVAL;
	}
	pr_info("[%s:%d]\n", __func__, __LINE__);

	ret = mipi_csis_phy_init(state);
	if (ret < 0)
		return ret;

	of_id = of_match_node(mipi_csis_of_match, dev->of_node);
	if (!of_id || !of_id->data) {
		dev_err(dev, "No match data for %s\n", dev_name(dev));
		return -EINVAL;
	}
	phy_reset_fn = of_id->data;	// = mipi_csis_of_match.data = mipi_csis_phy_reset
	state->phy_reset_fn = phy_reset_fn;

	ret = of_clk_set_defaults(dev->of_node, false);
	if (ret < 0) {
		pr_err("clk: couldn't set desired clock for CSI\n");
		return ret;
	}
	pr_info("[%s:%d]\n", __func__, __LINE__);

	ret = mipi_csis_of_parse_resets(state);
	if (ret < 0) {
		dev_err(dev, "Can not parse reset control\n");
		return ret;
	}
	pr_info("[%s:%d]\n", __func__, __LINE__);

	mem_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	state->regs = devm_ioremap_resource(dev, mem_res);
	//state->regs = ioremap(0x32e40000, 0x00001000);
	if (IS_ERR(state->regs))
		return PTR_ERR(state->regs);
	pr_info("[%s:%d]\n", __func__, __LINE__);

	state->irq = platform_get_irq(pdev, 0);
	if (state->irq < 0) {
		dev_err(dev, "Failed to get irq\n");
		return state->irq;
	}
	pr_info("[%s:%d]\n", __func__, __LINE__);

	ret = mipi_csis_clk_get(state);
	if (ret < 0)
		return ret;
	pr_info("[%s:%d]\n", __func__, __LINE__);

	ret = mipi_csis_clk_enable(state);
	if (ret < 0)
		return ret;
	pr_info("[%s:%d]\n", __func__, __LINE__);

	disp_mix_clks_enable(state->clk_enable, true);
	disp_mix_sft_rstn(state->soft_resetn, false);
	phy_reset_fn(state);

	/* mipi_csis_clk_disable(state); */

	ret = devm_request_irq(dev, state->irq, mipi_csis_irq_handler, 0,
			       dev_name(dev), state);
	if (ret) {
		dev_err(dev, "Interrupt request failed\n");
		return ret;
	}
	pr_info("[%s:%d]\n", __func__, __LINE__);

	platform_set_drvdata(pdev, state);
	ret = mipi_csis_subdev_init(&state->sd, pdev, &mipi_csis_subdev_ops);
	if (ret < 0) {
		dev_err(dev, "mipi csi subdev init failed\n");
		return ret;
	}
	pr_info("[%s:%d]\n", __func__, __LINE__);

	state->pads[MIPI_CSIS_VC0_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	state->pads[MIPI_CSIS_VC1_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	state->pads[MIPI_CSIS_VC2_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	state->pads[MIPI_CSIS_VC3_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	state->pads[MIPI_CSIS_VC0_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	state->pads[MIPI_CSIS_VC1_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	state->pads[MIPI_CSIS_VC2_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	state->pads[MIPI_CSIS_VC3_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	ret =
	    media_entity_pads_init(&state->sd.entity, MIPI_CSIS_VCX_PADS_NUM,
				   state->pads);
	if (ret < 0) {
		dev_err(dev, "mipi csi entity pad init failed\n");
		return ret;
	}
	pr_info("[%s:%d]\n", __func__, __LINE__);

	memcpy(state->events, mipi_csis_events, sizeof(state->events));
	state->sd.entity.ops = &mipi_csi2_sd_media_ops;

	pm_runtime_enable(dev);
	/* mipi_csis_s_stream(&state->sd, 1); */

	dev_info(&pdev->dev,
		 "lanes: %d, hs_settle: %d, clk_settle: %d, wclk: %d, freq: %u\n",
		 state->num_lanes, state->hs_settle, state->clk_settle,
		 state->wclk_ext, state->clk_frequency);
	pr_info("[%s:%d]\n", __func__, __LINE__);
	return 0;
}

static int mipi_csis_system_suspend(struct device *dev)
{
	return pm_runtime_force_suspend(dev);;
}

static int mipi_csis_system_resume(struct device *dev)
{
	int ret;

	ret = pm_runtime_force_resume(dev);
	if (ret < 0) {
		dev_err(dev, "force resume %s failed!\n", dev_name(dev));
		return ret;
	}

	return 0;
}

static int mipi_csis_runtime_suspend(struct device *dev)
{
	struct csi_state *state = dev_get_drvdata(dev);
	int ret;

	ret = regulator_disable(state->mipi_phy_regulator);
	if (ret < 0)
		return ret;

	disp_mix_clks_enable(state->clk_enable, false);
	mipi_csis_clk_disable(state);
	return 0;
}

static int mipi_csis_runtime_resume(struct device *dev)
{
	struct csi_state *state = dev_get_drvdata(dev);
	int ret;

	ret = regulator_enable(state->mipi_phy_regulator);
	if (ret < 0)
		return ret;

	ret = mipi_csis_clk_enable(state);
	if (ret < 0)
		return ret;

	disp_mix_clks_enable(state->clk_enable, true);
	disp_mix_sft_rstn(state->soft_resetn, false);

	if (state->phy_reset_fn)
		state->phy_reset_fn(state);

	return 0;
}

static int mipi_csis_remove(struct platform_device *pdev)
{
	struct csi_state *state = platform_get_drvdata(pdev);

	media_entity_cleanup(&state->sd.entity);

	v4l2_device_unregister_subdev(&state->sd);
	v4l2_device_disconnect(state->v4l2_dev);
	v4l2_device_put(state->v4l2_dev);

	return 0;
}

static const struct dev_pm_ops mipi_csis_pm_ops = {
	SET_RUNTIME_PM_OPS(mipi_csis_runtime_suspend, mipi_csis_runtime_resume,
			   NULL)
	    SET_SYSTEM_SLEEP_PM_OPS(mipi_csis_system_suspend,
				    mipi_csis_system_resume)
};

static const struct of_device_id mipi_csis_of_match[] = {
	{.compatible = "thead,light-mipi-csi",
	 .data = (void *)&mipi_csis_phy_reset,
	 },
	{ /* sentinel */ },
};

MODULE_DEVICE_TABLE(of, mipi_csis_of_match);

static struct platform_driver mipi_csis_driver = {
	.driver = {
		   .name = CSIS_DRIVER_NAME,
		   .owner = THIS_MODULE,
		   .pm = &mipi_csis_pm_ops,
		   .of_match_table = mipi_csis_of_match,
		   },
	.probe = mipi_csis_probe,
	.remove = mipi_csis_remove,
};

static int __init csi_init_module(void)
{
	int ret = 0;
	pr_debug("enter %s\n", __func__);

	ret = platform_driver_register(&mipi_csis_driver);
	if (ret) {
		pr_err("register platform driver failed.\n");
		return ret;
	}
	return ret;
}

static void __exit csi_exit_module(void)
{
	pr_debug("enter %s\n", __func__);
	platform_driver_unregister(&mipi_csis_driver);
}

module_init(csi_init_module);
module_exit(csi_exit_module);

MODULE_DESCRIPTION("T-Head MIPI-CSI2 receiver driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("CSI Capture");
MODULE_VERSION("1.0");
