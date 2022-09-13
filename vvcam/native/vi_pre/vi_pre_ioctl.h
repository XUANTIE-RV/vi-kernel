/*
 * Copyright (C) 2021 Alibaba Group Holding Limited
 * Author: Shenwuyi <shenwuyi.swy@alibaba-inc.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _VI_PRE_IOC_H_
#define _VI_PRE_IOC_H_
#include <linux/ioctl.h>

struct vi_pre_reg_t {
	unsigned int offset;
	unsigned int value;
};

#define VIPER_IOCTL_MAGIC 'r'

#define VI_PRE_IOCTL_RESET                  _IOWR(VIPER_IOCTL_MAGIC, 1, int)
#define VI_PRE_IOCTL_WRITE_REG              _IOWR(VIPER_IOCTL_MAGIC, 2, int)
#define VI_PRE_IOCTL_READ_REG               _IOWR(VIPER_IOCTL_MAGIC, 3, int)
#define VI_PRE_IOCTL_SET_IPI_RESOLUTION     _IOWR(VIPER_IOCTL_MAGIC, 4, int)
#define VI_PRE_IOCTL_SET_PIPLINE            _IOWR(VIPER_IOCTL_MAGIC, 5, int)
#define VI_PRE_IOCTL_SET_IPI_MODE           _IOWR(VIPER_IOCTL_MAGIC, 6, int)
#define VI_PRE_IOCTL_SET_IPI_IDNUM          _IOWR(VIPER_IOCTL_MAGIC, 7, int)
#define VI_PRE_IOCTL_ENABLE_IPI             _IOWR(VIPER_IOCTL_MAGIC, 8, int)
#define VI_PRE_IOCTL_DISABLE_IPI            _IOWR(VIPER_IOCTL_MAGIC, 9, int)
#define VI_PRE_IOCTL_SET_MIPI2DMA_M_FRMAE   _IOWR(VIPER_IOCTL_MAGIC, 10, int)
#define VI_PRE_IOCTL_SET_MIPI2DMA_N_LINE    _IOWR(VIPER_IOCTL_MAGIC, 11, int)
#define VI_PRE_IOCTL_SET_MIPI2DMA_START     _IOWR(VIPER_IOCTL_MAGIC, 12, int)
#define VI_PRE_IOCTL_SET_MIPI2DMA_STOP      _IOWR(VIPER_IOCTL_MAGIC, 13, int)
#define VI_PRE_IOCTL_SET_HDRPRO_MODE        _IOWR(VIPER_IOCTL_MAGIC, 14, int)
#define VI_PRE_IOCTL_SET_HDRPRO_RESOLUTION  _IOWR(VIPER_IOCTL_MAGIC, 15, int)
#define VI_PRE_IOCTL_SET_HDRPRO_BLACK_PARA  _IOWR(VIPER_IOCTL_MAGIC, 16, int)
#define VI_PRE_IOCTL_SET_HDRPRO_COLOR_PARA  _IOWR(VIPER_IOCTL_MAGIC, 17, int)
#define VI_PRE_IOCTL_SET_HDRPRO_MERGE_PARA  _IOWR(VIPER_IOCTL_MAGIC, 18, int)
#define VI_PRE_IOCTL_PRESS_INTERROR         _IOWR(VIPER_IOCTL_MAGIC, 19, int)
#define VI_PRE_IOCTL_HDRPRO_ENABLE          _IOWR(VIPER_IOCTL_MAGIC, 20, int)
#define VI_PRE_IOCTL_HDRPRO_DISABLE         _IOWR(VIPER_IOCTL_MAGIC, 21, int)
#define VI_PRE_IOCTL_MAX                    _IOWR(VIPER_IOCTL_MAGIC, 22, int)

typedef enum {
    ipi_mode_nomal = 0,
    ipi_mode_hdr_2_frame,
    ipi_mode_hdr_3_frame,
    ipi_mode_sony_dol_hdr_2_frame,
    ipi_mode_sony_dol_hdr_3_frame,
} ipi_mode_t;

typedef struct {
    int glue_idx;
    ipi_mode_t mode;
} ipi_mode_cfg_t;

typedef struct {
    int glue_idx;
    int id_third_sync;
    int id_second_sync;
    int id_first_sync;
    int id_3;
    int id_2;
    int id_1;
}ipi_idnum_cfg_t;

enum hdrpro_raw_mode {
	//RAWMOD_HDRPRO
	HDRPRO_RAW6 = 0 << 21,
	HDRPRO_RAW7 = 1 << 21,
	HDRPRO_RAW8 = 2 << 21,
	HDRPRO_RAW10 = 3 << 21,
	HDRPRO_RAW12 = 4 << 21,
};

enum hdrpro_bayer_modesl {
	//BAYER_MODSEL
	HDRPRO_BAYER_MODSEL_BAYER_A = 0 << 19,
	HDRPRO_BAYER_MODSEL_BAYER_B = 1 << 19,
	HDRPRO_BAYER_MODSEL_BAYER_C = 2 << 19,
	HDRPRO_BAYER_MODSEL_BAYER_D = 3 << 19,
};

enum hdrpro_color_modesl {
	//COLOR_MODSEL
	HDRPRO_COLOR_MODSEL_BLACK_NODOWN_2X2 = 0 << 16,
	HDRPRO_COLOR_MODSEL_BLACK_DOWN_2X2 = 1 << 16,
	HDRPRO_COLOR_MODSEL_RGB_NODOWN_2X2 = 2 << 16,
	HDRPRO_COLOR_MODSEL_RGB_DOWN_2X2 = 3 << 16,
	HDRPRO_COLOR_MODSEL_RGB_NODOWN_3X3 = 4 << 16,
	HDRPRO_COLOR_MODSEL_RGB_DOWN_3X3 = 5 << 16,
};

enum hdrpro_enable {
	//HDRPRO_EN
	HDRPRO_BYPASS = 0 << 0,
	HDRPRO_EN = 1 << 0
};

// need to use bit length to define para after the final register map released
struct hdrpro_para {
	unsigned int para[16];
};

// need to use bit length to define para after the final register map released
struct hdrpro_black_weight_para {
	unsigned int wr;
	unsigned int wb;
	unsigned int wg0;
	unsigned int wg1;
};

// need to use bit length to define para after the final register map released
struct hdrpro_color_weight_para {
	unsigned int pattern_00;
	unsigned int pattern_01;
	unsigned int pattern_02;
	unsigned int pattern_10;
	unsigned int pattern_11;
	unsigned int pattern_12;
	unsigned int pattern_20;
	unsigned int pattern_21;
	unsigned int pattern_22;
};

// need to use bit length to define para after the final register map released
struct hdrpro_black_para {
	struct hdrpro_black_weight_para weight_para;
};

struct hdrpro_merge_para {
	struct hdrpro_para a;
	struct hdrpro_para b;
	struct hdrpro_para w;
};

// need to use bit length to define para after the final register map released
struct hdrpro_color_para {
	struct hdrpro_color_weight_para wr;
	struct hdrpro_color_weight_para wb;
	struct hdrpro_color_weight_para wg0;
	struct hdrpro_color_weight_para wg1;
};

// need to use bit length to define para after the final register map released
struct hdrpro_mode {
	enum hdrpro_raw_mode raw_mode;
	enum hdrpro_bayer_modesl bayer_modesel;
	enum hdrpro_color_modesl color_modesel;
	unsigned int last_delay;
	unsigned int id;
};

struct hdrpro_resolution {
	unsigned int width;
	unsigned int height;
};

extern unsigned int vi_pre_priv_ioctl(struct vi_pre_dev *dev, unsigned int cmd, void *args);

#endif /* _VI_PRE_IOC_H_ */
