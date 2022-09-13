/*
 * Copyright (C) 2021 Alibaba Group Holding Limited
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef _DEC400_IOC_H_
#define _DEC400_IOC_H_
#include <linux/ioctl.h>

enum {
	DEC400IOC_RESET = 0,
	DEC400IOC_WRITE_REG,
	DEC400IOC_READ_REG,
	DEC400IOC_COMPRESS_INIT,
	DEC400IOC_DECOMPRESS_INIT,
	DEC400IOC_COMPRESS_SET_BUFFER,
	DEC400IOC_DECOMPRESS_SET_BUFFER,
	DEC400IOC_MMU_CONFIG
};

struct dec400_compress_init {
	unsigned char enable_global_bypass;
	unsigned char enable_hw_flush;
};

struct dec400_decompress_init {
	unsigned char enable_global_bypass;
	unsigned char enable_hw_flush;
};

struct dec400_mmu_para {
	unsigned long long page_bable_addr; 
};

enum dec400_compress_channel {
	DEC400_COMPRESS_CHANNEL_0 = 0,
	DEC400_COMPRESS_CHANNEL_1,
	DEC400_COMPRESS_CHANNEL_2,
	DEC400_COMPRESS_CHANNEL_3,
	DEC400_COMPRESS_CHANNEL_4,
	DEC400_COMPRESS_CHANNEL_5,
	DEC400_COMPRESS_CHANNEL_6,
	DEC400_COMPRESS_CHANNEL_7,
	DEC400_COMPRESS_CHANNEL_8,
	DEC400_COMPRESS_CHANNEL_9,
	DEC400_COMPRESS_CHANNEL_10,
	DEC400_COMPRESS_CHANNEL_11,
	DEC400_COMPRESS_CHANNEL_12,
	DEC400_COMPRESS_CHANNEL_13,
	DEC400_COMPRESS_CHANNEL_14,
	DEC400_COMPRESS_CHANNEL_15
};

enum dec400_decompress_channel {
	DEC400_DECOMPRESS_CHANNEL_0 = 0,
	DEC400_DECOMPRESS_CHANNEL_1,
	DEC400_DECOMPRESS_CHANNEL_2,
	DEC400_DECOMPRESS_CHANNEL_3,
	DEC400_DECOMPRESS_CHANNEL_4,
	DEC400_DECOMPRESS_CHANNEL_5,
	DEC400_DECOMPRESS_CHANNEL_6,
	DEC400_DECOMPRESS_CHANNEL_7,
	DEC400_DECOMPRESS_CHANNEL_8,
	DEC400_DECOMPRESS_CHANNEL_9,
	DEC400_DECOMPRESS_CHANNEL_10,
	DEC400_DECOMPRESS_CHANNEL_11,
	DEC400_DECOMPRESS_CHANNEL_12,
	DEC400_DECOMPRESS_CHANNEL_13,
	DEC400_DECOMPRESS_CHANNEL_14,
	DEC400_DECOMPRESS_CHANNEL_15
};
	
enum dec400_format {
	DEC_FMT_ARGB8 = 0,
	DEC_FMT_XRGB8 = 1,
	DEC_FMT_AYUV = 2,
	DEC_FMT_UYVY = 3,
	DEC_FMT_YUY2 = 4,
	DEC_FMT_YUV_ONLY = 5,
	DEC_FMT_UV_MIX = 6,
	DEC_FMT_ARGB4 = 7,
	DEC_FMT_XRGB4 = 8,
	DEC_FMT_A1RGB5 = 9,
	DEC_FMT_X1RGB5 = 10,
	DEC_FMT_R5G6B5 = 11,
	DEC_FMT_A2R10G10B10 = 15,
	DEC_FMT_BAYER = 16,
	DEC_FMT_COEFFICIENT = 18,
	DEC_FMT_ARGB16 = 19,
	DEC_FMT_X2RGB10 = 21
};
enum dec400_tile_mode {
	DEC_TILE8X8_XMAJOR = 0,
	DEC_TILE8X8_YMAJOR = 1,
	DEC_TILE16X4 = 2,
	DEC_TILE8X4 = 3,
	DEC_TILE4X8 = 4,
	DEC_RASTER16X4 = 6,
	DEC_TILE64X4 = 7,
	DEC_TILE32X4 = 8,
	DEC_RASTER256X1 = 9,
	DEC_RASTER128X1 = 10,
	DEC_RASTER64X4 = 11,
	DEC_RASTER256X2 = 12,
	DEC_RASTER128X2 = 13,
	DEC_RASTER128X4 = 14,
	DEC_RASTER64X1 = 15,
	DEC_TILE16X8 = 16,
	DEC_TILE8X16 = 17,
	DEC_RASTER512X1 = 18,
	DEC_RASTER32X4 = 19,
	DEC_RASTER64X2 = 20,
	DEC_RASTER32X2 = 21,
	DEC_RASTER32X1 = 22,
	DEC_RASTER16X1 = 23,
	DEC_TILE128X4 = 24,
	DEC_TILE256X4 = 25,
	DEC_TILE512X4 = 26,
	DEC_TILE16X16 = 27,
	DEC_TILE32X16 = 28,
	DEC_TILE64X16 = 29,
	DEC_TILE128X8 = 30,
	DEC_TILE8X4_S = 31,
	DEC_TILE16X4_S = 32,
	DEC_TILE32X4_S = 33,
	DEC_TILE16X4_LSB = 34,
	DEC_TILE32X4_LSB = 35,
	DEC_TILE32X8 = 36
};

enum dec400_aligh_mode {
	DEC_ALIGN_1_BYTE = 0,
	DEC_ALIGN_16_BYTE = 1,
	DEC_ALIGN_32_BYTE = 2,
	DEC_ALIGN_64_BYTE = 3,
};

struct dec400_compress_para {
	unsigned char enable;
	enum dec400_compress_channel channel;
	enum dec400_format format;
	enum dec400_tile_mode tile_mode;
	enum dec400_aligh_mode align_mode;
	unsigned long long physical_stream_start;
	unsigned long long physical_stream_end;
	unsigned long long physical_tile_start;
};

struct dec400_decompress_para {
	unsigned char enable;
	enum dec400_decompress_channel channel;
	enum dec400_format format;
	enum dec400_tile_mode tile_mode;
	enum dec400_aligh_mode align_mode;
	unsigned long long physical_stream_start;
	unsigned long long physical_stream_end;
	unsigned long long physical_tile_start;
};

#endif /* _DEC400_IOC_H_ */
