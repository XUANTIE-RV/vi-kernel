#!/bin/sh
#insmod vvcam_i2c.ko
insmod vvcam_sensor.ko
#insmod vvcam_csi.ko
insmod bm_visys.ko
insmod bm_csi.ko
insmod vvcam_isp.ko
insmod vvcam_soc.ko
insmod vvcam_dw200.ko
insmod vi_pre.ko
insmod vvcam_isp_ry.ko
