#!/bin/sh
#insmod vvcam_i2c
modprobe vvcam_sensor
#insmod vvcam_csi
modprobe bm_visys
modprobe bm_csi
modprobe vvcam_isp
modprobe vvcam_isp_ry
modprobe vvcam_soc
modprobe vvcam_dw200
modprobe vi_pre
modprobe vvcam_dec400
modprobe thead_video
modprobe vidmem
modprobe vvcam_flash_led
