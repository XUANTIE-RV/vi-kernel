#!/bin/bash
set -x
cd /bsp/isp/ko
insmod vvcam-video.ko
insmod ov2775.ko
insmod vvcam-csis.ko
insmod vvcam-dwe.ko
insmod vvcam-isp.ko
lsmod
