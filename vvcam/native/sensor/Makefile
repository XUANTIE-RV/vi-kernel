include $(PWD)/../Param.mk
include $(PWD)/../../version/$(VERSION_CFG).mk
TARGET = vvcam_sensor

obj-m +=$(TARGET).o
$(TARGET)-objs += vvcam_sensor_driver.o
$(TARGET)-objs += sensor_ioctl.o
$(TARGET)-objs += vvcam_sensor_sysfs.o


EXTRA_CFLAGS += -I$(PWD)/../../common

ifeq ($(SENSR0_TYPE), ov2775)
$(TARGET)-objs += ./omnivision_ov2775/ov2775_driver.o
$(TARGET)-objs += ./omnivision_ov2775/ov2775_mipi4lane_1080p_30fps_linear.o
EXTRA_CFLAGS += -I$(PWD)/omnivision_ov2775
EXTRA_CFLAGS += -DSENSR0_FUNCTION=ov2775_function
endif

ifeq ($(SENSR1_TYPE), ov2775)
#$(TARGET)-objs += ./omnivision_ov2775/ov2775_driver.o
#$(TARGET)-objs += ./omnivision_ov2775/ov2775_mipi4lane_1080p_30fps_linear.o
#EXTRA_CFLAGS += -I$(PWD)/omnivision_ov2775
EXTRA_CFLAGS += -DSENSR1_FUNCTION=ov2775_function
endif

ifeq ($(SENSR0_TYPE), imx334)
$(TARGET)-objs += ./sony_imx334/imx334_driver.o
$(TARGET)-objs += ./sony_imx334/imx334_mipi4lane_1080p_30fps_linear.o
EXTRA_CFLAGS += -I$(PWD)/sony_imx334
EXTRA_CFLAGS += -DSENSR0_FUNCTION=imx334_function
endif

ifeq ($(SENSR1_TYPE), imx334)
#$(TARGET)-objs += ./omnivision_ov5693/ov2775_driver.o
#$(TARGET)-objs += ./omnivision_ov2775/ov2775_mipi4lane_1080p_30fps_linear.o
#EXTRA_CFLAGS += -I$(PWD)/omnivision_ov2775
EXTRA_CFLAGS += -DSENSR1_FUNCTION=imx334_function
endif

ifeq ($(SENSR0_TYPE), ov5693)
$(TARGET)-objs += ./omnivision_ov5693/ov5693_driver.o
$(TARGET)-objs += ./omnivision_ov5693/ov5693_mipi4lane_1080p_30fps_linear.o
EXTRA_CFLAGS += -I$(PWD)/omnivision_ov5693
EXTRA_CFLAGS += -DSENSR0_FUNCTION=ov5693_function
endif

ifeq ($(SENSR1_TYPE), ov5693)
#$(TARGET)-objs += ./omnivision_ov5693/ov5693_driver.o
#$(TARGET)-objs += ./omnivision_ov5693/ov5693_mipi4lane_1080p_30fps_linear.o
#EXTRA_CFLAGS += -I$(PWD)/omnivision_ov5693
EXTRA_CFLAGS += -DSENSR1_FUNCTION=ov5693_function
endif


ifeq ($(SENSR0_TYPE), imx290)
$(TARGET)-objs += ./sony_imx290/imx290_driver.o
$(TARGET)-objs += ./sony_imx290/imx290_mipi4lane_1080p_30fps_linear.o
EXTRA_CFLAGS += -I$(PWD)/sony_imx290
EXTRA_CFLAGS += -DSENSR0_FUNCTION=imx290_function
endif

ifeq ($(SENSR1_TYPE), imx290)
#$(TARGET)-objs += ./omnivision_ov5693/ov5693_driver.o
#$(TARGET)-objs += ./omnivision_ov5693/ov5693_mipi4lane_1080p_30fps_linear.o
#EXTRA_CFLAGS += -I$(PWD)/omnivision_ov5693
EXTRA_CFLAGS += -DSENSR1_FUNCTION=imx290_function
endif

ifeq ($(PLATFORM), gen6)
$(TARGET)-objs += ./platform/platform_gen6_driver.o
EXTRA_CFLAGS += -I$(PWD)/platform
endif

EXTRA_CFLAGS += -DSENSR0_TYPE
EXTRA_CFLAGS += -DSENSR1_TYPE

EXTRA_CFLAGS += -I$(PWD)/

PWD :=$(shell pwd)

$(info YANDONG KERNEL=$(KERNEL))
$(info YANDONG KERNEL_SRC=$(KERNEL_SRC))

all:
	make  V=1 -C $(KERNEL_SRC) M=$(PWD)  modules
modules_install:
	make V=1 -C $(KERNEL_SRC) M=$(SRC) modules_install
clean:
	rm -rf $($(TARGET)-objs)
	make -C $(KERNEL_SRC) M=`pwd` clean

