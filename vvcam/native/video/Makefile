include $(PWD)/../Param.mk
include $(PWD)/../../version/$(VERSION_CFG).mk
TARGET = thead_video

obj-m +=$(TARGET).o

$(TARGET)-objs += video.o
$(TARGET)-objs += video_sysfs.o
$(TARGET)-objs += video_hw_info.o
$(TARGET)-objs += subdev.o

EXTRA_CFLAGS += -I$(PWD)/../../common

PWD :=$(shell pwd)

all:
	make   -C $(KERNEL) M=$(PWD)  modules
modules_install:
	make -C $(KERNEL_SRC) M=$(SRC) modules_install
clean:
	rm -rf $($(TARGET)-objs)
	make -C $(KERNEL) M=`pwd` clean
