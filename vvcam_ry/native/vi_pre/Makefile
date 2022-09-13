include $(PWD)/../Param.mk
include $(PWD)/../../version/$(VERSION_CFG).mk
TARGET = vi_pre

obj-m +=$(TARGET).o

$(TARGET)-objs += vi_pre_driver_of.o
$(TARGET)-objs += vi_pre_ioctl.o
EXTRA_CFLAGS += -I$(PWD)/../../common

PWD :=$(shell pwd)

all:
	make   -C $(KERNEL) M=$(PWD)  modules
modules_install:
	make -C $(KERNEL_SRC) M=$(SRC) modules_install
clean:
	rm -rf $($(TARGET)-objs)
	make -C $(KERNEL) M=`pwd` clean
