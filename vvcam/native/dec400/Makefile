include $(PWD)/../Param.mk
include $(PWD)/../../version/$(VERSION_CFG).mk
TARGET = vvcam_dec400

obj-m +=$(TARGET).o

$(TARGET)-objs += dec400.o
$(TARGET)-objs += dec400_ioctl.o
EXTRA_CFLAGS += -I$(PWD)/../../common

PWD :=$(shell pwd)

all:
	make   -C $(KERNEL) M=$(PWD)  modules
modules_install:
	make -C $(KERNEL_SRC) M=$(SRC) modules_install
clean:
	rm -rf $($(TARGET)-objs)
	make -C $(KERNEL) M=`pwd` clean
