include $(PWD)/../Param.mk
include $(PWD)/../../version/$(VERSION_CFG).mk
TARGET = vvcam_dw200

obj-m +=$(TARGET).o

$(TARGET)-objs += vvcam_dwe_driver_of.o
$(TARGET)-objs += ../../dw200/vivdw200_irq_queue.o
$(TARGET)-objs += ../../dw200/dw200_ioctl.o
EXTRA_CFLAGS += -I$(PWD)/../../dw200
EXTRA_CFLAGS += -I$(PWD)/../../common


PWD :=$(shell pwd)

all:
	make   -C $(KERNEL) M=$(PWD)  modules
modules_install:
	make -C $(KERNEL_SRC) M=$(SRC) modules_install
clean:
	rm -rf $($(TARGET)-objs)
	make -C $(KERNEL) M=`pwd` clean

