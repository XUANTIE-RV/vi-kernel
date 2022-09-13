include $(PWD)/../Param.mk
include $(PWD)/../../version/$(VERSION_CFG).mk
TARGET = vvcam_soc

obj-m +=$(TARGET).o
$(TARGET)-objs += vvcam_soc_driver.o
$(TARGET)-objs += soc_ioctl.o
$(TARGET)-objs += vivsoc_hub.o
$(TARGET)-objs += vsi_core_gen6.o

PWD :=$(shell pwd)

all:
	make   -C $(KERNEL) M=$(PWD)  modules
modules_install:
	make -C $(KERNEL_SRC) M=$(SRC) modules_install
clean:
	rm -rf $($(TARGET)-objs)
	make -C $(KERNEL) M=`pwd` clean

