include $(PWD)/../Param.mk
include $(PWD)/../../version/$(VERSION_CFG).mk
TARGET = vvcam_isp_ry

obj-m +=$(TARGET).o
$(TARGET)-objs += vvcam_isp_driver_of.o
$(TARGET)-objs += ../../isp/isp_miv1.o
$(TARGET)-objs += ../../isp/isp_miv2.o
$(TARGET)-objs += ../../isp/isp_wdr3.o
$(TARGET)-objs += ../../isp/isp_wdr4.o
$(TARGET)-objs += ../../isp/isp_wdr.o
$(TARGET)-objs += ../../isp/isp_3dnr.o
$(TARGET)-objs += ../../isp/isp_3dnr2.o
$(TARGET)-objs += ../../isp/isp_tdnr3.o
$(TARGET)-objs += ../../isp/isp_hdr.o
$(TARGET)-objs += ../../isp/isp_dpf.o
$(TARGET)-objs += ../../isp/isp_compand.o
$(TARGET)-objs += ../../isp/isp_gcmono.o
$(TARGET)-objs += ../../isp/isp_rgbgamma.o
$(TARGET)-objs += ../../isp/isp_ioctl.o
$(TARGET)-objs += ../../isp/isp_dmsc2.o
$(TARGET)-objs += ../../isp/isp_rgbir.o
$(TARGET)-objs += ../../isp/isp_irq_queue.o



EXTRA_CFLAGS += -I$(PWD)/../../isp
EXTRA_CFLAGS += -I$(PWD)/../../driver
EXTRA_CFLAGS += -I$(PWD)/../../common


PWD :=$(shell pwd)

all:
	make   -C $(KERNEL) M=$(PWD)  modules
modules_install:
	make -C $(KERNEL_SRC) M=$(SRC) modules_install
clean:
	rm -rf $($(TARGET)-objs)
	make -C $(KERNEL) M=`pwd` clean

