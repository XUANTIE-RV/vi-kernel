include $(PWD)/../Param.mk
include $(PWD)/../../version/$(VERSION_CFG).mk
TARGET = vvcam_i2c

obj-m +=$(TARGET).o
$(TARGET)-objs += soc_i2c_driver.o

PWD :=$(shell pwd)

all:
	make   -C $(KERNEL) M=$(PWD)  modules
modules_install:
	make -C $(KERNEL_SRC) M=$(SRC) modules_install
clean:
	rm -rf $($(TARGET)-objs)
	make -C $(KERNEL) M=`pwd` clean

