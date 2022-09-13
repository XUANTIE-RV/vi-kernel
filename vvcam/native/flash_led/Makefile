include $(PWD)/../Param.mk
include $(PWD)/../../version/$(VERSION_CFG).mk
TARGET = vvcam_flash_led

obj-m +=$(TARGET).o
$(TARGET)-objs += flash_led_driver.o
$(TARGET)-objs += flash_led_ioctl.o
$(TARGET)-objs += flash_led_sysfs.o
$(TARGET)-objs += ntc.o


EXTRA_CFLAGS += -I$(PWD)/../../common

ifeq ($(FLASH_LED_TYPE1), aw36515)
$(TARGET)-objs += ./aw36515/aw36515_driver.o
EXTRA_CFLAGS += -I$(PWD)/aw36515
EXTRA_CFLAGS += -DPROJECTION_FUNCTION=aw36515_function
endif

ifeq ($(FLASH_LED_TYPE2), aw36413)
$(TARGET)-objs += ./aw36413/aw36413_driver.o
EXTRA_CFLAGS += -I$(PWD)/aw36413
EXTRA_CFLAGS += -DFLOODLIGHT_FUNCTION=aw36413_function
endif



#EXTRA_CFLAGS += -DSENSR0_TYPE
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

