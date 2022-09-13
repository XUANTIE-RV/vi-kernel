##
 # Copyright (C) 2020 Alibaba Group Holding Limited
##
test = $(shell if [ -f "../.param" ]; then echo "exist"; else echo "noexist"; fi)
ifeq ("$(test)", "exist")
  include ../.param
endif

CONFIG_COMPILE_PLATFORM=RISCV

CONFIG_ISP_BUILD_TYPE=DEBUG
#CONFIG_ISP_BUILD_TYPE=RELEASE
CONFIG_ISP_VERSION=ISP8000L_V2008

CONFIG_ISP_RY_BUILD_TYPE=DEBUG
#CONFIG_ISP_RY_BUILD_TYPE=RELEASE
CONFIG_ISP_RY_VERSION=ISP8000_V2009


CONFIG_MAKE_ISP_DRIVER_CMD:= \
    make -C vvcam/native VERSION_CFG=$(CONFIG_ISP_VERSION) \
	    KERNEL=$(LINUX_DIR) CROSS=$(CROSS_COMPILE) ARCH=$(ARCH) BOARD_NAME=$(BOARD_NAME)

CONFIG_DW200_CMAKE_CMD:= \
	cmake -DCMODEL=$(CONFIG_DW200_CMODEL) \
		-DV4L2=$(CONFIG_DW200_V4L2) \
		-DARM64_LOCAL_CROSS=OFF ..

CONFIG_DEC400_CMAKE_CMD:= \
	cmake -Wno-dev ..


CONFIG_MAKE_ISP_RY_DRIVER_CMD:= \
    make -C vvcam_ry/native VERSION_CFG=$(CONFIG_ISP_RY_VERSION) \
	    KERNEL=$(LINUX_DIR) CROSS=$(CROSS_COMPILE) ARCH=$(ARCH) BOARD_NAME=$(BOARD_NAME)

DIR_ISP_TARGET_BASE=bsp/isp
DIR_ISP_TARGET_KO  =bsp/isp/ko

DIR_ISP_RY_TARGET_BASE=bsp/isp_ry
DIR_ISP_RY_TARGET_KO  =bsp/isp_ry/ko

DIR_DW200_TARGET_BASE=bsp/dw200


DIR_DEC400_TARGET_BASE=bsp/dec400

MODULE_NAME=ISP
BUILD_LOG_START="\033[47;30m>>> $(MODULE_NAME) $@ begin\033[0m"
BUILD_LOG_END  ="\033[47;30m<<< $(MODULE_NAME) $@ end\033[0m"

#
# Do a parallel build with multiple jobs, based on the number of CPUs online
# in this system: 'make -j8' on a 8-CPU system, etc.
#
# (To override it, run 'make JOBS=1' and similar.)
#
ifeq ($(JOBS),)
  JOBS := $(shell grep -c ^processor /proc/cpuinfo 2>/dev/null)
  ifeq ($(JOBS),)
    JOBS := 1
  endif
endif

all:    info isp_driver isp_ry_driver\
		install_local_output install_rootfs
.PHONY: info isp_driver isp_ry_driver\
		install_local_output install_rootfs \
        clean_isp_driver clean_isp_ry_driver \
        clean_output clean

info:
	@echo $(BUILD_LOG_START)
	@echo "  ====== Build Info from repo project ======"
	@echo "    BUILDROOT_DIR="$(BUILDROOT_DIR)
	@echo "    CROSS_COMPILE="$(CROSS_COMPILE)
	@echo "    LINUX_DIR="$(LINUX_DIR)
	@echo "    ARCH="$(ARCH)
	@echo "    BOARD_NAME="$(BOARD_NAME)
	@echo "    KERNEL_ID="$(KERNELVERSION)
	@echo "    KERNEL_DIR="$(LINUX_DIR)
	@echo "    INSTALL_DIR_ROOTFS="$(INSTALL_DIR_ROOTFS)
	@echo "    DIR_MODULE_TOP="$(DIR_MODULE_TOP)
	@echo "  ====== Build configuration by settings ======"
	@echo "    COMPILE_PLATFORM="$(CONFIG_COMPILE_PLATFORM)
	@echo "    JOBS="$(JOBS)
	@echo "    ISP_BUILD_TYPE="$(CONFIG_ISP_BUILD_TYPE)
	@echo "    ISP_VERSION="$(CONFIG_ISP_VERSION)
	@echo "    CMAKE_ISP_DRIVER_CMD=" $(CONFIG_MAKE_ISP_DRIVER_CMD)
	@echo "    DW200_CMODEL="$(CONFIG_DW200_CMODEL)
	@echo "    DW200_V4L2="$(CONFIG_DW200_V4L2)
	@echo "    DW200_BUILD_DIR="$(CONFIG_DW200_BUILD_DIR)
	@echo "    DW200_CMAKE_CMD="$(CONFIG_DW200_CMAKE_CMD)
	@echo "    DEC400_BUILD_DIR="$(CONFIG_DEC400_BUILD_DIR)
	@echo "    DEC400_CMAKE_CMD="$(CONFIG_DEC400_CMAKE_CMD)
	@echo "    ISP_RY_BUILD_TYPE="$(CONFIG_ISP_RY_BUILD_TYPE)
	@echo "    ISP_RY_VERSION="$(CONFIG_ISP_RY_VERSION)

	@echo $(BUILD_LOG_END)

isp_driver:
	@echo $(BUILD_LOG_START)
	$(CONFIG_MAKE_ISP_DRIVER_CMD)
	@echo $(BUILD_LOG_END)

clean_isp_driver:
	@echo $(BUILD_LOG_START)
	make -C vvcam/native VERSION_CFG=$(CONFIG_ISP_VERSION) clean
	#make -C vvcam/v4l2 VERSION_CFG=$(CONFIG_ISP_VERSION) clean
	rm -f vvcam/dw200/.*.o.cmd
	rm -f vvcam/isp/.*.o.cmd
	rm -f vvcam/native/bin/*.ko
	@echo $(BUILD_LOG_END)

isp_ry_driver:
	@echo $(BUILD_LOG_START)
	$(CONFIG_MAKE_ISP_RY_DRIVER_CMD)
	@echo $(BUILD_LOG_END)

clean_isp_ry_driver:
	@echo $(BUILD_LOG_START)
	make -C vvcam_ry/native VERSION_CFG=$(CONFIG_ISP_RY_VERSION) clean
	make -C vvcam_ry/v4l2 VERSION_CFG=$(CONFIG_ISP_RY_VERSION) clean
	rm -f vvcam_ry/dw200/.*.o.cmd
	rm -f vvcam_ry/isp/.*.o.cmd
	rm -f vvcam_ry/native/bin/*.ko
	@echo $(BUILD_LOG_END)





install_local_output: isp_driver isp_ry_driver
	@echo $(BUILD_LOG_START)
	# isp driver files
	mkdir -p ./output/rootfs/$(DIR_ISP_TARGET_KO)
	chmod +x ./vvcam/native/bin/*.sh
	cp -f ./vvcam/native/bin/* ./output/rootfs/$(DIR_ISP_TARGET_KO)

	# isp ry driver files
	mkdir -p ./output/rootfs/$(DIR_ISP_RY_TARGET_KO)
	chmod +x ./vvcam_ry/native/bin/*.sh
	cp -f ./vvcam_ry/native/bin/vvcam_isp_ry.ko ./output/rootfs/$(DIR_ISP_TARGET_KO)

install_rootfs: install_local_output
	@echo $(BUILD_LOG_START)
#	cp -rf output/rootfs/* $(INSTALL_DIR_ROOTFS)
	@echo $(BUILD_LOG_END)

clean_output:
	@echo $(BUILD_LOG_START)
	rm -rf ./output
	rm -rf $(INSTALL_DIR_ROOTFS)/$(DIR_ISP_TARGET_BASE)
	rm -rf $(INSTALL_DIR_ROOTFS)/$(DIR_ISP_RY_TARGET_BASE)
	rm -rf $(INSTALL_DIR_ROOTFS)/$(DIR_DW200_TARGET_BASE)
	rm -rf $(INSTALL_DIR_ROOTFS)/$(DIR_DEC400_TARGET_BASE)
	@echo $(BUILD_LOG_END)

clean_proprietories_include:
	@echo $(BUILD_LOG_START)
	@echo $(BUILD_LOG_END)

clean: clean_output clean_isp_driver \
	clean_isp_ry_driver clean_proprietories_include
